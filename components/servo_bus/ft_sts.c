// SPDX-License-Identifier: MIT
//
// Feetech STS / HD1910M (飞特 1910) on half-duplex TTL UART.
// Protocol: FT-SCS (FF FF … checksum), little-endian 16-bit with BIT15 sign —
// same application layer Microduck Robot uses for HD1910M / STS3215.
// Control feedback: SyncRead Present Position+Speed @56, SyncWrite Goal @42.

#include "ft_sts.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FT_HDR0 0xFF
#define FT_HDR1 0xFF
#define FT_BROADCAST 0xFE

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

#define ADDR_RETURN_DELAY 7
#define ADDR_P_COEFF 21
#define ADDR_OPERATING_MODE 33
#define ADDR_TORQUE_ENABLE 40
#define ADDR_GOAL_POSITION 42
#define ADDR_TEMP_KP 50 /* HD1910M temporary KP (SRAM) */
#define ADDR_LOCK 55
#define ADDR_PRESENT_POSITION 56
#define ADDR_PRESENT_SPEED 58

#define MODE_POSITION 0

static const char *kTag = "ft_sts";
static bool s_up = false;

/// Position: 4096 counts / turn → same upright mapping as duck-control bus.rs.
/// Speed: 0.732 RPM per count (STS / HD1910M), BIT15 = sign.
static const float kVelRadPerUnit = (0.732f * 2.0f * (float)M_PI / 60.0f);

static uint8_t checksum(const uint8_t *body, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + body[i]);
    }
    return (uint8_t)(~sum);
}

/** Encode signed magnitude with BIT15 as sign (Feetech Host2SCS). */
static uint16_t encode_signed(int32_t v)
{
    if (v < 0) {
        return (uint16_t)((-v) | 0x8000);
    }
    return (uint16_t)(v & 0x7FFF);
}

static int32_t decode_signed(uint16_t raw)
{
    if (raw & 0x8000) {
        return -(int32_t)(raw & 0x7FFF);
    }
    return (int32_t)(raw & 0x7FFF);
}

static void uart_tx(const uint8_t *data, size_t len)
{
#if CONFIG_MICRODUCK_SERVO_DIR_GPIO >= 0
    gpio_set_level((gpio_num_t)CONFIG_MICRODUCK_SERVO_DIR_GPIO, 1);
#endif
    uart_write_bytes(CONFIG_MICRODUCK_SERVO_UART_NUM, data, len);
    uart_wait_tx_done(CONFIG_MICRODUCK_SERVO_UART_NUM, pdMS_TO_TICKS(25));
#if CONFIG_MICRODUCK_SERVO_DIR_GPIO >= 0
    gpio_set_level((gpio_num_t)CONFIG_MICRODUCK_SERVO_DIR_GPIO, 0);
#endif
}

static void tx_packet(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t param_len)
{
    uint8_t pkt[256];
    uint8_t length = (uint8_t)(param_len + 2); /* instruction + params + checksum counted as Length */
    int idx = 0;
    pkt[idx++] = FT_HDR0;
    pkt[idx++] = FT_HDR1;
    pkt[idx++] = id;
    pkt[idx++] = length;
    pkt[idx++] = inst;
    if (param_len) {
        memcpy(pkt + idx, params, param_len);
        idx += param_len;
    }
    pkt[idx++] = checksum(pkt + 2, 3 + param_len); /* ID + Length + Inst + params */
    uart_tx(pkt, (size_t)idx);
}

static int rx_packet(uint8_t *buf, int buf_sz, int timeout_ms)
{
    int got = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (got < 4 && esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(CONFIG_MICRODUCK_SERVO_UART_NUM, buf + got, 4 - got, pdMS_TO_TICKS(2));
        if (n > 0) {
            got += n;
        }
    }
    if (got < 4 || buf[0] != FT_HDR0 || buf[1] != FT_HDR1) {
        return -1;
    }
    uint8_t length = buf[3];
    int total = 4 + length; /* hdr(2)+id+len already; length includes status/inst area + checksum */
    if (total > buf_sz) {
        return -2;
    }
    while (got < total && esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(CONFIG_MICRODUCK_SERVO_UART_NUM, buf + got, total - got, pdMS_TO_TICKS(2));
        if (n > 0) {
            got += n;
        }
    }
    if (got < total) {
        return -3;
    }
    uint8_t expect = checksum(buf + 2, total - 3);
    if (buf[total - 1] != expect) {
        return -4;
    }
    return total;
}

static bool write_u8(uint8_t id, uint8_t addr, uint8_t val)
{
    uint8_t p[2] = {addr, val};
    tx_packet(id, INST_WRITE, p, 2);
    uint8_t buf[16];
    return rx_packet(buf, sizeof(buf), 8) >= 0;
}

static bool write_u16(uint8_t id, uint8_t addr, uint16_t val)
{
    uint8_t p[3] = {addr, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8)};
    tx_packet(id, INST_WRITE, p, 3);
    uint8_t buf[16];
    return rx_packet(buf, sizeof(buf), 8) >= 0;
}

bool ft_sts_bus_init(void)
{
    if (s_up) {
        return true;
    }
#if CONFIG_MICRODUCK_SERVO_DIR_GPIO >= 0
    gpio_config_t io = {.pin_bit_mask = 1ULL << CONFIG_MICRODUCK_SERVO_DIR_GPIO,
                        .mode = GPIO_MODE_OUTPUT,
                        .pull_up_en = 0,
                        .pull_down_en = 0,
                        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io);
    gpio_set_level((gpio_num_t)CONFIG_MICRODUCK_SERVO_DIR_GPIO, 0);
#endif
    uart_config_t cfg = {
        .baud_rate = CONFIG_MICRODUCK_SERVO_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_MICRODUCK_SERVO_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_MICRODUCK_SERVO_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_MICRODUCK_SERVO_UART_NUM, CONFIG_MICRODUCK_SERVO_TX_GPIO,
                                CONFIG_MICRODUCK_SERVO_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_up = true;
    ESP_LOGI(kTag, "Feetech STS/HD1910 UART%d @ %d", CONFIG_MICRODUCK_SERVO_UART_NUM,
             CONFIG_MICRODUCK_SERVO_UART_BAUD);
    return true;
}

int ft_sts_ping(uint8_t id)
{
    tx_packet(id, INST_PING, NULL, 0);
    uint8_t buf[16];
    return rx_packet(buf, sizeof(buf), 8) >= 0 ? 1 : 0;
}

bool ft_sts_configure_servo(uint8_t id, uint16_t position_p_gain)
{
    if (!write_u8(id, ADDR_TORQUE_ENABLE, 0)) {
        return false;
    }
    (void)write_u8(id, ADDR_RETURN_DELAY, 0);
    if (!write_u8(id, ADDR_OPERATING_MODE, MODE_POSITION)) {
        return false;
    }
    /* HD1910M Temporary KP (SRAM @50); clamp to 0..254. Also try EEPROM P if unlocked. */
    uint8_t kp = position_p_gain > 254 ? 254 : (uint8_t)position_p_gain;
    (void)write_u8(id, ADDR_TEMP_KP, kp);
    (void)write_u8(id, ADDR_LOCK, 0);
    (void)write_u8(id, ADDR_P_COEFF, kp);
    (void)write_u8(id, ADDR_LOCK, 1);
    return write_u8(id, ADDR_TORQUE_ENABLE, 1);
}

int32_t ft_sts_rad_to_position_int(float rad)
{
    /* Center 2048 = 0 rad, same geometry as original Dynamixel mapping. */
    return (int32_t)lroundf(4096.0f * ((float)M_PI + rad) / (2.0f * (float)M_PI));
}

float ft_sts_position_unit_to_rad(int32_t unit)
{
    /* Present position is absolute 0..4095 (BIT15 unused in servo mode). */
    int32_t raw = unit & 0x0FFF;
    if (unit & 0x8000) {
        /* Signed direction encoding if firmware returns it that way. */
        raw = -(unit & 0x7FFF);
        /* Map negative magnitudes into encoder space around center — rare. */
        return (float)raw * (2.0f * (float)M_PI / 4096.0f);
    }
    return (2.0f * (float)M_PI * (float)raw / 4096.0f) - (float)M_PI;
}

float ft_sts_velocity_unit_to_rad_s(int32_t unit)
{
    return (float)decode_signed((uint16_t)unit) * kVelRadPerUnit;
}

bool ft_sts_sync_read_pv(const uint8_t *ids, int n, float *pos_rad, float *vel_rad_s)
{
    /* One SyncRead: Present Position (56) + Present Speed (58) = 4 bytes. */
    uint8_t param[2 + 32];
    param[0] = ADDR_PRESENT_POSITION;
    param[1] = 4;
    for (int i = 0; i < n; ++i) {
        param[2 + i] = ids[i];
    }
    tx_packet(FT_BROADCAST, INST_SYNC_READ, param, (uint8_t)(2 + n));
    for (int i = 0; i < n; ++i) {
        uint8_t buf[32];
        if (rx_packet(buf, sizeof(buf), 20) < 0) {
            return false;
        }
        if (buf[2] != ids[i]) {
            return false;
        }
        /* Status: [4]=error, [5..]=data. Length includes error+data+checksum. */
        uint16_t pos_u = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
        uint16_t vel_u = (uint16_t)buf[7] | ((uint16_t)buf[8] << 8);
        pos_rad[i] = ft_sts_position_unit_to_rad((int32_t)pos_u);
        vel_rad_s[i] = ft_sts_velocity_unit_to_rad_s((int32_t)vel_u);
    }
    return true;
}

bool ft_sts_sync_write_goal_rad(const uint8_t *ids, const float *goal_rad, int n)
{
    /* SyncWrite Goal Position (2 bytes LE, BIT15 sign). */
    uint8_t param[2 + 3 * 32];
    param[0] = ADDR_GOAL_POSITION;
    param[1] = 2;
    int off = 2;
    for (int i = 0; i < n; ++i) {
        int32_t g = ft_sts_rad_to_position_int(goal_rad[i]);
        if (g < 0) {
            g = 0;
        }
        if (g > 4095) {
            g = 4095;
        }
        uint16_t enc = encode_signed(g); /* goals are absolute 0..4095; BIT15 clear */
        (void)enc;
        param[off++] = ids[i];
        param[off++] = (uint8_t)(g & 0xFF);
        param[off++] = (uint8_t)((g >> 8) & 0xFF);
    }
    tx_packet(FT_BROADCAST, INST_SYNC_WRITE, param, (uint8_t)off);
    uart_flush_input(CONFIG_MICRODUCK_SERVO_UART_NUM);
    return true;
}

bool ft_sts_set_torque_enable(uint8_t id, bool on) { return write_u8(id, ADDR_TORQUE_ENABLE, on ? 1 : 0); }
