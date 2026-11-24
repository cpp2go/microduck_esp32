// SPDX-License-Identifier: MIT
//
// Protocol 2.0 XL330 helpers aligned with pollen-robotics/microduck duck-control/src/bus.rs:
// one SyncRead of the 12-byte block at Present PWM (124) → current/vel/pos, and the same
// radian ↔ raw conversion (2π·raw/4096 − π).

#include "xl330_dxl.h"

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

#define DXL_HDR0 0xFF
#define DXL_HDR1 0xFF
#define DXL_HDR2 0xFD
#define DXL_BROADCAST 0xFE

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

#define ADDR_RETURN_DELAY 9
#define ADDR_OPERATING_MODE 11
#define ADDR_CURRENT_LIMIT 38
#define ADDR_TORQUE_ENABLE 64
#define ADDR_POS_P_GAIN 84
#define ADDR_GOAL_POSITION 116
/** Contiguous Present PWM / current / velocity / position — duck-control bus.rs READ_ADDR. */
#define ADDR_PRESENT_BLOCK 124
#define PRESENT_BLOCK_LEN 12

#define MODE_POSITION 3
#define CURRENT_LIMIT_MA 1000

static const char *kTag = "xl330";
static bool s_up = false;

/// 0.229 rev/min per count → rad/s (same as duck-control RAD_PER_SEC_PER_COUNT).
static const float kVelRadPerUnit = (0.229f * 2.0f * (float)M_PI / 60.0f);

static uint16_t dxl_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8005) : (crc >> 1);
        }
    }
    return crc;
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

static void tx_packet(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t param_len)
{
    uint8_t pkt[256];
    uint16_t idx = 0;
    pkt[idx++] = DXL_HDR0;
    pkt[idx++] = DXL_HDR1;
    pkt[idx++] = DXL_HDR2;
    pkt[idx++] = id;
    pkt[idx++] = (uint8_t)(param_len & 0xFF);
    pkt[idx++] = (uint8_t)((param_len >> 8) & 0xFF);
    pkt[idx++] = 0;
    pkt[idx++] = inst;
    if (param_len > 0) {
        memcpy(pkt + idx, params, param_len);
        idx += param_len;
    }
    uint16_t crc = dxl_crc16(pkt + 3, idx - 3);
    pkt[idx++] = (uint8_t)(crc & 0xFF);
    pkt[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    uart_tx(pkt, idx);
}

static int rx_packet(uint8_t *buf, int buf_sz, int timeout_ms)
{
    int got = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (got < 7 && esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(CONFIG_MICRODUCK_SERVO_UART_NUM, buf + got, 7 - got, pdMS_TO_TICKS(2));
        if (n > 0) {
            got += n;
        }
    }
    if (got < 7 || buf[0] != DXL_HDR0 || buf[1] != DXL_HDR1 || buf[2] != DXL_HDR2) {
        return -1;
    }
    uint16_t len = buf[4] | ((uint16_t)buf[5] << 8);
    int total = 7 + (int)len + 2;
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
    uint16_t crc = dxl_crc16(buf + 3, (uint16_t)(4 + len));
    uint16_t recv = buf[7 + len] | ((uint16_t)buf[7 + len + 1] << 8);
    if (crc != recv) {
        return -4;
    }
    return total;
}

static bool write_u8(uint8_t id, uint16_t addr, uint8_t val)
{
    uint8_t p[3] = {(uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), val};
    tx_packet(id, INST_WRITE, p, 3);
    uint8_t buf[32];
    return rx_packet(buf, sizeof(buf), 8) >= 0;
}

static bool write_u16(uint8_t id, uint16_t addr, uint16_t val)
{
    uint8_t p[4] = {(uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), (uint8_t)(val & 0xFF),
                    (uint8_t)(val >> 8)};
    tx_packet(id, INST_WRITE, p, 4);
    uint8_t buf[32];
    return rx_packet(buf, sizeof(buf), 8) >= 0;
}

bool xl330_bus_init(void)
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
    ESP_LOGI(kTag, "UART%d @ %d", CONFIG_MICRODUCK_SERVO_UART_NUM, CONFIG_MICRODUCK_SERVO_UART_BAUD);
    return true;
}

int xl330_ping(uint8_t id)
{
    uint8_t p[] = {0x00, 0x00, 0x02, 0x00}; /* read 2 bytes @0 */
    tx_packet(id, INST_READ, p, 4);
    uint8_t buf[32];
    return rx_packet(buf, sizeof(buf), 8) >= 0 ? 1 : 0;
}

bool xl330_configure_servo(uint8_t id, uint16_t position_p_gain)
{
    if (!write_u8(id, ADDR_TORQUE_ENABLE, 0)) {
        return false;
    }
    /* EXPECTED_REGISTERS in duck-control/model.rs — factory 250 µs×2 turnaround eats the 20 ms tick. */
    if (!write_u8(id, ADDR_RETURN_DELAY, 0)) {
        return false;
    }
    if (!write_u8(id, ADDR_OPERATING_MODE, MODE_POSITION)) {
        return false;
    }
    if (!write_u16(id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_MA)) {
        return false;
    }
    if (!write_u16(id, ADDR_POS_P_GAIN, position_p_gain)) {
        return false;
    }
    return write_u8(id, ADDR_TORQUE_ENABLE, 1);
}

int32_t xl330_rad_to_position_int(float rad)
{
    /* Inverse of (2π·raw/4096) − π — matches duck-control bus.rs tests. */
    return (int32_t)lroundf(4096.0f * ((float)M_PI + rad) / (2.0f * (float)M_PI));
}

float xl330_position_unit_to_rad(int32_t unit)
{
    return (2.0f * (float)M_PI * (float)unit / 4096.0f) - (float)M_PI;
}

float xl330_velocity_unit_to_rad_s(int32_t unit) { return (float)unit * kVelRadPerUnit; }

bool xl330_sync_read_pv(const uint8_t *ids, int n, float *pos_rad, float *vel_rad_s)
{
    /* One SyncRead of the 12-byte Present block (PWM+current+vel+pos), same layout as robotd. */
    uint8_t param[4 + 32];
    param[0] = (uint8_t)(ADDR_PRESENT_BLOCK & 0xFF);
    param[1] = (uint8_t)(ADDR_PRESENT_BLOCK >> 8);
    param[2] = PRESENT_BLOCK_LEN;
    param[3] = 0;
    for (int i = 0; i < n; ++i) {
        param[4 + i] = ids[i];
    }
    tx_packet(DXL_BROADCAST, INST_SYNC_READ, param, (uint16_t)(4 + n));
    for (int i = 0; i < n; ++i) {
        uint8_t buf[48];
        if (rx_packet(buf, sizeof(buf), 20) < 0) {
            return false;
        }
        if (buf[3] != ids[i]) {
            return false;
        }
        /* Status: [7]=err, [8..19]=12-byte block. Offsets match bus.rs decode. */
        const uint8_t *block = buf + 8;
        int32_t velocity = (int32_t)((uint32_t)block[4] | ((uint32_t)block[5] << 8) |
                                     ((uint32_t)block[6] << 16) | ((uint32_t)block[7] << 24));
        int32_t position = (int32_t)((uint32_t)block[8] | ((uint32_t)block[9] << 8) |
                                     ((uint32_t)block[10] << 16) | ((uint32_t)block[11] << 24));
        vel_rad_s[i] = xl330_velocity_unit_to_rad_s(velocity);
        pos_rad[i] = xl330_position_unit_to_rad(position);
    }
    return true;
}

bool xl330_sync_write_goal_rad(const uint8_t *ids, const float *goal_rad, int n)
{
    uint8_t param[4 + 5 * 32];
    param[0] = (uint8_t)(ADDR_GOAL_POSITION & 0xFF);
    param[1] = (uint8_t)(ADDR_GOAL_POSITION >> 8);
    param[2] = 4;
    param[3] = 0;
    int off = 4;
    for (int i = 0; i < n; ++i) {
        int32_t g = xl330_rad_to_position_int(goal_rad[i]);
        param[off++] = ids[i];
        param[off++] = (uint8_t)(g & 0xFF);
        param[off++] = (uint8_t)((g >> 8) & 0xFF);
        param[off++] = (uint8_t)((g >> 16) & 0xFF);
        param[off++] = (uint8_t)((g >> 24) & 0xFF);
    }
    tx_packet(DXL_BROADCAST, INST_SYNC_WRITE, param, (uint16_t)off);
    uart_flush_input(CONFIG_MICRODUCK_SERVO_UART_NUM);
    return true;
}

bool xl330_set_torque_enable(uint8_t id, bool on) { return write_u8(id, ADDR_TORQUE_ENABLE, on ? 1 : 0); }