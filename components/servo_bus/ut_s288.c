// SPDX-License-Identifier: MIT
//
// Unitree S288 / J288 on half-duplex TTL UART @ 6 Mbps.
// No SyncRead/Write — each joint is a 20 B command → 26 B feedback round-trip.

#include "ut_s288.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "ut_s288_codec.h"

static const char *kTag = "ut_s288";
static bool s_up;
static float s_kp = 10.0f;
static float s_kd = 1.0f;
static float s_last_goal[16];

static void uart_tx(const uint8_t *data, size_t len)
{
#if CONFIG_MICRODUCK_SERVO_DIR_GPIO >= 0
    gpio_set_level((gpio_num_t)CONFIG_MICRODUCK_SERVO_DIR_GPIO, 1);
#endif
    uart_write_bytes(CONFIG_MICRODUCK_SERVO_UART_NUM, data, len);
    uart_wait_tx_done(CONFIG_MICRODUCK_SERVO_UART_NUM, pdMS_TO_TICKS(10));
#if CONFIG_MICRODUCK_SERVO_DIR_GPIO >= 0
    gpio_set_level((gpio_num_t)CONFIG_MICRODUCK_SERVO_DIR_GPIO, 0);
#endif
}

static int rx_exact(uint8_t *buf, int need, int timeout_ms)
{
    int got = 0;
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (got < need && esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(CONFIG_MICRODUCK_SERVO_UART_NUM, buf + got, need - got, pdMS_TO_TICKS(1));
        if (n > 0) {
            got += n;
        }
    }
    return got;
}

static bool cmd_fbk(uint8_t id, uint8_t mode, float goal_rad, float kp, float kd, ut_s288_feedback_t *fbk)
{
    uint8_t cmd[UT_S288_CMD_LEN];
    ut_s288_build_control(id, mode, true, 0.0f, 0.0f, goal_rad, kp, kd, cmd);
    uart_flush_input(CONFIG_MICRODUCK_SERVO_UART_NUM);
    uart_tx(cmd, sizeof(cmd));
    if (id == 15) {
        return true; /* broadcast — no response */
    }
    uint8_t raw[UT_S288_FBK_LEN];
    if (rx_exact(raw, UT_S288_FBK_LEN, 5) != UT_S288_FBK_LEN) {
        return false;
    }
    /* Resync if leading bytes wrong (bus noise). */
    if (raw[0] != 0xFC || raw[1] != 0xEE) {
        int shift = -1;
        for (int i = 0; i < UT_S288_FBK_LEN - 1; ++i) {
            if (raw[i] == 0xFC && raw[i + 1] == 0xEE) {
                shift = i;
                break;
            }
        }
        if (shift <= 0) {
            return false;
        }
        memmove(raw, raw + shift, (size_t)(UT_S288_FBK_LEN - shift));
        if (rx_exact(raw + (UT_S288_FBK_LEN - shift), shift, 3) != shift) {
            return false;
        }
    }
    ut_s288_feedback_t parsed;
    if (!ut_s288_parse_feedback(raw, &parsed)) {
        return false;
    }
    if (parsed.id != id) {
        return false;
    }
    if (fbk) {
        *fbk = parsed;
    }
    return true;
}

bool ut_s288_bus_init(void)
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
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_MICRODUCK_SERVO_UART_NUM, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_MICRODUCK_SERVO_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_MICRODUCK_SERVO_UART_NUM, CONFIG_MICRODUCK_SERVO_TX_GPIO,
                                 CONFIG_MICRODUCK_SERVO_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    memset(s_last_goal, 0, sizeof(s_last_goal));
    s_up = true;
    ESP_LOGI(kTag, "Unitree S288 UART%d @ %d (fixed protocol 6 Mbps recommended)",
             CONFIG_MICRODUCK_SERVO_UART_NUM, CONFIG_MICRODUCK_SERVO_UART_BAUD);
    return true;
}

int ut_s288_ping(uint8_t id)
{
    ut_s288_feedback_t fbk;
    return cmd_fbk(id, UT_S288_MODE_FOC, s_last_goal[id & 15], 0.0f, 0.0f, &fbk) ? 1 : 0;
}

bool ut_s288_configure_servo(uint8_t id, float kp, float kd)
{
    s_kp = kp;
    s_kd = kd;
    ut_s288_feedback_t fbk;
    /* Soft hold at current goal with configured gains. */
    return cmd_fbk(id, UT_S288_MODE_FOC, s_last_goal[id & 15], s_kp, s_kd, &fbk);
}

bool ut_s288_set_torque_enable(uint8_t id, bool on)
{
    ut_s288_feedback_t fbk;
    if (on) {
        return cmd_fbk(id, UT_S288_MODE_FOC, s_last_goal[id & 15], s_kp, s_kd, &fbk);
    }
    return cmd_fbk(id, UT_S288_MODE_LOCK, s_last_goal[id & 15], 0.0f, 0.0f, &fbk);
}

bool ut_s288_sync_read_pv(const uint8_t *ids, int n, float *pos_rad, float *vel_rad_s)
{
    if (!ids || !pos_rad || !vel_rad_s || n <= 0) {
        return false;
    }
    for (int i = 0; i < n; ++i) {
        const uint8_t id = ids[i];
        ut_s288_feedback_t fbk;
        if (!cmd_fbk(id, UT_S288_MODE_FOC, s_last_goal[id & 15], s_kp, s_kd, &fbk)) {
            return false;
        }
        pos_rad[i] = fbk.output_pos;
        vel_rad_s[i] = fbk.output_spd;
    }
    return true;
}

bool ut_s288_sync_write_goal_rad(const uint8_t *ids, const float *goal_rad, int n)
{
    if (!ids || !goal_rad || n <= 0) {
        return false;
    }
    for (int i = 0; i < n; ++i) {
        const uint8_t id = ids[i];
        s_last_goal[id & 15] = goal_rad[i];
        ut_s288_feedback_t fbk;
        if (!cmd_fbk(id, UT_S288_MODE_FOC, goal_rad[i], s_kp, s_kd, &fbk)) {
            return false;
        }
    }
    return true;
}
