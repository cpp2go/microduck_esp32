// SPDX-License-Identifier: MIT

#include "cmd_link.h"

#include "sdkconfig.h"

#if CONFIG_MICRODUCK_CMD_UART_ENABLE

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "cmd_bus.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "microduck_link.h"

static const char *kTag = "cmd_uart";

static void uart_ingest_task(void *)
{
    uint8_t buf[sizeof(microduck_link_frame_t)];
    int filled = 0;

    for (;;) {
        int n = uart_read_bytes(CONFIG_MICRODUCK_CMD_UART_NUM, buf + filled,
                                (int)sizeof(buf) - filled, pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }
        filled += n;
        if (filled < (int)sizeof(microduck_link_frame_t)) {
            continue;
        }
        filled = 0;
        microduck_link_frame_t frame;
        memcpy(&frame, buf, sizeof(frame));
        if (frame.magic != MICRODUCK_LINK_MAGIC || frame.version != MICRODUCK_LINK_VERSION ||
            frame.type != 1) {
            ESP_LOGW(kTag, "bad frame magic=%08" PRIx32, frame.magic);
            continue;
        }
        cmd_bus_set(&frame.cmd, frame.seq);
    }
}

void cmd_link_start_uart_ingest(void)
{
    uart_config_t cfg = {
        .baud_rate = CONFIG_MICRODUCK_CMD_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_MICRODUCK_CMD_UART_NUM, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_MICRODUCK_CMD_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_MICRODUCK_CMD_UART_NUM, CONFIG_MICRODUCK_CMD_UART_TX_GPIO,
                                 CONFIG_MICRODUCK_CMD_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    xTaskCreate(uart_ingest_task, "cmd_uart", 4096, NULL, 3, NULL);
    ESP_LOGI(kTag, "UART%d ingest @ %d baud (bench only)", CONFIG_MICRODUCK_CMD_UART_NUM,
             CONFIG_MICRODUCK_CMD_UART_BAUD);
}

#else

void cmd_link_start_uart_ingest(void) {}

#endif /* CONFIG_MICRODUCK_CMD_UART_ENABLE */
