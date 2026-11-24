/*
 * ESP32-C5 wireless MCU — does NOT run RL inference.
 * Bring-up: pushes microduck_link_frame_t to P4 over UART (idle twist).
 * Next: WiFi / BLE / phone bridge sets twist from network, still forwards to P4.
 */
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "microduck_link.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *kTag = "c5";

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = CONFIG_MICRODUCK_C5_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_MICRODUCK_C5_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_MICRODUCK_C5_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_MICRODUCK_C5_UART_NUM, CONFIG_MICRODUCK_C5_UART_TX_GPIO,
                                 CONFIG_MICRODUCK_C5_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static void send_cmd_task(void *arg)
{
    (void)arg;
    const int period_ms = 1000 / CONFIG_MICRODUCK_C5_CMD_HZ;
    uint32_t seq = 0;
    microduck_link_frame_t frame = {
        .magic = MICRODUCK_LINK_MAGIC,
        .version = MICRODUCK_LINK_VERSION,
        .type = 1,
        .cmd = MICRODUCK_COMMAND_IDLE,
        .crc32 = 0,
    };

    for (;;) {
        frame.seq = ++seq;
        // TODO: when WiFi/MQTT/UDP is up, memcpy remote twist into frame.cmd.
        uart_write_bytes(CONFIG_MICRODUCK_C5_UART_NUM, &frame, sizeof(frame));
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(kTag, "microduck C5 wireless — commands to P4, no policy here");
    uart_init();
    xTaskCreate(send_cmd_task, "cmd_tx", 3072, NULL, 4, NULL);
    ESP_LOGI(kTag, "UART%d @ %d baud, %d Hz idle cmd stream", CONFIG_MICRODUCK_C5_UART_NUM,
             CONFIG_MICRODUCK_C5_UART_BAUD, CONFIG_MICRODUCK_C5_CMD_HZ);
}