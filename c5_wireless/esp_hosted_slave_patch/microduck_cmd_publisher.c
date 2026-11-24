/*
 * ESP-Hosted **slave** (ESP32-C5) add-on — push remote commands to P4 over SDIO.
 *
 * Integrate into esp-hosted-mcu/slave:
 *   1. Copy this file into slave/main/ (or components/).
 *   2. Add to slave/main/CMakeLists.txt SRCS.
 *   3. Call microduck_cmd_publisher_start() from esp_hosted_coprocessor.c after
 *      the coprocessor link is up (search for example_peer_data_transfer_init).
 *
 * Requires: CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER=y on the slave build.
 */
#include <string.h>

#include "esp_hosted.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "microduck_link.h"
#include "sdkconfig.h"

static const char *kTag = "md_cmd_pub";

#ifndef CONFIG_MICRODUCK_C5_CMD_HZ
#define CONFIG_MICRODUCK_C5_CMD_HZ 20
#endif

static void publish_task(void *)
{
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
        /* TODO: after WiFi/MQTT/UDP, fill frame.cmd from the wireless parser. */
        esp_err_t err =
            esp_hosted_send_custom_data(MICRODUCK_MSG_ID_CMD, (const uint8_t *)&frame, sizeof(frame));
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "peer-data send: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

void microduck_cmd_publisher_start(void)
{
    xTaskCreate(publish_task, "md_cmd", 4096, NULL, 4, NULL);
    ESP_LOGI(kTag, "C5→P4 SDIO cmd publisher %d Hz", CONFIG_MICRODUCK_C5_CMD_HZ);
}