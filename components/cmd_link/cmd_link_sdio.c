// SPDX-License-Identifier: MIT
//
// P4 host: SDIO (esp_hosted) peer-data from ESP32-C5 → cmd_bus.

#include "cmd_link.h"

#include <inttypes.h>
#include <string.h>

#include "cmd_bus.h"
#include "esp_hosted.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "microduck_link.h"
#include "sdkconfig.h"

static const char *kTag = "cmd_sdio";

static void on_cmd_from_c5(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    if (msg_id != MICRODUCK_MSG_ID_CMD) {
        return;
    }
    if (!data || data_len != sizeof(microduck_link_frame_t)) {
        ESP_LOGW(kTag, "bad cmd frame len=%u", (unsigned)data_len);
        return;
    }
    microduck_link_frame_t frame;
    memcpy(&frame, data, sizeof(frame));
    if (frame.magic != MICRODUCK_LINK_MAGIC || frame.version != MICRODUCK_LINK_VERSION || frame.type != 1) {
        return;
    }
    cmd_bus_set(&frame.cmd, frame.seq);
}

static void hosted_bootstrap_task(void *arg)
{
    (void)arg;
    esp_err_t ret = esp_hosted_init();
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "esp_hosted_init: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ret = esp_hosted_connect_to_slave();
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "connect C5 SDIO slave: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ret = esp_hosted_register_custom_callback(MICRODUCK_MSG_ID_CMD, on_cmd_from_c5);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "register cmd callback: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(kTag, "SDIO cmd link ready (peer-data MSG 0x%08" PRIX32 ")", MICRODUCK_MSG_ID_CMD);
    vTaskDelete(NULL);
}

void cmd_link_start_sdio_ingest(void)
{
#if !CONFIG_MICRODUCK_CMD_SDIO_ENABLE
    return;
#endif
    xTaskCreate(hosted_bootstrap_task, "cmd_sdio", 8192, NULL, 6, NULL);
}