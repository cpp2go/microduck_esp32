// SPDX-License-Identifier: MIT

#include "vision.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *kTag = "vision";

#if CONFIG_MICRODUCK_VISION_ENABLE
static uint8_t *s_buf;
static uint32_t s_w;
static uint32_t s_h;
static uint64_t s_seq;
static int s_mean = -1;
static bool s_up;
#endif

bool vision_init(void)
{
#if !CONFIG_MICRODUCK_VISION_ENABLE
    ESP_LOGW(kTag, "vision disabled");
    return false;
#else
    s_w = CONFIG_MICRODUCK_VISION_WIDTH;
    s_h = CONFIG_MICRODUCK_VISION_HEIGHT;
    s_buf = (uint8_t *)calloc(1, (size_t)s_w * (size_t)s_h);
    if (!s_buf) {
        ESP_LOGE(kTag, "frame buffer alloc failed");
        return false;
    }
    s_up = true;
    ESP_LOGI(kTag, "vision ready backend=%s %ux%u (mediad/V4L2/YOLO not on P4)",
             vision_backend_name(), (unsigned)s_w, (unsigned)s_h);
#if CONFIG_MICRODUCK_VISION_CSI_HOOK
    ESP_LOGW(kTag, "CSI hook compiled but not wired — see docs/vision-sound-port.md");
#endif
    return true;
#endif
}

bool vision_available(void)
{
#if CONFIG_MICRODUCK_VISION_ENABLE
    return s_up;
#else
    return false;
#endif
}

const char *vision_backend_name(void)
{
#if !CONFIG_MICRODUCK_VISION_ENABLE
    return "disabled";
#elif CONFIG_MICRODUCK_VISION_CSI_HOOK
    return "synthetic+csi-hook";
#else
    return "synthetic";
#endif
}

bool vision_grab(vision_frame_t *out)
{
#if !CONFIG_MICRODUCK_VISION_ENABLE
    (void)out;
    return false;
#else
    if (!s_up || !out || !s_buf) {
        return false;
    }
    /* Moving diagonal stripe so mean luma changes over time — useful for shell bring-up. */
    ++s_seq;
    const unsigned phase = (unsigned)(s_seq % s_w);
    long sum = 0;
    for (uint32_t y = 0; y < s_h; ++y) {
        for (uint32_t x = 0; x < s_w; ++x) {
            uint8_t v = (uint8_t)(((x + phase) ^ (y * 3u)) & 0xFFu);
            s_buf[y * s_w + x] = v;
            sum += v;
        }
    }
    s_mean = (int)(sum / (long)(s_w * s_h));
    out->width = s_w;
    out->height = s_h;
    out->stride = s_w;
    out->seq = s_seq;
    out->ts_us = esp_timer_get_time();
    out->gray = s_buf;
    return true;
#endif
}

void vision_release(void) { /* synthetic buffer is persistent */ }

bool vision_detect_ball(vision_detect_t *out)
{
    if (!out) {
        return false;
    }
    /* Original duck-detect/YOLO needs camera + NPU/CPU YOLO — not ported. */
    out->present = false;
    out->x = out->y = out->score = 0.f;
    return true;
}

bool vision_detect_pet(vision_detect_t *out)
{
    if (!out) {
        return false;
    }
    out->present = false;
    out->x = out->y = out->score = 0.f;
    return true;
}

int vision_mean_luma(void)
{
#if CONFIG_MICRODUCK_VISION_ENABLE
    return s_mean;
#else
    return -1;
#endif
}
