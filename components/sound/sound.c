// SPDX-License-Identifier: MIT

#include "sound.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_MICRODUCK_SOUND_I2S_ENABLE
#include "driver/i2s_std.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *kTag = "sound";

/// Tags aligned with robotd emotions / voice bank directories (subset).
static const char *const kTags[] = {
    "yes", "no", "sad", "angry", "curious", "excited", "laugh", "mmh", "mock",
    "impatient", "defiant", "devastated", "yes_fast", "hello", "bye",
};

static bool s_up;
#if CONFIG_MICRODUCK_SOUND_I2S_ENABLE
static i2s_chan_handle_t s_tx;
#endif

static float tag_hz(const char *tag)
{
    /* Distinct pitches so cues are recognizable without WAV assets. */
    unsigned h = 0;
    for (const char *p = tag; *p; ++p) {
        h = h * 33u + (unsigned)(uint8_t)*p;
    }
    return 440.0f + (float)(h % 500);
}

bool sound_init(void)
{
#if !CONFIG_MICRODUCK_SOUND_ENABLE
    ESP_LOGW(kTag, "sound disabled");
    return false;
#endif
#if CONFIG_MICRODUCK_SOUND_I2S_ENABLE
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chan, &s_tx, NULL) != ESP_OK) {
        ESP_LOGE(kTag, "i2s channel failed — falling back to log-only");
    } else {
        i2s_std_config_t std = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg =
                {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = CONFIG_MICRODUCK_SOUND_I2S_BCLK_GPIO,
                    .ws = CONFIG_MICRODUCK_SOUND_I2S_WS_GPIO,
                    .dout = CONFIG_MICRODUCK_SOUND_I2S_DOUT_GPIO,
                    .din = I2S_GPIO_UNUSED,
                    .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
                },
        };
        if (i2s_channel_init_std_mode(s_tx, &std) != ESP_OK || i2s_channel_enable(s_tx) != ESP_OK) {
            ESP_LOGE(kTag, "i2s init failed — log-only cues");
            s_tx = NULL;
        } else {
            ESP_LOGI(kTag, "I2S TX ready BCLK=%d WS=%d DOUT=%d", CONFIG_MICRODUCK_SOUND_I2S_BCLK_GPIO,
                     CONFIG_MICRODUCK_SOUND_I2S_WS_GPIO, CONFIG_MICRODUCK_SOUND_I2S_DOUT_GPIO);
        }
    }
#endif
    s_up = true;
    ESP_LOGI(kTag, "sound cues ready (%d tags)%s", (int)(sizeof(kTags) / sizeof(kTags[0])),
#if CONFIG_MICRODUCK_SOUND_I2S_ENABLE
             s_tx ? "" : " [log-only]"
#else
             " [log-only]"
#endif
    );
    return true;
}

bool sound_available(void) { return s_up; }

const char *const *sound_tags(int *count_out)
{
    if (count_out) {
        *count_out = (int)(sizeof(kTags) / sizeof(kTags[0]));
    }
    return kTags;
}

bool sound_play(const char *tag)
{
    if (!s_up || !tag || !*tag) {
        return false;
    }
    const float hz = tag_hz(tag);
    ESP_LOGI(kTag, "play tag='%s' (%.0f Hz)", tag, (double)hz);

#if CONFIG_MICRODUCK_SOUND_I2S_ENABLE
    if (s_tx) {
        enum { N = 1600 }; /* 100 ms @ 16 kHz */
        int16_t buf[N];
        for (int i = 0; i < N; ++i) {
            float t = (float)i / 16000.0f;
            float env = (i < 80) ? (i / 80.0f) : ((i > N - 80) ? ((N - i) / 80.0f) : 1.0f);
            buf[i] = (int16_t)(sinf(2.0f * (float)M_PI * hz * t) * 8000.0f * env);
        }
        size_t written = 0;
        (void)i2s_channel_write(s_tx, buf, sizeof(buf), &written, pdMS_TO_TICKS(200));
    }
#endif
    return true;
}
