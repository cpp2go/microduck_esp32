// SPDX-License-Identifier: MIT

#include "sound.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "es8311_dac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *kTag = "sound";

/// Tags aligned with robotd emotions / voice bank directories (subset).
static const char *const kTags[] = {
    "yes", "no", "sad", "angry", "curious", "excited", "laugh", "mmh", "mock",
    "impatient", "defiant", "devastated", "yes_fast", "hello", "bye",
};

enum { kSampleHz = 16000, kMclkMultiple = 256, kMclkHz = kSampleHz * kMclkMultiple };

static bool s_up;
static bool s_hw;
static i2s_chan_handle_t s_tx;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static es8311_dac_t s_codec;

static float tag_hz(const char *tag)
{
    unsigned h = 0;
    for (const char *p = tag; *p; ++p) {
        h = h * 33u + (unsigned)(uint8_t)*p;
    }
    return 440.0f + (float)(h % 500);
}

static int tag_ms(const char *tag)
{
    if (!strcmp(tag, "yes_fast") || !strcmp(tag, "mmh")) {
        return 80;
    }
    if (!strcmp(tag, "bye") || !strcmp(tag, "sad") || !strcmp(tag, "devastated")) {
        return 220;
    }
    return 140;
}

static void pa_set(bool on)
{
#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
    const int pin = CONFIG_MICRODUCK_SOUND_PA_GPIO;
    if (pin < 0) {
        return;
    }
    gpio_set_level((gpio_num_t)pin, on ? 1 : 0);
#else
    (void)on;
#endif
}

#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
static bool init_pa(void)
{
    const int pin = CONFIG_MICRODUCK_SOUND_PA_GPIO;
    if (pin < 0) {
        ESP_LOGI(kTag, "NS4150B PA_CTRL unused (hard-wired?)");
        return true;
    }
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) {
        ESP_LOGE(kTag, "PA_CTRL gpio%d config failed", pin);
        return false;
    }
    gpio_set_level((gpio_num_t)pin, 0);
    ESP_LOGI(kTag, "NS4150B PA_CTRL gpio%d", pin);
    return true;
}

static bool init_i2c_codec(void)
{
    i2c_master_bus_config_t bus = {
        .i2c_port = CONFIG_MICRODUCK_SOUND_I2C_PORT,
        .sda_io_num = CONFIG_MICRODUCK_SOUND_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_MICRODUCK_SOUND_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };
    if (i2c_new_master_bus(&bus, &s_i2c_bus) != ESP_OK) {
        ESP_LOGE(kTag, "I2C bus failed");
        return false;
    }
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_MICRODUCK_SOUND_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(s_i2c_bus, &dev, &s_i2c_dev) != ESP_OK) {
        ESP_LOGE(kTag, "I2C add ES8311 failed");
        return false;
    }
    if (es8311_dac_init(&s_codec, s_i2c_dev, kSampleHz, kMclkHz) != ESP_OK) {
        ESP_LOGE(kTag, "ES8311 init failed");
        return false;
    }
    (void)es8311_dac_set_volume(&s_codec, CONFIG_MICRODUCK_SOUND_VOLUME);
    ESP_LOGI(kTag, "ES8311 ready I2C%d SDA=%d SCL=%d addr=0x%02x vol=%d", CONFIG_MICRODUCK_SOUND_I2C_PORT,
             CONFIG_MICRODUCK_SOUND_I2C_SDA_GPIO, CONFIG_MICRODUCK_SOUND_I2C_SCL_GPIO, CONFIG_MICRODUCK_SOUND_I2C_ADDR,
             CONFIG_MICRODUCK_SOUND_VOLUME);
    return true;
}

static bool init_i2s(void)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan.auto_clear = true;
    if (i2s_new_channel(&chan, &s_tx, NULL) != ESP_OK) {
        ESP_LOGE(kTag, "i2s channel failed");
        return false;
    }
    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleHz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = CONFIG_MICRODUCK_SOUND_I2S_MCLK_GPIO,
                .bclk = CONFIG_MICRODUCK_SOUND_I2S_BCLK_GPIO,
                .ws = CONFIG_MICRODUCK_SOUND_I2S_WS_GPIO,
                .dout = CONFIG_MICRODUCK_SOUND_I2S_DOUT_GPIO,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
            },
    };
    std.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    if (i2s_channel_init_std_mode(s_tx, &std) != ESP_OK || i2s_channel_enable(s_tx) != ESP_OK) {
        ESP_LOGE(kTag, "i2s init/enable failed");
        s_tx = NULL;
        return false;
    }
    ESP_LOGI(kTag, "I2S TX MCLK=%d BCLK=%d WS=%d DOUT=%d @ %d Hz", CONFIG_MICRODUCK_SOUND_I2S_MCLK_GPIO,
             CONFIG_MICRODUCK_SOUND_I2S_BCLK_GPIO, CONFIG_MICRODUCK_SOUND_I2S_WS_GPIO,
             CONFIG_MICRODUCK_SOUND_I2S_DOUT_GPIO, kSampleHz);
    return true;
}
#endif /* CONFIG_MICRODUCK_SOUND_HW_ENABLE */

bool sound_init(void)
{
#if !CONFIG_MICRODUCK_SOUND_ENABLE
    ESP_LOGW(kTag, "sound disabled");
    return false;
#endif

    s_hw = false;
#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
    const bool pa_ok = init_pa();
    const bool codec_ok = init_i2c_codec();
    const bool i2s_ok = init_i2s();
    s_hw = pa_ok && codec_ok && i2s_ok && s_tx;
    if (s_hw) {
        pa_set(true);
    } else {
        ESP_LOGW(kTag, "HW path incomplete — cues are log-only");
    }
#else
    ESP_LOGI(kTag, "HW path off (menuconfig) — log-only cues");
#endif

    s_up = true;
    ESP_LOGI(kTag, "sound ready (%d tags) backend=%s", (int)(sizeof(kTags) / sizeof(kTags[0])),
             s_hw ? "ES8311+NS4150B" : "log-only");
    return true;
}

bool sound_available(void) { return s_up; }

bool sound_hw_ready(void) { return s_hw; }

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
    const int ms = tag_ms(tag);
    ESP_LOGI(kTag, "play tag='%s' (%.0f Hz, %d ms)%s", tag, (double)hz, ms, s_hw ? "" : " [log-only]");

#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
    if (s_hw && s_tx) {
        pa_set(true);
        (void)es8311_dac_mute(&s_codec, false);
        const int frames = (kSampleHz * ms) / 1000;
        /* Stereo interleaved L/R. Keep stack modest — chunk in blocks. */
        enum { BLOCK = 256 };
        int16_t buf[BLOCK * 2];
        int produced = 0;
        while (produced < frames) {
            const int n = (frames - produced) > BLOCK ? BLOCK : (frames - produced);
            for (int i = 0; i < n; ++i) {
                const int idx = produced + i;
                float t = (float)idx / (float)kSampleHz;
                float env = 1.0f;
                const int attack = frames / 10;
                const int release = frames / 5;
                if (idx < attack) {
                    env = (float)idx / (float)(attack ? attack : 1);
                } else if (idx > frames - release) {
                    env = (float)(frames - idx) / (float)(release ? release : 1);
                }
                /* Fundamental + soft 2nd harmonic — slightly less "beep-y". */
                float s = sinf(2.0f * (float)M_PI * hz * t);
                s += 0.25f * sinf(4.0f * (float)M_PI * hz * t);
                int16_t v = (int16_t)(s * 10000.0f * env);
                buf[i * 2] = v;
                buf[i * 2 + 1] = v;
            }
            size_t written = 0;
            (void)i2s_channel_write(s_tx, buf, (size_t)n * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(500));
            produced += n;
        }
    }
#endif
    return true;
}
