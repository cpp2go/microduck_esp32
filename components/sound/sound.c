// SPDX-License-Identifier: MIT

#include "sound.h"
#include "sound_bank.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "es8311_codec.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *kTag = "sound";

static const char *const kTags[] = {
    "yes", "no", "sad", "angry", "curious", "excited", "laugh", "mmh", "mock",
    "impatient", "defiant", "devastated", "yes_fast", "hello", "bye",
};

enum { kSampleHz = 16000, kMclkMultiple = 256, kMclkHz = kSampleHz * kMclkMultiple };

static bool s_up;
static bool s_hw;
static bool s_mic;
static i2s_chan_handle_t s_tx;
static i2s_chan_handle_t s_rx;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static es8311_codec_t s_codec;

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
    if (es8311_codec_init(&s_codec, s_i2c_dev, kSampleHz, kMclkHz) != ESP_OK) {
        ESP_LOGE(kTag, "ES8311 init failed");
        return false;
    }
    (void)es8311_codec_set_dac_volume(&s_codec, CONFIG_MICRODUCK_SOUND_VOLUME);
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    if (es8311_codec_enable_mic(&s_codec, CONFIG_MICRODUCK_SOUND_MIC_GAIN_DB) != ESP_OK) {
        ESP_LOGW(kTag, "mic enable failed — record unavailable");
    }
#endif
    ESP_LOGI(kTag, "ES8311 ready I2C%d SDA=%d SCL=%d addr=0x%02x vol=%d", CONFIG_MICRODUCK_SOUND_I2C_PORT,
             CONFIG_MICRODUCK_SOUND_I2C_SDA_GPIO, CONFIG_MICRODUCK_SOUND_I2C_SCL_GPIO, CONFIG_MICRODUCK_SOUND_I2C_ADDR,
             CONFIG_MICRODUCK_SOUND_VOLUME);
    return true;
}

static bool init_i2s(void)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan.auto_clear = true;
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    if (i2s_new_channel(&chan, &s_tx, &s_rx) != ESP_OK) {
        ESP_LOGE(kTag, "i2s duplex channel failed");
        return false;
    }
#else
    if (i2s_new_channel(&chan, &s_tx, NULL) != ESP_OK) {
        ESP_LOGE(kTag, "i2s TX channel failed");
        return false;
    }
#endif

    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleHz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = CONFIG_MICRODUCK_SOUND_I2S_MCLK_GPIO,
                .bclk = CONFIG_MICRODUCK_SOUND_I2S_BCLK_GPIO,
                .ws = CONFIG_MICRODUCK_SOUND_I2S_WS_GPIO,
                .dout = CONFIG_MICRODUCK_SOUND_I2S_DOUT_GPIO,
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
                .din = CONFIG_MICRODUCK_SOUND_I2S_DIN_GPIO,
#else
                .din = I2S_GPIO_UNUSED,
#endif
                .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
            },
    };
    std.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    if (i2s_channel_init_std_mode(s_tx, &std) != ESP_OK) {
        ESP_LOGE(kTag, "i2s TX init failed");
        s_tx = NULL;
        return false;
    }
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    /* RX shares the controller; clocks already set by TX. Clear mclk on RX gpio to avoid conflict. */
    i2s_std_config_t rx_std = std;
    rx_std.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    rx_std.gpio_cfg.dout = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(s_rx, &rx_std) != ESP_OK) {
        ESP_LOGE(kTag, "i2s RX init failed — mic off");
        s_rx = NULL;
    }
#endif
    if (i2s_channel_enable(s_tx) != ESP_OK) {
        ESP_LOGE(kTag, "i2s TX enable failed");
        s_tx = NULL;
        return false;
    }
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    if (s_rx && i2s_channel_enable(s_rx) != ESP_OK) {
        ESP_LOGE(kTag, "i2s RX enable failed — mic off");
        s_rx = NULL;
    }
#endif

    ESP_LOGI(kTag, "I2S TX MCLK=%d BCLK=%d WS=%d DOUT=%d @ %d Hz", CONFIG_MICRODUCK_SOUND_I2S_MCLK_GPIO,
             CONFIG_MICRODUCK_SOUND_I2S_BCLK_GPIO, CONFIG_MICRODUCK_SOUND_I2S_WS_GPIO,
             CONFIG_MICRODUCK_SOUND_I2S_DOUT_GPIO, kSampleHz);
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    if (s_rx) {
        ESP_LOGI(kTag, "I2S RX DIN=%d (mic)", CONFIG_MICRODUCK_SOUND_I2S_DIN_GPIO);
    }
#endif
    return s_tx != NULL;
}
#endif /* CONFIG_MICRODUCK_SOUND_HW_ENABLE */

bool sound_init(void)
{
#if !CONFIG_MICRODUCK_SOUND_ENABLE
    ESP_LOGW(kTag, "sound disabled");
    return false;
#endif

    s_hw = false;
    s_mic = false;
#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
    const bool pa_ok = init_pa();
    const bool codec_ok = init_i2c_codec();
    const bool i2s_ok = init_i2s();
    s_hw = pa_ok && codec_ok && i2s_ok && s_tx;
#if CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    s_mic = s_hw && s_rx && s_codec.mic_on;
#endif
    if (s_hw) {
        pa_set(true);
    } else {
        ESP_LOGW(kTag, "HW path incomplete — cues are log-only");
    }
#else
    ESP_LOGI(kTag, "HW path off (menuconfig) — log-only cues");
#endif

    s_up = true;
    ESP_LOGI(kTag, "sound ready (%d tags) play=%s mic=%s", (int)(sizeof(kTags) / sizeof(kTags[0])),
             s_hw ? "ES8311+NS4150B" : "log-only", s_mic ? "on" : "off");
    return true;
}

bool sound_available(void) { return s_up; }

bool sound_hw_ready(void) { return s_hw; }

bool sound_mic_ready(void) { return s_mic; }

int sound_sample_rate(void) { return kSampleHz; }

const char *const *sound_tags(int *count_out)
{
    if (count_out) {
        *count_out = (int)(sizeof(kTags) / sizeof(kTags[0]));
    }
    return kTags;
}

static bool play_pcm_mono(const int16_t *mono, int frames)
{
#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
    if (!s_hw || !s_tx || !mono || frames <= 0) {
        return false;
    }
    pa_set(true);
    (void)es8311_codec_mute_dac(&s_codec, false);
    enum { BLOCK = 256 };
    int16_t buf[BLOCK * 2];
    int produced = 0;
    while (produced < frames) {
        const int n = (frames - produced) > BLOCK ? BLOCK : (frames - produced);
        for (int i = 0; i < n; ++i) {
            const int16_t v = mono[produced + i];
            buf[i * 2] = v;
            buf[i * 2 + 1] = v;
        }
        size_t written = 0;
        (void)i2s_channel_write(s_tx, buf, (size_t)n * 2 * sizeof(int16_t), &written,
                                pdMS_TO_TICKS(500));
        produced += n;
    }
    return true;
#else
    (void)mono;
    (void)frames;
    return false;
#endif
}

static bool play_synth_hz(float hz, int ms)
{
#if CONFIG_MICRODUCK_SOUND_HW_ENABLE
    if (!s_hw || !s_tx || hz <= 0.f || ms <= 0) {
        return false;
    }
    pa_set(true);
    (void)es8311_codec_mute_dac(&s_codec, false);
    const int frames = (kSampleHz * ms) / 1000;
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
            float s = sinf(2.0f * (float)M_PI * hz * t);
            s += 0.25f * sinf(4.0f * (float)M_PI * hz * t);
            int16_t v = (int16_t)(s * 10000.0f * env);
            buf[i * 2] = v;
            buf[i * 2 + 1] = v;
        }
        size_t written = 0;
        (void)i2s_channel_write(s_tx, buf, (size_t)n * 2 * sizeof(int16_t), &written,
                                pdMS_TO_TICKS(500));
        produced += n;
    }
    return true;
#else
    (void)hz;
    (void)ms;
    return false;
#endif
}

bool sound_bank_has(const char *tag) { return sound_bank_find(tag) != NULL; }

bool sound_play_tone(float hz, int ms)
{
    if (!s_up || hz <= 0.f || ms <= 0) {
        return false;
    }
    ESP_LOGI(kTag, "tone %.0f Hz %d ms%s", (double)hz, ms, s_hw ? "" : " [log-only]");
    (void)play_synth_hz(hz, ms);
    return true;
}

bool sound_play(const char *tag)
{
    if (!s_up || !tag || !*tag) {
        return false;
    }

    const sound_bank_clip_t *clip = sound_bank_find(tag);
    if (clip && clip->pcm && clip->samples > 0) {
        ESP_LOGI(kTag, "play tag='%s' bank %u samples @%d Hz%s", tag, (unsigned)clip->samples,
                 kSampleHz, s_hw ? "" : " [log-only]");
        (void)play_pcm_mono(clip->pcm, (int)clip->samples);
        return true;
    }

    const float hz = tag_hz(tag);
    const int ms = tag_ms(tag);
    ESP_LOGI(kTag, "play tag='%s' synth (%.0f Hz, %d ms)%s", tag, (double)hz, ms,
             s_hw ? "" : " [log-only]");
    (void)play_synth_hz(hz, ms);
    return true;
}

int sound_record(int16_t *mono, int max_samples, int timeout_ms)
{
    if (!mono || max_samples <= 0) {
        return -1;
    }
#if !CONFIG_MICRODUCK_SOUND_HW_ENABLE || !CONFIG_MICRODUCK_SOUND_MIC_ENABLE
    (void)timeout_ms;
    return -1;
#else
    if (!s_mic || !s_rx) {
        return -1;
    }
    /* Soft-mute PA while capturing to cut speaker→mic loop. */
    pa_set(false);
    (void)es8311_codec_mute_dac(&s_codec, true);

    enum { STEREO_BLOCK = 256 };
    int16_t stereo[STEREO_BLOCK * 2];
    int got = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : 1000);

    while (got < max_samples) {
        const int need = max_samples - got;
        const int nframe = need > STEREO_BLOCK ? STEREO_BLOCK : need;
        size_t bytes = 0;
        const TickType_t left = deadline - xTaskGetTickCount();
        if ((int32_t)left <= 0) {
            break;
        }
        esp_err_t err = i2s_channel_read(s_rx, stereo, (size_t)nframe * 2 * sizeof(int16_t), &bytes, left);
        if (err != ESP_OK || bytes == 0) {
            break;
        }
        const int frames = (int)(bytes / (2 * sizeof(int16_t)));
        for (int i = 0; i < frames && got < max_samples; ++i) {
            mono[got++] = stereo[i * 2]; /* left */
        }
    }

    (void)es8311_codec_mute_dac(&s_codec, false);
    pa_set(true);
    return got;
#endif
}

int sound_mic_level(int window_ms)
{
    if (window_ms <= 0) {
        window_ms = 100;
    }
    if (window_ms > 2000) {
        window_ms = 2000;
    }
    const int n = (kSampleHz * window_ms) / 1000;
    if (n <= 0) {
        return -1;
    }
    int16_t *buf = (int16_t *)malloc((size_t)n * sizeof(int16_t));
    if (!buf) {
        return -1;
    }
    const int got = sound_record(buf, n, window_ms + 200);
    int peak = 0;
    if (got > 0) {
        for (int i = 0; i < got; ++i) {
            int v = buf[i];
            if (v < 0) {
                v = -v;
            }
            if (v > peak) {
                peak = v;
            }
        }
    }
    free(buf);
    return got > 0 ? peak : -1;
}
