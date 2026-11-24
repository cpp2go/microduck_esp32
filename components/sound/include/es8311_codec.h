// SPDX-License-Identifier: MIT
//
// ES8311 codec control (I2C): DAC playback + ADC microphone.
// NS4150B amp enable is owned by sound.c (PA_CTRL GPIO).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ES8311_I2C_ADDR_0 0x18u
#define ES8311_I2C_ADDR_1 0x19u

typedef struct {
    i2c_master_dev_handle_t i2c;
    bool ready;
    bool mic_on;
} es8311_codec_t;

esp_err_t es8311_codec_probe(i2c_master_dev_handle_t i2c, uint8_t *id1, uint8_t *id2);

/// Reset, clocks, DAC + ADC SDP, analog bias. sample_hz / mclk_hz must match I2S.
esp_err_t es8311_codec_init(es8311_codec_t *dev, i2c_master_dev_handle_t i2c, int sample_hz, int mclk_hz);

esp_err_t es8311_codec_set_dac_volume(es8311_codec_t *dev, int volume_pct);
esp_err_t es8311_codec_mute_dac(es8311_codec_t *dev, bool mute);

/// Enable analog mic + PGA; mic_gain_db is 0/6/12/.../42 (clamped).
esp_err_t es8311_codec_enable_mic(es8311_codec_t *dev, int mic_gain_db);

/// ADC digital volume 0..100 (reg 0x17 scale).
esp_err_t es8311_codec_set_adc_volume(es8311_codec_t *dev, int volume_pct);

#ifdef __cplusplus
}
#endif
