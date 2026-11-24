// SPDX-License-Identifier: MIT
//
// Minimal ES8311 DAC bring-up for playback (I2C control). Companion amp is
// NS4150B (GPIO enable), driven from sound.c — not part of this codec API.
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
} es8311_dac_t;

/// Attach a device handle already added to an I2C master bus.
esp_err_t es8311_dac_init(es8311_dac_t *dev, i2c_master_dev_handle_t i2c, int sample_hz,
                          int mclk_hz);

/// 0..100 → DAC volume register.
esp_err_t es8311_dac_set_volume(es8311_dac_t *dev, int volume_pct);

esp_err_t es8311_dac_mute(es8311_dac_t *dev, bool mute);

/// Soft probe: read chip id registers (expect 0x83 / 0x11).
esp_err_t es8311_dac_probe(i2c_master_dev_handle_t i2c, uint8_t *id1, uint8_t *id2);

#ifdef __cplusplus
}
#endif
