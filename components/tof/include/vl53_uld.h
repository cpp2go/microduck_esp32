// SPDX-License-Identifier: MIT
// Flat VL53L8CX API (ESP port of microduck tof shim). Available when
// CONFIG_MICRODUCK_TOF_ULD is enabled.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int vl8_open(int i2c_port, int sda_gpio, int scl_gpio, uint8_t addr_7bit);
void vl8_close(void);
int vl8_is_alive(void);
int vl8_init(void);
int vl8_start(uint8_t freq_hz);
int vl8_stop(void);
int vl8_data_ready(void);
int vl8_get_frame(int16_t *dist_mm, uint8_t *status);

#ifdef __cplusplus
}
#endif
