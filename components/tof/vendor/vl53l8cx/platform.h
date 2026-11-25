/*
 * ESP-IDF I2C platform layer for the ST VL53L8CX ULD — microduck_esp32.
 * Adapted from pollen-robotics/microduck tof/vendor (Linux i2c-dev) to
 * ESP-IDF i2c_master. Same hook names the ULD expects.
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"

typedef struct {
    /* 8-bit I2C address (7-bit << 1), the format the ULD expects. */
    uint16_t address;
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_bus_handle_t i2c_bus;
    uint8_t owns_bus; /* 1 if platform opened the bus */
} VL53L8CX_Platform;

#define VL53L8CX_NB_TARGET_PER_ZONE 1U

#define VL53L8CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L8CX_DISABLE_NB_SPADS_ENABLED
#define VL53L8CX_DISABLE_NB_TARGET_DETECTED
#define VL53L8CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L8CX_DISABLE_RANGE_SIGMA_MM
#define VL53L8CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L8CX_DISABLE_MOTION_INDICATOR

uint8_t RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values,
                uint32_t size);
uint8_t WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values,
                uint32_t size);
void SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs);

#endif /* _PLATFORM_H_ */
