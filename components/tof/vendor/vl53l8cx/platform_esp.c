/*
 * ESP-IDF I2C master platform for ST VL53L8CX ULD.
 * Chunked register access matching the original Linux platform.c behaviour.
 */

#include "platform.h"

#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CHUNK 2048u

static uint8_t wr_buf[CHUNK + 2];

uint8_t RdByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t *value)
{
    if (!p || !p->i2c_dev || !value) {
        return 255;
    }
    uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return i2c_master_transmit_receive(p->i2c_dev, addr, 2, value, 1, 200) == ESP_OK ? 0 : 255;
}

uint8_t WrByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t value)
{
    if (!p || !p->i2c_dev) {
        return 255;
    }
    uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)reg, value};
    return i2c_master_transmit(p->i2c_dev, buf, 3, 200) == ESP_OK ? 0 : 255;
}

uint8_t RdMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *values, uint32_t size)
{
    if (!p || !p->i2c_dev || !values) {
        return 255;
    }
    for (uint32_t off = 0; off < size;) {
        uint32_t n = size - off > CHUNK ? CHUNK : size - off;
        uint8_t addr[2] = {(uint8_t)((reg + off) >> 8), (uint8_t)(reg + off)};
        if (i2c_master_transmit_receive(p->i2c_dev, addr, 2, values + off, n, 500) != ESP_OK) {
            return 255;
        }
        off += n;
    }
    return 0;
}

uint8_t WrMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *values, uint32_t size)
{
    if (!p || !p->i2c_dev || !values) {
        return 255;
    }
    for (uint32_t off = 0; off < size;) {
        uint32_t n = size - off > CHUNK ? CHUNK : size - off;
        wr_buf[0] = (uint8_t)((reg + off) >> 8);
        wr_buf[1] = (uint8_t)(reg + off);
        memcpy(wr_buf + 2, values + off, n);
        if (i2c_master_transmit(p->i2c_dev, wr_buf, n + 2, 500) != ESP_OK) {
            return 255;
        }
        off += n;
    }
    return 0;
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    for (uint16_t i = 0; i < size; i += 4) {
        uint8_t t0 = buffer[i];
        uint8_t t1 = buffer[i + 1];
        buffer[i] = buffer[i + 3];
        buffer[i + 1] = buffer[i + 2];
        buffer[i + 2] = t1;
        buffer[i + 3] = t0;
    }
}

uint8_t WaitMs(VL53L8CX_Platform *p, uint32_t TimeMs)
{
    (void)p;
    if (TimeMs == 0) {
        return 0;
    }
    if (TimeMs < 10) {
        esp_rom_delay_us(TimeMs * 1000);
    } else {
        vTaskDelay(pdMS_TO_TICKS(TimeMs));
    }
    return 0;
}
