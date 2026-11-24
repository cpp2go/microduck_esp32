// SPDX-License-Identifier: MIT
//
// Register program derived from Espressif esp-bsp es8311 (Apache-2.0),
// adapted to IDF i2c_master_* for DAC + ADC (mic) on ESP32-P4.

#include "es8311_codec.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "es8311";

#define ES8311_RESET_REG00       0x00
#define ES8311_CLK_MANAGER_REG01 0x01
#define ES8311_CLK_MANAGER_REG02 0x02
#define ES8311_CLK_MANAGER_REG03 0x03
#define ES8311_CLK_MANAGER_REG04 0x04
#define ES8311_CLK_MANAGER_REG05 0x05
#define ES8311_CLK_MANAGER_REG06 0x06
#define ES8311_CLK_MANAGER_REG07 0x07
#define ES8311_CLK_MANAGER_REG08 0x08
#define ES8311_SDPIN_REG09       0x09
#define ES8311_SDPOUT_REG0A      0x0A
#define ES8311_SYSTEM_REG0D      0x0D
#define ES8311_SYSTEM_REG0E      0x0E
#define ES8311_SYSTEM_REG12      0x12
#define ES8311_SYSTEM_REG13      0x13
#define ES8311_SYSTEM_REG14      0x14
#define ES8311_ADC_REG17         0x17
#define ES8311_ADC_REG1C         0x1C
#define ES8311_DAC_REG31         0x31
#define ES8311_DAC_REG32         0x32
#define ES8311_DAC_REG37         0x37
#define ES8311_CHD1_REGFD        0xFD
#define ES8311_CHD2_REGFE        0xFE

struct coeff_div {
    uint32_t mclk;
    uint32_t rate;
    uint8_t pre_div;
    uint8_t pre_multi;
    uint8_t adc_div;
    uint8_t dac_div;
    uint8_t fs_mode;
    uint8_t lrck_h;
    uint8_t lrck_l;
    uint8_t bclk_div;
    uint8_t adc_osr;
    uint8_t dac_osr;
};

static const struct coeff_div kCoeff[] = {
    {12288000, 16000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000, 16000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 16000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {4096000, 16000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 16000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2048000, 16000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {12288000, 48000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 48000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
};

static esp_err_t wr(i2c_master_dev_handle_t i2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(i2c, buf, sizeof(buf), 200);
}

static esp_err_t rd(i2c_master_dev_handle_t i2c, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(i2c, &reg, 1, val, 1, 200);
}

esp_err_t es8311_codec_probe(i2c_master_dev_handle_t i2c, uint8_t *id1, uint8_t *id2)
{
    uint8_t a = 0, b = 0;
    ESP_RETURN_ON_ERROR(rd(i2c, ES8311_CHD1_REGFD, &a), TAG, "id1");
    ESP_RETURN_ON_ERROR(rd(i2c, ES8311_CHD2_REGFE, &b), TAG, "id2");
    if (id1) {
        *id1 = a;
    }
    if (id2) {
        *id2 = b;
    }
    return ESP_OK;
}

static int find_coeff(uint32_t mclk, uint32_t rate)
{
    for (int i = 0; i < (int)(sizeof(kCoeff) / sizeof(kCoeff[0])); ++i) {
        if (kCoeff[i].mclk == mclk && kCoeff[i].rate == rate) {
            return i;
        }
    }
    return -1;
}

static esp_err_t apply_coeff(i2c_master_dev_handle_t i2c, const struct coeff_div *c)
{
    uint8_t regv = 0;
    ESP_RETURN_ON_ERROR(rd(i2c, ES8311_CLK_MANAGER_REG02, &regv), TAG, "r02");
    regv &= 0x07;
    regv |= (uint8_t)((c->pre_div - 1) << 5);
    regv |= (uint8_t)(c->pre_multi << 3);
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG02, regv), TAG, "w02");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG03, (uint8_t)((c->fs_mode << 6) | c->adc_osr)), TAG, "w03");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG04, c->dac_osr), TAG, "w04");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG05, (uint8_t)(((c->adc_div - 1) << 4) | (c->dac_div - 1))), TAG,
                        "w05");
    ESP_RETURN_ON_ERROR(rd(i2c, ES8311_CLK_MANAGER_REG06, &regv), TAG, "r06");
    regv &= 0xE0;
    regv |= (c->bclk_div < 19) ? (uint8_t)(c->bclk_div - 1) : c->bclk_div;
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG06, regv), TAG, "w06");
    ESP_RETURN_ON_ERROR(rd(i2c, ES8311_CLK_MANAGER_REG07, &regv), TAG, "r07");
    regv &= 0xC0;
    regv |= c->lrck_h;
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG07, regv), TAG, "w07");
    return wr(i2c, ES8311_CLK_MANAGER_REG08, c->lrck_l);
}

esp_err_t es8311_codec_init(es8311_codec_t *dev, i2c_master_dev_handle_t i2c, int sample_hz, int mclk_hz)
{
    ESP_RETURN_ON_FALSE(dev && i2c, ESP_ERR_INVALID_ARG, TAG, "null");
    dev->i2c = i2c;
    dev->ready = false;
    dev->mic_on = false;

    uint8_t id1 = 0, id2 = 0;
    ESP_RETURN_ON_ERROR(es8311_codec_probe(i2c, &id1, &id2), TAG, "probe");
    if (id1 != 0x83 || id2 != 0x11) {
        ESP_LOGW(TAG, "unexpected chip id %02x %02x (continuing)", id1, id2);
    } else {
        ESP_LOGI(TAG, "ES8311 detected");
    }

    const int coeff = find_coeff((uint32_t)mclk_hz, (uint32_t)sample_hz);
    ESP_RETURN_ON_FALSE(coeff >= 0, ESP_ERR_INVALID_ARG, TAG, "no coeff for mclk=%d rate=%d", mclk_hz, sample_hz);

    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_RESET_REG00, 0x1F), TAG, "rst");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_RESET_REG00, 0x00), TAG, "rst0");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_RESET_REG00, 0x80), TAG, "power");

    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_CLK_MANAGER_REG01, 0x3F), TAG, "clk01");
    ESP_RETURN_ON_ERROR(apply_coeff(i2c, &kCoeff[coeff]), TAG, "coeff");

    uint8_t reg00 = 0;
    ESP_RETURN_ON_ERROR(rd(i2c, ES8311_RESET_REG00, &reg00), TAG, "r00");
    reg00 &= 0xBF; /* slave */
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_RESET_REG00, reg00), TAG, "slave");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_SDPIN_REG09, 0x0C), TAG, "sdp-in");  /* 16-bit DAC */
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_SDPOUT_REG0A, 0x0C), TAG, "sdp-out"); /* 16-bit ADC */

    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_SYSTEM_REG0D, 0x01), TAG, "an");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_SYSTEM_REG0E, 0x02), TAG, "pga");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_SYSTEM_REG12, 0x00), TAG, "dacpwr");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_SYSTEM_REG13, 0x10), TAG, "hp");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_ADC_REG1C, 0x6A), TAG, "adc-eq");
    ESP_RETURN_ON_ERROR(wr(i2c, ES8311_DAC_REG37, 0x08), TAG, "dac-eq");

    ESP_RETURN_ON_ERROR(es8311_codec_set_dac_volume(dev, 70), TAG, "vol");
    ESP_RETURN_ON_ERROR(es8311_codec_mute_dac(dev, false), TAG, "unmute");

    dev->ready = true;
    return ESP_OK;
}

esp_err_t es8311_codec_set_dac_volume(es8311_codec_t *dev, int volume_pct)
{
    ESP_RETURN_ON_FALSE(dev && dev->i2c, ESP_ERR_INVALID_ARG, TAG, "null");
    if (volume_pct < 0) {
        volume_pct = 0;
    }
    if (volume_pct > 100) {
        volume_pct = 100;
    }
    const int reg32 = (volume_pct == 0) ? 0 : ((volume_pct * 256 / 100) - 1);
    return wr(dev->i2c, ES8311_DAC_REG32, (uint8_t)reg32);
}

esp_err_t es8311_codec_mute_dac(es8311_codec_t *dev, bool mute)
{
    ESP_RETURN_ON_FALSE(dev && dev->i2c, ESP_ERR_INVALID_ARG, TAG, "null");
    uint8_t reg31 = 0;
    ESP_RETURN_ON_ERROR(rd(dev->i2c, ES8311_DAC_REG31, &reg31), TAG, "r31");
    if (mute) {
        reg31 |= 0x20;
    } else {
        reg31 &= (uint8_t)~0x20;
    }
    return wr(dev->i2c, ES8311_DAC_REG31, reg31);
}

esp_err_t es8311_codec_enable_mic(es8311_codec_t *dev, int mic_gain_db)
{
    ESP_RETURN_ON_FALSE(dev && dev->i2c, ESP_ERR_INVALID_ARG, TAG, "null");
    /* PGA steps: 0,6,...,42 dB → bits in REG14[3:0] style used by BSP (0x1A base). */
    if (mic_gain_db < 0) {
        mic_gain_db = 0;
    }
    if (mic_gain_db > 42) {
        mic_gain_db = 42;
    }
    const int step = mic_gain_db / 6; /* 0..7 */
    const uint8_t reg14 = (uint8_t)(0x10 | (step & 0x07)); /* analog mic + PGA */
    ESP_RETURN_ON_ERROR(wr(dev->i2c, ES8311_SYSTEM_REG14, reg14), TAG, "mic");
    ESP_RETURN_ON_ERROR(wr(dev->i2c, ES8311_ADC_REG17, 0xC8), TAG, "adc-gain");
    ESP_RETURN_ON_ERROR(wr(dev->i2c, ES8311_SYSTEM_REG0E, 0x02), TAG, "adc-mod");
    dev->mic_on = true;
    ESP_LOGI(TAG, "mic enabled PGA~%ddB", step * 6);
    return ESP_OK;
}

esp_err_t es8311_codec_set_adc_volume(es8311_codec_t *dev, int volume_pct)
{
    ESP_RETURN_ON_FALSE(dev && dev->i2c, ESP_ERR_INVALID_ARG, TAG, "null");
    if (volume_pct < 0) {
        volume_pct = 0;
    }
    if (volume_pct > 100) {
        volume_pct = 100;
    }
    /* Map 0..100 → a usable digital gain around the BSP default 0xC8. */
    const uint8_t reg = (uint8_t)(0x40 + (volume_pct * 0xBF) / 100);
    return wr(dev->i2c, ES8311_ADC_REG17, reg);
}
