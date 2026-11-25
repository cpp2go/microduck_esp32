// SPDX-License-Identifier: MIT
//
// ICM-42688 (SPI) → gyro + projected gravity for the 61-D observation.
// Public contract matches pollen-robotics/microduck duck-control ImuData
// (gyro rad/s, gravity unit vector upright [0,0,-1]). Mahony replaces the
// original imu_to_dxl SFLP quaternion when the IMU is local to the P4.

#include "imu.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace imu {
namespace {

constexpr const char *kTag = "imu";

// Bank 0 registers (TDK ICM-42688-P DS-000347).
constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t REG_TEMP_DATA1 = 0x1D; // burst: temp(2) + accel(6) + gyro(6)
constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_BANK_SEL = 0x76;

constexpr uint8_t WHO_AM_I_42688_P = 0x47;
constexpr uint8_t WHO_AM_I_42688 = 0xDB; // some ICM-42688 markings

// GYRO/ACCEL ODR nibble: 0x07 = 200 Hz, 0x08 = 100 Hz (TDK DS-000347).
constexpr uint8_t GYRO_FS_2000_ODR_200 = 0x07;
constexpr uint8_t ACCEL_FS_2G_ODR_200 = 0x67;
constexpr uint8_t GYRO_FS_2000_ODR_100 = 0x08;
constexpr uint8_t ACCEL_FS_2G_ODR_100 = 0x68;

// ±2000 dps → 16.4 LSB/dps; ±2 g → 16384 LSB/g
constexpr float kGyrLsbDps = 16.4f;
constexpr float kAccLsbG = 16384.0f;
constexpr float kRadPerDps = static_cast<float>(M_PI) / 180.0f;

spi_device_handle_t s_spi = nullptr;
bool s_up = false;
float s_gyro[3] = {0};
float s_accel[3] = {0};
float s_gravity[3] = {0, 0, -1};
portMUX_TYPE s_spin = portMUX_INITIALIZER_UNLOCKED;

constexpr float kTwoKp = 2.0f;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
int64_t s_last_us = 0;

std::atomic<bool> s_fast_running{false};
std::atomic<int64_t> s_fast_ticks{0};
std::atomic<int64_t> s_fast_misses{0};
std::atomic<int64_t> s_fast_min{INT64_MAX};
std::atomic<int64_t> s_fast_max{0};
std::atomic<int64_t> s_fast_sum{0};

bool spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!s_spi || len == 0) {
        return false;
    }
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_transmit(s_spi, &t) == ESP_OK;
}

bool write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
    return spi_xfer(tx, nullptr, 2);
}

bool read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    // SPI read: first byte = reg | 0x80, then len data clocks (dummy TX).
    uint8_t tx[16];
    uint8_t rx[16];
    if (len + 1 > sizeof(tx)) {
        return false;
    }
    memset(tx, 0, len + 1);
    tx[0] = static_cast<uint8_t>(reg | 0x80);
    if (!spi_xfer(tx, rx, len + 1)) {
        return false;
    }
    memcpy(data, rx + 1, len);
    return true;
}

bool read_u8(uint8_t reg, uint8_t *out) { return read_regs(reg, out, 1); }

bool who_am_i_ok(uint8_t id) { return id == WHO_AM_I_42688_P || id == WHO_AM_I_42688; }

} // namespace

bool Init()
{
#if !CONFIG_MICRODUCK_IMU_ENABLE
    ESP_LOGW(kTag, "IMU disabled in sdkconfig — observation gyro/gravity stay synthetic");
    return false;
#endif
    spi_bus_config_t bus = {};
    bus.mosi_io_num = CONFIG_MICRODUCK_IMU_SPI_MOSI_GPIO;
    bus.miso_io_num = CONFIG_MICRODUCK_IMU_SPI_MISO_GPIO;
    bus.sclk_io_num = CONFIG_MICRODUCK_IMU_SPI_SCLK_GPIO;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 32;

    const spi_host_device_t host = static_cast<spi_host_device_t>(CONFIG_MICRODUCK_IMU_SPI_HOST);
    esp_err_t err = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // INVALID_STATE: bus already up (e.g. re-init) — reuse it.
        ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = CONFIG_MICRODUCK_IMU_SPI_HZ;
    dev.mode = 0; // CPOL=0 CPHA=0 — ICM-42688 SPI mode 0 / 3
    dev.spics_io_num = CONFIG_MICRODUCK_IMU_SPI_CS_GPIO;
    dev.queue_size = 1;
    err = spi_bus_add_device(host, &dev, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "SPI device add failed: %s", esp_err_to_name(err));
        return false;
    }

    // Select user bank 0, soft reset, wait.
    (void)write_reg(REG_BANK_SEL, 0x00);
    (void)write_reg(REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t who = 0;
    if (!read_u8(REG_WHO_AM_I, &who) || !who_am_i_ok(who)) {
        ESP_LOGE(kTag, "ICM-42688 WHO_AM_I=0x%02X (want 0x47/0xDB) — IMU stays synthetic", who);
        return false;
    }

    // Gyro ±2000 dps @ 200 Hz, accel ±2 g @ 200 Hz — matches original imu_to_dxl@200 rate.
    if (!write_reg(REG_GYRO_CONFIG0, GYRO_FS_2000_ODR_200) ||
        !write_reg(REG_ACCEL_CONFIG0, ACCEL_FS_2G_ODR_200)) {
        ESP_LOGE(kTag, "config write failed");
        return false;
    }
    (void)GYRO_FS_2000_ODR_100;
    (void)ACCEL_FS_2G_ODR_100;
    // PWR_MGMT0: GYRO_MODE=LN (10), ACCEL_MODE=LN (11) → 0x0B
    if (!write_reg(REG_PWR_MGMT0, 0x0B)) {
        ESP_LOGE(kTag, "PWR_MGMT0 failed");
        return false;
    }
    // Datasheet: wait ≥1 ms after enabling sensors before first valid sample.
    vTaskDelay(pdMS_TO_TICKS(50));

    s_up = true;
    Poll();
    const float mag = std::sqrt(s_accel[0] * s_accel[0] + s_accel[1] * s_accel[1] +
                                s_accel[2] * s_accel[2]);
    ESP_LOGI(kTag,
             "ICM-42688 SPI ready WHO=0x%02X CS=%d SCLK=%d MOSI=%d MISO=%d @%dHz, |accel|=%.3f g",
             who, CONFIG_MICRODUCK_IMU_SPI_CS_GPIO, CONFIG_MICRODUCK_IMU_SPI_SCLK_GPIO,
             CONFIG_MICRODUCK_IMU_SPI_MOSI_GPIO, CONFIG_MICRODUCK_IMU_SPI_MISO_GPIO,
             CONFIG_MICRODUCK_IMU_SPI_HZ, static_cast<double>(mag));
    if (mag < 0.5f || mag > 1.5f) {
        ESP_LOGE(kTag, "accel sanity failed — IMU stays synthetic");
        s_up = false;
        return false;
    }
    return true;
}

void Poll()
{
    if (!s_up) {
        return;
    }

    uint8_t raw[14];
    if (!read_regs(REG_TEMP_DATA1, raw, sizeof(raw))) {
        return;
    }
    // Layout: temp[0:2], ax[2:4], ay[4:6], az[6:8], gx[8:10], gy[10:12], gz[12:14]
    auto be16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };
    const int16_t ax = be16(raw + 2);
    const int16_t ay = be16(raw + 4);
    const int16_t az = be16(raw + 6);
    const int16_t gx = be16(raw + 8);
    const int16_t gy = be16(raw + 10);
    const int16_t gz = be16(raw + 12);

    float accel[3] = {ax / kAccLsbG, ay / kAccLsbG, az / kAccLsbG};
    float gyro[3] = {gx / kGyrLsbDps * kRadPerDps, gy / kGyrLsbDps * kRadPerDps,
                     gz / kGyrLsbDps * kRadPerDps};

    const int64_t now = esp_timer_get_time();
    float dt = 0.005f;
    if (s_last_us != 0) {
        dt = (now - s_last_us) / 1e6f;
        dt = std::clamp(dt, 0.001f, 0.1f);
    }
    s_last_us = now;

    float an[3] = {accel[0], accel[1], accel[2]};
    const float anorm = std::sqrt(an[0] * an[0] + an[1] * an[1] + an[2] * an[2]);
    if (anorm > 1e-4f) {
        an[0] /= anorm;
        an[1] /= anorm;
        an[2] /= anorm;
        const float v0 = 2.0f * (q1 * q3 - q0 * q2);
        const float v1 = 2.0f * (q0 * q1 + q2 * q3);
        const float v2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        const float ex = an[1] * v2 - an[2] * v1;
        const float ey = an[2] * v0 - an[0] * v2;
        const float ez = an[0] * v1 - an[1] * v0;
        q1 += kTwoKp * ex * dt * 0.5f;
        q2 += kTwoKp * ey * dt * 0.5f;
        q3 += kTwoKp * ez * dt * 0.5f;
    }
    const float hx = gyro[0], hy = gyro[1], hz = gyro[2];
    const float dq0 = 0.5f * (-q1 * hx - q2 * hy - q3 * hz);
    const float dq1 = 0.5f * (q0 * hx + q2 * hz - q3 * hy);
    const float dq2 = 0.5f * (q0 * hy - q1 * hz + q3 * hx);
    const float dq3 = 0.5f * (q0 * hz + q1 * hy - q2 * hx);
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;
    const float qn = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= qn;
    q1 /= qn;
    q2 /= qn;
    q3 /= qn;

    // Projected gravity = Rᵀ(q)·[0,0,-1] — same upright convention as obs.rs.
    const float grav[3] = {-(2.0f * (q1 * q3 - q0 * q2)), -(2.0f * (q0 * q1 + q2 * q3)),
                           -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)};

    portENTER_CRITICAL(&s_spin);
    memcpy(s_accel, accel, sizeof(s_accel));
    memcpy(s_gyro, gyro, sizeof(s_gyro));
    memcpy(s_gravity, grav, sizeof(s_gravity));
    portEXIT_CRITICAL(&s_spin);
}

const float *GyroRad() { return s_gyro; }
const float *ProjGravity() { return s_gravity; }
const float *AccelG() { return s_accel; }

void Snapshot(float gyro_out[3], float gravity_out[3])
{
    portENTER_CRITICAL(&s_spin);
    if (gyro_out) {
        memcpy(gyro_out, s_gyro, sizeof(s_gyro));
    }
    if (gravity_out) {
        memcpy(gravity_out, s_gravity, sizeof(s_gravity));
    }
    portEXIT_CRITICAL(&s_spin);
}

static void FastTask(void *)
{
    TickType_t next = xTaskGetTickCount();
    while (s_fast_running.load(std::memory_order_relaxed)) {
        const int64_t t0 = esp_timer_get_time();
        Poll();
        const int64_t dt = esp_timer_get_time() - t0;
        s_fast_ticks.fetch_add(1, std::memory_order_relaxed);
        s_fast_sum.fetch_add(dt, std::memory_order_relaxed);
        if (dt > 5000) {
            s_fast_misses.fetch_add(1, std::memory_order_relaxed);
        }
        int64_t mn = s_fast_min.load(std::memory_order_relaxed);
        while (dt < mn && !s_fast_min.compare_exchange_weak(mn, dt)) {
        }
        int64_t mx = s_fast_max.load(std::memory_order_relaxed);
        while (dt > mx && !s_fast_max.compare_exchange_weak(mx, dt)) {
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(5)); /* 200 Hz */
    }
    vTaskDelete(nullptr);
}

bool StartFastTask()
{
    if (!s_up) {
        return false;
    }
    if (s_fast_running.exchange(true)) {
        return true;
    }
    s_fast_ticks = 0;
    s_fast_misses = 0;
    s_fast_min = INT64_MAX;
    s_fast_max = 0;
    s_fast_sum = 0;
    /* Core 1 with policy; 200 Hz is light SPI reads. */
    xTaskCreatePinnedToCore(FastTask, "imu200", 4096, nullptr, 6, nullptr, 1);
    ESP_LOGI(kTag, "IMU fast task @200 Hz (SPI stand-in for imu_to_dxl@200)");
    return true;
}

void StopFastTask() { s_fast_running = false; }

bool FastTaskRunning() { return s_fast_running.load(); }

void FastStats(int64_t *ticks, int64_t *misses, int64_t *min_us, int64_t *max_us, int64_t *sum_us)
{
    if (ticks) {
        *ticks = s_fast_ticks.load();
    }
    if (misses) {
        *misses = s_fast_misses.load();
    }
    if (min_us) {
        *min_us = s_fast_min.load();
    }
    if (max_us) {
        *max_us = s_fast_max.load();
    }
    if (sum_us) {
        *sum_us = s_fast_sum.load();
    }
}

} // namespace imu
