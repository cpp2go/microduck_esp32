// Copyright 2026 ESP-Duck Authors
// SPDX-License-Identifier: MIT
//
// BMI270 via the OFFICIAL Bosch BMI270_SensorAPI (vendored in vendor/,
// BSD-3-Clause) with an I2C glue layer, plus a Mahony attitude filter.
//
// Why the full API instead of a hand-rolled driver: the chip's init-firmware
// upload has protocol quirks (half-word INIT_ADDR addressing, burst rules,
// power-save gating) that a minimal implementation gets wrong silently — the
// gyro comes up but the accelerometer never produces data. The official path
// (bmi270_init -> sensor enable) handles all of it.
//
// Produces the two IMU blocks of the Microduck observation contract
// (upstream duck-control/src/imu.rs):
//   gyro              — angular velocity in the trunk frame, rad/s;
//   projected gravity — gravity direction in the trunk frame, UNIT vector,
//                       upright = [0, 0, -1].
// The trunk-frame axis mapping of the CoreS3's PCB is identity for now; a
// physical mount needs the axis/sign constants re-checked.

#include "imu.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esp_log.h"
#include "hal/gpio_types.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "bmi270.h"
#include "bmi2_defs.h"
#include "sdkconfig.h"

namespace imu {
namespace {

constexpr const char *kTag = "imu";

static gpio_num_t SdaGpio() {
    return static_cast<gpio_num_t>(CONFIG_MICRODUCK_IMU_SDA_GPIO);
}
static gpio_num_t SclGpio() {
    return static_cast<gpio_num_t>(CONFIG_MICRODUCK_IMU_SCL_GPIO);
}
constexpr int kBusHz = 400000;
constexpr uint8_t kAddrs[] = {0x68, 0x69}; // BMI270 SDO strap either way

// Post-init defaults: accel ±2 g → 16384 LSB/g; gyro ±2000 dps → 16.384 LSB/dps.
constexpr float kAccLsbG = 4096.0f;
constexpr float kGyrLsbDps = 16.384f;
constexpr float kRadPerDps = static_cast<float>(M_PI) / 180.0f;

i2c_master_dev_handle_t s_dev = nullptr;
uint8_t s_addr = 0;
struct bmi2_dev s_api;      // Bosch API device handle (glue state lives here)
bool s_api_up = false;
float s_gyro[3] = {0};
float s_accel[3] = {0};
float s_gravity[3] = {0, 0, -1};

// Mahony state: q is body→world, scalar first. kp trades gyro-drift rejection
// against accel corruption during motion; no integral term (ki = 0) keeps the
// filter unconditionally stable, and the policy only sees the gravity direction.
constexpr float kTwoKp = 2.0f;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
int64_t s_last_us = 0;

// --- Bosch API glue (I2C + delay) -----------------------------------------

BMI2_INTF_RETURN_TYPE api_read(uint8_t reg, uint8_t *data, uint32_t len, void *) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100) == ESP_OK
               ? BMI2_INTF_RET_SUCCESS
               : BMI2_E_COM_FAIL;
}

BMI2_INTF_RETURN_TYPE api_write(uint8_t reg, const uint8_t *data, uint32_t len, void *) {
    // Boot-only path with a possibly >32 B config burst; the heap is idle here.
    auto *buf = static_cast<uint8_t *>(malloc(len + 1));
    if (!buf) return BMI2_E_COM_FAIL;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    const esp_err_t err = i2c_master_transmit(s_dev, buf, len + 1, 100);
    free(buf);
    return err == ESP_OK ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

void api_delay_us(uint32_t us, void *) {
    esp_rom_delay_us(us);
}

} // namespace

bool Init() {
#if !CONFIG_MICRODUCK_IMU_ENABLE
    ESP_LOGW(kTag, "IMU disabled in sdkconfig — observation gyro/gravity stay synthetic");
    return false;
#endif
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = SdaGpio();
    bus_cfg.scl_io_num = SclGpio();
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus = nullptr;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        ESP_LOGE(kTag, "i2c bus init failed");
        return false;
    }

    // Probe both SDO straps, bind whichever answers.
    for (uint8_t a : kAddrs) {
        if (i2c_master_probe(bus, a, 100) == ESP_OK) {
            s_addr = a;
            break;
        }
    }
    if (s_addr == 0) {
        ESP_LOGE(kTag, "no BMI270 on 0x68/0x69 — IMU blocks stay synthetic");
        return false;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = s_addr;
    dev_cfg.scl_speed_hz = kBusHz;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(kTag, "i2c device add failed");
        return false;
    }

    // Official init: soft reset, chip id, init-firmware upload, then enable
    // accel + gyro with the API's default configuration (±2 g, ±2000 dps,
    // accel 100 Hz / gyro 200 Hz).
    static uint8_t addr_storage;
    addr_storage = s_addr;
    struct bmi2_dev &dev = s_api;
    memset(&dev, 0, sizeof dev);
    dev.intf = BMI2_I2C_INTF;
    dev.intf_ptr = &addr_storage;
    dev.read = api_read;
    dev.write = api_write;
    dev.delay_us = api_delay_us;
    dev.read_write_len = 32; // official I2C default burst length
    dev.config_file_ptr = nullptr;

    if (bmi270_init(&dev) != BMI2_OK) {
        ESP_LOGE(kTag, "bmi270_init failed (chip id / init firmware)");
        return false;
    }
    const uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    if (bmi270_sensor_enable(sens_list, 2, &dev) != BMI2_OK) {
        ESP_LOGE(kTag, "sensor enable failed");
        return false;
    }
    s_api_up = true;
    vTaskDelay(pdMS_TO_TICKS(100)); // first samples at accel 100 Hz ODR

    struct bmi2_sensor_data acc = {.type = BMI2_ACCEL};
    if (bmi2_get_sensor_data(&acc, 1, &s_api) != BMI2_OK) {
        ESP_LOGE(kTag, "accel data read failed");
        return false;
    }
    const float ax = acc.sens_data.acc.x / kAccLsbG;
    const float ay = acc.sens_data.acc.y / kAccLsbG;
    const float az = acc.sens_data.acc.z / kAccLsbG;
    const float mag = std::sqrt(ax * ax + ay * ay + az * az);
    ESP_LOGI(kTag, "BMI270 @ 0x%02X ready, |accel| %.3f g (raw %d %d %d)",
             s_addr, static_cast<double>(mag),
             acc.sens_data.acc.x, acc.sens_data.acc.y, acc.sens_data.acc.z);
    if (mag < 0.5f || mag > 1.5f) {
        ESP_LOGE(kTag, "accel sanity failed — IMU stays synthetic");
        return false;
    }

    Poll(); // seed gyro/gravity/accel getters before the first shell query
    return true;
}

void Poll() {
    if (!s_api_up) return;

    struct bmi2_sensor_data acc = {.type = BMI2_ACCEL};
    struct bmi2_sensor_data gyr = {.type = BMI2_GYRO};
    // Two single-sensor reads per tick — the combined n_sens=2 call returns
    // accel data in both slots on v2.63.1, silently swapping garbage into the
    // gyro channels.
    if (bmi2_get_sensor_data(&acc, 1, &s_api) != BMI2_OK) return;
    if (bmi2_get_sensor_data(&gyr, 1, &s_api) != BMI2_OK) return;

    s_accel[0] = acc.sens_data.acc.x / kAccLsbG;
    s_accel[1] = acc.sens_data.acc.y / kAccLsbG;
    s_accel[2] = acc.sens_data.acc.z / kAccLsbG;
    s_gyro[0] = gyr.sens_data.gyr.x / kGyrLsbDps * kRadPerDps;
    s_gyro[1] = gyr.sens_data.gyr.y / kGyrLsbDps * kRadPerDps;
    s_gyro[2] = gyr.sens_data.gyr.z / kGyrLsbDps * kRadPerDps;

    // Mahony: step the attitude quaternion, then read the gravity direction
    // (trunk frame, unit, upright [0,0,-1]) back out of it.
    const int64_t now = esp_timer_get_time();
    float dt = 0.02f;
    if (s_last_us != 0) {
        dt = (now - s_last_us) / 1e6f;
        dt = std::clamp(dt, 0.001f, 0.1f);
    }
    s_last_us = now;

    float an[3] = {s_accel[0], s_accel[1], s_accel[2]};
    const float anorm = std::sqrt(an[0] * an[0] + an[1] * an[1] + an[2] * an[2]);
    if (anorm > 1e-4f) {
        an[0] /= anorm;
        an[1] /= anorm;
        an[2] /= anorm;
        // "up in body" from the current estimate: Rᵀ(q)·[0,0,1]
        const float v0 = 2.0f * (q1 * q3 - q0 * q2);
        const float v1 = 2.0f * (q0 * q1 + q2 * q3);
        const float v2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        // accel and v both point up; their cross product is the tilt error.
        const float ex = an[1] * v2 - an[2] * v1;
        const float ey = an[2] * v0 - an[0] * v2;
        const float ez = an[0] * v1 - an[1] * v0;
        q1 += kTwoKp * ex * dt * 0.5f;
        q2 += kTwoKp * ey * dt * 0.5f;
        q3 += kTwoKp * ez * dt * 0.5f;
    }
    // q̇ = ½ q ⊗ [0, ω]
    const float hx = s_gyro[0], hy = s_gyro[1], hz = s_gyro[2];
    const float dq0 = 0.5f * (-q1 * hx - q2 * hy - q3 * hz);
    const float dq1 = 0.5f * (q0 * hx + q2 * hz - q3 * hy);
    const float dq2 = 0.5f * (q0 * hy - q1 * hz + q3 * hx);
    const float dq3 = 0.5f * (q0 * hz + q1 * hy - q2 * hx);
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;
    const float qn = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn;

    // Projected gravity = Rᵀ(q)·[0,0,-1] (unit vector, upright [0,0,-1]).
    s_gravity[0] = -(2.0f * (q1 * q3 - q0 * q2));
    s_gravity[1] = -(2.0f * (q0 * q1 + q2 * q3));
    s_gravity[2] = -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

const float *GyroRad() { return s_gyro; }
const float *ProjGravity() { return s_gravity; }
const float *AccelG() { return s_accel; }

} // namespace imu
