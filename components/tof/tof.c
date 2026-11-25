// SPDX-License-Identifier: MIT

#include "tof.h"
#include "tof_obstacle.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#if CONFIG_MICRODUCK_TOF_ULD
#include "vl53_uld.h"
#endif

static const char *kTag = "tof";

#if CONFIG_MICRODUCK_TOF_ENABLE
static tof_frame_t s_frame;
static tof_obs_state_t s_obs_st;
static tof_obstacle_t s_last_obs;
static SemaphoreHandle_t s_mux;
static bool s_up;
static uint64_t s_seq;
static char s_scene[16] = "clear";
static bool s_uld_live;
static bool s_force_inject; /* inject scenes override ULD until cleared */

static void lock_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
}

static void fill_uniform(tof_frame_t *fr, uint16_t mm, uint8_t st)
{
    for (int i = 0; i < TOF_ZONE_COUNT; ++i) {
        fr->dist_mm[i] = mm;
        fr->status[i] = st;
    }
}

static void paint_corridor(tof_frame_t *fr, uint16_t centre_mm, int bias_col)
{
    fill_uniform(fr, 2000, 255); /* far / clear */
    for (int r = 0; r < 4; ++r) {
        for (int c = 2; c < 6; ++c) {
            const int i = r * TOF_ZONE_N + c;
            uint16_t mm = centre_mm;
            if (bias_col < 0 && c < 4) {
                mm = (uint16_t)(centre_mm * 2 / 3);
            }
            if (bias_col > 0 && c >= 4) {
                mm = (uint16_t)(centre_mm * 2 / 3);
            }
            fr->dist_mm[i] = mm;
            fr->status[i] = 0;
        }
    }
}

static void synthesize(tof_frame_t *fr)
{
    ++s_seq;
    fr->seq = s_seq;
    fr->ts_us = esp_timer_get_time();

    if (strcmp(s_scene, "near") == 0) {
        paint_corridor(fr, 100, 0);
    } else if (strcmp(s_scene, "wall") == 0) {
        paint_corridor(fr, 300, 0);
    } else if (strcmp(s_scene, "left") == 0) {
        paint_corridor(fr, 250, -1);
    } else if (strcmp(s_scene, "right") == 0) {
        paint_corridor(fr, 250, 1);
    } else {
        fill_uniform(fr, 0, 255);
    }
}

#if CONFIG_MICRODUCK_TOF_ULD
static bool try_uld_frame(tof_frame_t *fr)
{
    if (!s_uld_live) {
        return false;
    }
    const int ready = vl8_data_ready();
    if (ready < 0) {
        return false;
    }
    if (ready == 0) {
        /* keep last frame timestamps fresh for age tracking */
        return false;
    }
    int16_t dist[TOF_ZONE_COUNT];
    uint8_t st[TOF_ZONE_COUNT];
    if (vl8_get_frame(dist, st) != 0) {
        return false;
    }
    ++s_seq;
    fr->seq = s_seq;
    fr->ts_us = esp_timer_get_time();
    for (int i = 0; i < TOF_ZONE_COUNT; ++i) {
        fr->dist_mm[i] = dist[i] < 0 ? 0 : (uint16_t)dist[i];
        fr->status[i] = st[i];
    }
    return true;
}

static bool bringup_uld(void)
{
    if (vl8_open(CONFIG_MICRODUCK_TOF_I2C_PORT, CONFIG_MICRODUCK_TOF_I2C_SDA_GPIO,
                 CONFIG_MICRODUCK_TOF_I2C_SCL_GPIO, CONFIG_MICRODUCK_TOF_I2C_ADDR) != 0) {
        ESP_LOGW(kTag, "VL53L8CX I2C open failed");
        return false;
    }
    if (!vl8_is_alive()) {
        ESP_LOGW(kTag, "VL53L8CX not alive");
        vl8_close();
        return false;
    }
    ESP_LOGI(kTag, "VL53L8CX alive — uploading ULD firmware (slow on first boot)");
    if (vl8_init() != 0) {
        ESP_LOGE(kTag, "vl53l8cx_init failed");
        vl8_close();
        return false;
    }
    if (vl8_start(CONFIG_MICRODUCK_TOF_RANGE_HZ) != 0) {
        ESP_LOGE(kTag, "vl53l8cx start ranging failed");
        vl8_close();
        return false;
    }
    ESP_LOGI(kTag, "VL53L8CX ranging @ %d Hz", CONFIG_MICRODUCK_TOF_RANGE_HZ);
    return true;
}
#elif CONFIG_MICRODUCK_TOF_I2C_PROBE
#include "driver/i2c_master.h"

static bool probe_vl53(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CONFIG_MICRODUCK_TOF_I2C_PORT,
        .sda_io_num = CONFIG_MICRODUCK_TOF_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_MICRODUCK_TOF_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        return false;
    }
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_MICRODUCK_TOF_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dh = NULL;
    bool ok = false;
    if (i2c_master_bus_add_device(bus, &dev, &dh) == ESP_OK) {
        /* ST VL53L8CX device/revision IDs at 0x0000 / 0x0001 before ULD load. */
        uint8_t reg[2] = {0x00, 0x00};
        uint8_t id[2] = {0};
        if (i2c_master_transmit_receive(dh, reg, 2, id, 2, 50) == ESP_OK) {
            ESP_LOGI(kTag, "I2C device @0x%02x id=%02x%02x (enable MICRODUCK_TOF_ULD for ranging)",
                     CONFIG_MICRODUCK_TOF_I2C_ADDR, id[0], id[1]);
            ok = true;
        }
        i2c_master_bus_rm_device(dh);
    }
    i2c_del_master_bus(bus);
    return ok;
}
#endif
#endif /* CONFIG_MICRODUCK_TOF_ENABLE */

bool tof_init(void)
{
#if !CONFIG_MICRODUCK_TOF_ENABLE
    ESP_LOGW(kTag, "tof disabled");
    return false;
#else
    lock_init();
    tof_obs_state_init(&s_obs_st);
    memset(&s_last_obs, 0, sizeof(s_last_obs));
    s_last_obs.ahead_m = INFINITY;
    s_last_obs.speed_scale = 1.f;
    strncpy(s_scene, "clear", sizeof(s_scene) - 1);
    s_uld_live = false;
    s_force_inject = false;
    s_up = true;
#if CONFIG_MICRODUCK_TOF_ULD
    s_uld_live = bringup_uld();
    if (s_uld_live) {
        ESP_LOGI(kTag, "tof ready backend=%s", tof_backend_name());
    } else {
        ESP_LOGW(kTag, "tof ready backend=synthetic (ULD bring-up failed)");
    }
#elif CONFIG_MICRODUCK_TOF_I2C_PROBE
    if (probe_vl53()) {
        ESP_LOGI(kTag, "tof ready backend=%s (chip ACKed; ULD off)", tof_backend_name());
    } else {
        ESP_LOGW(kTag, "tof ready backend=%s (no chip — synthetic/inject)", tof_backend_name());
    }
#else
    ESP_LOGI(kTag, "tof ready backend=%s", tof_backend_name());
#endif
    return true;
#endif
}

bool tof_available(void)
{
#if CONFIG_MICRODUCK_TOF_ENABLE
    return s_up;
#else
    return false;
#endif
}

const char *tof_backend_name(void)
{
#if !CONFIG_MICRODUCK_TOF_ENABLE
    return "disabled";
#elif CONFIG_MICRODUCK_TOF_ULD
    return s_uld_live ? "vl53l8cx-uld" : "synthetic+uld-failed";
#elif CONFIG_MICRODUCK_TOF_I2C_PROBE
    return "synthetic+i2c-probe";
#else
    return "synthetic";
#endif
}

bool tof_grab(tof_frame_t *out)
{
#if !CONFIG_MICRODUCK_TOF_ENABLE
    (void)out;
    return false;
#else
    if (!s_up || !out) {
        return false;
    }
    lock_init();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
#if CONFIG_MICRODUCK_TOF_ULD
        if (s_uld_live && !s_force_inject) {
            if (try_uld_frame(&s_frame)) {
                *out = s_frame;
                xSemaphoreGive(s_mux);
                return true;
            }
            /* No new frame yet — return last if we have one, else synthesize clear. */
            if (s_frame.seq > 0) {
                *out = s_frame;
                xSemaphoreGive(s_mux);
                return true;
            }
        }
#endif
        synthesize(&s_frame);
        *out = s_frame;
        xSemaphoreGive(s_mux);
    }
    return true;
#endif
}

bool tof_inject(const char *scene)
{
#if !CONFIG_MICRODUCK_TOF_ENABLE
    (void)scene;
    return false;
#else
    if (!s_up || !scene) {
        return false;
    }
    if (strcmp(scene, "clear") != 0 && strcmp(scene, "near") != 0 &&
        strcmp(scene, "wall") != 0 && strcmp(scene, "left") != 0 &&
        strcmp(scene, "right") != 0 && strcmp(scene, "live") != 0) {
        return false;
    }
    lock_init();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        if (strcmp(scene, "live") == 0) {
            s_force_inject = false;
            strncpy(s_scene, "clear", sizeof(s_scene) - 1);
        } else {
            s_force_inject = true;
            strncpy(s_scene, scene, sizeof(s_scene) - 1);
            s_scene[sizeof(s_scene) - 1] = '\0';
        }
        xSemaphoreGive(s_mux);
    }
    ESP_LOGI(kTag, "inject scene=%s", scene);
    return true;
#endif
}

void tof_obstacle_update(tof_obstacle_t *out)
{
#if !CONFIG_MICRODUCK_TOF_ENABLE
    if (out) {
        memset(out, 0, sizeof(*out));
        out->ahead_m = INFINITY;
        out->speed_scale = 1.f;
    }
#else
    if (!s_up) {
        if (out) {
            memset(out, 0, sizeof(*out));
            out->ahead_m = INFINITY;
            out->speed_scale = 1.f;
        }
        return;
    }
    tof_frame_t fr;
    if (!tof_grab(&fr)) {
        return;
    }
    tof_obs_gate_t gate;
    const float now_s = (float)(fr.ts_us * 1e-6);
    tof_obs_evaluate(&s_obs_st, fr.dist_mm, fr.status, now_s, &gate);
    s_last_obs.ahead_m = gate.ahead_m;
    s_last_obs.away = gate.away;
    s_last_obs.age_s = gate.age_s;
    s_last_obs.stop = gate.stop;
    s_last_obs.slow = gate.slow;
    s_last_obs.speed_scale = gate.speed_scale;
    if (out) {
        *out = s_last_obs;
    }
#endif
}

void tof_obstacle_apply_twist(const tof_obstacle_t *obs, float twist[3])
{
    if (!obs || !twist) {
        return;
    }
    tof_obs_gate_t g = {
        .ahead_m = obs->ahead_m,
        .away = obs->away,
        .age_s = obs->age_s,
        .stop = obs->stop,
        .slow = obs->slow,
        .speed_scale = obs->speed_scale,
    };
    tof_obs_apply_twist(&g, twist);
}

float tof_stop_m(void) { return TOF_OBS_STOP_M; }
float tof_react_m(void) { return TOF_OBS_REACT_M; }
