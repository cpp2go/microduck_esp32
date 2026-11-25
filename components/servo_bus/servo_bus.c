// SPDX-License-Identifier: MIT

#include "servo_bus.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
#include "ut_s288.h"
#else
#include "ft_sts.h"
#endif

static const char *kTag = "servo_bus";

static servo_state_t s_state;
static SemaphoreHandle_t s_mux;
static int s_fail_streak = 0;
static float s_action_scale;
static float s_prev_goal[MICRODUCK_NUM_JOINTS];
static float s_head_alpha;
static float s_leg_alpha;

static const uint8_t *joint_ids(void)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return MICRODUCK_S288_JOINT_IDS;
#else
    return MICRODUCK_JOINT_IDS;
#endif
}

static const char *backend_name(void)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return "Unitree S288";
#else
    return "Feetech 1910";
#endif
}

static void lock_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
}

static float clip_joint(float rad)
{
    const float lim = MICRODUCK_JOINT_LIMIT_RAD;
    if (rad > lim) {
        return lim;
    }
    if (rad < -lim) {
        return -lim;
    }
    return rad;
}

static bool is_head_joint(int joint)
{
    return joint >= 5 && joint <= 8;
}

static void apply_lowpass(float targets[MICRODUCK_NUM_JOINTS])
{
    if (s_head_alpha > 0.0f && s_head_alpha < 1.0f) {
        for (int j = 5; j <= 8; ++j) {
            targets[j] = s_head_alpha * targets[j] + (1.0f - s_head_alpha) * s_prev_goal[j];
        }
    }
    if (s_leg_alpha > 0.0f && s_leg_alpha < 1.0f) {
        for (int j = 0; j < MICRODUCK_NUM_JOINTS; ++j) {
            if (is_head_joint(j) || j == MICRODUCK_MOUTH_INDEX) {
                continue;
            }
            targets[j] = s_leg_alpha * targets[j] + (1.0f - s_leg_alpha) * s_prev_goal[j];
        }
    }
}

static bool bus_init_hw(void)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return ut_s288_bus_init();
#else
    return ft_sts_bus_init();
#endif
}

static int bus_ping(uint8_t id)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return ut_s288_ping(id);
#else
    return ft_sts_ping(id);
#endif
}

static bool bus_configure(uint8_t id)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    const float kp = CONFIG_MICRODUCK_SERVO_S288_KP_MILLI / 1000.0f;
    const float kd = CONFIG_MICRODUCK_SERVO_S288_KD_MILLI / 1000.0f;
    return ut_s288_configure_servo(id, kp, kd);
#else
    return ft_sts_configure_servo(id, (uint16_t)CONFIG_MICRODUCK_SERVO_POSITION_KP);
#endif
}

static bool bus_sync_read(float *pos, float *vel)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return ut_s288_sync_read_pv(joint_ids(), MICRODUCK_NUM_JOINTS, pos, vel);
#else
    return ft_sts_sync_read_pv(joint_ids(), MICRODUCK_NUM_JOINTS, pos, vel);
#endif
}

static bool bus_sync_write(const float *goals)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return ut_s288_sync_write_goal_rad(joint_ids(), goals, MICRODUCK_NUM_JOINTS);
#else
    return ft_sts_sync_write_goal_rad(joint_ids(), goals, MICRODUCK_NUM_JOINTS);
#endif
}

static bool bus_torque(uint8_t id, bool on)
{
#if CONFIG_MICRODUCK_SERVO_BACKEND_UNITREE_S288
    return ut_s288_set_torque_enable(id, on);
#else
    return ft_sts_set_torque_enable(id, on);
#endif
}

bool servo_bus_init(void)
{
#if !CONFIG_MICRODUCK_SERVO_ENABLE
    ESP_LOGW(kTag, "servo bus disabled in sdkconfig");
    return false;
#endif
    lock_init();
    s_action_scale = CONFIG_MICRODUCK_SERVO_ACTION_SCALE / 1000.0f;
    s_head_alpha = CONFIG_MICRODUCK_SERVO_HEAD_LOWPASS / 1000.0f;
    s_leg_alpha = CONFIG_MICRODUCK_SERVO_LEG_LOWPASS / 1000.0f;
    if (!bus_init_hw()) {
        return false;
    }
    const uint8_t *ids = joint_ids();
    int ok = 0;
    for (int i = 0; i < MICRODUCK_NUM_JOINTS; ++i) {
        const uint8_t id = ids[i];
        if (!bus_ping(id)) {
            ESP_LOGW(kTag, "joint %d ID %u no ping", i, id);
            continue;
        }
        if (!bus_configure(id)) {
            ESP_LOGW(kTag, "joint %d ID %u config failed", i, id);
            continue;
        }
        ++ok;
    }
    if (ok == 0) {
        ESP_LOGE(kTag, "no servos responded (%s)", backend_name());
        return false;
    }
    memset(s_state.last_policy_action, 0, sizeof(s_state.last_policy_action));
    for (int j = 0; j < MICRODUCK_NUM_JOINTS; ++j) {
        s_prev_goal[j] = MICRODUCK_HOME_POSE[j];
    }
    s_state.torque_enabled = true;
    float pos0[MICRODUCK_NUM_JOINTS];
    float vel0[MICRODUCK_NUM_JOINTS];
    if (bus_sync_read(pos0, vel0)) {
        float offsets[MICRODUCK_NUM_JOINTS];
        for (int j = 0; j < MICRODUCK_NUM_JOINTS; ++j) {
            s_state.pos_rad[j] = pos0[j];
            s_state.vel_rad_s[j] = vel0[j];
            s_state.pos_rel[j] = pos0[j] - MICRODUCK_HOME_POSE[j];
            offsets[j] = s_state.pos_rel[j];
            s_prev_goal[j] = pos0[j];
        }
        microduck_policy_joints_from_phys(offsets, s_state.last_policy_action);
        s_state.read_ok++;
    }
    ESP_LOGI(kTag, "%d/%d %s online, action_scale=%.3f head_lp=%.2f leg_lp=%.2f", ok, MICRODUCK_NUM_JOINTS,
             backend_name(), (double)s_action_scale, (double)s_head_alpha, (double)s_leg_alpha);
    return true;
}

void servo_bus_torque_enable(bool on)
{
    const uint8_t *ids = joint_ids();
    for (int i = 0; i < MICRODUCK_NUM_JOINTS; ++i) {
        bus_torque(ids[i], on);
    }
    s_state.torque_enabled = on;
    ESP_LOGI(kTag, "torque %s (%s)", on ? "ON" : "OFF", backend_name());
}

bool servo_bus_poll(servo_state_t *out)
{
#if !CONFIG_MICRODUCK_SERVO_ENABLE
    (void)out;
    return false;
#endif
    if (!s_state.torque_enabled) {
        return false;
    }
    float pos[MICRODUCK_NUM_JOINTS];
    float vel[MICRODUCK_NUM_JOINTS];
    bool ok = bus_sync_read(pos, vel);
    if (xSemaphoreTake(s_mux, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    if (ok) {
        s_fail_streak = 0;
        s_state.read_ok++;
        for (int i = 0; i < MICRODUCK_NUM_JOINTS; ++i) {
            s_state.pos_rad[i] = pos[i];
            s_state.vel_rad_s[i] = vel[i];
            s_state.pos_rel[i] = pos[i] - MICRODUCK_HOME_POSE[i];
        }
    } else {
        s_fail_streak++;
        s_state.read_fail++;
        if (s_fail_streak >= CONFIG_MICRODUCK_SERVO_FAIL_LIMIT) {
            ESP_LOGE(kTag, "bus fault — torque disabled");
            xSemaphoreGive(s_mux);
            servo_bus_torque_enable(false);
            if (out) {
                servo_bus_get_state(out);
            }
            return false;
        }
    }
    if (out) {
        *out = s_state;
    }
    xSemaphoreGive(s_mux);
    return ok;
}

bool servo_bus_write_goals_rel(const float policy_action[MICRODUCK_POLICY_ACTION_LEN])
{
#if !CONFIG_MICRODUCK_SERVO_ENABLE
    (void)policy_action;
    return false;
#endif
    if (!policy_action || !s_state.torque_enabled) {
        return false;
    }
    float scattered[MICRODUCK_NUM_JOINTS];
    float goals[MICRODUCK_NUM_JOINTS];
    microduck_scatter_policy_action(policy_action, scattered);
    if (xSemaphoreTake(s_mux, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    memcpy(s_state.last_policy_action, policy_action, sizeof(s_state.last_policy_action));
    for (int j = 0; j < MICRODUCK_NUM_JOINTS; ++j) {
        goals[j] = clip_joint(MICRODUCK_HOME_POSE[j] + scattered[j] * s_action_scale);
    }
    apply_lowpass(goals);
    memcpy(s_prev_goal, goals, sizeof(s_prev_goal));
    xSemaphoreGive(s_mux);
    return bus_sync_write(goals);
}

void servo_bus_get_state(servo_state_t *out)
{
    if (!out) {
        return;
    }
    lock_init();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        *out = s_state;
        xSemaphoreGive(s_mux);
    }
}

const servo_state_t *servo_bus_state(void) { return &s_state; }

void servo_bus_write_obs_proprio(float *obs61)
{
    if (!obs61) {
        return;
    }
    servo_state_t st;
    servo_bus_get_state(&st);
    float pos_pol[MICRODUCK_POLICY_ACTION_LEN];
    float vel_pol[MICRODUCK_POLICY_ACTION_LEN];
    microduck_policy_joints_from_phys(st.pos_rel, pos_pol);
    microduck_policy_joints_from_phys(st.vel_rad_s, vel_pol);
    for (int i = 0; i < MICRODUCK_POLICY_ACTION_LEN; ++i) {
        obs61[6 + i] = pos_pol[i];
        obs61[20 + i] = vel_pol[i];
        obs61[34 + i] = st.last_policy_action[i];
    }
}
