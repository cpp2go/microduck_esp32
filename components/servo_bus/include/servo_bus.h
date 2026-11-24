// SPDX-License-Identifier: MIT
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "microduck_pose.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pos_rad[MICRODUCK_NUM_JOINTS];
    float vel_rad_s[MICRODUCK_NUM_JOINTS];
    float pos_rel[MICRODUCK_NUM_JOINTS];
    float last_policy_action[MICRODUCK_POLICY_ACTION_LEN];
    uint32_t read_ok;
    uint32_t read_fail;
    bool torque_enabled;
} servo_state_t;

bool servo_bus_init(void);
void servo_bus_torque_enable(bool on);

/// Sync-read all joints (blocking, call from control task only).
bool servo_bus_poll(servo_state_t *out);

/// HOME + action_scale × scatter(policy action), with head/leg low-pass (robotd defaults).
bool servo_bus_write_goals_rel(const float policy_action[MICRODUCK_POLICY_ACTION_LEN]);

void servo_bus_get_state(servo_state_t *out);
const servo_state_t *servo_bus_state(void);

/// Fill observation blocks obs[6:48] (policy joint order, mouth excluded).
void servo_bus_write_obs_proprio(float *obs61);

#ifdef __cplusplus
}
#endif