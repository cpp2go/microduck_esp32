// SPDX-License-Identifier: MIT
//
// Latest on-device policy action (14-D, mouth excluded) published by the 50 Hz
// ESP32-P4 loop. Servo goals are applied in components/servo_bus from these
// raw actions (before action_scale), matching robotd last_action feedback.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// Layout matches pollen-robotics/microduck duck-control: 14 policy slots.
#define MICRODUCK_ACT_DIM 14
/// Full actor observation (proprio + command block).
#define MICRODUCK_OBS_DIM 61

/// Latest raw policy output (pre-scale). Written by the P4 policy loop.
void microduck_policy_set_actions(const float actions[MICRODUCK_ACT_DIM]);
void microduck_policy_get_actions(float out[MICRODUCK_ACT_DIM]);

#ifdef __cplusplus
}
#endif