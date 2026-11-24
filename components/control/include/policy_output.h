// SPDX-License-Identifier: MIT
//
// Thin contract between the 50 Hz policy task and the future servo / BAM layer
// on ESP32-P4. The AI stack only produces normalized joint deltas; driving
// Dynamixel XL330s belongs in a separate component.
#pragma once

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

/// Layout matches mjlab_microduck / microduck runtime: 14 actuated servos.
#define MICRODUCK_ACT_DIM 14
/// Full actor observation (proprio + command block).
#define MICRODUCK_OBS_DIM 61

/// Latest policy output (joint position targets, rad or cfg-specific units).
/// Written by the policy loop; read by the servo task when integrated.
void microduck_policy_set_actions(const float actions[MICRODUCK_ACT_DIM]);
void microduck_policy_get_actions(float out[MICRODUCK_ACT_DIM]);

#ifdef __cplusplus
}
#endif