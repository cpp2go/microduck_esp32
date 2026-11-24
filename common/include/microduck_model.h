// SPDX-License-Identifier: MIT
// Alpha robot tables — pollen-robotics/microduck duck-control/src/model.rs + duck_ipc_proto.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MICRODUCK_NUM_JOINTS 15
#define MICRODUCK_POLICY_ACTION_LEN 14
#define MICRODUCK_MOUTH_INDEX 9
#define MICRODUCK_OBS_LEN 61

/// Policy observation proprio slots (all joints except mouth).
#define MICRODUCK_OBS_JOINTS (MICRODUCK_NUM_JOINTS - 1)

/** Feetech STS/HD1910 bus IDs indexed as joint 0..14 (left leg · neck/head/mouth · right leg).
 *  Numbering matches the alpha layout used with pollen-robotics/microduck (same slot order). */
static const uint8_t MICRODUCK_JOINT_IDS[MICRODUCK_NUM_JOINTS] = {
    20, 21, 22, 23, 24, 10, 11, 12, 13, 14, 30, 31, 32, 33, 34,
};

/// Home pose — must match training `HOME_FRAME` / `DEFAULT_POSITION`.
static const float MICRODUCK_DEFAULT_POSITION[MICRODUCK_NUM_JOINTS] = {
    0.0f,
    -0.0873f,
    -0.4579f,
    -0.0049f,
    0.4530f,
    0.3491f,
    0.3491f,
    0.0f,
    0.0f,
    0.0f, /* mouth */
    0.0f,
    0.0873f,
    0.4579f,
    0.0049f,
    -0.4530f,
};

/** Original microduck imu_to_dxl ID (not used on this board — local ICM-42688 SPI). */
#define MICRODUCK_IMU_DXL_ID 200u

/// Physical joint index for policy slot `slot` (mouth skipped in the 14-wide vector).
static inline int microduck_joint_of_policy_slot(int slot)
{
    return slot < MICRODUCK_MOUTH_INDEX ? slot : slot + 1;
}

static inline void microduck_policy_joints_from_phys(const float phys[MICRODUCK_NUM_JOINTS],
                                                     float policy[MICRODUCK_POLICY_ACTION_LEN])
{
    for (int s = 0; s < MICRODUCK_POLICY_ACTION_LEN; ++s) {
        policy[s] = phys[microduck_joint_of_policy_slot(s)];
    }
}

/// Map 14 policy outputs onto 15 joints; mouth stays 0 until a separate mouth command exists.
static inline void microduck_scatter_policy_action(const float action[MICRODUCK_POLICY_ACTION_LEN],
                                                   float joints[MICRODUCK_NUM_JOINTS])
{
    for (int j = 0; j < MICRODUCK_NUM_JOINTS; ++j) {
        joints[j] = 0.0f;
    }
    for (int s = 0; s < MICRODUCK_POLICY_ACTION_LEN; ++s) {
        joints[microduck_joint_of_policy_slot(s)] = action[s];
    }
}

#ifdef __cplusplus
}
#endif