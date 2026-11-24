// SPDX-License-Identifier: MIT
//
// Shared wire format between ESP32-P4 (control) and ESP32-C5 (wireless).
// Training stays on Linux (microduck_rl); neither MCU runs RL training.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MICRODUCK_LINK_MAGIC 0x4D44554Bu /* 'MDUK' */
#define MICRODUCK_LINK_VERSION 1

/// esp_hosted peer-data message ID: C5 (slave) → P4 (host), payload = microduck_link_frame_t.
#define MICRODUCK_MSG_ID_CMD 0x4D444301u

/// Maps to actor observation indices [48:61]:
///   twist(3) + head_pose(4) + body_pose(6) — same order as mjlab_microduck.
typedef struct {
    float twist[3];
    float head_pose[4];
    float body_pose[6];
} microduck_command_block_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t type;   ///< 1 = command block from C5 → P4
    uint32_t seq;
    microduck_command_block_t cmd;
    uint32_t crc32;  ///< optional; 0 if unused
} microduck_link_frame_t;

static const microduck_command_block_t MICRODUCK_COMMAND_IDLE = {{
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
}};

#ifdef __cplusplus
}
#endif