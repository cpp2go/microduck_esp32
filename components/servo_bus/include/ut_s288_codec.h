// SPDX-License-Identifier: MIT
// Unitree J288/S288 digital servo packet codec (host-testable, no ESP-IDF).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UT_S288_CMD_LEN 20
#define UT_S288_FBK_LEN 26
#define UT_S288_RATIO (70070.0f / 243.0f) /* ≈ 288.35 */

#define UT_S288_MODE_LOCK 0
#define UT_S288_MODE_FOC 1

typedef struct {
    uint8_t id;
    uint8_t mode;
    bool timeout;
    float output_pos; /* rad, output shaft */
    float output_spd; /* rad/s */
    float output_tor; /* N·m */
} ut_s288_feedback_t;

uint32_t ut_s288_crc32(const uint8_t *data, size_t len);

/// Build 20-byte control packet. Positions/torques/speeds are output-shaft SI.
void ut_s288_build_control(uint8_t id, uint8_t mode, bool timeout, float output_tor, float output_spd,
                           float output_pos, float kp, float kd, uint8_t out20[UT_S288_CMD_LEN]);

bool ut_s288_parse_feedback(const uint8_t in26[UT_S288_FBK_LEN], ut_s288_feedback_t *out);

#ifdef __cplusplus
}
#endif
