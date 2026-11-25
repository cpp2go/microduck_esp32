// SPDX-License-Identifier: MIT
//
// Pure obstacle gate (host-testable). Logic inspired by
// pollen-robotics/microduck autod/src/obstacle.rs STOP_M / REACT_M.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOF_OBS_ZONE_N 8
#define TOF_OBS_ZONE_COUNT (TOF_OBS_ZONE_N * TOF_OBS_ZONE_N)

#ifndef TOF_OBS_STOP_M
#define TOF_OBS_STOP_M 0.15f
#endif
#ifndef TOF_OBS_REACT_M
#define TOF_OBS_REACT_M 0.45f
#endif
#ifndef TOF_OBS_STALE_S
#define TOF_OBS_STALE_S 1.5f
#endif

typedef struct {
    float ahead_m;
    float away;
    float age_s;
    bool stop;
    bool slow;
    float speed_scale;
} tof_obs_gate_t;

typedef struct {
    float remembered_ahead_m;
    float remembered_away;
    float last_seen_s;
    bool has_hit;
} tof_obs_state_t;

void tof_obs_state_init(tof_obs_state_t *st);

/// Evaluate one 8×8 mm frame at time `now_s`. Corridor = centre columns, near rows.
void tof_obs_evaluate(tof_obs_state_t *st, const uint16_t dist_mm[TOF_OBS_ZONE_COUNT],
                      const uint8_t status[TOF_OBS_ZONE_COUNT], float now_s,
                      tof_obs_gate_t *out);

void tof_obs_apply_twist(const tof_obs_gate_t *gate, float twist[3]);

#ifdef __cplusplus
}
#endif
