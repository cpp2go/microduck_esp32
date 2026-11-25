// SPDX-License-Identifier: MIT
//
// Head ToF stand-in for pollen-robotics/microduck `tof`/`tofd` (VL53L8CX 8×8).
// Publishes depth frames; obstacle gate mirrors autod STOP_M / REACT_M / age.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOF_ZONE_N 8
#define TOF_ZONE_COUNT (TOF_ZONE_N * TOF_ZONE_N)

/// One depth matrix (~15 Hz on original tofd). Distances in millimetres; 0 = invalid.
typedef struct {
    uint16_t dist_mm[TOF_ZONE_COUNT];
    uint8_t status[TOF_ZONE_COUNT]; ///< 0=ok, 255=clear/max, else unknown
    uint64_t seq;
    int64_t ts_us;
} tof_frame_t;

/// Obstacle summary for the walking corridor (beak-forward).
typedef struct {
    float ahead_m;   ///< nearest hit ahead; +inf when clear
    float away;      ///< +1 turn left, −1 turn right
    float age_s;     ///< seconds since path last observed
    bool stop;       ///< ahead < STOP_M
    bool slow;       ///< ahead < REACT_M
    float speed_scale; ///< 0..1 multiplier for twist.x / |twist|
} tof_obstacle_t;

bool tof_init(void);
bool tof_available(void);
const char *tof_backend_name(void);

/// Grab latest frame (synthetic / inject / I2C). Returns false if offline.
bool tof_grab(tof_frame_t *out);

/// Inject a named scene for bring-up without hardware: "clear", "near", "wall", "left", "right".
bool tof_inject(const char *scene);

/// Update obstacle model from the latest frame (call each control tick or after grab).
void tof_obstacle_update(tof_obstacle_t *out);

/// Apply gate to a twist [vx, vy, yaw]: zeros/scales when stop/slow.
void tof_obstacle_apply_twist(const tof_obstacle_t *obs, float twist[3]);

/// Defaults matching autod/src/obstacle.rs.
float tof_stop_m(void);
float tof_react_m(void);

#ifdef __cplusplus
}
#endif
