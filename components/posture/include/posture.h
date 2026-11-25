// SPDX-License-Identifier: MIT
//
// Lightweight posture classify from pollen-robotics/microduck robotd/src/posture.rs.
// Full kinematics FK is Linux/MuJoCo-only; here we use projected-gravity tilt only
// (lying vs upright). Seated vs standing without FK is reported as Unsure when upright.
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    POSTURE_STANDING = 0,
    POSTURE_SEATED,
    POSTURE_LYING,
    POSTURE_UNSURE,
} posture_t;

typedef struct {
    posture_t posture;
    float tilt_deg;     ///< angle between trunk up and world up
    float gravity_z;    ///< projected gravity z (upright ≈ -1)
} posture_reading_t;

/// Classify from projected gravity (trunk frame). Optional hip-fold heuristic via
/// mean |hip/knee| if joints_phys is non-NULL (15 joints wire order).
posture_reading_t posture_classify(const float gravity[3], const float *joints_phys_or_null);

const char *posture_name(posture_t p);

#ifdef __cplusplus
}
#endif
