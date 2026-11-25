// SPDX-License-Identifier: MIT

#include "posture.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* robotd posture.rs */
static const float kLyingTiltDeg = 45.f;

const char *posture_name(posture_t p)
{
    switch (p) {
    case POSTURE_STANDING:
        return "standing";
    case POSTURE_SEATED:
        return "seated";
    case POSTURE_LYING:
        return "lying";
    default:
        return "unsure";
    }
}

posture_reading_t posture_classify(const float gravity[3], const float *joints_phys_or_null)
{
    posture_reading_t r = {POSTURE_UNSURE, 0.f, -1.f};
    if (!gravity) {
        return r;
    }
    r.gravity_z = gravity[2];
    /* upright gravity ≈ [0,0,-1]; trunk up in world has z = -gravity·[0,0,1] wait —
     * projected gravity is gravity in trunk frame. Angle of trunk from upright is
     * acos(clamp(-gravity_z)). */
    float gz = gravity[2];
    if (gz > 1.f) {
        gz = 1.f;
    }
    if (gz < -1.f) {
        gz = -1.f;
    }
    r.tilt_deg = acosf(-gz) * (180.f / (float)M_PI);

    if (r.tilt_deg > kLyingTiltDeg) {
        r.posture = POSTURE_LYING;
        return r;
    }

    /* Without MuJoCo FK, use a coarse hip-fold proxy when joints are available:
     * mean abs of hip pitch + knee (indices 2,3 left and 12,13 right) large → seated. */
    if (joints_phys_or_null) {
        const float fold = (fabsf(joints_phys_or_null[2]) + fabsf(joints_phys_or_null[3]) +
                            fabsf(joints_phys_or_null[12]) + fabsf(joints_phys_or_null[13])) *
                           0.25f;
        if (fold > 0.8f) {
            r.posture = POSTURE_SEATED;
        } else if (fold < 0.35f && r.tilt_deg < 20.f) {
            r.posture = POSTURE_STANDING;
        } else {
            r.posture = POSTURE_UNSURE;
        }
        return r;
    }

    if (r.tilt_deg < 20.f) {
        r.posture = POSTURE_STANDING;
    } else {
        r.posture = POSTURE_UNSURE;
    }
    return r;
}
