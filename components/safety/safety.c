// SPDX-License-Identifier: MIT

#include "safety.h"

#include <math.h>
#include <string.h>

void fall_predictor_init(fall_predictor_t *p, const fall_predictor_config_t *cfg_or_null)
{
    if (!p) {
        return;
    }
    memset(p, 0, sizeof(*p));
    if (cfg_or_null) {
        p->cfg = *cfg_or_null;
    } else {
        p->cfg.tilt_z = -0.90f;
        p->cfg.predicted_z = -0.5f;
        p->cfg.lookahead_s = 0.30f;
        p->cfg.debounce_s = 0.060f;
        p->cfg.arm_s = 0.200f;
    }
}

void fall_predictor_reset(fall_predictor_t *p)
{
    if (!p) {
        return;
    }
    p->falling_for_s = 0.f;
    p->upright_for_s = 0.f;
    p->armed = false;
}

float fall_gravity_z_rate(const float gyro[3], const float gravity[3])
{
    if (!gyro || !gravity) {
        return 0.f;
    }
    return -(gyro[0] * gravity[1] - gyro[1] * gravity[0]);
}

bool fall_predictor_observe(fall_predictor_t *p, const float gyro[3], const float gravity[3],
                            float dt_s)
{
    if (!p || !gyro || !gravity) {
        return false;
    }
    if (dt_s < 0.f) {
        dt_s = 0.f;
    }

    if (gravity[2] <= p->cfg.tilt_z) {
        p->upright_for_s += dt_s;
        if (p->upright_for_s >= p->cfg.arm_s) {
            p->armed = true;
        }
    } else {
        p->upright_for_s = 0.f;
    }

    const float rate = fall_gravity_z_rate(gyro, gravity);
    const float predicted = gravity[2] + rate * p->cfg.lookahead_s;
    const bool going_down =
        gravity[2] > p->cfg.tilt_z && rate > 0.f && predicted > p->cfg.predicted_z;

    if (!going_down || !p->armed) {
        p->falling_for_s = 0.f;
        return false;
    }

    p->falling_for_s += dt_s;
    if (p->falling_for_s >= p->cfg.debounce_s) {
        p->armed = false;
        p->falling_for_s = 0.f;
        return true;
    }
    return false;
}

void safety_init(safety_state_t *s, const safety_config_t *cfg_or_null)
{
    if (!s) {
        return;
    }
    memset(s, 0, sizeof(*s));
    if (cfg_or_null) {
        s->cfg = *cfg_or_null;
    } else {
        s->cfg.fall_gravity_z = -0.5f;
        s->cfg.fall_debounce_s = 0.200f;
        s->cfg.deadman_s = 0.500f;
    }
}

bool safety_observe_gravity(safety_state_t *s, float gravity_z, float dt_s)
{
    if (!s) {
        return false;
    }
    if (dt_s < 0.f) {
        dt_s = 0.f;
    }
    if (gravity_z > s->cfg.fall_gravity_z) {
        s->falling_for_s += dt_s;
        if (s->falling_for_s >= s->cfg.fall_debounce_s) {
            s->fallen = true;
        }
    } else {
        s->falling_for_s = 0.f;
        /* Original publishes fallen as a report that clears when upright again. */
        s->fallen = false;
    }
    return s->fallen;
}

bool safety_fallen(const safety_state_t *s) { return s && s->fallen; }

int safety_clamp_targets(float *targets, int n)
{
    if (!targets || n <= 0) {
        return 0;
    }
    int limited = 0;
    for (int i = 0; i < n; ++i) {
        float v = targets[i];
        if (!isfinite(v)) {
            targets[i] = 0.f;
            ++limited;
            continue;
        }
        if (v < SAFETY_ACTUATOR_MIN) {
            targets[i] = SAFETY_ACTUATOR_MIN;
            ++limited;
        } else if (v > SAFETY_ACTUATOR_MAX) {
            targets[i] = SAFETY_ACTUATOR_MAX;
            ++limited;
        }
    }
    return limited;
}
