// SPDX-License-Identifier: MIT

#include "tof_obstacle.h"

#include <math.h>
#include <string.h>

#ifndef INFINITY
#define INFINITY (__builtin_inff())
#endif

void tof_obs_state_init(tof_obs_state_t *st)
{
    if (!st) {
        return;
    }
    memset(st, 0, sizeof(*st));
    st->remembered_ahead_m = INFINITY;
    st->last_seen_s = -1000.f;
}

static bool zone_valid(uint8_t st, uint16_t mm)
{
    /* 0 = measured ok; 255 = observed clear (theremin/autod convention). */
    if (st == 255) {
        return true;
    }
    if (st != 0) {
        return false;
    }
    return mm > 0 && mm < 4000;
}

void tof_obs_evaluate(tof_obs_state_t *st, const uint16_t dist_mm[TOF_OBS_ZONE_COUNT],
                      const uint8_t status[TOF_OBS_ZONE_COUNT], float now_s,
                      tof_obs_gate_t *out)
{
    if (!st || !dist_mm || !status || !out) {
        return;
    }

    float nearest = INFINITY;
    float left_sum = 0.f, right_sum = 0.f;
    int left_n = 0, right_n = 0;
    bool observed = false;

    /* Walking corridor: rows 0..3 (near→mid), columns 2..5; split L/R at col 4. */
    for (int r = 0; r < 4; ++r) {
        for (int c = 2; c < 6; ++c) {
            const int i = r * TOF_OBS_ZONE_N + c;
            const uint8_t stv = status[i];
            const uint16_t mm = dist_mm[i];
            if (stv == 255) {
                observed = true;
                continue;
            }
            if (!zone_valid(stv, mm)) {
                continue;
            }
            observed = true;
            const float m = mm * 0.001f;
            if (m < nearest) {
                nearest = m;
            }
            if (c < 4) {
                left_sum += m;
                ++left_n;
            } else {
                right_sum += m;
                ++right_n;
            }
        }
    }

    float away = 0.f;
    if (left_n > 0 && right_n > 0) {
        const float lm = left_sum / (float)left_n;
        const float rm = right_sum / (float)right_n;
        /* Turn toward the clearer (farther) side. */
        away = (rm > lm) ? 1.f : -1.f;
    } else if (left_n > 0) {
        away = 1.f;
    } else if (right_n > 0) {
        away = -1.f;
    }

    if (observed) {
        st->last_seen_s = now_s;
        if (nearest < INFINITY) {
            st->remembered_ahead_m = nearest;
            st->remembered_away = away;
            st->has_hit = true;
        } else {
            /* Fully clear corridor this frame. */
            st->remembered_ahead_m = INFINITY;
            st->has_hit = false;
            st->remembered_away = 0.f;
        }
    }

    const float age = now_s - st->last_seen_s;
    float ahead = st->has_hit ? st->remembered_ahead_m : INFINITY;
    /* Stale memory decays: after STALE_S treat as unobserved (no advance). */
    if (st->has_hit && age > TOF_OBS_STALE_S) {
        ahead = INFINITY;
    }

    out->ahead_m = ahead;
    out->away = st->has_hit ? st->remembered_away : 0.f;
    out->age_s = age;
    out->stop = (ahead < TOF_OBS_STOP_M);
    out->slow = (ahead < TOF_OBS_REACT_M) && !out->stop;

    if (!observed && age > TOF_OBS_STALE_S) {
        /* Unobserved path: may turn but not advance. */
        out->speed_scale = 0.f;
        out->stop = true;
    } else if (out->stop) {
        out->speed_scale = 0.f;
    } else if (out->slow) {
        const float t = (ahead - TOF_OBS_STOP_M) / (TOF_OBS_REACT_M - TOF_OBS_STOP_M);
        out->speed_scale = t < 0.f ? 0.f : (t > 1.f ? 1.f : t);
    } else {
        out->speed_scale = 1.f;
    }
}

void tof_obs_apply_twist(const tof_obs_gate_t *gate, float twist[3])
{
    if (!gate || !twist) {
        return;
    }
    if (gate->stop) {
        twist[0] = 0.f;
        twist[1] = 0.f;
        /* Bias yaw away from the obstacle when stopped. */
        if (gate->away != 0.f && fabsf(twist[2]) < 0.05f) {
            twist[2] = gate->away * 0.4f;
        }
        return;
    }
    twist[0] *= gate->speed_scale;
    twist[1] *= gate->speed_scale;
    if (gate->slow && gate->away != 0.f) {
        twist[2] += gate->away * 0.25f * (1.f - gate->speed_scale);
    }
}
