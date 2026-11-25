// SPDX-License-Identifier: MIT
// Host test: autod-style STOP/REACT obstacle gate (no ESP-IDF).
#include <cmath>
#include <cstdio>
#include <cstring>

#include "tof_obstacle.h"

static int g_fail;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAIL: %s\n", msg);                                     \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

static void clear_frame(uint16_t *mm, uint8_t *st)
{
    for (int i = 0; i < TOF_OBS_ZONE_COUNT; ++i) {
        mm[i] = 0;
        st[i] = 255;
    }
}

static void paint_near(uint16_t *mm, uint8_t *st, uint16_t centre_mm)
{
    clear_frame(mm, st);
    for (int r = 0; r < 4; ++r) {
        for (int c = 2; c < 6; ++c) {
            const int i = r * TOF_OBS_ZONE_N + c;
            mm[i] = centre_mm;
            st[i] = 0;
        }
    }
}

int main()
{
    uint16_t mm[TOF_OBS_ZONE_COUNT];
    uint8_t st[TOF_OBS_ZONE_COUNT];
    tof_obs_state_t state;
    tof_obs_gate_t gate;

    tof_obs_state_init(&state);
    clear_frame(mm, st);
    tof_obs_evaluate(&state, mm, st, 1.0f, &gate);
    CHECK(!gate.stop, "clear corridor should not stop");
    CHECK(gate.speed_scale >= 0.99f, "clear scale≈1");
    CHECK(std::isinf(gate.ahead_m), "clear ahead=inf");

    paint_near(mm, st, 100); /* 0.10 m < STOP */
    tof_obs_evaluate(&state, mm, st, 1.1f, &gate);
    CHECK(gate.stop, "near wall must stop");
    CHECK(gate.speed_scale == 0.f, "stop scale=0");
    CHECK(gate.ahead_m < TOF_OBS_STOP_M + 0.01f, "ahead near stop");

    float twist[3] = {0.3f, 0.f, 0.f};
    tof_obs_apply_twist(&gate, twist);
    CHECK(std::fabs(twist[0]) < 1e-6f, "stop zeros vx");

    paint_near(mm, st, 300); /* 0.30 m → slow */
    tof_obs_evaluate(&state, mm, st, 1.2f, &gate);
    CHECK(!gate.stop && gate.slow, "0.30 m is slow not stop");
    CHECK(gate.speed_scale > 0.f && gate.speed_scale < 1.f, "slow scale in (0,1)");

    twist[0] = 0.4f;
    twist[1] = 0.f;
    twist[2] = 0.f;
    const float before = twist[0];
    tof_obs_apply_twist(&gate, twist);
    CHECK(twist[0] < before && twist[0] > 0.f, "slow scales vx down");

    std::printf("tof_obstacle_test: %s (%d fails)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
