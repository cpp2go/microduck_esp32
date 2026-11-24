// Host-side layout checks against pollen-robotics/microduck duck-control/src/obs.rs.
// Does NOT run policy inference — that stays on ESP32-P4 (duck_policy INT8).
// Run: sh host_test/run.sh
#include <cmath>
#include <cstdio>
#include <cstring>

#include "microduck_model.h"

namespace {

bool near(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) <= eps; }

/// Mirror of cmd_bus_apply_to_obs / Observation::build command block (unsmoothed).
void fill_command(float *obs61, const float twist[3], const float head[4],
                  const float body_zrp[3])
{
    int o = 48;
    for (int i = 0; i < 3; ++i) obs61[o++] = twist[i];
    for (int i = 0; i < 4; ++i) obs61[o++] = head[i];
    obs61[o++] = 0.0f;
    obs61[o++] = 0.0f;
    obs61[o++] = body_zrp[0];
    obs61[o++] = body_zrp[1];
    obs61[o++] = body_zrp[2];
    obs61[o++] = 0.0f;
}

void fill_proprio(float *obs61, const float pos_rel_phys[MICRODUCK_NUM_JOINTS],
                  const float vel_phys[MICRODUCK_NUM_JOINTS],
                  const float last_action[MICRODUCK_POLICY_ACTION_LEN])
{
    float pos_pol[MICRODUCK_POLICY_ACTION_LEN];
    float vel_pol[MICRODUCK_POLICY_ACTION_LEN];
    microduck_policy_joints_from_phys(pos_rel_phys, pos_pol);
    microduck_policy_joints_from_phys(vel_phys, vel_pol);
    for (int i = 0; i < MICRODUCK_POLICY_ACTION_LEN; ++i) {
        obs61[6 + i] = pos_pol[i];
        obs61[20 + i] = vel_pol[i];
        obs61[34 + i] = last_action[i];
    }
}

} // namespace

int main()
{
    int fails = 0;

    if (MICRODUCK_OBS_LEN != 61 || MICRODUCK_POLICY_ACTION_LEN != 14 ||
        MICRODUCK_NUM_JOINTS != 15 || MICRODUCK_MOUTH_INDEX != 9) {
        std::printf("FAIL constants\n");
        return 1;
    }
    if (3 + 3 + MICRODUCK_OBS_JOINTS * 3 + 13 != MICRODUCK_OBS_LEN) {
        std::printf("FAIL layout widths\n");
        return 1;
    }

    // Joint IDs match duck-ipc-proto alpha table.
    const uint8_t expect_ids[15] = {20, 21, 22, 23, 24, 10, 11, 12, 13, 14, 30, 31, 32, 33, 34};
    for (int i = 0; i < 15; ++i) {
        if (MICRODUCK_JOINT_IDS[i] != expect_ids[i]) {
            std::printf("FAIL joint id[%d]=%u\n", i, MICRODUCK_JOINT_IDS[i]);
            ++fails;
        }
    }

    // scatter / gather skip mouth.
    float action[14];
    for (int i = 0; i < 14; ++i) action[i] = (float)(i + 1);
    float joints[15];
    microduck_scatter_policy_action(action, joints);
    if (!near(joints[MICRODUCK_MOUTH_INDEX], 0.0f) || !near(joints[0], 1.0f) ||
        !near(joints[8], 9.0f) || !near(joints[10], 10.0f) || !near(joints[14], 14.0f)) {
        std::printf("FAIL scatter\n");
        ++fails;
    }
    float back[14];
    microduck_policy_joints_from_phys(joints, back);
    for (int i = 0; i < 14; ++i) {
        if (!near(back[i], action[i])) {
            std::printf("FAIL gather slot %d\n", i);
            ++fails;
        }
    }

    // Home pose: mirrored legs (model.rs test).
    const int pairs[] = {1, 2, 3, 4};
    for (int k = 0; k < 4; ++k) {
        int pair = pairs[k];
        float l = MICRODUCK_DEFAULT_POSITION[pair];
        float r = MICRODUCK_DEFAULT_POSITION[pair + 10];
        if (!near(l + r, 0.0f, 1e-4f)) {
            std::printf("FAIL home mirror pair %d\n", pair);
            ++fails;
        }
    }

    float obs[61] = {};
    float pos_rel[15] = {};
    pos_rel[0] = 0.25f;
    pos_rel[MICRODUCK_MOUTH_INDEX] = 1.0f; // must not appear in obs
    float vel[15] = {};
    float last[14] = {};
    last[0] = -0.5f;
    last[13] = 0.75f;
    float twist[3] = {0.1f, 0.2f, 0.3f};
    float head[4] = {0.4f, 0.5f, 0.6f, 0.7f};
    float body[3] = {0.8f, 0.9f, 1.0f};

    obs[0] = 1.0f;
    obs[1] = 2.0f;
    obs[2] = 3.0f;
    obs[3] = 4.0f;
    obs[4] = 5.0f;
    obs[5] = 6.0f;
    fill_proprio(obs, pos_rel, vel, last);
    fill_command(obs, twist, head, body);

    if (!near(obs[6], 0.25f)) {
        std::printf("FAIL joint_pos[0] relative\n");
        ++fails;
    }
    for (int i = 6; i < 20; ++i) {
        if (i == 6) continue;
        if (!near(obs[i], 0.0f)) {
            std::printf("FAIL unexpected proprio at %d = %f (mouth leak?)\n", i, obs[i]);
            ++fails;
            break;
        }
    }
    if (!near(obs[34], -0.5f) || !near(obs[47], 0.75f)) {
        std::printf("FAIL last_action slots\n");
        ++fails;
    }
    if (!near(obs[48], 0.1f) || !near(obs[50], 0.3f) || !near(obs[51], 0.4f) ||
        !near(obs[54], 0.7f)) {
        std::printf("FAIL twist/head\n");
        ++fails;
    }
    if (!near(obs[55], 0.0f) || !near(obs[56], 0.0f) || !near(obs[60], 0.0f)) {
        std::printf("FAIL unbound body axes\n");
        ++fails;
    }
    if (!near(obs[57], 0.8f) || !near(obs[58], 0.9f) || !near(obs[59], 1.0f)) {
        std::printf("FAIL body z,roll,pitch order\n");
        ++fails;
    }

    // Position encoding: center raw 2048 → 0 rad (bus.rs).
    {
        const float two_pi = 2.0f * 3.14159265358979323846f;
        const float pi = 3.14159265358979323846f;
        auto to_rad = [&](int32_t raw) { return two_pi * (float)raw / 4096.0f - pi; };
        auto to_raw = [&](float rad) {
            return (int32_t)std::lround(4096.0f * (pi + rad) / two_pi);
        };
        if (!near(to_rad(2048), 0.0f, 1e-5f) || to_raw(0.0f) != 2048) {
            std::printf("FAIL position center encoding\n");
            ++fails;
        }
        if (to_raw(to_rad(1000)) != 1000) {
            std::printf("FAIL position round-trip\n");
            ++fails;
        }
    }

    if (fails) {
        std::printf("obs_layout: %d FAIL\n", fails);
        return 2;
    }
    std::printf("obs_layout: PASS (aligned with duck-control/obs.rs + bus.rs encoding)\n");
    return 0;
}
