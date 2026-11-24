// SPDX-License-Identifier: MIT
//
// Command block → obs[48:61], matching pollen-robotics/microduck duck-control/src/obs.rs
// plus robotd [control] cmd_alpha / head_alpha EMA on the live command.

#include "cmd_bus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "microduck_model.h"
#include "sdkconfig.h"

#ifndef CONFIG_MICRODUCK_CMD_ALPHA
#define CONFIG_MICRODUCK_CMD_ALPHA 200
#endif
#ifndef CONFIG_MICRODUCK_CMD_HEAD_ALPHA
#define CONFIG_MICRODUCK_CMD_HEAD_ALPHA 200
#endif

static microduck_command_block_t s_cmd = MICRODUCK_COMMAND_IDLE;
static microduck_command_block_t s_smoothed = MICRODUCK_COMMAND_IDLE;
static uint32_t s_seq = 0;
static SemaphoreHandle_t s_mux;
static bool s_smooth_seeded = false;

static void lock_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
}

static float ema(float prev, float target, float alpha)
{
    return prev + alpha * (target - prev);
}

void cmd_bus_init(void)
{
    lock_init();
    s_cmd = MICRODUCK_COMMAND_IDLE;
    s_smoothed = MICRODUCK_COMMAND_IDLE;
    s_smooth_seeded = false;
}

void cmd_bus_set(const microduck_command_block_t *cmd, uint32_t seq)
{
    if (!cmd) {
        return;
    }
    lock_init();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        s_cmd = *cmd;
        s_seq = seq;
        xSemaphoreGive(s_mux);
    }
}

void cmd_bus_get(microduck_command_block_t *cmd, uint32_t *seq_out)
{
    if (!cmd) {
        return;
    }
    lock_init();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        *cmd = s_cmd;
        if (seq_out) {
            *seq_out = s_seq;
        }
        xSemaphoreGive(s_mux);
    }
}

void cmd_bus_apply_to_obs(float *obs61)
{
    if (!obs61) {
        return;
    }
    microduck_command_block_t c;
    cmd_bus_get(&c, NULL);

    const float cmd_a = CONFIG_MICRODUCK_CMD_ALPHA / 1000.0f;
    const float head_a = CONFIG_MICRODUCK_CMD_HEAD_ALPHA / 1000.0f;

    lock_init();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        if (!s_smooth_seeded) {
            s_smoothed = c;
            s_smooth_seeded = true;
        } else {
            /* robotd: cmd += alpha * (target - cmd); twist uses cmd_alpha, head+body use head_alpha. */
            for (int i = 0; i < 3; ++i) {
                s_smoothed.twist[i] = ema(s_smoothed.twist[i], c.twist[i], cmd_a);
            }
            for (int i = 0; i < 4; ++i) {
                s_smoothed.head_pose[i] = ema(s_smoothed.head_pose[i], c.head_pose[i], head_a);
            }
            for (int i = 0; i < 6; ++i) {
                s_smoothed.body_pose[i] = ema(s_smoothed.body_pose[i], c.body_pose[i], head_a);
            }
        }
        c = s_smoothed;
        xSemaphoreGive(s_mux);
    }

    int o = 48;
    for (int i = 0; i < 3; ++i) {
        obs61[o++] = c.twist[i];
    }
    for (int i = 0; i < 4; ++i) {
        obs61[o++] = c.head_pose[i];
    }
    /* duck-control/obs.rs: body x,y,yaw are always zero; z,roll,pitch from command.body */
    obs61[o++] = 0.0f;
    obs61[o++] = 0.0f;
    obs61[o++] = c.body_pose[0];
    obs61[o++] = c.body_pose[1];
    obs61[o++] = c.body_pose[2];
    obs61[o++] = 0.0f;
}
