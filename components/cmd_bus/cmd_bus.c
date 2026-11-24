// SPDX-License-Identifier: MIT

#include "cmd_bus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "microduck_model.h"

static microduck_command_block_t s_cmd = MICRODUCK_COMMAND_IDLE;
static uint32_t s_seq = 0;
static SemaphoreHandle_t s_mux;

static void lock_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
}

void cmd_bus_init(void)
{
    lock_init();
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