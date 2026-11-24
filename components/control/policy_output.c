// SPDX-License-Identifier: MIT

#include "policy_output.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static float s_actions[MICRODUCK_ACT_DIM];
static SemaphoreHandle_t s_mux;

static void ensure_mux(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
}

void microduck_policy_set_actions(const float actions[MICRODUCK_ACT_DIM])
{
    ensure_mux();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        memcpy(s_actions, actions, sizeof(s_actions));
        xSemaphoreGive(s_mux);
    }
}

void microduck_policy_get_actions(float out[MICRODUCK_ACT_DIM])
{
    ensure_mux();
    if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
        memcpy(out, s_actions, sizeof(s_actions));
        xSemaphoreGive(s_mux);
    }
}