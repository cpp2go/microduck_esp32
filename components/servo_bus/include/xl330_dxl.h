// SPDX-License-Identifier: MIT
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool xl330_bus_init(void);
int xl330_ping(uint8_t id);

bool xl330_configure_servo(uint8_t id, uint16_t position_p_gain);

bool xl330_sync_read_pv(const uint8_t *ids, int n, float *pos_rad, float *vel_rad_s);
bool xl330_sync_write_goal_rad(const uint8_t *ids, const float *goal_rad, int n);

int32_t xl330_rad_to_position_int(float rad);
float xl330_position_unit_to_rad(int32_t unit);
bool xl330_set_torque_enable(uint8_t id, bool on);
float xl330_velocity_unit_to_rad_s(int32_t unit);

#ifdef __cplusplus
}
#endif