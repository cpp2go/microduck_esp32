// SPDX-License-Identifier: MIT
// Feetech STS / HD1910M bus (FT-SCS protocol) — not Dynamixel Protocol 2.0.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ft_sts_bus_init(void);
int ft_sts_ping(uint8_t id);

/// Torque off → set Temporary KP (HD1910M @50) → torque on.
bool ft_sts_configure_servo(uint8_t id, uint16_t position_p_gain);

/// Sync-read Present Position + Velocity (addr 56, 4 bytes) for all IDs.
bool ft_sts_sync_read_pv(const uint8_t *ids, int n, float *pos_rad, float *vel_rad_s);

bool ft_sts_sync_write_goal_rad(const uint8_t *ids, const float *goal_rad, int n);

int32_t ft_sts_rad_to_position_int(float rad);
float ft_sts_position_unit_to_rad(int32_t unit);
float ft_sts_velocity_unit_to_rad_s(int32_t unit);
bool ft_sts_set_torque_enable(uint8_t id, bool on);

#ifdef __cplusplus
}
#endif
