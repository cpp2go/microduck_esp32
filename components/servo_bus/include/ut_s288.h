// SPDX-License-Identifier: MIT
// Unitree S288 bus — half-duplex TTL @ 6 Mbps (not Feetech FT-SCS).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ut_s288_bus_init(void);

/// Probe: send FOC query (Kp=0) and expect a valid feedback CRC.
int ut_s288_ping(uint8_t id);

bool ut_s288_configure_servo(uint8_t id, float kp, float kd);

/// Sequential cmd/fbk for each ID (Unitree has no SyncRead). Fills output-shaft rad.
bool ut_s288_sync_read_pv(const uint8_t *ids, int n, float *pos_rad, float *vel_rad_s);

bool ut_s288_sync_write_goal_rad(const uint8_t *ids, const float *goal_rad, int n);

bool ut_s288_set_torque_enable(uint8_t id, bool on);

#ifdef __cplusplus
}
#endif
