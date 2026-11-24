// SPDX-License-Identifier: MIT
//
// Latest remote command from ESP32-C5 (or UART debug host) for the policy loop.
#pragma once

#include "microduck_link.h"

#ifdef __cplusplus
extern "C" {
#endif

void cmd_bus_init(void);
void cmd_bus_set(const microduck_command_block_t *cmd, uint32_t seq);
void cmd_bus_get(microduck_command_block_t *cmd, uint32_t *seq_out);
void cmd_bus_apply_to_obs(float *obs61);

#ifdef __cplusplus
}
#endif