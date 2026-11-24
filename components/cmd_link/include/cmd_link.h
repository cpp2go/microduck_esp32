// SPDX-License-Identifier: MIT
//
// P4-side ingest of microduck_link frames from ESP32-C5.
// Default: SDIO via esp_hosted peer-data. Optional UART for bench only.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// Starts the configured transport (SDIO or UART).
void cmd_link_start(void);

void cmd_link_start_sdio_ingest(void);
void cmd_link_start_uart_ingest(void);

#ifdef __cplusplus
}
#endif