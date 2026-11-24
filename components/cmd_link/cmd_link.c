// SPDX-License-Identifier: MIT

#include "cmd_link.h"

#include "sdkconfig.h"

void cmd_link_start(void)
{
#if CONFIG_MICRODUCK_CMD_SDIO_ENABLE
    cmd_link_start_sdio_ingest();
#elif CONFIG_MICRODUCK_CMD_UART_ENABLE
    cmd_link_start_uart_ingest();
#endif
}