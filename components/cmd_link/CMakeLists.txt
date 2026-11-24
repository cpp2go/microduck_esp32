set(_cmd_link_reqs cmd_bus driver esp_common)
if(CONFIG_MICRODUCK_CMD_SDIO_ENABLE)
    list(APPEND _cmd_link_reqs esp_hosted)
endif()

idf_component_register(
    SRCS "cmd_link.c" "cmd_link_sdio.c" "cmd_link_uart.c"
    INCLUDE_DIRS "include" "../../common/include"
    PRIV_REQUIRES ${_cmd_link_reqs}
)