# esp_hosted is always linked; UART bench builds still compile the SDIO
# stub behind CONFIG_MICRODUCK_CMD_SDIO_ENABLE. Component name follows the
# idf-component-manager layout (espressif__esp_hosted).
idf_component_register(
    SRCS "cmd_link.c" "cmd_link_sdio.c" "cmd_link_uart.c"
    INCLUDE_DIRS "include" "../../common/include"
    PRIV_REQUIRES cmd_bus driver esp_common espressif__esp_hosted
)
