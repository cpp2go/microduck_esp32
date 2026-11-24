idf_component_register(
    SRCS "cmd_bus.c"
    INCLUDE_DIRS "include" "../../common/include"
    PRIV_REQUIRES freertos
)