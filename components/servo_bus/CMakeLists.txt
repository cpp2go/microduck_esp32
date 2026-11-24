idf_component_register(
    SRCS "servo_bus.c" "ft_sts.c"
    INCLUDE_DIRS "include" "../../common/include"
    PRIV_REQUIRES driver esp_timer freertos
)
