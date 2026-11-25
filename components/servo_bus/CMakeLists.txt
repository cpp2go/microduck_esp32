set(SERVO_SRCS "servo_bus.c" "ft_sts.c" "ut_s288.c" "ut_s288_codec.c")

idf_component_register(
    SRCS ${SERVO_SRCS}
    INCLUDE_DIRS "include" "../../common/include"
    PRIV_REQUIRES driver esp_timer freertos
)
