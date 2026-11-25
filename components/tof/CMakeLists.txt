set(srcs "tof.c" "tof_obstacle.c")
set(priv esp_timer freertos)

if(CONFIG_MICRODUCK_TOF_I2C_PROBE)
    list(APPEND priv driver)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES ${priv}
)
