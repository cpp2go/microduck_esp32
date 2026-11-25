set(srcs "tof.c" "tof_obstacle.c")
set(priv esp_timer freertos)
set(incs "include")

if(CONFIG_MICRODUCK_TOF_I2C_PROBE OR CONFIG_MICRODUCK_TOF_ULD)
    list(APPEND priv driver)
endif()

if(CONFIG_MICRODUCK_TOF_ULD)
    list(APPEND srcs
        "vendor/vl53l8cx/platform_esp.c"
        "vendor/vl53l8cx/shim_esp.c"
        "vendor/vl53l8cx/vl53l8cx_api.c"
    )
    list(APPEND incs "vendor/vl53l8cx")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${incs}
    PRIV_REQUIRES ${priv}
)
