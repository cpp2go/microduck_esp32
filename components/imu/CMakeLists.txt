# Kconfig is picked up via components/imu/Kconfig (project_extra_components).

idf_component_register(
    SRCS "src/imu.cpp" "vendor/bmi2.c" "vendor/bmi270.c"
    INCLUDE_DIRS "include" "vendor"
    PRIV_REQUIRES esp_timer driver
)
