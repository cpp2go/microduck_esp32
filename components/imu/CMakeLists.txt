idf_component_register(
    SRCS "src/imu.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer driver
)
