idf_component_register(
    SRCS "sound.c" "es8311_codec.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer freertos
)
