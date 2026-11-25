idf_component_register(
    SRCS "sound.c" "es8311_codec.c" "sound_bank_data.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer freertos
)
