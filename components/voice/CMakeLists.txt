idf_component_register(
    SRCS "voice_control.c" "voice_match.c"
    INCLUDE_DIRS "include"
    REQUIRES sound
    PRIV_REQUIRES freertos
)
