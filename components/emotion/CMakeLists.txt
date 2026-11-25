idf_component_register(
    SRCS "emotion.c" "src/emotion_data.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES log
)
