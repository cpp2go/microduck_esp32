idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "." "../../common/include"
                       PRIV_REQUIRES driver esp_timer nvs_flash)