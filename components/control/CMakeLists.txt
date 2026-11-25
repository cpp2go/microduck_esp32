idf_component_register(
    SRCS "policy_output.c" "loop_validate.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos
)