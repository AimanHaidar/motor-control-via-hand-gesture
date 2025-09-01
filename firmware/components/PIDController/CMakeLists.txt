idf_component_register(
    SRCS "PIDController.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer
)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)