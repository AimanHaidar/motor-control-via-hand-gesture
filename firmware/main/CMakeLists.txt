idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES PIDController wifi_connection mqtt_comm driver micro_ros_espidf_component
)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
set(CONFIG_KCONFIG_PROJBUILD "${CMAKE_CURRENT_SOURCE_DIR}/Kconfig.projbuild")