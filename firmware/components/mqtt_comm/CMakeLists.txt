idf_component_register(SRCS "mqtt_comm.c"
                    REQUIRES mqtt nvs_flash esp_netif
                    INCLUDE_DIRS "include")
