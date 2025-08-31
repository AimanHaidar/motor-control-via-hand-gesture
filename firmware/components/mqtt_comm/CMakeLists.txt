idf_component_register(SRCS "mqtt_comm.c" "wifi_connection.c"
                    REQUIRES mqtt nvs_flash esp_netif esp_wifi
                    INCLUDE_DIRS "include")
