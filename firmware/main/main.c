#include <stdio.h>
#include "blink.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_connection.h"
#include "mqtt_comm.h"

#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

const char *topic = "test/topic";
void mqtt_message_callback(const char *topic, const char *payload)
{
    ESP_LOGI("mqtt_example", "Received message on topic %s: %s", topic, payload);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    mqtt_start();
    int msg_id = 0;
    do{
        msg_id = mqtt_subscribe_topic("/test/topic", 0, mqtt_message_callback);
        ESP_LOGI("hell: ","sent subscribe successful, msg_id=%d", msg_id);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(!mqtt_is_connected() || msg_id == 0);
    

}