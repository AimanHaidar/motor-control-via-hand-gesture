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
    mqtt_publish(topic, "go to hell", 0);
}

void app_main(void)
{
    blink_init();
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    mqtt_subscribe_topic("/test/topic", 0, mqtt_message_callback);
    mqtt_start();
    
    while(1){
        mqtt_publish("/test/topic", "go to hell", 0);
        ESP_LOGI("mqtt_example", "Received message on topic");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}