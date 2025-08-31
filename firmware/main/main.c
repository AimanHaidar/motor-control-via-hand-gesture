#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_connection.h"
#include "mqtt_comm.h"
#include "driver/gpio.h"

#define BLINK_GPIO GPIO_NUM_2

#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

const char *topic = "test/topic";
void mqtt_message_callback(const char *topic, const char *payload)
{
    if (strcmp(payload, "fuck") == 0)
    {
        ESP_LOGI("hell","fuck command received");
        gpio_set_level(BLINK_GPIO, 1);

    }
    else{
        ESP_LOGI("hell:","stop command received");
        gpio_set_level(BLINK_GPIO, 0);

    }
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLINK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    mqtt_start();
    int msg_id = 0;
    do{
        msg_id = mqtt_subscribe_topic("esp32/command/fuck", 0, mqtt_message_callback);
        ESP_LOGI("hell: ","sent subscribe successful, msg_id=%d", msg_id);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(!mqtt_is_connected() || msg_id == 0);
    

}