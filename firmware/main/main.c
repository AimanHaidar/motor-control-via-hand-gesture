#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_connection.h"
#include "mqtt_comm.h"
#include "driver/gpio.h"

#include "driver/ledc.h"
#include "esp_err.h"

#define LED_PIN 2
#define LED_CHANNEL LEDC_CHANNEL_0
#define LED_TIMER LEDC_TIMER_0
#define LED_MODE LEDC_HIGH_SPEED_MODE
#define LED_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution (0-255)
#define LED_FREQUENCY 5000 

#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

void mqtt_message_callback(const char *topic, const char *payload)
{
    if (strcmp(payload, "1") == 0)
    {
        ESP_LOGI("mode", "1");
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/5);
        ledc_update_duty(LED_MODE, LED_CHANNEL);

    }

    else if(strcmp(payload, "2") == 0){
        ESP_LOGI("mode", "2");
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/4);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else if (strcmp(payload, "3") == 0)
    {
        ESP_LOGI("mode", "3");
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/3);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else if (strcmp(payload, "4") == 0)
    {
        ESP_LOGI("mode", "4");
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/2);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else if (strcmp(payload, "5") == 0)
    {
        ESP_LOGI("mode", "5");
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else
    {
        ESP_LOGI("mode", "stop");
        ledc_set_duty(LED_MODE, LED_CHANNEL, 0);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
}

void app_main(void)
{

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LED_MODE,
        .timer_num        = LED_TIMER,
        .duty_resolution  = LED_RESOLUTION,
        .freq_hz          = LED_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure LED PWM channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LED_MODE,
        .channel        = LED_CHANNEL,
        .timer_sel      = LED_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LED_PIN,
        .duty           = 0, // start with LED off
        .hpoint         = 0,
    };
    ledc_channel_config(&ledc_channel);

    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    mqtt_start();
    int msg_id = 0;
    do{
        msg_id = mqtt_subscribe_topic("esp32/command/speed", 0, mqtt_message_callback);
        ESP_LOGI("hell: ","sent subscribe successful, msg_id=%d", msg_id);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(!mqtt_is_connected() || msg_id == 0);
    

}