extern "C"{
    #include <stdio.h>
    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "wifi_connection.h"
    #include "mqtt_comm.h"
    #include "driver/gpio.h"

    #include "driver/ledc.h"
    #include "esp_err.h"

    #include "rcl/rcl.h"
}

#include "PIDController.hpp"

#define LED_PIN 2
#define LED_CHANNEL LEDC_CHANNEL_0
#define LED_TIMER LEDC_TIMER_0
#define LED_MODE LEDC_HIGH_SPEED_MODE
#define LED_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution (0-255)
#define LED_FREQUENCY 5000 

#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

uint32_t pulses = 0;

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

void keep_wifi_and_mqtt(void* args){
    while(1){
        if(wifi_connected){
            ESP_LOGI("WIFI", "WiFi Still Connected");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        wifi_connect(WIFI_SSID, WIFI_PASSWORD);
        mqtt_start();
    }
}

static QueueHandle_t gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

static void IRAM_ATTR button_isr_handler(void* arg) {
    int pin = (int)arg;
    // For example, just print pin number
    ++pulses;
}

#define BUTTON_PIN GPIO_NUM_19  // GPIO0 as example

/*void button_task(void* arg) {
    uint32_t pin;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &pin, portMAX_DELAY)) {
            // Safe to print/log here (task context)
            ESP_LOGI("Button: ", "Button pressed on GPIO %d\n", pin);
        }
    }
}*/

void init_button() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;   // rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << BUTTON_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Install ISR service
    gpio_install_isr_service(0);  // default flags

    // Attach ISR handler
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, (void*)BUTTON_PIN);

    /*gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    */
}

extern "C" void app_main(void)
{

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LED_MODE,
        .duty_resolution  = LED_RESOLUTION,
        .timer_num        = LED_TIMER,
        .freq_hz          = LED_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure LED PWM channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LED_PIN,
        .speed_mode     = LED_MODE,
        .channel        = LED_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LED_TIMER,
        .duty           = 0, // start with LED off
        .hpoint         = 0,
    };
    ledc_channel_config(&ledc_channel);

    init_button();

    xTaskCreate(
        keep_wifi_and_mqtt,
        "keep_wifi",
        4096,
        NULL,
        3,
        NULL
    );

    int msg_id = 0;
    do{
        msg_id = mqtt_subscribe_topic("esp32/command/speed", 0, mqtt_message_callback);
        ESP_LOGI("hell: ","sent subscribe successful, msg_id=%d", msg_id);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(!mqtt_is_connected() || msg_id == 0);
    

}