extern "C"{
    #include <stdio.h>
    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "wifi_connection.h"
    #include "mqtt_comm.h"
    #include "driver/gpio.h"
    #include <math.h>
    #include "driver/i2c.h"
    #include "driver/ledc.h"
    #include "esp_err.h"
    //#include "esp_lcd_panel_io.h"
    //#include "esp_lcd_panel_vendor.h"
    #include "rcl/rcl.h"
    #include "driver/i2c_master.h"
}

#include "PIDController.hpp"
//LEDC PIN FOR BUILT IN LED

#define LED_PIN GPIO_NUM_2
#define LED_CHANNEL LEDC_CHANNEL_0
#define LED_TIMER LEDC_TIMER_0
#define LED_MODE LEDC_HIGH_SPEED_MODE
#define LED_RESOLUTION LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define LED_FREQUENCY 8000 
#define ENCODER_PIN GPIO_NUM_19  // GPIO0 as example
// MOTOR DRIVER PINS
#define IN1_PIN GPIO_NUM_18 
#define IN2_PIN GPIO_NUM_17
#define ENA_PIN GPIO_NUM_23
#define ENA_CHANNEL LEDC_CHANNEL_1

// WIFI
#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

//PID parameters
float MOTOR_KP = 22.50;
float MOTOR_KI = 36.75;
float MOTOR_KD = 4.75;

PIDController pid(MOTOR_KP, MOTOR_KI, MOTOR_KD);

#define PULSES_PER_TURN 24

volatile uint16_t pulses = 0;
static portMUX_TYPE pulses_mux = portMUX_INITIALIZER_UNLOCKED;
float speed = 0.0;
float enable = 0.0;

//=== Forward Declarations =====================================================
void mqtt_message_callback(const char *topic, const char *payload);
void pid_message_callback(const char *topic, const char *payload);
void keep_wifi_and_mqtt(void* args);
static void IRAM_ATTR encoder_pin_isr_handler(void* arg);
void measureMotorSpeed(void *args);
void init_encoder_pin();
void init_driver_pins();
void init_ledc_2();
void pid_control_task(void *args);
//=============================================================================

extern "C" void app_main(void)
{

    init_encoder_pin();
    init_ledc_2();
    init_driver_pins();
    
    // task for keeping WiFi and MQTT connection alive
    xTaskCreate(
        keep_wifi_and_mqtt,
        "keep_wifi",
        4096,
        NULL,
        15,
        NULL
    );
    // task for measuring motor speed in rpm
    xTaskCreate(
        measureMotorSpeed,
        "measureMotorSpeed",
        2048, NULL, 12, NULL
    );

    xTaskCreate(
        pid_control_task,
        "pid_control_task",
        2048, NULL, 12, NULL
    );

    //this loop add subscribers to mqtt when mqtt connect
    int msg_id = 0;
    do{
        msg_id = mqtt_subscribe_topic("esp32/command/speed", 0, mqtt_message_callback);
        ESP_LOGI("hell: ","sent subscribe successful, msg_id=%d", msg_id);
        msg_id = mqtt_subscribe_topic("esp32/command/pid", 0, pid_message_callback);
        ESP_LOGI("hell: ","sent subscribe successful, msg_id=%d", msg_id);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    while(!mqtt_is_connected() || msg_id == 0);
    
}

/**
 * @brief this function handles the MQTT message callback.
 *
 */
void mqtt_message_callback(const char *topic, const char *payload)
{
    if (strcmp(payload, "1") == 0)
    {
        ESP_LOGI("mode", "1");
        
        speed = 60.0f;
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/5);
        ledc_update_duty(LED_MODE, LED_CHANNEL);

    }

    else if(strcmp(payload, "2") == 0){
        ESP_LOGI("mode", "2");
        
        speed = 100.0f;
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/4);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else if (strcmp(payload, "3") == 0)
    {
        ESP_LOGI("mode", "3");
        
        speed = 140.0f;
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/3);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else if (strcmp(payload, "4") == 0)
    {
        ESP_LOGI("mode", "4");
        
        speed = 200.0f;
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255/2);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else if (strcmp(payload, "5") == 0)
    {
        ESP_LOGI("mode", "5");
        
        speed = 250.0f;
        ledc_set_duty(LED_MODE, LED_CHANNEL, 255);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
    else
    {
        ESP_LOGI("mode", "stop");
        gpio_set_level(IN1_PIN, 0);
        gpio_set_level(IN2_PIN, 0);
        speed = 0;
        ledc_set_duty(LED_MODE, LED_CHANNEL, 0);
        ledc_update_duty(LED_MODE, LED_CHANNEL);
    }
}

/**
 * @brief this function handles the PID message callback.
 *
 */
void pid_message_callback(const char *topic, const char *payload)
{
    if (strcmp(topic, "esp32/command/pid") == 0)
    {
        // Parse the PID values from the payload
        float kp, ki, kd;

        sscanf(payload, "%f,%f,%f", &kp, &ki, &kd);
        
        ESP_LOGI("PID", "Received new PID values: Kp=%0.2f, Ki=%0.2f, Kd=%0.2f", kp, ki, kd);

        // Update the PID controller with the new values
        //pid.setTunings(kp, ki, kd);
    }
}

/**
 * @brief this function keeps the WiFi and MQTT connection alive.
 *
 */
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

/**
 * @brief this function handles the encoder pin interrupt.
 *
 */
static void IRAM_ATTR encoder_pin_isr_handler(void* arg) {
    portENTER_CRITICAL_ISR(&pulses_mux);
    pulses++;
    portEXIT_CRITICAL_ISR(&pulses_mux);
}



/*void encoder_pin_task(void* arg) {
    uint32_t pin;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &pin, portMAX_DELAY)) {
            // Safe to print/log here (task context)
            ESP_LOGI("encoder_pin: ", "encoder_pin pressed on GPIO %d\n", pin);
        }
    }
}*/
float V = 0.0f;
float V_Filt = 0.0f;        
float V_Prev = 0.0f;
void measureMotorSpeed(void *args){
    while(1){
        int32_t rpm = 0;
        // updating every 0.1 second
        taskENTER_CRITICAL(&pulses_mux); // protect pulses while reading/resetting
        // calculate rpm 
        V = ((60 *1000.0 / PULSES_PER_TURN)/ (50)) * pulses;
        pulses = 0;
        taskEXIT_CRITICAL(&pulses_mux);
        V_Filt = 0.854 * V_Filt + 0.0728 * V + 0.0728 * V_Prev;
        V_Prev = V;
        //ESP_LOGI("Speed: ", "Speed: %0.2f RPM", V_Filt,"set Speed: ", "%0.2f RPM", speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief this function initializes the encoder pin.
 *
 */
void init_encoder_pin() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;   // rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << ENCODER_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Install ISR service
    gpio_install_isr_service(0);  // default flags

    // Attach ISR handler
    gpio_isr_handler_add(ENCODER_PIN, encoder_pin_isr_handler, (void*)ENCODER_PIN);

    /*gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(encoder_pin_task, "encoder_pin_task", 2048, NULL, 10, NULL);
    */
}

void init_driver_pins(){
    ledc_channel_config_t ledc_channel1 = {
    .gpio_num   = ENA_PIN,
    .speed_mode = LED_MODE,
    .channel    = LEDC_CHANNEL_1,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel  = LED_TIMER,
    .duty       = 0,
    .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel1);

    // Set direction pins
    gpio_set_direction(IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_PIN, GPIO_MODE_OUTPUT);

}

/**
 * @brief this function initializes the LEDC for the built in led of esp32.
 *
 */
void init_ledc_2(void){
    /*
    
    */
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
}

void pid_control_task(void *args) {
    
    while (1) {
        // Implement PID control logic here
        pid.setValues(speed, V_Filt); // Example setpoint and actual value
        pid.control();
        //ESP_LOGI("PID", "PID Output: %0.2f", pid.output);
        float u = pid.output;

        // Clamp PWM
        int pwm = std::min(255, (int)fabs(u));

        // Set PWM duty
        
        

        // Motor direction
        if(speed != 0){
            if(u < 1023 && u > 0){
                gpio_set_level(IN1_PIN, 1);
                gpio_set_level(IN2_PIN, 0);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, pwm);
            } else if(u> 1023) {
                gpio_set_level(IN1_PIN, 1);
                gpio_set_level(IN2_PIN, 0);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 1023);
            }
            else {
                gpio_set_level(IN1_PIN, 0);
                gpio_set_level(IN2_PIN, 0);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
            }
        } else {
            gpio_set_level(IN1_PIN, 0);
            gpio_set_level(IN2_PIN, 0);
        }
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);



        /*if (pid.output < 256 && pid.output > 0) {
            
        }
        else if(pid.output <= 0){
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        }
        else if(pid.output > 255){
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 255);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        }*/
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}