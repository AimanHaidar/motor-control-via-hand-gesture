#ifndef BLINK_H
#define BLINK_H

// Correct ESP-IDF GPIO driver header
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BLINK_GPIO GPIO_NUM_2   // On most ESP32 boards, GPIO2 is onboard LED

void blink_init(void);
void blink_task(void *pvParameter);

#endif // BLINK_H
