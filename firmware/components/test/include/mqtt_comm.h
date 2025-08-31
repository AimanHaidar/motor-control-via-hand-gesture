#ifndef BLINK_H
#define BLINK_H

#include "driver_gpio.h"

#define BLINK_GPIO GPIO_NUM_2   // On most ESP32 boards, GPIO2 is onboard LED

void blink_init(void);
void blink_task(void *pvParameter);

#endif // BLINK_H
