#include <stdio.h>
#include "mqtt_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    blink_init();
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
}