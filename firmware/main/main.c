#include <stdio.h>
#include "blink.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_connection.h"

#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

void app_main(void)
{
    blink_init();
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    wifi_connect(WIFI_SSID, WIFI_PASSWORD);
}