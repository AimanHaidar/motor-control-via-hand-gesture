#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "wifi_connection.h"

#define CONFIG_BROKER_URL "mqtt://192.168.0.81"


#define WIFI_SSID "MaPh"
#define WIFI_PASSWORD "Aiman#Alabsi#2018"

// Public API
static void mqtt_app_start(void);
void mqtt_start(void);

#endif // MQTT_COMM_H
