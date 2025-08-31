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

typedef struct {
    const char *topic;
    int qos;
    void (*callback)(const char *topic, const char *payload);
} mqtt_subscribe_item_t;

// Public API
static void mqtt_app_start(void);
void mqtt_start(void);
// Publish a message
esp_err_t mqtt_publish(const char *topic, const char *payload, int qos);

// Subscribe to a topic
esp_err_t mqtt_subscribe_topic(const char *topic, int qos, void (*callback)(const char *, const char *));

// Unsubscribe from a topic
esp_err_t mqtt_unsubscribe_topic(const char *topic);

#endif // MQTT_COMM_H
