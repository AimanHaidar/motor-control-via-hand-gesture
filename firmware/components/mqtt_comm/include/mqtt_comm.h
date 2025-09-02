#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"

typedef struct {
    const char *topic;
    int qos;
    void (*callback)(const char *topic, const char *payload);
} mqtt_subscribe_item_t;

// Opaque client handle (read-only for users)
extern esp_mqtt_client_handle_t client;

// Public API
void mqtt_start(void);
// Publish a message
esp_err_t mqtt_publish(const char *topic, const char *payload, int qos);

// Subscribe to a topic
esp_err_t mqtt_subscribe_topic(const char *topic, int qos, void (*callback)(const char *, const char *));

// Unsubscribe from a topic
esp_err_t mqtt_unsubscribe_topic(const char *topic);

// Return true if underlying MQTT client is connected
bool mqtt_is_connected(void);

// Block until connected or timeout (timeout_ms < 0 -> wait forever)
esp_err_t mqtt_wait_connected(int timeout_ms);

#endif // MQTT_COMM_H
