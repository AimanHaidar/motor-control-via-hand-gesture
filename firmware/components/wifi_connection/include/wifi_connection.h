#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include "esp_err.h"

/**
 * @brief Connect to a Wi-Fi network (station mode)
 *
 * @param ssid     Wi-Fi SSID
 * @param password Wi-Fi password
 * @return ESP_OK if connected successfully, ESP_FAIL otherwise
 */
extern int wifi_connected;
esp_err_t wifi_connect(const char* ssid, const char* password);

#endif // WIFI_CONNECTION_H
