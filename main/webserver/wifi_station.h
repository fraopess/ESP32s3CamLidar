/*
 * WiFi Station Mode Module
 *
 * Manages WiFi connection in station mode with auto-reconnect.
 */

#ifndef WIFI_STATION_H
#define WIFI_STATION_H

#include "esp_err.h"
#include "esp_netif.h"

/**
 * Callback function types
 */
typedef void (*wifi_connected_cb_t)(esp_ip4_addr_t ip);
typedef void (*wifi_disconnected_cb_t)(void);

/**
 * Initialize WiFi station mode
 * Must be called before wifi_station_strt()
 *
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t wifi_station_init(void);

/**
 * Start WiFi and connect to configured network
 * Uses CONFIG_ESP_WIFI_SSID and CONFIG_ESP_WIFI_PASSWORD from Kconfig
 *
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t wifi_station_strt(void);

/**
 * Stop WiFi and disconnect from network
 *
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t wifi_station_stp(void);

/**
 * Set callbacks for WiFi events
 *
 * @param connected_cb Called when WiFi connects and IP is assigned
 * @param disconnected_cb Called when WiFi disconnects
 */
void wifi_station_set_callbacks(wifi_connected_cb_t connected_cb,
                                 wifi_disconnected_cb_t disconnected_cb);

/**
 * Check if WiFi is currently connected
 *
 * @return true if connected, false otherwise
 */
bool wifi_station_is_connected(void);

#endif // WIFI_STATION_H
