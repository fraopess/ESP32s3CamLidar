/*
 * Webserver Controller Module
 *
 * Unified API for webserver lifecycle management.
 * Coordinates WiFi connection and HTTP server.
 */

#ifndef WEBSERVER_CTRL_H
#define WEBSERVER_CTRL_H

#include "esp_err.h"
#include "app_httpd.h"

/**
 * Webserver configuration
 */
typedef struct {
    webserver_sensor_data_t *sensor_data;  // Pointer to shared sensor data
    SemaphoreHandle_t sensor_mutex;        // Mutex for sensor data access
    int stream_fps_limit;                  // Maximum streaming FPS
} webserver_config_t;

/**
 * Initialize webserver controller
 * Must be called before webserver_ctrl_start()
 *
 * @param config Webserver configuration
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t webserver_ctrl_init(const webserver_config_t *config);

/**
 * Start webserver (WiFi + HTTP server + mDNS)
 * Blocks until WiFi is connected
 *
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t webserver_ctrl_start(void);

/**
 * Stop webserver (HTTP server + WiFi)
 *
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t webserver_ctrl_stop(void);

/**
 * Check if webserver is running
 *
 * @return true if running, false otherwise
 */
bool webserver_ctrl_is_running(void);

#endif // WEBSERVER_CTRL_H
