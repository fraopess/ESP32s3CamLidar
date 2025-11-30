/*
 * HTTP Server Module
 *
 * Provides web interface for camera streaming, sensor data, and camera controls.
 */

#ifndef APP_HTTPD_H
#define APP_HTTPD_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Sensor data structure shared with HTTP server
 */
typedef struct {
    int64_t timestamp;      // Timestamp in microseconds
    float velocity_x;       // X velocity in m/s
    float velocity_y;       // Y velocity in m/s
    uint16_t distance;      // LiDAR distance in cm
    bool lidar_valid;       // LiDAR data validity flag
    float fps;              // Optical flow FPS
} webserver_sensor_data_t;

/**
 * HTTP server configuration
 */
typedef struct {
    webserver_sensor_data_t *sensor_data;  // Pointer to shared sensor data
    SemaphoreHandle_t sensor_mutex;        // Mutex for sensor data access
    int stream_fps_limit;                  // Maximum streaming FPS
} app_httpd_config_t;

/**
 * Start HTTP server
 *
 * @param config HTTP server configuration
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t app_httpd_start(const app_httpd_config_t *config);

/**
 * Stop HTTP server
 */
void app_httpd_stop(void);

/**
 * Check if HTTP server is running
 *
 * @return true if running, false otherwise
 */
bool app_httpd_is_running(void);

#endif // APP_HTTPD_H
