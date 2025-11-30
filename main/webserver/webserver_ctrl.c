/*
 * Webserver Controller Implementation
 *
 * Coordinates WiFi, HTTP server, and mDNS services.
 */

#include "webserver_ctrl.h"
#include "wifi_station.h"
#include "app_httpd.h"
#include "esp_log.h"
#include "mdns.h"
#include <string.h>

static const char *TAG = "WEBSERVER_CTRL";

// State
static bool s_initialized = false;
static bool s_running = false;
static webserver_config_t s_config = {0};

/**
 * WiFi connected callback - start HTTP server and mDNS
 */
static void on_wifi_connected(esp_ip4_addr_t ip)
{
    ESP_LOGI(TAG, "WiFi connected with IP: " IPSTR, IP2STR(&ip));

    // Start mDNS
    ESP_LOGI(TAG, "Starting mDNS...");
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS init failed: %d", err);
    } else {
        mdns_hostname_set(CONFIG_MDNS_HOSTNAME);
        mdns_instance_name_set("ESP32-S3 Camera");

        // Add HTTP service
        mdns_service_add(NULL, "_http", "_tcp", CONFIG_WEBSERVER_PORT, NULL, 0);

        ESP_LOGI(TAG, "mDNS started: %s.local", CONFIG_MDNS_HOSTNAME);
    }

    // Start HTTP server
    app_httpd_config_t http_config = {
        .sensor_data = s_config.sensor_data,
        .sensor_mutex = s_config.sensor_mutex,
        .stream_fps_limit = s_config.stream_fps_limit,
    };

    if (app_httpd_start(&http_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    } else {
        ESP_LOGI(TAG, "Webserver ready!");
        ESP_LOGI(TAG, "  - http://" IPSTR, IP2STR(&ip));
        ESP_LOGI(TAG, "  - http://%s.local", CONFIG_MDNS_HOSTNAME);
    }
}

/**
 * WiFi disconnected callback - stop HTTP server
 */
static void on_wifi_disconnected(void)
{
    ESP_LOGW(TAG, "WiFi disconnected");

    if (app_httpd_is_running()) {
        ESP_LOGI(TAG, "Stopping HTTP server");
        app_httpd_stop();
    }

    // mDNS will auto-stop when WiFi disconnects
}

esp_err_t webserver_ctrl_init(const webserver_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Webserver controller already initialized");
        return ESP_OK;
    }

    if (!config || !config->sensor_data || !config->sensor_mutex) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_FAIL;
    }

    // Store configuration
    memcpy(&s_config, config, sizeof(webserver_config_t));

    // Initialize WiFi
    if (wifi_station_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed");
        return ESP_FAIL;
    }

    // Set WiFi callbacks
    wifi_station_set_callbacks(on_wifi_connected, on_wifi_disconnected);

    s_initialized = true;
    ESP_LOGI(TAG, "Webserver controller initialized");

    return ESP_OK;
}

esp_err_t webserver_ctrl_start(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Webserver not initialized, call webserver_ctrl_init() first");
        return ESP_FAIL;
    }

    if (s_running) {
        ESP_LOGW(TAG, "Webserver already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting webserver...");

    // Start WiFi (will trigger callback to start HTTP server when connected)
    if (wifi_station_strt() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed");
        return ESP_FAIL;
    }

    s_running = true;
    ESP_LOGI(TAG, "Webserver started");

    return ESP_OK;
}

esp_err_t webserver_ctrl_stop(void)
{
    if (!s_running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping webserver...");

    // Stop HTTP server
    if (app_httpd_is_running()) {
        app_httpd_stop();
    }

    // Stop mDNS
    mdns_free();

    // Stop WiFi
    wifi_station_stp();

    s_running = false;
    ESP_LOGI(TAG, "Webserver stopped");

    return ESP_OK;
}

bool webserver_ctrl_is_running(void)
{
    return s_running;
}
