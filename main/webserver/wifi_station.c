/*
 * WiFi Station Mode Implementation
 *
 * Handles WiFi connection, event management, and auto-reconnect.
 */

#include "wifi_station.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "WIFI_STATION";

// Event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Maximum retry attempts
#define WIFI_MAXIMUM_RETRY  10

// State variables
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;
static bool s_is_connected = false;
static esp_netif_t *s_netif = NULL;

// Callbacks
static wifi_connected_cb_t s_connected_cb = NULL;
static wifi_disconnected_cb_t s_disconnected_cb = NULL;

/**
 * WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started, connecting to AP...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_is_connected = false;

        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to AP... (attempt %d/%d)",
                     s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to AP after %d attempts", WIFI_MAXIMUM_RETRY);
        }

        // Call disconnect callback
        if (s_disconnected_cb) {
            s_disconnected_cb();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));

        s_retry_num = 0;
        s_is_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Call connect callback
        if (s_connected_cb) {
            s_connected_cb(event->ip_info.ip);
        }
    }
}

esp_err_t wifi_station_init(void)
{
    if (s_wifi_event_group != NULL) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return ESP_OK;
    }

    // Create event group
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    // Initialize TCP/IP adapter
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default WiFi station interface
    s_netif = esp_netif_create_default_wifi_sta();
    if (!s_netif) {
        ESP_LOGE(TAG, "Failed to create WiFi station netif");
        return ESP_FAIL;
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_LOGI(TAG, "WiFi station initialized");
    return ESP_OK;
}

esp_err_t wifi_station_strt(void)
{
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "WiFi not initialized, call wifi_station_init() first");
        return ESP_FAIL;
    }

    // Configure WiFi station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi station started, connecting to SSID: %s", CONFIG_ESP_WIFI_SSID);

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to AP");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Unexpected event");
        return ESP_FAIL;
    }
}

esp_err_t wifi_station_stp(void)
{
    if (s_is_connected) {
        s_is_connected = false;
        ESP_LOGI(TAG, "Stopping WiFi station");
        return esp_wifi_stop();
    }

    return ESP_OK;
}

void wifi_station_set_callbacks(wifi_connected_cb_t connected_cb,
                                 wifi_disconnected_cb_t disconnected_cb)
{
    s_connected_cb = connected_cb;
    s_disconnected_cb = disconnected_cb;
    ESP_LOGI(TAG, "WiFi callbacks registered");
}

bool wifi_station_is_connected(void)
{
    return s_is_connected;
}
