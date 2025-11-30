/*
 * HTTP Server Implementation
 *
 * Provides HTTP endpoints for:
 * - GET /  - Web interface (HTML page)
 * - GET /stream - Grayscale camera stream
 * - GET /status - Sensor data JSON
 * - POST /control - Camera control API
 */

#include "app_httpd.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <string.h>
#include <sys/param.h>  // For MIN/MAX macros

// Include embedded HTML (will be generated from web_index.html)
#include "web_index_html.h"

static const char *TAG = "APP_HTTPD";

// HTTP server handle
static httpd_handle_t server = NULL;

// Shared sensor data and mutex
static webserver_sensor_data_t *g_sensor_data = NULL;
static SemaphoreHandle_t g_sensor_mutex = NULL;
static int g_stream_fps_limit = 15;

// Multipart boundary for streaming
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: application/octet-stream\r\nContent-Length: %u\r\nX-Width: %d\r\nX-Height: %d\r\n\r\n";

/**
 * Handler for GET / - Root index page
 */
static esp_err_t index_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving index page");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");

    return httpd_resp_send(req, web_index_html, HTTPD_RESP_USE_STRLEN);
}

/**
 * Handler for GET /stream - Grayscale frame stream
 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    char * part_buf[128];

    ESP_LOGI(TAG, "Stream session started");

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Calculate delay between frames based on FPS limit
    const int frame_delay_ms = 1000 / g_stream_fps_limit;

    while (true) {
        // Get frame from camera
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        } else {
            if (fb->format != PIXFORMAT_GRAYSCALE) {
                ESP_LOGW(TAG, "Camera not in GRAYSCALE format, got format: %d", fb->format);
                // Still try to stream it
            }

            _jpg_buf_len = fb->len;
        }

        if (res == ESP_OK) {
            // Send boundary
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        if (res == ESP_OK) {
            // Send headers with frame metadata
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART,
                                  _jpg_buf_len, fb->width, fb->height);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }

        if (res == ESP_OK) {
            // Send frame data (raw grayscale bytes)
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, _jpg_buf_len);
        }

        // Return frame buffer
        if (fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
        }

        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Stream session ended");
            break;
        }

        // Frame rate limiting - allow optical flow to run
        vTaskDelay(pdMS_TO_TICKS(frame_delay_ms));
    }

    return res;
}

/**
 * Handler for GET /status - Sensor telemetry JSON
 */
static esp_err_t status_handler(httpd_req_t *req)
{
    char json_response[256];
    webserver_sensor_data_t data = {0};

    // Read sensor data (thread-safe)
    if (g_sensor_mutex && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (g_sensor_data) {
            memcpy(&data, g_sensor_data, sizeof(webserver_sensor_data_t));
        }
        xSemaphoreGive(g_sensor_mutex);
    }

    // Get free heap for debugging
    uint32_t free_heap = esp_get_free_heap_size();

    // Format JSON response
    snprintf(json_response, sizeof(json_response),
             "{"
             "\"timestamp\":%lld,"
             "\"velocity_x\":%.3f,"
             "\"velocity_y\":%.3f,"
             "\"distance\":%u,"
             "\"lidar_valid\":%s,"
             "\"fps\":%.1f,"
             "\"free_heap\":%lu"
             "}",
             data.timestamp,
             data.velocity_x,
             data.velocity_y,
             data.distance,
             data.lidar_valid ? "true" : "false",
             data.fps,
             free_heap);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

/**
 * Parse camera control value from JSON
 */
static bool parse_control_value(const char *json, const char *key, int *value)
{
    char search_key[32];
    snprintf(search_key, sizeof(search_key), "\"%s\":", key);

    const char *p = strstr(json, search_key);
    if (p) {
        p += strlen(search_key);
        // Skip whitespace
        while (*p == ' ' || *p == '\t') p++;
        *value = atoi(p);
        return true;
    }

    return false;
}

/**
 * Handler for POST /control - Camera settings control
 */
static esp_err_t control_handler(httpd_req_t *req)
{
    char buf[200];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        ESP_LOGE(TAG, "Request too large");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too large");
        return ESP_FAIL;
    }

    // Read request body
    ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    ESP_LOGI(TAG, "Control request: %s", buf);

    // Get sensor handle
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera sensor not available");
        return ESP_FAIL;
    }

    // Parse and apply camera controls
    int value;
    if (parse_control_value(buf, "brightness", &value)) {
        ESP_LOGI(TAG, "Setting brightness: %d", value);
        s->set_brightness(s, value);
    }
    if (parse_control_value(buf, "contrast", &value)) {
        ESP_LOGI(TAG, "Setting contrast: %d", value);
        s->set_contrast(s, value);
    }
    if (parse_control_value(buf, "saturation", &value)) {
        ESP_LOGI(TAG, "Setting saturation: %d", value);
        s->set_saturation(s, value);
    }
    if (parse_control_value(buf, "aec_value", &value)) {
        ESP_LOGI(TAG, "Setting AEC value: %d", value);
        s->set_aec_value(s, value);
    }
    if (parse_control_value(buf, "agc_gain", &value)) {
        ESP_LOGI(TAG, "Setting AGC gain: %d", value);
        s->set_agc_gain(s, value);
    }
    if (parse_control_value(buf, "hmirror", &value)) {
        ESP_LOGI(TAG, "Setting H-mirror: %d", value);
        s->set_hmirror(s, value);
    }
    if (parse_control_value(buf, "vflip", &value)) {
        ESP_LOGI(TAG, "Setting V-flip: %d", value);
        s->set_vflip(s, value);
    }
    if (parse_control_value(buf, "ae_level", &value)) {
        ESP_LOGI(TAG, "Setting AE level: %d", value);
        s->set_ae_level(s, value);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

esp_err_t app_httpd_start(const app_httpd_config_t *config)
{
    if (server) {
        ESP_LOGW(TAG, "HTTP server already running");
        return ESP_OK;
    }

    if (!config || !config->sensor_data || !config->sensor_mutex) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_FAIL;
    }

    // Store configuration
    g_sensor_data = config->sensor_data;
    g_sensor_mutex = config->sensor_mutex;
    g_stream_fps_limit = config->stream_fps_limit;

    // HTTP server config
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = CONFIG_WEBSERVER_PORT;
    http_config.ctrl_port = CONFIG_WEBSERVER_PORT + 1;
    http_config.max_open_sockets = 7;
    http_config.stack_size = 8192;
    http_config.task_priority = 5;
    http_config.core_id = 1;  // Run on Core 1

    ESP_LOGI(TAG, "Starting HTTP server on port %d", http_config.server_port);

    // Start HTTP server
    if (httpd_start(&server, &http_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }

    // Register URI handlers
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &stream_uri);

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &status_uri);

    httpd_uri_t control_uri = {
        .uri       = "/control",
        .method    = HTTP_POST,
        .handler   = control_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &control_uri);

    ESP_LOGI(TAG, "HTTP server started successfully");
    ESP_LOGI(TAG, "Stream FPS limit: %d", g_stream_fps_limit);

    return ESP_OK;
}

void app_httpd_stop(void)
{
    if (server) {
        ESP_LOGI(TAG, "Stopping HTTP server");
        httpd_stop(server);
        server = NULL;
    }
}

bool app_httpd_is_running(void)
{
    return (server != NULL);
}
