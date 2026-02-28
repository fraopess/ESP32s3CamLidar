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
static SemaphoreHandle_t g_fps_mutex = NULL;

// Declare external optical flow config (defined in main.c)
typedef struct {
    float min_gradient_threshold;
    int min_valid_pixels;
    float flow_smoothing_alpha;
    int optical_flow_step;
    float focal_length_px;
    float pixel_size_mm;
} optical_flow_config_t;

extern optical_flow_config_t of_config;
extern SemaphoreHandle_t of_config_mutex;

// Frame size control
typedef struct {
    int width;
    int height;
    framesize_t framesize;
} frame_config_t;

extern frame_config_t frame_config;
extern SemaphoreHandle_t frame_config_mutex;
extern bool camera_task_suspended;
extern uint8_t *img_prev, *img_cur;

// Forward declare realloc function
esp_err_t camera_optical_flow_realloc_buffers(int width, int height,
                                               uint8_t** prev, uint8_t** cur);

// Multipart boundary for streaming
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
// Stream part headers now include sensor data (avoids needing separate /status requests)
static const char* _STREAM_PART = "Content-Type: application/octet-stream\r\n"
    "Content-Length: %u\r\n"
    "X-Width: %d\r\n"
    "X-Height: %d\r\n"
    "X-Dist: %u\r\n"
    "X-Valid: %d\r\n"
    "X-VelX: %.3f\r\n"
    "X-VelY: %.3f\r\n"
    "X-Fps: %.1f\r\n"
    "X-Heap: %lu\r\n"
    "\r\n";

/**
 * Handler for GET / - Root index page
 */
static esp_err_t index_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Serving index page");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");

    return httpd_resp_send(req, (const char *)web_index_html, HTTPD_RESP_USE_STRLEN);
}

/**
 * Handler for GET /stream - Grayscale frame stream
 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    char part_buf[256];

    ESP_LOGI(TAG, "Stream session started");

    // Check if camera is available
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "Camera sensor not available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not available");
        return ESP_FAIL;
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set content type");
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Calculate delay between frames based on FPS limit
    int current_fps_limit = g_stream_fps_limit;
    if (g_fps_mutex && xSemaphoreTake(g_fps_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        current_fps_limit = g_stream_fps_limit;
        xSemaphoreGive(g_fps_mutex);
    }
    int frame_delay_ms = 1000 / current_fps_limit;

    int frame_counter = 0;
    while (true) {
        // Periodically update FPS limit (every 30 frames)
        if (frame_counter % 30 == 0 && g_fps_mutex && xSemaphoreTake(g_fps_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            current_fps_limit = g_stream_fps_limit;
            frame_delay_ms = 1000 / current_fps_limit;
            xSemaphoreGive(g_fps_mutex);
        }
        // Get frame from camera
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (fb->format != PIXFORMAT_GRAYSCALE) {
            ESP_LOGW(TAG, "Camera not in GRAYSCALE format, got format: %d", fb->format);
            // Still try to stream it
        }

        _jpg_buf_len = fb->len;

        // Log first frame details
        if (frame_counter == 0) {
            ESP_LOGI(TAG, "First frame: size=%zu, width=%d, height=%d, format=%d",
                     fb->len, fb->width, fb->height, fb->format);
        }

        // Send boundary
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send boundary (frame %d), err=0x%x (%d)",
                     frame_counter, res, res);
            break;
        }

        // Read sensor data for this frame's headers
        webserver_sensor_data_t sdata = {0};
        if (g_sensor_mutex && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            if (g_sensor_data) {
                memcpy(&sdata, g_sensor_data, sizeof(webserver_sensor_data_t));
            }
            xSemaphoreGive(g_sensor_mutex);
        }

        // Send headers with frame metadata + sensor data
        size_t hlen = snprintf((char *)part_buf, sizeof(part_buf), _STREAM_PART,
                              _jpg_buf_len, fb->width, fb->height,
                              sdata.distance, sdata.lidar_valid,
                              sdata.velocity_x, sdata.velocity_y,
                              sdata.fps, esp_get_free_heap_size());
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send headers (frame %d), err=0x%x (%d)",
                     frame_counter, res, res);
            break;
        }

        // Send frame data (raw grayscale bytes)
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, _jpg_buf_len);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send frame data (frame %d), err=0x%x (%d), heap=%lu",
                     frame_counter, res, res, esp_get_free_heap_size());
            break;
        }

        // Return frame buffer
        esp_camera_fb_return(fb);
        fb = NULL;

        frame_counter++;
        if (frame_counter % 30 == 0) {
            ESP_LOGI(TAG, "Stream: sent %d frames, heap=%lu",
                     frame_counter, esp_get_free_heap_size());
        }

        // Frame rate limiting
        vTaskDelay(pdMS_TO_TICKS(frame_delay_ms));
    }

    // Clean up if we still have a frame buffer
    if (fb) {
        esp_camera_fb_return(fb);
    }

    ESP_LOGW(TAG, "Stream session ended");

    return res;
}

/**
 * Handler for GET /status - Sensor telemetry JSON
 */
static esp_err_t status_handler(httpd_req_t *req)
{
    char json_response[256];
    webserver_sensor_data_t data = {0};
    static uint32_t status_req_count = 0;

    // Read sensor data (thread-safe)
    if (g_sensor_mutex && xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (g_sensor_data) {
            memcpy(&data, g_sensor_data, sizeof(webserver_sensor_data_t));
        }
        xSemaphoreGive(g_sensor_mutex);
    }

    status_req_count++;
    if (status_req_count % 10 == 1) {
        ESP_LOGI(TAG, "/status[%lu]: dist=%u valid=%d fps=%.1f",
                 status_req_count, data.distance, data.lidar_valid, data.fps);
    }

    // Get free heap for debugging
    uint32_t free_heap = esp_get_free_heap_size();

    // Get current stream FPS limit
    int current_stream_fps = g_stream_fps_limit;
    if (g_fps_mutex && xSemaphoreTake(g_fps_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        current_stream_fps = g_stream_fps_limit;
        xSemaphoreGive(g_fps_mutex);
    }

    // Get current frame dimensions
    int current_frame_width = 160;
    int current_frame_height = 120;
    if (frame_config_mutex && xSemaphoreTake(frame_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        current_frame_width = frame_config.width;
        current_frame_height = frame_config.height;
        xSemaphoreGive(frame_config_mutex);
    }

    // Format JSON response
    snprintf(json_response, sizeof(json_response),
             "{"
             "\"timestamp\":%lld,"
             "\"velocity_x\":%.3f,"
             "\"velocity_y\":%.3f,"
             "\"distance\":%u,"
             "\"lidar_valid\":%s,"
             "\"fps\":%.1f,"
             "\"free_heap\":%lu,"
             "\"stream_fps\":%d,"
             "\"frame_width\":%d,"
             "\"frame_height\":%d"
             "}",
             data.timestamp,
             data.velocity_x,
             data.velocity_y,
             data.distance,
             data.lidar_valid ? "true" : "false",
             data.fps,
             free_heap,
             current_stream_fps,
             current_frame_width,
             current_frame_height);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");  // Close connection immediately after response
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");  // Don't cache status data

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
            httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Request Timeout");
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
    if (parse_control_value(buf, "stream_fps", &value)) {
        if (value >= 5 && value <= 60) {
            if (g_fps_mutex && xSemaphoreTake(g_fps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_stream_fps_limit = value;
                xSemaphoreGive(g_fps_mutex);
                ESP_LOGI(TAG, "Setting stream FPS: %d", value);
            }
        } else {
            ESP_LOGW(TAG, "Invalid FPS value: %d (must be 5-60)", value);
        }
    }

    // Optical Flow parameters
    if (parse_control_value(buf, "of_gradient_threshold", &value)) {
        float fval = (float)value / 10.0f;  // Slider sends 10x value
        if (fval >= 1.0f && fval <= 15.0f) {
            if (of_config_mutex && xSemaphoreTake(of_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                of_config.min_gradient_threshold = fval;
                xSemaphoreGive(of_config_mutex);
                ESP_LOGI(TAG, "Setting gradient threshold: %.1f", fval);
            }
        }
    }
    if (parse_control_value(buf, "of_valid_pixels", &value)) {
        if (value >= 50 && value <= 300) {
            if (of_config_mutex && xSemaphoreTake(of_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                of_config.min_valid_pixels = value;
                xSemaphoreGive(of_config_mutex);
                ESP_LOGI(TAG, "Setting min valid pixels: %d", value);
            }
        }
    }
    if (parse_control_value(buf, "of_smoothing", &value)) {
        float fval = (float)value / 100.0f;  // Slider sends 0-100, convert to 0.0-1.0
        if (fval >= 0.0f && fval <= 1.0f) {
            if (of_config_mutex && xSemaphoreTake(of_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                of_config.flow_smoothing_alpha = fval;
                xSemaphoreGive(of_config_mutex);
                ESP_LOGI(TAG, "Setting smoothing alpha: %.2f", fval);
            }
        }
    }
    if (parse_control_value(buf, "of_step", &value)) {
        if (value >= 1 && value <= 4) {
            if (of_config_mutex && xSemaphoreTake(of_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                of_config.optical_flow_step = value;
                xSemaphoreGive(of_config_mutex);
                ESP_LOGI(TAG, "Setting optical flow step: %d", value);
            }
        }
    }
    if (parse_control_value(buf, "of_focal_length", &value)) {
        float fval = (float)value / 10.0f;  // Slider sends 10x value
        if (fval >= 20.0f && fval <= 60.0f) {
            if (of_config_mutex && xSemaphoreTake(of_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                of_config.focal_length_px = fval;
                xSemaphoreGive(of_config_mutex);
                ESP_LOGI(TAG, "Setting focal length: %.1f px", fval);
            }
        }
    }
    if (parse_control_value(buf, "of_pixel_size", &value)) {
        float fval = (float)value / 10000.0f;  // Slider sends 10000x value
        if (fval >= 0.001f && fval <= 0.01f) {
            if (of_config_mutex && xSemaphoreTake(of_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                of_config.pixel_size_mm = fval;
                xSemaphoreGive(of_config_mutex);
                ESP_LOGI(TAG, "Setting pixel size: %.4f mm", fval);
            }
        }
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");  // Close connection immediately after response
    return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

/**
 * Handler for POST /change_framesize - Change camera frame size
 */
static esp_err_t change_framesize_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive request");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    int value;
    if (!parse_control_value(buf, "framesize", &value)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid framesize parameter");
        return ESP_FAIL;
    }

    // Map web values to framesize constants
    framesize_t new_framesize;
    int new_width, new_height;

    switch (value) {
        case 0:  // QQVGA 160x120
            new_framesize = FRAMESIZE_QQVGA;
            new_width = 160;
            new_height = 120;
            break;
        case 1:  // QVGA 320x240
            new_framesize = FRAMESIZE_QVGA;
            new_width = 320;
            new_height = 240;
            break;
        case 2:  // VGA 640x480
            new_framesize = FRAMESIZE_VGA;
            new_width = 640;
            new_height = 480;
            break;
        default:
            ESP_LOGW(TAG, "Invalid framesize: %d", value);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid framesize value");
            return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Changing frame size to %dx%d...", new_width, new_height);

    // Suspend camera task
    camera_task_suspended = true;
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for current frame to finish

    // Change sensor framesize
    sensor_t *s = esp_camera_sensor_get();
    if (s->set_framesize(s, new_framesize) != 0) {
        ESP_LOGE(TAG, "Failed to set framesize on sensor");
        camera_task_suspended = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set camera framesize");
        return ESP_FAIL;
    }

    // Reallocate buffers
    esp_err_t err = camera_optical_flow_realloc_buffers(new_width, new_height,
                                                         &img_prev, &img_cur);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reallocate buffers");
        camera_task_suspended = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to reallocate frame buffers");
        return ESP_FAIL;
    }

    // Update frame config
    if (frame_config_mutex && xSemaphoreTake(frame_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        frame_config.width = new_width;
        frame_config.height = new_height;
        frame_config.framesize = new_framesize;
        xSemaphoreGive(frame_config_mutex);
    }

    // Resume camera task
    camera_task_suspended = false;

    ESP_LOGI(TAG, "Frame size changed successfully to %dx%d", new_width, new_height);
    httpd_resp_set_type(req, "application/json");
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
    g_fps_mutex = xSemaphoreCreateMutex();

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

    httpd_uri_t change_framesize_uri = {
        .uri       = "/change_framesize",
        .method    = HTTP_POST,
        .handler   = change_framesize_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &change_framesize_uri);

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
