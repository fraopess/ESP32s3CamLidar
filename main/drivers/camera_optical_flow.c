/*
 * Camera-based Optical Flow Driver Implementation
 */

#include "camera_optical_flow.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

static const char *TAG = "CAM_FLOW";

static float filtered_flow_x = 0.0f;
static float filtered_flow_y = 0.0f;

// ============================================================================
// Initialize camera
// ============================================================================
esp_err_t camera_optical_flow_init(const camera_config_optical_flow_t* config)
{
    camera_config_t cam_config = {
        .pin_pwdn = config->pin_pwdn,
        .pin_reset = config->pin_reset,
        .pin_xclk = config->pin_xclk,
        .pin_sccb_sda = config->pin_siod,
        .pin_sccb_scl = config->pin_sioc,
        .pin_d7 = config->pin_d7,
        .pin_d6 = config->pin_d6,
        .pin_d5 = config->pin_d5,
        .pin_d4 = config->pin_d4,
        .pin_d3 = config->pin_d3,
        .pin_d2 = config->pin_d2,
        .pin_d1 = config->pin_d1,
        .pin_d0 = config->pin_d0,
        .pin_vsync = config->pin_vsync,
        .pin_href = config->pin_href,
        .pin_pclk = config->pin_pclk,

        .xclk_freq_hz = config->xclk_freq,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = config->frame_size,

        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();

    // Configuration commune
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, 5);
    s->set_bpc(s, 0);
    s->set_wpc(s, 0);
    s->set_lenc(s, 0);

    return ESP_OK;
}

// ============================================================================
// Allocate memory for image buffers
// ============================================================================
esp_err_t camera_optical_flow_alloc_buffers(int img_width, int img_height, uint8_t** img_prev, uint8_t** img_cur)
{
    size_t buffer_size = img_width * img_height;

    *img_prev = (uint8_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    *img_cur = (uint8_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);

    if (!(*img_prev) || !(*img_cur)) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

// ============================================================================
// Compute optical flow using Lucas-Kanade
// ============================================================================
void camera_optical_flow_compute(uint8_t* prev, uint8_t* cur, int width, int height, int step,
                                  float min_gradient_threshold, int min_valid_pixels,
                                  float smoothing_alpha, volatile float* flow_x, volatile float* flow_y)
{
    float sum_Ix2 = 0.0f;
    float sum_Iy2 = 0.0f;
    float sum_IxIy = 0.0f;
    float sum_IxIt = 0.0f;
    float sum_IyIt = 0.0f;
    int valid_pixels = 0;

    const int margin = 5;
    const int start_y = margin;
    const int end_y = height - margin;
    const int start_x = margin;
    const int end_x = width - margin;

    for (int y = start_y; y < end_y; y += step) {
        const int row_offset = y * width;
        const int row_offset_prev = (y - 1) * width;
        const int row_offset_next = (y + 1) * width;

        for (int x = start_x; x < end_x; x += step) {
            const int idx = row_offset + x;

            const int Ix_int =
                -cur[idx - 1 - width] + cur[idx + 1 - width] +
                -(cur[idx - 1] << 1) + (cur[idx + 1] << 1) +
                -cur[idx - 1 + width] + cur[idx + 1 + width];
            const float Ix = (float)Ix_int * 0.125f;

            const int Iy_int =
                -cur[row_offset_prev + x - 1] - (cur[row_offset_prev + x] << 1) - cur[row_offset_prev + x + 1] +
                 cur[row_offset_next + x - 1] + (cur[row_offset_next + x] << 1) + cur[row_offset_next + x + 1];
            const float Iy = (float)Iy_int * 0.125f;

            const float It = (float)cur[idx] - (float)prev[idx];

            const float grad_mag_sq = Ix * Ix + Iy * Iy;
            if (grad_mag_sq > min_gradient_threshold * min_gradient_threshold) {
                sum_Ix2 += Ix * Ix;
                sum_Iy2 += Iy * Iy;
                sum_IxIy += Ix * Iy;
                sum_IxIt += Ix * It;
                sum_IyIt += Iy * It;
                valid_pixels++;
            }
        }
    }

    if (valid_pixels > min_valid_pixels) {
        const float det = sum_Ix2 * sum_Iy2 - sum_IxIy * sum_IxIy;

        if (fabsf(det) > 1e-5f) {
            const float inv_det = 1.0f / det;

            float calc_flow_x = -(sum_Iy2 * sum_IxIt - sum_IxIy * sum_IyIt) * inv_det;
            float calc_flow_y = -(sum_Ix2 * sum_IyIt - sum_IxIy * sum_IxIt) * inv_det;

            calc_flow_x = (calc_flow_x > 50.0f) ? 50.0f : ((calc_flow_x < -50.0f) ? -50.0f : calc_flow_x);
            calc_flow_y = (calc_flow_y > 50.0f) ? 50.0f : ((calc_flow_y < -50.0f) ? -50.0f : calc_flow_y);

            filtered_flow_x = filtered_flow_x * (1.0f - smoothing_alpha) + calc_flow_x * smoothing_alpha;
            filtered_flow_y = filtered_flow_y * (1.0f - smoothing_alpha) + calc_flow_y * smoothing_alpha;

            *flow_x = filtered_flow_x;
            *flow_y = filtered_flow_y;
        } else {
            *flow_x = 0.0f;
            *flow_y = 0.0f;
        }
    } else {
        *flow_x = 0.0f;
        *flow_y = 0.0f;
    }
}
