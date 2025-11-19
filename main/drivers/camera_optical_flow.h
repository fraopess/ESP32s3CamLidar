/*
 * Camera-based Optical Flow Driver
 * Supports OV2640, OV5640, OV7725
 * Lucas-Kanade Optical Flow Algorithm
 */

#ifndef CAMERA_OPTICAL_FLOW_H
#define CAMERA_OPTICAL_FLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

// Camera configuration structure
typedef struct {
    int pin_pwdn;
    int pin_reset;
    int pin_xclk;
    int pin_siod;
    int pin_sioc;
    int pin_d7;
    int pin_d6;
    int pin_d5;
    int pin_d4;
    int pin_d3;
    int pin_d2;
    int pin_d1;
    int pin_d0;
    int pin_vsync;
    int pin_href;
    int pin_pclk;
    int xclk_freq;
    framesize_t frame_size;
    int img_width;
    int img_height;
    float focal_length_px;
    float pixel_size_mm;
    float flow_smoothing_alpha;
    float min_gradient_threshold;
    int min_valid_pixels;
} camera_config_optical_flow_t;

/**
 * @brief Initialize camera for optical flow
 *
 * @param config Camera configuration
 * @return ESP error code
 */
esp_err_t camera_optical_flow_init(const camera_config_optical_flow_t* config);

/**
 * @brief Allocate memory for image buffers
 *
 * @param img_width Image width
 * @param img_height Image height
 * @param img_prev Pointer to store previous image buffer
 * @param img_cur Pointer to store current image buffer
 * @return ESP error code
 */
esp_err_t camera_optical_flow_alloc_buffers(int img_width, int img_height, uint8_t** img_prev, uint8_t** img_cur);

/**
 * @brief Compute optical flow using Lucas-Kanade algorithm
 *
 * @param prev Previous image buffer
 * @param cur Current image buffer
 * @param width Image width
 * @param height Image height
 * @param step Pixel step for computation
 * @param min_gradient_threshold Minimum gradient threshold
 * @param min_valid_pixels Minimum valid pixels required
 * @param smoothing_alpha Smoothing factor (0-1)
 * @param flow_x Output: horizontal flow
 * @param flow_y Output: vertical flow
 */
void camera_optical_flow_compute(uint8_t* prev, uint8_t* cur, int width, int height, int step,
                                  float min_gradient_threshold, int min_valid_pixels,
                                  float smoothing_alpha, float* flow_x, float* flow_y);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_OPTICAL_FLOW_H
