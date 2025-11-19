/*
 * TFMini LiDAR Driver
 * UART Communication at 115200 baud
 */

#ifndef TFMINI_H
#define TFMINI_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// TFMini Configuration
#define TFMINI_BAUDRATE    115200

// TFMini Data Structure
typedef struct {
    uint16_t distance;    // Distance in cm
    uint16_t strength;    // Signal strength
    bool valid;           // Valid measurement flag
} tfmini_data_t;

/**
 * @brief Initialize TFMini UART
 *
 * @param uart_num UART port number
 * @param tx_pin TX GPIO pin
 * @param rx_pin RX GPIO pin
 * @param buf_size UART buffer size
 * @return ESP error code
 */
esp_err_t tfmini_init(uart_port_t uart_num, int tx_pin, int rx_pin, int buf_size);

/**
 * @brief Read and parse TFMini data
 *
 * @param uart_num UART port number
 * @param data Pointer to data structure to fill
 * @param max_valid_distance_cm Maximum valid distance in cm
 */
void tfmini_read(uart_port_t uart_num, volatile tfmini_data_t* data, uint16_t max_valid_distance_cm);

#ifdef __cplusplus
}
#endif

#endif // TFMINI_H
