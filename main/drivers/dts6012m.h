/*
 * DTS6012M ToF LiDAR Driver
 * UART Communication at 921600 baud
 * A5 Protocol with CRC16
 */

#ifndef DTS6012M_H
#define DTS6012M_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// DTS6012M Configuration
#define DTS6012M_BAUDRATE    921600

// DTS6012M Data Structure
typedef struct {
    uint16_t distance;    // Distance in cm
    uint16_t strength;    // Signal strength
    bool valid;           // Valid measurement flag
} dts6012m_data_t;

/**
 * @brief Initialize DTS6012M UART
 *
 * @param uart_num UART port number
 * @param tx_pin TX GPIO pin
 * @param rx_pin RX GPIO pin
 * @param buf_size UART buffer size
 * @return ESP error code
 */
esp_err_t dts6012m_init(uart_port_t uart_num, int tx_pin, int rx_pin, int buf_size);

/**
 * @brief Send start measurement command to DTS6012M
 *
 * @param uart_num UART port number
 */
void dts6012m_start_measurement(uart_port_t uart_num);

/**
 * @brief Read and parse DTS6012M data
 *
 * @param uart_num UART port number
 * @param data Pointer to data structure to fill
 * @param max_valid_distance_cm Maximum valid distance in cm
 */
void dts6012m_read(uart_port_t uart_num, volatile dts6012m_data_t* data, uint16_t max_valid_distance_cm);

#ifdef __cplusplus
}
#endif

#endif // DTS6012M_H
