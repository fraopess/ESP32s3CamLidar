/*
 * MTF-02 Combined Distance + Optical Flow Sensor Driver
 * Micolink Protocol
 * UART Communication
 */

#ifndef MTF02_H
#define MTF02_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// MTF-02 Configuration
#define MTF02_BAUDRATE           115200  // Default baudrate, adjust if needed
#define MTF02_FLOW_QUALITY_MIN   30      // Minimum optical flow quality threshold (0-255)

// Micolink Protocol Constants (from MTF-01/02 documentation)
#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_DEV_ID              0x0F  // Expected device ID
#define MICOLINK_SYS_ID              0x00  // Expected system ID
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             (MICOLINK_MAX_PAYLOAD_LEN + 7)

// Message IDs
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51  // Range Sensor with optical flow

// Message Structure
typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    // Parser state
    uint8_t status;
    uint8_t payload_cnt;
} micolink_msg_t;

// Range Sensor Payload (packed structure from micolink protocol)
#pragma pack(1)
typedef struct {
    uint32_t time_ms;        // System time in ms
    uint32_t distance;       // Distance in mm (0 indicates unavailable)
    uint8_t  strength;       // Signal strength
    uint8_t  precision;      // Distance precision
    uint8_t  dis_status;     // Distance status
    uint8_t  reserved1;      // Reserved
    int16_t  flow_vel_x;     // Optical flow velocity X (cm/s @ 1m)
    int16_t  flow_vel_y;     // Optical flow velocity Y (cm/s @ 1m)
    uint8_t  flow_quality;   // Optical flow quality
    uint8_t  flow_status;    // Optical flow status
    uint16_t reserved2;      // Reserved
} micolink_payload_range_sensor_t;
#pragma pack()

// MTF-02 Data Structure (simplified for main.c compatibility)
typedef struct {
    uint16_t distance;       // Distance in cm
    uint16_t strength;       // Signal strength
    bool valid;              // Valid measurement flag

    // Optical flow data
    int16_t flow_vel_x;      // Optical flow velocity X (cm/s @ 1m)
    int16_t flow_vel_y;      // Optical flow velocity Y (cm/s @ 1m)
    uint8_t flow_quality;    // Flow quality indicator
    bool flow_valid;         // Flow data valid flag
} mtf02_data_t;

/**
 * @brief Initialize MTF-02 UART
 *
 * @param uart_num UART port number
 * @param tx_pin TX GPIO pin
 * @param rx_pin RX GPIO pin
 * @param buf_size UART buffer size
 * @return ESP error code
 */
esp_err_t mtf02_init(uart_port_t uart_num, int tx_pin, int rx_pin, int buf_size);

/**
 * @brief Send start command to MTF-02 (if needed)
 *
 * @param uart_num UART port number
 */
void mtf02_start_measurement(uart_port_t uart_num);

/**
 * @brief Read and parse MTF-02 data
 *
 * @param uart_num UART port number
 * @param data Pointer to data structure to fill
 * @param max_valid_distance_cm Maximum valid distance in cm
 */
void mtf02_read(uart_port_t uart_num, volatile mtf02_data_t* data, uint16_t max_valid_distance_cm);

/**
 * @brief Get the latest optical flow velocities from MTF-02
 *
 * @param data Pointer to MTF-02 data structure
 * @param velocity_x Pointer to store velocity X (m/s)
 * @param velocity_y Pointer to store velocity Y (m/s)
 * @param height_m Height above ground in meters (for velocity calculation)
 * @return true if flow data is valid, false otherwise
 */
bool mtf02_get_flow_velocity(const volatile mtf02_data_t* data, float* velocity_x, float* velocity_y, float height_m);

#ifdef __cplusplus
}
#endif

#endif // MTF02_H
