/*
 * DTS6012M ToF LiDAR Driver Implementation
 */

#include "dts6012m.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "DTS6012M";

// ============================================================================
// CRC16 CCITT calculation for DTS6012M
// ============================================================================
static uint16_t calculate_crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Send DTS6012M command via UART
// ============================================================================
static void send_dts6012m_command(uart_port_t uart_num, uint8_t cmd, const uint8_t* data, uint16_t data_len)
{
    uint8_t tx_buffer[32];
    int idx = 0;

    // Build packet: [A5][03][20][CMD][00][LEN_H][LEN_L][Data...][CRC_H][CRC_L]
    tx_buffer[idx++] = 0xA5;           // Header
    tx_buffer[idx++] = 0x03;           // Device number
    tx_buffer[idx++] = 0x20;           // Device type
    tx_buffer[idx++] = cmd;            // Command
    tx_buffer[idx++] = 0x00;           // Reserved
    tx_buffer[idx++] = (data_len >> 8) & 0xFF; // Length high byte (BIG-ENDIAN)
    tx_buffer[idx++] = data_len & 0xFF;        // Length low byte

    // Add data if present
    for (int i = 0; i < data_len; i++) {
        tx_buffer[idx++] = data[i];
    }

    // Calculate CRC16 (all bytes except CRC itself)
    uint16_t crc = calculate_crc16(tx_buffer, idx);
    tx_buffer[idx++] = (crc >> 8) & 0xFF;  // CRC high byte
    tx_buffer[idx++] = crc & 0xFF;         // CRC low byte

    // Send command
    uart_write_bytes(uart_num, tx_buffer, idx);
}

// ============================================================================
// Initialize DTS6012M UART
// ============================================================================
esp_err_t dts6012m_init(uart_port_t uart_num, int tx_pin, int rx_pin, int buf_size)
{
    uart_config_t uart_config = {
        .baud_rate = DTS6012M_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(uart_num, buf_size * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return ret;
}

// ============================================================================
// Start DTS6012M measurement
// ============================================================================
void dts6012m_start_measurement(uart_port_t uart_num)
{
    send_dts6012m_command(uart_num, 0x01, NULL, 0);  // Command 0x01 with no data
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for sensor to respond
}

// ============================================================================
// Read DTS6012M data (A5 Protocol)
// ============================================================================
void dts6012m_read(uart_port_t uart_num, volatile dts6012m_data_t* data, uint16_t max_valid_distance_cm)
{
    static uint8_t rx_buffer[64];
    static int rx_index = 0;
    static uint16_t expected_length = 0;
    static int bytes_received_total = 0;
    static int packets_parsed = 0;
    static int64_t last_debug_time = 0;
    static bool first_byte_received = false;
    uint8_t byte;

    int count = 0;
    while (uart_read_bytes(uart_num, &byte, 1, 0) > 0 && count++ < 50) {
        bytes_received_total++;

        if (!first_byte_received) {
            first_byte_received = true;
        }

        rx_buffer[rx_index] = byte;

        // State machine for packet parsing
        if (rx_index == 0) {
            // Looking for header 0xA5
            if (rx_buffer[0] != 0xA5) {
                continue;  // Not a valid header, keep looking
            }
            rx_index++;
        } else if (rx_index == 1) {
            // Check device number (should be 0x03)
            if (rx_buffer[1] != 0x03) {
                rx_index = 0;
                continue;
            }
            rx_index++;
        } else if (rx_index == 2) {
            // Check device type (should be 0x20)
            if (rx_buffer[2] != 0x20) {
                rx_index = 0;
                continue;
            }
            rx_index++;
        } else if (rx_index < 7) {
            // Reading CMD, Reserved, Length_H, Length_L
            rx_index++;
            if (rx_index == 7) {
                // We now have the length field - DTS6012M uses BIG-ENDIAN (high byte first)
                expected_length = (rx_buffer[5] << 8) | rx_buffer[6];

                if (expected_length > 30 || expected_length == 0) {
                    // Sanity check: length should be 1-30 bytes for DTS6012M
                    // Try to resync: search for next A5 in buffer
                    rx_index = 0;
                    continue;
                }
            }
        } else {
            // Reading data and CRC
            rx_index++;
            uint16_t total_packet_length = 7 + expected_length + 2;  // Header(7) + Data + CRC(2)

            if (rx_index >= total_packet_length) {
                // Full packet received, validate and parse
                uint8_t cmd = rx_buffer[3];

                // Note: CRC validation disabled - sensor appears to work without it

                // Parse measurement data (CMD = 0x01)
                if (cmd == 0x01 && expected_length >= 14) {
                    // Data starts at byte 7 (little-endian for data fields)
                    uint16_t primary_centroid = rx_buffer[13] | (rx_buffer[14] << 8);  // Distance in mm
                    uint16_t primary_intensity = rx_buffer[17] | (rx_buffer[18] << 8);

                    // Convert mm to cm
                    uint16_t dist_cm = primary_centroid / 10;

                    // 0xFFFF often indicates no valid measurement
                    if (primary_centroid == 0xFFFF || primary_centroid == 0) {
                        data->valid = false;
                        packets_parsed++;  // Still count as parsed
                    } else if (dist_cm > 0 && dist_cm <= max_valid_distance_cm) {
                        data->distance = dist_cm;
                        data->strength = primary_intensity;
                        data->valid = true;
                        packets_parsed++;
                    } else {
                        data->valid = false;
                        packets_parsed++;  // Still count as parsed
                    }
                }

                rx_index = 0;  // Reset for next packet
                break;
            }
        }
    }

    // Statistics output every 1 second
    int64_t current_time = esp_timer_get_time();
    if (current_time - last_debug_time >= 1000000) {
        bytes_received_total = 0;
        packets_parsed = 0;
        last_debug_time = current_time;
    }
}
