/*
 * MTF-02 Combined Distance + Optical Flow Sensor Driver Implementation
 * Micolink Protocol
 */

#include "mtf02.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MTF02";

// ============================================================================
// Checksum validation for Micolink protocol
// Per documentation: checksum = sum of all previous data (HEAD to PAYLOAD)
// ============================================================================
static bool micolink_check_sum(micolink_msg_t* msg)
{
    uint8_t checksum = 0;

    // Sum: HEAD + DEV_ID + SYS_ID + MSG_ID + SEQ + LEN
    checksum += msg->head;
    checksum += msg->dev_id;
    checksum += msg->sys_id;
    checksum += msg->msg_id;
    checksum += msg->seq;
    checksum += msg->len;

    // Sum all payload bytes
    for (uint8_t i = 0; i < msg->len; i++) {
        checksum += msg->payload[i];
    }

    if (checksum == msg->checksum) {
        return true;
    } else {
        // static int debug_count = 0;
        // if (debug_count < 3) {
        //     debug_count++;
        //     ESP_LOGI(TAG, "Checksum mismatch: calculated=0x%02X, received=0x%02X", checksum, msg->checksum);
        //     ESP_LOGI(TAG, "  HEAD=0x%02X, DEV=0x%02X, SYS=0x%02X, MSG=0x%02X, SEQ=0x%02X, LEN=0x%02X",
        //              msg->head, msg->dev_id, msg->sys_id, msg->msg_id, msg->seq, msg->len);
        // }
        return false;
    }
}

// ============================================================================
// Parse single character using Micolink protocol state machine
// ============================================================================
static bool micolink_parse_char(micolink_msg_t* msg, uint8_t data)
{
    static int header_attempts = 0;
    static bool logged_header_search = false;

    switch (msg->status)
    {
    case 0:
        if (data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
            // if (!logged_header_search) {
            //     logged_header_search = true;
            //     ESP_LOGI(TAG, "Found header 0xEF after %d attempts", header_attempts);
            // }
            header_attempts = 0;
        } else {
            header_attempts++;
            // if (header_attempts == 1) {
            //     ESP_LOGI(TAG, "Searching for header 0xEF, received: 0x%02X", data);
            // }
            // if (header_attempts % 100 == 0) {
            //     ESP_LOGI(TAG, "Still searching for header, attempt %d, last byte: 0x%02X", header_attempts, data);
            // }
        }
        break;

    case 1:  // Device ID
        msg->dev_id = data;
        // Validate device ID (optional - can be disabled if sensor uses different ID)
        // if (data != MICOLINK_DEV_ID) {
        //     ESP_LOGW(TAG, "Unexpected DEV_ID: 0x%02X (expected 0x%02X)", data, MICOLINK_DEV_ID);
        // }
        msg->status++;
        break;

    case 2:  // System ID
        msg->sys_id = data;
        // Validate system ID (optional - can be disabled if sensor uses different ID)
        // if (data != MICOLINK_SYS_ID) {
        //     ESP_LOGW(TAG, "Unexpected SYS_ID: 0x%02X (expected 0x%02X)", data, MICOLINK_SYS_ID);
        // }
        msg->status++;
        break;

    case 3:  // Message ID
        msg->msg_id = data;
        msg->status++;
        break;

    case 4:  // Sequence
        msg->seq = data;
        msg->status++;
        break;

    case 5:  // Payload length
        msg->len = data;
        if (msg->len == 0)
            msg->status += 2;
        else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;

    case 6:  // Payload receive
        msg->payload[msg->payload_cnt++] = data;
        if (msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7:  // Checksum
        msg->checksum = data;
        msg->status = 0;
        if (micolink_check_sum(msg))
        {
            return true;
        } else {
            // static int checksum_failures = 0;
            // checksum_failures++;
            // if (checksum_failures <= 5 || checksum_failures % 50 == 0) {
            //     ESP_LOGI(TAG, "Checksum failed (count: %d), msg_id: 0x%02X, len: %d, checksum: 0x%02X",
            //              checksum_failures, msg->msg_id, msg->len, msg->checksum);
            // }
        }
        break;

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}

// ============================================================================
// Initialize MTF-02 UART
// ============================================================================
esp_err_t mtf02_init(uart_port_t uart_num, int tx_pin, int rx_pin, int buf_size)
{
    uart_config_t uart_config = {
        .baud_rate = MTF02_BAUDRATE,
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

    // ESP_LOGI(TAG, "MTF-02 initialized:");
    // ESP_LOGI(TAG, "  UART: %d", uart_num);
    // ESP_LOGI(TAG, "  Baud: %d", MTF02_BAUDRATE);
    // ESP_LOGI(TAG, "  TX Pin: %d", tx_pin);
    // ESP_LOGI(TAG, "  RX Pin: %d", rx_pin);
    // ESP_LOGI(TAG, "  Protocol: Micolink (header 0xEF)");

    return ret;
}

// ============================================================================
// Send start measurement command to MTF-02 (experimental)
// ============================================================================
void mtf02_start_measurement(uart_port_t uart_num)
{
    // Try sending a simple command to start streaming
    // This is experimental - MTF-02 might start automatically
    // ESP_LOGI(TAG, "Attempting to start MTF-02 measurement...");

    // Flush RX buffer first
    uart_flush_input(uart_num);

    // Some sensors need a newline or specific command
    // We'll try a few common patterns
    const char* commands[] = {
        "\r\n",           // Newline
        "START\r\n",      // Text command
        NULL
    };

    for (int i = 0; commands[i] != NULL; i++) {
        uart_write_bytes(uart_num, commands[i], strlen(commands[i]));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ESP_LOGI(TAG, "MTF-02 start commands sent");
}

// ============================================================================
// Read and parse MTF-02 data using Micolink protocol
// ============================================================================
void mtf02_read(uart_port_t uart_num, volatile mtf02_data_t* data, uint16_t max_valid_distance_cm)
{
    static micolink_msg_t msg = {0};
    static int packets_parsed = 0;
    static int bytes_received = 0;
    static int64_t last_debug_time = 0;
    static bool first_byte_received = false;
    static uint8_t hex_dump[32];
    static int hex_dump_count = 0;
    uint8_t byte;

    int count = 0;
    while (uart_read_bytes(uart_num, &byte, 1, 0) > 0 && count++ < 100) {
        bytes_received++;

        // if (!first_byte_received) {
        //     first_byte_received = true;
        //     ESP_LOGI(TAG, "First byte received: 0x%02X", byte);
        // }

        // // Collect first 64 bytes for hex dump (increased for better analysis)
        // if (hex_dump_count < 32) {
        //     hex_dump[hex_dump_count++] = byte;
        //     if (hex_dump_count == 32) {
        //         ESP_LOGI(TAG, "First 32 bytes received (hex dump):");
        //         for (int i = 0; i < 32; i++) {
        //             printf("0x%02X ", hex_dump[i]);
        //             if ((i + 1) % 16 == 0) printf("\n");
        //         }
        //         printf("\n");
        //     }
        // }

        if (micolink_parse_char(&msg, byte) == false)
            continue;

        // Complete message received
        switch (msg.msg_id)
        {
            case MICOLINK_MSG_ID_RANGE_SENSOR:
            {
                if (msg.len < sizeof(micolink_payload_range_sensor_t)) {
                    // Invalid payload size
                    break;
                }

                micolink_payload_range_sensor_t payload;
                memcpy(&payload, msg.payload, sizeof(micolink_payload_range_sensor_t));

                // Process distance data
                // payload.distance is in mm, convert to cm
                uint16_t dist_cm = payload.distance / 10;

                // Minimum effective distance is 10mm according to protocol
                if (payload.distance >= 10 && dist_cm <= max_valid_distance_cm) {
                    data->distance = dist_cm;
                    data->strength = payload.strength;
                    data->valid = true;
                } else {
                    data->valid = false;
                }

                // Process optical flow data
                // Flow velocity unit: cm/s @ 1m
                // Calculation formula: speed(cm/s) = optical flow velocity * height(m)
                data->flow_vel_x = payload.flow_vel_x;
                data->flow_vel_y = payload.flow_vel_y;
                data->flow_quality = payload.flow_quality;

                // Check if flow data is valid
                // Note: flow_status seems to be 1 when flow is active (not 0)
                // Use quality threshold instead
                data->flow_valid = (payload.flow_quality >= MTF02_FLOW_QUALITY_MIN);

                packets_parsed++;

                // // Debug: Log optical flow data periodically
                // static int flow_debug_count = 0;
                // flow_debug_count++;
                // if (flow_debug_count % 30 == 0) {
                //     ESP_LOGI(TAG, "=== MTF-02 Raw Data ===");
                //     ESP_LOGI(TAG, "Distance: %d mm, Strength: %d", payload.distance, payload.strength);
                //     ESP_LOGI(TAG, "Flow vel_x: %d, vel_y: %d (cm/s @ 1m)", payload.flow_vel_x, payload.flow_vel_y);
                //     ESP_LOGI(TAG, "Flow quality: %d, status: %d", payload.flow_quality, payload.flow_status);
                //     ESP_LOGI(TAG, "Flow valid: %d (quality>=%d: %d)",
                //              data->flow_valid,
                //              MTF02_FLOW_QUALITY_MIN,
                //              (payload.flow_quality >= MTF02_FLOW_QUALITY_MIN));
                // }
                break;
            }

            default:
                break;
        }
    }

    // Statistics output every 1 second (commented out for production)
    // int64_t current_time = esp_timer_get_time();
    // if (current_time - last_debug_time >= 1000000) {
    //     ESP_LOGI(TAG, "Stats: %d bytes/s, %d packets/s | Distance: %d cm, Flow: (%d, %d), Quality: %d, Valid: %d/%d",
    //              bytes_received, packets_parsed,
    //              data->distance, data->flow_vel_x, data->flow_vel_y,
    //              data->flow_quality, data->valid, data->flow_valid);
    //     packets_parsed = 0;
    //     bytes_received = 0;
    //     last_debug_time = current_time;
    // }
}

// ============================================================================
// Get optical flow velocity from MTF-02
// ============================================================================
bool mtf02_get_flow_velocity(const volatile mtf02_data_t* data, float* velocity_x, float* velocity_y, float height_m)
{
    if (!data->flow_valid || height_m <= 0.0f) {
        *velocity_x = 0.0f;
        *velocity_y = 0.0f;
        return false;
    }

    // MTF-02 protocol: optical flow velocity unit is cm/s @ 1m
    // Calculation formula: speed(cm/s) = optical flow velocity * height(m)
    // Convert to m/s by dividing by 100
    *velocity_x = (data->flow_vel_x * height_m) / 100.0f;  // Convert cm/s to m/s
    *velocity_y = (data->flow_vel_y * height_m) / 100.0f;  // Convert cm/s to m/s

    return true;
}
