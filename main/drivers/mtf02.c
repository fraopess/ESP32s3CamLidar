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
// ============================================================================
static bool micolink_check_sum(micolink_msg_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for (uint8_t i = 0; i < length; i++) {
        checksum += temp[i];
    }

    if (checksum == msg->checksum)
        return true;
    else
        return false;
}

// ============================================================================
// Parse single character using Micolink protocol state machine
// ============================================================================
static bool micolink_parse_char(micolink_msg_t* msg, uint8_t data)
{
    switch (msg->status)
    {
    case 0:
        if (data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;

    case 1:  // Device ID
        msg->dev_id = data;
        msg->status++;
        break;

    case 2:  // System ID
        msg->sys_id = data;
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

    ESP_LOGI(TAG, "MTF-02 initialized on UART%d at %d baud", uart_num, MTF02_BAUDRATE);

    return ret;
}

// ============================================================================
// Read and parse MTF-02 data using Micolink protocol
// ============================================================================
void mtf02_read(uart_port_t uart_num, volatile mtf02_data_t* data, uint16_t max_valid_distance_cm)
{
    static micolink_msg_t msg = {0};
    static int packets_parsed = 0;
    static int64_t last_debug_time = 0;
    uint8_t byte;

    int count = 0;
    while (uart_read_bytes(uart_num, &byte, 1, 0) > 0 && count++ < 100) {

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

                // Check if flow data is valid (status and quality checks)
                data->flow_valid = (payload.flow_status == 0) && (payload.flow_quality > 0);

                packets_parsed++;
                break;
            }

            default:
                break;
        }
    }

    // Statistics output every 1 second
    int64_t current_time = esp_timer_get_time();
    if (current_time - last_debug_time >= 1000000) {
        if (packets_parsed > 0) {
            ESP_LOGI(TAG, "Packets/sec: %d, Distance: %d cm, Flow: (%d, %d), Quality: %d",
                     packets_parsed, data->distance, data->flow_vel_x, data->flow_vel_y, data->flow_quality);
        }
        packets_parsed = 0;
        last_debug_time = current_time;
    }
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
