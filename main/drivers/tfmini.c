/*
 * TFMini LiDAR Driver Implementation
 */

#include "tfmini.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "TFMINI";

// ============================================================================
// Initialize TFMini UART
// ============================================================================
esp_err_t tfmini_init(uart_port_t uart_num, int tx_pin, int rx_pin, int buf_size)
{
    uart_config_t uart_config = {
        .baud_rate = TFMINI_BAUDRATE,
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
// Read TFMini data
// ============================================================================
void tfmini_read(uart_port_t uart_num, volatile tfmini_data_t* data, uint16_t max_valid_distance_cm)
{
    static uint8_t rx_buffer[9];
    static int rx_index = 0;
    uint8_t byte;

    int count = 0;
    while (uart_read_bytes(uart_num, &byte, 1, 0) > 0 && count++ < 20) {
        rx_buffer[rx_index] = byte;

        if (rx_index == 0 && rx_buffer[0] != 0x59) continue;
        if (rx_index == 1 && rx_buffer[1] != 0x59) {
            rx_index = 0;
            continue;
        }

        rx_index++;

        if (rx_index == 9) {
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) {
                checksum += rx_buffer[i];
            }

            if (rx_buffer[8] == (checksum & 0xFF)) {
                uint16_t dist = rx_buffer[2] | (rx_buffer[3] << 8);
                uint16_t strength = rx_buffer[4] | (rx_buffer[5] << 8);

                if (dist > 0 && dist <= max_valid_distance_cm) {
                    data->distance = dist;
                    data->strength = strength;
                    data->valid = true;
                } else {
                    data->valid = false;
                }
            }

            rx_index = 0;
            break;
        }
    }
}
