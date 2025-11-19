/*
 * PMW3901 Optical Flow Sensor Driver for ESP32
 */

#include "pmw3901.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char* TAG = "PMW3901";

// ============================================================================
// Structure interne du handle
// ============================================================================
struct pmw3901_handle_s {
    spi_device_handle_t spi_device;
    int pin_cs;
};

// ============================================================================
// Délais requis pour la communication SPI
// ============================================================================
#define PMW3901_DELAY_WRITE_US   50
#define PMW3901_DELAY_READ_US    100
#define PMW3901_DELAY_RESET_US   5000

// ============================================================================
// Écriture d'un registre
// ============================================================================
static esp_err_t pmw3901_write_reg(pmw3901_handle_t handle, uint8_t reg, uint8_t value)
{
    esp_err_t ret;
    spi_transaction_t trans;
    uint8_t tx_data[2];

    // Mettre le bit 7 à 1 pour l'écriture
    tx_data[0] = reg | 0x80;
    tx_data[1] = value;

    memset(&trans, 0, sizeof(trans));
    trans.length = 16;  // 2 octets
    trans.tx_buffer = tx_data;

    esp_rom_delay_us(PMW3901_DELAY_WRITE_US);
    ret = spi_device_transmit(handle->spi_device, &trans);
    esp_rom_delay_us(PMW3901_DELAY_WRITE_US);

    return ret;
}

// ============================================================================
// Lecture d'un registre
// ============================================================================
static esp_err_t pmw3901_read_reg(pmw3901_handle_t handle, uint8_t reg, uint8_t* value)
{
    esp_err_t ret;
    spi_transaction_t trans;
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    // Bit 7 à 0 pour la lecture
    tx_data[0] = reg & 0x7F;
    tx_data[1] = 0x00;

    memset(&trans, 0, sizeof(trans));
    trans.length = 16;  // 2 octets
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    esp_rom_delay_us(PMW3901_DELAY_WRITE_US);
    ret = spi_device_transmit(handle->spi_device, &trans);
    esp_rom_delay_us(PMW3901_DELAY_READ_US);

    if (ret == ESP_OK) {
        *value = rx_data[1];
    }

    return ret;
}

// ============================================================================
// Séquence d'initialisation des registres (optimisation performance)
// ============================================================================
static esp_err_t pmw3901_init_registers(pmw3901_handle_t handle)
{
    esp_err_t ret;

    // Séquence d'initialisation basée sur le driver Bitcraze
    // Cette séquence configure le capteur pour des performances optimales

    struct {
        uint8_t reg;
        uint8_t val;
    } init_seq[] = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
        {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
        {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
        {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
        {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
        {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
        {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
        {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11},
        {0x55, 0x80}, {0x74, 0x21}, {0x75, 0x1F}, {0x4A, 0x78},
        {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
        {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08},
        {0x63, 0x70}, {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07},
        {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F},
        {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
        {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}
    };

    for (int i = 0; i < sizeof(init_seq) / sizeof(init_seq[0]); i++) {
        ret = pmw3901_write_reg(handle, init_seq[i].reg, init_seq[i].val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write register 0x%02X", init_seq[i].reg);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Retour à la page 0
    ret = pmw3901_write_reg(handle, 0x7F, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Performance registers configured");
    return ESP_OK;
}

// ============================================================================
// Initialisation du capteur
// ============================================================================
esp_err_t pmw3901_init(const pmw3901_config_t* config, pmw3901_handle_t* handle)
{
    esp_err_t ret;

    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Allouer le handle
    *handle = (pmw3901_handle_t)malloc(sizeof(struct pmw3901_handle_s));
    if (!*handle) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return ESP_ERR_NO_MEM;
    }

    (*handle)->pin_cs = config->pin_cs;

    // Configuration du bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    ret = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE signifie que le bus est déjà initialisé (OK)
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    // Configuration du device SPI
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,  // 4 MHz
        .mode = 3,                           // SPI Mode 3 (CPOL=1, CPHA=1)
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL,
    };

    ret = spi_bus_add_device(config->spi_host, &devcfg, &(*handle)->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %d", ret);
        spi_bus_free(config->spi_host);
        free(*handle);
        *handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "SPI initialized: CS=%d, SCK=%d, MOSI=%d, MISO=%d",
             config->pin_cs, config->pin_sck, config->pin_mosi, config->pin_miso);

    // Reset du capteur
    vTaskDelay(pdMS_TO_TICKS(50));

    ret = pmw3901_write_reg(*handle, PMW3901_REG_POWER_UP_RESET, 0x5A);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset sensor");
        pmw3901_deinit(*handle);
        *handle = NULL;
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Vérifier l'ID du produit
    uint8_t product_id = 0;
    ret = pmw3901_read_reg(*handle, PMW3901_REG_PRODUCT_ID, &product_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read product ID");
        pmw3901_deinit(*handle);
        *handle = NULL;
        return ret;
    }

    if (product_id != PMW3901_PRODUCT_ID) {
        ESP_LOGE(TAG, "Invalid product ID: 0x%02X (expected 0x%02X)",
                 product_id, PMW3901_PRODUCT_ID);
        pmw3901_deinit(*handle);
        *handle = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "PMW3901 detected, Product ID: 0x%02X", product_id);

    // Vérifier l'inverse product ID
    uint8_t inv_id = 0;
    ret = pmw3901_read_reg(*handle, PMW3901_REG_INVERSE_PRODUCT_ID, &inv_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Inverse Product ID: 0x%02X", inv_id);
    }

    // Lecture initiale des registres de mouvement (requis)
    uint8_t dummy;
    pmw3901_read_reg(*handle, PMW3901_REG_MOTION, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_X_L, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_X_H, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_Y_L, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_Y_H, &dummy);

    // Initialiser les registres de performance
    ret = pmw3901_init_registers(*handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize performance registers");
        pmw3901_deinit(*handle);
        *handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "PMW3901 initialized successfully");
    return ESP_OK;
}

// ============================================================================
// Lecture des données de mouvement
// ============================================================================
esp_err_t pmw3901_read_motion(pmw3901_handle_t handle, pmw3901_motion_t* motion)
{
    if (!handle || !motion) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t motion_reg, dx_l, dx_h, dy_l, dy_h, squal;

    // Lire le registre de mouvement
    ret = pmw3901_read_reg(handle, PMW3901_REG_MOTION, &motion_reg);
    if (ret != ESP_OK) {
        return ret;
    }

    motion->motion = (motion_reg & 0x80) != 0;

    // Lire Delta X (16-bit signé)
    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_X_L, &dx_l);
    if (ret != ESP_OK) return ret;

    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_X_H, &dx_h);
    if (ret != ESP_OK) return ret;

    motion->delta_x = (int16_t)((dx_h << 8) | dx_l);

    // Lire Delta Y (16-bit signé)
    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_Y_L, &dy_l);
    if (ret != ESP_OK) return ret;

    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_Y_H, &dy_h);
    if (ret != ESP_OK) return ret;

    motion->delta_y = (int16_t)((dy_h << 8) | dy_l);

    // Lire la qualité du signal
    ret = pmw3901_read_reg(handle, PMW3901_REG_SQUAL, &squal);
    if (ret != ESP_OK) return ret;

    motion->squal = squal;

    return ESP_OK;
}

// ============================================================================
// Contrôle du LED
// ============================================================================
esp_err_t pmw3901_set_led(pmw3901_handle_t handle, bool led_on)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Aller à la page 0x14
    ret = pmw3901_write_reg(handle, 0x7F, 0x14);
    if (ret != ESP_OK) return ret;

    // Activer/désactiver le LED
    ret = pmw3901_write_reg(handle, 0x6F, led_on ? 0x1C : 0x00);
    if (ret != ESP_OK) return ret;

    // Retour à la page 0
    ret = pmw3901_write_reg(handle, 0x7F, 0x00);

    return ret;
}

// ============================================================================
// Lecture de l'ID produit
// ============================================================================
esp_err_t pmw3901_read_id(pmw3901_handle_t handle, uint8_t* product_id)
{
    if (!handle || !product_id) {
        return ESP_ERR_INVALID_ARG;
    }

    return pmw3901_read_reg(handle, PMW3901_REG_PRODUCT_ID, product_id);
}

// ============================================================================
// Libération des ressources
// ============================================================================
esp_err_t pmw3901_deinit(pmw3901_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_remove_device(handle->spi_device);
    free(handle);

    ESP_LOGI(TAG, "PMW3901 deinitialized");
    return ESP_OK;
}
