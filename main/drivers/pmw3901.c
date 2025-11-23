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
// Délais requis pour la communication SPI (basé sur Bitcraze driver)
// ============================================================================
#define PMW3901_DELAY_BEFORE_US     50
#define PMW3901_DELAY_AFTER_US      50
#define PMW3901_DELAY_WRITE_POST_US 200
#define PMW3901_DELAY_READ_PRE_US   100
#define PMW3901_DELAY_RESET_US      5000

// ============================================================================
// Direct measurement approach - minimal filtering
//
// Driver provides near-raw pixel displacement data:
// 1. SQUAL threshold (5) - basic quality check
// 2. Dead zone DISABLED (0) - detects all movements
// 3. Outlier clamping - prevents unrealistic spikes (max 100 pixels/sample)
//
// No median filtering, no exponential smoothing - direct pixel output
// Velocity calculation done in main.c with simple formula
//
// Hardware tips to reduce noise:
// - Maintain 80-300mm distance to textured surface
// - Ensure good lighting (LED should be on)
// - Mount sensor rigidly to avoid vibration
// - Use matte textured surface (not glossy/uniform)
// ============================================================================
#define PMW3901_SQUAL_THRESHOLD  5     // Minimum signal quality (0-255) - lowered for small movement detection
#define PMW3901_NOISE_DEADZONE   0     // Dead zone disabled - no attenuation
#define PMW3901_MAX_VELOCITY     100   // Max velocity clamp (pixels/sample)
#define PMW3901_FILTER_ALPHA     1.0f  // No driver-level smoothing (alpha=1.0 = raw data passthrough)

// ============================================================================
// Filtrage multi-étapes des données de mouvement
// ============================================================================
static void pmw3901_filter_motion(int16_t raw_dx, int16_t raw_dy, uint8_t squal,
                                   int16_t* filtered_dx, int16_t* filtered_dy,
                                   bool is_burst_mode)
{
    // === STAGE 1: Quality-based filtering ===
    if (squal < PMW3901_SQUAL_THRESHOLD) {
        *filtered_dx = 0;
        *filtered_dy = 0;
        return;
    }

    // === STAGE 2: Soft dead zone (disabled when DEADZONE=0) ===
    // Skip dead zone processing entirely when DEADZONE is 0
    if (PMW3901_NOISE_DEADZONE > 0) {
        if (abs(raw_dx) < PMW3901_NOISE_DEADZONE) {
            raw_dx = 0;
        } else if (abs(raw_dx) < PMW3901_NOISE_DEADZONE * 2) {
            float scale = (abs(raw_dx) - PMW3901_NOISE_DEADZONE) / (float)PMW3901_NOISE_DEADZONE;
            raw_dx = (int16_t)(raw_dx * scale);
        }

        if (abs(raw_dy) < PMW3901_NOISE_DEADZONE) {
            raw_dy = 0;
        } else if (abs(raw_dy) < PMW3901_NOISE_DEADZONE * 2) {
            float scale = (abs(raw_dy) - PMW3901_NOISE_DEADZONE) / (float)PMW3901_NOISE_DEADZONE;
            raw_dy = (int16_t)(raw_dy * scale);
        }
    }

    // === STAGE 3: Outlier rejection ===
    if (abs(raw_dx) > PMW3901_MAX_VELOCITY) {
        raw_dx = (raw_dx > 0) ? PMW3901_MAX_VELOCITY : -PMW3901_MAX_VELOCITY;
    }
    if (abs(raw_dy) > PMW3901_MAX_VELOCITY) {
        raw_dy = (raw_dy > 0) ? PMW3901_MAX_VELOCITY : -PMW3901_MAX_VELOCITY;
    }

    // Direct output - no filtering
    *filtered_dx = raw_dx;
    *filtered_dy = raw_dy;
}

// ============================================================================
// Écriture d'un registre (Bitcraze timing)
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

    esp_rom_delay_us(PMW3901_DELAY_BEFORE_US);
    ret = spi_device_transmit(handle->spi_device, &trans);
    esp_rom_delay_us(PMW3901_DELAY_AFTER_US);
    esp_rom_delay_us(PMW3901_DELAY_WRITE_POST_US);  // Critical 200us post-write delay

    return ret;
}

// ============================================================================
// Lecture d'un registre (Bitcraze timing)
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

    esp_rom_delay_us(PMW3901_DELAY_BEFORE_US);
    ret = spi_device_transmit(handle->spi_device, &trans);
    esp_rom_delay_us(PMW3901_DELAY_AFTER_US);
    esp_rom_delay_us(PMW3901_DELAY_READ_PRE_US);  // Critical 100us delay before reading

    if (ret == ESP_OK) {
        *value = rx_data[1];
    }

    return ret;
}

// ============================================================================
// Séquence d'initialisation des registres (optimisation performance)
// Basée sur Bitcraze_PMW3901.cpp - COMPLETE initialization sequence
// ============================================================================
static esp_err_t pmw3901_init_registers(pmw3901_handle_t handle)
{
    esp_err_t ret;

    // PHASE 1: Initial performance optimization registers (Bitcraze lines 192-251)
    struct {
        uint8_t reg;
        uint8_t val;
    } init_seq_phase1[] = {
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


    for (int i = 0; i < sizeof(init_seq_phase1) / sizeof(init_seq_phase1[0]); i++) {
        ret = pmw3901_write_reg(handle, init_seq_phase1[i].reg, init_seq_phase1[i].val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 1: Failed to write register 0x%02X", init_seq_phase1[i].reg);
            return ret;
        }
    }

    // CRITICAL: 100ms delay between phase 1 and phase 2 (Bitcraze line 252)
    vTaskDelay(pdMS_TO_TICKS(100));

    // PHASE 2: Final configuration registers (Bitcraze lines 253-267)
    // This phase is CRITICAL for motion detection to work!
    struct {
        uint8_t reg;
        uint8_t val;
    } init_seq_phase2[] = {
        {0x32, 0x44},
        {0x7F, 0x07},
        {0x40, 0x40},
        {0x7F, 0x06},
        {0x62, 0xf0},
        {0x63, 0x00},
        {0x7F, 0x0D},
        {0x48, 0xC0},
        {0x6F, 0xd5},
        {0x7F, 0x00},
        {0x5B, 0xa0},
        {0x4E, 0xA8},
        {0x5A, 0x50},
        {0x40, 0x80}
    };


    for (int i = 0; i < sizeof(init_seq_phase2) / sizeof(init_seq_phase2[0]); i++) {
        ret = pmw3901_write_reg(handle, init_seq_phase2[i].reg, init_seq_phase2[i].val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Phase 2: Failed to write register 0x%02X", init_seq_phase2[i].reg);
            return ret;
        }
    }

    // Ensure we're on page 0
    ret = pmw3901_write_reg(handle, 0x7F, 0x00);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Complete Bitcraze initialization sequence applied successfully");
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
        .clock_speed_hz = 2 * 1000 * 1000,  // 2 MHz (reduced from 4MHz for stability)
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

    // Initial delay before reset
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

    if (ret != ESP_OK || product_id != PMW3901_PRODUCT_ID) {
        ESP_LOGE(TAG, "Failed to detect PMW3901 (Product ID: 0x%02X, expected: 0x%02X)",
                 product_id, PMW3901_PRODUCT_ID);
        pmw3901_deinit(*handle);
        *handle = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // Lecture initiale des registres de mouvement (requis par Bitcraze)
    uint8_t dummy;
    pmw3901_read_reg(*handle, PMW3901_REG_MOTION, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_X_L, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_X_H, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_Y_L, &dummy);
    pmw3901_read_reg(*handle, PMW3901_REG_DELTA_Y_H, &dummy);
    vTaskDelay(pdMS_TO_TICKS(1));  // Delay after dummy reads per Bitcraze

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

    // CRITICAL: Lire Delta X - HIGH byte FIRST, then LOW (comme Bitcraze)
    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_X_H, &dx_h);
    if (ret != ESP_OK) return ret;

    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_X_L, &dx_l);
    if (ret != ESP_OK) return ret;

    int16_t raw_dx = (int16_t)((dx_h << 8) | dx_l);

    // CRITICAL: Lire Delta Y - HIGH byte FIRST, then LOW (comme Bitcraze)
    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_Y_H, &dy_h);
    if (ret != ESP_OK) return ret;

    ret = pmw3901_read_reg(handle, PMW3901_REG_DELTA_Y_L, &dy_l);
    if (ret != ESP_OK) return ret;

    int16_t raw_dy = (int16_t)((dy_h << 8) | dy_l);

    // Lire la qualité du signal
    ret = pmw3901_read_reg(handle, PMW3901_REG_SQUAL, &squal);
    if (ret != ESP_OK) return ret;

    motion->squal = squal;

    // Apply multi-stage filtering
    pmw3901_filter_motion(raw_dx, raw_dy, squal,
                         &motion->delta_x, &motion->delta_y,
                         false);  // Regular mode

    return ESP_OK;
}

// ============================================================================
// Lecture des données de mouvement en mode BURST (plus rapide!)
// Lit tous les registres en une seule transaction SPI
// ============================================================================
esp_err_t pmw3901_read_motion_burst(pmw3901_handle_t handle, pmw3901_motion_t* motion)
{
    if (!handle || !motion) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    spi_transaction_t trans;
    uint8_t tx_data[13];  // 1 command byte + 12 data bytes
    uint8_t rx_data[13];

    tx_data[0] = PMW3901_REG_MOTION_BURST & 0x7F;  // Read command
    memset(&tx_data[1], 0, 12);

    memset(&trans, 0, sizeof(trans));
    trans.length = 13 * 8;  // 13 bytes
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    esp_rom_delay_us(PMW3901_DELAY_BEFORE_US);
    ret = spi_device_transmit(handle->spi_device, &trans);
    esp_rom_delay_us(PMW3901_DELAY_AFTER_US);
    esp_rom_delay_us(PMW3901_DELAY_READ_PRE_US);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse burst data (skip first byte which is command echo)
    uint8_t motion_reg = rx_data[1];
    // uint8_t obs = rx_data[2];  // Observation register (unused)
    int16_t raw_dx = (int16_t)((rx_data[4] << 8) | rx_data[3]);
    int16_t raw_dy = (int16_t)((rx_data[6] << 8) | rx_data[5]);
    uint8_t squal = rx_data[7];

    motion->motion = (motion_reg & 0x80) != 0;
    motion->squal = squal;

    // Apply multi-stage filtering
    pmw3901_filter_motion(raw_dx, raw_dy, squal,
                         &motion->delta_x, &motion->delta_y,
                         true);  // Burst mode

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

    // Bitcraze has 200ms delay at start
    vTaskDelay(pdMS_TO_TICKS(200));

    // Aller à la page 0x14
    ret = pmw3901_write_reg(handle, 0x7F, 0x14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select LED bank");
        return ret;
    }

    // Activer/désactiver le LED
    uint8_t led_value = led_on ? 0x1C : 0x00;
    ret = pmw3901_write_reg(handle, 0x6F, led_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LED register");
        return ret;
    }

    // Retour à la page 0
    ret = pmw3901_write_reg(handle, 0x7F, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to return to bank 0");
        return ret;
    }

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
// Helper: Calculate velocity from optical flow (optional enhancement)
// ============================================================================
void pmw3901_calculate_velocity(int16_t delta_x, int16_t delta_y,
                                float distance_m, float dt,
                                float* velocity_x, float* velocity_y)
{
    // Physical constants for PMW3901
    const float PIXEL_SIZE_MM = 0.024f;   // 24 micrometers per pixel
    const float FOCAL_LENGTH_PX = 1400.0f; // Approximate focal length in pixels

    if (distance_m <= 0.0f || dt <= 0.0f) {
        *velocity_x = 0.0f;
        *velocity_y = 0.0f;
        return;
    }

    // Convert pixel displacement to velocity (m/s)
    // Formula: velocity = (delta_pixels * distance * pixel_size) / (focal_length * dt)
    *velocity_x = (delta_x * distance_m * PIXEL_SIZE_MM * 1000.0f) / (FOCAL_LENGTH_PX * dt);
    *velocity_y = (delta_y * distance_m * PIXEL_SIZE_MM * 1000.0f) / (FOCAL_LENGTH_PX * dt);
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

