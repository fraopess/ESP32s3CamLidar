/*
 * PMW3901 Optical Flow Sensor Driver for ESP32
 *
 * Interface SPI pour le capteur PMW3901
 * Retourne les déplacements en pixels (Delta X, Delta Y)
 */

#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "esp_err.h"

// ============================================================================
// Registres PMW3901
// ============================================================================
#define PMW3901_REG_PRODUCT_ID          0x00  // Devrait retourner 0x49
#define PMW3901_REG_REVISION_ID         0x01
#define PMW3901_REG_MOTION              0x02
#define PMW3901_REG_DELTA_X_L           0x03
#define PMW3901_REG_DELTA_X_H           0x04
#define PMW3901_REG_DELTA_Y_L           0x05
#define PMW3901_REG_DELTA_Y_H           0x06
#define PMW3901_REG_SQUAL               0x07
#define PMW3901_REG_RAWDATA_SUM         0x08
#define PMW3901_REG_MAXIMUM_RAWDATA     0x09
#define PMW3901_REG_MINIMUM_RAWDATA     0x0A
#define PMW3901_REG_SHUTTER_LOWER       0x0B
#define PMW3901_REG_SHUTTER_UPPER       0x0C
#define PMW3901_REG_OBSERVATION         0x15
#define PMW3901_REG_MOTION_BURST        0x16
#define PMW3901_REG_POWER_UP_RESET      0x3A
#define PMW3901_REG_SHUTDOWN            0x3B
#define PMW3901_REG_RAWDATA_GRAB        0x58
#define PMW3901_REG_RAWDATA_GRAB_STATUS 0x59
#define PMW3901_REG_INVERSE_PRODUCT_ID  0x5F  // Devrait retourner 0xB6

// Valeurs
#define PMW3901_PRODUCT_ID              0x49
#define PMW3901_INVERSE_PRODUCT_ID_VAL  0xB6

// ============================================================================
// Structure de configuration
// ============================================================================
typedef struct {
    spi_host_device_t spi_host;  // SPI2_HOST ou SPI3_HOST
    int pin_cs;                   // Pin Chip Select
    int pin_sck;                  // Pin SPI Clock
    int pin_mosi;                 // Pin SPI MOSI
    int pin_miso;                 // Pin SPI MISO
} pmw3901_config_t;

// ============================================================================
// Structure de données du capteur
// ============================================================================
typedef struct {
    int16_t delta_x;     // Déplacement X en pixels
    int16_t delta_y;     // Déplacement Y en pixels
    uint8_t squal;       // Qualité du signal (0-255)
    bool motion;         // Mouvement détecté
} pmw3901_motion_t;

// ============================================================================
// Handle du capteur (opaque)
// ============================================================================
typedef struct pmw3901_handle_s* pmw3901_handle_t;

// ============================================================================
// API publique
// ============================================================================

/**
 * @brief Initialise le capteur PMW3901
 *
 * @param config Configuration des pins SPI
 * @param handle Pointeur vers le handle (sera alloué)
 * @return ESP_OK en cas de succès
 */
esp_err_t pmw3901_init(const pmw3901_config_t* config, pmw3901_handle_t* handle);

/**
 * @brief Lit les données de mouvement
 *
 * @param handle Handle du capteur
 * @param motion Structure pour recevoir les données
 * @return ESP_OK en cas de succès
 */
esp_err_t pmw3901_read_motion(pmw3901_handle_t handle, pmw3901_motion_t* motion);

/**
 * @brief Active/désactive le LED du capteur
 *
 * @param handle Handle du capteur
 * @param led_on true pour activer, false pour désactiver
 * @return ESP_OK en cas de succès
 */
esp_err_t pmw3901_set_led(pmw3901_handle_t handle, bool led_on);

/**
 * @brief Lit l'ID produit pour vérifier la communication
 *
 * @param handle Handle du capteur
 * @param product_id Pointeur pour stocker l'ID
 * @return ESP_OK en cas de succès
 */
esp_err_t pmw3901_read_id(pmw3901_handle_t handle, uint8_t* product_id);

/**
 * @brief Libère les ressources
 *
 * @param handle Handle du capteur
 * @return ESP_OK en cas de succès
 */
esp_err_t pmw3901_deinit(pmw3901_handle_t handle);

#endif // PMW3901_H
