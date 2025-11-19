/*
 * ESP32-S3-CAM Optical Flow + LiDAR
 * Support caméras: OV2640, OV5640, OV7725 (sélectionnable)
 * Support lidars: TFMini, DTS6012M (sélectionnable)
 *
 * Pour choisir la caméra et le lidar, décommenter UNE seule ligne dans chaque section:
 */

// ============================================================================
// SÉLECTION DU MODE DE FLUX OPTIQUE (décommenter UNE SEULE ligne)
// ============================================================================
//#define USE_CAMERA        // Utiliser la caméra pour le flux optique
//#define USE_PMW3901       // Utiliser le capteur PMW3901 pour le flux optique

// ============================================================================
// SÉLECTION DE LA CAMÉRA (seulement si USE_CAMERA est activé)
// ============================================================================
// NOTE: Si vous avez un OV7725 qui retourne PID=0xFE, utilisez OV7725_CLONE
//#define USE_OV2640        // Décommenter pour OV2640 (caméra commune ESP32-CAM)
//#define USE_OV5640        // Décommenter pour OV5640
#define USE_OV7725        // Décommenter pour OV7725 original (PID=0x77)

// ============================================================================
// Vérification configuration mode flux optique
// ============================================================================
#if (defined(USE_CAMERA) + defined(USE_PMW3901)) > 1
    #error "ERREUR: Sélectionnez UN SEUL mode de flux optique (USE_CAMERA ou USE_PMW3901)!"
#endif

#if !defined(USE_CAMERA) && !defined(USE_PMW3901)
    #error "ERREUR: Sélectionnez un mode (USE_CAMERA ou USE_PMW3901)"
#endif

// ============================================================================
// Vérification configuration caméra (seulement si USE_CAMERA est activé)
// ============================================================================
#ifdef USE_CAMERA
    #if (defined(USE_OV2640) + defined(USE_OV5640) + defined(USE_OV7725) ) > 1
        #error "ERREUR: Sélectionnez UNE SEULE caméra!"
    #endif

    #if !defined(USE_OV2640) && !defined(USE_OV5640) && !defined(USE_OV7725)
        #error "ERREUR: Sélectionnez une caméra (décommentez une ligne)"
    #endif
#endif

// ============================================================================
// SÉLECTION DU LIDAR (décommenter UNE SEULE ligne)
// ============================================================================
//#define USE_TFMINI    // Décommenter pour TFMini (115200 baud)
#define USE_DTS6012M  // Décommenter pour DTS6012M (921600 baud, haute vitesse)

// ============================================================================
// Vérification configuration lidar
// ============================================================================
#if (defined(USE_TFMINI) + defined(USE_DTS6012M)) > 1
    #error "ERREUR: Sélectionnez UN SEUL lidar!"
#endif

#if !defined(USE_TFMINI) && !defined(USE_DTS6012M)
    #error "ERREUR: Sélectionnez un lidar (décommentez une ligne)"
#endif

// ============================================================================
// Includes
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"

#ifdef USE_CAMERA
#include "esp_camera.h"
#endif

#ifdef USE_PMW3901
#include "pmw3901.h"
#endif

static const char *TAG = "OPTICAL_FLOW";

#pragma GCC optimize ("O3")
#pragma GCC optimize ("unroll-loops")

// ============================================================================
// Configuration pins ESP32-S3-CAM (seulement si USE_CAMERA)
// ============================================================================
#ifdef USE_CAMERA
    #define CAM_PIN_PWDN    -1
    #define CAM_PIN_RESET   -1
    #define CAM_PIN_XCLK    15
    #define CAM_PIN_SIOD    4
    #define CAM_PIN_SIOC    5
    #define CAM_PIN_D7      16
    #define CAM_PIN_D6      17
    #define CAM_PIN_D5      18
    #define CAM_PIN_D4      12
    #define CAM_PIN_D3      10
    #define CAM_PIN_D2      8
    #define CAM_PIN_D1      9
    #define CAM_PIN_D0      11
    #define CAM_PIN_VSYNC   6
    #define CAM_PIN_HREF    7
    #define CAM_PIN_PCLK    13
#endif

// ============================================================================
// Configuration UART LiDAR
// ============================================================================
#define LIDAR_UART_NUM     UART_NUM_1
#define LIDAR_TX_PIN       47
#define LIDAR_RX_PIN       14
#define LIDAR_BUF_SIZE     512

#ifdef USE_TFMINI
    #define LIDAR_BAUDRATE     115200
    #define LIDAR_MODEL        "TFMini"
#endif

#ifdef USE_DTS6012M
    #define LIDAR_BAUDRATE     921600
    #define LIDAR_MODEL        "DTS6012M"
#endif

// ============================================================================
// Configuration SPI PMW3901 (Optical Flow Sensor)
// ============================================================================
#ifdef USE_PMW3901
    #define PMW3901_SPI_HOST   SPI2_HOST
    #define PMW3901_PIN_CS     1    // À adapter selon votre câblage
    #define PMW3901_PIN_SCK    2    // À adapter selon votre câblage
    #define PMW3901_PIN_MOSI   3    // À adapter selon votre câblage
    #define PMW3901_PIN_MISO   21   // À adapter selon votre câblage

    // Paramètres du capteur PMW3901
    #define PMW3901_FOCAL_LENGTH_PX    42.0f     // FOV de 42°, ~35x35 pixels effectifs
    #define PMW3901_PIXEL_SIZE_MM      0.030f    // Estimation basée sur FOV
    #define PMW3901_MAX_DELTA          127       // Valeur maximale de delta (8-bit signé)
    #define PMW3901_FLOW_SCALE         1.0f      // Facteur d'échelle pour le flux
#endif

// ============================================================================
// Configuration spécifique selon la caméra (seulement si USE_CAMERA est activé)
// ============================================================================
#ifdef USE_CAMERA

    #ifdef USE_OV2640
        // Configuration OV2640
        #define CAMERA_MODEL        "OV2640"
        #define CAMERA_PID          OV2640_PID
        #define IMG_WIDTH           160
        #define IMG_HEIGHT          120
        #define CAMERA_FRAME_SIZE   FRAMESIZE_QQVGA
        #define XCLK_FREQ           10000000    // 10MHz (réduit pour meilleure stabilité I2C)
        #define FOCAL_LENGTH_PX     50.0f       // À calibrer
        #define PIXEL_SIZE_MM       0.0028f     // 2.8µm
        #define FLOW_SMOOTHING_ALPHA      1.00f  // Filtre moyen (OV2640 plus bruité)
        #define MIN_GRADIENT_THRESHOLD    8.0f  // Seuil plus élevé
        #define MIN_VALID_PIXELS          150   // Pixels minimum
    #endif

    #ifdef USE_OV5640
        // Configuration OV5640
        #define CAMERA_MODEL        "OV5640"
        #define CAMERA_PID          OV5640_PID
        #define IMG_WIDTH           160
        #define IMG_HEIGHT          120
        #define CAMERA_FRAME_SIZE   FRAMESIZE_QQVGA
        #define XCLK_FREQ           20000000    // 20MHz (réduit pour meilleure stabilité I2C)
        #define FOCAL_LENGTH_PX     40.27f      // À calibrer
        #define PIXEL_SIZE_MM       0.0014f     // 1.4µm
        #define FLOW_SMOOTHING_ALPHA      1.00f // Filtre léger (OV5640 moins bruité)
        #define MIN_GRADIENT_THRESHOLD    5.0f  // Seuil plus bas (meilleur SNR)
        #define MIN_VALID_PIXELS          120   // Moins de pixels requis
    #endif

    #ifdef USE_OV7725
        // Configuration OV7725 (optimisée pour vitesse maximale)
        #define CAMERA_MODEL        "OV7725"
        #define CAMERA_PID          OV7725_PID
        #define IMG_WIDTH           160
        #define IMG_HEIGHT          120
        #define CAMERA_FRAME_SIZE   FRAMESIZE_QQVGA
        #define XCLK_FREQ           20000000    // 20MHz (requis pour OV7725)
        #define FOCAL_LENGTH_PX     45.0f       // À calibrer
        #define PIXEL_SIZE_MM       0.006f      // 6µm (pixels plus grands = moins de bruit)
        #define FLOW_SMOOTHING_ALPHA      1.00f // Pas de filtrage pour réactivité maximale
        #define MIN_GRADIENT_THRESHOLD    6.0f  // Seuil modéré
        #define MIN_VALID_PIXELS          100   // Moins de pixels pour vitesse
    #endif

#endif  // USE_CAMERA

// ============================================================================
// Configuration commune
// ============================================================================
#define MAX_VALID_DISTANCE_CM  1000
#define OPTICAL_FLOW_STEP 2      

// ============================================================================
// Structures
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t header1;
    uint8_t header2;
    uint32_t timestamp_ms;
    int16_t velocity_x;
    int16_t velocity_y;
    uint16_t distance;
    uint8_t checksum;
} serial_packet_t;

typedef struct {
    int64_t timestamp;
    float velocity_x;
    float velocity_y;
    uint16_t distance;
    bool lidar_valid;
    float fps;
} sensor_data_t;

// ============================================================================
// Variables globales
// ============================================================================
#ifdef USE_CAMERA
static uint8_t *img_prev = NULL;
static uint8_t *img_cur = NULL;
static bool first_frame = true;
#endif

#ifdef USE_PMW3901
static pmw3901_handle_t pmw3901_handle = NULL;
#endif

static volatile float global_flow_x = 0.0f;
static volatile float global_flow_y = 0.0f;
static volatile float filtered_flow_x = 0.0f;
static volatile float filtered_flow_y = 0.0f;

static volatile struct {
    uint16_t distance;
    uint16_t strength;
    bool valid;
} lidar_data = {0, 0, false};

static volatile uint32_t frame_count = 0;
static volatile int64_t last_stat_time = 0;
static volatile float current_fps = 0.0f;

static QueueHandle_t dataQueue;
static SemaphoreHandle_t frameMutex;

// ============================================================================
// Calcul checksum
// ============================================================================
static uint8_t calculate_checksum(const uint8_t* data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// ============================================================================
// Envoi paquet binaire
// ============================================================================
static void send_binary_packet(uint32_t timestamp_ms, float velocity_x, float velocity_y, uint16_t distance)
{
    serial_packet_t packet;
    
    packet.header1 = 0xAA;
    packet.header2 = 0x55;
    packet.timestamp_ms = timestamp_ms;
    packet.velocity_x = (int16_t)(velocity_x * 1000.0f);
    packet.velocity_y = (int16_t)(velocity_y * 1000.0f);
    packet.distance = distance;
    
    uint8_t* data_ptr = (uint8_t*)&packet.timestamp_ms;
    size_t data_len = sizeof(packet.timestamp_ms) + sizeof(packet.velocity_x) + 
                      sizeof(packet.velocity_y) + sizeof(packet.distance);
    packet.checksum = calculate_checksum(data_ptr, data_len);
    
    printf("%c%c", packet.header1, packet.header2);
    fwrite(&packet.timestamp_ms, 1, data_len, stdout);
    printf("%c", packet.checksum);
    fflush(stdout);
}

// ============================================================================
// Initialisation UART LiDAR
// ============================================================================
static void init_lidar_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = LIDAR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART_NUM, LIDAR_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "%s UART initialized at %d baud", LIDAR_MODEL, LIDAR_BAUDRATE);
}

// ============================================================================
// Lecture TFMini
// ============================================================================
#ifdef USE_TFMINI
static void read_tfmini(void)
{
    static uint8_t rx_buffer[9];
    static int rx_index = 0;
    uint8_t data;

    int count = 0;
    while (uart_read_bytes(LIDAR_UART_NUM, &data, 1, 0) > 0 && count++ < 20) {
        rx_buffer[rx_index] = data;

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

                if (dist > 0 && dist <= MAX_VALID_DISTANCE_CM) {
                    lidar_data.distance = dist;
                    lidar_data.strength = strength;
                    lidar_data.valid = true;
                } else {
                    lidar_data.valid = false;
                }
            }

            rx_index = 0;
            break;
        }
    }
}
#endif

// ============================================================================
// Lecture DTS6012M
// ============================================================================
#ifdef USE_DTS6012M
static void read_dts6012m(void)
{
    static uint8_t rx_buffer[15];
    static int rx_index = 0;
    uint8_t data;

    int count = 0;
    while (uart_read_bytes(LIDAR_UART_NUM, &data, 1, 0) > 0 && count++ < 50) {
        rx_buffer[rx_index] = data;

        // Recherche header 0xAA 0x55
        if (rx_index == 0 && rx_buffer[0] != 0xAA) continue;
        if (rx_index == 1 && rx_buffer[1] != 0x55) {
            rx_index = 0;
            continue;
        }

        rx_index++;

        // Paquet complet: 15 octets
        // [0xAA 0x55][dist_L dist_H][int_L int_H][dist2_L dist2_H][int2_L int2_H][temp_L temp_H][status][crc_L crc_H]
        if (rx_index == 15) {
            // CRC-16 CCITT validation (optionnel pour performance)
            // Pour vitesse maximale, on peut sauter la vérification CRC

            uint16_t dist_primary = rx_buffer[2] | (rx_buffer[3] << 8);
            uint16_t intensity_primary = rx_buffer[4] | (rx_buffer[5] << 8);
            uint8_t status = rx_buffer[12];

            // Le DTS6012M retourne la distance en millimètres, convertir en cm
            dist_primary /= 10;

            // Status byte: bit 0 = valid measurement
            if ((status & 0x01) && dist_primary > 0 && dist_primary <= MAX_VALID_DISTANCE_CM) {
                lidar_data.distance = dist_primary;
                lidar_data.strength = intensity_primary;
                lidar_data.valid = true;
            } else {
                lidar_data.valid = false;
            }

            rx_index = 0;
            break;
        }
    }
}
#endif

// ============================================================================
// Lecture LiDAR (appelle la fonction appropriée)
// ============================================================================
static inline void read_lidar(void)
{
#ifdef USE_TFMINI
    read_tfmini();
#endif
#ifdef USE_DTS6012M
    read_dts6012m();
#endif
}

// ============================================================================
// Flux optique Lucas-Kanade optimisé (seulement si USE_CAMERA est activé)
// ============================================================================
#ifdef USE_CAMERA
static void compute_optical_flow_optimized(uint8_t* prev, uint8_t* cur)
{
    float sum_Ix2 = 0.0f;
    float sum_Iy2 = 0.0f;
    float sum_IxIy = 0.0f;
    float sum_IxIt = 0.0f;
    float sum_IyIt = 0.0f;
    int valid_pixels = 0;
    
    const int width = IMG_WIDTH;
    const int height = IMG_HEIGHT;
    const int step = OPTICAL_FLOW_STEP;
    
    const int margin = 5;
    const int start_y = margin;
    const int end_y = height - margin;
    const int start_x = margin;
    const int end_x = width - margin;
    
    for (int y = start_y; y < end_y; y += step) {
        const int row_offset = y * width;
        const int row_offset_prev = (y - 1) * width;
        const int row_offset_next = (y + 1) * width;
        
        for (int x = start_x; x < end_x; x += step) {
            const int idx = row_offset + x;
            
            const int Ix_int = 
                -cur[idx - 1 - width] + cur[idx + 1 - width] +
                -(cur[idx - 1] << 1) + (cur[idx + 1] << 1) +
                -cur[idx - 1 + width] + cur[idx + 1 + width];
            const float Ix = (float)Ix_int * 0.125f;
            
            const int Iy_int =
                -cur[row_offset_prev + x - 1] - (cur[row_offset_prev + x] << 1) - cur[row_offset_prev + x + 1] +
                 cur[row_offset_next + x - 1] + (cur[row_offset_next + x] << 1) + cur[row_offset_next + x + 1];
            const float Iy = (float)Iy_int * 0.125f;
            
            const float It = (float)cur[idx] - (float)prev[idx];
            
            const float grad_mag_sq = Ix * Ix + Iy * Iy;
            if (grad_mag_sq > MIN_GRADIENT_THRESHOLD * MIN_GRADIENT_THRESHOLD) {
                sum_Ix2 += Ix * Ix;
                sum_Iy2 += Iy * Iy;
                sum_IxIy += Ix * Iy;
                sum_IxIt += Ix * It;
                sum_IyIt += Iy * It;
                valid_pixels++;
            }
        }
    }
    
    if (valid_pixels > MIN_VALID_PIXELS) {
        const float det = sum_Ix2 * sum_Iy2 - sum_IxIy * sum_IxIy;
        
        if (fabsf(det) > 1e-5f) {
            const float inv_det = 1.0f / det;
            
            float flow_x = -(sum_Iy2 * sum_IxIt - sum_IxIy * sum_IyIt) * inv_det;
            float flow_y = -(sum_Ix2 * sum_IyIt - sum_IxIy * sum_IxIt) * inv_det;
            
            flow_x = (flow_x > 50.0f) ? 50.0f : ((flow_x < -50.0f) ? -50.0f : flow_x);
            flow_y = (flow_y > 50.0f) ? 50.0f : ((flow_y < -50.0f) ? -50.0f : flow_y);
            
            filtered_flow_x = filtered_flow_x * (1.0f - FLOW_SMOOTHING_ALPHA) + flow_x * FLOW_SMOOTHING_ALPHA;
            filtered_flow_y = filtered_flow_y * (1.0f - FLOW_SMOOTHING_ALPHA) + flow_y * FLOW_SMOOTHING_ALPHA;
            
            global_flow_x = filtered_flow_x;
            global_flow_y = filtered_flow_y;
        } else {
            global_flow_x = 0.0f;
            global_flow_y = 0.0f;
        }
    } else {
        global_flow_x = 0.0f;
        global_flow_y = 0.0f;
    }
}
#endif  // USE_CAMERA

// ============================================================================
// Initialisation caméra (OV2640 ou OV5640)
// ============================================================================
#ifdef USE_CAMERA
static esp_err_t init_camera(void)
{
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        
        .xclk_freq_hz = XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = CAMERA_FRAME_SIZE,
        
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    
    ESP_LOGI(TAG, "Initializing camera...");
    ESP_LOGI(TAG, "Expected camera: %s (PID=0x%x)", CAMERA_MODEL, CAMERA_PID);
    ESP_LOGI(TAG, "XCLK frequency: %d MHz", XCLK_FREQ / 1000000);
    ESP_LOGI(TAG, "I2C pins: SDA=%d, SCL=%d", CAM_PIN_SIOD, CAM_PIN_SIOC);

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  1. Camera not connected properly");
        ESP_LOGE(TAG, "  2. Wrong camera model (try USE_OV2640 or USE_OV5640)");
        ESP_LOGE(TAG, "  3. Power supply issue");
        ESP_LOGE(TAG, "  4. I2C/SCCB communication problem");
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();

    // Afficher les informations du capteur détecté
    ESP_LOGI(TAG, "Camera detected - PID: 0x%x, VER: 0x%x, MIDL: 0x%x, MIDH: 0x%x",
             s->id.PID, s->id.VER, s->id.MIDL, s->id.MIDH);

    // Vérifier que le bon capteur est détecté
    if (s->id.PID != CAMERA_PID) {
        ESP_LOGW(TAG, "Camera PID mismatch! Expected 0x%x, got 0x%x", CAMERA_PID, s->id.PID);
        ESP_LOGW(TAG, "La caméra détectée ne correspond pas à %s", CAMERA_MODEL);
        ESP_LOGW(TAG, "Identifiez votre caméra:");
        ESP_LOGW(TAG, "  PID 0x26 = OV2640");
        ESP_LOGW(TAG, "  PID 0x5640 = OV5640");
        ESP_LOGW(TAG, "  PID 0x77 = OV7725 (original)");
        ESP_LOGW(TAG, "  PID 0x76 = OV7670");
        ESP_LOGW(TAG, "Changez le #define dans main.c pour correspondre à votre caméra");
    } else {
        ESP_LOGI(TAG, "%s confirmed!", CAMERA_MODEL);
    }

    // Configuration commune
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, 5);
    s->set_bpc(s, 0);
    s->set_wpc(s, 0);
    s->set_lenc(s, 0);

    // Configuration spécifique OV2640
    #ifdef USE_OV2640
    if (s->id.PID == OV2640_PID) {
        s->set_raw_gma(s, 0);
        s->set_dcw(s, 0);
        ESP_LOGI(TAG, "OV2640 configured");
    }
    #endif

    // Configuration spécifique OV5640
    #ifdef USE_OV5640
    if (s->id.PID == OV5640_PID) {
        ESP_LOGI(TAG, "OV5640 configured");
    }
    #endif

    // Configuration spécifique OV7725 (optimisée pour vitesse)
    #if defined(USE_OV7725) 
    if (s->id.PID == OV7725_PID ) {
        // OV7725 optimisations pour framerate maximal
        s->set_brightness(s, 0);      // Brightness neutre
        s->set_contrast(s, 0);        // Contrast neutre
        s->set_saturation(s, 0);      // Saturation neutre (N&B)
        s->set_special_effect(s, 0);  // Pas d'effets spéciaux
        s->set_colorbar(s, 0);        // Pas de colorbar

        // Désactiver tous les traitements pour vitesse maximale
        s->set_raw_gma(s, 0);         // Pas de gamma correction
        s->set_dcw(s, 0);             // Pas de downsize

        ESP_LOGI(TAG, "%s configured for maximum speed", CAMERA_MODEL);
    }
    #endif
    
    return ESP_OK;
}
#endif  // USE_CAMERA

// ============================================================================
// Initialisation PMW3901
// ============================================================================
#ifdef USE_PMW3901
static esp_err_t init_pmw3901(void)
{
    pmw3901_config_t config = {
        .spi_host = PMW3901_SPI_HOST,
        .pin_cs = PMW3901_PIN_CS,
        .pin_sck = PMW3901_PIN_SCK,
        .pin_mosi = PMW3901_PIN_MOSI,
        .pin_miso = PMW3901_PIN_MISO,
    };

    ESP_LOGI(TAG, "Initializing PMW3901 optical flow sensor...");

    esp_err_t ret = pmw3901_init(&config, &pmw3901_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMW3901 initialization failed: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "PMW3901 initialized successfully");

    // Activer le LED du capteur (optionnel)
    pmw3901_set_led(pmw3901_handle, true);

    return ESP_OK;
}
#endif  // USE_PMW3901

// ============================================================================
// CORE 0: Tâche caméra + flux optique (seulement si USE_CAMERA)
// ============================================================================
#ifdef USE_CAMERA
static void camera_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Camera task started on core %d", xPortGetCoreID());
    
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        if (fb->len != IMG_WIDTH * IMG_HEIGHT) {
            esp_camera_fb_return(fb);
            continue;
        }
        
        int64_t timestamp = esp_timer_get_time();
        
        if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            if (first_frame) {
                memcpy(img_prev, fb->buf, fb->len);
                first_frame = false;
            } else {
                memcpy(img_cur, fb->buf, fb->len);
                compute_optical_flow_optimized(img_prev, img_cur);
                memcpy(img_prev, img_cur, fb->len);
                
                frame_count++;
                int64_t current_time = esp_timer_get_time();
                if (current_time - last_stat_time >= 1000000) {
                    current_fps = (float)frame_count * 1000000.0f / (current_time - last_stat_time);
                    frame_count = 0;
                    last_stat_time = current_time;
                }
                
                float velocity_x = 0.0f;
                float velocity_y = 0.0f;
                
                if (lidar_data.valid && current_fps > 1.0f) {
                    float distance_m = lidar_data.distance / 100.0f;
                    float dt = 1.0f / current_fps;
                    
                    velocity_x = (global_flow_x * distance_m * PIXEL_SIZE_MM * 1000.0f) / 
                                 (FOCAL_LENGTH_PX * dt);
                    velocity_y = (global_flow_y * distance_m * PIXEL_SIZE_MM * 1000.0f) / 
                                 (FOCAL_LENGTH_PX * dt);
                }
                
                sensor_data_t data = {
                    .timestamp = timestamp,
                    .velocity_x = velocity_x,
                    .velocity_y = velocity_y,
                    .distance = lidar_data.distance,
                    .lidar_valid = lidar_data.valid,
                    .fps = current_fps
                };
                
                xQueueSend(dataQueue, &data, 0);
            }
            
            xSemaphoreGive(frameMutex);
        }
        
        esp_camera_fb_return(fb);
        taskYIELD();
    }
}
#endif  // USE_CAMERA

// ============================================================================
// CORE 0: Tâche PMW3901 + flux optique (seulement si USE_PMW3901)
// ============================================================================
#ifdef USE_PMW3901
static void pmw3901_task(void *pvParameters)
{
    ESP_LOGI(TAG, "PMW3901 task started on core %d", xPortGetCoreID());

    pmw3901_motion_t motion;

    while (1) {
        int64_t timestamp = esp_timer_get_time();

        // Lire les données du PMW3901
        esp_err_t ret = pmw3901_read_motion(pmw3901_handle, &motion);

        if (ret == ESP_OK) {
            // Les valeurs delta sont en pixels depuis la dernière lecture
            // Mise à jour du compteur de frames
            frame_count++;
            int64_t current_time = esp_timer_get_time();
            if (current_time - last_stat_time >= 1000000) {
                current_fps = (float)frame_count * 1000000.0f / (current_time - last_stat_time);
                frame_count = 0;
                last_stat_time = current_time;
            }

            // Convertir les deltas en flux optique
            // Le PMW3901 retourne des déplacements cumulés en pixels
            global_flow_x = (float)motion.delta_x * PMW3901_FLOW_SCALE;
            global_flow_y = (float)motion.delta_y * PMW3901_FLOW_SCALE;

            // Calcul de la vitesse si on a des données LiDAR valides
            float velocity_x = 0.0f;
            float velocity_y = 0.0f;

            if (lidar_data.valid && current_fps > 1.0f) {
                float distance_m = lidar_data.distance / 100.0f;
                float dt = 1.0f / current_fps;

                // Formule: vitesse (m/s) = (flux_pixels * distance * taille_pixel) / (focale * dt)
                velocity_x = (global_flow_x * distance_m * PMW3901_PIXEL_SIZE_MM * 1000.0f) /
                             (PMW3901_FOCAL_LENGTH_PX * dt);
                velocity_y = (global_flow_y * distance_m * PMW3901_PIXEL_SIZE_MM * 1000.0f) /
                             (PMW3901_FOCAL_LENGTH_PX * dt);
            }

            sensor_data_t data = {
                .timestamp = timestamp,
                .velocity_x = velocity_x,
                .velocity_y = velocity_y,
                .distance = lidar_data.distance,
                .lidar_valid = lidar_data.valid,
                .fps = current_fps
            };

            xQueueSend(dataQueue, &data, 0);
        }

        // Le PMW3901 peut échantillonner à ~121 FPS
        // On peut mettre un délai réduit pour maximiser la vitesse
        vTaskDelay(pdMS_TO_TICKS(8));  // ~125 Hz
    }
}
#endif  // USE_PMW3901

// ============================================================================
// CORE 1: Tâche LiDAR + transmission
// ============================================================================
static void lidar_serial_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LiDAR/Serial task started on core %d", xPortGetCoreID());

    sensor_data_t data;

    printf("\n=================================\n");
    printf("ESP32-S3 Optical Flow System\n");
#ifdef USE_CAMERA
    printf("Mode: Camera-based optical flow\n");
    printf("Camera: %s\n", CAMERA_MODEL);
    printf("Resolution: %dx%d\n", IMG_WIDTH, IMG_HEIGHT);
    printf("XCLK: %d MHz\n", XCLK_FREQ / 1000000);
    printf("Pixel size: %.4f mm\n", PIXEL_SIZE_MM);
    printf("Focal length: %.2f px\n", FOCAL_LENGTH_PX);
#endif
#ifdef USE_PMW3901
    printf("Mode: PMW3901 optical flow sensor\n");
    printf("Sensor: PMW3901\n");
    printf("FOV: 42 degrees\n");
    printf("Max rate: 121 FPS\n");
    printf("Pixel size: %.4f mm\n", PMW3901_PIXEL_SIZE_MM);
    printf("Focal length: %.2f px\n", PMW3901_FOCAL_LENGTH_PX);
#endif
    printf("LiDAR: %s\n", LIDAR_MODEL);
    printf("=================================\n");
    printf("Binary Protocol Mode\n");
    printf("Packet: [0xAA 0x55][ts(4)][vx(2)][vy(2)][dist(2)][chk(1)]\n");
    printf("Starting transmission...\n\n");

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        read_lidar();

        if (xQueueReceive(dataQueue, &data, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint32_t timestamp_ms = (uint32_t)(data.timestamp / 1000);
            send_binary_packet(timestamp_ms, data.velocity_x, data.velocity_y,
                             data.lidar_valid ? data.distance : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// Point d'entrée principal
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 Optical Flow + LiDAR ===");

#ifdef USE_CAMERA
    ESP_LOGI(TAG, "Mode: Camera-based optical flow");
    ESP_LOGI(TAG, "Camera selected: %s", CAMERA_MODEL);

    ESP_ERROR_CHECK(init_camera());
    ESP_LOGI(TAG, "Camera initialized");

    img_prev = (uint8_t *)heap_caps_malloc(IMG_WIDTH * IMG_HEIGHT, MALLOC_CAP_SPIRAM);
    img_cur = (uint8_t *)heap_caps_malloc(IMG_WIDTH * IMG_HEIGHT, MALLOC_CAP_SPIRAM);

    if (!img_prev || !img_cur) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Memory allocated");
#endif

#ifdef USE_PMW3901
    ESP_LOGI(TAG, "Mode: PMW3901 optical flow sensor");

    ESP_ERROR_CHECK(init_pmw3901());
    ESP_LOGI(TAG, "PMW3901 initialized");
#endif

    init_lidar_uart();

    frameMutex = xSemaphoreCreateMutex();
    dataQueue = xQueueCreate(20, sizeof(sensor_data_t));

    if (!frameMutex || !dataQueue) {
        ESP_LOGE(TAG, "Failed to create mutex/queue");
        return;
    }

    last_stat_time = esp_timer_get_time();

#ifdef USE_CAMERA
    xTaskCreatePinnedToCore(camera_task, "CameraTask", 8192, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Camera task created on core 0");
#endif

#ifdef USE_PMW3901
    xTaskCreatePinnedToCore(pmw3901_task, "PMW3901Task", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "PMW3901 task created on core 0");
#endif

    xTaskCreatePinnedToCore(lidar_serial_task, "LidarTask", 4096, NULL, 3, NULL, 1);
    ESP_LOGI(TAG, "LiDAR task created on core 1");

#ifdef USE_CAMERA
    ESP_LOGI(TAG, "System running with %s", CAMERA_MODEL);
#endif
#ifdef USE_PMW3901
    ESP_LOGI(TAG, "System running with PMW3901");
#endif
}