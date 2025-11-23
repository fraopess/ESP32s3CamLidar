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
#define USE_PMW3901       // Utiliser le capteur PMW3901 pour le flux optique

// ============================================================================
// SÉLECTION DE LA CAMÉRA (seulement si USE_CAMERA est activé)
// ============================================================================
//#define USE_OV2640        // Décommenter pour OV2640 (caméra commune ESP32-CAM)
//#define USE_OV5640        // Décommenter pour OV5640
//#define USE_OV7725        // Décommenter pour OV7725 original (PID=0x77)

// ============================================================================
// Vérification configuration mode flux optique
// ============================================================================
// Note: Si USE_CAMERA et USE_PMW3901 sont tous les deux définis, USE_PMW3901 sera prioritaire

// Note: Si aucun mode n'est sélectionné, le système fonctionnera uniquement avec le LiDAR
#if !defined(USE_CAMERA) && !defined(USE_PMW3901)
    #define LIDAR_ONLY_MODE
#endif

// ============================================================================
// Vérification configuration caméra (seulement si USE_CAMERA est activé)
// ============================================================================
// Note: Si USE_CAMERA est défini, sélectionnez UNE SEULE caméra ci-dessous
// Si plusieurs caméras sont définies, OV2640 sera prioritaire
#ifdef USE_CAMERA
    #if !defined(USE_OV2640) && !defined(USE_OV5640) && !defined(USE_OV7725)
        // Par défaut, utiliser OV2640 si aucune caméra n'est spécifiée
        #define USE_OV2640
    #endif
#endif

// ============================================================================
// SÉLECTION DU LIDAR (décommenter UNE SEULE ligne)
// ============================================================================
//#define USE_TFMINI    // Décommenter pour TFMini (115200 baud)
#define USE_DTS6012M  // Décommenter pour DTS6012M (921600 baud, haute vitesse)
//#define USE_MTF02     // Décommenter pour MTF-02 (115200 baud, distance + flux optique intégré)

// ============================================================================
// Vérification configuration lidar
// ============================================================================
#if (defined(USE_TFMINI) + defined(USE_DTS6012M) + defined(USE_MTF02)) > 1
    #error "ERREUR: Sélectionnez UN SEUL lidar!"
#endif

#if !defined(USE_TFMINI) && !defined(USE_DTS6012M) && !defined(USE_MTF02)
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
#include "drivers/camera_optical_flow.h"
#endif

#ifdef USE_PMW3901
#include "drivers/pmw3901.h"
#endif

#ifdef USE_TFMINI
#include "drivers/tfmini.h"
#endif

#ifdef USE_DTS6012M
#include "drivers/dts6012m.h"
#endif

#ifdef USE_MTF02
#include "drivers/mtf02.h"
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
    #define LIDAR_BAUDRATE     921600    // DTS6012M factory default
    #define LIDAR_MODEL        "DTS6012M"
#endif

#ifdef USE_MTF02
    #define LIDAR_BAUDRATE     115200    // MTF-02 default (adjust if needed)
    #define LIDAR_MODEL        "MTF-02"
#endif

// ============================================================================
// Configuration SPI PMW3901 (Optical Flow Sensor)
// ============================================================================
#ifdef USE_PMW3901
    #define PMW3901_SPI_HOST   SPI2_HOST
    #define PMW3901_PIN_CS     10    // À adapter selon votre câblage
    #define PMW3901_PIN_SCK    12    // À adapter selon votre câblage
    #define PMW3901_PIN_MISO   11    // Try swapping MISO/MOSI if getting 0x00
    #define PMW3901_PIN_MOSI   13    // Try swapping MISO/MOSI if getting 0x00

    // Paramètres du capteur PMW3901
    #define PMW3901_FOCAL_LENGTH_PX    42.0f     // FOV de 42°, ~35x35 pixels effectifs
    #define PMW3901_PIXEL_SIZE_MM      0.030f    // Estimation basée sur FOV
    #define PMW3901_MAX_DELTA          127       // Valeur maximale de delta (8-bit signé)
    #define PMW3901_FLOW_SCALE         1.0f      // Facteur d'échelle pour le flux
    #define PMW3901_ALPHA 0.1f               // Complementary filter coefficient

    // Mode de lecture: burst mode enabled (faster single-transaction read)
    //#define PMW3901_USE_BURST_MODE
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

#ifdef USE_TFMINI
static volatile tfmini_data_t lidar_data = {0, 0, false};
#endif

#ifdef USE_DTS6012M
static volatile dts6012m_data_t lidar_data = {0, 0, false};
#endif

#ifdef USE_MTF02
static volatile mtf02_data_t lidar_data = {0, 0, false, 0, 0, 0, false};
#endif

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
#ifdef USE_TFMINI
    ESP_ERROR_CHECK(tfmini_init(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BUF_SIZE));
#endif

#ifdef USE_DTS6012M
    ESP_ERROR_CHECK(dts6012m_init(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BUF_SIZE));
#endif

#ifdef USE_MTF02
    ESP_ERROR_CHECK(mtf02_init(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BUF_SIZE));
#endif
}



// ============================================================================
// Lecture LiDAR (appelle la fonction appropriée)
// ============================================================================
static inline void read_lidar(void)
{
#ifdef USE_TFMINI
    tfmini_read(LIDAR_UART_NUM, &lidar_data, MAX_VALID_DISTANCE_CM);
#endif
#ifdef USE_DTS6012M
    dts6012m_read(LIDAR_UART_NUM, &lidar_data, MAX_VALID_DISTANCE_CM);
#endif
#ifdef USE_MTF02
    mtf02_read(LIDAR_UART_NUM, &lidar_data, MAX_VALID_DISTANCE_CM);
#endif
}

// ============================================================================
// Initialisation caméra (OV2640 ou OV5640)
// ============================================================================
#ifdef USE_CAMERA
static esp_err_t init_camera(void)
{
    camera_config_optical_flow_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_siod = CAM_PIN_SIOD,
        .pin_sioc = CAM_PIN_SIOC,
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
        .xclk_freq = XCLK_FREQ,
        .frame_size = CAMERA_FRAME_SIZE,
        .img_width = IMG_WIDTH,
        .img_height = IMG_HEIGHT,
        .focal_length_px = FOCAL_LENGTH_PX,
        .pixel_size_mm = PIXEL_SIZE_MM,
        .flow_smoothing_alpha = FLOW_SMOOTHING_ALPHA,
        .min_gradient_threshold = MIN_GRADIENT_THRESHOLD,
        .min_valid_pixels = MIN_VALID_PIXELS,
    };

    return camera_optical_flow_init(&config);
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

    // Activer le LED du capteur
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
                camera_optical_flow_compute(img_prev, img_cur, IMG_WIDTH, IMG_HEIGHT, OPTICAL_FLOW_STEP,
                                           MIN_GRADIENT_THRESHOLD, MIN_VALID_PIXELS, FLOW_SMOOTHING_ALPHA,
                                           &global_flow_x, &global_flow_y);
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
    static int debug_count = 0;

    while (1) {
        int64_t timestamp = esp_timer_get_time();

        // Lire les données du PMW3901
#ifdef PMW3901_USE_BURST_MODE
        esp_err_t ret = pmw3901_read_motion_burst(pmw3901_handle, &motion);
#else
        esp_err_t ret = pmw3901_read_motion(pmw3901_handle, &motion);
#endif

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

            // Velocity calculation with complementary filter
            static float smooth_velocity_x = 0.0f;
            static float smooth_velocity_y = 0.0f;
            

            float velocity_x = 0.0f;
            float velocity_y = 0.0f;

            if (lidar_data.valid && current_fps > 1.0f) {
                float distance_m = lidar_data.distance / 100.0f;
                float dt = 1.0f / current_fps;

                // Direct formula: velocity (m/s) = (pixels * distance * pixel_size) / (focal_length * dt)
                float raw_vel_x = (global_flow_x * distance_m * PMW3901_PIXEL_SIZE_MM * 1000.0f) /
                                  (PMW3901_FOCAL_LENGTH_PX * dt);
                float raw_vel_y = (global_flow_y * distance_m * PMW3901_PIXEL_SIZE_MM * 1000.0f) /
                                  (PMW3901_FOCAL_LENGTH_PX * dt);

                // Apply complementary filter: output = alpha * new + (1-alpha) * previous
                smooth_velocity_x = PMW3901_ALPHA * raw_vel_x + (1.0f - PMW3901_ALPHA) * smooth_velocity_x;
                smooth_velocity_y = PMW3901_ALPHA * raw_vel_y + (1.0f - PMW3901_ALPHA) * smooth_velocity_y;

                velocity_x = smooth_velocity_x;
                velocity_y = smooth_velocity_y;
            } else {
                // Decay to zero when no valid data
                smooth_velocity_x *= 0.9f;
                smooth_velocity_y *= 0.9f;
                velocity_x = smooth_velocity_x;
                velocity_y = smooth_velocity_y;
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

            // Debug output every 50 iterations or when motion detected
            // debug_count++;
            // if (motion.motion || debug_count >= 50) {
            //     if (debug_count >= 50) debug_count = 0;

            //     ESP_LOGI(TAG, "=== PMW3901 Flow Data ===");
            //     ESP_LOGI(TAG, "Motion: %d, SQUAL: %d, FPS: %.1f",
            //              motion.motion, motion.squal, current_fps);
            //     ESP_LOGI(TAG, "Raw delta: dx=%d, dy=%d pixels",
            //              motion.delta_x, motion.delta_y);
            //     ESP_LOGI(TAG, "Flow: fx=%.2f, fy=%.2f",
            //              global_flow_x, global_flow_y);
            //     ESP_LOGI(TAG, "LiDAR: valid=%d, dist=%d cm",
            //              lidar_data.valid, lidar_data.distance);
            //     if (lidar_data.valid) {
            //         ESP_LOGI(TAG, "Velocity: vx=%.3f, vy=%.3f m/s",
            //                  velocity_x, velocity_y);
            //     } else {
            //         ESP_LOGI(TAG, "Velocity: N/A (no LiDAR data)");
            //     }
            // }
        }

        // Le PMW3901 peut échantillonner à ~121 FPS
        // On peut mettre un délai réduit pour maximiser la vitesse
        vTaskDelay(pdMS_TO_TICKS(8));  // ~125 Hz
    }
}
#endif  // USE_PMW3901

// ============================================================================
// CORE 0: Tâche MTF-02 (distance + flux optique intégré)
// ============================================================================
#ifdef USE_MTF02
static void mtf02_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MTF-02 task started on core %d", xPortGetCoreID());

    sensor_data_t data;
    int loop_count = 0;
    static bool first_valid_data = true;

    while (1) {
        read_lidar();

        // loop_count++;
        // if (loop_count % 100 == 0) {
        //     ESP_LOGI(TAG, "MTF-02 task alive, loop: %d, valid: %d", loop_count, lidar_data.valid);
        // }

        if (lidar_data.valid) {
            // if (first_valid_data) {
            //     first_valid_data = false;
            //     ESP_LOGI(TAG, "First valid MTF-02 data received!");
            //     ESP_LOGI(TAG, "  Distance: %d cm", lidar_data.distance);
            //     ESP_LOGI(TAG, "  Flow: (%d, %d)", lidar_data.flow_vel_x, lidar_data.flow_vel_y);
            // }

            int64_t timestamp = esp_timer_get_time();

            // Mise à jour du compteur de frames
            frame_count++;
            int64_t current_time = esp_timer_get_time();
            if (current_time - last_stat_time >= 1000000) {
                current_fps = (float)frame_count * 1000000.0f / (current_time - last_stat_time);
                frame_count = 0;
                last_stat_time = current_time;
            }

            // Calculer la vitesse à partir du flux optique intégré
            float velocity_x = 0.0f;
            float velocity_y = 0.0f;

            if (lidar_data.flow_valid && lidar_data.distance > 0) {
                float height_m = lidar_data.distance / 100.0f;
                mtf02_get_flow_velocity(&lidar_data, &velocity_x, &velocity_y, height_m);
            }

            // // Debug: Log calculated velocities periodically
            // static int vel_debug_count = 0;
            // vel_debug_count++;
            // if (vel_debug_count % 30 == 0) {
            //     ESP_LOGI(TAG, "=== MTF-02 Calculated Velocities ===");
            //     ESP_LOGI(TAG, "Conditions: flow_valid=%d, distance=%d cm",
            //              lidar_data.flow_valid, lidar_data.distance);
            //     ESP_LOGI(TAG, "Raw flow: vel_x=%d, vel_y=%d (cm/s @ 1m)",
            //              lidar_data.flow_vel_x, lidar_data.flow_vel_y);
            //     if (lidar_data.distance > 0) {
            //         float height_m = lidar_data.distance / 100.0f;
            //         ESP_LOGI(TAG, "Height: %.2f m", height_m);
            //         ESP_LOGI(TAG, "Expected vx: %.3f m/s, vy: %.3f m/s",
            //                  (lidar_data.flow_vel_x * height_m) / 100.0f,
            //                  (lidar_data.flow_vel_y * height_m) / 100.0f);
            //     }
            //     ESP_LOGI(TAG, "Final: vx=%.3f m/s, vy=%.3f m/s", velocity_x, velocity_y);
            //     ESP_LOGI(TAG, "Packet values: vx=%d, vy=%d (x1000)",
            //              (int16_t)(velocity_x * 1000.0f), (int16_t)(velocity_y * 1000.0f));
            // }

            data.timestamp = timestamp;
            data.velocity_x = velocity_x;
            data.velocity_y = velocity_y;
            data.distance = lidar_data.distance;
            data.lidar_valid = lidar_data.valid;
            data.fps = current_fps;

            xQueueSend(dataQueue, &data, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // ~100 Hz
    }
}
#endif  // USE_MTF02

// ============================================================================
// CORE 1: Tâche LiDAR + transmission
// ============================================================================
static void lidar_serial_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LiDAR/Serial task started on core %d", xPortGetCoreID());

    sensor_data_t data;
    static int packets_sent = 0;
    static bool first_packet_sent = false;

    // printf("\n=================================\n");
    // printf("ESP32-S3 Optical Flow System\n");
// #ifdef LIDAR_ONLY_MODE
    // printf("Mode: LiDAR only\n");
// #endif
// #ifdef USE_CAMERA
    // printf("Mode: Camera-based optical flow\n");
    // printf("Camera: %s\n", CAMERA_MODEL);
    // printf("Resolution: %dx%d\n", IMG_WIDTH, IMG_HEIGHT);
    // printf("XCLK: %d MHz\n", XCLK_FREQ / 1000000);
    // printf("Pixel size: %.4f mm\n", PIXEL_SIZE_MM);
    // printf("Focal length: %.2f px\n", FOCAL_LENGTH_PX);
// #endif
// #ifdef USE_PMW3901
    // printf("Mode: PMW3901 optical flow sensor\n");
    // printf("Sensor: PMW3901\n");
    // printf("FOV: 42 degrees\n");
    // printf("Max rate: 121 FPS\n");
    // printf("Pixel size: %.4f mm\n", PMW3901_PIXEL_SIZE_MM);
    // printf("Focal length: %.2f px\n", PMW3901_FOCAL_LENGTH_PX);
// #endif
    // printf("LiDAR: %s\n", LIDAR_MODEL);
    // printf("=================================\n");
    // printf("Binary Protocol Mode\n");
    // printf("Packet: [0xAA 0x55][ts(4)][vx(2)][vy(2)][dist(2)][chk(1)]\n");
    // printf("Starting transmission...\n\n");

    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
#ifdef USE_MTF02
        // Mode MTF-02 avec flux optique intégré - données reçues via queue
        if (xQueueReceive(dataQueue, &data, pdMS_TO_TICKS(10)) == pdTRUE) {
            // if (!first_packet_sent) {
            //     first_packet_sent = true;
            //     ESP_LOGI(TAG, "First packet from queue received and sending!");
            //     ESP_LOGI(TAG, "  Vel: (%.3f, %.3f) m/s, Dist: %d cm",
            //              data.velocity_x, data.velocity_y, data.distance);
            // }

            uint32_t timestamp_ms = (uint32_t)(data.timestamp / 1000);
            send_binary_packet(timestamp_ms, data.velocity_x, data.velocity_y,
                             data.lidar_valid ? data.distance : 0);
            // packets_sent++;

            // if (packets_sent % 100 == 0) {
            //     ESP_LOGI(TAG, "Packets sent: %d, FPS: %.1f", packets_sent, data.fps);
            // }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
#else
        read_lidar();

#ifdef LIDAR_ONLY_MODE
        // Mode LiDAR uniquement - pas de flux optique
        if (lidar_data.valid) {
            int64_t timestamp = esp_timer_get_time();
            uint32_t timestamp_ms = (uint32_t)(timestamp / 1000);
            // Envoyer uniquement les données LiDAR, vitesse = 0
            send_binary_packet(timestamp_ms, 0.0f, 0.0f, lidar_data.distance);
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // ~100 Hz pour LiDAR seul
#else
        // Mode avec flux optique
        if (xQueueReceive(dataQueue, &data, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint32_t timestamp_ms = (uint32_t)(data.timestamp / 1000);
            send_binary_packet(timestamp_ms, data.velocity_x, data.velocity_y,
                             data.lidar_valid ? data.distance : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
#endif
#endif
    }
}

// ============================================================================
// Point d'entrée principal
// ============================================================================
void app_main(void)
{
    // Very first output - should appear immediately
    printf("\n\n\n*** APP_MAIN STARTED ***\n");
    printf("*** UART OUTPUT TEST ***\n");
    printf("*** If you see this, UART is working! ***\n\n");

    ESP_LOGI(TAG, "=== ESP32-S3 Optical Flow + LiDAR ===");

#ifdef LIDAR_ONLY_MODE
    ESP_LOGI(TAG, "Mode: LiDAR only (no optical flow sensor)");
#endif

#ifdef USE_CAMERA
    // ESP_LOGI(TAG, "Mode: Camera-based optical flow");
    // ESP_LOGI(TAG, "Camera selected: %s", CAMERA_MODEL);

    ESP_ERROR_CHECK(init_camera());
    // ESP_LOGI(TAG, "Camera initialized");

    ESP_ERROR_CHECK(camera_optical_flow_alloc_buffers(IMG_WIDTH, IMG_HEIGHT, &img_prev, &img_cur));
    // ESP_LOGI(TAG, "Memory allocated");
#endif

#ifdef USE_PMW3901
    // ESP_LOGI(TAG, "Mode: PMW3901 optical flow sensor");

    esp_err_t ret = init_pmw3901();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMW3901 initialization failed. Continuing in LiDAR-only mode.");
    }
#endif

    init_lidar_uart();

#ifdef USE_DTS6012M
    // Send start measurement command to DTS6012M
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for sensor to be ready
    dts6012m_start_measurement(LIDAR_UART_NUM);
#endif

#ifdef USE_MTF02
    // Try to start MTF-02 measurement (may not be needed)
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for sensor to be ready
    mtf02_start_measurement(LIDAR_UART_NUM);

    ESP_LOGI(TAG, "Mode: MTF-02 integrated distance + optical flow sensor");
    // MTF-02 has integrated optical flow, create queue for data
    dataQueue = xQueueCreate(20, sizeof(sensor_data_t));
    if (!dataQueue) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
    ESP_LOGI(TAG, "MTF-02 queue created successfully");
#else
#ifndef LIDAR_ONLY_MODE
    frameMutex = xSemaphoreCreateMutex();
    dataQueue = xQueueCreate(20, sizeof(sensor_data_t));

    if (!frameMutex || !dataQueue) {
        // ESP_LOGE(TAG, "Failed to create mutex/queue");
        return;
    }
#else
    dataQueue = xQueueCreate(20, sizeof(sensor_data_t));
    if (!dataQueue) {
        // ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
#endif
#endif

    last_stat_time = esp_timer_get_time();

#ifdef USE_CAMERA
    xTaskCreatePinnedToCore(camera_task, "CameraTask", 8192, NULL, 5, NULL, 0);
    // ESP_LOGI(TAG, "Camera task created on core 0");
#endif

#ifdef USE_PMW3901
    if (pmw3901_handle != NULL) {
        xTaskCreatePinnedToCore(pmw3901_task, "PMW3901Task", 4096, NULL, 5, NULL, 0);
        // ESP_LOGI(TAG, "PMW3901 task created on core 0");
    }
#endif

#ifdef USE_MTF02
    xTaskCreatePinnedToCore(mtf02_task, "MTF02Task", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "MTF-02 task created on core 0");
#endif

    xTaskCreatePinnedToCore(lidar_serial_task, "LidarTask", 4096, NULL, 3, NULL, 1);
    ESP_LOGI(TAG, "LiDAR task created on core 1");

#ifdef USE_CAMERA
    ESP_LOGI(TAG, "System running with %s", CAMERA_MODEL);
#endif
#ifdef USE_PMW3901
    if (pmw3901_handle != NULL) {
        ESP_LOGI(TAG, "System running with PMW3901");
    }
#endif
#ifdef USE_MTF02
    ESP_LOGI(TAG, "System running with %s", LIDAR_MODEL);
#endif
#ifdef LIDAR_ONLY_MODE
    ESP_LOGI(TAG, "System running with LiDAR only");
#endif

    ESP_LOGI(TAG, "*** Initialization complete ***");
    ESP_LOGI(TAG, "System running - binary data streaming...");

    // Keep app_main alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}