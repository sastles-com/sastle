/**
 * @file app_config.h
 * @brief Application configuration constants and structures
 * @author Isolation Sphere Team
 * @date 2024
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * System Configuration Constants
 * =============================================================================*/

// Hardware Configuration
#define SPHERE_RADIUS_MM            55.0f
#define NUM_LEDS                    800
#define NUM_LED_STRIPS              4
#define LEDS_PER_STRIP              (NUM_LEDS / NUM_LED_STRIPS)

// Image Processing
#define IMAGE_WIDTH                 320
#define IMAGE_HEIGHT                160
#define MAX_JPEG_SIZE               (64 * 1024)  // 64KB

// Performance Targets
#define TARGET_FPS                  30.0f
#define TARGET_FRAME_TIME_MS        33
#define MAX_PROCESSING_TIME_MS      25
#define IMU_SAMPLE_RATE_HZ          100
#define IMU_FILTER_FREQUENCY        30.0f

// GPIO Pin Definitions
#define PIN_I2C_SDA                 21
#define PIN_I2C_SCL                 22
#define PIN_SPI_MOSI                23
#define PIN_SPI_MISO                19
#define PIN_SPI_SCLK                18
#define PIN_SPI_CS_RP2350           5
#define PIN_SPI_CS_DISPLAY          15
#define PIN_SPI_DC_DISPLAY          2
#define PIN_VSYNC                   25

// Communication Configuration
#define I2C_MASTER_FREQ_HZ          400000
#define SPI_CLOCK_SPEED_HZ          40000000
#define WIFI_MAXIMUM_RETRY          10

/* =============================================================================
 * Task Configuration
 * =============================================================================*/

// Task Stack Sizes (in words, not bytes)
#define TASK_STACK_SIZE_IMU         (4 * 1024)
#define TASK_STACK_SIZE_ROS2_RX     (8 * 1024)
#define TASK_STACK_SIZE_ROS2_TX     (4 * 1024)
#define TASK_STACK_SIZE_IMAGE_PROC  (8 * 1024)
#define TASK_STACK_SIZE_SPI_TX      (4 * 1024)
#define TASK_STACK_SIZE_DISPLAY     (4 * 1024)
#define TASK_STACK_SIZE_MONITOR     (2 * 1024)

// Task Priorities (0-24, higher number = higher priority)
#define TASK_PRIORITY_IMU           24
#define TASK_PRIORITY_IMAGE_PROC    24
#define TASK_PRIORITY_SPI_TX        22
#define TASK_PRIORITY_ROS2_RX       20
#define TASK_PRIORITY_ROS2_TX       18
#define TASK_PRIORITY_DISPLAY       16
#define TASK_PRIORITY_MONITOR       10

// Task Core Assignment
#define TASK_CORE_COMMUNICATION     0  // Core 0: IMU, ROS2
#define TASK_CORE_PROCESSING        1  // Core 1: Image, SPI, Display

// Queue Sizes
#define QUEUE_SIZE_IMAGE            2   // Double buffering
#define QUEUE_SIZE_IMU              5   // IMU filtering
#define QUEUE_SIZE_UI_COMMAND       10  // UI commands
#define QUEUE_SIZE_SPI              4   // One per strip

/* =============================================================================
 * Data Structures
 * =============================================================================*/

/**
 * @brief Quaternion structure for orientation data
 */
typedef struct {
    float w, x, y, z;
    uint32_t timestamp;
    uint8_t calibration_status;
} quaternion_data_t;

/**
 * @brief RGB color structure
 */
typedef struct {
    uint8_t r, g, b;
} rgb_color_t;

/**
 * @brief LED position structure
 */
typedef struct {
    float x, y, z;          // Cartesian coordinates (mm)
    float theta, phi;       // Spherical coordinates (radians)
    uint8_t strip_id;       // Strip number (0-3)
    uint16_t strip_index;   // Index within strip (0-199)
} led_position_t;

/**
 * @brief Image frame structure
 */
typedef struct {
    uint8_t* jpeg_data;     // JPEG compressed data
    size_t jpeg_size;       // Size of JPEG data
    rgb_color_t* rgb_data;  // Decompressed RGB data
    uint32_t timestamp;     // Frame timestamp
    uint32_t frame_id;      // Sequential frame ID
    bool is_valid;          // Data validity flag
} image_frame_t;

/**
 * @brief SPI packet structure for RP2350 communication
 */
typedef struct {
    uint16_t magic;         // 0xAA55 magic number
    uint8_t frame_id;       // Frame sequence number
    uint8_t strip_id;       // LED strip ID (0-3)
    uint16_t data_len;      // Payload length (600)
    uint8_t led_data[600];  // RGB data for 200 LEDs
    uint16_t crc16;         // CRC16 checksum
} __attribute__((packed)) spi_packet_t;

/**
 * @brief UI command structure
 */
typedef struct {
    enum {
        CMD_BRIGHTNESS,
        CMD_OFFSET,
        CMD_MODE,
        CMD_PLAY,
        CMD_STOP,
        CMD_CALIBRATE
    } cmd_type;
    
    union {
        uint8_t brightness;     // 0-255
        quaternion_data_t offset; // Pose offset
        enum {
            MODE_NORMAL,
            MODE_DEMO,
            MODE_TEST
        } mode;
        uint32_t playlist_id;
    } value;
    
    uint32_t timestamp;
} ui_command_t;

/**
 * @brief System status structure
 */
typedef struct {
    bool wifi_connected;
    bool ros2_connected;
    bool imu_calibrated;
    float fps_current;
    float fps_average;
    uint32_t frame_count;
    uint32_t dropped_frames;
    size_t free_heap;
    size_t free_psram;
    float cpu_usage[2];     // Core 0, Core 1
    uint32_t uptime_ms;
} system_status_t;

/**
 * @brief Application context structure
 */
typedef struct {
    // Synchronization primitives
    QueueHandle_t queue_image;
    QueueHandle_t queue_imu;
    QueueHandle_t queue_ui_command;
    QueueHandle_t queue_spi[NUM_LED_STRIPS];
    
    SemaphoreHandle_t mutex_led_buffer;
    SemaphoreHandle_t mutex_system_status;
    
    EventGroupHandle_t event_group_system;
    
    // Data buffers
    image_frame_t image_buffers[2];
    rgb_color_t led_buffers[2][NUM_LEDS];
    uint8_t active_buffer;
    
    // System state
    system_status_t system_status;
    led_position_t* led_positions;
    uint16_t* mapping_lut_u;
    uint16_t* mapping_lut_v;
    
    // Configuration
    bool debug_mode;
    uint8_t log_level;
    char wifi_ssid[32];
    char wifi_password[64];
    char ros2_agent_ip[16];
    uint16_t ros2_agent_port;
} app_context_t;

/* =============================================================================
 * Event Group Bits
 * =============================================================================*/

#define EVENT_BIT_WIFI_CONNECTED    BIT0
#define EVENT_BIT_ROS2_CONNECTED    BIT1
#define EVENT_BIT_IMU_READY         BIT2
#define EVENT_BIT_IMAGE_READY       BIT3
#define EVENT_BIT_SYSTEM_ERROR      BIT4
#define EVENT_BIT_SHUTDOWN          BIT5

/* =============================================================================
 * Error Codes
 * =============================================================================*/

typedef enum {
    APP_OK = 0,
    APP_ERR_INVALID_ARG,
    APP_ERR_NO_MEM,
    APP_ERR_TIMEOUT,
    APP_ERR_NOT_FOUND,
    APP_ERR_NOT_SUPPORTED,
    APP_ERR_INVALID_STATE,
    APP_ERR_INIT_FAILED,
    APP_ERR_COMMUNICATION,
    APP_ERR_HARDWARE,
    APP_ERR_CONFIG
} app_err_t;

/* =============================================================================
 * Global Variables Declaration
 * =============================================================================*/

extern app_context_t* g_app_ctx;

/* =============================================================================
 * Function Declarations
 * =============================================================================*/

/**
 * @brief Initialize application context
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_context_init(void);

/**
 * @brief Cleanup application context
 */
void app_context_cleanup(void);

/**
 * @brief Get application context
 * @return Pointer to application context
 */
app_context_t* app_get_context(void);

#ifdef __cplusplus
}
#endif

#endif // APP_CONFIG_H