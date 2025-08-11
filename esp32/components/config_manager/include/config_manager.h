#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 設定構造体定義
typedef struct {
    // 画像設定
    struct {
        uint16_t width;
        uint16_t height;
        uint8_t fps;
        uint8_t quality;
        char format[16];
    } image;
    
    // LED設定
    struct {
        uint16_t count;
        uint8_t brightness;
        uint8_t strip_count;
        uint16_t leds_per_strip;
        uint32_t update_rate_hz;
    } led;
    
    // IMU設定
    struct {
        uint8_t sample_rate_hz;
        bool calibration_enabled;
        float orientation_offset[4]; // quaternion
        uint8_t filter_strength;
    } imu;
    
    // 通信設定
    struct {
        char wifi_ssid[32];
        char wifi_password[64];
        uint16_t ros2_domain_id;
        char ros2_namespace[32];
        uint16_t update_rate_hz;
    } communication;
    
    // システム設定
    struct {
        uint8_t log_level;
        bool debug_mode;
        uint32_t watchdog_timeout_ms;
        bool performance_monitoring;
    } system;
} sphere_config_t;

// デフォルト設定値
#define DEFAULT_IMAGE_WIDTH 320
#define DEFAULT_IMAGE_HEIGHT 160
#define DEFAULT_IMAGE_FPS 10
#define DEFAULT_IMAGE_QUALITY 80
#define DEFAULT_IMAGE_FORMAT "JPEG"

#define DEFAULT_LED_COUNT 800
#define DEFAULT_LED_BRIGHTNESS 128
#define DEFAULT_LED_STRIP_COUNT 4
#define DEFAULT_LEDS_PER_STRIP 200
#define DEFAULT_LED_UPDATE_RATE 30

#define DEFAULT_IMU_SAMPLE_RATE 100
#define DEFAULT_IMU_FILTER_STRENGTH 5

#define DEFAULT_ROS2_DOMAIN_ID 0
#define DEFAULT_COMM_UPDATE_RATE 30

#define DEFAULT_LOG_LEVEL 3  // ESP_LOG_INFO
#define DEFAULT_WATCHDOG_TIMEOUT 5000

// 設定ファイルパス
#define CONFIG_FILE_PATH "/spiffs/sphere_config.json"
#define CONFIG_BACKUP_PATH "/spiffs/sphere_config_backup.json"

// 関数プロトタイプ
esp_err_t config_manager_init(void);
esp_err_t config_manager_deinit(void);
esp_err_t config_load_from_file(sphere_config_t *config);
esp_err_t config_save_to_file(const sphere_config_t *config);
esp_err_t config_load_defaults(sphere_config_t *config);
esp_err_t config_validate(const sphere_config_t *config);
esp_err_t config_backup(void);
esp_err_t config_restore_from_backup(void);
bool config_file_exists(void);

// 個別設定更新関数
esp_err_t config_update_image_settings(uint16_t width, uint16_t height, uint8_t fps, uint8_t quality);
esp_err_t config_update_led_settings(uint16_t count, uint8_t brightness, uint32_t update_rate);
esp_err_t config_update_imu_settings(uint8_t sample_rate, bool calibration, float *orientation_offset);
esp_err_t config_update_communication_settings(const char *ssid, const char *password, uint16_t domain_id);
esp_err_t config_update_system_settings(uint8_t log_level, bool debug_mode);

// 設定情報取得関数
const sphere_config_t* config_get_current(void);
void config_print_current(void);

#ifdef __cplusplus
}
#endif