/**
 * @file system_config.h
 * @brief System configuration management header
 * @author Isolation Sphere Team
 * @date 2024
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief System configuration structure
 */
typedef struct {
    // Network configuration
    char wifi_ssid[32];
    char wifi_password[64];
    char ros2_agent_ip[16];
    uint16_t ros2_agent_port;
    
    // Display configuration
    float target_fps;
    uint8_t brightness;
    float sphere_radius;
    
    // IMU configuration
    float imu_filter_frequency;
    bool auto_calibrate;
    
    // System configuration
    bool debug_mode;
    bool led_test_mode;
} system_config_t;

/**
 * @brief Load configuration from NVS
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_load(void);

/**
 * @brief Save configuration to NVS
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_save(void);

/**
 * @brief Get current configuration
 * @return Pointer to current configuration
 */
const system_config_t* system_config_get(void);

/**
 * @brief Set WiFi credentials
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_wifi(const char* ssid, const char* password);

/**
 * @brief Set ROS2 agent address
 * @param ip IP address string
 * @param port Port number
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_ros2_agent(const char* ip, uint16_t port);

/**
 * @brief Set display brightness
 * @param brightness Brightness value (0-255)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_brightness(uint8_t brightness);

/**
 * @brief Set target FPS
 * @param fps Target frame rate
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_target_fps(float fps);

/**
 * @brief Set IMU filter frequency
 * @param frequency Filter frequency in Hz
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_imu_filter_freq(float frequency);

/**
 * @brief Set debug mode
 * @param enabled Debug mode enabled/disabled
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_debug_mode(bool enabled);

/**
 * @brief Set auto calibration
 * @param enabled Auto calibration enabled/disabled
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_auto_calibrate(bool enabled);

/**
 * @brief Set LED test mode
 * @param enabled LED test mode enabled/disabled
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_set_led_test_mode(bool enabled);

/**
 * @brief Reset configuration to defaults
 * @return ESP_OK on success, error code on failure
 */
esp_err_t system_config_reset_to_defaults(void);

/**
 * @brief Print current configuration
 */
void system_config_print(void);

#ifdef __cplusplus
}
#endif

#endif // SYSTEM_CONFIG_H