/**
 * @file system_config.c
 * @brief System configuration management using NVS
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "system_config.h"
#include "app_config.h"

static const char *TAG = "SYS_CONFIG";
static const char *NVS_NAMESPACE = "isolation_sphere";

// Default configuration values
static const system_config_t default_config = {
    .wifi_ssid = "IsolationSphere",
    .wifi_password = "sphere123",
    .ros2_agent_ip = "192.168.1.100",
    .ros2_agent_port = 8888,
    .target_fps = TARGET_FPS,
    .imu_filter_frequency = IMU_FILTER_FREQUENCY,
    .brightness = 128,
    .debug_mode = true,
    .auto_calibrate = true,
    .led_test_mode = false,
    .sphere_radius = SPHERE_RADIUS_MM
};

static system_config_t current_config;

/**
 * @brief Load configuration from NVS
 */
esp_err_t system_config_load(void) {
    ESP_LOGI(TAG, "Loading system configuration from NVS");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace, using defaults");
        current_config = default_config;
        return system_config_save(); // Save defaults to NVS
    }
    
    // Start with defaults
    current_config = default_config;
    
    // Load string values
    size_t required_size = sizeof(current_config.wifi_ssid);
    nvs_get_str(nvs_handle, "wifi_ssid", current_config.wifi_ssid, &required_size);
    
    required_size = sizeof(current_config.wifi_password);
    nvs_get_str(nvs_handle, "wifi_password", current_config.wifi_password, &required_size);
    
    required_size = sizeof(current_config.ros2_agent_ip);
    nvs_get_str(nvs_handle, "ros2_agent_ip", current_config.ros2_agent_ip, &required_size);
    
    // Load numeric values
    nvs_get_u16(nvs_handle, "ros2_agent_port", &current_config.ros2_agent_port);
    nvs_get_f(nvs_handle, "target_fps", &current_config.target_fps);
    nvs_get_f(nvs_handle, "imu_filter_freq", &current_config.imu_filter_frequency);
    nvs_get_u8(nvs_handle, "brightness", &current_config.brightness);
    nvs_get_f(nvs_handle, "sphere_radius", &current_config.sphere_radius);
    
    // Load boolean values
    uint8_t bool_val;
    if (nvs_get_u8(nvs_handle, "debug_mode", &bool_val) == ESP_OK) {
        current_config.debug_mode = bool_val != 0;
    }
    if (nvs_get_u8(nvs_handle, "auto_calibrate", &bool_val) == ESP_OK) {
        current_config.auto_calibrate = bool_val != 0;
    }
    if (nvs_get_u8(nvs_handle, "led_test_mode", &bool_val) == ESP_OK) {
        current_config.led_test_mode = bool_val != 0;
    }
    
    nvs_close(nvs_handle);
    
    // Apply configuration to global context
    if (g_app_ctx) {
        strncpy(g_app_ctx->wifi_ssid, current_config.wifi_ssid, sizeof(g_app_ctx->wifi_ssid) - 1);
        strncpy(g_app_ctx->wifi_password, current_config.wifi_password, sizeof(g_app_ctx->wifi_password) - 1);
        strncpy(g_app_ctx->ros2_agent_ip, current_config.ros2_agent_ip, sizeof(g_app_ctx->ros2_agent_ip) - 1);
        g_app_ctx->ros2_agent_port = current_config.ros2_agent_port;
        g_app_ctx->debug_mode = current_config.debug_mode;
    }
    
    ESP_LOGI(TAG, "Configuration loaded:");
    ESP_LOGI(TAG, "  WiFi SSID: %s", current_config.wifi_ssid);
    ESP_LOGI(TAG, "  ROS2 Agent: %s:%d", current_config.ros2_agent_ip, current_config.ros2_agent_port);
    ESP_LOGI(TAG, "  Target FPS: %.1f", current_config.target_fps);
    ESP_LOGI(TAG, "  Brightness: %d", current_config.brightness);
    ESP_LOGI(TAG, "  Debug mode: %s", current_config.debug_mode ? "ON" : "OFF");
    
    return ESP_OK;
}

/**
 * @brief Save configuration to NVS
 */
esp_err_t system_config_save(void) {
    ESP_LOGI(TAG, "Saving system configuration to NVS");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save string values
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "wifi_ssid", current_config.wifi_ssid));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "wifi_password", current_config.wifi_password));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "ros2_agent_ip", current_config.ros2_agent_ip));
    
    // Save numeric values
    ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "ros2_agent_port", current_config.ros2_agent_port));
    ESP_ERROR_CHECK(nvs_set_f(nvs_handle, "target_fps", current_config.target_fps));
    ESP_ERROR_CHECK(nvs_set_f(nvs_handle, "imu_filter_freq", current_config.imu_filter_frequency));
    ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "brightness", current_config.brightness));
    ESP_ERROR_CHECK(nvs_set_f(nvs_handle, "sphere_radius", current_config.sphere_radius));
    
    // Save boolean values
    ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "debug_mode", current_config.debug_mode ? 1 : 0));
    ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "auto_calibrate", current_config.auto_calibrate ? 1 : 0));
    ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "led_test_mode", current_config.led_test_mode ? 1 : 0));
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Configuration saved successfully");
    }
    
    nvs_close(nvs_handle);
    return ret;
}

/**
 * @brief Get current configuration
 */
const system_config_t* system_config_get(void) {
    return &current_config;
}

/**
 * @brief Set WiFi credentials
 */
esp_err_t system_config_set_wifi(const char* ssid, const char* password) {
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ssid) >= sizeof(current_config.wifi_ssid) ||
        strlen(password) >= sizeof(current_config.wifi_password)) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    strncpy(current_config.wifi_ssid, ssid, sizeof(current_config.wifi_ssid) - 1);
    strncpy(current_config.wifi_password, password, sizeof(current_config.wifi_password) - 1);
    current_config.wifi_ssid[sizeof(current_config.wifi_ssid) - 1] = '\0';
    current_config.wifi_password[sizeof(current_config.wifi_password) - 1] = '\0';
    
    ESP_LOGI(TAG, "WiFi credentials updated: SSID=%s", ssid);
    return ESP_OK;
}

/**
 * @brief Set ROS2 agent address
 */
esp_err_t system_config_set_ros2_agent(const char* ip, uint16_t port) {
    if (!ip) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ip) >= sizeof(current_config.ros2_agent_ip)) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    strncpy(current_config.ros2_agent_ip, ip, sizeof(current_config.ros2_agent_ip) - 1);
    current_config.ros2_agent_ip[sizeof(current_config.ros2_agent_ip) - 1] = '\0';
    current_config.ros2_agent_port = port;
    
    ESP_LOGI(TAG, "ROS2 agent updated: %s:%d", ip, port);
    return ESP_OK;
}

/**
 * @brief Set display brightness
 */
esp_err_t system_config_set_brightness(uint8_t brightness) {
    current_config.brightness = brightness;
    ESP_LOGI(TAG, "Brightness updated: %d", brightness);
    return ESP_OK;
}

/**
 * @brief Set target FPS
 */
esp_err_t system_config_set_target_fps(float fps) {
    if (fps <= 0.0f || fps > 60.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    
    current_config.target_fps = fps;
    ESP_LOGI(TAG, "Target FPS updated: %.1f", fps);
    return ESP_OK;
}

/**
 * @brief Set IMU filter frequency
 */
esp_err_t system_config_set_imu_filter_freq(float frequency) {
    if (frequency <= 0.0f || frequency > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    
    current_config.imu_filter_frequency = frequency;
    ESP_LOGI(TAG, "IMU filter frequency updated: %.1f Hz", frequency);
    return ESP_OK;
}

/**
 * @brief Set debug mode
 */
esp_err_t system_config_set_debug_mode(bool enabled) {
    current_config.debug_mode = enabled;
    ESP_LOGI(TAG, "Debug mode %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Set auto calibration
 */
esp_err_t system_config_set_auto_calibrate(bool enabled) {
    current_config.auto_calibrate = enabled;
    ESP_LOGI(TAG, "Auto calibration %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Set LED test mode
 */
esp_err_t system_config_set_led_test_mode(bool enabled) {
    current_config.led_test_mode = enabled;
    ESP_LOGI(TAG, "LED test mode %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Reset configuration to defaults
 */
esp_err_t system_config_reset_to_defaults(void) {
    ESP_LOGI(TAG, "Resetting configuration to defaults");
    
    current_config = default_config;
    
    // Erase NVS partition
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    
    return system_config_save();
}

/**
 * @brief Print current configuration
 */
void system_config_print(void) {
    ESP_LOGI(TAG, "Current System Configuration:");
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "WiFi:");
    ESP_LOGI(TAG, "  SSID: %s", current_config.wifi_ssid);
    ESP_LOGI(TAG, "  Password: %s", current_config.debug_mode ? current_config.wifi_password : "***");
    ESP_LOGI(TAG, "ROS2:");
    ESP_LOGI(TAG, "  Agent IP: %s", current_config.ros2_agent_ip);
    ESP_LOGI(TAG, "  Agent Port: %d", current_config.ros2_agent_port);
    ESP_LOGI(TAG, "Display:");
    ESP_LOGI(TAG, "  Target FPS: %.1f", current_config.target_fps);
    ESP_LOGI(TAG, "  Brightness: %d", current_config.brightness);
    ESP_LOGI(TAG, "  Sphere Radius: %.1f mm", current_config.sphere_radius);
    ESP_LOGI(TAG, "IMU:");
    ESP_LOGI(TAG, "  Filter Frequency: %.1f Hz", current_config.imu_filter_frequency);
    ESP_LOGI(TAG, "  Auto Calibrate: %s", current_config.auto_calibrate ? "YES" : "NO");
    ESP_LOGI(TAG, "System:");
    ESP_LOGI(TAG, "  Debug Mode: %s", current_config.debug_mode ? "ON" : "OFF");
    ESP_LOGI(TAG, "  LED Test Mode: %s", current_config.led_test_mode ? "ON" : "OFF");
    ESP_LOGI(TAG, "================================");
}