#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "config_manager.h"

static const char *TAG = "SPIFFS_TEST";

void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SPIFFS Configuration Manager Test");
    ESP_LOGI(TAG, "========================================");
    
    // 設定マネージャー初期化
    esp_err_t ret = config_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize config manager: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "✓ Config manager initialized successfully");
    
    // 現在の設定を表示
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Current Configuration:");
    ESP_LOGI(TAG, "========================================");
    config_print_current();
    
    // 設定値の取得テスト
    const sphere_config_t *config = config_get_current();
    if (config) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Configuration Access Test:");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Image resolution: %dx%d", config->image.width, config->image.height);
        ESP_LOGI(TAG, "LED count: %d", config->led.count);
        ESP_LOGI(TAG, "IMU sample rate: %d Hz", config->imu.sample_rate_hz);
        ESP_LOGI(TAG, "ROS2 namespace: %s", config->communication.ros2_namespace);
    }
    
    // 設定更新テスト
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Configuration Update Test:");
    ESP_LOGI(TAG, "========================================");
    
    ret = config_update_image_settings(640, 480, 15, 90);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Image settings updated successfully");
    } else {
        ESP_LOGE(TAG, "✗ Failed to update image settings: %s", esp_err_to_name(ret));
    }
    
    // 更新後の設定を表示
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Updated Configuration:");
    ESP_LOGI(TAG, "========================================");
    config_print_current();
    
    // 設定バックアップテスト
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Configuration Backup Test:");
    ESP_LOGI(TAG, "========================================");
    
    ret = config_backup();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Configuration backed up successfully");
    } else {
        ESP_LOGE(TAG, "✗ Failed to backup configuration: %s", esp_err_to_name(ret));
    }
    
    // 設定検証テスト
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Configuration Validation Test:");
    ESP_LOGI(TAG, "========================================");
    
    config = config_get_current();
    ret = config_validate(config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Configuration validation passed");
    } else {
        ESP_LOGE(TAG, "✗ Configuration validation failed: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SPIFFS Configuration Test Completed!");
    ESP_LOGI(TAG, "========================================");
    
    // メインループ
    int counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Running... iteration %d", ++counter);
        
        if (counter >= 5) {
            ESP_LOGI(TAG, "Test completed. Shutting down...");
            config_manager_deinit();
            break;
        }
    }
}