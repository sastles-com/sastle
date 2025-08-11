#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"

static const char *TAG = "IMU_MAIN";

// Forward declarations
esp_err_t run_imu_integration_test(void);
esp_err_t run_simple_hardware_test(void);
esp_err_t run_direct_imu_lcd_test(void);

void print_system_info(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Isolation Sphere - IMU Integration Test");
    ESP_LOGI(TAG, "========================================");
    
    // Chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s, cores: %d, revision: %d", 
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);
    
    // Flash information
    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash size: %ldMB", flash_size / (1024 * 1024));
    }
    
    // Memory information
    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size > 0) {
        ESP_LOGI(TAG, "PSRAM: %d bytes (%.1fMB)", psram_size, psram_size / (1024.0 * 1024.0));
        ESP_LOGI(TAG, "Free PSRAM: %ld bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        ESP_LOGI(TAG, "PSRAM: Not detected");
    }
    
    ESP_LOGI(TAG, "========================================");
}

void app_main(void)
{
    // Print system information
    print_system_info();
    
    // Wait a moment for system stabilization
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Run IMU integration test (original working implementation)
    ESP_LOGI(TAG, "Starting IMU integration test...");
    esp_err_t ret = run_imu_integration_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU integration test failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System will continue with basic functionality");
        
        // Fallback: basic system monitoring
        while(1) {
            ESP_LOGI(TAG, "System monitoring... Free heap: %ld", esp_get_free_heap_size());
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
    
    // IMU test is running in background task
    // Main task can perform other system management tasks
    while(1) {
        ESP_LOGI(TAG, "[System] Memory: %ld bytes free", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(30000));  // Low frequency system monitoring
    }
}