#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
// #include "esp_psram/esp_psram.h"

static const char *TAG = "SIMPLE_TEST";

// Simple PSRAM test function
void simple_psram_test(void)
{
    ESP_LOGI(TAG, "=== Simple PSRAM Test ===");
    
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size > 0) {
        ESP_LOGI(TAG, "PSRAM found: %d bytes", psram_size);
        
        // Test simple allocation
        void *test_ptr = heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
        if (test_ptr) {
            ESP_LOGI(TAG, "PSRAM allocation successful: %p", test_ptr);
            heap_caps_free(test_ptr);
        } else {
            ESP_LOGE(TAG, "PSRAM allocation failed");
        }
    } else {
        ESP_LOGE(TAG, "PSRAM not found");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===== ESP32S3 Simple Test Start =====");
    
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
    
    // Heap information
    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM: %ld bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    // Simple PSRAM test
    simple_psram_test();
    
    ESP_LOGI(TAG, "===== Test Complete =====");
    
    // Main loop
    while(1) {
        ESP_LOGI(TAG, "System running... Free heap: %ld", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}