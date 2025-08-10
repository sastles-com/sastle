#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"

static const char *TAG = "HELLO_TEST";

void print_chip_info(void) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32S3R Isolation Sphere Test");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Chip: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Features:%s%s%s%s",
            (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? " WiFi" : "",
            (chip_info.features & CHIP_FEATURE_BT) ? " BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? " BLE" : "",
            (chip_info.features & CHIP_FEATURE_IEEE802154) ? " 802.15.4" : "");
    ESP_LOGI(TAG, "Silicon revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "Flash size: %dMB %s",
            spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}

void print_memory_info(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Memory Information");
    ESP_LOGI(TAG, "========================================");
    
    // Internal RAM
    ESP_LOGI(TAG, "Internal RAM:");
    ESP_LOGI(TAG, "  Total: %d bytes", heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  Free: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  Largest free block: %d bytes", 
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    
    // PSRAM check
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM:");
        ESP_LOGI(TAG, "  Status: Initialized ✓");
        ESP_LOGI(TAG, "  Total: %d bytes (%.2f MB)", 
                esp_psram_get_size(), 
                esp_psram_get_size() / (1024.0 * 1024.0));
        ESP_LOGI(TAG, "  Free: %d bytes", 
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI(TAG, "  Largest free block: %d bytes", 
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        
        // PSRAM test allocation
        void *test_ptr = heap_caps_malloc(1024 * 1024, MALLOC_CAP_SPIRAM);
        if (test_ptr != NULL) {
            ESP_LOGI(TAG, "  Test: 1MB allocation successful ✓");
            heap_caps_free(test_ptr);
        } else {
            ESP_LOGE(TAG, "  Test: 1MB allocation failed ✗");
        }
    } else {
        ESP_LOGW(TAG, "PSRAM: Not initialized ✗");
        ESP_LOGW(TAG, "Check menuconfig: Component config → ESP PSRAM");
    }
}

void print_task_info(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Task Information");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Current task: %s", pcTaskGetName(NULL));
    ESP_LOGI(TAG, "Running on core: %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Priority: %d", uxTaskPriorityGet(NULL));
    ESP_LOGI(TAG, "Stack high water mark: %d bytes", 
            uxTaskGetStackHighWaterMark(NULL));
}

void test_task_core0(void *pvParameters) {
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "[Core 0] Task iteration %d", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void test_task_core1(void *pvParameters) {
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "[Core 1] Task iteration %d", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    // Print system information
    print_chip_info();
    print_memory_info();
    print_task_info();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Multi-Core Test");
    ESP_LOGI(TAG, "========================================");
    
    // Create test tasks on different cores
    TaskHandle_t task0_handle = NULL;
    TaskHandle_t task1_handle = NULL;
    
    BaseType_t ret;
    ret = xTaskCreatePinnedToCore(
        test_task_core0,           // Task function
        "test_core0",               // Task name
        2048,                       // Stack size
        NULL,                       // Parameters
        10,                         // Priority
        &task0_handle,              // Task handle
        0                           // Core ID
    );
    
    if (ret == pdPASS) {
        ESP_LOGI(TAG, "Task created on Core 0 ✓");
    } else {
        ESP_LOGE(TAG, "Failed to create task on Core 0 ✗");
    }
    
    ret = xTaskCreatePinnedToCore(
        test_task_core1,
        "test_core1",
        2048,
        NULL,
        10,
        &task1_handle,
        1
    );
    
    if (ret == pdPASS) {
        ESP_LOGI(TAG, "Task created on Core 1 ✓");
    } else {
        ESP_LOGE(TAG, "Failed to create task on Core 1 ✗");
    }
    
    // Main loop
    int counter = 0;
    while (1) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Main loop iteration: %d", ++counter);
        ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
        ESP_LOGI(TAG, "Minimum free heap: %d bytes", esp_get_minimum_free_heap_size());
        ESP_LOGI(TAG, "========================================");
        
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (counter >= 10) {
            ESP_LOGI(TAG, "Test completed successfully! ✓");
            ESP_LOGI(TAG, "System is ready for development.");
            break;
        }
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Environment test finished.");
    ESP_LOGI(TAG, "========================================");
}