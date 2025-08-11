#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
#include "config_manager.h"

static const char *TAG = "HELLO_TEST";

// PSRAM test function declaration
void run_psram_tests(void);

// Config management test function
void test_config_management(void);

// GPIO test function declaration
void run_gpio_tests(void);

// I2C and IMU test function declaration
void run_i2c_imu_tests(void);
void cleanup_i2c_imu_tests(void);

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
    // Get flash size using newer ESP-IDF v6.0 API
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash size: %dMB %s",
            flash_size / (1024 * 1024),
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
    
#if CONFIG_SPIRAM
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
#else
    ESP_LOGI(TAG, "PSRAM: Disabled in configuration");
#endif
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

void test_config_management(void) {
    ESP_LOGI(TAG, "Initializing SPIFFS configuration manager...");
    
    esp_err_t ret = config_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize config manager: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "✓ Config manager initialized successfully");
    
    // 現在の設定を表示
    config_print_current();
    
    // 設定値の取得テスト
    const sphere_config_t *config = config_get_current();
    if (config) {
        ESP_LOGI(TAG, "Configuration access test:");
        ESP_LOGI(TAG, "  Image: %dx%d @ %d fps", config->image.width, config->image.height, config->image.fps);
        ESP_LOGI(TAG, "  LED count: %d, brightness: %d", config->led.count, config->led.brightness);
        ESP_LOGI(TAG, "  IMU sample rate: %d Hz", config->imu.sample_rate_hz);
    }
    
    // 設定更新テスト
    ESP_LOGI(TAG, "Testing configuration update...");
    ret = config_update_image_settings(640, 480, 15, 85);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Image settings updated successfully");
        config_print_current();
    } else {
        ESP_LOGE(TAG, "✗ Failed to update image settings");
    }
    
    // 設定検証テスト
    config = config_get_current();
    ret = config_validate(config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Configuration validation passed");
    } else {
        ESP_LOGE(TAG, "✗ Configuration validation failed");
    }
    
    ESP_LOGI(TAG, "SPIFFS configuration management test completed");
}

void app_main(void) {
    // Print system information
    print_chip_info();
    print_memory_info();
    print_task_info();
    
    // Test SPIFFS Configuration Management
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SPIFFS Configuration Management Test");
    ESP_LOGI(TAG, "========================================");
    test_config_management();
    
    // Run PSRAM comprehensive tests
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "PSRAM Comprehensive Test");
    ESP_LOGI(TAG, "========================================");
    run_psram_tests();
    
    // Run GPIO comprehensive tests
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GPIO Hardware Test");
    ESP_LOGI(TAG, "========================================");
    run_gpio_tests();
    
    // Run I2C and IMU comprehensive tests
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C and IMU Test");
    ESP_LOGI(TAG, "========================================");
    run_i2c_imu_tests();
    
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
            
            // Cleanup all test resources before finishing
            ESP_LOGI(TAG, "Cleaning up test resources...");
            cleanup_i2c_imu_tests();
            
            break;
        }
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Environment test finished.");
    ESP_LOGI(TAG, "========================================");
}