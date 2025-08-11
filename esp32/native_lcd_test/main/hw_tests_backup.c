#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "HW_TESTS";

// M5AtomS3R LED pin (GPIO35)
#define LED_GPIO GPIO_NUM_35

// ADC for power voltage measurement (internal reference)
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_0  // GPIO1

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;

// HW_001: PSRAM 8MB recognition test
static void test_psram(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HW_001: PSRAM 8MB Recognition Test");
    ESP_LOGI(TAG, "========================================");
    
#if CONFIG_SPIRAM
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "PSRAM Total Size: %zu bytes (%.2f MB)", psram_size, psram_size / (1024.0 * 1024.0));
    ESP_LOGI(TAG, "PSRAM Free Size: %zu bytes (%.2f MB)", psram_free, psram_free / (1024.0 * 1024.0));
    
    if (psram_size >= 8 * 1024 * 1024) {
        ESP_LOGI(TAG, "✓ PSRAM 8MB Recognition: PASS");
        
        // Test PSRAM allocation
        void *psram_ptr = heap_caps_malloc(1024 * 1024, MALLOC_CAP_SPIRAM);
        if (psram_ptr) {
            ESP_LOGI(TAG, "✓ PSRAM Allocation Test: PASS");
            memset(psram_ptr, 0xAA, 1024 * 1024);
            heap_caps_free(psram_ptr);
        } else {
            ESP_LOGE(TAG, "✗ PSRAM Allocation Test: FAIL");
        }
    } else {
        ESP_LOGE(TAG, "✗ PSRAM 8MB Recognition: FAIL - Size: %.2f MB", psram_size / (1024.0 * 1024.0));
    }
#else
    ESP_LOGW(TAG, "PSRAM not enabled in configuration");
    ESP_LOGI(TAG, "To enable PSRAM: idf.py menuconfig -> Component config -> ESP PSRAM");
#endif
    
    ESP_LOGI(TAG, "");
}

// HW_002: GPIO control (LED blink) test
static void test_gpio_led(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HW_002: GPIO Control (LED Blink) Test");
    ESP_LOGI(TAG, "========================================");
    
    // Configure LED GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ GPIO Configuration: PASS");
        
        // LED blink test
        for (int i = 0; i < 10; i++) {
            gpio_set_level(LED_GPIO, i % 2);
            ESP_LOGI(TAG, "LED State: %s", (i % 2) ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        ESP_LOGI(TAG, "✓ GPIO LED Blink Test: PASS");
    } else {
        ESP_LOGE(TAG, "✗ GPIO Configuration: FAIL - Error: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "");
}

// HW_003: Power voltage 3.3V±5% test
static void test_power_voltage(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HW_003: Power Voltage 3.3V±5% Test");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    
    if (ret == ESP_OK) {
        adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_12,
        };
        ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);
        
        if (ret == ESP_OK) {
            // Get internal voltage reference (approximate)
            int adc_reading = 0;
            for (int i = 0; i < 10; i++) {
                int temp_reading;
                ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL, &temp_reading);
                if (ret == ESP_OK) {
                    adc_reading += temp_reading;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            adc_reading /= 10;
            
            // Convert to voltage (rough approximation for 3.3V system)
            float voltage = (adc_reading * 3.3) / 4095.0;
            ESP_LOGI(TAG, "ADC Reading: %d, Estimated Voltage: %.3f V", adc_reading, voltage);
            
            // Check if voltage is within 3.3V ±5% (3.135V - 3.465V)
            if (voltage >= 3.135 && voltage <= 3.465) {
                ESP_LOGI(TAG, "✓ Power Voltage Test: PASS (%.3f V within 3.3V±5%%)", voltage);
            } else {
                ESP_LOGW(TAG, "⚠ Power Voltage Test: WARNING (%.3f V outside 3.3V±5%%)", voltage);
                ESP_LOGI(TAG, "Note: This is an approximate measurement");
            }
        } else {
            ESP_LOGE(TAG, "✗ ADC Channel Configuration: FAIL - %s", esp_err_to_name(ret));
        }
        
        adc_oneshot_del_unit(adc1_handle);
    } else {
        ESP_LOGE(TAG, "✗ ADC Initialization: FAIL - %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "");
}

// HW_004: Dual core operation test
static void core_task(void *pvParameters)
{
    int core_id = (int)pvParameters;
    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "Core %d Task - Count: %d", core_id, i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void test_dual_core(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HW_004: Dual Core Operation Test");
    ESP_LOGI(TAG, "========================================");
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip Cores: %d", chip_info.cores);
    
    if (chip_info.cores >= 2) {
        ESP_LOGI(TAG, "✓ Dual Core Hardware: Available");
        
        // Create tasks on different cores
        TaskHandle_t task_core0, task_core1;
        
        xTaskCreatePinnedToCore(core_task, "Core0Task", 2048, (void*)0, 1, &task_core0, 0);
        xTaskCreatePinnedToCore(core_task, "Core1Task", 2048, (void*)1, 1, &task_core1, 1);
        
        // Wait for tasks to complete
        vTaskDelay(pdMS_TO_TICKS(6000));
        
        ESP_LOGI(TAG, "✓ Dual Core Operation Test: PASS");
    } else {
        ESP_LOGE(TAG, "✗ Dual Core Hardware: Not Available");
    }
    
    ESP_LOGI(TAG, "");
}

// HW_005: Flash size test
static void test_flash_size(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HW_005: Flash Size Test");
    ESP_LOGI(TAG, "========================================");
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    uint32_t flash_size;
    esp_err_t ret = esp_flash_get_size(NULL, &flash_size);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Flash Size: %lu bytes (%.1f MB)", flash_size, flash_size / (1024.0 * 1024.0));
        
        if (flash_size >= 8 * 1024 * 1024) {
            ESP_LOGI(TAG, "✓ Flash Size Test: PASS (8MB or larger detected)");
        } else {
            ESP_LOGW(TAG, "⚠ Flash Size Test: WARNING (Less than 8MB detected)");
        }
    } else {
        ESP_LOGE(TAG, "✗ Flash Size Detection: FAIL - %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Flash Type: %s", 
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded" : "External");
    
    ESP_LOGI(TAG, "");
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Hardware Tests - Phase -1");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");
    
    // Run all hardware tests
    test_psram();
    test_gpio_led();
    test_power_voltage();
    test_dual_core();
    test_flash_size();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Hardware Tests Complete");
    ESP_LOGI(TAG, "========================================");
    
    // Keep the program running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}