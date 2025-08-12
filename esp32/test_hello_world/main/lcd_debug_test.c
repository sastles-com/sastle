/**
 * @file lcd_debug_test.c
 * @brief LCD debug test with detailed logging
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LCD_DEBUG";

// Test different pin configurations
typedef struct {
    const char* name;
    int mosi;
    int sclk;
    int cs;
    int dc;
    int rst;
} pin_config_t;

// Multiple pin configurations to test
static const pin_config_t pin_configs[] = {
    {"Config 1 (Original)", 21, 15, 14, 42, 48},
    {"Config 2 (Alternative)", 5, 6, 3, 4, 8},
    {"Config 3 (M5Stack common)", 23, 18, 14, 27, 33},
    {"Config 4 (ESP32S3 SPI2)", 11, 12, 10, 9, 8},
};

static spi_device_handle_t spi = NULL;

// Test SPI communication
static esp_err_t test_spi_communication(const pin_config_t *config) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing %s", config->name);
    ESP_LOGI(TAG, "MOSI=%d, SCLK=%d, CS=%d, DC=%d, RST=%d", 
             config->mosi, config->sclk, config->cs, config->dc, config->rst);
    
    // Clean up previous SPI if exists
    if (spi) {
        spi_bus_remove_device(spi);
        spi = NULL;
        spi_bus_free(SPI2_HOST);
    }
    
    // Configure GPIO
    ESP_LOGI(TAG, "Configuring GPIO pins...");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->dc) | (1ULL << config->rst),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus...");
    spi_bus_config_t buscfg = {
        .mosi_io_num = config->mosi,
        .miso_io_num = -1,
        .sclk_io_num = config->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 100000,  // Very slow for testing - 100kHz
        .mode = 0,
        .spics_io_num = config->cs,
        .queue_size = 7,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return ret;
    }
    
    ESP_LOGI(TAG, "SPI configured successfully!");
    
    // Test reset sequence
    ESP_LOGI(TAG, "Testing RST pin toggle...");
    gpio_set_level(config->rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(config->rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "RST pin toggled");
    
    // Test DC pin
    ESP_LOGI(TAG, "Testing DC pin toggle...");
    gpio_set_level(config->dc, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(config->dc, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(config->dc, 0);
    ESP_LOGI(TAG, "DC pin toggled");
    
    // Test SPI transmission
    ESP_LOGI(TAG, "Testing SPI data transmission...");
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    spi_transaction_t t = {
        .length = sizeof(test_data) * 8,
        .tx_buffer = test_data,
    };
    
    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPI transmit successful!");
    }
    
    return ret;
}

// Test all GPIO pins to find working ones
static void test_all_gpio_pins(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing all GPIO pins for output capability");
    ESP_LOGI(TAG, "==========================================");
    
    // Test GPIOs that are typically available on ESP32S3
    int test_pins[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 
                       21, 38, 39, 40, 41, 42, 45, 46, 47, 48};
    
    for (int i = 0; i < sizeof(test_pins)/sizeof(test_pins[0]); i++) {
        int pin = test_pins[i];
        
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        
        esp_err_t ret = gpio_config(&io_conf);
        if (ret == ESP_OK) {
            // Test toggle
            gpio_set_level(pin, 0);
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(pin, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
            gpio_set_level(pin, 0);
            
            ESP_LOGI(TAG, "GPIO %d: OK - Can be used for output", pin);
        } else {
            ESP_LOGW(TAG, "GPIO %d: FAILED - %s", pin, esp_err_to_name(ret));
        }
    }
}

// Try to detect any LCD response
static void probe_lcd_detection(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Probing for LCD detection");
    ESP_LOGI(TAG, "==========================================");
    
    // Try different SPI modes
    for (int mode = 0; mode < 4; mode++) {
        ESP_LOGI(TAG, "Testing SPI Mode %d", mode);
        
        // Clean up previous SPI
        if (spi) {
            spi_bus_remove_device(spi);
            spi = NULL;
            spi_bus_free(SPI2_HOST);
        }
        
        // Initialize with different mode
        spi_bus_config_t buscfg = {
            .mosi_io_num = 21,
            .miso_io_num = -1,
            .sclk_io_num = 15,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        
        if (spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) == ESP_OK) {
            spi_device_interface_config_t devcfg = {
                .clock_speed_hz = 100000,
                .mode = mode,
                .spics_io_num = 14,
                .queue_size = 1,
            };
            
            if (spi_bus_add_device(SPI2_HOST, &devcfg, &spi) == ESP_OK) {
                // Send test pattern
                uint8_t cmd = 0x00;  // NOP or read ID command
                spi_transaction_t t = {
                    .length = 8,
                    .tx_buffer = &cmd,
                };
                
                if (spi_device_polling_transmit(spi, &t) == ESP_OK) {
                    ESP_LOGI(TAG, "Mode %d: Transmission OK", mode);
                } else {
                    ESP_LOGW(TAG, "Mode %d: Transmission failed", mode);
                }
            }
        }
    }
}

// Main debug test
esp_err_t run_lcd_debug_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R LCD Debug Test");
    ESP_LOGI(TAG, "==========================================");
    
    // Test 1: Test all GPIO pins
    test_all_gpio_pins();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 2: Try different pin configurations
    for (int i = 0; i < sizeof(pin_configs)/sizeof(pin_configs[0]); i++) {
        test_spi_communication(&pin_configs[i]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Test 3: Probe for LCD detection
    probe_lcd_detection();
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Debug test complete!");
    ESP_LOGI(TAG, "Check the logs above to identify issues");
    ESP_LOGI(TAG, "==========================================");
    
    // Clean up
    if (spi) {
        spi_bus_remove_device(spi);
        spi_bus_free(SPI2_HOST);
    }
    
    return ESP_OK;
}