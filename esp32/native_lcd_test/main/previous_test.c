#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BACKLIGHT_TEST";

// M5AtomS3R potential backlight pins to test
#define BACKLIGHT_PIN_16    16
#define BACKLIGHT_PIN_33    33
#define BACKLIGHT_PIN_5     5
#define BACKLIGHT_PIN_46    46

void test_gpio_pin(int pin) {
    ESP_LOGI(TAG, "Testing GPIO %d", pin);
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", pin, esp_err_to_name(ret));
        return;
    }
    
    // Test HIGH (5 seconds)
    ESP_LOGI(TAG, "GPIO %d -> HIGH for 5 seconds", pin);
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test LOW (2 seconds)  
    ESP_LOGI(TAG, "GPIO %d -> LOW for 2 seconds", pin);
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test PWM-like blinking (10 times)
    ESP_LOGI(TAG, "GPIO %d -> Blinking 10 times", pin);
    for (int i = 0; i < 10; i++) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "GPIO %d test complete", pin);
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Simple Backlight Pin Test");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "This test will try different GPIO pins");
    ESP_LOGI(TAG, "Watch the LCD screen for backlight changes");
    ESP_LOGI(TAG, "========================================");
    
    int pins_to_test[] = {BACKLIGHT_PIN_16, BACKLIGHT_PIN_33, BACKLIGHT_PIN_5, BACKLIGHT_PIN_46};
    int pin_count = sizeof(pins_to_test) / sizeof(pins_to_test[0]);
    
    while (1) {
        for (int i = 0; i < pin_count; i++) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "==== Testing Pin %d (attempt %d) ====", pins_to_test[i], i+1);
            test_gpio_pin(pins_to_test[i]);
            
            // Wait between tests
            ESP_LOGI(TAG, "Waiting 3 seconds before next test...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "All pins tested. Repeating in 10 seconds...");
        ESP_LOGI(TAG, "");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}