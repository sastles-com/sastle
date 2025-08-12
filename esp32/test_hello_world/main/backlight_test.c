/**
 * @file backlight_test.c
 * @brief M5AtomS3R backlight control test
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "BACKLIGHT_TEST";

// Test different backlight pins (GPIO32 removed due to WDT reset issue)
static const int backlight_pins[] = {16, 46, 47, 48, 15, 2, 4, 5};
static const int num_pins = sizeof(backlight_pins) / sizeof(backlight_pins[0]);

// Test GPIO output for backlight
static void test_backlight_gpio(int pin) {
    ESP_LOGI(TAG, "Testing GPIO %d as backlight pin", pin);
    
    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d config failed: %s", pin, esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "GPIO %d configured, testing HIGH signal...", pin);
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "GPIO %d testing LOW signal...", pin);
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "GPIO %d back to HIGH...", pin);
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// Test PWM control for backlight
static void test_backlight_pwm(int pin) {
    ESP_LOGI(TAG, "Testing GPIO %d with PWM control", pin);
    
    // Configure LEDC for PWM
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,  // 1kHz
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
    };
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "PWM configured for GPIO %d", pin);
    
    // Test different PWM duty cycles
    int duty_levels[] = {0, 256, 512, 768, 1023};  // 0%, 25%, 50%, 75%, 100%
    
    for (int i = 0; i < 5; i++) {
        int duty = duty_levels[i];
        ESP_LOGI(TAG, "Setting PWM duty to %d (%d%%)", duty, duty * 100 / 1023);
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Stop PWM and cleanup
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

// Test all possible backlight control methods
static void comprehensive_backlight_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Comprehensive Backlight Test");
    ESP_LOGI(TAG, "==========================================");
    
    for (int i = 0; i < num_pins; i++) {
        int pin = backlight_pins[i];
        
        ESP_LOGI(TAG, "==========================================");
        ESP_LOGI(TAG, "Testing GPIO %d", pin);
        ESP_LOGI(TAG, "==========================================");
        
        // Test 1: Simple GPIO HIGH/LOW
        ESP_LOGI(TAG, "Phase 1: Simple GPIO control");
        test_backlight_gpio(pin);
        
        // Test 2: PWM control
        ESP_LOGI(TAG, "Phase 2: PWM control");
        test_backlight_pwm(pin);
        
        ESP_LOGI(TAG, "Completed testing GPIO %d", pin);
        ESP_LOGI(TAG, "Check if backlight responded during this test");
        ESP_LOGI(TAG, "------------------------------------------");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Test multiple pins simultaneously
static void test_multiple_pins_simultaneously(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing Multiple Pins Simultaneously");
    ESP_LOGI(TAG, "==========================================");
    
    // Configure all pins as outputs
    uint64_t pin_mask = 0;
    for (int i = 0; i < num_pins; i++) {
        pin_mask |= (1ULL << backlight_pins[i]);
    }
    
    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Multi-pin config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "All pins configured, setting all HIGH...");
    for (int i = 0; i < num_pins; i++) {
        gpio_set_level(backlight_pins[i], 1);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "Setting all LOW...");
    for (int i = 0; i < num_pins; i++) {
        gpio_set_level(backlight_pins[i], 0);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "Setting all HIGH again...");
    for (int i = 0; i < num_pins; i++) {
        gpio_set_level(backlight_pins[i], 1);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
}

// Main backlight test function
esp_err_t run_backlight_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R Backlight Test");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing pins: ");
    for (int i = 0; i < num_pins; i++) {
        printf("GPIO%d ", backlight_pins[i]);
    }
    printf("\n");
    ESP_LOGI(TAG, "==========================================");
    
    // Test 1: Individual pin testing
    comprehensive_backlight_test();
    
    // Test 2: Multiple pins simultaneously
    test_multiple_pins_simultaneously();
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Backlight test complete!");
    ESP_LOGI(TAG, "If backlight responded, note which GPIO worked");
    ESP_LOGI(TAG, "==========================================");
    
    return ESP_OK;
}