/**
 * @file gpio_backlight_test.c
 * @brief M5AtomS3R GPIO16 backlight control test
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "m5atoms3r_lcd.h"

static const char *TAG = "GPIO_BACKLIGHT_TEST";

// GPIO16 backlight control configuration
#define BACKLIGHT_GPIO      16
#define BACKLIGHT_PWM_FREQ  500     // 500Hz PWM frequency
#define BACKLIGHT_PWM_DUTY_RES LEDC_TIMER_10_BIT

// Initialize GPIO16 for backlight control
static esp_err_t backlight_gpio_init(void) {
    ESP_LOGI(TAG, "Initializing GPIO%d for backlight control", BACKLIGHT_GPIO);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BACKLIGHT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d config failed: %s", BACKLIGHT_GPIO, esp_err_to_name(ret));
        return ret;
    }
    
    // Start with backlight OFF
    gpio_set_level(BACKLIGHT_GPIO, 0);
    ESP_LOGI(TAG, "GPIO%d configured successfully for backlight control", BACKLIGHT_GPIO);
    
    return ESP_OK;
}

// Simple ON/OFF backlight control
static esp_err_t backlight_set_enable(bool enable) {
    ESP_LOGI(TAG, "Setting backlight %s", enable ? "ON" : "OFF");
    gpio_set_level(BACKLIGHT_GPIO, enable ? 1 : 0);
    return ESP_OK;
}

// Initialize PWM for backlight brightness control
static esp_err_t backlight_pwm_init(void) {
    ESP_LOGI(TAG, "Initializing PWM for brightness control");
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = BACKLIGHT_PWM_DUTY_RES,
        .freq_hz = BACKLIGHT_PWM_FREQ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BACKLIGHT_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
    };
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PWM configured successfully for GPIO%d", BACKLIGHT_GPIO);
    return ESP_OK;
}

// Set backlight brightness (0-100%)
static esp_err_t backlight_set_brightness(uint8_t brightness) {
    if (brightness > 100) {
        brightness = 100;
    }
    
    uint32_t duty = (brightness * 1023) / 100;  // Convert to 10-bit duty cycle
    ESP_LOGI(TAG, "Setting backlight brightness to %d%% (duty: %ld)", brightness, duty);
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC set duty failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC update duty failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

// Test basic ON/OFF functionality
static void test_backlight_on_off(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing Backlight ON/OFF Control");
    ESP_LOGI(TAG, "==========================================");
    
    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "Test %d/5: Turning backlight ON", i + 1);
        backlight_set_enable(true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Test %d/5: Turning backlight OFF", i + 1);
        backlight_set_enable(false);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Leave backlight ON
    ESP_LOGI(TAG, "Leaving backlight ON");
    backlight_set_enable(true);
}

// Test PWM brightness control
static void test_backlight_brightness(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing Backlight PWM Brightness Control");
    ESP_LOGI(TAG, "==========================================");
    
    // Initialize PWM
    esp_err_t ret = backlight_pwm_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM initialization failed, skipping brightness test");
        return;
    }
    
    // Fade from 0% to 100%
    ESP_LOGI(TAG, "Fading from 0%% to 100%%...");
    for (int brightness = 0; brightness <= 100; brightness += 5) {
        backlight_set_brightness(brightness);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Fade from 100% to 0%
    ESP_LOGI(TAG, "Fading from 100%% to 0%%...");
    for (int brightness = 100; brightness >= 0; brightness -= 5) {
        backlight_set_brightness(brightness);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Set to maximum brightness
    ESP_LOGI(TAG, "Setting to maximum brightness (100%%)");
    backlight_set_brightness(100);
}

// Test with LCD display
static void test_backlight_with_lcd(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing Backlight with LCD Display");
    ESP_LOGI(TAG, "==========================================");
    
    // Initialize LCD
    m5atoms3r_lcd_t lcd;
    esp_err_t ret = m5atoms3r_lcd_init(&lcd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Display test pattern
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    m5atoms3r_lcd_print_string(&lcd, 10, 20, "GPIO16 TEST", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1);
    m5atoms3r_lcd_print_string(&lcd, 10, 40, "Backlight", LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1);
    m5atoms3r_lcd_print_string(&lcd, 10, 60, "Control", LCD_COLOR_CYAN, LCD_COLOR_BLACK, 1);
    
    // Test with backlight control
    ESP_LOGI(TAG, "You should see the LCD display now");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Toggle backlight with display visible
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "Turning backlight OFF - display should go dark");
        backlight_set_enable(false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Turning backlight ON - display should be visible");
        backlight_set_enable(true);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Success display
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_GREEN);
    m5atoms3r_lcd_print_string(&lcd, 10, 30, "SUCCESS!", LCD_COLOR_WHITE, LCD_COLOR_GREEN, 2);
    m5atoms3r_lcd_print_string(&lcd, 10, 60, "GPIO16", LCD_COLOR_WHITE, LCD_COLOR_GREEN, 1);
    m5atoms3r_lcd_print_string(&lcd, 10, 80, "working", LCD_COLOR_WHITE, LCD_COLOR_GREEN, 1);
    
    ESP_LOGI(TAG, "Test complete - LCD should show success message");
}

// Main GPIO backlight test function
esp_err_t run_gpio_backlight_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R GPIO16 Backlight Test");
    ESP_LOGI(TAG, "==========================================");
    
    // Step 1: Initialize GPIO16 for backlight control
    esp_err_t ret = backlight_gpio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Backlight GPIO initialization failed");
        return ret;
    }
    
    // Step 2: Test basic ON/OFF functionality
    test_backlight_on_off();
    
    // Step 3: Test PWM brightness control
    test_backlight_brightness();
    
    // Step 4: Test with LCD display
    test_backlight_with_lcd();
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "GPIO16 Backlight Test Complete!");
    ESP_LOGI(TAG, "If working, you should have seen:");
    ESP_LOGI(TAG, "1. Backlight ON/OFF cycles");
    ESP_LOGI(TAG, "2. Brightness fade effects");
    ESP_LOGI(TAG, "3. LCD display with backlight control");
    ESP_LOGI(TAG, "==========================================");
    
    return ESP_OK;
}