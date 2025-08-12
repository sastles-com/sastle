#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "m5atoms3r_lcd.h"

static const char *TAG = "SIMPLE_TEST";

// Test LCD functionality
esp_err_t test_lcd_basic(void)
{
    static m5atoms3r_lcd_t lcd;
    
    ESP_LOGI(TAG, "===== BASIC LCD TEST =====");
    ESP_LOGI(TAG, "Testing LCD initialization...");
    esp_err_t ret = m5atoms3r_lcd_init(&lcd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "LCD initialized successfully!");
    ESP_LOGI(TAG, "Starting visual test sequence...");
    
    // Test 1: Fill entire screen with RED
    ESP_LOGI(TAG, "Test 1: Fill screen RED - Look at LCD now!");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test 2: Fill entire screen with GREEN
    ESP_LOGI(TAG, "Test 2: Fill screen GREEN - Look at LCD now!");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test 3: Fill entire screen with BLUE
    ESP_LOGI(TAG, "Test 3: Fill screen BLUE - Look at LCD now!");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test 4: Fill entire screen with WHITE
    ESP_LOGI(TAG, "Test 4: Fill screen WHITE - Look at LCD now!");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_WHITE);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test 5: BLACK screen
    ESP_LOGI(TAG, "Test 5: Fill screen BLACK - Look at LCD now!");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 6: Draw colored rectangles
    ESP_LOGI(TAG, "Test 6: Drawing colored rectangles - Look at LCD now!");
    m5atoms3r_lcd_fill_rect(&lcd, 10, 10, 50, 50, LCD_COLOR_RED);
    m5atoms3r_lcd_fill_rect(&lcd, 70, 10, 50, 50, LCD_COLOR_GREEN);
    m5atoms3r_lcd_fill_rect(&lcd, 10, 70, 50, 50, LCD_COLOR_BLUE);
    m5atoms3r_lcd_fill_rect(&lcd, 70, 70, 50, 50, LCD_COLOR_YELLOW);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Test 7: Text display
    ESP_LOGI(TAG, "Test 7: Text display - Look at LCD now!");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    m5atoms3r_lcd_printf(&lcd, 20, 50, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2, "TEST OK");
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    ESP_LOGI(TAG, "===== LCD TEST COMPLETED =====");
    ESP_LOGI(TAG, "If you saw colors and text, LCD is working!");
    
    return ESP_OK;
}

// Main test function
esp_err_t run_simple_hardware_test(void)
{
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R LCD Basic Test");
    ESP_LOGI(TAG, "==========================================");
    
    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
    
    // Test LCD only
    ESP_LOGI(TAG, "Running basic LCD test...");
    esp_err_t ret = test_lcd_basic();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD test failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "LCD test completed successfully");
    ESP_LOGI(TAG, "==========================================");
    
    return ESP_OK;
}

// GPIO backlight test function
esp_err_t run_gpio_backlight_test(void)
{
    static m5atoms3r_lcd_t lcd;
    
    ESP_LOGI(TAG, "===== GPIO BACKLIGHT TEST (GPIO16) =====");
    ESP_LOGI(TAG, "Initializing LCD with GPIO16 backlight control...");
    
    esp_err_t ret = m5atoms3r_lcd_init(&lcd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "LCD initialized! Testing backlight control...");
    
    // Test 1: Display content first
    ESP_LOGI(TAG, "Test 1: Displaying test pattern...");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_WHITE);
    m5atoms3r_lcd_printf(&lcd, 20, 40, LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2, "GPIO16");
    m5atoms3r_lcd_printf(&lcd, 20, 60, LCD_COLOR_BLACK, LCD_COLOR_WHITE, 2, "TEST");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 2: Backlight ON/OFF toggle test
    ESP_LOGI(TAG, "Test 2: Backlight toggle test - watch the screen!");
    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "Turning backlight OFF (iteration %d/5)", i+1);
        m5atoms3r_lcd_set_backlight(false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Turning backlight ON (iteration %d/5)", i+1);
        m5atoms3r_lcd_set_backlight(true);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Test 3: PWM brightness test
    ESP_LOGI(TAG, "Test 3: PWM brightness control test...");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLUE);
    m5atoms3r_lcd_printf(&lcd, 10, 50, LCD_COLOR_WHITE, LCD_COLOR_BLUE, 2, "PWM TEST");
    
    // Fade from 0% to 100%
    ESP_LOGI(TAG, "Fading brightness 0%% -> 100%%");
    for (int brightness = 0; brightness <= 100; brightness += 10) {
        ESP_LOGI(TAG, "Setting brightness to %d%%", brightness);
        m5atoms3r_lcd_set_backlight_pwm(brightness);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Fade from 100% to 0%
    ESP_LOGI(TAG, "Fading brightness 100%% -> 0%%");
    for (int brightness = 100; brightness >= 0; brightness -= 10) {
        ESP_LOGI(TAG, "Setting brightness to %d%%", brightness);
        m5atoms3r_lcd_set_backlight_pwm(brightness);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Set back to full brightness
    ESP_LOGI(TAG, "Setting brightness back to 100%%");
    m5atoms3r_lcd_set_backlight_pwm(100);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Final success display
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_GREEN);
    m5atoms3r_lcd_printf(&lcd, 15, 40, LCD_COLOR_BLACK, LCD_COLOR_GREEN, 2, "SUCCESS!");
    m5atoms3r_lcd_printf(&lcd, 10, 60, LCD_COLOR_BLACK, LCD_COLOR_GREEN, 1, "GPIO16 working");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "===== GPIO BACKLIGHT TEST COMPLETED =====");
    ESP_LOGI(TAG, "If backlight turned on/off and brightness changed, GPIO16 is working!");
    
    return ESP_OK;
}