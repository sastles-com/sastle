#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "m5atoms3r_lcd.h"

static const char *TAG = "SIMPLE_TEST";

// M5AtomS3R I2C pins (Correct pins for M5AtomS3R)
#define I2C_MASTER_SDA_PIN      GPIO_NUM_2
#define I2C_MASTER_SCL_PIN      GPIO_NUM_1
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

// Forward declarations
esp_err_t test_bno055_direct(void);
esp_err_t test_i2c_pins_diagnostic(void);

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
    ESP_LOGI(TAG, "Test 1: Fill screen RED...");
    ret = m5atoms3r_lcd_clear(&lcd, LCD_COLOR_RED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Clear screen RED failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));  // 3 seconds
    
    // Test 2: Fill entire screen with GREEN
    ESP_LOGI(TAG, "Test 2: Fill screen GREEN...");
    ret = m5atoms3r_lcd_clear(&lcd, LCD_COLOR_GREEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Clear screen GREEN failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));  // 3 seconds
    
    // Test 3: Fill entire screen with BLUE
    ESP_LOGI(TAG, "Test 3: Fill screen BLUE...");
    ret = m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLUE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Clear screen BLUE failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));  // 3 seconds
    
    // Test 4: Fill entire screen with WHITE
    ESP_LOGI(TAG, "Test 4: Fill screen WHITE...");
    ret = m5atoms3r_lcd_clear(&lcd, LCD_COLOR_WHITE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Clear screen WHITE failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));  // 3 seconds
    
    // Test 5: Clear to BLACK and draw simple patterns
    ESP_LOGI(TAG, "Test 5: Draw patterns on BLACK...");
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    
    // Draw large rectangles for visibility
    ESP_LOGI(TAG, "Drawing large colored rectangles...");
    m5atoms3r_lcd_fill_rect(&lcd, 5, 5, 40, 40, LCD_COLOR_RED);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    m5atoms3r_lcd_fill_rect(&lcd, 50, 5, 40, 40, LCD_COLOR_GREEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    m5atoms3r_lcd_fill_rect(&lcd, 5, 50, 40, 40, LCD_COLOR_BLUE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    m5atoms3r_lcd_fill_rect(&lcd, 50, 50, 40, 40, LCD_COLOR_YELLOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 6: Text display
    ESP_LOGI(TAG, "Test 6: Text display...");
    m5atoms3r_lcd_printf(&lcd, 10, 100, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2, "LCD OK!");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "===== LCD TEST COMPLETED =====");
    ESP_LOGI(TAG, "If you can see colored squares and text, LCD is working!");
    
    return ESP_OK;
}

// Test I2C bus scanning
esp_err_t test_i2c_scan(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus...");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized. Scanning for devices...");
    ESP_LOGI(TAG, "Scanning I2C addresses 0x08 to 0x77...");
    ESP_LOGI(TAG, "");

    int devices_found = 0;
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    
    for (int row = 0; row < 8; row++) {
        char line[80];
        int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "%02x: ", row * 16);
        
        for (int col = 0; col < 16; col++) {
            uint8_t addr = row * 16 + col;
            
            if (addr < 0x08 || addr > 0x77) {
                pos += snprintf(line + pos, sizeof(line) - pos, "   ");
                continue;
            }
            
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | 0x00, true);
            i2c_master_stop(cmd);
            
            ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                pos += snprintf(line + pos, sizeof(line) - pos, "%02x ", addr);
                devices_found++;
                
                // Log specific device types
                if (addr == 0x28 || addr == 0x29) {
                    ESP_LOGI(TAG, ">>> BNO055 IMU detected at 0x%02X <<<", addr);
                } else if (addr >= 0x48 && addr <= 0x4F) {
                    ESP_LOGI(TAG, ">>> Possible temp sensor at 0x%02X <<<", addr);
                } else if (addr == 0x3C || addr == 0x3D) {
                    ESP_LOGI(TAG, ">>> Possible OLED display at 0x%02X <<<", addr);
                } else {
                    ESP_LOGI(TAG, ">>> Unknown device at 0x%02X <<<", addr);
                }
            } else {
                pos += snprintf(line + pos, sizeof(line) - pos, "-- ");
            }
        }
        ESP_LOGI(TAG, "%s", line);
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "I2C scan completed. Found %d devices.", devices_found);
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices detected!");
        ESP_LOGW(TAG, "Check:");
        ESP_LOGW(TAG, "  - Device connections (SDA: GPIO2, SCL: GPIO1)");
        ESP_LOGW(TAG, "  - Pull-up resistors (4.7kΩ)");
        ESP_LOGW(TAG, "  - Device power supply (3.3V)");
        ESP_LOGW(TAG, "  - Device addresses and compatibility");
    }
    
    // Test BNO055 specifically
    ESP_LOGI(TAG, "Testing BNO055 communication...");
    ret = test_bno055_direct();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 direct test failed");
    }
    
    // I2C pins diagnostic
    ESP_LOGI(TAG, "Running I2C pins diagnostic...");
    test_i2c_pins_diagnostic();
    
    return ESP_OK;
}

// Direct BNO055 test
esp_err_t test_bno055_direct(void)
{
    ESP_LOGI(TAG, "Testing BNO055 direct communication...");
    
    uint8_t bno055_addr = 0x28; // BNO055_I2C_ADDR_A
    
    // Try to read chip ID register (0x00)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bno055_addr << 1) | 0x00, true); // Write
    i2c_master_write_byte(cmd, 0x00, true); // Register address (Chip ID)
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (bno055_addr << 1) | 0x01, true); // Read
    
    uint8_t chip_id;
    i2c_master_read_byte(cmd, &chip_id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BNO055 Chip ID: 0x%02X (Expected: 0xA0)", chip_id);
        if (chip_id == 0xA0) {
            ESP_LOGI(TAG, "  >>> BNO055 chip ID verified! <<<");
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "  >>> Unexpected chip ID <<<");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Failed to read BNO055 chip ID: %s", esp_err_to_name(ret));
        return ret;
    }
}

// I2C pins diagnostic
esp_err_t test_i2c_pins_diagnostic(void)
{
    ESP_LOGI(TAG, "=== I2C Pins Diagnostic ===");
    ESP_LOGI(TAG, "SDA Pin: GPIO%d", I2C_MASTER_SDA_PIN);
    ESP_LOGI(TAG, "SCL Pin: GPIO%d", I2C_MASTER_SCL_PIN);
    ESP_LOGI(TAG, "I2C Port: %d", I2C_MASTER_PORT);
    ESP_LOGI(TAG, "I2C Frequency: %d Hz", I2C_MASTER_FREQ_HZ);
    
    // Test GPIO pin states
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << I2C_MASTER_SDA_PIN) | (1ULL << I2C_MASTER_SCL_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read pin states
    int sda_level = gpio_get_level(I2C_MASTER_SDA_PIN);
    int scl_level = gpio_get_level(I2C_MASTER_SCL_PIN);
    
    ESP_LOGI(TAG, "GPIO States (with internal pullup):");
    ESP_LOGI(TAG, "  SDA (GPIO%d): %s", I2C_MASTER_SDA_PIN, sda_level ? "HIGH" : "LOW");
    ESP_LOGI(TAG, "  SCL (GPIO%d): %s", I2C_MASTER_SCL_PIN, scl_level ? "HIGH" : "LOW");
    
    if (sda_level == 0 || scl_level == 0) {
        ESP_LOGW(TAG, "WARNING: One or both I2C lines are LOW!");
        ESP_LOGW(TAG, "This may indicate:");
        ESP_LOGW(TAG, "  - Missing pull-up resistors");
        ESP_LOGW(TAG, "  - Device holding line low");
        ESP_LOGW(TAG, "  - Short circuit to ground");
    } else {
        ESP_LOGI(TAG, "I2C lines are HIGH (good for communication)");
    }
    
    // Test different I2C frequencies
    ESP_LOGI(TAG, "Testing different I2C frequencies...");
    uint32_t test_freqs[] = {50000, 100000, 200000, 400000};
    const char* freq_names[] = {"50kHz", "100kHz", "200kHz", "400kHz"};
    
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Testing at %s (%lu Hz)...", freq_names[i], test_freqs[i]);
        
        // Reconfigure I2C with different frequency
        i2c_driver_delete(I2C_MASTER_PORT);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_PIN,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = test_freqs[i],
        };
        
        ret = i2c_param_config(I2C_MASTER_PORT, &conf);
        if (ret != ESP_OK) continue;
        
        ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
        if (ret != ESP_OK) continue;
        
        // Quick scan for any device
        bool device_found = false;
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | 0x00, true);
            i2c_master_stop(cmd);
            
            esp_err_t scan_ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(10));
            i2c_cmd_link_delete(cmd);
            
            if (scan_ret == ESP_OK) {
                device_found = true;
                break;
            }
        }
        
        ESP_LOGI(TAG, "  %s: %s", freq_names[i], device_found ? "Device detected" : "No devices");
    }
    
    // Restore original frequency
    i2c_driver_delete(I2C_MASTER_PORT);
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    
    ESP_LOGI(TAG, "=== I2C Diagnostic Complete ===");
    return ESP_OK;
}

// Main test function
esp_err_t run_simple_hardware_test(void)
{
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R LCD Only Test");
    ESP_LOGI(TAG, "==========================================");
    
    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
    
    // Test LCD only - focused testing
    ESP_LOGI(TAG, "Running focused LCD test...");
    esp_err_t ret = test_lcd_basic();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD test failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "LCD-only test completed");
    ESP_LOGI(TAG, "Check the display for visual confirmation");
    ESP_LOGI(TAG, "==========================================");
    
    // Keep the display visible for observation
    ESP_LOGI(TAG, "Test completed. System will continue running...");
    
    return ESP_OK;
}