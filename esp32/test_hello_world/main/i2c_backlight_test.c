/**
 * @file i2c_backlight_test.c
 * @brief M5AtomS3R I2C-based backlight control test
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "I2C_BACKLIGHT_TEST";

// I2C configuration for M5AtomS3R backlight control
#define I2C_MASTER_SCL_IO           0       /*!< GPIO0 for I2C master clock */
#define I2C_MASTER_SDA_IO           45      /*!< GPIO45 for I2C master data */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ          100000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// M5AtomS3R backlight controller I2C address and registers
#define BACKLIGHT_I2C_ADDR          0x30    /*!< I2C address 48 (0x30) */
#define BACKLIGHT_REG_ENABLE        0x00    /*!< Enable register */
#define BACKLIGHT_REG_CONFIG        0x08    /*!< Configuration register */  
#define BACKLIGHT_REG_MISC          0x70    /*!< Miscellaneous register */
#define BACKLIGHT_REG_BRIGHTNESS    0x0e    /*!< Brightness control register */

// Initialize I2C master
static esp_err_t i2c_master_init(void) {
    ESP_LOGI(TAG, "Initializing I2C master...");
    ESP_LOGI(TAG, "SCL: GPIO%d, SDA: GPIO%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// Write to I2C register
static esp_err_t i2c_write_register(uint8_t reg_addr, uint8_t data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BACKLIGHT_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write to reg 0x%02x failed: %s", reg_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C write to reg 0x%02x = 0x%02x successful", reg_addr, data);
    }
    
    return ret;
}

// Read from I2C register
static esp_err_t i2c_read_register(uint8_t reg_addr, uint8_t *data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BACKLIGHT_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BACKLIGHT_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read from reg 0x%02x failed: %s", reg_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C read from reg 0x%02x = 0x%02x", reg_addr, *data);
    }
    
    return ret;
}

// Initialize backlight controller with extended sequence
static esp_err_t backlight_init(void) {
    ESP_LOGI(TAG, "Initializing M5AtomS3R backlight controller...");
    
    // Extended initialization sequence based on LP5562 chip
    
    // Step 1: Reset and enable chip (register 0x00 = 0x40)
    esp_err_t ret = i2c_write_register(0x00, 0x40);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Step 2: Configuration register (register 0x08 = 0x01) 
    ret = i2c_write_register(0x08, 0x01);
    if (ret != ESP_OK) return ret;
    
    // Step 3: Program configuration (register 0x70 = 0x00)
    ret = i2c_write_register(0x70, 0x00);
    if (ret != ESP_OK) return ret;
    
    // Step 4: Additional LP5562 initialization
    // LED mapping registers
    ret = i2c_write_register(0x09, 0x00); // LED1 map
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x0A, 0x00); // LED2 map  
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x0B, 0x00); // LED3 map
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x0C, 0x00); // LED4 map
    if (ret != ESP_OK) return ret;
    
    // Step 5: Enable outputs
    ret = i2c_write_register(0x00, 0x47); // Enable all outputs + chip enable
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Step 6: Set initial brightness to maximum for all LEDs
    ret = i2c_write_register(0x02, 0xFF); // LED1 brightness (R)
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x03, 0xFF); // LED2 brightness (G)  
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x04, 0xFF); // LED3 brightness (B)
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x05, 0xFF); // LED4 brightness (W)
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Extended backlight controller initialized successfully");
    return ESP_OK;
}

// Set backlight brightness - use individual LED brightness registers
static esp_err_t backlight_set_brightness(uint8_t brightness) {
    ESP_LOGI(TAG, "Setting backlight brightness to %d", brightness);
    esp_err_t ret;
    
    // Set brightness for all 4 LED channels (R, G, B, W)
    ret = i2c_write_register(0x02, brightness); // LED1 (Red)
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x03, brightness); // LED2 (Green) 
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x04, brightness); // LED3 (Blue)
    if (ret != ESP_OK) return ret;
    
    ret = i2c_write_register(0x05, brightness); // LED4 (White)
    if (ret != ESP_OK) return ret;
    
    // Also try register 0x0e in case it's a global brightness
    ret = i2c_write_register(0x0e, brightness);
    
    return ret;
}

// Test I2C device detection
static void test_i2c_scan(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "I2C Device Scan");
    ESP_LOGI(TAG, "==========================================");
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at address 0x%02x (%d)", addr, addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan complete");
}

// Test backlight brightness levels
static void test_backlight_brightness_levels(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Testing Backlight Brightness Levels");
    ESP_LOGI(TAG, "==========================================");
    
    uint8_t brightness_levels[] = {0, 32, 64, 128, 192, 255};
    int num_levels = sizeof(brightness_levels) / sizeof(brightness_levels[0]);
    
    for (int i = 0; i < num_levels; i++) {
        uint8_t brightness = brightness_levels[i];
        ESP_LOGI(TAG, "Testing brightness level: %d (%d%%)", brightness, brightness * 100 / 255);
        
        esp_err_t ret = backlight_set_brightness(brightness);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set brightness to %d", brightness);
        } else {
            ESP_LOGI(TAG, "Brightness set to %d successfully", brightness);
        }
        
        vTaskDelay(pdMS_TO_TICKS(3000));  // Wait 3 seconds
    }
}

// Test register reading
static void test_register_read(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Reading Backlight Controller Registers");
    ESP_LOGI(TAG, "==========================================");
    
    uint8_t registers[] = {0x00, 0x08, 0x0e, 0x70};
    int num_regs = sizeof(registers) / sizeof(registers[0]);
    
    for (int i = 0; i < num_regs; i++) {
        uint8_t reg = registers[i];
        uint8_t value;
        esp_err_t ret = i2c_read_register(reg, &value);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Register 0x%02x = 0x%02x", reg, value);
        }
    }
}

// Main I2C backlight test function
esp_err_t run_i2c_backlight_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R I2C Backlight Test");
    ESP_LOGI(TAG, "==========================================");
    
    // Step 1: Initialize I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return ret;
    }
    
    // Step 2: Scan for I2C devices
    test_i2c_scan();
    
    // Step 3: Initialize backlight controller
    ret = backlight_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Backlight initialization failed");
        return ret;
    }
    
    // Step 4: Test register reading
    test_register_read();
    
    // Step 5: Test brightness levels
    test_backlight_brightness_levels();
    
    // Step 6: Set to maximum brightness and keep it on
    ESP_LOGI(TAG, "Setting backlight to maximum brightness (255)");
    backlight_set_brightness(255);
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "I2C Backlight Test Complete!");
    ESP_LOGI(TAG, "If working, LCD backlight should now be ON");
    ESP_LOGI(TAG, "==========================================");
    
    return ESP_OK;
}