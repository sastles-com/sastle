#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "m5atoms3r_lcd.h"
#include "simple_bno055.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "DIRECT_IMU_LCD";

// M5AtomS3R I2C pins
#define I2C_MASTER_SDA_PIN      GPIO_NUM_2
#define I2C_MASTER_SCL_PIN      GPIO_NUM_1
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

static m5atoms3r_lcd_t lcd_device;
static simple_bno055_t imu_device;

/**
 * @brief Direct IMU + LCD test without complex initialization
 */
esp_err_t run_direct_imu_lcd_test(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Direct IMU + LCD Test Starting");
    ESP_LOGI(TAG, "====================================");
    
    // Initialize LCD
    ESP_LOGI(TAG, "Initializing LCD...");
    esp_err_t ret = m5atoms3r_lcd_init(&lcd_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear LCD and show startup message
    m5atoms3r_lcd_clear(&lcd_device, LCD_COLOR_BLACK);
    m5atoms3r_lcd_printf(&lcd_device, 10, 10, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 2, "IMU TEST");
    m5atoms3r_lcd_printf(&lcd_device, 10, 40, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, "Initializing...");
    
    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        m5atoms3r_lcd_printf(&lcd_device, 10, 60, LCD_COLOR_RED, LCD_COLOR_BLACK, 1, "I2C Config FAIL");
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C install failed: %s", esp_err_to_name(ret));
        m5atoms3r_lcd_printf(&lcd_device, 10, 60, LCD_COLOR_RED, LCD_COLOR_BLACK, 1, "I2C Install FAIL");
        return ret;
    }
    
    // Scan I2C bus first
    ESP_LOGI(TAG, "Scanning I2C bus for BNO055...");
    m5atoms3r_lcd_printf(&lcd_device, 10, 60, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1, "Scan I2C bus...");
    
    bool bno055_found = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | 0x00, true);
        i2c_master_stop(cmd);
        
        esp_err_t scan_ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (scan_ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at 0x%02X", addr);
            if (addr == 0x28 || addr == 0x29) {
                bno055_found = true;
                ESP_LOGI(TAG, ">>> BNO055 detected at 0x%02X <<<", addr);
                break;
            }
        }
    }
    
    if (!bno055_found) {
        ESP_LOGE(TAG, "BNO055 not found on I2C bus!");
        m5atoms3r_lcd_printf(&lcd_device, 10, 80, LCD_COLOR_RED, LCD_COLOR_BLACK, 1, "BNO055 NOT FOUND");
        vTaskDelay(pdMS_TO_TICKS(5000));
        return ESP_ERR_NOT_FOUND;
    }
    
    // Initialize BNO055
    ESP_LOGI(TAG, "Initializing BNO055...");
    m5atoms3r_lcd_printf(&lcd_device, 10, 80, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1, "Init BNO055...");
    
    ret = simple_bno055_init(&imu_device, I2C_MASTER_PORT, 0x28);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 init failed: %s", esp_err_to_name(ret));
        m5atoms3r_lcd_printf(&lcd_device, 10, 100, LCD_COLOR_RED, LCD_COLOR_BLACK, 1, "BNO055 INIT FAIL");
        vTaskDelay(pdMS_TO_TICKS(5000));
        return ret;
    }
    
    ESP_LOGI(TAG, "BNO055 initialized successfully!");
    m5atoms3r_lcd_printf(&lcd_device, 10, 100, LCD_COLOR_GREEN, LCD_COLOR_BLACK, 1, "BNO055 OK!");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Clear screen for data display
    m5atoms3r_lcd_clear(&lcd_device, LCD_COLOR_BLACK);
    m5atoms3r_lcd_printf(&lcd_device, 2, 2, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 2, "IMU Data");
    
    // Main data display loop
    ESP_LOGI(TAG, "Starting data display loop...");
    uint32_t frame_count = 0;
    
    while (1) {
        // Clear data area
        m5atoms3r_lcd_fill_rect(&lcd_device, 2, 30, 124, 96, LCD_COLOR_BLACK);
        
        // Read quaternion
        simple_quaternion_t quat;
        ret = simple_bno055_read_quaternion(&imu_device, &quat);
        
        if (ret == ESP_OK) {
            // Display connection status
            m5atoms3r_lcd_printf(&lcd_device, 80, 2, LCD_COLOR_GREEN, LCD_COLOR_BLACK, 2, "CONN");
            
            // Display quaternion
            m5atoms3r_lcd_printf(&lcd_device, 4, 35, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 2, 
                                "W:%.2f", quat.w);
            m5atoms3r_lcd_printf(&lcd_device, 4, 55, LCD_COLOR_RED, LCD_COLOR_BLACK, 2,
                                "X:%.2f", quat.x);
            m5atoms3r_lcd_printf(&lcd_device, 4, 75, LCD_COLOR_GREEN, LCD_COLOR_BLACK, 2,
                                "Y:%.2f", quat.y);
            m5atoms3r_lcd_printf(&lcd_device, 4, 95, LCD_COLOR_BLUE, LCD_COLOR_BLACK, 2,
                                "Z:%.2f", quat.z);
            
            // Convert to Euler angles
            float roll = atan2f(2 * (quat.w * quat.x + quat.y * quat.z),
                               1 - 2 * (quat.x * quat.x + quat.y * quat.y)) * 180.0f / M_PI;
            float pitch = asinf(2 * (quat.w * quat.y - quat.z * quat.x)) * 180.0f / M_PI;
            float yaw = atan2f(2 * (quat.w * quat.z + quat.x * quat.y),
                              1 - 2 * (quat.y * quat.y + quat.z * quat.z)) * 180.0f / M_PI;
            
            m5atoms3r_lcd_printf(&lcd_device, 4, 115, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1,
                                "R:%.0f P:%.0f Y:%.0f", roll, pitch, yaw);
            
            // Frame counter
            frame_count++;
            
        } else {
            // Display disconnection
            m5atoms3r_lcd_printf(&lcd_device, 80, 2, LCD_COLOR_RED, LCD_COLOR_BLACK, 2, "DISC");
            m5atoms3r_lcd_printf(&lcd_device, 4, 50, LCD_COLOR_RED, LCD_COLOR_BLACK, 2,
                                "IMU ERROR");
            ESP_LOGW(TAG, "Failed to read IMU: %s", esp_err_to_name(ret));
        }
        
        // Update display at 10Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return ESP_OK;
}