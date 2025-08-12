/**
 * @file imu_lcd_display_test.c
 * @brief IMU データをLCDに表示するテスト
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "m5atoms3r_lcd.h"
#include "simple_bno055.h"

static const char *TAG = "IMU_LCD_DISPLAY";

// LCD instance
static m5atoms3r_lcd_t lcd;

// Initialize LCD with backlight
static esp_err_t init_lcd_display(void) {
    ESP_LOGI(TAG, "Initializing LCD display...");
    
    esp_err_t ret = m5atoms3r_lcd_init(&lcd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Enable backlight (GPIO16)
    ret = m5atoms3r_lcd_set_backlight(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Backlight enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Clear screen with black background
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    
    ESP_LOGI(TAG, "LCD display initialized successfully");
    return ESP_OK;
}

// Convert quaternion to Euler angles (roll, pitch, yaw in degrees)
static void quaternion_to_euler(float qw, float qx, float qy, float qz, 
                               float* roll, float* pitch, float* yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1) {
        *pitch = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
    } else {
        *pitch = asin(sinp) * 180.0 / M_PI;
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

// Display IMU data on LCD
static void display_imu_data(const simple_quaternion_t* quat) {
    // Clear screen
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    
    // Convert quaternion to Euler angles
    float roll, pitch, yaw;
    quaternion_to_euler(quat->w, quat->x, quat->y, quat->z, &roll, &pitch, &yaw);
    
    // Display title
    m5atoms3r_lcd_printf(&lcd, 5, 5, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "IMU Data");
    
    // Display quaternion data
    m5atoms3r_lcd_printf(&lcd, 5, 20, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 1, 
                        "Quaternion:");
    m5atoms3r_lcd_printf(&lcd, 5, 30, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "W: %.3f", quat->w);
    m5atoms3r_lcd_printf(&lcd, 5, 40, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "X: %.3f", quat->x);
    m5atoms3r_lcd_printf(&lcd, 5, 50, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "Y: %.3f", quat->y);
    m5atoms3r_lcd_printf(&lcd, 5, 60, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "Z: %.3f", quat->z);
    
    // Display Euler angles
    m5atoms3r_lcd_printf(&lcd, 5, 75, LCD_COLOR_YELLOW, LCD_COLOR_BLACK, 1, 
                        "Euler (deg):");
    m5atoms3r_lcd_printf(&lcd, 5, 85, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "Roll:  %.1f", roll);
    m5atoms3r_lcd_printf(&lcd, 5, 95, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "Pitch: %.1f", pitch);
    m5atoms3r_lcd_printf(&lcd, 5, 105, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                        "Yaw:   %.1f", yaw);
}

// IMU display task
static void imu_display_task(void* param) {
    ESP_LOGI(TAG, "Starting IMU display task");
    
    simple_bno055_t* bno055 = (simple_bno055_t*)param;
    simple_quaternion_t quat;
    
    uint32_t update_count = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Read IMU data
        esp_err_t ret = simple_bno055_read_quaternion(bno055, &quat);
        if (ret == ESP_OK) {
            // Display data on LCD
            display_imu_data(&quat);
            
            update_count++;
            
            // Log to serial every 10 updates
            if (update_count % 10 == 0) {
                ESP_LOGI(TAG, "Display update %ld - Q: w=%.3f x=%.3f y=%.3f z=%.3f", 
                        update_count, quat.w, quat.x, quat.y, quat.z);
            }
        } else {
            // Display error message
            m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
            m5atoms3r_lcd_printf(&lcd, 5, 50, LCD_COLOR_RED, LCD_COLOR_BLACK, 1, 
                                "IMU Error: %s", esp_err_to_name(ret));
            ESP_LOGW(TAG, "IMU read error: %s", esp_err_to_name(ret));
        }
        
        // Update at 10Hz (100ms interval)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

// Initialize IMU and start display
esp_err_t run_imu_lcd_display_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "IMU LCD Display Test");
    ESP_LOGI(TAG, "==========================================");
    
    // Step 1: Initialize LCD
    esp_err_t ret = init_lcd_display();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD initialization failed");
        return ret;
    }
    
    // Display initialization message
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    m5atoms3r_lcd_printf(&lcd, 10, 50, LCD_COLOR_CYAN, LCD_COLOR_BLACK, 1, 
                        "Initializing IMU...");
    
    // Step 2: Initialize I2C for IMU
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 45,  // M5AtomS3R SDA pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 0,   // M5AtomS3R SCL pin  
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };
    
    ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed");
        return ret;
    }
    
    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return ret;
    }
    
    // Step 3: Initialize IMU
    static simple_bno055_t bno055;
    ret = simple_bno055_init(&bno055, I2C_NUM_0, BNO055_I2C_ADDR_A);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed");
        m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
        m5atoms3r_lcd_printf(&lcd, 5, 40, LCD_COLOR_RED, LCD_COLOR_BLACK, 1, 
                            "IMU Init Failed!");
        m5atoms3r_lcd_printf(&lcd, 5, 60, LCD_COLOR_WHITE, LCD_COLOR_BLACK, 1, 
                            "Error: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set IMU to NDOF mode
    ret = simple_bno055_set_mode(&bno055, BNO055_OPERATION_MODE_NDOF);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set NDOF mode, continuing anyway");
    }
    
    // Display success message
    m5atoms3r_lcd_clear(&lcd, LCD_COLOR_BLACK);
    m5atoms3r_lcd_printf(&lcd, 10, 50, LCD_COLOR_GREEN, LCD_COLOR_BLACK, 1, 
                        "IMU Ready!");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 4: Start display task
    BaseType_t task_ret = xTaskCreate(
        imu_display_task,
        "imu_display_task", 
        4096,
        &bno055,  // Pass BNO055 handle to task
        5,        // Priority
        NULL
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU display task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "IMU LCD Display Test Started!");
    ESP_LOGI(TAG, "Display updates at 10Hz");
    ESP_LOGI(TAG, "==========================================");
    
    return ESP_OK;
}