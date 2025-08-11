#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bno055.h"
#include "lcd_imu_display.h"

static const char *TAG = "I2C_IMU_TEST";

// I2C Configuration
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_PIN      GPIO_NUM_38  // M5AtomS3R SDA
#define I2C_MASTER_SCL_PIN      GPIO_NUM_39  // M5AtomS3R SCL
#define I2C_MASTER_FREQ_HZ      100000       // 100kHz
#define I2C_MASTER_TX_BUF_LEN   0
#define I2C_MASTER_RX_BUF_LEN   0

// BNO055 Configuration
#define BNO055_I2C_ADDRESS      BNO055_I2C_ADDR_PRIMARY

// Global variables
static bno055_handle_t bno055_handle = NULL;
static TaskHandle_t imu_monitor_task_handle = NULL;

void test_i2c_bus_scan(void) {
    ESP_LOGI(TAG, "=== I2C Bus Scan Test ===");
    
    int devices_found = 0;
    
    ESP_LOGI(TAG, "Scanning I2C bus...");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
    
    for (int addr = 0; addr < 128; addr++) {
        if (addr % 16 == 0) {
            printf("%02X: ", addr);
        }
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("%02X ", addr);
            devices_found++;
            
            if (addr == BNO055_I2C_ADDR_PRIMARY || addr == BNO055_I2C_ADDR_SECONDARY) {
                ESP_LOGI(TAG, "Found BNO055 at address 0x%02X", addr);
            }
        } else {
            printf("-- ");
        }
        
        if ((addr + 1) % 16 == 0) {
            printf("\n");
        }
    }
    
    ESP_LOGI(TAG, "I2C scan completed. Found %d devices", devices_found);
}

void test_i2c_basic_communication(void) {
    ESP_LOGI(TAG, "=== I2C Basic Communication Test ===");
    
    // Test basic I2C communication with various common addresses
    uint8_t test_addresses[] = {BNO055_I2C_ADDR_PRIMARY, BNO055_I2C_ADDR_SECONDARY, 0x68, 0x1D};
    int num_addresses = sizeof(test_addresses) / sizeof(test_addresses[0]);
    
    for (int i = 0; i < num_addresses; i++) {
        uint8_t addr = test_addresses[i];
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true); // Try to read register 0x00
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        
        uint8_t data;
        i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Address 0x%02X: Communication OK, Register 0x00 = 0x%02X", addr, data);
        } else {
            ESP_LOGD(TAG, "Address 0x%02X: No response", addr);
        }
    }
}

void test_bno055_initialization(void) {
    ESP_LOGI(TAG, "=== BNO055 Initialization Test ===");
    
    bno055_config_t config = {
        .i2c_port = I2C_MASTER_PORT,
        .device_address = BNO055_I2C_ADDRESS,
        .sda_pin = I2C_MASTER_SDA_PIN,
        .scl_pin = I2C_MASTER_SCL_PIN,
        .clock_speed = I2C_MASTER_FREQ_HZ,
        .operating_mode = BNO055_MODE_NDOF,
        .power_mode = BNO055_POWER_MODE_NORMAL,
        .external_crystal = true
    };
    
    esp_err_t ret = bno055_init(&config, &bno055_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BNO055 initialization successful ✓");
        
        // Read device information
        uint8_t chip_id;
        ret = bno055_get_chip_id(bno055_handle, &chip_id);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Chip ID: 0x%02X", chip_id);
        }
        
        // Get system status
        bno055_system_status_t sys_status;
        ret = bno055_get_system_status(bno055_handle, &sys_status);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "System Status: 0x%02X", sys_status.system_status);
            ESP_LOGI(TAG, "System Error: 0x%02X", sys_status.system_error);
            ESP_LOGI(TAG, "Self Test Result: 0x%02X", sys_status.self_test_result);
        }
        
        // Read temperature
        float temperature;
        ret = bno055_read_temperature(bno055_handle, &temperature);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.1f °C", temperature);
        }
        
    } else {
        ESP_LOGE(TAG, "BNO055 initialization failed: %s", esp_err_to_name(ret));
    }
}

void test_bno055_data_reading(void) {
    ESP_LOGI(TAG, "=== BNO055 Data Reading Test ===");
    
    if (!bno055_handle) {
        ESP_LOGE(TAG, "BNO055 not initialized");
        return;
    }
    
    const int num_samples = 10;
    ESP_LOGI(TAG, "Reading %d samples of IMU data...", num_samples);
    
    for (int i = 0; i < num_samples; i++) {
        // Read quaternion data
        bno055_quaternion_t quat;
        esp_err_t ret = bno055_read_quaternion(bno055_handle, &quat);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Sample %d - Quaternion: W=%.3f, X=%.3f, Y=%.3f, Z=%.3f", 
                     i + 1, quat.w, quat.x, quat.y, quat.z);
            
            // Calculate quaternion magnitude (should be close to 1.0)
            float magnitude = sqrt(quat.w*quat.w + quat.x*quat.x + quat.y*quat.y + quat.z*quat.z);
            if (magnitude < 0.9f || magnitude > 1.1f) {
                ESP_LOGW(TAG, "  Warning: Quaternion magnitude = %.3f (expected ~1.0)", magnitude);
            }
        } else {
            ESP_LOGE(TAG, "Sample %d - Failed to read quaternion: %s", i + 1, esp_err_to_name(ret));
        }
        
        // Read Euler angles
        bno055_euler_t euler;
        ret = bno055_read_euler(bno055_handle, &euler);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Euler: Heading=%.1f°, Roll=%.1f°, Pitch=%.1f°", 
                     euler.heading, euler.roll, euler.pitch);
        }
        
        // Read raw sensor data
        bno055_vector3_t accel, gyro, mag;
        
        ret = bno055_read_accelerometer(bno055_handle, &accel);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²", accel.x, accel.y, accel.z);
        }
        
        ret = bno055_read_gyroscope(bno055_handle, &gyro);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Gyro: X=%.2f, Y=%.2f, Z=%.2f °/s", gyro.x, gyro.y, gyro.z);
        }
        
        ret = bno055_read_magnetometer(bno055_handle, &mag);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Mag: X=%.2f, Y=%.2f, Z=%.2f µT", mag.x, mag.y, mag.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGI(TAG, "Data reading test completed ✓");
}

void test_bno055_calibration_status(void) {
    ESP_LOGI(TAG, "=== BNO055 Calibration Status Test ===");
    
    if (!bno055_handle) {
        ESP_LOGE(TAG, "BNO055 not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Monitoring calibration status for 30 seconds...");
    ESP_LOGI(TAG, "Move the device in figure-8 pattern for magnetometer calibration");
    ESP_LOGI(TAG, "Place on 6 different stable positions for accelerometer calibration");
    ESP_LOGI(TAG, "Keep stationary for gyroscope calibration");
    
    int64_t start_time = esp_timer_get_time();
    int last_print_time = 0;
    
    while ((esp_timer_get_time() - start_time) < 30000000) { // 30 seconds
        bno055_calibration_status_t calib;
        esp_err_t ret = bno055_get_calibration_status(bno055_handle, &calib);
        
        if (ret == ESP_OK) {
            int current_time = (esp_timer_get_time() - start_time) / 1000000;
            
            if (current_time != last_print_time && current_time % 2 == 0) { // Print every 2 seconds
                ESP_LOGI(TAG, "Calibration [%02ds] - Sys:%d Gyro:%d Accel:%d Mag:%d", 
                        current_time, calib.system, calib.gyroscope, 
                        calib.accelerometer, calib.magnetometer);
                
                if (calib.system == 3 && calib.gyroscope == 3 && 
                    calib.accelerometer == 3 && calib.magnetometer == 3) {
                    ESP_LOGI(TAG, "All sensors fully calibrated! ✓");
                    break;
                }
                last_print_time = current_time;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Final calibration check
    bool is_calibrated;
    esp_err_t ret = bno055_is_calibrated(bno055_handle, &is_calibrated);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Final calibration status: %s", is_calibrated ? "FULLY CALIBRATED ✓" : "NOT FULLY CALIBRATED");
    }
}

void test_imu_performance(void) {
    ESP_LOGI(TAG, "=== IMU Performance Test ===");
    
    if (!bno055_handle) {
        ESP_LOGE(TAG, "BNO055 not initialized");
        return;
    }
    
    const int num_reads = 100;
    int64_t total_time = 0;
    int successful_reads = 0;
    
    ESP_LOGI(TAG, "Performing %d quaternion reads for performance measurement...", num_reads);
    
    for (int i = 0; i < num_reads; i++) {
        int64_t start = esp_timer_get_time();
        
        bno055_quaternion_t quat;
        esp_err_t ret = bno055_read_quaternion(bno055_handle, &quat);
        
        int64_t end = esp_timer_get_time();
        
        if (ret == ESP_OK) {
            total_time += (end - start);
            successful_reads++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between reads
    }
    
    if (successful_reads > 0) {
        float avg_time_ms = (total_time / successful_reads) / 1000.0f;
        float max_rate_hz = 1000.0f / avg_time_ms;
        
        ESP_LOGI(TAG, "Performance Results:");
        ESP_LOGI(TAG, "  Successful reads: %d/%d", successful_reads, num_reads);
        ESP_LOGI(TAG, "  Average read time: %.2f ms", avg_time_ms);
        ESP_LOGI(TAG, "  Maximum rate: %.1f Hz", max_rate_hz);
        
        if (max_rate_hz >= 100.0f) {
            ESP_LOGI(TAG, "Performance: Excellent ✓ (>100 Hz capable)");
        } else if (max_rate_hz >= 50.0f) {
            ESP_LOGI(TAG, "Performance: Good ✓ (>50 Hz capable)");
        } else {
            ESP_LOGW(TAG, "Performance: Limited (<%1f Hz)", max_rate_hz);
        }
    } else {
        ESP_LOGE(TAG, "Performance test failed - no successful reads");
    }
}

void imu_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "IMU Monitor Task started");
    
    if (!bno055_handle) {
        ESP_LOGE(TAG, "BNO055 not initialized for monitoring");
        vTaskDelete(NULL);
        return;
    }
    
    const int report_interval_ms = 5000; // Report every 5 seconds
    int64_t last_report_time = esp_timer_get_time();
    
    bno055_quaternion_t last_quat = {1, 0, 0, 0};
    int sample_count = 0;
    float total_change = 0.0f;
    
    while (1) {
        bno055_quaternion_t quat;
        esp_err_t ret = bno055_read_quaternion(bno055_handle, &quat);
        
        if (ret == ESP_OK) {
            // Calculate quaternion change from last reading
            float change = fabs(quat.w - last_quat.w) + fabs(quat.x - last_quat.x) + 
                          fabs(quat.y - last_quat.y) + fabs(quat.z - last_quat.z);
            
            total_change += change;
            sample_count++;
            last_quat = quat;
            
            int64_t current_time = esp_timer_get_time();
            if (current_time - last_report_time >= report_interval_ms * 1000) {
                float avg_change = sample_count > 0 ? total_change / sample_count : 0.0f;
                
                ESP_LOGI(TAG, "IMU Monitor - Samples: %d, Avg Change: %.4f, Current Q: [%.3f,%.3f,%.3f,%.3f]",
                        sample_count, avg_change, quat.w, quat.x, quat.y, quat.z);
                
                // Reset counters
                sample_count = 0;
                total_change = 0.0f;
                last_report_time = current_time;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(33)); // ~30Hz monitoring
    }
}

void run_i2c_imu_tests(void) {
    ESP_LOGI(TAG, "Starting I2C and IMU comprehensive tests...");
    
    // Test I2C bus functionality
    test_i2c_bus_scan();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_i2c_basic_communication();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test BNO055 IMU
    test_bno055_initialization();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    if (bno055_handle) {
        test_bno055_data_reading();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        test_bno055_calibration_status();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        test_imu_performance();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Initialize LCD display for quaternion visualization
        ESP_LOGI(TAG, "Initializing LCD display for quaternion visualization...");
        esp_err_t lcd_ret = lcd_imu_display_init(bno055_handle);
        if (lcd_ret == ESP_OK) {
            ESP_LOGI(TAG, "LCD IMU display initialized ✓");
        } else {
            ESP_LOGE(TAG, "LCD IMU display initialization failed ✗");
        }
        
        // Start background monitoring task
        ESP_LOGI(TAG, "Starting IMU monitoring task...");
        BaseType_t ret = xTaskCreatePinnedToCore(
            imu_monitor_task,
            "imu_monitor",
            4096,
            NULL,
            6,
            &imu_monitor_task_handle,
            1
        );
        
        if (ret == pdPASS) {
            ESP_LOGI(TAG, "IMU Monitor task created ✓");
        } else {
            ESP_LOGE(TAG, "IMU Monitor task creation failed ✗");
        }
    }
    
    ESP_LOGI(TAG, "I2C and IMU tests completed ✓");
}

void cleanup_i2c_imu_tests(void) {
    ESP_LOGI(TAG, "Cleaning up I2C and IMU tests...");
    
    // Cleanup LCD display first
    lcd_imu_display_cleanup();
    
    // Delete monitoring task
    if (imu_monitor_task_handle) {
        vTaskDelete(imu_monitor_task_handle);
        imu_monitor_task_handle = NULL;
        ESP_LOGI(TAG, "IMU Monitor task deleted");
    }
    
    // Deinitialize BNO055
    if (bno055_handle) {
        bno055_deinit(bno055_handle);
        bno055_handle = NULL;
        ESP_LOGI(TAG, "BNO055 deinitialized");
    }
    
    ESP_LOGI(TAG, "I2C and IMU cleanup completed ✓");
}