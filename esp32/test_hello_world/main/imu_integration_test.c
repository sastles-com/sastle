#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "simple_bno055.h"
#include "m5atoms3r_lcd.h"
#include "quaternion_visualizer.h"

static const char *TAG = "IMU_INTEGRATION";

// I2C Configuration for M5AtomS3R
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_PIN      GPIO_NUM_2   // M5AtomS3R SDA (Correct pin)
#define I2C_MASTER_SCL_PIN      GPIO_NUM_1   // M5AtomS3R SCL (Correct pin)
#define I2C_MASTER_FREQ_HZ      100000       // 100kHz
#define I2C_MASTER_TIMEOUT_MS   1000

// BNO055 Configuration
#define BNO055_I2C_ADDRESS      BNO055_I2C_ADDR_A

// Test parameters
#define IMU_UPDATE_RATE_HZ      30           // Target: 30Hz update rate
#define IMU_UPDATE_PERIOD_MS    (1000 / IMU_UPDATE_RATE_HZ)

static simple_bno055_t bno055_device;
static m5atoms3r_lcd_t lcd_device;
static quaternion_visualizer_t visualizer;

/**
 * @brief Initialize I2C bus
 */
esp_err_t init_i2c_bus(void)
{
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

    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize BNO055 sensor
 */
esp_err_t init_bno055_sensor(void)
{
    esp_err_t ret = simple_bno055_init(&bno055_device, I2C_MASTER_PORT, BNO055_I2C_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BNO055 sensor initialized successfully");
    return ESP_OK;
}

/**
 * @brief Verify sensor communication
 */
esp_err_t verify_sensor_communication(void)
{
    uint8_t chip_id;
    esp_err_t ret = simple_bno055_read_chip_id(&bno055_device, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != BNO055_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected: 0x%02X)", chip_id, BNO055_ID);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Chip ID verified: 0x%02X ✓", chip_id);
    return ESP_OK;
}

/**
 * @brief Print quaternion as readable Euler angles
 */
void print_quaternion_info(const simple_quaternion_t* quat)
{
    // Convert quaternion to Euler angles (degrees)
    float roll, pitch, yaw;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (quat->w * quat->x + quat->y * quat->z);
    float cosr_cosp = 1 - 2 * (quat->x * quat->x + quat->y * quat->y);
    roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (quat->w * quat->y - quat->z * quat->x);
    if (fabsf(sinp) >= 1)
        pitch = copysignf(M_PI / 2, sinp) * 180.0f / M_PI;
    else
        pitch = asinf(sinp) * 180.0f / M_PI;
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (quat->w * quat->z + quat->x * quat->y);
    float cosy_cosp = 1 - 2 * (quat->y * quat->y + quat->z * quat->z);
    yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    // Calculate magnitude
    float magnitude = sqrtf(quat->w * quat->w + quat->x * quat->x + 
                           quat->y * quat->y + quat->z * quat->z);

    ESP_LOGI(TAG, "Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f (|q|=%.3f)",
             quat->w, quat->x, quat->y, quat->z, magnitude);
    ESP_LOGI(TAG, "Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°", roll, pitch, yaw);
}

/**
 * @brief Print calibration status
 */
void print_calibration_status(void)
{
    simple_calibration_status_t calib_status;
    esp_err_t ret = simple_bno055_read_calibration_status(&bno055_device, &calib_status);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration - Sys:%d, Gyr:%d, Acc:%d, Mag:%d",
                 calib_status.system, calib_status.gyroscope, 
                 calib_status.accelerometer, calib_status.magnetometer);
    }
}

/**
 * @brief Real-time IMU monitoring task with LCD display
 */
void imu_monitoring_task(void *param)
{
    ESP_LOGI(TAG, "Starting real-time IMU monitoring (Target: %dHz)", IMU_UPDATE_RATE_HZ);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_display_time = xTaskGetTickCount();
    int64_t last_timestamp = esp_timer_get_time();
    uint32_t sample_count = 0;
    
    // LCD display update parameters (10Hz)
    const uint32_t display_update_period = pdMS_TO_TICKS(100); // 100ms = 10Hz
    
    while (1) {
        int64_t current_timestamp = esp_timer_get_time();
        float actual_rate = 1000000.0f / (current_timestamp - last_timestamp);
        last_timestamp = current_timestamp;
        
        // Update visualizer data (30Hz)
        esp_err_t ret = quat_vis_update(&visualizer);
        
        if (ret == ESP_OK) {
            sample_count++;
            
            // Update LCD display at 10Hz
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_display_time) >= display_update_period) {
                quat_vis_render_frame(&visualizer);
                last_display_time = current_time;
            }
            
            // Print detailed info every 90 samples (approximately every 3 seconds at 30Hz)
            if (sample_count % 90 == 0) {
                simple_quaternion_t quaternion;
                if (simple_bno055_read_quaternion(&bno055_device, &quaternion) == ESP_OK) {
                    ESP_LOGI(TAG, "=== IMU Sample #%ld (Rate: %.1fHz) ===", sample_count, actual_rate);
                    print_quaternion_info(&quaternion);
                    print_calibration_status();
                    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
                    ESP_LOGI(TAG, "========================================");
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to update visualizer: %s", esp_err_to_name(ret));
        }
        
        // Maintain target update rate
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief Main IMU integration test function
 */
esp_err_t run_imu_integration_test(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "BNO055 IMU Integration Test Start");
    ESP_LOGI(TAG, "====================================");
    
    // Initialize I2C bus
    esp_err_t ret = init_i2c_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return ret;
    }
    
    // Initialize BNO055 sensor
    ret = init_bno055_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 initialization failed");
        return ret;
    }
    
    // Verify communication
    ret = verify_sensor_communication();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor communication verification failed");
        return ret;
    }
    
    // Initialize LCD display
    ESP_LOGI(TAG, "Initializing LCD display...");
    ret = m5atoms3r_lcd_init(&lcd_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize visualizer
    ESP_LOGI(TAG, "Initializing quaternion visualizer...");
    ret = quat_vis_init(&visualizer, &lcd_device, &bno055_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Visualizer initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Waiting 2 seconds for sensor stabilization...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Create monitoring task
    TaskHandle_t monitoring_task_handle;
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        imu_monitoring_task,
        "imu_monitor",
        4096,
        NULL,
        10,
        &monitoring_task_handle,
        1  // Run on Core 1
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU monitoring task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "IMU + LCD Integration Test running successfully!");
    ESP_LOGI(TAG, "Move the device to see quaternion changes on LCD");
    ESP_LOGI(TAG, "Calibration will improve with movement");
    
    return ESP_OK;
}