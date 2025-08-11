/**
 * @file bno055_driver.c
 * @brief BNO055 IMU Sensor Driver Main Implementation
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bno055_driver.h"
#include "quaternion_filter.h"

static const char *TAG = "BNO055_DRIVER";

/**
 * @brief BNO055 driver context structure
 */
typedef struct {
    bno055_handle_t handle;
    quaternion_filter_t filter;
    bool filter_enabled;
    TaskHandle_t task_handle;
    bool task_running;
} bno055_driver_context_t;

static bno055_driver_context_t g_bno055_ctx = {0};

/**
 * @brief BNO055 data processing task
 */
static void bno055_task(void *param) {
    bno055_driver_context_t *ctx = (bno055_driver_context_t*)param;
    bno055_data_t imu_data;
    bno055_quaternion_t filtered_quat;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "BNO055 task started");
    
    while (ctx->task_running) {
        // Read IMU data
        esp_err_t ret = bno055_read_data(&ctx->handle, &imu_data);
        if (ret == ESP_OK && imu_data.data_valid) {
            
            // Apply filtering if enabled
            if (ctx->filter_enabled) {
                ret = quaternion_filter_update(&ctx->filter, 
                                             &imu_data.quaternion,
                                             imu_data.timestamp,
                                             &filtered_quat);
                if (ret == ESP_OK) {
                    imu_data.quaternion = filtered_quat;
                }
            }
            
            // Log calibration status periodically
            static uint32_t last_calib_log = 0;
            if (imu_data.timestamp - last_calib_log > 10000) {  // Every 10 seconds
                ESP_LOGI(TAG, "Calibration: Sys=%d, Gyro=%d, Acc=%d, Mag=%d, Temp=%d°C",
                         imu_data.calibration.system,
                         imu_data.calibration.gyroscope,
                         imu_data.calibration.accelerometer,
                         imu_data.calibration.magnetometer,
                         imu_data.temperature);
                
                // Save calibration data if fully calibrated
                if (bno055_is_fully_calibrated(&imu_data.calibration)) {
                    static bool calib_saved = false;
                    if (!calib_saved) {
                        ESP_LOGI(TAG, "Sensor fully calibrated, saving calibration data");
                        bno055_save_calibration_data(&ctx->handle);
                        calib_saved = true;
                    }
                }
                
                last_calib_log = imu_data.timestamp;
            }
            
        } else {
            ESP_LOGW(TAG, "Failed to read IMU data: %s", esp_err_to_name(ret));
        }
        
        // Task runs at 100Hz (10ms period)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "BNO055 task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Initialize BNO055 driver with default configuration
 */
esp_err_t bno055_driver_init(void) {
    bno055_config_t config;
    bno055_get_default_config(&config);
    
    return bno055_driver_init_with_config(&config);
}

/**
 * @brief Initialize BNO055 driver with custom configuration
 */
esp_err_t bno055_driver_init_with_config(const bno055_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing BNO055 driver");
    
    // Initialize BNO055 sensor
    esp_err_t ret = bno055_init(&g_bno055_ctx.handle, config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BNO055 sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize quaternion filter
    quaternion_filter_config_t filter_config;
    quaternion_filter_get_default_config(&filter_config);
    
    ret = quaternion_filter_init(&g_bno055_ctx.filter, &filter_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize quaternion filter: %s", esp_err_to_name(ret));
        bno055_deinit(&g_bno055_ctx.handle);
        return ret;
    }
    
    g_bno055_ctx.filter_enabled = true;
    
    ESP_LOGI(TAG, "BNO055 driver initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start BNO055 data processing task
 */
esp_err_t bno055_driver_start_task(void) {
    if (g_bno055_ctx.task_running) {
        ESP_LOGW(TAG, "BNO055 task already running");
        return ESP_OK;
    }
    
    g_bno055_ctx.task_running = true;
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        bno055_task,
        "bno055_task",
        4096,
        &g_bno055_ctx,
        24,  // High priority for real-time IMU data
        &g_bno055_ctx.task_handle,
        1    // Run on Core 1 (APP CPU)
    );
    
    if (ret != pdPASS) {
        g_bno055_ctx.task_running = false;
        ESP_LOGE(TAG, "Failed to create BNO055 task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "BNO055 task started successfully");
    return ESP_OK;
}

/**
 * @brief Stop BNO055 data processing task
 */
esp_err_t bno055_driver_stop_task(void) {
    if (!g_bno055_ctx.task_running) {
        return ESP_OK;
    }
    
    g_bno055_ctx.task_running = false;
    
    // Wait for task to finish
    if (g_bno055_ctx.task_handle) {
        while (eTaskGetState(g_bno055_ctx.task_handle) != eDeleted) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        g_bno055_ctx.task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "BNO055 task stopped");
    return ESP_OK;
}

/**
 * @brief Deinitialize BNO055 driver
 */
esp_err_t bno055_driver_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing BNO055 driver");
    
    // Stop task first
    bno055_driver_stop_task();
    
    // Deinitialize sensor
    esp_err_t ret = bno055_deinit(&g_bno055_ctx.handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to deinitialize BNO055 sensor: %s", esp_err_to_name(ret));
    }
    
    // Reset context
    memset(&g_bno055_ctx, 0, sizeof(bno055_driver_context_t));
    
    ESP_LOGI(TAG, "BNO055 driver deinitialized");
    return ESP_OK;
}

/**
 * @brief Get current IMU data
 */
esp_err_t bno055_driver_get_data(bno055_data_t* data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bno055_ctx.handle.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return bno055_read_data(&g_bno055_ctx.handle, data);
}

/**
 * @brief Get current quaternion with filtering
 */
esp_err_t bno055_driver_get_quaternion(bno055_quaternion_t* quaternion) {
    if (!quaternion) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bno055_ctx.handle.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    bno055_quaternion_t raw_quat;
    esp_err_t ret = bno055_read_quaternion(&g_bno055_ctx.handle, &raw_quat);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (g_bno055_ctx.filter_enabled) {
        uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ret = quaternion_filter_update(&g_bno055_ctx.filter, &raw_quat, timestamp, quaternion);
    } else {
        *quaternion = raw_quat;
    }
    
    return ret;
}

/**
 * @brief Enable/disable quaternion filtering
 */
esp_err_t bno055_driver_set_filter_enabled(bool enabled) {
    g_bno055_ctx.filter_enabled = enabled;
    
    if (!enabled) {
        // Reset filter when disabling
        quaternion_filter_reset(&g_bno055_ctx.filter);
    }
    
    ESP_LOGI(TAG, "Quaternion filtering %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Get filter statistics
 */
esp_err_t bno055_driver_get_filter_stats(quaternion_filter_stats_t* stats) {
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return quaternion_filter_get_stats(&g_bno055_ctx.filter, stats);
}

/**
 * @brief Check if driver is ready
 */
bool bno055_driver_is_ready(void) {
    return g_bno055_ctx.handle.initialized && bno055_is_ready(&g_bno055_ctx.handle);
}

/**
 * @brief Get calibration status
 */
esp_err_t bno055_driver_get_calibration_status(bno055_calibration_t* calibration) {
    if (!calibration) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bno055_ctx.handle.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return bno055_read_calibration_status(&g_bno055_ctx.handle, calibration);
}

/**
 * @brief Force save calibration data
 */
esp_err_t bno055_driver_save_calibration(void) {
    if (!g_bno055_ctx.handle.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return bno055_save_calibration_data(&g_bno055_ctx.handle);
}

/**
 * @brief Reset sensor
 */
esp_err_t bno055_driver_reset(void) {
    if (!g_bno055_ctx.handle.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting BNO055 sensor");
    
    // Reset filter
    quaternion_filter_reset(&g_bno055_ctx.filter);
    
    // Reset sensor
    return bno055_reset(&g_bno055_ctx.handle);
}

/**
 * @brief Perform self-test
 */
esp_err_t bno055_driver_self_test(uint8_t* result) {
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_bno055_ctx.handle.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return bno055_self_test(&g_bno055_ctx.handle, result);
}