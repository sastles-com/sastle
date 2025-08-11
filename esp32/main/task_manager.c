/**
 * @file task_manager.c
 * @brief Task management and coordination
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "task_manager.h"
#include "app_config.h"
#include "bno055_driver.h"
#include "error_handler.h"

static const char *TAG = "TASK_MGR";

// Task handles
static TaskHandle_t task_handle_imu = NULL;
static TaskHandle_t task_handle_ros2_rx = NULL;
static TaskHandle_t task_handle_ros2_tx = NULL;
static TaskHandle_t task_handle_image_proc = NULL;
static TaskHandle_t task_handle_spi_tx = NULL;
static TaskHandle_t task_handle_display = NULL;

// Task statistics
typedef struct {
    uint32_t executions;
    uint32_t total_time_us;
    uint32_t max_time_us;
    uint32_t last_execution_us;
} task_stats_t;

static task_stats_t task_stats[6] = {0}; // One for each task

/**
 * @brief IMU task with BNO055 driver integration
 * Core 0: Handles BNO055 communication and quaternion filtering
 */
static void imu_task(void *pvParameters) {
    ESP_LOGI(TAG, "IMU task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / IMU_SAMPLE_RATE_HZ); // 100Hz
    
    uint32_t execution_count = 0;
    uint32_t error_count = 0;
    uint32_t last_calib_log = 0;
    
    // Initialize BNO055 driver
    esp_err_t ret = bno055_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BNO055 driver: %s", esp_err_to_name(ret));
        LOG_ERROR_CRITICAL("IMU", "BNO055 initialization failed", ret);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "BNO055 driver initialized successfully");
    
    while (1) {
        uint32_t start_time = esp_timer_get_time();
        
        // Read quaternion from BNO055 with filtering
        bno055_quaternion_t quaternion;
        bno055_calibration_t calibration;
        
        ret = bno055_driver_get_quaternion(&quaternion);
        if (ret == ESP_OK) {
            // Get calibration status
            ret = bno055_driver_get_calibration_status(&calibration);
            if (ret == ESP_OK) {
                // Prepare IMU data for other tasks
                quaternion_data_t imu_data = {
                    .w = quaternion.w,
                    .x = quaternion.x,
                    .y = quaternion.y,
                    .z = quaternion.z,
                    .timestamp = esp_timer_get_time(),
                    .calibration_status = (calibration.system << 6) | 
                                        (calibration.gyroscope << 4) |
                                        (calibration.accelerometer << 2) |
                                        calibration.magnetometer
                };
                
                // Send to image processing queue
                if (g_app_ctx && g_app_ctx->queue_imu) {
                    if (xQueueSend(g_app_ctx->queue_imu, &imu_data, pdMS_TO_TICKS(1)) != pdTRUE) {
                        // Queue full, drop oldest
                        quaternion_data_t dummy;
                        xQueueReceive(g_app_ctx->queue_imu, &dummy, 0);
                        xQueueSend(g_app_ctx->queue_imu, &imu_data, 0);
                    }
                }
                
                // Log calibration status periodically
                uint32_t current_time = esp_timer_get_time();
                if (current_time - last_calib_log > 10000000) { // Every 10 seconds
                    ESP_LOGI(TAG, "IMU Calibration: Sys=%d, Gyro=%d, Acc=%d, Mag=%d",
                             calibration.system, calibration.gyroscope, 
                             calibration.accelerometer, calibration.magnetometer);
                    
                    if (bno055_is_fully_calibrated(&calibration)) {
                        ESP_LOGI(TAG, "IMU fully calibrated");
                        // Auto-save calibration when fully calibrated
                        bno055_driver_save_calibration();
                    }
                    
                    last_calib_log = current_time;
                }
                
                error_count = 0; // Reset error count on success
            } else {
                error_count++;
                ESP_LOGW(TAG, "Failed to read calibration status: %s", esp_err_to_name(ret));
            }
        } else {
            error_count++;
            ESP_LOGW(TAG, "Failed to read quaternion: %s", esp_err_to_name(ret));
            
            // Handle persistent errors
            if (error_count > 100) {  // 1 second of consecutive errors at 100Hz
                ESP_LOGE(TAG, "Too many IMU read errors, attempting reset");
                LOG_ERROR_ERROR("IMU", "Persistent read errors", ret);
                
                bno055_driver_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                error_count = 0;
            }
        }
        
        // Update statistics
        uint32_t execution_time = esp_timer_get_time() - start_time;
        task_stats[0].executions++;
        task_stats[0].total_time_us += execution_time;
        task_stats[0].last_execution_us = execution_time;
        if (execution_time > task_stats[0].max_time_us) {
            task_stats[0].max_time_us = execution_time;
        }
        
        // Set IMU ready flag after first successful read
        if (execution_count == 0 && ret == ESP_OK && g_app_ctx) {
            xEventGroupSetBits(g_app_ctx->event_group_system, EVENT_BIT_IMU_READY);
            ESP_LOGI(TAG, "IMU ready flag set");
        }
        execution_count++;
        
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/**
 * @brief Placeholder ROS2 receive task
 * Core 0: Handles image frame reception from ROS2 topics
 */
static void ros2_rx_task(void *pvParameters) {
    ESP_LOGI(TAG, "ROS2 RX task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(TARGET_FRAME_TIME_MS); // 30fps
    
    while (1) {
        uint32_t start_time = esp_timer_get_time();
        
        // TODO: Implement micro-ROS image reception
        // 1. Subscribe to /video/frame topic
        // 2. Receive JPEG compressed image
        // 3. Validate and buffer image
        
        // Mock implementation
        static image_frame_t mock_frame = {0};
        mock_frame.jpeg_size = 1024;  // Mock size
        mock_frame.timestamp = esp_timer_get_time();
        mock_frame.frame_id++;
        mock_frame.is_valid = true;
        
        // Send to image processing
        if (g_app_ctx && g_app_ctx->queue_image) {
            image_frame_t* frame_ptr = &mock_frame;
            if (xQueueSend(g_app_ctx->queue_image, &frame_ptr, pdMS_TO_TICKS(5)) != pdTRUE) {
                ESP_LOGW(TAG, "Image queue full, dropping frame %lu", mock_frame.frame_id);
                if (g_app_ctx->system_status.dropped_frames < UINT32_MAX) {
                    g_app_ctx->system_status.dropped_frames++;
                }
            }
        }
        
        // Update statistics
        uint32_t execution_time = esp_timer_get_time() - start_time;
        task_stats[1].executions++;
        task_stats[1].total_time_us += execution_time;
        task_stats[1].last_execution_us = execution_time;
        if (execution_time > task_stats[1].max_time_us) {
            task_stats[1].max_time_us = execution_time;
        }
        
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/**
 * @brief Placeholder ROS2 transmit task
 * Core 0: Handles IMU data transmission via ROS2
 */
static void ros2_tx_task(void *pvParameters) {
    ESP_LOGI(TAG, "ROS2 TX task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / 30); // 30Hz IMU publishing
    
    while (1) {
        uint32_t start_time = esp_timer_get_time();
        
        // TODO: Implement micro-ROS publishing
        // 1. Get latest IMU data
        // 2. Publish to /imu/quaternion topic
        // 3. Handle connection status
        
        // Mock implementation
        quaternion_data_t imu_data;
        if (g_app_ctx && g_app_ctx->queue_imu) {
            if (xQueuePeek(g_app_ctx->queue_imu, &imu_data, 0) == pdTRUE) {
                // Would publish IMU data here
                ESP_LOGD(TAG, "Publishing IMU: w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                         imu_data.w, imu_data.x, imu_data.y, imu_data.z);
            }
        }
        
        // Update statistics
        uint32_t execution_time = esp_timer_get_time() - start_time;
        task_stats[2].executions++;
        task_stats[2].total_time_us += execution_time;
        task_stats[2].last_execution_us = execution_time;
        if (execution_time > task_stats[2].max_time_us) {
            task_stats[2].max_time_us = execution_time;
        }
        
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/**
 * @brief Placeholder image processing task
 * Core 1: Handles JPEG decode, sphere mapping, and LED value calculation
 */
static void image_processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "Image processing task started on core %d", xPortGetCoreID());
    
    quaternion_data_t current_orientation = {1.0f, 0.0f, 0.0f, 0.0f, 0, 0xFF};
    
    while (1) {
        uint32_t start_time = esp_timer_get_time();
        
        // Get latest image frame
        image_frame_t* frame = NULL;
        if (g_app_ctx && xQueueReceive(g_app_ctx->queue_image, &frame, pdMS_TO_TICKS(50))) {
            
            // Get latest IMU data
            quaternion_data_t new_orientation;
            if (xQueueReceive(g_app_ctx->queue_imu, &new_orientation, 0) == pdTRUE) {
                current_orientation = new_orientation;
            }
            
            // TODO: Implement image processing pipeline
            // 1. Decode JPEG to RGB
            // 2. Apply sphere mapping with current orientation
            // 3. Generate LED values for all 800 LEDs
            // 4. Split into 4 SPI packets (200 LEDs each)
            
            // Mock implementation - generate test pattern
            if (xSemaphoreTake(g_app_ctx->mutex_led_buffer, pdMS_TO_TICKS(10))) {
                uint8_t active_buf = g_app_ctx->active_buffer;
                
                for (int i = 0; i < NUM_LEDS; i++) {
                    // Simple rotating pattern
                    float phase = (float)i / NUM_LEDS * 2.0f * 3.14159f + 
                                  (float)(esp_timer_get_time() / 100000) * 0.1f;
                    g_app_ctx->led_buffers[active_buf][i].r = (uint8_t)((sinf(phase) + 1.0f) * 127.5f);
                    g_app_ctx->led_buffers[active_buf][i].g = (uint8_t)((sinf(phase + 2.094f) + 1.0f) * 127.5f);
                    g_app_ctx->led_buffers[active_buf][i].b = (uint8_t)((sinf(phase + 4.188f) + 1.0f) * 127.5f);
                }
                
                // Swap buffers
                g_app_ctx->active_buffer = 1 - active_buf;
                
                xSemaphoreGive(g_app_ctx->mutex_led_buffer);
            }
            
            // Create SPI packets for each strip
            for (int strip = 0; strip < NUM_LED_STRIPS; strip++) {
                spi_packet_t packet = {0};
                packet.magic = 0xAA55;
                packet.frame_id = (uint8_t)(frame->frame_id & 0xFF);
                packet.strip_id = strip;
                packet.data_len = LEDS_PER_STRIP * 3;
                
                // Copy LED data for this strip
                int start_led = strip * LEDS_PER_STRIP;
                for (int i = 0; i < LEDS_PER_STRIP; i++) {
                    int led_idx = start_led + i;
                    packet.led_data[i * 3] = g_app_ctx->led_buffers[1 - g_app_ctx->active_buffer][led_idx].r;
                    packet.led_data[i * 3 + 1] = g_app_ctx->led_buffers[1 - g_app_ctx->active_buffer][led_idx].g;
                    packet.led_data[i * 3 + 2] = g_app_ctx->led_buffers[1 - g_app_ctx->active_buffer][led_idx].b;
                }
                
                // Calculate CRC16 (mock implementation)
                packet.crc16 = 0x1234;  // Would calculate actual CRC here
                
                // Send to SPI task
                if (g_app_ctx->queue_spi[strip]) {
                    xQueueSend(g_app_ctx->queue_spi[strip], &packet, pdMS_TO_TICKS(5));
                }
            }
            
            // Update frame counter
            if (g_app_ctx->system_status.frame_count < UINT32_MAX) {
                g_app_ctx->system_status.frame_count++;
            }
            
            // Set image ready flag
            xEventGroupSetBits(g_app_ctx->event_group_system, EVENT_BIT_IMAGE_READY);
        }
        
        // Update statistics
        uint32_t execution_time = esp_timer_get_time() - start_time;
        task_stats[3].executions++;
        task_stats[3].total_time_us += execution_time;
        task_stats[3].last_execution_us = execution_time;
        if (execution_time > task_stats[3].max_time_us) {
            task_stats[3].max_time_us = execution_time;
        }
        
        // Performance warning
        if (execution_time > MAX_PROCESSING_TIME_MS * 1000) {
            ESP_LOGW(TAG, "Image processing took %lu us (limit: %d us)", 
                     execution_time, MAX_PROCESSING_TIME_MS * 1000);
        }
    }
}

/**
 * @brief Placeholder SPI transfer task
 * Core 1: Handles DMA transfers to RP2350
 */
static void spi_transfer_task(void *pvParameters) {
    ESP_LOGI(TAG, "SPI transfer task started on core %d", xPortGetCoreID());
    
    while (1) {
        uint32_t start_time = esp_timer_get_time();
        bool sent_any = false;
        
        // Process all SPI queues
        for (int strip = 0; strip < NUM_LED_STRIPS; strip++) {
            spi_packet_t packet;
            if (g_app_ctx && g_app_ctx->queue_spi[strip]) {
                if (xQueueReceive(g_app_ctx->queue_spi[strip], &packet, 0) == pdTRUE) {
                    
                    // TODO: Implement SPI DMA transfer
                    // 1. Setup DMA transfer
                    // 2. Assert CS for RP2350
                    // 3. Transfer packet data
                    // 4. Wait for completion
                    // 5. Check ACK/NACK response
                    
                    // Mock implementation
                    ESP_LOGD(TAG, "Sending SPI packet: strip=%d, frame=%d, size=%d", 
                             packet.strip_id, packet.frame_id, packet.data_len);
                    
                    // Simulate transfer time (2ms for 608 bytes at 40MHz)
                    vTaskDelay(pdMS_TO_TICKS(2));
                    
                    sent_any = true;
                }
            }
        }
        
        // Update statistics
        uint32_t execution_time = esp_timer_get_time() - start_time;
        task_stats[4].executions++;
        task_stats[4].total_time_us += execution_time;
        task_stats[4].last_execution_us = execution_time;
        if (execution_time > task_stats[4].max_time_us) {
            task_stats[4].max_time_us = execution_time;
        }
        
        // If no packets to send, wait a bit
        if (!sent_any) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/**
 * @brief Placeholder display task
 * Core 1: Handles test round display
 */
static void display_task(void *pvParameters) {
    ESP_LOGI(TAG, "Display task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 10fps update
    
    while (1) {
        uint32_t start_time = esp_timer_get_time();
        
        // TODO: Implement round display updates
        // 1. Get current image frame (if available)
        // 2. Scale/crop for round display
        // 3. Update display via SPI
        // 4. Show debug information overlay
        
        // Mock implementation - just log status
        static uint32_t display_counter = 0;
        if (++display_counter % 100 == 0) {  // Every 10 seconds
            ESP_LOGI(TAG, "Display update #%lu", display_counter);
        }
        
        // Update statistics
        uint32_t execution_time = esp_timer_get_time() - start_time;
        task_stats[5].executions++;
        task_stats[5].total_time_us += execution_time;
        task_stats[5].last_execution_us = execution_time;
        if (execution_time > task_stats[5].max_time_us) {
            task_stats[5].max_time_us = execution_time;
        }
        
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/**
 * @brief Start all application tasks
 */
esp_err_t task_manager_start_all(void) {
    ESP_LOGI(TAG, "Starting all application tasks");
    
    // Start IMU task (Core 0)
    BaseType_t ret = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        TASK_STACK_SIZE_IMU,
        NULL,
        TASK_PRIORITY_IMU,
        &task_handle_imu,
        TASK_CORE_COMMUNICATION
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }
    
    // Start ROS2 RX task (Core 0)
    ret = xTaskCreatePinnedToCore(
        ros2_rx_task,
        "ros2_rx_task",
        TASK_STACK_SIZE_ROS2_RX,
        NULL,
        TASK_PRIORITY_ROS2_RX,
        &task_handle_ros2_rx,
        TASK_CORE_COMMUNICATION
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ROS2 RX task");
        return ESP_ERR_NO_MEM;
    }
    
    // Start ROS2 TX task (Core 0)
    ret = xTaskCreatePinnedToCore(
        ros2_tx_task,
        "ros2_tx_task", 
        TASK_STACK_SIZE_ROS2_TX,
        NULL,
        TASK_PRIORITY_ROS2_TX,
        &task_handle_ros2_tx,
        TASK_CORE_COMMUNICATION
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ROS2 TX task");
        return ESP_ERR_NO_MEM;
    }
    
    // Start Image Processing task (Core 1)
    ret = xTaskCreatePinnedToCore(
        image_processing_task,
        "image_proc_task",
        TASK_STACK_SIZE_IMAGE_PROC,
        NULL,
        TASK_PRIORITY_IMAGE_PROC,
        &task_handle_image_proc,
        TASK_CORE_PROCESSING
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Image Processing task");
        return ESP_ERR_NO_MEM;
    }
    
    // Start SPI Transfer task (Core 1)
    ret = xTaskCreatePinnedToCore(
        spi_transfer_task,
        "spi_tx_task",
        TASK_STACK_SIZE_SPI_TX,
        NULL,
        TASK_PRIORITY_SPI_TX,
        &task_handle_spi_tx,
        TASK_CORE_PROCESSING
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPI Transfer task");
        return ESP_ERR_NO_MEM;
    }
    
    // Start Display task (Core 1)
    ret = xTaskCreatePinnedToCore(
        display_task,
        "display_task",
        TASK_STACK_SIZE_DISPLAY,
        NULL,
        TASK_PRIORITY_DISPLAY,
        &task_handle_display,
        TASK_CORE_PROCESSING
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Display task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "All tasks started successfully");
    return ESP_OK;
}

/**
 * @brief Stop all application tasks
 */
esp_err_t task_manager_stop_all(void) {
    ESP_LOGI(TAG, "Stopping all application tasks");
    
    // Delete all tasks
    if (task_handle_imu) {
        vTaskDelete(task_handle_imu);
        task_handle_imu = NULL;
    }
    
    if (task_handle_ros2_rx) {
        vTaskDelete(task_handle_ros2_rx);
        task_handle_ros2_rx = NULL;
    }
    
    if (task_handle_ros2_tx) {
        vTaskDelete(task_handle_ros2_tx);
        task_handle_ros2_tx = NULL;
    }
    
    if (task_handle_image_proc) {
        vTaskDelete(task_handle_image_proc);
        task_handle_image_proc = NULL;
    }
    
    if (task_handle_spi_tx) {
        vTaskDelete(task_handle_spi_tx);
        task_handle_spi_tx = NULL;
    }
    
    if (task_handle_display) {
        vTaskDelete(task_handle_display);
        task_handle_display = NULL;
    }
    
    ESP_LOGI(TAG, "All tasks stopped");
    return ESP_OK;
}

/**
 * @brief Get task performance statistics
 */
esp_err_t task_manager_get_stats(task_manager_stats_t* stats) {
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const char* task_names[] = {
        "IMU", "ROS2_RX", "ROS2_TX", "IMAGE_PROC", "SPI_TX", "DISPLAY"
    };
    
    for (int i = 0; i < 6; i++) {
        strncpy(stats->tasks[i].name, task_names[i], sizeof(stats->tasks[i].name) - 1);
        stats->tasks[i].executions = task_stats[i].executions;
        stats->tasks[i].avg_time_us = task_stats[i].executions > 0 ? 
                                      task_stats[i].total_time_us / task_stats[i].executions : 0;
        stats->tasks[i].max_time_us = task_stats[i].max_time_us;
        stats->tasks[i].last_time_us = task_stats[i].last_execution_us;
    }
    
    stats->num_tasks = 6;
    return ESP_OK;
}