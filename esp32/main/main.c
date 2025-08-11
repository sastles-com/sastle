/**
 * @file main.c
 * @brief ESP32 Isolation Sphere Main Application
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "task_manager.h"
#include "system_config.h"
#include "error_handler.h"

static const char *TAG = "MAIN";

// Global application context
app_context_t* g_app_ctx = NULL;

/**
 * @brief Initialize application context
 */
esp_err_t app_context_init(void) {
    ESP_LOGI(TAG, "Initializing application context");
    
    // Allocate main context structure
    g_app_ctx = (app_context_t*)malloc(sizeof(app_context_t));
    if (!g_app_ctx) {
        ESP_LOGE(TAG, "Failed to allocate application context");
        return ESP_ERR_NO_MEM;
    }
    
    memset(g_app_ctx, 0, sizeof(app_context_t));
    
    // Create queues
    g_app_ctx->queue_image = xQueueCreate(QUEUE_SIZE_IMAGE, sizeof(image_frame_t*));
    g_app_ctx->queue_imu = xQueueCreate(QUEUE_SIZE_IMU, sizeof(quaternion_data_t));
    g_app_ctx->queue_ui_command = xQueueCreate(QUEUE_SIZE_UI_COMMAND, sizeof(ui_command_t));
    
    for (int i = 0; i < NUM_LED_STRIPS; i++) {
        g_app_ctx->queue_spi[i] = xQueueCreate(QUEUE_SIZE_SPI, sizeof(spi_packet_t));
    }
    
    // Create synchronization primitives
    g_app_ctx->mutex_led_buffer = xSemaphoreCreateMutex();
    g_app_ctx->mutex_system_status = xSemaphoreCreateMutex();
    g_app_ctx->event_group_system = xEventGroupCreate();
    
    // Verify all resources were created
    if (!g_app_ctx->queue_image || !g_app_ctx->queue_imu || 
        !g_app_ctx->queue_ui_command || !g_app_ctx->mutex_led_buffer ||
        !g_app_ctx->mutex_system_status || !g_app_ctx->event_group_system) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        app_context_cleanup();
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate LED position data
    g_app_ctx->led_positions = (led_position_t*)malloc(NUM_LEDS * sizeof(led_position_t));
    g_app_ctx->mapping_lut_u = (uint16_t*)malloc(NUM_LEDS * sizeof(uint16_t));
    g_app_ctx->mapping_lut_v = (uint16_t*)malloc(NUM_LEDS * sizeof(uint16_t));
    
    if (!g_app_ctx->led_positions || !g_app_ctx->mapping_lut_u || !g_app_ctx->mapping_lut_v) {
        ESP_LOGE(TAG, "Failed to allocate LED mapping data");
        app_context_cleanup();
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize system status
    g_app_ctx->system_status.wifi_connected = false;
    g_app_ctx->system_status.ros2_connected = false;
    g_app_ctx->system_status.imu_calibrated = false;
    g_app_ctx->system_status.fps_current = 0.0f;
    g_app_ctx->system_status.fps_average = 0.0f;
    g_app_ctx->system_status.frame_count = 0;
    g_app_ctx->system_status.dropped_frames = 0;
    
    // Set default configuration
    strcpy(g_app_ctx->wifi_ssid, "IsolationSphere");
    strcpy(g_app_ctx->wifi_password, "sphere123");
    strcpy(g_app_ctx->ros2_agent_ip, "192.168.1.100");
    g_app_ctx->ros2_agent_port = 8888;
    g_app_ctx->debug_mode = true;
    g_app_ctx->log_level = ESP_LOG_INFO;
    
    ESP_LOGI(TAG, "Application context initialized successfully");
    return ESP_OK;
}

/**
 * @brief Cleanup application context
 */
void app_context_cleanup(void) {
    if (!g_app_ctx) return;
    
    ESP_LOGI(TAG, "Cleaning up application context");
    
    // Cleanup queues
    if (g_app_ctx->queue_image) vQueueDelete(g_app_ctx->queue_image);
    if (g_app_ctx->queue_imu) vQueueDelete(g_app_ctx->queue_imu);
    if (g_app_ctx->queue_ui_command) vQueueDelete(g_app_ctx->queue_ui_command);
    
    for (int i = 0; i < NUM_LED_STRIPS; i++) {
        if (g_app_ctx->queue_spi[i]) vQueueDelete(g_app_ctx->queue_spi[i]);
    }
    
    // Cleanup synchronization primitives
    if (g_app_ctx->mutex_led_buffer) vSemaphoreDelete(g_app_ctx->mutex_led_buffer);
    if (g_app_ctx->mutex_system_status) vSemaphoreDelete(g_app_ctx->mutex_system_status);
    if (g_app_ctx->event_group_system) vEventGroupDelete(g_app_ctx->event_group_system);
    
    // Free allocated memory
    if (g_app_ctx->led_positions) free(g_app_ctx->led_positions);
    if (g_app_ctx->mapping_lut_u) free(g_app_ctx->mapping_lut_u);
    if (g_app_ctx->mapping_lut_v) free(g_app_ctx->mapping_lut_v);
    
    free(g_app_ctx);
    g_app_ctx = NULL;
}

/**
 * @brief Get application context
 */
app_context_t* app_get_context(void) {
    return g_app_ctx;
}

/**
 * @brief System initialization
 */
static esp_err_t system_init(void) {
    ESP_LOGI(TAG, "Initializing system components");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize network stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize application context
    ESP_ERROR_CHECK(app_context_init());
    
    // Load configuration from NVS
    ESP_ERROR_CHECK(system_config_load());
    
    // Initialize error handler
    error_handler_init();
    
    ESP_LOGI(TAG, "System initialization completed");
    return ESP_OK;
}

/**
 * @brief Print system information
 */
static void print_system_info(void) {
    // ESP32 chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "ESP32 Isolation Sphere Firmware");
    ESP_LOGI(TAG, "Version: %s", "1.0.0");
    ESP_LOGI(TAG, "Chip: %s Rev %d", CONFIG_IDF_TARGET, chip_info.revision);
    ESP_LOGI(TAG, "CPU Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "WiFi%s%s", 
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
             
    // Memory information
    ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
    #ifdef CONFIG_SPIRAM
    ESP_LOGI(TAG, "Free PSRAM: %d bytes", esp_psram_get_size() - esp_psram_get_size());
    #endif
    
    // Flash information
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash size: %d MB", flash_size / (1024 * 1024));
    
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  Target FPS: %.1f", TARGET_FPS);
    ESP_LOGI(TAG, "  Image size: %dx%d", IMAGE_WIDTH, IMAGE_HEIGHT);
    ESP_LOGI(TAG, "  LED count: %d (%d strips)", NUM_LEDS, NUM_LED_STRIPS);
    ESP_LOGI(TAG, "  IMU rate: %d Hz", IMU_SAMPLE_RATE_HZ);
}

/**
 * @brief Wait for system ready
 */
static void wait_system_ready(void) {
    ESP_LOGI(TAG, "Waiting for system components to be ready...");
    
    EventBits_t bits = xEventGroupWaitBits(
        g_app_ctx->event_group_system,
        EVENT_BIT_WIFI_CONNECTED | EVENT_BIT_IMU_READY,
        false,  // Don't clear bits
        true,   // Wait for all bits
        pdMS_TO_TICKS(30000)  // 30 second timeout
    );
    
    if ((bits & (EVENT_BIT_WIFI_CONNECTED | EVENT_BIT_IMU_READY)) == 
        (EVENT_BIT_WIFI_CONNECTED | EVENT_BIT_IMU_READY)) {
        ESP_LOGI(TAG, "System ready - all critical components initialized");
    } else {
        ESP_LOGW(TAG, "System partially ready (bits: 0x%lx)", bits);
        if (!(bits & EVENT_BIT_WIFI_CONNECTED)) {
            ESP_LOGW(TAG, "  WiFi not connected");
        }
        if (!(bits & EVENT_BIT_IMU_READY)) {
            ESP_LOGW(TAG, "  IMU not ready");
        }
    }
}

/**
 * @brief Performance monitoring task
 */
static void monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Monitor task started on core %d", xPortGetCoreID());
    
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_frame_count = 0;
    
    while (1) {
        // Update system status
        if (xSemaphoreTake(g_app_ctx->mutex_system_status, pdMS_TO_TICKS(100))) {
            // Calculate FPS
            uint32_t current_frames = g_app_ctx->system_status.frame_count;
            uint32_t frames_delta = current_frames - last_frame_count;
            g_app_ctx->system_status.fps_current = frames_delta;  // 1 second interval
            
            // Update average FPS (simple moving average)
            if (g_app_ctx->system_status.fps_average == 0.0f) {
                g_app_ctx->system_status.fps_average = g_app_ctx->system_status.fps_current;
            } else {
                g_app_ctx->system_status.fps_average = 
                    (g_app_ctx->system_status.fps_average * 0.9f) + 
                    (g_app_ctx->system_status.fps_current * 0.1f);
            }
            
            // Update memory status
            g_app_ctx->system_status.free_heap = esp_get_free_heap_size();
            #ifdef CONFIG_SPIRAM
            g_app_ctx->system_status.free_psram = esp_psram_get_free_size();
            #endif
            
            // Update uptime
            g_app_ctx->system_status.uptime_ms = esp_timer_get_time() / 1000;
            
            last_frame_count = current_frames;
            
            xSemaphoreGive(g_app_ctx->mutex_system_status);
        }
        
        // Log system status periodically (every 10 seconds)
        static uint32_t log_counter = 0;
        if (++log_counter >= 10) {
            ESP_LOGI(TAG, "System Status: FPS=%.1f/%.1f, Heap=%d, PSRAM=%d, Uptime=%lu min",
                     g_app_ctx->system_status.fps_current,
                     g_app_ctx->system_status.fps_average,
                     g_app_ctx->system_status.free_heap,
                     g_app_ctx->system_status.free_psram,
                     g_app_ctx->system_status.uptime_ms / (60 * 1000));
            log_counter = 0;
        }
        
        // Check for system errors
        EventBits_t error_bits = xEventGroupGetBits(g_app_ctx->event_group_system);
        if (error_bits & EVENT_BIT_SYSTEM_ERROR) {
            ESP_LOGE(TAG, "System error detected, initiating recovery");
            // Could implement recovery logic here
        }
        
        // Sleep for 1 second
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Application main function
 */
void app_main(void) {
    ESP_LOGI(TAG, "ESP32 Isolation Sphere starting...");
    
    // Initialize system
    ESP_ERROR_CHECK(system_init());
    
    // Print system information
    print_system_info();
    
    // Start all tasks
    ESP_LOGI(TAG, "Starting application tasks");
    ESP_ERROR_CHECK(task_manager_start_all());
    
    // Start monitor task
    xTaskCreatePinnedToCore(
        monitor_task,
        "monitor_task",
        TASK_STACK_SIZE_MONITOR,
        NULL,
        TASK_PRIORITY_MONITOR,
        NULL,
        TASK_CORE_COMMUNICATION
    );
    
    // Wait for system to be ready
    wait_system_ready();
    
    ESP_LOGI(TAG, "Application startup completed");
    
    // Main loop - just monitoring
    while (1) {
        // Check for shutdown signal
        EventBits_t bits = xEventGroupWaitBits(
            g_app_ctx->event_group_system,
            EVENT_BIT_SHUTDOWN,
            false,  // Don't clear
            false,  // Wait for any bit
            pdMS_TO_TICKS(1000)
        );
        
        if (bits & EVENT_BIT_SHUTDOWN) {
            ESP_LOGI(TAG, "Shutdown signal received");
            break;
        }
    }
    
    // Cleanup
    ESP_LOGI(TAG, "Shutting down application");
    task_manager_stop_all();
    app_context_cleanup();
    
    ESP_LOGI(TAG, "Application shutdown completed");
    
    // Restart system
    esp_restart();
}