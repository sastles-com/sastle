/**
 * @file error_handler.c
 * @brief Error handling and logging system
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "error_handler.h"
#include "app_config.h"

static const char *TAG = "ERROR_HANDLER";

// Error log queue
static QueueHandle_t error_log_queue = NULL;
static TaskHandle_t error_handler_task_handle = NULL;

// Error statistics
static error_statistics_t error_stats = {0};

// Critical error threshold (number of errors in 1 minute)
#define CRITICAL_ERROR_THRESHOLD 10
#define ERROR_TIME_WINDOW_MS (60 * 1000)

/**
 * @brief Error handler task
 */
static void error_handler_task(void *pvParameters) {
    ESP_LOGI(TAG, "Error handler task started on core %d", xPortGetCoreID());
    
    error_log_entry_t error_entry;
    
    while (1) {
        if (xQueueReceive(error_log_queue, &error_entry, pdMS_TO_TICKS(1000))) {
            
            // Update statistics
            error_stats.total_errors++;
            
            switch (error_entry.severity) {
                case ERROR_SEVERITY_INFO:
                    error_stats.info_count++;
                    break;
                case ERROR_SEVERITY_WARNING:
                    error_stats.warning_count++;
                    break;
                case ERROR_SEVERITY_ERROR:
                    error_stats.error_count++;
                    break;
                case ERROR_SEVERITY_CRITICAL:
                    error_stats.critical_count++;
                    break;
            }
            
            // Log the error
            const char* severity_str[] = {"INFO", "WARN", "ERROR", "CRITICAL"};
            ESP_LOGE(TAG, "[%s] %s: %s (code: 0x%x, line: %d)",
                     severity_str[error_entry.severity],
                     error_entry.module,
                     error_entry.message,
                     error_entry.error_code,
                     error_entry.line_number);
            
            // Handle critical errors
            if (error_entry.severity == ERROR_SEVERITY_CRITICAL) {
                ESP_LOGE(TAG, "CRITICAL ERROR DETECTED - Initiating recovery procedures");
                
                // Set system error flag
                if (g_app_ctx && g_app_ctx->event_group_system) {
                    xEventGroupSetBits(g_app_ctx->event_group_system, EVENT_BIT_SYSTEM_ERROR);
                }
                
                // Could implement automatic recovery here
                // For now, just log additional system information
                ESP_LOGE(TAG, "System state at error:");
                ESP_LOGE(TAG, "  Free heap: %d bytes", esp_get_free_heap_size());
                #ifdef CONFIG_SPIRAM
                ESP_LOGE(TAG, "  Free PSRAM: %d bytes", esp_psram_get_free_size());
                #endif
                ESP_LOGE(TAG, "  Uptime: %llu ms", esp_timer_get_time() / 1000);
                
                // Optionally restart system after critical error
                if (error_stats.critical_count >= 3) {
                    ESP_LOGE(TAG, "Multiple critical errors - Restarting system in 5 seconds");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    esp_restart();
                }
            }
            
            // Check for error flood (too many errors in short time)
            uint32_t current_time = esp_timer_get_time() / 1000;
            static uint32_t last_check_time = 0;
            static uint32_t recent_error_count = 0;
            
            if (current_time - last_check_time > ERROR_TIME_WINDOW_MS) {
                // Reset counter every minute
                recent_error_count = 1;
                last_check_time = current_time;
            } else {
                recent_error_count++;
                if (recent_error_count > CRITICAL_ERROR_THRESHOLD) {
                    ESP_LOGE(TAG, "ERROR FLOOD DETECTED - %lu errors in last minute", 
                             recent_error_count);
                    
                    // Could implement flood protection here
                    // For now, just reduce logging frequency
                    vTaskDelay(pdMS_TO_TICKS(5000));
                }
            }
        }
    }
}

/**
 * @brief Initialize error handler
 */
esp_err_t error_handler_init(void) {
    ESP_LOGI(TAG, "Initializing error handler");
    
    // Create error log queue
    error_log_queue = xQueueCreate(ERROR_LOG_QUEUE_SIZE, sizeof(error_log_entry_t));
    if (!error_log_queue) {
        ESP_LOGE(TAG, "Failed to create error log queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Start error handler task
    BaseType_t ret = xTaskCreatePinnedToCore(
        error_handler_task,
        "error_handler",
        2048,  // Stack size
        NULL,
        15,    // High priority
        &error_handler_task_handle,
        TASK_CORE_COMMUNICATION  // Core 0
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create error handler task");
        vQueueDelete(error_log_queue);
        error_log_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Error handler initialized successfully");
    return ESP_OK;
}

/**
 * @brief Cleanup error handler
 */
void error_handler_cleanup(void) {
    ESP_LOGI(TAG, "Cleaning up error handler");
    
    if (error_handler_task_handle) {
        vTaskDelete(error_handler_task_handle);
        error_handler_task_handle = NULL;
    }
    
    if (error_log_queue) {
        vQueueDelete(error_log_queue);
        error_log_queue = NULL;
    }
}

/**
 * @brief Log an error
 */
esp_err_t error_handler_log_error(error_severity_t severity, 
                                  const char* module,
                                  const char* message,
                                  esp_err_t error_code,
                                  const char* file,
                                  int line) {
    if (!error_log_queue) {
        // Fallback to direct ESP_LOG if handler not initialized
        ESP_LOGE(module, "ERROR_HANDLER_NOT_INIT: %s (code: 0x%x)", message, error_code);
        return ESP_ERR_INVALID_STATE;
    }
    
    error_log_entry_t entry = {0};
    entry.timestamp = esp_timer_get_time();
    entry.severity = severity;
    entry.error_code = error_code;
    entry.line_number = line;
    
    // Copy strings safely
    strncpy(entry.module, module ? module : "UNKNOWN", sizeof(entry.module) - 1);
    strncpy(entry.message, message ? message : "No message", sizeof(entry.message) - 1);
    strncpy(entry.file, file ? file : "unknown", sizeof(entry.file) - 1);
    
    // Ensure null termination
    entry.module[sizeof(entry.module) - 1] = '\0';
    entry.message[sizeof(entry.message) - 1] = '\0';
    entry.file[sizeof(entry.file) - 1] = '\0';
    
    // Send to error handler task
    BaseType_t result = xQueueSend(error_log_queue, &entry, pdMS_TO_TICKS(100));
    if (result != pdTRUE) {
        // Queue full, try to make space by removing oldest entry
        error_log_entry_t dummy;
        if (xQueueReceive(error_log_queue, &dummy, 0) == pdTRUE) {
            result = xQueueSend(error_log_queue, &entry, 0);
        }
        
        if (result != pdTRUE) {
            ESP_LOGW(TAG, "Error log queue full, dropping error");
            return ESP_ERR_NO_MEM;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Get error statistics
 */
esp_err_t error_handler_get_stats(error_statistics_t* stats) {
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *stats = error_stats;
    return ESP_OK;
}

/**
 * @brief Clear error statistics
 */
esp_err_t error_handler_clear_stats(void) {
    memset(&error_stats, 0, sizeof(error_stats));
    ESP_LOGI(TAG, "Error statistics cleared");
    return ESP_OK;
}

/**
 * @brief Check system health based on error patterns
 */
system_health_t error_handler_get_system_health(void) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    static uint32_t last_health_check = 0;
    static system_health_t cached_health = SYSTEM_HEALTH_GOOD;
    
    // Update health status every 10 seconds
    if (current_time - last_health_check < 10000) {
        return cached_health;
    }
    
    last_health_check = current_time;
    
    // Analyze error patterns
    if (error_stats.critical_count > 0) {
        cached_health = SYSTEM_HEALTH_CRITICAL;
    } else if (error_stats.error_count > 10) {
        cached_health = SYSTEM_HEALTH_DEGRADED;
    } else if (error_stats.warning_count > 20) {
        cached_health = SYSTEM_HEALTH_WARNING;
    } else {
        cached_health = SYSTEM_HEALTH_GOOD;
    }
    
    // Additional health checks
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < 10000) {  // Less than 10KB free
        if (cached_health == SYSTEM_HEALTH_GOOD) {
            cached_health = SYSTEM_HEALTH_WARNING;
        }
    }
    
    #ifdef CONFIG_SPIRAM
    size_t free_psram = esp_psram_get_free_size();
    if (free_psram < 100000) {  // Less than 100KB free PSRAM
        if (cached_health == SYSTEM_HEALTH_GOOD) {
            cached_health = SYSTEM_HEALTH_WARNING;
        }
    }
    #endif
    
    return cached_health;
}

/**
 * @brief Print error statistics
 */
void error_handler_print_stats(void) {
    system_health_t health = error_handler_get_system_health();
    const char* health_str[] = {"GOOD", "WARNING", "DEGRADED", "CRITICAL"};
    
    ESP_LOGI(TAG, "Error Handler Statistics:");
    ESP_LOGI(TAG, "========================");
    ESP_LOGI(TAG, "System Health: %s", health_str[health]);
    ESP_LOGI(TAG, "Total Errors: %lu", error_stats.total_errors);
    ESP_LOGI(TAG, "  Info: %lu", error_stats.info_count);
    ESP_LOGI(TAG, "  Warnings: %lu", error_stats.warning_count);
    ESP_LOGI(TAG, "  Errors: %lu", error_stats.error_count);
    ESP_LOGI(TAG, "  Critical: %lu", error_stats.critical_count);
    ESP_LOGI(TAG, "========================");
}