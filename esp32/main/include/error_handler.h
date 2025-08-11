/**
 * @file error_handler.h
 * @brief Error handling and logging system header
 * @author Isolation Sphere Team
 * @date 2024
 */

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ERROR_LOG_QUEUE_SIZE 50

/**
 * @brief Error severity levels
 */
typedef enum {
    ERROR_SEVERITY_INFO = 0,
    ERROR_SEVERITY_WARNING,
    ERROR_SEVERITY_ERROR,
    ERROR_SEVERITY_CRITICAL
} error_severity_t;

/**
 * @brief System health status
 */
typedef enum {
    SYSTEM_HEALTH_GOOD = 0,
    SYSTEM_HEALTH_WARNING,
    SYSTEM_HEALTH_DEGRADED,
    SYSTEM_HEALTH_CRITICAL
} system_health_t;

/**
 * @brief Error log entry structure
 */
typedef struct {
    uint64_t timestamp;
    error_severity_t severity;
    esp_err_t error_code;
    char module[16];
    char message[128];
    char file[32];
    int line_number;
} error_log_entry_t;

/**
 * @brief Error statistics structure
 */
typedef struct {
    uint32_t total_errors;
    uint32_t info_count;
    uint32_t warning_count;
    uint32_t error_count;
    uint32_t critical_count;
} error_statistics_t;

/**
 * @brief Initialize error handler
 * @return ESP_OK on success, error code on failure
 */
esp_err_t error_handler_init(void);

/**
 * @brief Cleanup error handler
 */
void error_handler_cleanup(void);

/**
 * @brief Log an error
 * @param severity Error severity level
 * @param module Module name
 * @param message Error message
 * @param error_code ESP error code
 * @param file Source file name
 * @param line Line number
 * @return ESP_OK on success, error code on failure
 */
esp_err_t error_handler_log_error(error_severity_t severity, 
                                  const char* module,
                                  const char* message,
                                  esp_err_t error_code,
                                  const char* file,
                                  int line);

/**
 * @brief Get error statistics
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t error_handler_get_stats(error_statistics_t* stats);

/**
 * @brief Clear error statistics
 * @return ESP_OK on success, error code on failure
 */
esp_err_t error_handler_clear_stats(void);

/**
 * @brief Get system health status
 * @return System health level
 */
system_health_t error_handler_get_system_health(void);

/**
 * @brief Print error statistics
 */
void error_handler_print_stats(void);

// Convenience macros for logging errors
#define LOG_ERROR_INFO(module, message, code) \
    error_handler_log_error(ERROR_SEVERITY_INFO, module, message, code, __FILE__, __LINE__)

#define LOG_ERROR_WARNING(module, message, code) \
    error_handler_log_error(ERROR_SEVERITY_WARNING, module, message, code, __FILE__, __LINE__)

#define LOG_ERROR_ERROR(module, message, code) \
    error_handler_log_error(ERROR_SEVERITY_ERROR, module, message, code, __FILE__, __LINE__)

#define LOG_ERROR_CRITICAL(module, message, code) \
    error_handler_log_error(ERROR_SEVERITY_CRITICAL, module, message, code, __FILE__, __LINE__)

#ifdef __cplusplus
}
#endif

#endif // ERROR_HANDLER_H