/**
 * @file task_manager.h
 * @brief Task management and coordination header
 * @author Isolation Sphere Team
 * @date 2024
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Task performance statistics structure
 */
typedef struct {
    char name[16];
    uint32_t executions;
    uint32_t avg_time_us;
    uint32_t max_time_us;
    uint32_t last_time_us;
} task_performance_t;

/**
 * @brief Task manager statistics
 */
typedef struct {
    task_performance_t tasks[8];  // Support up to 8 tasks
    uint8_t num_tasks;
} task_manager_stats_t;

/**
 * @brief Start all application tasks
 * @return ESP_OK on success, error code on failure
 */
esp_err_t task_manager_start_all(void);

/**
 * @brief Stop all application tasks
 * @return ESP_OK on success, error code on failure
 */
esp_err_t task_manager_stop_all(void);

/**
 * @brief Get task performance statistics
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t task_manager_get_stats(task_manager_stats_t* stats);

#ifdef __cplusplus
}
#endif

#endif // TASK_MANAGER_H