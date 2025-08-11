/**
 * @file quaternion_filter.h
 * @brief Quaternion filtering and processing header
 * @author Isolation Sphere Team
 * @date 2024
 */

#ifndef QUATERNION_FILTER_H
#define QUATERNION_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bno055.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * Filter Types and Constants
 * =============================================================================*/

/**
 * @brief Quaternion filter types
 */
typedef enum {
    QUAT_FILTER_NONE = 0,           // No filtering (pass-through)
    QUAT_FILTER_LOW_PASS,           // Simple low-pass filter using SLERP
    QUAT_FILTER_COMPLEMENTARY       // Complementary filter (for future use)
} quaternion_filter_type_t;

/* =============================================================================
 * Data Structures
 * =============================================================================*/

/**
 * @brief Quaternion filter configuration
 */
typedef struct {
    quaternion_filter_type_t filter_type;  // Filter type
    float smoothing_alpha;                  // Smoothing factor (0.0 to 1.0)
    float change_threshold;                 // Angular change threshold in degrees
    uint32_t max_dt_ms;                    // Maximum time delta in milliseconds
    bool reject_outliers;                   // Enable outlier rejection
} quaternion_filter_config_t;

/**
 * @brief Quaternion filter state
 */
typedef struct {
    quaternion_filter_config_t config;     // Filter configuration
    bno055_quaternion_t filtered_quat;     // Current filtered quaternion
    bno055_quaternion_t last_quat;         // Previous quaternion sample
    uint32_t last_update_time;             // Last update timestamp
    uint32_t sample_count;                 // Number of processed samples
    bool initialized;                       // Filter initialization status
} quaternion_filter_t;

/**
 * @brief Filter statistics
 */
typedef struct quaternion_filter_stats_s {
    uint32_t sample_count;              // Total samples processed
    bool is_initialized;                // Filter initialization status
    uint32_t last_update_time;          // Last update timestamp
    bno055_quaternion_t current_quaternion; // Current filtered quaternion
    bno055_vector3_t current_euler;      // Current Euler angles (degrees)
} quaternion_filter_stats_t;

/* =============================================================================
 * Function Declarations
 * =============================================================================*/

/**
 * @brief Normalize a quaternion
 * @param q Pointer to quaternion to normalize
 */
void quaternion_normalize(bno055_quaternion_t* q);

/**
 * @brief Calculate quaternion magnitude
 * @param q Pointer to quaternion
 * @return Quaternion magnitude
 */
float quaternion_magnitude(const bno055_quaternion_t* q);

/**
 * @brief Calculate dot product of two quaternions
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Dot product
 */
float quaternion_dot(const bno055_quaternion_t* q1, const bno055_quaternion_t* q2);

/**
 * @brief Spherical Linear Interpolation (SLERP) between two quaternions
 * @param q1 Start quaternion
 * @param q2 End quaternion
 * @param t Interpolation factor (0.0 to 1.0)
 * @param result Pointer to store result
 */
void quaternion_slerp(const bno055_quaternion_t* q1, const bno055_quaternion_t* q2, 
                      float t, bno055_quaternion_t* result);

/**
 * @brief Apply quaternion rotation to a 3D vector
 * @param q Rotation quaternion
 * @param v_in Input vector
 * @param v_out Output vector
 */
void quaternion_rotate_vector(const bno055_quaternion_t* q, const bno055_vector3_t* v_in, 
                              bno055_vector3_t* v_out);

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param q Input quaternion
 * @param euler Output Euler angles in degrees
 */
void quaternion_to_euler(const bno055_quaternion_t* q, bno055_vector3_t* euler);

/**
 * @brief Initialize quaternion filter
 * @param filter Pointer to filter structure
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t quaternion_filter_init(quaternion_filter_t* filter, const quaternion_filter_config_t* config);

/**
 * @brief Reset quaternion filter
 * @param filter Pointer to filter structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t quaternion_filter_reset(quaternion_filter_t* filter);

/**
 * @brief Process new quaternion sample through filter
 * @param filter Pointer to filter structure
 * @param raw_quat Raw quaternion input
 * @param timestamp Timestamp in milliseconds
 * @param filtered_quat Filtered quaternion output
 * @return ESP_OK on success, error code on failure
 */
esp_err_t quaternion_filter_update(quaternion_filter_t* filter, const bno055_quaternion_t* raw_quat,
                                   uint32_t timestamp, bno055_quaternion_t* filtered_quat);

/**
 * @brief Get filter statistics
 * @param filter Pointer to filter structure
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t quaternion_filter_get_stats(const quaternion_filter_t* filter, 
                                      quaternion_filter_stats_t* stats);

/**
 * @brief Apply offset quaternion to current quaternion
 * @param input Input quaternion
 * @param offset Offset quaternion
 * @param output Output quaternion (input * offset)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t quaternion_apply_offset(const bno055_quaternion_t* input, 
                                  const bno055_quaternion_t* offset,
                                  bno055_quaternion_t* output);

/**
 * @brief Get default filter configuration
 * @param config Pointer to configuration structure
 */
void quaternion_filter_get_default_config(quaternion_filter_config_t* config);

#ifdef __cplusplus
}
#endif

#endif // QUATERNION_FILTER_H