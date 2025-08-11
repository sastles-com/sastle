/**
 * @file quaternion_filter.c
 * @brief Quaternion filtering and processing implementation
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "quaternion_filter.h"

static const char *TAG = "QUAT_FILTER";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Normalize a quaternion
 */
void quaternion_normalize(bno055_quaternion_t* q) {
    if (!q) return;
    
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    
    if (norm > 0.0001f) {  // Avoid division by zero
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    } else {
        // Default to identity quaternion
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}

/**
 * @brief Calculate quaternion magnitude
 */
float quaternion_magnitude(const bno055_quaternion_t* q) {
    if (!q) return 0.0f;
    return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

/**
 * @brief Calculate dot product of two quaternions
 */
float quaternion_dot(const bno055_quaternion_t* q1, const bno055_quaternion_t* q2) {
    if (!q1 || !q2) return 0.0f;
    return q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;
}

/**
 * @brief Spherical Linear Interpolation (SLERP) between two quaternions
 */
void quaternion_slerp(const bno055_quaternion_t* q1, const bno055_quaternion_t* q2, 
                      float t, bno055_quaternion_t* result) {
    if (!q1 || !q2 || !result) return;
    
    // Clamp t to [0, 1]
    if (t <= 0.0f) {
        *result = *q1;
        return;
    }
    if (t >= 1.0f) {
        *result = *q2;
        return;
    }
    
    // Calculate dot product
    float dot = quaternion_dot(q1, q2);
    
    // If dot is negative, slerp won't take the shorter path
    bno055_quaternion_t q2_copy = *q2;
    if (dot < 0.0f) {
        q2_copy.w = -q2_copy.w;
        q2_copy.x = -q2_copy.x;
        q2_copy.y = -q2_copy.y;
        q2_copy.z = -q2_copy.z;
        dot = -dot;
    }
    
    // If quaternions are very close, use linear interpolation
    if (dot > 0.9995f) {
        result->w = q1->w + t * (q2_copy.w - q1->w);
        result->x = q1->x + t * (q2_copy.x - q1->x);
        result->y = q1->y + t * (q2_copy.y - q1->y);
        result->z = q1->z + t * (q2_copy.z - q1->z);
        quaternion_normalize(result);
        return;
    }
    
    // Spherical interpolation
    float theta_0 = acosf(fabsf(dot));
    float sin_theta_0 = sinf(theta_0);
    float theta = theta_0 * t;
    float sin_theta = sinf(theta);
    
    float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;
    
    result->w = s0 * q1->w + s1 * q2_copy.w;
    result->x = s0 * q1->x + s1 * q2_copy.x;
    result->y = s0 * q1->y + s1 * q2_copy.y;
    result->z = s0 * q1->z + s1 * q2_copy.z;
    
    quaternion_normalize(result);
}

/**
 * @brief Apply quaternion rotation to a 3D vector
 */
void quaternion_rotate_vector(const bno055_quaternion_t* q, const bno055_vector_t* v_in, 
                              bno055_vector_t* v_out) {
    if (!q || !v_in || !v_out) return;
    
    // Convert vector to quaternion (w=0, x,y,z = vector)
    bno055_quaternion_t v_quat = {0.0f, v_in->x, v_in->y, v_in->z};
    
    // Calculate q * v * q_conjugate
    // First: q * v
    bno055_quaternion_t temp1;
    temp1.w = -q->x * v_quat.x - q->y * v_quat.y - q->z * v_quat.z;
    temp1.x =  q->w * v_quat.x + q->y * v_quat.z - q->z * v_quat.y;
    temp1.y =  q->w * v_quat.y + q->z * v_quat.x - q->x * v_quat.z;
    temp1.z =  q->w * v_quat.z + q->x * v_quat.y - q->y * v_quat.x;
    
    // Then: temp1 * q_conjugate
    bno055_quaternion_t result;
    result.w = temp1.w * q->w + temp1.x * q->x + temp1.y * q->y + temp1.z * q->z;
    result.x = temp1.x * q->w - temp1.w * q->x + temp1.y * q->z - temp1.z * q->y;
    result.y = temp1.y * q->w - temp1.w * q->y + temp1.z * q->x - temp1.x * q->z;
    result.z = temp1.z * q->w - temp1.w * q->z + temp1.x * q->y - temp1.y * q->x;
    
    v_out->x = result.x;
    v_out->y = result.y;
    v_out->z = result.z;
}

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 */
void quaternion_to_euler(const bno055_quaternion_t* q, bno055_vector_t* euler) {
    if (!q || !euler) return;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1 - 2 * (q->x * q->x + q->y * q->y);
    euler->x = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1)
        euler->y = copysignf(M_PI / 2, sinp) * 180.0f / M_PI; // Use 90 degrees if out of range
    else
        euler->y = asinf(sinp) * 180.0f / M_PI;
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
    euler->z = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

/**
 * @brief Initialize quaternion filter
 */
esp_err_t quaternion_filter_init(quaternion_filter_t* filter, const quaternion_filter_config_t* config) {
    if (!filter || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(filter, 0, sizeof(quaternion_filter_t));
    filter->config = *config;
    
    // Initialize with identity quaternion
    filter->filtered_quat.w = 1.0f;
    filter->filtered_quat.x = 0.0f;
    filter->filtered_quat.y = 0.0f;
    filter->filtered_quat.z = 0.0f;
    
    filter->initialized = false;
    filter->sample_count = 0;
    
    ESP_LOGI(TAG, "Quaternion filter initialized: alpha=%.3f, threshold=%.3f", 
             config->smoothing_alpha, config->change_threshold);
    
    return ESP_OK;
}

/**
 * @brief Reset quaternion filter
 */
esp_err_t quaternion_filter_reset(quaternion_filter_t* filter) {
    if (!filter) {
        return ESP_ERR_INVALID_ARG;
    }
    
    filter->filtered_quat.w = 1.0f;
    filter->filtered_quat.x = 0.0f;
    filter->filtered_quat.y = 0.0f;
    filter->filtered_quat.z = 0.0f;
    
    filter->initialized = false;
    filter->sample_count = 0;
    filter->last_update_time = 0;
    
    ESP_LOGI(TAG, "Quaternion filter reset");
    
    return ESP_OK;
}

/**
 * @brief Process new quaternion sample through filter
 */
esp_err_t quaternion_filter_update(quaternion_filter_t* filter, const bno055_quaternion_t* raw_quat,
                                   uint32_t timestamp, bno055_quaternion_t* filtered_quat) {
    if (!filter || !raw_quat || !filtered_quat) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Normalize input quaternion
    bno055_quaternion_t normalized_quat = *raw_quat;
    quaternion_normalize(&normalized_quat);
    
    // Check for invalid quaternion
    float magnitude = quaternion_magnitude(&normalized_quat);
    if (magnitude < 0.1f) {
        ESP_LOGW(TAG, "Invalid quaternion magnitude: %.6f", magnitude);
        return ESP_ERR_INVALID_ARG;
    }
    
    // First sample - initialize filter
    if (!filter->initialized) {
        filter->filtered_quat = normalized_quat;
        filter->last_quat = normalized_quat;
        filter->last_update_time = timestamp;
        filter->initialized = true;
        filter->sample_count = 1;
        *filtered_quat = filter->filtered_quat;
        ESP_LOGI(TAG, "Filter initialized with first quaternion sample");
        return ESP_OK;
    }
    
    // Calculate time delta
    uint32_t dt = timestamp - filter->last_update_time;
    if (dt > filter->config.max_dt_ms) {
        ESP_LOGW(TAG, "Large time gap detected: %lu ms, resetting filter", dt);
        return quaternion_filter_reset(filter);
    }
    
    // Check for sudden large changes (possible glitch)
    float dot = quaternion_dot(&filter->last_quat, &normalized_quat);
    float angular_change = acosf(fabsf(dot)) * 2.0f * 180.0f / M_PI;  // Convert to degrees
    
    if (angular_change > filter->config.change_threshold) {
        ESP_LOGW(TAG, "Large quaternion change detected: %.1f degrees", angular_change);
        
        if (filter->config.reject_outliers) {
            ESP_LOGW(TAG, "Rejecting outlier quaternion");
            *filtered_quat = filter->filtered_quat;
            return ESP_OK;  // Keep previous value
        }
    }
    
    // Apply filtering based on method
    switch (filter->config.filter_type) {
        case QUAT_FILTER_NONE:
            filter->filtered_quat = normalized_quat;
            break;
            
        case QUAT_FILTER_LOW_PASS:
            // Simple low-pass filter using SLERP
            quaternion_slerp(&filter->filtered_quat, &normalized_quat, 
                           filter->config.smoothing_alpha, &filter->filtered_quat);
            break;
            
        case QUAT_FILTER_COMPLEMENTARY:
            // Complementary filter (for future implementation with gyro integration)
            quaternion_slerp(&filter->filtered_quat, &normalized_quat, 
                           filter->config.smoothing_alpha, &filter->filtered_quat);
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown filter type: %d", filter->config.filter_type);
            return ESP_ERR_INVALID_ARG;
    }
    
    // Ensure filtered quaternion is normalized
    quaternion_normalize(&filter->filtered_quat);
    
    // Update filter state
    filter->last_quat = normalized_quat;
    filter->last_update_time = timestamp;
    filter->sample_count++;
    
    *filtered_quat = filter->filtered_quat;
    
    return ESP_OK;
}

/**
 * @brief Get filter statistics
 */
esp_err_t quaternion_filter_get_stats(const quaternion_filter_t* filter, 
                                      quaternion_filter_stats_t* stats) {
    if (!filter || !stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(stats, 0, sizeof(quaternion_filter_stats_t));
    
    stats->sample_count = filter->sample_count;
    stats->is_initialized = filter->initialized;
    stats->last_update_time = filter->last_update_time;
    
    if (filter->initialized) {
        stats->current_quaternion = filter->filtered_quat;
        quaternion_to_euler(&filter->filtered_quat, &stats->current_euler);
    }
    
    return ESP_OK;
}

/**
 * @brief Apply offset quaternion to current quaternion
 */
esp_err_t quaternion_apply_offset(const bno055_quaternion_t* input, 
                                  const bno055_quaternion_t* offset,
                                  bno055_quaternion_t* output) {
    if (!input || !offset || !output) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Multiply quaternions: output = offset * input
    output->w = offset->w * input->w - offset->x * input->x - offset->y * input->y - offset->z * input->z;
    output->x = offset->w * input->x + offset->x * input->w + offset->y * input->z - offset->z * input->y;
    output->y = offset->w * input->y - offset->x * input->z + offset->y * input->w + offset->z * input->x;
    output->z = offset->w * input->z + offset->x * input->y - offset->y * input->x + offset->z * input->w;
    
    quaternion_normalize(output);
    
    return ESP_OK;
}

/**
 * @brief Get default filter configuration
 */
void quaternion_filter_get_default_config(quaternion_filter_config_t* config) {
    if (!config) return;
    
    config->filter_type = QUAT_FILTER_LOW_PASS;
    config->smoothing_alpha = 0.1f;  // 10% of new value, 90% of old value
    config->change_threshold = 30.0f;  // 30 degrees
    config->max_dt_ms = 200;  // 200ms max time gap
    config->reject_outliers = true;
}