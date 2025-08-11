#include "quaternion_visualizer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "QUAT_VIS";

/**
 * @brief Smooth interpolation (Low-pass filter)
 */
static float smooth_value(float current, float target, float alpha)
{
    return current + alpha * (target - current);
}

/**
 * @brief Convert quaternion to Euler angles
 */
void quat_vis_euler_from_quaternion(const simple_quaternion_t* q, float* roll, float* pitch, float* yaw)
{
    if (!q || !roll || !pitch || !yaw) return;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1 - 2 * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1)
        *pitch = copysignf(M_PI / 2, sinp) * 180.0f / M_PI;
    else
        *pitch = asinf(sinp) * 180.0f / M_PI;
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

/**
 * @brief Draw progress bar
 */
void quat_vis_draw_progress_bar(quaternion_visualizer_t* vis, int16_t x, int16_t y, int16_t width, int16_t height, float value, uint16_t color)
{
    if (!vis || !vis->lcd) return;
    
    // Clamp value to [0, 1]
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;
    
    // Draw border
    m5atoms3r_lcd_fill_rect(vis->lcd, x, y, width, 1, LCD_COLOR_GRAY);
    m5atoms3r_lcd_fill_rect(vis->lcd, x, y + height - 1, width, 1, LCD_COLOR_GRAY);
    m5atoms3r_lcd_fill_rect(vis->lcd, x, y, 1, height, LCD_COLOR_GRAY);
    m5atoms3r_lcd_fill_rect(vis->lcd, x + width - 1, y, 1, height, LCD_COLOR_GRAY);
    
    // Fill background
    m5atoms3r_lcd_fill_rect(vis->lcd, x + 1, y + 1, width - 2, height - 2, LCD_COLOR_DARKGRAY);
    
    // Draw progress
    int16_t progress_width = (int16_t)((width - 2) * value);
    if (progress_width > 0) {
        m5atoms3r_lcd_fill_rect(vis->lcd, x + 1, y + 1, progress_width, height - 2, color);
    }
}

/**
 * @brief Initialize quaternion visualizer
 */
esp_err_t quat_vis_init(quaternion_visualizer_t* vis, m5atoms3r_lcd_t* lcd, simple_bno055_t* imu)
{
    if (!vis || !lcd || !imu) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing quaternion visualizer...");
    
    memset(vis, 0, sizeof(quaternion_visualizer_t));
    vis->lcd = lcd;
    vis->imu = imu;
    vis->frame_count = 0;
    vis->last_update_time = esp_timer_get_time() / 1000;
    
    // Initialize smoothed values
    vis->smooth_quat.w = 1.0f;
    vis->smooth_quat.x = 0.0f;
    vis->smooth_quat.y = 0.0f;
    vis->smooth_quat.z = 0.0f;
    vis->smooth_roll = 0.0f;
    vis->smooth_pitch = 0.0f;
    vis->smooth_yaw = 0.0f;
    
    vis->imu_connected = false;
    vis->initialized = true;
    
    // Clear screen and draw initial interface
    esp_err_t ret = quat_vis_clear_screen(vis);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear screen: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Quaternion visualizer initialized");
    return ESP_OK;
}

/**
 * @brief Clear screen and draw static elements
 */
esp_err_t quat_vis_clear_screen(quaternion_visualizer_t* vis)
{
    if (!vis || !vis->lcd) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clear screen
    m5atoms3r_lcd_clear(vis->lcd, VIS_BG_COLOR);
    
    // Draw header (larger text)
    m5atoms3r_lcd_printf(vis->lcd, 2, 2, VIS_HEADER_COLOR, VIS_BG_COLOR, 2, "IMU Display");
    
    // Draw section labels (larger text)
    m5atoms3r_lcd_printf(vis->lcd, 2, VIS_QUAT_DISPLAY_Y - 16, VIS_TEXT_COLOR, VIS_BG_COLOR, 2, "Quaternion:");
    m5atoms3r_lcd_printf(vis->lcd, 2, VIS_EULER_DISPLAY_Y - 16, VIS_TEXT_COLOR, VIS_BG_COLOR, 2, "Euler (deg):");
    m5atoms3r_lcd_printf(vis->lcd, 2, VIS_CALIB_DISPLAY_Y - 16, VIS_TEXT_COLOR, VIS_BG_COLOR, 2, "Calib:");
    m5atoms3r_lcd_printf(vis->lcd, 2, VIS_SYSTEM_INFO_Y - 16, VIS_TEXT_COLOR, VIS_BG_COLOR, 2, "System:");
    
    return ESP_OK;
}

/**
 * @brief Update visualizer data
 */
esp_err_t quat_vis_update(quaternion_visualizer_t* vis)
{
    if (!vis || !vis->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read quaternion data
    simple_quaternion_t current_quat;
    esp_err_t ret = simple_bno055_read_quaternion(vis->imu, &current_quat);
    if (ret == ESP_OK) {
        vis->imu_connected = true;
        
        // Apply smoothing
        vis->smooth_quat.w = smooth_value(vis->smooth_quat.w, current_quat.w, VIS_SMOOTHING_ALPHA);
        vis->smooth_quat.x = smooth_value(vis->smooth_quat.x, current_quat.x, VIS_SMOOTHING_ALPHA);
        vis->smooth_quat.y = smooth_value(vis->smooth_quat.y, current_quat.y, VIS_SMOOTHING_ALPHA);
        vis->smooth_quat.z = smooth_value(vis->smooth_quat.z, current_quat.z, VIS_SMOOTHING_ALPHA);
        
        // Convert to Euler angles
        float roll, pitch, yaw;
        quat_vis_euler_from_quaternion(&vis->smooth_quat, &roll, &pitch, &yaw);
        
        vis->smooth_roll = smooth_value(vis->smooth_roll, roll, VIS_SMOOTHING_ALPHA);
        vis->smooth_pitch = smooth_value(vis->smooth_pitch, pitch, VIS_SMOOTHING_ALPHA);
        vis->smooth_yaw = smooth_value(vis->smooth_yaw, yaw, VIS_SMOOTHING_ALPHA);
        
    } else {
        vis->imu_connected = false;
    }
    
    // Read calibration status
    simple_bno055_read_calibration_status(vis->imu, &vis->last_calib);
    
    vis->frame_count++;
    vis->last_update_time = esp_timer_get_time() / 1000;
    
    return ESP_OK;
}

/**
 * @brief Render visualization frame
 */
esp_err_t quat_vis_render_frame(quaternion_visualizer_t* vis)
{
    if (!vis || !vis->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Display connection status (larger text)
    if (vis->imu_connected) {
        m5atoms3r_lcd_printf(vis->lcd, 80, 2, VIS_CALIB_GOOD_COLOR, VIS_BG_COLOR, 2, "CONN");
    } else {
        m5atoms3r_lcd_printf(vis->lcd, 80, 2, VIS_CALIB_POOR_COLOR, VIS_BG_COLOR, 2, "DISC");
    }
    
    // Clear previous data areas (larger areas for bigger text)
    m5atoms3r_lcd_fill_rect(vis->lcd, 2, VIS_QUAT_DISPLAY_Y, 124, 32, VIS_BG_COLOR);
    m5atoms3r_lcd_fill_rect(vis->lcd, 2, VIS_EULER_DISPLAY_Y, 124, 32, VIS_BG_COLOR);
    m5atoms3r_lcd_fill_rect(vis->lcd, 2, VIS_CALIB_DISPLAY_Y, 124, 24, VIS_BG_COLOR);
    m5atoms3r_lcd_fill_rect(vis->lcd, 2, VIS_SYSTEM_INFO_Y, 124, 24, VIS_BG_COLOR);
    
    if (vis->imu_connected) {
        // Display quaternion values (larger text, simplified)
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_QUAT_DISPLAY_Y, VIS_QUAT_W_COLOR, VIS_BG_COLOR, 2, 
                            "W:%.2f", vis->smooth_quat.w);
        m5atoms3r_lcd_printf(vis->lcd, 70, VIS_QUAT_DISPLAY_Y, VIS_QUAT_X_COLOR, VIS_BG_COLOR, 2,
                            "X:%.2f", vis->smooth_quat.x);
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_QUAT_DISPLAY_Y + 16, VIS_QUAT_Y_COLOR, VIS_BG_COLOR, 2,
                            "Y:%.2f", vis->smooth_quat.y);
        m5atoms3r_lcd_printf(vis->lcd, 70, VIS_QUAT_DISPLAY_Y + 16, VIS_QUAT_Z_COLOR, VIS_BG_COLOR, 2,
                            "Z:%.2f", vis->smooth_quat.z);
        
        // Display Euler angles (larger text)
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_EULER_DISPLAY_Y, VIS_EULER_COLOR, VIS_BG_COLOR, 2,
                            "R:%.0f", vis->smooth_roll);
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_EULER_DISPLAY_Y + 16, VIS_EULER_COLOR, VIS_BG_COLOR, 2,
                            "P:%.0f Y:%.0f", vis->smooth_pitch, vis->smooth_yaw);
        
        // Display calibration as progress bars
        uint16_t calib_colors[] = {VIS_CALIB_POOR_COLOR, VIS_CALIB_POOR_COLOR, VIS_CALIB_OK_COLOR, VIS_CALIB_GOOD_COLOR};
        
        // Simplified calibration display (larger text)
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_CALIB_DISPLAY_Y, VIS_TEXT_COLOR, VIS_BG_COLOR, 2, 
                            "Cal: S%d G%d A%d M%d", 
                            vis->last_calib.system, vis->last_calib.gyroscope, 
                            vis->last_calib.accelerometer, vis->last_calib.magnetometer);
        
        // Calculate quaternion magnitude
        float quat_magnitude = sqrtf(vis->smooth_quat.w * vis->smooth_quat.w + 
                                   vis->smooth_quat.x * vis->smooth_quat.x +
                                   vis->smooth_quat.y * vis->smooth_quat.y + 
                                   vis->smooth_quat.z * vis->smooth_quat.z);
        
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_CALIB_DISPLAY_Y + 16, VIS_TEXT_COLOR, VIS_BG_COLOR, 2,
                            "|q|:%.2f", quat_magnitude);
        
    } else {
        // Display disconnected message (larger text)
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_QUAT_DISPLAY_Y + 8, VIS_CALIB_POOR_COLOR, VIS_BG_COLOR, 2,
                            "IMU DISC");
        m5atoms3r_lcd_printf(vis->lcd, 4, VIS_EULER_DISPLAY_Y + 8, VIS_TEXT_COLOR, VIS_BG_COLOR, 2,
                            "Check I2C");
    }
    
    // Display system information (larger text)
    uint32_t current_time = esp_timer_get_time() / 1000;
    uint32_t uptime_sec = current_time / 1000;
    float fps = (vis->frame_count > 0) ? 1000.0f / (current_time - vis->last_update_time + 1) : 0.0f;
    
    m5atoms3r_lcd_printf(vis->lcd, 4, VIS_SYSTEM_INFO_Y, VIS_TEXT_COLOR, VIS_BG_COLOR, 2,
                        "Up:%lus", uptime_sec);
    m5atoms3r_lcd_printf(vis->lcd, 4, VIS_SYSTEM_INFO_Y + 16, VIS_TEXT_COLOR, VIS_BG_COLOR, 2,
                        "Fr:%lu", vis->frame_count);
    
    return ESP_OK;
}