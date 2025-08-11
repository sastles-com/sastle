#ifndef LCD_IMU_DISPLAY_H
#define LCD_IMU_DISPLAY_H

#include "esp_err.h"
#include "bno055.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LCD display for IMU quaternion visualization
 * 
 * @param imu_handle BNO055 handle for reading IMU data
 * @return ESP_OK on success
 */
esp_err_t lcd_imu_display_init(bno055_handle_t imu_handle);

/**
 * @brief Cleanup LCD display resources
 */
void lcd_imu_display_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // LCD_IMU_DISPLAY_H