#ifndef QUATERNION_VISUALIZER_H
#define QUATERNION_VISUALIZER_H

#include "esp_err.h"
#include "m5atoms3r_lcd.h"
#include "simple_bno055.h"

// Visualizer configuration
#define VIS_UPDATE_RATE_HZ      10      // 10Hz display update
#define VIS_SMOOTHING_ALPHA     0.1f    // Smooth visualization changes

// Display layout (adjusted for larger text)
#define VIS_HEADER_HEIGHT       20
#define VIS_STATUS_HEIGHT       16
#define VIS_QUAT_DISPLAY_Y      36
#define VIS_EULER_DISPLAY_Y     72
#define VIS_CALIB_DISPLAY_Y     108
#define VIS_SYSTEM_INFO_Y       144

// Color scheme
#define VIS_BG_COLOR           LCD_COLOR_BLACK
#define VIS_TEXT_COLOR         LCD_COLOR_WHITE
#define VIS_HEADER_COLOR       LCD_COLOR_CYAN
#define VIS_QUAT_W_COLOR       LCD_COLOR_WHITE
#define VIS_QUAT_X_COLOR       LCD_COLOR_RED
#define VIS_QUAT_Y_COLOR       LCD_COLOR_GREEN
#define VIS_QUAT_Z_COLOR       LCD_COLOR_BLUE
#define VIS_EULER_COLOR        LCD_COLOR_YELLOW
#define VIS_CALIB_GOOD_COLOR   LCD_COLOR_GREEN
#define VIS_CALIB_POOR_COLOR   LCD_COLOR_RED
#define VIS_CALIB_OK_COLOR     LCD_COLOR_ORANGE

// Visualizer handle
typedef struct {
    m5atoms3r_lcd_t* lcd;
    simple_bno055_t* imu;
    bool initialized;
    uint32_t frame_count;
    uint32_t last_update_time;
    
    // Smoothed values for visualization
    simple_quaternion_t smooth_quat;
    float smooth_roll, smooth_pitch, smooth_yaw;
    
    // Status tracking
    bool imu_connected;
    simple_calibration_status_t last_calib;
} quaternion_visualizer_t;

// Function prototypes
esp_err_t quat_vis_init(quaternion_visualizer_t* vis, m5atoms3r_lcd_t* lcd, simple_bno055_t* imu);
esp_err_t quat_vis_update(quaternion_visualizer_t* vis);
esp_err_t quat_vis_render_frame(quaternion_visualizer_t* vis);
esp_err_t quat_vis_clear_screen(quaternion_visualizer_t* vis);

// Utility functions
void quat_vis_draw_progress_bar(quaternion_visualizer_t* vis, int16_t x, int16_t y, int16_t width, int16_t height, float value, uint16_t color);
void quat_vis_euler_from_quaternion(const simple_quaternion_t* q, float* roll, float* pitch, float* yaw);

#endif // QUATERNION_VISUALIZER_H