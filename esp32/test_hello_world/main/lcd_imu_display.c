#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bno055.h"

static const char *TAG = "LCD_IMU_DISPLAY";

// M5AtomS3R Display pins
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (26 * 1000 * 1000)  // 26MHz

#define PIN_NUM_MISO            -1
#define PIN_NUM_MOSI            21      // MOSI
#define PIN_NUM_CLK             15      // SCK
#define PIN_NUM_CS              14      // CS
#define PIN_NUM_DC              42      // RS/DC
#define PIN_NUM_RST             48      // RST

// Display dimensions
#define LCD_H_RES               128
#define LCD_V_RES               128

// Colors (RGB565)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_GRAY      0x8410
#define COLOR_ORANGE    0xFD20

// Font data (5x8 bitmap font - simplified)
static const uint8_t font_5x8[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x30, 0x30, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

static spi_device_handle_t spi = NULL;
static bno055_handle_t bno055_handle = NULL;
static TaskHandle_t lcd_update_task_handle = NULL;

// LCD Basic Functions
static void lcd_io_init(void) {
    ESP_LOGI(TAG, "Initialize LCD GPIO");
    
    gpio_config_t io_conf = {};
    
    // DC pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_DC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Reset pin
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_RST);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void lcd_reset(void) {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void lcd_cmd(const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    
    memset(&t, 0, sizeof(t));       
    t.length = 8;
    t.tx_buffer = &cmd;
    
    gpio_set_level(PIN_NUM_DC, 0); // Command mode
    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void lcd_data(const uint8_t *data, int len) {
    esp_err_t ret;
    spi_transaction_t t;
    
    if (len == 0) return;
    
    memset(&t, 0, sizeof(t));       
    t.length = len * 8;
    t.tx_buffer = data;
    
    gpio_set_level(PIN_NUM_DC, 1); // Data mode
    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void lcd_data_byte(uint8_t data) {
    lcd_data(&data, 1);
}

static void lcd_spi_init(void) {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2 + 8
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_PIXEL_CLOCK_HZ,   
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,             
        .queue_size = 7,
    };
    
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI initialized");
}

static void gc9107_init(void) {
    ESP_LOGI(TAG, "Initializing GC9107");
    
    lcd_cmd(0x01); // Software reset
    vTaskDelay(pdMS_TO_TICKS(120));
    
    lcd_cmd(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    
    lcd_cmd(0x3A); // Pixel Format Set (RGB565)
    lcd_data_byte(0x05);
    
    lcd_cmd(0x36); // Memory Access Control
    lcd_data_byte(0x00);
    
    lcd_cmd(0xB6); // Display Function Control
    lcd_data_byte(0x0A);
    lcd_data_byte(0x82);
    
    lcd_cmd(0xC0); // Power Control 1
    lcd_data_byte(0x23);
    
    lcd_cmd(0xC1); // Power Control 2
    lcd_data_byte(0x10);
    
    lcd_cmd(0xC5); // VCOM Control 1
    lcd_data_byte(0x3E);
    lcd_data_byte(0x28);
    
    lcd_cmd(0xC7); // VCOM Control 2
    lcd_data_byte(0x86);
    
    lcd_cmd(0xE0); // Positive Gamma
    uint8_t gamma_pos[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
    lcd_data(gamma_pos, sizeof(gamma_pos));
    
    lcd_cmd(0xE1); // Negative Gamma
    uint8_t gamma_neg[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
    lcd_data(gamma_neg, sizeof(gamma_neg));
    
    lcd_cmd(0x21); // Display Inversion On
    lcd_cmd(0x29); // Display ON
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "GC9107 initialization complete");
}

static void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_cmd(0x2A); // Column Address Set
    lcd_data_byte(x0 >> 8);
    lcd_data_byte(x0 & 0xFF);
    lcd_data_byte(x1 >> 8);
    lcd_data_byte(x1 & 0xFF);
    
    lcd_cmd(0x2B); // Page Address Set
    lcd_data_byte(y0 >> 8);
    lcd_data_byte(y0 & 0xFF);
    lcd_data_byte(y1 >> 8);
    lcd_data_byte(y1 & 0xFF);
    
    lcd_cmd(0x2C); // Memory Write
}

static void lcd_fill_screen(uint16_t color) {
    lcd_set_addr_window(0, 0, LCD_H_RES - 1, LCD_V_RES - 1);
    
    uint8_t color_bytes[2];
    color_bytes[0] = color >> 8;
    color_bytes[1] = color & 0xFF;
    
    gpio_set_level(PIN_NUM_DC, 1);
    
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 16;
        t.tx_buffer = color_bytes;
        spi_device_transmit(spi, &t);
    }
}

static void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= LCD_H_RES || y >= LCD_V_RES) return;
    
    lcd_set_addr_window(x, y, x, y);
    
    uint8_t color_bytes[2];
    color_bytes[0] = color >> 8;
    color_bytes[1] = color & 0xFF;
    
    lcd_data(color_bytes, 2);
}

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t size) {
    if (c < 32 || c > 126) c = 32; // Handle invalid characters
    
    const uint8_t *font_data = font_5x8[c - 32];
    
    for (int i = 0; i < 5; i++) {
        uint8_t line = font_data[i];
        for (int j = 0; j < 8; j++) {
            uint16_t pixel_color = (line & (1 << j)) ? color : bg_color;
            
            if (size == 1) {
                lcd_draw_pixel(x + i, y + j, pixel_color);
            } else {
                // Draw larger characters
                for (int sx = 0; sx < size; sx++) {
                    for (int sy = 0; sy < size; sy++) {
                        lcd_draw_pixel(x + i * size + sx, y + j * size + sy, pixel_color);
                    }
                }
            }
        }
    }
}

static void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color, uint8_t size) {
    uint16_t cur_x = x;
    
    while (*str) {
        lcd_draw_char(cur_x, y, *str, color, bg_color, size);
        cur_x += 6 * size; // Character width + spacing
        str++;
    }
}

static void lcd_draw_quaternion_data(bno055_quaternion_t *quat) {
    char buffer[32];
    
    // Clear screen with dark background
    lcd_fill_screen(COLOR_BLACK);
    
    // Title
    lcd_draw_string(20, 5, "IMU Quaternion", COLOR_WHITE, COLOR_BLACK, 1);
    
    // Draw separator line (simplified)
    for (int x = 10; x < 118; x++) {
        lcd_draw_pixel(x, 15, COLOR_GRAY);
    }
    
    // W component
    snprintf(buffer, sizeof(buffer), "W: %+.3f", quat->w);
    uint16_t w_color = (quat->w >= 0) ? COLOR_GREEN : COLOR_RED;
    lcd_draw_string(10, 25, buffer, w_color, COLOR_BLACK, 1);
    
    // X component
    snprintf(buffer, sizeof(buffer), "X: %+.3f", quat->x);
    uint16_t x_color = (quat->x >= 0) ? COLOR_CYAN : COLOR_MAGENTA;
    lcd_draw_string(10, 35, buffer, x_color, COLOR_BLACK, 1);
    
    // Y component
    snprintf(buffer, sizeof(buffer), "Y: %+.3f", quat->y);
    uint16_t y_color = (quat->y >= 0) ? COLOR_YELLOW : COLOR_ORANGE;
    lcd_draw_string(10, 45, buffer, y_color, COLOR_BLACK, 1);
    
    // Z component
    snprintf(buffer, sizeof(buffer), "Z: %+.3f", quat->z);
    uint16_t z_color = (quat->z >= 0) ? COLOR_BLUE : COLOR_RED;
    lcd_draw_string(10, 55, buffer, z_color, COLOR_BLACK, 1);
    
    // Calculate magnitude
    float magnitude = sqrt(quat->w * quat->w + quat->x * quat->x + 
                          quat->y * quat->y + quat->z * quat->z);
    
    snprintf(buffer, sizeof(buffer), "Mag: %.3f", magnitude);
    uint16_t mag_color = (magnitude > 0.95f && magnitude < 1.05f) ? COLOR_GREEN : COLOR_RED;
    lcd_draw_string(10, 70, buffer, mag_color, COLOR_BLACK, 1);
    
    // Convert to Euler for reference
    // Simplified conversion (yaw only)
    float yaw = atan2(2.0f * (quat->w * quat->z + quat->x * quat->y),
                      1.0f - 2.0f * (quat->y * quat->y + quat->z * quat->z));
    yaw = yaw * 180.0f / M_PI;
    
    snprintf(buffer, sizeof(buffer), "Yaw: %.1f deg", yaw);
    lcd_draw_string(10, 85, buffer, COLOR_WHITE, COLOR_BLACK, 1);
    
    // Visual quaternion representation (simplified)
    // Draw a cross representing orientation
    int center_x = 64;
    int center_y = 105;
    int radius = 15;
    
    // Calculate rotation from quaternion (simplified 2D projection)
    float cos_angle = quat->w;
    float sin_angle = sqrt(quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
    
    if (sin_angle > 0.001f) {
        int end_x = center_x + (int)(radius * quat->x / sin_angle * cos_angle);
        int end_y = center_y + (int)(radius * quat->y / sin_angle * cos_angle);
        
        // Draw orientation line
        // Simple line drawing (Bresenham's algorithm would be better)
        for (int t = 0; t <= 10; t++) {
            int x = center_x + (end_x - center_x) * t / 10;
            int y = center_y + (end_y - center_y) * t / 10;
            if (x >= 0 && x < LCD_H_RES && y >= 0 && y < LCD_V_RES) {
                lcd_draw_pixel(x, y, COLOR_GREEN);
            }
        }
    }
    
    // Draw center point
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            lcd_draw_pixel(center_x + dx, center_y + dy, COLOR_WHITE);
        }
    }
}

static void lcd_draw_calibration_status(bno055_calibration_status_t *calib) {
    char buffer[32];
    
    // Draw calibration status at bottom
    lcd_draw_string(5, 115, "Cal:", COLOR_WHITE, COLOR_BLACK, 1);
    
    snprintf(buffer, sizeof(buffer), "S%d", calib->system);
    uint16_t sys_color = (calib->system == 3) ? COLOR_GREEN : COLOR_RED;
    lcd_draw_string(30, 115, buffer, sys_color, COLOR_BLACK, 1);
    
    snprintf(buffer, sizeof(buffer), "G%d", calib->gyroscope);
    uint16_t gyro_color = (calib->gyroscope == 3) ? COLOR_GREEN : COLOR_RED;
    lcd_draw_string(50, 115, buffer, gyro_color, COLOR_BLACK, 1);
    
    snprintf(buffer, sizeof(buffer), "A%d", calib->accelerometer);
    uint16_t acc_color = (calib->accelerometer == 3) ? COLOR_GREEN : COLOR_RED;
    lcd_draw_string(70, 115, buffer, acc_color, COLOR_BLACK, 1);
    
    snprintf(buffer, sizeof(buffer), "M%d", calib->magnetometer);
    uint16_t mag_color = (calib->magnetometer == 3) ? COLOR_GREEN : COLOR_RED;
    lcd_draw_string(90, 115, buffer, mag_color, COLOR_BLACK, 1);
}

static void lcd_update_task(void *pvParameters) {
    ESP_LOGI(TAG, "LCD Update Task started");
    
    const int update_rate_ms = 100; // 10 Hz update rate
    
    while (1) {
        if (bno055_handle) {
            bno055_quaternion_t quat;
            esp_err_t ret = bno055_read_quaternion(bno055_handle, &quat);
            
            if (ret == ESP_OK) {
                lcd_draw_quaternion_data(&quat);
                
                // Also draw calibration status
                bno055_calibration_status_t calib;
                ret = bno055_get_calibration_status(bno055_handle, &calib);
                if (ret == ESP_OK) {
                    lcd_draw_calibration_status(&calib);
                }
            } else {
                // Show error message
                lcd_fill_screen(COLOR_BLACK);
                lcd_draw_string(20, 60, "IMU Read Error", COLOR_RED, COLOR_BLACK, 1);
            }
        } else {
            // Show no IMU message
            lcd_fill_screen(COLOR_BLACK);
            lcd_draw_string(25, 60, "No IMU Connected", COLOR_YELLOW, COLOR_BLACK, 1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(update_rate_ms));
    }
}

esp_err_t lcd_imu_display_init(bno055_handle_t imu_handle) {
    ESP_LOGI(TAG, "Initializing LCD IMU Display");
    
    // Store IMU handle
    bno055_handle = imu_handle;
    
    // Initialize LCD hardware
    lcd_io_init();
    lcd_spi_init();
    lcd_reset();
    gc9107_init();
    
    // Clear screen initially
    lcd_fill_screen(COLOR_BLACK);
    lcd_draw_string(30, 60, "LCD Initialized", COLOR_GREEN, COLOR_BLACK, 1);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Start LCD update task
    BaseType_t ret = xTaskCreatePinnedToCore(
        lcd_update_task,
        "lcd_update",
        4096,
        NULL,
        5,  // Medium priority
        &lcd_update_task_handle,
        0   // Core 0
    );
    
    if (ret == pdPASS) {
        ESP_LOGI(TAG, "LCD Update task created successfully");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "LCD Update task creation failed");
        return ESP_FAIL;
    }
}

void lcd_imu_display_cleanup(void) {
    ESP_LOGI(TAG, "Cleaning up LCD IMU Display");
    
    if (lcd_update_task_handle) {
        vTaskDelete(lcd_update_task_handle);
        lcd_update_task_handle = NULL;
        ESP_LOGI(TAG, "LCD Update task deleted");
    }
    
    if (spi) {
        lcd_fill_screen(COLOR_BLACK);
        lcd_draw_string(35, 60, "Display OFF", COLOR_RED, COLOR_BLACK, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        lcd_fill_screen(COLOR_BLACK);
    }
    
    bno055_handle = NULL;
    
    ESP_LOGI(TAG, "LCD IMU Display cleanup completed");
}