#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ledc.h"

static const char *TAG = "M5ATOMS3R_DISPLAY";

// M5AtomS3R LCD pin definitions
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (20 * 1000 * 1000)  // 20MHz

#define PIN_NUM_SCLK            17
#define PIN_NUM_MOSI            21
#define PIN_NUM_MISO            -1
#define PIN_NUM_LCD_DC          15
#define PIN_NUM_LCD_RST         34
#define PIN_NUM_LCD_CS          6
#define PIN_NUM_BK_LIGHT        16

// LCD resolution
#define LCD_H_RES               128
#define LCD_V_RES               128

// LCD command/parameter bits
#define LCD_CMD_BITS            8
#define LCD_PARAM_BITS          8

// I2C Configuration for BMI270 IMU
#define I2C_MASTER_SCL_IO       GPIO_NUM_2
#define I2C_MASTER_SDA_IO       GPIO_NUM_1
#define I2C_MASTER_NUM          0
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS   1000

// BMI270 Register definitions
#define BMI270_ADDR             0x68
#define BMI270_CHIP_ID          0x00
#define BMI270_CHIP_ID_VALUE    0x24
#define BMI270_PWR_CTRL         0x7D
#define BMI270_PWR_CONF         0x7C
#define BMI270_ACC_CONF         0x40
#define BMI270_GYR_CONF         0x42
#define BMI270_DATA_8           0x0C  // Accel X LSB
#define BMI270_DATA_14          0x12  // Gyro X LSB

// Color definitions (RGB565)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F

static esp_lcd_panel_handle_t panel_handle = NULL;
static uint16_t *frame_buffer = NULL;

// IMU data structure
typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    bool is_connected;
    uint32_t error_count;
} imu_data_t;

static imu_data_t imu_data = {0};

// Function prototypes
static esp_err_t i2c_master_init(void);
static esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
static esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data);
static esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t bmi270_init(void);
static esp_err_t bmi270_read_data(imu_data_t *data);
static void init_backlight(void);
static void init_lcd(void);
static void draw_text(int x, int y, const char* text, uint16_t color, uint16_t bg_color);
static void clear_screen(uint16_t color);
static void update_display(void);

// Simple 8x8 font (ASCII 32-127)
static const uint8_t font8x8[96][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Space
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // "
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // #
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // $
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // %
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // &
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // *
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // +
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x06,0x00}, // ,
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // .
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // /
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 9
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // :
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x06,0x00}, // ;
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // <
    {0x00,0x00,0x3F,0x00,0x3F,0x00,0x00,0x00}, // =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // >
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // ?
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // @
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // A
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // B
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // C
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // D
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // E
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // F
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // G
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // H
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // I
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // J
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // K
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // L
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // M
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // N
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // O
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // P
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // Q
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // R
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // S
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // T
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // U
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // X
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // Y
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // Z
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // [
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, // backslash
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // ]
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, // ^
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // _
};

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bmi270_init(void)
{
    uint8_t chip_id;
    esp_err_t ret;
    
    // Read chip ID
    ret = i2c_read_byte(BMI270_ADDR, BMI270_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMI270 chip ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (chip_id != BMI270_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X, expected 0x%02X", chip_id, BMI270_CHIP_ID_VALUE);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "BMI270 chip ID verified: 0x%02X", chip_id);
    
    // Initialize BMI270 - simplified initialization
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Power on accelerometer and gyroscope
    ret = i2c_write_byte(BMI270_ADDR, BMI270_PWR_CTRL, 0x0E);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power on sensors: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure accelerometer (±4g, 100Hz)
    ret = i2c_write_byte(BMI270_ADDR, BMI270_ACC_CONF, 0xA8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure gyroscope (±1000dps, 100Hz)
    ret = i2c_write_byte(BMI270_ADDR, BMI270_GYR_CONF, 0xA9);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for sensor to stabilize
    
    ESP_LOGI(TAG, "BMI270 initialized successfully");
    return ESP_OK;
}

static esp_err_t bmi270_read_data(imu_data_t *data)
{
    uint8_t raw_data[12];
    esp_err_t ret;
    
    // Read accelerometer and gyroscope data (6 bytes each)
    ret = i2c_read_bytes(BMI270_ADDR, BMI270_DATA_8, raw_data, 12);
    if (ret != ESP_OK) {
        data->error_count++;
        return ret;
    }
    
    // Convert to signed 16-bit values
    data->acc_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->acc_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    data->acc_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    data->gyr_x = (int16_t)((raw_data[7] << 8) | raw_data[6]);
    data->gyr_y = (int16_t)((raw_data[9] << 8) | raw_data[8]);
    data->gyr_z = (int16_t)((raw_data[11] << 8) | raw_data[10]);
    
    data->is_connected = true;
    return ESP_OK;
}

static void init_backlight(void)
{
    ESP_LOGI(TAG, "Initialize backlight PWM");
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 500,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .gpio_num       = PIN_NUM_BK_LIGHT,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // Set backlight to 75% brightness
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 6144));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void init_lcd(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install LCD driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    // Allocate frame buffer
    frame_buffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!frame_buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return;
    }
    
    ESP_LOGI(TAG, "LCD initialization complete");
}

static void draw_text(int x, int y, const char* text, uint16_t color, uint16_t bg_color)
{
    int len = strlen(text);
    for (int i = 0; i < len; i++) {
        char c = text[i];
        if (c < 32 || c > 127) c = 32; // Space for unsupported chars
        
        const uint8_t *font_char = font8x8[c - 32];
        
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 8; col++) {
                int px = x + i * 8 + col;
                int py = y + row;
                if (px >= 0 && px < LCD_H_RES && py >= 0 && py < LCD_V_RES) {
                    uint16_t pixel_color = (font_char[row] & (1 << (7 - col))) ? color : bg_color;
                    frame_buffer[py * LCD_H_RES + px] = pixel_color;
                }
            }
        }
    }
}

static void clear_screen(uint16_t color)
{
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        frame_buffer[i] = color;
    }
}

static void update_display(void)
{
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, frame_buffer);
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R PSRAM & IMU Display");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize hardware
    init_backlight();
    init_lcd();
    
    // Initialize I2C and IMU
    ESP_LOGI(TAG, "Initializing I2C...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
    }
    
    // Try to initialize BMI270
    if (bmi270_init() != ESP_OK) {
        ESP_LOGW(TAG, "BMI270 initialization failed, continuing without IMU");
        imu_data.is_connected = false;
    } else {
        imu_data.is_connected = true;
    }
    
    ESP_LOGI(TAG, "Starting display loop...");
    
    uint32_t loop_count = 0;
    char text_buffer[64];
    
    while (1) {
        // Clear screen
        clear_screen(COLOR_BLACK);
        
        // Title
        draw_text(16, 2, "M5AtomS3R", COLOR_WHITE, COLOR_BLACK);
        
        // Memory Information
        draw_text(0, 12, "Memory Info:", COLOR_YELLOW, COLOR_BLACK);
        
        uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        snprintf(text_buffer, sizeof(text_buffer), "Free:%"PRIu32"KB", free_heap / 1024);
        draw_text(0, 22, text_buffer, COLOR_GREEN, COLOR_BLACK);
        
#if CONFIG_SPIRAM
        if (esp_psram_is_initialized()) {
            uint32_t psram_size = esp_psram_get_size();
            uint32_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            draw_text(0, 32, "PSRAM: ON", COLOR_CYAN, COLOR_BLACK);
            snprintf(text_buffer, sizeof(text_buffer), "Size:%"PRIu32"MB", psram_size / (1024*1024));
            draw_text(0, 42, text_buffer, COLOR_CYAN, COLOR_BLACK);
            snprintf(text_buffer, sizeof(text_buffer), "Free:%"PRIu32"MB", free_psram / (1024*1024));
            draw_text(0, 52, text_buffer, COLOR_CYAN, COLOR_BLACK);
        } else {
            draw_text(0, 32, "PSRAM: ERR", COLOR_RED, COLOR_BLACK);
        }
#else
        draw_text(0, 32, "PSRAM: OFF", COLOR_MAGENTA, COLOR_BLACK);
#endif
        
        // IMU Information
        draw_text(0, 62, "IMU Status:", COLOR_YELLOW, COLOR_BLACK);
        
        if (imu_data.is_connected) {
            // Try to read IMU data
            esp_err_t imu_result = bmi270_read_data(&imu_data);
            
            if (imu_result == ESP_OK) {
                draw_text(0, 72, "BMI270: OK", COLOR_GREEN, COLOR_BLACK);
                
                // Display accelerometer data (scaled and rounded)
                snprintf(text_buffer, sizeof(text_buffer), "Ax:%d", imu_data.acc_x / 100);
                draw_text(0, 82, text_buffer, COLOR_WHITE, COLOR_BLACK);
                
                snprintf(text_buffer, sizeof(text_buffer), "Ay:%d", imu_data.acc_y / 100);
                draw_text(0, 92, text_buffer, COLOR_WHITE, COLOR_BLACK);
                
                snprintf(text_buffer, sizeof(text_buffer), "Az:%d", imu_data.acc_z / 100);
                draw_text(0, 102, text_buffer, COLOR_WHITE, COLOR_BLACK);
                
                // Display gyroscope data (scaled and rounded)
                snprintf(text_buffer, sizeof(text_buffer), "Gx:%d", imu_data.gyr_x / 100);
                draw_text(64, 82, text_buffer, COLOR_CYAN, COLOR_BLACK);
                
                snprintf(text_buffer, sizeof(text_buffer), "Gy:%d", imu_data.gyr_y / 100);
                draw_text(64, 92, text_buffer, COLOR_CYAN, COLOR_BLACK);
                
                snprintf(text_buffer, sizeof(text_buffer), "Gz:%d", imu_data.gyr_z / 100);
                draw_text(64, 102, text_buffer, COLOR_CYAN, COLOR_BLACK);
            } else {
                snprintf(text_buffer, sizeof(text_buffer), "Read ERR:%"PRIu32, imu_data.error_count);
                draw_text(0, 72, text_buffer, COLOR_RED, COLOR_BLACK);
            }
        } else {
            draw_text(0, 72, "IMU: N/A", COLOR_RED, COLOR_BLACK);
            draw_text(0, 82, "Check wiring", COLOR_RED, COLOR_BLACK);
        }
        
        // Loop counter and system info
        snprintf(text_buffer, sizeof(text_buffer), "Loop:%"PRIu32, loop_count);
        draw_text(0, 115, text_buffer, COLOR_MAGENTA, COLOR_BLACK);
        
        // Update display
        update_display();
        
        // Log to serial
        ESP_LOGI(TAG, "Loop %"PRIu32" - Free: %"PRIu32"KB, IMU: %s", 
                loop_count, free_heap/1024, 
                imu_data.is_connected ? "OK" : "N/A");
        
        if (imu_data.is_connected && imu_data.error_count == 0) {
            ESP_LOGI(TAG, "Acc: [%d, %d, %d] Gyr: [%d, %d, %d]", 
                    imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
                    imu_data.gyr_x, imu_data.gyr_y, imu_data.gyr_z);
        }
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(500)); // Update every 500ms (2Hz)
    }
}