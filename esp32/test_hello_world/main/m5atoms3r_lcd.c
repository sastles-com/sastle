#include "m5atoms3r_lcd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "M5ATOMS3R_LCD";

// 5x8 Font bitmap (ASCII 32-126)
static const uint8_t font5x8[][5] = {
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
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
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

/**
 * @brief Send command to LCD
 */
static esp_err_t lcd_send_command(m5atoms3r_lcd_t* lcd, uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t trans = {0};
    
    gpio_set_level(LCD_PIN_DC, 0);  // Command mode
    
    trans.length = 8;
    trans.tx_buffer = &cmd;
    
    ret = spi_device_polling_transmit(lcd->spi_device, &trans);
    return ret;
}

/**
 * @brief Send data to LCD
 */
static esp_err_t lcd_send_data(m5atoms3r_lcd_t* lcd, const uint8_t* data, size_t len)
{
    if (len == 0) return ESP_OK;
    
    esp_err_t ret;
    spi_transaction_t trans = {0};
    
    gpio_set_level(LCD_PIN_DC, 1);  // Data mode
    
    trans.length = len * 8;
    trans.tx_buffer = data;
    
    ret = spi_device_polling_transmit(lcd->spi_device, &trans);
    return ret;
}

/**
 * @brief Set LCD address window
 */
static esp_err_t lcd_set_addr_window(m5atoms3r_lcd_t* lcd, int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    esp_err_t ret;
    uint8_t data[4];
    
    // Column address set
    ret = lcd_send_command(lcd, GC9107_CASET);
    if (ret != ESP_OK) return ret;
    
    data[0] = (x1 >> 8) & 0xFF;
    data[1] = x1 & 0xFF;
    data[2] = (x2 >> 8) & 0xFF;
    data[3] = x2 & 0xFF;
    ret = lcd_send_data(lcd, data, 4);
    if (ret != ESP_OK) return ret;
    
    // Row address set
    ret = lcd_send_command(lcd, GC9107_RASET);
    if (ret != ESP_OK) return ret;
    
    data[0] = (y1 >> 8) & 0xFF;
    data[1] = y1 & 0xFF;
    data[2] = (y2 >> 8) & 0xFF;
    data[3] = y2 & 0xFF;
    ret = lcd_send_data(lcd, data, 4);
    if (ret != ESP_OK) return ret;
    
    // Memory write
    ret = lcd_send_command(lcd, GC9107_RAMWR);
    return ret;
}

/**
 * @brief Initialize M5AtomS3R LCD
 */
esp_err_t m5atoms3r_lcd_init(m5atoms3r_lcd_t* lcd)
{
    if (!lcd) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing M5AtomS3R LCD...");
    
    // Initialize GPIO pins (DC, RST, and Backlight)
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST) | (1ULL << LCD_PIN_BL);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize SPI bus (if not already initialized)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2
    };
    
    ret = spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = LCD_SPI_FREQ,
        .mode = LCD_SPI_MODE,
        .spics_io_num = LCD_PIN_CS,
        .queue_size = 7,
        .flags = 0,  // Remove SPI_DEVICE_NO_DUMMY flag
    };
    
    ret = spi_bus_add_device(LCD_SPI_HOST, &dev_cfg, &lcd->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Hardware reset
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Software reset
    lcd_send_command(lcd, GC9107_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Exit sleep mode
    lcd_send_command(lcd, GC9107_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Memory access control
    lcd_send_command(lcd, GC9107_MADCTL);
    uint8_t madctl_data = 0x00;  // Normal orientation
    lcd_send_data(lcd, &madctl_data, 1);
    
    // Pixel format (16-bit RGB565)
    lcd_send_command(lcd, GC9107_COLMOD);
    uint8_t colmod_data = 0x55;
    lcd_send_data(lcd, &colmod_data, 1);
    
    // Normal display mode
    lcd_send_command(lcd, GC9107_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Display on
    lcd_send_command(lcd, GC9107_DISPON);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Enable backlight (GPIO16)
    gpio_set_level(LCD_PIN_BL, 1);  // Turn on backlight
    ESP_LOGI(TAG, "M5AtomS3R LCD backlight enabled on GPIO16");
    
    lcd->width = LCD_WIDTH;
    lcd->height = LCD_HEIGHT;
    lcd->rotation = 0;
    lcd->initialized = true;
    
    ESP_LOGI(TAG, "M5AtomS3R LCD initialized successfully (%dx%d)", lcd->width, lcd->height);
    
    return ESP_OK;
}

/**
 * @brief Deinitialize LCD
 */
esp_err_t m5atoms3r_lcd_deinit(m5atoms3r_lcd_t* lcd)
{
    if (!lcd || !lcd->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Turn off backlight
    gpio_set_level(LCD_PIN_BL, 0);
    
    // Display off
    lcd_send_command(lcd, GC9107_DISPOFF);
    
    // Sleep mode
    lcd_send_command(lcd, GC9107_SLPIN);
    
    // Remove SPI device
    spi_bus_remove_device(lcd->spi_device);
    
    lcd->initialized = false;
    
    ESP_LOGI(TAG, "M5AtomS3R LCD deinitialized");
    return ESP_OK;
}

/**
 * @brief Clear LCD with color
 */
esp_err_t m5atoms3r_lcd_clear(m5atoms3r_lcd_t* lcd, uint16_t color)
{
    if (!lcd || !lcd->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return m5atoms3r_lcd_fill_rect(lcd, 0, 0, lcd->width, lcd->height, color);
}

/**
 * @brief Fill rectangle with color
 */
esp_err_t m5atoms3r_lcd_fill_rect(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    if (!lcd || !lcd->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (x >= lcd->width || y >= lcd->height) {
        return ESP_OK;  // Outside screen
    }
    
    // Clip rectangle
    if (x + w > lcd->width) w = lcd->width - x;
    if (y + h > lcd->height) h = lcd->height - y;
    
    esp_err_t ret = lcd_set_addr_window(lcd, x, y, x + w - 1, y + h - 1);
    if (ret != ESP_OK) return ret;
    
    // Convert color to big-endian
    uint16_t color_be = (color << 8) | (color >> 8);
    
    // Send color data
    size_t pixel_count = w * h;
    uint16_t* buffer = malloc(pixel_count * 2);
    if (!buffer) {
        return ESP_ERR_NO_MEM;
    }
    
    for (size_t i = 0; i < pixel_count; i++) {
        buffer[i] = color_be;
    }
    
    ret = lcd_send_data(lcd, (uint8_t*)buffer, pixel_count * 2);
    free(buffer);
    
    return ret;
}

/**
 * @brief Draw single pixel
 */
esp_err_t m5atoms3r_lcd_draw_pixel(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, uint16_t color)
{
    if (!lcd || !lcd->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (x < 0 || y < 0 || x >= lcd->width || y >= lcd->height) {
        return ESP_OK;  // Outside screen
    }
    
    esp_err_t ret = lcd_set_addr_window(lcd, x, y, x, y);
    if (ret != ESP_OK) return ret;
    
    uint16_t color_be = (color << 8) | (color >> 8);
    ret = lcd_send_data(lcd, (uint8_t*)&color_be, 2);
    
    return ret;
}

/**
 * @brief Draw character
 */
esp_err_t m5atoms3r_lcd_draw_char(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t size)
{
    if (!lcd || !lcd->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (c < 32 || c > 126) {
        c = 32;  // Default to space for unsupported characters
    }
    
    const uint8_t* font_data = font5x8[c - 32];
    
    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = font_data[i];
        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) {
                    m5atoms3r_lcd_draw_pixel(lcd, x + i, y + j, color);
                } else {
                    m5atoms3r_lcd_fill_rect(lcd, x + i * size, y + j * size, size, size, color);
                }
            } else if (bg_color != color) {
                if (size == 1) {
                    m5atoms3r_lcd_draw_pixel(lcd, x + i, y + j, bg_color);
                } else {
                    m5atoms3r_lcd_fill_rect(lcd, x + i * size, y + j * size, size, size, bg_color);
                }
            }
            line >>= 1;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Print string
 */
esp_err_t m5atoms3r_lcd_print_string(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg_color, uint8_t size)
{
    if (!lcd || !lcd->initialized || !str) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t cursor_x = x;
    int16_t cursor_y = y;
    
    while (*str) {
        if (*str == '\n') {
            cursor_x = x;
            cursor_y += 8 * size;
        } else if (*str == '\r') {
            cursor_x = x;
        } else {
            m5atoms3r_lcd_draw_char(lcd, cursor_x, cursor_y, *str, color, bg_color, size);
            cursor_x += 6 * size;  // 5 pixels + 1 space
        }
        str++;
    }
    
    return ESP_OK;
}

/**
 * @brief Printf-style text display
 */
esp_err_t m5atoms3r_lcd_printf(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, uint16_t color, uint16_t bg_color, uint8_t size, const char* format, ...)
{
    if (!lcd || !lcd->initialized || !format) {
        return ESP_ERR_INVALID_ARG;
    }
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    return m5atoms3r_lcd_print_string(lcd, x, y, buffer, color, bg_color, size);
}

/**
 * @brief Set backlight on/off
 */
esp_err_t m5atoms3r_lcd_set_backlight(bool enable)
{
    gpio_set_level(LCD_PIN_BL, enable ? 1 : 0);
    ESP_LOGI(TAG, "Backlight %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Set backlight brightness using PWM (0-100%)
 */
esp_err_t m5atoms3r_lcd_set_backlight_pwm(uint8_t brightness)
{
    static bool ledc_initialized = false;
    
    if (!ledc_initialized) {
        // Configure LEDC timer
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = 500,  // 500Hz as recommended for LCD backlights
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0
        };
        esp_err_t ret = ledc_timer_config(&ledc_timer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Configure LEDC channel
        ledc_channel_config_t ledc_channel = {
            .channel = LEDC_CHANNEL_0,
            .duty = 0,
            .gpio_num = LCD_PIN_BL,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0
        };
        ret = ledc_channel_config(&ledc_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ledc_initialized = true;
    }
    
    // Convert brightness percentage to duty cycle (0-255)
    uint32_t duty = (brightness * 255) / 100;
    
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LEDC duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update LEDC duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Backlight brightness set to %d%%", brightness);
    return ESP_OK;
}

/**
 * @brief Convert RGB to RGB565
 */
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}