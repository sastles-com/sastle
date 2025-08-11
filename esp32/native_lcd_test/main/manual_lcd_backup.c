#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "soc/rtc.h"

static const char *TAG = "LCD_TEXT_DISPLAY";

// M5AtomS3R Display pins (correct pinout)
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (26 * 1000 * 1000)

#define PIN_NUM_MISO            -1
#define PIN_NUM_MOSI            21      // G21 - MOSI
#define PIN_NUM_CLK             15      // G15 - SCK
#define PIN_NUM_CS              14      // G14 - CS
#define PIN_NUM_DC              42      // G42 - RS/DC
#define PIN_NUM_RST             48      // G48 - RST

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

static spi_device_handle_t spi;

// Simple 8x8 font for ASCII characters 32-126
static const uint8_t font8x8[95][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, // !
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // "
    {0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00}, // #
    {0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00}, // $
    {0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00}, // %
    {0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00}, // &
    {0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}, // '
    {0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00}, // (
    {0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00}, // )
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00}, // *
    {0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00}, // +
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x06, 0x00}, // ,
    {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00}, // -
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00}, // .
    {0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00}, // /
    {0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00}, // 0
    {0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00}, // 1
    {0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00}, // 2
    {0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00}, // 3
    {0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00}, // 4
    {0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00}, // 5
    {0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00}, // 6
    {0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00}, // 7
    {0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00}, // 8
    {0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00}, // 9
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00}, // :
    {0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x06, 0x00}, // ;
    {0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00}, // <
    {0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00}, // =
    {0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00}, // >
    {0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00}, // ?
    {0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00}, // @
    {0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00}, // A
    {0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00}, // B
    {0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00}, // C
    {0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00}, // D
    {0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00}, // E
    {0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00}, // F
    {0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00}, // G
    {0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00}, // H
    {0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, // I
    {0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00}, // J
    {0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00}, // K
    {0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00}, // L
    {0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00}, // M
    {0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00}, // N
    {0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00}, // O
    {0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00}, // P
    {0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00}, // Q
    {0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00}, // R
    {0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00}, // S
    {0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00}, // T
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00}, // U
    {0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00}, // V
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}, // W
    {0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00}, // X
    {0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00}, // Y
    {0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00}, // Z
};

// Initialize GPIO pins
static void lcd_io_init(void)
{
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
    
    ESP_LOGI(TAG, "Display IO initialized");
}

// Reset the display
static void lcd_reset(void)
{
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Display reset complete");
}

// Send a command to the LCD
static void lcd_cmd(const uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(PIN_NUM_DC, 0);
    spi_device_transmit(spi, &t);
}

// Send data to the LCD
static void lcd_data(const uint8_t *data, int len)
{
    if (len == 0) return;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(PIN_NUM_DC, 1);
    spi_device_transmit(spi, &t);
}

static void lcd_data_byte(uint8_t data)
{
    lcd_data(&data, 1);
}

// Initialize SPI
static void lcd_spi_init(void)
{
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
    
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &spi));
    ESP_LOGI(TAG, "SPI initialized");
}

// GC9107 initialization with complete setup
static void gc9107_init(void)
{
    ESP_LOGI(TAG, "Initializing GC9107 display controller");
    
    lcd_cmd(0x01); // Software reset
    vTaskDelay(pdMS_TO_TICKS(120));
    
    lcd_cmd(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Frame Rate Control
    lcd_cmd(0xB1);
    lcd_data_byte(0x01);
    lcd_data_byte(0x2C);
    lcd_data_byte(0x2D);
    
    // Frame Rate Control (Idle Mode)
    lcd_cmd(0xB2);
    lcd_data_byte(0x01);
    lcd_data_byte(0x2C);
    lcd_data_byte(0x2D);
    
    // Frame Rate Control (Partial Mode)
    lcd_cmd(0xB3);
    lcd_data_byte(0x01);
    lcd_data_byte(0x2C);
    lcd_data_byte(0x2D);
    lcd_data_byte(0x01);
    lcd_data_byte(0x2C);
    lcd_data_byte(0x2D);
    
    // Display Function Control
    lcd_cmd(0xB6);
    lcd_data_byte(0x0A);
    lcd_data_byte(0x82);
    lcd_data_byte(0x27);
    lcd_data_byte(0x00);
    
    // Power Control 1
    lcd_cmd(0xC0);
    lcd_data_byte(0x23);
    
    // Power Control 2
    lcd_cmd(0xC1);
    lcd_data_byte(0x10);
    
    // VCOM Control 1
    lcd_cmd(0xC5);
    lcd_data_byte(0x3E);
    lcd_data_byte(0x28);
    
    // VCOM Control 2
    lcd_cmd(0xC7);
    lcd_data_byte(0x86);
    
    // Memory Access Control - IMPORTANT for coordinate system
    lcd_cmd(0x36);
    lcd_data_byte(0x00); // No rotation, no mirroring
    
    // Pixel Format Set (RGB565)
    lcd_cmd(0x3A);
    lcd_data_byte(0x05);
    
    // Column Address Set - ensure full range
    lcd_cmd(0x2A);
    lcd_data_byte(0x00);
    lcd_data_byte(0x00);
    lcd_data_byte(0x00);
    lcd_data_byte(0x7F); // 128-1 = 127 = 0x7F
    
    // Page Address Set - ensure full range  
    lcd_cmd(0x2B);
    lcd_data_byte(0x00);
    lcd_data_byte(0x00);
    lcd_data_byte(0x00);
    lcd_data_byte(0x7F); // 128-1 = 127 = 0x7F
    
    // Positive Gamma Correction
    lcd_cmd(0xE0);
    uint8_t gamma_pos[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 
                          0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
    lcd_data(gamma_pos, sizeof(gamma_pos));
    
    // Negative Gamma Correction
    lcd_cmd(0xE1);
    uint8_t gamma_neg[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 
                          0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
    lcd_data(gamma_neg, sizeof(gamma_neg));
    
    // Display Inversion On (critical for GC9107)
    lcd_cmd(0x21);
    
    // Display ON
    lcd_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clear screen immediately after init
    lcd_fill_screen(COLOR_BLACK);
    
    ESP_LOGI(TAG, "GC9107 initialization complete");
}

// Set drawing area
static void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
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

// Force complete screen reset
static void lcd_force_full_clear(uint16_t color)
{
    ESP_LOGI(TAG, "Force clearing entire 128x128 display");
    
    // Set full address window with explicit commands
    lcd_cmd(0x2A); // Column Address Set
    lcd_data_byte(0x00); // Start column high
    lcd_data_byte(0x00); // Start column low 
    lcd_data_byte(0x00); // End column high
    lcd_data_byte(0x7F); // End column low (127)
    
    lcd_cmd(0x2B); // Page Address Set  
    lcd_data_byte(0x00); // Start page high
    lcd_data_byte(0x00); // Start page low
    lcd_data_byte(0x00); // End page high  
    lcd_data_byte(0x7F); // End page low (127)
    
    lcd_cmd(0x2C); // Memory Write
    
    // Prepare color data
    uint8_t color_bytes[2];
    color_bytes[0] = color >> 8;
    color_bytes[1] = color & 0xFF;
    
    gpio_set_level(PIN_NUM_DC, 1); // Data mode
    
    // Send 128x128 = 16384 pixels
    for (int i = 0; i < 16384; i++) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 16; // 16 bits per pixel
        t.tx_buffer = color_bytes;
        spi_device_transmit(spi, &t);
    }
    
    ESP_LOGI(TAG, "Screen clear complete - sent %d pixels", 16384);
}

// Fill screen with color - optimized version
static void lcd_fill_screen(uint16_t color)
{
    lcd_force_full_clear(color);
}

// Draw a character at specified position
static void lcd_draw_char(int x, int y, char c, uint16_t color, uint16_t bg_color)
{
    if (c < 32 || c > 126) c = 32; // Default to space for unsupported chars
    
    const uint8_t *char_data = font8x8[c - 32];
    
    lcd_set_addr_window(x, y, x + 7, y + 7);
    
    gpio_set_level(PIN_NUM_DC, 1);
    
    for (int row = 0; row < 8; row++) {
        uint8_t byte = char_data[row];
        for (int col = 0; col < 8; col++) {
            uint16_t pixel_color = (byte & (1 << (7 - col))) ? color : bg_color;
            
            uint8_t color_bytes[2];
            color_bytes[0] = pixel_color >> 8;
            color_bytes[1] = pixel_color & 0xFF;
            
            spi_transaction_t t;
            memset(&t, 0, sizeof(t));
            t.length = 16;
            t.tx_buffer = color_bytes;
            spi_device_transmit(spi, &t);
        }
    }
}

// Draw a string
static void lcd_draw_string(int x, int y, const char *str, uint16_t color, uint16_t bg_color)
{
    int cur_x = x;
    int cur_y = y;
    
    while (*str) {
        if (*str == '\n') {
            cur_x = x;
            cur_y += 8;
            if (cur_y >= LCD_V_RES) break;
        } else {
            if (cur_x + 8 <= LCD_H_RES) {
                lcd_draw_char(cur_x, cur_y, *str, color, bg_color);
                cur_x += 8;
            } else {
                cur_x = x;
                cur_y += 8;
                if (cur_y >= LCD_V_RES) break;
                lcd_draw_char(cur_x, cur_y, *str, color, bg_color);
                cur_x += 8;
            }
        }
        str++;
    }
}

// Clear a rectangular area
static void lcd_clear_area(int x, int y, int width, int height, uint16_t color)
{
    lcd_set_addr_window(x, y, x + width - 1, y + height - 1);
    
    uint8_t color_bytes[2];
    color_bytes[0] = color >> 8;
    color_bytes[1] = color & 0xFF;
    
    gpio_set_level(PIN_NUM_DC, 1);
    
    for (int i = 0; i < width * height; i++) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 16;
        t.tx_buffer = color_bytes;
        spi_device_transmit(spi, &t);
    }
}

// Display hardware information on LCD
static void display_hardware_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    
#if CONFIG_SPIRAM    
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
#else
    size_t psram_size = 0;
#endif
    
    char info_lines[8][16]; // Max 8 lines, 16 chars each for 128px width
    
    // Prepare text
    snprintf(info_lines[0], 16, "M5AtomS3R");
    snprintf(info_lines[1], 16, "ESP-IDF 6.0");
    snprintf(info_lines[2], 16, "Cores:%d", chip_info.cores);
    snprintf(info_lines[3], 16, "Flash:%.1fMB", flash_size / (1024.0 * 1024.0));
    snprintf(info_lines[4], 16, "PSRAM:%.1fMB", psram_size / (1024.0 * 1024.0));
    snprintf(info_lines[5], 16, "Freq:240MHz");
    snprintf(info_lines[6], 16, "LCD:128x128");
    snprintf(info_lines[7], 16, "Status:OK");
    
    // Clear entire screen completely
    ESP_LOGI(TAG, "Clearing screen...");
    lcd_fill_screen(COLOR_BLACK);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for clear to complete
    
    // Draw information with consistent spacing
    ESP_LOGI(TAG, "Drawing hardware information...");
    for (int i = 0; i < 8; i++) {
        // Clear the line area first
        lcd_clear_area(0, i * 12, LCD_H_RES, 12, COLOR_BLACK);
        
        // Draw text
        uint16_t text_color = (i == 0) ? COLOR_YELLOW : COLOR_WHITE;
        lcd_draw_string(2, 2 + i * 12, info_lines[i], text_color, COLOR_BLACK);
    }
    
    ESP_LOGI(TAG, "Hardware information displayed on LCD");
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R LCD Text Display Test");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize display
    lcd_io_init();
    lcd_spi_init();
    lcd_reset();
    gc9107_init();
    
    ESP_LOGI(TAG, "Starting LCD text display demo...");
    
    // Main loop
    int counter = 0;
    while (1) {
        // Display hardware info
        display_hardware_info();
        
        // Wait 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // Display counter with clean background
        ESP_LOGI(TAG, "Displaying counter: %d", counter);
        lcd_fill_screen(COLOR_BLUE);
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for clear to complete
        
        char counter_text[20];
        snprintf(counter_text, 20, "Count: %d", counter);
        
        // Clear text area and draw counter
        lcd_clear_area(10, 55, 108, 20, COLOR_BLUE);
        lcd_draw_string(20, 60, counter_text, COLOR_WHITE, COLOR_BLUE);
        
        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}