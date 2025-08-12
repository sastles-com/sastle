/**
 * @file lcd_unit_test.c
 * @brief Minimal LCD unit test for M5AtomS3R
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LCD_UNIT_TEST";

// GPIO Pin Configuration (confirmed working pins)
#define PIN_MOSI     21
#define PIN_SCLK     15
#define PIN_CS       14
#define PIN_DC       42
#define PIN_RST      48

// LCD dimensions
#define LCD_WIDTH    128
#define LCD_HEIGHT   128

// SPI handle
static spi_device_handle_t spi;

// Basic LCD commands for GC9107
#define LCD_CMD_SWRESET  0x01
#define LCD_CMD_SLPOUT   0x11
#define LCD_CMD_INVON    0x21
#define LCD_CMD_DISPON   0x29
#define LCD_CMD_CASET    0x2A
#define LCD_CMD_RASET    0x2B
#define LCD_CMD_RAMWR    0x2C
#define LCD_CMD_MADCTL   0x36
#define LCD_CMD_COLMOD   0x3A

// Send command to LCD
static void lcd_cmd(uint8_t cmd) {
    gpio_set_level(PIN_DC, 0);  // Command mode
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

// Send data to LCD
static void lcd_data(uint8_t data) {
    gpio_set_level(PIN_DC, 1);  // Data mode
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

// Send data buffer to LCD
static void lcd_data_buffer(const uint8_t *data, size_t len) {
    gpio_set_level(PIN_DC, 1);  // Data mode
    
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

// Initialize LCD with minimal setup
static void lcd_init_minimal(void) {
    ESP_LOGI(TAG, "Starting minimal LCD initialization...");
    
    // Hardware reset
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Software reset
    lcd_cmd(LCD_CMD_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Sleep out
    lcd_cmd(LCD_CMD_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Color mode - 16-bit RGB565
    lcd_cmd(LCD_CMD_COLMOD);
    lcd_data(0x55);
    
    // Memory access control
    lcd_cmd(LCD_CMD_MADCTL);
    lcd_data(0x00);
    
    // Inversion on
    lcd_cmd(LCD_CMD_INVON);
    
    // Display on
    lcd_cmd(LCD_CMD_DISPON);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    ESP_LOGI(TAG, "LCD initialization complete");
}

// Set window for drawing
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column address set
    lcd_cmd(LCD_CMD_CASET);
    lcd_data(x0 >> 8);
    lcd_data(x0 & 0xFF);
    lcd_data(x1 >> 8);
    lcd_data(x1 & 0xFF);
    
    // Row address set
    lcd_cmd(LCD_CMD_RASET);
    lcd_data(y0 >> 8);
    lcd_data(y0 & 0xFF);
    lcd_data(y1 >> 8);
    lcd_data(y1 & 0xFF);
    
    // Write to RAM
    lcd_cmd(LCD_CMD_RAMWR);
}

// Fill screen with solid color
static void lcd_fill_screen(uint16_t color) {
    ESP_LOGI(TAG, "Filling screen with color 0x%04X", color);
    
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;
    
    // Send one line at a time
    uint8_t line[LCD_WIDTH * 2];
    for (int i = 0; i < LCD_WIDTH; i++) {
        line[i * 2] = color_high;
        line[i * 2 + 1] = color_low;
    }
    
    for (int y = 0; y < LCD_HEIGHT; y++) {
        lcd_data_buffer(line, sizeof(line));
    }
}

// Test pattern - vertical stripes
static void lcd_test_pattern_stripes(void) {
    ESP_LOGI(TAG, "Drawing vertical stripes pattern");
    
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    
    uint8_t line[LCD_WIDTH * 2];
    
    // Create stripe pattern (alternating red and blue)
    for (int x = 0; x < LCD_WIDTH; x++) {
        uint16_t color = (x % 16 < 8) ? 0xF800 : 0x001F;  // Red : Blue
        line[x * 2] = color >> 8;
        line[x * 2 + 1] = color & 0xFF;
    }
    
    // Send pattern for all lines
    for (int y = 0; y < LCD_HEIGHT; y++) {
        lcd_data_buffer(line, sizeof(line));
    }
}

// Test pattern - checkerboard
static void lcd_test_pattern_checkerboard(void) {
    ESP_LOGI(TAG, "Drawing checkerboard pattern");
    
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    
    for (int y = 0; y < LCD_HEIGHT; y++) {
        for (int x = 0; x < LCD_WIDTH; x++) {
            uint16_t color = ((x / 8) + (y / 8)) % 2 ? 0xFFFF : 0x0000;
            uint8_t data[2] = {color >> 8, color & 0xFF};
            lcd_data_buffer(data, 2);
        }
    }
}

// Main unit test function
esp_err_t run_lcd_unit_test(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "M5AtomS3R LCD Unit Test");
    ESP_LOGI(TAG, "==========================================");
    
    // Initialize GPIO pins
    ESP_LOGI(TAG, "Initializing GPIO pins...");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DC) | (1ULL << PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Initialize SPI
    ESP_LOGI(TAG, "Initializing SPI bus...");
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1MHz for testing
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
    };
    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
    
    // Initialize LCD
    lcd_init_minimal();
    
    ESP_LOGI(TAG, "Starting LCD test sequence...");
    ESP_LOGI(TAG, "Each test will run for 3 seconds");
    
    // Test 1: Red screen
    ESP_LOGI(TAG, "Test 1: RED screen");
    lcd_fill_screen(0xF800);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 2: Green screen
    ESP_LOGI(TAG, "Test 2: GREEN screen");
    lcd_fill_screen(0x07E0);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 3: Blue screen
    ESP_LOGI(TAG, "Test 3: BLUE screen");
    lcd_fill_screen(0x001F);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 4: White screen
    ESP_LOGI(TAG, "Test 4: WHITE screen");
    lcd_fill_screen(0xFFFF);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 5: Black screen
    ESP_LOGI(TAG, "Test 5: BLACK screen");
    lcd_fill_screen(0x0000);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 6: Vertical stripes
    ESP_LOGI(TAG, "Test 6: Vertical stripes");
    lcd_test_pattern_stripes();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Test 7: Checkerboard
    ESP_LOGI(TAG, "Test 7: Checkerboard");
    lcd_test_pattern_checkerboard();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "LCD Unit Test Complete!");
    ESP_LOGI(TAG, "==========================================");
    
    return ESP_OK;
}