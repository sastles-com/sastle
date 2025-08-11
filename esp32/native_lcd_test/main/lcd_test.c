#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "M5ATOMS3R_MANUAL_LCD";

// M5AtomS3R Display pins (correct pinout from official specs)
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (26 * 1000 * 1000)  // 26MHz (stable for GC9107)

#define PIN_NUM_MISO            -1
#define PIN_NUM_MOSI            21      // G21 - MOSI (correct)
#define PIN_NUM_CLK             15      // G15 - SCK (corrected)
#define PIN_NUM_CS              14      // G14 - CS (corrected) 
#define PIN_NUM_DC              42      // G42 - RS/DC (corrected)
#define PIN_NUM_RST             48      // G48 - RST (corrected)
// Note: BL is controlled by LP5562 I2C chip, not direct GPIO

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

// Initialize GPIO pins
static void lcd_io_init(void)
{
    ESP_LOGI(TAG, "Initialize display IO");
    
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
    
    // Note: Backlight controlled by LP5562 I2C chip - skip GPIO setup
    
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

// LP5562 backlight control (I2C based - simplified version)
static void lcd_backlight_on(void)
{
    ESP_LOGI(TAG, "Backlight control via LP5562 I2C chip (not implemented in this test)");
    // TODO: Implement LP5562 I2C control for proper backlight management
}

// Send a command to the LCD
static void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    
    memset(&t, 0, sizeof(t));       
    t.length = 8;                   // Command is 8 bits
    t.tx_buffer = &cmd;             // The data is the cmd itself
    
    gpio_set_level(PIN_NUM_DC, 0); // Command mode
    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
}

// Send data to the LCD
static void lcd_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    
    if (len == 0) return;           // no need to send anything
    
    memset(&t, 0, sizeof(t));       
    t.length = len * 8;             // Data length in bits
    t.tx_buffer = data;             // The data buffer
    
    gpio_set_level(PIN_NUM_DC, 1); // Data mode
    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
}

// Send a byte of data to LCD
static void lcd_data_byte(uint8_t data)
{
    lcd_data(&data, 1);
}

// Initialize SPI
static void lcd_spi_init(void)
{
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
        .mode = 0,                              // SPI mode 0
        .spics_io_num = PIN_NUM_CS,             
        .queue_size = 7,                        
    };
    
    // Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI initialized");
}

// GC9107 initialization sequence (similar to GC9A01)
static void gc9107_init(void)
{
    ESP_LOGI(TAG, "Initializing GC9107 display controller");
    
    // Software reset
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Sleep out
    lcd_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Pixel Format Set (RGB565)
    lcd_cmd(0x3A);
    lcd_data_byte(0x05);
    
    // Memory Access Control
    lcd_cmd(0x36);
    lcd_data_byte(0x00);
    
    // Display Function Control
    lcd_cmd(0xB6);
    lcd_data_byte(0x0A);
    lcd_data_byte(0x82);
    
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
    
    // Positive Gamma Correction
    lcd_cmd(0xE0);
    uint8_t gamma_pos[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
    lcd_data(gamma_pos, sizeof(gamma_pos));
    
    // Negative Gamma Correction
    lcd_cmd(0xE1);
    uint8_t gamma_neg[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
    lcd_data(gamma_neg, sizeof(gamma_neg));
    
    // Display Inversion On (important for GC9107)
    lcd_cmd(0x21);
    
    // Display ON
    lcd_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "GC9107 initialization complete");
}

// Set drawing area
static void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // Column Address Set
    lcd_cmd(0x2A);
    lcd_data_byte(x0 >> 8);
    lcd_data_byte(x0 & 0xFF);
    lcd_data_byte(x1 >> 8);
    lcd_data_byte(x1 & 0xFF);
    
    // Page Address Set
    lcd_cmd(0x2B);
    lcd_data_byte(y0 >> 8);
    lcd_data_byte(y0 & 0xFF);
    lcd_data_byte(y1 >> 8);
    lcd_data_byte(y1 & 0xFF);
    
    // Memory Write
    lcd_cmd(0x2C);
}

// Fill screen with color
static void lcd_fill_screen(uint16_t color)
{
    lcd_set_addr_window(0, 0, LCD_H_RES - 1, LCD_V_RES - 1);
    
    // Prepare color data
    uint8_t color_bytes[2];
    color_bytes[0] = color >> 8;
    color_bytes[1] = color & 0xFF;
    
    gpio_set_level(PIN_NUM_DC, 1); // Data mode
    
    // Send color data for all pixels
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        esp_err_t ret;
        spi_transaction_t t;
        
        memset(&t, 0, sizeof(t));
        t.length = 16;
        t.tx_buffer = color_bytes;
        
        ret = spi_device_transmit(spi, &t);
        assert(ret == ESP_OK);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Manual LCD Control Test");
    ESP_LOGI(TAG, "Direct GC9107 SPI Communication");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize all components
    lcd_io_init();
    lcd_spi_init();
    lcd_reset();
    gc9107_init();
    lcd_backlight_on();
    
    ESP_LOGI(TAG, "Starting display test patterns...");
    
    // Test colors
    uint16_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK, COLOR_YELLOW, COLOR_CYAN, COLOR_MAGENTA};
    const char* color_names[] = {"RED", "GREEN", "BLUE", "WHITE", "BLACK", "YELLOW", "CYAN", "MAGENTA"};
    int color_count = sizeof(colors) / sizeof(colors[0]);
    
    int counter = 0;
    
    while (1) {
        int color_index = counter % color_count;
        
        ESP_LOGI(TAG, "Displaying: %s (pattern %d)", color_names[color_index], counter);
        
        lcd_fill_screen(colors[color_index]);
        
        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds per color
    }
}