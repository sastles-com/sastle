#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

static const char *TAG = "ESP_LCD_GC9107";

// M5AtomS3R Display pins (correct pinout)
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (26 * 1000 * 1000)
#define PIN_NUM_MOSI            21      // G21 - MOSI
#define PIN_NUM_CLK             15      // G15 - SCK
#define PIN_NUM_CS              14      // G14 - CS
#define PIN_NUM_DC              42      // G42 - RS/DC
#define PIN_NUM_RST             48      // G48 - RST
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

// Simple 8x8 font bitmap for numbers and basic characters
static const uint8_t font8x8_basic[10][8] = {
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
};

esp_lcd_panel_handle_t panel_handle = NULL;

static void draw_number_at(int x, int y, int number, uint16_t color, uint16_t *framebuffer)
{
    if (number < 0 || number > 9) return;
    
    const uint8_t *char_data = font8x8_basic[number];
    
    for (int row = 0; row < 8; row++) {
        uint8_t byte = char_data[row];
        for (int col = 0; col < 8; col++) {
            int px = x + col;
            int py = y + row;
            if (px >= 0 && px < LCD_H_RES && py >= 0 && py < LCD_V_RES) {
                // 修正: ビットの読み取り順序を正しい向きに変更
                uint16_t pixel_color = (byte & (1 << col)) ? color : COLOR_BLACK;
                framebuffer[py * LCD_H_RES + px] = pixel_color;
            }
        }
    }
}

static void draw_hardware_info(uint16_t *framebuffer)
{
    // Clear framebuffer
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        framebuffer[i] = COLOR_BLACK;
    }
    
    // Get hardware information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    
#if CONFIG_SPIRAM    
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
#else
    size_t psram_size = 0;
#endif
    
    // Draw simple patterns to show hardware info
    // Top bar - Flash size indicator (blue intensity)
    uint16_t flash_color = (flash_size >= 8*1024*1024) ? COLOR_BLUE : COLOR_RED;
    for (int x = 0; x < LCD_H_RES; x++) {
        for (int y = 0; y < 8; y++) {
            framebuffer[y * LCD_H_RES + x] = flash_color;
        }
    }
    
    // Middle section - PSRAM indicator (green bar if available)
    if (psram_size > 0) {
        uint16_t psram_color = COLOR_GREEN;
        for (int x = 0; x < LCD_H_RES; x++) {
            for (int y = 20; y < 28; y++) {
                framebuffer[y * LCD_H_RES + x] = psram_color;
            }
        }
    }
    
    // CPU cores indicator - two squares for dual core
    for (int core = 0; core < chip_info.cores && core < 2; core++) {
        uint16_t core_color = COLOR_YELLOW;
        int start_x = 20 + core * 30;
        for (int x = start_x; x < start_x + 20; x++) {
            for (int y = 40; y < 60; y++) {
                framebuffer[y * LCD_H_RES + x] = core_color;
            }
        }
    }
    
    // Bottom area - Status OK (white border)
    for (int x = 0; x < LCD_H_RES; x++) {
        framebuffer[(LCD_V_RES - 1) * LCD_H_RES + x] = COLOR_WHITE;  // Bottom line
        framebuffer[(LCD_V_RES - 8) * LCD_H_RES + x] = COLOR_WHITE;  // Top of bottom area
    }
    for (int y = LCD_V_RES - 8; y < LCD_V_RES; y++) {
        framebuffer[y * LCD_H_RES + 0] = COLOR_WHITE;  // Left line
        framebuffer[y * LCD_H_RES + (LCD_H_RES - 1)] = COLOR_WHITE;  // Right line
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R ESP-IDF LCD Driver Test");
    ESP_LOGI(TAG, "Using esp_lcd panel interface");
    ESP_LOGI(TAG, "========================================");
    
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    ESP_LOGI(TAG, "SPI bus initialized");
    
    // Configure panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    
    ESP_LOGI(TAG, "Panel IO initialized");
    
    // Configure LCD panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    
    ESP_LOGI(TAG, "LCD panel created (using ST7789 driver for GC9107)");
    
    // Reset and initialize the panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Set inversion (important for GC9107)
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    // Turn on the display
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    ESP_LOGI(TAG, "LCD panel initialized successfully");
    
    // Allocate framebuffer
    uint16_t *framebuffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (framebuffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer");
        return;
    }
    
    ESP_LOGI(TAG, "Framebuffer allocated: %d bytes", LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    
    int counter = 0;
    while (1) {
        // Show hardware info
        draw_hardware_info(framebuffer);
        
        // Draw counter in the center
        int tens = (counter / 10) % 10;
        int ones = counter % 10;
        draw_number_at(50, 80, tens, COLOR_CYAN, framebuffer);
        draw_number_at(60, 80, ones, COLOR_CYAN, framebuffer);
        
        // Update display
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, framebuffer));
        
        ESP_LOGI(TAG, "Display updated - Counter: %d", counter);
        
        counter++;
        if (counter >= 100) counter = 0;
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}