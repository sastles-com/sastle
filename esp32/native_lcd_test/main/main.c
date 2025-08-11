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
#include "esp_system.h"

static const char *TAG = "MEMORY_INFO";

// M5AtomS3R LCD pins
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (26 * 1000 * 1000)
#define PIN_NUM_MOSI            21
#define PIN_NUM_CLK             15
#define PIN_NUM_CS              14
#define PIN_NUM_DC              42
#define PIN_NUM_RST             48
#define LCD_H_RES               128
#define LCD_V_RES               128

// Colors
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F

// Font for characters (8x8)
static const uint8_t font8x8_basic[96][8] = {
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

esp_lcd_panel_handle_t panel_handle = NULL;

static void draw_char_at(int x, int y, char c, uint16_t color, uint16_t *framebuffer)
{
    if (c >= 32 && c <= 127) {
        int char_index = c - 32;
        const uint8_t *char_data = font8x8_basic[char_index];
        
        for (int row = 0; row < 8; row++) {
            uint8_t byte = char_data[row];
            for (int col = 0; col < 8; col++) {
                int px = x + col;
                int py = y + row;
                if (px >= 0 && px < LCD_H_RES && py >= 0 && py < LCD_V_RES) {
                    uint16_t pixel_color = (byte & (1 << col)) ? color : COLOR_BLACK;
                    framebuffer[py * LCD_H_RES + px] = pixel_color;
                }
            }
        }
    }
}

static void draw_string_at(int x, int y, const char *str, uint16_t color, uint16_t *framebuffer)
{
    int cur_x = x;
    int cur_y = y;
    
    for (int i = 0; str[i]; i++) {
        if (str[i] == '\n') {
            cur_x = x;
            cur_y += 10;
        } else if (cur_x + 8 <= LCD_H_RES) {
            draw_char_at(cur_x, cur_y, str[i], color, framebuffer);
            cur_x += 6; // 6 pixels spacing for better readability
        }
    }
}

static void display_memory_info(uint16_t *framebuffer)
{
    // Clear screen
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        framebuffer[i] = COLOR_BLACK;
    }
    
    // Get chip info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    // Get Flash size
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    
    // Get PSRAM info
#if CONFIG_SPIRAM    
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#else
    size_t psram_total = 0;
    size_t psram_free = 0;
#endif
    
    // Get regular RAM info
    size_t internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    
    char line1[20], line2[20], line3[20], line4[20], line5[20], line6[20];
    
    // Format display strings  
    snprintf(line1, sizeof(line1), "Flash:%luMB", flash_size / (1024*1024));
    snprintf(line2, sizeof(line2), "PSRAM:%zuMB", psram_total / (1024*1024));
    snprintf(line3, sizeof(line3), "SRAM:%zuKB", internal_total / 1024);
    snprintf(line4, sizeof(line4), "Cores:%d", chip_info.cores);
    snprintf(line5, sizeof(line5), "CPU:%dMHz", 240); // ESP32-S3 default
    snprintf(line6, sizeof(line6), "Chip:ESP32S3");
    
    // Display information
    draw_string_at(4, 10, line6, COLOR_YELLOW, framebuffer);
    draw_string_at(4, 25, line1, (flash_size >= 8*1024*1024) ? COLOR_GREEN : COLOR_RED, framebuffer);
    draw_string_at(4, 40, line2, (psram_total >= 8*1024*1024) ? COLOR_GREEN : COLOR_RED, framebuffer);
    draw_string_at(4, 55, line3, COLOR_CYAN, framebuffer);
    draw_string_at(4, 70, line4, COLOR_CYAN, framebuffer);
    draw_string_at(4, 85, line5, COLOR_CYAN, framebuffer);
    
    // Status indicators (visual bars)
    // Flash bar (top)
    uint16_t flash_color = (flash_size >= 8*1024*1024) ? COLOR_GREEN : COLOR_RED;
    for (int x = 0; x < LCD_H_RES; x++) {
        for (int y = 0; y < 4; y++) {
            framebuffer[y * LCD_H_RES + x] = flash_color;
        }
    }
    
    // PSRAM bar (second)
    uint16_t psram_color = (psram_total >= 8*1024*1024) ? COLOR_GREEN : COLOR_RED;
    for (int x = 0; x < LCD_H_RES; x++) {
        for (int y = 100; y < 104; y++) {
            framebuffer[y * LCD_H_RES + x] = psram_color;
        }
    }
    
    // Border
    for (int x = 0; x < LCD_H_RES; x++) {
        framebuffer[(LCD_V_RES-1) * LCD_H_RES + x] = COLOR_WHITE;
    }
    for (int y = 0; y < LCD_V_RES; y++) {
        framebuffer[y * LCD_H_RES + 0] = COLOR_WHITE;
        framebuffer[y * LCD_H_RES + (LCD_H_RES-1)] = COLOR_WHITE;
    }
    
    // Log detailed information
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Memory Information");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Chip: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "CPU Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "CPU Frequency: 240 MHz");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "FLASH Memory:");
    ESP_LOGI(TAG, "  Size: %lu bytes (%.1f MB)", flash_size, flash_size / (1024.0*1024.0));
    ESP_LOGI(TAG, "  Status: %s", (flash_size >= 8*1024*1024) ? "✓ 8MB+ PASS" : "✗ <8MB FAIL");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "PSRAM Memory:");
    ESP_LOGI(TAG, "  Total: %zu bytes (%.1f MB)", psram_total, psram_total / (1024.0*1024.0));
    ESP_LOGI(TAG, "  Free: %zu bytes (%.1f MB)", psram_free, psram_free / (1024.0*1024.0));
    ESP_LOGI(TAG, "  Status: %s", (psram_total >= 8*1024*1024) ? "✓ 8MB+ PASS" : "✗ <8MB FAIL");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Internal SRAM:");
    ESP_LOGI(TAG, "  Total: %zu bytes (%.1f KB)", internal_total, internal_total / 1024.0);
    ESP_LOGI(TAG, "  Free: %zu bytes (%.1f KB)", internal_free, internal_free / 1024.0);
    ESP_LOGI(TAG, "========================================");
    
    // Test PSRAM allocation
    if (psram_total > 0) {
        ESP_LOGI(TAG, "Testing PSRAM allocation...");
        void *test_ptr = heap_caps_malloc(1024*1024, MALLOC_CAP_SPIRAM);
        if (test_ptr) {
            ESP_LOGI(TAG, "✓ PSRAM 1MB allocation: SUCCESS");
            // Write test pattern
            memset(test_ptr, 0xAA, 1024*1024);
            ESP_LOGI(TAG, "✓ PSRAM write test: SUCCESS");
            heap_caps_free(test_ptr);
        } else {
            ESP_LOGE(TAG, "✗ PSRAM allocation: FAILED");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "M5AtomS3R Memory Information Display");
    
    // Initialize LCD
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
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
    
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    // Allocate framebuffer
    uint16_t *framebuffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!framebuffer) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer");
        return;
    }
    
    while (1) {
        display_memory_info(framebuffer);
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, framebuffer));
        vTaskDelay(pdMS_TO_TICKS(3000)); // Update every 3 seconds
    }
}