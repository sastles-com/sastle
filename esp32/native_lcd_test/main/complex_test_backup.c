#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ledc.h"

static const char *TAG = "M5ATOMS3R_CORRECT_TEST";

// M5AtomS3R正確なピン設定 (LovyanGFXと参考コードから)
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (80 * 1000 * 1000)  // 80MHz (LovyanGFXと同じ)

// 正確なピン配置
#define PIN_NUM_SCLK            17   // SPI SCLK
#define PIN_NUM_MOSI            21   // SPI MOSI  
#define PIN_NUM_MISO            -1   // SPI MISO (未使用)
#define PIN_NUM_LCD_DC          15   // D/C pin
#define PIN_NUM_LCD_RST         34   // Reset pin
#define PIN_NUM_LCD_CS          6    // Chip Select pin
#define PIN_NUM_BK_LIGHT        16   // Backlight pin

// LCD resolution
#define LCD_H_RES               128
#define LCD_V_RES               128

// LCD command/parameter bits
#define LCD_CMD_BITS            8
#define LCD_PARAM_BITS          8

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

static void init_backlight(void)
{
    ESP_LOGI(TAG, "Initialize backlight with PWM (M5AtomS3R)");
    
    // Configure backlight pin first as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Set HIGH first
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, 1));
    ESP_LOGI(TAG, "Backlight GPIO16 set to HIGH");
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set up PWM for backlight control (matching LovyanGFX approach)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 500,  // 500Hz as in LovyanGFX example
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
    
    // Set backlight to maximum brightness
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_LOGI(TAG, "Backlight PWM set to maximum brightness");
}

static void init_lcd(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus for M5AtomS3R");
    
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2, // Full frame buffer size
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO (GC9107 compatible)");
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

    ESP_LOGI(TAG, "Install GC9107 panel driver (using ST7789 compatible)");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,  // GC9107 typically uses BGR
        .bits_per_pixel = 16,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    
    // Reset panel
    ESP_LOGI(TAG, "Reset LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(120)); // Wait longer for GC9107
    
    // Initialize panel
    ESP_LOGI(TAG, "Initialize LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // M5AtomS3R specific settings based on LovyanGFX config
    // Set color inversion (important for GC9107)
    ESP_LOGI(TAG, "Set color inversion for GC9107");
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    // Turn on display
    ESP_LOGI(TAG, "Turn on display");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(120));
    
    ESP_LOGI(TAG, "LCD initialization complete - M5AtomS3R ready");
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Correct Pin Test");
    ESP_LOGI(TAG, "Based on LovyanGFX and M5Unified config");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize hardware with correct timing
    ESP_LOGI(TAG, "Step 1: Initialize backlight");
    init_backlight();
    
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for backlight to stabilize
    
    ESP_LOGI(TAG, "Step 2: Initialize LCD");
    init_lcd();
    
    ESP_LOGI(TAG, "Step 3: Starting visual test");
    
    // Create test buffer
    uint16_t *test_buffer = malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    if (!test_buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer!");
        return;
    }
    
    int test_pattern = 0;
    
    while (1) {
        switch (test_pattern % 7) {
            case 0:
                // Full RED screen
                ESP_LOGI(TAG, "Test pattern: FULL RED");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_RED;
                }
                break;
                
            case 1:
                // Full GREEN screen
                ESP_LOGI(TAG, "Test pattern: FULL GREEN");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_GREEN;
                }
                break;
                
            case 2:
                // Full BLUE screen
                ESP_LOGI(TAG, "Test pattern: FULL BLUE");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_BLUE;
                }
                break;
                
            case 3:
                // Full WHITE screen (maximum brightness test)
                ESP_LOGI(TAG, "Test pattern: FULL WHITE");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_WHITE;
                }
                break;
                
            case 4:
                // Full BLACK screen (backlight visibility test)
                ESP_LOGI(TAG, "Test pattern: FULL BLACK");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_BLACK;
                }
                break;
                
            case 5:
                // Gradient pattern
                ESP_LOGI(TAG, "Test pattern: GRADIENT");
                for (int y = 0; y < LCD_V_RES; y++) {
                    uint16_t color = (y * 31 / LCD_V_RES) << 11; // Red gradient
                    for (int x = 0; x < LCD_H_RES; x++) {
                        test_buffer[y * LCD_H_RES + x] = color;
                    }
                }
                break;
                
            case 6:
                // Large checkerboard (easier to see)
                ESP_LOGI(TAG, "Test pattern: LARGE CHECKERBOARD");
                for (int y = 0; y < LCD_V_RES; y++) {
                    for (int x = 0; x < LCD_H_RES; x++) {
                        if ((x / 32 + y / 32) % 2 == 0) {
                            test_buffer[y * LCD_H_RES + x] = COLOR_WHITE;
                        } else {
                            test_buffer[y * LCD_H_RES + x] = COLOR_BLACK;
                        }
                    }
                }
                break;
        }
        
        // Draw to screen
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, test_buffer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw bitmap: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Pattern %d drawn successfully", test_pattern);
        }
        
        test_pattern++;
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds per pattern
    }
    
    free(test_buffer);
}