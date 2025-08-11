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

static const char *TAG = "SIMPLE_LCD_TEST";

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
#define PIN_NUM_BK_LIGHT_ALT    33  // Alternative backlight pin to try

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
    ESP_LOGI(TAG, "Initialize backlight - trying multiple pins and methods");
    
    // Try GPIO16 first (primary)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT, 1));
    ESP_LOGI(TAG, "GPIO16 set to HIGH");
    
    // Also try GPIO33 (alternative)
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_BK_LIGHT_ALT);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(PIN_NUM_BK_LIGHT_ALT, 1));
    ESP_LOGI(TAG, "GPIO33 set to HIGH");
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Try PWM on GPIO16
    ESP_LOGI(TAG, "Setting up PWM on GPIO16");
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 1000,
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
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_LOGI(TAG, "PWM on GPIO16 set to maximum");
    
    // Try different LCD panel settings for backlight control
    ESP_LOGI(TAG, "Backlight initialization complete - testing multiple approaches");
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
    
    // Reset sequence
    ESP_LOGI(TAG, "Resetting LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize panel
    ESP_LOGI(TAG, "Initializing LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set invert mode (some GC9107 need this)
    ESP_LOGI(TAG, "Setting panel invert");
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    // Turn on display
    ESP_LOGI(TAG, "Turning on display");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "LCD initialization complete");
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Simple LCD Test");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize hardware
    init_backlight();
    init_lcd();
    
    ESP_LOGI(TAG, "Starting LCD test pattern...");
    
    // Create a simple test pattern
    uint16_t *test_buffer = malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    if (!test_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test buffer");
        return;
    }
    
    int test_case = 0;
    
    while (1) {
        switch (test_case % 6) {
            case 0:
                // Fill screen with RED
                ESP_LOGI(TAG, "Test case 0: RED screen");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_RED;
                }
                break;
                
            case 1:
                // Fill screen with GREEN
                ESP_LOGI(TAG, "Test case 1: GREEN screen");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_GREEN;
                }
                break;
                
            case 2:
                // Fill screen with BLUE
                ESP_LOGI(TAG, "Test case 2: BLUE screen");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_BLUE;
                }
                break;
                
            case 3:
                // Fill screen with WHITE
                ESP_LOGI(TAG, "Test case 3: WHITE screen");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_WHITE;
                }
                break;
                
            case 4:
                // Fill screen with YELLOW
                ESP_LOGI(TAG, "Test case 4: YELLOW screen");
                for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                    test_buffer[i] = COLOR_YELLOW;
                }
                break;
                
            case 5:
                // Checkerboard pattern
                ESP_LOGI(TAG, "Test case 5: Checkerboard pattern");
                for (int y = 0; y < LCD_V_RES; y++) {
                    for (int x = 0; x < LCD_H_RES; x++) {
                        if ((x / 16 + y / 16) % 2 == 0) {
                            test_buffer[y * LCD_H_RES + x] = COLOR_BLACK;
                        } else {
                            test_buffer[y * LCD_H_RES + x] = COLOR_WHITE;
                        }
                    }
                }
                break;
        }
        
        // Update display
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, test_buffer);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to draw bitmap: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Display updated successfully");
        }
        
        test_case++;
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds between tests
    }
    
    free(test_buffer);
}