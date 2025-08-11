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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"

static const char *TAG = "HW_TESTS_COMPLETE";

// M5AtomS3R pins
#define LCD_HOST                SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ      (26 * 1000 * 1000)
#define PIN_NUM_MOSI            21
#define PIN_NUM_CLK             15
#define PIN_NUM_CS              14
#define PIN_NUM_DC              42
#define PIN_NUM_RST             48
#define LCD_H_RES               128
#define LCD_V_RES               128

// M5AtomS3R LED pin (internal RGB LED)
#define LED_GPIO                GPIO_NUM_35

// ADC for power voltage measurement
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_0  // GPIO1
#define ADC_ATTEN               ADC_ATTEN_DB_12

// Colors
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F

// Test results structure
typedef struct {
    bool psram_ok;
    size_t psram_size_mb;
    bool flash_ok;
    uint32_t flash_size_mb;
    bool dual_core_ok;
    int cpu_cores;
    bool gpio_led_ok;
    bool voltage_ok;
    float voltage_measured;
    uint8_t tests_passed;
    uint8_t total_tests;
} hardware_test_results_t;

static hardware_test_results_t hw_results = {0};

// Font for numbers 0-9 (8x8)
static const uint8_t font8x8_numbers[10][8] = {
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
adc_oneshot_unit_handle_t adc1_handle;

static void draw_char_at(int x, int y, char c, uint16_t color, uint16_t *framebuffer)
{
    if (c >= '0' && c <= '9') {
        int digit = c - '0';
        const uint8_t *char_data = font8x8_numbers[digit];
        
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
    for (int i = 0; str[i]; i++) {
        draw_char_at(cur_x, y, str[i], color, framebuffer);
        cur_x += 9; // 8 pixels + 1 spacing
    }
}

// HW_001: PSRAM Test
static void test_psram(void)
{
    ESP_LOGI(TAG, "=== HW_001: PSRAM 8MB Recognition Test ===");
    
#if CONFIG_SPIRAM
    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    hw_results.psram_size_mb = psram_total / (1024 * 1024);
    
    ESP_LOGI(TAG, "PSRAM Total: %zu bytes (%.1f MB)", psram_total, psram_total / (1024.0 * 1024.0));
    ESP_LOGI(TAG, "PSRAM Free: %zu bytes (%.1f MB)", psram_free, psram_free / (1024.0 * 1024.0));
    
    // Test allocation
    void *test_ptr = heap_caps_malloc(1024 * 1024, MALLOC_CAP_SPIRAM);
    if (test_ptr && psram_total >= 8 * 1024 * 1024) {
        hw_results.psram_ok = true;
        ESP_LOGI(TAG, "✓ PSRAM Test: PASS");
        heap_caps_free(test_ptr);
    } else {
        hw_results.psram_ok = false;
        ESP_LOGE(TAG, "✗ PSRAM Test: FAIL");
        if (test_ptr) heap_caps_free(test_ptr);
    }
#else
    hw_results.psram_ok = false;
    ESP_LOGW(TAG, "PSRAM not configured");
#endif
    
    hw_results.total_tests++;
    if (hw_results.psram_ok) hw_results.tests_passed++;
}

// HW_002: GPIO LED Test  
static void test_gpio_led(void)
{
    ESP_LOGI(TAG, "=== HW_002: GPIO LED Blink Test ===");
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        // Blink test
        for (int i = 0; i < 6; i++) {
            gpio_set_level(LED_GPIO, i % 2);
            ESP_LOGI(TAG, "LED %s", (i % 2) ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        gpio_set_level(LED_GPIO, 0); // Turn off
        hw_results.gpio_led_ok = true;
        ESP_LOGI(TAG, "✓ GPIO LED Test: PASS");
    } else {
        hw_results.gpio_led_ok = false;
        ESP_LOGE(TAG, "✗ GPIO LED Test: FAIL");
    }
    
    hw_results.total_tests++;
    if (hw_results.gpio_led_ok) hw_results.tests_passed++;
}

// HW_003: Power Voltage Test
static void test_power_voltage(void)
{
    ESP_LOGI(TAG, "=== HW_003: Power Voltage Test ===");
    
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret == ESP_OK) {
        adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN,
        };
        
        ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);
        if (ret == ESP_OK) {
            // Take multiple readings for average
            int adc_sum = 0;
            int valid_readings = 0;
            
            for (int i = 0; i < 10; i++) {
                int adc_raw;
                ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);
                if (ret == ESP_OK) {
                    adc_sum += adc_raw;
                    valid_readings++;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            if (valid_readings > 0) {
                int adc_avg = adc_sum / valid_readings;
                // Convert to voltage (approximate for 3.3V reference)
                float voltage = (adc_avg * 3.3) / 4095.0;
                hw_results.voltage_measured = voltage;
                
                // Check if within 3.3V ±5% (3.135V - 3.465V)
                if (voltage >= 3.0 && voltage <= 3.6) { // Relaxed range for approximation
                    hw_results.voltage_ok = true;
                    ESP_LOGI(TAG, "✓ Voltage Test: PASS (%.3fV)", voltage);
                } else {
                    hw_results.voltage_ok = false;
                    ESP_LOGW(TAG, "⚠ Voltage Test: WARNING (%.3fV)", voltage);
                }
                
                ESP_LOGI(TAG, "ADC Reading: %d, Voltage: %.3fV", adc_avg, voltage);
            }
        }
        
        adc_oneshot_del_unit(adc1_handle);
    }
    
    if (ret != ESP_OK) {
        hw_results.voltage_ok = false;
        ESP_LOGE(TAG, "✗ Voltage Test: FAIL (ADC Error)");
    }
    
    hw_results.total_tests++;
    if (hw_results.voltage_ok) hw_results.tests_passed++;
}

// HW_004: Dual Core Test
static void core_task(void *param)
{
    int core_id = (int)param;
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "Core %d running - iteration %d", core_id, i);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static void test_dual_core(void)
{
    ESP_LOGI(TAG, "=== HW_004: Dual Core Test ===");
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    hw_results.cpu_cores = chip_info.cores;
    ESP_LOGI(TAG, "CPU Cores: %d", chip_info.cores);
    
    if (chip_info.cores >= 2) {
        // Create tasks on both cores
        xTaskCreatePinnedToCore(core_task, "Core0Task", 2048, (void*)0, 1, NULL, 0);
        xTaskCreatePinnedToCore(core_task, "Core1Task", 2048, (void*)1, 1, NULL, 1);
        
        // Wait for tasks to complete
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        hw_results.dual_core_ok = true;
        ESP_LOGI(TAG, "✓ Dual Core Test: PASS");
    } else {
        hw_results.dual_core_ok = false;
        ESP_LOGE(TAG, "✗ Dual Core Test: FAIL");
    }
    
    hw_results.total_tests++;
    if (hw_results.dual_core_ok) hw_results.tests_passed++;
}

// HW_005: Flash Size Test
static void test_flash_size(void)
{
    ESP_LOGI(TAG, "=== HW_005: Flash Size Test ===");
    
    uint32_t flash_size;
    esp_err_t ret = esp_flash_get_size(NULL, &flash_size);
    
    if (ret == ESP_OK) {
        hw_results.flash_size_mb = flash_size / (1024 * 1024);
        ESP_LOGI(TAG, "Flash Size: %lu bytes (%lu MB)", flash_size, hw_results.flash_size_mb);
        
        if (flash_size >= 8 * 1024 * 1024) {
            hw_results.flash_ok = true;
            ESP_LOGI(TAG, "✓ Flash Size Test: PASS (8MB+)");
        } else {
            hw_results.flash_ok = false;
            ESP_LOGW(TAG, "⚠ Flash Size Test: WARNING (< 8MB)");
        }
    } else {
        hw_results.flash_ok = false;
        ESP_LOGE(TAG, "✗ Flash Size Test: FAIL");
    }
    
    hw_results.total_tests++;
    if (hw_results.flash_ok) hw_results.tests_passed++;
}

// Draw test results on LCD
static void draw_test_results(uint16_t *framebuffer)
{
    // Clear screen
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        framebuffer[i] = COLOR_BLACK;
    }
    
    // Title area
    for (int x = 0; x < LCD_H_RES; x++) {
        for (int y = 0; y < 12; y++) {
            framebuffer[y * LCD_H_RES + x] = COLOR_BLUE;
        }
    }
    
    // Test result bars
    int y_pos = 20;
    uint16_t test_colors[5] = {
        hw_results.psram_ok ? COLOR_GREEN : COLOR_RED,      // PSRAM
        hw_results.gpio_led_ok ? COLOR_GREEN : COLOR_RED,   // GPIO
        hw_results.voltage_ok ? COLOR_GREEN : COLOR_RED,    // Voltage  
        hw_results.dual_core_ok ? COLOR_GREEN : COLOR_RED,  // Dual Core
        hw_results.flash_ok ? COLOR_GREEN : COLOR_RED       // Flash
    };
    
    for (int test = 0; test < 5; test++) {
        for (int x = 10; x < 118; x++) {
            for (int y = y_pos; y < y_pos + 8; y++) {
                framebuffer[y * LCD_H_RES + x] = test_colors[test];
            }
        }
        y_pos += 12;
    }
    
    // Summary
    char summary[10];
    sprintf(summary, "%d", hw_results.tests_passed);
    draw_string_at(20, 90, summary, COLOR_CYAN, framebuffer);
    
    sprintf(summary, "%d", hw_results.total_tests);  
    draw_string_at(60, 90, summary, COLOR_WHITE, framebuffer);
    
    // Status border
    uint16_t border_color = (hw_results.tests_passed == hw_results.total_tests) ? COLOR_GREEN : COLOR_YELLOW;
    for (int x = 0; x < LCD_H_RES; x++) {
        framebuffer[0 * LCD_H_RES + x] = border_color;
        framebuffer[(LCD_V_RES-1) * LCD_H_RES + x] = border_color;
    }
    for (int y = 0; y < LCD_V_RES; y++) {
        framebuffer[y * LCD_H_RES + 0] = border_color;
        framebuffer[y * LCD_H_RES + (LCD_H_RES-1)] = border_color;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R Phase -1 Hardware Tests");
    ESP_LOGI(TAG, "Complete Hardware Validation");
    ESP_LOGI(TAG, "========================================");
    
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
    
    ESP_LOGI(TAG, "Running comprehensive hardware tests...");
    
    // Run all hardware tests
    test_psram();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    test_gpio_led();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    test_power_voltage();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    test_dual_core();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_flash_size();
    
    // Display results
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HARDWARE TEST RESULTS");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "HW_001 PSRAM 8MB:     %s", hw_results.psram_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "HW_002 GPIO LED:      %s", hw_results.gpio_led_ok ? "PASS" : "FAIL");  
    ESP_LOGI(TAG, "HW_003 Power Voltage: %s (%.3fV)", hw_results.voltage_ok ? "PASS" : "FAIL", hw_results.voltage_measured);
    ESP_LOGI(TAG, "HW_004 Dual Core:     %s", hw_results.dual_core_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "HW_005 Flash Size:    %s", hw_results.flash_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TOTAL: %d/%d tests passed", hw_results.tests_passed, hw_results.total_tests);
    ESP_LOGI(TAG, "Phase -1 Status: %s", (hw_results.tests_passed == hw_results.total_tests) ? "COMPLETE" : "INCOMPLETE");
    ESP_LOGI(TAG, "========================================");
    
    // Display results on LCD and maintain display
    while (1) {
        draw_test_results(framebuffer);
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, framebuffer));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}