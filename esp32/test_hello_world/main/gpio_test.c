#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "GPIO_TEST";

// M5AtomS3R Hardware Definitions
#define M5ATOMS3R_LED_PIN         GPIO_NUM_35   // Internal RGB LED
#define M5ATOMS3R_BUTTON_PIN      GPIO_NUM_41   // Internal button
#define M5ATOMS3R_ADC_PIN         ADC_CHANNEL_0 // GPIO1 - ADC for voltage monitoring

// LED PWM Configuration
#define LED_PWM_TIMER             LEDC_TIMER_0
#define LED_PWM_MODE              LEDC_LOW_SPEED_MODE
#define LED_PWM_CHANNEL_R         LEDC_CHANNEL_0
#define LED_PWM_CHANNEL_G         LEDC_CHANNEL_1
#define LED_PWM_CHANNEL_B         LEDC_CHANNEL_2
#define LED_PWM_DUTY_RES          LEDC_TIMER_13_BIT
#define LED_PWM_FREQUENCY         5000

// Global variables for ADC
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration1 = false;

// Task handles
static TaskHandle_t gpio_task_handle = NULL;
static TaskHandle_t led_blink_task_handle = NULL;
static TaskHandle_t button_monitor_task_handle = NULL;

void gpio_basic_setup(void) {
    ESP_LOGI(TAG, "=== GPIO Basic Setup ===");
    
    // Configure internal LED pin as output
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << M5ATOMS3R_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&led_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LED GPIO%d configured ✓", M5ATOMS3R_LED_PIN);
    } else {
        ESP_LOGE(TAG, "LED GPIO%d configuration failed: %s", M5ATOMS3R_LED_PIN, esp_err_to_name(ret));
    }
    
    // Configure internal button pin as input with pullup
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << M5ATOMS3R_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Interrupt on falling edge
    };
    
    ret = gpio_config(&button_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Button GPIO%d configured ✓", M5ATOMS3R_BUTTON_PIN);
    } else {
        ESP_LOGE(TAG, "Button GPIO%d configuration failed: %s", M5ATOMS3R_BUTTON_PIN, esp_err_to_name(ret));
    }
}

void led_pwm_setup(void) {
    ESP_LOGI(TAG, "=== LED PWM Setup ===");
    
    // Configure the timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LED_PWM_MODE,
        .timer_num = LED_PWM_TIMER,
        .duty_resolution = LED_PWM_DUTY_RES,
        .freq_hz = LED_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PWM Timer configured ✓");
    } else {
        ESP_LOGE(TAG, "PWM Timer configuration failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure RGB channels (assuming WS2812 or similar RGB LED on GPIO35)
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LED_PWM_MODE,
        .channel = LED_PWM_CHANNEL_R,
        .timer_sel = LED_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = M5ATOMS3R_LED_PIN,
        .duty = 0,
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LED PWM Channel configured ✓");
    } else {
        ESP_LOGE(TAG, "LED PWM Channel configuration failed: %s", esp_err_to_name(ret));
    }
}

void adc_setup(void) {
    ESP_LOGI(TAG, "=== ADC Setup for Voltage Monitoring ===");
    
    // ADC1 Init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // ADC1 Channel Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    
    ret = adc_oneshot_config_channel(adc1_handle, M5ATOMS3R_ADC_PIN, &config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Channel configured ✓");
    } else {
        ESP_LOGE(TAG, "ADC Channel configuration failed: %s", esp_err_to_name(ret));
    }
    
    // ADC Calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = M5ATOMS3R_ADC_PIN,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (ret == ESP_OK) {
        do_calibration1 = true;
        ESP_LOGI(TAG, "ADC Calibration initialized ✓");
    } else {
        ESP_LOGW(TAG, "ADC Calibration initialization failed: %s", esp_err_to_name(ret));
    }
}

void test_basic_gpio_output(void) {
    ESP_LOGI(TAG, "=== Basic GPIO Output Test ===");
    
    // Simple LED on/off test
    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "LED ON (iteration %d)", i + 1);
        gpio_set_level(M5ATOMS3R_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        ESP_LOGI(TAG, "LED OFF (iteration %d)", i + 1);
        gpio_set_level(M5ATOMS3R_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGI(TAG, "Basic GPIO output test completed ✓");
}

void test_pwm_output(void) {
    ESP_LOGI(TAG, "=== PWM Output Test ===");
    
    const uint32_t max_duty = (1 << LED_PWM_DUTY_RES) - 1;
    
    // Fade in
    ESP_LOGI(TAG, "PWM Fade In...");
    for (uint32_t duty = 0; duty <= max_duty; duty += max_duty / 100) {
        ledc_set_duty(LED_PWM_MODE, LED_PWM_CHANNEL_R, duty);
        ledc_update_duty(LED_PWM_MODE, LED_PWM_CHANNEL_R);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Fade out
    ESP_LOGI(TAG, "PWM Fade Out...");
    for (uint32_t duty = max_duty; duty > 0; duty -= max_duty / 100) {
        ledc_set_duty(LED_PWM_MODE, LED_PWM_CHANNEL_R, duty);
        ledc_update_duty(LED_PWM_MODE, LED_PWM_CHANNEL_R);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Turn off
    ledc_set_duty(LED_PWM_MODE, LED_PWM_CHANNEL_R, 0);
    ledc_update_duty(LED_PWM_MODE, LED_PWM_CHANNEL_R);
    
    ESP_LOGI(TAG, "PWM output test completed ✓");
}

void test_gpio_input(void) {
    ESP_LOGI(TAG, "=== GPIO Input Test ===");
    
    ESP_LOGI(TAG, "Press the button to test GPIO input...");
    
    int button_press_count = 0;
    int last_state = gpio_get_level(M5ATOMS3R_BUTTON_PIN);
    int64_t test_start = esp_timer_get_time();
    
    // Test for 10 seconds or 3 button presses, whichever comes first
    while ((esp_timer_get_time() - test_start) < 10000000 && button_press_count < 3) {
        int current_state = gpio_get_level(M5ATOMS3R_BUTTON_PIN);
        
        if (last_state == 1 && current_state == 0) { // Button pressed (falling edge)
            button_press_count++;
            ESP_LOGI(TAG, "Button pressed! Count: %d", button_press_count);
            
            // Visual feedback
            gpio_set_level(M5ATOMS3R_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(M5ATOMS3R_LED_PIN, 0);
        }
        
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (button_press_count > 0) {
        ESP_LOGI(TAG, "GPIO input test completed ✓ (%d button presses detected)", button_press_count);
    } else {
        ESP_LOGW(TAG, "GPIO input test completed - no button presses detected");
    }
}

void test_voltage_monitoring(void) {
    ESP_LOGI(TAG, "=== Voltage Monitoring Test ===");
    
    if (!adc1_handle) {
        ESP_LOGE(TAG, "ADC not initialized");
        return;
    }
    
    int adc_raw;
    int voltage_mv;
    float voltage_v;
    
    // Take multiple readings for accuracy
    int samples = 10;
    int total_raw = 0;
    int total_voltage = 0;
    
    for (int i = 0; i < samples; i++) {
        esp_err_t ret = adc_oneshot_read(adc1_handle, M5ATOMS3R_ADC_PIN, &adc_raw);
        if (ret == ESP_OK) {
            total_raw += adc_raw;
            
            if (do_calibration1) {
                ret = adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv);
                if (ret == ESP_OK) {
                    total_voltage += voltage_mv;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Calculate averages
    adc_raw = total_raw / samples;
    voltage_mv = total_voltage / samples;
    voltage_v = voltage_mv / 1000.0f;
    
    ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);
    if (do_calibration1) {
        ESP_LOGI(TAG, "Voltage: %d mV (%.3f V)", voltage_mv, voltage_v);
        
        // Check if voltage is within 3.3V ± 5% range
        if (voltage_v >= 3.135f && voltage_v <= 3.465f) {
            ESP_LOGI(TAG, "Voltage level check: ✓ (3.3V ± 5%)");
        } else {
            ESP_LOGW(TAG, "Voltage level check: ⚠ (Outside 3.3V ± 5% range)");
        }
    }
    
    ESP_LOGI(TAG, "Voltage monitoring test completed ✓");
}

void led_blink_task(void *pvParameters) {
    const int blink_period_ms = 1000;
    const int duty_cycle_percent = 50;
    const int on_time = (blink_period_ms * duty_cycle_percent) / 100;
    const int off_time = blink_period_ms - on_time;
    
    ESP_LOGI(TAG, "LED Blink Task started (Period: %dms, Duty: %d%%)", blink_period_ms, duty_cycle_percent);
    
    while (1) {
        gpio_set_level(M5ATOMS3R_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(on_time));
        
        gpio_set_level(M5ATOMS3R_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(off_time));
    }
}

void button_monitor_task(void *pvParameters) {
    int last_button_state = 1;
    int button_press_count = 0;
    
    ESP_LOGI(TAG, "Button Monitor Task started");
    
    while (1) {
        int current_button_state = gpio_get_level(M5ATOMS3R_BUTTON_PIN);
        
        if (last_button_state == 1 && current_button_state == 0) {
            button_press_count++;
            ESP_LOGI(TAG, "Button Press #%d detected by monitor task", button_press_count);
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void run_gpio_tests(void) {
    ESP_LOGI(TAG, "Starting GPIO comprehensive tests...");
    
    // Setup hardware
    gpio_basic_setup();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    led_pwm_setup();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    adc_setup();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Run tests
    test_basic_gpio_output();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_pwm_output();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_gpio_input();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_voltage_monitoring();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Start background tasks
    ESP_LOGI(TAG, "Starting background monitoring tasks...");
    
    BaseType_t ret = xTaskCreatePinnedToCore(
        led_blink_task,
        "led_blink",
        2048,
        NULL,
        5,
        &led_blink_task_handle,
        0
    );
    
    if (ret == pdPASS) {
        ESP_LOGI(TAG, "LED Blink task created ✓");
    } else {
        ESP_LOGE(TAG, "LED Blink task creation failed ✗");
    }
    
    ret = xTaskCreatePinnedToCore(
        button_monitor_task,
        "button_monitor",
        2048,
        NULL,
        5,
        &button_monitor_task_handle,
        1
    );
    
    if (ret == pdPASS) {
        ESP_LOGI(TAG, "Button Monitor task created ✓");
    } else {
        ESP_LOGE(TAG, "Button Monitor task creation failed ✗");
    }
    
    ESP_LOGI(TAG, "GPIO tests completed ✓");
    ESP_LOGI(TAG, "Background tasks running for hardware monitoring...");
}

void cleanup_gpio_tests(void) {
    ESP_LOGI(TAG, "Cleaning up GPIO tests...");
    
    // Delete tasks if running
    if (led_blink_task_handle) {
        vTaskDelete(led_blink_task_handle);
        led_blink_task_handle = NULL;
        ESP_LOGI(TAG, "LED Blink task deleted");
    }
    
    if (button_monitor_task_handle) {
        vTaskDelete(button_monitor_task_handle);
        button_monitor_task_handle = NULL;
        ESP_LOGI(TAG, "Button Monitor task deleted");
    }
    
    // Cleanup ADC
    if (adc1_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
        adc1_cali_handle = NULL;
    }
    
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    
    // Turn off LED
    gpio_set_level(M5ATOMS3R_LED_PIN, 0);
    
    ESP_LOGI(TAG, "GPIO cleanup completed ✓");
}