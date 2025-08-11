#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_log.h"

static const char *TAG = "PSRAM_TEST";

void test_psram_basic(void) {
    ESP_LOGI(TAG, "=== PSRAM Basic Test ===");
    
    // PSRAMの初期化状態確認
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM is initialized ✓");
        ESP_LOGI(TAG, "PSRAM size: %zu bytes", esp_psram_get_size());
    } else {
        ESP_LOGE(TAG, "PSRAM is not initialized ✗");
        return;
    }
    
    // ヒープ情報表示
    ESP_LOGI(TAG, "Internal RAM free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "PSRAM free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Total free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    
    ESP_LOGI(TAG, "Largest internal block: %zu bytes", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Largest PSRAM block: %zu bytes", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}

void test_psram_allocation(void) {
    ESP_LOGI(TAG, "=== PSRAM Allocation Test ===");
    
    // 1MBのテストサイズ
    const size_t test_size = 1024 * 1024;
    void *psram_buffer = NULL;
    
    // PSRAMからメモリ確保
    psram_buffer = heap_caps_malloc(test_size, MALLOC_CAP_SPIRAM);
    if (psram_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes from PSRAM", test_size);
        return;
    }
    
    ESP_LOGI(TAG, "Successfully allocated %zu bytes from PSRAM at address: %p", test_size, psram_buffer);
    
    // メモリの書き込み・読み出しテスト
    ESP_LOGI(TAG, "Testing memory write/read...");
    uint8_t *buffer = (uint8_t *)psram_buffer;
    
    // パターン書き込み
    for (size_t i = 0; i < test_size; i++) {
        buffer[i] = (uint8_t)(i & 0xFF);
    }
    
    // パターン検証
    bool test_passed = true;
    for (size_t i = 0; i < test_size; i++) {
        if (buffer[i] != (uint8_t)(i & 0xFF)) {
            ESP_LOGE(TAG, "Memory test failed at offset %zu: expected %02X, got %02X", 
                     i, (uint8_t)(i & 0xFF), buffer[i]);
            test_passed = false;
            break;
        }
    }
    
    if (test_passed) {
        ESP_LOGI(TAG, "Memory write/read test PASSED ✓");
    } else {
        ESP_LOGE(TAG, "Memory write/read test FAILED ✗");
    }
    
    // メモリ解放
    heap_caps_free(psram_buffer);
    ESP_LOGI(TAG, "Memory freed");
}

void test_psram_performance(void) {
    ESP_LOGI(TAG, "=== PSRAM Performance Test ===");
    
    const size_t test_size = 256 * 1024; // 256KB
    const int iterations = 10;
    
    void *psram_buffer = heap_caps_malloc(test_size, MALLOC_CAP_SPIRAM);
    void *internal_buffer = heap_caps_malloc(test_size, MALLOC_CAP_INTERNAL);
    
    if (psram_buffer == NULL || internal_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers for performance test");
        if (psram_buffer) heap_caps_free(psram_buffer);
        if (internal_buffer) heap_caps_free(internal_buffer);
        return;
    }
    
    // PSRAM書き込み性能測定
    uint32_t start_time = xTaskGetTickCount();
    for (int i = 0; i < iterations; i++) {
        memset(psram_buffer, 0xAA, test_size);
    }
    uint32_t psram_write_time = xTaskGetTickCount() - start_time;
    
    // Internal RAM書き込み性能測定
    start_time = xTaskGetTickCount();
    for (int i = 0; i < iterations; i++) {
        memset(internal_buffer, 0xAA, test_size);
    }
    uint32_t internal_write_time = xTaskGetTickCount() - start_time;
    
    ESP_LOGI(TAG, "PSRAM write time: %lu ms", psram_write_time * portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Internal RAM write time: %lu ms", internal_write_time * portTICK_PERIOD_MS);
    
    heap_caps_free(psram_buffer);
    heap_caps_free(internal_buffer);
}

// External function from psram_advanced_test.c
extern void run_psram_advanced_tests(void);

void run_psram_tests(void) {
    ESP_LOGI(TAG, "Starting PSRAM comprehensive tests...");
    
    test_psram_basic();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_psram_allocation();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_psram_performance();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Run advanced PSRAM tests for 8MB validation
    ESP_LOGI(TAG, "Proceeding to advanced PSRAM tests...");
    run_psram_advanced_tests();
    
    ESP_LOGI(TAG, "All PSRAM tests completed ✓");
}