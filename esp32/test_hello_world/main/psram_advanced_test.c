#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "PSRAM_ADVANCED";

#define PSRAM_TEST_SIZE_1MB   (1024 * 1024)
#define PSRAM_TEST_SIZE_2MB   (2 * 1024 * 1024)  
#define PSRAM_TEST_SIZE_4MB   (4 * 1024 * 1024)
#define PSRAM_TEST_SIZE_6MB   (6 * 1024 * 1024)
#define PSRAM_TEST_SIZE_8MB   (8 * 1024 * 1024)

typedef struct {
    void *ptr;
    size_t size;
    char name[32];
} psram_allocation_t;

static psram_allocation_t allocations[16];
static int allocation_count = 0;

void psram_memory_map_test(void) {
    ESP_LOGI(TAG, "=== PSRAM Memory Mapping Test ===");
    
    // Test different allocation sizes
    size_t test_sizes[] = {
        1024,       // 1KB
        64 * 1024,  // 64KB  
        256 * 1024, // 256KB
        1024 * 1024,// 1MB
        2 * 1024 * 1024, // 2MB
        0 // End marker
    };
    
    for (int i = 0; test_sizes[i] > 0; i++) {
        void *ptr = heap_caps_malloc(test_sizes[i], MALLOC_CAP_SPIRAM);
        if (ptr) {
            // Fill with test pattern
            uint8_t pattern = 0xAA + i;
            memset(ptr, pattern, test_sizes[i]);
            
            // Verify pattern
            uint8_t *check = (uint8_t*)ptr;
            bool verify_ok = true;
            for (size_t j = 0; j < test_sizes[i]; j++) {
                if (check[j] != pattern) {
                    verify_ok = false;
                    break;
                }
            }
            
            ESP_LOGI(TAG, "Size: %6d KB - Alloc: ✓ Pattern: %s Addr: %p", 
                    test_sizes[i] / 1024, 
                    verify_ok ? "✓" : "✗",
                    ptr);
                    
            if (allocation_count < 16) {
                allocations[allocation_count].ptr = ptr;
                allocations[allocation_count].size = test_sizes[i];
                snprintf(allocations[allocation_count].name, 32, "test_%dkb", test_sizes[i] / 1024);
                allocation_count++;
            }
        } else {
            ESP_LOGE(TAG, "Size: %6d KB - Alloc: ✗", test_sizes[i] / 1024);
        }
    }
}

void psram_performance_test(void) {
    ESP_LOGI(TAG, "=== PSRAM Performance Test ===");
    
    const size_t test_size = 1024 * 1024; // 1MB test
    uint8_t *psram_buffer = heap_caps_malloc(test_size, MALLOC_CAP_SPIRAM);
    uint8_t *internal_buffer = heap_caps_malloc(test_size, MALLOC_CAP_INTERNAL);
    
    if (!psram_buffer || !internal_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test buffers");
        if (psram_buffer) heap_caps_free(psram_buffer);
        if (internal_buffer) heap_caps_free(internal_buffer);
        return;
    }
    
    // Write speed test
    int64_t start_time = esp_timer_get_time();
    for (size_t i = 0; i < test_size; i++) {
        psram_buffer[i] = (uint8_t)(i & 0xFF);
    }
    int64_t psram_write_time = esp_timer_get_time() - start_time;
    
    start_time = esp_timer_get_time();
    for (size_t i = 0; i < test_size; i++) {
        internal_buffer[i] = (uint8_t)(i & 0xFF);
    }
    int64_t internal_write_time = esp_timer_get_time() - start_time;
    
    // Read speed test
    uint32_t psram_checksum = 0;
    start_time = esp_timer_get_time();
    for (size_t i = 0; i < test_size; i++) {
        psram_checksum += psram_buffer[i];
    }
    int64_t psram_read_time = esp_timer_get_time() - start_time;
    
    uint32_t internal_checksum = 0;
    start_time = esp_timer_get_time();
    for (size_t i = 0; i < test_size; i++) {
        internal_checksum += internal_buffer[i];
    }
    int64_t internal_read_time = esp_timer_get_time() - start_time;
    
    ESP_LOGI(TAG, "1MB Sequential Write:");
    ESP_LOGI(TAG, "  PSRAM:    %lld µs (%.2f MB/s)", psram_write_time, 
             (double)test_size / psram_write_time);
    ESP_LOGI(TAG, "  Internal: %lld µs (%.2f MB/s)", internal_write_time,
             (double)test_size / internal_write_time);
             
    ESP_LOGI(TAG, "1MB Sequential Read:");
    ESP_LOGI(TAG, "  PSRAM:    %lld µs (%.2f MB/s) Checksum: 0x%08X", 
             psram_read_time, (double)test_size / psram_read_time, psram_checksum);
    ESP_LOGI(TAG, "  Internal: %lld µs (%.2f MB/s) Checksum: 0x%08X", 
             internal_read_time, (double)test_size / internal_read_time, internal_checksum);
    
    heap_caps_free(psram_buffer);
    heap_caps_free(internal_buffer);
}

void psram_large_allocation_test(void) {
    ESP_LOGI(TAG, "=== PSRAM Large Allocation Test ===");
    
    size_t test_sizes[] = {
        PSRAM_TEST_SIZE_1MB,
        PSRAM_TEST_SIZE_2MB,
        PSRAM_TEST_SIZE_4MB,
        PSRAM_TEST_SIZE_6MB,
        0 // End marker
    };
    
    for (int i = 0; test_sizes[i] > 0; i++) {
        ESP_LOGI(TAG, "Testing %d MB allocation...", test_sizes[i] / (1024 * 1024));
        
        void *large_ptr = heap_caps_malloc(test_sizes[i], MALLOC_CAP_SPIRAM);
        if (large_ptr) {
            ESP_LOGI(TAG, "  Allocation: ✓ Address: %p", large_ptr);
            
            // Simple pattern test for large memory
            uint32_t *uint32_ptr = (uint32_t*)large_ptr;
            size_t uint32_count = test_sizes[i] / 4;
            
            ESP_LOGI(TAG, "  Writing pattern...");
            int64_t start = esp_timer_get_time();
            for (size_t j = 0; j < uint32_count; j += 1024) { // Test every 4KB
                uint32_ptr[j] = j;
            }
            int64_t write_time = esp_timer_get_time() - start;
            
            ESP_LOGI(TAG, "  Verifying pattern...");
            start = esp_timer_get_time();
            bool verify_ok = true;
            for (size_t j = 0; j < uint32_count; j += 1024) {
                if (uint32_ptr[j] != j) {
                    verify_ok = false;
                    break;
                }
            }
            int64_t verify_time = esp_timer_get_time() - start;
            
            ESP_LOGI(TAG, "  Pattern Test: %s (Write: %lld µs, Verify: %lld µs)", 
                    verify_ok ? "✓" : "✗", write_time, verify_time);
            
            heap_caps_free(large_ptr);
            ESP_LOGI(TAG, "  Memory freed: ✓");
        } else {
            ESP_LOGE(TAG, "  Allocation: ✗ (Failed)");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Brief pause between tests
    }
}

void psram_fragmentation_test(void) {
    ESP_LOGI(TAG, "=== PSRAM Fragmentation Test ===");
    
    const int num_allocs = 64;
    void *ptrs[num_allocs];
    const size_t alloc_size = 64 * 1024; // 64KB each
    
    // Allocate many small blocks
    int successful_allocs = 0;
    for (int i = 0; i < num_allocs; i++) {
        ptrs[i] = heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
        if (ptrs[i]) {
            successful_allocs++;
            // Write simple pattern
            memset(ptrs[i], 0xCC + (i % 16), alloc_size);
        }
    }
    
    ESP_LOGI(TAG, "Allocated %d/%d blocks of %d KB each", 
            successful_allocs, num_allocs, alloc_size / 1024);
    
    // Free every other block to create fragmentation
    int freed_count = 0;
    for (int i = 1; i < num_allocs; i += 2) {
        if (ptrs[i]) {
            heap_caps_free(ptrs[i]);
            ptrs[i] = NULL;
            freed_count++;
        }
    }
    
    ESP_LOGI(TAG, "Freed %d blocks to create fragmentation", freed_count);
    
    // Try to allocate larger block in fragmented space
    size_t large_size = freed_count * alloc_size / 2;
    void *large_ptr = heap_caps_malloc(large_size, MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "Large allocation (%d KB) in fragmented space: %s",
            large_size / 1024, large_ptr ? "✓" : "✗");
    
    if (large_ptr) {
        heap_caps_free(large_ptr);
    }
    
    // Free remaining blocks
    for (int i = 0; i < num_allocs; i += 2) {
        if (ptrs[i]) {
            heap_caps_free(ptrs[i]);
        }
    }
    
    ESP_LOGI(TAG, "All blocks freed");
}

void psram_concurrent_access_test(void) {
    ESP_LOGI(TAG, "=== PSRAM Concurrent Access Test ===");
    
    const size_t buffer_size = 512 * 1024; // 512KB
    uint8_t *psram_buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    
    if (!psram_buffer) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM buffer for concurrent test");
        return;
    }
    
    // Initialize buffer with known pattern
    for (size_t i = 0; i < buffer_size; i++) {
        psram_buffer[i] = (uint8_t)(i % 256);
    }
    
    ESP_LOGI(TAG, "Buffer initialized at %p", psram_buffer);
    
    // Test concurrent read from multiple points
    uint32_t checksums[4];
    int64_t start_time = esp_timer_get_time();
    
    // Simulate concurrent access by reading different portions
    for (int core = 0; core < 4; core++) {
        size_t start_idx = (buffer_size / 4) * core;
        size_t end_idx = start_idx + (buffer_size / 4);
        
        uint32_t checksum = 0;
        for (size_t i = start_idx; i < end_idx; i++) {
            checksum += psram_buffer[i];
        }
        checksums[core] = checksum;
    }
    
    int64_t read_time = esp_timer_get_time() - start_time;
    
    ESP_LOGI(TAG, "Concurrent read test completed in %lld µs", read_time);
    ESP_LOGI(TAG, "Checksums - Q1: 0x%08X Q2: 0x%08X Q3: 0x%08X Q4: 0x%08X",
            checksums[0], checksums[1], checksums[2], checksums[3]);
    
    heap_caps_free(psram_buffer);
}

void psram_memory_statistics(void) {
    ESP_LOGI(TAG, "=== PSRAM Memory Statistics ===");
    
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "PSRAM Heap Info:");
    ESP_LOGI(TAG, "  Total free bytes: %d", info.total_free_bytes);
    ESP_LOGI(TAG, "  Total allocated bytes: %d", info.total_allocated_bytes);
    ESP_LOGI(TAG, "  Largest free block: %d", info.largest_free_block);
    ESP_LOGI(TAG, "  Minimum free bytes: %d", info.minimum_free_bytes);
    ESP_LOGI(TAG, "  Allocated blocks: %d", info.allocated_blocks);
    ESP_LOGI(TAG, "  Free blocks: %d", info.free_blocks);
    ESP_LOGI(TAG, "  Total blocks: %d", info.total_blocks);
    
    // Calculate utilization
    float utilization = (float)info.total_allocated_bytes / 
                       (info.total_allocated_bytes + info.total_free_bytes) * 100.0f;
    
    ESP_LOGI(TAG, "  Memory utilization: %.1f%%", utilization);
    
    // Show active allocations from our tests
    ESP_LOGI(TAG, "Active test allocations:");
    for (int i = 0; i < allocation_count; i++) {
        if (allocations[i].ptr) {
            ESP_LOGI(TAG, "  %s: %d bytes at %p", 
                    allocations[i].name, 
                    allocations[i].size,
                    allocations[i].ptr);
        }
    }
}

void psram_cleanup_test_allocations(void) {
    ESP_LOGI(TAG, "=== Cleanup Test Allocations ===");
    
    for (int i = 0; i < allocation_count; i++) {
        if (allocations[i].ptr) {
            ESP_LOGI(TAG, "Freeing %s (%d bytes)", 
                    allocations[i].name, allocations[i].size);
            heap_caps_free(allocations[i].ptr);
            allocations[i].ptr = NULL;
        }
    }
    allocation_count = 0;
    
    ESP_LOGI(TAG, "All test allocations freed");
}

void run_psram_advanced_tests(void) {
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM not initialized - skipping advanced tests");
        return;
    }
    
    ESP_LOGI(TAG, "Starting PSRAM 8MB Advanced Tests...");
    ESP_LOGI(TAG, "PSRAM Size: %.2f MB", esp_psram_get_size() / (1024.0 * 1024.0));
    
    // Run all advanced tests
    psram_memory_map_test();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    psram_performance_test();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    psram_large_allocation_test();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    psram_fragmentation_test();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    psram_concurrent_access_test();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    psram_memory_statistics();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    psram_cleanup_test_allocations();
    
    ESP_LOGI(TAG, "PSRAM Advanced Tests Completed ✓");
}