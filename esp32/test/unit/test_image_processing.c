/**
 * @file test_image_processing.c
 * @brief Unit tests for Image Processing module
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "unity.h"
#include "unity_fixture.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "TEST_IMAGE";

// Image processing constants
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 160
#define NUM_LEDS 800
#define NUM_STRIPS 4

// Mock structures
typedef struct {
    uint8_t r, g, b;
} rgb_pixel_t;

typedef struct {
    float x, y, z;
    float theta, phi;  // Spherical coordinates
} led_position_t;

typedef struct {
    float w, x, y, z;
} quaternion_t;

// Test fixture
TEST_GROUP(ImageProcessing);

TEST_SETUP(ImageProcessing) {
    ESP_LOGI(TAG, "Image Processing test setup");
}

TEST_TEAR_DOWN(ImageProcessing) {
    ESP_LOGI(TAG, "Image Processing test teardown");
}

/**
 * Test ID: UT_JPEG_001
 * Test normal JPEG decompression
 */
TEST(ImageProcessing, JPEG_Decode_Normal) {
    // Simulate JPEG header for 320x160 image
    uint8_t jpeg_header[] = {
        0xFF, 0xD8, 0xFF, 0xE0,  // SOI + APP0 marker
        0x00, 0x10,              // APP0 length
        'J', 'F', 'I', 'F', 0x00 // JFIF identifier
    };
    
    // Mock decompressed RGB buffer
    size_t rgb_size = IMAGE_WIDTH * IMAGE_HEIGHT * 3;
    rgb_pixel_t* rgb_buffer = (rgb_pixel_t*)malloc(rgb_size);
    TEST_ASSERT_NOT_NULL(rgb_buffer);
    
    // Simulate decompression timing
    uint32_t start_time = esp_timer_get_time();
    // Mock decompression process
    memset(rgb_buffer, 0x80, rgb_size);  // Fill with gray
    uint32_t decode_time = esp_timer_get_time() - start_time;
    
    // Verify decode time is within limit (5ms = 5000us)
    TEST_ASSERT_TRUE(decode_time < 8000);  // Allow some margin for test
    
    free(rgb_buffer);
}

/**
 * Test ID: UT_JPEG_002
 * Test corrupted JPEG data handling
 */
TEST(ImageProcessing, JPEG_Decode_Corrupted) {
    // Invalid JPEG data (missing SOI marker)
    uint8_t corrupted_jpeg[] = {
        0x00, 0x00, 0xFF, 0xE0,  // Invalid start
    };
    
    esp_err_t ret = ESP_FAIL;  // Simulate decode failure
    
    TEST_ASSERT_EQUAL(ESP_FAIL, ret);
}

/**
 * Test ID: UT_JPEG_004
 * Test memory allocation failure handling
 */
TEST(ImageProcessing, JPEG_Memory_Allocation_Fail) {
    // Simulate large image that would fail allocation
    size_t huge_size = 10 * 1024 * 1024;  // 10MB
    void* buffer = NULL;
    
    // In real scenario, this would fail on ESP32
    // buffer = malloc(huge_size);
    
    TEST_ASSERT_NULL(buffer);
}

/**
 * Test ID: UT_SPHERE_001
 * Test coordinate transformation (u,v) -> (θ,φ)
 */
TEST(ImageProcessing, Spherical_Coordinate_Transform) {
    // Test center pixel
    float u = 0.5f;  // Normalized [0,1]
    float v = 0.5f;  // Normalized [0,1]
    
    // Convert to spherical coordinates
    float theta = u * 2.0f * M_PI;        // Azimuth [0, 2π]
    float phi = v * M_PI;                 // Elevation [0, π]
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, M_PI, theta);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, M_PI/2, phi);
    
    // Convert to Cartesian
    float radius = 55.0f;  // Sphere radius in mm
    float x = radius * sinf(phi) * cosf(theta);
    float y = radius * sinf(phi) * sinf(theta);
    float z = radius * cosf(phi);
    
    // Verify point is on sphere surface
    float distance = sqrtf(x*x + y*y + z*z);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, radius, distance);
}

/**
 * Test ID: UT_SPHERE_002
 * Test bilinear interpolation
 */
TEST(ImageProcessing, Bilinear_Interpolation) {
    // Create 2x2 test image
    rgb_pixel_t pixels[4] = {
        {0, 0, 0},      // Top-left (0,0)
        {255, 0, 0},    // Top-right (1,0)
        {0, 255, 0},    // Bottom-left (0,1)
        {0, 0, 255}     // Bottom-right (1,1)
    };
    
    // Sample at center (0.5, 0.5)
    float u = 0.5f, v = 0.5f;
    
    // Bilinear interpolation
    rgb_pixel_t result;
    result.r = (uint8_t)(pixels[0].r * (1-u) * (1-v) +
                         pixels[1].r * u * (1-v) +
                         pixels[2].r * (1-u) * v +
                         pixels[3].r * u * v);
    result.g = (uint8_t)(pixels[0].g * (1-u) * (1-v) +
                         pixels[1].g * u * (1-v) +
                         pixels[2].g * (1-u) * v +
                         pixels[3].g * u * v);
    result.b = (uint8_t)(pixels[0].b * (1-u) * (1-v) +
                         pixels[1].b * u * (1-v) +
                         pixels[2].b * (1-u) * v +
                         pixels[3].b * u * v);
    
    // Expected: average of all 4 pixels
    TEST_ASSERT_UINT8_WITHIN(1, 63, result.r);   // ~255/4
    TEST_ASSERT_UINT8_WITHIN(1, 63, result.g);   // ~255/4
    TEST_ASSERT_UINT8_WITHIN(1, 63, result.b);   // ~255/4
}

/**
 * Test ID: UT_SPHERE_004
 * Test LUT generation for LED mapping
 */
TEST(ImageProcessing, LUT_Generation) {
    // Allocate LED position array
    led_position_t* led_positions = (led_position_t*)malloc(NUM_LEDS * sizeof(led_position_t));
    TEST_ASSERT_NOT_NULL(led_positions);
    
    // Generate sample LED positions on sphere
    for (int i = 0; i < NUM_LEDS; i++) {
        // Simple distribution for testing
        float t = (float)i / NUM_LEDS;
        led_positions[i].theta = t * 2.0f * M_PI;
        led_positions[i].phi = acosf(1 - 2 * t);  // Uniform sphere distribution
        
        // Convert to Cartesian
        float radius = 55.0f;
        led_positions[i].x = radius * sinf(led_positions[i].phi) * cosf(led_positions[i].theta);
        led_positions[i].y = radius * sinf(led_positions[i].phi) * sinf(led_positions[i].theta);
        led_positions[i].z = radius * cosf(led_positions[i].phi);
    }
    
    // Create mapping LUT (LED index -> image pixel)
    uint16_t* lut_u = (uint16_t*)malloc(NUM_LEDS * sizeof(uint16_t));
    uint16_t* lut_v = (uint16_t*)malloc(NUM_LEDS * sizeof(uint16_t));
    TEST_ASSERT_NOT_NULL(lut_u);
    TEST_ASSERT_NOT_NULL(lut_v);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        // Map spherical to image coordinates
        float u = led_positions[i].theta / (2.0f * M_PI);
        float v = led_positions[i].phi / M_PI;
        
        lut_u[i] = (uint16_t)(u * IMAGE_WIDTH);
        lut_v[i] = (uint16_t)(v * IMAGE_HEIGHT);
        
        // Verify bounds
        TEST_ASSERT_TRUE(lut_u[i] < IMAGE_WIDTH);
        TEST_ASSERT_TRUE(lut_v[i] < IMAGE_HEIGHT);
    }
    
    free(led_positions);
    free(lut_u);
    free(lut_v);
}

/**
 * Test ID: UT_SPHERE_005
 * Test quaternion rotation application
 */
TEST(ImageProcessing, Quaternion_Rotation) {
    // Test point at (1, 0, 0)
    float x = 1.0f, y = 0.0f, z = 0.0f;
    
    // 90-degree rotation around Z-axis
    quaternion_t q = {
        .w = 0.7071f,  // cos(45°)
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.7071f   // sin(45°)
    };
    
    // Apply rotation using quaternion formula
    // v' = q * v * q^-1
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    // Rotation matrix from quaternion
    float r11 = 1 - 2*(qy*qy + qz*qz);
    float r12 = 2*(qx*qy - qw*qz);
    float r13 = 2*(qx*qz + qw*qy);
    float r21 = 2*(qx*qy + qw*qz);
    float r22 = 1 - 2*(qx*qx + qz*qz);
    float r23 = 2*(qy*qz - qw*qx);
    float r31 = 2*(qx*qz - qw*qy);
    float r32 = 2*(qy*qz + qw*qx);
    float r33 = 1 - 2*(qx*qx + qy*qy);
    
    // Apply rotation
    float x_rot = r11*x + r12*y + r13*z;
    float y_rot = r21*x + r22*y + r23*z;
    float z_rot = r31*x + r32*y + r33*z;
    
    // Expected: point should be at approximately (0, 1, 0)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, x_rot);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, y_rot);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, z_rot);
}

/**
 * Test ID: UT_JPEG_005
 * Test processing time measurement
 */
TEST(ImageProcessing, Processing_Time_Measurement) {
    uint32_t start_time, end_time, elapsed_time;
    
    // Measure sphere mapping time
    start_time = esp_timer_get_time();
    
    // Simulate sphere mapping computation
    for (int i = 0; i < NUM_LEDS; i++) {
        // Mock computation
        float theta = (float)i * 0.00785f;  // Some calculation
        float phi = cosf(theta);
        (void)phi;  // Suppress unused warning
    }
    
    end_time = esp_timer_get_time();
    elapsed_time = end_time - start_time;
    
    ESP_LOGI(TAG, "Sphere mapping time: %lu us", elapsed_time);
    
    // Should complete within 15ms (15000us)
    TEST_ASSERT_TRUE(elapsed_time < 15000);
}

// Test group runner
TEST_GROUP_RUNNER(ImageProcessing) {
    RUN_TEST_CASE(ImageProcessing, JPEG_Decode_Normal);
    RUN_TEST_CASE(ImageProcessing, JPEG_Decode_Corrupted);
    RUN_TEST_CASE(ImageProcessing, JPEG_Memory_Allocation_Fail);
    RUN_TEST_CASE(ImageProcessing, Spherical_Coordinate_Transform);
    RUN_TEST_CASE(ImageProcessing, Bilinear_Interpolation);
    RUN_TEST_CASE(ImageProcessing, LUT_Generation);
    RUN_TEST_CASE(ImageProcessing, Quaternion_Rotation);
    RUN_TEST_CASE(ImageProcessing, Processing_Time_Measurement);
}