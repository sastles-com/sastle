#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"

static const char *TAG = "IMU_TEST";

// I2C configuration for M5AtomS3R
#define I2C_MASTER_SCL_IO           GPIO_NUM_2     // M5AtomS3R SCL pin (assumed)
#define I2C_MASTER_SDA_IO           GPIO_NUM_1     // M5AtomS3R SDA pin (assumed)
#define I2C_MASTER_NUM              0              // I2C port number
#define I2C_MASTER_FREQ_HZ          400000         // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0              // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0              // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// BMI270 I2C addresses and registers
#define BMI270_ADDR                 0x68           // BMI270 I2C address (7-bit)
#define BMI270_CHIP_ID              0x00           // Chip ID register
#define BMI270_CHIP_ID_VALUE        0x24           // Expected chip ID value
#define BMI270_ACC_X_LSB            0x0C           // Accelerometer X LSB
#define BMI270_GYR_X_LSB            0x12           // Gyroscope X LSB
#define BMI270_PWR_CTRL             0x7D           // Power control register
#define BMI270_PWR_CONF             0x7C           // Power configuration register

// BMM150 I2C addresses and registers (magnetometer)
#define BMM150_ADDR                 0x10           // BMM150 I2C address (7-bit)
#define BMM150_CHIP_ID              0x40           // Chip ID register
#define BMM150_CHIP_ID_VALUE        0x32           // Expected chip ID value

// Structure to hold IMU data
typedef struct {
    int16_t acc_x, acc_y, acc_z;    // Accelerometer data
    int16_t gyr_x, gyr_y, gyr_z;    // Gyroscope data
    int16_t mag_x, mag_y, mag_z;    // Magnetometer data
} imu_data_t;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(i2c_master_port, conf.mode, 
                             I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Write a byte to I2C device register
 */
static esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read a byte from I2C device register
 */
static esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read multiple bytes from I2C device register
 */
static esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Initialize BMI270 IMU sensor
 */
static esp_err_t bmi270_init(void)
{
    uint8_t chip_id;
    esp_err_t ret;
    
    // Read chip ID
    ret = i2c_read_byte(BMI270_ADDR, BMI270_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMI270 chip ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BMI270 Chip ID: 0x%02X (expected: 0x%02X)", chip_id, BMI270_CHIP_ID_VALUE);
    
    if (chip_id != BMI270_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "BMI270 chip ID mismatch!");
        return ESP_FAIL;
    }
    
    // Power on accelerometer and gyroscope
    ret = i2c_write_byte(BMI270_ADDR, BMI270_PWR_CTRL, 0x0E);  // Enable acc and gyr
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power on BMI270: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for sensor to stabilize
    
    ESP_LOGI(TAG, "BMI270 initialized successfully");
    return ESP_OK;
}

void print_chip_info(void) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "M5AtomS3R IMU Test");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Chip: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Features:%s%s%s%s",
            (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? " WiFi" : "",
            (chip_info.features & CHIP_FEATURE_BT) ? " BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? " BLE" : "",
            (chip_info.features & CHIP_FEATURE_IEEE802154) ? " 802.15.4" : "");
    ESP_LOGI(TAG, "Silicon revision: %d", chip_info.revision);
    // Get flash size using newer ESP-IDF v6.0 API
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash size: %dMB %s",
            flash_size / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}

/**
 * @brief Read IMU data from BMI270
 */
static esp_err_t read_imu_data(imu_data_t *imu_data)
{
    esp_err_t ret;
    uint8_t data[12];
    
    // Read accelerometer and gyroscope data from BMI270
    ret = i2c_read_bytes(BMI270_ADDR, BMI270_ACC_X_LSB, data, 12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMI270 data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Parse accelerometer data (bytes 0-5)
    imu_data->acc_x = (int16_t)((data[1] << 8) | data[0]);
    imu_data->acc_y = (int16_t)((data[3] << 8) | data[2]);
    imu_data->acc_z = (int16_t)((data[5] << 8) | data[4]);
    
    // Parse gyroscope data (bytes 6-11)
    imu_data->gyr_x = (int16_t)((data[7] << 8) | data[6]);
    imu_data->gyr_y = (int16_t)((data[9] << 8) | data[8]);
    imu_data->gyr_z = (int16_t)((data[11] << 8) | data[10]);
    
    // For now, set magnetometer data to zero
    imu_data->mag_x = 0;
    imu_data->mag_y = 0;
    imu_data->mag_z = 0;
    
    return ESP_OK;
}

/**
 * @brief IMU monitoring task
 */
static void imu_task(void *args)
{
    imu_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(33); // ~30Hz update rate
    
    ESP_LOGI(TAG, "IMU monitoring task started");
    
    while (1) {
        if (read_imu_data(&imu_data) == ESP_OK) {
            // Convert raw values to meaningful units (example scaling)
            float acc_x_g = imu_data.acc_x * 0.00024414f; // Approximate scaling for ±8g range
            float acc_y_g = imu_data.acc_y * 0.00024414f;
            float acc_z_g = imu_data.acc_z * 0.00024414f;
            
            float gyr_x_dps = imu_data.gyr_x * 0.0305176f; // Approximate scaling for ±1000 dps
            float gyr_y_dps = imu_data.gyr_y * 0.0305176f;
            float gyr_z_dps = imu_data.gyr_z * 0.0305176f;
            
            ESP_LOGI(TAG, "IMU Data:");
            ESP_LOGI(TAG, "  Acc(g): X=%.3f, Y=%.3f, Z=%.3f", acc_x_g, acc_y_g, acc_z_g);
            ESP_LOGI(TAG, "  Gyr(°/s): X=%.2f, Y=%.2f, Z=%.2f", gyr_x_dps, gyr_y_dps, gyr_z_dps);
            ESP_LOGI(TAG, "  Mag: X=%d, Y=%d, Z=%d", imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
            ESP_LOGI(TAG, "---");
        }
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

void print_memory_info(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Memory Information");
    ESP_LOGI(TAG, "========================================");
    
    // Internal RAM
    ESP_LOGI(TAG, "Internal RAM:");
    ESP_LOGI(TAG, "  Total: %d bytes", heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  Free: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  Largest free block: %d bytes", 
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    
#if CONFIG_SPIRAM
    // PSRAM check
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM:");
        ESP_LOGI(TAG, "  Status: Initialized ✓");
        ESP_LOGI(TAG, "  Total: %d bytes (%.2f MB)", 
                esp_psram_get_size(), 
                esp_psram_get_size() / (1024.0 * 1024.0));
        ESP_LOGI(TAG, "  Free: %d bytes", 
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI(TAG, "  Largest free block: %d bytes", 
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        
        // PSRAM test allocation
        void *test_ptr = heap_caps_malloc(1024 * 1024, MALLOC_CAP_SPIRAM);
        if (test_ptr != NULL) {
            ESP_LOGI(TAG, "  Test: 1MB allocation successful ✓");
            heap_caps_free(test_ptr);
        } else {
            ESP_LOGE(TAG, "  Test: 1MB allocation failed ✗");
        }
    } else {
        ESP_LOGW(TAG, "PSRAM: Not initialized ✗");
        ESP_LOGW(TAG, "Check menuconfig: Component config → ESP PSRAM");
    }
#else
    ESP_LOGI(TAG, "PSRAM: Disabled in configuration");
#endif
}


void app_main(void)
{
    ESP_LOGI(TAG, "=== M5AtomS3R IMU Test ===");
    
    // Print basic system information
    print_chip_info();
    print_memory_info();
    
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "PSRAM size: %zu bytes", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    
    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C master...");
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C master initialized successfully");
    
    // Scan I2C bus for devices
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found I2C device at address: 0x%02X", addr);
        }
    }
    
    // Initialize BMI270
    ESP_LOGI(TAG, "Initializing BMI270...");
    if (bmi270_init() != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 initialization failed!");
        ESP_LOGI(TAG, "Trying alternative I2C pins...");
        
        // Try alternative pin configuration
        i2c_driver_delete(I2C_MASTER_NUM);
        
        // Alternative configuration for M5AtomS3R
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_NUM_38,  // Alternative SDA
            .scl_io_num = GPIO_NUM_39,  // Alternative SCL
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
        
        esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (err == ESP_OK) {
            err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Trying alternative pins: SDA=38, SCL=39");
                if (bmi270_init() == ESP_OK) {
                    ESP_LOGI(TAG, "BMI270 initialized with alternative pins!");
                    goto imu_success;
                }
            }
        }
        
        ESP_LOGE(TAG, "Failed to initialize BMI270 with any pin configuration");
        return;
    }
    
imu_success:
    ESP_LOGI(TAG, "Creating IMU monitoring task...");
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "IMU test application started successfully");
}