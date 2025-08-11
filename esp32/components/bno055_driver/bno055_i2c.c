/**
 * @file bno055_i2c.c
 * @brief BNO055 I2C Communication Implementation
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bno055_driver.h"

static const char *TAG = "BNO055_I2C";

// I2C communication timeout
#define I2C_TIMEOUT_MS                  100
#define BNO055_STARTUP_TIME_MS          650     // BNO055 startup time
#define BNO055_MODE_SWITCH_DELAY_MS     30      // Delay after mode switch

/**
 * @brief Read single byte from BNO055 register
 */
esp_err_t bno055_read_byte(bno055_handle_t* handle, uint8_t reg, uint8_t* data) {
    if (!handle || !data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = i2c_master_write_read_device(
        handle->config.i2c_port,
        handle->config.dev_addr,
        &reg,
        1,
        data,
        1,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Write single byte to BNO055 register
 */
esp_err_t bno055_write_byte(bno055_handle_t* handle, uint8_t reg, uint8_t data) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t write_buf[2] = {reg, data};
    esp_err_t ret = i2c_master_write_to_device(
        handle->config.i2c_port,
        handle->config.dev_addr,
        write_buf,
        2,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read multiple bytes from BNO055 registers
 */
esp_err_t bno055_read_bytes(bno055_handle_t* handle, uint8_t reg, uint8_t* data, size_t len) {
    if (!handle || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = i2c_master_write_read_device(
        handle->config.i2c_port,
        handle->config.dev_addr,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read %d bytes from register 0x%02x: %s", 
                 len, reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Initialize I2C bus
 */
static esp_err_t bno055_init_i2c(const bno055_config_t* config) {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->clk_speed,
    };
    
    esp_err_t ret = i2c_param_config(config->i2c_port, &i2c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(config->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized: Port=%d, SDA=%d, SCL=%d, Speed=%luHz", 
             config->i2c_port, config->sda_pin, config->scl_pin, config->clk_speed);
    
    return ESP_OK;
}

/**
 * @brief Verify BNO055 chip ID and related IDs
 */
static esp_err_t bno055_verify_chip_id(bno055_handle_t* handle) {
    uint8_t chip_id, acc_id, mag_id, gyro_id;
    
    // Read and verify chip ID
    ESP_ERROR_CHECK(bno055_read_byte(handle, BNO055_REG_CHIP_ID, &chip_id));
    if (chip_id != BNO055_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: expected 0x%02x, got 0x%02x", BNO055_CHIP_ID, chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Read accelerometer ID
    ESP_ERROR_CHECK(bno055_read_byte(handle, BNO055_REG_ACC_ID, &acc_id));
    
    // Read magnetometer ID
    ESP_ERROR_CHECK(bno055_read_byte(handle, BNO055_REG_MAG_ID, &mag_id));
    
    // Read gyroscope ID
    ESP_ERROR_CHECK(bno055_read_byte(handle, BNO055_REG_GYRO_ID, &gyro_id));
    
    ESP_LOGI(TAG, "BNO055 IDs verified - Chip: 0x%02x, Acc: 0x%02x, Mag: 0x%02x, Gyro: 0x%02x",
             chip_id, acc_id, mag_id, gyro_id);
    
    return ESP_OK;
}

/**
 * @brief Configure BNO055 sensor settings
 */
static esp_err_t bno055_configure_sensor(bno055_handle_t* handle) {
    // Set to config mode first
    ESP_ERROR_CHECK(bno055_set_operation_mode(handle, BNO055_OPERATION_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(BNO055_MODE_SWITCH_DELAY_MS));
    
    // Set power mode to normal
    ESP_ERROR_CHECK(bno055_set_power_mode(handle, BNO055_POWER_MODE_NORMAL));
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Use external crystal if configured
    if (handle->config.use_external_crystal) {
        ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_SYS_TRIGGER, 0x80));
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "External crystal oscillator enabled");
    }
    
    // Set temperature source
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_TEMP_SOURCE, handle->config.temp_source));
    
    // Set unit selection (degrees, dps, m/s^2, Celsius)
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_UNIT_SEL, 0x00));
    
    // Set to desired operation mode
    ESP_ERROR_CHECK(bno055_set_operation_mode(handle, handle->config.op_mode));
    vTaskDelay(pdMS_TO_TICKS(BNO055_MODE_SWITCH_DELAY_MS));
    
    ESP_LOGI(TAG, "BNO055 sensor configured successfully");
    return ESP_OK;
}

/**
 * @brief Get default BNO055 configuration
 */
void bno055_get_default_config(bno055_config_t* config) {
    if (!config) return;
    
    config->i2c_port = I2C_NUM_0;
    config->dev_addr = BNO055_I2C_ADDR_A;
    config->sda_pin = GPIO_NUM_21;
    config->scl_pin = GPIO_NUM_22;
    config->clk_speed = 400000;  // 400kHz
    config->op_mode = BNO055_OPERATION_MODE_NDOF;
    config->use_external_crystal = true;
    config->temp_source = 0x01;  // Gyroscope temperature
}

/**
 * @brief Initialize BNO055 sensor
 */
esp_err_t bno055_init(bno055_handle_t* handle, const bno055_config_t* config) {
    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing BNO055 sensor");
    
    // Copy configuration
    handle->config = *config;
    handle->initialized = false;
    handle->last_read_time = 0;
    handle->data_ready = false;
    
    // Initialize I2C bus
    esp_err_t ret = bno055_init_i2c(config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for BNO055 startup
    vTaskDelay(pdMS_TO_TICKS(BNO055_STARTUP_TIME_MS));
    
    // Verify chip presence
    ret = bno055_verify_chip_id(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 chip verification failed");
        return ret;
    }
    
    // Configure sensor
    ret = bno055_configure_sensor(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 configuration failed");
        return ret;
    }
    
    // Try to load calibration data
    bno055_load_calibration_data(handle);
    
    handle->initialized = true;
    ESP_LOGI(TAG, "BNO055 initialization completed successfully");
    
    return ESP_OK;
}

/**
 * @brief Deinitialize BNO055 sensor
 */
esp_err_t bno055_deinit(bno055_handle_t* handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->initialized) {
        // Set to config mode
        bno055_set_operation_mode(handle, BNO055_OPERATION_MODE_CONFIG);
        
        // Set to suspend mode to save power
        bno055_set_power_mode(handle, BNO055_POWER_MODE_SUSPEND);
        
        handle->initialized = false;
        ESP_LOGI(TAG, "BNO055 deinitialized");
    }
    
    // Uninstall I2C driver
    i2c_driver_delete(handle->config.i2c_port);
    
    return ESP_OK;
}

/**
 * @brief Check if BNO055 is ready
 */
bool bno055_is_ready(const bno055_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return false;
    }
    
    // Check if enough time has passed since last read
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (current_time - handle->last_read_time) >= 10;  // 100Hz max
}

/**
 * @brief Set operation mode
 */
esp_err_t bno055_set_operation_mode(bno055_handle_t* handle, bno055_opmode_t mode) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = bno055_write_byte(handle, BNO055_REG_OPR_MODE, (uint8_t)mode);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BNO055_MODE_SWITCH_DELAY_MS));
        ESP_LOGD(TAG, "Operation mode set to: 0x%02x", mode);
    }
    
    return ret;
}

/**
 * @brief Get operation mode
 */
esp_err_t bno055_get_operation_mode(bno055_handle_t* handle, bno055_opmode_t* mode) {
    if (!handle || !mode) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mode_reg;
    esp_err_t ret = bno055_read_byte(handle, BNO055_REG_OPR_MODE, &mode_reg);
    if (ret == ESP_OK) {
        *mode = (bno055_opmode_t)(mode_reg & 0x0F);
    }
    
    return ret;
}

/**
 * @brief Set power mode
 */
esp_err_t bno055_set_power_mode(bno055_handle_t* handle, bno055_powermode_t mode) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = bno055_write_byte(handle, BNO055_REG_PWR_MODE, (uint8_t)mode);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGD(TAG, "Power mode set to: 0x%02x", mode);
    }
    
    return ret;
}

/**
 * @brief Reset sensor
 */
esp_err_t bno055_reset(bno055_handle_t* handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Resetting BNO055 sensor");
    
    esp_err_t ret = bno055_write_byte(handle, BNO055_REG_SYS_TRIGGER, 0x20);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BNO055_STARTUP_TIME_MS));
        handle->initialized = false;
        ESP_LOGI(TAG, "BNO055 reset completed");
    }
    
    return ret;
}

/**
 * @brief Read quaternion data
 */
esp_err_t bno055_read_quaternion(bno055_handle_t* handle, bno055_quaternion_t* quaternion) {
    if (!handle || !quaternion || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[8];
    esp_err_t ret = bno055_read_bytes(handle, BNO055_REG_QUATERNION_DATA_W_LSB, data, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw data to quaternion (BNO055 format: 1 LSB = 1/16384)
    int16_t w_raw = (int16_t)(data[1] << 8 | data[0]);
    int16_t x_raw = (int16_t)(data[3] << 8 | data[2]);
    int16_t y_raw = (int16_t)(data[5] << 8 | data[4]);
    int16_t z_raw = (int16_t)(data[7] << 8 | data[6]);
    
    const float scale = 1.0f / 16384.0f;
    quaternion->w = w_raw * scale;
    quaternion->x = x_raw * scale;
    quaternion->y = y_raw * scale;
    quaternion->z = z_raw * scale;
    
    return ESP_OK;
}

/**
 * @brief Read calibration status
 */
esp_err_t bno055_read_calibration_status(bno055_handle_t* handle, bno055_calibration_t* calibration) {
    if (!handle || !calibration || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t calib_stat;
    esp_err_t ret = bno055_read_byte(handle, BNO055_REG_CALIB_STAT, &calib_stat);
    if (ret != ESP_OK) {
        return ret;
    }
    
    calibration->system = (calib_stat >> 6) & 0x03;
    calibration->gyroscope = (calib_stat >> 4) & 0x03;
    calibration->accelerometer = (calib_stat >> 2) & 0x03;
    calibration->magnetometer = calib_stat & 0x03;
    
    handle->last_calibration = *calibration;
    
    return ESP_OK;
}

/**
 * @brief Check if sensor is fully calibrated
 */
bool bno055_is_fully_calibrated(const bno055_calibration_t* calibration) {
    if (!calibration) return false;
    
    return (calibration->system >= 3 && 
            calibration->gyroscope >= 3 && 
            calibration->accelerometer >= 3 && 
            calibration->magnetometer >= 3);
}

/**
 * @brief Read complete IMU data
 */
esp_err_t bno055_read_data(bno055_handle_t* handle, bno055_data_t* data) {
    if (!handle || !data || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    data->data_valid = false;
    
    // Read quaternion
    esp_err_t ret = bno055_read_quaternion(handle, &data->quaternion);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read calibration status
    ret = bno055_read_calibration_status(handle, &data->calibration);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read temperature
    ret = bno055_get_temperature(handle, &data->temperature);
    if (ret != ESP_OK) {
        data->temperature = 25;  // Default value
    }
    
    data->data_valid = true;
    handle->last_read_time = data->timestamp;
    
    return ESP_OK;
}

/**
 * @brief Get temperature reading
 */
esp_err_t bno055_get_temperature(bno055_handle_t* handle, int8_t* temperature) {
    if (!handle || !temperature || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t temp_data;
    esp_err_t ret = bno055_read_byte(handle, BNO055_REG_TEMP, &temp_data);
    if (ret == ESP_OK) {
        *temperature = (int8_t)temp_data;
        handle->temperature = *temperature;
    }
    
    return ret;
}