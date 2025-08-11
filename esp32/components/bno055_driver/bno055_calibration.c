/**
 * @file bno055_calibration.c
 * @brief BNO055 Calibration Data Management Implementation
 * @author Isolation Sphere Team
 * @date 2024
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bno055_driver.h"

static const char *TAG = "BNO055_CALIB";

// NVS namespace for BNO055 calibration data
#define BNO055_NVS_NAMESPACE "bno055_calib"

// NVS keys for calibration data
#define NVS_KEY_ACCEL_OFFSET_X  "acc_off_x"
#define NVS_KEY_ACCEL_OFFSET_Y  "acc_off_y"
#define NVS_KEY_ACCEL_OFFSET_Z  "acc_off_z"
#define NVS_KEY_MAG_OFFSET_X    "mag_off_x"
#define NVS_KEY_MAG_OFFSET_Y    "mag_off_y"
#define NVS_KEY_MAG_OFFSET_Z    "mag_off_z"
#define NVS_KEY_GYRO_OFFSET_X   "gyr_off_x"
#define NVS_KEY_GYRO_OFFSET_Y   "gyr_off_y"
#define NVS_KEY_GYRO_OFFSET_Z   "gyr_off_z"
#define NVS_KEY_ACCEL_RADIUS    "acc_radius"
#define NVS_KEY_MAG_RADIUS      "mag_radius"
#define NVS_KEY_CALIB_STATUS    "calib_stat"

/**
 * @brief Calibration data structure
 */
typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t accel_radius;
    int16_t mag_radius;
    uint8_t calib_status;
} bno055_calib_data_t;

/**
 * @brief Read calibration offsets from BNO055
 */
static esp_err_t bno055_read_calibration_offsets(bno055_handle_t* handle, bno055_calib_data_t* calib_data) {
    if (!handle || !calib_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Must be in CONFIG mode to read calibration data
    bno055_opmode_t current_mode;
    ESP_ERROR_CHECK(bno055_get_operation_mode(handle, &current_mode));
    
    if (current_mode != BNO055_OPERATION_MODE_CONFIG) {
        ESP_ERROR_CHECK(bno055_set_operation_mode(handle, BNO055_OPERATION_MODE_CONFIG));
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    
    uint8_t data[2];
    
    // Read accelerometer offsets
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_ACCEL_OFFSET_X_LSB, data, 2));
    calib_data->accel_offset_x = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_ACCEL_OFFSET_Y_LSB, data, 2));
    calib_data->accel_offset_y = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_ACCEL_OFFSET_Z_LSB, data, 2));
    calib_data->accel_offset_z = (int16_t)(data[1] << 8 | data[0]);
    
    // Read magnetometer offsets
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_MAG_OFFSET_X_LSB, data, 2));
    calib_data->mag_offset_x = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_MAG_OFFSET_Y_LSB, data, 2));
    calib_data->mag_offset_y = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_MAG_OFFSET_Z_LSB, data, 2));
    calib_data->mag_offset_z = (int16_t)(data[1] << 8 | data[0]);
    
    // Read gyroscope offsets
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_GYRO_OFFSET_X_LSB, data, 2));
    calib_data->gyro_offset_x = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_GYRO_OFFSET_Y_LSB, data, 2));
    calib_data->gyro_offset_y = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_GYRO_OFFSET_Z_LSB, data, 2));
    calib_data->gyro_offset_z = (int16_t)(data[1] << 8 | data[0]);
    
    // Read radius values
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_ACCEL_RADIUS_LSB, data, 2));
    calib_data->accel_radius = (int16_t)(data[1] << 8 | data[0]);
    
    ESP_ERROR_CHECK(bno055_read_bytes(handle, BNO055_REG_MAG_RADIUS_LSB, data, 2));
    calib_data->mag_radius = (int16_t)(data[1] << 8 | data[0]);
    
    // Read calibration status
    ESP_ERROR_CHECK(bno055_read_byte(handle, BNO055_REG_CALIB_STAT, &calib_data->calib_status));
    
    // Restore original mode
    if (current_mode != BNO055_OPERATION_MODE_CONFIG) {
        ESP_ERROR_CHECK(bno055_set_operation_mode(handle, current_mode));
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    
    return ESP_OK;
}

/**
 * @brief Write calibration offsets to BNO055
 */
static esp_err_t bno055_write_calibration_offsets(bno055_handle_t* handle, const bno055_calib_data_t* calib_data) {
    if (!handle || !calib_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Must be in CONFIG mode to write calibration data
    bno055_opmode_t current_mode;
    ESP_ERROR_CHECK(bno055_get_operation_mode(handle, &current_mode));
    
    ESP_ERROR_CHECK(bno055_set_operation_mode(handle, BNO055_OPERATION_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(30));
    
    // Write accelerometer offsets
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_OFFSET_X_LSB, calib_data->accel_offset_x & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_OFFSET_X_MSB, (calib_data->accel_offset_x >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_OFFSET_Y_LSB, calib_data->accel_offset_y & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_OFFSET_Y_MSB, (calib_data->accel_offset_y >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_OFFSET_Z_LSB, calib_data->accel_offset_z & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_OFFSET_Z_MSB, (calib_data->accel_offset_z >> 8) & 0xFF));
    
    // Write magnetometer offsets
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_OFFSET_X_LSB, calib_data->mag_offset_x & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_OFFSET_X_MSB, (calib_data->mag_offset_x >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_OFFSET_Y_LSB, calib_data->mag_offset_y & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_OFFSET_Y_MSB, (calib_data->mag_offset_y >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_OFFSET_Z_LSB, calib_data->mag_offset_z & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_OFFSET_Z_MSB, (calib_data->mag_offset_z >> 8) & 0xFF));
    
    // Write gyroscope offsets
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_GYRO_OFFSET_X_LSB, calib_data->gyro_offset_x & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_GYRO_OFFSET_X_MSB, (calib_data->gyro_offset_x >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_GYRO_OFFSET_Y_LSB, calib_data->gyro_offset_y & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_GYRO_OFFSET_Y_MSB, (calib_data->gyro_offset_y >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_GYRO_OFFSET_Z_LSB, calib_data->gyro_offset_z & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_GYRO_OFFSET_Z_MSB, (calib_data->gyro_offset_z >> 8) & 0xFF));
    
    // Write radius values
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_RADIUS_LSB, calib_data->accel_radius & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_ACCEL_RADIUS_MSB, (calib_data->accel_radius >> 8) & 0xFF));
    
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_RADIUS_LSB, calib_data->mag_radius & 0xFF));
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_MAG_RADIUS_MSB, (calib_data->mag_radius >> 8) & 0xFF));
    
    // Restore original mode
    ESP_ERROR_CHECK(bno055_set_operation_mode(handle, current_mode));
    vTaskDelay(pdMS_TO_TICKS(30));
    
    ESP_LOGI(TAG, "Calibration data written to BNO055");
    
    return ESP_OK;
}

/**
 * @brief Save calibration data to NVS
 */
esp_err_t bno055_save_calibration_data(bno055_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Saving calibration data to NVS");
    
    // Read current calibration data from sensor
    bno055_calib_data_t calib_data;
    esp_err_t ret = bno055_read_calibration_offsets(handle, &calib_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data from sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Open NVS handle
    nvs_handle_t nvs_handle;
    ret = nvs_open(BNO055_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save each calibration parameter
    nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_OFFSET_X, calib_data.accel_offset_x);
    nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_OFFSET_Y, calib_data.accel_offset_y);
    nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_OFFSET_Z, calib_data.accel_offset_z);
    
    nvs_set_i16(nvs_handle, NVS_KEY_MAG_OFFSET_X, calib_data.mag_offset_x);
    nvs_set_i16(nvs_handle, NVS_KEY_MAG_OFFSET_Y, calib_data.mag_offset_y);
    nvs_set_i16(nvs_handle, NVS_KEY_MAG_OFFSET_Z, calib_data.mag_offset_z);
    
    nvs_set_i16(nvs_handle, NVS_KEY_GYRO_OFFSET_X, calib_data.gyro_offset_x);
    nvs_set_i16(nvs_handle, NVS_KEY_GYRO_OFFSET_Y, calib_data.gyro_offset_y);
    nvs_set_i16(nvs_handle, NVS_KEY_GYRO_OFFSET_Z, calib_data.gyro_offset_z);
    
    nvs_set_i16(nvs_handle, NVS_KEY_ACCEL_RADIUS, calib_data.accel_radius);
    nvs_set_i16(nvs_handle, NVS_KEY_MAG_RADIUS, calib_data.mag_radius);
    
    nvs_set_u8(nvs_handle, NVS_KEY_CALIB_STATUS, calib_data.calib_status);
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Calibration data saved successfully");
    }
    
    // Close NVS handle
    nvs_close(nvs_handle);
    
    return ret;
}

/**
 * @brief Load calibration data from NVS
 */
esp_err_t bno055_load_calibration_data(bno055_handle_t* handle) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Loading calibration data from NVS");
    
    // Open NVS handle
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(BNO055_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "No calibration data found in NVS");
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    bno055_calib_data_t calib_data = {0};
    size_t required_size;
    
    // Load each calibration parameter
    required_size = sizeof(int16_t);
    nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_OFFSET_X, &calib_data.accel_offset_x);
    nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_OFFSET_Y, &calib_data.accel_offset_y);
    nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_OFFSET_Z, &calib_data.accel_offset_z);
    
    nvs_get_i16(nvs_handle, NVS_KEY_MAG_OFFSET_X, &calib_data.mag_offset_x);
    nvs_get_i16(nvs_handle, NVS_KEY_MAG_OFFSET_Y, &calib_data.mag_offset_y);
    nvs_get_i16(nvs_handle, NVS_KEY_MAG_OFFSET_Z, &calib_data.mag_offset_z);
    
    nvs_get_i16(nvs_handle, NVS_KEY_GYRO_OFFSET_X, &calib_data.gyro_offset_x);
    nvs_get_i16(nvs_handle, NVS_KEY_GYRO_OFFSET_Y, &calib_data.gyro_offset_y);
    nvs_get_i16(nvs_handle, NVS_KEY_GYRO_OFFSET_Z, &calib_data.gyro_offset_z);
    
    nvs_get_i16(nvs_handle, NVS_KEY_ACCEL_RADIUS, &calib_data.accel_radius);
    nvs_get_i16(nvs_handle, NVS_KEY_MAG_RADIUS, &calib_data.mag_radius);
    
    required_size = sizeof(uint8_t);
    nvs_get_u8(nvs_handle, NVS_KEY_CALIB_STATUS, &calib_data.calib_status);
    
    // Close NVS handle
    nvs_close(nvs_handle);
    
    // Write calibration data to sensor
    ret = bno055_write_calibration_offsets(handle, &calib_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write calibration data to sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Calibration data loaded and applied successfully");
    ESP_LOGI(TAG, "Calibration status: System=%d, Gyro=%d, Accel=%d, Mag=%d",
             (calib_data.calib_status >> 6) & 0x03,
             (calib_data.calib_status >> 4) & 0x03,
             (calib_data.calib_status >> 2) & 0x03,
             calib_data.calib_status & 0x03);
    
    return ESP_OK;
}

/**
 * @brief Clear calibration data from NVS
 */
esp_err_t bno055_clear_calibration_data(void) {
    ESP_LOGI(TAG, "Clearing calibration data from NVS");
    
    // Open NVS handle
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(BNO055_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Erase all calibration data
    ret = nvs_erase_all(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error erasing NVS data: %s", esp_err_to_name(ret));
    } else {
        ret = nvs_commit(nvs_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error committing NVS erase: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Calibration data cleared successfully");
        }
    }
    
    // Close NVS handle
    nvs_close(nvs_handle);
    
    return ret;
}

/**
 * @brief Perform self-test
 */
esp_err_t bno055_self_test(bno055_handle_t* handle, uint8_t* result) {
    if (!handle || !result || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Performing BNO055 self-test");
    
    // Must be in CONFIG mode for self-test
    bno055_opmode_t current_mode;
    ESP_ERROR_CHECK(bno055_get_operation_mode(handle, &current_mode));
    
    ESP_ERROR_CHECK(bno055_set_operation_mode(handle, BNO055_OPERATION_MODE_CONFIG));
    vTaskDelay(pdMS_TO_TICKS(30));
    
    // Trigger self-test
    ESP_ERROR_CHECK(bno055_write_byte(handle, BNO055_REG_SYS_TRIGGER, 0x01));
    
    // Wait for self-test to complete
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Read self-test result
    ESP_ERROR_CHECK(bno055_read_byte(handle, BNO055_REG_SELFTEST_RESULT, result));
    
    // Restore original mode
    ESP_ERROR_CHECK(bno055_set_operation_mode(handle, current_mode));
    vTaskDelay(pdMS_TO_TICKS(30));
    
    // Log results
    bool acc_test = (*result & 0x01) != 0;
    bool mag_test = (*result & 0x02) != 0;
    bool gyro_test = (*result & 0x04) != 0;
    bool mcu_test = (*result & 0x08) != 0;
    
    ESP_LOGI(TAG, "Self-test results: Accel=%s, Mag=%s, Gyro=%s, MCU=%s",
             acc_test ? "PASS" : "FAIL",
             mag_test ? "PASS" : "FAIL", 
             gyro_test ? "PASS" : "FAIL",
             mcu_test ? "PASS" : "FAIL");
    
    if (*result != 0x0F) {
        ESP_LOGW(TAG, "Self-test failed with result: 0x%02x", *result);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    ESP_LOGI(TAG, "Self-test completed successfully");
    return ESP_OK;
}