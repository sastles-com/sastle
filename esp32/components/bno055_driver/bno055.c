#include <string.h>
#include <math.h>
#include "bno055.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BNO055";

#define BNO055_I2C_TIMEOUT_MS 1000
#define BNO055_MODE_SWITCH_DELAY_MS 30
#define BNO055_RESET_DELAY_MS 650

// Internal handle structure
struct bno055_handle_s {
    i2c_port_t i2c_port;
    uint8_t device_address;
    bno055_mode_t current_mode;
    bool initialized;
};

// Internal helper functions
static esp_err_t bno055_write_register(bno055_handle_t handle, uint8_t reg, uint8_t data);
static esp_err_t bno055_read_register(bno055_handle_t handle, uint8_t reg, uint8_t* data);
static esp_err_t bno055_read_registers(bno055_handle_t handle, uint8_t reg, uint8_t* data, size_t length);
static int16_t bno055_combine_bytes(uint8_t lsb, uint8_t msb);

esp_err_t bno055_init(const bno055_config_t* config, bno055_handle_t* handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing BNO055...");
    
    // Allocate handle
    *handle = calloc(1, sizeof(struct bno055_handle_s));
    if (!*handle) {
        return ESP_ERR_NO_MEM;
    }
    
    // Set up handle
    (*handle)->i2c_port = config->i2c_port;
    (*handle)->device_address = config->device_address;
    (*handle)->current_mode = BNO055_MODE_CONFIG;
    (*handle)->initialized = false;
    
    // Initialize I2C if not already done
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->clock_speed,
    };
    
    esp_err_t ret = i2c_param_config(config->i2c_port, &i2c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        free(*handle);
        return ret;
    }
    
    ret = i2c_driver_install(config->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        free(*handle);
        return ret;
    }
    
    // Verify communication
    uint8_t chip_id;
    ret = bno055_get_chip_id(*handle, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        free(*handle);
        return ret;
    }
    
    if (chip_id != BNO055_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BNO055_CHIP_ID);
        free(*handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "BNO055 chip ID verified: 0x%02X", chip_id);
    
    // Perform soft reset
    ret = bno055_soft_reset(*handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        free(*handle);
        return ret;
    }
    
    // Set power mode
    ret = bno055_write_register(*handle, BNO055_REG_PWR_MODE, config->power_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power mode");
        free(*handle);
        return ret;
    }
    
    // Configure external crystal if requested
    if (config->external_crystal) {
        ret = bno055_set_external_crystal(*handle, true);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set external crystal");
        }
    }
    
    // Set operating mode
    ret = bno055_set_mode(*handle, config->operating_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operating mode");
        free(*handle);
        return ret;
    }
    
    (*handle)->initialized = true;
    ESP_LOGI(TAG, "BNO055 initialization successful");
    
    return ESP_OK;
}

esp_err_t bno055_deinit(bno055_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->initialized) {
        // Set to config mode before shutdown
        bno055_set_mode(handle, BNO055_MODE_CONFIG);
        
        // Note: We don't uninstall I2C driver as it might be used by other devices
        ESP_LOGI(TAG, "BNO055 deinitialized");
    }
    
    free(handle);
    return ESP_OK;
}

esp_err_t bno055_get_chip_id(bno055_handle_t handle, uint8_t* chip_id) {
    if (!handle || !chip_id) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return bno055_read_register(handle, BNO055_REG_CHIP_ID, chip_id);
}

esp_err_t bno055_set_mode(bno055_handle_t handle, bno055_mode_t mode) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = bno055_write_register(handle, BNO055_REG_OPR_MODE, mode);
    if (ret == ESP_OK) {
        handle->current_mode = mode;
        // Mode switch requires delay
        vTaskDelay(pdMS_TO_TICKS(BNO055_MODE_SWITCH_DELAY_MS));
        ESP_LOGI(TAG, "Mode set to: 0x%02X", mode);
    }
    
    return ret;
}

esp_err_t bno055_get_mode(bno055_handle_t handle, bno055_mode_t* mode) {
    if (!handle || !mode) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mode_reg;
    esp_err_t ret = bno055_read_register(handle, BNO055_REG_OPR_MODE, &mode_reg);
    if (ret == ESP_OK) {
        *mode = (bno055_mode_t)(mode_reg & 0x0F);
    }
    
    return ret;
}

esp_err_t bno055_read_quaternion(bno055_handle_t handle, bno055_quaternion_t* quaternion) {
    if (!handle || !quaternion) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->current_mode == BNO055_MODE_CONFIG) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t buffer[8];
    esp_err_t ret = bno055_read_registers(handle, BNO055_REG_QUA_DATA_W_LSB, buffer, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine bytes and convert to float (scale factor 1/16384)
    int16_t w_raw = bno055_combine_bytes(buffer[0], buffer[1]);
    int16_t x_raw = bno055_combine_bytes(buffer[2], buffer[3]);
    int16_t y_raw = bno055_combine_bytes(buffer[4], buffer[5]);
    int16_t z_raw = bno055_combine_bytes(buffer[6], buffer[7]);
    
    quaternion->w = w_raw / 16384.0f;
    quaternion->x = x_raw / 16384.0f;
    quaternion->y = y_raw / 16384.0f;
    quaternion->z = z_raw / 16384.0f;
    
    return ESP_OK;
}

esp_err_t bno055_read_euler(bno055_handle_t handle, bno055_euler_t* euler) {
    if (!handle || !euler) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->current_mode == BNO055_MODE_CONFIG) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_registers(handle, BNO055_REG_EUL_DATA_X_LSB, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine bytes and convert to degrees (scale factor 1/16)
    int16_t heading_raw = bno055_combine_bytes(buffer[0], buffer[1]);
    int16_t roll_raw = bno055_combine_bytes(buffer[2], buffer[3]);
    int16_t pitch_raw = bno055_combine_bytes(buffer[4], buffer[5]);
    
    euler->heading = heading_raw / 16.0f;
    euler->roll = roll_raw / 16.0f;
    euler->pitch = pitch_raw / 16.0f;
    
    return ESP_OK;
}

esp_err_t bno055_read_accelerometer(bno055_handle_t handle, bno055_vector3_t* accel) {
    if (!handle || !accel) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_registers(handle, BNO055_REG_ACC_DATA_X_LSB, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine bytes and convert to m/s² (scale factor 1/100)
    int16_t x_raw = bno055_combine_bytes(buffer[0], buffer[1]);
    int16_t y_raw = bno055_combine_bytes(buffer[2], buffer[3]);
    int16_t z_raw = bno055_combine_bytes(buffer[4], buffer[5]);
    
    accel->x = x_raw / 100.0f;
    accel->y = y_raw / 100.0f;
    accel->z = z_raw / 100.0f;
    
    return ESP_OK;
}

esp_err_t bno055_read_magnetometer(bno055_handle_t handle, bno055_vector3_t* mag) {
    if (!handle || !mag) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_registers(handle, BNO055_REG_MAG_DATA_X_LSB, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine bytes and convert to µT (scale factor 1/16)
    int16_t x_raw = bno055_combine_bytes(buffer[0], buffer[1]);
    int16_t y_raw = bno055_combine_bytes(buffer[2], buffer[3]);
    int16_t z_raw = bno055_combine_bytes(buffer[4], buffer[5]);
    
    mag->x = x_raw / 16.0f;
    mag->y = y_raw / 16.0f;
    mag->z = z_raw / 16.0f;
    
    return ESP_OK;
}

esp_err_t bno055_read_gyroscope(bno055_handle_t handle, bno055_vector3_t* gyro) {
    if (!handle || !gyro) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_registers(handle, BNO055_REG_GYR_DATA_X_LSB, buffer, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine bytes and convert to °/s (scale factor 1/16)
    int16_t x_raw = bno055_combine_bytes(buffer[0], buffer[1]);
    int16_t y_raw = bno055_combine_bytes(buffer[2], buffer[3]);
    int16_t z_raw = bno055_combine_bytes(buffer[4], buffer[5]);
    
    gyro->x = x_raw / 16.0f;
    gyro->y = y_raw / 16.0f;
    gyro->z = z_raw / 16.0f;
    
    return ESP_OK;
}

esp_err_t bno055_get_calibration_status(bno055_handle_t handle, bno055_calibration_status_t* status) {
    if (!handle || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t calib_data;
    esp_err_t ret = bno055_read_register(handle, BNO055_REG_CALIB_STAT, &calib_data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    status->system = (calib_data >> 6) & 0x03;
    status->gyroscope = (calib_data >> 4) & 0x03;
    status->accelerometer = (calib_data >> 2) & 0x03;
    status->magnetometer = calib_data & 0x03;
    
    return ESP_OK;
}

esp_err_t bno055_get_system_status(bno055_handle_t handle, bno055_system_status_t* status) {
    if (!handle || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = bno055_read_register(handle, BNO055_REG_SYS_STAT, &status->system_status);
    if (ret != ESP_OK) return ret;
    
    ret = bno055_read_register(handle, BNO055_REG_SYS_ERR, &status->system_error);
    if (ret != ESP_OK) return ret;
    
    ret = bno055_read_register(handle, BNO055_REG_ST_RESULT, &status->self_test_result);
    return ret;
}

esp_err_t bno055_read_temperature(bno055_handle_t handle, float* temp_celsius) {
    if (!handle || !temp_celsius) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t temp_data;
    esp_err_t ret = bno055_read_register(handle, BNO055_REG_TEMP, &temp_data);
    if (ret == ESP_OK) {
        *temp_celsius = (int8_t)temp_data; // Temperature is signed
    }
    
    return ret;
}

esp_err_t bno055_soft_reset(bno055_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Performing soft reset...");
    
    esp_err_t ret = bno055_write_register(handle, BNO055_REG_SYS_TRIGGER, 0x20);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BNO055_RESET_DELAY_MS));
        handle->current_mode = BNO055_MODE_CONFIG;
        ESP_LOGI(TAG, "Soft reset completed");
    }
    
    return ret;
}

esp_err_t bno055_is_calibrated(bno055_handle_t handle, bool* is_calibrated) {
    if (!handle || !is_calibrated) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bno055_calibration_status_t status;
    esp_err_t ret = bno055_get_calibration_status(handle, &status);
    if (ret == ESP_OK) {
        // Consider calibrated if all sensors are at level 3
        *is_calibrated = (status.system == 3 && 
                         status.accelerometer == 3 && 
                         status.magnetometer == 3 && 
                         status.gyroscope == 3);
    }
    
    return ret;
}

esp_err_t bno055_set_external_crystal(bno055_handle_t handle, bool use_external) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Must be in config mode to change crystal setting
    bno055_mode_t original_mode = handle->current_mode;
    if (original_mode != BNO055_MODE_CONFIG) {
        esp_err_t ret = bno055_set_mode(handle, BNO055_MODE_CONFIG);
        if (ret != ESP_OK) return ret;
    }
    
    uint8_t trigger_value = use_external ? 0x80 : 0x00;
    esp_err_t ret = bno055_write_register(handle, BNO055_REG_SYS_TRIGGER, trigger_value);
    
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for crystal switch
        ESP_LOGI(TAG, "External crystal %s", use_external ? "enabled" : "disabled");
    }
    
    // Restore original mode
    if (original_mode != BNO055_MODE_CONFIG) {
        ret = bno055_set_mode(handle, original_mode);
    }
    
    return ret;
}

// Internal helper functions

static esp_err_t bno055_write_register(bno055_handle_t handle, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(BNO055_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t bno055_read_register(bno055_handle_t handle, uint8_t reg, uint8_t* data) {
    return bno055_read_registers(handle, reg, data, 1);
}

static esp_err_t bno055_read_registers(bno055_handle_t handle, uint8_t reg, uint8_t* data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->device_address << 1) | I2C_MASTER_READ, true);
    
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(BNO055_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static int16_t bno055_combine_bytes(uint8_t lsb, uint8_t msb) {
    return (int16_t)((msb << 8) | lsb);
}