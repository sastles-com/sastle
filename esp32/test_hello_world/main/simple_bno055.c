#include "simple_bno055.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SIMPLE_BNO055";

#define I2C_TIMEOUT_MS 1000

/**
 * @brief Read single byte from BNO055
 */
static esp_err_t read_register(simple_bno055_t* bno055, uint8_t reg_addr, uint8_t* data)
{
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bno055->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(bno055->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bno055->dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(bno055->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Write single byte to BNO055
 */
static esp_err_t write_register(simple_bno055_t* bno055, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bno055->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(bno055->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Read multiple bytes from BNO055
 */
static esp_err_t read_registers(simple_bno055_t* bno055, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bno055->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(bno055->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (bno055->dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(bno055->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Initialize simple BNO055
 */
esp_err_t simple_bno055_init(simple_bno055_t* bno055, i2c_port_t i2c_port, uint8_t dev_addr)
{
    if (!bno055) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bno055->i2c_port = i2c_port;
    bno055->dev_addr = dev_addr;
    bno055->initialized = false;
    
    ESP_LOGI(TAG, "Initializing BNO055 on I2C port %d, address 0x%02X", i2c_port, dev_addr);
    
    // Read chip ID
    uint8_t chip_id;
    esp_err_t ret = read_register(bno055, BNO055_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (chip_id != BNO055_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected: 0x%02X)", chip_id, BNO055_ID);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "BNO055 chip ID verified: 0x%02X", chip_id);
    
    // Switch to config mode
    ret = write_register(bno055, BNO055_OPR_MODE, BNO055_OPERATION_MODE_CONFIG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    
    // Reset system
    ret = write_register(bno055, BNO055_SYS_TRIGGER, 0x20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset system: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for reset
    
    // Wait for chip to be ready again
    do {
        ret = read_register(bno055, BNO055_CHIP_ID, &chip_id);
        if (ret == ESP_OK && chip_id == BNO055_ID) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    } while (1);
    
    // Use external crystal
    ret = write_register(bno055, BNO055_SYS_TRIGGER, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable external crystal: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set to NDOF mode
    ret = write_register(bno055, BNO055_OPR_MODE, BNO055_OPERATION_MODE_NDOF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    
    bno055->initialized = true;
    ESP_LOGI(TAG, "BNO055 initialized successfully");
    
    return ESP_OK;
}

/**
 * @brief Read chip ID
 */
esp_err_t simple_bno055_read_chip_id(simple_bno055_t* bno055, uint8_t* chip_id)
{
    if (!bno055 || !chip_id) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return read_register(bno055, BNO055_CHIP_ID, chip_id);
}

/**
 * @brief Set operation mode
 */
esp_err_t simple_bno055_set_mode(simple_bno055_t* bno055, bno055_operation_mode_t mode)
{
    if (!bno055) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = write_register(bno055, BNO055_OPR_MODE, (uint8_t)mode);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));  // Mode switch delay
    }
    
    return ret;
}

/**
 * @brief Read quaternion data
 */
esp_err_t simple_bno055_read_quaternion(simple_bno055_t* bno055, simple_quaternion_t* quaternion)
{
    if (!bno055 || !quaternion || !bno055->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[8];
    esp_err_t ret = read_registers(bno055, BNO055_QUATERNION_DATA_W_LSB, buffer, 8);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw data to quaternion (scale: 1/16384)
    int16_t w = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t x = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t y = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t z = (int16_t)((buffer[7] << 8) | buffer[6]);
    
    quaternion->w = w / 16384.0f;
    quaternion->x = x / 16384.0f;
    quaternion->y = y / 16384.0f;
    quaternion->z = z / 16384.0f;
    
    return ESP_OK;
}

/**
 * @brief Read calibration status
 */
esp_err_t simple_bno055_read_calibration_status(simple_bno055_t* bno055, simple_calibration_status_t* status)
{
    if (!bno055 || !status || !bno055->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t calib_stat;
    esp_err_t ret = read_register(bno055, BNO055_CALIB_STAT, &calib_stat);
    if (ret != ESP_OK) {
        return ret;
    }
    
    status->system = (calib_stat >> 6) & 0x03;
    status->gyroscope = (calib_stat >> 4) & 0x03;
    status->accelerometer = (calib_stat >> 2) & 0x03;
    status->magnetometer = calib_stat & 0x03;
    
    return ESP_OK;
}