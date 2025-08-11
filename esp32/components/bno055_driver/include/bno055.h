#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// BNO055 I2C Addresses
#define BNO055_I2C_ADDR_PRIMARY   0x28  // COM3 = LOW
#define BNO055_I2C_ADDR_SECONDARY 0x29  // COM3 = HIGH

// BNO055 Register Map
#define BNO055_REG_CHIP_ID        0x00  // Should read 0xA0
#define BNO055_REG_ACC_ID         0x01
#define BNO055_REG_MAG_ID         0x02
#define BNO055_REG_GYR_ID         0x03
#define BNO055_REG_SW_REV_ID_LSB  0x04
#define BNO055_REG_SW_REV_ID_MSB  0x05
#define BNO055_REG_BL_REV_ID      0x06
#define BNO055_REG_PAGE_ID        0x07
#define BNO055_REG_ACC_DATA_X_LSB 0x08
#define BNO055_REG_ACC_DATA_X_MSB 0x09
#define BNO055_REG_ACC_DATA_Y_LSB 0x0A
#define BNO055_REG_ACC_DATA_Y_MSB 0x0B
#define BNO055_REG_ACC_DATA_Z_LSB 0x0C
#define BNO055_REG_ACC_DATA_Z_MSB 0x0D
#define BNO055_REG_MAG_DATA_X_LSB 0x0E
#define BNO055_REG_MAG_DATA_X_MSB 0x0F
#define BNO055_REG_MAG_DATA_Y_LSB 0x10
#define BNO055_REG_MAG_DATA_Y_MSB 0x11
#define BNO055_REG_MAG_DATA_Z_LSB 0x12
#define BNO055_REG_MAG_DATA_Z_MSB 0x13
#define BNO055_REG_GYR_DATA_X_LSB 0x14
#define BNO055_REG_GYR_DATA_X_MSB 0x15
#define BNO055_REG_GYR_DATA_Y_LSB 0x16
#define BNO055_REG_GYR_DATA_Y_MSB 0x17
#define BNO055_REG_GYR_DATA_Z_LSB 0x18
#define BNO055_REG_GYR_DATA_Z_MSB 0x19
#define BNO055_REG_EUL_DATA_X_LSB 0x1A
#define BNO055_REG_EUL_DATA_X_MSB 0x1B
#define BNO055_REG_EUL_DATA_Y_LSB 0x1C
#define BNO055_REG_EUL_DATA_Y_MSB 0x1D
#define BNO055_REG_EUL_DATA_Z_LSB 0x1E
#define BNO055_REG_EUL_DATA_Z_MSB 0x1F
#define BNO055_REG_QUA_DATA_W_LSB 0x20
#define BNO055_REG_QUA_DATA_W_MSB 0x21
#define BNO055_REG_QUA_DATA_X_LSB 0x22
#define BNO055_REG_QUA_DATA_X_MSB 0x23
#define BNO055_REG_QUA_DATA_Y_LSB 0x24
#define BNO055_REG_QUA_DATA_Y_MSB 0x25
#define BNO055_REG_QUA_DATA_Z_LSB 0x26
#define BNO055_REG_QUA_DATA_Z_MSB 0x27
#define BNO055_REG_LIA_DATA_X_LSB 0x28
#define BNO055_REG_LIA_DATA_X_MSB 0x29
#define BNO055_REG_LIA_DATA_Y_LSB 0x2A
#define BNO055_REG_LIA_DATA_Y_MSB 0x2B
#define BNO055_REG_LIA_DATA_Z_LSB 0x2C
#define BNO055_REG_LIA_DATA_Z_MSB 0x2D
#define BNO055_REG_GRV_DATA_X_LSB 0x2E
#define BNO055_REG_GRV_DATA_X_MSB 0x2F
#define BNO055_REG_GRV_DATA_Y_LSB 0x30
#define BNO055_REG_GRV_DATA_Y_MSB 0x31
#define BNO055_REG_GRV_DATA_Z_LSB 0x32
#define BNO055_REG_GRV_DATA_Z_MSB 0x33
#define BNO055_REG_TEMP           0x34
#define BNO055_REG_CALIB_STAT     0x35
#define BNO055_REG_ST_RESULT      0x36
#define BNO055_REG_INT_STA        0x37
#define BNO055_REG_SYS_CLK_STAT   0x38
#define BNO055_REG_SYS_STAT       0x39
#define BNO055_REG_SYS_ERR        0x3A
#define BNO055_REG_UNIT_SEL       0x3B
#define BNO055_REG_OPR_MODE       0x3D
#define BNO055_REG_PWR_MODE       0x3E
#define BNO055_REG_SYS_TRIGGER    0x3F
#define BNO055_REG_TEMP_SOURCE    0x40

// BNO055 Expected Chip IDs
#define BNO055_CHIP_ID    0xA0
#define BNO055_ACC_ID     0xFB
#define BNO055_MAG_ID     0x32
#define BNO055_GYR_ID     0x0F

// Operating Modes
typedef enum {
    BNO055_MODE_CONFIG = 0x00,
    BNO055_MODE_ACCONLY = 0x01,
    BNO055_MODE_MAGONLY = 0x02,
    BNO055_MODE_GYRONLY = 0x03,
    BNO055_MODE_ACCMAG = 0x04,
    BNO055_MODE_ACCGYRO = 0x05,
    BNO055_MODE_MAGGYRO = 0x06,
    BNO055_MODE_AMG = 0x07,
    BNO055_MODE_IMUPLUS = 0x08,
    BNO055_MODE_COMPASS = 0x09,
    BNO055_MODE_M4G = 0x0A,
    BNO055_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_MODE_NDOF = 0x0C
} bno055_mode_t;

// Power Modes
typedef enum {
    BNO055_POWER_MODE_NORMAL = 0x00,
    BNO055_POWER_MODE_LOW = 0x01,
    BNO055_POWER_MODE_SUSPEND = 0x02
} bno055_power_mode_t;

// Data structures
typedef struct {
    float w, x, y, z;
} bno055_quaternion_t;

typedef struct {
    float x, y, z;
} bno055_vector3_t;

typedef struct {
    float heading, roll, pitch;  // in degrees
} bno055_euler_t;

typedef struct {
    uint8_t system;
    uint8_t accelerometer;
    uint8_t magnetometer;
    uint8_t gyroscope;
} bno055_calibration_status_t;

typedef struct {
    uint8_t system_status;
    uint8_t system_error;
    uint8_t self_test_result;
} bno055_system_status_t;

// Configuration structure
typedef struct {
    i2c_port_t i2c_port;
    uint8_t device_address;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t clock_speed;
    bno055_mode_t operating_mode;
    bno055_power_mode_t power_mode;
    bool external_crystal;
} bno055_config_t;

// BNO055 handle
typedef struct bno055_handle_s* bno055_handle_t;

// Function prototypes

/**
 * @brief Initialize BNO055 sensor
 * @param config Configuration structure
 * @param handle Pointer to handle
 * @return ESP_OK on success
 */
esp_err_t bno055_init(const bno055_config_t* config, bno055_handle_t* handle);

/**
 * @brief Deinitialize BNO055 sensor
 * @param handle BNO055 handle
 * @return ESP_OK on success
 */
esp_err_t bno055_deinit(bno055_handle_t handle);

/**
 * @brief Read chip ID to verify communication
 * @param handle BNO055 handle
 * @param chip_id Pointer to store chip ID
 * @return ESP_OK on success
 */
esp_err_t bno055_get_chip_id(bno055_handle_t handle, uint8_t* chip_id);

/**
 * @brief Set operating mode
 * @param handle BNO055 handle
 * @param mode Operating mode
 * @return ESP_OK on success
 */
esp_err_t bno055_set_mode(bno055_handle_t handle, bno055_mode_t mode);

/**
 * @brief Get current operating mode
 * @param handle BNO055 handle
 * @param mode Pointer to store current mode
 * @return ESP_OK on success
 */
esp_err_t bno055_get_mode(bno055_handle_t handle, bno055_mode_t* mode);

/**
 * @brief Read quaternion data
 * @param handle BNO055 handle
 * @param quaternion Pointer to store quaternion data
 * @return ESP_OK on success
 */
esp_err_t bno055_read_quaternion(bno055_handle_t handle, bno055_quaternion_t* quaternion);

/**
 * @brief Read Euler angles
 * @param handle BNO055 handle
 * @param euler Pointer to store Euler angles
 * @return ESP_OK on success
 */
esp_err_t bno055_read_euler(bno055_handle_t handle, bno055_euler_t* euler);

/**
 * @brief Read accelerometer data
 * @param handle BNO055 handle
 * @param accel Pointer to store accelerometer data
 * @return ESP_OK on success
 */
esp_err_t bno055_read_accelerometer(bno055_handle_t handle, bno055_vector3_t* accel);

/**
 * @brief Read magnetometer data
 * @param handle BNO055 handle
 * @param mag Pointer to store magnetometer data
 * @return ESP_OK on success
 */
esp_err_t bno055_read_magnetometer(bno055_handle_t handle, bno055_vector3_t* mag);

/**
 * @brief Read gyroscope data
 * @param handle BNO055 handle
 * @param gyro Pointer to store gyroscope data
 * @return ESP_OK on success
 */
esp_err_t bno055_read_gyroscope(bno055_handle_t handle, bno055_vector3_t* gyro);

/**
 * @brief Get calibration status
 * @param handle BNO055 handle
 * @param status Pointer to store calibration status
 * @return ESP_OK on success
 */
esp_err_t bno055_get_calibration_status(bno055_handle_t handle, bno055_calibration_status_t* status);

/**
 * @brief Get system status
 * @param handle BNO055 handle
 * @param status Pointer to store system status
 * @return ESP_OK on success
 */
esp_err_t bno055_get_system_status(bno055_handle_t handle, bno055_system_status_t* status);

/**
 * @brief Get temperature reading
 * @param handle BNO055 handle
 * @param temp_celsius Pointer to store temperature in Celsius
 * @return ESP_OK on success
 */
esp_err_t bno055_read_temperature(bno055_handle_t handle, float* temp_celsius);

/**
 * @brief Perform soft reset
 * @param handle BNO055 handle
 * @return ESP_OK on success
 */
esp_err_t bno055_soft_reset(bno055_handle_t handle);

/**
 * @brief Check if sensor is calibrated
 * @param handle BNO055 handle
 * @param is_calibrated Pointer to store calibration result
 * @return ESP_OK on success
 */
esp_err_t bno055_is_calibrated(bno055_handle_t handle, bool* is_calibrated);

/**
 * @brief Enable/disable external crystal
 * @param handle BNO055 handle
 * @param use_external True to use external crystal
 * @return ESP_OK on success
 */
esp_err_t bno055_set_external_crystal(bno055_handle_t handle, bool use_external);

#ifdef __cplusplus
}
#endif

#endif // BNO055_H