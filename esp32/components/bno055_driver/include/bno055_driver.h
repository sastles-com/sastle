/**
 * @file bno055_driver.h
 * @brief BNO055 IMU Sensor Driver Header
 * @author Isolation Sphere Team
 * @date 2024
 */

#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

// Forward declarations
typedef struct quaternion_filter_stats_s quaternion_filter_stats_t;

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * BNO055 Register Definitions
 * =============================================================================*/

// I2C addresses
#define BNO055_I2C_ADDR_A               0x28    // ADR pin = LOW
#define BNO055_I2C_ADDR_B               0x29    // ADR pin = HIGH

// Page 0 registers
#define BNO055_REG_CHIP_ID              0x00
#define BNO055_REG_ACC_ID               0x01
#define BNO055_REG_MAG_ID               0x02
#define BNO055_REG_GYRO_ID              0x03
#define BNO055_REG_SW_REV_ID_LSB        0x04
#define BNO055_REG_SW_REV_ID_MSB        0x05
#define BNO055_REG_BL_REV_ID            0x06

// Data registers
#define BNO055_REG_ACC_DATA_X_LSB       0x08
#define BNO055_REG_ACC_DATA_X_MSB       0x09
#define BNO055_REG_ACC_DATA_Y_LSB       0x0A
#define BNO055_REG_ACC_DATA_Y_MSB       0x0B
#define BNO055_REG_ACC_DATA_Z_LSB       0x0C
#define BNO055_REG_ACC_DATA_Z_MSB       0x0D

#define BNO055_REG_MAG_DATA_X_LSB       0x0E
#define BNO055_REG_MAG_DATA_X_MSB       0x0F
#define BNO055_REG_MAG_DATA_Y_LSB       0x10
#define BNO055_REG_MAG_DATA_Y_MSB       0x11
#define BNO055_REG_MAG_DATA_Z_LSB       0x12
#define BNO055_REG_MAG_DATA_Z_MSB       0x13

#define BNO055_REG_GYRO_DATA_X_LSB      0x14
#define BNO055_REG_GYRO_DATA_X_MSB      0x15
#define BNO055_REG_GYRO_DATA_Y_LSB      0x16
#define BNO055_REG_GYRO_DATA_Y_MSB      0x17
#define BNO055_REG_GYRO_DATA_Z_LSB      0x18
#define BNO055_REG_GYRO_DATA_Z_MSB      0x19

// Euler angles
#define BNO055_REG_EULER_H_LSB          0x1A
#define BNO055_REG_EULER_H_MSB          0x1B
#define BNO055_REG_EULER_R_LSB          0x1C
#define BNO055_REG_EULER_R_MSB          0x1D
#define BNO055_REG_EULER_P_LSB          0x1E
#define BNO055_REG_EULER_P_MSB          0x1F

// Quaternion data
#define BNO055_REG_QUATERNION_DATA_W_LSB 0x20
#define BNO055_REG_QUATERNION_DATA_W_MSB 0x21
#define BNO055_REG_QUATERNION_DATA_X_LSB 0x22
#define BNO055_REG_QUATERNION_DATA_X_MSB 0x23
#define BNO055_REG_QUATERNION_DATA_Y_LSB 0x24
#define BNO055_REG_QUATERNION_DATA_Y_MSB 0x25
#define BNO055_REG_QUATERNION_DATA_Z_LSB 0x26
#define BNO055_REG_QUATERNION_DATA_Z_MSB 0x27

// Linear acceleration data
#define BNO055_REG_LINEAR_ACCEL_DATA_X_LSB 0x28
#define BNO055_REG_LINEAR_ACCEL_DATA_X_MSB 0x29
#define BNO055_REG_LINEAR_ACCEL_DATA_Y_LSB 0x2A
#define BNO055_REG_LINEAR_ACCEL_DATA_Y_MSB 0x2B
#define BNO055_REG_LINEAR_ACCEL_DATA_Z_LSB 0x2C
#define BNO055_REG_LINEAR_ACCEL_DATA_Z_MSB 0x2D

// Gravity data
#define BNO055_REG_GRAVITY_DATA_X_LSB   0x2E
#define BNO055_REG_GRAVITY_DATA_X_MSB   0x2F
#define BNO055_REG_GRAVITY_DATA_Y_LSB   0x30
#define BNO055_REG_GRAVITY_DATA_Y_MSB   0x31
#define BNO055_REG_GRAVITY_DATA_Z_LSB   0x32
#define BNO055_REG_GRAVITY_DATA_Z_MSB   0x33

// Temperature
#define BNO055_REG_TEMP                 0x34

// Calibration status
#define BNO055_REG_CALIB_STAT           0x35
#define BNO055_REG_SELFTEST_RESULT      0x36
#define BNO055_REG_INTR_STAT            0x37
#define BNO055_REG_SYS_CLK_STAT         0x38
#define BNO055_REG_SYS_STAT             0x39
#define BNO055_REG_SYS_ERR              0x3A

// Unit selection
#define BNO055_REG_UNIT_SEL             0x3B
#define BNO055_REG_DATA_SELECT          0x3C

// Operating mode
#define BNO055_REG_OPR_MODE             0x3D
#define BNO055_REG_PWR_MODE             0x3E

#define BNO055_REG_SYS_TRIGGER          0x3F
#define BNO055_REG_TEMP_SOURCE          0x40

// Axis remap
#define BNO055_REG_AXIS_MAP_CONFIG      0x41
#define BNO055_REG_AXIS_MAP_SIGN        0x42

// SIC registers
#define BNO055_REG_SIC_MATRIX_0_LSB     0x43
#define BNO055_REG_SIC_MATRIX_0_MSB     0x44

// Accelerometer Offset registers
#define BNO055_REG_ACCEL_OFFSET_X_LSB   0x55
#define BNO055_REG_ACCEL_OFFSET_X_MSB   0x56
#define BNO055_REG_ACCEL_OFFSET_Y_LSB   0x57
#define BNO055_REG_ACCEL_OFFSET_Y_MSB   0x58
#define BNO055_REG_ACCEL_OFFSET_Z_LSB   0x59
#define BNO055_REG_ACCEL_OFFSET_Z_MSB   0x5A

// Magnetometer Offset registers
#define BNO055_REG_MAG_OFFSET_X_LSB     0x5B
#define BNO055_REG_MAG_OFFSET_X_MSB     0x5C
#define BNO055_REG_MAG_OFFSET_Y_LSB     0x5D
#define BNO055_REG_MAG_OFFSET_Y_MSB     0x5E
#define BNO055_REG_MAG_OFFSET_Z_LSB     0x5F
#define BNO055_REG_MAG_OFFSET_Z_MSB     0x60

// Gyroscope Offset registers
#define BNO055_REG_GYRO_OFFSET_X_LSB    0x61
#define BNO055_REG_GYRO_OFFSET_X_MSB    0x62
#define BNO055_REG_GYRO_OFFSET_Y_LSB    0x63
#define BNO055_REG_GYRO_OFFSET_Y_MSB    0x64
#define BNO055_REG_GYRO_OFFSET_Z_LSB    0x65
#define BNO055_REG_GYRO_OFFSET_Z_MSB    0x66

// Radius registers
#define BNO055_REG_ACCEL_RADIUS_LSB     0x67
#define BNO055_REG_ACCEL_RADIUS_MSB     0x68
#define BNO055_REG_MAG_RADIUS_LSB       0x69
#define BNO055_REG_MAG_RADIUS_MSB       0x6A

/* =============================================================================
 * BNO055 Constants
 * =============================================================================*/

// Chip IDs
#define BNO055_CHIP_ID                  0xA0
#define BNO055_ACC_ID                   0xFB
#define BNO055_MAG_ID                   0x32
#define BNO055_GYRO_ID                  0x0F

// Operation modes
typedef enum {
    BNO055_OPERATION_MODE_CONFIG        = 0x00,
    BNO055_OPERATION_MODE_ACCONLY       = 0x01,
    BNO055_OPERATION_MODE_MAGONLY       = 0x02,
    BNO055_OPERATION_MODE_GYRONLY       = 0x03,
    BNO055_OPERATION_MODE_ACCMAG        = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO       = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO       = 0x06,
    BNO055_OPERATION_MODE_AMG           = 0x07,
    BNO055_OPERATION_MODE_IMUPLUS       = 0x08,
    BNO055_OPERATION_MODE_COMPASS       = 0x09,
    BNO055_OPERATION_MODE_M4G           = 0x0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF  = 0x0B,
    BNO055_OPERATION_MODE_NDOF          = 0x0C
} bno055_opmode_t;

// Power modes
typedef enum {
    BNO055_POWER_MODE_NORMAL            = 0x00,
    BNO055_POWER_MODE_LOWPOWER          = 0x01,
    BNO055_POWER_MODE_SUSPEND           = 0x02
} bno055_powermode_t;

/* =============================================================================
 * Data Structures
 * =============================================================================*/

/**
 * @brief Vector3 structure for 3D data
 */
typedef struct {
    float x, y, z;
} bno055_vector_t;

/**
 * @brief Quaternion structure
 */
typedef struct {
    float w, x, y, z;
} bno055_quaternion_t;

/**
 * @brief Calibration status structure
 */
typedef struct {
    uint8_t system;         // 0-3
    uint8_t gyroscope;      // 0-3  
    uint8_t accelerometer;  // 0-3
    uint8_t magnetometer;   // 0-3
} bno055_calibration_t;

/**
 * @brief BNO055 device configuration
 */
typedef struct {
    i2c_port_t i2c_port;           // I2C port number
    uint8_t dev_addr;              // Device I2C address
    gpio_num_t sda_pin;            // SDA pin
    gpio_num_t scl_pin;            // SCL pin
    uint32_t clk_speed;            // I2C clock speed
    bno055_opmode_t op_mode;       // Operation mode
    bool use_external_crystal;     // Use external crystal
    uint8_t temp_source;           // Temperature source
} bno055_config_t;

/**
 * @brief BNO055 device handle
 */
typedef struct {
    bno055_config_t config;
    bool initialized;
    uint32_t last_read_time;
    bno055_calibration_t last_calibration;
    int8_t temperature;
    bool data_ready;
} bno055_handle_t;

/**
 * @brief Complete IMU data structure
 */
typedef struct {
    uint32_t timestamp;
    bno055_quaternion_t quaternion;
    bno055_vector_t accelerometer;
    bno055_vector_t magnetometer;
    bno055_vector_t gyroscope;
    bno055_vector_t euler;
    bno055_vector_t linear_accel;
    bno055_vector_t gravity;
    bno055_calibration_t calibration;
    int8_t temperature;
    bool data_valid;
} bno055_data_t;

/* =============================================================================
 * Function Declarations
 * =============================================================================*/

/**
 * @brief Initialize BNO055 sensor
 * @param handle Pointer to device handle
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_init(bno055_handle_t* handle, const bno055_config_t* config);

/**
 * @brief Deinitialize BNO055 sensor
 * @param handle Pointer to device handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_deinit(bno055_handle_t* handle);

/**
 * @brief Check if BNO055 is ready
 * @param handle Pointer to device handle
 * @return true if ready, false otherwise
 */
bool bno055_is_ready(const bno055_handle_t* handle);

/**
 * @brief Read complete IMU data from BNO055
 * @param handle Pointer to device handle
 * @param data Pointer to data structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_read_data(bno055_handle_t* handle, bno055_data_t* data);

/**
 * @brief Read quaternion data from BNO055
 * @param handle Pointer to device handle
 * @param quaternion Pointer to quaternion structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_read_quaternion(bno055_handle_t* handle, bno055_quaternion_t* quaternion);

/**
 * @brief Read calibration status
 * @param handle Pointer to device handle
 * @param calibration Pointer to calibration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_read_calibration_status(bno055_handle_t* handle, bno055_calibration_t* calibration);

/**
 * @brief Set operation mode
 * @param handle Pointer to device handle
 * @param mode Operation mode
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_set_operation_mode(bno055_handle_t* handle, bno055_opmode_t mode);

/**
 * @brief Get operation mode
 * @param handle Pointer to device handle
 * @param mode Pointer to store current mode
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_get_operation_mode(bno055_handle_t* handle, bno055_opmode_t* mode);

/**
 * @brief Set power mode
 * @param handle Pointer to device handle
 * @param mode Power mode
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_set_power_mode(bno055_handle_t* handle, bno055_powermode_t mode);

/**
 * @brief Reset sensor
 * @param handle Pointer to device handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_reset(bno055_handle_t* handle);

/**
 * @brief Perform self-test
 * @param handle Pointer to device handle
 * @param result Pointer to store test result
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_self_test(bno055_handle_t* handle, uint8_t* result);

/**
 * @brief Get temperature reading
 * @param handle Pointer to device handle
 * @param temperature Pointer to store temperature
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_get_temperature(bno055_handle_t* handle, int8_t* temperature);

/**
 * @brief Save calibration data to NVS
 * @param handle Pointer to device handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_save_calibration_data(bno055_handle_t* handle);

/**
 * @brief Load calibration data from NVS
 * @param handle Pointer to device handle
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_load_calibration_data(bno055_handle_t* handle);

/**
 * @brief Check if sensor is fully calibrated
 * @param calibration Pointer to calibration structure
 * @return true if fully calibrated, false otherwise
 */
bool bno055_is_fully_calibrated(const bno055_calibration_t* calibration);

/**
 * @brief Get default configuration
 * @param config Pointer to configuration structure
 */
void bno055_get_default_config(bno055_config_t* config);

/**
 * @brief Read single byte from BNO055 register
 * @param handle Pointer to device handle
 * @param reg Register address
 * @param data Pointer to store read data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_read_byte(bno055_handle_t* handle, uint8_t reg, uint8_t* data);

/**
 * @brief Write single byte to BNO055 register
 * @param handle Pointer to device handle
 * @param reg Register address
 * @param data Data to write
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_write_byte(bno055_handle_t* handle, uint8_t reg, uint8_t data);

/**
 * @brief Read multiple bytes from BNO055 registers
 * @param handle Pointer to device handle
 * @param reg Starting register address
 * @param data Pointer to store read data
 * @param len Number of bytes to read
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_read_bytes(bno055_handle_t* handle, uint8_t reg, uint8_t* data, size_t len);

/**
 * @brief Clear calibration data from NVS
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_clear_calibration_data(void);

/* =============================================================================
 * High-Level Driver API Functions
 * =============================================================================*/

/**
 * @brief Initialize BNO055 driver with default configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_init(void);

/**
 * @brief Initialize BNO055 driver with custom configuration
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_init_with_config(const bno055_config_t* config);

/**
 * @brief Start BNO055 data processing task
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_start_task(void);

/**
 * @brief Stop BNO055 data processing task
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_stop_task(void);

/**
 * @brief Deinitialize BNO055 driver
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_deinit(void);

/**
 * @brief Get current IMU data
 * @param data Pointer to data structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_get_data(bno055_data_t* data);

/**
 * @brief Get current quaternion with filtering
 * @param quaternion Pointer to quaternion structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_get_quaternion(bno055_quaternion_t* quaternion);

/**
 * @brief Enable/disable quaternion filtering
 * @param enabled Filter enable/disable flag
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_set_filter_enabled(bool enabled);

/**
 * @brief Get filter statistics
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_get_filter_stats(quaternion_filter_stats_t* stats);

/**
 * @brief Check if driver is ready
 * @return true if ready, false otherwise
 */
bool bno055_driver_is_ready(void);

/**
 * @brief Get calibration status
 * @param calibration Pointer to calibration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_get_calibration_status(bno055_calibration_t* calibration);

/**
 * @brief Force save calibration data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_save_calibration(void);

/**
 * @brief Reset sensor
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_reset(void);

/**
 * @brief Perform self-test
 * @param result Pointer to store test result
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bno055_driver_self_test(uint8_t* result);

#ifdef __cplusplus
}
#endif

#endif // BNO055_DRIVER_H