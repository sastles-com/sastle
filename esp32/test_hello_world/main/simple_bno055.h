#ifndef SIMPLE_BNO055_H
#define SIMPLE_BNO055_H

#include "esp_err.h"
#include "driver/i2c.h"

// BNO055 I2C addresses
#define BNO055_I2C_ADDR_A    0x28
#define BNO055_I2C_ADDR_B    0x29

// BNO055 register addresses
#define BNO055_CHIP_ID       0x00
#define BNO055_ACCEL_REV_ID  0x01
#define BNO055_MAG_REV_ID    0x02
#define BNO055_GYRO_REV_ID   0x03
#define BNO055_SW_REV_ID_LSB 0x04
#define BNO055_SW_REV_ID_MSB 0x05
#define BNO055_BL_REV_ID     0x06

#define BNO055_PAGE_ID       0x07

// Calibration data
#define BNO055_CALIB_STAT    0x35

// Data output
#define BNO055_QUATERNION_DATA_W_LSB 0x20
#define BNO055_QUATERNION_DATA_W_MSB 0x21
#define BNO055_QUATERNION_DATA_X_LSB 0x22
#define BNO055_QUATERNION_DATA_X_MSB 0x23
#define BNO055_QUATERNION_DATA_Y_LSB 0x24
#define BNO055_QUATERNION_DATA_Y_MSB 0x25
#define BNO055_QUATERNION_DATA_Z_LSB 0x26
#define BNO055_QUATERNION_DATA_Z_MSB 0x27

// Operating mode
#define BNO055_OPR_MODE      0x3D
#define BNO055_PWR_MODE      0x3E
#define BNO055_SYS_TRIGGER   0x3F
#define BNO055_TEMP_SOURCE   0x40

// BNO055 constants
#define BNO055_ID            0xA0

// Operating modes
typedef enum {
    BNO055_OPERATION_MODE_CONFIG     = 0x00,
    BNO055_OPERATION_MODE_ACCONLY    = 0x01,
    BNO055_OPERATION_MODE_MAGONLY    = 0x02,
    BNO055_OPERATION_MODE_GYRONLY    = 0x03,
    BNO055_OPERATION_MODE_ACCMAG     = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO    = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO    = 0x06,
    BNO055_OPERATION_MODE_AMG        = 0x07,
    BNO055_OPERATION_MODE_IMUPLUS    = 0x08,
    BNO055_OPERATION_MODE_COMPASS    = 0x09,
    BNO055_OPERATION_MODE_M4G        = 0x0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPERATION_MODE_NDOF       = 0x0C
} bno055_operation_mode_t;

// Data structures
typedef struct {
    float w, x, y, z;
} simple_quaternion_t;

typedef struct {
    uint8_t system;
    uint8_t gyroscope;
    uint8_t accelerometer;
    uint8_t magnetometer;
} simple_calibration_status_t;

// Simple BNO055 handle
typedef struct {
    i2c_port_t i2c_port;
    uint8_t dev_addr;
    bool initialized;
} simple_bno055_t;

// Function prototypes
esp_err_t simple_bno055_init(simple_bno055_t* bno055, i2c_port_t i2c_port, uint8_t dev_addr);
esp_err_t simple_bno055_read_chip_id(simple_bno055_t* bno055, uint8_t* chip_id);
esp_err_t simple_bno055_set_mode(simple_bno055_t* bno055, bno055_operation_mode_t mode);
esp_err_t simple_bno055_read_quaternion(simple_bno055_t* bno055, simple_quaternion_t* quaternion);
esp_err_t simple_bno055_read_calibration_status(simple_bno055_t* bno055, simple_calibration_status_t* status);

#endif // SIMPLE_BNO055_H