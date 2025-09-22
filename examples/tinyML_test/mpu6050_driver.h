/**
 * @file mpu6050_driver.h
 * @brief MPU6050 accelerometer/gyroscope driver for nRF52840 via I2C
 */

#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <stdint.h>
#include "nrf_drv_twi.h"
#include "sdk_errors.h"

#ifdef __cplusplus
extern "C" {
#endif

// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_PWR_MGMT_2     0x6C
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_WHO_AM_I       0x75
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_ACCEL_XOUT_L   0x3C
#define MPU6050_REG_ACCEL_YOUT_H   0x3D
#define MPU6050_REG_ACCEL_YOUT_L   0x3E
#define MPU6050_REG_ACCEL_ZOUT_H   0x3F
#define MPU6050_REG_ACCEL_ZOUT_L   0x40
#define MPU6050_REG_GYRO_XOUT_H    0x43
#define MPU6050_REG_GYRO_XOUT_L    0x44
#define MPU6050_REG_GYRO_YOUT_H    0x45
#define MPU6050_REG_GYRO_YOUT_L    0x46
#define MPU6050_REG_GYRO_ZOUT_H    0x47
#define MPU6050_REG_GYRO_ZOUT_L    0x48

// MPU6050 configuration values
#define MPU6050_WHO_AM_I_VALUE     0x68
#define MPU6050_PWR_MGMT_1_CLK_PLL 0x01
#define MPU6050_PWR_MGMT_2_ALL_ON  0x00
#define MPU6050_ACCEL_FS_2G        0x00
#define MPU6050_GYRO_FS_250DPS     0x00
#define MPU6050_DLPF_BW_20HZ       0x04

// Sensitivity values
#define MPU6050_ACCEL_SENSITIVITY_2G    16384.0f
#define MPU6050_GYRO_SENSITIVITY_250DPS 131.0f

#ifndef M_PI
#define M_PI 3.1415926
#endif

/**
 * @brief MPU6050 data structure
 */
typedef struct {
    float accel_x_g;   // Acceleration X in g
    float accel_y_g;   // Acceleration Y in g
    float accel_z_g;   // Acceleration Z in g
    float gyro_x_dps;  // Gyroscope X in degrees per second
    float gyro_y_dps;  // Gyroscope Y in degrees per second
    float gyro_z_dps;  // Gyroscope Z in degrees per second
} mpu6050_data_t;

// Fake data patterns for testing
typedef enum {
    MPU6050_FAKE_PATTERN_STATIC,  // Static values
    MPU6050_FAKE_PATTERN_SINE,    // Sine wave
    MPU6050_FAKE_PATTERN_SHAKE,   // Shake motion
    MPU6050_FAKE_PATTERN_TILT,    // Tilt motion
    MPU6050_FAKE_PATTERN_CIRCLE,  // Circle motion
    MPU6050_FAKE_PATTERN_COUNT
} mpu6050_fake_pattern_t;

/**
 * @brief Initialize MPU6050 sensor
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C address (0x68 or 0x69)
 * @return NRF_SUCCESS or error code
 */
ret_code_t mpu6050_init(const nrf_drv_twi_t *p_twi, uint8_t device_addr);

/**
 * @brief Verify MPU6050 connection
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C address
 * @param who_am_i Pointer to store value
 * @return NRF_SUCCESS or error code
 */
ret_code_t mpu6050_verify(const nrf_drv_twi_t *p_twi, uint8_t device_addr, uint8_t *who_am_i);

/**
 * @brief Read accelerometer data
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C address
 * @param accel_x X-axis (g)
 * @param accel_y Y-axis (g)
 * @param accel_z Z-axis (g)
 * @return NRF_SUCCESS or error code
 */
ret_code_t mpu6050_read_accel(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                              float *accel_x, float *accel_y, float *accel_z);

/**
 * @brief Read gyroscope data
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C address
 * @param gyro_x X-axis (dps)
 * @param gyro_y Y-axis (dps)
 * @param gyro_z Z-axis (dps)
 * @return NRF_SUCCESS or error code
 */
ret_code_t mpu6050_read_gyro(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                             float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief Read all data (accel + gyro)
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C address
 * @param data Pointer to data struct
 * @return NRF_SUCCESS or error code
 */
ret_code_t mpu6050_read_all(const nrf_drv_twi_t *p_twi, uint8_t device_addr, mpu6050_data_t *data);

/**
 * @brief Enable fake data mode
 * @param pattern Pattern type
 * @return NRF_SUCCESS or error code
 */
ret_code_t mpu6050_enable_fake_data(mpu6050_fake_pattern_t pattern);

/**
 * @brief Disable fake data mode
 * @return NRF_SUCCESS
 */
ret_code_t mpu6050_disable_fake_data(void);

/**
 * @brief Check if fake data enabled
 * @return true if enabled
 */
bool mpu6050_is_fake_data_enabled(void);

/**
 * @brief Set custom fake data values
 * @param accel_x X accel (g)
 * @param accel_y Y accel (g)
 * @param accel_z Z accel (g)
 * @param gyro_x X gyro (dps)
 * @param gyro_y Y gyro (dps)
 * @param gyro_z Z gyro (dps)
 * @return NRF_SUCCESS
 */
ret_code_t mpu6050_set_custom_fake_data(float accel_x, float accel_y, float accel_z,
                                        float gyro_x, float gyro_y, float gyro_z);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_DRIVER_H
