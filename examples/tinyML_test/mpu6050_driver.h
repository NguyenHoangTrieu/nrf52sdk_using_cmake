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
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40

#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48

// MPU6050 configuration values
#define MPU6050_WHO_AM_I_VALUE      0x68
#define MPU6050_PWR_MGMT_1_CLK_PLL  0x01
#define MPU6050_PWR_MGMT_2_ALL_ON   0x00
#define MPU6050_ACCEL_FS_2G         0x00
#define MPU6050_GYRO_FS_250DPS      0x00
#define MPU6050_DLPF_BW_20HZ        0x04

// Sensitivity values
#define MPU6050_ACCEL_SENSITIVITY_2G    16384.0f
#define MPU6050_GYRO_SENSITIVITY_250DPS 131.0f

/**
 * @brief MPU6050 data structure
 */
typedef struct {
    float accel_x_g;    // Acceleration X in g
    float accel_y_g;    // Acceleration Y in g  
    float accel_z_g;    // Acceleration Z in g
    float gyro_x_dps;   // Gyroscope X in degrees per second
    float gyro_y_dps;   // Gyroscope Y in degrees per second
    float gyro_z_dps;   // Gyroscope Z in degrees per second
} mpu6050_data_t;

/**
 * @brief Initialize MPU6050 sensor
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C device address of MPU6050 (typically 0x68 or 0x69)
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t mpu6050_init(const nrf_drv_twi_t *p_twi, uint8_t device_addr);

/**
 * @brief Read WHO_AM_I register to verify MPU6050 connection
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C device address of MPU6050
 * @param who_am_i Pointer to store WHO_AM_I value
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t mpu6050_verify(const nrf_drv_twi_t *p_twi, uint8_t device_addr, uint8_t *who_am_i);

/**
 * @brief Read accelerometer data only
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C device address of MPU6050
 * @param accel_x Pointer to store X-axis acceleration (in g)
 * @param accel_y Pointer to store Y-axis acceleration (in g)
 * @param accel_z Pointer to store Z-axis acceleration (in g)
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t mpu6050_read_accel(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                              float *accel_x, float *accel_y, float *accel_z);

/**
 * @brief Read gyroscope data only
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C device address of MPU6050
 * @param gyro_x Pointer to store X-axis gyroscope (in dps)
 * @param gyro_y Pointer to store Y-axis gyroscope (in dps)
 * @param gyro_z Pointer to store Z-axis gyroscope (in dps)
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t mpu6050_read_gyro(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                             float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief Read all MPU6050 data (accelerometer + gyroscope)
 * @param p_twi Pointer to TWI instance
 * @param device_addr I2C device address of MPU6050
 * @param data Pointer to mpu6050_data_t structure to store data
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t mpu6050_read_all(const nrf_drv_twi_t *p_twi, uint8_t device_addr, mpu6050_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_DRIVER_H
