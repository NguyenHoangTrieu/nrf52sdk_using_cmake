/**
 * @file mpu6050_driver.c
 * @brief MPU6050 accelerometer/gyroscope driver implementation
 */

#include "mpu6050_driver.h"
#include "nrf_delay.h"
#include <stdint.h>

// Internal helper functions
static ret_code_t mpu6050_write_reg(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                                     uint8_t reg_addr, uint8_t value);
static ret_code_t mpu6050_read_reg(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                                   uint8_t reg_addr, uint8_t *value);
static ret_code_t mpu6050_read_regs(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                                    uint8_t reg_addr, uint8_t *data, uint8_t length);

ret_code_t mpu6050_init(const nrf_drv_twi_t *p_twi, uint8_t device_addr)
{
    ret_code_t err_code;
    uint8_t who_am_i;

    // Verify device connection
    err_code = mpu6050_verify(p_twi, device_addr, &who_am_i);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        return NRF_ERROR_NOT_FOUND;
    }

    // Reset device
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_PWR_MGMT_1, 0x80);
    if (err_code != NRF_SUCCESS) return err_code;
    nrf_delay_ms(100);

    // Wake up device and set clock source
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLK_PLL);
    if (err_code != NRF_SUCCESS) return err_code;
    nrf_delay_ms(10);

    // Enable all sensors
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_PWR_MGMT_2, MPU6050_PWR_MGMT_2_ALL_ON);
    if (err_code != NRF_SUCCESS) return err_code;

    // Set sample rate to 100 Hz (1kHz / (9 + 1) = 100 Hz)
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_SMPLRT_DIV, 9);
    if (err_code != NRF_SUCCESS) return err_code;

    // Configure DLPF (Digital Low Pass Filter) to 20 Hz
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_CONFIG, MPU6050_DLPF_BW_20HZ);
    if (err_code != NRF_SUCCESS) return err_code;

    // Configure accelerometer (±2g)
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
    if (err_code != NRF_SUCCESS) return err_code;

    // Configure gyroscope (±250 dps)
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250DPS);
    if (err_code != NRF_SUCCESS) return err_code;

    nrf_delay_ms(10);
    
    return NRF_SUCCESS;
}

ret_code_t mpu6050_verify(const nrf_drv_twi_t *p_twi, uint8_t device_addr, uint8_t *who_am_i)
{
    return mpu6050_read_reg(p_twi, device_addr, MPU6050_REG_WHO_AM_I, who_am_i);
}

ret_code_t mpu6050_read_accel(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                              float *accel_x, float *accel_y, float *accel_z)
{
    ret_code_t err_code;
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;

    // Read 6 bytes starting from ACCEL_XOUT_H
    err_code = mpu6050_read_regs(p_twi, device_addr, MPU6050_REG_ACCEL_XOUT_H, data, 6);
    if (err_code != NRF_SUCCESS) return err_code;

    // Combine high and low bytes
    raw_x = (int16_t)((data[0] << 8) | data[1]);
    raw_y = (int16_t)((data[2] << 8) | data[3]);
    raw_z = (int16_t)((data[4] << 8) | data[5]);

    // Convert to g units
    *accel_x = (float)raw_x / MPU6050_ACCEL_SENSITIVITY_2G;
    *accel_y = (float)raw_y / MPU6050_ACCEL_SENSITIVITY_2G;
    *accel_z = (float)raw_z / MPU6050_ACCEL_SENSITIVITY_2G;

    return NRF_SUCCESS;
}

ret_code_t mpu6050_read_gyro(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                             float *gyro_x, float *gyro_y, float *gyro_z)
{
    ret_code_t err_code;
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;

    // Read 6 bytes starting from GYRO_XOUT_H
    err_code = mpu6050_read_regs(p_twi, device_addr, MPU6050_REG_GYRO_XOUT_H, data, 6);
    if (err_code != NRF_SUCCESS) return err_code;

    // Combine high and low bytes
    raw_x = (int16_t)((data[0] << 8) | data[1]);
    raw_y = (int16_t)((data[2] << 8) | data[3]);
    raw_z = (int16_t)((data[4] << 8) | data[5]);

    // Convert to dps units
    *gyro_x = (float)raw_x / MPU6050_GYRO_SENSITIVITY_250DPS;
    *gyro_y = (float)raw_y / MPU6050_GYRO_SENSITIVITY_250DPS;
    *gyro_z = (float)raw_z / MPU6050_GYRO_SENSITIVITY_250DPS;

    return NRF_SUCCESS;
}

ret_code_t mpu6050_read_all(const nrf_drv_twi_t *p_twi, uint8_t device_addr, mpu6050_data_t *data)
{
    ret_code_t err_code;
    uint8_t raw_data[14];  // 6 accel + 2 temp + 6 gyro
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

    // Read all data starting from ACCEL_XOUT_H (14 bytes total)
    err_code = mpu6050_read_regs(p_twi, device_addr, MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (err_code != NRF_SUCCESS) return err_code;

    // Parse accelerometer data
    raw_accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    raw_accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    raw_accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // Parse gyroscope data (skip temperature data at bytes 6-7)
    raw_gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    raw_gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    raw_gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    // Convert to physical units
    data->accel_x_g = (float)raw_accel_x / MPU6050_ACCEL_SENSITIVITY_2G;
    data->accel_y_g = (float)raw_accel_y / MPU6050_ACCEL_SENSITIVITY_2G;
    data->accel_z_g = (float)raw_accel_z / MPU6050_ACCEL_SENSITIVITY_2G;
    
    data->gyro_x_dps = (float)raw_gyro_x / MPU6050_GYRO_SENSITIVITY_250DPS;
    data->gyro_y_dps = (float)raw_gyro_y / MPU6050_GYRO_SENSITIVITY_250DPS;
    data->gyro_z_dps = (float)raw_gyro_z / MPU6050_GYRO_SENSITIVITY_250DPS;

    return NRF_SUCCESS;
}

// Internal helper functions
static ret_code_t mpu6050_write_reg(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                                     uint8_t reg_addr, uint8_t value)
{
    uint8_t data[2] = {reg_addr, value};
    return nrf_drv_twi_tx(p_twi, device_addr, data, sizeof(data), false);
}

static ret_code_t mpu6050_read_reg(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                                   uint8_t reg_addr, uint8_t *value)
{
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_tx(p_twi, device_addr, &reg_addr, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
    
    return nrf_drv_twi_rx(p_twi, device_addr, value, 1);
}

static ret_code_t mpu6050_read_regs(const nrf_drv_twi_t *p_twi, uint8_t device_addr, 
                                    uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    ret_code_t err_code;
    
    err_code = nrf_drv_twi_tx(p_twi, device_addr, &reg_addr, 1, true);
    if (err_code != NRF_SUCCESS) return err_code;
    
    return nrf_drv_twi_rx(p_twi, device_addr, data, length);
}
