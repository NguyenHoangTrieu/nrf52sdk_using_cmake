/**
 * @file mpu6050_driver.c
 * @brief MPU6050 driver implementation
 */

#include "mpu6050_driver.h"
#include "nrf_delay.h"
#include <math.h>

// Internal helper functions
static ret_code_t mpu6050_write_reg(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                                    uint8_t reg_addr, uint8_t value);
static ret_code_t mpu6050_read_reg(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                                   uint8_t reg_addr, uint8_t *value);
static ret_code_t mpu6050_read_regs(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                                    uint8_t reg_addr, uint8_t *data, uint8_t length);

// Fake data state
static bool m_fake_data_enabled = false;
static mpu6050_fake_pattern_t m_fake_pattern = MPU6050_FAKE_PATTERN_STATIC;
static uint32_t m_fake_data_counter = 0;
static mpu6050_data_t m_custom_fake_data = {0};

// Fake data generators
static void generate_fake_data_static(mpu6050_data_t* data);
static void generate_fake_data_sine(mpu6050_data_t* data);
static void generate_fake_data_shake(mpu6050_data_t* data);
static void generate_fake_data_tilt(mpu6050_data_t* data);
static void generate_fake_data_circle(mpu6050_data_t* data);

ret_code_t mpu6050_init(const nrf_drv_twi_t *p_twi, uint8_t device_addr)
{
    ret_code_t err_code;
    uint8_t who_am_i;

    // Verify device
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

    // Wake up and set clock
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLK_PLL);
    if (err_code != NRF_SUCCESS) return err_code;
    nrf_delay_ms(10);

    // Enable sensors
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_PWR_MGMT_2, MPU6050_PWR_MGMT_2_ALL_ON);
    if (err_code != NRF_SUCCESS) return err_code;

    // Set sample rate 100Hz
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_SMPLRT_DIV, 9);
    if (err_code != NRF_SUCCESS) return err_code;

    // DLPF 20Hz
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_CONFIG, MPU6050_DLPF_BW_20HZ);
    if (err_code != NRF_SUCCESS) return err_code;

    // Accel ±2g
    err_code = mpu6050_write_reg(p_twi, device_addr, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
    if (err_code != NRF_SUCCESS) return err_code;

    // Gyro ±250dps
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
    // Check fake mode
    if (m_fake_data_enabled) {
        mpu6050_data_t fake_data;
        switch (m_fake_pattern) {
            case MPU6050_FAKE_PATTERN_STATIC: generate_fake_data_static(&fake_data); break;
            case MPU6050_FAKE_PATTERN_SINE: generate_fake_data_sine(&fake_data); break;
            case MPU6050_FAKE_PATTERN_SHAKE: generate_fake_data_shake(&fake_data); break;
            case MPU6050_FAKE_PATTERN_TILT: generate_fake_data_tilt(&fake_data); break;
            case MPU6050_FAKE_PATTERN_CIRCLE: generate_fake_data_circle(&fake_data); break;
            default: fake_data = m_custom_fake_data; break;
        }
        *accel_x = fake_data.accel_x_g;
        *accel_y = fake_data.accel_y_g;
        *accel_z = fake_data.accel_z_g;
        m_fake_data_counter++;
        return NRF_SUCCESS;
    }

    // Real read
    ret_code_t err_code;
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;

    err_code = mpu6050_read_regs(p_twi, device_addr, MPU6050_REG_ACCEL_XOUT_H, data, 6);
    if (err_code != NRF_SUCCESS) return err_code;

    raw_x = (int16_t)((data[0] << 8) | data[1]);
    raw_y = (int16_t)((data[2] << 8) | data[3]);
    raw_z = (int16_t)((data[4] << 8) | data[5]);

    *accel_x = (float)raw_x / MPU6050_ACCEL_SENSITIVITY_2G;
    *accel_y = (float)raw_y / MPU6050_ACCEL_SENSITIVITY_2G;
    *accel_z = (float)raw_z / MPU6050_ACCEL_SENSITIVITY_2G;

    return NRF_SUCCESS;
}

ret_code_t mpu6050_read_gyro(const nrf_drv_twi_t *p_twi, uint8_t device_addr,
                             float *gyro_x, float *gyro_y, float *gyro_z)
{
    // Check fake mode
    if (m_fake_data_enabled) {
        mpu6050_data_t fake_data;
        switch (m_fake_pattern) {
            case MPU6050_FAKE_PATTERN_STATIC: generate_fake_data_static(&fake_data); break;
            case MPU6050_FAKE_PATTERN_SINE: generate_fake_data_sine(&fake_data); break;
            case MPU6050_FAKE_PATTERN_SHAKE: generate_fake_data_shake(&fake_data); break;
            case MPU6050_FAKE_PATTERN_TILT: generate_fake_data_tilt(&fake_data); break;
            case MPU6050_FAKE_PATTERN_CIRCLE: generate_fake_data_circle(&fake_data); break;
            default: fake_data = m_custom_fake_data; break;
        }
        *gyro_x = fake_data.gyro_x_dps;
        *gyro_y = fake_data.gyro_y_dps;
        *gyro_z = fake_data.gyro_z_dps;
        m_fake_data_counter++;
        return NRF_SUCCESS;
    }

    // Real read
    ret_code_t err_code;
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;

    err_code = mpu6050_read_regs(p_twi, device_addr, MPU6050_REG_GYRO_XOUT_H, data, 6);
    if (err_code != NRF_SUCCESS) return err_code;

    raw_x = (int16_t)((data[0] << 8) | data[1]);
    raw_y = (int16_t)((data[2] << 8) | data[3]);
    raw_z = (int16_t)((data[4] << 8) | data[5]);

    *gyro_x = (float)raw_x / MPU6050_GYRO_SENSITIVITY_250DPS;
    *gyro_y = (float)raw_y / MPU6050_GYRO_SENSITIVITY_250DPS;
    *gyro_z = (float)raw_z / MPU6050_GYRO_SENSITIVITY_250DPS;

    return NRF_SUCCESS;
}

ret_code_t mpu6050_read_all(const nrf_drv_twi_t *p_twi, uint8_t device_addr, mpu6050_data_t *data)
{
    // Check fake mode
    if (m_fake_data_enabled) {
        switch (m_fake_pattern) {
            case MPU6050_FAKE_PATTERN_STATIC: generate_fake_data_static(data); break;
            case MPU6050_FAKE_PATTERN_SINE: generate_fake_data_sine(data); break;
            case MPU6050_FAKE_PATTERN_SHAKE: generate_fake_data_shake(data); break;
            case MPU6050_FAKE_PATTERN_TILT: generate_fake_data_tilt(data); break;
            case MPU6050_FAKE_PATTERN_CIRCLE: generate_fake_data_circle(data); break;
            default: *data = m_custom_fake_data; break;
        }
        m_fake_data_counter++;
        return NRF_SUCCESS;
    }

    // Real read
    ret_code_t err_code;
    uint8_t raw_data[14];
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

    err_code = mpu6050_read_regs(p_twi, device_addr, MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (err_code != NRF_SUCCESS) return err_code;

    raw_accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    raw_accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    raw_accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    raw_gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    raw_gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    raw_gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    data->accel_x_g = (float)raw_accel_x / MPU6050_ACCEL_SENSITIVITY_2G;
    data->accel_y_g = (float)raw_accel_y / MPU6050_ACCEL_SENSITIVITY_2G;
    data->accel_z_g = (float)raw_accel_z / MPU6050_ACCEL_SENSITIVITY_2G;
    data->gyro_x_dps = (float)raw_gyro_x / MPU6050_GYRO_SENSITIVITY_250DPS;
    data->gyro_y_dps = (float)raw_gyro_y / MPU6050_GYRO_SENSITIVITY_250DPS;
    data->gyro_z_dps = (float)raw_gyro_z / MPU6050_GYRO_SENSITIVITY_250DPS;

    return NRF_SUCCESS;
}

// Fake data control functions
ret_code_t mpu6050_enable_fake_data(mpu6050_fake_pattern_t pattern)
{
    if (pattern >= MPU6050_FAKE_PATTERN_COUNT) return NRF_ERROR_INVALID_PARAM;
    m_fake_pattern = pattern;
    m_fake_data_enabled = true;
    m_fake_data_counter = 0;
    return NRF_SUCCESS;
}

ret_code_t mpu6050_disable_fake_data(void)
{
    m_fake_data_enabled = false;
    return NRF_SUCCESS;
}

bool mpu6050_is_fake_data_enabled(void)
{
    return m_fake_data_enabled;
}

ret_code_t mpu6050_set_custom_fake_data(float accel_x, float accel_y, float accel_z,
                                        float gyro_x, float gyro_y, float gyro_z)
{
    m_custom_fake_data.accel_x_g = accel_x;
    m_custom_fake_data.accel_y_g = accel_y;
    m_custom_fake_data.accel_z_g = accel_z;
    m_custom_fake_data.gyro_x_dps = gyro_x;
    m_custom_fake_data.gyro_y_dps = gyro_y;
    m_custom_fake_data.gyro_z_dps = gyro_z;
    return NRF_SUCCESS;
}

// Fake data generators
static void generate_fake_data_static(mpu6050_data_t* data)
{
    data->accel_x_g = 0.0f;
    data->accel_y_g = 0.0f;
    data->accel_z_g = 1.0f;  // Gravity
    data->gyro_x_dps = 0.0f;
    data->gyro_y_dps = 0.0f;
    data->gyro_z_dps = 0.0f;
}

static void generate_fake_data_sine(mpu6050_data_t* data)
{
    float time = (float)m_fake_data_counter * 0.02f;  // 50Hz
    data->accel_x_g = 0.5f * sinf(2.0f * M_PI * 1.0f * time);
    data->accel_y_g = 0.3f * sinf(2.0f * M_PI * 1.5f * time);
    data->accel_z_g = 1.0f + 0.2f * sinf(2.0f * M_PI * 0.5f * time);
    data->gyro_x_dps = 30.0f * sinf(2.0f * M_PI * 2.0f * time);
    data->gyro_y_dps = 45.0f * sinf(2.0f * M_PI * 1.2f * time);
    data->gyro_z_dps = 20.0f * sinf(2.0f * M_PI * 0.8f * time);
}

static void generate_fake_data_shake(mpu6050_data_t* data)
{
    float time = (float)m_fake_data_counter * 0.02f;
    float freq = 5.0f;
    data->accel_x_g = 2.0f * sinf(2.0f * M_PI * freq * time) * (0.5f + 0.5f * sinf(2.0f * M_PI * 0.3f * time));
    data->accel_y_g = 1.5f * sinf(2.0f * M_PI * freq * time + M_PI/3) * (0.5f + 0.5f * sinf(2.0f * M_PI * 0.4f * time));
    data->accel_z_g = 1.0f + 1.0f * sinf(2.0f * M_PI * freq * time + 2*M_PI/3) * (0.5f + 0.5f * sinf(2.0f * M_PI * 0.2f * time));
    data->gyro_x_dps = 150.0f * sinf(2.0f * M_PI * freq * time);
    data->gyro_y_dps = 120.0f * sinf(2.0f * M_PI * freq * time + M_PI/4);
    data->gyro_z_dps = 100.0f * sinf(2.0f * M_PI * freq * time + M_PI/2);
}

static void generate_fake_data_tilt(mpu6050_data_t* data)
{
    float time = (float)m_fake_data_counter * 0.02f;
    float angle = 0.5f * sinf(2.0f * M_PI * 0.2f * time);
    data->accel_x_g = sinf(angle);
    data->accel_y_g = 0.3f * sinf(2.0f * M_PI * 0.15f * time);
    data->accel_z_g = cosf(angle);
    data->gyro_x_dps = 20.0f * cosf(2.0f * M_PI * 0.2f * time);
    data->gyro_y_dps = 15.0f * sinf(2.0f * M_PI * 0.15f * time);
    data->gyro_z_dps = 5.0f * sinf(2.0f * M_PI * 0.1f * time);
}

static void generate_fake_data_circle(mpu6050_data_t* data)
{
    float time = (float)m_fake_data_counter * 0.02f;
    float freq = 0.5f;
    data->accel_x_g = 1.5f * cosf(2.0f * M_PI * freq * time);
    data->accel_y_g = 1.5f * sinf(2.0f * M_PI * freq * time);
    data->accel_z_g = 1.0f;
    data->gyro_x_dps = -90.0f * sinf(2.0f * M_PI * freq * time);
    data->gyro_y_dps = 90.0f * cosf(2.0f * M_PI * freq * time);
    data->gyro_z_dps = 60.0f * cosf(2.0f * M_PI * freq * time);
}

// Internal helpers
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
