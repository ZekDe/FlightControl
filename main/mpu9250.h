#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "i2c_driver.h"

/* ============================================================================
 * MPU9250 I2C Address
 * ============================================================================ */
#define MPU9250_I2C_ADDR            0x68    /* AD0 = LOW */
#define MPU9250_WHO_AM_I_VALUE      0x71    /* MPU9250 WHO_AM_I */
#define MPU9255_WHO_AM_I_VALUE      0x73    /* MPU9255 WHO_AM_I */

/* ============================================================================
 * MPU9250 Register Map (Only needed registers)
 * ============================================================================ */

/* Sample Rate Divider */
#define MPU9250_REG_SMPLRT_DIV      0x19

/* Configuration */
#define MPU9250_REG_CONFIG          0x1A
#define MPU9250_REG_GYRO_CONFIG     0x1B
#define MPU9250_REG_ACCEL_CONFIG    0x1C
#define MPU9250_REG_ACCEL_CONFIG2   0x1D

/* Interrupt */
#define MPU9250_REG_INT_PIN_CFG     0x37
#define MPU9250_REG_INT_ENABLE      0x38
#define MPU9250_REG_INT_STATUS      0x3A

/* Accelerometer Data */
#define MPU9250_REG_ACCEL_XOUT_H    0x3B
#define MPU9250_REG_ACCEL_XOUT_L    0x3C
#define MPU9250_REG_ACCEL_YOUT_H    0x3D
#define MPU9250_REG_ACCEL_YOUT_L    0x3E
#define MPU9250_REG_ACCEL_ZOUT_H    0x3F
#define MPU9250_REG_ACCEL_ZOUT_L    0x40

/* Temperature Data */
#define MPU9250_REG_TEMP_OUT_H      0x41
#define MPU9250_REG_TEMP_OUT_L      0x42

/* Gyroscope Data */
#define MPU9250_REG_GYRO_XOUT_H     0x43
#define MPU9250_REG_GYRO_XOUT_L     0x44
#define MPU9250_REG_GYRO_YOUT_H     0x45
#define MPU9250_REG_GYRO_YOUT_L     0x46
#define MPU9250_REG_GYRO_ZOUT_H     0x47
#define MPU9250_REG_GYRO_ZOUT_L     0x48

/* Power Management */
#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_PWR_MGMT_2      0x6C

/* WHO AM I */
#define MPU9250_REG_WHO_AM_I        0x75

/* ============================================================================
 * Gyroscope Full Scale Range
 * ============================================================================ */
typedef enum {
    MPU9250_GYRO_FS_250DPS  = 0x00,     /* ±250 °/s  */
    MPU9250_GYRO_FS_500DPS  = 0x01,     /* ±500 °/s  */
    MPU9250_GYRO_FS_1000DPS = 0x02,     /* ±1000 °/s */
    MPU9250_GYRO_FS_2000DPS = 0x03,     /* ±2000 °/s */
} mpu9250_gyro_fs_t;

/* ============================================================================
 * Accelerometer Full Scale Range
 * ============================================================================ */
typedef enum {
    MPU9250_ACCEL_FS_2G  = 0x00,        /* ±2g  */
    MPU9250_ACCEL_FS_4G  = 0x01,        /* ±4g  */
    MPU9250_ACCEL_FS_8G  = 0x02,        /* ±8g  */
    MPU9250_ACCEL_FS_16G = 0x03,        /* ±16g */
} mpu9250_accel_fs_t;

/* ============================================================================
 * DLPF (Digital Low Pass Filter) Configuration
 * Gyro DLPF - CONFIG register bits [2:0]
 * ============================================================================ */
typedef enum {
    MPU9250_DLPF_250HZ = 0x00,          /* 250Hz, 0.97ms delay */
    MPU9250_DLPF_184HZ = 0x01,          /* 184Hz, 2.9ms delay  */
    MPU9250_DLPF_92HZ  = 0x02,          /* 92Hz,  3.9ms delay  */
    MPU9250_DLPF_41HZ  = 0x03,          /* 41Hz,  5.9ms delay  */
    MPU9250_DLPF_20HZ  = 0x04,          /* 20Hz,  9.9ms delay  */
    MPU9250_DLPF_10HZ  = 0x05,          /* 10Hz,  17.85ms delay */
    MPU9250_DLPF_5HZ   = 0x06,          /* 5Hz,   33.48ms delay */
} mpu9250_dlpf_t;

/* ============================================================================
 * Accelerometer DLPF Configuration
 * ACCEL_CONFIG2 register bits [2:0]
 * ============================================================================ */
typedef enum {
    MPU9250_ACCEL_DLPF_218HZ = 0x01,    /* 218.1Hz, 1.88ms delay */
    MPU9250_ACCEL_DLPF_99HZ  = 0x02,    /* 99Hz,    2.88ms delay */
    MPU9250_ACCEL_DLPF_45HZ  = 0x03,    /* 44.8Hz,  4.88ms delay */
    MPU9250_ACCEL_DLPF_21HZ  = 0x04,    /* 21.2Hz,  8.87ms delay */
    MPU9250_ACCEL_DLPF_10HZ  = 0x05,    /* 10.2Hz,  16.83ms delay */
    MPU9250_ACCEL_DLPF_5HZ   = 0x06,    /* 5.05Hz,  32.48ms delay */
    MPU9250_ACCEL_DLPF_420HZ = 0x07,    /* 420Hz,   1.38ms delay */
} mpu9250_accel_dlpf_t;

/* ============================================================================
 * Raw Data Structure (16-bit signed)
 * ============================================================================ */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu9250_raw_data_t;

/* ============================================================================
 * Scaled Data Structure (float, physical units)
 * ============================================================================ */
typedef struct {
    float accel_x;      /* m/s² */
    float accel_y;      /* m/s² */
    float accel_z;      /* m/s² */
    float temp;         /* °C   */
    float gyro_x;       /* rad/s */
    float gyro_y;       /* rad/s */
    float gyro_z;       /* rad/s */
} mpu9250_scaled_data_t;

/* ============================================================================
 * MPU9250 Configuration Structure
 * ============================================================================ */
typedef struct {
    mpu9250_gyro_fs_t       gyro_fs;
    mpu9250_accel_fs_t      accel_fs;
    mpu9250_dlpf_t          gyro_dlpf;
    mpu9250_accel_dlpf_t    accel_dlpf;
    uint8_t                 sample_rate_div;    /* Output Rate = 1kHz / (1 + div) */
} mpu9250_config_t;

/* ============================================================================
 * MPU9250 Device Handle
 * ============================================================================ */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    mpu9250_config_t        config;
    float                   gyro_scale;     /* LSB to rad/s */
    float                   accel_scale;    /* LSB to m/s²  */
    uint8_t                 is_initialized;
} mpu9250_handle_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize MPU9250 with given configuration
 * 
 * @param handle Pointer to MPU9250 handle
 * @param i2c_driver Pointer to initialized I2C driver
 * @param config Pointer to configuration structure
 * @return int8_t 0 on success, negative on error
 */
int8_t mpu9250Init(mpu9250_handle_t *handle, i2c_driver_t *i2c_driver, 
                   const mpu9250_config_t *config);

/**
 * @brief Deinitialize MPU9250
 * 
 * @param handle Pointer to MPU9250 handle
 * @return int8_t 0 on success, negative on error
 */
int8_t mpu9250Deinit(mpu9250_handle_t *handle);

/**
 * @brief Read WHO_AM_I register to verify device
 * 
 * @param handle Pointer to MPU9250 handle
 * @param who_am_i Pointer to store WHO_AM_I value
 * @return int8_t 0 on success, negative on error
 */
int8_t mpu9250ReadWhoAmI(mpu9250_handle_t *handle, uint8_t *who_am_i);

/**
 * @brief Read raw sensor data (accel, temp, gyro)
 * 
 * @param handle Pointer to MPU9250 handle
 * @param raw_data Pointer to store raw data
 * @return int8_t 0 on success, negative on error
 */
int8_t mpu9250ReadRaw(mpu9250_handle_t *handle, mpu9250_raw_data_t *raw_data);

/**
 * @brief Read and convert sensor data to physical units
 * 
 * @param handle Pointer to MPU9250 handle
 * @param scaled_data Pointer to store scaled data
 * @return int8_t 0 on success, negative on error
 */
int8_t mpu9250ReadScaled(mpu9250_handle_t *handle, mpu9250_scaled_data_t *scaled_data);

/**
 * @brief Convert raw data to scaled data
 * 
 * @param handle Pointer to MPU9250 handle
 * @param raw_data Pointer to raw data
 * @param scaled_data Pointer to store scaled data
 */
void mpu9250ConvertRawToScaled(mpu9250_handle_t *handle, 
                                const mpu9250_raw_data_t *raw_data,
                                mpu9250_scaled_data_t *scaled_data);

/**
 * @brief Reset MPU9250 device
 * 
 * @param handle Pointer to MPU9250 handle
 * @return int8_t 0 on success, negative on error
 */
int8_t mpu9250Reset(mpu9250_handle_t *handle);

/**
 * @brief Get default configuration (2000dps, 8g, 92Hz DLPF, 1kHz sample rate)
 * 
 * @param config Pointer to store default configuration
 */
void mpu9250GetDefaultConfig(mpu9250_config_t *config);

#endif /* MPU9250_H */
