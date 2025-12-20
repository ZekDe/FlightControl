/**
 * @file ak8963.h
 * @brief AK8963 Magnetometer Driver for MPU9250
 * @details 3-axis magnetometer embedded in MPU9250/9255
 * 
 * @note Accessed via MPU9250 I2C bypass mode
 * @note Full scale: ±4912 µT (14-bit) or ±4912 µT (16-bit)
 */

#ifndef AK8963_H
#define AK8963_H

#include <stdint.h>
#include "i2c_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * AK8963 I2C Address
 ******************************************************************************/
#define AK8963_I2C_ADDR         0x0C    /* Default I2C address */
#define AK8963_WHO_AM_I_VALUE   0x48    /* Device ID */

/*******************************************************************************
 * AK8963 Register Map
 ******************************************************************************/
#define AK8963_REG_WIA          0x00    /* Device ID (should return 0x48) */
#define AK8963_REG_INFO         0x01    /* Device information */
#define AK8963_REG_ST1          0x02    /* Status 1 (data ready) */
#define AK8963_REG_HXL          0x03    /* X-axis data LSB */
#define AK8963_REG_HXH          0x04    /* X-axis data MSB */
#define AK8963_REG_HYL          0x05    /* Y-axis data LSB */
#define AK8963_REG_HYH          0x06    /* Y-axis data MSB */
#define AK8963_REG_HZL          0x07    /* Z-axis data LSB */
#define AK8963_REG_HZH          0x08    /* Z-axis data MSB */
#define AK8963_REG_ST2          0x09    /* Status 2 (overflow, output setting) */
#define AK8963_REG_CNTL1        0x0A    /* Control 1 (mode, output bit) */
#define AK8963_REG_CNTL2        0x0B    /* Control 2 (soft reset) */
#define AK8963_REG_ASTC         0x0C    /* Self-test control */
#define AK8963_REG_TS1          0x0D    /* Test 1 (factory use) */
#define AK8963_REG_TS2          0x0E    /* Test 2 (factory use) */
#define AK8963_REG_I2CDIS       0x0F    /* I2C disable */
#define AK8963_REG_ASAX         0x10    /* Sensitivity adjustment X */
#define AK8963_REG_ASAY         0x11    /* Sensitivity adjustment Y */
#define AK8963_REG_ASAZ         0x12    /* Sensitivity adjustment Z */

/*******************************************************************************
 * MPU9250 Registers for Bypass Mode
 ******************************************************************************/
#define MPU9250_REG_INT_PIN_CFG 0x37    /* INT Pin/Bypass Enable Config */
#define MPU9250_REG_USER_CTRL   0x6A    /* User Control */

/*******************************************************************************
 * AK8963 Operation Modes
 ******************************************************************************/
typedef enum {
    AK8963_MODE_POWER_DOWN  = 0x00,     /* Power-down mode */
    AK8963_MODE_SINGLE      = 0x01,     /* Single measurement mode */
    AK8963_MODE_CONT1       = 0x02,     /* Continuous 1 (8Hz) */
    AK8963_MODE_CONT2       = 0x06,     /* Continuous 2 (100Hz) */
    AK8963_MODE_EXT_TRIG    = 0x04,     /* External trigger mode */
    AK8963_MODE_SELF_TEST   = 0x08,     /* Self-test mode */
    AK8963_MODE_FUSE_ROM    = 0x0F,     /* Fuse ROM access mode */
} ak8963_mode_t;

/*******************************************************************************
 * AK8963 Output Bit Setting
 ******************************************************************************/
typedef enum {
    AK8963_OUTPUT_14BIT     = 0x00,     /* 14-bit output (0.6 µT/LSB) */
    AK8963_OUTPUT_16BIT     = 0x10,     /* 16-bit output (0.15 µT/LSB) */
} ak8963_output_t;

/*******************************************************************************
 * Calibration Data Structure
 ******************************************************************************/
typedef struct {
    /* Hard iron offset (µT) */
    float offset_x;
    float offset_y;
    float offset_z;
    
    /* Soft iron scale factors */
    float scale_x;
    float scale_y;
    float scale_z;
    
    /* Rotation matrix for misalignment (optional) */
    float rotation[9];  /* 3x3 row-major */
    
    uint8_t is_calibrated;
} ak8963_calibration_t;

/*******************************************************************************
 * Raw Data Structure
 ******************************************************************************/
typedef struct {
    int16_t hx;     /* X-axis raw */
    int16_t hy;     /* Y-axis raw */
    int16_t hz;     /* Z-axis raw */
    uint8_t st1;    /* Status 1 */
    uint8_t st2;    /* Status 2 */
} ak8963_raw_data_t;

/*******************************************************************************
 * Scaled Data Structure
 ******************************************************************************/
typedef struct {
    float mx;       /* X-axis [µT] */
    float my;       /* Y-axis [µT] */
    float mz;       /* Z-axis [µT] */
    float heading;  /* Magnetic heading [degrees] (0-360) */
} ak8963_scaled_data_t;

/*******************************************************************************
 * Configuration Structure
 ******************************************************************************/
typedef struct {
    ak8963_mode_t mode;
    ak8963_output_t output;
} ak8963_config_t;

/*******************************************************************************
 * Device Handle
 ******************************************************************************/
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_dev_handle_t mpu_dev;    /* MPU9250 device for bypass control */
    
    ak8963_config_t config;
    ak8963_calibration_t calibration;
    
    /* Sensitivity adjustment values from Fuse ROM */
    float asa_x;    /* Sensitivity adjustment X */
    float asa_y;    /* Sensitivity adjustment Y */
    float asa_z;    /* Sensitivity adjustment Z */
    
    /* Scale factor: LSB to µT */
    float mag_scale;
    
    uint8_t is_initialized;
} ak8963_handle_t;

/*******************************************************************************
 * Function Prototypes - Initialization
 ******************************************************************************/

/**
 * @brief Enable I2C bypass mode on MPU9250
 * @param mpu_dev MPU9250 I2C device handle
 * @return 0 on success, negative on error
 * 
 * @note Must be called before AK8963 can be accessed
 */
int8_t ak8963EnableBypass(i2c_master_dev_handle_t mpu_dev);

/**
 * @brief Initialize AK8963 magnetometer
 * @param handle Pointer to AK8963 handle
 * @param i2c_driver Pointer to I2C driver
 * @param mpu_dev MPU9250 device handle (for bypass control)
 * @param config Configuration settings
 * @return 0 on success, negative on error
 */
int8_t ak8963Init(ak8963_handle_t *handle, 
                   i2c_driver_t *i2c_driver,
                   i2c_master_dev_handle_t mpu_dev,
                   const ak8963_config_t *config);

/**
 * @brief Get default configuration (Continuous 100Hz, 16-bit)
 * @param config Pointer to store default config
 */
void ak8963GetDefaultConfig(ak8963_config_t *config);

/**
 * @brief Deinitialize AK8963
 * @param handle Pointer to AK8963 handle
 * @return 0 on success, negative on error
 */
int8_t ak8963Deinit(ak8963_handle_t *handle);

/**
 * @brief Soft reset AK8963
 * @param handle Pointer to AK8963 handle
 * @return 0 on success, negative on error
 */
int8_t ak8963Reset(ak8963_handle_t *handle);

/*******************************************************************************
 * Function Prototypes - Data Reading
 ******************************************************************************/

/**
 * @brief Check if new data is ready
 * @param handle Pointer to AK8963 handle
 * @param ready Pointer to store result (1 = ready)
 * @return 0 on success, negative on error
 */
int8_t ak8963DataReady(ak8963_handle_t *handle, uint8_t *ready);

/**
 * @brief Read raw magnetometer data
 * @param handle Pointer to AK8963 handle
 * @param raw_data Pointer to store raw data
 * @return 0 on success, negative on error
 */
int8_t ak8963ReadRaw(ak8963_handle_t *handle, ak8963_raw_data_t *raw_data);

/**
 * @brief Read and convert to scaled data (µT)
 * @param handle Pointer to AK8963 handle
 * @param scaled_data Pointer to store scaled data
 * @return 0 on success, negative on error
 */
int8_t ak8963ReadScaled(ak8963_handle_t *handle, ak8963_scaled_data_t *scaled_data);

/**
 * @brief Read calibrated magnetometer data
 * @param handle Pointer to AK8963 handle
 * @param scaled_data Pointer to store calibrated data
 * @return 0 on success, negative on error
 * 
 * @note Applies hard/soft iron calibration if available
 */
int8_t ak8963ReadCalibrated(ak8963_handle_t *handle, ak8963_scaled_data_t *scaled_data);

/*******************************************************************************
 * Function Prototypes - Calibration
 ******************************************************************************/

/**
 * @brief Set calibration parameters
 * @param handle Pointer to AK8963 handle
 * @param calibration Pointer to calibration data
 * @return 0 on success, negative on error
 */
int8_t ak8963SetCalibration(ak8963_handle_t *handle, 
                             const ak8963_calibration_t *calibration);

/**
 * @brief Get default calibration (no correction)
 * @param calibration Pointer to store default calibration
 */
void ak8963GetDefaultCalibration(ak8963_calibration_t *calibration);

/**
 * @brief Calculate heading from magnetometer data
 * @param mx X-axis magnetic field [µT]
 * @param my Y-axis magnetic field [µT]
 * @param declination Magnetic declination [degrees] (optional, pass 0)
 * @return Heading in degrees (0-360)
 */
float ak8963CalculateHeading(float mx, float my, float declination);

/**
 * @brief Calculate tilt-compensated heading
 * @param mx X-axis magnetic field [µT]
 * @param my Y-axis magnetic field [µT]
 * @param mz Z-axis magnetic field [µT]
 * @param roll Roll angle [rad]
 * @param pitch Pitch angle [rad]
 * @param declination Magnetic declination [degrees]
 * @return Tilt-compensated heading in degrees (0-360)
 */
float ak8963CalculateTiltCompensatedHeading(float mx, float my, float mz,
                                             float roll, float pitch,
                                             float declination);

#ifdef __cplusplus
}
#endif

#endif /* AK8963_H */
