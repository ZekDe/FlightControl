/**
 * @file ak8963.c
 * @brief AK8963 Magnetometer Driver Implementation
 */

#include "ak8963.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "AK8963";

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define MAG_SCALE_14BIT     (0.6f)      /* µT per LSB (14-bit mode) */
#define MAG_SCALE_16BIT     (0.15f)     /* µT per LSB (16-bit mode) */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
 * @brief Normalize angle to 0-360 range
 */
static float normalizeHeading(float heading)
{
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    return heading;
}

/*******************************************************************************
 * Bypass Mode Control
 ******************************************************************************/

int8_t ak8963EnableBypass(i2c_master_dev_handle_t mpu_dev)
{
    if (mpu_dev == NULL) {
        return -1;
    }
    
    int8_t ret;
    
    /* 
     * To access AK8963, we need to enable I2C bypass mode on MPU9250.
     * This connects the auxiliary I2C bus directly to the main I2C bus.
     * 
     * INT_PIN_CFG register (0x37):
     * Bit 1 (BYPASS_EN) = 1: Enable bypass mode
     * Bit 7 (ACTL) = 0: INT active high (default)
     * Bit 6 (OPEN) = 0: INT push-pull (default)
     * Bit 5 (LATCH_INT_EN) = 0: INT pulse (default)
     * Bit 4 (INT_ANYRD_2CLEAR) = 1: INT cleared on any read
     * 
     * Value = 0x12 (bypass enabled, int cleared on any read)
     */
    
    /* First, disable I2C master mode */
    ret = i2cWriteByte(mpu_dev, MPU9250_REG_USER_CTRL, 0x00);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to disable I2C master mode");
        return -2;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Enable bypass mode */
    ret = i2cWriteByte(mpu_dev, MPU9250_REG_INT_PIN_CFG, 0x02);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to enable bypass mode");
        return -3;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "MPU9250 bypass mode enabled");
    return 0;
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void ak8963GetDefaultConfig(ak8963_config_t *config)
{
    if (config == NULL) return;
    
    config->mode = AK8963_MODE_CONT2;       /* Continuous 100Hz */
    config->output = AK8963_OUTPUT_16BIT;   /* 16-bit resolution */
}

int8_t ak8963Init(ak8963_handle_t *handle, 
                   i2c_driver_t *i2c_driver,
                   i2c_master_dev_handle_t mpu_dev,
                   const ak8963_config_t *config)
{
    if (handle == NULL || i2c_driver == NULL || config == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return -1;
    }
    
    int8_t ret;
    memset(handle, 0, sizeof(ak8963_handle_t));
    
    /* Store MPU device handle for bypass control */
    handle->mpu_dev = mpu_dev;
    
    /* Enable bypass mode first */
    ret = ak8963EnableBypass(mpu_dev);
    if (ret != 0) {
        return -2;
    }
    
    /* Add AK8963 to I2C bus */
    ret = i2cDriverAddDevice(i2c_driver, AK8963_I2C_ADDR, &handle->i2c_dev);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to add AK8963 to I2C bus");
        return -3;
    }
    
    /* Verify device by reading WHO_AM_I */
    uint8_t who_am_i;
    ret = i2cReadByte(handle->i2c_dev, AK8963_REG_WIA, &who_am_i);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -4;
    }
    
    if (who_am_i != AK8963_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: 0x%02X (expected 0x%02X)", 
                 who_am_i, AK8963_WHO_AM_I_VALUE);
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -5;
    }
    
    ESP_LOGI(TAG, "AK8963 detected (WHO_AM_I: 0x%02X)", who_am_i);
    
    /* Soft reset */
    ret = ak8963Reset(handle);
    if (ret != 0) {
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -6;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* Read sensitivity adjustment values from Fuse ROM */
    ret = i2cWriteByte(handle->i2c_dev, AK8963_REG_CNTL1, AK8963_MODE_FUSE_ROM);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to enter Fuse ROM mode");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -7;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint8_t asa[3];
    ret = i2cReadBytes(handle->i2c_dev, AK8963_REG_ASAX, asa, 3);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read sensitivity adjustment");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -8;
    }
    
    /* 
     * Calculate sensitivity adjustment factors
     * Formula from datasheet: Hadj = H * ((ASA - 128) * 0.5 / 128 + 1)
     * Simplified: Hadj = H * (ASA - 128) / 256 + H
     */
    handle->asa_x = (float)(asa[0] - 128) / 256.0f + 1.0f;
    handle->asa_y = (float)(asa[1] - 128) / 256.0f + 1.0f;
    handle->asa_z = (float)(asa[2] - 128) / 256.0f + 1.0f;
    
    ESP_LOGI(TAG, "Sensitivity adjustment: X=%.3f Y=%.3f Z=%.3f",
             handle->asa_x, handle->asa_y, handle->asa_z);
    
    /* Power down before mode change */
    ret = i2cWriteByte(handle->i2c_dev, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN);
    if (ret != 0) {
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -9;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Set operating mode */
    uint8_t cntl1 = (uint8_t)config->mode | (uint8_t)config->output;
    ret = i2cWriteByte(handle->i2c_dev, AK8963_REG_CNTL1, cntl1);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set operating mode");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -10;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Store configuration */
    handle->config = *config;
    
    /* Set scale factor based on output bit setting */
    handle->mag_scale = (config->output == AK8963_OUTPUT_16BIT) ? 
                         MAG_SCALE_16BIT : MAG_SCALE_14BIT;
    
    /* Set default calibration (no correction) */
    ak8963GetDefaultCalibration(&handle->calibration);
    
    handle->is_initialized = 1;
    
    ESP_LOGI(TAG, "AK8963 initialized successfully");
    ESP_LOGI(TAG, "  Mode: %s, Resolution: %d-bit",
             (config->mode == AK8963_MODE_CONT2) ? "Continuous 100Hz" : "Other",
             (config->output == AK8963_OUTPUT_16BIT) ? 16 : 14);
    
    return 0;
}

int8_t ak8963Deinit(ak8963_handle_t *handle)
{
    if (handle == NULL) return -1;
    
    if (!handle->is_initialized) return 0;
    
    /* Power down */
    i2cWriteByte(handle->i2c_dev, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN);
    
    /* Remove from I2C bus */
    i2cDriverRemoveDevice(handle->i2c_dev);
    
    handle->is_initialized = 0;
    
    ESP_LOGI(TAG, "AK8963 deinitialized");
    return 0;
}

int8_t ak8963Reset(ak8963_handle_t *handle)
{
    if (handle == NULL) return -1;
    
    /* Soft reset: CNTL2 bit 0 = 1 */
    return i2cWriteByte(handle->i2c_dev, AK8963_REG_CNTL2, 0x01);
}

/*******************************************************************************
 * Data Reading
 ******************************************************************************/

int8_t ak8963DataReady(ak8963_handle_t *handle, uint8_t *ready)
{
    if (handle == NULL || ready == NULL) return -1;
    if (!handle->is_initialized) return -2;
    
    uint8_t st1;
    int8_t ret = i2cReadByte(handle->i2c_dev, AK8963_REG_ST1, &st1);
    if (ret != 0) return -3;
    
    *ready = (st1 & 0x01) ? 1 : 0;  /* DRDY bit */
    
    return 0;
}

int8_t ak8963ReadRaw(ak8963_handle_t *handle, ak8963_raw_data_t *raw_data)
{
    if (handle == NULL || raw_data == NULL) return -1;
    if (!handle->is_initialized) return -2;
    
    /* Read ST1 through ST2 (7 bytes total) */
    uint8_t buffer[7];
    int8_t ret = i2cReadBytes(handle->i2c_dev, AK8963_REG_ST1, buffer, 7);
    if (ret != 0) return -3;
    
    raw_data->st1 = buffer[0];
    
    /* Little-endian data */
    raw_data->hx = (int16_t)((buffer[2] << 8) | buffer[1]);
    raw_data->hy = (int16_t)((buffer[4] << 8) | buffer[3]);
    raw_data->hz = (int16_t)((buffer[6] << 8) | buffer[5]);
    
    /* ST2 must be read to signal end of measurement */
    ret = i2cReadByte(handle->i2c_dev, AK8963_REG_ST2, &raw_data->st2);
    if (ret != 0) return -4;
    
    /* Check for magnetic overflow */
    if (raw_data->st2 & 0x08) {
        ESP_LOGW(TAG, "Magnetic sensor overflow!");
        return -5;
    }
    
    return 0;
}

int8_t ak8963ReadScaled(ak8963_handle_t *handle, ak8963_scaled_data_t *scaled_data)
{
    if (handle == NULL || scaled_data == NULL) return -1;
    
    ak8963_raw_data_t raw;
    int8_t ret = ak8963ReadRaw(handle, &raw);
    if (ret != 0) return ret;
    
    /* Apply sensitivity adjustment and scale to µT */
    scaled_data->mx = (float)raw.hx * handle->mag_scale * handle->asa_x;
    scaled_data->my = (float)raw.hy * handle->mag_scale * handle->asa_y;
    scaled_data->mz = (float)raw.hz * handle->mag_scale * handle->asa_z;
    
    /* Calculate simple heading (no tilt compensation) */
    scaled_data->heading = ak8963CalculateHeading(scaled_data->mx, 
                                                   scaled_data->my, 0.0f);
    
    return 0;
}

int8_t ak8963ReadCalibrated(ak8963_handle_t *handle, ak8963_scaled_data_t *scaled_data)
{
    if (handle == NULL || scaled_data == NULL) return -1;
    if (!handle->is_initialized) return -2;
    
    /* First read scaled (uncalibrated) data */
    int8_t ret = ak8963ReadScaled(handle, scaled_data);
    if (ret != 0) return ret;
    
    /* Apply calibration if available */
    if (handle->calibration.is_calibrated) {
        /* Remove hard iron offset */
        float mx = scaled_data->mx - handle->calibration.offset_x;
        float my = scaled_data->my - handle->calibration.offset_y;
        float mz = scaled_data->mz - handle->calibration.offset_z;
        
        /* Apply soft iron correction (scale) */
        scaled_data->mx = mx * handle->calibration.scale_x;
        scaled_data->my = my * handle->calibration.scale_y;
        scaled_data->mz = mz * handle->calibration.scale_z;
        
        /* Recalculate heading with calibrated data */
        scaled_data->heading = ak8963CalculateHeading(scaled_data->mx,
                                                       scaled_data->my, 0.0f);
    }
    
    return 0;
}

/*******************************************************************************
 * Calibration
 ******************************************************************************/

void ak8963GetDefaultCalibration(ak8963_calibration_t *calibration)
{
    if (calibration == NULL) return;
    
    memset(calibration, 0, sizeof(ak8963_calibration_t));
    
    /* No offset correction */
    calibration->offset_x = 0.0f;
    calibration->offset_y = 0.0f;
    calibration->offset_z = 0.0f;
    
    /* Unity scale (no scaling) */
    calibration->scale_x = 1.0f;
    calibration->scale_y = 1.0f;
    calibration->scale_z = 1.0f;
    
    /* Identity rotation matrix */
    calibration->rotation[0] = 1.0f;
    calibration->rotation[4] = 1.0f;
    calibration->rotation[8] = 1.0f;
    
    calibration->is_calibrated = 0;
}

int8_t ak8963SetCalibration(ak8963_handle_t *handle, 
                             const ak8963_calibration_t *calibration)
{
    if (handle == NULL || calibration == NULL) return -1;
    if (!handle->is_initialized) return -2;
    
    handle->calibration = *calibration;
    
    ESP_LOGI(TAG, "Calibration set:");
    ESP_LOGI(TAG, "  Offsets: X=%.2f Y=%.2f Z=%.2f µT",
             calibration->offset_x, calibration->offset_y, calibration->offset_z);
    ESP_LOGI(TAG, "  Scales: X=%.3f Y=%.3f Z=%.3f",
             calibration->scale_x, calibration->scale_y, calibration->scale_z);
    
    return 0;
}

/*******************************************************************************
 * Heading Calculation
 ******************************************************************************/

float ak8963CalculateHeading(float mx, float my, float declination)
{
    /* 
     * Calculate heading from magnetometer X and Y
     * Heading = atan2(my, mx) converted to degrees
     * 
     * Note: This is only valid when the device is level!
     * For accurate heading when tilted, use tilt-compensated version.
     */
    
    float heading_rad = atan2f(my, mx);
    float heading_deg = heading_rad * 180.0f / M_PI;
    
    /* Add magnetic declination */
    heading_deg += declination;
    
    return normalizeHeading(heading_deg);
}

float ak8963CalculateTiltCompensatedHeading(float mx, float my, float mz,
                                             float roll, float pitch,
                                             float declination)
{
    /*
     * Tilt-compensated heading calculation
     * 
     * When the device is tilted, the raw magnetometer reading does not
     * represent the horizontal magnetic field component. We need to
     * project the magnetic vector onto the horizontal plane.
     * 
     * Using rotation matrix to compensate:
     * mx_h = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch)
     * my_h = my*cos(roll) - mz*sin(roll)
     */
    
    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);
    
    /* Horizontal components */
    float mx_h = mx * cos_pitch + 
                 my * sin_roll * sin_pitch + 
                 mz * cos_roll * sin_pitch;
    
    float my_h = my * cos_roll - mz * sin_roll;
    
    /* Calculate heading */
    float heading_rad = atan2f(my_h, mx_h);
    float heading_deg = heading_rad * 180.0f / M_PI;
    
    /* Add magnetic declination */
    heading_deg += declination;
    
    return normalizeHeading(heading_deg);
}
