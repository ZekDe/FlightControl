#include "mpu9250.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MPU9250";

/* ============================================================================
 * Constants
 * ============================================================================ */
#define GRAVITY_MS2         9.80665f
#define DEG_TO_RAD          (M_PI / 180.0f)

/* Gyro sensitivity (LSB per °/s) */
#define GYRO_SENS_250DPS    131.0f
#define GYRO_SENS_500DPS    65.5f
#define GYRO_SENS_1000DPS   32.8f
#define GYRO_SENS_2000DPS   16.4f

/* Accel sensitivity (LSB per g) */
#define ACCEL_SENS_2G       16384.0f
#define ACCEL_SENS_4G       8192.0f
#define ACCEL_SENS_8G       4096.0f
#define ACCEL_SENS_16G      2048.0f

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static float getGyroScale(mpu9250_gyro_fs_t fs)
{
    float sensitivity;
    
    switch (fs) {
        case MPU9250_GYRO_FS_250DPS:
            sensitivity = GYRO_SENS_250DPS;
            break;
        case MPU9250_GYRO_FS_500DPS:
            sensitivity = GYRO_SENS_500DPS;
            break;
        case MPU9250_GYRO_FS_1000DPS:
            sensitivity = GYRO_SENS_1000DPS;
            break;
        case MPU9250_GYRO_FS_2000DPS:
        default:
            sensitivity = GYRO_SENS_2000DPS;
            break;
    }
    
    /* Return scale factor: LSB -> rad/s */
    return DEG_TO_RAD / sensitivity;
}

static float getAccelScale(mpu9250_accel_fs_t fs)
{
    float sensitivity;
    
    switch (fs) {
        case MPU9250_ACCEL_FS_2G:
            sensitivity = ACCEL_SENS_2G;
            break;
        case MPU9250_ACCEL_FS_4G:
            sensitivity = ACCEL_SENS_4G;
            break;
        case MPU9250_ACCEL_FS_8G:
        default:
            sensitivity = ACCEL_SENS_8G;
            break;
        case MPU9250_ACCEL_FS_16G:
            sensitivity = ACCEL_SENS_16G;
            break;
    }
    
    /* Return scale factor: LSB -> m/s² */
    return GRAVITY_MS2 / sensitivity;
}

/* ============================================================================
 * Get Default Configuration
 * ============================================================================ */
void mpu9250GetDefaultConfig(mpu9250_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    config->gyro_fs = MPU9250_GYRO_FS_2000DPS;
    config->accel_fs = MPU9250_ACCEL_FS_8G;
    config->gyro_dlpf = MPU9250_DLPF_92HZ;
    config->accel_dlpf = MPU9250_ACCEL_DLPF_99HZ;  /* Closest to 92Hz */
    config->sample_rate_div = 0;  /* 1kHz output rate */
}

/* ============================================================================
 * Initialize MPU9250
 * ============================================================================ */
int8_t mpu9250Init(mpu9250_handle_t *handle, i2c_driver_t *i2c_driver,
                   const mpu9250_config_t *config)
{
    if (handle == NULL || i2c_driver == NULL || config == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return -1;
    }

    if (!i2c_driver->is_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized");
        return -2;
    }

    int8_t ret;

    /* Add MPU9250 device to I2C bus */
    ret = i2cDriverAddDevice(i2c_driver, MPU9250_I2C_ADDR, &handle->i2c_dev);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to add MPU9250 to I2C bus");
        return -3;
    }

    /* Store configuration */
    handle->config = *config;
    handle->gyro_scale = getGyroScale(config->gyro_fs);
    handle->accel_scale = getAccelScale(config->accel_fs);

    /* Verify device by reading WHO_AM_I */
    uint8_t who_am_i;
    ret = mpu9250ReadWhoAmI(handle, &who_am_i);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -4;
    }

    if (who_am_i != MPU9250_WHO_AM_I_VALUE && who_am_i != MPU9255_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: 0x%02X (expected 0x71 or 0x73)", who_am_i);
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -5;
    }

    ESP_LOGI(TAG, "%s detected (WHO_AM_I: 0x%02X)", 
             (who_am_i == MPU9255_WHO_AM_I_VALUE) ? "MPU9255" : "MPU9250", who_am_i);


    /* Reset device */
    ret = mpu9250Reset(handle);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to reset MPU9250");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -6;
    }

    /* Wait for reset to complete */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Wake up device, use best available clock (PLL with Gyro X) */
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_PWR_MGMT_1, 0x01);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to wake up MPU9250");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -7;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    /* Enable all axes (default) */
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_PWR_MGMT_2, 0x00);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to enable axes");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -8;
    }

    /* Configure sample rate divider */
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_SMPLRT_DIV, config->sample_rate_div);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set sample rate divider");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -9;
    }

    /* Configure DLPF (Gyro) */
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_CONFIG, config->gyro_dlpf);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to configure gyro DLPF");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -10;
    }

    /* Configure Gyro full scale range */
    uint8_t gyro_config = (config->gyro_fs << 3);
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_GYRO_CONFIG, gyro_config);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to configure gyro range");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -11;
    }

    /* Configure Accel full scale range */
    uint8_t accel_config = (config->accel_fs << 3);
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_ACCEL_CONFIG, accel_config);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to configure accel range");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -12;
    }

    /* Configure Accel DLPF */
    ret = i2cWriteByte(handle->i2c_dev, MPU9250_REG_ACCEL_CONFIG2, config->accel_dlpf);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to configure accel DLPF");
        i2cDriverRemoveDevice(handle->i2c_dev);
        return -13;
    }

    handle->is_initialized = 1;

    ESP_LOGI(TAG, "MPU9250 initialized successfully");
    ESP_LOGI(TAG, "  Gyro: ±%d dps, DLPF: %d", 
             250 << config->gyro_fs, config->gyro_dlpf);
    ESP_LOGI(TAG, "  Accel: ±%dg, DLPF: %d",
             2 << config->accel_fs, config->accel_dlpf);
    ESP_LOGI(TAG, "  Sample rate: %d Hz", 1000 / (1 + config->sample_rate_div));

    return 0;
}

/* ============================================================================
 * Deinitialize MPU9250
 * ============================================================================ */
int8_t mpu9250Deinit(mpu9250_handle_t *handle)
{
    if (handle == NULL) {
        return -1;
    }

    if (!handle->is_initialized) {
        return 0;
    }

    /* Put device to sleep */
    i2cWriteByte(handle->i2c_dev, MPU9250_REG_PWR_MGMT_1, 0x40);

    /* Remove from I2C bus */
    i2cDriverRemoveDevice(handle->i2c_dev);

    handle->is_initialized = 0;
    handle->i2c_dev = NULL;

    ESP_LOGI(TAG, "MPU9250 deinitialized");
    return 0;
}

/* ============================================================================
 * Read WHO_AM_I Register
 * ============================================================================ */
int8_t mpu9250ReadWhoAmI(mpu9250_handle_t *handle, uint8_t *who_am_i)
{
    if (handle == NULL || who_am_i == NULL) {
        return -1;
    }

    return i2cReadByte(handle->i2c_dev, MPU9250_REG_WHO_AM_I, who_am_i);
}

/* ============================================================================
 * Reset MPU9250
 * ============================================================================ */
int8_t mpu9250Reset(mpu9250_handle_t *handle)
{
    if (handle == NULL) {
        return -1;
    }

    /* Set H_RESET bit */
    return i2cWriteByte(handle->i2c_dev, MPU9250_REG_PWR_MGMT_1, 0x80);
}

/* ============================================================================
 * Read Raw Data
 * ============================================================================ */
int8_t mpu9250ReadRaw(mpu9250_handle_t *handle, mpu9250_raw_data_t *raw_data)
{
    if (handle == NULL || raw_data == NULL) {
        return -1;
    }

    if (!handle->is_initialized) {
        return -2;
    }

    /* Read all 14 bytes in burst (ACCEL_XOUT_H to GYRO_ZOUT_L) */
    uint8_t buffer[14];
    int8_t ret = i2cReadBytes(handle->i2c_dev, MPU9250_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != 0) {
        return -3;
    }

    /* Convert to 16-bit signed values (big-endian) */
    raw_data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw_data->temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw_data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return 0;
}

/* ============================================================================
 * Convert Raw to Scaled Data
 * ============================================================================ */
void mpu9250ConvertRawToScaled(mpu9250_handle_t *handle,
                                const mpu9250_raw_data_t *raw_data,
                                mpu9250_scaled_data_t *scaled_data)
{
    if (handle == NULL || raw_data == NULL || scaled_data == NULL) {
        return;
    }

    /* Accel: LSB -> m/s² */
    scaled_data->accel_x = (float)raw_data->accel_x * handle->accel_scale;
    scaled_data->accel_y = (float)raw_data->accel_y * handle->accel_scale;
    scaled_data->accel_z = (float)raw_data->accel_z * handle->accel_scale;

    /* Gyro: LSB -> rad/s */
    scaled_data->gyro_x = (float)raw_data->gyro_x * handle->gyro_scale;
    scaled_data->gyro_y = (float)raw_data->gyro_y * handle->gyro_scale;
    scaled_data->gyro_z = (float)raw_data->gyro_z * handle->gyro_scale;

    /* Temperature: Formula from datasheet */
    /* Temp (°C) = (TEMP_OUT / 333.87) + 21.0 */
    scaled_data->temp = ((float)raw_data->temp / 333.87f) + 21.0f;
}

/* ============================================================================
 * Read Scaled Data
 * ============================================================================ */
int8_t mpu9250ReadScaled(mpu9250_handle_t *handle, mpu9250_scaled_data_t *scaled_data)
{
    if (handle == NULL || scaled_data == NULL) {
        return -1;
    }

    mpu9250_raw_data_t raw_data;
    int8_t ret = mpu9250ReadRaw(handle, &raw_data);
    if (ret != 0) {
        return ret;
    }

    mpu9250ConvertRawToScaled(handle, &raw_data, scaled_data);
    return 0;
}
