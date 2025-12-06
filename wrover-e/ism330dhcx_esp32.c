/**
 * @file ism330dhcx_esp32.c
 * @brief ESP32 SPI platform layer implementation for ISM330DHCX
 */

#include "ism330dhcx_esp32.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

/*******************************************************************************
 * Constants
 ******************************************************************************/

static const char *TAG = "ISM330DHCX";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Conversion constants based on full scale settings */
/* These will be updated based on configuration */
static float accel_sensitivity = 0.244f;    /* mg/LSB for ±8g */
static float gyro_sensitivity = 70.0f;      /* mdps/LSB for ±2000dps */

/*******************************************************************************
 * Platform Functions (ST Driver Callbacks)
 ******************************************************************************/

/**
 * @brief SPI write function for ST driver
 * 
 * @details SPI write protocol for ISM330DHCX:
 *          - First byte: Register address (bit 7 = 0 for write)
 *          - Following bytes: Data to write
 */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    Ism330dhcxHandle_t *dev = (Ism330dhcxHandle_t *)handle;
    
    if (dev == NULL || bufp == NULL) {
        return -1;
    }
    static uint8_t tx_buf[64];
    /* Prepare transaction buffer: register + data */
    tx_buf[0] = reg & 0x7F;  /* Bit 7 = 0 for write */
    memcpy(&tx_buf[1], bufp, len);
    
    spi_transaction_t trans = {
        .length = (len + 1) * 8,    /* Length in bits */
        .tx_buffer = tx_buf,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);

    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * @brief SPI read function for ST driver
 * 
 * @details SPI read protocol for ISM330DHCX:
 *          - First byte: Register address with bit 7 = 1 (read)
 *          - Following bytes: Dummy bytes, device returns data
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    Ism330dhcxHandle_t *dev = (Ism330dhcxHandle_t *)handle;
    
    if (dev == NULL || bufp == NULL) {
        return -1;
    }
    
    /* Prepare TX buffer: register address with read bit */

    static uint8_t tx_buf[64];
    static uint8_t rx_buf[64];  
    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = reg | 0x80;  /* Bit 7 = 1 for read */
    
    spi_transaction_t trans = {
        .length = (len + 1) * 8,    /* Length in bits */
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };
    
    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);
    
    if (ret == ESP_OK) {
        /* Skip first byte (received during address transmission) */
        memcpy(bufp, &rx_buf[1], len);
        return 0;
    }
    
    return -1;
}

/**
 * @brief Delay function for ST driver
 */
static void platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t ism330dhcxInit(Ism330dhcxHandle_t *handle)
{
    Ism330dhcxPins_t default_pins = {
        .miso = ISM330DHCX_PIN_MISO,
        .mosi = ISM330DHCX_PIN_MOSI,
        .sclk = ISM330DHCX_PIN_SCLK,
        .cs = ISM330DHCX_PIN_CS,
        .int1 = ISM330DHCX_PIN_INT1,
        .int2 = ISM330DHCX_PIN_INT2
    };
    
    return ism330dhcxInitPins(handle, &default_pins);
}

int8_t ism330dhcxInitPins(Ism330dhcxHandle_t *handle, const Ism330dhcxPins_t *pins)
{
    if (handle == NULL || pins == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    esp_err_t ret;
    
    /* Store pin configuration */
    handle->pins = *pins;
    handle->is_initialized = 0;
    
    /*=========================================================================
     * Initialize SPI Bus
     *=========================================================================
     */
    spi_bus_config_t bus_cfg = {
        .miso_io_num = pins->miso,
        .mosi_io_num = pins->mosi,
        .sclk_io_num = pins->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256
    };
    
    ret = spi_bus_initialize(ISM330DHCX_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means bus already initialized - that's OK */
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ISM330DHCX_ERR_SPI;
    }
    
    /*=========================================================================
     * Add SPI Device
     *=========================================================================
     * ISM330DHCX SPI requirements:
     * - Mode 3: CPOL=1, CPHA=1 (clock idle high, sample on falling edge)
     * - Max 10 MHz
     * - MSB first
     */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ISM330DHCX_SPI_FREQ_HZ,
        .mode = 3,                      /* SPI Mode 3 (CPOL=1, CPHA=1) */
        .spics_io_num = pins->cs,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    
    ret = spi_bus_add_device(ISM330DHCX_SPI_HOST, &dev_cfg, &handle->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ISM330DHCX_ERR_SPI;
    }
    
    /*=========================================================================
     * Initialize ST Driver Context
     *=========================================================================
     */
    handle->dev_ctx.write_reg = platform_write;
    handle->dev_ctx.read_reg = platform_read;
    handle->dev_ctx.mdelay = platform_delay;
    handle->dev_ctx.handle = handle;    /* Pass our handle for callbacks */
    
    /* Wait for sensor boot */
    platform_delay(100);
    
    /*=========================================================================
     * Verify Device ID
     *=========================================================================
     */
    uint8_t who_am_i;
    ism330dhcx_device_id_get(&handle->dev_ctx, &who_am_i);
    
    if (who_am_i != ISM330DHCX_ID) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X", 
                 ISM330DHCX_ID, who_am_i);
        return ISM330DHCX_ERR_WHO_AM_I;
    }
    
    ESP_LOGI(TAG, "ISM330DHCX detected (WHO_AM_I: 0x%02X)", who_am_i);
    
    /*=========================================================================
     * Reset Device
     *=========================================================================
     */
    ism330dhcx_reset_set(&handle->dev_ctx, PROPERTY_ENABLE);
    
    uint8_t rst;
    do {
        ism330dhcx_reset_get(&handle->dev_ctx, &rst);
        platform_delay(1);
    } while (rst);
    
    platform_delay(100);
    
    handle->is_initialized = 1;
    ESP_LOGI(TAG, "ISM330DHCX initialized successfully");
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxConfigure(Ism330dhcxHandle_t *handle, const Ism330dhcxConfig_t *config)
{
    if (handle == NULL || config == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    stmdev_ctx_t *ctx = &handle->dev_ctx;
    
    /* Set accelerometer ODR and full scale */
    ism330dhcx_xl_data_rate_set(ctx, config->accel_odr);
    ism330dhcx_xl_full_scale_set(ctx, config->accel_fs);
    
    /* Set gyroscope ODR and full scale */
    ism330dhcx_gy_data_rate_set(ctx, config->gyro_odr);
    ism330dhcx_gy_full_scale_set(ctx, config->gyro_fs);
    
    /* Update sensitivity based on full scale */
    switch (config->accel_fs) {
        case ISM330DHCX_2g:  accel_sensitivity = 0.061f; break;
        case ISM330DHCX_4g:  accel_sensitivity = 0.122f; break;
        case ISM330DHCX_8g:  accel_sensitivity = 0.244f; break;
        case ISM330DHCX_16g: accel_sensitivity = 0.488f; break;
        default: accel_sensitivity = 0.244f; break;
    }
    
    switch (config->gyro_fs) {
        case ISM330DHCX_125dps:  gyro_sensitivity = 4.375f; break;
        case ISM330DHCX_250dps:  gyro_sensitivity = 8.75f; break;
        case ISM330DHCX_500dps:  gyro_sensitivity = 17.5f; break;
        case ISM330DHCX_1000dps: gyro_sensitivity = 35.0f; break;
        case ISM330DHCX_2000dps: gyro_sensitivity = 70.0f; break;
        case ISM330DHCX_4000dps: gyro_sensitivity = 140.0f; break;
        default: gyro_sensitivity = 70.0f; break;
    }
    
    ESP_LOGI(TAG, "Configured: Accel sens=%.3f mg/LSB, Gyro sens=%.2f mdps/LSB",
             accel_sensitivity, gyro_sensitivity);
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxConfigureFlightController(Ism330dhcxHandle_t *handle)
{
    if (handle == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }

    stmdev_ctx_t *ctx = &handle->dev_ctx;
    ESP_LOGI("STACK", "Free stack: %u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
    ESP_LOGI(TAG, "Configuring for flight controller...");

    /*=========================================================================
     * Basic Configuration
     *=========================================================================
     */
    /* Enable device configuration */
    ism330dhcx_device_conf_set(&handle->dev_ctx, PROPERTY_ENABLE);
    
    /* Enable Block Data Update (prevent reading partial data) */
    ism330dhcx_block_data_update_set(&handle->dev_ctx, PROPERTY_ENABLE);
    //ism330dhcx_device_conf_set(&handle->dev_ctx, PROPERTY_DISABLE);
    platform_delay(100);
    /*=========================================================================
     * Accelerometer Configuration
     *=========================================================================
     * - ODR: 1666 Hz (matching gyro, good for EKF)
     * - Full Scale: ±8g (covers aggressive maneuvers)
     * - LPF2: Enabled for noise reduction
     */

    ism330dhcx_xl_data_rate_set(&handle->dev_ctx, ISM330DHCX_XL_ODR_1666Hz);
    ism330dhcx_gy_data_rate_set(&handle->dev_ctx, ISM330DHCX_GY_ODR_1666Hz);
    /* Set full scale */
    ism330dhcx_xl_full_scale_set(&handle->dev_ctx, ISM330DHCX_8g);
    ism330dhcx_gy_full_scale_set(&handle->dev_ctx, ISM330DHCX_2000dps);
    /* Configure filtering chain(No aux interface)
    *
    * Accelerometer - LPF1 + LPF2 path
    */
    ism330dhcx_xl_hp_path_on_out_set(&handle->dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
    ism330dhcx_xl_filter_lp2_set(&handle->dev_ctx, PROPERTY_ENABLE);



    /*=========================================================================
     * Gyroscope Configuration
     *=========================================================================
     * - ODR: 1666 Hz (good balance of speed and noise)
     * - Full Scale: ±2000 dps (covers aggressive flight)
     * - HPF: Enabled for bias rejection
     */
 
    /* Enable gyro high-pass filter */
    
    //ism330dhcx_gy_hp_path_internal_set(handle->dev_ctx, ISM330DHCX_HP_FILTER_16mHz);
    
    /* Update sensitivities */
    accel_sensitivity = 0.244f;     /* ±8g: 0.244 mg/LSB */
    gyro_sensitivity = 70.0f;       /* ±2000dps: 70 mdps/LSB */
    
    ESP_LOGI(TAG, "Flight controller config: 1666Hz, ±8g, ±2000dps");
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxDeinit(Ism330dhcxHandle_t *handle)
{
    if (handle == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (handle->is_initialized) {
        /* Power down sensors */
        ism330dhcx_xl_data_rate_set(&handle->dev_ctx, ISM330DHCX_XL_ODR_OFF);
        ism330dhcx_gy_data_rate_set(&handle->dev_ctx, ISM330DHCX_GY_ODR_OFF);
        
        /* Remove SPI device */
        spi_bus_remove_device(handle->spi_handle);
        
        handle->is_initialized = 0;
    }
    
    return ISM330DHCX_OK;
}

/*******************************************************************************
 * Data Reading Functions
 ******************************************************************************/

// int8_t ism330dhcxReadRaw(Ism330dhcxHandle_t *handle, Ism330dhcxRawData_t *data)
// {
//     if (handle == NULL || data == NULL) {
//         return ISM330DHCX_ERR_NULL_PTR;
//     }
    
//     if (!handle->is_initialized) {
//         return ISM330DHCX_ERR_NOT_INIT;
//     }
    
//     stmdev_ctx_t *ctx = &handle->dev_ctx;
    
//     /* Read accelerometer */
//     ism330dhcx_acceleration_raw_get(ctx, data->accel);
    
//     /* Read gyroscope */
//     ism330dhcx_angular_rate_raw_get(ctx, data->gyro);
    
//     /* Read temperature */
//ism330dhcx_temp_flag_data_ready_get(&handle->dev_ctx, &data->temp_flag);
//     ism330dhcx_temperature_raw_get(ctx, &data->temp);
    
//     return ISM330DHCX_OK;
// }

int8_t ism330dhcxReadRaw(Ism330dhcxHandle_t *handle, Ism330dhcxRawData_t *data)
{
    if (handle == NULL || data == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    /*=========================================================================
     * Burst Read: Tüm verileri tek SPI transaction'da oku
     *=========================================================================
     * OUTX_L_G (0x22) -> OUTZ_H_XL (0x2D) = 12 byte
     * Gyro XYZ (6 byte) + Accel XYZ (6 byte)
     */
    uint8_t raw_buf[12];
    
    /* 0x22 adresinden 12 byte oku (gyro + accel) */
    uint8_t accel_ready, gyro_ready;
    // ism330dhcxDataReady(handle, &accel_ready, &gyro_ready);
    // if (accel_ready == 0 || gyro_ready == 0)
    // {
    //     return -1;
    // }
    
    platform_read(handle, 0x22, raw_buf, 12);
    
    /* Parse: Little endian (LSB first) */
    data->gyro[0] = (int16_t)(raw_buf[1] << 8 | raw_buf[0]);   /* OUTX_G */
    data->gyro[1] = (int16_t)(raw_buf[3] << 8 | raw_buf[2]);   /* OUTY_G */
    data->gyro[2] = (int16_t)(raw_buf[5] << 8 | raw_buf[4]);   /* OUTZ_G */
    
    data->accel[0] = (int16_t)(raw_buf[7] << 8 | raw_buf[6]);  /* OUTX_XL */
    data->accel[1] = (int16_t)(raw_buf[9] << 8 | raw_buf[8]);  /* OUTY_XL */
    data->accel[2] = (int16_t)(raw_buf[11] << 8 | raw_buf[10]);/* OUTZ_XL */
    
    /* Temperature ayrı okunabilir, kritik değil */
    //ism330dhcx_temperature_raw_get(&handle->dev_ctx, &data->temp);
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxRead(Ism330dhcxHandle_t *handle, Ism330dhcxData_t *data)
{
    if (handle == NULL || data == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    Ism330dhcxRawData_t raw;
    int8_t ret = ism330dhcxReadRaw(handle, &raw);
    
    if (ret != ISM330DHCX_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Convert Accelerometer Data
     *=========================================================================
     * Raw value * sensitivity (mg/LSB) / 1000 = g
     */
    data->accel_g[0] = (float)raw.accel[0] * accel_sensitivity / 1000.0f;
    data->accel_g[1] = (float)raw.accel[1] * accel_sensitivity / 1000.0f;
    data->accel_g[2] = (float)raw.accel[2] * accel_sensitivity / 1000.0f;
    
    /*=========================================================================
     * Convert Gyroscope Data
     *=========================================================================
     * Raw value * sensitivity (mdps/LSB) / 1000 = dps
     * dps * PI / 180 = rad/s
     */
    data->gyro_dps[0] = (float)raw.gyro[0] * gyro_sensitivity / 1000.0f;
    data->gyro_dps[1] = (float)raw.gyro[1] * gyro_sensitivity / 1000.0f;
    data->gyro_dps[2] = (float)raw.gyro[2] * gyro_sensitivity / 1000.0f;
    
    data->gyro_rps[0] = data->gyro_dps[0] * (M_PI / 180.0f);
    data->gyro_rps[1] = data->gyro_dps[1] * (M_PI / 180.0f);
    data->gyro_rps[2] = data->gyro_dps[2] * (M_PI / 180.0f);
    
    /*=========================================================================
     * Convert Temperature
     *=========================================================================
     * Temp (°C) = raw / 256 + 25
     */
    data->temp_c = (float)raw.temp / 256.0f + 25.0f;
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxReadAccel(Ism330dhcxHandle_t *handle, float *accel_g)
{
    if (handle == NULL || accel_g == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    int16_t raw[3];
    ism330dhcx_acceleration_raw_get(&handle->dev_ctx, raw);
    
    accel_g[0] = (float)raw[0] * accel_sensitivity / 1000.0f;
    accel_g[1] = (float)raw[1] * accel_sensitivity / 1000.0f;
    accel_g[2] = (float)raw[2] * accel_sensitivity / 1000.0f;
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxReadGyro(Ism330dhcxHandle_t *handle, float *gyro_dps)
{
    if (handle == NULL || gyro_dps == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    int16_t raw[3];
    ism330dhcx_angular_rate_raw_get(&handle->dev_ctx, raw);
    
    gyro_dps[0] = (float)raw[0] * gyro_sensitivity / 1000.0f;
    gyro_dps[1] = (float)raw[1] * gyro_sensitivity / 1000.0f;
    gyro_dps[2] = (float)raw[2] * gyro_sensitivity / 1000.0f;
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxReadGyroRad(Ism330dhcxHandle_t *handle, float *gyro_rps)
{
    if (handle == NULL || gyro_rps == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    float gyro_dps[3];
    int8_t ret = ism330dhcxReadGyro(handle, gyro_dps);
    
    if (ret != ISM330DHCX_OK) {
        return ret;
    }
    
    gyro_rps[0] = gyro_dps[0] * (M_PI / 180.0f);
    gyro_rps[1] = gyro_dps[1] * (M_PI / 180.0f);
    gyro_rps[2] = gyro_dps[2] * (M_PI / 180.0f);
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxDataReady(Ism330dhcxHandle_t *handle, 
                           uint8_t *accel_ready, 
                           uint8_t *gyro_ready)
{
    if (handle == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    stmdev_ctx_t *ctx = &handle->dev_ctx;
    
    while(accel_ready == 0)
    {
        ism330dhcx_xl_flag_data_ready_get(ctx, accel_ready);
    }
    
    while(gyro_ready == 0)
    {
        ism330dhcx_gy_flag_data_ready_get(ctx, gyro_ready);
    }
    
    
    return ISM330DHCX_OK;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

int8_t ism330dhcxWhoAmI(Ism330dhcxHandle_t *handle, uint8_t *who_am_i)
{
    if (handle == NULL || who_am_i == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    ism330dhcx_device_id_get(&handle->dev_ctx, who_am_i);
    
    return ISM330DHCX_OK;
}

int8_t ism330dhcxReset(Ism330dhcxHandle_t *handle)
{
    if (handle == NULL) {
        return ISM330DHCX_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return ISM330DHCX_ERR_NOT_INIT;
    }
    
    ism330dhcx_reset_set(&handle->dev_ctx, PROPERTY_ENABLE);
    
    uint8_t rst;
    do {
        ism330dhcx_reset_get(&handle->dev_ctx, &rst);
        platform_delay(1);
    } while (rst);
    
    return ISM330DHCX_OK;
}

stmdev_ctx_t* ism330dhcxGetCtx(Ism330dhcxHandle_t *handle)
{
    if (handle == NULL) {
        return NULL;
    }
    
    return &handle->dev_ctx;
}
