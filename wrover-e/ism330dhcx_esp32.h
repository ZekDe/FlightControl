/**
 * @file ism330dhcx_esp32.h
 * @brief ESP32 SPI platform layer for ISM330DHCX IMU
 * 
 * @details Provides ESP32-IDF SPI driver integration for ISM330DHCX.
 *          Designed for flight controller applications with high-speed
 *          data acquisition (up to 6.667 kHz ODR).
 * 
 * @note SPI Mode 3 (CPOL=1, CPHA=1) as per ISM330DHCX datasheet
 * @note Max SPI clock: 10 MHz
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef ISM330DHCX_ESP32_H
#define ISM330DHCX_ESP32_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ism330dhcx_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Default SPI configuration
 * @note Adjust pins according to your hardware
 */
#define ISM330DHCX_SPI_HOST         SPI2_HOST       /**< SPI peripheral (HSPI) */
#define ISM330DHCX_SPI_FREQ_HZ      1000000        /**< 10 MHz SPI clock */

/* Default pin assignments - CHANGE THESE FOR YOUR BOARD */
#define ISM330DHCX_PIN_MISO         GPIO_NUM_19     /**< SPI MISO (SDO) */
#define ISM330DHCX_PIN_MOSI         GPIO_NUM_23     /**< SPI MOSI (SDI) */
#define ISM330DHCX_PIN_SCLK         GPIO_NUM_18     /**< SPI Clock */
#define ISM330DHCX_PIN_CS           GPIO_NUM_5      /**< Chip Select */
#define ISM330DHCX_PIN_INT1         GPIO_NUM_4      /**< Interrupt 1 (optional) */
#define ISM330DHCX_PIN_INT2         GPIO_NUM_2      /**< Interrupt 2 (optional) */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief ISM330DHCX SPI pin configuration
 */
typedef struct {
    gpio_num_t miso;        /**< MISO pin (Master In, Slave Out) */
    gpio_num_t mosi;        /**< MOSI pin (Master Out, Slave In) */
    gpio_num_t sclk;        /**< SPI Clock pin */
    gpio_num_t cs;          /**< Chip Select pin */
    gpio_num_t int1;        /**< INT1 pin (-1 if not used) */
    gpio_num_t int2;        /**< INT2 pin (-1 if not used) */
} Ism330dhcxPins_t;

/**
 * @brief ISM330DHCX device handle
 */
typedef struct {
    spi_device_handle_t spi_handle;     /**< ESP-IDF SPI device handle */
    stmdev_ctx_t dev_ctx;               /**< ST driver context */
    Ism330dhcxPins_t pins;              /**< Pin configuration */
    uint8_t is_initialized;             /**< Initialization flag */
} Ism330dhcxHandle_t;

/**
 * @brief Sensor configuration
 */
typedef struct {
    ism330dhcx_odr_xl_t accel_odr;      /**< Accelerometer output data rate */
    ism330dhcx_fs_xl_t accel_fs;        /**< Accelerometer full scale */
    ism330dhcx_odr_g_t gyro_odr;        /**< Gyroscope output data rate */
    ism330dhcx_fs_g_t gyro_fs;          /**< Gyroscope full scale */
} Ism330dhcxConfig_t;

/**
 * @brief Raw sensor data (16-bit signed)
 */
typedef struct {
    int16_t accel[3];       /**< Accelerometer [x, y, z] raw */
    int16_t gyro[3];        /**< Gyroscope [x, y, z] raw */
    int16_t temp;           /**< Temperature raw */
} Ism330dhcxRawData_t;

/**
 * @brief Converted sensor data (float)
 */
typedef struct {
    float accel_g[3];       /**< Accelerometer [x, y, z] in g */
    float gyro_dps[3];      /**< Gyroscope [x, y, z] in deg/s */
    float gyro_rps[3];      /**< Gyroscope [x, y, z] in rad/s */
    float temp_c;           /**< Temperature in °C */
} Ism330dhcxData_t;

/*******************************************************************************
 * Error Codes
 ******************************************************************************/

#define ISM330DHCX_OK               0
#define ISM330DHCX_ERR_NULL_PTR    -1
#define ISM330DHCX_ERR_SPI         -2
#define ISM330DHCX_ERR_WHO_AM_I    -3
#define ISM330DHCX_ERR_CONFIG      -4
#define ISM330DHCX_ERR_NOT_INIT    -5

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize ISM330DHCX with default pins
 * 
 * @param[out] handle   Device handle
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxInit(Ism330dhcxHandle_t *handle);

/**
 * @brief Initialize ISM330DHCX with custom pins
 * 
 * @param[out] handle   Device handle
 * @param[in]  pins     Pin configuration
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxInitPins(Ism330dhcxHandle_t *handle, const Ism330dhcxPins_t *pins);

/**
 * @brief Configure sensor ODR and full scale
 * 
 * @param[in,out] handle    Device handle
 * @param[in]     config    Sensor configuration
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxConfigure(Ism330dhcxHandle_t *handle, const Ism330dhcxConfig_t *config);

/**
 * @brief Configure for flight controller (1666 Hz, ±2000 dps, ±8g)
 * 
 * @param[in,out] handle    Device handle
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxConfigureFlightController(Ism330dhcxHandle_t *handle);

/**
 * @brief Deinitialize and release resources
 * 
 * @param[in,out] handle    Device handle
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxDeinit(Ism330dhcxHandle_t *handle);

/*******************************************************************************
 * Data Reading Functions
 ******************************************************************************/

/**
 * @brief Read raw sensor data
 * 
 * @param[in]  handle   Device handle
 * @param[out] data     Raw data output
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxReadRaw(Ism330dhcxHandle_t *handle, Ism330dhcxRawData_t *data);

/**
 * @brief Read and convert sensor data
 * 
 * @param[in]  handle   Device handle
 * @param[out] data     Converted data output
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxRead(Ism330dhcxHandle_t *handle, Ism330dhcxData_t *data);

/**
 * @brief Read only accelerometer
 * 
 * @param[in]  handle   Device handle
 * @param[out] accel_g  Accelerometer data [x, y, z] in g
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxReadAccel(Ism330dhcxHandle_t *handle, float *accel_g);

/**
 * @brief Read only gyroscope
 * 
 * @param[in]  handle   Device handle
 * @param[out] gyro_dps Gyroscope data [x, y, z] in deg/s
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxReadGyro(Ism330dhcxHandle_t *handle, float *gyro_dps);

/**
 * @brief Read gyroscope in rad/s
 * 
 * @param[in]  handle   Device handle
 * @param[out] gyro_rps Gyroscope data [x, y, z] in rad/s
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxReadGyroRad(Ism330dhcxHandle_t *handle, float *gyro_rps);

/**
 * @brief Check if new data is available
 * 
 * @param[in]  handle       Device handle
 * @param[out] accel_ready  1 if new accel data available
 * @param[out] gyro_ready   1 if new gyro data available
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxDataReady(Ism330dhcxHandle_t *handle, 
                           uint8_t *accel_ready, 
                           uint8_t *gyro_ready);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Get WHO_AM_I register value
 * 
 * @param[in]  handle   Device handle
 * @param[out] who_am_i WHO_AM_I value (should be 0x6B)
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxWhoAmI(Ism330dhcxHandle_t *handle, uint8_t *who_am_i);

/**
 * @brief Software reset
 * 
 * @param[in,out] handle    Device handle
 * @return ISM330DHCX_OK on success, error code otherwise
 */
int8_t ism330dhcxReset(Ism330dhcxHandle_t *handle);

/**
 * @brief Get ST driver context (for advanced usage)
 * 
 * @param[in] handle    Device handle
 * @return Pointer to stmdev_ctx_t
 */
stmdev_ctx_t* ism330dhcxGetCtx(Ism330dhcxHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* ISM330DHCX_ESP32_H */
