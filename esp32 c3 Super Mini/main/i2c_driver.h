#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* ============================================================================
 * I2C Configuration
 * ============================================================================ */
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_IO       6
#define I2C_MASTER_SCL_IO       7
#define I2C_MASTER_FREQ_HZ      100000      /* 100kHz standard mode */
#define I2C_MASTER_TIMEOUT_MS   1000

/* ============================================================================
 * I2C Driver Handle (opaque)
 * ============================================================================ */
typedef struct {
    i2c_master_bus_handle_t bus_handle;
    uint8_t                 is_initialized;
} i2c_driver_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize I2C master bus
 * 
 * @param driver Pointer to driver structure
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cDriverInit(i2c_driver_t *driver);

/**
 * @brief Deinitialize I2C master bus
 * 
 * @param driver Pointer to driver structure
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cDriverDeinit(i2c_driver_t *driver);

/**
 * @brief Add a device to the I2C bus
 * 
 * @param driver Pointer to driver structure
 * @param device_addr 7-bit device address
 * @param dev_handle Pointer to store device handle
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cDriverAddDevice(i2c_driver_t *driver, uint8_t device_addr, 
                          i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Remove a device from the I2C bus
 * 
 * @param dev_handle Device handle to remove
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cDriverRemoveDevice(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Write single byte to register
 * 
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param data Data byte to write
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cWriteByte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief Read single byte from register
 * 
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param data Pointer to store read data
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cReadByte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Read multiple bytes from register
 * 
 * @param dev_handle Device handle
 * @param reg_addr Starting register address
 * @param data Pointer to buffer for read data
 * @param len Number of bytes to read
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cReadBytes(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, 
                    uint8_t *data, uint16_t len);

/**
 * @brief Write multiple bytes to register
 * 
 * @param dev_handle Device handle
 * @param reg_addr Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return int8_t 0 on success, negative on error
 */
int8_t i2cWriteBytes(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                     uint8_t *data, uint16_t len);

#endif /* I2C_DRIVER_H */
