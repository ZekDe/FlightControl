#include "i2c_driver.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "I2C_DRV";

/* ============================================================================
 * I2C Bus Initialization
 * ============================================================================ */
int8_t i2cDriverInit(i2c_driver_t *driver)
{
    if (driver == NULL) {
        ESP_LOGE(TAG, "Driver pointer is NULL");
        return -1;
    }

    if (driver->is_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return 0;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false,  /* internal pull-ups used */
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &driver->bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return -2;
    }

    driver->is_initialized = 1;
    ESP_LOGI(TAG, "I2C master initialized on port %d (SDA:%d, SCL:%d, %dHz)",
             I2C_MASTER_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    return 0;
}

/* ============================================================================
 * I2C Bus Deinitialization
 * ============================================================================ */
int8_t i2cDriverDeinit(i2c_driver_t *driver)
{
    if (driver == NULL) {
        return -1;
    }

    if (!driver->is_initialized) {
        return 0;
    }

    esp_err_t ret = i2c_del_master_bus(driver->bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
        return -2;
    }

    driver->is_initialized = 0;
    driver->bus_handle = NULL;
    ESP_LOGI(TAG, "I2C master deinitialized");

    return 0;
}

/* ============================================================================
 * Add Device to I2C Bus
 * ============================================================================ */
int8_t i2cDriverAddDevice(i2c_driver_t *driver, uint8_t device_addr,
                          i2c_master_dev_handle_t *dev_handle)
{
    if (driver == NULL || dev_handle == NULL) {
        return -1;
    }

    if (!driver->is_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return -2;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(driver->bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", device_addr, esp_err_to_name(ret));
        return -3;
    }

    ESP_LOGI(TAG, "Device 0x%02X added to I2C bus", device_addr);
    return 0;
}

/* ============================================================================
 * Remove Device from I2C Bus
 * ============================================================================ */
int8_t i2cDriverRemoveDevice(i2c_master_dev_handle_t dev_handle)
{
    if (dev_handle == NULL) {
        return -1;
    }

    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        return -2;
    }

    return 0;
}

/* ============================================================================
 * Write Single Byte to Register
 * ============================================================================ */
int8_t i2cWriteByte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    if (dev_handle == NULL) {
        return -1;
    }

    uint8_t write_buf[2] = {reg_addr, data};
    
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                                        I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write byte failed (reg 0x%02X): %s", reg_addr, esp_err_to_name(ret));
        return -2;
    }

    return 0;
}

/* ============================================================================
 * Read Single Byte from Register
 * ============================================================================ */
int8_t i2cReadByte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data)
{
    if (dev_handle == NULL || data == NULL) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, 1,
                                                 I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read byte failed (reg 0x%02X): %s", reg_addr, esp_err_to_name(ret));
        return -2;
    }

    return 0;
}

/* ============================================================================
 * Read Multiple Bytes from Register
 * ============================================================================ */
int8_t i2cReadBytes(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                    uint8_t *data, uint16_t len)
{
    if (dev_handle == NULL || data == NULL || len == 0) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len,
                                                 I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read bytes failed (reg 0x%02X, len %d): %s", 
                 reg_addr, len, esp_err_to_name(ret));
        return -2;
    }

    return 0;
}

/* ============================================================================
 * Write Multiple Bytes to Register
 * ============================================================================ */
#define I2C_MAX_WRITE_LEN   64  /* Maximum write buffer size */

int8_t i2cWriteBytes(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                     uint8_t *data, uint16_t len)
{
    if (dev_handle == NULL || data == NULL || len == 0) {
        return -1;
    }

    /* Check buffer size limit to prevent stack overflow */
    if (len + 1 > I2C_MAX_WRITE_LEN) {
        ESP_LOGE(TAG, "Write length %d exceeds max %d", len, I2C_MAX_WRITE_LEN - 1);
        return -3;
    }

    /* Create buffer with register address + data (static size) */
    uint8_t write_buf[I2C_MAX_WRITE_LEN];
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);

    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, len + 1,
                                        I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write bytes failed (reg 0x%02X, len %d): %s",
                 reg_addr, len, esp_err_to_name(ret));
        return -2;
    }

    return 0;
}
