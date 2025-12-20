/**
 * @file fc_error.h
 * @brief Centralized error codes for Flight Controller
 * @details Unified error handling across all modules
 */

#ifndef FC_ERROR_H
#define FC_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Error Code Categories
 * 
 * 0:        Success
 * -1 to -9: Generic errors
 * -10 to -19: I2C errors
 * -20 to -29: Sensor errors
 * -30 to -39: Control errors
 * -40 to -49: Communication errors
 * -50 to -59: Storage errors
 ******************************************************************************/

typedef enum {
    /* Success */
    FC_OK = 0,
    
    /* Generic errors (-1 to -9) */
    FC_ERR_NULL_PTR         = -1,   /**< Null pointer passed */
    FC_ERR_INVALID_PARAM    = -2,   /**< Invalid parameter */
    FC_ERR_NOT_INITIALIZED  = -3,   /**< Module not initialized */
    FC_ERR_ALREADY_INIT     = -4,   /**< Module already initialized */
    FC_ERR_TIMEOUT          = -5,   /**< Operation timed out */
    FC_ERR_BUSY             = -6,   /**< Resource busy */
    FC_ERR_NO_MEMORY        = -7,   /**< Out of memory */
    FC_ERR_NOT_SUPPORTED    = -8,   /**< Operation not supported */
    FC_ERR_UNKNOWN          = -9,   /**< Unknown error */
    
    /* I2C errors (-10 to -19) */
    FC_ERR_I2C_INIT         = -10,  /**< I2C initialization failed */
    FC_ERR_I2C_TIMEOUT      = -11,  /**< I2C timeout */
    FC_ERR_I2C_NACK         = -12,  /**< I2C device not responding */
    FC_ERR_I2C_WRITE        = -13,  /**< I2C write failed */
    FC_ERR_I2C_READ         = -14,  /**< I2C read failed */
    FC_ERR_I2C_BUS          = -15,  /**< I2C bus error */
    FC_ERR_I2C_BUFFER       = -16,  /**< I2C buffer overflow */
    
    /* Sensor errors (-20 to -29) */
    FC_ERR_SENSOR_NOT_FOUND = -20,  /**< Sensor not detected */
    FC_ERR_SENSOR_WHO_AM_I  = -21,  /**< Wrong WHO_AM_I value */
    FC_ERR_SENSOR_CONFIG    = -22,  /**< Sensor configuration failed */
    FC_ERR_SENSOR_CALIB     = -23,  /**< Sensor calibration failed */
    FC_ERR_SENSOR_READ      = -24,  /**< Sensor read failed */
    FC_ERR_SENSOR_OVERFLOW  = -25,  /**< Sensor data overflow */
    
    /* Control errors (-30 to -39) */
    FC_ERR_CTRL_NOT_INIT    = -30,  /**< Controller not initialized */
    FC_ERR_CTRL_SATURATED   = -31,  /**< Controller output saturated */
    FC_ERR_CTRL_DIVERGED    = -32,  /**< Controller diverged */
    FC_ERR_CTRL_MATRIX      = -33,  /**< Matrix operation failed */
    FC_ERR_CTRL_SINGULAR    = -34,  /**< Singular matrix (inversion failed) */
    FC_ERR_CTRL_COMPUTE     = -35,  /**< Computation error */
    
    /* Communication errors (-40 to -49) */
    FC_ERR_COMM_DISCONN     = -40,  /**< Communication disconnected */
    FC_ERR_COMM_TIMEOUT     = -41,  /**< Communication timeout */
    FC_ERR_COMM_PARSE       = -42,  /**< Parse error */
    FC_ERR_COMM_CRC         = -43,  /**< CRC error */
    FC_ERR_COMM_AUTH        = -44,  /**< Authentication failed */
    FC_ERR_COMM_ENCRYPT     = -45,  /**< Encryption error */
    
    /* Storage errors (-50 to -59) */
    FC_ERR_NVS_OPEN         = -50,  /**< NVS open failed */
    FC_ERR_NVS_READ         = -51,  /**< NVS read failed */
    FC_ERR_NVS_WRITE        = -52,  /**< NVS write failed */
    FC_ERR_NVS_ERASE        = -53,  /**< NVS erase failed */
    
} fc_error_t;

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Check if error code is success
 * @param err Error code
 * @return 1 if success, 0 otherwise
 */
static inline int fc_is_ok(fc_error_t err) {
    return (err == FC_OK);
}

/**
 * @brief Check if error code is an error
 * @param err Error code
 * @return 1 if error, 0 if success
 */
static inline int fc_is_error(fc_error_t err) {
    return (err != FC_OK);
}

/**
 * @brief Get error category
 * @param err Error code
 * @return Category string
 */
static inline const char* fc_error_category(fc_error_t err) {
    if (err == FC_OK) return "OK";
    if (err >= -9)  return "GENERIC";
    if (err >= -19) return "I2C";
    if (err >= -29) return "SENSOR";
    if (err >= -39) return "CONTROL";
    if (err >= -49) return "COMM";
    if (err >= -59) return "STORAGE";
    return "UNKNOWN";
}

/**
 * @brief Get error description string
 * @param err Error code
 * @return Description string
 */
const char* fc_error_str(fc_error_t err);

#ifdef __cplusplus
}
#endif

#endif /* FC_ERROR_H */
