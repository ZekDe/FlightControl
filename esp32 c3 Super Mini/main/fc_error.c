/**
 * @file fc_error.c
 * @brief Error code string implementations
 */

#include "fc_error.h"

const char* fc_error_str(fc_error_t err)
{
    switch (err) {
        case FC_OK:                 return "Success";
        
        /* Generic */
        case FC_ERR_NULL_PTR:       return "Null pointer";
        case FC_ERR_INVALID_PARAM:  return "Invalid parameter";
        case FC_ERR_NOT_INITIALIZED:return "Not initialized";
        case FC_ERR_ALREADY_INIT:   return "Already initialized";
        case FC_ERR_TIMEOUT:        return "Timeout";
        case FC_ERR_BUSY:           return "Busy";
        case FC_ERR_NO_MEMORY:      return "Out of memory";
        case FC_ERR_NOT_SUPPORTED:  return "Not supported";
        case FC_ERR_UNKNOWN:        return "Unknown error";
        
        /* I2C */
        case FC_ERR_I2C_INIT:       return "I2C init failed";
        case FC_ERR_I2C_TIMEOUT:    return "I2C timeout";
        case FC_ERR_I2C_NACK:       return "I2C NACK";
        case FC_ERR_I2C_WRITE:      return "I2C write failed";
        case FC_ERR_I2C_READ:       return "I2C read failed";
        case FC_ERR_I2C_BUS:        return "I2C bus error";
        case FC_ERR_I2C_BUFFER:     return "I2C buffer overflow";
        
        /* Sensor */
        case FC_ERR_SENSOR_NOT_FOUND: return "Sensor not found";
        case FC_ERR_SENSOR_WHO_AM_I:  return "Wrong WHO_AM_I";
        case FC_ERR_SENSOR_CONFIG:    return "Sensor config failed";
        case FC_ERR_SENSOR_CALIB:     return "Calibration failed";
        case FC_ERR_SENSOR_READ:      return "Sensor read failed";
        case FC_ERR_SENSOR_OVERFLOW:  return "Sensor overflow";
        
        /* Control */
        case FC_ERR_CTRL_NOT_INIT:  return "Controller not init";
        case FC_ERR_CTRL_SATURATED: return "Controller saturated";
        case FC_ERR_CTRL_DIVERGED:  return "Controller diverged";
        case FC_ERR_CTRL_MATRIX:    return "Matrix op failed";
        case FC_ERR_CTRL_SINGULAR:  return "Singular matrix";
        case FC_ERR_CTRL_COMPUTE:   return "Computation error";
        
        /* Communication */
        case FC_ERR_COMM_DISCONN:   return "Disconnected";
        case FC_ERR_COMM_TIMEOUT:   return "Comm timeout";
        case FC_ERR_COMM_PARSE:     return "Parse error";
        case FC_ERR_COMM_CRC:       return "CRC error";
        case FC_ERR_COMM_AUTH:      return "Auth failed";
        case FC_ERR_COMM_ENCRYPT:   return "Encryption error";
        
        /* Storage */
        case FC_ERR_NVS_OPEN:       return "NVS open failed";
        case FC_ERR_NVS_READ:       return "NVS read failed";
        case FC_ERR_NVS_WRITE:      return "NVS write failed";
        case FC_ERR_NVS_ERASE:      return "NVS erase failed";
        
        default:                    return "Unknown";
    }
}
