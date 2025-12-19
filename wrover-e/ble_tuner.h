/**
 * @file ble_tuner.h
 * @brief BLE PID Tuner for Quadcopter Flight Controller
 * @author Claude & duatepe
 * @date 2024
 * 
 * @details Nordic UART Service (NUS) based BLE interface for:
 *          - Real-time PID gain tuning
 *          - Flight control commands (ARM, DISARM, throttle, etc.)
 *          - Telemetry streaming (attitude, motors, gyro)
 *          - Connection watchdog with auto-disarm
 * 
 * Compatible with:
 *          - nRF Connect for Mobile
 *          - Any NUS-compatible BLE terminal
 *          - ESP32-WROVER-E and ESP32-C3
 * 
 * Command Format (ASCII):
 *          ARM, DISARM, THR=15.5, ROLL=5.0, PITCH=-3.0, YAW=10.0
 *          RATE_RP=Kp,Ki,Kd    (e.g., RATE_RP=0.25,0.30,0.003)
 *          RATE_YAW=Kp,Ki,Kd
 *          ATT_RP=Kp,Ki,Kd
 *          TEL=1/0, GET, HB
 * 
 * Telemetry Format:
 *          $ATT,roll,pitch,yaw*
 *          $MOT,m1,m2,m3,m4*
 *          $GYR,gx,gy,gz*
 */

#ifndef BLE_TUNER_H
#define BLE_TUNER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define BLE_TUNER_DEVICE_NAME_MAX       32      /**< Max device name length */
#define BLE_TUNER_CMD_MAX_LEN           64      /**< Max command length */
#define BLE_TUNER_TX_MAX_LEN            128     /**< Max TX packet length */

#define BLE_TUNER_HB_TIMEOUT_MS         2000    /**< Heartbeat timeout (ms) */
#define BLE_TUNER_TELEMETRY_PERIOD_MS   100     /**< Telemetry period (10 Hz) */

/*******************************************************************************
 * Command IDs
 ******************************************************************************/

typedef enum {
    BLE_CMD_UNKNOWN = 0,
    
    /* Control commands */
    BLE_CMD_ARM,
    BLE_CMD_DISARM,
    BLE_CMD_THROTTLE,
    BLE_CMD_ROLL,
    BLE_CMD_PITCH,
    BLE_CMD_YAW,
    
    /* PID tuning commands */
    BLE_CMD_RATE_RP,        /**< Rate roll/pitch gains: Kp,Ki,Kd */
    BLE_CMD_RATE_YAW,       /**< Rate yaw gains: Kp,Ki,Kd */
    BLE_CMD_ATT_RP,         /**< Attitude roll/pitch gains: Kp,Ki,Kd */
    
    /* Telemetry commands */
    BLE_CMD_TEL_ENABLE,     /**< Enable/disable telemetry */
    BLE_CMD_GET,            /**< Get current gains */
    
    /* System commands */
    BLE_CMD_HEARTBEAT,      /**< Connection heartbeat */
} bleTunerCmd_t;

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief PID gains structure (for command parsing)
 */
typedef struct {
    float kp;
    float ki;
    float kd;
} bleTunerPidGains_t;

/**
 * @brief Parsed command structure
 */
typedef struct {
    bleTunerCmd_t cmd;              /**< Command ID */
    union {
        float value;                /**< Single float value (THR, ROLL, etc.) */
        bleTunerPidGains_t gains;   /**< PID gains */
        uint8_t enable;             /**< Enable flag (TEL) */
    } data;
} bleTunerCommand_t;

/**
 * @brief Telemetry data structure
 */
typedef struct {
    /* Attitude (degrees) */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    
    /* Motor outputs (%) */
    float motor[4];
    
    /* Gyro rates (deg/s) */
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} bleTunerTelemetry_t;

/**
 * @brief Current gains structure (for GET command response)
 */
typedef struct {
    bleTunerPidGains_t rate_rp;     /**< Rate roll/pitch gains */
    bleTunerPidGains_t rate_yaw;    /**< Rate yaw gains */
    bleTunerPidGains_t att_rp;      /**< Attitude roll/pitch gains */
} bleTunerGains_t;

/*******************************************************************************
 * Callback Types
 ******************************************************************************/

/**
 * @brief Command received callback
 * @param cmd Parsed command structure
 * @note Called from BLE context - keep processing minimal!
 *       For heavy operations, set a flag and process in main loop.
 */
typedef void (*bleTunerCmdCallback_t)(const bleTunerCommand_t *cmd);

/**
 * @brief Disconnect callback (for auto-disarm)
 * @note Called when connection is lost or heartbeat timeout occurs
 */
typedef void (*bleTunerDisconnectCallback_t)(void);

/**
 * @brief Get current gains callback (for GET command)
 * @param gains Pointer to fill with current gains
 */
typedef void (*bleTunerGetGainsCallback_t)(bleTunerGains_t *gains);

/*******************************************************************************
 * Public Functions - Initialization
 ******************************************************************************/

/**
 * @brief Initialize BLE Tuner
 * @param device_name BLE device name (e.g., "QUAD-FC")
 * @return 0 on success, -1 on error
 * @note Must be called after NVS is initialized
 */
int8_t bleTunerInit(const char *device_name);

/**
 * @brief Start BLE advertising
 * @return 0 on success, -1 on error
 */
int8_t bleTunerStartAdvertising(void);

/**
 * @brief Stop BLE advertising
 * @return 0 on success, -1 on error
 */
int8_t bleTunerStopAdvertising(void);

/*******************************************************************************
 * Public Functions - Callbacks
 ******************************************************************************/

/**
 * @brief Set command received callback
 * @param callback Function to call when command received
 */
void bleTunerSetCmdCallback(bleTunerCmdCallback_t callback);

/**
 * @brief Set disconnect callback (for auto-disarm)
 * @param callback Function to call on disconnect/timeout
 */
void bleTunerSetDisconnectCallback(bleTunerDisconnectCallback_t callback);

/**
 * @brief Set get gains callback (for GET command)
 * @param callback Function to call to retrieve current gains
 */
void bleTunerSetGetGainsCallback(bleTunerGetGainsCallback_t callback);

/*******************************************************************************
 * Public Functions - Telemetry
 ******************************************************************************/

/**
 * @brief Send telemetry data
 * @param telemetry Pointer to telemetry data
 * @return 0 on success, -1 if not connected or telemetry disabled
 * @note Call this at 10 Hz from main loop
 */
int8_t bleTunerSendTelemetry(const bleTunerTelemetry_t *telemetry);

/**
 * @brief Send response string
 * @param response Response string (e.g., "OK:ARM")
 * @return 0 on success, -1 on error
 */
int8_t bleTunerSendResponse(const char *response);

/*******************************************************************************
 * Public Functions - Status
 ******************************************************************************/

/**
 * @brief Check if BLE is connected
 * @return 1 if connected, 0 if not
 */
uint8_t bleTunerIsConnected(void);

/**
 * @brief Check if telemetry is enabled
 * @return 1 if enabled, 0 if disabled
 */
uint8_t bleTunerIsTelemetryEnabled(void);

/**
 * @brief Check heartbeat timeout
 * @param current_time_ms Current system time in milliseconds
 * @return 1 if timeout occurred (should disarm), 0 if OK
 * @note Call this periodically from main loop
 */
uint8_t bleTunerCheckHeartbeatTimeout(uint32_t current_time_ms);

/**
 * @brief Reset heartbeat timer
 * @param current_time_ms Current system time in milliseconds
 * @note Called internally when HB command received
 */
void bleTunerResetHeartbeat(uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLE_TUNER_H */
