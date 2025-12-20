/**
 * @file web_server.h
 * @brief Embedded Web Server for Flight Controller Dashboard
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/
#define WEB_SERVER_PORT         80
#define WEB_MAX_CONNECTIONS     4

/*******************************************************************************
 * Telemetry Data Structure (for API)
 ******************************************************************************/
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    float motor[4];
    float gyro[3];
    float accel[3];
    float battery_voltage;
    uint8_t armed;
    uint8_t mode;
} web_telemetry_t;

/*******************************************************************************
 * PID Gains Structure
 ******************************************************************************/
typedef struct {
    float rate_rp_p, rate_rp_i, rate_rp_d;
    float rate_yaw_p, rate_yaw_i, rate_yaw_d;
    float att_rp_p, att_rp_i, att_rp_d;
} web_pid_gains_t;

/*******************************************************************************
 * Callback Types
 ******************************************************************************/
typedef void (*web_arm_cb_t)(uint8_t arm);
typedef void (*web_pid_cb_t)(const web_pid_gains_t *gains);
typedef void (*web_get_telemetry_cb_t)(web_telemetry_t *telemetry);
typedef void (*web_get_pid_cb_t)(web_pid_gains_t *gains);

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize web server
 * @return 0 on success, negative on error
 */
int8_t webServerInit(void);

/**
 * @brief Start web server
 * @return 0 on success, negative on error
 */
int8_t webServerStart(void);

/**
 * @brief Stop web server
 * @return 0 on success, negative on error
 */
int8_t webServerStop(void);

/**
 * @brief Set arm/disarm callback
 * @param callback Function to call on arm command
 */
void webServerSetArmCallback(web_arm_cb_t callback);

/**
 * @brief Set PID update callback
 * @param callback Function to call on PID update
 */
void webServerSetPidCallback(web_pid_cb_t callback);

/**
 * @brief Set telemetry getter callback
 * @param callback Function to call to get telemetry
 */
void webServerSetTelemetryCallback(web_get_telemetry_cb_t callback);

/**
 * @brief Set PID getter callback
 * @param callback Function to call to get PID gains
 */
void webServerSetGetPidCallback(web_get_pid_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
