/**
 * @file attitude_controller.h
 * @brief Quaternion-based attitude controller (outer loop)
 * 
 * @details This module implements the outer loop of the cascaded PID control
 *          structure for quadcopter attitude stabilization.
 * 
 * CASCADED CONTROL STRUCTURE:
 * ===========================
 * 
 *   RC Command      ┌─────────────────┐     Rate        ┌──────────────┐     Motor
 *   (angles)   ───▶ │ ATTITUDE CTRL   │ ─── Setpoint ─▶ │  RATE CTRL   │ ──▶ Commands
 *                   │ (This module)   │                 │ (Inner loop) │
 *                   └────────┬────────┘                 └──────┬───────┘
 *                            │                                  │
 *                            ▼                                  ▼
 *                       EKF Quaternion                    Gyro (direct)
 * 
 * WHY CASCADED CONTROL?
 * =====================
 * 1. Inner loop (rate) runs faster, rejects disturbances quickly
 * 2. Outer loop (angle) provides setpoint for inner loop
 * 3. Easier to tune - each loop can be tuned separately
 * 4. Better handling of actuator saturation
 * 
 * QUATERNION ERROR CALCULATION:
 * =============================
 * Instead of working with Euler angles (which have gimbal lock issues),
 * we compute attitude error directly in quaternion space:
 * 
 *   q_error = q_desired^(-1) ⊗ q_current
 * 
 * For small angles, the vector part of q_error (q1, q2, q3) approximates
 * half the rotation angle about each axis:
 *   
 *   θ_error ≈ 2 * [q1_error, q2_error, q3_error]
 * 
 * The output of the attitude controller is angular rate setpoints (deg/s)
 * that are fed to the rate controller.
 * 
 * @note Runs at 500 Hz (every 2ms) - half the rate of IMU/rate controller
 * @note Input: Desired angles from RC + Current quaternion from EKF
 * @note Output: Angular rate setpoints for rate controller
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <stdint.h>
#include "control_types.h"
#include "pid_controller.h"
#include "quaternion_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Default attitude controller gains
 * 
 * @note These are starting values - tune for your specific drone!
 * 
 * For attitude (angle) loop:
 * - Kp: Higher = faster response to angle error, but may overshoot
 * - Ki: Usually small or zero (rate loop handles steady-state)
 * - Kd: Usually zero (derivative is provided by rate feedback)
 * 
 * Typical values:
 * - Kp: 4-8 for roll/pitch, 2-4 for yaw
 * - Ki: 0-1 (often zero, rate loop has integral)
 * - Kd: 0 (not needed in outer loop)
 */
#define ATT_DEFAULT_KP_ROLL_PITCH   6.0f    /**< Roll/Pitch Kp [rate_dps / angle_deg] */
#define ATT_DEFAULT_KI_ROLL_PITCH   0.5f    /**< Roll/Pitch Ki */
#define ATT_DEFAULT_KD_ROLL_PITCH   0.0f    /**< Roll/Pitch Kd (usually 0) */

#define ATT_DEFAULT_KP_YAW          4.0f    /**< Yaw Kp */
#define ATT_DEFAULT_KI_YAW          0.2f    /**< Yaw Ki */
#define ATT_DEFAULT_KD_YAW          0.0f    /**< Yaw Kd */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Attitude controller configuration
 */
typedef struct {
    /* Roll/Pitch gains (symmetric, same gains for both) */
    PidGains_t roll_pitch_gains;
    
    /* Yaw gains (different because moment of inertia differs) */
    PidGains_t yaw_gains;
    
    /* Rate limits (output limits) */
    float max_roll_rate;        /**< Max roll rate output [deg/s] */
    float max_pitch_rate;       /**< Max pitch rate output [deg/s] */
    float max_yaw_rate;         /**< Max yaw rate output [deg/s] */
    
    /* Angle limits (input limits) */
    float max_roll_angle;       /**< Max roll angle command [deg] */
    float max_pitch_angle;      /**< Max pitch angle command [deg] */
    
    /* Timing */
    float sample_time;          /**< Controller sample time [s] */
} AttitudeConfig_t;

/**
 * @brief Attitude controller handle
 */
typedef struct {
    /* PID controllers for each axis */
    PidHandle_t pid_roll;
    PidHandle_t pid_pitch;
    PidHandle_t pid_yaw;
    
    /* Configuration */
    AttitudeConfig_t config;
    
    /* Desired attitude (from RC command, converted to quaternion) */
    Quaternion_t q_desired;
    
    /* Last computed error (for telemetry) */
    Quaternion_t q_error;
    float error_roll_deg;
    float error_pitch_deg;
    float error_yaw_deg;
    
    /* Status */
    uint8_t is_initialized;
} AttitudeHandle_t;

/**
 * @brief Attitude setpoint structure (input from RC)
 */
typedef struct {
    float roll_deg;     /**< Desired roll angle [degrees] */
    float pitch_deg;    /**< Desired pitch angle [degrees] */
    float yaw_rate_dps; /**< Desired yaw RATE [deg/s] - note: rate, not angle! */
} AttitudeSetpoint_t;

/**
 * @brief Attitude controller debug output
 */
typedef struct {
    /* Quaternion error components (small angle approx: 2*q1,q2,q3 ≈ θ) */
    float q_error_x;        /**< q1 component of error quaternion */
    float q_error_y;        /**< q2 component of error quaternion */
    float q_error_z;        /**< q3 component of error quaternion */
    
    /* Angle errors in degrees */
    float error_roll_deg;
    float error_pitch_deg;
    float error_yaw_deg;
    
    /* PID outputs (rate setpoints) */
    float rate_sp_roll;
    float rate_sp_pitch;
    float rate_sp_yaw;
    
    /* Individual PID debug info */
    PidDebug_t pid_debug_roll;
    PidDebug_t pid_debug_pitch;
    PidDebug_t pid_debug_yaw;
} AttitudeDebug_t;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize attitude controller with configuration
 * 
 * @param[out] att      Attitude controller handle
 * @param[in]  config   Configuration parameters
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeInit(AttitudeHandle_t *att, const AttitudeConfig_t *config);

/**
 * @brief Initialize attitude controller with default parameters
 * 
 * @param[out] att          Attitude controller handle
 * @param[in]  sample_time  Sample time [s]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeInitDefault(AttitudeHandle_t *att, float sample_time);

/**
 * @brief Reset attitude controller
 * 
 * @param[in,out] att   Attitude controller handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeReset(AttitudeHandle_t *att);

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

/**
 * @brief Update attitude controller
 * 
 * @param[in,out] att           Attitude controller handle
 * @param[in]     setpoint      Desired attitude (roll/pitch angles, yaw rate)
 * @param[in]     q_current     Current attitude quaternion (from EKF)
 * @param[out]    rate_setpoint Output rate setpoints [deg/s]
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Algorithm:
 * 
 * 1. CONVERT SETPOINT TO QUATERNION:
 *    For roll/pitch, we create a quaternion representing the desired attitude.
 *    For yaw, we use rate mode (common in RC flying):
 *    - Yaw input controls yaw RATE, not angle
 *    - Desired yaw angle integrates yaw rate input
 *    
 *    This way, pilot doesn't need to hold yaw stick to maintain heading.
 * 
 * 2. COMPUTE QUATERNION ERROR:
 *    q_error = q_desired^(-1) ⊗ q_current
 *    
 *    This gives us the rotation needed to go from current to desired attitude.
 *    For a unit quaternion: q^(-1) = conjugate(q)
 * 
 * 3. EXTRACT AXIS ERRORS:
 *    For small angles: θ ≈ 2 * arcsin(|q_vec|) ≈ 2 * |q_vec|
 *    
 *    error_roll  ≈ 2 * q_error.q1
 *    error_pitch ≈ 2 * q_error.q2
 *    error_yaw   ≈ 2 * q_error.q3
 *    
 *    Convert to degrees and feed to PID.
 * 
 * 4. RUN PID FOR EACH AXIS:
 *    rate_setpoint_roll  = PID_roll(0, error_roll)   // setpoint=0, meas=error
 *    rate_setpoint_pitch = PID_pitch(0, error_pitch)
 *    rate_setpoint_yaw   = yaw_rate_command (direct, or PID for heading hold)
 * 
 * @note Yaw is handled differently - typically as rate command, not angle
 */
int8_t attitudeUpdate(AttitudeHandle_t *att,
                      const AttitudeSetpoint_t *setpoint,
                      const Quaternion_t *q_current,
                      RateSetpoint_t *rate_setpoint);

/**
 * @brief Update attitude controller with debug output
 * 
 * @param[in,out] att           Attitude controller handle
 * @param[in]     setpoint      Desired attitude
 * @param[in]     q_current     Current attitude quaternion
 * @param[out]    rate_setpoint Output rate setpoints
 * @param[out]    debug         Debug information (can be NULL)
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeUpdateDebug(AttitudeHandle_t *att,
                           const AttitudeSetpoint_t *setpoint,
                           const Quaternion_t *q_current,
                           RateSetpoint_t *rate_setpoint,
                           AttitudeDebug_t *debug);

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

/**
 * @brief Set roll/pitch PID gains
 * 
 * @param[in,out] att   Attitude controller handle
 * @param[in]     kp    Proportional gain
 * @param[in]     ki    Integral gain
 * @param[in]     kd    Derivative gain
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeSetRollPitchGains(AttitudeHandle_t *att, float kp, float ki, float kd);

/**
 * @brief Set yaw PID gains
 * 
 * @param[in,out] att   Attitude controller handle
 * @param[in]     kp    Proportional gain
 * @param[in]     ki    Integral gain
 * @param[in]     kd    Derivative gain
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeSetYawGains(AttitudeHandle_t *att, float kp, float ki, float kd);

/**
 * @brief Set maximum rate outputs
 * 
 * @param[in,out] att               Attitude controller handle
 * @param[in]     max_roll_rate     Max roll rate [deg/s]
 * @param[in]     max_pitch_rate    Max pitch rate [deg/s]
 * @param[in]     max_yaw_rate      Max yaw rate [deg/s]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeSetRateLimits(AttitudeHandle_t *att,
                             float max_roll_rate,
                             float max_pitch_rate,
                             float max_yaw_rate);

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

/**
 * @brief Get current attitude error in degrees
 * 
 * @param[in]  att          Attitude controller handle
 * @param[out] roll_err     Roll error [degrees]
 * @param[out] pitch_err    Pitch error [degrees]
 * @param[out] yaw_err      Yaw error [degrees]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeGetError(const AttitudeHandle_t *att,
                        float *roll_err, float *pitch_err, float *yaw_err);

/**
 * @brief Get error quaternion
 * 
 * @param[in]  att      Attitude controller handle
 * @param[out] q_error  Error quaternion
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeGetErrorQuat(const AttitudeHandle_t *att, Quaternion_t *q_error);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Convert Euler angles (degrees) to quaternion
 * 
 * @param[in]  roll_deg     Roll angle [degrees]
 * @param[in]  pitch_deg    Pitch angle [degrees]
 * @param[in]  yaw_deg      Yaw angle [degrees]
 * @param[out] q            Output quaternion
 * @return CTRL_OK on success, error code otherwise
 */
int8_t attitudeEulerToQuat(float roll_deg, float pitch_deg, float yaw_deg,
                           Quaternion_t *q);

/**
 * @brief Compute quaternion error
 * 
 * @param[in]  q_desired    Desired quaternion
 * @param[in]  q_current    Current quaternion
 * @param[out] q_error      Error quaternion (q_desired^-1 * q_current)
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details The error quaternion represents the rotation from desired to current.
 *          q_error = conj(q_desired) ⊗ q_current
 *          
 *          We ensure the result has positive scalar part (q0 > 0) to choose
 *          the shorter rotation path.
 */
int8_t attitudeComputeQuatError(const Quaternion_t *q_desired,
                                const Quaternion_t *q_current,
                                Quaternion_t *q_error);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_CONTROLLER_H */
