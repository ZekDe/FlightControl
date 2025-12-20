/**
 * @file rate_controller.h
 * @brief Angular rate controller (inner loop)
 * 
 * @details This module implements the inner loop of the cascaded PID control
 *          structure. It runs at the highest frequency (1000 Hz) and uses
 *          gyroscope measurements directly for fast disturbance rejection.
 * 
 * INNER LOOP OPERATION:
 * =====================
 * 
 *   Rate Setpoint     ┌─────────────────┐     Roll/Pitch/Yaw    ┌─────────┐
 *   (from attitude  ─▶│  RATE CONTROLLER│────   Commands    ───▶│  MIXER  │
 *    controller)      │  (This module)  │                       └─────────┘
 *                     └────────┬────────┘
 *                              │
 *                              ▼
 *                         Gyroscope
 *                       (1000 Hz direct)
 * 
 * WHY DIRECT GYRO (not EKF)?
 * ==========================
 * 1. Latency: EKF adds processing delay, gyro is immediate
 * 2. Frequency: Gyro available at full 1000 Hz, EKF at 500 Hz
 * 3. Rate measurement: Gyro directly measures angular rate
 * 4. Noise: For rate control, gyro noise is acceptable (derivative not needed)
 * 
 * The EKF quaternion is used by the OUTER loop (attitude) to get accurate
 * angle estimates. The INNER loop (rate) uses raw gyro for speed.
 * 
 * PID TUNING STRATEGY:
 * ====================
 * Rate loop is typically tuned first, then attitude loop.
 * 
 * Rate PID characteristics:
 * - Kp: Main response - higher = stiffer, but may oscillate
 * - Ki: Removes steady-state error from wind/CG offset
 * - Kd: Dampens oscillations - very important for quads!
 * 
 * Typical rate PID values (starting point):
 * - Roll/Pitch: Kp=0.1-0.5, Ki=0.1-0.5, Kd=0.001-0.01
 * - Yaw: Kp=0.2-0.8, Ki=0.1-0.3, Kd=0.0
 * 
 * @note Input: Rate setpoints (deg/s) from attitude controller + Gyro (deg/s)
 * @note Output: Control commands for motor mixer
 * @note Runs at 1000 Hz (every 1ms)
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef RATE_CONTROLLER_H
#define RATE_CONTROLLER_H

#include <stdint.h>
#include "control_types.h"
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Default rate controller gains
 * 
 * @note These are conservative starting values - tune for your drone!
 * 
 * For rate loop:
 * - Kp: Proportional to rate error
 * - Ki: Handles constant disturbances (wind, CG offset)
 * - Kd: Critical for damping - prevents oscillation
 * 
 * The D term is very important in rate loop (unlike attitude loop).
 */
#define RATE_DEFAULT_KP_ROLL_PITCH  0.25f   /**< Roll/Pitch Kp */
#define RATE_DEFAULT_KI_ROLL_PITCH  0.30f   /**< Roll/Pitch Ki */
#define RATE_DEFAULT_KD_ROLL_PITCH  0.003f  /**< Roll/Pitch Kd */

#define RATE_DEFAULT_KP_YAW         0.40f   /**< Yaw Kp (higher, yaw is sluggish) */
#define RATE_DEFAULT_KI_YAW         0.20f   /**< Yaw Ki */
#define RATE_DEFAULT_KD_YAW         0.0f    /**< Yaw Kd (usually 0 for yaw) */

/**
 * @brief Maximum PID output
 * 
 * @note Output range determines mixer headroom.
 *       Typically [-500, 500] or [-1, 1] normalized.
 *       We use a scale that allows good resolution.
 */
#define RATE_OUTPUT_MAX             500.0f  /**< Max PID output magnitude */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Rate controller configuration
 */
typedef struct {
    /* Roll/Pitch gains (symmetric) */
    PidGains_t roll_pitch_gains;
    
    /* Yaw gains */
    PidGains_t yaw_gains;
    
    /* Output limits */
    float output_max;           /**< Maximum PID output magnitude */
    
    /* Timing */
    float sample_time;          /**< Sample time [s] */
} RateConfig_t;

/**
 * @brief Rate controller handle
 */
typedef struct {
    /* PID controllers */
    PidHandle_t pid_roll;
    PidHandle_t pid_pitch;
    PidHandle_t pid_yaw;
    
    /* Configuration */
    RateConfig_t config;
    
    /* Last gyro rates (for telemetry) */
    float gyro_roll_dps;
    float gyro_pitch_dps;
    float gyro_yaw_dps;
    
    /* Last setpoints (for telemetry) */
    float setpoint_roll_dps;
    float setpoint_pitch_dps;
    float setpoint_yaw_dps;
    
    /* Status */
    uint8_t is_initialized;
} RateHandle_t;

/**
 * @brief Gyro measurement input (in degrees per second)
 */
typedef struct {
    float roll;     /**< Roll rate from gyro [deg/s] */
    float pitch;    /**< Pitch rate from gyro [deg/s] */
    float yaw;      /**< Yaw rate from gyro [deg/s] */
} GyroData_t;

/**
 * @brief Rate controller output (to mixer)
 */
typedef struct {
    float roll;     /**< Roll control output */
    float pitch;    /**< Pitch control output */
    float yaw;      /**< Yaw control output */
} RateOutput_t;

/**
 * @brief Rate controller debug output
 */
typedef struct {
    /* Setpoints */
    float setpoint_roll;
    float setpoint_pitch;
    float setpoint_yaw;
    
    /* Gyro measurements */
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
    
    /* Errors */
    float error_roll;
    float error_pitch;
    float error_yaw;
    
    /* PID outputs */
    float output_roll;
    float output_pitch;
    float output_yaw;
    
    /* Individual PID debug */
    PidDebug_t pid_debug_roll;
    PidDebug_t pid_debug_pitch;
    PidDebug_t pid_debug_yaw;
} RateDebug_t;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize rate controller with configuration
 * 
 * @param[out] rate     Rate controller handle
 * @param[in]  config   Configuration parameters
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateInit(RateHandle_t *rate, const RateConfig_t *config);

/**
 * @brief Initialize rate controller with default parameters
 * 
 * @param[out] rate         Rate controller handle
 * @param[in]  sample_time  Sample time [s]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateInitDefault(RateHandle_t *rate, float sample_time);

/**
 * @brief Reset rate controller
 * 
 * @param[in,out] rate  Rate controller handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateReset(RateHandle_t *rate);

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

/**
 * @brief Update rate controller
 * 
 * @param[in,out] rate          Rate controller handle
 * @param[in]     setpoint      Desired rates [deg/s]
 * @param[in]     gyro          Current gyro measurements [deg/s]
 * @param[out]    output        Control outputs for mixer
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Algorithm:
 * 
 * 1. For each axis (roll, pitch, yaw):
 *    output = PID(setpoint, gyro_measurement)
 *    
 * 2. The PID computes:
 *    error = setpoint - measurement
 *    output = Kp*error + Ki*∫error + Kd*d(measurement)/dt
 *    
 * 3. Output is constrained to [-output_max, +output_max]
 * 
 * @note Call this at 1000 Hz (or your configured rate)
 * @note Gyro input should be in deg/s (same units as setpoint)
 */
int8_t rateUpdate(RateHandle_t *rate,
                  const RateSetpoint_t *setpoint,
                  const GyroData_t *gyro,
                  RateOutput_t *output);

/**
 * @brief Update rate controller with debug output
 * 
 * @param[in,out] rate      Rate controller handle
 * @param[in]     setpoint  Desired rates [deg/s]
 * @param[in]     gyro      Current gyro measurements [deg/s]
 * @param[out]    output    Control outputs for mixer
 * @param[out]    debug     Debug information (can be NULL)
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateUpdateDebug(RateHandle_t *rate,
                       const RateSetpoint_t *setpoint,
                       const GyroData_t *gyro,
                       RateOutput_t *output,
                       RateDebug_t *debug);

/**
 * @brief Update rate controller with raw gyro (rad/s)
 * 
 * @param[in,out] rate          Rate controller handle
 * @param[in]     setpoint      Desired rates [deg/s]
 * @param[in]     gyro_rad_x    Gyro X in rad/s
 * @param[in]     gyro_rad_y    Gyro Y in rad/s
 * @param[in]     gyro_rad_z    Gyro Z in rad/s
 * @param[out]    output        Control outputs for mixer
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Convenience function that converts rad/s to deg/s internally
 */
int8_t rateUpdateRaw(RateHandle_t *rate,
                     const RateSetpoint_t *setpoint,
                     float gyro_rad_x, float gyro_rad_y, float gyro_rad_z,
                     RateOutput_t *output);

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

/**
 * @brief Set roll/pitch PID gains
 * 
 * @param[in,out] rate  Rate controller handle
 * @param[in]     kp    Proportional gain
 * @param[in]     ki    Integral gain
 * @param[in]     kd    Derivative gain
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateSetRollPitchGains(RateHandle_t *rate, float kp, float ki, float kd);

/**
 * @brief Set yaw PID gains
 * 
 * @param[in,out] rate  Rate controller handle
 * @param[in]     kp    Proportional gain
 * @param[in]     ki    Integral gain
 * @param[in]     kd    Derivative gain
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateSetYawGains(RateHandle_t *rate, float kp, float ki, float kd);

/**
 * @brief Set all gains at once
 * 
 * @param[in,out] rate          Rate controller handle
 * @param[in]     roll_pitch    Roll/pitch gains
 * @param[in]     yaw           Yaw gains
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateSetAllGains(RateHandle_t *rate,
                       const PidGains_t *roll_pitch,
                       const PidGains_t *yaw);

/**
 * @brief Set output limits
 * 
 * @param[in,out] rate          Rate controller handle
 * @param[in]     output_max    Maximum output magnitude
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateSetOutputLimit(RateHandle_t *rate, float output_max);

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

/**
 * @brief Get current rate errors
 * 
 * @param[in]  rate         Rate controller handle
 * @param[out] roll_err     Roll rate error [deg/s]
 * @param[out] pitch_err    Pitch rate error [deg/s]
 * @param[out] yaw_err      Yaw rate error [deg/s]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateGetError(const RateHandle_t *rate,
                    float *roll_err, float *pitch_err, float *yaw_err);

/**
 * @brief Check if any axis is saturated
 * 
 * @param[in]  rate         Rate controller handle
 * @param[out] roll_sat     1 if roll saturated
 * @param[out] pitch_sat    1 if pitch saturated
 * @param[out] yaw_sat      1 if yaw saturated
 * @return CTRL_OK on success, error code otherwise
 */
int8_t rateIsSaturated(const RateHandle_t *rate,
                       uint8_t *roll_sat, uint8_t *pitch_sat, uint8_t *yaw_sat);

#ifdef __cplusplus
}
#endif

#endif /* RATE_CONTROLLER_H */
