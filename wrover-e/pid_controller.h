/**
 * @file pid_controller.h
 * @brief Professional PID controller with advanced features
 * 
 * @details Features:
 *          - Tustin (bilinear) transform for integral term
 *          - Derivative on measurement (not on error) to avoid setpoint kick
 *          - First-order low-pass filter on derivative term
 *          - Back-calculation anti-windup mechanism
 *          - Output saturation with configurable limits
 * 
 * @note Theory:
 * 
 * STANDARD PID EQUATION (continuous time):
 *   u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 * 
 * TUSTIN TRANSFORM (s-domain to z-domain):
 *   The Tustin (bilinear) transform approximates:
 *     s ≈ (2/T) * (z-1)/(z+1)
 *   
 *   For integral term (1/s):
 *     I[n] = I[n-1] + (Ki*T/2) * (e[n] + e[n-1])
 *   
 *   This preserves frequency response better than backward Euler.
 * 
 * DERIVATIVE ON MEASUREMENT:
 *   Instead of: d(setpoint - measurement)/dt
 *   We use:     -d(measurement)/dt
 *   
 *   This avoids large spikes when setpoint changes suddenly.
 *   The negative sign is because: de/dt = d(SP-PV)/dt = -dPV/dt (if SP constant)
 * 
 * DERIVATIVE LOW-PASS FILTER:
 *   Raw derivative amplifies high-frequency noise.
 *   First-order LPF: D_filtered = α*D_raw + (1-α)*D_prev
 *   where α = T / (T + τ), τ = 1/(2π*fc), fc = cutoff frequency
 * 
 * BACK-CALCULATION ANTI-WINDUP:
 *   When output saturates, integrator continues growing = windup.
 *   Back-calculation feeds back the saturation error:
 *     I[n] = I[n-1] + Ki*e*T + Kb*(u_sat - u_unsat)
 *   where Kb = 1/Tt (tracking time constant, typically Kb = Ki)
 *   
 *   This "unwinds" the integrator when output is saturated.
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define PID_EPSILON             1e-6f   /**< Small value for float comparisons */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief PID gain structure
 */
typedef struct {
    float kp;           /**< Proportional gain */
    float ki;           /**< Integral gain */
    float kd;           /**< Derivative gain */
} PidGains_t;

/**
 * @brief PID configuration structure
 */
typedef struct {
    PidGains_t gains;           /**< PID gains (Kp, Ki, Kd) */
    float output_min;           /**< Minimum output limit */
    float output_max;           /**< Maximum output limit */
    float integral_max;         /**< Maximum integral term (anti-windup limit) */
    float derivative_lpf_hz;    /**< Derivative LPF cutoff frequency [Hz] */
    float sample_time;          /**< Sample time [seconds] */
} PidConfig_t;

/**
 * @brief PID internal state structure
 * 
 * @details Stores all state variables needed between iterations:
 *          - Previous measurement for derivative calculation
 *          - Previous error for Tustin integral
 *          - Filtered derivative value
 *          - Integral accumulator
 */
typedef struct {
    /* Derivative state */
    float prev_measurement;     /**< Previous measurement (for derivative on measurement) */
    float derivative_filtered;  /**< Low-pass filtered derivative term */
    float derivative_alpha;     /**< LPF coefficient: α = dt/(dt + τ) */
    
    /* Integral state (Tustin) */
    float integral;             /**< Integral accumulator */
    float prev_error;           /**< Previous error (for Tustin transform) */
    
    /* Anti-windup state */
    float kb;                   /**< Back-calculation gain (typically = Ki) */
    
    /* Output state */
    float prev_output_unsat;    /**< Previous unsaturated output */
    float prev_output_sat;      /**< Previous saturated output */
    
    /* Status */
    uint8_t is_initialized;     /**< Initialization flag */
    uint8_t is_saturated;       /**< Output saturation flag */
} PidState_t;

/**
 * @brief Complete PID controller handle
 */
typedef struct {
    PidConfig_t config;         /**< Configuration parameters */
    PidState_t state;           /**< Internal state */
} PidHandle_t;

/**
 * @brief PID debug/telemetry structure
 */
typedef struct {
    float p_term;               /**< Proportional term contribution */
    float i_term;               /**< Integral term contribution */
    float d_term;               /**< Derivative term contribution */
    float output_unsat;         /**< Output before saturation */
    float output_sat;           /**< Output after saturation */
    float error;                /**< Current error value */
    uint8_t saturated;          /**< Saturation flag */
} PidDebug_t;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize PID controller with configuration
 * 
 * @param[out] pid          PID handle to initialize
 * @param[in]  config       Configuration parameters
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Initializes:
 *          - Stores configuration
 *          - Calculates derivative LPF coefficient
 *          - Sets back-calculation gain Kb = Ki
 *          - Resets all state variables
 * 
 * @note Call this once before using the PID controller
 */
int8_t pidInit(PidHandle_t *pid, const PidConfig_t *config);

/**
 * @brief Initialize PID with individual parameters
 * 
 * @param[out] pid              PID handle to initialize
 * @param[in]  kp               Proportional gain
 * @param[in]  ki               Integral gain
 * @param[in]  kd               Derivative gain
 * @param[in]  output_min       Minimum output
 * @param[in]  output_max       Maximum output
 * @param[in]  derivative_lpf   Derivative LPF cutoff [Hz]
 * @param[in]  sample_time      Sample time [s]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t pidInitParams(PidHandle_t *pid,
                     float kp, float ki, float kd,
                     float output_min, float output_max,
                     float derivative_lpf, float sample_time);

/**
 * @brief Reset PID controller state
 * 
 * @param[in,out] pid   PID handle
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Resets:
 *          - Integral accumulator to zero
 *          - Previous measurement to zero
 *          - Filtered derivative to zero
 *          - Previous error to zero
 * 
 * @note Call this when switching modes or after a large disturbance
 */
int8_t pidReset(PidHandle_t *pid);

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

/**
 * @brief Update PID controller (main calculation)
 * 
 * @param[in,out] pid           PID handle
 * @param[in]     setpoint      Desired value
 * @param[in]     measurement   Current measured value
 * @param[out]    output        Calculated control output
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Algorithm:
 * 
 *   1. PROPORTIONAL TERM:
 *      error = setpoint - measurement
 *      P = Kp * error
 * 
 *   2. INTEGRAL TERM (Tustin transform):
 *      I[n] = I[n-1] + (Ki*T/2)*(e[n] + e[n-1]) + Kb*(u_sat - u_unsat)
 *      
 *      The Tustin term (Ki*T/2)*(e[n]+e[n-1]) averages current and previous
 *      error, providing better frequency response than backward Euler.
 *      
 *      The back-calculation term Kb*(u_sat-u_unsat) "unwinds" the integrator
 *      when output was saturated in the previous iteration.
 * 
 *   3. DERIVATIVE TERM (on measurement, filtered):
 *      d_raw = -(measurement - prev_measurement) / T
 *      
 *      Note the negative sign: we use derivative on measurement to avoid
 *      setpoint kick, so d(error)/dt = -d(measurement)/dt
 *      
 *      d_filtered = α*d_raw + (1-α)*d_prev   (first-order LPF)
 *      D = Kd * d_filtered
 * 
 *   4. OUTPUT:
 *      u_unsat = P + I + D
 *      u_sat = constrain(u_unsat, output_min, output_max)
 *      output = u_sat
 * 
 * @note Must be called at regular intervals (sample_time)
 */
int8_t pidUpdate(PidHandle_t *pid, float setpoint, float measurement, float *output);

/**
 * @brief Update PID with debug output
 * 
 * @param[in,out] pid           PID handle
 * @param[in]     setpoint      Desired value
 * @param[in]     measurement   Current measured value
 * @param[out]    output        Calculated control output
 * @param[out]    debug         Debug information (can be NULL)
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Same as pidUpdate() but also fills debug structure with
 *          individual term contributions for telemetry/tuning.
 */
int8_t pidUpdateDebug(PidHandle_t *pid, float setpoint, float measurement,
                      float *output, PidDebug_t *debug);

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

/**
 * @brief Set PID gains at runtime
 * 
 * @param[in,out] pid   PID handle
 * @param[in]     kp    New proportional gain
 * @param[in]     ki    New integral gain
 * @param[in]     kd    New derivative gain
 * @return CTRL_OK on success, error code otherwise
 * 
 * @note Updates back-calculation gain Kb when Ki changes
 * @note Does NOT reset integral term (call pidReset if needed)
 */
int8_t pidSetGains(PidHandle_t *pid, float kp, float ki, float kd);

/**
 * @brief Set output limits
 * 
 * @param[in,out] pid           PID handle
 * @param[in]     output_min    New minimum output
 * @param[in]     output_max    New maximum output
 * @return CTRL_OK on success, error code otherwise
 */
int8_t pidSetOutputLimits(PidHandle_t *pid, float output_min, float output_max);

/**
 * @brief Set integral limit (for additional anti-windup)
 * 
 * @param[in,out] pid           PID handle
 * @param[in]     integral_max  Maximum integral term magnitude
 * @return CTRL_OK on success, error code otherwise
 */
int8_t pidSetIntegralLimit(PidHandle_t *pid, float integral_max);

/**
 * @brief Set derivative low-pass filter cutoff
 * 
 * @param[in,out] pid           PID handle
 * @param[in]     lpf_hz        LPF cutoff frequency [Hz]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t pidSetDerivativeFilter(PidHandle_t *pid, float lpf_hz);

/**
 * @brief Set sample time (if rate changes)
 * 
 * @param[in,out] pid           PID handle
 * @param[in]     sample_time   New sample time [s]
 * @return CTRL_OK on success, error code otherwise
 * 
 * @note Recalculates derivative filter coefficient
 */
int8_t pidSetSampleTime(PidHandle_t *pid, float sample_time);

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

/**
 * @brief Check if output is currently saturated
 * 
 * @param[in]  pid          PID handle
 * @param[out] saturated    1 if saturated, 0 if not
 * @return CTRL_OK on success, error code otherwise
 */
int8_t pidIsSaturated(const PidHandle_t *pid, uint8_t *saturated);

/**
 * @brief Get current integral value
 * 
 * @param[in]  pid          PID handle
 * @param[out] integral     Current integral term
 * @return CTRL_OK on success, error code otherwise
 */
int8_t pidGetIntegral(const PidHandle_t *pid, float *integral);

/**
 * @brief Set integral value directly (for bumpless transfer)
 * 
 * @param[in,out] pid       PID handle
 * @param[in]     integral  New integral value
 * @return CTRL_OK on success, error code otherwise
 * 
 * @note Useful for bumpless transfer between controllers
 */
int8_t pidSetIntegral(PidHandle_t *pid, float integral);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
