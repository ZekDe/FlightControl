/**
 * @file pid_controller.c
 * @brief Implementation of professional PID controller
 */

#include "pid_controller.h"
#include <math.h>
#include <stddef.h>

/*******************************************************************************
 * Constants
 ******************************************************************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

/**
 * @brief Calculate derivative filter coefficient alpha
 * 
 * @details First-order low-pass filter coefficient:
 *          α = dt / (dt + τ)
 *          where τ = 1 / (2π * fc)
 *          
 *          At cutoff frequency fc, gain is -3dB (0.707)
 *          Higher α = more responsive but more noise
 *          Lower α = smoother but more lag
 */
static float calculateFilterAlpha(float sample_time, float cutoff_hz)
{
    if (cutoff_hz < PID_EPSILON) {
        return 0.0f;    /* Filter disabled */
    }
    
    float tau = 1.0f / (2.0f * M_PI * cutoff_hz);
    float alpha = sample_time / (sample_time + tau);
    
    return alpha;
}

/**
 * @brief Constrain float value between limits
 */
static float constrainFloat(float value, float min_val, float max_val)
{
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    }
    return value;
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t pidInit(PidHandle_t *pid, const PidConfig_t *config)
{
    if (pid == NULL || config == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /* Validate parameters */
    if (config->sample_time < PID_EPSILON) {
        return CTRL_ERR_INVALID_PARAM;
    }
    
    /* Store configuration */
    pid->config = *config;
    
    /* Calculate derivative filter coefficient
     * 
     * The filter equation is: y[n] = α*x[n] + (1-α)*y[n-1]
     * α = dt / (dt + τ), where τ = 1/(2πfc)
     */
    pid->state.derivative_alpha = calculateFilterAlpha(
        config->sample_time, 
        config->derivative_lpf_hz
    );
    
    /* Set back-calculation anti-windup gain
     * 
     * Kb determines how fast the integrator "unwinds" when saturated.
     * Common choices:
     *   - Kb = Ki (most common, gives tracking time = Ti)
     *   - Kb = sqrt(Ki/Kd) (compromise between I and D)
     * 
     * We use Kb = Ki for simplicity
     */
    pid->state.kb = config->gains.ki;
    
    /* Reset all state variables */
    pid->state.prev_measurement = 0.0f;
    pid->state.derivative_filtered = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.prev_error = 0.0f;
    pid->state.prev_output_unsat = 0.0f;
    pid->state.prev_output_sat = 0.0f;
    pid->state.is_saturated = 0;
    pid->state.is_initialized = 1;
    
    return CTRL_OK;
}

int8_t pidInitParams(PidHandle_t *pid,
                     float kp, float ki, float kd,
                     float output_min, float output_max,
                     float derivative_lpf, float sample_time)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    PidConfig_t config = {
        .gains = {
            .kp = kp,
            .ki = ki,
            .kd = kd
        },
        .output_min = output_min,
        .output_max = output_max,
        .integral_max = output_max,  /* Default: same as output limit */
        .derivative_lpf_hz = derivative_lpf,
        .sample_time = sample_time
    };
    
    return pidInit(pid, &config);
}

int8_t pidReset(PidHandle_t *pid)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Reset all state while preserving configuration */
    pid->state.prev_measurement = 0.0f;
    pid->state.derivative_filtered = 0.0f;
    pid->state.integral = 0.0f;
    pid->state.prev_error = 0.0f;
    pid->state.prev_output_unsat = 0.0f;
    pid->state.prev_output_sat = 0.0f;
    pid->state.is_saturated = 0;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

int8_t pidUpdate(PidHandle_t *pid, float setpoint, float measurement, float *output)
{
    return pidUpdateDebug(pid, setpoint, measurement, output, NULL);
}

int8_t pidUpdateDebug(PidHandle_t *pid, float setpoint, float measurement,
                      float *output, PidDebug_t *debug)
{
    if (pid == NULL || output == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    float dt = pid->config.sample_time;
    PidGains_t *gains = &pid->config.gains;
    PidState_t *state = &pid->state;
    
    /*=========================================================================
     * STEP 1: Calculate Error
     *=========================================================================
     * error = setpoint - measurement
     * Positive error means measurement is below setpoint
     */
    float error = setpoint - measurement;
    
    /*=========================================================================
     * STEP 2: Proportional Term
     *=========================================================================
     * P = Kp * error
     * 
     * Proportional term provides immediate response proportional to error.
     * Large Kp = fast response but may overshoot/oscillate
     * Small Kp = slow response, possible steady-state error if no I term
     */
    float p_term = gains->kp * error;
    
    /*=========================================================================
     * STEP 3: Integral Term (Tustin Transform + Back-Calculation Anti-Windup)
     *=========================================================================
     * TUSTIN TRANSFORM:
     * Instead of backward Euler: I += Ki * e * dt
     * We use Tustin: I += (Ki * dt / 2) * (e[n] + e[n-1])
     * 
     * This averages current and previous error, giving better frequency
     * response approximation from continuous to discrete time.
     * 
     * BACK-CALCULATION ANTI-WINDUP:
     * When output saturates, integrator keeps growing (windup).
     * Back-calculation adds a correction term:
     *   I += Kb * (u_saturated - u_unsaturated)
     * 
     * When output is NOT saturated: u_sat = u_unsat, correction = 0
     * When output IS saturated: correction is negative (if positive sat)
     *   or positive (if negative sat), which "unwinds" the integrator.
     */
    
    /* Tustin integral increment */
    float integral_increment = (gains->ki * dt / 2.0f) * (error + state->prev_error);
    
    /* Back-calculation anti-windup correction */
    float back_calc_correction = state->kb * (state->prev_output_sat - state->prev_output_unsat);
    
    /* Update integral */
    state->integral += integral_increment + back_calc_correction;
    
    /* Additional integral limiting (belt and suspenders with back-calc) */
    state->integral = constrainFloat(state->integral, 
                                     -pid->config.integral_max, 
                                     pid->config.integral_max);
    
    float i_term = state->integral;
    
    /*=========================================================================
     * STEP 4: Derivative Term (On Measurement, Filtered)
     *=========================================================================
     * DERIVATIVE ON MEASUREMENT:
     * Standard PID uses: D = Kd * d(error)/dt = Kd * d(SP - PV)/dt
     * 
     * Problem: When setpoint changes suddenly, d(SP)/dt creates a spike
     * called "derivative kick" or "setpoint kick".
     * 
     * Solution: Use derivative on measurement only:
     *   D = -Kd * d(PV)/dt
     * 
     * The negative sign comes from: d(error)/dt = d(SP)/dt - d(PV)/dt
     * If SP is constant (or we ignore it), d(error)/dt ≈ -d(PV)/dt
     * 
     * LOW-PASS FILTER:
     * Raw derivative = (measurement - prev_measurement) / dt
     * This amplifies high-frequency noise.
     * 
     * First-order LPF: d_filtered = α * d_raw + (1-α) * d_filtered_prev
     * where α = dt / (dt + τ), τ = 1/(2π*fc)
     * 
     * Smaller fc = more filtering = smoother but more phase lag
     */
    
    /* Calculate raw derivative (on measurement, so sign is negative for error derivative) */
    float derivative_raw = -(measurement - state->prev_measurement) / dt;
    
    /* Apply first-order low-pass filter */
    float alpha = state->derivative_alpha;
    state->derivative_filtered = alpha * derivative_raw + 
                                 (1.0f - alpha) * state->derivative_filtered;
    
    float d_term = gains->kd * state->derivative_filtered;
    
    /*=========================================================================
     * STEP 5: Calculate Output
     *=========================================================================
     * Sum all terms and apply output saturation
     */
    float output_unsat = p_term + i_term + d_term;
    float output_sat = constrainFloat(output_unsat, 
                                      pid->config.output_min, 
                                      pid->config.output_max);
    
    /* Update saturation flag */
    state->is_saturated = (fabsf(output_sat - output_unsat) > PID_EPSILON) ? 1 : 0;
    
    /*=========================================================================
     * STEP 6: Update State for Next Iteration
     *=========================================================================
     */
    state->prev_error = error;
    state->prev_measurement = measurement;
    state->prev_output_unsat = output_unsat;
    state->prev_output_sat = output_sat;
    
    /*=========================================================================
     * STEP 7: Output Results
     *=========================================================================
     */
    *output = output_sat;
    
    /* Fill debug structure if provided */
    if (debug != NULL) {
        debug->p_term = p_term;
        debug->i_term = i_term;
        debug->d_term = d_term;
        debug->output_unsat = output_unsat;
        debug->output_sat = output_sat;
        debug->error = error;
        debug->saturated = state->is_saturated;
    }
    
    return CTRL_OK;
}

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

int8_t pidSetGains(PidHandle_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    pid->config.gains.kp = kp;
    pid->config.gains.ki = ki;
    pid->config.gains.kd = kd;
    
    /* Update back-calculation gain when Ki changes */
    pid->state.kb = ki;
    
    return CTRL_OK;
}

int8_t pidSetOutputLimits(PidHandle_t *pid, float output_min, float output_max)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (output_min >= output_max) {
        return CTRL_ERR_INVALID_PARAM;
    }
    
    pid->config.output_min = output_min;
    pid->config.output_max = output_max;
    
    return CTRL_OK;
}

int8_t pidSetIntegralLimit(PidHandle_t *pid, float integral_max)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    pid->config.integral_max = fabsf(integral_max);
    
    return CTRL_OK;
}

int8_t pidSetDerivativeFilter(PidHandle_t *pid, float lpf_hz)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    pid->config.derivative_lpf_hz = lpf_hz;
    pid->state.derivative_alpha = calculateFilterAlpha(pid->config.sample_time, lpf_hz);
    
    return CTRL_OK;
}

int8_t pidSetSampleTime(PidHandle_t *pid, float sample_time)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (sample_time < PID_EPSILON) {
        return CTRL_ERR_INVALID_PARAM;
    }
    
    pid->config.sample_time = sample_time;
    
    /* Recalculate filter coefficient */
    pid->state.derivative_alpha = calculateFilterAlpha(
        sample_time, 
        pid->config.derivative_lpf_hz
    );
    
    return CTRL_OK;
}

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

int8_t pidIsSaturated(const PidHandle_t *pid, uint8_t *saturated)
{
    if (pid == NULL || saturated == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *saturated = pid->state.is_saturated;
    
    return CTRL_OK;
}

int8_t pidGetIntegral(const PidHandle_t *pid, float *integral)
{
    if (pid == NULL || integral == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *integral = pid->state.integral;
    
    return CTRL_OK;
}

int8_t pidSetIntegral(PidHandle_t *pid, float integral)
{
    if (pid == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!pid->state.is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    pid->state.integral = constrainFloat(integral,
                                         -pid->config.integral_max,
                                         pid->config.integral_max);
    
    return CTRL_OK;
}
