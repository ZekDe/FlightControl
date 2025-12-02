/**
 * @file rate_controller.c
 * @brief Implementation of angular rate controller (inner loop)
 */

#include "rate_controller.h"
#include <math.h>
#include <stddef.h>

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t rateInit(RateHandle_t *rate, const RateConfig_t *config)
{
    if (rate == NULL || config == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /* Store configuration */
    rate->config = *config;
    
    /*=========================================================================
     * Initialize Roll PID
     *=========================================================================
     * Input: rate setpoint [deg/s] and gyro measurement [deg/s]
     * Output: control command for mixer [-output_max, +output_max]
     */
    PidConfig_t roll_config = {
        .gains = config->roll_pitch_gains,
        .output_min = -config->output_max,
        .output_max = config->output_max,
        .integral_max = config->output_max * 0.4f,  /* Limit integral to 40% of max */
        .derivative_lpf_hz = CTRL_DERIVATIVE_LPF_HZ,
        .sample_time = config->sample_time
    };
    
    int8_t ret = pidInit(&rate->pid_roll, &roll_config);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize Pitch PID (same as roll for symmetric drone)
     *=========================================================================
     */
    PidConfig_t pitch_config = {
        .gains = config->roll_pitch_gains,
        .output_min = -config->output_max,
        .output_max = config->output_max,
        .integral_max = config->output_max * 0.4f,
        .derivative_lpf_hz = CTRL_DERIVATIVE_LPF_HZ,
        .sample_time = config->sample_time
    };
    
    ret = pidInit(&rate->pid_pitch, &pitch_config);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize Yaw PID
     *=========================================================================
     * Yaw has different gains because:
     * 1. Moment of inertia about Z is usually larger
     * 2. Yaw authority (motor differential) is typically less
     * 3. D term usually not needed (yaw is naturally damped)
     */
    PidConfig_t yaw_config = {
        .gains = config->yaw_gains,
        .output_min = -config->output_max,
        .output_max = config->output_max,
        .integral_max = config->output_max * 0.3f,  /* Less integral for yaw */
        .derivative_lpf_hz = CTRL_DERIVATIVE_LPF_HZ,
        .sample_time = config->sample_time
    };
    
    ret = pidInit(&rate->pid_yaw, &yaw_config);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /* Clear telemetry values */
    rate->gyro_roll_dps = 0.0f;
    rate->gyro_pitch_dps = 0.0f;
    rate->gyro_yaw_dps = 0.0f;
    rate->setpoint_roll_dps = 0.0f;
    rate->setpoint_pitch_dps = 0.0f;
    rate->setpoint_yaw_dps = 0.0f;
    
    rate->is_initialized = 1;
    
    return CTRL_OK;
}

int8_t rateInitDefault(RateHandle_t *rate, float sample_time)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    RateConfig_t config = {
        .roll_pitch_gains = {
            .kp = RATE_DEFAULT_KP_ROLL_PITCH,
            .ki = RATE_DEFAULT_KI_ROLL_PITCH,
            .kd = RATE_DEFAULT_KD_ROLL_PITCH
        },
        .yaw_gains = {
            .kp = RATE_DEFAULT_KP_YAW,
            .ki = RATE_DEFAULT_KI_YAW,
            .kd = RATE_DEFAULT_KD_YAW
        },
        .output_max = RATE_OUTPUT_MAX,
        .sample_time = sample_time
    };
    
    return rateInit(rate, &config);
}

int8_t rateReset(RateHandle_t *rate)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Reset all PIDs */
    pidReset(&rate->pid_roll);
    pidReset(&rate->pid_pitch);
    pidReset(&rate->pid_yaw);
    
    /* Clear telemetry */
    rate->gyro_roll_dps = 0.0f;
    rate->gyro_pitch_dps = 0.0f;
    rate->gyro_yaw_dps = 0.0f;
    rate->setpoint_roll_dps = 0.0f;
    rate->setpoint_pitch_dps = 0.0f;
    rate->setpoint_yaw_dps = 0.0f;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

int8_t rateUpdate(RateHandle_t *rate,
                  const RateSetpoint_t *setpoint,
                  const GyroData_t *gyro,
                  RateOutput_t *output)
{
    return rateUpdateDebug(rate, setpoint, gyro, output, NULL);
}

int8_t rateUpdateDebug(RateHandle_t *rate,
                       const RateSetpoint_t *setpoint,
                       const GyroData_t *gyro,
                       RateOutput_t *output,
                       RateDebug_t *debug)
{
    if (rate == NULL || setpoint == NULL || gyro == NULL || output == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /*=========================================================================
     * Store values for telemetry
     *=========================================================================
     */
    rate->gyro_roll_dps = gyro->roll;
    rate->gyro_pitch_dps = gyro->pitch;
    rate->gyro_yaw_dps = gyro->yaw;
    rate->setpoint_roll_dps = setpoint->roll_rate;
    rate->setpoint_pitch_dps = setpoint->pitch_rate;
    rate->setpoint_yaw_dps = setpoint->yaw_rate;
    
    /*=========================================================================
     * Roll Rate PID
     *=========================================================================
     * error = setpoint - measurement
     * If setpoint = 100 deg/s and gyro = 80 deg/s, error = +20
     * Positive error -> increase roll rate -> positive output
     */
    float roll_output;
    int8_t ret = pidUpdateDebug(&rate->pid_roll,
                                setpoint->roll_rate,
                                gyro->roll,
                                &roll_output,
                                debug ? &debug->pid_debug_roll : NULL);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Pitch Rate PID
     *=========================================================================
     */
    float pitch_output;
    ret = pidUpdateDebug(&rate->pid_pitch,
                         setpoint->pitch_rate,
                         gyro->pitch,
                         &pitch_output,
                         debug ? &debug->pid_debug_pitch : NULL);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Yaw Rate PID
     *=========================================================================
     */
    float yaw_output;
    ret = pidUpdateDebug(&rate->pid_yaw,
                         setpoint->yaw_rate,
                         gyro->yaw,
                         &yaw_output,
                         debug ? &debug->pid_debug_yaw : NULL);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Output
     *=========================================================================
     */
    output->roll = roll_output;
    output->pitch = pitch_output;
    output->yaw = yaw_output;
    
    /* Fill debug structure */
    if (debug != NULL) {
        debug->setpoint_roll = setpoint->roll_rate;
        debug->setpoint_pitch = setpoint->pitch_rate;
        debug->setpoint_yaw = setpoint->yaw_rate;
        
        debug->gyro_roll = gyro->roll;
        debug->gyro_pitch = gyro->pitch;
        debug->gyro_yaw = gyro->yaw;
        
        debug->error_roll = setpoint->roll_rate - gyro->roll;
        debug->error_pitch = setpoint->pitch_rate - gyro->pitch;
        debug->error_yaw = setpoint->yaw_rate - gyro->yaw;
        
        debug->output_roll = roll_output;
        debug->output_pitch = pitch_output;
        debug->output_yaw = yaw_output;
    }
    
    return CTRL_OK;
}

int8_t rateUpdateRaw(RateHandle_t *rate,
                     const RateSetpoint_t *setpoint,
                     float gyro_rad_x, float gyro_rad_y, float gyro_rad_z,
                     RateOutput_t *output)
{
    /* Convert rad/s to deg/s */
    GyroData_t gyro = {
        .roll = CTRL_RAD_TO_DEG(gyro_rad_x),
        .pitch = CTRL_RAD_TO_DEG(gyro_rad_y),
        .yaw = CTRL_RAD_TO_DEG(gyro_rad_z)
    };
    
    return rateUpdate(rate, setpoint, &gyro, output);
}

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

int8_t rateSetRollPitchGains(RateHandle_t *rate, float kp, float ki, float kd)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    rate->config.roll_pitch_gains.kp = kp;
    rate->config.roll_pitch_gains.ki = ki;
    rate->config.roll_pitch_gains.kd = kd;
    
    pidSetGains(&rate->pid_roll, kp, ki, kd);
    pidSetGains(&rate->pid_pitch, kp, ki, kd);
    
    return CTRL_OK;
}

int8_t rateSetYawGains(RateHandle_t *rate, float kp, float ki, float kd)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    rate->config.yaw_gains.kp = kp;
    rate->config.yaw_gains.ki = ki;
    rate->config.yaw_gains.kd = kd;
    
    pidSetGains(&rate->pid_yaw, kp, ki, kd);
    
    return CTRL_OK;
}

int8_t rateSetAllGains(RateHandle_t *rate,
                       const PidGains_t *roll_pitch,
                       const PidGains_t *yaw)
{
    if (rate == NULL || roll_pitch == NULL || yaw == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    int8_t ret = rateSetRollPitchGains(rate, roll_pitch->kp, roll_pitch->ki, roll_pitch->kd);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    return rateSetYawGains(rate, yaw->kp, yaw->ki, yaw->kd);
}

int8_t rateSetOutputLimit(RateHandle_t *rate, float output_max)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    rate->config.output_max = output_max;
    
    pidSetOutputLimits(&rate->pid_roll, -output_max, output_max);
    pidSetOutputLimits(&rate->pid_pitch, -output_max, output_max);
    pidSetOutputLimits(&rate->pid_yaw, -output_max, output_max);
    
    return CTRL_OK;
}

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

int8_t rateGetError(const RateHandle_t *rate,
                    float *roll_err, float *pitch_err, float *yaw_err)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (roll_err != NULL) {
        *roll_err = rate->setpoint_roll_dps - rate->gyro_roll_dps;
    }
    if (pitch_err != NULL) {
        *pitch_err = rate->setpoint_pitch_dps - rate->gyro_pitch_dps;
    }
    if (yaw_err != NULL) {
        *yaw_err = rate->setpoint_yaw_dps - rate->gyro_yaw_dps;
    }
    
    return CTRL_OK;
}

int8_t rateIsSaturated(const RateHandle_t *rate,
                       uint8_t *roll_sat, uint8_t *pitch_sat, uint8_t *yaw_sat)
{
    if (rate == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!rate->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (roll_sat != NULL) {
        pidIsSaturated(&rate->pid_roll, roll_sat);
    }
    if (pitch_sat != NULL) {
        pidIsSaturated(&rate->pid_pitch, pitch_sat);
    }
    if (yaw_sat != NULL) {
        pidIsSaturated(&rate->pid_yaw, yaw_sat);
    }
    
    return CTRL_OK;
}
