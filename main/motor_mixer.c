/**
 * @file motor_mixer.c
 * @brief Implementation of quadcopter motor mixer (X configuration)
 */

#include "motor_mixer.h"
#include <math.h>
#include <stddef.h>

/*******************************************************************************
 * Private Helper Functions
 ******************************************************************************/

/**
 * @brief Constrain float value
 */
static float constrainFloat(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Find maximum of 4 values
 */
static float max4(float a, float b, float c, float d)
{
    float max_ab = (a > b) ? a : b;
    float max_cd = (c > d) ? c : d;
    return (max_ab > max_cd) ? max_ab : max_cd;
}

/**
 * @brief Find minimum of 4 values
 */
static float min4(float a, float b, float c, float d)
{
    float min_ab = (a < b) ? a : b;
    float min_cd = (c < d) ? c : d;
    return (min_ab < min_cd) ? min_ab : min_cd;
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t mixerInit(MixerHandle_t *mixer, const MixerConfig_t *config)
{
    if (mixer == NULL || config == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /* Store configuration */
    mixer->config = *config;
    
    /* Clear motor outputs */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mixer->motor_output[i] = 0.0f;
    }
    
    /* Clear state */
    mixer->is_saturated = 0;
    mixer->max_command = 0.0f;
    mixer->min_motor = 0.0f;
    mixer->max_motor = 0.0f;
    
    mixer->is_initialized = 1;
    
    return CTRL_OK;
}

int8_t mixerInitDefault(MixerHandle_t *mixer)
{
    if (mixer == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    MixerConfig_t config = {
        .roll_scale = MIXER_DEFAULT_ROLL_SCALE,
        .pitch_scale = MIXER_DEFAULT_PITCH_SCALE,
        .yaw_scale = MIXER_DEFAULT_YAW_SCALE,
        .motor_min = CTRL_MOTOR_MIN,
        .motor_max = CTRL_MOTOR_MAX,
        .motor_idle = CTRL_MOTOR_IDLE,
        .airmode_enabled = 0
    };
    
    return mixerInit(mixer, &config);
}

int8_t mixerReset(MixerHandle_t *mixer)
{
    if (mixer == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Clear motor outputs */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mixer->motor_output[i] = 0.0f;
    }
    
    mixer->is_saturated = 0;
    mixer->max_command = 0.0f;
    mixer->min_motor = 0.0f;
    mixer->max_motor = 0.0f;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

int8_t mixerUpdate(MixerHandle_t *mixer,
                   const MixerInput_t *input,
                   MotorOutput_t *output)
{
    return mixerUpdateDebug(mixer, input, output, NULL);
}

int8_t mixerUpdateDebug(MixerHandle_t *mixer,
                        const MixerInput_t *input,
                        MotorOutput_t *output,
                        MixerDebug_t *debug)
{
    if (mixer == NULL || input == NULL || output == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /*=========================================================================
     * STEP 1: Scale Control Commands
     *=========================================================================
     * PID outputs have arbitrary units (e.g., ±500).
     * We scale them to a percentage that makes sense for mixing.
     * 
     * Example: If PID outputs ±500 and scale=0.1, command range is ±50%
     */
    float roll_cmd = input->roll * mixer->config.roll_scale;
    float pitch_cmd = input->pitch * mixer->config.pitch_scale;
    float yaw_cmd = input->yaw * mixer->config.yaw_scale;
    float throttle = input->throttle;
    
    /* Track maximum command for telemetry */
    float abs_roll = fabsf(roll_cmd);
    float abs_pitch = fabsf(pitch_cmd);
    float abs_yaw = fabsf(yaw_cmd);
    mixer->max_command = max4(abs_roll, abs_pitch, abs_yaw, mixer->max_command * 0.99f);
    
    /*=========================================================================
     * STEP 2: Compute Raw Motor Values (X Configuration)
     *=========================================================================
     * X Configuration Mixing Matrix:
     * 
     *         Throttle  Roll  Pitch  Yaw
     * M1 (FR):   +1      -1    -1    -1   (CW)
     * M2 (RR):   +1      -1    +1    +1   (CCW)
     * M3 (RL):   +1      +1    +1    -1   (CW)
     * M4 (FL):   +1      +1    -1    +1   (CCW)
     * 
     * Sign convention (right-hand rule, NED frame):
     * - +Roll  = right wing down = left side up
     * - +Pitch = nose up
     * - +Yaw   = nose to the right (clockwise from above)
     * 
     * For positive roll (right down):
     *   Need more lift on left (M3,M4), less on right (M1,M2)
     *   M1,M2 decrease, M3,M4 increase -> signs: M1=-1, M2=-1, M3=+1, M4=+1
     * 
     * For positive pitch (nose up):
     *   Need more lift on rear (M2,M3), less on front (M1,M4)
     *   M1,M4 decrease, M2,M3 increase -> signs: M1=-1, M2=+1, M3=+1, M4=-1
     * 
     * For positive yaw (nose right):
     *   Need more torque from CCW motors (M2,M4), less from CW (M1,M3)
     *   CW motors (M1,M3) produce CCW reaction, so reduce them
     *   CCW motors (M2,M4) produce CW reaction, so increase them
     *   M1,M3 decrease, M2,M4 increase -> signs: M1=-1, M2=+1, M3=-1, M4=+1
     */
    
    float motor_raw[MOTOR_COUNT];
    
    /* M1: Front-Right (CW rotation) */
    motor_raw[MOTOR_FRONT_RIGHT] = throttle - roll_cmd - pitch_cmd - yaw_cmd;
    
    /* M2: Rear-Right (CCW rotation) */
    motor_raw[MOTOR_REAR_RIGHT] = throttle - roll_cmd + pitch_cmd + yaw_cmd;
    
    /* M3: Rear-Left (CW rotation) */
    motor_raw[MOTOR_REAR_LEFT] = throttle + roll_cmd + pitch_cmd - yaw_cmd;
    
    /* M4: Front-Left (CCW rotation) */
    motor_raw[MOTOR_FRONT_LEFT] = throttle + roll_cmd - pitch_cmd + yaw_cmd;
    
    /* Track raw min/max for telemetry */
    mixer->min_motor = min4(motor_raw[0], motor_raw[1], motor_raw[2], motor_raw[3]);
    mixer->max_motor = max4(motor_raw[0], motor_raw[1], motor_raw[2], motor_raw[3]);
    
    /*=========================================================================
     * STEP 3: Handle Saturation
     *=========================================================================
     * Motors have physical limits (0-100%).
     * When throttle is low, control authority is limited.
     * When throttle is high, headroom for control is limited.
     * 
     * Simple approach: Constrain each motor to [min, max]
     * 
     * Advanced approaches (not implemented here):
     * - Throttle boost: Increase throttle to preserve control authority
     * - Priority mixing: Prioritize attitude over altitude
     * - Airmode: Allow negative motor commands (spin down) for control
     */
    
    mixer->is_saturated = 0;
    
    /* Determine minimum motor output */
    float effective_min = mixer->config.motor_min;
    
    /* If throttle is above 0, use idle as minimum (keeps motors spinning) */
    if (throttle > mixer->config.motor_idle) {
        effective_min = mixer->config.motor_idle;
    }
    
    /* With airmode, allow going below idle for control authority */
    if (mixer->config.airmode_enabled && throttle > 0) {
        effective_min = mixer->config.motor_min;
    }
    
    /* Constrain each motor */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        float motor_out = motor_raw[i];
        
        /* Check saturation */
        if (motor_out > mixer->config.motor_max) {
            mixer->is_saturated = 1;
            motor_out = mixer->config.motor_max;
            
            if (debug != NULL) {
                debug->saturated_high[i] = 1;
            }
        } else {
            if (debug != NULL) {
                debug->saturated_high[i] = 0;
            }
        }
        
        if (motor_out < effective_min) {
            mixer->is_saturated = 1;
            motor_out = effective_min;
            
            if (debug != NULL) {
                debug->saturated_low[i] = 1;
            }
        } else {
            if (debug != NULL) {
                debug->saturated_low[i] = 0;
            }
        }
        
        /* Store output */
        mixer->motor_output[i] = motor_out;
        output->motor[i] = motor_out;
    }
    
    /*=========================================================================
     * STEP 4: Fill Debug Output
     *=========================================================================
     */
    if (debug != NULL) {
        debug->throttle = throttle;
        debug->roll_scaled = roll_cmd;
        debug->pitch_scaled = pitch_cmd;
        debug->yaw_scaled = yaw_cmd;
        
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            debug->motor_raw[i] = motor_raw[i];
            debug->motor_out[i] = output->motor[i];
        }
        
        /* Calculate headroom (how much more throttle we can add) */
        debug->headroom = mixer->config.motor_max - mixer->max_motor;
    }
    
    return CTRL_OK;
}

int8_t mixerMix(MixerHandle_t *mixer,
                float throttle, float roll, float pitch, float yaw,
                MotorOutput_t *output)
{
    MixerInput_t input = {
        .throttle = throttle,
        .roll = roll,
        .pitch = pitch,
        .yaw = yaw
    };
    
    return mixerUpdate(mixer, &input, output);
}

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

int8_t mixerSetScaling(MixerHandle_t *mixer,
                       float roll_scale, float pitch_scale, float yaw_scale)
{
    if (mixer == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    mixer->config.roll_scale = roll_scale;
    mixer->config.pitch_scale = pitch_scale;
    mixer->config.yaw_scale = yaw_scale;
    
    return CTRL_OK;
}

int8_t mixerSetLimits(MixerHandle_t *mixer,
                      float motor_min, float motor_max, float motor_idle)
{
    if (mixer == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (motor_min >= motor_max || motor_idle > motor_max) {
        return CTRL_ERR_INVALID_PARAM;
    }
    
    mixer->config.motor_min = motor_min;
    mixer->config.motor_max = motor_max;
    mixer->config.motor_idle = motor_idle;
    
    return CTRL_OK;
}

int8_t mixerSetAirmode(MixerHandle_t *mixer, uint8_t enable)
{
    if (mixer == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    mixer->config.airmode_enabled = enable ? 1 : 0;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

int8_t mixerGetOutput(const MixerHandle_t *mixer, MotorOutput_t *output)
{
    if (mixer == NULL || output == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        output->motor[i] = mixer->motor_output[i];
    }
    
    return CTRL_OK;
}

int8_t mixerIsSaturated(const MixerHandle_t *mixer, uint8_t *saturated)
{
    if (mixer == NULL || saturated == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *saturated = mixer->is_saturated;
    
    return CTRL_OK;
}

int8_t mixerGetMotor(const MixerHandle_t *mixer, uint8_t motor, float *output)
{
    if (mixer == NULL || output == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (motor >= MOTOR_COUNT) {
        return CTRL_ERR_INVALID_PARAM;
    }
    
    *output = mixer->motor_output[motor];
    
    return CTRL_OK;
}

/*******************************************************************************
 * Disarm Function
 ******************************************************************************/

int8_t mixerDisarm(MixerHandle_t *mixer, MotorOutput_t *output)
{
    if (mixer == NULL || output == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!mixer->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Set all motors to zero */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        mixer->motor_output[i] = 0.0f;
        output->motor[i] = 0.0f;
    }
    
    mixer->is_saturated = 0;
    
    return CTRL_OK;
}
