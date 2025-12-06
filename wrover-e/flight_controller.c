/**
 * @file flight_controller.c
 * @brief Implementation of main flight controller
 */

#include "flight_controller.h"
#include "ekf_quat.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t flightInit(FlightHandle_t *flight, const FlightConfig_t *config)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /* Clear structure */
    memset(flight, 0, sizeof(FlightHandle_t));
    
    /* Determine timing */
    float inner_dt = CTRL_RATE_PID_DT;
    float outer_dt = CTRL_ANGLE_PID_DT;
    
    if (config != NULL) {
        if (config->inner_loop_dt > 0) {
            inner_dt = config->inner_loop_dt;
        }
        if (config->outer_loop_dt > 0) {
            outer_dt = config->outer_loop_dt;
        }
    }
    
    /*=========================================================================
     * Initialize Attitude Controller (Outer Loop)
     *=========================================================================
     */
    int8_t ret;
    
    if (config != NULL && config->att_config != NULL) {
        ret = attitudeInit(&flight->attitude, config->att_config);
    } else {
        ret = attitudeInitDefault(&flight->attitude, outer_dt);
    }
    
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize Rate Controller (Inner Loop)
     *=========================================================================
     */
    if (config != NULL && config->rate_config != NULL) {
        ret = rateInit(&flight->rate, config->rate_config);
    } else {
        ret = rateInitDefault(&flight->rate, inner_dt);
    }
    
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize Motor Mixer
     *=========================================================================
     */
    if (config != NULL && config->mixer_config != NULL) {
        ret = mixerInit(&flight->mixer, config->mixer_config);
    } else {
        ret = mixerInitDefault(&flight->mixer);
    }
    
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize State
     *=========================================================================
     */
    flight->ekf = NULL;
    quatIdentity(&flight->q_current);
    
    flight->rate_setpoint.roll_rate = 0.0f;
    flight->rate_setpoint.pitch_rate = 0.0f;
    flight->rate_setpoint.yaw_rate = 0.0f;
    
    flight->loop_counter = 0;
    flight->outer_loop_divider = FLIGHT_OUTER_LOOP_DIVIDER;
    
    flight->arm_state = ARM_STATE_DISARMED;
    flight->flight_mode = FLIGHT_MODE_STABILIZE;
    
    flight->status.is_armed = 0;
    flight->status.ekf_healthy = 0;
    flight->status.gyro_healthy = 1;
    flight->status.rc_healthy = 1;
    flight->status.motor_saturated = 0;
    
    flight->is_initialized = 1;
    
    return CTRL_OK;
}

int8_t flightInitDefault(FlightHandle_t *flight)
{
    return flightInit(flight, NULL);
}

int8_t flightSetEKF(FlightHandle_t *flight, EKF_Handle_t *ekf)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    flight->ekf = ekf;
    
    if (ekf != NULL) {
        flight->status.ekf_healthy = 1;
    }
    
    return CTRL_OK;
}

int8_t flightReset(FlightHandle_t *flight)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Reset sub-controllers */
    attitudeReset(&flight->attitude);
    rateReset(&flight->rate);
    mixerReset(&flight->mixer);
    
    /* Reset state */
    quatIdentity(&flight->q_current);
    flight->rate_setpoint.roll_rate = 0.0f;
    flight->rate_setpoint.pitch_rate = 0.0f;
    flight->rate_setpoint.yaw_rate = 0.0f;
    
    flight->loop_counter = 0;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Main Control Loop
 ******************************************************************************/

int8_t flightUpdate(FlightHandle_t *flight,
                    const ImuData_t *imu,
                    const RcCommand_t *rc,
                    MotorOutput_t *motors)
{
    return flightUpdateTelemetry(flight, imu, rc, motors, NULL);
}

int8_t flightUpdateTelemetry(FlightHandle_t *flight,
                             const ImuData_t *imu,
                             const RcCommand_t *rc,
                             MotorOutput_t *motors,
                             FlightTelemetry_t *telem)
{
    if (flight == NULL || imu == NULL || rc == NULL || motors == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    int8_t ret;
    uint8_t outer_loop_ran = 0;
    
    /* Store RC command */
    flight->last_rc_command = *rc;
    
    /*=========================================================================
     * STEP 1: Check Arm State
     *=========================================================================
     * If disarmed, skip controllers and set motors to zero.
     */
    if (flight->arm_state == ARM_STATE_DISARMED) {
        mixerDisarm(&flight->mixer, motors);
        flight->last_motor_output = *motors;
        flight->status.is_armed = 0;
        
        /* Still update loop counter and EKF for attitude monitoring */
        flight->loop_counter++;
        
        /* Run EKF even when disarmed (for attitude display) */
        if (flight->ekf != NULL) {
            float gyro[3] = {imu->gyro_x, imu->gyro_y, imu->gyro_z};
            float accel[3] = {imu->accel_x, imu->accel_y, imu->accel_z};
            
            ekfPredict(flight->ekf, gyro);
            
            if ((flight->loop_counter % flight->outer_loop_divider) == 0) {
                ekfCorrect(flight->ekf, accel);
                ekfGetQuaternion(flight->ekf, &flight->q_current);
            }
        }
        
        /* Fill telemetry if requested */
        if (telem != NULL) {
            telem->loop_count = flight->loop_counter;
            telem->outer_loop_ran = 0;
            telem->arm_state = ARM_STATE_DISARMED;
            
            /* Get attitude from EKF if available */
            if (flight->ekf != NULL) {
                EulerAngles_t euler;
                quatToEulerZYX(&flight->q_current, &euler, NULL);
                telem->roll_deg = CTRL_RAD_TO_DEG(euler.roll);
                telem->pitch_deg = CTRL_RAD_TO_DEG(euler.pitch);
                telem->yaw_deg = CTRL_RAD_TO_DEG(euler.yaw);
            }
            
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                telem->motor[i] = 0.0f;
            }
        }
        
        return CTRL_OK;
    }
    
    /*=========================================================================
     * STEP 2: Run EKF and Outer Loop (500 Hz)
     *=========================================================================
     * Every 2 iterations (500 Hz), we:
     * 1. Run EKF predict + correct
     * 2. Run attitude controller
     * 
     * This provides updated rate setpoints for the inner loop.
     */
    if ((flight->loop_counter % flight->outer_loop_divider) == 0) {
        outer_loop_ran = 1;
        
        /* Run EKF if available */
        if (flight->ekf != NULL) {
            float gyro[3] = {imu->gyro_x, imu->gyro_y, imu->gyro_z};
            float accel[3] = {imu->accel_x, imu->accel_y, imu->accel_z};
            
            /* EKF predict (uses gyro) */
            ret = ekfPredict(flight->ekf, gyro);
            if (ret != EKF_OK) {
                flight->status.ekf_healthy = 0;
            }
            
            /* EKF correct (uses accel) */
            ret = ekfCorrect(flight->ekf, accel);
            if (ret != EKF_OK) {
                flight->status.ekf_healthy = 0;
            }
            
            /* Get updated quaternion */
            ekfGetQuaternion(flight->ekf, &flight->q_current);
        }
        
        /* Run attitude controller */
        AttitudeSetpoint_t att_sp = {
            .roll_deg = rc->roll,
            .pitch_deg = rc->pitch,
            .yaw_rate_dps = rc->yaw
        };
        
        ret = attitudeUpdate(&flight->attitude, &att_sp, &flight->q_current,
                            &flight->rate_setpoint);
        if (ret != CTRL_OK) {
            /* On error, zero rate setpoints */
            flight->rate_setpoint.roll_rate = 0.0f;
            flight->rate_setpoint.pitch_rate = 0.0f;
            flight->rate_setpoint.yaw_rate = 0.0f;
        }
    }
    
    /*=========================================================================
     * STEP 3: Run Inner Loop (1000 Hz)
     *=========================================================================
     * Every iteration:
     * 1. Convert gyro to deg/s
     * 2. Run rate controller
     * 3. Run mixer
     */
    
    /* Convert gyro from rad/s to deg/s */
    GyroData_t gyro_dps = {
        .roll = CTRL_RAD_TO_DEG(imu->gyro_x),
        .pitch = CTRL_RAD_TO_DEG(imu->gyro_y),
        .yaw = CTRL_RAD_TO_DEG(imu->gyro_z)
    };
    
    /* Run rate controller */
    RateOutput_t rate_output;
    ret = rateUpdate(&flight->rate, &flight->rate_setpoint, &gyro_dps, &rate_output);
    if (ret != CTRL_OK) {
        rate_output.roll = 0.0f;
        rate_output.pitch = 0.0f;
        rate_output.yaw = 0.0f;
    }
    
    /*=========================================================================
     * STEP 4: Motor Mixing
     *=========================================================================
     */
    MixerInput_t mixer_input = {
        .throttle = rc->throttle,
        .roll = rate_output.roll,
        .pitch = rate_output.pitch,
        .yaw = rate_output.yaw
    };
    
    ret = mixerUpdate(&flight->mixer, &mixer_input, motors);
    if (ret != CTRL_OK) {
        /* On error, set motors to idle */
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motors->motor[i] = CTRL_MOTOR_IDLE;
        }
    }
    
    /* Update status */
    uint8_t saturated;
    mixerIsSaturated(&flight->mixer, &saturated);
    flight->status.motor_saturated = saturated;
    flight->status.is_armed = 1;
    
    /* Store output */
    flight->last_motor_output = *motors;
    
    /*=========================================================================
     * STEP 5: Update Counter and Telemetry
     *=========================================================================
     */
    flight->loop_counter++;
    
    if (telem != NULL) {
        telem->loop_count = flight->loop_counter;
        telem->outer_loop_ran = outer_loop_ran;
        telem->arm_state = flight->arm_state;
        
        /* Attitude */
        EulerAngles_t euler;
        quatToEulerZYX(&flight->q_current, &euler, NULL);
        telem->roll_deg = CTRL_RAD_TO_DEG(euler.roll);
        telem->pitch_deg = CTRL_RAD_TO_DEG(euler.pitch);
        telem->yaw_deg = CTRL_RAD_TO_DEG(euler.yaw);
        telem->q_current = flight->q_current;
        
        /* Attitude errors */
        attitudeGetError(&flight->attitude,
                        &telem->att_error_roll,
                        &telem->att_error_pitch,
                        &telem->att_error_yaw);
        
        /* Gyro and rate setpoints */
        telem->gyro_roll_dps = gyro_dps.roll;
        telem->gyro_pitch_dps = gyro_dps.pitch;
        telem->gyro_yaw_dps = gyro_dps.yaw;
        telem->rate_sp_roll = flight->rate_setpoint.roll_rate;
        telem->rate_sp_pitch = flight->rate_setpoint.pitch_rate;
        telem->rate_sp_yaw = flight->rate_setpoint.yaw_rate;
        
        /* Control outputs */
        telem->ctrl_roll = rate_output.roll;
        telem->ctrl_pitch = rate_output.pitch;
        telem->ctrl_yaw = rate_output.yaw;
        
        /* Motors */
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            telem->motor[i] = motors->motor[i];
        }
        
        telem->is_saturated = saturated;
    }
    
    return CTRL_OK;
}

int8_t flightUpdateWithQuat(FlightHandle_t *flight,
                            const Quaternion_t *q_current,
                            const float *gyro_rad,
                            const RcCommand_t *rc,
                            MotorOutput_t *motors)
{
    if (flight == NULL || q_current == NULL || gyro_rad == NULL ||
        rc == NULL || motors == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Store quaternion */
    flight->q_current = *q_current;
    
    /* Create IMU struct with just gyro (accel not used in this path) */
    ImuData_t imu = {
        .gyro_x = gyro_rad[0],
        .gyro_y = gyro_rad[1],
        .gyro_z = gyro_rad[2],
        .accel_x = 0.0f,
        .accel_y = 0.0f,
        .accel_z = 1.0f
    };
    
    /* Temporarily disable EKF for this update */
    EKF_Handle_t *ekf_backup = flight->ekf;
    flight->ekf = NULL;
    
    /* Force outer loop to run (since we have fresh quaternion) */
    flight->loop_counter = 0;
    
    int8_t ret = flightUpdate(flight, &imu, rc, motors);
    
    /* Restore EKF */
    flight->ekf = ekf_backup;
    
    return ret;
}

/*******************************************************************************
 * Arming Functions
 ******************************************************************************/

int8_t flightArm(FlightHandle_t *flight)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Reset controllers before arming */
    attitudeReset(&flight->attitude);
    rateReset(&flight->rate);
    mixerReset(&flight->mixer);
    
    /* Reset rate setpoints */
    flight->rate_setpoint.roll_rate = 0.0f;
    flight->rate_setpoint.pitch_rate = 0.0f;
    flight->rate_setpoint.yaw_rate = 0.0f;
    
    /* Arm */
    flight->arm_state = ARM_STATE_ARMED;
    flight->status.is_armed = 1;
    
    return CTRL_OK;
}

int8_t flightDisarm(FlightHandle_t *flight)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    flight->arm_state = ARM_STATE_DISARMED;
    flight->status.is_armed = 0;
    
    return CTRL_OK;
}

int8_t flightIsArmed(const FlightHandle_t *flight, uint8_t *armed)
{
    if (flight == NULL || armed == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *armed = (flight->arm_state == ARM_STATE_ARMED) ? 1 : 0;
    
    return CTRL_OK;
}

int8_t flightToggleArm(FlightHandle_t *flight)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (flight->arm_state == ARM_STATE_ARMED) {
        return flightDisarm(flight);
    } else {
        return flightArm(flight);
    }
}

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

int8_t flightSetAttitudeGains(FlightHandle_t *flight, CtrlAxis_e axis,
                              float kp, float ki, float kd)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    switch (axis) {
        case CTRL_AXIS_ROLL:
        case CTRL_AXIS_PITCH:
            /* Roll and pitch use same gains */
            return attitudeSetRollPitchGains(&flight->attitude, kp, ki, kd);
            
        case CTRL_AXIS_YAW:
            return attitudeSetYawGains(&flight->attitude, kp, ki, kd);
            
        default:
            return CTRL_ERR_INVALID_PARAM;
    }
}

int8_t flightSetRateGains(FlightHandle_t *flight, CtrlAxis_e axis,
                          float kp, float ki, float kd)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    switch (axis) {
        case CTRL_AXIS_ROLL:
        case CTRL_AXIS_PITCH:
            return rateSetRollPitchGains(&flight->rate, kp, ki, kd);
            
        case CTRL_AXIS_YAW:
            return rateSetYawGains(&flight->rate, kp, ki, kd);
            
        default:
            return CTRL_ERR_INVALID_PARAM;
    }
}

int8_t flightSetMode(FlightHandle_t *flight, FlightMode_e mode)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    flight->flight_mode = mode;
    
    /* Reset controllers when mode changes */
    attitudeReset(&flight->attitude);
    rateReset(&flight->rate);
    
    return CTRL_OK;
}

int8_t flightSetMixerScaling(FlightHandle_t *flight,
                             float roll_scale, float pitch_scale, float yaw_scale)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    return mixerSetScaling(&flight->mixer, roll_scale, pitch_scale, yaw_scale);
}

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

int8_t flightGetAttitude(const FlightHandle_t *flight,
                         float *roll, float *pitch, float *yaw)
{
    if (flight == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    EulerAngles_t euler;
    quatToEulerZYX(&flight->q_current, &euler, NULL);
    
    if (roll != NULL) {
        *roll = CTRL_RAD_TO_DEG(euler.roll);
    }
    if (pitch != NULL) {
        *pitch = CTRL_RAD_TO_DEG(euler.pitch);
    }
    if (yaw != NULL) {
        *yaw = CTRL_RAD_TO_DEG(euler.yaw);
    }
    
    return CTRL_OK;
}

int8_t flightGetQuaternion(const FlightHandle_t *flight, Quaternion_t *q)
{
    if (flight == NULL || q == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *q = flight->q_current;
    
    return CTRL_OK;
}

int8_t flightGetStatus(const FlightHandle_t *flight, SystemStatus_t *status)
{
    if (flight == NULL || status == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *status = flight->status;
    
    return CTRL_OK;
}

int8_t flightGetMotors(const FlightHandle_t *flight, MotorOutput_t *motors)
{
    if (flight == NULL || motors == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!flight->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *motors = flight->last_motor_output;
    
    return CTRL_OK;
}
