/**
 * @file attitude_controller.c
 * @brief Implementation of quaternion-based attitude controller
 * 
 * @details PURE QUATERNION IMPLEMENTATION - NO GIMBAL LOCK
 * 
 * This implementation avoids Euler angle conversions during runtime.
 * All attitude error calculations are performed directly in quaternion space.
 * 
 * KEY PRINCIPLE:
 * ==============
 * Instead of: RC angles → Euler → Quaternion → Error
 * We use:     RC angles → Incremental quaternion → Update desired quat → Error
 * 
 * For ROLL/PITCH (absolute angle mode):
 *   - We construct a "level reference" quaternion that only contains yaw
 *   - Then apply roll/pitch rotations to this reference
 *   - This avoids Euler extraction entirely
 * 
 * For YAW (rate mode):
 *   - Yaw stick controls yaw RATE, not angle
 *   - We integrate yaw rate into desired quaternion incrementally
 *   - No need to extract current yaw angle
 * 
 * QUATERNION ERROR:
 * =================
 * q_error = q_desired⁻¹ ⊗ q_current
 * 
 * The vector part (q1, q2, q3) directly gives us the rotation error
 * about body axes, no Euler conversion needed.
 */

#include "attitude_controller.h"
#include <math.h>
#include <stddef.h>

/*******************************************************************************
 * Constants
 ******************************************************************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ATT_EPSILON 1e-6f

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
 * @brief Create quaternion from axis-angle representation
 * 
 * @param[out] q        Output quaternion
 * @param[in]  axis_x   Rotation axis X component (normalized)
 * @param[in]  axis_y   Rotation axis Y component (normalized)
 * @param[in]  axis_z   Rotation axis Z component (normalized)
 * @param[in]  angle    Rotation angle [radians]
 * 
 * @details q = [cos(θ/2), sin(θ/2)*axis]
 *          This is fundamental quaternion construction - no Euler involved!
 */
static void quatFromAxisAngle(Quaternion_t *q, 
                              float axis_x, float axis_y, float axis_z,
                              float angle)
{
    float half_angle = angle * 0.5f;
    float sin_half = sinf(half_angle);
    float cos_half = cosf(half_angle);
    
    q->q0 = cos_half;
    q->q1 = axis_x * sin_half;
    q->q2 = axis_y * sin_half;
    q->q3 = axis_z * sin_half;
}

/**
 * @brief Create desired attitude quaternion from roll/pitch angles
 * 
 * @details This function creates the desired quaternion WITHOUT extracting
 *          Euler angles from current attitude.
 * 
 *          Method:
 *          1. Extract ONLY the yaw component from current quaternion
 *             (this is safe - we're extracting yaw, not computing near ±90° pitch)
 *          2. Apply desired roll and pitch to this yaw-only reference
 * 
 *          The yaw extraction is done via:
 *          q_yaw = normalize([q0, 0, 0, q3])  -- projects to yaw-only rotation
 * 
 *          This avoids the gimbal lock issue because:
 *          - We never compute pitch angle
 *          - We only extract yaw, which is safe
 *          - Roll and pitch are applied as axis-angle rotations
 */
static void createDesiredQuaternion(const Quaternion_t *q_current,
                                    float roll_rad, float pitch_rad,
                                    Quaternion_t *q_desired)
{
    /*=========================================================================
     * Step 1: Extract yaw-only component from current quaternion
     *=========================================================================
     * For a ZYX rotation, the yaw component can be extracted by projecting
     * the quaternion onto the Z-axis rotation subspace.
     * 
     * q_yaw = normalize([q0, 0, 0, q3])
     * 
     * This gives us a quaternion that represents ONLY the yaw rotation,
     * with roll and pitch removed.
     */
    float q0 = q_current->q0;
    float q3 = q_current->q3;
    
    /* Normalize the yaw-only quaternion */
    float yaw_norm = sqrtf(q0 * q0 + q3 * q3);
    
    Quaternion_t q_yaw;
    if (yaw_norm > ATT_EPSILON) {
        q_yaw.q0 = q0 / yaw_norm;
        q_yaw.q1 = 0.0f;
        q_yaw.q2 = 0.0f;
        q_yaw.q3 = q3 / yaw_norm;
    } else {
        /* Degenerate case - use identity */
        quatIdentity(&q_yaw);
    }
    
    /*=========================================================================
     * Step 2: Create roll rotation quaternion (about X axis)
     *=========================================================================
     * q_roll = [cos(φ/2), sin(φ/2), 0, 0]
     */
    Quaternion_t q_roll;
    quatFromAxisAngle(&q_roll, 1.0f, 0.0f, 0.0f, roll_rad);
    
    /*=========================================================================
     * Step 3: Create pitch rotation quaternion (about Y axis)
     *=========================================================================
     * q_pitch = [cos(θ/2), 0, sin(θ/2), 0]
     */
    Quaternion_t q_pitch;
    quatFromAxisAngle(&q_pitch, 0.0f, 1.0f, 0.0f, pitch_rad);
    
    /*=========================================================================
     * Step 4: Combine: q_desired = q_yaw ⊗ q_pitch ⊗ q_roll
     *=========================================================================
     * This applies rotations in ZYX order (yaw, then pitch, then roll)
     * which matches our convention.
     * 
     * Note: Quaternion multiplication order is "reverse" of rotation order
     * q_total = q_first ⊗ q_second means: apply q_second first, then q_first
     * 
     * So for ZYX: q = q_yaw ⊗ q_pitch ⊗ q_roll
     */
    Quaternion_t q_temp;
    quatMultiply(&q_yaw, &q_pitch, &q_temp);
    quatMultiply(&q_temp, &q_roll, q_desired);
    
    /* Ensure normalized */
    quatNormalize(q_desired);
}

/**
 * @brief Update desired yaw by integrating yaw rate command
 * 
 * @details Instead of extracting yaw angle and modifying it, we apply
 *          an incremental yaw rotation directly to the desired quaternion.
 * 
 *          Δq_yaw = [cos(ω_z*dt/2), 0, 0, sin(ω_z*dt/2)]
 *          q_desired_new = q_desired_old ⊗ Δq_yaw  (for body-frame yaw)
 *          
 *          Or for NED frame yaw:
 *          q_desired_new = Δq_yaw ⊗ q_desired_old
 */
static void integrateYawRate(Quaternion_t *q_desired, float yaw_rate_rad, float dt)
{
    /* Small angle yaw increment */
    float delta_yaw = yaw_rate_rad * dt;
    
    /* Create incremental yaw quaternion */
    Quaternion_t dq_yaw;
    quatFromAxisAngle(&dq_yaw, 0.0f, 0.0f, 1.0f, delta_yaw);
    
    /* Apply: q_new = dq_yaw ⊗ q_old (NED frame yaw rotation) */
    Quaternion_t q_temp = *q_desired;
    quatMultiply(&dq_yaw, &q_temp, q_desired);
    
    quatNormalize(q_desired);
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t attitudeInit(AttitudeHandle_t *att, const AttitudeConfig_t *config)
{
    if (att == NULL || config == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /* Store configuration */
    att->config = *config;
    
    /*=========================================================================
     * Initialize Roll PID
     *=========================================================================
     */
    PidConfig_t roll_config = {
        .gains = config->roll_pitch_gains,
        .output_min = -config->max_roll_rate,
        .output_max = config->max_roll_rate,
        .integral_max = config->max_roll_rate * 0.5f,
        .derivative_lpf_hz = CTRL_DERIVATIVE_LPF_HZ,
        .sample_time = config->sample_time
    };
    
    int8_t ret = pidInit(&att->pid_roll, &roll_config);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize Pitch PID
     *=========================================================================
     */
    PidConfig_t pitch_config = {
        .gains = config->roll_pitch_gains,
        .output_min = -config->max_pitch_rate,
        .output_max = config->max_pitch_rate,
        .integral_max = config->max_pitch_rate * 0.5f,
        .derivative_lpf_hz = CTRL_DERIVATIVE_LPF_HZ,
        .sample_time = config->sample_time
    };
    
    ret = pidInit(&att->pid_pitch, &pitch_config);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * Initialize Yaw PID
     *=========================================================================
     */
    PidConfig_t yaw_config = {
        .gains = config->yaw_gains,
        .output_min = -config->max_yaw_rate,
        .output_max = config->max_yaw_rate,
        .integral_max = config->max_yaw_rate * 0.5f,
        .derivative_lpf_hz = CTRL_DERIVATIVE_LPF_HZ,
        .sample_time = config->sample_time
    };
    
    ret = pidInit(&att->pid_yaw, &yaw_config);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /* Initialize desired quaternion to identity */
    quatIdentity(&att->q_desired);
    quatIdentity(&att->q_error);
    
    /* Clear error values */
    att->error_roll_deg = 0.0f;
    att->error_pitch_deg = 0.0f;
    att->error_yaw_deg = 0.0f;
    
    att->is_initialized = 1;
    
    return CTRL_OK;
}

int8_t attitudeInitDefault(AttitudeHandle_t *att, float sample_time)
{
    if (att == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    AttitudeConfig_t config = {
        .roll_pitch_gains = {
            .kp = ATT_DEFAULT_KP_ROLL_PITCH,
            .ki = ATT_DEFAULT_KI_ROLL_PITCH,
            .kd = ATT_DEFAULT_KD_ROLL_PITCH
        },
        .yaw_gains = {
            .kp = ATT_DEFAULT_KP_YAW,
            .ki = ATT_DEFAULT_KI_YAW,
            .kd = ATT_DEFAULT_KD_YAW
        },
        .max_roll_rate = CTRL_MAX_ROLL_RATE_DPS,
        .max_pitch_rate = CTRL_MAX_PITCH_RATE_DPS,
        .max_yaw_rate = CTRL_MAX_YAW_RATE_DPS,
        .max_roll_angle = CTRL_MAX_ROLL_ANGLE_DEG,
        .max_pitch_angle = CTRL_MAX_PITCH_ANGLE_DEG,
        .sample_time = sample_time
    };
    
    return attitudeInit(att, &config);
}

int8_t attitudeReset(AttitudeHandle_t *att)
{
    if (att == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /* Reset all PIDs */
    pidReset(&att->pid_roll);
    pidReset(&att->pid_pitch);
    pidReset(&att->pid_yaw);
    
    /* Reset desired to identity */
    quatIdentity(&att->q_desired);
    quatIdentity(&att->q_error);
    
    att->error_roll_deg = 0.0f;
    att->error_pitch_deg = 0.0f;
    att->error_yaw_deg = 0.0f;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

int8_t attitudeUpdate(AttitudeHandle_t *att,
                      const AttitudeSetpoint_t *setpoint,
                      const Quaternion_t *q_current,
                      RateSetpoint_t *rate_setpoint)
{
    return attitudeUpdateDebug(att, setpoint, q_current, rate_setpoint, NULL);
}

int8_t attitudeUpdateDebug(AttitudeHandle_t *att,
                           const AttitudeSetpoint_t *setpoint,
                           const Quaternion_t *q_current,
                           RateSetpoint_t *rate_setpoint,
                           AttitudeDebug_t *debug)
{
    if (att == NULL || setpoint == NULL || q_current == NULL || rate_setpoint == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    /*=========================================================================
     * STEP 1: Constrain and Convert Setpoints
     *=========================================================================
     */
    float roll_cmd = constrainFloat(setpoint->roll_deg,
                                    -att->config.max_roll_angle,
                                    att->config.max_roll_angle);
    
    float pitch_cmd = constrainFloat(setpoint->pitch_deg,
                                     -att->config.max_pitch_angle,
                                     att->config.max_pitch_angle);
    
    float yaw_rate_cmd = constrainFloat(setpoint->yaw_rate_dps,
                                        -att->config.max_yaw_rate,
                                        att->config.max_yaw_rate);
    
    /* Convert to radians */
    float roll_rad = CTRL_DEG_TO_RAD(roll_cmd);
    float pitch_rad = CTRL_DEG_TO_RAD(pitch_cmd);
    float yaw_rate_rad = CTRL_DEG_TO_RAD(yaw_rate_cmd);
    
    /* Suppress unused variable warning - used in heading hold mode */
    (void)yaw_rate_rad;
    
    /*=========================================================================
     * STEP 2: Create Desired Quaternion (PURE QUATERNION - NO EULER!)
     *=========================================================================
     * This creates the target attitude quaternion using:
     * 1. Yaw extracted from current orientation (safe operation)
     * 2. Commanded roll/pitch applied as axis-angle rotations
     * 
     * NO gimbal-lock-prone Euler extraction!
     */
    createDesiredQuaternion(q_current, roll_rad, pitch_rad, &att->q_desired);
    
    /*=========================================================================
     * STEP 3: Integrate Yaw Rate Command (Optional heading hold)
     *=========================================================================
     * If you want heading hold instead of rate mode, uncomment this:
     * 
     * integrateYawRate(&att->q_desired, yaw_rate_rad, att->config.sample_time);
     * 
     * For now, we use direct rate passthrough (more common for RC flying)
     */
    
    /*=========================================================================
     * STEP 4: Compute Quaternion Error (PURE QUATERNION!)
     *=========================================================================
     * q_error = q_desired⁻¹ ⊗ q_current
     * 
     * This is the rotation FROM desired TO current.
     * The vector part gives axis-angle error directly!
     */
    int8_t ret = attitudeComputeQuatError(&att->q_desired, q_current, &att->q_error);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * STEP 5: Extract Axis Errors from Quaternion (NOT Euler!)
     *=========================================================================
     * For a unit quaternion: q = [cos(θ/2), sin(θ/2)*axis]
     * 
     * For small angles: θ ≈ 2 * |q_vec|
     * And the axis is approximately q_vec / |q_vec|
     * 
     * So: error_x ≈ 2*q1, error_y ≈ 2*q2, error_z ≈ 2*q3 (in radians)
     * 
     * This is NOT Euler angles! This is axis-angle representation,
     * which doesn't have gimbal lock.
     * 
     * For larger angles, we could use: θ = 2*atan2(|q_vec|, q0)
     * But small angle approximation works well for control purposes.
     */
    float error_x_rad = 2.0f * att->q_error.q1;  /* Roll axis error */
    float error_y_rad = 2.0f * att->q_error.q2;  /* Pitch axis error */
    float error_z_rad = 2.0f * att->q_error.q3;  /* Yaw axis error */
    
    /* Convert to degrees for PID and telemetry */
    att->error_roll_deg = CTRL_RAD_TO_DEG(error_x_rad);
    att->error_pitch_deg = CTRL_RAD_TO_DEG(error_y_rad);
    att->error_yaw_deg = CTRL_RAD_TO_DEG(error_z_rad);
    
    /*=========================================================================
     * STEP 6: Run PIDs
     *=========================================================================
     * PID converts angle error to rate setpoint.
     * Input: error (degrees)
     * Output: rate setpoint (deg/s)
     */
    
    /* Roll PID */
    float roll_rate_out;
    ret = pidUpdateDebug(&att->pid_roll, 0.0f, -att->error_roll_deg, &roll_rate_out,
                         debug ? &debug->pid_debug_roll : NULL);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /* Pitch PID */
    float pitch_rate_out;
    ret = pidUpdateDebug(&att->pid_pitch, 0.0f, -att->error_pitch_deg, &pitch_rate_out,
                         debug ? &debug->pid_debug_pitch : NULL);
    if (ret != CTRL_OK) {
        return ret;
    }
    
    /*=========================================================================
     * STEP 7: Yaw Handling (Rate Mode)
     *=========================================================================
     * For yaw, we typically use rate mode:
     * - Stick directly commands yaw rate
     * - No angle hold (pilot controls heading manually)
     * 
     * Alternative: Use yaw PID for heading hold mode
     */
    float yaw_rate_out = yaw_rate_cmd;  /* Direct passthrough */
    
    /* Fill debug yaw PID with zeros for rate mode */
    if (debug != NULL) {
        debug->pid_debug_yaw.p_term = 0;
        debug->pid_debug_yaw.i_term = 0;
        debug->pid_debug_yaw.d_term = 0;
        debug->pid_debug_yaw.output_sat = yaw_rate_out;
        debug->pid_debug_yaw.error = 0;
    }
    
    /*=========================================================================
     * STEP 8: Output Rate Setpoints
     *=========================================================================
     */
    rate_setpoint->roll_rate = roll_rate_out;
    rate_setpoint->pitch_rate = pitch_rate_out;
    rate_setpoint->yaw_rate = yaw_rate_out;
    
    /* Fill debug output */
    if (debug != NULL) {
        debug->q_error_x = att->q_error.q1;
        debug->q_error_y = att->q_error.q2;
        debug->q_error_z = att->q_error.q3;
        debug->error_roll_deg = att->error_roll_deg;
        debug->error_pitch_deg = att->error_pitch_deg;
        debug->error_yaw_deg = att->error_yaw_deg;
        debug->rate_sp_roll = roll_rate_out;
        debug->rate_sp_pitch = pitch_rate_out;
        debug->rate_sp_yaw = yaw_rate_out;
    }
    
    return CTRL_OK;
}

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

int8_t attitudeSetRollPitchGains(AttitudeHandle_t *att, float kp, float ki, float kd)
{
    if (att == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    att->config.roll_pitch_gains.kp = kp;
    att->config.roll_pitch_gains.ki = ki;
    att->config.roll_pitch_gains.kd = kd;
    
    pidSetGains(&att->pid_roll, kp, ki, kd);
    pidSetGains(&att->pid_pitch, kp, ki, kd);
    
    return CTRL_OK;
}

int8_t attitudeSetYawGains(AttitudeHandle_t *att, float kp, float ki, float kd)
{
    if (att == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    att->config.yaw_gains.kp = kp;
    att->config.yaw_gains.ki = ki;
    att->config.yaw_gains.kd = kd;
    
    pidSetGains(&att->pid_yaw, kp, ki, kd);
    
    return CTRL_OK;
}

int8_t attitudeSetRateLimits(AttitudeHandle_t *att,
                             float max_roll_rate,
                             float max_pitch_rate,
                             float max_yaw_rate)
{
    if (att == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    att->config.max_roll_rate = max_roll_rate;
    att->config.max_pitch_rate = max_pitch_rate;
    att->config.max_yaw_rate = max_yaw_rate;
    
    pidSetOutputLimits(&att->pid_roll, -max_roll_rate, max_roll_rate);
    pidSetOutputLimits(&att->pid_pitch, -max_pitch_rate, max_pitch_rate);
    pidSetOutputLimits(&att->pid_yaw, -max_yaw_rate, max_yaw_rate);
    
    return CTRL_OK;
}

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

int8_t attitudeGetError(const AttitudeHandle_t *att,
                        float *roll_err, float *pitch_err, float *yaw_err)
{
    if (att == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    if (roll_err != NULL) {
        *roll_err = att->error_roll_deg;
    }
    if (pitch_err != NULL) {
        *pitch_err = att->error_pitch_deg;
    }
    if (yaw_err != NULL) {
        *yaw_err = att->error_yaw_deg;
    }
    
    return CTRL_OK;
}

int8_t attitudeGetErrorQuat(const AttitudeHandle_t *att, Quaternion_t *q_error)
{
    if (att == NULL || q_error == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    if (!att->is_initialized) {
        return CTRL_ERR_NOT_INITIALIZED;
    }
    
    *q_error = att->q_error;
    
    return CTRL_OK;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

int8_t attitudeEulerToQuat(float roll_deg, float pitch_deg, float yaw_deg,
                           Quaternion_t *q)
{
    if (q == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /* Convert degrees to radians */
    EulerAngles_t euler = {
        .roll = CTRL_DEG_TO_RAD(roll_deg),
        .pitch = CTRL_DEG_TO_RAD(pitch_deg),
        .yaw = CTRL_DEG_TO_RAD(yaw_deg)
    };
    
    /* Use quaternion_math library function */
    int8_t ret = eulerToQuatZYX(&euler, q);
    
    return (ret == QUAT_OK) ? CTRL_OK : CTRL_ERR_COMPUTATION;
}

int8_t attitudeComputeQuatError(const Quaternion_t *q_desired,
                                const Quaternion_t *q_current,
                                Quaternion_t *q_error)
{
    if (q_desired == NULL || q_current == NULL || q_error == NULL) {
        return CTRL_ERR_NULL_PTR;
    }
    
    /*=========================================================================
     * Quaternion Error Calculation
     *=========================================================================
     * q_error = q_desired⁻¹ ⊗ q_current
     * 
     * For unit quaternions: q⁻¹ = conjugate(q) = [q0, -q1, -q2, -q3]
     */
    
    /* Get conjugate of desired quaternion */
    Quaternion_t q_des_conj;
    quatConjugate(q_desired, &q_des_conj);
    
    /* Multiply: q_error = conj(q_desired) * q_current */
    quatMultiply(&q_des_conj, q_current, q_error);
    
    /*=========================================================================
     * Ensure Shortest Path (q0 > 0)
     *=========================================================================
     * q and -q represent the same rotation.
     * We want the shorter path (rotation < 180°), which has q0 > 0.
     */
    if (q_error->q0 < 0.0f) {
        q_error->q0 = -q_error->q0;
        q_error->q1 = -q_error->q1;
        q_error->q2 = -q_error->q2;
        q_error->q3 = -q_error->q3;
    }
    
    return CTRL_OK;
}
