/**
 * @file ekf_quat.c
 * @brief Implementation of Extended Kalman Filter for quaternion attitude estimation
 */

#include "ekf_quat.h"
#include <math.h>
#include <string.h>

/*******************************************************************************
 * Default Configuration Values
 ******************************************************************************/

/* Default process noise (Q matrix diagonal) */
#define DEFAULT_Q_QUAT      1e-6f   /**< Quaternion process noise */
#define DEFAULT_Q_BIAS      1e-8f   /**< Gyro bias process noise (drift) */

/* Default measurement noise (R matrix diagonal) */
#define DEFAULT_R_ACCEL     0.1f    /**< Accelerometer noise variance */

/* Default initial covariance (P matrix diagonal) */
#define DEFAULT_P_QUAT      1e-2f   /**< Initial quaternion uncertainty */
#define DEFAULT_P_BIAS      1e-4f   /**< Initial bias uncertainty */

/*******************************************************************************
 * Internal Helper Functions
 ******************************************************************************/

/**
 * @brief Normalize quaternion portion of state vector
 */
static int8_t normalizeStateQuaternion(float *x)
{
    float norm = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    
    if (norm < 1e-10f) {
        return EKF_ERR_INVALID_PARAM;
    }
    
    float inv_norm = 1.0f / norm;
    x[0] *= inv_norm;
    x[1] *= inv_norm;
    x[2] *= inv_norm;
    x[3] *= inv_norm;
    
    return EKF_OK;
}

/**
 * @brief Initialize workspace matrices
 */
static int8_t initWorkspaceMatrices(EKF_Handle_t *handle)
{
    int8_t ret;
    
    /* State transition Jacobian (7x7) */
    ret = matrixInit(&handle->F, EKF_STATE_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Measurement Jacobian (3x7) */
    ret = matrixInit(&handle->H, EKF_MEAS_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Kalman gain (7x3) */
    ret = matrixInit(&handle->K, EKF_STATE_DIM, EKF_MEAS_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Temporary 7x7 matrices */
    ret = matrixInit(&handle->temp7x7_1, EKF_STATE_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    ret = matrixInit(&handle->temp7x7_2, EKF_STATE_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    ret = matrixInit(&handle->temp7x7_3, EKF_STATE_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Temporary 7x3 matrices */
    ret = matrixInit(&handle->temp7x3, EKF_STATE_DIM, EKF_MEAS_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    ret = matrixInit(&handle->temp7x3_2, EKF_STATE_DIM, EKF_MEAS_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Temporary 3x7 matrix */
    ret = matrixInit(&handle->temp3x7, EKF_MEAS_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Temporary 3x3 matrices */
    ret = matrixInit(&handle->temp3x3, EKF_MEAS_DIM, EKF_MEAS_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    ret = matrixInit(&handle->temp3x3_2, EKF_MEAS_DIM, EKF_MEAS_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    return EKF_OK;
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

int8_t ekfInit(EKF_Handle_t *handle,
               EKF_TransitionFunc_t transitionFunc,
               EKF_MeasurementFunc_t measurementFunc,
               float dt)
{
    if (handle == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (dt <= 0.0f) {
        return EKF_ERR_INVALID_PARAM;
    }
    
    int8_t ret;
    
    /* Clear handle */
    memset(handle, 0, sizeof(EKF_Handle_t));
    
    /* Initialize state to identity quaternion and zero bias */
    /* x = [q0, q1, q2, q3, bx, by, bz] = [1, 0, 0, 0, 0, 0, 0] */
    handle->x[0] = 1.0f;  /* q0 = 1 (identity quaternion) */
    handle->x[1] = 0.0f;  /* q1 */
    handle->x[2] = 0.0f;  /* q2 */
    handle->x[3] = 0.0f;  /* q3 */
    handle->x[4] = 0.0f;  /* bx */
    handle->x[5] = 0.0f;  /* by */
    handle->x[6] = 0.0f;  /* bz */
    
    /* Initialize state covariance P */
    ret = matrixInit(&handle->P, EKF_STATE_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Set default initial covariance (diagonal) */
    for (uint8_t i = 0; i < 4; i++) {
        matrixSet(&handle->P, i, i, DEFAULT_P_QUAT);
    }
    for (uint8_t i = 4; i < 7; i++) {
        matrixSet(&handle->P, i, i, DEFAULT_P_BIAS);
    }
    
    /* Initialize process noise Q */
    ret = matrixInit(&handle->Q, EKF_STATE_DIM, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Set default process noise (diagonal) */
    for (uint8_t i = 0; i < 4; i++) {
        matrixSet(&handle->Q, i, i, DEFAULT_Q_QUAT);
    }
    for (uint8_t i = 4; i < 7; i++) {
        matrixSet(&handle->Q, i, i, DEFAULT_Q_BIAS);
    }
    
    /* Initialize measurement noise R */
    ret = matrixInit(&handle->R, EKF_MEAS_DIM, EKF_MEAS_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Set default measurement noise (diagonal) */
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        matrixSet(&handle->R, i, i, DEFAULT_R_ACCEL);
    }
    
    /* Initialize workspace matrices */
    ret = initWorkspaceMatrices(handle);
    if (ret != EKF_OK) return ret;
    
    /* Set callbacks - use defaults if not provided */
    handle->transitionFunc = (transitionFunc != NULL) ? transitionFunc : ekfDefaultTransitionFunc;
    handle->measurementFunc = (measurementFunc != NULL) ? measurementFunc : ekfDefaultMeasurementFunc;
    
    /* Set time step */
    handle->dt = dt;
    
    /* Initialize status */
    handle->gimbal_status.is_gimbal_lock = 0;
    handle->gimbal_status.pitch_clamped = 0;
    
    /* Mark as initialized */
    handle->is_initialized = 1;
    
    return EKF_OK;
}

int8_t ekfSetProcessNoise(EKF_Handle_t *handle, const EKF_ProcessNoise_t *noise)
{
    if (handle == NULL || noise == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    /* Set Q matrix diagonal */
    for (uint8_t i = 0; i < 4; i++) {
        matrixSet(&handle->Q, i, i, noise->q_quat);
    }
    for (uint8_t i = 4; i < 7; i++) {
        matrixSet(&handle->Q, i, i, noise->q_bias);
    }
    
    return EKF_OK;
}

int8_t ekfSetMeasurementNoise(EKF_Handle_t *handle, const EKF_MeasurementNoise_t *noise)
{
    if (handle == NULL || noise == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    /* Set R matrix diagonal */
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        matrixSet(&handle->R, i, i, noise->r_accel);
    }
    
    return EKF_OK;
}

int8_t ekfSetInitialCovariance(EKF_Handle_t *handle, float p_quat, float p_bias)
{
    if (handle == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    /* Reset P to zero and set diagonal */
    matrixInit(&handle->P, EKF_STATE_DIM, EKF_STATE_DIM);
    
    for (uint8_t i = 0; i < 4; i++) {
        matrixSet(&handle->P, i, i, p_quat);
    }
    for (uint8_t i = 4; i < 7; i++) {
        matrixSet(&handle->P, i, i, p_bias);
    }
    
    return EKF_OK;
}

int8_t ekfReset(EKF_Handle_t *handle)
{
    if (handle == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    /* Reset state to identity quaternion and zero bias */
    handle->x[0] = 1.0f;
    handle->x[1] = 0.0f;
    handle->x[2] = 0.0f;
    handle->x[3] = 0.0f;
    handle->x[4] = 0.0f;
    handle->x[5] = 0.0f;
    handle->x[6] = 0.0f;
    
    /* Reset covariance to defaults */
    ekfSetInitialCovariance(handle, DEFAULT_P_QUAT, DEFAULT_P_BIAS);
    
    /* Reset status */
    handle->gimbal_status.is_gimbal_lock = 0;
    handle->gimbal_status.pitch_clamped = 0;
    
    return EKF_OK;
}

/*******************************************************************************
 * EKF Core Functions
 ******************************************************************************/

int8_t ekfPredict(EKF_Handle_t *handle, const float *gyro)
{
    if (handle == NULL || gyro == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    int8_t ret;
    float x_new[EKF_STATE_DIM];
    
    /*
     * PREDICT STEP (Time Update)
     * 
     * 1. Propagate state: x_new = f(x, gyro, dt)
     * 2. Compute Jacobian F = df/dx
     * 3. Update covariance: P = F * P * F' + Q
     */
    
    /* Call transition function to get new state and Jacobian */
    ret = handle->transitionFunc(x_new, handle->F.data, handle->x, gyro, handle->dt, handle);
    if (ret != EKF_OK) {
        return ret;
    }
    
    /* Update state */
    memcpy(handle->x, x_new, sizeof(x_new));
    
    /* Normalize quaternion part */
    normalizeStateQuaternion(handle->x);
    
    /*
     * Update covariance: P = F * P * F' + Q
     * 
     * Step 1: temp7x7_1 = F * P
     * Step 2: temp7x7_2 = F' (transpose)
     * Step 3: P = temp7x7_1 * temp7x7_2 + Q
     */
    
    /* temp7x7_1 = F * P */
    ret = matrixMul(&handle->F, &handle->P, &handle->temp7x7_1);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp7x7_2 = F' */
    ret = matrixTranspose(&handle->F, &handle->temp7x7_2);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* P = temp7x7_1 * temp7x7_2 */
    ret = matrixMul(&handle->temp7x7_1, &handle->temp7x7_2, &handle->P);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* P = P + Q */
    ret = matrixAdd(&handle->P, &handle->Q, &handle->P);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    return EKF_OK;
}

int8_t ekfCorrect(EKF_Handle_t *handle, const float *accel)
{
    if (handle == NULL || accel == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    int8_t ret;
    float z_pred[EKF_MEAS_DIM];
    float y[EKF_MEAS_DIM];      /* Innovation */
    
    /*
     * CORRECT STEP (Measurement Update)
     * 
     * 1. Compute predicted measurement: z_pred = h(x)
     * 2. Compute measurement Jacobian: H = dh/dx
     * 3. Compute innovation: y = z - z_pred
     * 4. Compute innovation covariance: S = H * P * H' + R
     * 5. Compute Kalman gain: K = P * H' * S^(-1)
     * 6. Update state: x = x + K * y
     * 7. Update covariance: P = (I - K * H) * P
     */
    
    /* Call measurement function to get predicted measurement and Jacobian */
    ret = handle->measurementFunc(z_pred, handle->H.data, handle->x, handle);
    if (ret != EKF_OK) {
        return ret;
    }
    
    /* Compute innovation: y = z - z_pred */
    for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
        y[i] = accel[i] - z_pred[i];
    }
    
    /*
     * Compute innovation covariance: S = H * P * H' + R
     * 
     * Step 1: temp7x3 = H' (transpose)
     * Step 2: temp7x3_2 = P * H' (PH_T)
     * Step 3: temp3x3 = H * PH_T (S)
     * Step 4: temp3x3 = S + R
     */
    
    /* temp7x3 = H' */
    ret = matrixTranspose(&handle->H, &handle->temp7x3);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp7x3_2 = P * H' (this is PH_T) */
    ret = matrixMul(&handle->P, &handle->temp7x3, &handle->temp7x3_2);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp3x3 = H * PH_T = S */
    ret = matrixMul(&handle->H, &handle->temp7x3_2, &handle->temp3x3);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp3x3 = S + R */
    ret = matrixAdd(&handle->temp3x3, &handle->R, &handle->temp3x3);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Compute S^(-1) into temp3x3_2 */
    ret = matrixInverse(&handle->temp3x3, &handle->temp3x3_2);
    if (ret != MATRIX_OK) return EKF_ERR_SINGULAR;
    
    /*
     * Compute Kalman gain: K = P * H' * S^(-1)
     * K = PH_T * S_inv = temp7x3_2 * temp3x3_2
     */
    ret = matrixMul(&handle->temp7x3_2, &handle->temp3x3_2, &handle->K);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Update state: x = x + K * y */
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        float correction = 0.0f;
        for (uint8_t j = 0; j < EKF_MEAS_DIM; j++) {
            correction += handle->K.data[i * EKF_MEAS_DIM + j] * y[j];
        }
        handle->x[i] += correction;
    }
    
    /* Normalize quaternion part */
    normalizeStateQuaternion(handle->x);
    
    /*
     * Update covariance: P = (I - K * H) * P
     * 
     * Step 1: temp7x7_1 = K * H
     * Step 2: temp7x7_2 = I - temp7x7_1
     * Step 3: temp7x7_3 = temp7x7_2 * P (P_new)
     * Step 4: P = temp7x7_3
     */
    
    /* temp7x7_1 = K * H */
    ret = matrixMul(&handle->K, &handle->H, &handle->temp7x7_1);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp7x7_2 = I */
    ret = matrixEye(&handle->temp7x7_2, EKF_STATE_DIM);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp7x7_2 = I - K * H */
    ret = matrixSub(&handle->temp7x7_2, &handle->temp7x7_1, &handle->temp7x7_2);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* temp7x7_3 = (I - K * H) * P = P_new */
    ret = matrixMul(&handle->temp7x7_2, &handle->P, &handle->temp7x7_3);
    if (ret != MATRIX_OK) return EKF_ERR_MATRIX_OP;
    
    /* Copy P_new back to P */
    matrixCopy(&handle->temp7x7_3, &handle->P);
    
    return EKF_OK;
}

/*******************************************************************************
 * State Access Functions
 ******************************************************************************/

int8_t ekfGetQuaternion(const EKF_Handle_t *handle, Quaternion_t *q)
{
    if (handle == NULL || q == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    q->q0 = handle->x[0];
    q->q1 = handle->x[1];
    q->q2 = handle->x[2];
    q->q3 = handle->x[3];
    
    return EKF_OK;
}

int8_t ekfGetEulerZYX(EKF_Handle_t *handle, EulerAngles_t *euler)
{
    if (handle == NULL || euler == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    Quaternion_t q;
    q.q0 = handle->x[0];
    q.q1 = handle->x[1];
    q.q2 = handle->x[2];
    q.q3 = handle->x[3];
    
    int8_t ret = quatToEulerZYX(&q, euler, &handle->gimbal_status);
    
    if (ret == QUAT_ERR_GIMBAL_LOCK) {
        /* Gimbal lock detected but we still have valid (clamped) Euler angles */
        return EKF_OK;  /* Return OK but gimbal_status is set */
    }
    
    return (ret == QUAT_OK) ? EKF_OK : EKF_ERR_INVALID_PARAM;
}

int8_t ekfGetEulerDegrees(EKF_Handle_t *handle, float *roll, float *pitch, float *yaw)
{
    if (handle == NULL || roll == NULL || pitch == NULL || yaw == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    EulerAngles_t euler;
    int8_t ret = ekfGetEulerZYX(handle, &euler);
    
    if (ret != EKF_OK) {
        return ret;
    }
    
    *roll = radToDeg(euler.roll);
    *pitch = radToDeg(euler.pitch);
    *yaw = radToDeg(euler.yaw);
    
    return EKF_OK;
}

int8_t ekfGetGyroBias(const EKF_Handle_t *handle, float *bias)
{
    if (handle == NULL || bias == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    bias[0] = handle->x[4];
    bias[1] = handle->x[5];
    bias[2] = handle->x[6];
    
    return EKF_OK;
}

int8_t ekfGetState(const EKF_Handle_t *handle, EKF_State_t *state)
{
    if (handle == NULL || state == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    state->q[0] = handle->x[0];
    state->q[1] = handle->x[1];
    state->q[2] = handle->x[2];
    state->q[3] = handle->x[3];
    
    state->bias[0] = handle->x[4];
    state->bias[1] = handle->x[5];
    state->bias[2] = handle->x[6];
    
    return EKF_OK;
}

int8_t ekfGetDCM(const EKF_Handle_t *handle, DCM_t *dcm)
{
    if (handle == NULL || dcm == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    Quaternion_t q;
    q.q0 = handle->x[0];
    q.q1 = handle->x[1];
    q.q2 = handle->x[2];
    q.q3 = handle->x[3];
    
    return quatToDCM(&q, dcm);
}

int8_t ekfGetGimbalStatus(const EKF_Handle_t *handle, GimbalStatus_t *status)
{
    if (handle == NULL || status == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    status->is_gimbal_lock = handle->gimbal_status.is_gimbal_lock;
    status->pitch_clamped = handle->gimbal_status.pitch_clamped;
    
    return EKF_OK;
}

int8_t ekfGetCovarianceDiag(const EKF_Handle_t *handle, float *p_diag)
{
    if (handle == NULL || p_diag == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    if (!handle->is_initialized) {
        return EKF_ERR_NOT_INITIALIZED;
    }
    
    for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
        matrixGet(&handle->P, i, i, &p_diag[i]);
    }
    
    return EKF_OK;
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

int8_t ekfQuatDerivative(float *q_dot, const float *q, const float *omega)
{
    if (q_dot == NULL || q == NULL || omega == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    /*
     * Quaternion derivative from angular velocity (body frame)
     * 
     * q_dot = 0.5 * q âŠ— [0, Ï‰]
     * 
     * Expanded (Hamilton convention):
     * q_dot[0] = -0.5 * (q[1]*Ï‰x + q[2]*Ï‰y + q[3]*Ï‰z)
     * q_dot[1] =  0.5 * (q[0]*Ï‰x + q[2]*Ï‰z - q[3]*Ï‰y)
     * q_dot[2] =  0.5 * (q[0]*Ï‰y - q[1]*Ï‰z + q[3]*Ï‰x)
     * q_dot[3] =  0.5 * (q[0]*Ï‰z + q[1]*Ï‰y - q[2]*Ï‰x)
     */
    
    float wx = omega[0];
    float wy = omega[1];
    float wz = omega[2];
    
    q_dot[0] = 0.5f * (-q[1] * wx - q[2] * wy - q[3] * wz);
    q_dot[1] = 0.5f * (q[0] * wx + q[2] * wz - q[3] * wy);
    q_dot[2] = 0.5f * (q[0] * wy - q[1] * wz + q[3] * wx);
    q_dot[3] = 0.5f * (q[0] * wz + q[1] * wy - q[2] * wx);
    
    return EKF_OK;
}

int8_t ekfRK4QuatIntegrate(float *q_new, const float *q, const float *omega, float dt)
{
    if (q_new == NULL || q == NULL || omega == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    /*
     * 4th Order Runge-Kutta Integration for Quaternion Kinematics
     * 
     * k1 = f(t, q)
     * k2 = f(t + dt/2, q + dt/2 * k1)
     * k3 = f(t + dt/2, q + dt/2 * k2)
     * k4 = f(t + dt, q + dt * k3)
     * q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
     * 
     * where f(t, q) = q_dot = 0.5 * q âŠ— [0, Ï‰]
     */
    
    float k1[4], k2[4], k3[4], k4[4];
    float q_temp[4];
    float dt_half = dt * 0.5f;
    float dt_sixth = dt / 6.0f;
    
    /* k1 = f(q) */
    ekfQuatDerivative(k1, q, omega);
    
    /* k2 = f(q + dt/2 * k1) */
    for (uint8_t i = 0; i < 4; i++) {
        q_temp[i] = q[i] + dt_half * k1[i];
    }
    ekfQuatDerivative(k2, q_temp, omega);
    
    /* k3 = f(q + dt/2 * k2) */
    for (uint8_t i = 0; i < 4; i++) {
        q_temp[i] = q[i] + dt_half * k2[i];
    }
    ekfQuatDerivative(k3, q_temp, omega);
    
    /* k4 = f(q + dt * k3) */
    for (uint8_t i = 0; i < 4; i++) {
        q_temp[i] = q[i] + dt * k3[i];
    }
    ekfQuatDerivative(k4, q_temp, omega);
    
    /* q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4) */
    for (uint8_t i = 0; i < 4; i++) {
        q_new[i] = q[i] + dt_sixth * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }
    
    /* Normalize result */
    float norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1] + 
                       q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (norm > 1e-10f) {
        float inv_norm = 1.0f / norm;
        q_new[0] *= inv_norm;
        q_new[1] *= inv_norm;
        q_new[2] *= inv_norm;
        q_new[3] *= inv_norm;
    }
    
    return EKF_OK;
}

/*******************************************************************************
 * Default Callback Implementations
 ******************************************************************************/

int8_t ekfDefaultTransitionFunc(float *x_new, float *F,
                                 const float *x, const float *gyro,
                                 float dt, const EKF_Handle_t *handle)
{
    (void)handle;  /* Unused in default implementation */
    
    if (x_new == NULL || F == NULL || x == NULL || gyro == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    /*
     * State Transition Function
     * 
     * State: x = [q0, q1, q2, q3, bx, by, bz]
     * 
     * 1. Subtract bias from gyro measurement
     * 2. Integrate quaternion using RK4
     * 3. Bias remains constant (random walk model)
     * 4. Compute Jacobian F numerically
     */
    
    /* Bias-corrected angular velocity */
    float omega[3];
    omega[0] = gyro[0] - x[4];  /* Ï‰x - bx */
    omega[1] = gyro[1] - x[5];  /* Ï‰y - by */
    omega[2] = gyro[2] - x[6];  /* Ï‰z - bz */
    
    /* Integrate quaternion using RK4 */
    float q_new[4];
    ekfRK4QuatIntegrate(q_new, x, omega, dt);
    
    /* Copy new state */
    x_new[0] = q_new[0];
    x_new[1] = q_new[1];
    x_new[2] = q_new[2];
    x_new[3] = q_new[3];
    x_new[4] = x[4];  /* Bias unchanged (random walk) */
    x_new[5] = x[5];
    x_new[6] = x[6];
    
    /*
     * Compute Jacobian F numerically using finite differences
     * F[i][j] = (f(x + Î´*e_j) - f(x - Î´*e_j)) / (2*Î´)
     */
    
    float delta = EKF_JACOBIAN_DELTA;
    float x_plus[EKF_STATE_DIM];
    float x_minus[EKF_STATE_DIM];
    float f_plus[EKF_STATE_DIM];
    float f_minus[EKF_STATE_DIM];
    
    for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
        /* Perturb state in +/- direction */
        memcpy(x_plus, x, sizeof(float) * EKF_STATE_DIM);
        memcpy(x_minus, x, sizeof(float) * EKF_STATE_DIM);
        
        x_plus[j] += delta;
        x_minus[j] -= delta;
        
        /* Evaluate transition function at perturbed states */
        /* For quaternion perturbation, renormalize */
        if (j < 4) {
            float norm_plus = sqrtf(x_plus[0]*x_plus[0] + x_plus[1]*x_plus[1] + 
                                    x_plus[2]*x_plus[2] + x_plus[3]*x_plus[3]);
            float norm_minus = sqrtf(x_minus[0]*x_minus[0] + x_minus[1]*x_minus[1] + 
                                     x_minus[2]*x_minus[2] + x_minus[3]*x_minus[3]);
            if (norm_plus > 1e-10f) {
                x_plus[0] /= norm_plus;
                x_plus[1] /= norm_plus;
                x_plus[2] /= norm_plus;
                x_plus[3] /= norm_plus;
            }
            if (norm_minus > 1e-10f) {
                x_minus[0] /= norm_minus;
                x_minus[1] /= norm_minus;
                x_minus[2] /= norm_minus;
                x_minus[3] /= norm_minus;
            }
        }
        
        /* Compute f(x+) */
        float omega_plus[3], omega_minus[3];
        omega_plus[0] = gyro[0] - x_plus[4];
        omega_plus[1] = gyro[1] - x_plus[5];
        omega_plus[2] = gyro[2] - x_plus[6];
        
        omega_minus[0] = gyro[0] - x_minus[4];
        omega_minus[1] = gyro[1] - x_minus[5];
        omega_minus[2] = gyro[2] - x_minus[6];
        
        float q_plus[4], q_minus[4];
        ekfRK4QuatIntegrate(q_plus, x_plus, omega_plus, dt);
        ekfRK4QuatIntegrate(q_minus, x_minus, omega_minus, dt);
        
        f_plus[0] = q_plus[0];
        f_plus[1] = q_plus[1];
        f_plus[2] = q_plus[2];
        f_plus[3] = q_plus[3];
        f_plus[4] = x_plus[4];
        f_plus[5] = x_plus[5];
        f_plus[6] = x_plus[6];
        
        f_minus[0] = q_minus[0];
        f_minus[1] = q_minus[1];
        f_minus[2] = q_minus[2];
        f_minus[3] = q_minus[3];
        f_minus[4] = x_minus[4];
        f_minus[5] = x_minus[5];
        f_minus[6] = x_minus[6];
        
        /* Compute Jacobian column: F[:, j] = (f+ - f-) / (2*delta) */
        float inv_2delta = 1.0f / (2.0f * delta);
        for (uint8_t i = 0; i < EKF_STATE_DIM; i++) {
            F[i * EKF_STATE_DIM + j] = (f_plus[i] - f_minus[i]) * inv_2delta;
        }
    }
    
    return EKF_OK;
}

int8_t ekfDefaultMeasurementFunc(float *z_pred, float *H,
                                  const float *x,
                                  const EKF_Handle_t *handle)
{
    (void)handle;  /* Unused in default implementation */
    
    if (z_pred == NULL || H == NULL || x == NULL) {
        return EKF_ERR_NULL_PTR;
    }
    
    /*
     * Measurement Function
     * 
     * Expected accelerometer measurement when stationary:
     * - In NED frame, gravity is [0, 0, g] (pointing down)
     * - Accelerometer measures specific force = -gravity (in body frame)
     * - Expected measurement (in g units): rotate [0, 0, 1] from NED to body
     * 
     * Using quaternion inverse rotation:
     * a_body = q* âŠ— [0, 0, 1] âŠ— q
     * 
     * But actually, accelerometer measures reaction to gravity, so:
     * a_meas = q* âŠ— [0, 0, -1] âŠ— q (in NED convention)
     * 
     * Wait - let's be careful:
     * - NED: g_nav = [0, 0, +g] (gravity points down along +Z)
     * - Accelerometer measures: a = g_body / g = C_bn^T * g_nav / g
     * - Where C_bn transforms from body to nav
     * - So a_body = C_nb * [0, 0, 1] where C_nb = C_bn^T
     * 
     * For quaternion: C_nb corresponds to q* rotation
     * a_body = q* âŠ— [0, 0, 1] âŠ— q
     */
    
    float q0 = x[0];
    float q1 = x[1];
    float q2 = x[2];
    float q3 = x[3];
    
    /*
     * Rotate gravity unit vector [0, 0, 1] from NED to body frame
     * using quaternion conjugate (inverse for unit quaternion)
     * 
     * For g_ned = [0, 0, 1], the body-frame measurement is:
     * a_x = 2*(q1*q3 - q0*q2)
     * a_y = 2*(q2*q3 + q0*q1)
     * a_z = q0Â² - q1Â² - q2Â² + q3Â²
     * 
     * Note: This assumes NED frame where +Z is down (gravity positive)
     */
    
    z_pred[0] = 2.0f * (q1 * q3 - q0 * q2);
    z_pred[1] = 2.0f * (q2 * q3 + q0 * q1);
    z_pred[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    /*
     * Compute Measurement Jacobian H numerically
     * H is 3x7 matrix
     * H[i][j] = (h(x + Î´*e_j) - h(x - Î´*e_j)) / (2*Î´)
     */
    
    float delta = EKF_JACOBIAN_DELTA;
    float x_plus[EKF_STATE_DIM];
    float x_minus[EKF_STATE_DIM];
    float h_plus[EKF_MEAS_DIM];
    float h_minus[EKF_MEAS_DIM];
    
    for (uint8_t j = 0; j < EKF_STATE_DIM; j++) {
        /* Perturb state */
        memcpy(x_plus, x, sizeof(float) * EKF_STATE_DIM);
        memcpy(x_minus, x, sizeof(float) * EKF_STATE_DIM);
        
        x_plus[j] += delta;
        x_minus[j] -= delta;
        
        /* Renormalize quaternion if perturbed */
        if (j < 4) {
            float norm_plus = sqrtf(x_plus[0]*x_plus[0] + x_plus[1]*x_plus[1] + 
                                    x_plus[2]*x_plus[2] + x_plus[3]*x_plus[3]);
            float norm_minus = sqrtf(x_minus[0]*x_minus[0] + x_minus[1]*x_minus[1] + 
                                     x_minus[2]*x_minus[2] + x_minus[3]*x_minus[3]);
            if (norm_plus > 1e-10f) {
                x_plus[0] /= norm_plus;
                x_plus[1] /= norm_plus;
                x_plus[2] /= norm_plus;
                x_plus[3] /= norm_plus;
            }
            if (norm_minus > 1e-10f) {
                x_minus[0] /= norm_minus;
                x_minus[1] /= norm_minus;
                x_minus[2] /= norm_minus;
                x_minus[3] /= norm_minus;
            }
        }
        
        /* Compute h(x+) */
        float q0p = x_plus[0], q1p = x_plus[1], q2p = x_plus[2], q3p = x_plus[3];
        h_plus[0] = 2.0f * (q1p * q3p - q0p * q2p);
        h_plus[1] = 2.0f * (q2p * q3p + q0p * q1p);
        h_plus[2] = q0p * q0p - q1p * q1p - q2p * q2p + q3p * q3p;
        
        /* Compute h(x-) */
        float q0m = x_minus[0], q1m = x_minus[1], q2m = x_minus[2], q3m = x_minus[3];
        h_minus[0] = 2.0f * (q1m * q3m - q0m * q2m);
        h_minus[1] = 2.0f * (q2m * q3m + q0m * q1m);
        h_minus[2] = q0m * q0m - q1m * q1m - q2m * q2m + q3m * q3m;
        
        /* Compute Jacobian column */
        float inv_2delta = 1.0f / (2.0f * delta);
        for (uint8_t i = 0; i < EKF_MEAS_DIM; i++) {
            H[i * EKF_STATE_DIM + j] = (h_plus[i] - h_minus[i]) * inv_2delta;
        }
    }
    
    return EKF_OK;
}
