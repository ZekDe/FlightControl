/**
 * @file quaternion_math.c
 * @brief Implementation of quaternion mathematics library
 */

#include "quaternion_math.h"
#include <math.h>
#include <stddef.h>

/*******************************************************************************
 * Constants
 ******************************************************************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*******************************************************************************
 * Quaternion Basic Operations
 ******************************************************************************/

int8_t quatIdentity(Quaternion_t *q)
{
    if (q == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /* Identity quaternion: no rotation */
    q->q0 = 1.0f;
    q->q1 = 0.0f;
    q->q2 = 0.0f;
    q->q3 = 0.0f;
    
    return QUAT_OK;
}

int8_t quatSet(Quaternion_t *q, float q0, float q1, float q2, float q3)
{
    if (q == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    q->q0 = q0;
    q->q1 = q1;
    q->q2 = q2;
    q->q3 = q3;
    
    return QUAT_OK;
}

int8_t quatCopy(const Quaternion_t *src, Quaternion_t *dest)
{
    if (src == NULL || dest == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    dest->q0 = src->q0;
    dest->q1 = src->q1;
    dest->q2 = src->q2;
    dest->q3 = src->q3;
    
    return QUAT_OK;
}

int8_t quatNorm(const Quaternion_t *q, float *norm)
{
    if (q == NULL || norm == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /* norm = sqrt(q0² + q1² + q2² + q3²) */
    *norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    
    return QUAT_OK;
}

int8_t quatNormalize(Quaternion_t *q)
{
    if (q == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    float norm;
    quatNorm(q, &norm);
    
    if (norm < QUAT_EPSILON) {
        return QUAT_ERR_ZERO_NORM;
    }
    
    float inv_norm = 1.0f / norm;
    q->q0 *= inv_norm;
    q->q1 *= inv_norm;
    q->q2 *= inv_norm;
    q->q3 *= inv_norm;
    
    return QUAT_OK;
}

int8_t quatConjugate(const Quaternion_t *q, Quaternion_t *result)
{
    if (q == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /* conj(q) = [q0, -q1, -q2, -q3] */
    result->q0 = q->q0;
    result->q1 = -q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;
    
    return QUAT_OK;
}

int8_t quatInverse(const Quaternion_t *q, Quaternion_t *result)
{
    if (q == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /* inv(q) = conj(q) / ||q||² */
    float norm_sq = q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3;
    
    if (norm_sq < QUAT_EPSILON) {
        return QUAT_ERR_ZERO_NORM;
    }
    
    float inv_norm_sq = 1.0f / norm_sq;
    
    result->q0 = q->q0 * inv_norm_sq;
    result->q1 = -q->q1 * inv_norm_sq;
    result->q2 = -q->q2 * inv_norm_sq;
    result->q3 = -q->q3 * inv_norm_sq;
    
    return QUAT_OK;
}

/*******************************************************************************
 * Quaternion Arithmetic Operations
 ******************************************************************************/

int8_t quatMultiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result)
{
    if (q1 == NULL || q2 == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * Hamilton product: q1 ⊗ q2
     * 
     * result.q0 = q1.q0*q2.q0 - q1.q1*q2.q1 - q1.q2*q2.q2 - q1.q3*q2.q3
     * result.q1 = q1.q0*q2.q1 + q1.q1*q2.q0 + q1.q2*q2.q3 - q1.q3*q2.q2
     * result.q2 = q1.q0*q2.q2 - q1.q1*q2.q3 + q1.q2*q2.q0 + q1.q3*q2.q1
     * result.q3 = q1.q0*q2.q3 + q1.q1*q2.q2 - q1.q2*q2.q1 + q1.q3*q2.q0
     */
    
    float r0 = q1->q0 * q2->q0 - q1->q1 * q2->q1 - q1->q2 * q2->q2 - q1->q3 * q2->q3;
    float r1 = q1->q0 * q2->q1 + q1->q1 * q2->q0 + q1->q2 * q2->q3 - q1->q3 * q2->q2;
    float r2 = q1->q0 * q2->q2 - q1->q1 * q2->q3 + q1->q2 * q2->q0 + q1->q3 * q2->q1;
    float r3 = q1->q0 * q2->q3 + q1->q1 * q2->q2 - q1->q2 * q2->q1 + q1->q3 * q2->q0;
    
    result->q0 = r0;
    result->q1 = r1;
    result->q2 = r2;
    result->q3 = r3;
    
    return QUAT_OK;
}

int8_t quatAdd(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result)
{
    if (q1 == NULL || q2 == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    result->q0 = q1->q0 + q2->q0;
    result->q1 = q1->q1 + q2->q1;
    result->q2 = q1->q2 + q2->q2;
    result->q3 = q1->q3 + q2->q3;
    
    return QUAT_OK;
}

int8_t quatScale(const Quaternion_t *q, float scalar, Quaternion_t *result)
{
    if (q == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    result->q0 = q->q0 * scalar;
    result->q1 = q->q1 * scalar;
    result->q2 = q->q2 * scalar;
    result->q3 = q->q3 * scalar;
    
    return QUAT_OK;
}

/*******************************************************************************
 * Quaternion-Vector Operations
 ******************************************************************************/

int8_t quatRotateVector(const Quaternion_t *q, const Vector3_t *v, Vector3_t *result)
{
    if (q == NULL || v == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * Rotate vector: v' = q ⊗ v ⊗ q*
     * Optimized formula (avoiding two quaternion multiplications):
     * 
     * t = 2 * (q.xyz × v)
     * v' = v + q.w * t + (q.xyz × t)
     */
    
    /* t = 2 * (q.xyz × v) */
    float tx = 2.0f * (q->q2 * v->z - q->q3 * v->y);
    float ty = 2.0f * (q->q3 * v->x - q->q1 * v->z);
    float tz = 2.0f * (q->q1 * v->y - q->q2 * v->x);
    
    /* v' = v + q0 * t + (q.xyz × t) */
    result->x = v->x + q->q0 * tx + (q->q2 * tz - q->q3 * ty);
    result->y = v->y + q->q0 * ty + (q->q3 * tx - q->q1 * tz);
    result->z = v->z + q->q0 * tz + (q->q1 * ty - q->q2 * tx);
    
    return QUAT_OK;
}

int8_t quatRotateVectorInverse(const Quaternion_t *q, const Vector3_t *v, Vector3_t *result)
{
    if (q == NULL || v == NULL || result == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * Inverse rotation: v' = q* ⊗ v ⊗ q
     * Using conjugate of quaternion
     */
    
    /* Use conjugate: negate vector part */
    float tx = 2.0f * (-q->q2 * v->z + q->q3 * v->y);
    float ty = 2.0f * (-q->q3 * v->x + q->q1 * v->z);
    float tz = 2.0f * (-q->q1 * v->y + q->q2 * v->x);
    
    result->x = v->x + q->q0 * tx + (-q->q2 * tz + q->q3 * ty);
    result->y = v->y + q->q0 * ty + (-q->q3 * tx + q->q1 * tz);
    result->z = v->z + q->q0 * tz + (-q->q1 * ty + q->q2 * tx);
    
    return QUAT_OK;
}

/*******************************************************************************
 * Quaternion-Euler Conversions (ZYX-321 Convention)
 ******************************************************************************/

int8_t quatToEulerZYX(const Quaternion_t *q, EulerAngles_t *euler, GimbalStatus_t *status)
{
    if (q == NULL || euler == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    int8_t ret = QUAT_OK;
    
    /* Initialize status if provided */
    if (status != NULL) {
        status->is_gimbal_lock = 0;
        status->pitch_clamped = 0;
    }
    
    /*
     * ZYX-321 Euler angles from quaternion (NED frame)
     * 
     * roll  = atan2(2(q0*q1 + q2*q3), 1 - 2(q1² + q2²))
     * pitch = asin(2(q0*q2 - q3*q1))
     * yaw   = atan2(2(q0*q3 + q1*q2), 1 - 2(q2² + q3²))
     */
    
    float q0 = q->q0;
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;
    
    /* Roll (phi) - rotation about X axis */
    float sin_roll = 2.0f * (q0 * q1 + q2 * q3);
    float cos_roll = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    euler->roll = atan2f(sin_roll, cos_roll);
    
    /* Pitch (theta) - rotation about Y axis */
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    
    /* Clamp to avoid numerical issues with asin */
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    
    euler->pitch = asinf(sin_pitch);
    
    /* Check for gimbal lock (pitch near ±90°) */
    float gimbal_limit_rad = degToRad(QUAT_GIMBAL_LIMIT);
    
    if (fabsf(euler->pitch) > gimbal_limit_rad) {
        if (status != NULL) {
            status->is_gimbal_lock = 1;
            status->pitch_clamped = 1;
        }
        
        /* Clamp pitch to limit */
        if (euler->pitch > 0) {
            euler->pitch = gimbal_limit_rad;
        } else {
            euler->pitch = -gimbal_limit_rad;
        }
        
        ret = QUAT_ERR_GIMBAL_LOCK;
    }
    
    /* Yaw (psi) - rotation about Z axis */
    float sin_yaw = 2.0f * (q0 * q3 + q1 * q2);
    float cos_yaw = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    euler->yaw = atan2f(sin_yaw, cos_yaw);
    
    return ret;
}

int8_t eulerToQuatZYX(const EulerAngles_t *euler, Quaternion_t *q)
{
    if (euler == NULL || q == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * ZYX-321: q = q_yaw ⊗ q_pitch ⊗ q_roll
     * 
     * Half angles for efficiency
     */
    
    float half_roll = euler->roll * 0.5f;
    float half_pitch = euler->pitch * 0.5f;
    float half_yaw = euler->yaw * 0.5f;
    
    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);
    
    /*
     * Combined formula for ZYX rotation sequence:
     * q = q_yaw ⊗ q_pitch ⊗ q_roll
     */
    q->q0 = cr * cp * cy + sr * sp * sy;
    q->q1 = sr * cp * cy - cr * sp * sy;
    q->q2 = cr * sp * cy + sr * cp * sy;
    q->q3 = cr * cp * sy - sr * sp * cy;
    
    /* Ensure unit quaternion */
    quatNormalize(q);
    
    return QUAT_OK;
}

/*******************************************************************************
 * Quaternion-DCM Conversions
 ******************************************************************************/

int8_t quatToDCM(const Quaternion_t *q, DCM_t *dcm)
{
    if (q == NULL || dcm == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * DCM from quaternion (body to navigation frame transformation)
     * 
     * C = | 1-2(q2²+q3²)    2(q1q2-q0q3)    2(q1q3+q0q2) |
     *     | 2(q1q2+q0q3)    1-2(q1²+q3²)    2(q2q3-q0q1) |
     *     | 2(q1q3-q0q2)    2(q2q3+q0q1)    1-2(q1²+q2²) |
     */
    
    float q0 = q->q0;
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;
    
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    
    /* Row 0 */
    dcm->m[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    dcm->m[0][1] = 2.0f * (q1q2 - q0q3);
    dcm->m[0][2] = 2.0f * (q1q3 + q0q2);
    
    /* Row 1 */
    dcm->m[1][0] = 2.0f * (q1q2 + q0q3);
    dcm->m[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    dcm->m[1][2] = 2.0f * (q2q3 - q0q1);
    
    /* Row 2 */
    dcm->m[2][0] = 2.0f * (q1q3 - q0q2);
    dcm->m[2][1] = 2.0f * (q2q3 + q0q1);
    dcm->m[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
    
    return QUAT_OK;
}

int8_t dcmToQuat(const DCM_t *dcm, Quaternion_t *q)
{
    if (dcm == NULL || q == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * Shepperd's method for DCM to quaternion conversion
     * Numerically stable for all rotation angles
     * 
     * Choose the largest diagonal element to avoid division by small numbers
     */
    
    float trace = dcm->m[0][0] + dcm->m[1][1] + dcm->m[2][2];
    
    if (trace > 0) {
        /* Trace is positive: use standard formula */
        float s = sqrtf(trace + 1.0f) * 2.0f;  /* s = 4 * q0 */
        q->q0 = 0.25f * s;
        q->q1 = (dcm->m[2][1] - dcm->m[1][2]) / s;
        q->q2 = (dcm->m[0][2] - dcm->m[2][0]) / s;
        q->q3 = (dcm->m[1][0] - dcm->m[0][1]) / s;
    } else if ((dcm->m[0][0] > dcm->m[1][1]) && (dcm->m[0][0] > dcm->m[2][2])) {
        /* m[0][0] is largest diagonal */
        float s = sqrtf(1.0f + dcm->m[0][0] - dcm->m[1][1] - dcm->m[2][2]) * 2.0f;
        q->q0 = (dcm->m[2][1] - dcm->m[1][2]) / s;
        q->q1 = 0.25f * s;
        q->q2 = (dcm->m[0][1] + dcm->m[1][0]) / s;
        q->q3 = (dcm->m[0][2] + dcm->m[2][0]) / s;
    } else if (dcm->m[1][1] > dcm->m[2][2]) {
        /* m[1][1] is largest diagonal */
        float s = sqrtf(1.0f + dcm->m[1][1] - dcm->m[0][0] - dcm->m[2][2]) * 2.0f;
        q->q0 = (dcm->m[0][2] - dcm->m[2][0]) / s;
        q->q1 = (dcm->m[0][1] + dcm->m[1][0]) / s;
        q->q2 = 0.25f * s;
        q->q3 = (dcm->m[1][2] + dcm->m[2][1]) / s;
    } else {
        /* m[2][2] is largest diagonal */
        float s = sqrtf(1.0f + dcm->m[2][2] - dcm->m[0][0] - dcm->m[1][1]) * 2.0f;
        q->q0 = (dcm->m[1][0] - dcm->m[0][1]) / s;
        q->q1 = (dcm->m[0][2] + dcm->m[2][0]) / s;
        q->q2 = (dcm->m[1][2] + dcm->m[2][1]) / s;
        q->q3 = 0.25f * s;
    }
    
    /* Ensure unit quaternion and positive scalar part */
    quatNormalize(q);
    
    /* Convention: keep q0 positive (equivalent rotation) */
    if (q->q0 < 0) {
        q->q0 = -q->q0;
        q->q1 = -q->q1;
        q->q2 = -q->q2;
        q->q3 = -q->q3;
    }
    
    return QUAT_OK;
}

int8_t dcmToEulerZYX(const DCM_t *dcm, EulerAngles_t *euler, GimbalStatus_t *status)
{
    if (dcm == NULL || euler == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    int8_t ret = QUAT_OK;
    
    if (status != NULL) {
        status->is_gimbal_lock = 0;
        status->pitch_clamped = 0;
    }
    
    /*
     * ZYX Euler angles from DCM
     * 
     * roll  = atan2(C[2][1], C[2][2])
     * pitch = -asin(C[2][0])
     * yaw   = atan2(C[1][0], C[0][0])
     */
    
    /* Pitch (theta) */
    float sin_pitch = -dcm->m[2][0];
    
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    
    euler->pitch = asinf(sin_pitch);
    
    /* Check gimbal lock */
    float gimbal_limit_rad = degToRad(QUAT_GIMBAL_LIMIT);
    
    if (fabsf(euler->pitch) > gimbal_limit_rad) {
        if (status != NULL) {
            status->is_gimbal_lock = 1;
            status->pitch_clamped = 1;
        }
        
        if (euler->pitch > 0) {
            euler->pitch = gimbal_limit_rad;
        } else {
            euler->pitch = -gimbal_limit_rad;
        }
        
        ret = QUAT_ERR_GIMBAL_LOCK;
    }
    
    /* Roll (phi) */
    euler->roll = atan2f(dcm->m[2][1], dcm->m[2][2]);
    
    /* Yaw (psi) */
    euler->yaw = atan2f(dcm->m[1][0], dcm->m[0][0]);
    
    return ret;
}

int8_t eulerToDCM_ZYX(const EulerAngles_t *euler, DCM_t *dcm)
{
    if (euler == NULL || dcm == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * DCM from ZYX Euler angles
     * C = Cz(yaw) * Cy(pitch) * Cx(roll)
     */
    
    float cr = cosf(euler->roll);
    float sr = sinf(euler->roll);
    float cp = cosf(euler->pitch);
    float sp = sinf(euler->pitch);
    float cy = cosf(euler->yaw);
    float sy = sinf(euler->yaw);
    
    /* Row 0 */
    dcm->m[0][0] = cy * cp;
    dcm->m[0][1] = cy * sp * sr - sy * cr;
    dcm->m[0][2] = cy * sp * cr + sy * sr;
    
    /* Row 1 */
    dcm->m[1][0] = sy * cp;
    dcm->m[1][1] = sy * sp * sr + cy * cr;
    dcm->m[1][2] = sy * sp * cr - cy * sr;
    
    /* Row 2 */
    dcm->m[2][0] = -sp;
    dcm->m[2][1] = cp * sr;
    dcm->m[2][2] = cp * cr;
    
    return QUAT_OK;
}

/*******************************************************************************
 * Quaternion Kinematics
 ******************************************************************************/

int8_t quatDerivative(const Quaternion_t *q, const Vector3_t *omega, Quaternion_t *q_dot)
{
    if (q == NULL || omega == NULL || q_dot == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * Quaternion derivative from angular velocity
     * 
     * q_dot = 0.5 * q ⊗ ω_quat
     * where ω_quat = [0, ωx, ωy, ωz]
     * 
     * Expanded:
     * q_dot.q0 = -0.5 * (q1*ωx + q2*ωy + q3*ωz)
     * q_dot.q1 =  0.5 * (q0*ωx + q2*ωz - q3*ωy)
     * q_dot.q2 =  0.5 * (q0*ωy - q1*ωz + q3*ωx)
     * q_dot.q3 =  0.5 * (q0*ωz + q1*ωy - q2*ωx)
     */
    
    float wx = omega->x;
    float wy = omega->y;
    float wz = omega->z;
    
    q_dot->q0 = 0.5f * (-q->q1 * wx - q->q2 * wy - q->q3 * wz);
    q_dot->q1 = 0.5f * (q->q0 * wx + q->q2 * wz - q->q3 * wy);
    q_dot->q2 = 0.5f * (q->q0 * wy - q->q1 * wz + q->q3 * wx);
    q_dot->q3 = 0.5f * (q->q0 * wz + q->q1 * wy - q->q2 * wx);
    
    return QUAT_OK;
}

int8_t quatBuildOmegaMatrix(const Vector3_t *omega, float *Omega)
{
    if (omega == NULL || Omega == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    /*
     * Omega matrix for quaternion kinematics: q_dot = 0.5 * Omega * q
     * 
     * Omega = |  0   -ωx  -ωy  -ωz |
     *         | ωx    0    ωz  -ωy |
     *         | ωy  -ωz    0    ωx |
     *         | ωz   ωy  -ωx    0  |
     * 
     * Stored row-major: Omega[row*4 + col]
     */
    
    float wx = omega->x;
    float wy = omega->y;
    float wz = omega->z;
    
    /* Row 0 */
    Omega[0] = 0.0f;   Omega[1] = -wx;    Omega[2] = -wy;    Omega[3] = -wz;
    /* Row 1 */
    Omega[4] = wx;     Omega[5] = 0.0f;   Omega[6] = wz;     Omega[7] = -wy;
    /* Row 2 */
    Omega[8] = wy;     Omega[9] = -wz;    Omega[10] = 0.0f;  Omega[11] = wx;
    /* Row 3 */
    Omega[12] = wz;    Omega[13] = wy;    Omega[14] = -wx;   Omega[15] = 0.0f;
    
    return QUAT_OK;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

float degToRad(float degrees)
{
    return degrees * (M_PI / 180.0f);
}

float radToDeg(float radians)
{
    return radians * (180.0f / M_PI);
}

float normalizeAngle(float angle)
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

int8_t vector3Set(Vector3_t *v, float x, float y, float z)
{
    if (v == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    v->x = x;
    v->y = y;
    v->z = z;
    
    return QUAT_OK;
}

int8_t vector3Norm(const Vector3_t *v, float *norm)
{
    if (v == NULL || norm == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    *norm = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
    
    return QUAT_OK;
}

int8_t vector3Normalize(Vector3_t *v)
{
    if (v == NULL) {
        return QUAT_ERR_NULL_PTR;
    }
    
    float norm;
    vector3Norm(v, &norm);
    
    if (norm < QUAT_EPSILON) {
        return QUAT_ERR_ZERO_NORM;
    }
    
    float inv_norm = 1.0f / norm;
    v->x *= inv_norm;
    v->y *= inv_norm;
    v->z *= inv_norm;
    
    return QUAT_OK;
}
