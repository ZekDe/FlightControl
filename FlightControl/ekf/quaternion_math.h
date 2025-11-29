/**
 * @file quaternion_math.h
 * @brief Quaternion mathematics library for attitude estimation
 * @details Provides quaternion operations, conversions to/from Euler angles,
 *          DCM (Direction Cosine Matrix), and related transformations.
 *          Uses Hamilton convention: q = q0 + q1*i + q2*j + q3*k
 *          where q0 is the scalar part.
 * 
 * @note Navigation frame: NED (North-East-Down)
 * @note Euler convention: ZYX-321 (yaw -> pitch -> roll)
 * @note Uses float precision (single precision FPU)
 */

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define QUAT_EPSILON        1e-6f       /**< Threshold for floating point comparisons */
#define QUAT_GIMBAL_LIMIT   89.9f       /**< Pitch limit in degrees for gimbal lock detection */

/*******************************************************************************
 * Error Codes
 ******************************************************************************/

#define QUAT_OK                     0   /**< Operation successful */
#define QUAT_ERR_NULL_PTR          -1   /**< Null pointer passed */
#define QUAT_ERR_ZERO_NORM         -2   /**< Quaternion has zero norm */
#define QUAT_ERR_GIMBAL_LOCK       -3   /**< Gimbal lock detected (pitch near ±90°) */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Quaternion structure (Hamilton convention)
 * @note q = q0 + q1*i + q2*j + q3*k, where q0 is scalar part
 */
typedef struct {
    float q0;   /**< Scalar part (w) */
    float q1;   /**< i component (x) */
    float q2;   /**< j component (y) */
    float q3;   /**< k component (z) */
} Quaternion_t;

/**
 * @brief Euler angles structure (ZYX-321 convention)
 * @note All angles in radians
 * @note NED frame: roll about X (North), pitch about Y (East), yaw about Z (Down)
 */
typedef struct {
    float roll;     /**< Roll angle (rotation about X/North axis) [rad] */
    float pitch;    /**< Pitch angle (rotation about Y/East axis) [rad] */
    float yaw;      /**< Yaw angle (rotation about Z/Down axis) [rad] */
} EulerAngles_t;

/**
 * @brief 3D vector structure
 */
typedef struct {
    float x;    /**< X component */
    float y;    /**< Y component */
    float z;    /**< Z component */
} Vector3_t;

/**
 * @brief Direction Cosine Matrix (3x3 rotation matrix)
 * @note Row-major storage: m[row][col]
 */
typedef struct {
    float m[3][3];  /**< 3x3 rotation matrix */
} DCM_t;

/**
 * @brief Gimbal lock status flags
 */
typedef struct {
    uint8_t is_gimbal_lock;     /**< 1 if gimbal lock detected, 0 otherwise */
    uint8_t pitch_clamped;      /**< 1 if pitch was clamped to limit */
} GimbalStatus_t;

/*******************************************************************************
 * Quaternion Basic Operations
 ******************************************************************************/

/**
 * @brief Initialize quaternion to identity (no rotation)
 * @param[out] q    Pointer to quaternion
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details Identity quaternion: q = [1, 0, 0, 0]
 *          Represents zero rotation.
 */
int8_t quatIdentity(Quaternion_t *q);

/**
 * @brief Set quaternion components
 * @param[out] q    Pointer to quaternion
 * @param[in]  q0   Scalar component
 * @param[in]  q1   i component
 * @param[in]  q2   j component
 * @param[in]  q3   k component
 * @return QUAT_OK on success, error code otherwise
 */
int8_t quatSet(Quaternion_t *q, float q0, float q1, float q2, float q3);

/**
 * @brief Copy quaternion
 * @param[in]  src   Source quaternion
 * @param[out] dest  Destination quaternion
 * @return QUAT_OK on success, error code otherwise
 */
int8_t quatCopy(const Quaternion_t *src, Quaternion_t *dest);

/**
 * @brief Compute quaternion norm (magnitude)
 * @param[in]  q      Pointer to quaternion
 * @param[out] norm   Pointer to store norm value
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details norm = sqrt(q0² + q1² + q2² + q3²)
 */
int8_t quatNorm(const Quaternion_t *q, float *norm);

/**
 * @brief Normalize quaternion to unit length
 * @param[in,out] q   Pointer to quaternion (modified in place)
 * @return QUAT_OK on success, QUAT_ERR_ZERO_NORM if norm is zero
 * 
 * @details q_normalized = q / ||q||
 *          Essential for maintaining valid rotation representation.
 */
int8_t quatNormalize(Quaternion_t *q);

/**
 * @brief Compute quaternion conjugate
 * @param[in]  q       Input quaternion
 * @param[out] result  Conjugate quaternion
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details conj(q) = [q0, -q1, -q2, -q3]
 *          For unit quaternions: conj(q) = inverse(q)
 */
int8_t quatConjugate(const Quaternion_t *q, Quaternion_t *result);

/**
 * @brief Compute quaternion inverse
 * @param[in]  q       Input quaternion
 * @param[out] result  Inverse quaternion
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details inv(q) = conj(q) / ||q||²
 *          For unit quaternions: inv(q) = conj(q)
 */
int8_t quatInverse(const Quaternion_t *q, Quaternion_t *result);

/*******************************************************************************
 * Quaternion Arithmetic Operations
 ******************************************************************************/

/**
 * @brief Quaternion multiplication (Hamilton product)
 * @param[in]  q1     First quaternion
 * @param[in]  q2     Second quaternion
 * @param[out] result Product quaternion
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details Hamilton product: result = q1 ⊗ q2
 *          Non-commutative: q1 ⊗ q2 ≠ q2 ⊗ q1
 *          Combines rotations: first q2, then q1
 */
int8_t quatMultiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result);

/**
 * @brief Quaternion addition
 * @param[in]  q1     First quaternion
 * @param[in]  q2     Second quaternion
 * @param[out] result Sum quaternion
 * @return QUAT_OK on success, error code otherwise
 */
int8_t quatAdd(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result);

/**
 * @brief Quaternion scalar multiplication
 * @param[in]  q       Input quaternion
 * @param[in]  scalar  Scalar multiplier
 * @param[out] result  Scaled quaternion
 * @return QUAT_OK on success, error code otherwise
 */
int8_t quatScale(const Quaternion_t *q, float scalar, Quaternion_t *result);

/*******************************************************************************
 * Quaternion-Vector Operations
 ******************************************************************************/

/**
 * @brief Rotate vector by quaternion
 * @param[in]  q       Unit quaternion representing rotation
 * @param[in]  v       Input vector
 * @param[out] result  Rotated vector
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details v' = q ⊗ v ⊗ q*
 *          Rotates vector v from body frame to navigation frame
 */
int8_t quatRotateVector(const Quaternion_t *q, const Vector3_t *v, Vector3_t *result);

/**
 * @brief Rotate vector by quaternion inverse (reverse rotation)
 * @param[in]  q       Unit quaternion representing rotation
 * @param[in]  v       Input vector
 * @param[out] result  Rotated vector
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details v' = q* ⊗ v ⊗ q
 *          Rotates vector v from navigation frame to body frame
 */
int8_t quatRotateVectorInverse(const Quaternion_t *q, const Vector3_t *v, Vector3_t *result);

/*******************************************************************************
 * Quaternion-Euler Conversions (ZYX-321 Convention)
 ******************************************************************************/

/**
 * @brief Convert quaternion to Euler angles (ZYX-321)
 * @param[in]  q        Input quaternion
 * @param[out] euler    Output Euler angles [rad]
 * @param[out] status   Optional gimbal lock status (can be NULL)
 * @return QUAT_OK on success, QUAT_ERR_GIMBAL_LOCK if gimbal lock (pitch clamped)
 * 
 * @details ZYX-321 convention (aerospace): yaw(Z) -> pitch(Y) -> roll(X)
 *          NED frame: X=North, Y=East, Z=Down
 *          
 *          roll  = atan2(2(q0*q1 + q2*q3), 1 - 2(q1² + q2²))
 *          pitch = asin(2(q0*q2 - q3*q1))
 *          yaw   = atan2(2(q0*q3 + q1*q2), 1 - 2(q2² + q3²))
 *          
 * @warning Gimbal lock occurs when pitch ≈ ±90°. If detected, pitch is
 *          clamped to ±QUAT_GIMBAL_LIMIT and status flag is set.
 */
int8_t quatToEulerZYX(const Quaternion_t *q, EulerAngles_t *euler, GimbalStatus_t *status);

/**
 * @brief Convert Euler angles to quaternion (ZYX-321)
 * @param[in]  euler  Input Euler angles [rad]
 * @param[out] q      Output quaternion (normalized)
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details ZYX-321: First rotate yaw about Z, then pitch about Y, then roll about X
 *          q = q_yaw ⊗ q_pitch ⊗ q_roll
 */
int8_t eulerToQuatZYX(const EulerAngles_t *euler, Quaternion_t *q);

/*******************************************************************************
 * Quaternion-DCM Conversions
 ******************************************************************************/

/**
 * @brief Convert quaternion to Direction Cosine Matrix (DCM)
 * @param[in]  q    Input quaternion
 * @param[out] dcm  Output DCM (3x3 rotation matrix)
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details DCM transforms vectors from body frame to navigation frame.
 *          C = | 1-2(q2²+q3²)    2(q1q2-q0q3)    2(q1q3+q0q2) |
 *              | 2(q1q2+q0q3)    1-2(q1²+q3²)    2(q2q3-q0q1) |
 *              | 2(q1q3-q0q2)    2(q2q3+q0q1)    1-2(q1²+q2²) |
 */
int8_t quatToDCM(const Quaternion_t *q, DCM_t *dcm);

/**
 * @brief Convert DCM to quaternion (Shepperd's method)
 * @param[in]  dcm  Input DCM (3x3 rotation matrix)
 * @param[out] q    Output quaternion (normalized)
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details Uses Shepperd's method for numerical stability.
 *          Handles all rotation angles without singularity.
 */
int8_t dcmToQuat(const DCM_t *dcm, Quaternion_t *q);

/**
 * @brief Convert DCM to Euler angles (ZYX-321)
 * @param[in]  dcm     Input DCM
 * @param[out] euler   Output Euler angles [rad]
 * @param[out] status  Optional gimbal lock status (can be NULL)
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details Extracts ZYX Euler angles directly from DCM.
 *          roll  = atan2(C[2][1], C[2][2])
 *          pitch = -asin(C[2][0])
 *          yaw   = atan2(C[1][0], C[0][0])
 */
int8_t dcmToEulerZYX(const DCM_t *dcm, EulerAngles_t *euler, GimbalStatus_t *status);

/**
 * @brief Convert Euler angles to DCM (ZYX-321)
 * @param[in]  euler  Input Euler angles [rad]
 * @param[out] dcm    Output DCM
 * @return QUAT_OK on success, error code otherwise
 */
int8_t eulerToDCM_ZYX(const EulerAngles_t *euler, DCM_t *dcm);

/*******************************************************************************
 * Quaternion Kinematics
 ******************************************************************************/

/**
 * @brief Compute quaternion derivative from angular velocity
 * @param[in]  q       Current quaternion
 * @param[in]  omega   Angular velocity in body frame [rad/s]
 * @param[out] q_dot   Quaternion time derivative
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details q_dot = 0.5 * q ⊗ ω_quat
 *          where ω_quat = [0, ωx, ωy, ωz]
 *          
 *          Used in quaternion integration:
 *          q(t+dt) ≈ q(t) + q_dot * dt  (Euler method)
 */
int8_t quatDerivative(const Quaternion_t *q, const Vector3_t *omega, Quaternion_t *q_dot);

/**
 * @brief Build omega matrix for quaternion kinematics
 * @param[in]  omega  Angular velocity [rad/s]
 * @param[out] Omega  4x4 omega matrix (stored as 16-element array, row-major)
 * @return QUAT_OK on success, error code otherwise
 * 
 * @details Omega matrix for q_dot = 0.5 * Omega * q
 *          Omega = |  0   -ωx  -ωy  -ωz |
 *                  | ωx    0    ωz  -ωy |
 *                  | ωy  -ωz    0    ωx |
 *                  | ωz   ωy  -ωx    0  |
 */
int8_t quatBuildOmegaMatrix(const Vector3_t *omega, float *Omega);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Convert degrees to radians
 * @param[in] degrees  Angle in degrees
 * @return Angle in radians
 */
float degToRad(float degrees);

/**
 * @brief Convert radians to degrees
 * @param[in] radians  Angle in radians
 * @return Angle in degrees
 */
float radToDeg(float radians);

/**
 * @brief Normalize angle to [-π, π] range
 * @param[in] angle  Input angle [rad]
 * @return Normalized angle [rad]
 */
float normalizeAngle(float angle);

/**
 * @brief Initialize Vector3
 * @param[out] v  Pointer to vector
 * @param[in]  x  X component
 * @param[in]  y  Y component
 * @param[in]  z  Z component
 * @return QUAT_OK on success, error code otherwise
 */
int8_t vector3Set(Vector3_t *v, float x, float y, float z);

/**
 * @brief Compute Vector3 norm
 * @param[in]  v     Pointer to vector
 * @param[out] norm  Pointer to store norm
 * @return QUAT_OK on success, error code otherwise
 */
int8_t vector3Norm(const Vector3_t *v, float *norm);

/**
 * @brief Normalize Vector3
 * @param[in,out] v  Pointer to vector (modified in place)
 * @return QUAT_OK on success, error code otherwise
 */
int8_t vector3Normalize(Vector3_t *v);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_MATH_H */
