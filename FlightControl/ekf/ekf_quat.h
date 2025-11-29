/**
 * @file ekf_quat.h
 * @brief Extended Kalman Filter for quaternion-based attitude estimation
 * @details 7-state EKF: [q0, q1, q2, q3, bx, by, bz]
 *          - Quaternion (4 states): Attitude representation
 *          - Gyro bias (3 states): Compensates gyroscope drift
 * 
 * @note Navigation frame: NED (North-East-Down)
 * @note Euler convention: ZYX-321
 * @note Uses RK4 for state propagation
 * @note Uses numerical Jacobian (finite difference)
 * @note No magnetometer - initial heading is reference (0)
 */

#ifndef EKF_QUAT_H
#define EKF_QUAT_H

#include <stdint.h>
#include "matrix_math.h"
#include "quaternion_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define EKF_STATE_DIM       7       /**< State dimension: [q0,q1,q2,q3,bx,by,bz] */
#define EKF_MEAS_DIM        3       /**< Measurement dimension: [ax, ay, az] */
#define EKF_JACOBIAN_DELTA  1e-4f   /**< Perturbation for numerical Jacobian */
#define EKF_GRAVITY_NED     9.81f   /**< Gravity magnitude [m/s²] in NED (down positive) */

/*******************************************************************************
 * Error Codes
 ******************************************************************************/

#define EKF_OK                      0   /**< Operation successful */
#define EKF_ERR_NULL_PTR           -1   /**< Null pointer passed */
#define EKF_ERR_NOT_INITIALIZED    -2   /**< EKF not initialized */
#define EKF_ERR_MATRIX_OP          -3   /**< Matrix operation failed */
#define EKF_ERR_SINGULAR           -4   /**< Matrix inversion failed */
#define EKF_ERR_INVALID_PARAM      -5   /**< Invalid parameter */

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/* Forward declaration */
typedef struct EKF_Handle_s EKF_Handle_t;

/**
 * @brief Transition function callback type
 * @details Called during predict step to propagate state using gyro data.
 *          Must implement RK4 integration of quaternion kinematics.
 * 
 * @param[out] x_new    New state vector (7 elements)
 * @param[out] F        State transition Jacobian (7x7 matrix, row-major)
 * @param[in]  x        Current state vector (7 elements)
 * @param[in]  gyro     Gyroscope data [rad/s] (3 elements: gx, gy, gz)
 * @param[in]  dt       Time step [s]
 * @param[in]  handle   EKF handle for accessing internal data if needed
 * @return 0 on success, negative error code otherwise
 * 
 * @note x_new and F are computed by the callback
 * @note Gyro bias from state should be subtracted from gyro measurement
 * @note Quaternion in x_new should be normalized
 */
typedef int8_t (*EKF_TransitionFunc_t)(float *x_new, float *F, 
                                        const float *x, const float *gyro, 
                                        float dt, const EKF_Handle_t *handle);

/**
 * @brief Measurement function callback type
 * @details Called during correct step to compute expected measurement
 *          and measurement Jacobian.
 * 
 * @param[out] z_pred   Predicted measurement (3 elements: expected accel)
 * @param[out] H        Measurement Jacobian (3x7 matrix, row-major)
 * @param[in]  x        Current state vector (7 elements)
 * @param[in]  handle   EKF handle for accessing internal data if needed
 * @return 0 on success, negative error code otherwise
 * 
 * @note Expected accelerometer reading in body frame when stationary
 *       is the gravity vector rotated by quaternion inverse
 * @note Accel measurement is in g units, so expected is [0, 0, 1]^T in NED
 *       rotated to body frame
 */
typedef int8_t (*EKF_MeasurementFunc_t)(float *z_pred, float *H,
                                         const float *x,
                                         const EKF_Handle_t *handle);

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Process noise configuration
 * @details Q matrix diagonal elements
 */
typedef struct {
    float q_quat;       /**< Quaternion process noise variance */
    float q_bias;       /**< Gyro bias process noise variance */
} EKF_ProcessNoise_t;

/**
 * @brief Measurement noise configuration
 * @details R matrix diagonal elements
 */
typedef struct {
    float r_accel;      /**< Accelerometer measurement noise variance */
} EKF_MeasurementNoise_t;

/**
 * @brief EKF state structure
 */
typedef struct {
    float q[4];         /**< Quaternion: [q0, q1, q2, q3] */
    float bias[3];      /**< Gyro bias: [bx, by, bz] [rad/s] */
} EKF_State_t;

/**
 * @brief EKF handle structure
 */
struct EKF_Handle_s {
    /* State */
    float x[EKF_STATE_DIM];             /**< State vector: [q0,q1,q2,q3,bx,by,bz] */
    
    /* Covariance */
    Matrix_t P;                          /**< State covariance (7x7) */
    
    /* Noise matrices */
    Matrix_t Q;                          /**< Process noise covariance (7x7) */
    Matrix_t R;                          /**< Measurement noise covariance (3x3) */
    
    /* Callbacks */
    EKF_TransitionFunc_t transitionFunc;     /**< State transition callback */
    EKF_MeasurementFunc_t measurementFunc;   /**< Measurement model callback */
    
    /* Configuration */
    float dt;                            /**< Time step [s] */
    uint8_t is_initialized;              /**< Initialization flag */
    
    /* Status */
    GimbalStatus_t gimbal_status;        /**< Gimbal lock status */
    
    /* Workspace matrices (avoid repeated allocation) */
    Matrix_t F;                          /**< State transition Jacobian (7x7) */
    Matrix_t H;                          /**< Measurement Jacobian (3x7) */
    Matrix_t K;                          /**< Kalman gain (7x3) */
    Matrix_t temp7x7_1;                  /**< Temporary 7x7 matrix */
    Matrix_t temp7x7_2;                  /**< Temporary 7x7 matrix */
    Matrix_t temp7x3;                    /**< Temporary 7x3 matrix */
    Matrix_t temp3x7;                    /**< Temporary 3x7 matrix */
    Matrix_t temp3x3;                    /**< Temporary 3x3 matrix */
};

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize EKF handle
 * @param[out] handle           EKF handle to initialize
 * @param[in]  transitionFunc   State transition callback
 * @param[in]  measurementFunc  Measurement model callback
 * @param[in]  dt               Time step [s] (e.g., 0.005 for 200Hz)
 * @return EKF_OK on success, error code otherwise
 * 
 * @details Initializes:
 *          - State to identity quaternion and zero bias
 *          - Covariance P to default values
 *          - Q and R to default values (tune later)
 *          - Workspace matrices
 */
int8_t ekfInit(EKF_Handle_t *handle,
               EKF_TransitionFunc_t transitionFunc,
               EKF_MeasurementFunc_t measurementFunc,
               float dt);

/**
 * @brief Set process noise covariance Q
 * @param[in,out] handle  EKF handle
 * @param[in]     noise   Process noise configuration
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfSetProcessNoise(EKF_Handle_t *handle, const EKF_ProcessNoise_t *noise);

/**
 * @brief Set measurement noise covariance R
 * @param[in,out] handle  EKF handle
 * @param[in]     noise   Measurement noise configuration
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfSetMeasurementNoise(EKF_Handle_t *handle, const EKF_MeasurementNoise_t *noise);

/**
 * @brief Set initial state covariance P
 * @param[in,out] handle   EKF handle
 * @param[in]     p_quat   Initial quaternion covariance (diagonal)
 * @param[in]     p_bias   Initial bias covariance (diagonal)
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfSetInitialCovariance(EKF_Handle_t *handle, float p_quat, float p_bias);

/**
 * @brief Reset EKF to initial state
 * @param[in,out] handle  EKF handle
 * @return EKF_OK on success, error code otherwise
 * 
 * @details Resets state to identity quaternion and zero bias.
 *          Covariance P is reset to initial values.
 */
int8_t ekfReset(EKF_Handle_t *handle);

/*******************************************************************************
 * EKF Core Functions
 ******************************************************************************/

/**
 * @brief EKF Predict step (Time Update)
 * @param[in,out] handle  EKF handle
 * @param[in]     gyro    Gyroscope measurement [rad/s] (array of 3: gx, gy, gz)
 * @return EKF_OK on success, error code otherwise
 * 
 * @details Propagates state and covariance using gyro measurement:
 *          1. Calls transitionFunc to get x_new and F Jacobian
 *          2. Updates state: x = x_new
 *          3. Updates covariance: P = F * P * F' + Q
 *          4. Normalizes quaternion part of state
 * 
 * @note Call this at IMU rate (e.g., 200Hz)
 */
int8_t ekfPredict(EKF_Handle_t *handle, const float *gyro);

/**
 * @brief EKF Correct step (Measurement Update)
 * @param[in,out] handle  EKF handle
 * @param[in]     accel   Accelerometer measurement [g] (array of 3: ax, ay, az)
 * @return EKF_OK on success, error code otherwise
 * 
 * @details Corrects state using accelerometer measurement:
 *          1. Calls measurementFunc to get z_pred and H Jacobian
 *          2. Computes innovation: y = z - z_pred
 *          3. Computes innovation covariance: S = H * P * H' + R
 *          4. Computes Kalman gain: K = P * H' * S^(-1)
 *          5. Updates state: x = x + K * y
 *          6. Updates covariance: P = (I - K * H) * P
 *          7. Normalizes quaternion
 * 
 * @note Best used when device is relatively stationary (no large accelerations)
 * @note Can call at same rate as predict or lower rate
 */
int8_t ekfCorrect(EKF_Handle_t *handle, const float *accel);

/*******************************************************************************
 * State Access Functions
 ******************************************************************************/

/**
 * @brief Get current quaternion from state
 * @param[in]  handle  EKF handle
 * @param[out] q       Output quaternion structure
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetQuaternion(const EKF_Handle_t *handle, Quaternion_t *q);

/**
 * @brief Get current Euler angles (ZYX-321)
 * @param[in]  handle  EKF handle
 * @param[out] euler   Output Euler angles [rad]
 * @return EKF_OK on success, EKF error or QUAT_ERR_GIMBAL_LOCK if gimbal lock
 * 
 * @note Updates handle->gimbal_status internally
 */
int8_t ekfGetEulerZYX(EKF_Handle_t *handle, EulerAngles_t *euler);

/**
 * @brief Get current Euler angles in degrees
 * @param[in]  handle  EKF handle
 * @param[out] roll    Roll angle [degrees]
 * @param[out] pitch   Pitch angle [degrees]
 * @param[out] yaw     Yaw angle [degrees]
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetEulerDegrees(EKF_Handle_t *handle, float *roll, float *pitch, float *yaw);

/**
 * @brief Get current gyro bias estimate
 * @param[in]  handle  EKF handle
 * @param[out] bias    Output bias array (3 elements) [rad/s]
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetGyroBias(const EKF_Handle_t *handle, float *bias);

/**
 * @brief Get full state vector
 * @param[in]  handle  EKF handle
 * @param[out] state   Output state structure
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetState(const EKF_Handle_t *handle, EKF_State_t *state);

/**
 * @brief Get DCM (Direction Cosine Matrix)
 * @param[in]  handle  EKF handle
 * @param[out] dcm     Output DCM
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetDCM(const EKF_Handle_t *handle, DCM_t *dcm);

/**
 * @brief Get gimbal lock status
 * @param[in]  handle  EKF handle
 * @param[out] status  Output status
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetGimbalStatus(const EKF_Handle_t *handle, GimbalStatus_t *status);

/**
 * @brief Get state covariance diagonal (uncertainty)
 * @param[in]  handle    EKF handle
 * @param[out] p_diag    Output diagonal elements (7 elements)
 * @return EKF_OK on success, error code otherwise
 */
int8_t ekfGetCovarianceDiag(const EKF_Handle_t *handle, float *p_diag);

/*******************************************************************************
 * Default Callback Implementations
 ******************************************************************************/

/**
 * @brief Default state transition function using RK4
 * @details Implements quaternion kinematics with RK4 integration
 *          and numerical Jacobian computation.
 * 
 * @see EKF_TransitionFunc_t for parameter details
 */
int8_t ekfDefaultTransitionFunc(float *x_new, float *F,
                                 const float *x, const float *gyro,
                                 float dt, const EKF_Handle_t *handle);

/**
 * @brief Default measurement function
 * @details Computes expected accelerometer measurement from gravity
 *          rotated to body frame, with numerical Jacobian.
 * 
 * @see EKF_MeasurementFunc_t for parameter details
 */
int8_t ekfDefaultMeasurementFunc(float *z_pred, float *H,
                                  const float *x,
                                  const EKF_Handle_t *handle);

/*******************************************************************************
 * Helper Functions (for custom callbacks)
 ******************************************************************************/

/**
 * @brief Compute quaternion derivative for RK4
 * @param[out] q_dot    Quaternion derivative (4 elements)
 * @param[in]  q        Current quaternion (4 elements)
 * @param[in]  omega    Angular velocity [rad/s] (3 elements)
 * @return EKF_OK on success, error code otherwise
 * 
 * @details q_dot = 0.5 * q ⊗ [0, omega]
 */
int8_t ekfQuatDerivative(float *q_dot, const float *q, const float *omega);

/**
 * @brief RK4 integration step for quaternion
 * @param[out] q_new    New quaternion after integration (4 elements)
 * @param[in]  q        Current quaternion (4 elements)
 * @param[in]  omega    Angular velocity [rad/s] (3 elements)
 * @param[in]  dt       Time step [s]
 * @return EKF_OK on success, error code otherwise
 * 
 * @details 4th order Runge-Kutta integration of quaternion kinematics
 */
int8_t ekfRK4QuatIntegrate(float *q_new, const float *q, const float *omega, float dt);

/**
 * @brief Compute numerical Jacobian using finite difference
 * @param[out] J        Jacobian matrix (rows x cols, row-major)
 * @param[in]  func     Function to differentiate
 * @param[in]  x        Evaluation point
 * @param[in]  rows     Number of output dimensions
 * @param[in]  cols     Number of input dimensions
 * @param[in]  delta    Perturbation size
 * @param[in]  args     Additional arguments passed to func
 * @return EKF_OK on success, error code otherwise
 * 
 * @note Used internally for numerical Jacobian computation
 */
int8_t ekfNumericalJacobian(float *J,
                             int8_t (*func)(float *y, const float *x, void *args),
                             const float *x,
                             uint8_t rows, uint8_t cols,
                             float delta, void *args);

#ifdef __cplusplus
}
#endif

#endif /* EKF_QUAT_H */
