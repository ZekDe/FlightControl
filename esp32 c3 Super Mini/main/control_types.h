/**
 * @file control_types.h
 * @brief Common data types and constants for quadcopter flight control system
 * 
 * @details This file defines shared structures, enumerations, and constants
 *          used across all control modules (PID, attitude, rate, mixer).
 * 
 * @note Designed for embedded systems (ESP32, STM32, etc.)
 * @note Uses float precision for FPU compatibility
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * System Configuration
 ******************************************************************************/

/**
 * @defgroup LoopRates Control Loop Frequencies
 * @{
 */

 /* Timing periods (ms) */
#define PERIOD_IMU_MS           5       /* 200 Hz */
#define PERIOD_EKF_MS           10      /* 100 Hz */
#define PERIOD_RATE_MS          5       /* 200 Hz */
#define PERIOD_UI_MS            500     /* 10 Hz */

/* dt values for controllers (seconds) */
#define DT_EKF                  (PERIOD_EKF_MS / 1000.0f)
#define DT_ATTITUDE             (PERIOD_EKF_MS / 1000.0f)
#define DT_RATE                 (PERIOD_RATE_MS / 1000.0f)


/** @} */

/**
 * @defgroup RateLimits Maximum Angular Rates and Angles
 * @{
 */
#define CTRL_MAX_ROLL_ANGLE_DEG     45.0f   /**< Maximum roll angle [degrees] */
#define CTRL_MAX_PITCH_ANGLE_DEG    45.0f   /**< Maximum pitch angle [degrees] */
#define CTRL_MAX_ROLL_RATE_DPS      250.0f  /**< Maximum roll rate [deg/s] */
#define CTRL_MAX_PITCH_RATE_DPS     250.0f  /**< Maximum pitch rate [deg/s] */
#define CTRL_MAX_YAW_RATE_DPS       180.0f  /**< Maximum yaw rate [deg/s] */
/** @} */

/**
 * @defgroup MotorConfig Motor and Throttle Configuration
 * @{
 */
#define CTRL_MOTOR_COUNT            4       /**< Number of motors (quadcopter) */
#define CTRL_MOTOR_MIN              0.0f    /**< Minimum motor output [%] */
#define CTRL_MOTOR_MAX              100.0f  /**< Maximum motor output [%] */
#define CTRL_MOTOR_IDLE             5.0f    /**< Idle throttle when armed [%] */
/** @} */

/**
 * @defgroup FilterConfig Filter Configuration
 * @{
 */
#define CTRL_DERIVATIVE_LPF_HZ      80.0f   /**< Derivative low-pass filter cutoff [Hz] */
/** @} */

/*******************************************************************************
 * Error Codes
 ******************************************************************************/

#define CTRL_OK                     0       /**< Operation successful */
#define CTRL_ERR_NULL_PTR          -1       /**< Null pointer passed */
#define CTRL_ERR_INVALID_PARAM     -2       /**< Invalid parameter value */
#define CTRL_ERR_NOT_INITIALIZED   -3       /**< Module not initialized */
#define CTRL_ERR_NOT_ARMED         -4       /**< System not armed */
#define CTRL_ERR_COMPUTATION       -5       /**< Computation error */

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/**
 * @brief Control axis enumeration
 */
typedef enum {
    CTRL_AXIS_ROLL  = 0,    /**< Roll axis (X) */
    CTRL_AXIS_PITCH = 1,    /**< Pitch axis (Y) */
    CTRL_AXIS_YAW   = 2,    /**< Yaw axis (Z) */
    CTRL_AXIS_COUNT = 3     /**< Number of axes */
} CtrlAxis_e;

/**
 * @brief Motor position enumeration (X configuration)
 * 
 * @details Motor layout (top view, arrow is front):
 *          
 *               Front
 *                 ↑
 *          M1 (CW)   M2 (CCW)
 *              \     /
 *               \   /
 *                \ /
 *                 X
 *                / \
 *               /   \
 *              /     \
 *          M4 (CCW)  M3 (CW)
 *               Rear
 */
typedef enum {
    MOTOR_FRONT_RIGHT = 0,  /**< Motor 1: Front-right, CW rotation */
    MOTOR_REAR_RIGHT  = 1,  /**< Motor 2: Rear-right, CCW rotation */
    MOTOR_REAR_LEFT   = 2,  /**< Motor 3: Rear-left, CW rotation */
    MOTOR_FRONT_LEFT  = 3,  /**< Motor 4: Front-left, CCW rotation */
    MOTOR_COUNT       = 4   /**< Total motor count */
} MotorPosition_e;

/**
 * @brief Arming state enumeration
 */
typedef enum {
    ARM_STATE_DISARMED = 0, /**< Motors disabled, safe state */
    ARM_STATE_ARMED    = 1  /**< Motors enabled, ready to fly */
} ArmState_e;

/**
 * @brief Flight mode enumeration (for future expansion)
 */
typedef enum {
    FLIGHT_MODE_STABILIZE = 0,  /**< Angle stabilization mode */
    FLIGHT_MODE_ACRO      = 1,  /**< Rate/Acro mode (future) */
    FLIGHT_MODE_ALT_HOLD  = 2,  /**< Altitude hold (future) */
    FLIGHT_MODE_POS_HOLD  = 3   /**< Position hold (future) */
} FlightMode_e;

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief 3-axis float vector
 * @note Used for gyro, accel, angles, rates, etc.
 */
typedef struct {
    float roll;     /**< Roll component (X axis) */
    float pitch;    /**< Pitch component (Y axis) */
    float yaw;      /**< Yaw component (Z axis) */
} AxisData_t;

/**
 * @brief RC command input structure
 * 
 * @details Commands from receiver/transmitter
 *          Roll/Pitch: Angle setpoints in degrees [-MAX_ANGLE, +MAX_ANGLE]
 *          Yaw: Rate setpoint in deg/s [-MAX_YAW_RATE, +MAX_YAW_RATE]
 *          Throttle: Direct throttle command [0, 100]%
 */
typedef struct {
    float roll;         /**< Roll angle command [degrees] */
    float pitch;        /**< Pitch angle command [degrees] */
    float yaw;          /**< Yaw rate command [deg/s] */
    float throttle;     /**< Throttle command [0-100 %] */
} RcCommand_t;

/**
 * @brief Motor output structure
 */
typedef struct {
    float motor[MOTOR_COUNT];   /**< Motor outputs [0-100 %] */
} MotorOutput_t;

/**
 * @brief Rate setpoint structure (output of angle controller)
 */
typedef struct {
    float roll_rate;    /**< Desired roll rate [deg/s] */
    float pitch_rate;   /**< Desired pitch rate [deg/s] */
    float yaw_rate;     /**< Desired yaw rate [deg/s] */
} RateSetpoint_t;

/**
 * @brief PID output structure
 */
typedef struct {
    float roll;         /**< Roll PID output */
    float pitch;        /**< Pitch PID output */
    float yaw;          /**< Yaw PID output */
} PidOutput_t;

/**
 * @brief System status flags
 */
typedef struct {
    uint8_t is_armed;           /**< 1 if armed, 0 if disarmed */
    uint8_t ekf_healthy;        /**< 1 if EKF is healthy */
    uint8_t gyro_healthy;       /**< 1 if gyro data valid */
    uint8_t rc_healthy;         /**< 1 if RC signal valid */
    uint8_t motor_saturated;    /**< 1 if any motor at limit */
} SystemStatus_t;

/**
 * @brief IMU data structure (raw sensor data)
 */
typedef struct {
    float gyro_x;       /**< Gyroscope X [rad/s] */
    float gyro_y;       /**< Gyroscope Y [rad/s] */
    float gyro_z;       /**< Gyroscope Z [rad/s] */
    float accel_x;      /**< Accelerometer X [g] */
    float accel_y;      /**< Accelerometer Y [g] */
    float accel_z;      /**< Accelerometer Z [g] */
} ImuData_t;

/*******************************************************************************
 * Utility Macros
 ******************************************************************************/

/**
 * @brief Constrain value between min and max
 */
#define CTRL_CONSTRAIN(val, min, max) \
    (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

/**
 * @brief Convert degrees to radians
 */
#define CTRL_DEG_TO_RAD(deg)    ((deg) * 0.01745329252f)

/**
 * @brief Convert radians to degrees
 */
#define CTRL_RAD_TO_DEG(rad)    ((rad) * 57.29577951f)

/**
 * @brief Absolute value for float
 */
#define CTRL_ABS(x)             (((x) < 0) ? (-(x)) : (x))

/**
 * @brief Sign of a value (-1, 0, or 1)
 */
#define CTRL_SIGN(x)            (((x) > 0) - ((x) < 0))

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_TYPES_H */
