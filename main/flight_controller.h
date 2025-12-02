/**
 * @file flight_controller.h
 * @brief Main flight controller - orchestrates all control modules
 * 
 * @details This is the top-level module that ties together:
 *          - EKF (attitude estimation)
 *          - Attitude controller (outer loop)
 *          - Rate controller (inner loop)
 *          - Motor mixer
 * 
 * CONTROL LOOP TIMING:
 * ====================
 * 
 *   IMU @ 1000 Hz
 *        │
 *        ▼
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │                     Every 1ms (1000 Hz):                     │
 *   │                                                              │
 *   │  1. Read IMU (gyro + accel)                                 │
 *   │  2. Rate Controller (inner loop) - uses gyro directly       │
 *   │  3. Motor Mixer                                             │
 *   │  4. Update PWM outputs                                      │
 *   │                                                              │
 *   │  Every 2ms (500 Hz):                                        │
 *   │  5. EKF Predict + Correct                                   │
 *   │  6. Attitude Controller (outer loop)                        │
 *   │                                                              │
 *   └─────────────────────────────────────────────────────────────┘
 * 
 * USAGE EXAMPLE:
 * ==============
 * 
 *   // Initialization (once at startup)
 *   FlightHandle_t flight;
 *   flightInit(&flight);
 *   
 *   // Main loop (called from timer interrupt or main loop)
 *   void controlLoop(void) {
 *       static uint32_t loop_counter = 0;
 *       
 *       // Read sensors
 *       ImuData_t imu = readIMU();
 *       RcCommand_t rc = readRC();
 *       
 *       // Run flight controller
 *       MotorOutput_t motors;
 *       flightUpdate(&flight, &imu, &rc, &motors);
 *       
 *       // Apply motor outputs
 *       setMotorPWM(motors.motor);
 *       
 *       loop_counter++;
 *   }
 * 
 * ARMING/DISARMING:
 * =================
 * The flight controller includes an arming mechanism for safety:
 * - Disarmed: Motors are forced to 0% regardless of commands
 * - Armed: Normal operation, motors respond to throttle/control
 * 
 * Arming requirements (typical):
 * - Throttle at minimum
 * - Stick command (e.g., throttle low + yaw right for 2 sec)
 * - No errors (EKF healthy, gyro calibrated, etc.)
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>
#include "control_types.h"
#include "attitude_controller.h"
#include "rate_controller.h"
#include "motor_mixer.h"
#include "quaternion_math.h"

/* Forward declaration for EKF handle */
struct EKF_Handle_s;
typedef struct EKF_Handle_s EKF_Handle_t;

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Flight controller runs outer loop every N inner loop iterations
 * 
 * Inner loop: 1000 Hz (every 1ms)
 * Outer loop: 500 Hz (every 2ms) = every 2 inner loop cycles
 */
#define FLIGHT_OUTER_LOOP_DIVIDER   2

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Flight controller configuration
 */
typedef struct {
    /* Timing */
    float inner_loop_dt;        /**< Inner loop (rate) sample time [s] */
    float outer_loop_dt;        /**< Outer loop (attitude) sample time [s] */
    
    /* Sub-module configs (optional, use defaults if NULL) */
    AttitudeConfig_t *att_config;
    RateConfig_t *rate_config;
    MixerConfig_t *mixer_config;
} FlightConfig_t;

/**
 * @brief Flight controller state
 */
typedef struct {
    /* Sub-controllers */
    AttitudeHandle_t attitude;      /**< Attitude controller (outer loop) */
    RateHandle_t rate;              /**< Rate controller (inner loop) */
    MixerHandle_t mixer;            /**< Motor mixer */
    
    /* EKF handle (external, user provides) */
    EKF_Handle_t *ekf;              /**< Pointer to EKF handle */
    
    /* Current attitude (from EKF or last known) */
    Quaternion_t q_current;         /**< Current attitude quaternion */
    
    /* Rate setpoint (output of attitude controller) */
    RateSetpoint_t rate_setpoint;   /**< Current rate setpoints [deg/s] */
    
    /* Timing */
    uint32_t loop_counter;          /**< Counts inner loop iterations */
    uint8_t outer_loop_divider;     /**< Run outer loop every N iterations */
    
    /* Status */
    ArmState_e arm_state;           /**< Current arm state */
    FlightMode_e flight_mode;       /**< Current flight mode */
    SystemStatus_t status;          /**< System status flags */
    uint8_t is_initialized;         /**< Initialization flag */
    
    /* Last commands (for telemetry) */
    RcCommand_t last_rc_command;
    MotorOutput_t last_motor_output;
} FlightHandle_t;

/**
 * @brief Flight controller telemetry/debug output
 */
typedef struct {
    /* Timing */
    uint32_t loop_count;
    uint8_t outer_loop_ran;
    
    /* Attitude */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    Quaternion_t q_current;
    
    /* Errors */
    float att_error_roll;
    float att_error_pitch;
    float att_error_yaw;
    
    /* Rate */
    float gyro_roll_dps;
    float gyro_pitch_dps;
    float gyro_yaw_dps;
    float rate_sp_roll;
    float rate_sp_pitch;
    float rate_sp_yaw;
    
    /* Control outputs */
    float ctrl_roll;
    float ctrl_pitch;
    float ctrl_yaw;
    
    /* Motors */
    float motor[MOTOR_COUNT];
    
    /* Status */
    ArmState_e arm_state;
    uint8_t is_saturated;
} FlightTelemetry_t;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize flight controller with configuration
 * 
 * @param[out] flight   Flight controller handle
 * @param[in]  config   Configuration (NULL for defaults)
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightInit(FlightHandle_t *flight, const FlightConfig_t *config);

/**
 * @brief Initialize flight controller with defaults
 * 
 * @param[out] flight   Flight controller handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightInitDefault(FlightHandle_t *flight);

/**
 * @brief Set EKF handle reference
 * 
 * @param[in,out] flight    Flight controller handle
 * @param[in]     ekf       Pointer to initialized EKF handle
 * @return CTRL_OK on success, error code otherwise
 * 
 * @note Must be called before flightUpdate if using EKF
 */
int8_t flightSetEKF(FlightHandle_t *flight, EKF_Handle_t *ekf);

/**
 * @brief Reset flight controller
 * 
 * @param[in,out] flight    Flight controller handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightReset(FlightHandle_t *flight);

/*******************************************************************************
 * Main Control Loop
 ******************************************************************************/

/**
 * @brief Main flight controller update function
 * 
 * @param[in,out] flight    Flight controller handle
 * @param[in]     imu       IMU sensor data (gyro in rad/s, accel in g)
 * @param[in]     rc        RC command input
 * @param[out]    motors    Motor outputs [0-100 %]
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details This is the main function to call every 1ms (1000 Hz).
 *          It handles:
 *          1. EKF predict/correct (every 2 calls = 500 Hz)
 *          2. Attitude controller (every 2 calls = 500 Hz)
 *          3. Rate controller (every call = 1000 Hz)
 *          4. Motor mixer (every call = 1000 Hz)
 * 
 * @note Gyro input should be in RAD/S (will be converted internally)
 * @note RC roll/pitch in degrees, yaw in deg/s, throttle 0-100%
 */
int8_t flightUpdate(FlightHandle_t *flight,
                    const ImuData_t *imu,
                    const RcCommand_t *rc,
                    MotorOutput_t *motors);

/**
 * @brief Main update with telemetry output
 * 
 * @param[in,out] flight    Flight controller handle
 * @param[in]     imu       IMU sensor data
 * @param[in]     rc        RC command input
 * @param[out]    motors    Motor outputs
 * @param[out]    telem     Telemetry output (can be NULL)
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightUpdateTelemetry(FlightHandle_t *flight,
                             const ImuData_t *imu,
                             const RcCommand_t *rc,
                             MotorOutput_t *motors,
                             FlightTelemetry_t *telem);

/**
 * @brief Update without EKF (provide quaternion directly)
 * 
 * @param[in,out] flight        Flight controller handle
 * @param[in]     q_current     Current attitude quaternion
 * @param[in]     gyro_rad      Gyro data [rad/s] (array of 3)
 * @param[in]     rc            RC command input
 * @param[out]    motors        Motor outputs
 * @return CTRL_OK on success, error code otherwise
 * 
 * @note Use this if you're running EKF separately
 */
int8_t flightUpdateWithQuat(FlightHandle_t *flight,
                            const Quaternion_t *q_current,
                            const float *gyro_rad,
                            const RcCommand_t *rc,
                            MotorOutput_t *motors);

/*******************************************************************************
 * Arming Functions
 ******************************************************************************/

/**
 * @brief Arm the flight controller
 * 
 * @param[in,out] flight    Flight controller handle
 * @return CTRL_OK on success, error code otherwise
 * 
 * @note Resets all controllers when arming
 */
int8_t flightArm(FlightHandle_t *flight);

/**
 * @brief Disarm the flight controller
 * 
 * @param[in,out] flight    Flight controller handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightDisarm(FlightHandle_t *flight);

/**
 * @brief Check if flight controller is armed
 * 
 * @param[in]  flight   Flight controller handle
 * @param[out] armed    1 if armed, 0 if disarmed
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightIsArmed(const FlightHandle_t *flight, uint8_t *armed);

/**
 * @brief Toggle arm state
 * 
 * @param[in,out] flight    Flight controller handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightToggleArm(FlightHandle_t *flight);

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

/**
 * @brief Set attitude (outer loop) PID gains
 * 
 * @param[in,out] flight    Flight controller handle
 * @param[in]     axis      CTRL_AXIS_ROLL, CTRL_AXIS_PITCH, or CTRL_AXIS_YAW
 * @param[in]     kp        Proportional gain
 * @param[in]     ki        Integral gain
 * @param[in]     kd        Derivative gain
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightSetAttitudeGains(FlightHandle_t *flight, CtrlAxis_e axis,
                              float kp, float ki, float kd);

/**
 * @brief Set rate (inner loop) PID gains
 * 
 * @param[in,out] flight    Flight controller handle
 * @param[in]     axis      CTRL_AXIS_ROLL, CTRL_AXIS_PITCH, or CTRL_AXIS_YAW
 * @param[in]     kp        Proportional gain
 * @param[in]     ki        Integral gain
 * @param[in]     kd        Derivative gain
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightSetRateGains(FlightHandle_t *flight, CtrlAxis_e axis,
                          float kp, float ki, float kd);

/**
 * @brief Set flight mode
 * 
 * @param[in,out] flight    Flight controller handle
 * @param[in]     mode      Flight mode
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightSetMode(FlightHandle_t *flight, FlightMode_e mode);

/**
 * @brief Set mixer scaling
 * 
 * @param[in,out] flight        Flight controller handle
 * @param[in]     roll_scale    Roll command scaling
 * @param[in]     pitch_scale   Pitch command scaling
 * @param[in]     yaw_scale     Yaw command scaling
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightSetMixerScaling(FlightHandle_t *flight,
                             float roll_scale, float pitch_scale, float yaw_scale);

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

/**
 * @brief Get current attitude in degrees
 * 
 * @param[in]  flight   Flight controller handle
 * @param[out] roll     Roll angle [degrees]
 * @param[out] pitch    Pitch angle [degrees]
 * @param[out] yaw      Yaw angle [degrees]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightGetAttitude(const FlightHandle_t *flight,
                         float *roll, float *pitch, float *yaw);

/**
 * @brief Get current attitude quaternion
 * 
 * @param[in]  flight   Flight controller handle
 * @param[out] q        Quaternion
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightGetQuaternion(const FlightHandle_t *flight, Quaternion_t *q);

/**
 * @brief Get system status
 * 
 * @param[in]  flight   Flight controller handle
 * @param[out] status   System status
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightGetStatus(const FlightHandle_t *flight, SystemStatus_t *status);

/**
 * @brief Get motor outputs
 * 
 * @param[in]  flight   Flight controller handle
 * @param[out] motors   Motor outputs [%]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t flightGetMotors(const FlightHandle_t *flight, MotorOutput_t *motors);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROLLER_H */
