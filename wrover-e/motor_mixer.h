/**
 * @file motor_mixer.h
 * @brief Motor mixer for quadcopter (X configuration)
 * 
 * @details Converts throttle and control commands (roll, pitch, yaw) into
 *          individual motor outputs for a quadcopter in X configuration.
 * 
 * MOTOR MIXING THEORY:
 * ====================
 * A quadcopter generates torques by differential motor speeds:
 * 
 * - ROLL: Difference between left and right motor pairs
 *   Positive roll (right side down) -> increase left motors, decrease right
 * 
 * - PITCH: Difference between front and rear motor pairs
 *   Positive pitch (nose up) -> increase rear motors, decrease front
 * 
 * - YAW: Difference between CW and CCW motor pairs
 *   Positive yaw (nose right) -> increase CCW motors, decrease CW
 * 
 * - THRUST: Sum of all motor outputs
 *   Throttle directly added to all motors
 * 
 * X CONFIGURATION LAYOUT:
 * =======================
 * 
 *              Front
 *                ↑
 *                │
 *     M1 (CW)    │    M2 (CCW)
 *         ╲      │      ╱
 *          ╲     │     ╱
 *           ╲    │    ╱
 *            ╲   │   ╱
 *             ╲  │  ╱
 *              ╲ │ ╱
 *               ╲│╱
 *                X ────────▶ Right
 *               ╱│╲
 *              ╱ │ ╲
 *             ╱  │  ╲
 *            ╱   │   ╲
 *           ╱    │    ╲
 *          ╱     │     ╲
 *         ╱      │      ╲
 *     M4 (CCW)   │   M3 (CW)
 *                │
 *              Rear
 * 
 * Motor rotation:
 * - M1 (front-right): Clockwise (CW)     - creates CCW yaw torque
 * - M2 (rear-right):  Counter-CW (CCW)   - creates CW yaw torque
 * - M3 (rear-left):   Clockwise (CW)     - creates CCW yaw torque
 * - M4 (front-left):  Counter-CW (CCW)   - creates CW yaw torque
 * 
 * MIXING EQUATIONS (X config):
 * ============================
 * All motors at 45° from body axes, so all contribute equally to roll/pitch.
 * 
 * M1 = Throttle - Roll - Pitch - Yaw  (front-right, CW)
 * M2 = Throttle - Roll + Pitch + Yaw  (rear-right, CCW)
 * M3 = Throttle + Roll + Pitch - Yaw  (rear-left, CW)
 * M4 = Throttle + Roll - Pitch + Yaw  (front-left, CCW)
 * 
 * Sign convention:
 * - Positive roll  = right side down = increase M3,M4, decrease M1,M2
 * - Positive pitch = nose up         = increase M2,M3, decrease M1,M4
 * - Positive yaw   = nose right      = increase M2,M4, decrease M1,M3
 * 
 * OUTPUT SCALING:
 * ===============
 * Control inputs (roll, pitch, yaw) have arbitrary units from PID.
 * Throttle is 0-100%.
 * Motor output is 0-100%.
 * 
 * The mixer:
 * 1. Scales control commands by configurable gains
 * 2. Adds to base throttle
 * 3. Handles saturation (motors can't go below 0 or above 100)
 * 4. Optionally scales to preserve control authority when throttle limited
 * 
 * @author Claude & duatepe
 * @date 2024
 */

#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdint.h>
#include "control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

/**
 * @brief Default motor output scaling
 * 
 * @note These scale the PID output (typically ±500) to motor range.
 *       Adjust based on your PID output range.
 *       
 *       If rate PID outputs ±500 and you want max 50% motor differential:
 *       scale = 50 / 500 = 0.1
 */
#define MIXER_DEFAULT_ROLL_SCALE    0.1f    /**< Roll command scale */
#define MIXER_DEFAULT_PITCH_SCALE   0.1f    /**< Pitch command scale */
#define MIXER_DEFAULT_YAW_SCALE     0.1f    /**< Yaw command scale */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Mixer configuration
 */
typedef struct {
    /* Command scaling factors */
    float roll_scale;       /**< Roll command scaling (PID output -> motor %) */
    float pitch_scale;      /**< Pitch command scaling */
    float yaw_scale;        /**< Yaw command scaling */
    
    /* Output limits */
    float motor_min;        /**< Minimum motor output [%] */
    float motor_max;        /**< Maximum motor output [%] */
    float motor_idle;       /**< Idle motor output when armed [%] */
    
    /* Options */
    uint8_t airmode_enabled;    /**< 1 = allow motors below idle for control */
} MixerConfig_t;

/**
 * @brief Mixer input (from controllers)
 */
typedef struct {
    float throttle;     /**< Throttle command [0-100 %] */
    float roll;         /**< Roll control command (from rate PID) */
    float pitch;        /**< Pitch control command (from rate PID) */
    float yaw;          /**< Yaw control command (from rate PID) */
} MixerInput_t;

/**
 * @brief Mixer handle
 */
typedef struct {
    /* Configuration */
    MixerConfig_t config;
    
    /* State */
    float motor_output[MOTOR_COUNT];    /**< Last motor outputs [%] */
    uint8_t is_saturated;               /**< 1 if any motor hit limit */
    uint8_t is_initialized;             /**< Initialization flag */
    
    /* Statistics */
    float max_command;      /**< Largest control command magnitude */
    float min_motor;        /**< Lowest motor output (before limiting) */
    float max_motor;        /**< Highest motor output (before limiting) */
} MixerHandle_t;

/**
 * @brief Mixer debug output
 */
typedef struct {
    /* Inputs (scaled) */
    float throttle;
    float roll_scaled;
    float pitch_scaled;
    float yaw_scaled;
    
    /* Raw motor values (before limiting) */
    float motor_raw[MOTOR_COUNT];
    
    /* Final motor values */
    float motor_out[MOTOR_COUNT];
    
    /* Status */
    uint8_t saturated_high[MOTOR_COUNT];    /**< 1 if motor hit max */
    uint8_t saturated_low[MOTOR_COUNT];     /**< 1 if motor hit min */
    float headroom;                          /**< Available throttle headroom */
} MixerDebug_t;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

/**
 * @brief Initialize mixer with configuration
 * 
 * @param[out] mixer    Mixer handle
 * @param[in]  config   Configuration parameters
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerInit(MixerHandle_t *mixer, const MixerConfig_t *config);

/**
 * @brief Initialize mixer with default parameters
 * 
 * @param[out] mixer    Mixer handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerInitDefault(MixerHandle_t *mixer);

/**
 * @brief Reset mixer state
 * 
 * @param[in,out] mixer Mixer handle
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerReset(MixerHandle_t *mixer);

/*******************************************************************************
 * Runtime Functions
 ******************************************************************************/

/**
 * @brief Mix control commands to motor outputs
 * 
 * @param[in,out] mixer     Mixer handle
 * @param[in]     input     Throttle and control commands
 * @param[out]    output    Motor outputs [%]
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Mixing algorithm:
 * 
 * 1. Scale control commands:
 *    roll_cmd  = roll * roll_scale
 *    pitch_cmd = pitch * pitch_scale
 *    yaw_cmd   = yaw * yaw_scale
 * 
 * 2. Compute raw motor values (X config):
 *    M1 = throttle - roll_cmd - pitch_cmd - yaw_cmd
 *    M2 = throttle - roll_cmd + pitch_cmd + yaw_cmd
 *    M3 = throttle + roll_cmd + pitch_cmd - yaw_cmd
 *    M4 = throttle + roll_cmd - pitch_cmd + yaw_cmd
 * 
 * 3. Handle saturation:
 *    - Constrain to [motor_min, motor_max]
 *    - If armed and throttle > 0: minimum = idle
 *    - Set saturation flags
 * 
 * @note Motor indices: 0=M1(FR), 1=M2(RR), 2=M3(RL), 3=M4(FL)
 */
int8_t mixerUpdate(MixerHandle_t *mixer,
                   const MixerInput_t *input,
                   MotorOutput_t *output);

/**
 * @brief Mix with debug output
 * 
 * @param[in,out] mixer     Mixer handle
 * @param[in]     input     Throttle and control commands
 * @param[out]    output    Motor outputs [%]
 * @param[out]    debug     Debug information (can be NULL)
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerUpdateDebug(MixerHandle_t *mixer,
                        const MixerInput_t *input,
                        MotorOutput_t *output,
                        MixerDebug_t *debug);

/**
 * @brief Convenience function with individual inputs
 * 
 * @param[in,out] mixer     Mixer handle
 * @param[in]     throttle  Throttle [0-100 %]
 * @param[in]     roll      Roll command (from rate PID)
 * @param[in]     pitch     Pitch command (from rate PID)
 * @param[in]     yaw       Yaw command (from rate PID)
 * @param[out]    output    Motor outputs [%]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerMix(MixerHandle_t *mixer,
                float throttle, float roll, float pitch, float yaw,
                MotorOutput_t *output);

/*******************************************************************************
 * Configuration Functions
 ******************************************************************************/

/**
 * @brief Set command scaling factors
 * 
 * @param[in,out] mixer         Mixer handle
 * @param[in]     roll_scale    Roll scaling
 * @param[in]     pitch_scale   Pitch scaling
 * @param[in]     yaw_scale     Yaw scaling
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerSetScaling(MixerHandle_t *mixer,
                       float roll_scale, float pitch_scale, float yaw_scale);

/**
 * @brief Set motor output limits
 * 
 * @param[in,out] mixer         Mixer handle
 * @param[in]     motor_min     Minimum output [%]
 * @param[in]     motor_max     Maximum output [%]
 * @param[in]     motor_idle    Idle output [%]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerSetLimits(MixerHandle_t *mixer,
                      float motor_min, float motor_max, float motor_idle);

/**
 * @brief Enable/disable airmode
 * 
 * @param[in,out] mixer     Mixer handle
 * @param[in]     enable    1 to enable, 0 to disable
 * @return CTRL_OK on success, error code otherwise
 * 
 * @details Airmode allows motors to go below idle for better control
 *          authority at low throttle. Use with caution!
 */
int8_t mixerSetAirmode(MixerHandle_t *mixer, uint8_t enable);

/*******************************************************************************
 * Query Functions
 ******************************************************************************/

/**
 * @brief Get current motor outputs
 * 
 * @param[in]  mixer    Mixer handle
 * @param[out] output   Motor outputs [%]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerGetOutput(const MixerHandle_t *mixer, MotorOutput_t *output);

/**
 * @brief Check if mixer output is saturated
 * 
 * @param[in]  mixer        Mixer handle
 * @param[out] saturated    1 if any motor at limit, 0 otherwise
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerIsSaturated(const MixerHandle_t *mixer, uint8_t *saturated);

/**
 * @brief Get motor output for specific motor
 * 
 * @param[in]  mixer    Mixer handle
 * @param[in]  motor    Motor index (0-3)
 * @param[out] output   Motor output [%]
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerGetMotor(const MixerHandle_t *mixer, uint8_t motor, float *output);

/*******************************************************************************
 * Disarm Function
 ******************************************************************************/

/**
 * @brief Set all motors to zero (disarmed state)
 * 
 * @param[in,out] mixer     Mixer handle
 * @param[out]    output    Motor outputs (will be all zeros)
 * @return CTRL_OK on success, error code otherwise
 */
int8_t mixerDisarm(MixerHandle_t *mixer, MotorOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MIXER_H */
