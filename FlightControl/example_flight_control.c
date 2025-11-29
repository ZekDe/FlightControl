/**
 * @file example_flight_control.c
 * @brief Example usage of the quadcopter flight control system
 * 
 * @details This file demonstrates how to integrate and use the flight
 *          controller with the EKF attitude estimation system.
 * 
 * SYSTEM OVERVIEW:
 * ================
 * 
 *   ┌─────────────┐     ┌─────────────┐     ┌─────────────────────────────────┐
 *   │    IMU      │────▶│    EKF      │────▶│     FLIGHT CONTROLLER           │
 *   │ (MPU9250)   │     │ (Attitude)  │     │                                 │
 *   │ @ 1000 Hz   │     │ @ 500 Hz    │     │  ┌─────────┐    ┌─────────┐    │
 *   └─────────────┘     └─────────────┘     │  │Attitude │───▶│  Rate   │    │
 *                                           │  │  PID    │    │  PID    │    │
 *   ┌─────────────┐                         │  │ @ 500Hz │    │ @ 1kHz  │    │
 *   │     RC      │────────────────────────▶│  └─────────┘    └────┬────┘    │
 *   │  Receiver   │                         │                      │         │
 *   └─────────────┘                         │  ┌─────────┐         │         │
 *                                           │  │  Mixer  │◀────────┘         │
 *                                           │  └────┬────┘                   │
 *                                           └───────┼───────────────────────-┘
 *                                                   │
 *                                                   ▼
 *                                           ┌─────────────┐
 *                                           │   Motors    │
 *                                           │  (4x ESC)   │
 *                                           └─────────────┘
 * 
 * TIMING DIAGRAM:
 * ===============
 * 
 *   Time:    0ms   1ms   2ms   3ms   4ms   5ms   6ms   7ms   8ms
 *            │     │     │     │     │     │     │     │     │
 *   IMU:     ●     ●     ●     ●     ●     ●     ●     ●     ●  (1000 Hz)
 *   EKF:     ●           ●           ●           ●           ●  (500 Hz)
 *   Angle:   ●           ●           ●           ●           ●  (500 Hz)
 *   Rate:    ●     ●     ●     ●     ●     ●     ●     ●     ●  (1000 Hz)
 *   Motors:  ●     ●     ●     ●     ●     ●     ●     ●     ●  (1000 Hz)
 * 
 * @note This is a simulation example - adapt sensor/motor interfaces for your hardware
 */

#include <stdio.h>
#include <string.h>

/* Include all control modules */
#include "control_types.h"
#include "pid_controller.h"
#include "attitude_controller.h"
#include "rate_controller.h"
#include "motor_mixer.h"
#include "flight_controller.h"

/* Include EKF modules */
#include "ekf_quat.h"
#include "quaternion_math.h"
#include "matrix_math.h"

/*******************************************************************************
 * Hardware Abstraction (Replace with your actual hardware drivers)
 ******************************************************************************/

/**
 * @brief Simulated IMU reading
 */
static void readIMU(ImuData_t *imu)
{
    /* In real implementation, read from MPU9250 or similar */
    /* Values in rad/s for gyro, g for accel */
    
    /* Simulated level hover with slight disturbance */
    imu->gyro_x = 0.01f;    /* Small roll rate */
    imu->gyro_y = 0.005f;   /* Small pitch rate */
    imu->gyro_z = 0.0f;     /* No yaw rate */
    
    imu->accel_x = 0.0f;    /* Level */
    imu->accel_y = 0.0f;    /* Level */
    imu->accel_z = 1.0f;    /* 1g down (NED: +Z is down) */
}

/**
 * @brief Simulated RC command reading
 */
static void readRC(RcCommand_t *rc)
{
    /* In real implementation, read from SBUS, PPM, or similar */
    
    /* Simulated stick positions for hover */
    rc->roll = 0.0f;        /* Centered */
    rc->pitch = 0.0f;       /* Centered */
    rc->yaw = 0.0f;         /* Centered (no yaw rate) */
    rc->throttle = 50.0f;   /* Mid throttle for hover */
}

/**
 * @brief Apply motor outputs to ESCs
 */
static void setMotors(const MotorOutput_t *motors)
{
    /* In real implementation, write to PWM/DShot */
    
    printf("Motors: M1=%.1f%%, M2=%.1f%%, M3=%.1f%%, M4=%.1f%%\n",
           motors->motor[0], motors->motor[1],
           motors->motor[2], motors->motor[3]);
}

/**
 * @brief Delay function (for simulation timing)
 */
static void delayMs(uint32_t ms)
{
    /* In real implementation, use hardware timer or RTOS delay */
    (void)ms;
}

/*******************************************************************************
 * Example 1: Basic Flight Controller Usage
 ******************************************************************************/

/**
 * @brief Basic example using integrated flight controller + EKF
 */
void example_basic_usage(void)
{
    printf("\n=== Example 1: Basic Flight Controller ===\n\n");
    
    /*=========================================================================
     * Step 1: Initialize EKF
     *=========================================================================
     */
    EKF_Handle_t ekf;
    
    /* Initialize with default transition and measurement functions */
    int8_t ret = ekfInit(&ekf, 
                         ekfDefaultTransitionFunc, 
                         ekfDefaultMeasurementFunc,
                         CTRL_ANGLE_PID_DT);  /* 2ms = 500 Hz */
    
    if (ret != EKF_OK) {
        printf("EKF init failed: %d\n", ret);
        return;
    }
    
    /* Configure EKF noise parameters */
    EKF_ProcessNoise_t q_noise = {
        .q_quat = 1e-6f,    /* Quaternion process noise */
        .q_bias = 1e-8f     /* Gyro bias process noise */
    };
    ekfSetProcessNoise(&ekf, &q_noise);
    
    EKF_MeasurementNoise_t r_noise = {
        .r_accel = 0.1f     /* Accelerometer measurement noise */
    };
    ekfSetMeasurementNoise(&ekf, &r_noise);
    
    printf("EKF initialized\n");
    
    /*=========================================================================
     * Step 2: Initialize Flight Controller
     *=========================================================================
     */
    FlightHandle_t flight;
    
    ret = flightInitDefault(&flight);
    if (ret != CTRL_OK) {
        printf("Flight controller init failed: %d\n", ret);
        return;
    }
    
    /* Link EKF to flight controller */
    flightSetEKF(&flight, &ekf);
    
    printf("Flight controller initialized\n");
    
    /*=========================================================================
     * Step 3: Main Control Loop (Simulated)
     *=========================================================================
     */
    printf("\nStarting control loop simulation...\n\n");
    
    /* Arm the system */
    flightArm(&flight);
    printf("System ARMED\n\n");
    
    /* Simulate 10 control iterations */
    for (int i = 0; i < 10; i++) {
        /* Read sensors */
        ImuData_t imu;
        readIMU(&imu);
        
        /* Read RC */
        RcCommand_t rc;
        readRC(&rc);
        
        /* Run flight controller */
        MotorOutput_t motors;
        FlightTelemetry_t telem;
        
        ret = flightUpdateTelemetry(&flight, &imu, &rc, &motors, &telem);
        
        if (ret == CTRL_OK) {
            printf("Loop %d: ", i + 1);
            printf("Att[%.1f, %.1f, %.1f] ", 
                   telem.roll_deg, telem.pitch_deg, telem.yaw_deg);
            printf("RateSP[%.1f, %.1f, %.1f] ",
                   telem.rate_sp_roll, telem.rate_sp_pitch, telem.rate_sp_yaw);
            setMotors(&motors);
        }
        
        delayMs(1);  /* 1ms loop */
    }
    
    /* Disarm */
    flightDisarm(&flight);
    printf("\nSystem DISARMED\n");
}

/*******************************************************************************
 * Example 2: Custom PID Tuning
 ******************************************************************************/

/**
 * @brief Example showing how to tune PID gains
 */
void example_pid_tuning(void)
{
    printf("\n=== Example 2: PID Tuning ===\n\n");
    
    FlightHandle_t flight;
    flightInitDefault(&flight);
    
    /* Get current gains and display */
    printf("Default attitude gains (roll/pitch): Kp=6.0, Ki=0.5, Kd=0.0\n");
    printf("Default rate gains (roll/pitch): Kp=0.25, Ki=0.30, Kd=0.003\n\n");
    
    /*=========================================================================
     * Tune Attitude Controller (Outer Loop)
     *=========================================================================
     * Start with:
     * - Kp: Increase until responsive but not oscillating
     * - Ki: Add small amount to eliminate steady-state error
     * - Kd: Usually 0 for attitude loop
     */
    printf("Setting custom attitude gains...\n");
    flightSetAttitudeGains(&flight, CTRL_AXIS_ROLL, 8.0f, 0.3f, 0.0f);
    flightSetAttitudeGains(&flight, CTRL_AXIS_PITCH, 8.0f, 0.3f, 0.0f);
    flightSetAttitudeGains(&flight, CTRL_AXIS_YAW, 5.0f, 0.2f, 0.0f);
    
    /*=========================================================================
     * Tune Rate Controller (Inner Loop)
     *=========================================================================
     * Tune this FIRST, then tune attitude loop.
     * Start with:
     * - Kp: Main response, increase until slight oscillation
     * - Ki: Add to remove drift
     * - Kd: Important! Add to dampen oscillation
     */
    printf("Setting custom rate gains...\n");
    flightSetRateGains(&flight, CTRL_AXIS_ROLL, 0.30f, 0.35f, 0.004f);
    flightSetRateGains(&flight, CTRL_AXIS_PITCH, 0.30f, 0.35f, 0.004f);
    flightSetRateGains(&flight, CTRL_AXIS_YAW, 0.50f, 0.25f, 0.0f);
    
    printf("Custom gains set!\n");
    
    /*=========================================================================
     * Tune Mixer Scaling
     *=========================================================================
     * Adjust how much the PID outputs affect motor speeds.
     * Lower = smoother but less responsive
     * Higher = more aggressive response
     */
    printf("Setting mixer scaling...\n");
    flightSetMixerScaling(&flight, 0.12f, 0.12f, 0.08f);
    
    printf("Tuning complete!\n");
}

/*******************************************************************************
 * Example 3: Direct Module Usage (Without FlightController Wrapper)
 ******************************************************************************/

/**
 * @brief Example using individual modules directly
 */
void example_direct_module_usage(void)
{
    printf("\n=== Example 3: Direct Module Usage ===\n\n");
    
    /*=========================================================================
     * Initialize Individual Modules
     *=========================================================================
     */
    
    /* Attitude controller */
    AttitudeHandle_t attitude;
    attitudeInitDefault(&attitude, CTRL_ANGLE_PID_DT);
    
    /* Rate controller */
    RateHandle_t rate;
    rateInitDefault(&rate, CTRL_RATE_PID_DT);
    
    /* Motor mixer */
    MixerHandle_t mixer;
    mixerInitDefault(&mixer);
    
    printf("Modules initialized\n");
    
    /*=========================================================================
     * Simulate One Control Iteration
     *=========================================================================
     */
    
    /* Simulated inputs */
    Quaternion_t q_current;
    quatIdentity(&q_current);  /* Level attitude */
    
    AttitudeSetpoint_t att_setpoint = {
        .roll_deg = 5.0f,       /* Command 5° roll */
        .pitch_deg = 0.0f,
        .yaw_rate_dps = 0.0f
    };
    
    GyroData_t gyro = {
        .roll = 0.0f,           /* No current rotation */
        .pitch = 0.0f,
        .yaw = 0.0f
    };
    
    float throttle = 50.0f;     /* 50% throttle */
    
    /*=========================================================================
     * Step 1: Attitude Controller -> Rate Setpoints
     *=========================================================================
     */
    RateSetpoint_t rate_setpoint;
    attitudeUpdate(&attitude, &att_setpoint, &q_current, &rate_setpoint);
    
    printf("Attitude error -> Rate setpoint: [%.2f, %.2f, %.2f] deg/s\n",
           rate_setpoint.roll_rate, rate_setpoint.pitch_rate, rate_setpoint.yaw_rate);
    
    /*=========================================================================
     * Step 2: Rate Controller -> Control Commands
     *=========================================================================
     */
    RateOutput_t rate_output;
    rateUpdate(&rate, &rate_setpoint, &gyro, &rate_output);
    
    printf("Rate error -> Control output: [%.2f, %.2f, %.2f]\n",
           rate_output.roll, rate_output.pitch, rate_output.yaw);
    
    /*=========================================================================
     * Step 3: Mixer -> Motor Outputs
     *=========================================================================
     */
    MixerInput_t mixer_input = {
        .throttle = throttle,
        .roll = rate_output.roll,
        .pitch = rate_output.pitch,
        .yaw = rate_output.yaw
    };
    
    MotorOutput_t motors;
    mixerUpdate(&mixer, &mixer_input, &motors);
    
    printf("Mixer -> Motor outputs: [%.1f%%, %.1f%%, %.1f%%, %.1f%%]\n",
           motors.motor[0], motors.motor[1], motors.motor[2], motors.motor[3]);
}

/*******************************************************************************
 * Example 4: PID Debug Output
 ******************************************************************************/

/**
 * @brief Example showing PID debug/telemetry
 */
void example_pid_debug(void)
{
    printf("\n=== Example 4: PID Debug Output ===\n\n");
    
    /* Create a simple PID */
    PidHandle_t pid;
    pidInitParams(&pid, 
                  1.0f,      /* Kp */
                  0.1f,      /* Ki */
                  0.01f,     /* Kd */
                  -100.0f,   /* min output */
                  100.0f,    /* max output */
                  80.0f,     /* derivative LPF Hz */
                  0.001f);   /* 1ms sample time */
    
    /* Simulate step response */
    float setpoint = 10.0f;
    float measurement = 0.0f;
    
    printf("Step response (setpoint = %.1f):\n", setpoint);
    printf("Time    Meas    Error   P       I       D       Output\n");
    printf("------  ------  ------  ------  ------  ------  ------\n");
    
    for (int i = 0; i < 20; i++) {
        float output;
        PidDebug_t debug;
        
        pidUpdateDebug(&pid, setpoint, measurement, &output, &debug);
        
        printf("%4dms  %6.2f  %6.2f  %6.2f  %6.2f  %6.2f  %6.2f\n",
               i, measurement, debug.error, 
               debug.p_term, debug.i_term, debug.d_term,
               debug.output_sat);
        
        /* Simulate plant response (simple integrator) */
        measurement += output * 0.01f;
    }
}

/*******************************************************************************
 * Main Entry Point
 ******************************************************************************/

int main(void)
{
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║      QUADCOPTER FLIGHT CONTROL SYSTEM - EXAMPLE USAGE         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    /* Run examples */
    example_basic_usage();
    example_pid_tuning();
    example_direct_module_usage();
    example_pid_debug();
    
    printf("\n=== All Examples Complete ===\n");
    
    return 0;
}

/*******************************************************************************
 * ESP32 Integration Notes
 ******************************************************************************/

/*
 * For ESP32 integration, replace the simulation functions with actual drivers:
 * 
 * 1. IMU (MPU9250/MPU6050):
 *    - Configure I2C or SPI interface
 *    - Set sample rate to 1000 Hz
 *    - Configure gyro full-scale (±500 or ±1000 dps recommended)
 *    - Configure accel full-scale (±4g or ±8g recommended)
 *    - Apply calibration offsets
 * 
 * 2. RC Receiver (SBUS/IBUS/PPM):
 *    - Configure UART for SBUS (100000 baud, 8E2)
 *    - Parse RC channels to roll/pitch/yaw/throttle
 *    - Apply expo/rates if desired
 *    - Handle failsafe
 * 
 * 3. ESCs (PWM/DShot):
 *    - Configure LEDC peripheral for PWM (400Hz standard, or 1-2kHz for fast ESCs)
 *    - Or configure RMT for DShot protocol
 *    - Map motor outputs (0-100%) to PWM duty cycle (1000-2000µs)
 * 
 * 4. Timing:
 *    - Use hardware timer for precise 1ms interrupt
 *    - Run flightUpdate() in timer ISR or high-priority task
 *    - Keep ISR short, defer non-critical work
 * 
 * 5. Safety:
 *    - Implement arming checks (throttle low, sensors OK)
 *    - Add watchdog timer
 *    - Implement failsafe (cut throttle on RC loss)
 * 
 * Example ESP32 Timer Setup:
 * 
 *   hw_timer_t *timer = timerBegin(0, 80, true);  // 80 prescaler = 1MHz
 *   timerAttachInterrupt(timer, &controlLoopISR, true);
 *   timerAlarmWrite(timer, 1000, true);  // 1000µs = 1kHz
 *   timerAlarmEnable(timer);
 * 
 *   void IRAM_ATTR controlLoopISR() {
 *       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 *       vTaskNotifyGiveFromISR(controlTaskHandle, &xHigherPriorityTaskWoken);
 *       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
 *   }
 */
