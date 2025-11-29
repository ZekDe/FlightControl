/**
 * @file example_usage.c
 * @brief Example usage of EKF quaternion attitude estimation library
 * @details Demonstrates initialization, configuration, and usage
 *          of the quaternion-based EKF for IMU attitude estimation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ekf_quat.h"

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define IMU_SAMPLE_RATE_HZ      200     /* 200 Hz sample rate */
#define IMU_DT                  (1.0f / IMU_SAMPLE_RATE_HZ)

/* Simulated IMU noise parameters */
#define GYRO_NOISE_STD          0.001f  /* rad/s */
#define ACCEL_NOISE_STD         0.05f   /* g */
#define GYRO_BIAS_DRIFT         1e-6f   /* rad/s per sample */

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Generate random Gaussian noise
 */
static float gaussianNoise(float std_dev)
{
    /* Box-Muller transform */
    static int have_spare = 0;
    static float spare;
    
    if (have_spare) {
        have_spare = 0;
        return std_dev * spare;
    }
    
    have_spare = 1;
    float u, v, s;
    do {
        u = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        v = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);
    
    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return std_dev * u * s;
}

/**
 * @brief Simulate IMU measurements for a given true attitude
 * @param[in]  true_euler   True Euler angles [rad]
 * @param[in]  true_omega   True angular velocity [rad/s]
 * @param[out] gyro         Simulated gyroscope output [rad/s]
 * @param[out] accel        Simulated accelerometer output [g]
 */
static void simulateIMU(const EulerAngles_t *true_euler, 
                        const Vector3_t *true_omega,
                        float *gyro, float *accel)
{
    /* Gyro: true angular velocity + noise */
    gyro[0] = true_omega->x + gaussianNoise(GYRO_NOISE_STD);
    gyro[1] = true_omega->y + gaussianNoise(GYRO_NOISE_STD);
    gyro[2] = true_omega->z + gaussianNoise(GYRO_NOISE_STD);
    
    /* Accel: gravity rotated to body frame + noise */
    /* In NED: gravity = [0, 0, 1] in g units (down is positive) */
    /* Need to rotate from NED to body using euler angles */
    
    float cr = cosf(true_euler->roll);
    float sr = sinf(true_euler->roll);
    float cp = cosf(true_euler->pitch);
    float sp = sinf(true_euler->pitch);
    
    /* Rotation matrix from NED to body (C_nb) for gravity vector */
    /* g_body = C_nb * [0, 0, 1]^T */
    accel[0] = -sp + gaussianNoise(ACCEL_NOISE_STD);
    accel[1] = sr * cp + gaussianNoise(ACCEL_NOISE_STD);
    accel[2] = cr * cp + gaussianNoise(ACCEL_NOISE_STD);
}

/*******************************************************************************
 * Example 1: Basic Initialization and Single Step
 ******************************************************************************/

static void example1_basicUsage(void)
{
    printf("\n=== Example 1: Basic Initialization ===\n\n");
    
    EKF_Handle_t ekf;
    int8_t ret;
    
    /* Initialize EKF with default callbacks and 200Hz sample rate */
    ret = ekfInit(&ekf, NULL, NULL, IMU_DT);
    if (ret != EKF_OK) {
        printf("EKF init failed: %d\n", ret);
        return;
    }
    printf("EKF initialized successfully\n");
    
    /* Configure noise parameters (tune these for your IMU) */
    EKF_ProcessNoise_t q_noise = {
        .q_quat = 1e-6f,    /* Quaternion process noise */
        .q_bias = 1e-10f    /* Gyro bias drift noise */
    };
    ekfSetProcessNoise(&ekf, &q_noise);
    
    EKF_MeasurementNoise_t r_noise = {
        .r_accel = 0.01f    /* Accelerometer noise variance */
    };
    ekfSetMeasurementNoise(&ekf, &r_noise);
    
    /* Simulate stationary IMU (no rotation, level) */
    float gyro[3] = {0.0f, 0.0f, 0.0f};     /* No rotation [rad/s] */
    float accel[3] = {0.0f, 0.0f, 1.0f};    /* Gravity in g [g] */
    
    /* Single predict-correct cycle */
    ret = ekfPredict(&ekf, gyro);
    if (ret != EKF_OK) {
        printf("Predict failed: %d\n", ret);
        return;
    }
    
    ret = ekfCorrect(&ekf, accel);
    if (ret != EKF_OK) {
        printf("Correct failed: %d\n", ret);
        return;
    }
    
    /* Get estimated attitude */
    float roll, pitch, yaw;
    ekfGetEulerDegrees(&ekf, &roll, &pitch, &yaw);
    
    printf("Estimated attitude (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
           roll, pitch, yaw);
    
    /* Get quaternion */
    Quaternion_t q;
    ekfGetQuaternion(&ekf, &q);
    printf("Quaternion: [%.4f, %.4f, %.4f, %.4f]\n", q.q0, q.q1, q.q2, q.q3);
    
    /* Get gyro bias estimate */
    float bias[3];
    ekfGetGyroBias(&ekf, bias);
    printf("Gyro bias (rad/s): [%.6f, %.6f, %.6f]\n", bias[0], bias[1], bias[2]);
}

/*******************************************************************************
 * Example 2: Continuous Operation with Simulated Data
 ******************************************************************************/

static void example2_continuousOperation(void)
{
    printf("\n=== Example 2: Continuous Operation ===\n\n");
    
    EKF_Handle_t ekf;
    int8_t ret;
    
    /* Initialize */
    ret = ekfInit(&ekf, NULL, NULL, IMU_DT);
    if (ret != EKF_OK) return;
    
    /* Configure noise */
    EKF_ProcessNoise_t q_noise = {.q_quat = 1e-6f, .q_bias = 1e-10f};
    EKF_MeasurementNoise_t r_noise = {.r_accel = 0.01f};
    ekfSetProcessNoise(&ekf, &q_noise);
    ekfSetMeasurementNoise(&ekf, &r_noise);
    
    /* Simulate tilted stationary pose */
    EulerAngles_t true_euler = {
        .roll = degToRad(10.0f),     /* 10 deg roll */
        .pitch = degToRad(-5.0f),    /* -5 deg pitch */
        .yaw = degToRad(0.0f)        /* 0 deg yaw (no magnetometer) */
    };
    
    Vector3_t true_omega = {0.0f, 0.0f, 0.0f};  /* Stationary */
    
    printf("True attitude: Roll=10.0 deg, Pitch=-5.0 deg, Yaw=0.0 deg\n\n");
    
    /* Run for 2 seconds (400 samples at 200Hz) */
    int num_samples = 400;
    float gyro[3], accel[3];
    
    for (int i = 0; i < num_samples; i++) {
        /* Generate simulated IMU data */
        simulateIMU(&true_euler, &true_omega, gyro, accel);
        
        /* EKF predict and correct */
        ekfPredict(&ekf, gyro);
        ekfCorrect(&ekf, accel);
        
        /* Print every 100 samples */
        if ((i + 1) % 100 == 0) {
            float roll, pitch, yaw;
            ekfGetEulerDegrees(&ekf, &roll, &pitch, &yaw);
            
            printf("Sample %3d: Roll=%6.2f, Pitch=%6.2f, Yaw=%6.2f (deg)\n",
                   i + 1, roll, pitch, yaw);
        }
    }
    
    /* Final estimate */
    printf("\nFinal estimate after %d samples:\n", num_samples);
    
    float roll, pitch, yaw;
    ekfGetEulerDegrees(&ekf, &roll, &pitch, &yaw);
    printf("  Roll: %.2f deg (true: 10.0)\n", roll);
    printf("  Pitch: %.2f deg (true: -5.0)\n", pitch);
    printf("  Yaw: %.2f deg (true: 0.0)\n", yaw);
    
    /* Error */
    printf("\nErrors:\n");
    printf("  Roll error: %.2f deg\n", roll - 10.0f);
    printf("  Pitch error: %.2f deg\n", pitch - (-5.0f));
}

/*******************************************************************************
 * Example 3: Dynamic Motion Simulation
 ******************************************************************************/

static void example3_dynamicMotion(void)
{
    printf("\n=== Example 3: Dynamic Motion ===\n\n");
    
    EKF_Handle_t ekf;
    int8_t ret;
    
    /* Initialize */
    ret = ekfInit(&ekf, NULL, NULL, IMU_DT);
    if (ret != EKF_OK) return;
    
    /* Configure noise */
    EKF_ProcessNoise_t q_noise = {.q_quat = 1e-5f, .q_bias = 1e-10f};
    EKF_MeasurementNoise_t r_noise = {.r_accel = 0.1f};
    ekfSetProcessNoise(&ekf, &q_noise);
    ekfSetMeasurementNoise(&ekf, &r_noise);
    
    printf("Simulating slow roll rotation (10 deg/s)...\n\n");
    
    /* Simulate slow roll rotation */
    float true_roll = 0.0f;     /* Start at 0 */
    float roll_rate = degToRad(10.0f);  /* 10 deg/s */
    
    float gyro[3], accel[3];
    int num_samples = 600;  /* 3 seconds */
    
    for (int i = 0; i < num_samples; i++) {
        /* Update true roll */
        true_roll += roll_rate * IMU_DT;
        
        /* True attitude */
        EulerAngles_t true_euler = {
            .roll = true_roll,
            .pitch = 0.0f,
            .yaw = 0.0f
        };
        
        /* True angular velocity (body frame) */
        Vector3_t true_omega = {roll_rate, 0.0f, 0.0f};
        
        /* Generate simulated IMU data */
        simulateIMU(&true_euler, &true_omega, gyro, accel);
        
        /* EKF predict and correct */
        ekfPredict(&ekf, gyro);
        ekfCorrect(&ekf, accel);
        
        /* Print every 100 samples */
        if ((i + 1) % 100 == 0) {
            float roll, pitch, yaw;
            ekfGetEulerDegrees(&ekf, &roll, &pitch, &yaw);
            
            float true_roll_deg = radToDeg(true_roll);
            
            printf("t=%.1fs: True Roll=%.1f, Est Roll=%.1f, Error=%.2f deg\n",
                   (i + 1) * IMU_DT,
                   true_roll_deg,
                   roll,
                   roll - true_roll_deg);
        }
    }
}

/*******************************************************************************
 * Example 4: Covariance and Uncertainty Monitoring
 ******************************************************************************/

static void example4_covarianceMonitoring(void)
{
    printf("\n=== Example 4: Covariance Monitoring ===\n\n");
    
    EKF_Handle_t ekf;
    int8_t ret;
    
    /* Initialize */
    ret = ekfInit(&ekf, NULL, NULL, IMU_DT);
    if (ret != EKF_OK) return;
    
    /* Set high initial uncertainty */
    ekfSetInitialCovariance(&ekf, 0.1f, 0.001f);
    
    printf("Initial covariance (high uncertainty):\n");
    float p_diag[7];
    ekfGetCovarianceDiag(&ekf, p_diag);
    printf("  P_quat: %.6f, %.6f, %.6f, %.6f\n", p_diag[0], p_diag[1], p_diag[2], p_diag[3]);
    printf("  P_bias: %.6f, %.6f, %.6f\n", p_diag[4], p_diag[5], p_diag[6]);
    
    /* Configure noise */
    EKF_ProcessNoise_t q_noise = {.q_quat = 1e-6f, .q_bias = 1e-10f};
    EKF_MeasurementNoise_t r_noise = {.r_accel = 0.01f};
    ekfSetProcessNoise(&ekf, &q_noise);
    ekfSetMeasurementNoise(&ekf, &r_noise);
    
    /* Run with stationary IMU data */
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 1.0f};
    
    printf("\nCovariance convergence over time:\n");
    
    for (int i = 0; i < 500; i++) {
        ekfPredict(&ekf, gyro);
        ekfCorrect(&ekf, accel);
        
        if ((i + 1) % 100 == 0) {
            ekfGetCovarianceDiag(&ekf, p_diag);
            printf("  After %3d samples: P_q0=%.8f, P_bx=%.8f\n",
                   i + 1, p_diag[0], p_diag[4]);
        }
    }
    
    printf("\nFinal covariance (low uncertainty):\n");
    ekfGetCovarianceDiag(&ekf, p_diag);
    printf("  P_quat: %.8f, %.8f, %.8f, %.8f\n", p_diag[0], p_diag[1], p_diag[2], p_diag[3]);
    printf("  P_bias: %.8f, %.8f, %.8f\n", p_diag[4], p_diag[5], p_diag[6]);
}

/*******************************************************************************
 * Example 5: DCM and Quaternion Access
 ******************************************************************************/

static void example5_dcmQuaternion(void)
{
    printf("\n=== Example 5: DCM and Quaternion Access ===\n\n");
    
    EKF_Handle_t ekf;
    int8_t ret;
    
    /* Initialize */
    ret = ekfInit(&ekf, NULL, NULL, IMU_DT);
    if (ret != EKF_OK) return;
    
    /* Simulate 30 degree roll tilt */
    EulerAngles_t true_euler = {
        .roll = degToRad(30.0f),
        .pitch = degToRad(0.0f),
        .yaw = degToRad(0.0f)
    };
    Vector3_t true_omega = {0.0f, 0.0f, 0.0f};
    
    float gyro[3], accel[3];
    
    /* Run 200 samples to converge */
    for (int i = 0; i < 200; i++) {
        simulateIMU(&true_euler, &true_omega, gyro, accel);
        ekfPredict(&ekf, gyro);
        ekfCorrect(&ekf, accel);
    }
    
    /* Get quaternion */
    Quaternion_t q;
    ekfGetQuaternion(&ekf, &q);
    printf("Quaternion: [%.6f, %.6f, %.6f, %.6f]\n", q.q0, q.q1, q.q2, q.q3);
    
    /* Get DCM */
    DCM_t dcm;
    ekfGetDCM(&ekf, &dcm);
    printf("\nDCM (body to nav):\n");
    printf("  [%.6f  %.6f  %.6f]\n", dcm.m[0][0], dcm.m[0][1], dcm.m[0][2]);
    printf("  [%.6f  %.6f  %.6f]\n", dcm.m[1][0], dcm.m[1][1], dcm.m[1][2]);
    printf("  [%.6f  %.6f  %.6f]\n", dcm.m[2][0], dcm.m[2][1], dcm.m[2][2]);
    
    /* Verify: Convert DCM back to Euler */
    EulerAngles_t euler_from_dcm;
    dcmToEulerZYX(&dcm, &euler_from_dcm, NULL);
    printf("\nEuler from DCM: Roll=%.2f, Pitch=%.2f, Yaw=%.2f deg\n",
           radToDeg(euler_from_dcm.roll),
           radToDeg(euler_from_dcm.pitch),
           radToDeg(euler_from_dcm.yaw));
    
    /* Get Euler from EKF directly */
    EulerAngles_t euler;
    ekfGetEulerZYX(&ekf, &euler);
    printf("Euler from EKF: Roll=%.2f, Pitch=%.2f, Yaw=%.2f deg\n",
           radToDeg(euler.roll),
           radToDeg(euler.pitch),
           radToDeg(euler.yaw));
}

/*******************************************************************************
 * Example 6: Gimbal Lock Detection
 ******************************************************************************/

static void example6_gimbalLock(void)
{
    printf("\n=== Example 6: Gimbal Lock Detection ===\n\n");
    
    EKF_Handle_t ekf;
    int8_t ret;
    
    /* Initialize */
    ret = ekfInit(&ekf, NULL, NULL, IMU_DT);
    if (ret != EKF_OK) return;
    
    printf("Simulating pitch approaching 90 degrees...\n\n");
    
    /* Simulate increasing pitch */
    float true_pitch = 0.0f;
    float pitch_rate = degToRad(30.0f);  /* 30 deg/s */
    
    float gyro[3], accel[3];
    
    for (int i = 0; i < 180; i++) {  /* 0.9 seconds, reaching ~27 deg */
        true_pitch += pitch_rate * IMU_DT;
        
        EulerAngles_t true_euler = {
            .roll = 0.0f,
            .pitch = true_pitch,
            .yaw = 0.0f
        };
        
        Vector3_t true_omega = {0.0f, pitch_rate, 0.0f};
        simulateIMU(&true_euler, &true_omega, gyro, accel);
        
        ekfPredict(&ekf, gyro);
        ekfCorrect(&ekf, accel);
    }
    
    /* Continue to near 90 degrees (need more aggressive simulation) */
    printf("Forcing near-gimbal-lock state...\n");
    
    /* Directly set a high pitch state for demonstration */
    Quaternion_t q_high_pitch;
    EulerAngles_t high_pitch_euler = {0.0f, degToRad(88.0f), 0.0f};
    eulerToQuatZYX(&high_pitch_euler, &q_high_pitch);
    
    /* Manually set EKF state (not normally recommended) */
    ekf.x[0] = q_high_pitch.q0;
    ekf.x[1] = q_high_pitch.q1;
    ekf.x[2] = q_high_pitch.q2;
    ekf.x[3] = q_high_pitch.q3;
    
    /* Get Euler angles - should trigger gimbal lock warning */
    EulerAngles_t euler;
    ret = ekfGetEulerZYX(&ekf, &euler);
    
    GimbalStatus_t status;
    ekfGetGimbalStatus(&ekf, &status);
    
    printf("\nPitch = %.1f degrees\n", radToDeg(euler.pitch));
    printf("Gimbal lock: %s\n", status.is_gimbal_lock ? "YES" : "NO");
    printf("Pitch clamped: %s\n", status.pitch_clamped ? "YES" : "NO");
    
    /* Try even higher pitch */
    high_pitch_euler.pitch = degToRad(91.0f);  /* Beyond limit */
    eulerToQuatZYX(&high_pitch_euler, &q_high_pitch);
    
    ekf.x[0] = q_high_pitch.q0;
    ekf.x[1] = q_high_pitch.q1;
    ekf.x[2] = q_high_pitch.q2;
    ekf.x[3] = q_high_pitch.q3;
    
    ekfGetEulerZYX(&ekf, &euler);
    ekfGetGimbalStatus(&ekf, &status);
    
    printf("\nRequested pitch = 91 degrees (clamped to limit)\n");
    printf("Output pitch = %.1f degrees\n", radToDeg(euler.pitch));
    printf("Gimbal lock: %s\n", status.is_gimbal_lock ? "YES" : "NO");
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int main(void)
{
    printf("========================================\n");
    printf("EKF Quaternion Attitude Estimation Demo\n");
    printf("========================================\n");
    
    /* Seed random number generator */
    srand(42);
    
    /* Run examples */
    example1_basicUsage();
    example2_continuousOperation();
    example3_dynamicMotion();
    example4_covarianceMonitoring();
    example5_dcmQuaternion();
    example6_gimbalLock();
    
    printf("\n========================================\n");
    printf("Demo completed!\n");
    printf("========================================\n");
    
    return 0;
}
