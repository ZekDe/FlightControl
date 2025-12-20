#include <stdio.h>
#include <string.h>
#include <math.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_chip_info.h"


#include "mpu9250.h"

#include "ekf_quat.h"
#include "quaternion_math.h"
#include "attitude_controller.h"
#include "rate_controller.h"
#include "motor_mixer.h"
#include "control_types.h"

/* Timing utilities */
#include "ton.h"
#include "edge_detection.h"

#include "ble_tuner.h"
#include "helper.h"

static const char *TAG = "MAIN";


/* PWM Configuration */
#define PWM_FREQUENCY           400     /* 400 Hz PWM */
#define PWM_RESOLUTION          LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY            1023

/* Motor GPIO Pins */
#define MOTOR1_GPIO             0      /* Front-Right (M1) */
#define MOTOR2_GPIO             1       /* Rear-Right (M2) */
#define MOTOR3_GPIO             2       /* Rear-Left (M3) */
#define MOTOR4_GPIO             3       /* Front-Left (M4) */


/*******************************************************************************
 * Global Variables - Control Modules
 ******************************************************************************/

static EKF_Handle_t g_ekf = {0};
static AttitudeHandle_t g_attitude = {0};
static RateHandle_t g_rate = {0};
static MixerHandle_t g_mixer = {0};


/*******************************************************************************
 * Global Variables - State
 ******************************************************************************/

/* Sensor data */
static float g_gyro[3] = {0};                   /* rad/s */
static float g_accel[3] = {0};                  /* m/s² */
static float g_accel_normalized[3] = {0, 0, 1}; /* Unit vector for EKF */

/* Attitude */
static Quaternion_t g_quaternion = {1, 0, 0, 0};
static float g_yaw_integrated_rad = 0.0f;

/* Control outputs */
static RateSetpoint_t g_rate_setpoint = {0};
static RateOutput_t g_rate_output = {0};
static MotorOutput_t g_motors = {0};

/* Arm state */
static uint8_t g_is_armed = 0;

/* RC Commands - Bu değişkenleri istediğin gibi değiştir */
static RcCommand_t g_rc = {
    .roll = 0.0f,       /* Roll açı komutu (derece) */
    .pitch = 0.0f,      /* Pitch açı komutu (derece) */
    .yaw = 0.0f,        /* Yaw rate komutu (derece/s) */
    .throttle = 0.0f    /* Throttle (0-100 %) */
};


/*******************************************************************************
 * Global Variables - Timing (TON timers)
 ******************************************************************************/

static ton_t g_ton_imu = {0};
static ton_t g_ton_ekf = {0};
static ton_t g_ton_rate = {0};
static ton_t g_ton_ui = {0};

static edge_detection_t g_ed_imu = {0};
static edge_detection_t g_ed_ekf = {0};
static edge_detection_t g_ed_rate = {0};
static edge_detection_t g_ed_ui = {0};


/* BLE Tuner timing */
#define PERIOD_BLE_TELEMETRY_MS     500 
static ton_t g_ton_ble_telemetry = {0};
static edge_detection_t g_ed_ble_telemetry = {0};




/*******************************************************************************
 * Global Variables - Hardware
 ******************************************************************************/

static i2c_driver_t g_i2c = {0};
static mpu9250_handle_t g_imu = {0};


/*******************************************************************************
 * PWM Motor Functions
 ******************************************************************************/
void cmdArm(void);
void cmdDisarm(void);
void cmdSetThrottle(float throttle);
void cmdSetRoll(float deg);
void cmdSetPitch(float deg);
void cmdSetYawRate(float dps);
void cmdResetYaw(void);
void cmdSetAttitudeGains(float kp, float ki, float kd);
void cmdSetRateGains(float kp, float ki, float kd);

static void onBleCommand(const bleTunerCommand_t *cmd);
static void onBleDisconnect(void);
static void onBleGetGains(bleTunerGains_t *gains);


static int8_t pwmMotorsInit(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    if (ledc_timer_config(&timer_conf) != ESP_OK) {
        ESP_LOGE(TAG, "PWM timer config failed");
        return -1;
    }

    uint8_t motor_pins[4] = {MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO};
    
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = motor_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        
        if (ledc_channel_config(&channel_conf) != ESP_OK) {
            ESP_LOGE(TAG, "PWM channel %d config failed", i);
            return -1;
        }
    }

    ESP_LOGI(TAG, "PWM: GPIO %d,%d,%d,%d @ %dHz", 
             MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO, PWM_FREQUENCY);
    
    return 0;
}


static void pwmMotorsSet(const MotorOutput_t *motors)
{
    for (int i = 0; i < 4; i++) {
        uint32_t duty = (uint32_t)(motors->motor[i] * PWM_MAX_DUTY / 100.0f);
        if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

static void pwmMotorsStop(void)
{
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}


/*******************************************************************************
 * System Tick (ms)
 ******************************************************************************/

static uint32_t getSysTickMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}



/**
 * @brief IMU Read Task - 200 Hz
 */
static void taskImuRead(void)
{
    // Ism330dhcxData_t data;
    // int8_t ret = ism330dhcxRead(&imu, &data);
    mpu9250_scaled_data_t sensor_data;
    if (mpu9250ReadScaled(&g_imu, &sensor_data) != 0) {
        return;
    }
    //todo: compansate eklenecek(helper.h)


        /* Store gyro (rad/s) */
    g_gyro[0] = sensor_data.gyro_x;
    g_gyro[1] = sensor_data.gyro_y;
    g_gyro[2] = sensor_data.gyro_z;

    /* Store accel (m/s²) */
    g_accel[0] = sensor_data.accel_x;
    g_accel[1] = sensor_data.accel_y;
    g_accel[2] = sensor_data.accel_z;

    /* Normalize accel for EKF (unit vector) */
    float norm = sqrtf(g_accel[0]*g_accel[0] + 
                       g_accel[1]*g_accel[1] + 
                       g_accel[2]*g_accel[2]);
    
    if (norm > 0.1f) {
        g_accel_normalized[0] = g_accel[0] / norm;
        g_accel_normalized[1] = g_accel[1] / norm;
        g_accel_normalized[2] = g_accel[2] / norm;
    }

    /* Yaw integration (gyro Z) */
    g_yaw_integrated_rad += g_gyro[2] * (PERIOD_IMU_MS / 1000.0f);
    g_yaw_integrated_rad = normalizeAngle(g_yaw_integrated_rad);
}


/**
 * @brief EKF + Attitude Controller Task - 100 Hz
 */
static void taskEkfAttitude(void)
{
    /* EKF predict + correct */
    ekfPredict(&g_ekf, g_gyro);
    ekfCorrect(&g_ekf, g_accel_normalized);
    
    /* Get quaternion */
    ekfGetQuaternion(&g_ekf, &g_quaternion);

    /* Attitude controller: angle error → rate setpoint */
    AttitudeSetpoint_t att_sp = {
        .roll_deg = g_rc.roll,
        .pitch_deg = g_rc.pitch,
        .yaw_rate_dps = g_rc.yaw   /* Yaw is rate mode (passthrough) */
    };

    attitudeUpdate(&g_attitude, &att_sp, &g_quaternion, &g_rate_setpoint);
}


/**
 * @brief Rate Controller + Mixer Task - 200 Hz
 */
static void taskRateMixer(void)
{
    /* Rate controller: rate error → control output */
    GyroData_t gyro_dps = {
        .roll = CTRL_RAD_TO_DEG(g_gyro[0]),
        .pitch = CTRL_RAD_TO_DEG(g_gyro[1]),
        .yaw = CTRL_RAD_TO_DEG(g_gyro[2])
    };

    rateUpdate(&g_rate, &g_rate_setpoint, &gyro_dps, &g_rate_output);

    /* Mixer: control output → motor % */
    if (g_is_armed) {
        MixerInput_t mixer_in = {
            .throttle = g_rc.throttle,
            .roll = g_rate_output.roll,
            .pitch = g_rate_output.pitch,
            .yaw = g_rate_output.yaw
        };

        mixerUpdate(&g_mixer, &mixer_in, &g_motors);
        pwmMotorsSet(&g_motors);
    } else {
        /* Disarmed - motors off */
        mixerDisarm(&g_mixer, &g_motors);
        pwmMotorsStop();
    }
}

/**
 * @brief UI Telemetry Task - 10 Hz
 */
static void taskUiTelemetry(void)
{
    /* Get Euler angles */
    float roll, pitch;
    EulerAngles_t euler;
    quatToEulerZYX(&g_quaternion, &euler, NULL);
    roll = CTRL_RAD_TO_DEG(euler.roll);
    pitch = CTRL_RAD_TO_DEG(euler.pitch);
    float yaw = radToDeg(g_yaw_integrated_rad);  /* Use integrated yaw */

    ESP_LOGI(TAG, "[%s] R:%6.2f P:%6.2f Y:%7.2f | Thr:%5.1f%% | M:[%4.1f %4.1f %4.1f %4.1f]",
             g_is_armed ? "ARM" : "DIS",
             roll, pitch, yaw,
             g_rc.throttle,
             g_motors.motor[0], g_motors.motor[1],
             g_motors.motor[2], g_motors.motor[3]);

             ESP_LOGI(TAG, "Acc(n): %.2f %.2f %.2f", g_accel_normalized[0],
             g_accel_normalized[1], g_accel_normalized[2]);

             ESP_LOGI(TAG, "Acc: %.2f %.2f %.2f", g_accel[0],
             g_accel[1], g_accel[2]);

             ESP_LOGI(TAG, "Gyro: %.2f %.2f %.2f", g_gyro[0],
             g_gyro[1], g_gyro[2]);
}


/**
 * @brief Sensor noise karakterizasyonu
 * Cihaz hareketsiz dururken 10 saniye veri topla
 */
#define NOISE_SAMPLES 2500 
float gyro_samples[NOISE_SAMPLES][3];
float accel_samples[NOISE_SAMPLES][3];
//todo: adaptif Q/R üzerine çalışılabilir.
void characterizeSensorNoise(void)
{
    ESP_LOGI(TAG, "Noise characterization starting - KEEP STILL!");
    
    // Veri topla
    for (uint16_t i = 0; i < NOISE_SAMPLES; i++) 
    {
        mpu9250_scaled_data_t data;
        mpu9250ReadScaled(&g_imu, &data);
        
        gyro_samples[i][0] = data.gyro_x;
        gyro_samples[i][1] = data.gyro_y;
        gyro_samples[i][2] = data.gyro_z;
        accel_samples[i][0] = data.accel_x;
        accel_samples[i][1] = data.accel_y;
        accel_samples[i][2] = data.accel_z;
        
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    
    // Standart sapma hesapla
    float gyro_std[3] = {0};
    float accel_std[3] = {0};
    
    // Mean
    float gyro_mean[3] = {0};
    float accel_mean[3] = {0};
    for (uint16_t i = 0; i < NOISE_SAMPLES; i++) {
        for (int axis = 0; axis < 3; axis++) {
            gyro_mean[axis] += gyro_samples[i][axis];
            accel_mean[axis] += accel_samples[i][axis];
        }
    }
    for (int axis = 0; axis < 3; axis++) {
        gyro_mean[axis] /= NOISE_SAMPLES;
        accel_mean[axis] /= NOISE_SAMPLES;
    }
    
    // Variance
    for (uint16_t i = 0; i < NOISE_SAMPLES; i++) {
        for (int axis = 0; axis < 3; axis++) {
            float g_diff = gyro_samples[i][axis] - gyro_mean[axis];
            float a_diff = accel_samples[i][axis] - accel_mean[axis];
            gyro_std[axis] += g_diff * g_diff;
            accel_std[axis] += a_diff * a_diff;
        }
    }
    
    // Std dev
    for (int axis = 0; axis < 3; axis++) {
        gyro_std[axis] = sqrtf(gyro_std[axis] / NOISE_SAMPLES);
        accel_std[axis] = sqrtf(accel_std[axis] / NOISE_SAMPLES);
    }
    
    ESP_LOGI(TAG, "=== Sensor Noise Characterization ===");
    ESP_LOGI(TAG, "Gyro Std Dev (rad/s): X=%.6f Y=%.6f Z=%.6f",
             gyro_std[0], gyro_std[1], gyro_std[2]);
    ESP_LOGI(TAG, "Accel Std Dev (m/s²): X=%.4f Y=%.4f Z=%.4f",
             accel_std[0], accel_std[1], accel_std[2]);
    
    // EKF parametrelerine çevir
    float gyro_noise_avg = (gyro_std[0] + gyro_std[1] + gyro_std[2]) / 3.0f;
    float accel_noise_avg = (accel_std[0] + accel_std[1] + accel_std[2]) / 3.0f;
    
    // Process noise (quaternion): gyro noise'ın karesi × dt
    float q_quat_calc = gyro_noise_avg * gyro_noise_avg * (PERIOD_EKF_MS / 1000.0f);
    
    // Measurement noise (accel): variance
    float r_accel_calc = accel_noise_avg * accel_noise_avg;
    
    ESP_LOGI(TAG, "=== Recommended EKF Parameters ===");
    ESP_LOGI(TAG, "q_quat (process noise):      %.6f", q_quat_calc);
    ESP_LOGI(TAG, "r_accel (measurement noise): %.6f", r_accel_calc);
    ESP_LOGI(TAG, "q_bias (start with):         %.6f (tune manually)", q_quat_calc * 10.0f);
}

static int8_t systemInit(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("ESP32 revision: %d\n", chip_info.revision);

    /*=========================================================================
     * I2C Driver
     *=========================================================================*/
    int ret = i2cDriverInit(&g_i2c);
    if (ret != 0) {
        ESP_LOGE(TAG, "I2C init failed: %d", ret);
        return -1;
    }

    /*=========================================================================
     * MPU9250/9255 IMU
     *=========================================================================*/
    mpu9250_config_t imu_config;
    mpu9250GetDefaultConfig(&imu_config);
    
    ret = mpu9250Init(&g_imu, &g_i2c, &imu_config);
    if (ret != 0) {
        ESP_LOGE(TAG, "IMU init failed: %d", ret);
        return -2;
    }

    // Noise karakterizasyonu yap
    //characterizeSensorNoise(); 
    
    /*=========================================================================
     * EKF - Roll/Pitch estimation
     *=========================================================================*/
    ret = ekfInit(&g_ekf, NULL, NULL, DT_EKF);
    if (ret != EKF_OK) {
        ESP_LOGE(TAG, "EKF init failed: %d", ret);
        return -3;
    }

    EKF_ProcessNoise_t q_noise = {.q_quat = 1e-3f, .q_bias = 1e-2f};
    EKF_MeasurementNoise_t r_noise = {.r_accel = 0.00039f};
    ekfSetProcessNoise(&g_ekf, &q_noise);
    ekfSetMeasurementNoise(&g_ekf, &r_noise);

    /*=========================================================================
     * Attitude Controller (Outer Loop) 
     *=========================================================================*/
    ret = attitudeInitDefault(&g_attitude, DT_ATTITUDE);
    if (ret != CTRL_OK) {
        ESP_LOGE(TAG, "Attitude controller init failed: %d", ret);
        return -4;
    }

    /*=========================================================================
     * Rate Controller (Inner Loop) 
     *=========================================================================*/
    ret = rateInitDefault(&g_rate, DT_RATE);
    if (ret != CTRL_OK) {
        ESP_LOGE(TAG, "Rate controller init failed: %d", ret);
        return -5;
    }


    /*=========================================================================
     * Motor Mixer
     *=========================================================================*/
    ret = mixerInitDefault(&g_mixer);
    if (ret != CTRL_OK) {
        ESP_LOGE(TAG, "Mixer init failed: %d", ret);
        return -6;
    }

    /*=========================================================================
     * PWM Motors
     *=========================================================================*/
    ret = pwmMotorsInit();
    if (ret != 0) {
        ESP_LOGE(TAG, "PWM init failed: %d", ret);
        return -7;
    }

    pwmMotorsStop();

    ESP_LOGI(TAG, "=== Init Complete ===");
    ESP_LOGI(TAG, "IMU:%dHz EKF:%dHz Rate:%dHz", 
             1000/PERIOD_IMU_MS, 1000/PERIOD_EKF_MS, 1000/PERIOD_RATE_MS);

   return 0;
}



void app_main(void)
{

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(1);  /* 1ms period */

    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  ESP32-C3 Quadcopter Flight Controller ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");

    if (systemInit() != 0) {
        ESP_LOGE(TAG, "System init failed!");
        return;
    }

    // BLE Tuner Init
    if (bleTunerInit("QUAD-FC") != 0) {
        ESP_LOGE(TAG, "BLE Tuner init failed");
        return;
    }

    
    // Register callbacks
    bleTunerSetCmdCallback(onBleCommand);
    bleTunerSetDisconnectCallback(onBleDisconnect);
    bleTunerSetGetGainsCallback(onBleGetGains);
    
    ESP_LOGI(TAG, "BLE Tuner ready, advertising as: QUAD-FC");

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Starting main loop... (DISARMED)");
    //cmdSetThrottle(4);
    //cmdArm();

    /*=========================================================================
     * Main Loop - TON timer based scheduling
     *=========================================================================*/
    uint32_t systick;
    uint8_t pulse_imu, pulse_ekf, pulse_rate, pulse_ui;

    while (1)
    {
        systick = getSysTickMs();

        /*=====================================================================
         * IMU Read - 200 Hz (every 5ms)
         *=====================================================================*/
        if (TON(&g_ton_imu, 1, systick, PERIOD_IMU_MS)) {
            TON(&g_ton_imu, 0, 0, 0);
        }
        pulse_imu = edgeDetection(&g_ed_imu, g_ton_imu.aux == 0);
        
        if (pulse_imu) {
            taskImuRead();
        }

        /*=====================================================================
         * EKF + Attitude Controller - 100 Hz (every 10ms)
         *=====================================================================*/
        if (TON(&g_ton_ekf, 1, systick, PERIOD_EKF_MS)) {
            TON(&g_ton_ekf, 0, 0, 0);
        }
        pulse_ekf = edgeDetection(&g_ed_ekf, g_ton_ekf.aux == 0);
        
        if (pulse_ekf) {
            taskEkfAttitude();
        }

        /*=====================================================================
         * Rate Controller + Mixer - 200 Hz (every 5ms)
         *=====================================================================*/
        if (TON(&g_ton_rate, 1, systick, PERIOD_RATE_MS)) {
            TON(&g_ton_rate, 0, 0, 0);
        }
        pulse_rate = edgeDetection(&g_ed_rate, g_ton_rate.aux == 0);
        
        if (pulse_rate) {
            taskRateMixer();
        }

        /*=====================================================================
         * UI Telemetry - 10 Hz (every 100ms)
         *=====================================================================*/
        if (TON(&g_ton_ui, 1, systick, PERIOD_UI_MS)) {
            TON(&g_ton_ui, 0, 0, 0);
        }
        pulse_ui = edgeDetection(&g_ed_ui, g_ton_ui.aux == 0);
        
        if (pulse_ui) {
            taskUiTelemetry();
        }

        //=====================================================================
        // BLE Heartbeat Check 
        //=====================================================================
        if (bleTunerCheckHeartbeatTimeout(systick)) {
            ESP_LOGW(TAG, "BLE Heartbeat timeout - Auto DISARM!");
            cmdDisarm();
            bleTunerResetHeartbeat(systick);  // Tekrar timeout olmaması için reset
        }
        
        //=====================================================================
        // BLE Telemetry - 10 Hz (every 100ms)
        //=====================================================================
        if (TON(&g_ton_ble_telemetry, 1, systick, PERIOD_BLE_TELEMETRY_MS)) {
            TON(&g_ton_ble_telemetry, 0, 0, 0);
        }
        uint8_t pulse_ble_tel = edgeDetection(&g_ed_ble_telemetry, g_ton_ble_telemetry.aux == 0);
        
        if (pulse_ble_tel && bleTunerIsConnected() && bleTunerIsTelemetryEnabled())
        {
            // Telemetri verisi hazırla
            bleTunerTelemetry_t tel = {0};
            
            // Quaternion'dan Euler açıları
            GimbalStatus_t gimbal_status;
            EulerAngles_t euler;
            Quaternion_t q;

            quatToEulerZYX(&g_quaternion, &euler, &gimbal_status);
            
            tel.roll_deg = CTRL_RAD_TO_DEG(euler.roll);
            tel.pitch_deg = CTRL_RAD_TO_DEG(euler.pitch);
            tel.yaw_deg = CTRL_RAD_TO_DEG(g_yaw_integrated_rad);
            
            // Motor çıkışları
            tel.motor[0] = g_motors.motor[0];
            tel.motor[1] = g_motors.motor[1];
            tel.motor[2] = g_motors.motor[2];
            tel.motor[3] = g_motors.motor[3];
            
            // Gyro (rad/s -> deg/s)
            tel.gyro_x_dps = CTRL_RAD_TO_DEG(g_gyro[0]);
            tel.gyro_y_dps = CTRL_RAD_TO_DEG(g_gyro[1]);
            tel.gyro_z_dps = CTRL_RAD_TO_DEG(g_gyro[2]);
            
            bleTunerSendTelemetry(&tel);
        }

        /* Small delay to prevent CPU hogging */
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}



/*******************************************************************************
 * Command Functions - Serial/Bluetooth ile çağırılabilir
 ******************************************************************************/

/**
 * @brief Arm - Motorları aktif et
 * @note DİKKAT: Pervaneler dönmeye başlar!
 */
void cmdArm(void)
{
    if (g_rc.throttle < 5.0f) {
        /* Reset controllers */
        attitudeReset(&g_attitude);
        rateReset(&g_rate);
        mixerReset(&g_mixer);
        
        g_is_armed = 1;
        ESP_LOGW(TAG, ">>> ARMED! Motors active! <<<");
    } else {
        ESP_LOGE(TAG, "Cannot arm: throttle must be < 5%%");
    }
}

/**
 * @brief Disarm - Motorları durdur
 */
void cmdDisarm(void)
{
    g_is_armed = 0;
    pwmMotorsStop();
    ESP_LOGI(TAG, ">>> DISARMED <<<");
}

/**
 * @brief Set throttle (0-100%)
 */
void cmdSetThrottle(float throttle)
{
    g_rc.throttle = CTRL_CONSTRAIN(throttle, 0.0f, 100.0f);
    ESP_LOGI(TAG, "Throttle: %.1f%%", g_rc.throttle);
}

/**
 * @brief Set roll command (degrees, -45 to +45)
 */
void cmdSetRoll(float deg)
{
    g_rc.roll = CTRL_CONSTRAIN(deg, -CTRL_MAX_ROLL_ANGLE_DEG, CTRL_MAX_ROLL_ANGLE_DEG);
    ESP_LOGI(TAG, "Roll: %.1f deg", g_rc.roll);
}

/**
 * @brief Set pitch command (degrees, -45 to +45)
 */
void cmdSetPitch(float deg)
{
    g_rc.pitch = CTRL_CONSTRAIN(deg, -CTRL_MAX_PITCH_ANGLE_DEG, CTRL_MAX_PITCH_ANGLE_DEG);
    ESP_LOGI(TAG, "Pitch: %.1f deg", g_rc.pitch);
}

/**
 * @brief Set yaw rate command (degrees/s, -180 to +180)
 */
void cmdSetYawRate(float dps)
{
    g_rc.yaw = CTRL_CONSTRAIN(dps, -CTRL_MAX_YAW_RATE_DPS, CTRL_MAX_YAW_RATE_DPS);
    ESP_LOGI(TAG, "Yaw rate: %.1f deg/s", g_rc.yaw);
}

/**
 * @brief Reset yaw to zero
 */
void cmdResetYaw(void)
{
    g_yaw_integrated_rad = 0.0f;
    ESP_LOGI(TAG, "Yaw reset");
}

/**
 * @brief Set attitude PID gains (roll/pitch)
 */
void cmdSetAttitudeGains(float kp, float ki, float kd)
{
    attitudeSetRollPitchGains(&g_attitude, kp, ki, kd);
    ESP_LOGI(TAG, "Attitude gains: P=%.2f I=%.2f D=%.3f", kp, ki, kd);
}

/**
 * @brief Set rate PID gains (roll/pitch)
 */
void cmdSetRateGains(float kp, float ki, float kd)
{
    rateSetRollPitchGains(&g_rate, kp, ki, kd);
    ESP_LOGI(TAG, "Rate gains: P=%.2f I=%.2f D=%.4f", kp, ki, kd);
}






/*******************************************************************************
 * 3. BLE CALLBACK FONKSİYONLARI s
 ******************************************************************************/


/**
 * @brief BLE komut callback'i
 * @note BLE context'inden çağrılır, hızlı olmalı!
 */
static void onBleCommand(const bleTunerCommand_t *cmd)
{
    switch (cmd->cmd) {
        case BLE_CMD_ARM:
            cmdArm();
            break;
            
        case BLE_CMD_DISARM:
            cmdDisarm();
            break;
            
        case BLE_CMD_THROTTLE:
            cmdSetThrottle(cmd->data.value);
            break;
            
        case BLE_CMD_ROLL:
            cmdSetRoll(cmd->data.value);
            break;
            
        case BLE_CMD_PITCH:
            cmdSetPitch(cmd->data.value);
            break;
            
        case BLE_CMD_YAW:
            cmdSetYawRate(cmd->data.value);
            break;
            
        case BLE_CMD_RATE_RP:
            cmdSetRateGains(cmd->data.gains.kp, 
                           cmd->data.gains.ki, 
                           cmd->data.gains.kd);
            break;
            
        case BLE_CMD_RATE_YAW:
            rateSetYawGains(&g_rate, 
                           cmd->data.gains.kp, 
                           cmd->data.gains.ki, 
                           cmd->data.gains.kd);
            ESP_LOGI(TAG, "Rate Yaw gains: P=%.2f I=%.2f D=%.4f", 
                     cmd->data.gains.kp, cmd->data.gains.ki, cmd->data.gains.kd);
            break;
            
        case BLE_CMD_ATT_RP:
            cmdSetAttitudeGains(cmd->data.gains.kp, 
                               cmd->data.gains.ki, 
                               cmd->data.gains.kd);
            break;
            
        case BLE_CMD_HEARTBEAT:
            bleTunerResetHeartbeat(getSysTickMs());
            break;
            
        case BLE_CMD_TEL_ENABLE:
            ESP_LOGI(TAG, "Telemetry %s", cmd->data.enable ? "enabled" : "disabled");
            break;
            
        case BLE_CMD_GET:
            /* GET komutu ble_tuner.c içinde handle ediliyor */
            break;
            
        default:
            break;
    }
}

/**
 * @brief BLE disconnect callback (auto-disarm)
 */
static void onBleDisconnect(void)
{
    ESP_LOGW(TAG, "BLE Disconnected - Auto DISARM!");
    cmdDisarm();
}

/**
 * @brief BLE get gains callback
 */
static void onBleGetGains(bleTunerGains_t *gains)
{
    if (gains == NULL) return;
    
    /* Rate Roll/Pitch gains */
    gains->rate_rp.kp = g_rate.config.roll_pitch_gains.kp;
    gains->rate_rp.ki = g_rate.config.roll_pitch_gains.ki;
    gains->rate_rp.kd = g_rate.config.roll_pitch_gains.kd;
    
    /* Rate Yaw gains */
    gains->rate_yaw.kp = g_rate.config.yaw_gains.kp;
    gains->rate_yaw.ki = g_rate.config.yaw_gains.ki;
    gains->rate_yaw.kd = g_rate.config.yaw_gains.kd;
    
    /* Attitude Roll/Pitch gains */
    gains->att_rp.kp = g_attitude.config.roll_pitch_gains.kp;
    gains->att_rp.ki = g_attitude.config.roll_pitch_gains.ki;
    gains->att_rp.kd = g_attitude.config.roll_pitch_gains.kd;
}






/*******************************************************************************
 * 6. NRF CONNECT KULLANIM KILAVUZU
 ******************************************************************************/

/*
nRF Connect for Mobile ile Kullanım:

1. Uygulamayı aç
2. "QUAD-FC" cihazını tara ve bağlan
3. "Nordic UART Service" altındaki servisi aç
4. TX characteristic'te "Enable CCCDs" (notify) butonuna bas
5. RX characteristic'e komut yaz:

Örnek Komutlar:
---------------
ARM                     -> Motorları aktif et
DISARM                  -> Motorları durdur
THR=10.0               -> %10 throttle
ROLL=5.0               -> 5 derece roll komutu
PITCH=-3.0             -> -3 derece pitch komutu
YAW=15.0               -> 15 deg/s yaw rate
RATE_RP=0.25,0.30,0.003  -> Rate roll/pitch PID
RATE_YAW=0.40,0.20,0.0   -> Rate yaw PID
ATT_RP=2.5,0.1,0.0       -> Attitude roll/pitch PID
TEL=1                    -> Telemetri aç
TEL=0                    -> Telemetri kapat
GET                      -> Mevcut gain'leri getir
HB                       -> Heartbeat (auto-disarm için)

Telemetri Formatı (TEL=1 ile):
------------------------------
$ATT,12.3,-5.1,45.2*     -> Roll, Pitch, Yaw (derece)
$MOT,15.2,14.8,15.1,14.9* -> Motor 1-4 (%)
$GYR,1.2,-0.8,5.3*       -> Gyro X,Y,Z (deg/s)

Notlar:
-------
- ARM için throttle < %5 olmalı
- HB komutu 2 saniyede bir gönderilmeli (yoksa auto-disarm)
- Bağlantı kopunca otomatik disarm olur
- TEL=1 göndermeden telemetri gelmez
*/
