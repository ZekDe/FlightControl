#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include "mpu9250.h"

static const char *TAG = "MAIN";

static i2c_driver_t g_i2c_driver = {0};
static mpu9250_handle_t g_mpu9250 = {0};

void app_main(void)
{
    ESP_LOGI(TAG, "Flight Controller - MPU9250 Test");

    int8_t ret;

    /* Initialize I2C driver */
    ret = i2cDriverInit(&g_i2c_driver);
    if (ret != 0) {
        ESP_LOGE(TAG, "I2C init failed: %d", ret);
        return;
    }

    /* Get default MPU9250 configuration */
    mpu9250_config_t mpu_config;
    mpu9250GetDefaultConfig(&mpu_config);

    /* Initialize MPU9250 */
    ret = mpu9250Init(&g_mpu9250, &g_i2c_driver, &mpu_config);
    if (ret != 0) {
        ESP_LOGE(TAG, "MPU9250 init failed: %d", ret);
        i2cDriverDeinit(&g_i2c_driver);
        return;
    }

    ESP_LOGI(TAG, "System initialized, starting sensor read loop...");

    /* Main loop - 1kHz polling */
    mpu9250_scaled_data_t sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(1);  /* 1ms period */

    while (1) {
        /* Read sensor data */
        ret = mpu9250ReadScaled(&g_mpu9250, &sensor_data);
        if (ret == 0) {
            /* Print every 500ms for debugging */
            static uint32_t counter = 0;
            if (++counter >= 500) {
                counter = 0;
                ESP_LOGI(TAG, "Accel: X=%.2f Y=%.2f Z=%.2f m/s²",
                         sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
                ESP_LOGI(TAG, "Gyro:  X=%.2f Y=%.2f Z=%.2f rad/s",
                         sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
                ESP_LOGI(TAG, "Temp:  %.1f °C", sensor_data.temp);
            }
        }

        /* Wait for next period */
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
