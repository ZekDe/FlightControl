/**
 * @file sbus_receiver.c
 * @brief SBUS RC Receiver Implementation
 */

#include "sbus_receiver.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "sbus";

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static sbus_config_t s_config;
static sbus_frame_t s_frame;
static uint8_t s_is_initialized = 0;

static sbus_frame_cb_t s_frame_cb = NULL;
static sbus_failsafe_cb_t s_failsafe_cb = NULL;

static TaskHandle_t s_task_handle = NULL;
static QueueHandle_t s_uart_queue = NULL;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static uint32_t getTimeMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Parse SBUS frame
 * @param buf 25-byte SBUS frame
 * @param frame Parsed output
 * @return 1 if valid, 0 if invalid
 */
static uint8_t parseSbusFrame(const uint8_t *buf, sbus_frame_t *frame)
{
    /* Validate header and footer */
    if (buf[0] != SBUS_FRAME_HEADER) {
        return 0;
    }
    
    /* 
     * SBUS uses 11-bit channels packed into bytes 1-22
     * Channel 0:  bits 0-10 of bytes 1-2
     * Channel 1:  bits 11-21 → bits 3-13 of bytes 2-3
     * ... and so on
     * 
     * The packing is:
     * buf[1-22] contains 16 channels × 11 bits = 176 bits = 22 bytes
     */
    
    frame->channels[0]  = ((uint16_t)buf[1]       | (uint16_t)buf[2]  << 8) & 0x07FF;
    frame->channels[1]  = ((uint16_t)buf[2]  >> 3 | (uint16_t)buf[3]  << 5) & 0x07FF;
    frame->channels[2]  = ((uint16_t)buf[3]  >> 6 | (uint16_t)buf[4]  << 2 | (uint16_t)buf[5] << 10) & 0x07FF;
    frame->channels[3]  = ((uint16_t)buf[5]  >> 1 | (uint16_t)buf[6]  << 7) & 0x07FF;
    frame->channels[4]  = ((uint16_t)buf[6]  >> 4 | (uint16_t)buf[7]  << 4) & 0x07FF;
    frame->channels[5]  = ((uint16_t)buf[7]  >> 7 | (uint16_t)buf[8]  << 1 | (uint16_t)buf[9] << 9) & 0x07FF;
    frame->channels[6]  = ((uint16_t)buf[9]  >> 2 | (uint16_t)buf[10] << 6) & 0x07FF;
    frame->channels[7]  = ((uint16_t)buf[10] >> 5 | (uint16_t)buf[11] << 3) & 0x07FF;
    frame->channels[8]  = ((uint16_t)buf[12]      | (uint16_t)buf[13] << 8) & 0x07FF;
    frame->channels[9]  = ((uint16_t)buf[13] >> 3 | (uint16_t)buf[14] << 5) & 0x07FF;
    frame->channels[10] = ((uint16_t)buf[14] >> 6 | (uint16_t)buf[15] << 2 | (uint16_t)buf[16] << 10) & 0x07FF;
    frame->channels[11] = ((uint16_t)buf[16] >> 1 | (uint16_t)buf[17] << 7) & 0x07FF;
    frame->channels[12] = ((uint16_t)buf[17] >> 4 | (uint16_t)buf[18] << 4) & 0x07FF;
    frame->channels[13] = ((uint16_t)buf[18] >> 7 | (uint16_t)buf[19] << 1 | (uint16_t)buf[20] << 9) & 0x07FF;
    frame->channels[14] = ((uint16_t)buf[20] >> 2 | (uint16_t)buf[21] << 6) & 0x07FF;
    frame->channels[15] = ((uint16_t)buf[21] >> 5 | (uint16_t)buf[22] << 3) & 0x07FF;
    
    /* Parse flags byte (byte 23) */
    frame->flags.ch17 = (buf[23] & 0x01) ? 1 : 0;
    frame->flags.ch18 = (buf[23] & 0x02) ? 1 : 0;
    frame->flags.frame_lost = (buf[23] & 0x04) ? 1 : 0;
    frame->flags.failsafe = (buf[23] & 0x08) ? 1 : 0;
    
    frame->timestamp_ms = getTimeMs();
    frame->is_valid = !frame->flags.failsafe && !frame->flags.frame_lost;
    
    return 1;
}

/**
 * @brief SBUS receive task
 */
static void sbusTask(void *arg)
{
    uint8_t buf[SBUS_FRAME_SIZE];
    int index = 0;
    uint8_t byte;
    
    ESP_LOGI(TAG, "SBUS task started");
    
    while (1) {
        /* Read one byte at a time */
        int len = uart_read_bytes(s_config.uart_num, &byte, 1, pdMS_TO_TICKS(10));
        
        if (len > 0) {
            if (index == 0 && byte != SBUS_FRAME_HEADER) {
                /* Wait for header */
                continue;
            }
            
            buf[index++] = byte;
            
            if (index == SBUS_FRAME_SIZE) {
                /* Complete frame received */
                if (parseSbusFrame(buf, &s_frame)) {
                    /* Callback */
                    if (s_frame_cb) {
                        s_frame_cb(&s_frame);
                    }
                    
                    /* Check failsafe */
                    if (s_frame.flags.failsafe && s_failsafe_cb) {
                        s_failsafe_cb();
                    }
                }
                index = 0;
            }
        } else {
            /* Timeout - check for failsafe */
            if (s_frame.is_valid) {
                uint32_t elapsed = getTimeMs() - s_frame.timestamp_ms;
                if (elapsed > SBUS_FAILSAFE_TIMEOUT_MS) {
                    s_frame.is_valid = 0;
                    s_frame.flags.failsafe = 1;
                    ESP_LOGW(TAG, "Signal lost (timeout)");
                    if (s_failsafe_cb) {
                        s_failsafe_cb();
                    }
                }
            }
        }
    }
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void sbusGetDefaultConfig(sbus_config_t *config)
{
    if (config == NULL) return;
    
    config->gpio_rx = 20;           /* Adjust for your board */
    config->uart_num = 1;           /* UART1 */
    config->inverted = 1;           /* SBUS is inverted */
    
    /* Default channel mapping (Betaflight/TAER) */
    config->roll_channel = 0;       /* CH1 */
    config->pitch_channel = 1;      /* CH2 */
    config->throttle_channel = 2;   /* CH3 */
    config->yaw_channel = 3;        /* CH4 */
    config->arm_channel = 4;        /* CH5 */
    config->mode_channel = 5;       /* CH6 */
}

int8_t sbusInit(const sbus_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid config");
        return -1;
    }
    
    if (s_is_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return 0;
    }
    
    s_config = *config;
    memset(&s_frame, 0, sizeof(s_frame));
    
    /* Configure UART */
    uart_config_t uart_config = {
        .baud_rate = SBUS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(config->uart_num, 256, 0, 10, &s_uart_queue, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART install failed: %s", esp_err_to_name(ret));
        return -2;
    }
    
    ret = uart_param_config(config->uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(ret));
        return -3;
    }
    
    /* Set pins */
    ret = uart_set_pin(config->uart_num, UART_PIN_NO_CHANGE, config->gpio_rx, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return -4;
    }
    
    /* Enable signal inversion if needed */
    if (config->inverted) {
        uart_set_line_inverse(config->uart_num, UART_SIGNAL_RXD_INV);
    }
    
    /* Create receiver task */
    BaseType_t task_ret = xTaskCreate(sbusTask, "sbus_rx", 4096, NULL, 10, &s_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Task create failed");
        return -5;
    }
    
    s_is_initialized = 1;
    ESP_LOGI(TAG, "SBUS initialized on GPIO %d (UART%d)", config->gpio_rx, config->uart_num);
    
    return 0;
}

int8_t sbusDeinit(void)
{
    if (!s_is_initialized) return 0;
    
    if (s_task_handle) {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }
    
    uart_driver_delete(s_config.uart_num);
    
    s_is_initialized = 0;
    ESP_LOGI(TAG, "SBUS deinitialized");
    
    return 0;
}

/*******************************************************************************
 * Data Reading
 ******************************************************************************/

int8_t sbusGetFrame(sbus_frame_t *frame)
{
    if (frame == NULL) return -1;
    if (!s_is_initialized) return -2;
    
    *frame = s_frame;
    return 0;
}

int8_t sbusGetNormalized(sbus_normalized_t *data)
{
    if (data == NULL) return -1;
    if (!s_is_initialized) return -2;
    
    sbus_frame_t frame = s_frame;
    
    /* Map channels to normalized values */
    data->roll = sbusNormalizeValue(frame.channels[s_config.roll_channel]);
    data->pitch = sbusNormalizeValue(frame.channels[s_config.pitch_channel]);
    data->yaw = sbusNormalizeValue(frame.channels[s_config.yaw_channel]);
    data->throttle = sbusNormalizeThrottle(frame.channels[s_config.throttle_channel]);
    
    /* Aux channels */
    data->aux1 = sbusNormalizeValue(frame.channels[4]);
    data->aux2 = sbusNormalizeValue(frame.channels[5]);
    data->aux3 = sbusNormalizeValue(frame.channels[6]);
    data->aux4 = sbusNormalizeValue(frame.channels[7]);
    
    /* Switches */
    data->arm_switch = sbusGetSwitchPosition(frame.channels[s_config.arm_channel]);
    data->mode_switch = sbusGetSwitchPosition(frame.channels[s_config.mode_channel]);
    data->alt_hold_switch = sbusGetSwitchPosition(frame.channels[6]);
    
    data->failsafe = frame.flags.failsafe || !frame.is_valid;
    
    return 0;
}

int8_t sbusGetChannel(uint8_t channel, uint16_t *value)
{
    if (value == NULL) return -1;
    if (!s_is_initialized) return -2;
    if (channel >= SBUS_NUM_CHANNELS) return -3;
    
    *value = s_frame.channels[channel];
    return 0;
}

/*******************************************************************************
 * Status
 ******************************************************************************/

uint8_t sbusIsValid(void)
{
    if (!s_is_initialized) return 0;
    
    /* Check timeout */
    uint32_t elapsed = getTimeMs() - s_frame.timestamp_ms;
    if (elapsed > SBUS_FAILSAFE_TIMEOUT_MS) {
        return 0;
    }
    
    return s_frame.is_valid;
}

uint8_t sbusIsFailsafe(void)
{
    if (!s_is_initialized) return 1;
    return s_frame.flags.failsafe || !sbusIsValid();
}

uint32_t sbusGetTimeSinceLastFrame(void)
{
    if (!s_is_initialized) return 0xFFFFFFFF;
    return getTimeMs() - s_frame.timestamp_ms;
}

/*******************************************************************************
 * Callbacks
 ******************************************************************************/

void sbusSetFrameCallback(sbus_frame_cb_t callback)
{
    s_frame_cb = callback;
}

void sbusSetFailsafeCallback(sbus_failsafe_cb_t callback)
{
    s_failsafe_cb = callback;
}

/*******************************************************************************
 * Utilities
 ******************************************************************************/

float sbusNormalizeValue(uint16_t raw)
{
    /* Convert 172-1811 to -1.0 to +1.0 */
    float normalized = ((float)raw - SBUS_MID_VALUE) / ((float)(SBUS_MAX_VALUE - SBUS_MID_VALUE));
    
    /* Clamp to -1 to +1 */
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    
    return normalized;
}

float sbusNormalizeThrottle(uint16_t raw)
{
    /* Convert 172-1811 to 0.0 to 1.0 */
    float normalized = ((float)raw - SBUS_MIN_VALUE) / ((float)(SBUS_MAX_VALUE - SBUS_MIN_VALUE));
    
    /* Clamp to 0 to 1 */
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    
    return normalized;
}

int8_t sbusGetSwitchPosition(uint16_t raw)
{
    /* 3-position switch: low(-1), mid(0), high(+1) */
    if (raw < 500) return -1;
    if (raw > 1500) return 1;
    return 0;
}
