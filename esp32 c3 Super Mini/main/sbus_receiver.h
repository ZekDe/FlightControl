/**
 * @file sbus_receiver.h
 * @brief SBUS RC Receiver Driver
 * @details Futaba SBUS protocol (inverted UART, 100kbaud)
 * 
 * @note SBUS frame: 25 bytes, 8E2 format
 * @note Frame period: ~7ms (analog) or ~9ms (highspeed)
 */

#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/
#define SBUS_NUM_CHANNELS       16      /* Standard SBUS channels */
#define SBUS_FRAME_SIZE         25      /* Frame size in bytes */
#define SBUS_BAUD_RATE          100000  /* 100kbaud */
#define SBUS_FRAME_HEADER       0x0F    /* Start byte */
#define SBUS_FRAME_FOOTER       0x00    /* End byte */

/* Channel value range */
#define SBUS_MIN_VALUE          172     /* Corresponds to ~1000µs */
#define SBUS_MID_VALUE          992     /* Corresponds to ~1500µs */
#define SBUS_MAX_VALUE          1811    /* Corresponds to ~2000µs */

/* Failsafe timeout */
#define SBUS_FAILSAFE_TIMEOUT_MS    500

/*******************************************************************************
 * SBUS Status Flags
 ******************************************************************************/
typedef struct {
    uint8_t frame_lost : 1;     /* Frame lost indicator */
    uint8_t failsafe : 1;       /* Failsafe activated */
    uint8_t ch17 : 1;           /* Digital channel 17 */
    uint8_t ch18 : 1;           /* Digital channel 18 */
} sbus_flags_t;

/*******************************************************************************
 * SBUS Frame Data
 ******************************************************************************/
typedef struct {
    uint16_t channels[SBUS_NUM_CHANNELS];   /* Channel values (172-1811) */
    sbus_flags_t flags;
    uint32_t timestamp_ms;                   /* Last update time */
    uint8_t is_valid;                        /* Data valid flag */
} sbus_frame_t;

/*******************************************************************************
 * Normalized RC Data (for flight controller)
 ******************************************************************************/
typedef struct {
    float roll;         /* -1.0 to +1.0 */
    float pitch;        /* -1.0 to +1.0 */
    float yaw;          /* -1.0 to +1.0 */
    float throttle;     /* 0.0 to 1.0 */
    
    /* Aux channels */
    float aux1;         /* CH5: -1.0 to +1.0 */
    float aux2;         /* CH6: -1.0 to +1.0 */
    float aux3;         /* CH7: -1.0 to +1.0 */
    float aux4;         /* CH8: -1.0 to +1.0 */
    
    /* Switches (3-position) */
    int8_t arm_switch;          /* -1, 0, +1 */
    int8_t mode_switch;         /* -1, 0, +1 */
    int8_t alt_hold_switch;     /* -1, 0, +1 */
    
    uint8_t failsafe;           /* 1 = failsafe active */
} sbus_normalized_t;

/*******************************************************************************
 * Configuration Structure
 ******************************************************************************/
typedef struct {
    int gpio_rx;                /* UART RX GPIO pin */
    uint8_t uart_num;           /* UART port number (0, 1, or 2) */
    uint8_t inverted;           /* 1 = signal is inverted (most SBUS) */
    
    /* Channel mapping */
    uint8_t roll_channel;       /* Default: 0 (CH1) */
    uint8_t pitch_channel;      /* Default: 1 (CH2) */
    uint8_t throttle_channel;   /* Default: 2 (CH3) */
    uint8_t yaw_channel;        /* Default: 3 (CH4) */
    uint8_t arm_channel;        /* Default: 4 (CH5) */
    uint8_t mode_channel;       /* Default: 5 (CH6) */
} sbus_config_t;

/*******************************************************************************
 * Callback Types
 ******************************************************************************/
typedef void (*sbus_frame_cb_t)(const sbus_frame_t *frame);
typedef void (*sbus_failsafe_cb_t)(void);

/*******************************************************************************
 * Function Prototypes - Initialization
 ******************************************************************************/

/**
 * @brief Get default SBUS configuration
 * @param config Pointer to store default config
 */
void sbusGetDefaultConfig(sbus_config_t *config);

/**
 * @brief Initialize SBUS receiver
 * @param config Pointer to configuration
 * @return 0 on success, negative on error
 */
int8_t sbusInit(const sbus_config_t *config);

/**
 * @brief Deinitialize SBUS receiver
 * @return 0 on success, negative on error
 */
int8_t sbusDeinit(void);

/*******************************************************************************
 * Function Prototypes - Data Reading
 ******************************************************************************/

/**
 * @brief Get latest SBUS frame data
 * @param frame Pointer to store frame data
 * @return 0 on success, negative on error
 */
int8_t sbusGetFrame(sbus_frame_t *frame);

/**
 * @brief Get normalized RC data
 * @param data Pointer to store normalized data
 * @return 0 on success, negative on error
 */
int8_t sbusGetNormalized(sbus_normalized_t *data);

/**
 * @brief Get single channel value (raw)
 * @param channel Channel number (0-15)
 * @param value Pointer to store value
 * @return 0 on success, negative on error
 */
int8_t sbusGetChannel(uint8_t channel, uint16_t *value);

/*******************************************************************************
 * Function Prototypes - Status
 ******************************************************************************/

/**
 * @brief Check if signal is valid
 * @return 1 if valid, 0 if failsafe/lost
 */
uint8_t sbusIsValid(void);

/**
 * @brief Check if failsafe is active
 * @return 1 if failsafe, 0 otherwise
 */
uint8_t sbusIsFailsafe(void);

/**
 * @brief Get time since last valid frame
 * @return Time in milliseconds
 */
uint32_t sbusGetTimeSinceLastFrame(void);

/*******************************************************************************
 * Function Prototypes - Callbacks
 ******************************************************************************/

/**
 * @brief Set frame received callback
 * @param callback Function to call on new frame
 */
void sbusSetFrameCallback(sbus_frame_cb_t callback);

/**
 * @brief Set failsafe callback
 * @param callback Function to call on failsafe
 */
void sbusSetFailsafeCallback(sbus_failsafe_cb_t callback);

/*******************************************************************************
 * Function Prototypes - Utilities
 ******************************************************************************/

/**
 * @brief Convert raw SBUS value to normalized (-1 to +1)
 * @param raw Raw SBUS value (172-1811)
 * @return Normalized value (-1.0 to +1.0)
 */
float sbusNormalizeValue(uint16_t raw);

/**
 * @brief Convert raw SBUS value to throttle (0 to 1)
 * @param raw Raw SBUS value (172-1811)
 * @return Throttle value (0.0 to 1.0)
 */
float sbusNormalizeThrottle(uint16_t raw);

/**
 * @brief Convert raw SBUS value to switch position
 * @param raw Raw SBUS value
 * @return -1, 0, or +1 (3-position switch)
 */
int8_t sbusGetSwitchPosition(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_RECEIVER_H */
