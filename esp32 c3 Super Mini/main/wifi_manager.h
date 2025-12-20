/**
 * @file wifi_manager.h
 * @brief WiFi Manager for ESP32 Flight Controller
 * @details Supports AP mode (configuration) and STA mode (home network)
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/
#define WIFI_SSID_MAX_LEN       32
#define WIFI_PASS_MAX_LEN       64
#define WIFI_AP_SSID_DEFAULT    "QUAD-FC-SETUP"
#define WIFI_AP_PASS_DEFAULT    "12345678"
#define WIFI_AP_CHANNEL         6
#define WIFI_AP_MAX_CONN        4

/*******************************************************************************
 * WiFi Mode
 ******************************************************************************/
typedef enum {
    WIFI_MODE_DISABLED = 0,
    WIFI_MODE_AP,           /* Access Point mode (configuration) */
    WIFI_MODE_STA,          /* Station mode (connect to network) */
    WIFI_MODE_APSTA,        /* Both AP and STA mode */
} wifi_manager_mode_t;

/*******************************************************************************
 * Connection Status
 ******************************************************************************/
typedef enum {
    WIFI_STATUS_DISCONNECTED = 0,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_CONNECTION_FAILED,
    WIFI_STATUS_AP_STARTED,
} wifi_manager_status_t;

/*******************************************************************************
 * WiFi Credentials
 ******************************************************************************/
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];
    char password[WIFI_PASS_MAX_LEN];
} wifi_credentials_t;

/*******************************************************************************
 * Connection Info
 ******************************************************************************/
typedef struct {
    wifi_manager_mode_t mode;
    wifi_manager_status_t status;
    char ip_addr[16];       /* e.g., "192.168.4.1" */
    char gateway[16];
    char netmask[16];
    int8_t rssi;            /* Signal strength (STA mode) */
} wifi_connection_info_t;

/*******************************************************************************
 * Callback Types
 ******************************************************************************/
typedef void (*wifi_connected_cb_t)(void);
typedef void (*wifi_disconnected_cb_t)(void);

/*******************************************************************************
 * Function Prototypes - Initialization
 ******************************************************************************/

/**
 * @brief Initialize WiFi manager
 * @return 0 on success, negative on error
 */
int8_t wifiManagerInit(void);

/**
 * @brief Deinitialize WiFi manager
 * @return 0 on success, negative on error
 */
int8_t wifiManagerDeinit(void);

/*******************************************************************************
 * Function Prototypes - AP Mode
 ******************************************************************************/

/**
 * @brief Start Access Point mode
 * @param ssid AP network name (NULL for default)
 * @param password AP password (NULL for default, min 8 chars)
 * @return 0 on success, negative on error
 */
int8_t wifiManagerStartAP(const char *ssid, const char *password);

/**
 * @brief Stop Access Point mode
 * @return 0 on success, negative on error
 */
int8_t wifiManagerStopAP(void);

/*******************************************************************************
 * Function Prototypes - STA Mode
 ******************************************************************************/

/**
 * @brief Connect to WiFi network
 * @param ssid Network SSID
 * @param password Network password
 * @return 0 on success, negative on error
 */
int8_t wifiManagerConnect(const char *ssid, const char *password);

/**
 * @brief Disconnect from current network
 * @return 0 on success, negative on error
 */
int8_t wifiManagerDisconnect(void);

/*******************************************************************************
 * Function Prototypes - Credentials Storage (NVS)
 ******************************************************************************/

/**
 * @brief Save WiFi credentials to NVS
 * @param creds Pointer to credentials
 * @return 0 on success, negative on error
 */
int8_t wifiManagerSaveCredentials(const wifi_credentials_t *creds);

/**
 * @brief Load WiFi credentials from NVS
 * @param creds Pointer to store credentials
 * @return 0 on success, negative on error (no saved credentials)
 */
int8_t wifiManagerLoadCredentials(wifi_credentials_t *creds);

/**
 * @brief Clear saved credentials from NVS
 * @return 0 on success, negative on error
 */
int8_t wifiManagerClearCredentials(void);

/*******************************************************************************
 * Function Prototypes - Status
 ******************************************************************************/

/**
 * @brief Get current connection info
 * @param info Pointer to store connection info
 * @return 0 on success, negative on error
 */
int8_t wifiManagerGetInfo(wifi_connection_info_t *info);

/**
 * @brief Check if connected
 * @return 1 if connected, 0 otherwise
 */
uint8_t wifiManagerIsConnected(void);

/*******************************************************************************
 * Function Prototypes - Callbacks
 ******************************************************************************/

/**
 * @brief Set connection callback
 * @param callback Function to call on connection
 */
void wifiManagerSetConnectedCallback(wifi_connected_cb_t callback);

/**
 * @brief Set disconnection callback
 * @param callback Function to call on disconnection
 */
void wifiManagerSetDisconnectedCallback(wifi_disconnected_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */
