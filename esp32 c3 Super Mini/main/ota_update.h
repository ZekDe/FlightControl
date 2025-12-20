/**
 * @file ota_update.h
 * @brief OTA (Over-The-Air) Firmware Update for ESP32
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * OTA Status
 ******************************************************************************/
typedef enum {
    OTA_STATUS_IDLE = 0,
    OTA_STATUS_DOWNLOADING,
    OTA_STATUS_VERIFYING,
    OTA_STATUS_COMPLETE,
    OTA_STATUS_ERROR,
} ota_status_t;

/*******************************************************************************
 * OTA Progress Callback
 ******************************************************************************/
typedef void (*ota_progress_cb_t)(uint32_t downloaded, uint32_t total);
typedef void (*ota_complete_cb_t)(int8_t success);

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/**
 * @brief Initialize OTA update system
 * @return 0 on success, negative on error
 */
int8_t otaInit(void);

/**
 * @brief Start HTTP server for OTA upload
 * @return 0 on success, negative on error
 * 
 * @note Starts a web server on port 80 with upload page at /update
 */
int8_t otaStartServer(void);

/**
 * @brief Stop OTA server
 * @return 0 on success, negative on error
 */
int8_t otaStopServer(void);

/**
 * @brief Start OTA from URL
 * @param url HTTP(S) URL to firmware binary
 * @return 0 on success, negative on error
 */
int8_t otaStartFromUrl(const char *url);

/**
 * @brief Get current OTA status
 * @return OTA status
 */
ota_status_t otaGetStatus(void);

/**
 * @brief Get OTA progress
 * @param downloaded Pointer to store downloaded bytes
 * @param total Pointer to store total bytes
 */
void otaGetProgress(uint32_t *downloaded, uint32_t *total);

/**
 * @brief Set progress callback
 * @param callback Function to call on progress
 */
void otaSetProgressCallback(ota_progress_cb_t callback);

/**
 * @brief Set completion callback
 * @param callback Function to call on completion
 */
void otaSetCompleteCallback(ota_complete_cb_t callback);

/**
 * @brief Rollback to previous firmware
 * @return 0 on success, negative on error
 * 
 * @note Requires app rollback support in partition table
 */
int8_t otaRollback(void);

/**
 * @brief Mark current firmware as valid
 * @return 0 on success, negative on error
 * 
 * @note Should be called after successful boot
 */
int8_t otaMarkValid(void);

#ifdef __cplusplus
}
#endif

#endif /* OTA_UPDATE_H */
