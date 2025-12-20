/**
 * @file wifi_manager.c
 * @brief WiFi Manager Implementation
 */

#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "wifi_mgr";

/*******************************************************************************
 * NVS Keys
 ******************************************************************************/
#define NVS_NAMESPACE       "wifi_creds"
#define NVS_KEY_SSID        "ssid"
#define NVS_KEY_PASS        "password"

/*******************************************************************************
 * Event Group Bits
 ******************************************************************************/
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;

static wifi_manager_mode_t s_current_mode = WIFI_MODE_DISABLED;
static wifi_manager_status_t s_current_status = WIFI_STATUS_DISCONNECTED;
static char s_ip_addr[16] = {0};

static wifi_connected_cb_t s_connected_cb = NULL;
static wifi_disconnected_cb_t s_disconnected_cb = NULL;

static uint8_t s_is_initialized = 0;
static int s_retry_count = 0;
#define MAX_RETRY   5

/*******************************************************************************
 * Event Handler
 ******************************************************************************/
static void wifiEventHandler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA started, connecting...");
                s_current_status = WIFI_STATUS_CONNECTING;
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_count < MAX_RETRY) {
                    esp_wifi_connect();
                    s_retry_count++;
                    ESP_LOGI(TAG, "Retry connecting (%d/%d)", s_retry_count, MAX_RETRY);
                } else {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    s_current_status = WIFI_STATUS_CONNECTION_FAILED;
                    ESP_LOGW(TAG, "Connection failed after %d retries", MAX_RETRY);
                    if (s_disconnected_cb) s_disconnected_cb();
                }
                break;
                
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "AP started");
                s_current_status = WIFI_STATUS_AP_STARTED;
                break;
                
            case WIFI_EVENT_AP_STACONNECTED:
            {
                wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "Station connected: " MACSTR, MAC2STR(event->mac));
                break;
            }
                
            case WIFI_EVENT_AP_STADISCONNECTED:
            {
                wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "Station disconnected: " MACSTR, MAC2STR(event->mac));
                break;
            }
                
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            snprintf(s_ip_addr, sizeof(s_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Got IP: %s", s_ip_addr);
            s_retry_count = 0;
            s_current_status = WIFI_STATUS_CONNECTED;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            if (s_connected_cb) s_connected_cb();
        }
    }
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

int8_t wifiManagerInit(void)
{
    if (s_is_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return 0;
    }
    
    esp_err_t ret;
    
    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    /* Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    
    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    /* Create event group */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return -1;
    }
    
    /* Create default network interfaces */
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    
    /* Initialize WiFi with default config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, NULL, NULL));
    
    s_is_initialized = 1;
    ESP_LOGI(TAG, "WiFi Manager initialized");
    
    return 0;
}

int8_t wifiManagerDeinit(void)
{
    if (!s_is_initialized) return 0;
    
    esp_wifi_stop();
    esp_wifi_deinit();
    
    if (s_sta_netif) esp_netif_destroy(s_sta_netif);
    if (s_ap_netif) esp_netif_destroy(s_ap_netif);
    
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
    
    s_is_initialized = 0;
    return 0;
}

/*******************************************************************************
 * AP Mode
 ******************************************************************************/

int8_t wifiManagerStartAP(const char *ssid, const char *password)
{
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return -1;
    }
    
    const char *ap_ssid = (ssid != NULL) ? ssid : WIFI_AP_SSID_DEFAULT;
    const char *ap_pass = (password != NULL) ? password : WIFI_AP_PASS_DEFAULT;
    
    wifi_config_t wifi_config = {
        .ap = {
            .channel = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    
    strncpy((char *)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    strncpy((char *)wifi_config.ap.password, ap_pass, sizeof(wifi_config.ap.password) - 1);
    
    if (strlen(ap_pass) < 8) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    s_current_mode = WIFI_MODE_AP;
    
    /* Get AP IP address */
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(s_ap_netif, &ip_info);
    snprintf(s_ip_addr, sizeof(s_ip_addr), IPSTR, IP2STR(&ip_info.ip));
    
    ESP_LOGI(TAG, "AP started: SSID=%s, IP=%s", ap_ssid, s_ip_addr);
    
    return 0;
}

int8_t wifiManagerStopAP(void)
{
    if (!s_is_initialized) return -1;
    
    esp_wifi_stop();
    s_current_mode = WIFI_MODE_DISABLED;
    s_current_status = WIFI_STATUS_DISCONNECTED;
    
    return 0;
}

/*******************************************************************************
 * STA Mode
 ******************************************************************************/

int8_t wifiManagerConnect(const char *ssid, const char *password)
{
    if (!s_is_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return -1;
    }
    
    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return -2;
    }
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password != NULL) {
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    
    s_retry_count = 0;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    s_current_mode = WIFI_MODE_STA;
    
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);
    
    /* Wait for connection */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));
    
    if (bits & WIFI_CONNECTED_BIT) {
        return 0;
    } else if (bits & WIFI_FAIL_BIT) {
        return -3;
    }
    
    return -4;  /* Timeout */
}

int8_t wifiManagerDisconnect(void)
{
    if (!s_is_initialized) return -1;
    
    esp_wifi_disconnect();
    s_current_status = WIFI_STATUS_DISCONNECTED;
    
    return 0;
}

/*******************************************************************************
 * Credentials Storage
 ******************************************************************************/

int8_t wifiManagerSaveCredentials(const wifi_credentials_t *creds)
{
    if (creds == NULL) return -1;
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return -2;
    }
    
    err = nvs_set_str(handle, NVS_KEY_SSID, creds->ssid);
    if (err != ESP_OK) {
        nvs_close(handle);
        return -3;
    }
    
    err = nvs_set_str(handle, NVS_KEY_PASS, creds->password);
    if (err != ESP_OK) {
        nvs_close(handle);
        return -4;
    }
    
    err = nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Credentials saved");
    return (err == ESP_OK) ? 0 : -5;
}

int8_t wifiManagerLoadCredentials(wifi_credentials_t *creds)
{
    if (creds == NULL) return -1;
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return -2;  /* No saved credentials */
    }
    
    size_t ssid_len = sizeof(creds->ssid);
    size_t pass_len = sizeof(creds->password);
    
    err = nvs_get_str(handle, NVS_KEY_SSID, creds->ssid, &ssid_len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return -3;
    }
    
    err = nvs_get_str(handle, NVS_KEY_PASS, creds->password, &pass_len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return -4;
    }
    
    nvs_close(handle);
    ESP_LOGI(TAG, "Credentials loaded: SSID=%s", creds->ssid);
    return 0;
}

int8_t wifiManagerClearCredentials(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return -1;
    
    nvs_erase_all(handle);
    nvs_commit(handle);
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Credentials cleared");
    return 0;
}

/*******************************************************************************
 * Status
 ******************************************************************************/

int8_t wifiManagerGetInfo(wifi_connection_info_t *info)
{
    if (info == NULL) return -1;
    
    info->mode = s_current_mode;
    info->status = s_current_status;
    strncpy(info->ip_addr, s_ip_addr, sizeof(info->ip_addr));
    
    if (s_current_status == WIFI_STATUS_CONNECTED) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            info->rssi = ap_info.rssi;
        }
    } else {
        info->rssi = 0;
    }
    
    return 0;
}

uint8_t wifiManagerIsConnected(void)
{
    return (s_current_status == WIFI_STATUS_CONNECTED) ? 1 : 0;
}

/*******************************************************************************
 * Callbacks
 ******************************************************************************/

void wifiManagerSetConnectedCallback(wifi_connected_cb_t callback)
{
    s_connected_cb = callback;
}

void wifiManagerSetDisconnectedCallback(wifi_disconnected_cb_t callback)
{
    s_disconnected_cb = callback;
}
