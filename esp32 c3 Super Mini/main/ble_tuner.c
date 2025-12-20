/**
 * @file ble_tuner.c
 * @brief BLE PID Tuner for Quadcopter Flight Controller
 * @author Claude & duatepe
 * @date 2024
 */

#include "ble_tuner.h"
#include "esp_log.h"

/* NimBLE includes */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "ble_tuner";

/*******************************************************************************
 * Nordic UART Service UUIDs (Standard - nRF Connect compatible)
 ******************************************************************************/

/* NUS Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* NUS RX Characteristic UUID: 6E400002-... (Phone writes here) */
static const ble_uuid128_t nus_chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

/* NUS TX Characteristic UUID: 6E400003-... (ESP32 notifies here) */
static const ble_uuid128_t nus_chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

static char ble_device_name[BLE_TUNER_DEVICE_NAME_MAX] = "QUAD-FC";
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_chr_val_handle;

static uint8_t is_initialized = 0;
static uint8_t telemetry_enabled = 0;
static uint8_t notify_enabled = 0;

/* Heartbeat tracking */
static uint32_t last_heartbeat_ms = 0;
static uint8_t heartbeat_active = 0;

/* Callbacks */
static bleTunerCmdCallback_t cmd_callback = NULL;
static bleTunerDisconnectCallback_t disconnect_callback = NULL;
static bleTunerGetGainsCallback_t get_gains_callback = NULL;

/*******************************************************************************
 * Private Function Declarations
 ******************************************************************************/

static int gattSvrChrAccessRx(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gattSvrChrAccessTx(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void bleHostTask(void *param);
static void bleOnSync(void);
static void bleOnReset(int reason);
static int bleGapEvent(struct ble_gap_event *event, void *arg);
static void startAdvertising(void);
static void parseCommand(const char *cmd_str, uint16_t len);
static int8_t parseFloat(const char *str, float *value);
static int8_t parsePidGains(const char *str, bleTunerPidGains_t *gains);
static void sendOkResponse(const char *cmd_name);
static void sendErrorResponse(const char *error_msg);
static void sendGainsResponse(void);

/*******************************************************************************
 * GATT Service Definition
 ******************************************************************************/

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Nordic UART Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX Characteristic - Receive commands from phone */
                .uuid = &nus_chr_rx_uuid.u,
                .access_cb = gattSvrChrAccessRx,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* TX Characteristic - Send telemetry/responses to phone */
                .uuid = &nus_chr_tx_uuid.u,
                .access_cb = gattSvrChrAccessTx,
                .val_handle = &tx_chr_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, /* No more characteristics */
            },
        },
    },
    {
        0, /* No more services */
    },
};

/*******************************************************************************
 * Public Functions - Initialization
 ******************************************************************************/

int8_t bleTunerInit(const char *device_name)
{
    if (is_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return 0;
    }

    ESP_LOGI(TAG, "Initializing BLE Tuner");

    /* Store device name */
    if (device_name != NULL) {
        strncpy(ble_device_name, device_name, sizeof(ble_device_name) - 1);
        ble_device_name[sizeof(ble_device_name) - 1] = '\0';
    }

    ESP_LOGI(TAG, "Device Name: %s", ble_device_name);

    /* Initialize NimBLE */
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Initialize GATT server */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Register GATT services */
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return -1;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return -1;
    }

    /* Set device name */
    rc = ble_svc_gap_device_name_set(ble_device_name);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_svc_gap_device_name_set failed: %d", rc);
    }

    /* Configure host callbacks */
    ble_hs_cfg.sync_cb = bleOnSync;
    ble_hs_cfg.reset_cb = bleOnReset;

    /* Start NimBLE host task */
    nimble_port_freertos_init(bleHostTask);

    is_initialized = 1;
    telemetry_enabled = 0;
    heartbeat_active = 0;

    ESP_LOGI(TAG, "BLE Tuner initialized");
    return 0;
}

int8_t bleTunerStartAdvertising(void)
{
    if (!is_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return -1;
    }

    startAdvertising();
    return 0;
}

int8_t bleTunerStopAdvertising(void)
{
    if (!is_initialized) {
        return -1;
    }

    int rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_stop failed: %d", rc);
        return -1;
    }

    ESP_LOGI(TAG, "Advertising stopped");
    return 0;
}

/*******************************************************************************
 * Public Functions - Callbacks
 ******************************************************************************/

void bleTunerSetCmdCallback(bleTunerCmdCallback_t callback)
{
    cmd_callback = callback;
}

void bleTunerSetDisconnectCallback(bleTunerDisconnectCallback_t callback)
{
    disconnect_callback = callback;
}

void bleTunerSetGetGainsCallback(bleTunerGetGainsCallback_t callback)
{
    get_gains_callback = callback;
}

/*******************************************************************************
 * Public Functions - Telemetry
 ******************************************************************************/

int8_t bleTunerSendTelemetry(const bleTunerTelemetry_t *telemetry)
{
    if (!is_initialized || conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return -1;
    }

    if (!telemetry_enabled || !notify_enabled) {
        return -1;
    }

    if (telemetry == NULL) {
        return -1;
    }

    char buf[BLE_TUNER_TX_MAX_LEN];
    struct os_mbuf *om;
    int rc;

    /* Send Attitude */
    snprintf(buf, sizeof(buf), "$ATT,%.1f,%.1f,%.1f*\n",
             telemetry->roll_deg, telemetry->pitch_deg, telemetry->yaw_deg);
    
    om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om != NULL) {
        rc = ble_gatts_notify_custom(conn_handle, tx_chr_val_handle, om);
        if (rc != 0) {
            ESP_LOGD(TAG, "ATT notify failed: %d", rc);
        }
    }

    /* Send Motors */
    snprintf(buf, sizeof(buf), "$MOT,%.1f,%.1f,%.1f,%.1f*\n",
             telemetry->motor[0], telemetry->motor[1],
             telemetry->motor[2], telemetry->motor[3]);
    
    om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om != NULL) {
        rc = ble_gatts_notify_custom(conn_handle, tx_chr_val_handle, om);
        if (rc != 0) {
            ESP_LOGD(TAG, "MOT notify failed: %d", rc);
        }
    }

    /* Send Gyro */
    snprintf(buf, sizeof(buf), "$GYR,%.1f,%.1f,%.1f*\n",
             telemetry->gyro_x_dps, telemetry->gyro_y_dps, telemetry->gyro_z_dps);
    
    om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om != NULL) {
        rc = ble_gatts_notify_custom(conn_handle, tx_chr_val_handle, om);
        if (rc != 0) {
            ESP_LOGD(TAG, "GYR notify failed: %d", rc);
        }
    }

    return 0;
}

int8_t bleTunerSendResponse(const char *response)
{
    if (!is_initialized || conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return -1;
    }

    if (!notify_enabled) {
        return -1;
    }

    if (response == NULL) {
        return -1;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(response, strlen(response));
    if (om == NULL) {
        return -1;
    }

    int rc = ble_gatts_notify_custom(conn_handle, tx_chr_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Response notify failed: %d", rc);
        return -1;
    }

    return 0;
}

/*******************************************************************************
 * Public Functions - Status
 ******************************************************************************/

uint8_t bleTunerIsConnected(void)
{
    return (conn_handle != BLE_HS_CONN_HANDLE_NONE) ? 1 : 0;
}

uint8_t bleTunerIsTelemetryEnabled(void)
{
    return telemetry_enabled;
}

uint8_t bleTunerCheckHeartbeatTimeout(uint32_t current_time_ms)
{
    if (!heartbeat_active) {
        return 0;  /* Heartbeat not active yet */
    }

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return 0;  /* Not connected */
    }

    uint32_t elapsed = current_time_ms - last_heartbeat_ms;
    if (elapsed > BLE_TUNER_HB_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Heartbeat timeout! Elapsed: %lu ms", (unsigned long)elapsed);
        return 1;
    }

    return 0;
}

void bleTunerResetHeartbeat(uint32_t current_time_ms)
{
    last_heartbeat_ms = current_time_ms;
    heartbeat_active = 1;
}

/*******************************************************************************
 * Private Functions - GATT Callbacks
 ******************************************************************************/

static int gattSvrChrAccessRx(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    /* Read the incoming data */
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= BLE_TUNER_CMD_MAX_LEN) {
        len = BLE_TUNER_CMD_MAX_LEN - 1;
    }

    char cmd_buf[BLE_TUNER_CMD_MAX_LEN] = {0};
    int rc = ble_hs_mbuf_to_flat(ctxt->om, cmd_buf, len, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "mbuf_to_flat failed: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }

    cmd_buf[len] = '\0';

    /* Remove trailing newline/carriage return */
    while (len > 0 && (cmd_buf[len - 1] == '\n' || cmd_buf[len - 1] == '\r')) {
        cmd_buf[--len] = '\0';
    }

    ESP_LOGI(TAG, "RX: %s", cmd_buf);

    /* Parse and process command */
    parseCommand(cmd_buf, len);

    return 0;
}

static int gattSvrChrAccessTx(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;

    /* TX characteristic is notify-only, no read/write access */
    return 0;
}

/*******************************************************************************
 * Private Functions - BLE Host
 ******************************************************************************/

static void bleHostTask(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "BLE Host Task started");
    
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void bleOnSync(void)
{
    ESP_LOGI(TAG, "BLE Host synced");

    /* Set preferred MTU */
    ble_att_set_preferred_mtu(256);

    /* Ensure we have proper identity address */
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }

    /* Start advertising */
    startAdvertising();
}

static void bleOnReset(int reason)
{
    ESP_LOGW(TAG, "BLE Host reset, reason: %d", reason);
}

static int bleGapEvent(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected, handle: %d", conn_handle);
                
                /* Reset heartbeat on new connection */
                heartbeat_active = 0;
                telemetry_enabled = 0;
                notify_enabled = 0;
            } else {
                ESP_LOGW(TAG, "Connection failed, status: %d", event->connect.status);
                conn_handle = BLE_HS_CONN_HANDLE_NONE;
                startAdvertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected, reason: %d", event->disconnect.reason);
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            heartbeat_active = 0;
            telemetry_enabled = 0;
            notify_enabled = 0;
            
            /* Call disconnect callback (for auto-disarm) */
            if (disconnect_callback != NULL) {
                disconnect_callback();
            }
            
            startAdvertising();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event, cur_notify: %d", event->subscribe.cur_notify);
            if (event->subscribe.attr_handle == tx_chr_val_handle) {
                notify_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "TX notifications %s", notify_enabled ? "enabled" : "disabled");
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            startAdvertising();
            break;

        default:
            ESP_LOGD(TAG, "GAP event: %d", event->type);
            break;
    }

    return 0;
}

static void startAdvertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    /* Advertise flags */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Include device name */
    fields.name = (uint8_t *)ble_device_name;
    fields.name_len = strlen(ble_device_name);
    fields.name_is_complete = 1;

    /* Include TX power level */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    /* Advertising parameters - fast interval for quick discovery */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;  /* 30ms */
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;  /* 60ms */

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, bleGapEvent, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started: %s", ble_device_name);
}

/*******************************************************************************
 * Private Functions - Command Parsing
 ******************************************************************************/

static void parseCommand(const char *cmd_str, uint16_t len)
{
    if (cmd_str == NULL || len == 0) {
        sendErrorResponse("EMPTY_CMD");
        return;
    }

    bleTunerCommand_t cmd = {0};
    cmd.cmd = BLE_CMD_UNKNOWN;

    /* Simple string comparisons for commands */
    
    /* ARM */
    if (strcmp(cmd_str, "ARM") == 0) {
        cmd.cmd = BLE_CMD_ARM;
        sendOkResponse("ARM");
    }
    /* DISARM */
    else if (strcmp(cmd_str, "DISARM") == 0) {
        cmd.cmd = BLE_CMD_DISARM;
        sendOkResponse("DISARM");
    }
    /* HB (Heartbeat) */
    else if (strcmp(cmd_str, "HB") == 0) {
        cmd.cmd = BLE_CMD_HEARTBEAT;
        /* Don't send response for HB to reduce traffic */
        /* Just update timestamp via callback */
    }
    /* GET */
    else if (strcmp(cmd_str, "GET") == 0) {
        cmd.cmd = BLE_CMD_GET;
        sendGainsResponse();
    }
    /* THR=value */
    else if (strncmp(cmd_str, "THR=", 4) == 0) {
        cmd.cmd = BLE_CMD_THROTTLE;
        if (parseFloat(cmd_str + 4, &cmd.data.value) == 0) {
            sendOkResponse("THR");
        } else {
            sendErrorResponse("THR_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* ROLL=value */
    else if (strncmp(cmd_str, "ROLL=", 5) == 0) {
        cmd.cmd = BLE_CMD_ROLL;
        if (parseFloat(cmd_str + 5, &cmd.data.value) == 0) {
            sendOkResponse("ROLL");
        } else {
            sendErrorResponse("ROLL_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* PITCH=value */
    else if (strncmp(cmd_str, "PITCH=", 6) == 0) {
        cmd.cmd = BLE_CMD_PITCH;
        if (parseFloat(cmd_str + 6, &cmd.data.value) == 0) {
            sendOkResponse("PITCH");
        } else {
            sendErrorResponse("PITCH_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* YAW=value */
    else if (strncmp(cmd_str, "YAW=", 4) == 0) {
        cmd.cmd = BLE_CMD_YAW;
        if (parseFloat(cmd_str + 4, &cmd.data.value) == 0) {
            sendOkResponse("YAW");
        } else {
            sendErrorResponse("YAW_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* RATE_RP=Kp,Ki,Kd */
    else if (strncmp(cmd_str, "RATE_RP=", 8) == 0) {
        cmd.cmd = BLE_CMD_RATE_RP;
        if (parsePidGains(cmd_str + 8, &cmd.data.gains) == 0) {
            char resp[64];
            snprintf(resp, sizeof(resp), "OK:RATE_RP=%.3f,%.3f,%.4f\n",
                     cmd.data.gains.kp, cmd.data.gains.ki, cmd.data.gains.kd);
            bleTunerSendResponse(resp);
        } else {
            sendErrorResponse("RATE_RP_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* RATE_YAW=Kp,Ki,Kd */
    else if (strncmp(cmd_str, "RATE_YAW=", 9) == 0) {
        cmd.cmd = BLE_CMD_RATE_YAW;
        if (parsePidGains(cmd_str + 9, &cmd.data.gains) == 0) {
            char resp[64];
            snprintf(resp, sizeof(resp), "OK:RATE_YAW=%.3f,%.3f,%.4f\n",
                     cmd.data.gains.kp, cmd.data.gains.ki, cmd.data.gains.kd);
            bleTunerSendResponse(resp);
        } else {
            sendErrorResponse("RATE_YAW_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* ATT_RP=Kp,Ki,Kd */
    else if (strncmp(cmd_str, "ATT_RP=", 7) == 0) {
        cmd.cmd = BLE_CMD_ATT_RP;
        if (parsePidGains(cmd_str + 7, &cmd.data.gains) == 0) {
            char resp[64];
            snprintf(resp, sizeof(resp), "OK:ATT_RP=%.3f,%.3f,%.4f\n",
                     cmd.data.gains.kp, cmd.data.gains.ki, cmd.data.gains.kd);
            bleTunerSendResponse(resp);
        } else {
            sendErrorResponse("ATT_RP_PARSE");
            cmd.cmd = BLE_CMD_UNKNOWN;
        }
    }
    /* TEL=0/1 */
    else if (strncmp(cmd_str, "TEL=", 4) == 0) {
        cmd.cmd = BLE_CMD_TEL_ENABLE;
        int val = atoi(cmd_str + 4);
        cmd.data.enable = (val != 0) ? 1 : 0;
        telemetry_enabled = cmd.data.enable;
        
        char resp[32];
        snprintf(resp, sizeof(resp), "OK:TEL=%d\n", telemetry_enabled);
        bleTunerSendResponse(resp);
    }
    /* Unknown command */
    else {
        sendErrorResponse("UNKNOWN_CMD");
        return;  /* Don't call callback for unknown commands */
    }

    /* Call user callback if command was valid */
    if (cmd.cmd != BLE_CMD_UNKNOWN && cmd_callback != NULL) {
        cmd_callback(&cmd);
    }
}

static int8_t parseFloat(const char *str, float *value)
{
    if (str == NULL || value == NULL) {
        return -1;
    }

    char *endptr;
    *value = strtof(str, &endptr);
    
    if (endptr == str) {
        return -1;  /* No conversion performed */
    }

    return 0;
}

static int8_t parsePidGains(const char *str, bleTunerPidGains_t *gains)
{
    if (str == NULL || gains == NULL) {
        return -1;
    }

    /* Expected format: "Kp,Ki,Kd" e.g., "0.25,0.30,0.003" */
    float kp, ki, kd;
    int parsed = sscanf(str, "%f,%f,%f", &kp, &ki, &kd);
    
    if (parsed != 3) {
        ESP_LOGW(TAG, "PID parse failed, got %d values from: %s", parsed, str);
        return -1;
    }

    gains->kp = kp;
    gains->ki = ki;
    gains->kd = kd;

    return 0;
}

static void sendOkResponse(const char *cmd_name)
{
    char resp[32];
    snprintf(resp, sizeof(resp), "OK:%s\n", cmd_name);
    bleTunerSendResponse(resp);
}

static void sendErrorResponse(const char *error_msg)
{
    char resp[48];
    snprintf(resp, sizeof(resp), "ERR:%s\n", error_msg);
    bleTunerSendResponse(resp);
}

static void sendGainsResponse(void)
{
    if (get_gains_callback == NULL) {
        sendErrorResponse("NO_GAINS_CB");
        return;
    }

    bleTunerGains_t gains = {0};
    get_gains_callback(&gains);

    char resp[128];
    
    /* Send Rate RP gains */
    snprintf(resp, sizeof(resp), "RATE_RP=%.3f,%.3f,%.4f\n",
             gains.rate_rp.kp, gains.rate_rp.ki, gains.rate_rp.kd);
    bleTunerSendResponse(resp);

    /* Send Rate Yaw gains */
    snprintf(resp, sizeof(resp), "RATE_YAW=%.3f,%.3f,%.4f\n",
             gains.rate_yaw.kp, gains.rate_yaw.ki, gains.rate_yaw.kd);
    bleTunerSendResponse(resp);

    /* Send Attitude RP gains */
    snprintf(resp, sizeof(resp), "ATT_RP=%.3f,%.3f,%.4f\n",
             gains.att_rp.kp, gains.att_rp.ki, gains.att_rp.kd);
    bleTunerSendResponse(resp);
}
