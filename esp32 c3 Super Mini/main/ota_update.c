/**
 * @file ota_update.c
 * @brief OTA Update Implementation
 */

#include "ota_update.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>

static const char *TAG = "ota";

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static httpd_handle_t s_server = NULL;
static ota_status_t s_status = OTA_STATUS_IDLE;
static uint32_t s_downloaded = 0;
static uint32_t s_total = 0;

static ota_progress_cb_t s_progress_cb = NULL;
static ota_complete_cb_t s_complete_cb = NULL;

/*******************************************************************************
 * HTML Page for Upload
 ******************************************************************************/
static const char *OTA_HTML = 
"<!DOCTYPE html>"
"<html><head><title>OTA Update</title>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style>"
"body{font-family:Arial,sans-serif;margin:40px;background:#1a1a2e;color:#eee;}"
"h1{color:#00d9ff;}"
".container{max-width:500px;margin:0 auto;padding:20px;background:#16213e;border-radius:10px;}"
"input[type=file]{width:100%;padding:10px;margin:10px 0;}"
"button{background:#00d9ff;color:#1a1a2e;border:none;padding:15px 30px;font-size:16px;cursor:pointer;border-radius:5px;width:100%;}"
"button:hover{background:#00b8d4;}"
".progress{width:100%;background:#0f3460;height:20px;border-radius:10px;margin:10px 0;}"
".progress-bar{width:0%;background:#00d9ff;height:100%;border-radius:10px;transition:width 0.3s;}"
"#status{margin-top:10px;padding:10px;background:#0f3460;border-radius:5px;}"
"</style></head><body>"
"<div class='container'>"
"<h1>Flight Controller OTA</h1>"
"<form id='form' enctype='multipart/form-data'>"
"<input type='file' name='firmware' id='firmware' accept='.bin'>"
"<button type='submit'>Upload Firmware</button>"
"</form>"
"<div class='progress'><div class='progress-bar' id='bar'></div></div>"
"<div id='status'>Ready for upload</div>"
"</div>"
"<script>"
"document.getElementById('form').onsubmit=function(e){"
"e.preventDefault();"
"var file=document.getElementById('firmware').files[0];"
"if(!file){alert('Select a file');return;}"
"var xhr=new XMLHttpRequest();"
"xhr.open('POST','/update',true);"
"xhr.upload.onprogress=function(e){"
"var p=Math.round(e.loaded/e.total*100);"
"document.getElementById('bar').style.width=p+'%';"
"document.getElementById('status').textContent='Uploading: '+p+'%';"
"};"
"xhr.onload=function(){"
"if(xhr.status==200){"
"document.getElementById('status').textContent='Success! Rebooting...';"
"setTimeout(function(){location.reload();},5000);"
"}else{"
"document.getElementById('status').textContent='Error: '+xhr.responseText;"
"}"
"};"
"xhr.send(file);"
"};"
"</script></body></html>";

/*******************************************************************************
 * HTTP Handlers
 ******************************************************************************/

/* GET /update - Serve upload page */
static esp_err_t otaPageHandler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, OTA_HTML, strlen(OTA_HTML));
    return ESP_OK;
}

/* POST /update - Handle firmware upload */
static esp_err_t otaUploadHandler(httpd_req_t *req)
{
    esp_ota_handle_t ota_handle;
    const esp_partition_t *update_partition;
    esp_err_t err;
    
    s_status = OTA_STATUS_DOWNLOADING;
    s_downloaded = 0;
    s_total = req->content_len;
    
    ESP_LOGI(TAG, "Starting OTA, size: %lu bytes", (unsigned long)s_total);
    
    /* Get update partition */
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found");
        s_status = OTA_STATUS_ERROR;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Writing to partition: %s", update_partition->label);
    
    /* Begin OTA */
    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        s_status = OTA_STATUS_ERROR;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }
    
    /* Receive and write data */
    char buf[1024];
    int received;
    
    while (s_downloaded < s_total) {
        received = httpd_req_recv(req, buf, sizeof(buf));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            esp_ota_abort(ota_handle);
            s_status = OTA_STATUS_ERROR;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }
        
        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            s_status = OTA_STATUS_ERROR;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }
        
        s_downloaded += received;
        
        if (s_progress_cb) {
            s_progress_cb(s_downloaded, s_total);
        }
    }
    
    s_status = OTA_STATUS_VERIFYING;
    ESP_LOGI(TAG, "Download complete, verifying...");
    
    /* End OTA */
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        s_status = OTA_STATUS_ERROR;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Verification failed");
        if (s_complete_cb) s_complete_cb(0);
        return ESP_FAIL;
    }
    
    /* Set boot partition */
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        s_status = OTA_STATUS_ERROR;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        if (s_complete_cb) s_complete_cb(0);
        return ESP_FAIL;
    }
    
    s_status = OTA_STATUS_COMPLETE;
    ESP_LOGI(TAG, "OTA complete! Rebooting...");
    
    httpd_resp_sendstr(req, "OK");
    
    if (s_complete_cb) s_complete_cb(1);
    
    /* Reboot after short delay */
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

int8_t otaInit(void)
{
    s_status = OTA_STATUS_IDLE;
    s_downloaded = 0;
    s_total = 0;
    
    ESP_LOGI(TAG, "OTA system initialized");
    return 0;
}

int8_t otaStartServer(void)
{
    if (s_server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return 0;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 8;
    
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(err));
        return -1;
    }
    
    /* Register handlers */
    httpd_uri_t page_uri = {
        .uri = "/update",
        .method = HTTP_GET,
        .handler = otaPageHandler,
    };
    httpd_register_uri_handler(s_server, &page_uri);
    
    httpd_uri_t upload_uri = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = otaUploadHandler,
    };
    httpd_register_uri_handler(s_server, &upload_uri);
    
    ESP_LOGI(TAG, "OTA server started on port %d", config.server_port);
    return 0;
}

int8_t otaStopServer(void)
{
    if (s_server == NULL) return 0;
    
    httpd_stop(s_server);
    s_server = NULL;
    
    ESP_LOGI(TAG, "OTA server stopped");
    return 0;
}

int8_t otaStartFromUrl(const char *url)
{
    /* TODO: Implement HTTP client OTA download */
    ESP_LOGW(TAG, "URL OTA not implemented yet: %s", url);
    return -1;
}

ota_status_t otaGetStatus(void)
{
    return s_status;
}

void otaGetProgress(uint32_t *downloaded, uint32_t *total)
{
    if (downloaded) *downloaded = s_downloaded;
    if (total) *total = s_total;
}

void otaSetProgressCallback(ota_progress_cb_t callback)
{
    s_progress_cb = callback;
}

void otaSetCompleteCallback(ota_complete_cb_t callback)
{
    s_complete_cb = callback;
}

int8_t otaRollback(void)
{
    esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Rollback failed: %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}

int8_t otaMarkValid(void)
{
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mark valid failed: %s", esp_err_to_name(err));
        return -1;
    }
    ESP_LOGI(TAG, "Firmware marked as valid");
    return 0;
}
