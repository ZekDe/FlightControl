/**
 * @file web_server.c
 * @brief Embedded Web Server Implementation
 */

#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "web_srv";

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static httpd_handle_t s_server = NULL;
static uint8_t s_is_initialized = 0;

static web_arm_cb_t s_arm_cb = NULL;
static web_pid_cb_t s_pid_cb = NULL;
static web_get_telemetry_cb_t s_telemetry_cb = NULL;
static web_get_pid_cb_t s_get_pid_cb = NULL;

/*******************************************************************************
 * Dashboard HTML
 ******************************************************************************/
static const char *DASHBOARD_HTML = 
"<!DOCTYPE html>"
"<html><head>"
"<title>Quad FC Dashboard</title>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:'Segoe UI',Arial,sans-serif;background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);min-height:100vh;color:#fff}"
".container{max-width:1200px;margin:0 auto;padding:20px}"
"h1{text-align:center;padding:20px;background:rgba(0,217,255,0.1);border-bottom:2px solid #00d9ff}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin-top:20px}"
".card{background:rgba(255,255,255,0.05);border-radius:15px;padding:20px;backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,0.1)}"
".card h2{color:#00d9ff;margin-bottom:15px;font-size:1.2em}"
".value{font-size:2em;font-weight:bold;color:#fff}"
".unit{color:#888;font-size:0.5em}"
".attitude{display:flex;justify-content:space-around;text-align:center}"
".motors{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
".motor{background:rgba(0,217,255,0.2);border-radius:10px;padding:15px;text-align:center}"
".motor-bar{height:100px;background:#333;border-radius:5px;margin-top:10px;position:relative;overflow:hidden}"
".motor-fill{position:absolute;bottom:0;width:100%;background:linear-gradient(to top,#00d9ff,#00ff88);border-radius:5px;transition:height 0.2s}"
".controls{display:flex;gap:10px;flex-wrap:wrap}"
".btn{padding:15px 30px;border:none;border-radius:8px;font-size:16px;cursor:pointer;transition:transform 0.1s}"
".btn-arm{background:#ff4444;color:#fff}"
".btn-arm.armed{background:#44ff44}"
".btn:hover{transform:scale(1.05)}"
".pid-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}"
".pid-input{width:100%;padding:10px;border:1px solid #333;border-radius:5px;background:#1a1a2e;color:#fff}"
".pid-label{font-size:0.8em;color:#888;margin-bottom:5px}"
".status{padding:10px;border-radius:5px;margin-bottom:10px}"
".status.ok{background:rgba(68,255,68,0.2);border:1px solid #44ff44}"
".status.error{background:rgba(255,68,68,0.2);border:1px solid #ff4444}"
"</style></head><body>"
"<h1>🚁 Flight Controller Dashboard</h1>"
"<div class='container'>"
"<div class='grid'>"
"<div class='card'><h2>📐 Attitude</h2>"
"<div class='attitude'>"
"<div><div class='value' id='roll'>0.0</div><div class='unit'>Roll °</div></div>"
"<div><div class='value' id='pitch'>0.0</div><div class='unit'>Pitch °</div></div>"
"<div><div class='value' id='yaw'>0.0</div><div class='unit'>Yaw °</div></div>"
"</div></div>"
"<div class='card'><h2>🔧 Motors</h2>"
"<div class='motors'>"
"<div class='motor'>M1<div class='motor-bar'><div class='motor-fill' id='m1' style='height:0%'></div></div></div>"
"<div class='motor'>M2<div class='motor-bar'><div class='motor-fill' id='m2' style='height:0%'></div></div></div>"
"<div class='motor'>M3<div class='motor-bar'><div class='motor-fill' id='m3' style='height:0%'></div></div></div>"
"<div class='motor'>M4<div class='motor-bar'><div class='motor-fill' id='m4' style='height:0%'></div></div></div>"
"</div></div>"
"<div class='card'><h2>🎮 Control</h2>"
"<div id='status' class='status'>Disconnected</div>"
"<div class='controls'>"
"<button class='btn btn-arm' id='armBtn' onclick='toggleArm()'>ARM</button>"
"</div></div>"
"<div class='card'><h2>⚙️ Rate PID (Roll/Pitch)</h2>"
"<div class='pid-grid'>"
"<div><div class='pid-label'>P</div><input class='pid-input' id='rate_p' value='0.25'></div>"
"<div><div class='pid-label'>I</div><input class='pid-input' id='rate_i' value='0.30'></div>"
"<div><div class='pid-label'>D</div><input class='pid-input' id='rate_d' value='0.003'></div>"
"</div>"
"<button class='btn' style='margin-top:15px;background:#00d9ff;color:#000' onclick='savePid()'>Save PID</button>"
"</div>"
"</div></div>"
"<script>"
"var armed=false;"
"function toggleArm(){"
"fetch('/api/arm',{method:'POST',body:JSON.stringify({arm:!armed})})"
".then(r=>r.json()).then(d=>{armed=d.armed;updateArmBtn()});"
"}"
"function updateArmBtn(){"
"var btn=document.getElementById('armBtn');"
"btn.textContent=armed?'DISARM':'ARM';"
"btn.classList.toggle('armed',armed);"
"}"
"function savePid(){"
"var data={rate_rp_p:parseFloat(document.getElementById('rate_p').value),"
"rate_rp_i:parseFloat(document.getElementById('rate_i').value),"
"rate_rp_d:parseFloat(document.getElementById('rate_d').value)};"
"fetch('/api/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})"
".then(r=>r.json()).then(d=>alert('PID saved!'));"
"}"
"function update(){"
"fetch('/api/telemetry').then(r=>r.json()).then(d=>{"
"document.getElementById('roll').textContent=d.roll.toFixed(1);"
"document.getElementById('pitch').textContent=d.pitch.toFixed(1);"
"document.getElementById('yaw').textContent=d.yaw.toFixed(1);"
"document.getElementById('m1').style.height=d.motor[0]+'%';"
"document.getElementById('m2').style.height=d.motor[1]+'%';"
"document.getElementById('m3').style.height=d.motor[2]+'%';"
"document.getElementById('m4').style.height=d.motor[3]+'%';"
"armed=d.armed;updateArmBtn();"
"document.getElementById('status').textContent='Connected - '+(armed?'ARMED':'Disarmed');"
"document.getElementById('status').className='status ok';"
"}).catch(e=>{"
"document.getElementById('status').textContent='Disconnected';"
"document.getElementById('status').className='status error';"
"});"
"}"
"setInterval(update,200);"
"update();"
"</script></body></html>";

/*******************************************************************************
 * HTTP Handlers
 ******************************************************************************/

/* GET / - Serve dashboard */
static esp_err_t dashboardHandler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, DASHBOARD_HTML, strlen(DASHBOARD_HTML));
    return ESP_OK;
}

/* GET /api/telemetry - Get telemetry JSON */
static esp_err_t telemetryHandler(httpd_req_t *req)
{
    web_telemetry_t tel = {0};
    
    if (s_telemetry_cb) {
        s_telemetry_cb(&tel);
    }
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "roll", tel.roll);
    cJSON_AddNumberToObject(root, "pitch", tel.pitch);
    cJSON_AddNumberToObject(root, "yaw", tel.yaw);
    cJSON_AddNumberToObject(root, "throttle", tel.throttle);
    
    cJSON *motors = cJSON_CreateArray();
    for (int i = 0; i < 4; i++) {
        cJSON_AddItemToArray(motors, cJSON_CreateNumber(tel.motor[i]));
    }
    cJSON_AddItemToObject(root, "motor", motors);
    
    cJSON_AddBoolToObject(root, "armed", tel.armed);
    cJSON_AddNumberToObject(root, "mode", tel.mode);
    cJSON_AddNumberToObject(root, "battery", tel.battery_voltage);
    
    char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    
    free(json);
    cJSON_Delete(root);
    
    return ESP_OK;
}

/* POST /api/arm - Arm/disarm */
static esp_err_t armHandler(httpd_req_t *req)
{
    char buf[64];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *arm_json = cJSON_GetObjectItem(root, "arm");
    uint8_t arm = cJSON_IsTrue(arm_json) ? 1 : 0;
    
    if (s_arm_cb) {
        s_arm_cb(arm);
    }
    
    cJSON_Delete(root);
    
    /* Response */
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "armed", arm);
    cJSON_AddStringToObject(resp, "status", "ok");
    
    char *json = cJSON_PrintUnformatted(resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    
    free(json);
    cJSON_Delete(resp);
    
    ESP_LOGI(TAG, "Arm command: %d", arm);
    
    return ESP_OK;
}

/* POST /api/pid - Set PID gains */
static esp_err_t pidHandler(httpd_req_t *req)
{
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    web_pid_gains_t gains = {0};
    
    cJSON *item;
    if ((item = cJSON_GetObjectItem(root, "rate_rp_p"))) gains.rate_rp_p = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "rate_rp_i"))) gains.rate_rp_i = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "rate_rp_d"))) gains.rate_rp_d = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "rate_yaw_p"))) gains.rate_yaw_p = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "rate_yaw_i"))) gains.rate_yaw_i = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "rate_yaw_d"))) gains.rate_yaw_d = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "att_rp_p"))) gains.att_rp_p = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "att_rp_i"))) gains.att_rp_i = (float)item->valuedouble;
    if ((item = cJSON_GetObjectItem(root, "att_rp_d"))) gains.att_rp_d = (float)item->valuedouble;
    
    if (s_pid_cb) {
        s_pid_cb(&gains);
    }
    
    cJSON_Delete(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    ESP_LOGI(TAG, "PID updated: P=%.3f I=%.3f D=%.4f", 
             gains.rate_rp_p, gains.rate_rp_i, gains.rate_rp_d);
    
    return ESP_OK;
}

/* GET /api/pid - Get PID gains */
static esp_err_t getPidHandler(httpd_req_t *req)
{
    web_pid_gains_t gains = {0};
    
    if (s_get_pid_cb) {
        s_get_pid_cb(&gains);
    }
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "rate_rp_p", gains.rate_rp_p);
    cJSON_AddNumberToObject(root, "rate_rp_i", gains.rate_rp_i);
    cJSON_AddNumberToObject(root, "rate_rp_d", gains.rate_rp_d);
    cJSON_AddNumberToObject(root, "rate_yaw_p", gains.rate_yaw_p);
    cJSON_AddNumberToObject(root, "rate_yaw_i", gains.rate_yaw_i);
    cJSON_AddNumberToObject(root, "rate_yaw_d", gains.rate_yaw_d);
    cJSON_AddNumberToObject(root, "att_rp_p", gains.att_rp_p);
    cJSON_AddNumberToObject(root, "att_rp_i", gains.att_rp_i);
    cJSON_AddNumberToObject(root, "att_rp_d", gains.att_rp_d);
    
    char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    
    free(json);
    cJSON_Delete(root);
    
    return ESP_OK;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

int8_t webServerInit(void)
{
    s_is_initialized = 1;
    ESP_LOGI(TAG, "Web server initialized");
    return 0;
}

int8_t webServerStart(void)
{
    if (s_server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return 0;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 10;
    
    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(ret));
        return -1;
    }
    
    /* Register handlers */
    httpd_uri_t dashboard = { .uri = "/", .method = HTTP_GET, .handler = dashboardHandler };
    httpd_uri_t telemetry = { .uri = "/api/telemetry", .method = HTTP_GET, .handler = telemetryHandler };
    httpd_uri_t arm = { .uri = "/api/arm", .method = HTTP_POST, .handler = armHandler };
    httpd_uri_t pid_set = { .uri = "/api/pid", .method = HTTP_POST, .handler = pidHandler };
    httpd_uri_t pid_get = { .uri = "/api/pid", .method = HTTP_GET, .handler = getPidHandler };
    
    httpd_register_uri_handler(s_server, &dashboard);
    httpd_register_uri_handler(s_server, &telemetry);
    httpd_register_uri_handler(s_server, &arm);
    httpd_register_uri_handler(s_server, &pid_set);
    httpd_register_uri_handler(s_server, &pid_get);
    
    ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    return 0;
}

int8_t webServerStop(void)
{
    if (s_server == NULL) return 0;
    
    httpd_stop(s_server);
    s_server = NULL;
    
    ESP_LOGI(TAG, "Web server stopped");
    return 0;
}

void webServerSetArmCallback(web_arm_cb_t callback)
{
    s_arm_cb = callback;
}

void webServerSetPidCallback(web_pid_cb_t callback)
{
    s_pid_cb = callback;
}

void webServerSetTelemetryCallback(web_get_telemetry_cb_t callback)
{
    s_telemetry_cb = callback;
}

void webServerSetGetPidCallback(web_get_pid_cb_t callback)
{
    s_get_pid_cb = callback;
}
