#include "web_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

static const char *TAG = "web_control";

typedef struct {
    SemaphoreHandle_t lock;
    web_control_config_t config;
    web_control_command_t command;
    web_control_telemetry_t telemetry;
    uint64_t last_command_us;
} web_state_t;

static web_state_t s_state;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void state_lock(void)
{
    xSemaphoreTake(s_state.lock, portMAX_DELAY);
}

static void state_unlock(void)
{
    xSemaphoreGive(s_state.lock);
}

web_control_config_t web_control_default_config(void)
{
    web_control_config_t config = {
        .ap_ssid = "Algorithm6-Control",
        .ap_password = "",
        .command_timeout_ms = 1000,
        .max_manual_linear_mps = 0.35f,
        .max_manual_angular_rps = 0.8f,
    };
    return config;
}

static esp_err_t send_text(httpd_req_t *request, const char *text,
                           const char *content_type)
{
    httpd_resp_set_type(request, content_type);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, text, HTTPD_RESP_USE_STRLEN);
}

static const char s_index_html[] =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Algorithm 6</title><style>body{font-family:sans-serif;max-width:720px;margin:auto;padding:16px}"
    "button{font-size:18px;margin:5px;padding:12px}.stop{background:#c22;color:#fff}</style></head><body>"
    "<h2>Algorithm 6 vehicle control</h2><div id='status'>connecting...</div>"
    "<p><button onclick=\"post('/mode?value=auto')\">AUTO</button>"
    "<button onclick=\"post('/mode?value=manual')\">MANUAL</button>"
    "<button onclick=\"post('/clear')\">CLEAR / ARM</button>"
    "<button class='stop' onclick=\"post('/estop')\">EMERGENCY STOP</button></p>"
    "<p><button onpointerdown=\"drive(.25,0)\" onpointerup='stop()'>FORWARD</button>"
    "<button onpointerdown=\"drive(-.2,0)\" onpointerup='stop()'>BACK</button>"
    "<button onpointerdown=\"drive(0,.6)\" onpointerup='stop()'>LEFT</button>"
    "<button onpointerdown=\"drive(0,-.6)\" onpointerup='stop()'>RIGHT</button>"
    "<button onclick='stop()'>STOP</button></p><pre id='telemetry'></pre>"
    "<script>let hold=null;async function post(u){try{await fetch(u,{method:'POST',cache:'no-store'})}catch(e){}}"
    "function drive(v,w){stop();post('/cmd?v='+v+'&w='+w);hold=setInterval(()=>post('/cmd?v='+v+'&w='+w),250)}"
    "function stop(){if(hold)clearInterval(hold);hold=null;post('/cmd?v=0&w=0')}"
    "setInterval(()=>post('/heartbeat'),300);setInterval(async()=>{try{let r=await fetch('/status',{cache:'no-store'});"
    "let j=await r.json();status.textContent='mode='+j.mode+' estop='+j.estop+' client='+j.client;"
    "telemetry.textContent=JSON.stringify(j,null,2)}catch(e){status.textContent='disconnected'}},500);</script></body></html>";

static esp_err_t root_handler(httpd_req_t *request)
{
    return send_text(request, s_index_html, "text/html");
}

static void mark_command_received(void)
{
    s_state.last_command_us = now_us();
    s_state.command.client_alive = true;
}

static esp_err_t heartbeat_handler(httpd_req_t *request)
{
    state_lock();
    mark_command_received();
    state_unlock();
    return send_text(request, "OK", "text/plain");
}

static esp_err_t estop_handler(httpd_req_t *request)
{
    state_lock();
    s_state.command.estop_latched = true;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
    return send_text(request, "ESTOP", "text/plain");
}

static esp_err_t clear_handler(httpd_req_t *request)
{
    state_lock();
    s_state.command.estop_latched = false;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
    return send_text(request, "ARMED", "text/plain");
}

static bool get_query_value(httpd_req_t *request, const char *key,
                            char *value, size_t value_size)
{
    char query[128];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    return httpd_query_key_value(query, key, value, value_size) == ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *request)
{
    char value[16];
    if (!get_query_value(request, "value", value, sizeof(value))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing mode");
    }
    web_control_mode_t mode;
    if (strcmp(value, "auto") == 0) {
        mode = WEB_CONTROL_MODE_AUTO;
    } else if (strcmp(value, "manual") == 0) {
        mode = WEB_CONTROL_MODE_MANUAL;
    } else {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid mode");
    }
    state_lock();
    s_state.command.mode = mode;
    s_state.command.manual_linear_mps = 0.0f;
    s_state.command.manual_angular_rps = 0.0f;
    mark_command_received();
    state_unlock();
    return send_text(request, "OK", "text/plain");
}

static esp_err_t command_handler(httpd_req_t *request)
{
    char linear_text[24];
    char angular_text[24];
    if (!get_query_value(request, "v", linear_text, sizeof(linear_text)) ||
        !get_query_value(request, "w", angular_text, sizeof(angular_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing command");
    }
    char *linear_end = NULL;
    char *angular_end = NULL;
    const float linear = strtof(linear_text, &linear_end);
    const float angular = strtof(angular_text, &angular_end);
    if (linear_end == linear_text || *linear_end != '\0' ||
        angular_end == angular_text || *angular_end != '\0') {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid command");
    }
    state_lock();
    if (s_state.command.mode != WEB_CONTROL_MODE_MANUAL) {
        state_unlock();
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "manual mode required");
    }
    s_state.command.manual_linear_mps =
        clampf(linear, -s_state.config.max_manual_linear_mps,
                s_state.config.max_manual_linear_mps);
    s_state.command.manual_angular_rps =
        clampf(angular, -s_state.config.max_manual_angular_rps,
                s_state.config.max_manual_angular_rps);
    mark_command_received();
    state_unlock();
    return send_text(request, "OK", "text/plain");
}

static esp_err_t status_handler(httpd_req_t *request)
{
    web_control_command_t command;
    web_control_telemetry_t telemetry;
    web_control_get_command(&command);
    state_lock();
    telemetry = s_state.telemetry;
    state_unlock();
    char json[768];
    const int length = snprintf(
        json, sizeof(json),
        "{\"mode\":\"%s\",\"estop\":%s,\"client\":%s,\"age_ms\":%lu,"
        "\"state\":\"%s\",\"uwb\":%s,\"lidar\":%s,\"ultra_left\":%s,"
        "\"ultra_right\":%s,\"fsr\":%s,\"encoder\":%s,"
        "\"target_m\":%.3f,\"bearing_rad\":%.3f,\"clearance_m\":%.3f,"
        "\"fsr_v\":%.3f,\"fsr_kg\":%.3f,\"fsr_raw\":%d,\"measured_v\":%.3f,\"measured_w\":%.3f,"
        "\"left_us\":%d,\"right_us\":%d}",
        command.mode == WEB_CONTROL_MODE_AUTO ? "auto" : "manual",
        command.estop_latched ? "true" : "false",
        command.client_alive ? "true" : "false",
        (unsigned long)command.command_age_ms,
        telemetry.state == NULL ? "BOOT" : telemetry.state,
        telemetry.uwb_ok ? "true" : "false",
        telemetry.lidar_ok ? "true" : "false",
        telemetry.ultrasonic_left_ok ? "true" : "false",
        telemetry.ultrasonic_right_ok ? "true" : "false",
        telemetry.fsr_ok ? "true" : "false",
        telemetry.encoder_ok ? "true" : "false",
        telemetry.target_distance_m, telemetry.target_bearing_rad,
        telemetry.front_clearance_m, telemetry.fsr_voltage_v,
        telemetry.fsr_weight_kg, telemetry.fsr_raw,
        telemetry.measured_linear_mps, telemetry.measured_angular_rps,
        telemetry.left_pulse_us, telemetry.right_pulse_us);
    if (length < 0 || length >= (int)sizeof(json)) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status overflow");
    }
    return send_text(request, json, "application/json");
}

static esp_err_t register_uri(httpd_handle_t server, const char *uri,
                              httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
    const httpd_uri_t descriptor = {
        .uri = uri,
        .method = method,
        .handler = handler,
        .user_ctx = NULL,
    };
    return httpd_register_uri_handler(server, &descriptor);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    if (id == WIFI_EVENT_AP_STADISCONNECTED && s_state.lock != NULL) {
        state_lock();
        s_state.command.client_alive = false;
        s_state.command.manual_linear_mps = 0.0f;
        s_state.command.manual_angular_rps = 0.0f;
        state_unlock();
    }
}

static esp_err_t start_wifi(const web_control_config_t *config)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret == ESP_OK) {
            ret = nvs_flash_init();
        }
    }
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    if (esp_netif_create_default_wifi_ap() == NULL) {
        return ESP_FAIL;
    }
    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&init);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    wifi_config_t wifi = {0};
    strlcpy((char *)wifi.ap.ssid, config->ap_ssid, sizeof(wifi.ap.ssid));
    wifi.ap.ssid_len = strlen((char *)wifi.ap.ssid);
    wifi.ap.channel = 1;
    wifi.ap.max_connection = 2;
    const size_t password_length = strlen(config->ap_password);
    if (password_length >= 8) {
        strlcpy((char *)wifi.ap.password, config->ap_password,
                sizeof(wifi.ap.password));
        wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi.ap.authmode = WIFI_AUTH_OPEN;
    }
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret == ESP_OK) {
        ret = esp_wifi_set_config(WIFI_IF_AP, &wifi);
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_start();
    }
    return ret;
}

esp_err_t web_control_init(const web_control_config_t *config)
{
    if (config == NULL || config->ap_ssid == NULL ||
        config->ap_password == NULL || config->command_timeout_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s_state, 0, sizeof(s_state));
    s_state.lock = xSemaphoreCreateMutex();
    if (s_state.lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_state.config = *config;
    s_state.command.mode = WEB_CONTROL_MODE_AUTO;
    s_state.command.estop_latched = true;
    esp_err_t ret = start_wifi(config);
    if (ret != ESP_OK) {
        return ret;
    }
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.max_uri_handlers = 8;
    httpd_handle_t server = NULL;
    ret = httpd_start(&server, &server_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = register_uri(server, "/", HTTP_GET, root_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/heartbeat", HTTP_POST, heartbeat_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/estop", HTTP_POST, estop_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/clear", HTTP_POST, clear_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/mode", HTTP_POST, mode_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/cmd", HTTP_POST, command_handler);
    if (ret == ESP_OK) ret = register_uri(server, "/status", HTTP_GET, status_handler);
    if (ret != ESP_OK) {
        httpd_stop(server);
        return ret;
    }
    ESP_LOGI(TAG, "control AP ready: SSID=%s (credentials are not logged)",
             config->ap_ssid);
    return ESP_OK;
}

void web_control_get_command(web_control_command_t *command)
{
    if (command == NULL || s_state.lock == NULL) {
        return;
    }
    state_lock();
    const uint64_t current_us = now_us();
    const uint64_t age_us = current_us >= s_state.last_command_us
                                ? current_us - s_state.last_command_us
                                : UINT64_MAX;
    const uint64_t timeout_us = (uint64_t)s_state.config.command_timeout_ms * 1000ULL;
    if (s_state.last_command_us == 0 || age_us > timeout_us) {
        s_state.command.client_alive = false;
        s_state.command.manual_linear_mps = 0.0f;
        s_state.command.manual_angular_rps = 0.0f;
    }
    s_state.command.command_age_ms = age_us == UINT64_MAX
                                         ? UINT32_MAX
                                         : (uint32_t)(age_us / 1000ULL);
    *command = s_state.command;
    state_unlock();
}

void web_control_publish(const web_control_telemetry_t *telemetry)
{
    if (telemetry == NULL || s_state.lock == NULL) {
        return;
    }
    state_lock();
    s_state.telemetry = *telemetry;
    state_unlock();
}
