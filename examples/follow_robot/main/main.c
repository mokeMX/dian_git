/*
 * follow_robot (算法2+Web) - smart follow-me suitcase with CLOSED-LOOP drive
 *                           + WiFi remote control dashboard.
 *
 * Wiring of the sensing/acting stack:
 *   UWB (BU0x)      -> follow target  (range + bearing to the user's tag)
 *   RPLIDAR C1      -> obstacle field (front 180 deg polar histogram)
 *   2x A02YYUW      -> front-corner near-field safety (ultrasonic)
 *   IMU             -> heading closed-loop (yaw error trims the turn command)
 *   chassis         -> rear diff-drive: APO-DL ESC (RC PWM) + AB encoder PID
 *
 * New in this version (算法2+Web):
 *   - WiFi SoftAP + embedded HTTP dashboard for phone-based monitoring
 *   - Remote safety state machine: E-STOP latch, CLEAR/ARM, heartbeat timeout
 *   - UWB EMA smoothing with outlier rejection for stable tracking
 *   - Real-time telemetry via /live JSON API
 *   - Optional SPIFFS flash CSV logging
 *
 * Architecture: each sensor runs in its own FreeRTOS task and publishes into a
 * mutex-protected snapshot. A fixed-rate control task reads the snapshot, runs
 * the follow_avoid algorithm, applies the IMU heading loop, enforces the remote
 * safety gate, and drives the chassis (set_velocity + update). If any sensor
 * fails to start, its data stays "stale/invalid" and the algorithm degrades
 * gracefully.
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "a02yyuw.h"
#include "bu_uwb.h"
#include "rplidar_c1.h"
#include "chassis.h"
#include "follow_avoid.h"

#include "driver/i2c_master.h"
#include "imu_i2c.h"

static const char *TAG = "follow_robot";

#define M_PI 3.14159265358979323846
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)
#define RAD2DEG(r) ((float)(r) * 180.0f / (float)M_PI)

#define LIDAR_SECTORS 36                 /* 5 deg per sector over 180 deg FOV */
#define LIDAR_FOV_RAD ((float)M_PI)

/* Freshness windows: data older than this is treated as missing. */
#define TARGET_FRESH_US 700000ULL        /* 0.7 s */
#define FIELD_FRESH_US 500000ULL         /* 0.5 s */
#define ULTRA_FRESH_US 500000ULL         /* 0.5 s */

/* ---- Compile-time switches (from Kconfig or hardcoded fallbacks) ---- */
#if defined(CONFIG_FOLLOW_ROBOT_WIFI_ENABLE) && CONFIG_FOLLOW_ROBOT_WIFI_ENABLE
#define FR_WIFI_ENABLED 1
#else
#define FR_WIFI_ENABLED 0
#endif

#if defined(CONFIG_FOLLOW_ROBOT_FLASH_LOG_ENABLE) && CONFIG_FOLLOW_ROBOT_FLASH_LOG_ENABLE
#define FR_FLASH_LOG_ENABLED 1
#else
#define FR_FLASH_LOG_ENABLED 0
#endif

/* WiFi remote defaults (with Kconfig overrides). */
#define FR_WIFI_AP_SSID              CONFIG_FOLLOW_ROBOT_WIFI_AP_SSID
#define FR_WIFI_AP_PASS              CONFIG_FOLLOW_ROBOT_WIFI_AP_PASS
#define FR_WIFI_AP_CHANNEL           CONFIG_FOLLOW_ROBOT_WIFI_AP_CHANNEL
#define FR_WIFI_AP_MAX_CONN          2
#define FR_WIFI_HB_TIMEOUT_US        ((uint64_t)CONFIG_FOLLOW_ROBOT_WIFI_HEARTBEAT_TIMEOUT_MS * 1000ULL)
#define FR_WIFI_REQUIRE_CLIENT       CONFIG_FOLLOW_ROBOT_WIFI_REQUIRE_CLIENT
#define FR_AUTO_ARM                  CONFIG_FOLLOW_ROBOT_AUTO_ARM

/* Flash log. */
#define FR_LOG_PARTITION_LABEL       "logs"
#define FR_LOG_BASE_PATH             "/spiffs"
#define FR_LOG_PATH                  "/spiffs/follow_log.csv"
#define FR_LOG_QUEUE_LEN             96
#ifndef CONFIG_FOLLOW_ROBOT_FLASH_LOG_HZ
#define FR_LOG_HZ                    5
#else
#define FR_LOG_HZ                    CONFIG_FOLLOW_ROBOT_FLASH_LOG_HZ
#endif
#define FR_LIVE_JSON_BUF_SIZE        4096

/* UWB smoothing. */
#define FR_UWB_SMOOTH_TAU_S          ((float)CONFIG_FOLLOW_ROBOT_UWB_SMOOTH_TAU_MS / 1000.0f)
#define FR_UWB_JUMP_MARGIN_M         ((float)CONFIG_FOLLOW_ROBOT_UWB_JUMP_MARGIN_MM / 1000.0f)
#define FR_UWB_OUTLIER_LIMIT         CONFIG_FOLLOW_ROBOT_UWB_OUTLIER_LIMIT
#define FR_UWB_MAX_TARGET_SPEED_MPS  3.0f

/* bool configs with default n need fallback defines (ESP-IDF does not
 * generate #define CONFIG_FOO for bool options set to 'n' in sdkconfig). */
#ifndef CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_INVERT
#define CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_INVERT 0
#endif
#ifndef CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_INVERT
#define CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_INVERT 0
#endif
#ifndef CONFIG_FOLLOW_ROBOT_LEFT_ENC_INVERT
#define CONFIG_FOLLOW_ROBOT_LEFT_ENC_INVERT 0
#endif
#ifndef CONFIG_FOLLOW_ROBOT_RIGHT_ENC_INVERT
#define CONFIG_FOLLOW_ROBOT_RIGHT_ENC_INVERT 0
#endif
#ifndef CONFIG_FOLLOW_ROBOT_UWB_LEFT_IS_POS_X
#define CONFIG_FOLLOW_ROBOT_UWB_LEFT_IS_POS_X 0
#endif
#ifndef CONFIG_FOLLOW_ROBOT_IMU_YAW_INVERT
#define CONFIG_FOLLOW_ROBOT_IMU_YAW_INVERT 0
#endif
#ifndef CONFIG_FOLLOW_ROBOT_HEADING_HOLD
#define CONFIG_FOLLOW_ROBOT_HEADING_HOLD 0
#endif

#define FR_HEADING_HOLD              CONFIG_FOLLOW_ROBOT_HEADING_HOLD

/* ---- Forward declarations ---- */
static const char *state_name(fa_state_t s);

/* ---- Utilities ---- */
static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ----------------------------------------------------- shared snapshot */

typedef enum {
    UWB_FRAME_NONE = 0,
    UWB_FRAME_TWR,
    UWB_FRAME_RANGE_ONLY,
    UWB_FRAME_PARSE_ERROR,
} uwb_frame_type_t;

static const char *uwb_frame_type_name(uwb_frame_type_t t)
{
    switch (t) {
    case UWB_FRAME_TWR:         return "twr";
    case UWB_FRAME_RANGE_ONLY:  return "range_only";
    case UWB_FRAME_PARSE_ERROR: return "parse_error";
    default:                    return "none";
    }
}

/* UWB detailed diagnostics (for telemetry / web UI). */
typedef struct {
    bool valid;
    bool filter_initialized;
    bool last_frame_accepted;
    bool bearing_stale;
    uwb_frame_type_t frame_type;

    int raw_x_cm;
    int raw_y_cm;
    int raw_distance_cm;
    float raw_fwd_m;
    float raw_left_m;
    float raw_range_m;
    float raw_bearing_rad;

    float filt_fwd_m;
    float filt_left_m;
    float filt_range_m;
    float filt_bearing_rad;
    float speed_mps;
    float bearing_rate_rps;

    uint64_t ts_us;
    uint64_t accepted_ts_us;
    uint32_t frame_count;
    uint32_t twr_count;
    uint32_t range_only_count;
    uint32_t parse_error_count;
    uint32_t outlier_count;
    uint32_t consecutive_outliers;
} uwb_debug_t;

typedef struct {
    SemaphoreHandle_t lock;

    /* UWB target */
    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;

    /* UWB diagnostics */
    uwb_debug_t uwb;

    /* Lidar obstacle field (latest complete scan) */
    fa_obstacle_field_t field;
    uint64_t field_ts_us;

    /* Ultrasonics (front corners) */
    float ul_m;
    uint64_t ul_ts_us;
    float ur_m;
    uint64_t ur_ts_us;
} shared_t;

static shared_t g_shared;

static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/* ----------------------------------------------------- Remote safety state machine */

typedef struct {
    bool client_connected;
    bool estop_latched;
    bool motion_armed;
    uint64_t last_heartbeat_us;
    uint64_t first_heartbeat_us;
    uint64_t last_cmd_us;
    uint32_t heartbeat_count;
    uint32_t estop_count;
    uint32_t clear_count;
    uint32_t motion_off_count;
    uint32_t timeout_count;
    uint32_t connect_count;
    uint32_t disconnect_count;
} remote_state_t;

static remote_state_t s_remote = {
    .client_connected = false,
    .estop_latched = true,
    .motion_armed = false,
};
static portMUX_TYPE s_remote_mux = portMUX_INITIALIZER_UNLOCKED;

/* Immediate E-stop flag: set by HTTP handler, polled by control_task each cycle. */
static volatile bool s_estop_pending = false;

static void remote_get_snapshot(remote_state_t *out)
{
    portENTER_CRITICAL(&s_remote_mux);
    *out = s_remote;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_set_client_connected(bool connected, uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.client_connected = connected;
    s_remote.last_cmd_us = t;
    if (connected) {
        s_remote.connect_count++;
    } else {
        s_remote.disconnect_count++;
    }
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_heartbeat(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.last_heartbeat_us = t;
    if (s_remote.first_heartbeat_us == 0) {
        s_remote.first_heartbeat_us = t;
    }
    s_remote.heartbeat_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_estop(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.estop_latched = true;
    s_remote.motion_armed = false;
    s_remote.last_cmd_us = t;
    s_remote.estop_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_clear_and_arm(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.estop_latched = false;
    s_remote.motion_armed = true;
    s_remote.last_heartbeat_us = t;
    if (s_remote.first_heartbeat_us == 0) {
        s_remote.first_heartbeat_us = t;
    }
    s_remote.last_cmd_us = t;
    s_remote.clear_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static void remote_motion_off(uint64_t t)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_remote.motion_armed = false;
    s_remote.last_cmd_us = t;
    s_remote.motion_off_count++;
    portEXIT_CRITICAL(&s_remote_mux);
}

static bool remote_startup_ready(void)
{
    remote_state_t r;
    remote_get_snapshot(&r);
    return r.client_connected && r.first_heartbeat_us != 0;
}

static bool remote_motion_allowed(void)
{
    remote_state_t r;
    remote_get_snapshot(&r);
    return (!r.estop_latched && r.motion_armed);
}

/* ----------------------------------------------------- Telemetry record (live + flash log) */

typedef struct {
    uint64_t t_us;

    /* Remote state */
    bool remote_client_connected;
    bool remote_estop_latched;
    bool remote_motion_armed;
    bool remote_motion_allowed;
    uint32_t remote_hb_age_ms;
    uint32_t remote_hb_count;

    /* Follow state */
    fa_state_t state;
    bool target_valid;
    uint32_t target_age_ms;
    float target_distance_m;
    float target_bearing_rad;

    /* UWB diagnostics */
    uwb_debug_t uwb;

    /* Control output */
    float algo_v_mps;
    float algo_w_rps;
    float cmd_v_mps;
    float cmd_w_rps;
    float meas_v_mps;
    float meas_w_rps;

    /* Chassis targets vs actuals */
    float target_left_mps;
    float target_right_mps;
    float cmd_left_us;
    float cmd_right_us;
    float meas_left_mps;
    float meas_right_mps;

    /* Obstacle avoidance */
    float front_clearance_m;
    bool blocked;

    /* Diagnostics */
    int chassis_update_ret;
    uint32_t log_dropped;
} telemetry_record_t;

static telemetry_record_t s_live;
static portMUX_TYPE s_live_mux = portMUX_INITIALIZER_UNLOCKED;

static void live_set(const telemetry_record_t *rec)
{
    portENTER_CRITICAL(&s_live_mux);
    s_live = *rec;
    portEXIT_CRITICAL(&s_live_mux);
}

static void live_get(telemetry_record_t *out)
{
    portENTER_CRITICAL(&s_live_mux);
    *out = s_live;
    portEXIT_CRITICAL(&s_live_mux);
}

/* ---- Flash log ---- */
static QueueHandle_t s_log_q = NULL;
static uint32_t s_log_dropped = 0;
static volatile bool s_clear_log_requested = false;

static void flash_log_enqueue(const telemetry_record_t *rec)
{
#if FR_FLASH_LOG_ENABLED
    if (!s_log_q) return;
    if (xQueueSend(s_log_q, rec, 0) != pdTRUE) {
        s_log_dropped++;
    }
#else
    (void)rec;
#endif
}

/* ----------------------------------------------------- UWB EMA filtering + task */

static void uwb_publish_twr(int x_cm, int y_cm, int distance_cm)
{
    const uint64_t t = now_us();
    const float left_sign = CONFIG_FOLLOW_ROBOT_UWB_LEFT_IS_POS_X ? 1.0f : -1.0f;
    const float fwd_m = (float)y_cm / 100.0f;
    const float left_m = left_sign * (float)x_cm / 100.0f;
    const float range_m = (distance_cm > 0) ? (float)distance_cm / 100.0f
                                           : sqrtf(fwd_m * fwd_m + left_m * left_m);
    float bearing = 0.0f;
    if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f) {
        bearing = atan2f(left_m, fwd_m);
    }

    lock();
    uwb_debug_t *u = &g_shared.uwb;

    const uint64_t prev_acc_ts = u->accepted_ts_us;
    const float prev_fwd = u->filt_fwd_m;
    const float prev_left = u->filt_left_m;
    const float prev_bearing = u->filt_bearing_rad;
    const bool had_filter = u->filter_initialized;
    float dt = 0.0f;
    if (prev_acc_ts != 0 && t > prev_acc_ts) {
        dt = (float)(t - prev_acc_ts) / 1e6f;
    }

    u->frame_count++;
    u->twr_count++;
    u->frame_type = UWB_FRAME_TWR;
    u->ts_us = t;
    u->raw_x_cm = x_cm;
    u->raw_y_cm = y_cm;
    u->raw_distance_cm = distance_cm;
    u->raw_fwd_m = fwd_m;
    u->raw_left_m = left_m;
    u->raw_range_m = range_m;
    u->raw_bearing_rad = bearing;
    u->bearing_stale = false;

    bool accept = true;
    bool reinit = !had_filter || dt <= 0.0f || dt > 1.5f;

    if (!reinit) {
        const float jump = sqrtf((fwd_m - prev_fwd) * (fwd_m - prev_fwd) +
                                 (left_m - prev_left) * (left_m - prev_left));
        const float max_jump = FR_UWB_MAX_TARGET_SPEED_MPS * dt + FR_UWB_JUMP_MARGIN_M;
        if (jump > max_jump && u->consecutive_outliers < FR_UWB_OUTLIER_LIMIT) {
            accept = false;
        } else if (u->consecutive_outliers >= FR_UWB_OUTLIER_LIMIT) {
            reinit = true;
        }
    }

    u->last_frame_accepted = accept;

    if (accept) {
        float new_fwd = fwd_m;
        float new_left = left_m;

        if (!reinit) {
            float alpha = dt / (FR_UWB_SMOOTH_TAU_S + dt);
            alpha = clampf(alpha, 0.05f, 0.85f);
            new_fwd = prev_fwd + alpha * (fwd_m - prev_fwd);
            new_left = prev_left + alpha * (left_m - prev_left);
        }

        u->filter_initialized = true;
        u->valid = true;
        u->consecutive_outliers = 0;
        u->filt_fwd_m = new_fwd;
        u->filt_left_m = new_left;
        u->filt_range_m = sqrtf(new_fwd * new_fwd + new_left * new_left);
        u->filt_bearing_rad = atan2f(new_left, new_fwd);
        u->accepted_ts_us = t;

        if (had_filter && dt > 1e-3f) {
            const float move = sqrtf((new_fwd - prev_fwd) * (new_fwd - prev_fwd) +
                                     (new_left - prev_left) * (new_left - prev_left));
            u->speed_mps = move / dt;
            u->bearing_rate_rps = fa_wrap_pi(u->filt_bearing_rad - prev_bearing) / dt;
        } else {
            u->speed_mps = 0.0f;
            u->bearing_rate_rps = 0.0f;
        }

        g_shared.tgt_distance_m = u->filt_range_m;
        g_shared.tgt_bearing_rad = u->filt_bearing_rad;
        g_shared.tgt_ts_us = t;
    } else {
        u->outlier_count++;
        u->consecutive_outliers++;
    }

    unlock();
}

static void uwb_publish_range_only(float distance_m)
{
    const uint64_t t = now_us();
    lock();
    uwb_debug_t *u = &g_shared.uwb;
    u->frame_count++;
    u->range_only_count++;
    u->frame_type = UWB_FRAME_RANGE_ONLY;
    u->ts_us = t;
    u->raw_distance_cm = (int)(distance_m * 100.0f);
    u->raw_range_m = distance_m;
    u->bearing_stale = true;
    u->last_frame_accepted = true;
    /* Range-only: log but do NOT refresh the control target. */
    unlock();
}

static void uwb_publish_parse_error(void)
{
    lock();
    g_shared.uwb.frame_count++;
    g_shared.uwb.parse_error_count++;
    g_shared.uwb.frame_type = UWB_FRAME_PARSE_ERROR;
    g_shared.uwb.ts_us = now_us();
    g_shared.uwb.last_frame_accepted = false;
    unlock();
}

static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];
    int cons_parse_errors = 0;

    while (1) {
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) {
            vTaskDelay(1);
            continue;
        }

        bu_uwb_twr_reading_t twr = {0};
        bu_uwb_distance_t dist = {0};

        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            cons_parse_errors = 0;
            uwb_publish_twr(twr.x_cm, twr.y_cm, twr.distance_cm);
        } else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            cons_parse_errors = 0;
            uwb_publish_range_only(dist.distance_m);
        } else {
            uwb_publish_parse_error();
            if (++cons_parse_errors >= 10) {
                cons_parse_errors = 0;
                vTaskDelay(1);
            }
        }
    }
}

/* ----------------------------------------------------- Lidar obstacle field */

static float lidar_angle_to_body_rad(float raw_deg)
{
    float rel = raw_deg - (float)CONFIG_FOLLOW_ROBOT_LIDAR_FORWARD_DEG;
    rel = -rel; /* lidar CW -> body CCW-positive */
    while (rel > 180.0f) {
        rel -= 360.0f;
    }
    while (rel < -180.0f) {
        rel += 360.0f;
    }
    return DEG2RAD(rel);
}

static void lidar_task(void *arg)
{
    rplidar_c1_t *lidar = (rplidar_c1_t *)arg;
    fa_obstacle_field_t work;
    fa_obstacle_reset(&work, LIDAR_SECTORS, LIDAR_FOV_RAD);

    while (1) {
        rplidar_c1_point_t p = {0};
        if (!rplidar_c1_read_point(lidar, &p)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (p.start_bit) {
            /* A new revolution begins: publish the scan we just finished. */
            lock();
            g_shared.field = work;
            g_shared.field_ts_us = now_us();
            unlock();
            fa_obstacle_reset(&work, LIDAR_SECTORS, LIDAR_FOV_RAD);
        }
        if (p.distance_mm > 0.0f && p.quality > 0) {
            /* 过滤无效角度，仅保留 0~65 度 和 295~360 度的点 */
            if ((p.angle_deg >= 0.0f && p.angle_deg <= 65.0f) ||
                (p.angle_deg >= 295.0f && p.angle_deg <= 360.0f)) {

                const float body = lidar_angle_to_body_rad(p.angle_deg);
                fa_obstacle_add(&work, body, p.distance_mm / 1000.0f);
            }
        }
    }
}

/* ----------------------------------------------------- Ultrasonics */

typedef struct {
    a02yyuw_t *dev;
    bool is_left;
} ultra_arg_t;

static void ultra_task(void *arg)
{
    ultra_arg_t *ua = (ultra_arg_t *)arg;
    while (1) {
        a02yyuw_reading_t r = {0};
        if (a02yyuw_read_dev(ua->dev, &r, 120) == ESP_OK && r.valid) {
            const float m = (float)r.distance_mm / 1000.0f;
            lock();
            if (ua->is_left) {
                g_shared.ul_m = m;
                g_shared.ul_ts_us = now_us();
            } else {
                g_shared.ur_m = m;
                g_shared.ur_ts_us = now_us();
            }
            unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ----------------------------------------------------- IMU (heading loop) */
static imu_i2c_t s_imu;
static bool s_imu_ok = false;

/* Read the IMU yaw (rad, CCW-positive after sign fix). Returns false if no
 * trustworthy reading is available this cycle. */
static bool imu_read_yaw(float *yaw_rad)
{
    if (!s_imu_ok) {
        return false;
    }
    imu_i2c_reading_t r;
    memset(&r, 0, sizeof(r));
    if (imu_i2c_read_all(&s_imu, &r) != ESP_OK || !r.valid) {
        return false;
    }
    *yaw_rad = (CONFIG_FOLLOW_ROBOT_IMU_YAW_INVERT ? -1.0f : 1.0f) * DEG2RAD(r.euler_deg[2]);
    return true;
}

/* ----------------------------------------------------- HTTP server + embedded web dashboard */

static httpd_handle_t s_httpd = NULL;

static esp_err_t http_send_text(httpd_req_t *req, const char *text, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

/* Embedded HTML dashboard — phone-optimized single-page app.
 * Shows real-time telemetry: UWB target, lidar clearance, chassis speeds,
 * obstacle-avoidance state, remote safety status. */
static const char s_index_html[] =
"<!doctype html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>Follow Robot</title>"
"<style>"
"*{box-sizing:border-box;-webkit-user-select:none;user-select:none;}"
"body{font-family:Arial,sans-serif;background:#101214;color:#eee;margin:0;padding:12px;}"
"h2{text-align:center;margin:8px 0;font-size:20px;}"
"button{font-size:18px;padding:12px 16px;margin:4px;border:0;border-radius:10px;touch-action:manipulation;}"
"#stop{background:#d00000;color:white;width:100%;height:72px;font-size:32px;font-weight:bold;margin:8px 0;}"
"#arm{background:#008f5a;color:white;}#off{background:#555;color:white;}#clearlog{background:#7655d9;color:white;}"
"#ctl{display:flex;gap:6px;justify-content:center;flex-wrap:wrap;margin:8px 0;}"
"#summary{background:#18202a;border:1px solid #38506a;border-radius:10px;padding:10px;margin:8px 0;font-size:15px;line-height:1.5;}"
"#summary b{font-size:22px;color:#8cc8ff;}"
"#info{background:#18202a;border:1px solid #38506a;border-radius:10px;padding:10px;margin:8px 0;font-size:13px;line-height:1.4;}"
"#err{color:#f66;font-weight:bold;font-size:14px;min-height:1.2em;text-align:center;}"
".warn{color:#ffd166;font-size:13px;}"
"a{color:#8cc8ff;font-size:16px;}"
"</style></head><body>"
"<h2>Follow Robot — 算法2+Web</h2>"
"<p class='warn'>STOP latches E-STOP. CLEAR/ARM enables motion. Page must stay open.</p>"
"<button id='stop' onclick='doStop()'>STOP</button>"
"<div id='ctl'>"
"<button id='arm' onclick='doArm()'>CLEAR / ARM</button>"
"<button id='off' onclick='doOff()'>MOTION OFF</button>"
"<button id='clearlog' onclick='doClearLog()'>CLEAR LOG</button>"
"<a href='/log' target='_blank'>CSV</a>"
"</div>"
"<div id='err'></div>"
"<div id='summary'>waiting for live data...</div>"
"<div id='info'></div>"
"<script>"
"function f(x,d){return (x===undefined||x===null||!isFinite(Number(x)))?'--':Number(x).toFixed(d);}"
"function render(j){"
"let r=j.remote||{},t=j.target||{},u=j.uwb||{},c=j.control||{},ch=j.chassis||{},oa=j.obstacle||{};"
"document.getElementById('err').textContent='';"
"document.getElementById('summary').innerHTML="
"'<b>'+f(t.distance_m,2)+' m</b> &nbsp; '+f(t.bearing_deg,1)+'&deg; &nbsp; state=<b>'+j.state+'</b><br>' +"
"'UWB filt: d='+f(u.filtered_distance_m,2)+'m brg='+f(u.bearing_deg,1)+'&deg; raw: '+f(u.raw_distance_m,2)+'m '+f(u.raw_bearing_deg,1)+'&deg;<br>' +"
"'target valid='+t.valid+' age='+t.age_ms+'ms frame='+u.frame_type+' outliers='+u.outlier_count;"
"document.getElementById('info').innerHTML="
"'cmd v='+f(c.cmd_v_mps,2)+' w='+f(c.cmd_w_rps,2)+' | meas v='+f(c.meas_v_mps,2)+' w='+f(c.meas_w_rps,2)+'<br>' +"
"'clearance='+f(oa.front_clearance_m,2)+'m blocked='+oa.blocked+' | L='+f(ch.meas_left_mps,2)+' R='+f(ch.meas_right_mps,2)+' m/s<br>' +"
"'Lpulse='+f(ch.cmd_left_us,1)+'us Rpulse='+f(ch.cmd_right_us,1)+'us | armed='+r.motion_armed+' ok='+r.motion_allowed+' hb='+r.hb_age_ms+'ms';"
"}"
"async function ft(u,o,m){let c=new AbortController(),t=setTimeout(function(){c.abort()},m);return fetch(u,Object.assign({},o,{signal:c.signal})).finally(function(){clearTimeout(t)});}"
"async function post(p){try{let r=await ft(p,{method:'POST',cache:'no-store'},3000);if(!r.ok)throw new Error(r.status);return true;}catch(e){document.getElementById('err').textContent='CMD FAIL: '+p+' ('+e.message+')';return false;}}"
"async function doStop(){if(await post('/estop'))await poll();}"
"async function doArm(){if(await post('/clear'))await poll();}"
"async function doOff(){if(await post('/motion_off'))await poll();}"
"async function doClearLog(){if(await post('/clear_log'))await poll();}"
"var pollBusy=false,nextPollDelay=400;"
"async function poll(){"
"if(pollBusy)return;pollBusy=true;"
"try{let r=await ft('/live',{cache:'no-store'},3000);"
"if(!r.ok)throw new Error('HTTP '+r.status);"
"let j=JSON.parse(await r.text());render(j);nextPollDelay=400;"
"}catch(e){if(e.name!=='AbortError'){document.getElementById('summary').textContent='连接中断...';nextPollDelay=800;}}"
"finally{pollBusy=false;setTimeout(poll,nextPollDelay);}}"
"setInterval(function(){ft('/hb',{method:'POST',cache:'no-store'},2000).catch(function(){});},500);"
"poll();"
"</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    return http_send_text(req, s_index_html, "text/html");
}

static esp_err_t hb_handler(httpd_req_t *req)
{
    remote_heartbeat(now_us());
    return http_send_text(req, "OK\n", "text/plain");
}

static esp_err_t estop_handler(httpd_req_t *req)
{
    remote_estop(now_us());
    s_estop_pending = true;
    ESP_LOGW(TAG, "REMOTE E-STOP latched");
    return http_send_text(req, "ESTOP\n", "text/plain");
}

static esp_err_t clear_handler(httpd_req_t *req)
{
    remote_clear_and_arm(now_us());
    ESP_LOGW(TAG, "REMOTE E-STOP cleared; motion armed");
    return http_send_text(req, "ARMED\n", "text/plain");
}

static esp_err_t motion_off_handler(httpd_req_t *req)
{
    remote_motion_off(now_us());
    ESP_LOGW(TAG, "motion armed flag cleared");
    return http_send_text(req, "MOTION_OFF\n", "text/plain");
}

static esp_err_t clear_log_handler(httpd_req_t *req)
{
    s_clear_log_requested = true;
    return http_send_text(req, "CLEAR_LOG_REQUESTED\n", "text/plain");
}

static esp_err_t live_handler(httpd_req_t *req)
{
    const uint64_t t = now_us();

    telemetry_record_t rec;
    remote_state_t r;
    live_get(&rec);
    remote_get_snapshot(&r);

    uint32_t hb_age_ms = 0xffffffffu;
    if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
        hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
    }

    char buf[FR_LIVE_JSON_BUF_SIZE];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"remote\":{\"connected\":%s,\"estop\":%s,\"armed\":%s,\"motion_allowed\":%s,"
        "\"hb_age_ms\":%lu,\"hb_count\":%lu,\"connect_count\":%lu},"
        "\"target\":{\"valid\":%s,\"age_ms\":%lu,\"distance_m\":%.3f,\"bearing_rad\":%.4f,\"bearing_deg\":%.2f},"
        "\"uwb\":{"
        "\"frame_type\":\"%s\",\"valid\":%s,\"accepted\":%s,\"stale_bearing\":%s,"
        "\"raw_x_cm\":%d,\"raw_y_cm\":%d,\"raw_distance_cm\":%d,"
        "\"raw_fwd_m\":%.3f,\"raw_left_m\":%.3f,\"raw_range_m\":%.3f,\"raw_bearing_deg\":%.2f,"
        "\"filt_fwd_m\":%.3f,\"filt_left_m\":%.3f,\"filtered_distance_m\":%.3f,\"bearing_deg\":%.2f,"
        "\"speed_mps\":%.3f,\"bearing_rate_rps\":%.3f,"
        "\"frame_count\":%lu,\"twr_count\":%lu,\"range_only_count\":%lu,\"parse_error_count\":%lu,\"outlier_count\":%lu"
        "},"
        "\"control\":{\"algo_v_mps\":%.3f,\"algo_w_rps\":%.3f,\"cmd_v_mps\":%.3f,\"cmd_w_rps\":%.3f,"
        "\"meas_v_mps\":%.3f,\"meas_w_rps\":%.3f},"
        "\"chassis\":{\"target_left_mps\":%.3f,\"target_right_mps\":%.3f,"
        "\"cmd_left_us\":%.1f,\"cmd_right_us\":%.1f,"
        "\"meas_left_mps\":%.3f,\"meas_right_mps\":%.3f},"
        "\"obstacle\":{\"front_clearance_m\":%.3f,\"blocked\":%s},"
        "\"state\":\"%s\","
        "\"log\":{\"dropped\":%lu}"
        "}\n",
        r.client_connected ? "true" : "false",
        r.estop_latched ? "true" : "false",
        r.motion_armed ? "true" : "false",
        rec.remote_motion_allowed ? "true" : "false",
        (unsigned long)hb_age_ms, (unsigned long)r.heartbeat_count,
        (unsigned long)r.connect_count,
        rec.target_valid ? "true" : "false",
        (unsigned long)rec.target_age_ms,
        rec.target_distance_m, rec.target_bearing_rad, RAD2DEG(rec.target_bearing_rad),
        uwb_frame_type_name(rec.uwb.frame_type),
        rec.uwb.valid ? "true" : "false",
        rec.uwb.last_frame_accepted ? "true" : "false",
        rec.uwb.bearing_stale ? "true" : "false",
        rec.uwb.raw_x_cm, rec.uwb.raw_y_cm, rec.uwb.raw_distance_cm,
        rec.uwb.raw_fwd_m, rec.uwb.raw_left_m, rec.uwb.raw_range_m,
        RAD2DEG(rec.uwb.raw_bearing_rad),
        rec.uwb.filt_fwd_m, rec.uwb.filt_left_m, rec.uwb.filt_range_m,
        RAD2DEG(rec.uwb.filt_bearing_rad),
        rec.uwb.speed_mps, rec.uwb.bearing_rate_rps,
        (unsigned long)rec.uwb.frame_count, (unsigned long)rec.uwb.twr_count,
        (unsigned long)rec.uwb.range_only_count, (unsigned long)rec.uwb.parse_error_count,
        (unsigned long)rec.uwb.outlier_count,
        rec.algo_v_mps, rec.algo_w_rps, rec.cmd_v_mps, rec.cmd_w_rps,
        rec.meas_v_mps, rec.meas_w_rps,
        rec.target_left_mps, rec.target_right_mps,
        rec.cmd_left_us, rec.cmd_right_us,
        rec.meas_left_mps, rec.meas_right_mps,
        rec.front_clearance_m, rec.blocked ? "true" : "false",
        state_name(rec.state),
        (unsigned long)rec.log_dropped);

    if (n < 0 || n >= (int)sizeof(buf)) {
        return http_send_text(req, "{\"error\":\"truncated\"}\n", "application/json");
    }

    return http_send_text(req, buf, "application/json");
}

static esp_err_t log_download_handler(httpd_req_t *req)
{
    FILE *f = fopen(FR_LOG_PATH, "r");
    if (!f) return http_send_text(req, "log file not available\n", "text/plain");

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=follow_log.csv");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char chunk[768];
    while (!feof(f)) {
        size_t n = fread(chunk, 1, sizeof(chunk), f);
        if (n > 0 && httpd_resp_send_chunk(req, chunk, n) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Captive portal: Android detects internet access. */
static esp_err_t captive_portal_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* 404: redirect all unknown URIs to root (captive portal fallback). */
static esp_err_t notfound_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    return http_send_text(req, s_index_html, "text/html");
}

static esp_err_t http_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;
    config.stack_size = 12288;
    config.max_uri_handlers = 16;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;

    esp_err_t ret = httpd_start(&s_httpd, &config);
    if (ret != ESP_OK) return ret;

    /* 404 handler catches captive portal probes. */
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, notfound_handler);

    httpd_uri_t u;
    memset(&u, 0, sizeof(u));

    /* Captive portal detection endpoints. */
    const char *portal_uris[] = {"/generate_204", "/gen_204", "/ncsi.txt", "/hotspot-detect.html"};
    for (int i = 0; i < 4; i++) {
        u.uri = portal_uris[i]; u.method = HTTP_GET; u.handler = captive_portal_handler;
        httpd_register_uri_handler(s_httpd, &u);
    }

    u.uri = "/";            u.method = HTTP_GET;  u.handler = index_handler;          httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/hb";          u.method = HTTP_POST; u.handler = hb_handler;             httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/estop";       u.method = HTTP_POST; u.handler = estop_handler;          httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/clear";       u.method = HTTP_POST; u.handler = clear_handler;          httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/motion_off";  u.method = HTTP_POST; u.handler = motion_off_handler;     httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/live";        u.method = HTTP_GET;  u.handler = live_handler;           httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/status";      u.method = HTTP_GET;  u.handler = live_handler;           httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/log";         u.method = HTTP_GET;  u.handler = log_download_handler;   httpd_register_uri_handler(s_httpd, &u);
    u.uri = "/clear_log";   u.method = HTTP_POST; u.handler = clear_log_handler;      httpd_register_uri_handler(s_httpd, &u);

    ESP_LOGI(TAG, "HTTP ready: http://192.168.4.1");
    return ESP_OK;
}

/* ----------------------------------------------------- WiFi SoftAP */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base != WIFI_EVENT) return;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "client connected AID=%d", e->aid);
        remote_set_client_connected(true, now_us());
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGW(TAG, "client disconnected AID=%d", e->aid);
        remote_set_client_connected(false, now_us());
    }
}

static esp_err_t wifi_start_softap(void)
{
#if !FR_WIFI_ENABLED
    ESP_LOGW(TAG, "WiFi remote control disabled in Kconfig");
    return ESP_OK;
#else
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) return ret;

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) return ret;

    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    strncpy((char *)wc.ap.ssid, FR_WIFI_AP_SSID, sizeof(wc.ap.ssid) - 1);
    strncpy((char *)wc.ap.password, FR_WIFI_AP_PASS, sizeof(wc.ap.password) - 1);
    wc.ap.ssid_len = strlen(FR_WIFI_AP_SSID);
    wc.ap.channel = FR_WIFI_AP_CHANNEL;
    wc.ap.max_connection = FR_WIFI_AP_MAX_CONN;
    wc.ap.authmode = (strlen(FR_WIFI_AP_PASS) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wc.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP: SSID=%s URL=http://192.168.4.1", FR_WIFI_AP_SSID);
    return http_start();
#endif
}

static void wifi_wait_for_client(void)
{
#if FR_WIFI_ENABLED && FR_WIFI_REQUIRE_CLIENT
    uint32_t last_log_ms = 0;
    while (!remote_startup_ready()) {
        uint32_t now_ms = (uint32_t)(now_us() / 1000ULL);
        if (now_ms - last_log_ms >= 1000) {
            last_log_ms = now_ms;
            ESP_LOGW(TAG, "waiting for phone client + heartbeat before bring-up...");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "phone connected and heartbeat received");
#else
    (void)0;
#endif
}

/* ----------------------------------------------------- Flash CSV log task */

static void flash_log_write_header(FILE *f)
{
    fprintf(f,
        "t_ms,remote_client,estop,armed,ok,hb_age_ms,"
        "state,target_valid,target_age_ms,"
        "uwb_frame,uwb_valid,uwb_raw_x_cm,uwb_raw_y_cm,uwb_raw_dist_cm,"
        "raw_fwd_m,raw_left_m,raw_range_m,raw_bearing_deg,"
        "filt_fwd_m,filt_left_m,filt_range_m,filt_bearing_deg,"
        "uwb_speed_mps,uwb_frame_count,uwb_twr_count,uwb_outlier_count,"
        "algo_v_mps,algo_w_rps,cmd_v_mps,cmd_w_rps,"
        "meas_v_mps,meas_w_rps,"
        "target_left_mps,target_right_mps,cmd_left_us,cmd_right_us,"
        "meas_left_mps,meas_right_mps,"
        "front_clearance_m,blocked,"
        "chassis_ret,log_dropped\n");
}

static FILE *flash_log_open_append(void)
{
    struct stat st;
    bool need_header = (stat(FR_LOG_PATH, &st) != 0) || (st.st_size == 0);
    FILE *f = fopen(FR_LOG_PATH, "a");
    if (!f) { ESP_LOGE(TAG, "open log: %s", strerror(errno)); return NULL; }
    if (need_header) { flash_log_write_header(f); fflush(f); }
    return f;
}

static void flash_log_task(void *arg)
{
    (void)arg;
    FILE *f = flash_log_open_append();
    telemetry_record_t rec;
    int flush_div = 0;

    while (1) {
        if (s_clear_log_requested) {
            s_clear_log_requested = false;
            if (f) { fclose(f); f = NULL; }
            remove(FR_LOG_PATH);
            f = flash_log_open_append();
        }
        if (xQueueReceive(s_log_q, &rec, pdMS_TO_TICKS(500)) != pdTRUE) {
            if (f) fflush(f);
            continue;
        }
        if (!f) { f = flash_log_open_append(); if (!f) continue; }

        fprintf(f,
            "%llu,%d,%d,%d,%d,%lu,"
            "%s,%d,%lu,"
            "%s,%d,%d,%d,%d,"
            "%.3f,%.3f,%.3f,%.2f,"
            "%.3f,%.3f,%.3f,%.2f,"
            "%.3f,%lu,%lu,%lu,"
            "%.3f,%.3f,%.3f,%.3f,"
            "%.3f,%.3f,"
            "%.3f,%.3f,%.1f,%.1f,"
            "%.3f,%.3f,"
            "%.3f,%d,"
            "%d,%lu\n",
            (unsigned long long)(rec.t_us / 1000ULL),
            rec.remote_client_connected, rec.remote_estop_latched,
            rec.remote_motion_armed, rec.remote_motion_allowed,
            (unsigned long)rec.remote_hb_age_ms,
            state_name(rec.state), rec.target_valid, (unsigned long)rec.target_age_ms,
            uwb_frame_type_name(rec.uwb.frame_type), rec.uwb.valid,
            rec.uwb.raw_x_cm, rec.uwb.raw_y_cm, rec.uwb.raw_distance_cm,
            rec.uwb.raw_fwd_m, rec.uwb.raw_left_m, rec.uwb.raw_range_m,
            RAD2DEG(rec.uwb.raw_bearing_rad),
            rec.uwb.filt_fwd_m, rec.uwb.filt_left_m, rec.uwb.filt_range_m,
            RAD2DEG(rec.uwb.filt_bearing_rad),
            rec.uwb.speed_mps,
            (unsigned long)rec.uwb.frame_count, (unsigned long)rec.uwb.twr_count,
            (unsigned long)rec.uwb.outlier_count,
            rec.algo_v_mps, rec.algo_w_rps, rec.cmd_v_mps, rec.cmd_w_rps,
            rec.meas_v_mps, rec.meas_w_rps,
            rec.target_left_mps, rec.target_right_mps,
            rec.cmd_left_us, rec.cmd_right_us,
            rec.meas_left_mps, rec.meas_right_mps,
            rec.front_clearance_m, rec.blocked,
            rec.chassis_update_ret,
            (unsigned long)rec.log_dropped);

        if (++flush_div >= FR_LOG_HZ) { flush_div = 0; fflush(f); }
    }
}

static esp_err_t flash_log_start(void)
{
#if !FR_FLASH_LOG_ENABLED
    return ESP_OK;
#else
    esp_vfs_spiffs_conf_t conf = {
        .base_path = FR_LOG_BASE_PATH,
        .partition_label = FR_LOG_PARTITION_LABEL,
        .max_files = 4,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info(FR_LOG_PARTITION_LABEL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%u used=%u", (unsigned)total, (unsigned)used);

    s_log_q = xQueueCreate(FR_LOG_QUEUE_LEN, sizeof(telemetry_record_t));
    if (!s_log_q) return ESP_ERR_NO_MEM;
    xTaskCreate(flash_log_task, "flash_log", 6144, NULL, 2, NULL);
    return ESP_OK;
#endif
}

/* ----------------------------------------------------- Control loop */

static fa_config_t build_fa_config(void)
{
    fa_config_t c = fa_default_config();
    c.follow_distance_m = CONFIG_FOLLOW_ROBOT_FOLLOW_DISTANCE_MM / 1000.0f;
    c.stop_band_m = CONFIG_FOLLOW_ROBOT_STOP_BAND_MM / 1000.0f;
    c.max_linear_mps = CONFIG_FOLLOW_ROBOT_MAX_LINEAR_MMPS / 1000.0f;
    c.max_angular_rps = CONFIG_FOLLOW_ROBOT_MAX_ANGULAR_MRADPS / 1000.0f;
    c.emergency_distance_m = CONFIG_FOLLOW_ROBOT_EMERGENCY_DIST_MM / 1000.0f;
    c.slow_distance_m = CONFIG_FOLLOW_ROBOT_SLOW_DIST_MM / 1000.0f;
    c.safe_distance_m = CONFIG_FOLLOW_ROBOT_SAFE_DIST_MM / 1000.0f;
    c.robot_half_width_m = CONFIG_FOLLOW_ROBOT_ROBOT_HALF_WIDTH_MM / 1000.0f;
    return c;
}

static chassis_config_t build_chassis_config(void)
{
    chassis_config_t cc = chassis_default_config();
    cc.esc_min_us = CONFIG_FOLLOW_ROBOT_ESC_MIN_US;
    cc.esc_mid_us = CONFIG_FOLLOW_ROBOT_ESC_MID_US;
    cc.esc_max_us = CONFIG_FOLLOW_ROBOT_ESC_MAX_US;
    cc.left_esc_gpio = CONFIG_FOLLOW_ROBOT_LEFT_ESC_GPIO;
    cc.right_esc_gpio = CONFIG_FOLLOW_ROBOT_RIGHT_ESC_GPIO;
    cc.left_invert = CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_INVERT;
    cc.right_invert = CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_INVERT;

    cc.left_enc_a_gpio = CONFIG_FOLLOW_ROBOT_LEFT_ENC_A_GPIO;
    cc.left_enc_b_gpio = CONFIG_FOLLOW_ROBOT_LEFT_ENC_B_GPIO;
    cc.right_enc_a_gpio = CONFIG_FOLLOW_ROBOT_RIGHT_ENC_A_GPIO;
    cc.right_enc_b_gpio = CONFIG_FOLLOW_ROBOT_RIGHT_ENC_B_GPIO;
    cc.left_enc_invert = CONFIG_FOLLOW_ROBOT_LEFT_ENC_INVERT;
    cc.right_enc_invert = CONFIG_FOLLOW_ROBOT_RIGHT_ENC_INVERT;
    cc.ticks_per_meter = (float)CONFIG_FOLLOW_ROBOT_TICKS_PER_METER;

    cc.track_width_m = CONFIG_FOLLOW_ROBOT_TRACK_WIDTH_MM / 1000.0f;
    cc.max_speed_mps = CONFIG_FOLLOW_ROBOT_MAX_WHEEL_SPEED_MMPS / 1000.0f;

    cc.kp = (float)CONFIG_FOLLOW_ROBOT_SPEED_KP;
    cc.ki = (float)CONFIG_FOLLOW_ROBOT_SPEED_KI;
    cc.kd = (float)CONFIG_FOLLOW_ROBOT_SPEED_KD;
    cc.pid_out_limit_us = (float)CONFIG_FOLLOW_ROBOT_SPEED_PID_LIMIT_US;
    return cc;
}

static const char *state_name(fa_state_t s)
{
    switch (s) {
    case FA_STATE_IDLE:   return "IDLE";
    case FA_STATE_SEARCH: return "SEARCH";
    case FA_STATE_FOLLOW: return "FOLLOW";
    case FA_STATE_AVOID:  return "AVOID";
    case FA_STATE_ESTOP:  return "ESTOP";
    default:              return "?";
    }
}

static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;

    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa.cfg = build_fa_config();
    const float max_omega = fa.cfg.max_angular_rps;
    const float heading_kp = CONFIG_FOLLOW_ROBOT_HEADING_KP_MILLI / 1000.0f;
    const bool heading_hold = CONFIG_FOLLOW_ROBOT_HEADING_HOLD;
    float yaw_ref = 0.0f;       /* IMU heading reference (rad) */
    bool yaw_ref_set = false;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ROBOT_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = now_us();
    int log_div = 0;
    int flash_log_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        const float dt = (float)(t - prev_us) / 1e6f;
        prev_us = t;

        /* ---- [新增] Fast-path E-STOP: bypass all sensor/algorithm logic ---- */
        if (s_estop_pending) {
            s_estop_pending = false;
            chassis_set_velocity(chassis, 0.0f, 0.0f);
            chassis_update(chassis, dt);
            yaw_ref_set = false;
            continue;   /* skip the rest of this cycle */
        }

        /* Snapshot shared sensor data under the lock. */
        fa_target_t target = {0};
        fa_obstacle_field_t field;
        fa_range_t ul = {0};
        fa_range_t ur = {0};
        bool have_field;
        uwb_debug_t uwb_diag;

        lock();
        target.valid = (t - g_shared.tgt_ts_us) < TARGET_FRESH_US;
        target.distance_m = g_shared.tgt_distance_m;
        target.bearing_rad = g_shared.tgt_bearing_rad;

        have_field = (t - g_shared.field_ts_us) < FIELD_FRESH_US;
        field = g_shared.field;

        ul.valid = (t - g_shared.ul_ts_us) < ULTRA_FRESH_US;
        ul.dist_m = g_shared.ul_m;
        ur.valid = (t - g_shared.ur_ts_us) < ULTRA_FRESH_US;
        ur.dist_m = g_shared.ur_m;

        uwb_diag = g_shared.uwb;
        unlock();

        fa_output_t out = fa_update(&fa, &target,
                                     have_field ? &field : NULL, &ul, &ur, dt);

        /* --- IMU heading closed-loop ------------------------------------ */
        float omega_cmd = out.omega_rps;
        const bool tracking =
            (out.state == FA_STATE_FOLLOW || out.state == FA_STATE_AVOID);
        float yaw_meas;
        if (heading_hold && tracking && imu_read_yaw(&yaw_meas)) {
            if (!yaw_ref_set) {
                yaw_ref = yaw_meas;
                yaw_ref_set = true;
            }
            yaw_ref = fa_wrap_pi(yaw_ref + out.omega_rps * dt);
            const float err = fa_wrap_pi(yaw_ref - yaw_meas);
            omega_cmd = out.omega_rps + heading_kp * err;
            if (omega_cmd > max_omega) {
                omega_cmd = max_omega;
            } else if (omega_cmd < -max_omega) {
                omega_cmd = -max_omega;
            }
        } else {
            yaw_ref_set = false;
        }

        /* ---- [新增] Remote safety gate: E-STOP latch or not armed → force zero ---- */
        bool motion_allowed = true;
#if FR_WIFI_ENABLED
        motion_allowed = remote_motion_allowed();
#endif
        if (!motion_allowed) {
            out.v_mps = 0.0f;
            omega_cmd = 0.0f;
            yaw_ref_set = false;
        }

        chassis_set_velocity(chassis, out.v_mps, omega_cmd);
        int update_ret = (int)chassis_update(chassis, dt);

        /* --- Telemetry --- */
        float mv = 0.0f, mw = 0.0f;
        float ml = 0.0f, mr = 0.0f;
        chassis_get_measured(chassis, &mv, &mw, &ml, &mr);

        uint32_t tgt_age_ms = 0xffffffffu;
        if (g_shared.tgt_ts_us != 0 && t >= g_shared.tgt_ts_us) {
            tgt_age_ms = (uint32_t)((t - g_shared.tgt_ts_us) / 1000ULL);
        }

        remote_state_t r;
        remote_get_snapshot(&r);
        uint32_t hb_age_ms = 0xffffffffu;
        if (r.last_heartbeat_us != 0 && t >= r.last_heartbeat_us) {
            hb_age_ms = (uint32_t)((t - r.last_heartbeat_us) / 1000ULL);
        }

        /* Build telemetry record. */
        telemetry_record_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.t_us = t;
        rec.remote_client_connected = r.client_connected;
        rec.remote_estop_latched = r.estop_latched;
        rec.remote_motion_armed = r.motion_armed;
        rec.remote_motion_allowed = motion_allowed;
        rec.remote_hb_age_ms = hb_age_ms;
        rec.remote_hb_count = r.heartbeat_count;
        rec.state = out.state;
        rec.target_valid = target.valid;
        rec.target_age_ms = tgt_age_ms;
        rec.target_distance_m = target.distance_m;
        rec.target_bearing_rad = target.bearing_rad;
        rec.uwb = uwb_diag;
        rec.algo_v_mps = out.v_mps;
        rec.algo_w_rps = out.omega_rps;
        rec.cmd_v_mps = out.v_mps;
        rec.cmd_w_rps = omega_cmd;
        rec.meas_v_mps = mv;
        rec.meas_w_rps = mw;
        rec.meas_left_mps = ml;
        rec.meas_right_mps = mr;
        rec.front_clearance_m = out.front_clearance_m;
        rec.blocked = out.blocked;
        rec.chassis_update_ret = update_ret;
        rec.log_dropped = s_log_dropped;
        /* Chassis targets (populated by chassis internals via get_measured only;
         * for now these are zero unless we add a get_target API to chassis.) */
        rec.target_left_mps = 0.0f;
        rec.target_right_mps = 0.0f;
        rec.cmd_left_us = 0.0f;
        rec.cmd_right_us = 0.0f;

        live_set(&rec);

#if FR_FLASH_LOG_ENABLED
        if (++flash_log_div >= CONFIG_FOLLOW_ROBOT_CONTROL_HZ / FR_LOG_HZ) {
            flash_log_div = 0;
            flash_log_enqueue(&rec);
        }
#endif

        /* Serial status log (~5 Hz) */
        if (++log_div >= CONFIG_FOLLOW_ROBOT_CONTROL_HZ / 5) {
            log_div = 0;
            ESP_LOGI(TAG,
                     "%-6s tgt=%s d=%.2f br=%+.2f | clr=%.2f blk=%d | "
                     "cmd v=%+.2f w=%+.2f | meas v=%+.2f w=%+.2f | "
                     "armed=%d ok=%d uwb_ok=%d",
                     state_name(out.state), target.valid ? "Y" : "N",
                     target.distance_m, target.bearing_rad,
                     out.front_clearance_m, out.blocked, out.v_mps, omega_cmd,
                     mv, mw,
                     r.motion_armed, motion_allowed, uwb_diag.valid);
        }
    }
}

/* ----------------------------------------------------- bring-up */

static rplidar_c1_t s_lidar;
static a02yyuw_t s_ultra_left;
static a02yyuw_t s_ultra_right;
static chassis_t s_chassis;
static ultra_arg_t s_ua_left = {.dev = &s_ultra_left, .is_left = true};
static ultra_arg_t s_ua_right = {.dev = &s_ultra_right, .is_left = false};

void app_main(void)
{
    ESP_LOGI(TAG, "Follow-me suitcase (算法2+Web, closed-loop + WiFi dashboard) starting");

    /* --- 1. 全局共享状态初始化 --- */
    memset(&g_shared, 0, sizeof(g_shared));
    g_shared.lock = xSemaphoreCreateMutex();
    fa_obstacle_reset(&g_shared.field, LIDAR_SECTORS, LIDAR_FOV_RAD);

    /* --- 2. [新增] WiFi SoftAP + HTTP Server 启动 --- */
#if FR_WIFI_ENABLED
    if (wifi_start_softap() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi/HTTP start FAILED; continuing without remote control");
    }
#if FR_WIFI_REQUIRE_CLIENT
    wifi_wait_for_client();
#endif
    /* [新增] SPIFFS Flash CSV log (optional). */
    if (flash_log_start() != ESP_OK) {
        ESP_LOGE(TAG, "flash log disabled");
    }
#endif

    /* --- 3. 底盘系统初始化 (ESC电调 + 编码器闭环) --- */
    chassis_config_t cc = build_chassis_config();
    if (chassis_init(&s_chassis, &cc) == ESP_OK) {
        chassis_stop(&s_chassis);
        ESP_LOGI(TAG, "chassis ready; arming ESC (hold neutral %d ms)",
                 CONFIG_FOLLOW_ROBOT_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FOLLOW_ROBOT_ESC_ARM_MS));
    } else {
        ESP_LOGE(TAG, "chassis init FAILED - check ESC/encoder GPIOs");
    }

    /* --- 4. UWB (超宽带) 目标定位模块初始化 --- */
    bu_uwb_config_t bu = bu_uwb_default_config(
        (uart_port_t)CONFIG_FOLLOW_ROBOT_UWB_UART,
        CONFIG_FOLLOW_ROBOT_UWB_RX_GPIO, CONFIG_FOLLOW_ROBOT_UWB_TX_GPIO);
    bu.baudrate = CONFIG_FOLLOW_ROBOT_UWB_BAUD;

    if (bu_uwb_init(&bu) == ESP_OK) {
        xTaskCreate(uwb_task, "uwb", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "uwb ready (RX=GPIO%d)", CONFIG_FOLLOW_ROBOT_UWB_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "uwb init FAILED");
    }

    /* --- 5. 激光雷达 (Lidar) 模块初始化 --- */
    rplidar_c1_config_t lc = rplidar_c1_default_config(
        (uart_port_t)CONFIG_FOLLOW_ROBOT_LIDAR_UART,
        CONFIG_FOLLOW_ROBOT_LIDAR_RX_GPIO, CONFIG_FOLLOW_ROBOT_LIDAR_TX_GPIO);
    lc.baudrate = CONFIG_FOLLOW_ROBOT_LIDAR_BAUD;

    if (rplidar_c1_init(&s_lidar, &lc) == ESP_OK &&
        rplidar_c1_start_scan(&s_lidar) == ESP_OK) {
        xTaskCreate(lidar_task, "lidar", 4096, &s_lidar, 6, NULL);
        ESP_LOGI(TAG, "lidar scanning");
    } else {
        ESP_LOGE(TAG, "lidar init/scan FAILED - avoidance falls back to ultrasonics");
    }

    /* --- 6. 超声波传感器 (行李箱前方左右边角) 初始化 --- */
    a02yyuw_config_t ulcfg = a02yyuw_default_config(
        (uart_port_t)0, CONFIG_FOLLOW_ROBOT_ULTRA_LEFT_RX_GPIO, -1);
    ulcfg.use_sw_uart = true;

    if (a02yyuw_init_dev(&s_ultra_left, &ulcfg) == ESP_OK) {
        xTaskCreate(ultra_task, "ultra_l", 3072, &s_ua_left, 5, NULL);
        ESP_LOGI(TAG, "ultrasonic L ready (RX=GPIO%d)",
                 CONFIG_FOLLOW_ROBOT_ULTRA_LEFT_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "ultrasonic L init FAILED");
    }

    a02yyuw_config_t urcfg = a02yyuw_default_config(
        (uart_port_t)0, CONFIG_FOLLOW_ROBOT_ULTRA_RIGHT_RX_GPIO, -1);
    urcfg.use_sw_uart = true;

    if (a02yyuw_init_dev(&s_ultra_right, &urcfg) == ESP_OK) {
        xTaskCreate(ultra_task, "ultra_r", 3072, &s_ua_right, 5, NULL);
        ESP_LOGI(TAG, "ultrasonic R ready (RX=GPIO%d)",
                 CONFIG_FOLLOW_ROBOT_ULTRA_RIGHT_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "ultrasonic R init FAILED");
    }

    /* --- 7. IMU (惯性测量单元) 初始化 --- */
    static i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = 0,
        .sda_io_num = CONFIG_FOLLOW_ROBOT_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_FOLLOW_ROBOT_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) == ESP_OK) {
        imu_i2c_config_t imucfg = imu_i2c_default_config();
        imucfg.sda_gpio = CONFIG_FOLLOW_ROBOT_I2C_SDA_GPIO;
        imucfg.scl_gpio = CONFIG_FOLLOW_ROBOT_I2C_SCL_GPIO;
        imucfg.device_address = CONFIG_FOLLOW_ROBOT_IMU_ADDR;
        imucfg.external_bus = i2c_bus;

        if (imu_i2c_init(&s_imu, &imucfg) == ESP_OK) {
            s_imu_ok = true;
            ESP_LOGI(TAG, "imu ready (heading loop %s)",
                     FR_HEADING_HOLD ? "ON" : "off");
        } else {
            ESP_LOGE(TAG, "imu init FAILED - heading loop disabled");
        }
    }

    /* --- 8. [新增] 远程安全初始状态 --- */
#if FR_WIFI_ENABLED && FR_AUTO_ARM
    remote_clear_and_arm(now_us());
    ESP_LOGW(TAG, "AUTO-ARM: motion armed automatically (bench debug mode)");
#else
    remote_motion_off(now_us());
    remote_estop(now_us());
    ESP_LOGI(TAG, "motion locked; press CLEAR/ARM on web page to enable motors");
#endif

    /* --- 9. 核心控制循环启动 --- */
    xTaskCreate(control_task, "control", 6144, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control loop running at %d Hz (WiFi=%d flash_log=%d)",
             CONFIG_FOLLOW_ROBOT_CONTROL_HZ, FR_WIFI_ENABLED, FR_FLASH_LOG_ENABLED);
}
