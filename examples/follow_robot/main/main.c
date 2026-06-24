/*
 * follow_robot - smart follow-me suitcase with obstacle avoidance.
 *
 * Wiring of the sensing/acting stack:
 *   UWB (BU0x)      -> follow target  (range + bearing to the user's tag)
 *   RPLIDAR C1      -> obstacle field (front 180 deg polar histogram)
 *   2x A02YYUW      -> front-corner near-field safety (ultrasonic)
 *   IMU             -> yaw telemetry (optional, heading hold hook)
 *   chassis         -> rear differential drive (front wheels are casters)
 *
 * Architecture: each sensor runs in its own FreeRTOS task and publishes into a
 * mutex-protected snapshot. A fixed-rate control task reads the snapshot, runs
 * the pure follow_avoid algorithm, and drives the chassis. If any sensor fails
 * to start, its data simply stays "stale/invalid" and the algorithm degrades
 * gracefully (e.g. no lidar -> rely on ultrasonics; no UWB -> search then idle).
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "a02yyuw.h"
#include "bu_uwb.h"
#include "rplidar_c1.h"
#include "chassis.h"
#include "follow_avoid.h"

#if CONFIG_FOLLOW_ROBOT_IMU_ENABLE
#include "driver/i2c_master.h"
#include "imu_i2c.h"
#endif

static const char *TAG = "follow_robot";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)

/* Kconfig 'bool' symbols are left #undef when disabled, so they cannot be used
 * directly inside C expressions - normalise them to concrete values here. */
#ifdef CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_INVERT
#define FR_LEFT_INVERT true
#else
#define FR_LEFT_INVERT false
#endif
#ifdef CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_INVERT
#define FR_RIGHT_INVERT true
#else
#define FR_RIGHT_INVERT false
#endif
#ifdef CONFIG_FOLLOW_ROBOT_UWB_LEFT_IS_POS_X
#define FR_UWB_LEFT_SIGN 1.0f
#else
#define FR_UWB_LEFT_SIGN -1.0f
#endif

#define LIDAR_SECTORS 36                 /* 5 deg per sector over 180 deg FOV */
#define LIDAR_FOV_RAD ((float)M_PI)

/* Freshness windows: data older than this is treated as missing. */
#define TARGET_FRESH_US 700000ULL        /* 0.7 s */
#define FIELD_FRESH_US 500000ULL         /* 0.5 s */
#define ULTRA_FRESH_US 500000ULL         /* 0.5 s */

/* ----------------------------------------------------- shared snapshot */

typedef struct {
    SemaphoreHandle_t lock;

    /* UWB target */
    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;

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

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/* ----------------------------------------------------- UWB follow target */

static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];
    const float left_sign = FR_UWB_LEFT_SIGN;

    while (1) {
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) {
            continue;
        }

        bu_uwb_twr_reading_t twr = {0};
        bu_uwb_distance_t dist = {0};

        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            const float fwd_m = (float)twr.y_cm / 100.0f;
            const float left_m = left_sign * (float)twr.x_cm / 100.0f;
            float range = (twr.distance_cm > 0) ? (float)twr.distance_cm / 100.0f
                                                : sqrtf(fwd_m * fwd_m + left_m * left_m);
            float bearing = 0.0f;
            if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f) {
                bearing = atan2f(left_m, fwd_m);
            }
            lock();
            g_shared.tgt_distance_m = range;
            g_shared.tgt_bearing_rad = bearing;
            g_shared.tgt_ts_us = now_us();
            unlock();
        } else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            /* Range-only frame: refresh distance, keep the last known bearing. */
            lock();
            g_shared.tgt_distance_m = dist.distance_m;
            g_shared.tgt_ts_us = now_us();
            unlock();
        }
    }
}

/* ----------------------------------------------------- Lidar obstacle field */

static float lidar_angle_to_body_rad(float raw_deg)
{
    float rel = raw_deg - (float)CONFIG_FOLLOW_ROBOT_LIDAR_FORWARD_DEG;
#if CONFIG_FOLLOW_ROBOT_LIDAR_CW
    rel = -rel; /* lidar CW -> body CCW-positive */
#endif
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
            const float body = lidar_angle_to_body_rad(p.angle_deg);
            fa_obstacle_add(&work, body, p.distance_mm / 1000.0f);
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
    cc.pwm_freq_hz = CONFIG_FOLLOW_ROBOT_PWM_FREQ_HZ;
    cc.left_pwm_gpio = CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_PWM_GPIO;
    cc.left_in1_gpio = CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_IN1_GPIO;
    cc.left_in2_gpio = CONFIG_FOLLOW_ROBOT_MOTOR_LEFT_IN2_GPIO;
    cc.left_invert = FR_LEFT_INVERT;
    cc.right_pwm_gpio = CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_PWM_GPIO;
    cc.right_in1_gpio = CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_IN1_GPIO;
    cc.right_in2_gpio = CONFIG_FOLLOW_ROBOT_MOTOR_RIGHT_IN2_GPIO;
    cc.right_invert = FR_RIGHT_INVERT;
    cc.track_width_m = CONFIG_FOLLOW_ROBOT_TRACK_WIDTH_MM / 1000.0f;
    cc.max_speed_mps = CONFIG_FOLLOW_ROBOT_MAX_WHEEL_SPEED_MMPS / 1000.0f;
    cc.min_duty = CONFIG_FOLLOW_ROBOT_MIN_DUTY_PCT / 100.0f;
    cc.max_duty = CONFIG_FOLLOW_ROBOT_MAX_DUTY_PCT / 100.0f;
    return cc;
}

static const char *state_name(fa_state_t s)
{
    switch (s) {
    case FA_STATE_IDLE: return "IDLE";
    case FA_STATE_SEARCH: return "SEARCH";
    case FA_STATE_FOLLOW: return "FOLLOW";
    case FA_STATE_AVOID: return "AVOID";
    case FA_STATE_ESTOP: return "ESTOP";
    default: return "?";
    }
}

static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;

    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa.cfg = build_fa_config();

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ROBOT_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = now_us();
    int log_div = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        const float dt = (float)(t - prev_us) / 1e6f;
        prev_us = t;

        /* Snapshot shared sensor data under the lock. */
        fa_target_t target = {0};
        fa_obstacle_field_t field;
        fa_range_t ul = {0};
        fa_range_t ur = {0};
        bool have_field;

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
        unlock();

        fa_output_t out = fa_update(&fa, &target,
                                    have_field ? &field : NULL, &ul, &ur, dt);

        chassis_set_velocity(chassis, out.v_mps, out.omega_rps);

        if (++log_div >= CONFIG_FOLLOW_ROBOT_CONTROL_HZ / 5) { /* ~5 Hz */
            log_div = 0;
            ESP_LOGI(TAG,
                     "%-6s tgt=%s d=%.2f br=%+.2f | clr=%.2f blk=%d | v=%+.2f w=%+.2f",
                     state_name(out.state), target.valid ? "Y" : "N",
                     target.distance_m, target.bearing_rad,
                     out.front_clearance_m, out.blocked, out.v_mps,
                     out.omega_rps);
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
    ESP_LOGI(TAG, "Follow-me suitcase starting");

    memset(&g_shared, 0, sizeof(g_shared));
    g_shared.lock = xSemaphoreCreateMutex();
    fa_obstacle_reset(&g_shared.field, LIDAR_SECTORS, LIDAR_FOV_RAD);

    /* --- Chassis --- */
    chassis_config_t cc = build_chassis_config();
    if (chassis_init(&s_chassis, &cc) == ESP_OK) {
        chassis_stop(&s_chassis);
        ESP_LOGI(TAG, "chassis ready");
    } else {
        ESP_LOGE(TAG, "chassis init FAILED - check motor GPIOs");
    }

    /* --- UWB target --- */
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

    /* --- Lidar --- */
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

    /* --- Ultrasonics (front corners) ---
     * Both run on the per-instance software UART: the two hardware UARTs are
     * taken by UWB (UART1) and the lidar (UART2), and A02YYUW only needs 9600
     * baud, which the bit-bang UART handles comfortably. */
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
    urcfg.use_sw_uart = true; /* second ultrasonic uses the bit-bang UART */
    if (a02yyuw_init_dev(&s_ultra_right, &urcfg) == ESP_OK) {
        xTaskCreate(ultra_task, "ultra_r", 3072, &s_ua_right, 5, NULL);
        ESP_LOGI(TAG, "ultrasonic R ready (RX=GPIO%d)",
                 CONFIG_FOLLOW_ROBOT_ULTRA_RIGHT_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "ultrasonic R init FAILED");
    }

#if CONFIG_FOLLOW_ROBOT_IMU_ENABLE
    /* IMU is optional telemetry; failure here is non-fatal. */
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
        static imu_i2c_t imu;
        imu_i2c_config_t imucfg = imu_i2c_default_config();
        imucfg.sda_gpio = CONFIG_FOLLOW_ROBOT_I2C_SDA_GPIO;
        imucfg.scl_gpio = CONFIG_FOLLOW_ROBOT_I2C_SCL_GPIO;
        imucfg.device_address = CONFIG_FOLLOW_ROBOT_IMU_ADDR;
        imucfg.external_bus = i2c_bus;
        if (imu_i2c_init(&imu, &imucfg) == ESP_OK) {
            ESP_LOGI(TAG, "imu ready (yaw available for heading hold)");
        }
    }
#endif

    /* --- Control loop owns the chassis from here on --- */
    xTaskCreate(control_task, "control", 4096, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control loop running at %d Hz", CONFIG_FOLLOW_ROBOT_CONTROL_HZ);
}
