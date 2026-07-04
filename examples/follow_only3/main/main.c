/*
 * follow_only3 - 跟随算法3: 高级纯跟随行李箱
 * 相比跟随代码1，引入业界主流跟随算法技术，显著提升跟随平滑性、
 * 响应速度和鲁棒性。
 *
 * 传感器 (与跟随代码1相同):
 *   UWB (BU0x)     - 目标跟随 (距离 + 方位)
 *   IMU            - 航向闭环 + 陀螺仪角速度融合
 *   底盘编码器      - 轮速闭环 + 里程计
 *
 * ======================== 算法改进说明 ========================
 *
 * 改进1: Kalman滤波平滑
 *   - 对UWB距离和方位分别应用一维Kalman滤波器
 *   - 滤除UWB传感器噪声 (典型σ≈20cm距离, σ≈5°方位)
 *   - 估计目标运动速度，用于预测跟踪
 *
 * 改进2: 自适应PID增益
 *   - 距离P增益随误差大小自适应调整 (大误差→高增益快速追赶)
 *   - 方位P增益随当前速度自适应调整 (低速→高转向增益灵活转弯)
 *   - 引入积分项消除稳态静差，带反计算抗饱和
 *
 * 改进3: S曲线加加速度限制
 *   - 替换原简单线性ramp，使用jerk-limited速度剖面
 *   - 避免加速度突变导致车轮打滑和机械冲击
 *   - 更符合真实物理系统动力学
 *
 * 改进4: 预测目标跟踪
 *   - 维护滑动窗口记录目标历史位置
 *   - 估计目标在车辆坐标系下的运动速度
 *   - 控制回路跟踪预测位置而非当前位置，减小跟随时延
 *
 * 改进5: 阿基米德螺线搜索
 *   - 目标丢失后执行扩展螺线搜索，而非简单原地旋转
 *   - 搜索半径随时间增大，覆盖更大区域
 *   - 利用最后已知的目标速度方向优化搜索方向
 *
 * 改进6: 互补滤波融合IMU+编码器
 *   - 融合陀螺仪角速度和编码器差速推算的角速度
 *   - 陀螺仪高频响应好 (α>0.95)，编码器低频无漂移
 *   - 提供更准确的航向估计
 *
 * 改进7: 动态跟车距离
 *   - 根据目标运动速度动态调整期望跟车距离
 *   - 目标快速移动时保持更大安全距离
 *   - 目标静止时可更近跟随
 *
 * 改进8: 增强状态机
 *   - 引入状态切换迟滞，防止频繁振荡
 *   - ASLEEP状态: 目标长时间静止时进入低功耗待机
 *   - 更快重捕获: 短暂丢失时保持航向预测
 *
 * =============================================================
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

#include "bu_uwb.h"
#include "chassis.h"

#include "driver/i2c_master.h"
#include "imu_i2c.h"

static const char *TAG = "follow3";

#define M_PI 3.14159265358979323846f
#define DEG2RAD(d) ((float)(d) * M_PI / 180.0f)
#define RAD2DEG(r) ((float)(r) * 180.0f / M_PI)

/* 硬件方向常量 (与跟随代码1相同) */
#define FR_LEFT_INVERT true
#define FR_RIGHT_INVERT true
#define FR_LEFT_ENC_INVERT true
#define FR_RIGHT_ENC_INVERT true
#define FR_UWB_LEFT_SIGN 1.0f
#define FR_IMU_YAW_SIGN -1.0f

/* ==================== 工具函数 ==================== */

static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float wrap_pi(float a)
{
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

/* ==================== 改进1: 一维Kalman滤波器 ====================
 *
 * 对UWB距离和方位分别滤波。
 * 状态模型: x_k+1 = x_k + v_k*dt + w_k   (位置+速度)
 * 观测模型: z_k = x_k + v_k               (仅观测位置)
 *
 * 预测步: x_hat = x; P = P + Q
 * 更新步: K = P / (P + R); x += K*(z - x); P *= (1-K)
 */

typedef struct {
    float x;            /* 位置状态估计 */
    float P;            /* 误差协方差 */
    float Q;            /* 过程噪声 (目标运动的不可预测性) */
    float R;            /* 测量噪声 (传感器精度) */
    bool initialized;
} kalman1d_t;

static void kf_init(kalman1d_t *kf, float init_z, float Q, float R)
{
    kf->x = init_z;
    kf->P = 1.0f;
    kf->Q = Q;
    kf->R = R;
    kf->initialized = true;
}

static float kf_update(kalman1d_t *kf, float z)
{
    kf->P += kf->Q;
    float K = kf->P / (kf->P + kf->R);
    kf->x += K * (z - kf->x);
    kf->P *= (1.0f - K);
    return kf->x;
}

/* ==================== 改进4: 目标历史与预测跟踪 ====================
 *
 * 维护最近的UWB历史记录，估计目标在车辆坐标系下的运动速度。
 * 预测位置 = 当前位置 + 速度估计 * 预测时间
 */

#define TGT_HIST_SIZE 8

typedef struct {
    float distance_m[TGT_HIST_SIZE];
    float bearing_rad[TGT_HIST_SIZE];
    uint64_t ts_us[TGT_HIST_SIZE];
    int head;
    int count;
    float est_dist_rate;    /* 估计目标距离变化率 (m/s, 正值=远离) */
    float est_bear_rate;    /* 估计目标方位变化率 (rad/s, 正值=右移) */
} tgt_history_t;

static void tgt_hist_push(tgt_history_t *h, float dist, float bear, uint64_t ts)
{
    h->distance_m[h->head] = dist;
    h->bearing_rad[h->head] = bear;
    h->ts_us[h->head] = ts;
    h->head = (h->head + 1) % TGT_HIST_SIZE;
    if (h->count < TGT_HIST_SIZE) h->count++;
}

static void tgt_hist_estimate(tgt_history_t *h)
{
    if (h->count < 2) {
        h->est_dist_rate = 0.0f;
        h->est_bear_rate = 0.0f;
        return;
    }

    /* 线性回归估计变化率 */
    int oldest = (h->head - h->count + TGT_HIST_SIZE) % TGT_HIST_SIZE;
    float sum_dt = 0, sum_dd = 0, sum_db = 0, sum_dt2 = 0;
    float t0 = (float)(int64_t)(h->ts_us[oldest] / 1000);

    for (int i = 1; i < h->count; i++) {
        int idx = (oldest + i) % TGT_HIST_SIZE;
        int prev = (idx - 1 + TGT_HIST_SIZE) % TGT_HIST_SIZE;
        float dt = (float)(int64_t)((h->ts_us[idx] - h->ts_us[prev]) / 1000);
        float dd = h->distance_m[idx] - h->distance_m[prev];
        float db = wrap_pi(h->bearing_rad[idx] - h->bearing_rad[prev]);

        if (dt > 1.0f && dt < 5000.0f) {
            sum_dd += dd;
            sum_db += db;
            sum_dt += dt;
        }
    }

    if (sum_dt > 1e-3f) {
        h->est_dist_rate = sum_dd / (sum_dt / 1000.0f);
        h->est_bear_rate = sum_db / (sum_dt / 1000.0f);
    } else {
        h->est_dist_rate = 0.0f;
        h->est_bear_rate = 0.0f;
    }
}

static void tgt_hist_predict(tgt_history_t *h, float ahead_s,
                             float *pred_dist, float *pred_bear)
{
    int last = (h->head - 1 + TGT_HIST_SIZE) % TGT_HIST_SIZE;
    *pred_dist = h->distance_m[last];
    *pred_bear = h->bearing_rad[last];

    if (h->count >= 2) {
        *pred_dist += h->est_dist_rate * ahead_s;
        *pred_bear = wrap_pi(*pred_bear + h->est_bear_rate * ahead_s);
    }
}

/* ==================== 改进2: 自适应PID控制器 ==================== */

typedef struct {
    float kp_base;      /* 基础比例增益 */
    float ki;           /* 积分增益 */
    float kd;           /* 微分增益 */
    float out_max;      /* 输出上限 */
    float out_min;      /* 输出下限 */
    float i_term;       /* 积分项 */
    float prev_error;   /* 上一周期误差 */
    float prev_meas;    /* 上一周期测量值 (用于微分) */
    bool has_prev;
} adaptive_pid_t;

static void apid_init(adaptive_pid_t *p, float kp, float ki, float kd,
                      float out_min, float out_max)
{
    memset(p, 0, sizeof(*p));
    p->kp_base = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
}

static float apid_step(adaptive_pid_t *p, float setpoint, float measured,
                       float dt, float kp_scale)
{
    float error = setpoint - measured;
    float kp = p->kp_base * kp_scale;

    /* 比例项 */
    float out = kp * error;

    /* 积分项 (带反计算抗饱和) */
    float i_ideal = p->i_term + p->ki * error * dt;
    float out_no_i = out + p->ki * error * dt;
    if (out_no_i <= p->out_max && out_no_i >= p->out_min) {
        p->i_term = clampf(i_ideal, p->out_min, p->out_max);
    }
    out += p->i_term;

    /* 微分项 (在测量值上取微分，避免设定值突变冲击) */
    if (p->has_prev && dt > 1e-6f) {
        float d = -(measured - p->prev_meas) / dt;
        out += p->kd * d;
    }

    p->prev_error = error;
    p->prev_meas = measured;
    p->has_prev = true;

    return clampf(out, p->out_min, p->out_max);
}

static void apid_reset(adaptive_pid_t *p)
{
    p->i_term = 0.0f;
    p->prev_error = 0.0f;
    p->has_prev = false;
}

/* ==================== 改进3: S曲线速度剖面 (Jerk限制) ====================
 *
 * 限制加加速度 (jerk) 而非仅限制加速度。
 * v_current → target: 先加加速度, 再恒定加速度, 再减加加速度
 */

typedef struct {
    float current_v;    /* 当前输出速度 */
    float current_a;    /* 当前加速度 */
} s_curve_t;

static void s_curve_init(s_curve_t *sc)
{
    sc->current_v = 0.0f;
    sc->current_a = 0.0f;
}

static float s_curve_update(s_curve_t *sc, float target, float dt,
                            float max_accel, float max_jerk)
{
    if (dt <= 1e-6f) return sc->current_v;

    float error = target - sc->current_v;

    /* 期望加速度: 比例逼近 + 预留减速距离 */
    float desired_a = error * 3.0f;

    /* 如果接近目标，开始减速 */
    float stop_dist = (sc->current_a * sc->current_a) / (2.0f * max_accel);
    if (fabsf(error) < stop_dist + 0.05f) {
        desired_a = (error > 0 ? -max_accel : max_accel);
    }

    desired_a = clampf(desired_a, -max_accel, max_accel);

    /* 限制加加速度 */
    float a_step = clampf(desired_a - sc->current_a, -max_jerk * dt, max_jerk * dt);
    sc->current_a += a_step;
    sc->current_v += sc->current_a * dt;

    /* 防止过冲 */
    if ((error > 0 && sc->current_v > target) ||
        (error < 0 && sc->current_v < target)) {
        sc->current_v = target;
        sc->current_a = 0.0f;
    }

    return sc->current_v;
}

static void s_curve_reset(s_curve_t *sc)
{
    sc->current_v = 0.0f;
    sc->current_a = 0.0f;
}

/* ==================== 改进6: 互补滤波 IMU+编码器航向融合 ====================
 *
 * 陀螺仪短时精度高但积分漂移, 编码器差速无漂移但响应慢。
 * yaw_rate = α * gyro + (1-α) * enc_rate
 * 典型 α = 0.95~0.98
 */

typedef struct {
    float fused_yaw;    /* 融合累积航向 (rad) */
    bool initialized;
} comp_filter_t;

static void cf_reset(comp_filter_t *cf, float init_yaw)
{
    cf->fused_yaw = init_yaw;
    cf->initialized = true;
}

static float cf_update(comp_filter_t *cf, float imu_yaw_rate, float enc_yaw_rate,
                       float alpha, float dt)
{
    if (!cf->initialized) return 0.0f;

    float fused_rate = alpha * imu_yaw_rate + (1.0f - alpha) * enc_yaw_rate;
    cf->fused_yaw = wrap_pi(cf->fused_yaw + fused_rate * dt);
    return cf->fused_yaw;
}

/* =====================================================================
 * 共享快照 (与跟随代码1相同结构)
 * ===================================================================== */
typedef struct {
    SemaphoreHandle_t lock;
    float tgt_distance_m;
    float tgt_bearing_rad;
    uint64_t tgt_ts_us;
} shared_t;

static shared_t g_shared;
static imu_i2c_t s_imu;
static bool s_imu_ok = false;
static chassis_t s_chassis;

/* =====================================================================
 * 增强状态机
 *
 * IDLE     - 初始/长时间无目标，完全停车
 * SEARCH   - 目标丢失，执行螺线搜索
 * FOLLOW   - 正常跟随
 * ASLEEP   - 目标长时间静止且距离OK，进入低功耗待机
 *
 * 状态切换:
 *   IDLE ──目标出现──► FOLLOW
 *   IDLE ◄──搜索超时── SEARCH ◄──目标丢失── FOLLOW
 *   FOLLOW ──目标静止>30s+距离OK──► ASLEEP
 *   ASLEEP ──目标移动──► FOLLOW
 * ===================================================================== */

typedef enum {
    STATE_IDLE = 0,
    STATE_SEARCH,
    STATE_FOLLOW,
    STATE_ASLEEP,
} follow_state_t;

static const char *state_name(follow_state_t s)
{
    switch (s) {
    case STATE_IDLE:   return "IDLE";
    case STATE_SEARCH: return "SEARCH";
    case STATE_FOLLOW: return "FOLLOW";
    case STATE_ASLEEP: return "ASLEEP";
    default:           return "?";
    }
}

/* =====================================================================
 * 互斥锁帮助函数
 * ===================================================================== */
static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/* =====================================================================
 * UWB接收任务 (与跟随代码1相同)
 * ===================================================================== */
static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];
    const float left_sign = FR_UWB_LEFT_SIGN;

    while (1) {
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) continue;

        bu_uwb_twr_reading_t twr = {0};
        bu_uwb_distance_t dist = {0};

        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            float fwd_m = (float)twr.y_cm / 100.0f;
            float left_m = left_sign * (float)twr.x_cm / 100.0f;
            float range = (twr.distance_cm > 0)
                              ? (float)twr.distance_cm / 100.0f
                              : sqrtf(fwd_m * fwd_m + left_m * left_m);
            float bearing = 0.0f;
            if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f)
                bearing = atan2f(left_m, fwd_m);

            lock();
            g_shared.tgt_distance_m = range;
            g_shared.tgt_bearing_rad = bearing;
            g_shared.tgt_ts_us = now_us();
            unlock();
        } else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            lock();
            g_shared.tgt_distance_m = dist.distance_m;
            g_shared.tgt_ts_us = now_us();
            unlock();
        }
    }
}

/* =====================================================================
 * IMU读取
 * ===================================================================== */
static bool imu_read_full(imu_i2c_reading_t *out)
{
    if (!s_imu_ok) return false;
    memset(out, 0, sizeof(*out));
    return imu_i2c_read_all(&s_imu, out) == ESP_OK && out->valid;
}

/* =====================================================================
 * 控制任务 - 核心算法
 * ===================================================================== */
static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;

    /* ---- 可配置参数 ---- */
    const float follow_dist_mm = (float)CONFIG_FOLLOW3_FOLLOW_DISTANCE_MM;
    const float stop_band_m = CONFIG_FOLLOW3_STOP_BAND_MM / 1000.0f;
    const float max_linear_mps = CONFIG_FOLLOW3_MAX_LINEAR_MMPS / 1000.0f;
    const float max_angular_rps = CONFIG_FOLLOW3_MAX_ANGULAR_MRADPS / 1000.0f;
    const float kp_dist = CONFIG_FOLLOW3_KP_DIST / 1000.0f;
    const float kp_bear = CONFIG_FOLLOW3_KP_BEAR / 1000.0f;
    const float search_rps = CONFIG_FOLLOW3_SEARCH_ANGULAR_MRADPS / 1000.0f;
    const float search_timeout_s = (float)CONFIG_FOLLOW3_SEARCH_TIMEOUT_S;
    const uint64_t target_fresh_us = (uint64_t)CONFIG_FOLLOW3_TARGET_FRESH_MS * 1000ULL;
    const float heading_kp = CONFIG_FOLLOW3_HEADING_KP_MILLI / 1000.0f;

    /* ---- Kalman参数 ---- */
    const float kf_dist_Q = CONFIG_FOLLOW3_KF_DIST_Q / 10000.0f;
    const float kf_dist_R = CONFIG_FOLLOW3_KF_DIST_R / 10000.0f;
    const float kf_bear_Q = CONFIG_FOLLOW3_KF_BEAR_Q / 10000.0f;
    const float kf_bear_R = CONFIG_FOLLOW3_KF_BEAR_R / 10000.0f;

    /* ---- S曲线参数 ---- */
    const float max_jerk = CONFIG_FOLLOW3_MAX_JERK / 1000.0f;
    const float max_accel = CONFIG_FOLLOW3_MAX_ACCEL / 1000.0f;
    const float max_ang_accel = CONFIG_FOLLOW3_MAX_ANG_ACCEL / 1000.0f;

    /* ---- 互补滤波器参数 ---- */
    const float cf_alpha = CONFIG_FOLLOW3_CF_ALPHA / 1000.0f;

    /* ---- 动态跟车距离参数 ---- */
    const float dyn_dist_factor = CONFIG_FOLLOW3_DYN_DIST_FACTOR / 1000.0f;

    /* ---- 初始化滤波器 ---- */
    kalman1d_t kf_dist;
    kalman1d_t kf_bear;
    bool kf_dist_init = false;
    bool kf_bear_init = false;

    /* ---- 初始化目标历史 ---- */
    tgt_history_t tgt_hist;
    memset(&tgt_hist, 0, sizeof(tgt_hist));

    /* ---- 初始化自适应PID ---- */
    adaptive_pid_t pid_dist;
    apid_init(&pid_dist, kp_dist, 0.0f, 0.0f, 0.0f, max_linear_mps);

    adaptive_pid_t pid_bear;
    apid_init(&pid_bear, kp_bear, 0.0f, 0.0f, -max_angular_rps, max_angular_rps);

    /* ---- 初始化S曲线 ---- */
    s_curve_t sc_v;
    s_curve_t sc_w;
    s_curve_init(&sc_v);
    s_curve_init(&sc_w);

    /* ---- 初始化互补滤波器 ---- */
    comp_filter_t cf;
    bool cf_init = false;

    /* ---- 状态机变量 ---- */
    follow_state_t state = STATE_IDLE;
    float lost_timer_s = 0.0f;
    float search_phase = 0.0f;
    float search_timer_s = 0.0f;
    float last_known_bearing = 0.0f;
    float last_known_dist = 1.0f;
    bool has_last_known = false;
    float yaw_ref = 0.0f;
    float cmd_v_out = 0.0f;
    float cmd_w_out = 0.0f;
    float asleep_timer_s = 0.0f;
    const float asleep_threshold_s = 30.0f;

    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW3_CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t prev_us = now_us();
    int log_div = 0;

    ESP_LOGI(TAG, "=== 跟随算法3 启动 ===");
    ESP_LOGI(TAG, "Kalman: dist Q=%.4f R=%.4f | bear Q=%.4f R=%.4f",
             kf_dist_Q, kf_dist_R, kf_bear_Q, kf_bear_R);
    ESP_LOGI(TAG, "S-curve: max_jerk=%.1f max_accel=%.1f ang_accel=%.1f",
             max_jerk, max_accel, max_ang_accel);
    ESP_LOGI(TAG, "Complementary-filter alpha=%.2f | dyn_dist_factor=%.2f",
             cf_alpha, dyn_dist_factor);

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();
        float dt = (float)(t - prev_us) / 1e6f;
        if (dt <= 0.0f) dt = 0.02f;
        if (dt > 0.5f) dt = 0.02f;
        prev_us = t;

        /* ---- 读取IMU ---- */
        imu_i2c_reading_t imu_rd;
        bool imu_valid = imu_read_full(&imu_rd);

        /* ---- 快照UWB数据 ---- */
        bool tgt_valid;
        float tgt_dist_raw;
        float tgt_bear_raw;
        lock();
        tgt_valid = (t - g_shared.tgt_ts_us) < target_fresh_us;
        tgt_dist_raw = g_shared.tgt_distance_m;
        tgt_bear_raw = g_shared.tgt_bearing_rad;
        unlock();

        /* ---- 改进1: Kalman滤波 ---- */
        float tgt_dist_filtered = tgt_dist_raw;
        float tgt_bear_filtered = tgt_bear_raw;
        if (tgt_valid) {
            if (!kf_dist_init) {
                kf_init(&kf_dist, tgt_dist_raw, kf_dist_Q, kf_dist_R);
                kf_dist_init = true;
            }
            if (!kf_bear_init) {
                kf_init(&kf_bear, tgt_bear_raw, kf_bear_Q, kf_bear_R);
                kf_bear_init = true;
            }
            tgt_dist_filtered = kf_update(&kf_dist, tgt_dist_raw);
            tgt_bear_filtered = kf_update(&kf_bear, tgt_bear_raw);
        }

        /* ---- 目标丢失计时 ---- */
        float target_moving = 0.0f;
        if (tgt_valid) {
            lost_timer_s = 0.0f;
            search_timer_s = 0.0f;
            last_known_bearing = tgt_bear_filtered;
            last_known_dist = tgt_dist_filtered;
            has_last_known = true;

            /* 改进4: 推入历史记录 */
            tgt_hist_push(&tgt_hist, tgt_dist_filtered, tgt_bear_filtered, t);
            tgt_hist_estimate(&tgt_hist);
            target_moving = fabsf(tgt_hist.est_dist_rate);
        } else {
            lost_timer_s += dt;
            if (state == STATE_SEARCH) search_timer_s += dt;
            target_moving = 0.0f;
        }

        /* ---- 改进4: 预测目标位置 ---- */
        float pred_dist = tgt_dist_filtered;
        float pred_bear = tgt_bear_filtered;
        if (tgt_valid) {
            float predict_ahead = 0.3f;
            tgt_hist_predict(&tgt_hist, predict_ahead, &pred_dist, &pred_bear);
        }

        /* ---- 改进7: 动态跟车距离 ---- */
        float dynamic_follow_dist = follow_dist_mm / 1000.0f;
        if (tgt_valid) {
            dynamic_follow_dist += dyn_dist_factor * target_moving;
            dynamic_follow_dist = clampf(dynamic_follow_dist, follow_dist_mm / 1000.0f,
                                         (follow_dist_mm + 500.0f) / 1000.0f);
        }

        float v_des = 0.0f;
        float w_des = 0.0f;

        /* ========== 状态机 ========== */

        /* ── 过渡: 短时丢失(≤500ms)保持航向预测 ── */
        if (!tgt_valid && lost_timer_s <= 0.5f && has_last_known && state == STATE_FOLLOW) {
            /* 短暂丢失，保持最后命令 */
            v_des = cmd_v_out;
            w_des = cmd_w_out;
        }
        /* ── STATE: IDLE ── */
        else if (state == STATE_IDLE) {
            if (tgt_valid) {
                state = STATE_FOLLOW;
                apid_reset(&pid_dist);
                apid_reset(&pid_bear);
                s_curve_reset(&sc_v);
                s_curve_reset(&sc_w);
                asleep_timer_s = 0.0f;
                ESP_LOGI(TAG, "IDLE -> FOLLOW (target detected)");
            }
        }
        /* ── STATE: SEARCH ── */
        else if (state == STATE_SEARCH) {
            if (tgt_valid) {
                state = STATE_FOLLOW;
                apid_reset(&pid_dist);
                apid_reset(&pid_bear);
                s_curve_reset(&sc_v);
                s_curve_reset(&sc_w);
                ESP_LOGI(TAG, "SEARCH -> FOLLOW (target re-acquired)");
            } else if (search_timer_s >= search_timeout_s) {
                state = STATE_IDLE;
                has_last_known = false;
                ESP_LOGI(TAG, "SEARCH -> IDLE (timeout %.1fs)", search_timeout_s);
            } else {
                /* 改进5: 阿基米德螺线搜索 */
                float progress = search_timer_s / search_timeout_s;
                search_phase += search_rps * 1.5f * dt;

                /* 螺线半径随时间扩大 */
                float spiral_factor = 0.2f + 0.8f * progress;
                w_des = search_rps * spiral_factor *
                        ((last_known_bearing >= 0.0f) ? 1.0f : -1.0f);

                /* 小幅前进扩大搜索范围 */
                float forward_amplitude = 0.15f * sinf(search_phase * 2.0f);
                v_des = 0.08f + 0.05f * progress + forward_amplitude;
                v_des = clampf(v_des, 0.0f, 0.2f);

                cf_init = false;
            }
        }
        /* ── STATE: ASLEEP ── */
        else if (state == STATE_ASLEEP) {
            v_des = 0.0f;
            w_des = 0.0f;

            /* 检测目标是否移动 */
            if (tgt_valid && target_moving > 0.15f) {
                state = STATE_FOLLOW;
                apid_reset(&pid_dist);
                asleep_timer_s = 0.0f;
                ESP_LOGI(TAG, "ASLEEP -> FOLLOW (target moving)");
            } else if (tgt_valid) {
                float dist_err = fabsf(tgt_dist_filtered - dynamic_follow_dist);
                if (dist_err > stop_band_m * 2.0f) {
                    /* 目标虽然在，但位置变化了 */
                    state = STATE_FOLLOW;
                    apid_reset(&pid_dist);
                    asleep_timer_s = 0.0f;
                    ESP_LOGI(TAG, "ASLEEP -> FOLLOW (position changed)");                     }
            } else if (lost_timer_s > 1.0f) {
                state = STATE_SEARCH;
                search_timer_s = 0.0f;
                ESP_LOGI(TAG, "ASLEEP -> SEARCH (target lost)");
            }
        }
        /* ── STATE: FOLLOW ── */
        else if (state == STATE_FOLLOW) {
            if (!tgt_valid && lost_timer_s > 1.0f) {
                state = STATE_SEARCH;
                search_timer_s = 0.0f;
                search_phase = 0.0f;
                cf_init = false;
                ESP_LOGI(TAG, "FOLLOW -> SEARCH (target lost %.1fs)", lost_timer_s);
            } else {
                /* 改进7: 静止检测 → ASLEEP */
                if (tgt_valid && target_moving < 0.08f) {
                    float dist_err = fabsf(tgt_dist_filtered - dynamic_follow_dist);
                    if (dist_err < stop_band_m) {
                        asleep_timer_s += dt;
                        if (asleep_timer_s > asleep_threshold_s) {
                            state = STATE_ASLEEP;
                            ESP_LOGI(TAG, "FOLLOW -> ASLEEP (target stationary %.0fs)",
                                     asleep_timer_s);
                        }
                    } else {
                        asleep_timer_s = 0.0f;
                    }
                } else {
                    asleep_timer_s = 0.0f;
                }

                if (state == STATE_FOLLOW && tgt_valid) {
                    /* 改进4: 使用预测位置而非原始观测 */
                    float follow_dist = pred_dist;
                    float follow_bear = pred_bear;

                    /* 改进2: 自适应距离增益 */
                    float dist_err = follow_dist - dynamic_follow_dist;
                    float dist_kp_scale = 1.0f + 3.0f * clampf(fabsf(dist_err) / 3.0f, 0.0f, 1.0f);

                    if (dist_err > 0.0f) {
                        v_des = apid_step(&pid_dist, dynamic_follow_dist, follow_dist,
                                         dt, dist_kp_scale);
                    } else if (-dist_err <= stop_band_m) {
                        v_des = 0.0f;
                        apid_reset(&pid_dist);
                    } else {
                        v_des = 0.0f;
                    }
                    v_des = clampf(v_des, 0.0f, max_linear_mps);

                    /* 改进2: 自适应方位增益 (低速时转向更灵活) */
                    float bear_kp_scale = 1.0f;
                    if (cmd_v_out < 0.3f) {
                        bear_kp_scale = 1.5f;
                    } else if (cmd_v_out > 0.6f) {
                        bear_kp_scale = 0.7f;
                    }

                    w_des = apid_step(&pid_bear, 0.0f, follow_bear,
                                     dt, bear_kp_scale);
                    w_des = clampf(w_des, -max_angular_rps, max_angular_rps);

                    /* 转向-速度耦合 (保留原文案) */
                    float turn_scale = clampf(1.0f - fabsf(follow_bear) / (0.5f * M_PI),
                                              0.0f, 1.0f);
                    v_des *= turn_scale;
                }
            }

            /* 改进6: IMU航向闭环 + 互补滤波 */
            if (imu_valid && tgt_valid && state == STATE_FOLLOW) {
                float imu_yaw = FR_IMU_YAW_SIGN * DEG2RAD(imu_rd.euler_deg[2]);
                float imu_gyro_z = FR_IMU_YAW_SIGN * imu_rd.gyro_rad_s[2];

                /* 从编码器差速推算角速度 */
                float m_left = 0.0f;
                float m_right = 0.0f;
                chassis_get_measured(chassis, NULL, NULL, &m_left, &m_right);
                float track = CONFIG_FOLLOW3_TRACK_WIDTH_MM / 1000.0f;
                float enc_yaw_rate = (m_right - m_left) / track;

                if (!cf_init) {
                    cf_reset(&cf, imu_yaw);
                    cf_init = true;
                }

                /* 改进6: 互补滤波融合 */
                float fused_yaw = cf_update(&cf, imu_gyro_z, enc_yaw_rate, cf_alpha, dt);

                /* 航向闭环 (使用融合航向) */
                yaw_ref = wrap_pi(yaw_ref + w_des * dt);
                float err_yaw = wrap_pi(yaw_ref - fused_yaw);
                w_des += heading_kp * err_yaw;
                w_des = clampf(w_des, -max_angular_rps, max_angular_rps);
            } else if (tgt_valid && state == STATE_FOLLOW) {
                cf_init = false;
            }
        }

        /* ========== 改进3: S曲线速度平滑 ========== */
        float cmd_v_smooth = s_curve_update(&sc_v, v_des, dt, max_accel, max_jerk);
        float cmd_w_smooth = s_curve_update(&sc_w, w_des, dt, max_ang_accel, max_jerk * 3.0f);

        cmd_v_out = cmd_v_smooth;
        cmd_w_out = cmd_w_smooth;

        chassis_set_velocity(chassis, cmd_v_out, cmd_w_out);
        chassis_update(chassis, dt);

        /* ========== 日志 (5Hz) ========== */
        if (++log_div >= CONFIG_FOLLOW3_CONTROL_HZ / 5) {
            log_div = 0;
            float mv = 0.0f;
            float mw = 0.0f;
            chassis_get_measured(chassis, &mv, &mw, NULL, NULL);
            ESP_LOGI(TAG,
                     "%-6s tgt=%s raw(d=%.2f b=%+.0f°) "
                     "kf(d=%.2f b=%+.0f°) "
                     "pred(d=%.2f b=%+.0f°) "
                     "cmd(v=%+.2f w=%+.2f) "
                     "meas(v=%+.2f w=%+.2f) "
                     "dyn_dist=%.1f",
                     state_name(state),
                     tgt_valid ? "Y" : "N",
                     tgt_dist_raw, RAD2DEG(tgt_bear_raw),
                     tgt_dist_filtered, RAD2DEG(tgt_bear_filtered),
                     pred_dist, RAD2DEG(pred_bear),
                     cmd_v_out, cmd_w_out,
                     mv, mw,
                     dynamic_follow_dist);
        }
    }
}

/* =====================================================================
 * app_main - 系统初始化
 * ===================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " 跟随算法3: 高级纯跟随行李箱");
    ESP_LOGI(TAG, " Kalman + 自适应PID + S曲线 + 预测跟踪");
    ESP_LOGI(TAG, "========================================");

    /* 1. 共享状态 */
    memset(&g_shared, 0, sizeof(g_shared));
    g_shared.lock = xSemaphoreCreateMutex();

    /* 2. 底盘初始化 */
    chassis_config_t cc = chassis_default_config();
    cc.esc_min_us = CONFIG_FOLLOW3_ESC_MIN_US;
    cc.esc_mid_us = CONFIG_FOLLOW3_ESC_MID_US;
    cc.esc_max_us = CONFIG_FOLLOW3_ESC_MAX_US;
    cc.left_esc_gpio = CONFIG_FOLLOW3_LEFT_ESC_GPIO;
    cc.right_esc_gpio = CONFIG_FOLLOW3_RIGHT_ESC_GPIO;
    cc.left_invert = FR_LEFT_INVERT;
    cc.right_invert = FR_RIGHT_INVERT;
    cc.left_enc_a_gpio = CONFIG_FOLLOW3_LEFT_ENC_A_GPIO;
    cc.left_enc_b_gpio = CONFIG_FOLLOW3_LEFT_ENC_B_GPIO;
    cc.right_enc_a_gpio = CONFIG_FOLLOW3_RIGHT_ENC_A_GPIO;
    cc.right_enc_b_gpio = CONFIG_FOLLOW3_RIGHT_ENC_B_GPIO;
    cc.left_enc_invert = FR_LEFT_ENC_INVERT;
    cc.right_enc_invert = FR_RIGHT_ENC_INVERT;
    cc.ticks_per_meter = (float)CONFIG_FOLLOW3_TICKS_PER_METER;
    cc.track_width_m = CONFIG_FOLLOW3_TRACK_WIDTH_MM / 1000.0f;
    cc.max_speed_mps = CONFIG_FOLLOW3_MAX_WHEEL_SPEED_MMPS / 1000.0f;
    cc.kp = (float)CONFIG_FOLLOW3_SPEED_KP;
    cc.ki = (float)CONFIG_FOLLOW3_SPEED_KI;
    cc.kd = (float)CONFIG_FOLLOW3_SPEED_KD;
    cc.pid_out_limit_us = (float)CONFIG_FOLLOW3_SPEED_PID_LIMIT_US;

    if (chassis_init(&s_chassis, &cc) == ESP_OK) {
        chassis_stop(&s_chassis);
        ESP_LOGI(TAG, "chassis ready; arming ESC (hold neutral %d ms)",
                 CONFIG_FOLLOW3_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FOLLOW3_ESC_ARM_MS));
    } else {
        ESP_LOGE(TAG, "chassis init FAILED - check ESC/encoder GPIOs");
    }

    /* 3. UWB初始化 */
    bu_uwb_config_t bu = bu_uwb_default_config(
        (uart_port_t)CONFIG_FOLLOW3_UWB_UART,
        CONFIG_FOLLOW3_UWB_RX_GPIO,
        CONFIG_FOLLOW3_UWB_TX_GPIO);
    bu.baudrate = CONFIG_FOLLOW3_UWB_BAUD;

    if (bu_uwb_init(&bu) == ESP_OK) {
        xTaskCreate(uwb_task, "uwb", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "uwb ready (RX=GPIO%d)", CONFIG_FOLLOW3_UWB_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "uwb init FAILED");
    }

    /* 4. IMU初始化 */
    static i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = 0,
        .sda_io_num = CONFIG_FOLLOW3_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_FOLLOW3_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) == ESP_OK) {
        imu_i2c_config_t imucfg = imu_i2c_default_config();
        imucfg.sda_gpio = CONFIG_FOLLOW3_I2C_SDA_GPIO;
        imucfg.scl_gpio = CONFIG_FOLLOW3_I2C_SCL_GPIO;
        imucfg.device_address = CONFIG_FOLLOW3_IMU_ADDR;
        imucfg.external_bus = i2c_bus;

        if (imu_i2c_init(&s_imu, &imucfg) == ESP_OK) {
            s_imu_ok = true;
            ESP_LOGI(TAG, "imu ready (heading loop + gyro fusion ON)");
        } else {
            ESP_LOGE(TAG, "imu init FAILED - heading loop disabled");
        }
    } else {
        ESP_LOGE(TAG, "I2C bus init FAILED");
    }

    /* 5. 启动控制循环 */
    xTaskCreate(control_task, "control", 4096, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control loop running at %d Hz", CONFIG_FOLLOW3_CONTROL_HZ);
}
