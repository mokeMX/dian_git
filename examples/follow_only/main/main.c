/*
 * follow_only - 纯跟随智能行李箱，无避障功能。
 *
 * 本程序对应"算法3"——最精简的跟随逻辑，移除了所有避障相关代码，
 * 仅保留核心的人体跟随能力。适用于空旷环境测试跟随精度和底盘闭环
 * 性能，或作为理解整个系统的入门切入点。
 *
 * 使用的传感器：
 *   UWB (BU0x)       -> 跟随目标定位（获取目标标签的距离和方位角）
 *   IMU              -> 航向闭环控制（用偏航角误差修正转向指令，防止跑偏）
 *   chassis (底盘)   -> 后轮差速驱动：APO-DL 无刷电调(RC PWM信号) + AB编码器PID闭环
 *
 * 三个示例之间的差异：
 *   - follow_only：无激光雷达，无超声波，无避障
 *   - follow_robot：完整版 = follow_only + 激光雷达 + 超声波 + VFH避障
 *   - sensor_hub：纯传感器诊断工具
 *
 * 状态机：简单的3态状态机：IDLE(空闲) -> SEARCH(搜索) -> FOLLOW(跟随)
 *
 * 硬件平台：ESP32-S3 + FreeRTOS
 * 通信总线：UART1(UWB)、I2C0(IMU)、LEDC/PWM(电调)、GPIO中断(编码器)
 */

/* ================================================================
 * 头文件引用
 * ================================================================ */
#include <math.h>       /* atan2f, sqrtf, fabsf, cos, sin 等数学函数 */
#include <stdio.h>      /* 标准输入输出 */
#include <string.h>     /* memset 等内存操作函数 */

/* FreeRTOS 实时操作系统相关头文件 */
#include "freertos/FreeRTOS.h"    /* 任务创建、调度、延时等核心功能 */
#include "freertos/semphr.h"      /* 信号量：互斥锁等线程同步原语 */
#include "freertos/task.h"        /* 任务管理：xTaskCreate, vTaskDelayUntil 等 */

/* ESP-IDF SDK 相关头文件 */
#include "esp_log.h"              /* 日志输出宏：ESP_LOGI, ESP_LOGE 等 */
#include "esp_timer.h"            /* 高精度微秒级定时器 */
#include "sdkconfig.h"            /* Kconfig 菜单配置生成的宏定义（引脚、PID参数等） */

/* 项目内部组件头文件 */
#include "bu_uwb.h"               /* BU0x系列UWB超宽带定位模块驱动 */
#include "chassis.h"              /* 差速底盘闭环控制模块（电调+编码器+运动学） */

/* ESP-IDF 硬件驱动 */
#include "driver/i2c_master.h"    /* I2C主机驱动（用于连接IMU） */
#include "imu_i2c.h"              /* I2C接口的9轴IMU传感器驱动 */

/* ================================================================
 * 常量和宏定义
 * ================================================================ */

static const char *TAG = "follow_only";  /* 日志标签，用于ESP_LOG输出时标识来源 */

/* 数学常量（ESP-IDF的math.h可能未定义M_PI，故手动定义） */
#define M_PI 3.14159265358979323846
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)  /* 角度转弧度 */

/*
 * 硬件安装方向校正标志（硬编码，如需更换电机/编码器安装方向请修改此处）
 * 注意：这些是物理标志，与Kconfig中可配置的参数不同
 */
#define FR_LEFT_INVERT true       /* 左轮电机是否有反转（true=实际接线与逻辑反向） */
#define FR_RIGHT_INVERT true      /* 右轮电机是否有反转 */
#define FR_LEFT_ENC_INVERT true   /* 左轮编码器计数方向是否取反 */
#define FR_RIGHT_ENC_INVERT true  /* 右轮编码器计数方向是否取反 */
#define FR_UWB_LEFT_SIGN 1.0f     /* UWB左侧坐标的符号修正（1=不变，-1=翻转） */
#define FR_IMU_YAW_SIGN -1.0f     /* IMU航向角的符号修正（-1代表取反以匹配坐标约定） */

/* ================================================================
 * 共享数据快照
 *
 * 设计理念：多个传感器任务和唯一的控制任务之间通过"快照"机制通信。
 * 使用互斥锁(mutex)保护共享结构体，避免数据竞态(Data Race)。
 * 控制任务以固定频率读取快照，而非直接阻塞等待每个传感器。
 * ================================================================ */

typedef struct {
    SemaphoreHandle_t lock;       /* 互斥锁句柄，保护下方数据的并发访问 */
    float tgt_distance_m;         /* 目标距离(米)，UWB解算结果 */
    float tgt_bearing_rad;        /* 目标方位角(弧度)，左正右负（CCW为正） */
    uint64_t tgt_ts_us;           /* 目标数据的时间戳(微秒)，用于判断数据新鲜度 */
} shared_t;

static shared_t g_shared;        /* 全局共享数据实例 */
static imu_i2c_t s_imu;          /* IMU传感器句柄 */
static bool s_imu_ok = false;    /* IMU是否初始化成功标志 */
static chassis_t s_chassis;      /* 底盘句柄（电调+编码器+PID状态） */

/* ================================================================
 * 状态机定义
 * ================================================================ */

typedef enum {
    FOLLOW_STATE_IDLE = 0,   /* 空闲：目标丢失超过搜索超时时间，停止运动 */
    FOLLOW_STATE_SEARCH,     /* 搜索：短暂丢失目标，按最后已知方位旋转搜索 */
    FOLLOW_STATE_FOLLOW,     /* 跟随：正常跟随目标，根据距离和方位计算速度指令 */
} follow_state_t;

/**
 * @brief 将状态枚举值转换为可读的字符串，用于日志输出
 * @param s 当前状态枚举值
 * @return 状态名称字符串指针
 */
static const char *state_name(follow_state_t s)
{
    switch (s) {
    case FOLLOW_STATE_IDLE:   return "IDLE";
    case FOLLOW_STATE_SEARCH: return "SEARCH";
    case FOLLOW_STATE_FOLLOW: return "FOLLOW";
    default:                  return "?";
    }
}

/* ================================================================
 * 工具函数
 * ================================================================ */

/** @brief 获取当前微秒时间戳（基于系统启动后的计时） */
static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

/** @brief 获取互斥锁（阻塞等待直到成功获取） */
static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }

/** @brief 释放互斥锁 */
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/**
 * @brief 将浮点数限制在指定范围内
 * @param x 输入值
 * @param lo 下限
 * @param hi 上限
 * @return 被限制后的值
 */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief 将角度归一化到 [-PI, PI] 范围内
 *        循环加/减2π，直到角度落在这个区间内
 * @param a 输入角度(弧度)
 * @return 归一化后的角度(弧度)
 */
static float wrap_pi(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/**
 * @brief 斜坡函数：限制指令变化率，实现平缓的加速/减速
 *        防止电调输出突变导致电流尖峰或机械冲击
 * @param current 当前指令值
 * @param target  目标指令值
 * @param rate    最大变化率（单位/秒）
 * @param dt     自从上次更新以来的时间间隔(秒)
 * @return 经过斜坡限制后的新指令值
 */
static float ramp(float current, float target, float rate, float dt)
{
    if (rate <= 0.0f || dt <= 0.0f) return target;  /* 未启用斜坡限制，直接返回目标值 */
    float step = rate * dt;         /* 本周期允许的最大变化量 */
    float d = target - current;
    if (d > step) d = step;          /* 限制变化不超过步长 */
    else if (d < -step) d = -step;
    return current + d;
}

/* ================================================================
 * UWB 数据读取任务
 *
 * 功能：持续从BU UWB模块的串口读取目标定位数据（TWR双面测距协议或
 *       简单距离数据），解析后写入共享数据快照，供控制任务使用。
 *
 * 优先级：6（较高，确保定位数据实时性）
 * 堆栈大小：4096字节
 * ================================================================ */

static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];       /* UWB串口行缓冲区 */
    const float left_sign = FR_UWB_LEFT_SIGN;  /* 左侧坐标符号修正 */

    while (1) {
        /* 阻塞读取UWB串口的一行数据，超时200ms */
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) continue;

        /* 初始化解析结果结构体 */
        bu_uwb_twr_reading_t twr = {0};   /* TWR双面测距数据 */
        bu_uwb_distance_t dist = {0};      /* 简单距离数据 */

        /* 尝试解析为TWR数据帧（包含距离+方位角信息） */
        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            /* 将厘米单位的TWR坐标转换为米单位 */
            float fwd_m = (float)twr.y_cm / 100.0f;    /* 前方距离(米) */
            float left_m = left_sign * (float)twr.x_cm / 100.0f;  /* 左侧距离(米)，应用符号修正 */
            float range = (twr.distance_cm > 0)
                              ? (float)twr.distance_cm / 100.0f   /* 直接使用距离值 */
                              : sqrtf(fwd_m * fwd_m + left_m * left_m);  /* 或由xy分量计算欧几里得距离 */
            float bearing = 0.0f;
            /* 计算目标方位角：atan2(左侧距离, 前方距离) */
            if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f)
                bearing = atan2f(left_m, fwd_m);

            /* 加锁后更新共享数据快照 */
            lock();
            g_shared.tgt_distance_m = range;
            g_shared.tgt_bearing_rad = bearing;
            g_shared.tgt_ts_us = now_us();
            unlock();
        }
        /* 如果不是TWR帧，尝试解析为简单距离帧（仅包含距离，无方位角） */
        else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            lock();
            g_shared.tgt_distance_m = dist.distance_m;  /* 只更新距离，保留上次方位角 */
            g_shared.tgt_ts_us = now_us();
            unlock();
        }
    }
}

/* ================================================================
 * IMU 航向角读取
 *
 * 功能：从9轴IMU读取欧拉角，提取偏航角(yaw)并应用符号修正。
 *       向左转为正(CCW-positive，与坐标系一致)。
 *
 * @param yaw_rad 输出参数，返回当前偏航角(弧度)
 * @return true=读取成功，false=IMU未就绪或数据无效
 * ================================================================ */

static bool imu_read_yaw(float *yaw_rad)
{
    if (!s_imu_ok) return false;    /* IMU未初始化成功，直接返回失败 */
    imu_i2c_reading_t r;
    memset(&r, 0, sizeof(r));
    if (imu_i2c_read_all(&s_imu, &r) != ESP_OK || !r.valid) return false;
    /* 读取第三个欧拉角(偏航角yaw)，应用符号修正和度转弧度 */
    *yaw_rad = FR_IMU_YAW_SIGN * DEG2RAD(r.euler_deg[2]);
    return true;
}

/* ================================================================
 * 核心控制任务 (control_task)
 *
 * 这是整个跟随系统的"大脑"——以固定频率运行的控制循环。
 * 工作流程：
 *   1. 从共享快照读取UWB目标数据
 *   2. 运行3态状态机(IDLE/SEARCH/FOLLOW)
 *   3. 计算理想速度指令(v, omega)
 *   4. 集成IMU航向闭环修正
 *   5. 应用加速度限制和平滑处理
 *   6. 下发速度指令到底盘（电调+编码器PID闭环执行）
 *   7. 约5Hz频率输出状态日志
 *
 * 优先级：7（最高优先级，确保控制指令实时输出）
 * 堆栈大小：4096字节
 * ================================================================ */

static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;  /* 获取底盘句柄指针 */

    /* ---- 从Kconfig读取所有配置参数并做单位转换 ---- */
    const float follow_distance_m = CONFIG_FOLLOW_ONLY_FOLLOW_DISTANCE_MM / 1000.0f;   /* 目标跟随距离(米) */
    const float stop_band_m = CONFIG_FOLLOW_ONLY_STOP_BAND_MM / 1000.0f;               /* 停止带宽度(米)：在此范围内停止 */
    const float max_linear_mps = CONFIG_FOLLOW_ONLY_MAX_LINEAR_MMPS / 1000.0f;          /* 最大线速度(m/s) */
    const float max_angular_rps = CONFIG_FOLLOW_ONLY_MAX_ANGULAR_MRADPS / 1000.0f;     /* 最大角速度(rad/s) */
    const float kp_dist = CONFIG_FOLLOW_ONLY_KP_DIST / 1000.0f;                        /* 距离误差比例系数P */
    const float kp_bear = CONFIG_FOLLOW_ONLY_KP_BEAR / 1000.0f;                        /* 方位角误差比例系数P */
    const float max_lin_accel = 0.8f;          /* 最大线加速度(m/s²)，用于斜坡限制 */
    const float max_lin_decel = 2.0f;          /* 最大线减速度(m/s²)，刹车比加速快以确保安全 */
    const float max_ang_accel = 6.0f;          /* 最大角加速度(rad/s²) */
    const float search_rps = CONFIG_FOLLOW_ONLY_SEARCH_ANGULAR_MRADPS / 1000.0f;       /* 搜索旋转角速度(rad/s) */
    const float search_timeout_s = (float)CONFIG_FOLLOW_ONLY_SEARCH_TIMEOUT_S;          /* 搜索超时时间(秒) */
    const uint64_t target_fresh_us = (uint64_t)CONFIG_FOLLOW_ONLY_TARGET_FRESH_MS * 1000ULL; /* 目标数据新鲜度阈值(微秒) */
    const float heading_kp = CONFIG_FOLLOW_ONLY_HEADING_KP_MILLI / 1000.0f;             /* IMU航向闭环的Kp系数 */

    /* ---- 状态机变量初始化 ---- */
    follow_state_t state = FOLLOW_STATE_IDLE;    /* 当前状态，初始为空闲 */
    float cmd_v = 0.0f;             /* 当前平滑后的线速度指令(m/s) */
    float cmd_w = 0.0f;             /* 当前平滑后的角速度指令(rad/s) */
    float lost_timer_s = 0.0f;      /* 目标丢失计时器(秒) */
    float search_timer_s = 0.0f;    /* 搜索状态计时器(秒) */
    float last_known_bearing = 0.0f; /* 最后已知的目标方位角(弧度) */
    bool has_last_known = false;    /* 是否有最后已知的方位角 */
    float yaw_ref = 0.0f;           /* IMU航向参考值(弧度) */
    bool yaw_ref_set = false;       /* 航向参考是否已经初始化 */

    /* ---- 定时控制 ---- */
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ONLY_CONTROL_HZ);  /* 控制周期(FreeRTOS ticks) */
    TickType_t last_wake = xTaskGetTickCount();  /* 上一次唤醒时刻 */
    uint64_t prev_us = now_us();                  /* 上一次循环的微秒时间戳 */
    int log_div = 0;                               /* 日志分频计数器 */

    while (1) {
        /* ---- 固定频率等待 ---- */
        vTaskDelayUntil(&last_wake, period);  /* 确保精确的控制周期 */
        const uint64_t t = now_us();          /* 获取当前时间戳 */
        const float dt = (float)(t - prev_us) / 1e6f;  /* 计算实际时间间隔(秒) */
        prev_us = t;

        /* ---- 从共享快照读取UWB目标数据（加锁操作） ---- */
        bool tgt_valid;
        float tgt_dist;
        float tgt_bear;
        lock();
        tgt_valid = (t - g_shared.tgt_ts_us) < target_fresh_us;  /* 检查数据是否在新鲜度窗口内 */
        tgt_dist = g_shared.tgt_distance_m;
        tgt_bear = g_shared.tgt_bearing_rad;
        unlock();

        /* ---- 更新目标丢失/搜索计时器 ---- */
        if (tgt_valid) {
            /* 有有效目标：重置所有计时器，保存方位角 */
            lost_timer_s = 0.0f;
            search_timer_s = 0.0f;
            last_known_bearing = tgt_bear;
            has_last_known = true;
        } else {
            /* 目标丢失：累计计时器 */
            lost_timer_s += dt;
            if (state == FOLLOW_STATE_SEARCH) search_timer_s += dt;
        }

        float v_des = 0.0f;  /* 本周期期望线速度(m/s) */
        float w_des = 0.0f;  /* 本周期期望角速度(rad/s) */

        /* ================================================================
         * 状态机逻辑
         * ================================================================ */

        /* ---- 情况1：目标丢失超过0.5秒 -> 进入搜索或空闲状态 ---- */
        if (!tgt_valid && lost_timer_s > 0.5f) {
            if (has_last_known && search_timer_s <= search_timeout_s) {
                /* 还有最后已知方向且在搜索超时内 -> 旋转搜索 */
                state = FOLLOW_STATE_SEARCH;
                float dir = (last_known_bearing >= 0.0f) ? 1.0f : -1.0f;  /* 按最后方位角方向旋转 */
                v_des = 0.0f;               /* 搜索时不前进 */
                w_des = dir * search_rps;   /* 原地旋转搜索 */
                yaw_ref_set = false;        /* 重置航向参考 */
            } else {
                /* 搜索超时或无最后方向 -> 进入空闲，停止所有运动 */
                state = FOLLOW_STATE_IDLE;
                v_des = 0.0f;
                w_des = 0.0f;
                has_last_known = false;
                yaw_ref_set = false;
            }
        }
        /* ---- 情况2：检测到有效目标 -> 进入跟随状态 ---- */
        else if (tgt_valid) {
            state = FOLLOW_STATE_FOLLOW;

            /* 距离控制：用P控制器计算线速度 */
            float err = tgt_dist - follow_distance_m;   /* 距离误差 = 实际距离 - 目标距离 */
            if (err > 0.0f) {
                /* 距离太远 -> 前进追赶 */
                v_des = kp_dist * err;
            } else if (-err <= stop_band_m) {
                /* 在停止带内 -> 停止 */
                v_des = 0.0f;
            } else {
                /* 距离太近（超过停止带） -> 停止（不后退，因为背部无传感器防护） */
                v_des = 0.0f;
            }
            v_des = clampf(v_des, 0.0f, max_linear_mps);  /* 限制最大线速度 */

            /* 方位控制：用P控制器计算角速度 */
            w_des = kp_bear * tgt_bear;  /* 比例控制：方位角越大，旋转越快 */
            w_des = clampf(w_des, -max_angular_rps, max_angular_rps);  /* 限制最大角速度 */

            /* 转向缩放：目标偏离正前方时降低前进速度，避免"甩尾" */
            /* 偏离90°时速度降为0，正前方时全速 */
            float turn_scale = clampf(1.0f - fabsf(tgt_bear) / (0.5f * (float)M_PI),
                                      0.0f, 1.0f);
            v_des *= turn_scale;

            /* ---- IMU 航向闭环控制 ---- */
            /* 目的：用IMU的实际航向反馈来修正角速度指令 */
            /* 原理：对角速度积分得到理论航向参考，与实际IMU航向比较，用P控制修正 */
            float yaw_meas;
            if (imu_read_yaw(&yaw_meas)) {
                if (!yaw_ref_set) {
                    /* 首次进入跟随状态：以当前航向为参考起点 */
                    yaw_ref = yaw_meas;
                    yaw_ref_set = true;
                }
                /* 对角速度指令积分，得到"本轮期望到达的航向" */
                yaw_ref = wrap_pi(yaw_ref + w_des * dt);
                /* 计算航向误差，用P控制器修正角速度 */
                float err_yaw = wrap_pi(yaw_ref - yaw_meas);
                w_des = w_des + heading_kp * err_yaw;
                w_des = clampf(w_des, -max_angular_rps, max_angular_rps);
            } else {
                /* IMU读取失败，放弃航向闭环，退化为开环角速度控制 */
                yaw_ref_set = false;
            }
        }
        /* ---- 情况3：目标短暂丢失（< 0.5秒），保持当前指令 ---- */
        else {
            v_des = 0.0f;
            w_des = 0.0f;
        }

        /* ---- 加速度限制（斜坡平滑） ---- */
        /* 防止指令突变导致的电机电流冲击和机械冲击 */
        float lin_rate = (v_des >= cmd_v) ? max_lin_accel : max_lin_decel;  /* 加速/减速使用不同斜率 */
        cmd_v = ramp(cmd_v, v_des, lin_rate, dt);
        cmd_w = ramp(cmd_w, w_des, max_ang_accel, dt);

        /* ---- 下发指令到底盘并执行控制更新 ---- */
        chassis_set_velocity(chassis, cmd_v, cmd_w);  /* 设置差速底盘的速度指令 */
        chassis_update(chassis, dt);                   /* 读取编码器、运行PID、写入电调PWM脉冲 */

        /* ---- 日志输出（约5Hz频率，避免串口拥堵） ---- */
        if (++log_div >= CONFIG_FOLLOW_ONLY_CONTROL_HZ / 5) {
            log_div = 0;
            float mv = 0.0f;  /* 编码器实测线速度 */
            float mw = 0.0f;  /* 编码器实测角速度 */
            chassis_get_measured(chassis, &mv, &mw, NULL, NULL);
            ESP_LOGI(TAG,
                     "%-6s tgt=%s d=%.2f br=%+.2f | cmd v=%+.2f w=%+.2f | "
                     "meas v=%+.2f w=%+.2f",
                     state_name(state), tgt_valid ? "Y" : "N",
                     tgt_dist, tgt_bear, cmd_v, cmd_w, mv, mw);
        }
    }
}

/* ================================================================
 * 系统启动入口 app_main
 *
 * 按顺序初始化所有硬件模块，然后启动传感器任务和控制任务。
 * 每个模块的初始化独立进行，允许部分模块失败而不影响整体启动——
 * 这种"优雅降级"设计确保系统在部分传感器故障时仍能工作。
 *
 * 初始化顺序：
 *   1. 共享状态 -> 2. 底盘(电调+编码器) -> 3. UWB -> 4. IMU -> 5. 控制任务
 * ================================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "Follow-only suitcase starting");

    /* ---- 1. 共享状态初始化 ---- */
    memset(&g_shared, 0, sizeof(g_shared));          /* 清空共享结构体 */
    g_shared.lock = xSemaphoreCreateMutex();          /* 创建互斥锁 */

    /* ---- 2. 底盘初始化（电调ESC + 编码器） ---- */
    chassis_config_t cc = chassis_default_config();   /* 获取底盘默认配置 */
    cc.esc_min_us = CONFIG_FOLLOW_ONLY_ESC_MIN_US;    /* 电调最小脉宽(微秒)：通常1000=全速反转 */
    cc.esc_mid_us = CONFIG_FOLLOW_ONLY_ESC_MID_US;    /* 电调中位脉宽(微秒)：通常1500=停止 */
    cc.esc_max_us = CONFIG_FOLLOW_ONLY_ESC_MAX_US;    /* 电调最大脉宽(微秒)：通常2000=全速正转 */
    cc.left_esc_gpio = CONFIG_FOLLOW_ONLY_LEFT_ESC_GPIO;     /* 左电调信号引脚 */
    cc.right_esc_gpio = CONFIG_FOLLOW_ONLY_RIGHT_ESC_GPIO;   /* 右电调信号引脚 */
    cc.left_invert = FR_LEFT_INVERT;                  /* 左轮电机反转标志 */
    cc.right_invert = FR_RIGHT_INVERT;                /* 右轮电机反转标志 */
    cc.left_enc_a_gpio = CONFIG_FOLLOW_ONLY_LEFT_ENC_A_GPIO;   /* 左编码器A相引脚 */
    cc.left_enc_b_gpio = CONFIG_FOLLOW_ONLY_LEFT_ENC_B_GPIO;   /* 左编码器B相引脚 */
    cc.right_enc_a_gpio = CONFIG_FOLLOW_ONLY_RIGHT_ENC_A_GPIO; /* 右编码器A相引脚 */
    cc.right_enc_b_gpio = CONFIG_FOLLOW_ONLY_RIGHT_ENC_B_GPIO; /* 右编码器B相引脚 */
    cc.left_enc_invert = FR_LEFT_ENC_INVERT;          /* 左编码器取反标志 */
    cc.right_enc_invert = FR_RIGHT_ENC_INVERT;        /* 右编码器取反标志 */
    cc.ticks_per_meter = (float)CONFIG_FOLLOW_ONLY_TICKS_PER_METER;  /* 每米编码器4x ticks数（需标定） */
    cc.track_width_m = CONFIG_FOLLOW_ONLY_TRACK_WIDTH_MM / 1000.0f;   /* 轮距(米) */
    cc.max_speed_mps = CONFIG_FOLLOW_ONLY_MAX_WHEEL_SPEED_MMPS / 1000.0f;  /* 单轮最大转速(m/s) */
    cc.kp = (float)CONFIG_FOLLOW_ONLY_SPEED_KP;       /* 轮速PID的Kp系数 */
    cc.ki = (float)CONFIG_FOLLOW_ONLY_SPEED_KI;       /* 轮速PID的Ki系数 */
    cc.kd = (float)CONFIG_FOLLOW_ONLY_SPEED_KD;       /* 轮速PID的Kd系数 */
    cc.pid_out_limit_us = (float)CONFIG_FOLLOW_ONLY_SPEED_PID_LIMIT_US;  /* PID输出限幅(微秒) */

    if (chassis_init(&s_chassis, &cc) == ESP_OK) {
        chassis_stop(&s_chassis);  /* 初始化为停止状态 */
        /*
         * 关键步骤：无刷电调(ESC)上电解锁(Arming)。
         * 电调在上电后需要持续接收一段中位PWM信号（1500微秒）才能解锁。
         * 这段时间内绝对不能发送任何非中位信号，否则电调会进入保护模式。
         */
        ESP_LOGI(TAG, "chassis ready; arming ESC (hold neutral %d ms)",
                 CONFIG_FOLLOW_ONLY_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FOLLOW_ONLY_ESC_ARM_MS));
    } else {
        ESP_LOGE(TAG, "chassis init FAILED - check ESC/encoder GPIOs");
    }

    /* ---- 3. UWB（超宽带定位模块）初始化 ---- */
    /*
     * UWB通过硬件UART与ESP32通信。
     * 使用TWR(Two-Way Ranging，双面测距)协议获取目标的精确距离和方位角。
     */
    bu_uwb_config_t bu = bu_uwb_default_config(
        (uart_port_t)CONFIG_FOLLOW_ONLY_UWB_UART,     /* 使用的UART端口号 */
        CONFIG_FOLLOW_ONLY_UWB_RX_GPIO,               /* RX接收引脚 */
        CONFIG_FOLLOW_ONLY_UWB_TX_GPIO);              /* TX发送引脚 */
    bu.baudrate = CONFIG_FOLLOW_ONLY_UWB_BAUD;        /* 波特率 */

    if (bu_uwb_init(&bu) == ESP_OK) {
        xTaskCreate(uwb_task, "uwb", 4096, NULL, 6, NULL);  /* 创建UWB数据读取任务 */
        ESP_LOGI(TAG, "uwb ready (RX=GPIO%d)", CONFIG_FOLLOW_ONLY_UWB_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "uwb init FAILED");
    }

    /* ---- 4. IMU（惯性测量单元）初始化 ---- */
    /*
     * IMU通过I2C总线与ESP32通信，提供9轴姿态数据。
     * 这里只使用偏航角(yaw)进行航向闭环控制。
     * 如果IMU初始化失败，系统仍可运行（退化为开环角速度控制）。
     */
    static i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = 0,                                /* 使用I2C0端口 */
        .sda_io_num = CONFIG_FOLLOW_ONLY_I2C_SDA_GPIO, /* SDA数据线引脚 */
        .scl_io_num = CONFIG_FOLLOW_ONLY_I2C_SCL_GPIO, /* SCL时钟线引脚 */
        .clk_source = I2C_CLK_SRC_DEFAULT,             /* 使用默认时钟源 */
        .glitch_ignore_cnt = 7,                        /* 硬件抗尖峰脉冲滤波 */
        .flags.enable_internal_pullup = true,          /* 启用内部上拉电阻 */
    };

    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) == ESP_OK) {
        imu_i2c_config_t imucfg = imu_i2c_default_config();
        imucfg.sda_gpio = CONFIG_FOLLOW_ONLY_I2C_SDA_GPIO;
        imucfg.scl_gpio = CONFIG_FOLLOW_ONLY_I2C_SCL_GPIO;
        imucfg.device_address = CONFIG_FOLLOW_ONLY_IMU_ADDR;   /* IMU的I2C设备地址 */
        imucfg.external_bus = i2c_bus;                         /* 使用已创建的I2C总线 */

        if (imu_i2c_init(&s_imu, &imucfg) == ESP_OK) {
            s_imu_ok = true;  /* 标记IMU就绪 */
            ESP_LOGI(TAG, "imu ready (heading loop ON)");
        } else {
            ESP_LOGE(TAG, "imu init FAILED - heading loop disabled");
        }
    } else {
        ESP_LOGE(TAG, "I2C bus init FAILED");
    }

    /* ---- 5. 启动核心控制任务 ---- */
    /*
     * 控制任务优先级设为最高(7)，高于所有传感器任务(6)，
     * 确保控制指令的计算不受传感器数据采集的干扰。
     * 控制频率由 Kconfig 中的 CONFIG_FOLLOW_ONLY_CONTROL_HZ 决定。
     */
    xTaskCreate(control_task, "control", 4096, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control loop running at %d Hz", CONFIG_FOLLOW_ONLY_CONTROL_HZ);
}
