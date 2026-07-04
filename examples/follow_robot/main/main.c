/*
 * follow_robot (算法2) - 智能跟随行李箱，带闭环驱动和VFH避障。
 *
 * 本程序是完整版跟随系统，在follow_only的基础上增加了：
 *   - 激光雷达(Lidar)：中远距离360°环境障碍物扫描
 *   - 超声波传感器：行李箱前左右边角的近距离安全防护
 *   - VFH(Vector Field Histogram)避障算法：实时路径规划绕开障碍物
 *
 * 传感器/执行器架构（感知-控制-执行栈）：
 *   UWB (BU0x)        -> 跟随目标定位（获取目标标签的距离和方位角）
 *   RPLIDAR C1        -> 障碍物场景（前方180度极坐标直方图）
 *   2x A02YYUW        -> 前角近场安全保护（超声波测距，不怕光线干扰）
 *   IMU               -> 航向闭环控制（偏航角误差修正转向指令）
 *   chassis (底盘)    -> 后轮差速驱动闭环：APO-DL ESC (RC PWM 50Hz) + AB编码器PID
 *
 * 与"算法1"的区别：
 *   本程序的底盘是"闭环"的。它通过LEDC模块发出真实的RC伺服脉冲驱动APO-DL
 *   无刷电调，并用GPIO中断捕捉AB编码器的4倍频脉冲，在每个轮子上运行独立的
 *   PID速度环。控制循环中还引入了IMU航向闭环——这意味着上位机下发的
 *   (v线速度, omega角速度)指令能真正被精确执行，而不是靠开环占空比"猜"。
 *
 * 软件架构：每个传感器在自己独立的FreeRTOS任务中运行，通过互斥锁保护的
 * "快照"结构体向控制任务发布数据。一个固定频率的控制任务从快照中读取
 * 所有数据，运行VFH避障算法，叠加IMU航向闭环修正，最后把指令发送到底盘
 * (chassis_set_velocity + chassis_update)。
 *
 * 优雅降级策略：
 *   若任一传感器未能启动，其数据始终保持"过时/无效"状态：
 *   - 无激光雷达 -> 仅依靠超声波进行近距离避障
 *   - 无UWB -> 进入搜索态然后空闲
 *   - 无编码器 -> 回退到纯前馈控制
 *   - 无IMU -> 退化为开环角速度控制
 *
 * 状态机：5态状态机：IDLE(空闲) -> SEARCH(搜索) -> FOLLOW(跟随)
 *                    -> AVOID(避障) -> ESTOP(紧急停止)
 */

/* ================================================================
 * 头文件引用
 * ================================================================ */
#include <math.h>       /* atan2f, sqrtf, fabsf, cos, sin 等数学函数 */
#include <stdio.h>      /* 标准输入输出 */
#include <string.h>     /* memset 等内存操作函数 */

/* FreeRTOS 实时操作系统 */
#include "freertos/FreeRTOS.h"    /* 任务创建、调度、延时等核心功能 */
#include "freertos/semphr.h"      /* 信号量（互斥锁） */
#include "freertos/task.h"        /* 任务管理：xTaskCreate, vTaskDelayUntil 等 */

/* ESP-IDF SDK */
#include "esp_log.h"              /* 日志输出宏 */
#include "esp_timer.h"            /* 高精度微秒级定时器 */
#include "sdkconfig.h"            /* Kconfig 菜单配置生成的宏定义 */

/* 项目内部组件（传感器驱动 + 控制算法） */
#include "a02yyuw.h"              /* A02YYUW超声波传感器驱动（UART/软件模拟串口） */
#include "bu_uwb.h"               /* BU0x UWB超宽带定位模块驱动 */
#include "rplidar_c1.h"           /* SLAMTEC RPLIDAR C1激光雷达驱动 */
#include "chassis.h"              /* 差速底盘闭环控制模块 */
#include "follow_avoid.h"         /* VFH(Vector Field Histogram)避障算法 */

/* ESP-IDF 硬件驱动 */
#include "driver/i2c_master.h"    /* I2C主机驱动（用于连接IMU） */
#include "imu_i2c.h"              /* I2C接口的9轴IMU驱动 */

/* ================================================================
 * 常量和宏定义
 * ================================================================ */

static const char *TAG = "follow_robot";  /* 日志标签 */

/* 数学常量 */
#define M_PI 3.14159265358979323846
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)  /* 角度转弧度 */

/* 硬件安装方向校正标志 */
#define FR_LEFT_INVERT true       /* 左轮电机反转标志 */
#define FR_RIGHT_INVERT true      /* 右轮电机反转标志 */
#define FR_LEFT_ENC_INVERT true   /* 左轮编码器取反标志 */
#define FR_RIGHT_ENC_INVERT true  /* 右轮编码器取反标志 */
#define FR_UWB_LEFT_SIGN 1.0f     /* UWB左侧坐标符号修正 */
#define FR_HEADING_HOLD true      /* 是否启用IMU航向闭环保持 */
#define FR_IMU_YAW_SIGN -1.0f     /* IMU偏航角符号修正 */
#define CONFIG_FOLLOW_ROBOT_HEADING_KP_MILLI 0  /* IMU航向闭环Kp系数 */

/* 激光雷达障碍物场景参数 */
#define LIDAR_SECTORS 36                 /* 扇区数量：180度FOV按5度/扇区划分 */
#define LIDAR_FOV_RAD ((float)M_PI)      /* 激光雷达视场角(弧度)：180度 */

/*
 * 数据新鲜度窗口：超过此时间的数据视为"过时/无效"。
 * 这是传感器数据质量的"保质期"——如果某个传感器长时间
 * 未更新，系统将忽略其数据，确保控制决策基于实时信息。
 */
#define TARGET_FRESH_US 700000ULL        /* 目标数据新鲜度：0.7秒 */
#define FIELD_FRESH_US 500000ULL         /* 雷达场景新鲜度：0.5秒 */
#define ULTRA_FRESH_US 500000ULL         /* 超声波数据新鲜度：0.5秒 */

/* ================================================================
 * 共享数据快照结构体
 *
 * 所有传感器任务将数据写入此结构体，控制任务从中读取。
 * 使用互斥锁(mutex)保护并发访问，避免数据竞态(Data Race)。
 * 每个数据字段都附加时间戳，控制任务根据新鲜度窗口判断
 * 数据的有效性，从而实现传感器数据的"软实时"融合。
 * ================================================================ */

typedef struct {
    SemaphoreHandle_t lock;        /* 互斥锁句柄 */

    /* UWB目标定位数据 */
    float tgt_distance_m;          /* 目标距离(米) */
    float tgt_bearing_rad;         /* 目标方位角(弧度)，正=左侧(CCW) */
    uint64_t tgt_ts_us;            /* 目标数据时间戳(微秒) */

    /* 激光雷达障碍物场景（最近一次完整扫描） */
    fa_obstacle_field_t field;     /* VFH障碍物势场/极坐标直方图 */
    uint64_t field_ts_us;          /* 雷达场景时间戳(微秒) */

    /* 超声波传感器数据（行李箱前方左右边角近场安全） */
    float ul_m;                    /* 左前侧超声波距离(米) */
    uint64_t ul_ts_us;             /* 左超声波时间戳(微秒) */
    float ur_m;                    /* 右前侧超声波距离(米) */
    uint64_t ur_ts_us;             /* 右超声波时间戳(微秒) */
} shared_t;

static shared_t g_shared;  /* 全局共享数据实例 */

/* ================================================================
 * 基础工具函数
 * ================================================================ */

/** @brief 获取当前微秒时间戳 */
static inline uint64_t now_us(void) { return (uint64_t)esp_timer_get_time(); }

/** @brief 获取互斥锁（阻塞等待） */
static void lock(void) { xSemaphoreTake(g_shared.lock, portMAX_DELAY); }

/** @brief 释放互斥锁 */
static void unlock(void) { xSemaphoreGive(g_shared.lock); }

/* ================================================================
 * UWB 目标定位任务
 *
 * 任务职责：持续从BU UWB模块的串口读取目标定位数据，解析TWR双面
 * 测距协议或简单距离数据帧，将结果写入共享快照。
 *
 * TWR协议提供 x/y/z 三维坐标，我们取 x(左右) 和 y(前后)
 * 分量来计算目标距离和方位角。如果仅收到距离帧，则只更新距离。
 *
 * 优先级：6（较高，定位数据是跟随的基础）
 * 堆栈：4096字节
 * ================================================================ */

static void uwb_task(void *arg)
{
    (void)arg;
    char line[BU_UWB_LINE_MAX];       /* UWB串口行缓冲区 */
    const float left_sign = FR_UWB_LEFT_SIGN;  /* 左侧坐标符号修正 */

    while (1) {
        /* 阻塞读取UWB串口的一行完整数据，超时200ms */
        if (bu_uwb_read_line(line, sizeof(line), 200) != ESP_OK) {
            continue;  /* 超时或读取失败则继续等待 */
        }

        bu_uwb_twr_reading_t twr = {0};   /* TWR双面测距数据结构体 */
        bu_uwb_distance_t dist = {0};      /* 简单距离数据结构体 */

        /* 尝试解析为TWR数据帧 */
        if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
            /* 将厘米转换为米 */
            const float fwd_m = (float)twr.y_cm / 100.0f;     /* 前后方向距离(米) */
            const float left_m = left_sign * (float)twr.x_cm / 100.0f;  /* 左右方向距离(米) */
            /* 计算欧几里得距离：优先使用TWR直接输出，否则由xy分量计算 */
            float range = (twr.distance_cm > 0) ? (float)twr.distance_cm / 100.0f
                                                : sqrtf(fwd_m * fwd_m + left_m * left_m);
            /* 计算方位角：atan2(左, 前)，左为正(CCW) */
            float bearing = 0.0f;
            if (fabsf(fwd_m) > 1e-3f || fabsf(left_m) > 1e-3f) {
                bearing = atan2f(left_m, fwd_m);
            }
            lock();
            g_shared.tgt_distance_m = range;
            g_shared.tgt_bearing_rad = bearing;
            g_shared.tgt_ts_us = now_us();  /* 记录更新时间戳 */
            unlock();
        } else if (bu_uwb_parse_distance_line(line, &dist) && dist.valid) {
            /* 仅距离帧：刷新距离，保留上次已知的方位角 */
            lock();
            g_shared.tgt_distance_m = dist.distance_m;
            g_shared.tgt_ts_us = now_us();
            unlock();
        }
    }
}

/* ================================================================
 * 激光雷达 障碍物场景构建任务
 *
 * 任务职责：持续从RPLIDAR C1激光雷达读取扫描点云数据，将有效点
 * 投影到机器人坐标系下的VFH极坐标直方图中。
 *
 * 关键处理：雷达的0°在默认方向，需要根据实际安装角度进行
 * 坐标系转换。每圈开始时，将上一圈的完整场景发布到共享快照。
 *
 * 注意：由于ESP32算力有限且行李箱后方有用户遮挡，这里仅采集
 * 正前方0~65度和后方295~360度（对应前方左右各约65度的扇区），
 * 减少无效数据处理的CPU开销。
 *
 * 优先级：6（与UWB同级）
 * 堆栈：4096字节
 * ================================================================ */

/**
 * @brief 将激光雷达的原始角度转换为机器人身体坐标系下的角度
 *
 * 雷达以CW为正方向旋转（从上方看顺时针），但机器人坐标系
 * 约定CCW为正（从上方看逆时针），所以需要取反。
 *
 * @param raw_deg 雷达原始角度(度)
 * @return 机器人坐标系下的角度(弧度)，CCW为正
 */
static float lidar_angle_to_body_rad(float raw_deg)
{
    /* 减去雷达正前方的安装偏角 */
    float rel = raw_deg - (float)CONFIG_FOLLOW_ROBOT_LIDAR_FORWARD_DEG;
    rel = -rel; /* 雷达CW正 -> 机器人CCW正，取反 */
    /* 角度归一化到 [-180, 180] 度 */
    while (rel > 180.0f) {
        rel -= 360.0f;
    }
    while (rel < -180.0f) {
        rel += 360.0f;
    }
    return DEG2RAD(rel);  /* 转换为弧度并返回 */
}

static void lidar_task(void *arg)
{
    rplidar_c1_t *lidar = (rplidar_c1_t *)arg;
    fa_obstacle_field_t work;  /* 当前正在构建的障碍物场景 */
    fa_obstacle_reset(&work, LIDAR_SECTORS, LIDAR_FOV_RAD);  /* 初始化势场 */

    while (1) {
        rplidar_c1_point_t p = {0};
        /* 从雷达读取一个扫描点 */
        if (!rplidar_c1_read_point(lidar, &p)) {
            vTaskDelay(pdMS_TO_TICKS(1));  /* 无数据则短暂延时 */
            continue;
        }
        /* start_bit为真表示新的一圈扫描开始 */
        if (p.start_bit) {
            /* 将上一圈建好的场景发布到共享快照 */
            lock();
            g_shared.field = work;
            g_shared.field_ts_us = now_us();
            unlock();
            /* 重置势场，开始新一轮构建 */
            fa_obstacle_reset(&work, LIDAR_SECTORS, LIDAR_FOV_RAD);
        }
        /* 过滤无效点：距离>0且信号质量>0才有效 */
        if (p.distance_mm > 0.0f && p.quality > 0) {
            /* 仅采集前方0~65度和后方295~360度的有效点（减少运算量） */
            if ((p.angle_deg >= 0.0f && p.angle_deg <= 65.0f) || 
                (p.angle_deg >= 295.0f && p.angle_deg <= 360.0f)) {
                
                /* 转换到机器人坐标系并加入障碍物场景 */
                const float body = lidar_angle_to_body_rad(p.angle_deg);
                fa_obstacle_add(&work, body, p.distance_mm / 1000.0f);  /* mm转m */
            }
        }
    }
}

/* ================================================================
 * 超声波传感器任务
 *
 * 任务职责：持续轮询A02YYUW超声波传感器，获取行李箱前方左右边角
 * 的近场障碍物距离。
 *
 * 设计说明：两个超声波模块通过软件模拟串口(SW-UART)连接，不占用
 * 有限的硬件UART资源（UART1给了UWB，UART2给了激光雷达）。
 *
 * 优先级：5（低于核心传感器）
 * 堆栈：3072字节
 * ================================================================ */

typedef struct {
    a02yyuw_t *dev;    /* 超声波设备句柄 */
    bool is_left;      /* 是否为左侧传感器（true=左, false=右） */
} ultra_arg_t;  /* 超声波任务参数结构体 */

static void ultra_task(void *arg)
{
    ultra_arg_t *ua = (ultra_arg_t *)arg;
    while (1) {
        a02yyuw_reading_t r = {0};
        /* 阻塞读取超声波距离数据，超时120ms */
        if (a02yyuw_read_dev(ua->dev, &r, 120) == ESP_OK && r.valid) {
            const float m = (float)r.distance_mm / 1000.0f;  /* mm转m */
            lock();
            /* 根据左右标志写入对应的共享字段 */
            if (ua->is_left) {
                g_shared.ul_m = m;
                g_shared.ul_ts_us = now_us();
            } else {
                g_shared.ur_m = m;
                g_shared.ur_ts_us = now_us();
            }
            unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(20));  /* 20ms轮询间隔 */
    }
}

/* ================================================================
 * IMU（惯性测量单元）航向角读取
 *
 * 功能：从9轴IMU读取欧拉角，提取偏航角(yaw)，应用符号修正后
 * 返回弧度值（CCW为正）。
 *
 * IMU初始化失败时，系统自动退化为开环角速度控制——
 * 这是典型的"优雅降级"设计。
 *
 * @param yaw_rad 输出参数，当前偏航角(弧度)，CCW为正
 * @return true=读取成功，false=IMU未就绪或数据无效
 * ================================================================ */
static imu_i2c_t s_imu;          /* IMU传感器句柄 */
static bool s_imu_ok = false;    /* IMU是否初始化成功的标志 */

static bool imu_read_yaw(float *yaw_rad)
{
    if (!s_imu_ok) {
        return false;  /* IMU未就绪，直接返回失败 */
    }
    imu_i2c_reading_t r;
    memset(&r, 0, sizeof(r));
    /* 读取IMU的全量数据（加速度+欧拉角） */
    if (imu_i2c_read_all(&s_imu, &r) != ESP_OK || !r.valid) {
        return false;
    }
    /* 提取偏航角(euler_deg[2]=yaw)，应用符号修正并转换为弧度 */
    *yaw_rad = FR_IMU_YAW_SIGN * DEG2RAD(r.euler_deg[2]);
    return true;
}

/* ================================================================
 * 避障算法配置构建
 *
 * 从Kconfig菜单配置中读取所有避障相关参数，构建follow_avoid
 * 算法所需的配置结构体。单位从mm/mrad转换为m/rad。
 * ================================================================ */

static fa_config_t build_fa_config(void)
{
    fa_config_t c = fa_default_config();  /* 获取默认配置 */
    c.follow_distance_m = CONFIG_FOLLOW_ROBOT_FOLLOW_DISTANCE_MM / 1000.0f;    /* 目标跟随距离(米) */
    c.stop_band_m = CONFIG_FOLLOW_ROBOT_STOP_BAND_MM / 1000.0f;                /* 停止带宽度(米) */
    c.max_linear_mps = CONFIG_FOLLOW_ROBOT_MAX_LINEAR_MMPS / 1000.0f;          /* 最大线速度(m/s) */
    c.max_angular_rps = CONFIG_FOLLOW_ROBOT_MAX_ANGULAR_MRADPS / 1000.0f;     /* 最大角速度(rad/s) */
    c.emergency_distance_m = CONFIG_FOLLOW_ROBOT_EMERGENCY_DIST_MM / 1000.0f;  /* 紧急停止距离(米) */
    c.slow_distance_m = CONFIG_FOLLOW_ROBOT_SLOW_DIST_MM / 1000.0f;            /* 减速距离(米) */
    c.safe_distance_m = CONFIG_FOLLOW_ROBOT_SAFE_DIST_MM / 1000.0f;            /* 安全距离(米) */
    c.robot_half_width_m = CONFIG_FOLLOW_ROBOT_ROBOT_HALF_WIDTH_MM / 1000.0f;  /* 机器人半宽(米) */
    return c;
}

/* ================================================================
 * 底盘配置构建
 *
 * 从Kconfig菜单配置中读取所有底盘硬件相关参数，构建chassis
 * 模块所需的配置结构体。参数包括：
 *   - ESC电调PWM脉宽范围（1000~2000微秒）
 *   - 编码器引脚和方向
 *   - 轮距、轮速、PID参数
 * ================================================================ */

static chassis_config_t build_chassis_config(void)
{
    chassis_config_t cc = chassis_default_config();  /* 获取默认配置 */
    cc.esc_min_us = CONFIG_FOLLOW_ROBOT_ESC_MIN_US;    /* 电调最小脉宽(微秒)，全速反转 */
    cc.esc_mid_us = CONFIG_FOLLOW_ROBOT_ESC_MID_US;    /* 电调中位脉宽(微秒)，停止 */
    cc.esc_max_us = CONFIG_FOLLOW_ROBOT_ESC_MAX_US;    /* 电调最大脉宽(微秒)，全速正转 */
    cc.left_esc_gpio = CONFIG_FOLLOW_ROBOT_LEFT_ESC_GPIO;     /* 左轮电调信号引脚 */
    cc.right_esc_gpio = CONFIG_FOLLOW_ROBOT_RIGHT_ESC_GPIO;   /* 右轮电调信号引脚 */
    cc.left_invert = FR_LEFT_INVERT;                  /* 左轮电机反转标志 */
    cc.right_invert = FR_RIGHT_INVERT;                /* 右轮电机反转标志 */

    cc.left_enc_a_gpio = CONFIG_FOLLOW_ROBOT_LEFT_ENC_A_GPIO;   /* 左编码器A相引脚 */
    cc.left_enc_b_gpio = CONFIG_FOLLOW_ROBOT_LEFT_ENC_B_GPIO;   /* 左编码器B相引脚 */
    cc.right_enc_a_gpio = CONFIG_FOLLOW_ROBOT_RIGHT_ENC_A_GPIO; /* 右编码器A相引脚 */
    cc.right_enc_b_gpio = CONFIG_FOLLOW_ROBOT_RIGHT_ENC_B_GPIO; /* 右编码器B相引脚 */
    cc.left_enc_invert = FR_LEFT_ENC_INVERT;          /* 左编码器取反标志 */
    cc.right_enc_invert = FR_RIGHT_ENC_INVERT;        /* 右编码器取反标志 */
    cc.ticks_per_meter = (float)CONFIG_FOLLOW_ROBOT_TICKS_PER_METER;  /* 每米编码器4x ticks数（需实际标定） */

    cc.track_width_m = CONFIG_FOLLOW_ROBOT_TRACK_WIDTH_MM / 1000.0f;  /* 轮距(米) */
    cc.max_speed_mps = CONFIG_FOLLOW_ROBOT_MAX_WHEEL_SPEED_MMPS / 1000.0f;  /* 单轮最大转速(m/s) */

    cc.kp = (float)CONFIG_FOLLOW_ROBOT_SPEED_KP;       /* 轮速PID Kp系数 */
    cc.ki = (float)CONFIG_FOLLOW_ROBOT_SPEED_KI;       /* 轮速PID Ki系数 */
    cc.kd = (float)CONFIG_FOLLOW_ROBOT_SPEED_KD;       /* 轮速PID Kd系数 */
    cc.pid_out_limit_us = (float)CONFIG_FOLLOW_ROBOT_SPEED_PID_LIMIT_US;  /* PID输出限幅(微秒) */
    return cc;
}

/**
 * @brief 将VFH避障算法的状态枚举转换为可读字符串
 * @param s 状态枚举值
 * @return 状态名称字符串
 */
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

/* ================================================================
 * 核心控制任务 (control_task)
 *
 * 这是整个跟随机器人系统的"大脑"——以固定频率运行的控制循环。
 * 设计原则：
 *   1. 定期快照所有传感器数据（带新鲜度校验）
 *   2. 运行VFH避障算法得到理想速度指令
 *   3. 叠加IMU航向闭环修正（消除万向轮拖拽导致的航向偏差）
 *   4. 下发指令到底盘闭环执行
 *
 * 为什么需要IMU航向闭环？
 *   行李箱的两个前轮是无动力的万向轮(Caster)。当地面不平坦或有
 *   侧向力时，万向轮会自行偏转导致行李箱的实际转向与指令角速度
 *   不匹配。IMU航向闭环通过比较"理论航向"(对角速度积分)和"实际
 *   航向"(IMU测量值)的差异，自动补偿这个偏差。
 *
 * 优先级：7（最高优先级，确保控制指令实时输出）
 * 堆栈：4096字节
 * ================================================================ */

static void control_task(void *arg)
{
    chassis_t *chassis = (chassis_t *)arg;  /* 获取底盘句柄 */

    /* ---- 初始化VFH避障算法上下文和配置 ---- */
    fa_ctx_t fa;
    fa_init(&fa, NULL);            /* 初始化算法状态 */
    fa.cfg = build_fa_config();    /* 读取Kconfig配置参数 */
    const float max_omega = fa.cfg.max_angular_rps;  /* 最大角速度(rad/s) */
    const float heading_kp = CONFIG_FOLLOW_ROBOT_HEADING_KP_MILLI / 1000.0f;  /* IMU航向闭环Kp */
    const bool heading_hold = FR_HEADING_HOLD;  /* 是否启用航向闭环 */
    float yaw_ref = 0.0f;          /* IMU航向参考值(弧度)——"理论航向" */
    bool yaw_ref_set = false;      /* 航向参考是否已初始化 */

    /* ---- 定时控制：按固定频率运行控制循环 ---- */
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FOLLOW_ROBOT_CONTROL_HZ);  /* 控制周期(ticks) */
    TickType_t last_wake = xTaskGetTickCount();  /* 上一次唤醒时刻 */
    uint64_t prev_us = now_us();                  /* 上一次循环的时间戳 */
    int log_div = 0;                               /* 日志分频计数器 */

    while (1) {
        /* ---- 固定频率等待，确保精确的控制周期 ---- */
        vTaskDelayUntil(&last_wake, period);
        const uint64_t t = now_us();          /* 当前时间戳 */
        const float dt = (float)(t - prev_us) / 1e6f;  /* 本周期实际时间间隔(秒) */
        prev_us = t;

        /* ---- 从共享快照读取所有传感器数据（加锁操作） ---- */
        fa_target_t target = {0};       /* VFH算法所需的目标数据 */
        fa_obstacle_field_t field;      /* VFH算法所需的障碍物场景 */
        fa_range_t ul = {0};            /* 左超声波距离数据 */
        fa_range_t ur = {0};            /* 右超声波距离数据 */
        bool have_field;                /* 雷达场景是否有效 */

        lock();
        /* UWB目标数据：检查新鲜度窗口 */
        target.valid = (t - g_shared.tgt_ts_us) < TARGET_FRESH_US;
        target.distance_m = g_shared.tgt_distance_m;
        target.bearing_rad = g_shared.tgt_bearing_rad;

        /* 激光雷达场景数据：检查新鲜度窗口 */
        have_field = (t - g_shared.field_ts_us) < FIELD_FRESH_US;
        field = g_shared.field;

        /* 超声波数据：分别检查左右两侧的新鲜度 */
        ul.valid = (t - g_shared.ul_ts_us) < ULTRA_FRESH_US;
        ul.dist_m = g_shared.ul_m;
        ur.valid = (t - g_shared.ur_ts_us) < ULTRA_FRESH_US;
        ur.dist_m = g_shared.ur_m;
        unlock();

        /* ---- 运行VFH避障算法，计算理想速度指令 ---- */
        fa_output_t out = fa_update(&fa, &target,
                                    have_field ? &field : NULL,   /* 无雷达则传NULL，算法内部降级处理 */
                                    &ul, &ur, dt);

        /* ---- IMU航向闭环控制 ---- */
        /*
         * 算法的omega是"前馈意图"——希望机器人以这个角速度转向。
         * 我们将其积分得到"理论航向"(yaw_ref)，与IMU测量的"实际航向"
         * 比较，用P控制器修正角速度指令。
         *
         * 只在真正跟踪用户(FOLLOW/AVOID状态)时启用航向闭环，
         * SEARCH和ESTOP状态下机器人需要自由旋转，不应被航向锁死。
         */
        float omega_cmd = out.omega_rps;  /* 最终输出到电机的角速度指令 */
        const bool tracking =
            (out.state == FA_STATE_FOLLOW || out.state == FA_STATE_AVOID);
        float yaw_meas;
        if (heading_hold && tracking && imu_read_yaw(&yaw_meas)) {
            /* 航向闭环激活条件全满足 */
            if (!yaw_ref_set) {
                /* 首次进入跟踪状态：以当前IMU航向作为参考起点 */
                yaw_ref = yaw_meas;
                yaw_ref_set = true;
            }
            /* 理论航向 = 上一周期参考 + 算法期望角速度的积分 */
            yaw_ref = fa_wrap_pi(yaw_ref + out.omega_rps * dt);
            /* 计算航向误差，用P控制修正角速度指令 */
            const float err = fa_wrap_pi(yaw_ref - yaw_meas);
            omega_cmd = out.omega_rps + heading_kp * err;
            /* 限幅保护 */
            if (omega_cmd > max_omega) {
                omega_cmd = max_omega;
            } else if (omega_cmd < -max_omega) {
                omega_cmd = -max_omega;
            }
        } else {
            /* IMU不可用或不需要航向保持：重置参考，退化为开环控制 */
            yaw_ref_set = false;
        }

        /* ---- 下发指令到底盘并执行控制更新 ---- */
        chassis_set_velocity(chassis, out.v_mps, omega_cmd);  /* 设置差速底盘的速度指令 */
        chassis_update(chassis, dt);  /* 读取编码器、运行PID、写入电调PWM脉冲 */

        /* ---- 日志输出（约5Hz频率） ---- */
        if (++log_div >= CONFIG_FOLLOW_ROBOT_CONTROL_HZ / 5) {
            log_div = 0;
            float mv = 0.0f;  /* 编码器实测线速度 */
            float mw = 0.0f;  /* 编码器实测角速度 */
            chassis_get_measured(chassis, &mv, &mw, NULL, NULL);
            ESP_LOGI(TAG,
                     "%-6s tgt=%s d=%.2f br=%+.2f | clr=%.2f blk=%d | "
                     "cmd v=%+.2f w=%+.2f | meas v=%+.2f w=%+.2f",
                     state_name(out.state), target.valid ? "Y" : "N",
                     target.distance_m, target.bearing_rad,
                     out.front_clearance_m, out.blocked, out.v_mps, omega_cmd,
                     mv, mw);
        }
    }
}

/* ================================================================
 * 系统启动入口 app_main
 *
 * 初始化顺序：
 *   1. 共享状态
 *   2. 底盘(电调+编码器) —— 需等待ESC解锁(arming)
 *   3. UWB目标定位
 *   4. 激光雷达(可选，失败则降级)
 *   5. 超声波传感器(左右各一)
 *   6. IMU(可选，失败则降级)
 *   7. 核心控制任务
 *
 * 设计亮点：
 *   - 每个初始化步骤独立进行，不因一个模块失败而阻止后续模块
 *   - 超声波使用软件模拟串口(SW-UART)，节省有限的硬件UART资源
 *   - 初始化失败=非致命错误，系统在所有有效传感器下尽力运行
 *   - 控制任务优先级最高(7)，确保实时性
 * ================================================================ */

static rplidar_c1_t s_lidar;             /* 激光雷达句柄 */
static a02yyuw_t s_ultra_left;           /* 左超声波句柄 */
static a02yyuw_t s_ultra_right;          /* 右超声波句柄 */
static chassis_t s_chassis;              /* 底盘句柄 */
static ultra_arg_t s_ua_left = {.dev = &s_ultra_left, .is_left = true};    /* 左超声波任务参数 */
static ultra_arg_t s_ua_right = {.dev = &s_ultra_right, .is_left = false}; /* 右超声波任务参数 */

void app_main(void)
{
    // 打印系统启动日志，标明当前运行的是"算法2（闭环控制）"版本的跟随程序
    ESP_LOGI(TAG, "Follow-me suitcase (算法2, closed-loop) starting");

    /* --- 1. 全局共享状态初始化 --- */
    // 清空全局共享结构体，该结构体用于在多个传感器任务和控制任务之间传递数据
    memset(&g_shared, 0, sizeof(g_shared));
    // 创建一个互斥锁(Mutex)，用于保护 g_shared，防止多任务并发读写时出现数据竞态(Data Race)
    g_shared.lock = xSemaphoreCreateMutex();
    // 初始化避障势场/障碍物地图，传入雷达的扇区数量和视场角(FOV)
    fa_obstacle_reset(&g_shared.field, LIDAR_SECTORS, LIDAR_FOV_RAD);

    /* --- 2. 底盘系统初始化 (ESC电调 + 编码器闭环) --- */
    chassis_config_t cc = build_chassis_config(); // 获取底盘硬件配置（如引脚分配、PID参数等）
    if (chassis_init(&s_chassis, &cc) == ESP_OK) {
        chassis_stop(&s_chassis); // 确保初始化后电机处于停止/中立状态
        
        // 【关键步骤】无刷电机电调(ESC)在上电时需要一段持续的中立位PWM信号进行"解锁(Arming)"，否则无法驱动。
        ESP_LOGI(TAG, "chassis ready; arming ESC (hold neutral %d ms)",
                 CONFIG_FOLLOW_ROBOT_ESC_ARM_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_FOLLOW_ROBOT_ESC_ARM_MS)); // 阻塞延时，等待解锁完成
    } else {
        // 如果初始化失败，通常是GPIO配置冲突或接线错误
        ESP_LOGE(TAG, "chassis init FAILED - check ESC/encoder GPIOs");
    }

    /* --- 3. UWB (超宽带) 目标定位模块初始化 --- */
    // UWB用于精准获取跟随目标（主人带的标签）的相对距离和角度
    // 配置UWB使用的硬件串口（通常是 UART1）及引脚
    bu_uwb_config_t bu = bu_uwb_default_config(
        (uart_port_t)CONFIG_FOLLOW_ROBOT_UWB_UART,           /* UART端口号 */
        CONFIG_FOLLOW_ROBOT_UWB_RX_GPIO,                     /* RX接收引脚 */
        CONFIG_FOLLOW_ROBOT_UWB_TX_GPIO);                    /* TX发送引脚 */
    bu.baudrate = CONFIG_FOLLOW_ROBOT_UWB_BAUD;              /* 波特率 */
    
    if (bu_uwb_init(&bu) == ESP_OK) {
        // 创建独立任务处理UWB数据解算。堆栈4096字节，优先级设为6（较高优先级，确保定位实时性）
        xTaskCreate(uwb_task, "uwb", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "uwb ready (RX=GPIO%d)", CONFIG_FOLLOW_ROBOT_UWB_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "uwb init FAILED");
    }

    /* --- 4. 激光雷达 (Lidar) 模块初始化 --- */
    // 雷达用于扫描周围环境，构建点云进行中远距离避障
    // 配置雷达使用的硬件串口（通常是 UART2）及引脚
    rplidar_c1_config_t lc = rplidar_c1_default_config(
        (uart_port_t)CONFIG_FOLLOW_ROBOT_LIDAR_UART,         /* UART端口号 */
        CONFIG_FOLLOW_ROBOT_LIDAR_RX_GPIO,                   /* RX接收引脚 */
        CONFIG_FOLLOW_ROBOT_LIDAR_TX_GPIO);                  /* TX发送引脚 */
    lc.baudrate = CONFIG_FOLLOW_ROBOT_LIDAR_BAUD;            /* 波特率 */
    
    // 初始化雷达并发送开始扫描指令
    if (rplidar_c1_init(&s_lidar, &lc) == ESP_OK &&
        rplidar_c1_start_scan(&s_lidar) == ESP_OK) {
        // 创建雷达数据解析任务，堆栈4096，优先级6（与UWB同级）
        xTaskCreate(lidar_task, "lidar", 4096, &s_lidar, 6, NULL);
        ESP_LOGI(TAG, "lidar scanning");
    } else {
        // 容错处理：如果雷达损坏或未接入，系统不崩溃，而是退化为仅依靠超声波进行近距离避障
        ESP_LOGE(TAG, "lidar init/scan FAILED - avoidance falls back to ultrasonics");
    }

    /* --- 5. 超声波传感器 (行李箱前方左右边角) 初始化 --- 
     * 【硬件资源调度策略说明】：
     * ESP32的可用硬件串口有限。由于 UART1 分给了 UWB，UART2 分给了 Lidar，
     * 剩下的 UART0 通常用于系统日志烧录(Console)。
     * 但超声波模块 (A02YYUW) 的波特率仅要求 9600，速度很慢。
     * 因此，这里巧妙地使用了软件模拟串口 (bit-bang/sw_uart) 来节省硬件串口资源。 */
    
    // 5.1 左侧超声波配置
    a02yyuw_config_t ulcfg = a02yyuw_default_config(
        (uart_port_t)0,                                     /* 端口号填0（占位） */
        CONFIG_FOLLOW_ROBOT_ULTRA_LEFT_RX_GPIO, -1);        /* 仅需RX引脚接收数据，TX填-1 */
    ulcfg.use_sw_uart = true; // 强制启用软件模拟串口
    
    if (a02yyuw_init_dev(&s_ultra_left, &ulcfg) == ESP_OK) {
        // 创建左超声波轮询任务，堆栈3072，优先级5（略低于核心传感器）
        xTaskCreate(ultra_task, "ultra_l", 3072, &s_ua_left, 5, NULL);
        ESP_LOGI(TAG, "ultrasonic L ready (RX=GPIO%d)",
                 CONFIG_FOLLOW_ROBOT_ULTRA_LEFT_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "ultrasonic L init FAILED");
    }

    // 5.2 右侧超声波配置
    a02yyuw_config_t urcfg = a02yyuw_default_config(
        (uart_port_t)0,                                     /* 端口号填0（占位） */
        CONFIG_FOLLOW_ROBOT_ULTRA_RIGHT_RX_GPIO, -1);       /* 仅需RX引脚，TX填-1 */
    urcfg.use_sw_uart = true; // 同样使用软件模拟串口
    
    if (a02yyuw_init_dev(&s_ultra_right, &urcfg) == ESP_OK) {
        // 创建右超声波轮询任务，优先级5
        xTaskCreate(ultra_task, "ultra_r", 3072, &s_ua_right, 5, NULL);
        ESP_LOGI(TAG, "ultrasonic R ready (RX=GPIO%d)",
                 CONFIG_FOLLOW_ROBOT_ULTRA_RIGHT_RX_GPIO);
    } else {
        ESP_LOGE(TAG, "ultrasonic R init FAILED");
    }

    /* --- 6. IMU (惯性测量单元) 初始化 --- */
    // IMU提供航向角(Heading)反馈，防止行李箱在跟随或避障时走偏。
    // 如果初始化失败属于"非致命错误"，控制循环将退化为开环角速度控制（open omega command）。
    
    static i2c_master_bus_handle_t i2c_bus;
    // 配置I2C主机总线参数
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = 0,                                /* 使用I2C0端口 */
        .sda_io_num = CONFIG_FOLLOW_ROBOT_I2C_SDA_GPIO,  /* SDA数据线引脚 */
        .scl_io_num = CONFIG_FOLLOW_ROBOT_I2C_SCL_GPIO,  /* SCL时钟线引脚 */
        .clk_source = I2C_CLK_SRC_DEFAULT,               /* 使用默认时钟源 */
        .glitch_ignore_cnt = 7,                          /* 硬件滤波抗干扰（忽略<7个时钟周期的尖峰脉冲） */
        .flags.enable_internal_pullup = true,            /* 启用内部上拉电阻 */
    };
    
    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) == ESP_OK) {
        imu_i2c_config_t imucfg = imu_i2c_default_config();
        imucfg.sda_gpio = CONFIG_FOLLOW_ROBOT_I2C_SDA_GPIO;
        imucfg.scl_gpio = CONFIG_FOLLOW_ROBOT_I2C_SCL_GPIO;
        imucfg.device_address = CONFIG_FOLLOW_ROBOT_IMU_ADDR;   /* IMU的I2C7位设备地址 */
        imucfg.external_bus = i2c_bus;                          /* 使用已创建的I2C总线句柄 */
        
        if (imu_i2c_init(&s_imu, &imucfg) == ESP_OK) {
            s_imu_ok = true; // 标记IMU就绪
            // 打印航向闭环(Heading Hold)是否在宏定义中被开启
            ESP_LOGI(TAG, "imu ready (heading loop %s)",
                     FR_HEADING_HOLD ? "ON" : "off");
        } else {
            ESP_LOGE(TAG, "imu init FAILED - heading loop disabled");
        }
    }

    /* --- 7. 核心控制循环启动 --- */
    // 当所有传感器任务就绪并开始往共享内存(g_shared)填充数据后，启动控制大脑。
    // 创建控制任务，优先级设为7（全场最高优先级，确保控制指令实时输出到底盘）。
    xTaskCreate(control_task, "control", 4096, &s_chassis, 7, NULL);
    ESP_LOGI(TAG, "control loop running at %d Hz", CONFIG_FOLLOW_ROBOT_CONTROL_HZ);
}
