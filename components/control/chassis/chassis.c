/**
 * chassis.c - 差速底盘闭环控制模块的实现
 *
 * 本文件实现了智能跟随行李箱的底层底盘驱动系统。它将在 ESP32-S3 目标上
 * 编译并驱动真实的硬件(APO-DL无刷电调+AB编码器)，同时部分纯计算功能
 * (PID、运动学混控)也可以在PC上进行单元测试。
 *
 * ========================== 系统架构概览 ==========================
 *
 *   control_task (main.c)
 *       |
 *       |  每周期调用
 *       v
 *   chassis_set_velocity(ch, v, omega)    <-- 设置差速速度指令
 *       |                                    (前馈 + PID闭环)
 *       v
 *   chassis_update(ch, dt_s)              <-- 执行一个控制周期
 *       |
 *       +-- 读取编码器计数值(ISR更新的volatile变量)
 *       |
 *       +-- 计算左右轮实际速度(差分/时间)
 *       |
 *       +-- 航位推算(Dead-reckoning)更新里程计
 *       |
 *       +-- 运行左右轮独立的PID速度环
 *       |    输出 = 前馈(PWM脉宽) + PID修正
 *       |
 *       +-- 保护机制:
 *       |     - 目标速度为0时清零积分器(防止电机爬行)
 *       |     - 无新指令超过0.3秒自动回中(故障安全)
 *       |     - 脉宽变化率限制(防止电流尖峰)
 *       |
 *       +-- 将PWM脉宽写入LEDC硬件寄存器
 *       |
 *       v
 *    APO-DL无刷电调 --> 无刷电机 --> 车轮转动
 *                                |
 *               AB编码器(霍尔传感器) <--+
 *                   |
 *               GPIO边沿中断(ISR)
 *                   |
 *               4倍频解码 + 计数
 *
 * ========================= 坐标系约定 ============================
 *
 *   - v > 0     = 前进 (Forward)
 *   - omega > 0 = 左转 (CCW, Counter-Clockwise，从上方看)
 *   - yaw > 0   = 左转 (CCW)
 *
 * ========================= 电调(Electronics Speed Controller) =====
 *
 *   类型：APO-DL 无刷电调
 *   信号：标准RC伺服PWM
 *      - 频率    : 50 Hz
 *      - 周期    : 20000 微秒
 *      - 1000 us : 全速反转
 *      - 1500 us : 中立/停止 (Neutral/Stop)
 *      - 2000 us : 全速正转
 *   输出：LEDC (LED PWM Controller) 14位精度 @ 50Hz
 *
 * ========================= 编码器 =================================
 *
 *   类型：AB正交编码器(霍尔传感器)
 *   解码：4倍频(4x decoding) —— 每个AB周期产生4个计数
 *   方式：GPIO边沿中断(ANYEDGE) + 查表法
 *   查表：(prev_A,prev_B) << 2 | (new_A,new_B) -> 增量(+1/-1/0)
 *
 * ========================= 控制算法 ===============================
 *
 *   PID速度环(每个轮独立)：
 *     输出 = 前馈(目标速度 * ff_us_per_mps) + PID(目标速度 - 实测速度)
 *     前馈 = 目标速度(m/s)映射到PWM脉宽(us)的线性估计
 *            使得电机即使在没有编码器反馈时也能大致运行(优雅降级)
 *     PID  = 修正前馈的误差(编码器测量)
 *     积分抗饱和(anti-windup)：积分项被限制在[-pid_out_limit, +pid_out_limit]
 *     微分在测量值上(非误差)：避免设定值突变时的微分尖峰
 */

#include "chassis.h"

#include <math.h>
#include <string.h>
#include "driver/gpio.h"       /* GPIO引脚配置和中断管理 */
#include "driver/ledc.h"       /* LEDC PWM控制器（用于生成RC伺服脉冲） */
#include "esp_attr.h"          /* IRAM_ATTR 等属性宏 */
#include "esp_log.h"           /* ESP32 日志输出 */
#include "freertos/FreeRTOS.h" /* FreeRTOS核心（portMUX_TYPE等） */

/* ===================================================================== *
 * 纯计算部分 (ESP 目标和 PC 测试构建均编译此段)                           *
 * 无硬件操作：仅运动学 + PID                                             *
 * ===================================================================== */

/**
 * @brief 将浮点数限制在 [lo, hi] 范围内
 * @param x  输入值
 * @param lo 下限
 * @param hi 上限
 * @return   被限制后的值
 */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/* ============================================================= PID ==== */

/**
 * @brief 重置PID控制器的内部状态
 *
 * 清零积分项和历史测量值。通常在以下情况调用：
 *   - PID初始化后
 *   - 底盘停止时（防止积分器累积导致重新启动时车辆猛冲）
 *   - 电机目标速度为零时
 *
 * @param pid PID控制器指针
 */
void chassis_pid_reset(chassis_pid_t *pid)
{
    if (pid == NULL) {
        return;  /* 空指针保护 */
    }
    pid->i_term = 0.0f;       /* 清零积分项 */
    pid->prev_meas = 0.0f;    /* 清零上次测量值 */
    pid->has_prev = false;    /* 标记"无历史数据" */
}

/**
 * @brief 执行一次PID速度控制步进计算
 *
 * 算法细节：
 *   - 比例项(P)：P = kp * (setpoint - measured)
 *     误差越大，输出越强
 *
 *   - 积分项(I)：I = sum(ki * err * dt)，受抗饱和限制
 *     消除稳态误差（例如匀速行驶时的微小偏差）
 *     积分器输出被限制在 [out_min, out_max] 范围内,
 *     确保积分器本身不会把指令推到饱和区以外
 *
 *   - 微分项(D)：D = -kd * (measured_now - measured_prev) / dt
 *     注意：微分在"测量值"上，不在"误差"上！
 *     这样当设定值(setpoint)发生阶跃变化时，微分项不会产生
 *     巨大的尖峰(spike)。微分项阻尼系统的震动和超调。
 *
 *   最终输出 = kp*err + i_term - kd*deriv，然后限制在 [out_min, out_max]
 *
 * @param pid      PID控制器指针
 * @param setpoint  目标速度 (m/s)
 * @param measured  实测速度 (m/s)
 * @param dt       自从上次调用以来的时间间隔(秒)
 * @return         PID输出值（单位：ESC微秒）
 */
float chassis_pid_step(chassis_pid_t *pid, float setpoint, float measured,
                       float dt)
{
    if (pid == NULL) {
        return 0.0f;  /* 空指针保护 */
    }
    /* 防止除零：如果dt<=0，给一个小的默认值 */
    if (dt <= 0.0f) {
        dt = 1e-3f;
    }
    const float err = setpoint - measured;  /* 速度误差 */

    /* 积分计算（带抗饱和anti-windup）：
     * 积分器本身被限制在输出上下限之间，
     * 所以它永远不会把命令推到饱和区以外 */
    pid->i_term += pid->ki * err * dt;
    pid->i_term = clampf(pid->i_term, pid->out_min, pid->out_max);

    /* 微分计算（在测量值上，而非误差上）：
     * deriv_on_measurement 避免设置值阶跃时的微分尖峰 */
    float deriv = 0.0f;
    if (pid->has_prev) {
        deriv = (measured - pid->prev_meas) / dt;  /* 测量值的变化率 */
    }
    pid->prev_meas = measured;  /* 保存本次测量值，供下次使用 */
    pid->has_prev = true;

    /* PID输出 = P * 误差 + I(积分) - D * 微分(测量值) */
    float out = pid->kp * err + pid->i_term - pid->kd * deriv;
    return clampf(out, pid->out_min, pid->out_max);  /* 限幅保护 */
}

/* ==================================================== 差速运动学 ==== */

/**
 * @brief 差速驱动混控：将机器人坐标系下的(v, omega)映射为左右轮归一化速度
 *
 * 这是差速运动学的核心——"差速混控器(Differential Drive Mixer)"：
 *   左轮速度 = (v - omega * 轮距/2) / 最大轮速
 *   右轮速度 = (v + omega * 轮距/2) / 最大轮速
 *
 * 举例：
 *   - 纯前进(v>0, omega=0) -> 左右轮等速
 *   - 原地左转(v=0, omega>0) -> 左轮反转, 右轮正转
 *   - 前进+左转(v>0, omega>0) -> 右轮 > 左轮 (内侧轮减速)
 *
 * 饱和处理：如果任一轮子的归一化速度超过±1，
 * 则"等比例"缩放两个轮子的速度，保持转向曲率不变。
 * 例如左轮=0.8, 右轮=1.2，则最大值m=1.2，
 * 缩放后：左轮=0.8/1.2=0.667, 右轮=1.2/1.2=1.0
 * 这样转向的曲率/轨迹保持不变，只是整体变慢了。
 *
 * @param v_mps           机器人期望线速度 (m/s)
 * @param omega_rps       机器人期望角速度 (rad/s)
 * @param track_width_m   后轮轮距 (米)
 * @param max_speed_mps   单轮最大速度 (m/s)
 * @param left_duty       输出：左轮归一化速度 [-1, 1]
 * @param right_duty      输出：右轮归一化速度 [-1, 1]
 */
void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty)
{
    float l = 0.0f;  /* 左轮归一化速度 */
    float r = 0.0f;  /* 右轮归一化速度 */
    if (max_speed_mps > 1e-6f) {
        const float half = 0.5f * track_width_m;  /* 半轮距 */

        /* 差速混控公式 */
        l = (v_mps - omega_rps * half) / max_speed_mps;  /* 左轮 */
        r = (v_mps + omega_rps * half) / max_speed_mps;  /* 右轮 */
    }

    /* 饱和处理：如果任一轮速度超过±1，等比例缩放保持转向几何 */
    float m = fabsf(l);       /* 找到绝对值最大值 */
    if (fabsf(r) > m) {
        m = fabsf(r);
    }
    if (m > 1.0f) {
        l /= m;  /* 等比例缩小 */
        r /= m;
    }

    /* 输出限幅在[-1,1]的同时写入输出指针 */
    if (left_duty) {
        *left_duty = clampf(l, -1.0f, 1.0f);
    }
    if (right_duty) {
        *right_duty = clampf(r, -1.0f, 1.0f);
    }
}

/* ======================================================== 默认配置 ==== */

/**
 * @brief 获取底盘硬件的默认配置参数
 *
 * 这些默认值与"动力轮代码"分支的测试硬件一致。
 * 通常会在app_main()中通过Kconfig参数覆盖。
 *
 * 返回的配置结构体中包含所有可调的物理参数：
 *   - ESC电调PWM范围（1000-2000 us @ 50Hz）
 *   - 编码器引脚和方向
 *   - 轮距、轮速限制
 *   - PID系数
 *   - 安全参数（脉宽变化率限制、故障安全超时）
 *
 * @return 包含默认值的 chassis_config_t 结构体
 */
chassis_config_t chassis_default_config(void)
{
    chassis_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));  /* 先全部清零 */

    /* ---- ESC电调RC-PWM输出（与动力轮代码分支一致） ---- */
    cfg.esc_freq_hz = 50;          /* 标准RC舵机电调频率 */
    cfg.esc_period_us = 20000;     /* 周期 = 1/50Hz = 20000微秒 */
    cfg.esc_min_us = 1000;         /* 最小脉宽 = 全速反转 */
    cfg.esc_mid_us = 1500;         /* 中位脉宽 = 停止/空档 */
    cfg.esc_max_us = 2000;         /* 最大脉宽 = 全速正转 */
    cfg.pwm_resolution_bits = 14;  /* 14位PWM分辨率 -> 16384级 */
    cfg.ledc_speed_mode = 0;       /* LEDC_LOW_SPEED_MODE (ESP32-S3) */
    cfg.ledc_timer = 0;            /* LEDC_TIMER_0 */
    cfg.left_ledc_channel = 0;     /* 左电调使用LEDC通道0 */
    cfg.right_ledc_channel = 1;    /* 右电调使用LEDC通道1 */
    cfg.left_esc_gpio = 4;         /* 左电调信号线 -> GPIO4 */
    cfg.right_esc_gpio = 5;        /* 右电调信号线 -> GPIO5 */
    cfg.left_invert = false;       /* 左轮电机不反转 */
    cfg.right_invert = false;      /* 右轮电机不反转 */

    /* ---- AB编码器（与动力轮代码分支一致） ---- */
    cfg.left_enc_a_gpio = 6;       /* 左编码器A相 -> GPIO6 */
    cfg.left_enc_b_gpio = 7;       /* 左编码器B相 -> GPIO7 */
    cfg.right_enc_a_gpio = 15;     /* 右编码器A相 -> GPIO15 */
    cfg.right_enc_b_gpio = 16;     /* 右编码器B相 -> GPIO16 */
    cfg.left_enc_invert = false;   /* 左编码器不取反 */
    cfg.right_enc_invert = false;  /* 右编码器不取反 */
    cfg.ticks_per_meter = 2000.0f; /* 【需标定】每米行程的4倍频编码器脉冲数 */

    cfg.track_width_m = 0.30f;     /* 后轮轮距：0.30米(30cm) */
    cfg.max_speed_mps = 0.8f;      /* 单轮最大转速：0.8 m/s */

    cfg.kp = 200.0f;               /* 速度环PID Kp：200 微秒/(m/s) */
    cfg.ki = 300.0f;               /* 速度环PID Ki：300 微秒/(m/s·s) */
    cfg.kd = 5.0f;                 /* 速度环PID Kd：5 微秒/(m/s/s) */
    cfg.pid_out_limit_us = 400.0f; /* PID输出限幅：±400微秒 */

    cfg.slew_us_per_s = 1500.0f;   /* 脉宽变化率限制：1500us/秒（约0.67秒从全反转到全正转） */
    cfg.failsafe_timeout_s = 0.3f; /* 故障安全超时：无新指令0.3秒后自动回中 */
    return cfg;
}

/* ===================================================================== *
 * ESP32目标实现 —— 以下代码仅在ESP32/S3上编译                              *
 * ===================================================================== */

static const char *TAG = "chassis";  /* 日志标签 */

/* ---- 编码器ISR同步保护 ---- */
/*
 * 编码器脉冲计数由GPIO ISR(中断服务例程)更新。
 * ISR运行在"临界区"内，使用自旋锁(spinlock)保护count变量的读写。
 * 
 * 一个全局自旋锁保护左右两个编码器的计数 —— 虽然两个轮子的ISR
 * 可能同时触发，但ISR必须在微秒级完成，所以共享锁是合理的。
 */
static portMUX_TYPE s_enc_mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * 4倍频正交编码器解码查找表
 *
 * AB两相信号的4种状态组合(00,01,10,11)构成格雷码循环。
 * 每次任意一相发生跳变时，通过查表确定旋转方向和计数增量。
 *
 * 表索引 = (prev_state << 2) | new_state
 *   prev_state = (prev_A << 1) | prev_B   [2位]
 *   new_state  = (new_A  << 1) | new_B    [2位]
 *   组成4位索引，共16种可能的转换，有效值3种：+1(正转), -1(反转), 0(错误/无变化)
 *
 * 格雷码单步特性保证：正常情况每次只有一相变化，得到+1或-1。
 * 如果两相同时变化(噪声/抖动)，查表返回0(跳过)。
 */
static const int8_t quad_table[16] = {
     0, -1,  1,  0,   /* 00->00(0), 00->01(-1), 00->10(+1), 00->11(0, 无效) */
     1,  0,  0, -1,   /* 01->00(+1), 01->01(0), 01->10(0, 无效), 01->11(-1) */
    -1,  0,  0,  1,   /* 10->00(-1), 10->01(0, 无效), 10->10(0), 10->11(+1) */
     0,  1, -1,  0,   /* 11->00(0, 无效), 11->01(+1), 11->10(-1), 11->11(0) */
};

/**
 * @brief 编码器GPIO中断服务例程 (ISR)
 *
 * 任何时候编码器的A相或B相GPIO电平发生变化，ESP32硬件
 * 自动调用此函数。函数在IRAM中执行，以保证最低延迟。
 *
 * 工作流程：
 *   1. 读取A和B两相的当前GPIO电平
 *   2. 构建当前状态：now = (A<<1) | B   [0..3]
 *   3. 构建查表索引：idx = (last_state<<2) | now  [0..15]
 *   4. 查表获取增量(+1/-1/0)并乘上方向符号
 *   5. 用64位有符号整数累加到count
 *
 * 使用64位计数器的原因：持续运行数小时也不会溢出。
 *
 * 关于IRAM_ATTR：
 *   ISR函数必须放在IRAM(指令RAM)中，不能在Flash中。
 *   Flash中的代码会在ISR触发时未映射而导致崩溃。
 *
 * @param arg 指向 chassis_enc_t 结构体的指针
 */
static void IRAM_ATTR enc_isr(void *arg)
{
    chassis_enc_t *e = (chassis_enc_t *)arg;
    /* 读取当前A/B两相的GPIO电平(0或1) */
    const int a = gpio_get_level(e->a_gpio);
    const int b = gpio_get_level(e->b_gpio);
    /* 构建新的2位状态编码 */
    const uint8_t now = (uint8_t)((a << 1) | b);
    /* 索引 = (上次状态<<2) | 本次状态，查4倍频表 */
    const uint8_t idx = (uint8_t)((e->last_state << 2) | now);

    /* 进入临界区（在所有核心上禁止上下文切换）保护64位变量原子性 */
    portENTER_CRITICAL_ISR(&s_enc_mux);
    e->count += (int64_t)e->sign * quad_table[idx];  /* 累加增量 * 方向符号 */
    e->last_state = now;                              /* 保存当前状态供下次使用 */
    portEXIT_CRITICAL_ISR(&s_enc_mux);
}

/* ---- ESC PWM脉冲生成 ---- */

/**
 * @brief 将脉冲宽度(微秒)转换为LEDC占空比计数值
 *
 * LEDC的14位分辨率为16384级（0~16383）。
 * 需要将微秒单位的脉宽按比例映射到占空比。
 *
 * 举例：
 *   esc_period_us = 20000us (50Hz)
 *   duty_max = 16383 (14位)
 *   pulse = 1500us -> duty = 1500/20000 * 16383 = 1229
 *
 * @param ch      底盘句柄
 * @param pulse_us 目标脉宽(微秒)
 * @return        LEDC占空比计数值
 */
static uint32_t pulse_us_to_duty(const chassis_t *ch, float pulse_us)
{
    const chassis_config_t *cfg = &ch->cfg;
    /* 先将脉宽限制在 [min, max] 安全范围内 */
    float p = clampf(pulse_us, (float)cfg->esc_min_us, (float)cfg->esc_max_us);
    /* 脉宽(us)映射为占空比 = pulse/period * max_duty */
    float duty = p * (float)ch->duty_max / (float)cfg->esc_period_us;
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    if (duty > (float)ch->duty_max) {
        duty = (float)ch->duty_max;
    }
    return (uint32_t)(duty + 0.5f);  /* 四舍五入到整数 */
}

/**
 * @brief 将计算好的脉宽(微秒)写入指定LEDC通道
 *
 * 这个函数直接操作LEDC硬件寄存器，立即生效。
 *
 * @param ch      底盘句柄
 * @param channel LEDC通道号 (0=左, 1=右)
 * @param pulse_us 脉冲宽度(微秒)
 */
static void write_pulse(chassis_t *ch, int channel, float pulse_us)
{
    const chassis_config_t *cfg = &ch->cfg;
    const uint32_t duty = pulse_us_to_duty(ch, pulse_us);
    /* 设置占空比 */
    ledc_set_duty((ledc_mode_t)cfg->ledc_speed_mode, (ledc_channel_t)channel,
                  duty);
    /* 通知硬件更新输出 */
    ledc_update_duty((ledc_mode_t)cfg->ledc_speed_mode,
                     (ledc_channel_t)channel);
}

/**
 * @brief 脉宽斜坡限制函数
 *
 * 限制ESC脉宽的变化率，防止以下问题：
 *   1. 电机电流尖峰(急剧加速时电流激增)
 *   2. 齿轮箱机械冲击(急剧转向)
 *   3. 电池电压骤降(大电流脉冲)
 *
 * @param target  目标脉宽(us)
 * @param current 当前脉宽(us)
 * @param max_step 最大允许的变化量(us)
 * @return       斜坡限制后的脉宽(us)
 */
static float slew(float target, float current, float max_step)
{
    if (max_step <= 0.0f) {
        return target;  /* 未启用斜坡限制，直接返回目标值 */
    }
    float d = target - current;
    if (d > max_step) {
        d = max_step;       /* 正向变化不超过max_step */
    } else if (d < -max_step) {
        d = -max_step;      /* 负向变化不超过max_step */
    }
    return current + d;     /* 按限制后的步长更新 */
}

/* ---- 硬件初始化辅助函数 ---- */

/**
 * @brief 配置LEDC通道(ESC信号输出通道)
 *
 * 每个电调需要一个独立的LEDC通道(虽然是相同频率和分辨率，
 * 但占空比各自独立)。本函数配置单个LEDC通道的引脚、速度模式
 * 和定时器绑定。
 *
 * @param cfg     底盘配置
 * @param channel LEDC通道号
 * @param gpio    GPIO引脚号
 * @return       ESP_OK 成功，否则为错误码
 */
static esp_err_t cfg_esc_channel(const chassis_config_t *cfg, int channel,
                                 int gpio)
{
    ledc_channel_config_t ch = {
        .gpio_num = gpio,                                    /* 信号输出引脚 */
        .speed_mode = (ledc_mode_t)cfg->ledc_speed_mode,     /* 速度模式 */
        .channel = (ledc_channel_t)channel,                  /* LEDC通道号 */
        .timer_sel = (ledc_timer_t)cfg->ledc_timer,          /* 绑定的定时器 */
        .duty = 0,                                           /* 初始占空比=0 */
        .hpoint = 0,                                         /* 无相位偏移 */
        .intr_type = LEDC_INTR_DISABLE,                      /* 不使用中断 */
    };
    return ledc_channel_config(&ch);
}

/**
 * @brief 初始化编码器GPIO和ISR
 *
 * 配置两个GPIO引脚为INPUT_PULLUP(带上拉电阻的输入模式)，
 * 注册边沿中断服务函数，启用4倍频解码。
 *
 * 编码器是可选的——如果GPIO为负(-1)则跳过初始化，
 * 底盘仍可通过前馈控制驱动(只是没有速度闭环反馈)。
 *
 * @param e       编码器状态结构体指针
 * @param a_gpio  A相信号引脚
 * @param b_gpio  B相信号引脚
 * @param invert  是否取反计数方向
 * @return       ESP_OK 成功，否则为错误码
 */
static esp_err_t setup_encoder(chassis_enc_t *e, int a_gpio, int b_gpio,
                               bool invert)
{
    e->a_gpio = a_gpio;
    e->b_gpio = b_gpio;
    e->sign = invert ? -1 : 1;  /* 方向符号 */
    e->count = 0;               /* 重置编码器计数 */

    if (a_gpio < 0 || b_gpio < 0) {
        return ESP_OK; /* 编码器可选：前馈控制仍可驱动电调 */
    }

    /* 配置A/B两相GPIO：上拉输入，检测任意边沿 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << a_gpio) | (1ULL << b_gpio),  /* 位掩码选中两个GPIO */
        .mode = GPIO_MODE_INPUT,                                /* 输入模式 */
        .pull_up_en = GPIO_PULLUP_ENABLE,                       /* 启用上拉电阻 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                  /* 禁用下拉电阻 */
        .intr_type = GPIO_INTR_ANYEDGE,                         /* 任意边沿触发中断 */
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        return ret;
    }
    /* 读取初始电平，初始化last_state */
    e->last_state =
        (uint8_t)((gpio_get_level(a_gpio) << 1) | gpio_get_level(b_gpio));

    /* 为A相和B相分别注册ISR —— 4倍频需要两路都产生中断 */
    ret = gpio_isr_handler_add(a_gpio, enc_isr, e);
    if (ret != ESP_OK) {
        return ret;
    }
    return gpio_isr_handler_add(b_gpio, enc_isr, e);
}

/* ====================================================== 底盘API实现 ==== */

/**
 * @brief 初始化底盘
 *
 * 按顺序完成以下初始化：
 *   1. 复制配置参数
 *   2. 计算前馈增益(ff_us_per_mps)
 *   3. 初始化LEDC定时器和通道(ESC PWM信号)
 *   4. 输出中位PWM信号(电调解锁准备)
 *   5. 安装GPIO ISR服务
 *   6. 配置左右编码器GPIO和中断
 *   7. 初始化左右轮PID控制器
 *
 * @param ch  底盘句柄指针(由调用者分配)
 * @param cfg 底盘配置参数指针
 * @return    ESP_OK 成功，否则为错误码
 */
esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg)
{
    if (ch == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;  /* 参数空指针检查 */
    }
    memset(ch, 0, sizeof(*ch));  /* 全部清零 */
    ch->cfg = *cfg;              /* 复制配置(值拷贝) */
    ch->duty_max = (1 << cfg->pwm_resolution_bits) - 1;  /* 14位 -> 16383 */

    /* 计算前馈增益：将速度(m/s)映射到PWM偏移量(us) */
    const float range = (float)(cfg->esc_max_us - cfg->esc_mid_us);
    ch->ff_us_per_mps =
        (cfg->max_speed_mps > 1e-6f) ? (range / cfg->max_speed_mps) : 0.0f;

    /* ---- 步骤1: 配置LEDC定时器(ESC PWM) ---- */
    ledc_timer_config_t tcfg = {
        .speed_mode = (ledc_mode_t)cfg->ledc_speed_mode,       /* 速度模式 */
        .timer_num = (ledc_timer_t)cfg->ledc_timer,            /* 定时器编号 */
        .duty_resolution = (ledc_timer_bit_t)cfg->pwm_resolution_bits,  /* 分辨率14位 */
        .freq_hz = cfg->esc_freq_hz,                           /* PWM频率50Hz */
        .clk_cfg = LEDC_AUTO_CLK,                              /* 自动选择最佳时钟源 */
    };
    esp_err_t ret = ledc_timer_config(&tcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", ret);
        return ret;
    }

    /* 配置左右电调通道 */
    ret = cfg_esc_channel(cfg, cfg->left_ledc_channel, cfg->left_esc_gpio);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = cfg_esc_channel(cfg, cfg->right_ledc_channel, cfg->right_esc_gpio);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 输出中位PWM信号 —— 电调需要这持续信号来解锁(Arm) */
    ch->cmd_pulse_l_us = (float)cfg->esc_mid_us;
    ch->cmd_pulse_r_us = (float)cfg->esc_mid_us;
    write_pulse(ch, cfg->left_ledc_channel, ch->cmd_pulse_l_us);
    write_pulse(ch, cfg->right_ledc_channel, ch->cmd_pulse_r_us);

    /* ---- 步骤2: 安装GPIO ISR服务(编码器中断) ---- */
    /*
     * gpio_install_isr_service 必须在添加任何ISR handler之前调用。
     * 参数0表示使用默认的ISR配置。如果已经安装过(其他组件已调用)，
     * 返回ESP_ERR_INVALID_STATE，这不算错误。
     */
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", isr_ret);
        return isr_ret;
    }

    /* 配置左右编码器 */
    ret = setup_encoder(&ch->enc_l, cfg->left_enc_a_gpio, cfg->left_enc_b_gpio,
                        cfg->left_enc_invert);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "left encoder setup failed: %d", ret);
        return ret;
    }
    ret = setup_encoder(&ch->enc_r, cfg->right_enc_a_gpio,
                        cfg->right_enc_b_gpio, cfg->right_enc_invert);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "right encoder setup failed: %d", ret);
        return ret;
    }

    /* ---- 步骤3: 初始化左右轮PID控制器 ---- */
    ch->pid_l.kp = ch->pid_r.kp = cfg->kp;
    ch->pid_l.ki = ch->pid_r.ki = cfg->ki;
    ch->pid_l.kd = ch->pid_r.kd = cfg->kd;
    ch->pid_l.out_min = ch->pid_r.out_min = -cfg->pid_out_limit_us;
    ch->pid_l.out_max = ch->pid_r.out_max = cfg->pid_out_limit_us;
    chassis_pid_reset(&ch->pid_l);
    chassis_pid_reset(&ch->pid_r);

    ch->initialized = true;  /* 标记初始化完成 */
    ESP_LOGI(TAG,
             "init: ESC L=GPIO%d R=GPIO%d @%dHz | enc L=%d/%d R=%d/%d | "
             "track=%.2fm vmax=%.2fm/s ff=%.0fus/(m/s)",
             cfg->left_esc_gpio, cfg->right_esc_gpio, cfg->esc_freq_hz,
             cfg->left_enc_a_gpio, cfg->left_enc_b_gpio, cfg->right_enc_a_gpio,
             cfg->right_enc_b_gpio, cfg->track_width_m, cfg->max_speed_mps,
             ch->ff_us_per_mps);
    return ESP_OK;
}

/**
 * @brief 直接设置左右轮的独立目标速度 (m/s)
 *
 * 这是底层的速度控制接口，绕过差速混控器。
 * 通常由 chassis_set_velocity() 内部调用。
 *
 * 调用此函数会重置"故障安全"计时器——意味着"我还在控制中"。
 *
 * @param ch         底盘句柄
 * @param left_mps   左轮目标速度 (m/s)，正值=前进
 * @param right_mps  右轮目标速度 (m/s)，正值=前进
 * @return          ESP_OK 成功
 */
esp_err_t chassis_set_wheel_speeds(chassis_t *ch, float left_mps,
                                   float right_mps)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;  /* 未初始化 */
    }
    const float vmax = ch->cfg.max_speed_mps;
    /* 限制目标速度不超过最大轮速 */
    ch->target_left_mps = clampf(left_mps, -vmax, vmax);
    ch->target_right_mps = clampf(right_mps, -vmax, vmax);
    ch->since_setpoint_s = 0.0f;  /* 每次收到新指令都重置故障安全计时器 */
    return ESP_OK;
}

/**
 * @brief 设置机器人坐标系下的速度指令 (v线速度 + omega角速度)
 *
 * 这是上层控制任务最常用的接口。
 * 内部调用差速混控器将(v, omega)分解为左右轮目标速度，
 * 然后调用 chassis_set_wheel_speeds() 设置。
 *
 * @param ch        底盘句柄
 * @param v_mps     期望线速度 (m/s)，正值=前进
 * @param omega_rps 期望角速度 (rad/s)，正值=左转(CCW)
 * @return         ESP_OK 成功
 */
esp_err_t chassis_set_velocity(chassis_t *ch, float v_mps, float omega_rps)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    float l = 0.0f;
    float r = 0.0f;
    /* 差速混控：将(v, omega)分解为左右轮归一化速度[-1,1] */
    chassis_diff_drive_mix(v_mps, omega_rps, ch->cfg.track_width_m,
                           ch->cfg.max_speed_mps, &l, &r);
    /* 映射回实际速度(m/s)并设置 */
    return chassis_set_wheel_speeds(ch, l * ch->cfg.max_speed_mps,
                                    r * ch->cfg.max_speed_mps);
}

/**
 * @brief 低级手动控制：直接设置ESC的原始RC脉冲宽度
 *
 * 绕过了PID和速度环，直接将脉宽写入硬件。
 * 仅用于调试/标定——正常控制流程应使用chassis_set_velocity + chassis_update。
 *
 * 脉宽会自动限制在 [esc_min_us, esc_max_us] 范围内。
 *
 * @param ch      底盘句柄
 * @param left_us  左电调脉宽(微秒)
 * @param right_us 右电调脉宽(微秒)
 * @return       ESP_OK 成功
 */
esp_err_t chassis_set_pulse_us(chassis_t *ch, int left_us, int right_us)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 限制脉宽范围并存储 */
    ch->cmd_pulse_l_us =
        clampf((float)left_us, (float)ch->cfg.esc_min_us,
               (float)ch->cfg.esc_max_us);
    ch->cmd_pulse_r_us =
        clampf((float)right_us, (float)ch->cfg.esc_min_us,
               (float)ch->cfg.esc_max_us);
    /* 直接写入硬件 */
    write_pulse(ch, ch->cfg.left_ledc_channel, ch->cmd_pulse_l_us);
    write_pulse(ch, ch->cfg.right_ledc_channel, ch->cmd_pulse_r_us);
    return ESP_OK;
}

/**
 * @brief 执行一个底盘控制更新周期
 *
 * 此函数是底盘的"心跳"——必须被控制任务以固定频率调用
 * (例如50Hz或100Hz)。每次调用完成以下操作：
 *
 *  1. 故障安全检查：如果超过failsafe_timeout_s没有收到新指令，
 *     自动将目标速度清零(防止行李箱失控狂奔)
 *
 *  2. 读取编码器：在自旋锁保护下原子地快照左右编码器的累加计数
 *
 *  3. 速度估计：计算本周期内的编码器差值，除以时间得到轮速
 *
 *  4. 航位推算(Dead-reckoning)：根据轮速差估计机器人在地面上的
 *     位姿变化(x, y, yaw)。纯依靠编码器，会累积漂移误差
 *
 *  5. PID速度环：对每个轮子执行 PID_step，输出 = 前馈 + PID修正
 *
 *  6. 保护逻辑：
 *     - 目标速度为零 -> 强制输出中位 + 清零积分器(防止电机爬行)
 *
 *  7. 斜坡限制：限制PWM脉宽的变化速率
 *
 *  8. 写入硬件：将计算好的PWM脉冲写入LEDC输出
 *
 * @param ch   底盘句柄
 * @param dt_s 本周期时间间隔(秒)
 * @return    ESP_OK 成功
 */
esp_err_t chassis_update(chassis_t *ch, float dt_s)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const chassis_config_t *cfg = &ch->cfg;
    if (dt_s <= 0.0f) {
        dt_s = 0.02f;  /* 最小默认间隔20ms(50Hz) */
    }

    /* ---- 1. 故障安全：超过超时时间未收到新指令 -> 自动停止 ---- */
    ch->since_setpoint_s += dt_s;
    if (ch->since_setpoint_s > cfg->failsafe_timeout_s) {
        /* 超时：清零目标速度(不是急停，而是让速度环自然减速) */
        ch->target_left_mps = 0.0f;
        ch->target_right_mps = 0.0f;
    }

    /* ---- 2. 快照编码器计数（自旋锁保护的原子读取） ---- */
    int64_t cl;
    int64_t cr;
    portENTER_CRITICAL(&s_enc_mux);    /* 进入临界区 */
    cl = ch->enc_l.count;              /* 原子地读取64位计数器 */
    cr = ch->enc_r.count;
    portEXIT_CRITICAL(&s_enc_mux);     /* 退出临界区 */

    /* 计算本周期内的编码器增量 */
    const int64_t dl = cl - ch->last_count_l;
    const int64_t dr = cr - ch->last_count_r;
    ch->last_count_l = cl;  /* 保存当前计数供下一周期使用 */
    ch->last_count_r = cr;

    /* ---- 3. 速度估计：编码器增量 -> 轮速(m/s) ---- */
    const float tpm = (cfg->ticks_per_meter > 1.0f) ? cfg->ticks_per_meter : 1.0f;
    const float left_dist = (float)dl / tpm;    /* 左轮行驶距离(米) */
    const float right_dist = (float)dr / tpm;   /* 右轮行驶距离(米) */
    ch->meas_left_mps = left_dist / dt_s;       /* 实测左轮速度 */
    ch->meas_right_mps = right_dist / dt_s;     /* 实测右轮速度 */

    /* ---- 4. 航位推算(Dead-reckoning)里程计 ---- */
    /*
     * 基于差速运动学模型估算机器人在地图上的位姿。
     * 
     * dc = 平均行进距离 = (左轮距离 + 右轮距离) / 2
     * dyaw = 角度变化 = (右轮距离 - 左轮距离) / 轮距
     * x += dc * cos(yaw)   -- 在x轴的投影
     * y += dc * sin(yaw)   -- 在y轴的投影
     *
     * 注意：这是"开环"推算，误差会随时间累积。
     * 未来可考虑用UWB或视觉定位来修正漂移。
     */
    const double dc = 0.5 * ((double)left_dist + (double)right_dist);
    const double track = (cfg->track_width_m > 1e-3f) ? cfg->track_width_m : 1e-3;
    ch->odo_yaw_rad += ((double)right_dist - (double)left_dist) / track;
    ch->odo_x_m += dc * cos(ch->odo_yaw_rad);
    ch->odo_y_m += dc * sin(ch->odo_yaw_rad);

    /* ---- 5. PID速度环：前馈 + PID修正 ---- */
    const float vmax = cfg->max_speed_mps;
    const float tl = clampf(ch->target_left_mps, -vmax, vmax);   /* 左轮目标 */
    const float tr = clampf(ch->target_right_mps, -vmax, vmax);  /* 右轮目标 */

    /* 前馈 = 目标速度(m/s) * 增益(us/(m/s)) => 脉宽偏移(us)
     *   + PID修正 = 消除前馈引入的误差 */
    float off_l = ch->ff_us_per_mps * tl +
                  chassis_pid_step(&ch->pid_l, tl, ch->meas_left_mps, dt_s);
    float off_r = ch->ff_us_per_mps * tr +
                  chassis_pid_step(&ch->pid_r, tr, ch->meas_right_mps, dt_s);

    /* ---- 6. 保护逻辑：目标速度为0 -> 强制中位 + 清零积分器 ---- */
    /*
     * 当控制任务下达"停止"指令时，我们不仅要输出中立脉宽，
     * 还要清零PID积分器。如果不清零，积分器可能累积了很大的
     * 正值(比如之前全速前进时积分器"推"着电机加速)，导致
     * 重启动瞬间车辆猛冲。
     */
    if (fabsf(tl) < 1e-4f) {       /* 左轮目标速度 ≈ 0 */
        off_l = 0.0f;              /* 强制输出中位 */
        chassis_pid_reset(&ch->pid_l);  /* 清零积分器 */
    }
    if (fabsf(tr) < 1e-4f) {       /* 右轮目标速度 ≈ 0 */
        off_r = 0.0f;
        chassis_pid_reset(&ch->pid_r);
    }

    /* 将前馈+PID偏移转换为实际脉宽：mid_us + offset(应用反转标志) */
    float pulse_l = (float)cfg->esc_mid_us + (cfg->left_invert ? -off_l : off_l);
    float pulse_r =
        (float)cfg->esc_mid_us + (cfg->right_invert ? -off_r : off_r);

    /* ---- 7. 脉宽斜坡限制（保护电调和电机） ---- */
    const float max_step = cfg->slew_us_per_s * dt_s;
    pulse_l = slew(pulse_l, ch->cmd_pulse_l_us, max_step);
    pulse_r = slew(pulse_r, ch->cmd_pulse_r_us, max_step);
    ch->cmd_pulse_l_us = pulse_l;  /* 保存当前脉宽供下周期斜坡使用 */
    ch->cmd_pulse_r_us = pulse_r;

    /* ---- 8. 写入硬件 ---- */
    write_pulse(ch, cfg->left_ledc_channel, pulse_l);
    write_pulse(ch, cfg->right_ledc_channel, pulse_r);
    return ESP_OK;
}

/**
 * @brief 获取编码器实测的速度
 *
 * 从编码器最近一个周期的差分计算得到。
 * 调用者可以传递NULL给任何不需要的输出参数。
 *
 * @param ch        底盘句柄
 * @param v_mps     输出：机器人线速度(m/s)
 * @param omega_rps 输出：机器人角速度(rad/s)
 * @param left_mps  输出：左轮速度(m/s)
 * @param right_mps 输出：右轮速度(m/s)
 */
void chassis_get_measured(chassis_t *ch, float *v_mps, float *omega_rps,
                          float *left_mps, float *right_mps)
{
    if (ch == NULL) {
        return;
    }
    const float l = ch->meas_left_mps;
    const float r = ch->meas_right_mps;
    if (v_mps) {
        *v_mps = 0.5f * (l + r);  /* 线速度 = 左右轮速度的平均值 */
    }
    if (omega_rps) {
        const float track =
            (ch->cfg.track_width_m > 1e-3f) ? ch->cfg.track_width_m : 1e-3f;
        *omega_rps = (r - l) / track;  /* 角速度 = 右轮减左轮 / 轮距 */
    }
    if (left_mps) {
        *left_mps = l;
    }
    if (right_mps) {
        *right_mps = r;
    }
}

/**
 * @brief 获取航位推算(Dead-reckoning)的里程计位姿
 *
 * 位姿是基于编码器累加计算得出的，会随时间累积漂移误差。
 * 调用者可传递NULL给不需要的输出参数。
 *
 * @param ch      底盘句柄
 * @param x_m     输出：x坐标(米)
 * @param y_m     输出：y坐标(米)
 * @param yaw_rad 输出：偏航角(弧度)
 */
void chassis_get_odometry(chassis_t *ch, float *x_m, float *y_m, float *yaw_rad)
{
    if (ch == NULL) {
        return;
    }
    if (x_m) {
        *x_m = (float)ch->odo_x_m;
    }
    if (y_m) {
        *y_m = (float)ch->odo_y_m;
    }
    if (yaw_rad) {
        *yaw_rad = (float)ch->odo_yaw_rad;
    }
}

/**
 * @brief 停止底盘：归中位 + 清零目标速度和积分器
 *
 * 与chassis_brake不同，这是"松油门"式的停止，不施加制动力。
 * 电机自由减速，靠摩擦力自然停止。
 */
void chassis_stop(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    ch->target_left_mps = 0.0f;        /* 左轮目标速度归零 */
    ch->target_right_mps = 0.0f;       /* 右轮目标速度归零 */
    ch->since_setpoint_s = 0.0f;       /* 重置故障安全计时器 */
    chassis_pid_reset(&ch->pid_l);     /* 清零左轮PID积分器 */
    chassis_pid_reset(&ch->pid_r);     /* 清零右轮PID积分器 */
    ch->cmd_pulse_l_us = (float)ch->cfg.esc_mid_us;  /* 左电调回到中位 */
    ch->cmd_pulse_r_us = (float)ch->cfg.esc_mid_us;  /* 右电调回到中位 */
    write_pulse(ch, ch->cfg.left_ledc_channel, ch->cmd_pulse_l_us);
    write_pulse(ch, ch->cfg.right_ledc_channel, ch->cmd_pulse_r_us);
}

/**
 * @brief 制动底盘
 *
 * RC无刷电调没有独立的"电子刹车"信号线——它不像有刷驱动那样
 * 可以短路刹车(Brake)。所以刹车和停止是一样的操作：回到中位。
 * 保留此函数是为了API兼容性。
 */
void chassis_brake(chassis_t *ch)
{
    chassis_stop(ch);
}

/**
 * @brief 反初始化底盘
 *
 * 步骤：
 *   1. 停止所有PWM输出
 *   2. 移除编码器ISR处理器
 *   3. 停止LEDC通道输出
 *   4. 标记为未初始化状态
 *
 * 这通常在系统关闭或不使用底盘时调用。
 */
void chassis_deinit(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    chassis_stop(ch);  /* 先安全停止 */

    /* 移除编码器的GPIO中断处理函数 */
    if (ch->enc_l.a_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_l.a_gpio);
    }
    if (ch->enc_l.b_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_l.b_gpio);
    }
    if (ch->enc_r.a_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_r.a_gpio);
    }
    if (ch->enc_r.b_gpio >= 0) {
        gpio_isr_handler_remove(ch->enc_r.b_gpio);
    }

    /* 停止LEDC通道输出PWM信号 */
    ledc_stop((ledc_mode_t)ch->cfg.ledc_speed_mode,
              (ledc_channel_t)ch->cfg.left_ledc_channel, 0);
    ledc_stop((ledc_mode_t)ch->cfg.ledc_speed_mode,
              (ledc_channel_t)ch->cfg.right_ledc_channel, 0);
    ch->initialized = false;  /* 标记为未初始化 */
}
