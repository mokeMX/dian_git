/**
 * chassis.h - 智能跟随行李箱的"闭环差速底盘"控制模块 头文件
 *             (算法2: 真实APO-DL无刷电调 + RC伺服PWM + AB编码器速度环)
 *
 * ========================== 机械结构 ==============================
 *
 *   行李箱底盘布局（从上往下看）：
 *
 *      [前部]
 *     ┌──────────┐
 *     │ ∘      ∘ │  <-- 两个前轮：无动力万向轮(Caster/Universal Wheel)
 *     │          │      它们自由跟随，不参与驱动力分配
 *     │          │
 *     │  行李箱  │
 *     │  箱体    │
 *     │          │
 *     │ ●      ● │  <-- 两个后轮：有动力轮，APO-DL电机驱动
 *     └──────────┘      经典后驱差速布局(Classic Rear-Axle Differential Drive)
 *      [后部]
 *
 *   运动学关系：
 *     v(线速度) = (v_left + v_right) / 2
 *     omega(角速度) = (v_right - v_left) / track_width
 *
 * ========================== 驱动电子系统 ============================
 *
 *   这个模块是"算法2"与"算法1"之间最根本的区别：
 *
 *   算法1(旧)：开环驱动
 *     - 电调PWM信号直接用占空比映射
 *     - 无编码器反馈
 *     - 实际速度取决于电池电压、地面摩擦等不可控因素
 *     - 2个前轮万向轮可能被"拖着走"，实际轨迹不可预测
 *
 *   算法2(新)：闭环驱动 (本模块)
 *     - APO-DL无刷电调通过LEDC产生标准RC伺服脉冲(50Hz, 1000~2000us)
 *       * 1500us = 中立/停止
 *       * >1500us = 前进(Forward)，数值越大速度越快
 *       * <1500us = 反转(Reverse)，数值越小速度越快
 *     - AB正交编码器通过GPIO边沿中断4倍频解码
 *       * 使用查表法高效解析旋转方向(+1/-1/0)
 *     - 每个轮子独立的PID速度闭环控制
 *       * 前馈(Feed-Forward)：目标速度 * ff_gain -> 估算的PWM脉宽
 *       * PID修正(Trim)：测量速度与目标的差值通过PID调节
 *       * 优雅降级：即使编码器断开，前馈部分仍可工作
 *     - 上层IMU航向闭环：对角速度积分得到"理论航向"，
 *       与IMU实测对比，修正角速度指令
 *       * 这样万向轮拖拽导致的航向偏差会被自动补偿
 *
 * ========================== 在控制循环中的使用方式 ====================
 *
 *   // 初始化（在 app_main 中）
 *   chassis_config_t cfg = chassis_default_config();
 *   // ... 覆盖Kconfig参数 ...
 *   chassis_init(&ch, &cfg);
 *   // 等待电调解锁(Arms): 持续输出1500us中位信号约1~3秒
 *   vTaskDelay(pdMS_TO_TICKS(3000));
 *
 *   // 控制循环（在control_task中，固定频率运行）
 *   while (1) {
 *       chassis_set_velocity(&ch, v_cmd, omega_cmd);  // 设置速度指令
 *       chassis_update(&ch, dt_s);                     // 执行闭环控制
 *       // dt_s是本周期的时间间隔(秒)
 *       vTaskDelayUntil(&last_wake, period);
 *   }
 *
 *   重要：chassis_update() 必须每个周期都被调用！
 *   它不仅执行PID，还运行故障安全计时器——
 *   如果超过 failsafe_timeout_s (默认0.3秒) 没有收到新指令，
 *   底盘会自动回到中位，防止失控。
 *
 * ========================== 坐标/符号约定 ============================
 *
 *   v > 0     = 前进 (Forward)
 *   omega > 0 = 左转 (CCW, Counter-Clockwise，从上方看)
 *   yaw > 0   = 左转 (CCW)
 *   pulse > 1500us = 前进(Forward)
 *   pulse < 1500us = 反转(Reverse)
 *
 * ========================== 故障安全特性 ============================
 *
 *   1. since_setpoint_s 计时器：
 *      如果在 failsafe_timeout_s 内没有收到新的 chassis_set_velocity()
 *      或 chassis_set_wheel_speeds() 调用，自动将目标速度归零。
 *
 *   2. 脉宽变化率限制(slew rate)：
 *      限制每个周期ESC脉冲的变化幅度，防止：
 *      - 电机瞬间大电流冲击
 *      - 齿轮箱机械损伤
 *      - 电池电压骤降
 *
 *   3. 积分器自动清零：
 *      当目标速度接近零时，PID积分器被清零。
 *      防止积分器在停止期间累积，导致重新启动时车辆猛冲。
 *
 *   4. 编码器可选：
 *      如果GPIO配置为-1，编码器初始化被跳过。
 *      底盘仍可通过纯前馈控制驱动。
 */

#pragma once  /* 头文件保护：确保只被包含一次 */

#include <stdbool.h>   /* bool 类型 */
#include <stdint.h>    /* int8_t, int16_t, int64_t 等固定宽度类型 */

#include "esp_err.h"   /* ESP-IDK 错误码类型：esp_err_t */

/* ============================================================
 * PID速度控制器 数据结构
 *
 * 这是每个轮子独立的P(比例)、I(积分)、D(微分)速度控制器。
 * 纯C实现，无硬件依赖 —— 因此可以在PC上进行单元测试。
 *
 * 设计要点：
 *   - 输出单位：ESC微秒（在中立位1500us基础上叠加补偿）
 *   - 微分在"测量值"上计算（而非误差），避免设定值阶跃时的微分冲击
 *   - 积分器被限制在 [out_min, out_max] 范围内（抗饱和/Anti-windup）
 * ============================================================ */
typedef struct {
    float kp;           /* 比例系数，单位：us per (m/s) */
    float ki;           /* 积分系数，单位：us per (m/s·s) */
    float kd;           /* 微分系数，单位：us per (m/s/s) */
    float out_min;      /* PID输出的下限（us），通常为负值 */
    float out_max;      /* PID输出的上限（us），通常为正值 */
    /* ---- 内部状态（外部不应直接修改） ---- */
    float i_term;       /* 积分累加器当前值 */
    float prev_meas;    /* 上一次的测量值（用于计算微分） */
    bool has_prev;      /* 是否有历史测量数据 */
} chassis_pid_t;

/**
 * @brief 重置PID控制器状态
 *
 * 将积分项和历史测量值清零。通常在以下情况调用：
 *   - 初始化后
 *   - 目标速度归零时
 *   - 底盘停止时
 *
 * @param pid PID控制器指针
 */
void chassis_pid_reset(chassis_pid_t *pid);

/**
 * @brief 执行一次PID速度控制步进
 *
 * 调用频率应与底盘控制循环一致（例如50Hz或100Hz）。
 *
 * @param pid      PID控制器指针
 * @param setpoint  目标速度 (m/s)
 * @param measured  实测速度 (m/s)，来自编码器
 * @param dt       自从上次调用以来的时间间隔(秒)
 * @return         PID控制器的输出贡献值（微秒）
 */
float chassis_pid_step(chassis_pid_t *pid, float setpoint, float measured,
                       float dt);

/* ============================================================
 * 编码器 数据结构
 *
 * 编码器通过GPIO中断实时更新 count 字段（volatile修饰），
 * 其余字段在初始化后保持不变。
 * ============================================================ */
typedef struct {
    int a_gpio;                /* A相信号连接的GPIO引脚号 */
    int b_gpio;                /* B相信号连接的GPIO引脚号 */
    int8_t sign;               /* 方向符号：+1=正常，-1=接线/安装方向反转 */
    volatile int64_t count;    /* 4倍频脉冲累加计数(volatile: ISR中修改) */
    volatile uint8_t last_state;  /* 上一次的AB状态 [0..3]，用于解码查表 */
} chassis_enc_t;

/* ============================================================
 * 底盘硬件配置 数据结构
 *
 * 包含了所有"物理世界"的参数：引脚分配、电调PWM范围、
 * 轮距、PID系数、安全限值等。
 *
 * 使用方式：先调用 chassis_default_config() 获取默认值，
 * 再用Kconfig中的宏覆盖需要定制的字段。
 * ============================================================ */
typedef struct {
    /* ---- ESC电调RC-PWM输出(LEDC生成) ---- */
    int esc_freq_hz;           /* PWM频率：50Hz（标准RC电调频率） */
    int esc_period_us;         /* PWM周期：20000微秒 (= 1/50Hz) */
    int esc_min_us;            /* 最小脉宽：1000微秒(全速反转) */
    int esc_mid_us;            /* 中位脉宽：1500微秒(停止/空档) */
    int esc_max_us;            /* 最大脉宽：2000微秒(全速正转) */
    int pwm_resolution_bits;   /* PWM分辨率：14位 -> 16384级精度 */
    int ledc_speed_mode;       /* LEDC速度模式：ESP32-S3用LEDC_LOW_SPEED_MODE */
    int ledc_timer;            /* LEDC定时器编号：LEDC_TIMER_0.. */
    int left_ledc_channel;     /* 左电调的LEDC通道号：LEDC_CHANNEL_0.. */
    int right_ledc_channel;    /* 右电调的LEDC通道号 */

    int left_esc_gpio;         /* 左后轮电调信号引脚(动力轮代码: GPIO4) */
    int right_esc_gpio;        /* 右后轮电调信号引脚(动力轮代码: GPIO5) */
    bool left_invert;          /* 左轮电机是否反转（true=交换正反转方向） */
    bool right_invert;         /* 右轮电机是否反转 */

    /* ---- AB编码器配置 ---- */
    int left_enc_a_gpio;       /* 左编码器A相(动力轮代码: GPIO6) */
    int left_enc_b_gpio;       /* 左编码器B相(动力轮代码: GPIO7) */
    int right_enc_a_gpio;      /* 右编码器A相(动力轮代码: GPIO15) */
    int right_enc_b_gpio;      /* 右编码器B相(动力轮代码: GPIO16) */
    bool left_enc_invert;      /* 左编码器计数方向是否取反 */
    bool right_enc_invert;     /* 右编码器计数方向是否取反 */
    float ticks_per_meter;     /* 【需标定】每米行程对应的4倍频编码器脉冲数 */

    /* ---- 运动学参数 / 速度限制 ---- */
    float track_width_m;       /* 后轮轮距(米)——两个后轮中心之间的水平距离 */
    float max_speed_mps;       /* 单轮最大转速(m/s); 同时定义前馈增益和速度限幅 */

    /* ---- 速度环PID系数（左右轮共用） ---- */
    float kp;                  /* 比例系数 */
    float ki;                  /* 积分系数 */
    float kd;                  /* 微分系数 */
    float pid_out_limit_us;    /* 每个车轮PID输出的绝对值上限(us)，通常为 ±400us */

    /* ---- 安全参数 ---- */
    float slew_us_per_s;       /* ESC脉宽最大变化速率(us/秒)，0=不限制 */
    float failsafe_timeout_s;  /* 故障安全超时：超时无新指令则自动归中(秒) */
} chassis_config_t;

/* ============================================================
 * 底盘运行时状态 数据结构
 *
 * 包含所有"当前"运行时状态：当前配置副本、编码器读数、
 * PID内部状态、目标速度、里程计等。
 *
 * 调用者通过 chassis_init() 初始化，通过 API 函数操作。
 * 内部字段对调用者基本透明。
 * ============================================================ */
typedef struct {
    chassis_config_t cfg;      /* 配置副本（值拷贝，可以安全修改本地副本） */
    bool initialized;           /* 底盘是否已初始化成功 */

    /* ---- LEDC硬件参数 ---- */
    int duty_max;               /* 最大占空比计数值 = (1<<resolution)-1, 14位时为16383 */
    float ff_us_per_mps;        /* 前馈增益(us/(m/s)): (esc_max-esc_mid) / max_speed */

    /* ---- 编码器状态 ---- */
    chassis_enc_t enc_l;        /* 左轮编码器ISR更新的状态 */
    chassis_enc_t enc_r;        /* 右轮编码器ISR更新的状态 */
    int64_t last_count_l;       /* 上一个周期的左轮编码器计数值(用于差分) */
    int64_t last_count_r;       /* 上一个周期的右轮编码器计数值(用于差分) */
    float meas_left_mps;        /* 最新编码器实测左轮速度(m/s) */
    float meas_right_mps;       /* 最新编码器实测右轮速度(m/s) */

    /* ---- PID控制器状态 ---- */
    chassis_pid_t pid_l;        /* 左轮PID控制器 */
    chassis_pid_t pid_r;        /* 右轮PID控制器 */
    float target_left_mps;      /* 左轮目标速度(m/s)，由set_velocity/set_wheel_speeds设置 */
    float target_right_mps;     /* 右轮目标速度(m/s) */

    /* ---- ESC脉宽状态 ---- */
    float cmd_pulse_l_us;       /* 左电调当前PWM脉宽(us)，用于斜坡限制计算 */
    float cmd_pulse_r_us;       /* 右电调当前PWM脉宽(us)，用于斜坡限制计算 */
    float since_setpoint_s;     /* 距离上一次收到速度指令的累计时间(秒)，用于故障安全检测 */

    /* ---- 航位推算里程计（基于编码器累加） ---- */
    double odo_x_m;             /* 推算的x坐标(米) */
    double odo_y_m;             /* 推算的y坐标(米) */
    double odo_yaw_rad;         /* 推算的偏航角(弧度) */
} chassis_t;

/* =====================================================================
 * API 函数声明
 *
 * 这些函数构成了底盘模块的公共接口。调用者应按以下顺序使用：
 *   1. chassis_default_config()  -> 获取默认配置
 *   2. 修改配置字段（覆盖Kconfig参数）
 *   3. chassis_init()            -> 初始化硬件
 *   4. 循环：chassis_set_velocity() + chassis_update()
 *   5. chassis_stop()
 *   6. chassis_deinit()（可选）
 * ===================================================================== */

/**
 * @brief 获取底盘硬件的默认配置参数
 *
 * 返回一个填充了合理默认值的配置结构体。
 * 这些默认值与"动力轮代码"分支的测试硬件一致。
 * 调用者应该在此之上覆盖Kconfig中的实际参数。
 *
 * @return 填充了默认值的 chassis_config_t
 */
chassis_config_t chassis_default_config(void);

/**
 * @brief 初始化底盘硬件和内部状态
 *
 * 按顺序：LEDC定时器 -> LEDC通道 -> GPIO ISR -> 编码器 -> PID
 * 初始化完成时电调输出中位信号(1500us)，供后续解锁(Arming)使用。
 *
 * @param ch  由调用者分配的底盘句柄（不需要初始化）
 * @param cfg 底盘配置参数指针（值会被拷贝到ch内部）
 * @return    ESP_OK成功，否则为错误码
 */
esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg);

/**
 * @brief 反初始化底盘
 *
 * 停止PWM输出，移除ISR，释放资源。
 * 调用此函数后底盘不再可用，除非重新调用chassis_init()。
 *
 * @param ch 底盘句柄
 */
void chassis_deinit(chassis_t *ch);

/**
 * @brief 设置机器人坐标系下的速度指令
 *
 * 这是上层控制任务最常用的API！
 * 内部：差速混控(v,omega)->左右轮目标速度->设置故障安全计时器。
 *
 * 注意：此函数仅设置"目标值"，不实际驱动硬件。
 *       硬件驱动在 chassis_update() 中完成。
 *       因此调用频率可以很低（最低看failsafe_timeout_s），
 *       但 chassis_update() 必须以高频率调用。
 *
 * @param ch        底盘句柄
 * @param v_mps     期望线速度 (m/s)，正值=前进
 * @param omega_rps 期望角速度 (rad/s)，正值=左转(CCW)
 * @return          ESP_OK成功
 */
esp_err_t chassis_set_velocity(chassis_t *ch, float v_mps, float omega_rps);

/**
 * @brief 直接设置左右轮的独立目标速度
 *
 * 绕过差速混控器 —— 对每个轮子直接设置速度指令。
 * 适用于需要非标准驱动模式的特例（如测试、标定）。
 *
 * @param ch        底盘句柄
 * @param left_mps  左轮目标速度(m/s)
 * @param right_mps 右轮目标速度(m/s)
 * @return          ESP_OK成功
 */
esp_err_t chassis_set_wheel_speeds(chassis_t *ch, float left_mps,
                                   float right_mps);

/**
 * @brief 低级手动控制：直接设置ESC的原始RC PWM脉冲宽度
 *
 * 绕过速度环和PID，直接操作硬件——仅用于调试/标定！
 * 正常控制流程应使用 chassis_set_velocity() + chassis_update()。
 * 脉宽会被自动限制在 [esc_min_us, esc_max_us] 安全范围内。
 *
 * @param ch       底盘句柄
 * @param left_us  左电调脉宽(微秒)：1000=全退, 1500=停, 2000=全进
 * @param right_us 右电调脉宽(微秒)
 * @return         ESP_OK成功
 */
esp_err_t chassis_set_pulse_us(chassis_t *ch, int left_us, int right_us);

/**
 * @brief 执行一个底盘控制周期
 *
 * 这是底盘模块的"心跳"—必须被周期性调用（例如与控制循环同频）。
 * 每次调用完成以下操作：
 *   1. 故障安全检查（超时归中）
 *   2. 快照编码器计数值
 *   3. 计算实测轮速(m/s)
 *   4. 更新航位推算里程计
 *   5. 运行左右轮独立的PID速度环（前馈+PID修正）
 *   6. 目标为零时清零积分器
 *   7. 应用脉宽变化率限制
 *   8. 写入LEDC硬件输出ESC信号
 *
 * @param ch   底盘句柄
 * @param dt_s 本周期的时间间隔(秒)。如果 <= 0，内部使用默认值20ms(50Hz)
 * @return     ESP_OK成功
 */
esp_err_t chassis_update(chassis_t *ch, float dt_s);

/**
 * @brief 获取编码器最新实测的速度
 *
 * 返回上一个 chassis_update() 周期计算的速度值。
 * 任意输出参数可以为NULL（表示不需要该值）。
 *
 * @param ch        底盘句柄
 * @param v_mps     输出：机器人线速度(m/s)
 * @param omega_rps 输出：机器人角速度(rad/s)
 * @param left_mps  输出：左轮速度(m/s)
 * @param right_mps 输出：右轮速度(m/s)
 */
void chassis_get_measured(chassis_t *ch, float *v_mps, float *omega_rps,
                          float *left_mps, float *right_mps);

/**
 * @brief 获取航位推算(Dead-reckoning)的里程计位姿
 *
 * 基于编码器累加的位姿估算。误差会随时间累积（漂移）。
 * 任意输出参数可以为NULL（表示不需要该值）。
 *
 * @param ch      底盘句柄
 * @param x_m     输出：x坐标(米)
 * @param y_m     输出：y坐标(米)
 * @param yaw_rad 输出：偏航角(弧度)
 */
void chassis_get_odometry(chassis_t *ch, float *x_m, float *y_m, float *yaw_rad);

/**
 * @brief 停止底盘
 *
 * 将目标速度归零，清零PID积分器，输出中位PWM信号(1500us)。
 * 这是一个"松油门"操作——电机自由减速，靠摩擦自然停止。
 */
void chassis_stop(chassis_t *ch);

/**
 * @brief 制动底盘
 *
 * 对于RC无刷电调，没有独立的电子刹车信号线。
 * 所以"刹车"与"停止"是相同的操作（回中位）。
 * 保留此函数是为了与有刷驱动等API兼容。
 */
void chassis_brake(chassis_t *ch);

/* =====================================================================
 * 纯运动学函数（无硬件依赖，可在PC上进行单元测试）
 *
 * 这些函数是纯数学运算，不涉及任何硬件操作，
 * 因此编译后可在常规PC环境中运行单元测试。
 * ===================================================================== */

/**
 * @brief 差速驱动混控：将(v, omega)映射为归一化轮速[-1, 1]
 *
 * 公式：
 *   left  = (v - omega * track_width / 2) / max_speed
 *   right = (v + omega * track_width / 2) / max_speed
 *
 * 饱和处理：如果任一轮速超过±1，等比例同时缩放两个轮子，
 * 保持转向曲率(曲率半径)不变，只是整体降速。
 *
 * （算法2中，调用者将归一化速度乘以max_speed得到m/s目标值，
 *  但归一化形式方便PC单元测试。）
 *
 * @param v_mps         期望线速度 (m/s)
 * @param omega_rps     期望角速度 (rad/s)
 * @param track_width_m 后轮轮距(米)
 * @param max_speed_mps 单轮最大速度(m/s)
 * @param left_duty     输出：左轮归一化速度 [-1, 1]
 * @param right_duty    输出：右轮归一化速度 [-1, 1]
 */
void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty);
