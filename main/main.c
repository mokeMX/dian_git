/**
 * 动力轮代码 (Power-Wheel Bring-up) — APO-DL 电调 + 编码器 键盘遥控测试
 *
 * ========================== 程序定位 ============================
 *
 *   这是「跟随行李箱」项目动力系统的**最小可用验证程序**。
 *   整个程序只有这一个 main.c 文件，实现了以下完整功能：
 *     1. RC航模PWM信号驱动APO-DL无刷电调（差速底盘）
 *     2. AB正交编码器4倍频解码（仅读取并显示，不参与速度闭环）
 *     3. USB-JTAG串口键盘实时遥控
 *     4. 安全特性：死人停车(1.2秒自动停止)
 *
 *   这是后续所有算法分支（follow_only、follow_robot）的**硬件基础**——
 *   先在这个分支确认电调、电机、编码器接线正确、性能正常，
 *   再上算法闭环。
 *
 * ========================== 硬件平台 ============================
 *
 *   MCU：ESP32-S3
 *   电调：APO-DL 双路无刷电调（A/B两通道，独立控制左右后轮）
 *   编码器：AB正交霍尔编码器（每个电机一个，共两个）
 *   通信：USB-Serial-JTAG（板载原生USB口，非UART0）
 *
 * ========================== 引脚连接 ============================
 *
 *   左电调信号 : GPIO4  -> APO-DL A通道 S脚(信号)
 *   右电调信号 : GPIO5  -> APO-DL B通道 S脚(信号)
 *   电调地线   : GND    -> APO-DL A/B通道 -脚(共地，必须接)
 *   电调正极   : 不接！ (APO-DL A/B通道 +脚悬空，电调已独立供电)
 *
 *   左编码器A/B : GPIO6 / GPIO7
 *   右编码器A/B : GPIO15 / GPIO16
 *   编码器供电   : 按模块标称(3.3V~5V)，共地
 *
 * ========================== RC航模PWM规范 =========================
 *
 *   这是标准航模/舵机控制协议，APO-DL电调完全兼容此协议：
 *     - 频率/周期: 50Hz = 20000微秒
 *     - 1000us  = 全速反转(后退)
 *     - 1500us  = 中立/停止(Neutral/Stop) —— 这就是"空档"
 *     - 2000us  = 全速正转(前进)
 *     - PWM信号由ESP32-S3的LEDC(硬件PWM控制器)生成
 *     - 分辨率：14位(16384级)，远高于RC航模要求的精度
 *
 *   为什么用14位而不是16位？
 *     50Hz频率下，时钟分频有上限。14位分辨率为20000us/16384≈1.22us，
 *     足够精确控制微秒级脉宽。16位在50Hz下反而时钟不够用。
 *
 * ========================== 编码器4倍频解码 ========================
 *
 *   AB正交编码器输出两路方波(A相和B相)，相位差90度。
 *   通过检测A/B两路的高低电平变化，可以判断旋转方向和速度：
 *
 *     状态序列(Gray Code单步变化):
 *       正转(CW):  00 -> 01 -> 11 -> 10 -> 00  (每个边沿+1)
 *       反转(CCW): 00 -> 10 -> 11 -> 01 -> 00  (每个边沿-1)
 *
 *   "4倍频"的含义：
 *     一个完整的AB周期有4个边沿变化(每个相位变化产生1个计数)。
 *     因此编码器一圈如果有N个脉冲，4倍频后得到4N个计数。
 *     这是最高精度的增量编码器解码方式。
 *
 *   解码方法：查表法(Lookup Table)
 *     - 将"A当前 B当前 A上一次 B上一次"4位组合作为索引(0~15)
 *     - 查表直接得到增量：+1(正转)、-1(反转)、0(无效/无变化)
 *     - 在GPIO边沿中断中执行，微秒级延迟
 *
 * ========================== 控制模型 =============================
 *
 *   这是"开环"控制模型 —— 没有速度反馈闭环：
 *     键盘输入 -> 目标动作(前进/后退/左转/右转/停止)
 *              -> 查表得到左右轮PWM脉宽
 *              -> 输出到电调
 *              -> 电机转动
 *
 *   编码器只读取显示，不参与速度调节。真正的速度闭环在算法2的
 *   components/control/chassis/ 中实现。
 *
 *   差速转向的数学关系：
 *     前进: 左轮=1500+Δ, 右轮=1500+Δ  (同向同速)
 *     后退: 左轮=1500-Δ, 右轮=1500-Δ  (同向同速反向)
 *     左转: 左轮=1500-Δ, 右轮=1500+Δ  (左轮减速, 右轮加速)
 *     右转: 左轮=1500+Δ, 右轮=1500-Δ  (左轮加速, 右轮减速)
 *     停止: 左轮=1500,   右轮=1500    (中立/空档)
 *
 * ========================== 安全设计 =============================
 *
 *   1. 上电立即输出1500us停止信号
 *      电调在接收到中立信号后才完成自检，所以初始化立即输出1500us。
 *      等听到电调"滴滴"自检完成声(通常1~3秒)后再操作。
 *
 *   2. 死人停车(Dead-man Switch)
 *      每次按下w/s/a/d只执行1.2秒就自动停止。
 *      这不是bug，是有意的安全设计——防止键盘卡住导致行李箱失控狂奔。
 *      要持续走就持续按键。按x立即停止。
 *
 *   3. 脉宽硬限幅
 *      所有PWM脉宽输出前都经过 clamp(1000, 2000) 限幅，
 *      确保不会因程序bug输出超出电调安全范围的信号。
 *
 *   4. 速度上下限
 *      speed_delta_us 被限制在 [100, 500] 范围内，
 *      防止过快引起失控或过慢无法移动。
 *
 * ========================== 系统架构 =============================
 *
 *   单文件单任务架构
 *   无FreeRTOS任务分工 —— 全部逻辑在app_main()的主循环中：
 *
 *   app_main() {
 *       初始化: esc_init() -> motor_stop() -> command_input_init() -> encoder_init()
 *       打印: 欢迎信息 + 帮助 + 状态
 *       主循环:
 *           while(1) {
 *               读取USB-JTAG键盘输入(20ms超时非阻塞)
 *               分发命令: w/s/a/d/x/c/z/+/-
 *               如果电机在运行: 累加超时计时器 -> 1.2秒到达则自动停止
 *           }
 *   }
 *
 *   中断服务:
 *     GPIO边沿中断 -> 编码器ISR -> 查表更新count（完全不阻塞主循环）
 *
 *   为什么不用FreeRTOS多任务？
 *     这是最简单的验证程序——所有传感器读取都是非阻塞的(20ms超时)，
 *     单任务轮询足够。算法分支才会引入多任务异步架构。
 */

#include <stdio.h>       /* printf, sprintf 等标准I/O */
#include <inttypes.h>    /* PRId64 等跨平台整数格式化宏(C99标准) */

#include "freertos/FreeRTOS.h"  /* FreeRTOS核心 */
#include "freertos/task.h"      /* vTaskDelay 延时函数 */

#include "driver/gpio.h"              /* GPIO引脚配置和边沿中断 */
#include "driver/ledc.h"              /* LEDC硬件PWM控制器(用于生成RC脉冲) */
#include "driver/usb_serial_jtag.h"   /* ESP32-S3内置USB-JTAG串口驱动 */

#include "esp_err.h"   /* ESP-IDF错误码: ESP_OK / ESP_FAIL */

// ===================== 引脚定义 =====================
/*
 * APO-DL电调是双通道无刷电调，A通道控制左后轮，B通道控制右后轮。
 * 电调信号线(S脚)是标准的RC航模PWM输入，电平兼容3.3V/5V。
 * 电调+脚(正极)不连接！电调通过电池独立供电，只共地即可。
 */

// APO-DL A/B 通道 RC 信号输入
#define LEFT_ESC_GPIO       GPIO_NUM_4   /* 左电调信号(S脚) -> ESP32 GPIO4 */
#define RIGHT_ESC_GPIO      GPIO_NUM_5   /* 右电调信号(S脚) -> ESP32 GPIO5 */

/*
 * AB正交编码器使用霍尔传感器检测电机旋转。
 * 每个电机有两路信号(A相和B相)，相位差90度，用于判断方向和速度。
 * 编码器数据在本程序中仅用于"显示查看"，不参与电机速度的闭环控制。
 */
#define LEFT_ENC_A_GPIO     GPIO_NUM_6   /* 左编码器A相 */
#define LEFT_ENC_B_GPIO     GPIO_NUM_7   /* 左编码器B相 */

#define RIGHT_ENC_A_GPIO    GPIO_NUM_15  /* 右编码器A相 */
#define RIGHT_ENC_B_GPIO    GPIO_NUM_16  /* 右编码器B相 */

/*
 * 电机方向修正标志:
 *
 * 如果某个电机转的方向与实际期望相反(例如发"前进"指令车却后退)，
 * 有两种解决办法：
 *   方案A(硬件): 手动把电机三相线中的任意两根对调(蓝/白/红)
 *               —— 这是推荐的物理方法
 *   方案B(软件): 把下面的 0 改成 1，启用软件反向
 *               —— 方便调试，但建议最终用硬件修正
 *
 * 这里已经手动把一边电机的蓝白线反接了，所以两个都设为0(不反向)。
 * 注意：硬件反向和软件反向不要同时启用(会负负得正，等于没改)。
 */
#define LEFT_MOTOR_REVERSE   0   /* 左轮不软件反向(已通过接线调整) */
#define RIGHT_MOTOR_REVERSE  0   /* 右轮不软件反向(已通过接线调整) */

// ===================== RC 航模信号参数 =====================
/*
 * 标准RC航模/舵机PWM信号规范
 *   频率: 50Hz(周期20ms) —— 绝大多数RC电调和舵机的标准频率
 *   中立: 1500us —— 电机停转(空档)
 *   全速反转: 1000us —— 向后退
 *   全速正转: 2000us —— 向前进
 *
 * 为什么是50Hz而不是更高的频率？
 *   - 更高频率(如400Hz)需要"数字伺服"或"one-shot"协议支持
 *   - APO-DL电调兼容标准RC 50Hz信号
 *   - 50Hz的20ms周期足够长，信号不易受电机电磁干扰
 */

#define ESC_FREQ_HZ          50      /* PWM频率: 50Hz */
#define ESC_PERIOD_US        20000   /* PWM周期: 1/50Hz = 20000微秒 */

#define ESC_MIN_US           1000    /* 电调最小脉宽: 1000us(全速反转) */
#define ESC_MID_US           1500    /* 电调中位脉宽: 1500us(停止/空档) */
#define ESC_MAX_US           2000    /* 电调最大脉宽: 2000us(全速正转) */

/*
 * LEDC PWM分辨率说明:
 *   14位分辨率 = 2^14 = 16384 级占空比
 *   50Hz下每个占空比单位 ≈ 20000us / 16384 ≈ 1.22us
 *   这意味着我们可以以约1.22us的精度设置脉宽，远超RC电调的需求
 *   (RC电调的脉宽分辨能力通常约2~4us)
 */
#define LEDC_DUTY_RES        LEDC_TIMER_14_BIT   /* 14位PWM分辨率 */
#define LEDC_DUTY_MAX        ((1 << 14) - 1)     /* 最大占空比 = 16383 */

/*
 * 速度力度控制:
 *   speed_delta_us = 中立脉宽1500us ± 这个偏移量
 *   比如默认300us时:
 *     前进 = 1500 + 300 = 1800us
 *     后退 = 1500 - 300 = 1200us
 *   +/- 按键可调整范围 [100, 500]
 *
 *   如果觉得太慢: 按 + 键增大
 *   如果觉得太快: 按 - 键减小
 */
#define DEFAULT_SPEED_DELTA_US   300   /* 初始速度力度(中立偏移): 300us */
#define MIN_SPEED_DELTA_US       100   /* 最小速度力度: 100us */
#define MAX_SPEED_DELTA_US       500   /* 最大速度力度: 500us */

/*
 * 死人停车计时器时长:
 *   每次按下移动键(w/s/a/d)后，电机只会运行1.2秒就自动停止。
 *   这确保了即使键盘卡住/程序卡死在移动状态，行李箱也会很快停下。
 *   要持续移动必须持续按键(每次按键都重置这个计时器)。
 *
 *   为什么是1200ms而不是更短/更长？
 *     - 1200ms ≈ 人按一次键的自然时长，体验上不会太"敏感"
 *     - 也足够让操作者判断方向是否正确
 *     - 如果太短(如300ms)，还没看到反应就停了，体验差
 */
#define COMMAND_TIMEOUT_MS       1200   /* 移动指令的最大执行时长(毫秒) */

// ===================== 全局变量 =====================
/*
 * 编码器计数变量 —— volatile 修饰的原因:
 *   这些变量由GPIO边沿中断服务程序(ISR)在异步上下文中修改，
 *   同时也在主循环(app_main的while)中读取。
 *   volatile 告诉编译器："不要在寄存器中缓存此变量的值，
 *   每次访问都必须从内存重新读取"。
 *
 *   使用64位有符号整数(int64_t)的原因:
 *   编码器脉冲计数是持续累加的，如果用32位(±21亿)，
 *   在高速旋转下几个小时内就会溢出回绕。64位几乎不会溢出。
 */
static volatile int64_t left_count = 0;   /* 左轮编码器的4倍频脉冲累加计数 */
static volatile int64_t right_count = 0;  /* 右轮编码器的4倍频脉冲累加计数 */

/*
 * 上一次的AB状态编码 [0~3]
 *   用于4倍频查表的"prev_state"部分。
 *   ISR把"上次状态<<2 | 当前状态"组成4位索引查表。
 */
static volatile uint8_t left_last_state = 0;   /* 左轮上一次A/B状态 */
static volatile uint8_t right_last_state = 0;  /* 右轮上一次A/B状态 */

static int speed_delta_us = DEFAULT_SPEED_DELTA_US;  /* 当前的速度力度设置 */

/*
 * 当前输出的PWM脉宽记录
 *   用于状态显示(按z键查看当前工作参数)
 */
static int last_left_pulse = ESC_MID_US;   /* 左电调当前脉宽 */
static int last_right_pulse = ESC_MID_US;  /* 右电调当前脉宽 */

/**
 * 4倍频正交编码器解码查找表
 *
 * 这是编码器解码的核心——一张16字节的静态表。
 *
 * 原理:
 *   AB两路信号的组合有4种状态: 00, 01, 10, 11(格雷码顺序)
 *   相邻状态之间每次只有1位变化(格雷码的特性)
 *   从"上一次状态"到"当前状态"的跳变确定了旋转方向
 *
 * 表索引 = (上一次的A<<1 | 上一次的B) << 2 | (当前的A<<1 | 当前的B)
 *          = 上一次状态 * 4 + 当前状态
 *          = [0~15]
 *
 * 表值含义:
 *   +1 = 正转一步(顺时针/CW)
 *   -1 = 反转一步(逆时针/CCW)
 *    0 = 无效跳变(如两相同时变化，可能是噪声或丢步)
 *
 * 为什么表里很多0？
 *   正常运行时，每次只有一相发生变化(Gray Code单步)，
 *   比如 00->01 是+1(正转)，00->10 是-1(反转)。
 *   如果出现00->11(两相同时变)，那是异常情况，返回0跳过。
 *
 * 举例解读几个表项:
 *   index=1 (00->01): 表值=-1, 表示从00跳到01是反转
 *   index=2 (00->10): 表值=+1, 表示从00跳到10是正转
 *   index=4 (01->00): 表值=+1, 表示从01跳回00是正转
 *   index=8 (10->00): 表值=-1, 表示从10跳回00是反转
 *
 *   你可以验证: 正转序列 00→01→11→10→00 依次查表得到
 *   -1, +1, -1, +1 的交替，这是合理的(方向符号取决于你"A相领先B相"
 *   还是"B相领先A相"的物理接线)
 */
static const int8_t quad_table[16] = {
     0, -1,  1,  0,    /* prev=00: 到00(0), 到01(-1), 到10(+1), 到11(无效) */
     1,  0,  0, -1,    /* prev=01: 到00(+1), 到01(0), 到10(无效), 到11(-1) */
    -1,  0,  0,  1,    /* prev=10: 到00(-1), 到01(无效), 到10(0), 到11(+1) */
     0,  1, -1,  0     /* prev=11: 到00(无效), 到01(+1), 到10(-1), 到11(0) */
};

// ===================== 工具函数 =====================

/**
 * @brief 将整数限制在 [min_value, max_value] 范围内
 *
 * 这是最基本的安全保护函数——所有的用户输入和计算值
 * 在写入硬件之前都必须经过限幅。
 *
 * @param value      输入值
 * @param min_value  允许的最小值
 * @param max_value  允许的最大值
 * @return          被限制后的值
 */
static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief 将脉宽值(微秒)转换为LEDC硬件占空比计数值
 *
 * 转换公式: duty = pulse_us * LEDC_DUTY_MAX / ESC_PERIOD_US
 * 举例: 1500us -> 1500 * 16383 / 20000 = 1229
 *
 * 在映射前会先将脉宽限制在安全范围内 [ESC_MIN_US, ESC_MAX_US]。
 *
 * @param pulse_us  微秒单位的脉宽(1000~2000)
 * @return          LEDC占空比计数值(0~16383)
 */
static uint32_t pulse_us_to_duty(int pulse_us)
{
    pulse_us = clamp_int(pulse_us, ESC_MIN_US, ESC_MAX_US);  /* 先限幅 */
    return (uint32_t)((pulse_us * LEDC_DUTY_MAX) / ESC_PERIOD_US);  /* 比例映射 */
}

/**
 * @brief 软件反转：如果需要反转电机方向，将脉宽关于1500us做镜像
 *
 * 反转前后的对应关系(关于中立1500us对称):
 *   1000 <-> 2000  (全速反转 <-> 全速正转)
 *   1100 <-> 1900
 *   1300 <-> 1700
 *   1500 <-> 1500  (中立点不变)
 *
 * 目前两个电机的 reverse 宏都是0，所以此函数实际上不产生任何效果。
 * 保留这个函数是为了将来需要软件反向时直接改宏即可，不用改动函数。
 *
 * @param pulse_us  原始脉宽
 * @param reverse   是否启用反转(0=不反转, 1=反转)
 * @return          反转后的脉宽(或不反转则原样返回)
 */
static int apply_reverse(int pulse_us, int reverse)
{
    if (!reverse) {
        return pulse_us;  /* 不需要反转，直接返回 */
    }

    /* 镜像反转公式: 新值 = min + max - 原值
     *   原值=1000 -> 新值=1000+2000-1000=2000
     *   原值=1500 -> 新值=1000+2000-1500=1500 (中立不变)
     *   原值=2000 -> 新值=1000+2000-2000=1000 */
    return ESC_MIN_US + ESC_MAX_US - pulse_us;
}

// ===================== 编码器中断服务例程 =====================
/*
 * GPIO边沿中断(ISR)是编码器读取的"引擎"。
 * 任何时候编码器A相或B相的GPIO引脚电平发生变化，
 * ESP32硬件自动跳转到相应的ISR函数执行。
 *
 * ISR性能要求:
 *   - 执行时间必须极短(微秒级)
 *   - 不能做阻塞操作(没有printf, 没有延时, 不能获取信号量)
 *   - 必须放在IRAM中（这边没有显式IRAM_ATTR因为它足够短小，编译器
 *     可能会自动优化，但算法2版本加了IRAM_ATTR以保证安全）
 *   - 只在ISR中做最简单的操作: 读GPIO + 查表 + 更新计数
 *
 * 为什么为左/右编码器写两个独立的ISR函数而不是共用一个？
 *   - 两个编码器的状态变量(left_count/right_count, last_state)不同
 *   - 独立函数避免了在ISR中判断"是左还是右"的额外开销
 *   - ISR中的每一条指令都影响系统实时性，越简单越好
 */

/**
 * @brief 左轮编码器GPIO边沿中断服务函数
 *
 * 每当左编码器的A相或B相GPIO电平跳变时(上升沿或下降沿)，此函数被调用。
 *
 * 执行步骤:
 *   1. 读取A/B两相的当前GPIO电平(0=低, 1=高)
 *   2. 构建当前状态编码: now = (A<<1) | B [取值: 0,1,2,3]
 *   3. 构建查表索引: idx = (上一次状态<<2) | 当前状态 [取值: 0~15]
 *   4. 查表: 得到增量 delta ∈ {-1, 0, +1}
 *   5. 累加到64位计数器
 *   6. 保存当前状态为"下一次的上一次状态"
 *
 * @param arg 未使用(FreeRTOS ISR框架要求此参数)
 */
static void left_encoder_isr(void *arg)
{
    /* 读取两相当前电平 */
    int a = gpio_get_level(LEFT_ENC_A_GPIO);
    int b = gpio_get_level(LEFT_ENC_B_GPIO);

    /* 构建当前2位状态编码 */
    uint8_t now = (uint8_t)((a << 1) | b);
    /* 构建4位查表索引 */
    uint8_t index = (uint8_t)((left_last_state << 2) | now);

    /* 查表累加 */
    left_count += quad_table[index];
    /* 保存当前状态为下一次的上一次状态 */
    left_last_state = now;
}

/**
 * @brief 右轮编码器GPIO边沿中断服务函数
 *
 * 与左轮ISR逻辑完全相同，只是操作的变量是right_count和right_last_state。
 * 详细的逐步解释请参见 left_encoder_isr 的注释。
 *
 * @param arg 未使用
 */
static void right_encoder_isr(void *arg)
{
    int a = gpio_get_level(RIGHT_ENC_A_GPIO);
    int b = gpio_get_level(RIGHT_ENC_B_GPIO);

    uint8_t now = (uint8_t)((a << 1) | b);
    uint8_t index = (uint8_t)((right_last_state << 2) | now);

    right_count += quad_table[index];
    right_last_state = now;
}

// ===================== RC PWM输出函数 =====================
/*
 * 这些函数是电机驱动的真正"执行层"。
 * 每条函数都做了完整的错误保护:
 *   1. apply_reverse() —— 如果需要反转方向
 *   2. clamp_int()     —— 确保脉宽在1000~2000us安全范围内
 *   3. 保存到last_*_pulse —— 供状态显示使用
 *   4. pulse_us_to_duty() —— 映射到LEDC占空比
 *   5. ledc_set_duty()     —— 写入LEDC寄存器
 *   6. ledc_update_duty()  —— 通知硬件更新输出
 *
 * 为什么有 ler_set + ledc_update 两步？
 *   ledc_set_duty只修改内部寄存器值，不立即生效。
 *   ledc_update_duty才通知硬件在下一个PWM周期开始时切换到新值。
 *   这种设计可以让你一次性准备好多个通道的新占空比值，
 *   然后一次性同步更新，防止出现半刻的左轮更新了右轮还没更新的状态。
 *
 * ESP_ERROR_CHECK宏:
 *   检查返回值是否为ESP_OK，如果不是则立即abort并打印错误信息。
 *   在初始化阶段使用可以及早暴露硬件初始化错误。
 */

/**
 * @brief 设置左电调PWM脉宽并输出到硬件
 *
 * @param pulse_us 目标脉宽(微秒)，自动限制在1000~2000范围内
 */
static void set_left_pulse(int pulse_us)
{
    pulse_us = apply_reverse(pulse_us, LEFT_MOTOR_REVERSE);  /* 软件反向(如果需要) */
    pulse_us = clamp_int(pulse_us, ESC_MIN_US, ESC_MAX_US);  /* 安全限幅 */

    last_left_pulse = pulse_us;  /* 保存供状态显示 */

    uint32_t duty = pulse_us_to_duty(pulse_us);  /* 转换为占空比计数值 */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));   /* 设置占空比 */
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));      /* 硬件更新生效 */
}

/**
 * @brief 设置右电调PWM脉宽并输出到硬件
 *
 * 与左电调同理，操作的是LEDC_CHANNEL_1通道。
 *
 * @param pulse_us 目标脉宽(微秒)，自动限制在1000~2000范围内
 */
static void set_right_pulse(int pulse_us)
{
    pulse_us = apply_reverse(pulse_us, RIGHT_MOTOR_REVERSE);
    pulse_us = clamp_int(pulse_us, ESC_MIN_US, ESC_MAX_US);

    last_right_pulse = pulse_us;

    uint32_t duty = pulse_us_to_duty(pulse_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

/**
 * @brief 停止所有电机运动（回到中立1500us）
 *
 * 这是最安全的操作——任何时候如果情况不对，立即调用此函数。
 * 无论是"死人停车"计时器超时，还是用户按x键，最终都调用这个函数。
 */
static void motor_stop(void)
{
    set_left_pulse(ESC_MID_US);   /* 左轮回到中立 */
    set_right_pulse(ESC_MID_US);  /* 右轮回到中立 */
}

/**
 * @brief 前进：左右轮同速正转（脉宽 = 1500 + speed_delta）
 */
static void motor_forward(void)
{
    set_left_pulse(ESC_MID_US + speed_delta_us);
    set_right_pulse(ESC_MID_US + speed_delta_us);
}

/**
 * @brief 后退：左右轮同速反转（脉宽 = 1500 - speed_delta）
 */
static void motor_backward(void)
{
    set_left_pulse(ESC_MID_US - speed_delta_us);
    set_right_pulse(ESC_MID_US - speed_delta_us);
}

/**
 * @brief 左转：左轮反转、右轮正转（原地差速左转）
 *
 * 差速转向的物理原理:
 *   左轮速度 < 右轮速度 -> 车头向左偏转
 *   极端情况(原地转向): 左轮反转(-)、右轮正转(+) => 几乎绕中心旋转
 */
static void motor_turn_left(void)
{
    set_left_pulse(ESC_MID_US - speed_delta_us);   /* 左轮减速/反转 */
    set_right_pulse(ESC_MID_US + speed_delta_us);  /* 右轮加速/正转 */
}

/**
 * @brief 右转：左轮正转、右轮反转（原地差速右转）
 *
 * 左转的反向操作: 左轮加速、右轮减速
 */
static void motor_turn_right(void)
{
    set_left_pulse(ESC_MID_US + speed_delta_us);   /* 左轮加速/正转 */
    set_right_pulse(ESC_MID_US - speed_delta_us);  /* 右轮减速/反转 */
}

// ===================== 初始化函数 =====================
/*
 * 初始化顺序非常重要:
 *   1. esc_init()           —— 生成1500us中立信号（电调上电后需先收到中立信号）
 *   2. motor_stop()         —— 确保输出是停止状态
 *   3. command_input_init() —— 配置USB-JTAG串口(键盘输入)
 *   4. encoder_init()       —— 配置编码器GPIO和中断
 *
 * 为什么电调初始化要最先做？
 *   电调在上电瞬间如果没收到有效的PWM信号，会进入错误/保护状态。
 *   因此程序的第一个操作就是配置LEDC输出1500us中立信号，
 *   让电调一上电就"看到"有效的中立位信号，顺利完成自检。
 */

/**
 * @brief 初始化ESC电调RC-PWM输出
 *
 * 配置LEDC硬件模块，生成50Hz的标准RC舵机/电调PWM信号。
 *
 * LEDC配置说明:
 *   - 1个定时器(LEDC_TIMER_0)驱动2个通道(左=CH0, 右=CH1)
 *     两个通道共享同一个50Hz频率和14位分辨率
 *     但各自的占空比独立可调(所以在同一频率下左右轮速度可以不同)
 *   - 初始占空比设为对应1500us中立脉宽的值
 *   - LEDC_LOW_SPEED_MODE: ESP32-S3的低速模式(适合电机控制的低频PWM)
 *
 * LEDC的时钟源选择(LEDC_AUTO_CLK):
 *   ESP32有多个时钟源(APB_CLK 80MHz, RC_FAST_CLK 8MHz等)。
 *   LEDC_AUTO_CLK让硬件自动选择最合适的时钟源来尽可能精确地
 *   产生目标频率。对于50Hz低频信号，硬件会自动选择较低频率的
 *   时钟源以获得更高的占空比分辨率。
 */
static void esc_init(void)
{
    /* ---- 步骤1: 配置LEDC定时器 ---- */
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,     /* 低速模式(ESP32-S3) */
        .timer_num = LEDC_TIMER_0,             /* 使用定时器0 */
        .duty_resolution = LEDC_DUTY_RES,      /* 14位分辨率 */
        .freq_hz = ESC_FREQ_HZ,                /* 50Hz输出频率 */
        .clk_cfg = LEDC_AUTO_CLK               /* 自动选择最佳时钟源 */
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    /* ---- 步骤2: 配置左电调LEDC通道(通道0) ---- */
    ledc_channel_config_t left_channel = {
        .gpio_num = LEFT_ESC_GPIO,             /* 信号输出: GPIO4 */
        .speed_mode = LEDC_LOW_SPEED_MODE,      /* 低速模式 */
        .channel = LEDC_CHANNEL_0,             /* 通道0 */
        .intr_type = LEDC_INTR_DISABLE,        /* 不使用中断(PWM是纯输出) */
        .timer_sel = LEDC_TIMER_0,             /* 绑定定时器0 */
        .duty = pulse_us_to_duty(ESC_MID_US),  /* 初始占空比 = 1500us中立 */
        .hpoint = 0                            /* 无相位偏移(不用) */
    };

    ESP_ERROR_CHECK(ledc_channel_config(&left_channel));

    /* ---- 步骤3: 配置右电调LEDC通道(通道1) ---- */
    ledc_channel_config_t right_channel = {
        .gpio_num = RIGHT_ESC_GPIO,            /* 信号输出: GPIO5 */
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,             /* 通道1(与左通道共用定时器0) */
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = pulse_us_to_duty(ESC_MID_US),  /* 初始占空比 = 1500us中立 */
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&right_channel));

    /* ---- 步骤4: 确保输出中立信号 ---- */
    motor_stop();  /* 输出1500us停止信号 */
}

/**
 * @brief 初始化编码器GPIO引脚和中断服务
 *
 * 4个编码器信号引脚(GPIO6,7,15,16)都需要配置为:
 *   - 输入模式(GPIO_MODE_INPUT)
 *   - 启用内部上拉电阻(避免悬空状态导致电平不确定)
 *   - 任意边沿触发中断(ANYEDGE): 上升沿和下降沿都中断
 *     => 这正是4倍频解码需要的: 每个相的每次变化都要捕捉
 *
 * 初始化后，读取初始电平作为last_state的初始值。
 * 否则第一个中断到来时last_state是未定义值，查表可能得到
 * 错误的方向(只影响第一个脉冲，之后自动纠正)。
 *
 * gpio_install_isr_service(0):
 *   ESP32的GPIO中断由统一的ISR服务管理。在添加任何handler之前，
 *   必须先安装这个服务。参数0表示使用默认配置。
 *   如果已经安装过(其他组件可能已调用)，会返回ESP_ERR_INVALID_STATE，
 *   在本程序中我们用ESP_ERROR_CHECK会abort。生产代码中应该像
 *   chassis.c那样处理这个情况(允许"已安装"状态)。
 */
static void encoder_init(void)
{
    /* 一次性配置4个编码器引脚的GPIO属性 */
    gpio_config_t enc_conf = {
        .pin_bit_mask =
            (1ULL << LEFT_ENC_A_GPIO) |    /* GPIO6 */
            (1ULL << LEFT_ENC_B_GPIO) |    /* GPIO7 */
            (1ULL << RIGHT_ENC_A_GPIO) |   /* GPIO15 */
            (1ULL << RIGHT_ENC_B_GPIO),    /* GPIO16 */
        .mode = GPIO_MODE_INPUT,            /* 输入模式 */
        .pull_up_en = GPIO_PULLUP_ENABLE,   /* 内部上拉电阻(防止悬空) */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  /* 不下拉 */
        .intr_type = GPIO_INTR_ANYEDGE      /* 上升沿和下降沿都触发中断 => 4倍频 */
    };

    ESP_ERROR_CHECK(gpio_config(&enc_conf));

    /* 读取初始电平并记录为起始状态
     * 格式: (A相电平 << 1) | B相电平 = 0/1/2/3 */
    left_last_state = (uint8_t)((gpio_get_level(LEFT_ENC_A_GPIO) << 1) |
                                gpio_get_level(LEFT_ENC_B_GPIO));

    right_last_state = (uint8_t)((gpio_get_level(RIGHT_ENC_A_GPIO) << 1) |
                                  gpio_get_level(RIGHT_ENC_B_GPIO));

    /* 安装GPIO中断服务（必须在添加handler之前调用） */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    /* 为左编码器的A/B两相分别注册中断处理函数
     * 必须A/B两相都注册，才能捕捉两个通道的所有边沿(实现4倍频) */
    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_ENC_A_GPIO, left_encoder_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LEFT_ENC_B_GPIO, left_encoder_isr, NULL));

    /* 为右编码器的A/B两相分别注册中断处理函数 */
    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_ENC_A_GPIO, right_encoder_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RIGHT_ENC_B_GPIO, right_encoder_isr, NULL));
}

/**
 * @brief 初始化USB-JTAG串口（用于键盘遥控）
 *
 * ESP32-S3内置了USB-Serial-JTAG控制器，可以通过板载的原生USB口
 * （不是UART0的TX/RX引脚！）直接与电脑通信。
 *
 * 优点:
 *   - 不需要外接USB转串口芯片(如CH340/CP2102)
 *   - 速度最高可达3Mbps
 *   - 支持JTAG调试(同一根USB线既能烧录又能调试)
 *   - 不占用任何UART硬件资源
 *
 * 缓冲区配置:
 *   - TX: 1024字节(ESP32 -> 电脑)
 *   - RX: 1024字节(电脑 -> ESP32) —— 键盘输入
 */
static void command_input_init(void)
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 1024,  /* 发送缓冲区大小 */
        .rx_buffer_size = 1024,  /* 接收缓冲区大小 */
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
}

// ===================== 状态显示函数 =====================

/**
 * @brief 打印当前运行状态
 *
 * 显示以下信息:
 *   - 速度力度(speed_delta_us): 当前的中立脉宽偏移量
 *   - 当前脉宽(L/R): 左右电调正在输出的实际PWM脉宽
 *   - 编码器计数(L/R): 左右编码器的累加脉冲数
 *   - 反转标志: 是否启用了软件方向反转
 *
 * PRId64 是C99跨平台整数格式化宏:
 *   在32位平台上展开成 "lld"，64位展开成 "ld"。
 *   确保在不同平台上 formatted print 64位整数都不会出错。
 */
static void print_status(void)
{
    printf("\nStatus:\n");
    printf("  speed_delta_us = %d\n", speed_delta_us);
    printf("  pulse L/R      = %dus / %dus\n", last_left_pulse, last_right_pulse);
    printf("  encoder L/R    = %" PRId64 " / %" PRId64 "\n", left_count, right_count);
    printf("  left reverse   = %d\n", LEFT_MOTOR_REVERSE);
    printf("  right reverse  = %d\n", RIGHT_MOTOR_REVERSE);
}

/**
 * @brief 打印帮助信息（键盘命令列表）
 */
static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  w = forward\n");
    printf("  s = backward\n");
    printf("  a = turn left\n");
    printf("  d = turn right\n");
    printf("  x = stop\n");
    printf("  c = show encoder count\n");
    printf("  z = show status\n");
    printf("  + = increase speed\n");
    printf("  - = decrease speed\n");
    printf("  h = help\n\n");
}

// ===================== 主程序入口 =====================
/*
 * app_main() 是ESP-IDF应用程序的入口函数，等价于普通C程序中的main()。
 * FreeRTOS在底层已经创建好了第一个任务(idle任务除外)并开始运行，
 * 所以进入app_main时已经是一个完整的FreeRTOS任务上下文了。
 *
 * 本程序只有一个任务(app_main的while循环)，不需要创建额外任务。
 * 编码器数据的采集完全靠GPIO硬件中断驱动，不消耗CPU时间。
 */

void app_main(void)
{
    // ---- 第0步: 初始化电调输出 ----
    // 这是最优先的操作——让电调在第一时间收到中立PWM信号
    esc_init();
    motor_stop();  /* 确保中立信号持续输出 */

    // ---- 第1步: 初始化USB-JTAG串口(键盘输入) ----
    command_input_init();

    // ---- 第2步: 初始化编码器(配置GPIO和中断) ----
    encoder_init();

    // ---- 第3步: 打印欢迎信息和接线说明 ----
    printf("\n========================================\n");
    printf("ESP32-S3 + APO-DL COMMAND CONTROL\n");
    printf("RC mode: 50Hz, 1000-2000us, 1500us stop\n");
    printf("GPIO4 -> APO-DL A channel S\n");
    printf("GPIO5 -> APO-DL B channel S\n");
    printf("GND   -> APO-DL A/B channel -\n");
    printf("APO-DL A/B channel + not connected\n");
    printf("========================================\n\n");

    printf("Output 1500us stop now.\n");
    printf("If APO-DL was reset, calibrate center at 1500us.\n");

    print_help();   /* 打印按键列表 */
    print_status(); /* 打印初始状态 */

    // ---- 第4步: 主控制循环 ----
    /*
     * 主循环以轮询模式运行，不创建多任务:
     *   1. 每20ms检查是否有键盘输入(非阻塞，超时后立即继续)
     *   2. 如果有按键: 解析命令并执行对应电机动作
     *   3. 如果电机在运行: 更新超时计时器
     *      -> 累计超过1.2秒则自动停止
     *   4. 重复
     *
     * 这种简单的轮询架构适合"最小验证程序"——
     * 所有响应在20ms内，对键盘遥控来说完全够用。
     * 如果是需要高实时性的控制(如50Hz传感器融合+底盘闭环)，
     * 则需要像算法2那样用 vTaskDelayUntil 实现精确周期性控制。
     */
    uint8_t ch = 0;               /* 存放读取到的单个按键字符 */
    int motor_running = 0;        /* 电机是否处于运行状态(1=运行中, 0=已停止) */
    int timeout_count = 0;        /* 死人停车计时器(毫秒) */

    while (1) {
        /*
         * 从USB-JTAG串口读取1个字节，超时20ms。
         * 这个函数是非阻塞的——如果用户在20ms内没有按键:
         *   len=0, 跳过命令分发，直接进入超时检查。
         * 如果用户按键:
         *   len>0, 解析命令、重置超时计时器。
         */
        int len = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(20));

        if (len > 0) {
            /* 忽略回车和换行符(有些终端会在字符后自动添加) */
            if (ch == '\r' || ch == '\n') {
                continue;
            }

            /* 每次收到新按键都重置死人停车计时器
             * (只有移动命令w/s/a/d需要重置，但这里统一重置更简单) */
            timeout_count = 0;

            /* ---- 命令分发: 根据按键执行对应功能 ---- */

            if (ch == 'w' || ch == 'W') {
                motor_forward();           /* 调用前进函数 */
                motor_running = 1;         /* 标记电机进入运行状态 */
                printf("CMD: FORWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 's' || ch == 'S') {
                motor_backward();          /* 调用后退函数 */
                motor_running = 1;
                printf("CMD: BACKWARD | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'a' || ch == 'A') {
                motor_turn_left();         /* 调用左转函数 */
                motor_running = 1;
                printf("CMD: TURN LEFT | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'd' || ch == 'D') {
                motor_turn_right();        /* 调用右转函数 */
                motor_running = 1;
                printf("CMD: TURN RIGHT | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'x' || ch == 'X') {
                motor_stop();              /* 立即停止 */
                motor_running = 0;         /* 退出运行状态 */
                timeout_count = 0;         /* 清零超时计时器 */
                printf("CMD: STOP | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);

            } else if (ch == 'c' || ch == 'C') {
                /* 显示编码器当前累加计数 */
                printf("Encoder count L/R: %" PRId64 " / %" PRId64 "\n",
                       left_count, right_count);

            } else if (ch == 'z' || ch == 'Z') {
                /* 显示完整运行状态 */
                print_status();

            } else if (ch == '+') {
                /* 增加速度力度: +50us */
                speed_delta_us += 50;
                /* 限制最大值不超过500us */
                speed_delta_us = clamp_int(speed_delta_us,
                                           MIN_SPEED_DELTA_US,
                                           MAX_SPEED_DELTA_US);
                printf("Speed increased: speed_delta_us = %d\n", speed_delta_us);

            } else if (ch == '-') {
                /* 减少速度力度: -50us */
                speed_delta_us -= 50;
                /* 限制最小值不低于100us */
                speed_delta_us = clamp_int(speed_delta_us,
                                           MIN_SPEED_DELTA_US,
                                           MAX_SPEED_DELTA_US);
                printf("Speed decreased: speed_delta_us = %d\n", speed_delta_us);

            } else if (ch == 'h' || ch == 'H') {
                /* 重新显示帮助信息 */
                print_help();

            } else {
                /* 未知按键: 提示用户按h查看帮助 */
                printf("Unknown command: %c\n", ch);
                printf("Press h for help.\n");
            }
        }

        /*
         * 死人停车逻辑:
         *   如果电机当前在运行(motor_running=1):
         *     每次循环增加20ms(即非阻塞读取的超时值)
         *     当累计超过 COMMAND_TIMEOUT_MS(1200ms) 时:
         *       自动调用 motor_stop() 停止
         *       重置运行状态和计时器
         *
         * 这个机制确保了即使PC端程序崩溃/键盘卡键/
         * ESC自检失败导致电机不受控等意外情况，
         * 行李箱也最多只会跑1.2秒就自动停下。
         */
        if (motor_running) {
            timeout_count += 20;  /* 每次主循环增加20ms */

            if (timeout_count >= COMMAND_TIMEOUT_MS) {
                motor_stop();           /* 超时，强制停止 */
                motor_running = 0;      /* 清除运行标志 */
                timeout_count = 0;      /* 重置计时器 */
                printf("TIMEOUT STOP | pulse L/R: %dus / %dus\n",
                       last_left_pulse, last_right_pulse);
            }
        }
    }
}
