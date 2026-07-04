/**
 * sensor_hub - 传感器诊断中心 (传感器修改5)
 *
 * 功能：同时在单个ESP32-S3上初始化并驱动全部7个传感器，通过串口
 * 输出实时原始数据，用于硬件调试、接线验证和性能测试。
 *
 * 这既不是生产固件也不是控制固件，而是一个"传感器诊断仪"——
 * 像一个万用表，帮助开发者确认每个传感器是否工作正常。
 *
 * 被测试的7个传感器：
 *   1. A02YYUW #1 (超声波距离传感器) —— SW-UART，GPIO4
 *   2. A02YYUW #2 (超声波距离传感器) —— SW-UART，GPIO5
 *   3. BU UWB (超宽带定位模块)       —— HW-UART1，GPIO6/7
 *   4. FSR ADC (力敏电阻/压力传感器) —— ADC1通道7，GPIO8
 *   5. RPLIDAR C1 (激光扫描雷达)      —— HW-UART2，GPIO17/18
 *   6. IMU (9轴惯性测量单元)         —— I2C0，地址0x23
 *   7. VL53L1X ToF (飞行时间测距)     —— I2C0，地址0x52
 *
 * 【硬件资源调度亮点】：
 *   - ESP32系列芯片只有3个硬件UART (0/1/2)
 *   - UART0: 系统日志/烧录(Console)
 *   - UART1: BU UWB 专用
 *   - UART2: RPLIDAR C1 专用
 *   - 两个A02YYUW超声波模块必须使用软件模拟串口(SW-UART)
 *     因为它们波特率仅9600，速度慢，用软件模拟不影响性能
 *   - I2C0总线被IMU和VL53L1X ToF两个设备共享（不同的I2C地址）
 *
 * 【任务调度策略】：
 *   - ESP32-S3是双核处理器(Core 0 + Core 1)
 *   - Core 0: 分配给两个A02YYUW超声波任务（对时序最敏感）
 *   - Core 1: 分配给其余所有任务（雷达、UWB、FSR、IMU、ToF）
 *   - 通过物理隔离核心，避免高带宽设备(UWB/雷达)的中断干扰
 *     超声波软件模拟串口的时序
 *
 * 硬件平台：ESP32-S3
 * 框架：ESP-IDF + FreeRTOS
 */

/* ================================================================
 * 头文件引用
 * ================================================================ */
#include <stdio.h>          /* printf 标准输出 */
#include <stdint.h>         /* 固定宽度整数类型 */
#include <string.h>         /* memset 等字符串/内存操作 */

#include "freertos/FreeRTOS.h"    /* FreeRTOS 核心 */
#include "freertos/task.h"        /* 任务管理 */
#include "driver/i2c_master.h"    /* I2C主机驱动 */
#include "esp_err.h"              /* ESP-IDF 错误码类型 */

/* 全部7个传感器驱动 */
#include "a02yyuw.h"              /* 超声波距离传感器 */
#include "bu_uwb.h"               /* UWB超宽带定位模块 */
#include "fsr_adc.h"              /* 力敏电阻(ADC采集) */
#include "imu_i2c.h"              /* IMU惯性测量单元(I2C) */
#include "rplidar_c1.h"           /* RPLIDAR C1激光雷达 */
#include "vl53l1x_tof.h"          /* VL53L1X飞行时间测距传感器 */

/* ================================================================
 * 引脚和硬件常量定义
 *
 * 关键设计决策（与传感器修改4相比）：
 *   - A02YYUW #1 从硬件UART1移至软件模拟UART(SW-UART)
 *     释放UART1给BU UWB使用
 *   - 所有其他引脚分配保持不变
 *   - 注意：GPIO38/39在经典ESP32上仅支持输入，但ESP32-S3
 *     支持双向IO，所以可以做I2C SDA/SCL
 * ================================================================ */

#define HUB_I2C_PORT           0       /* I2C端口号：0 */
#define HUB_I2C_SDA_GPIO      39       /* I2C SDA数据线引脚 */
#define HUB_I2C_SCL_GPIO      38       /* I2C SCL时钟线引脚 */
#define HUB_I2C_SPEED_HZ      400000   /* I2C总线速率：400kHz(快速模式) */

/* ---- 超声波传感器 #1 (SW-UART, GPIO4) ---- */
#define A02_1_RX_GPIO         4        /* 超声波#1的RX引脚（接收数据） */
#define A02_1_TX_GPIO         (-1)     /* 超声波#1的TX引脚（不需要发送，填-1） */
#define A02_1_UART_PORT        1       /* UART端口号（仅占位，因为使用SW-UART） */
#define A02_1_BAUDRATE        9600     /* 波特率：9600 */
#define A02_1_USE_SW_UART      1       /* 强制启用软件模拟串口 */

/* ---- 超声波传感器 #2 (SW-UART, GPIO5) ---- */
#define A02_2_RX_GPIO         5        /* 超声波#2的RX引脚 */
#define A02_2_TX_GPIO         (-1)     /* 超声波#2的TX引脚（不需要发送） */
#define A02_2_UART_PORT        2       /* UART端口号（仅占位） */
#define A02_2_BAUDRATE        9600     /* 波特率：9600 */
#define A02_2_USE_SW_UART      1       /* 强制启用软件模拟串口 */

/* ---- BU UWB 超宽带定位 (HW-UART1, GPIO6/7) ---- */
#define BU_UWB_UART_PORT       1       /* 使用硬件UART1 */
#define BU_UWB_RX_GPIO         6       /* UWB模块的TX -> ESP32的RX(GPIO6) */
#define BU_UWB_TX_GPIO         7       /* UWB模块的RX -> ESP32的TX(GPIO7) */
#define BU_UWB_BAUDRATE        115200  /* 波特率：115200 */

/* ---- FSR 力敏电阻 (ADC1通道7, GPIO8) ---- */
#define FSR_ADC_GPIO           8       /* FSR使用的ADC输入引脚 */
#define FSR_ADC_CHANNEL        7       /* ADC通道号(ADC1_CHANNEL_7) */

/* ---- RPLIDAR C1 激光雷达 (HW-UART2, GPIO17/18) ---- */
#define RPLIDAR_UART_PORT      2       /* 使用硬件UART2 */
#define RPLIDAR_RX_GPIO       17       /* 雷达TX -> ESP32 RX(GPIO17) */
#define RPLIDAR_TX_GPIO       18       /* 雷达RX -> ESP32 TX(GPIO18) */
#define RPLIDAR_BAUDRATE       460800  /* 波特率：460800(雷达出厂固定值) */

/* ---- IMU 惯性测量单元 (I2C0, 7位地址0x23) ---- */
#define IMU_I2C_ADDR           0x23    /* IMU的I2C 7位设备地址 */

/* ---- VL53L1X ToF 传感器 (I2C0, 8位地址0x52) ---- */
#define VL53L1X_ADDR_8BIT             0x52    /* ToF的I2C 8位写地址 */
#define VL53L1X_TIMING_BUDGET_MS      50      /* 单次测量时间预算(ms)：50ms=快速模式 */
#define VL53L1X_INTER_MEASUREMENT_MS  55      /* 测量间隔(ms)：需 >= timing_budget */

/* ================================================================
 * 全局共享资源和设备句柄
 *
 * 所有传感器设备句柄定义为全局静态变量，在整个app中可见。
 * I2C总线句柄被IMU和ToF两个设备共享使用。
 * ================================================================ */

static i2c_master_bus_handle_t g_shared_i2c;  /* I2C总线句柄（被IMU和ToF共用） */

static a02yyuw_t g_a02_1;        /* 超声波传感器#1的句柄 */
static a02yyuw_t g_a02_2;        /* 超声波传感器#2的句柄 */
static rplidar_c1_t g_lidar;     /* 激光雷达句柄 */
static imu_i2c_t g_imu;          /* IMU句柄 */
static vl53l1x_tof_t g_tof;      /* VL53L1X ToF句柄 */
static volatile bool g_lidar_scan_active;  /* 激光雷达扫描是否已启动（volatile：ISR/多核共享） */

/* ================================================================
 * 辅助工具函数
 * ================================================================ */

/**
 * @brief 打印传感器初始化结果的状态信息
 * @param name 传感器名称（用于标识）
 * @param ret  esp_err_t 类型的返回值
 */
static void print_status(const char *name, esp_err_t ret)
{
    printf("%s init: %s\n", name, (ret == ESP_OK) ? "OK" : "FAIL");
}

/**
 * @brief 以十六进制格式打印字节数组
 *        用于调试UWB等二进制协议数据
 * @param data 字节数组指针
 * @param len  字节数量
 */
static void print_hex(const uint8_t *data, int len)
{
    printf("[BU_UWB][HEX]");
    for (int i = 0; i < len; ++i) {
        printf(" %02X", data[i]);  /* 每个字节以2位大写十六进制输出 */
    }
    printf("\n");
}

/**
 * @brief 处理BU UWB模块接收到的原始字节数据
 *
 * 这是UWB数据解析的核心函数。BU UWB模块通过UART发送以换行符(\n)
 * 分隔的文本行数据。本函数将字节流转成行，然后解析每行的内容。
 *
 * 支持的协议行类型：
 *   - TWR (Two-Way Ranging)：包含x/y/z三维坐标和距离
 *   - DATA：通用数据行（如配置回显）
 *   - ERROR：错误信息
 *   - DISTANCE：简单距离数据
 *
 * @param data 原始字节数据
 * @param len  字节长度
 */
static void handle_bu_uwb_rx(const uint8_t *data, int len)
{
    static char line[BU_UWB_LINE_MAX];  /* 行缓冲区（静态变量，跨调用保持） */
    static size_t line_pos;              /* 当前行写入位置 */

    printf("\n========== BU_UWB DATA RECEIVED ==========\n");
    printf("[BU_UWB] RX len=%d bytes\n", len);
    printf("[BU_UWB][RAW] %.*s\n", len, (const char *)data);  /* 打印原始文本 */
    print_hex(data, len);  /* 同时打印十六进制，方便对比 */

    /* 逐字节处理，构建文本行 */
    for (int i = 0; i < len; ++i) {
        const char ch = (char)data[i];
        
        if (ch == '\r') {
            /* 跳过回车符(\r)，只处理换行符(\n)作为行结束 */
            continue;
        } else if (ch == '\n') {
            /* 遇到换行符：一行结束，开始解析 */
            line[line_pos] = '\0';  /* 添加字符串终止符 */
            
            /* 分类当前行的协议类型 */
            const bu_uwb_line_type_t type = (line_pos > 0) ? bu_uwb_classify_line(line) : BU_UWB_LINE_UNKNOWN;
            const char *payload = bu_uwb_line_payload(line);  /* 提取有效负载（去除类型前缀） */
            bu_uwb_distance_t distance = {0};
            bu_uwb_twr_reading_t twr = {0};

            printf("[BU_UWB][LINE] %s\n", line);

            /* 根据行类型分别处理 */
            switch (type) {
                case BU_UWB_LINE_DATA:
                    /* 通用数据行：打印负载内容 */
                    printf("[BU_UWB][DATA] %s\n", payload);
                    break;
                case BU_UWB_LINE_ERROR:
                    /* 错误行：打印错误信息 */
                    printf("[BU_UWB][ERR] %s\n", payload);
                    break;
                case BU_UWB_LINE_TWR:
                    /* TWR定位数据行：解析并输出三维坐标信息 */
                    if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
                        printf("[BU_UWB][TWR] frame=%s anchor=%s D=%dcm "
                               "Xcm=%d Ycm=%d R=%d P=%d T=%d V=%d O=%d "
                               "xyz=%d,%d,%d\n",
                               twr.frame_id, twr.anchor_id,          /* 帧ID和锚点ID */
                               twr.distance_cm,                       /* 距离(厘米) */
                               twr.x_cm, twr.y_cm,                   /* 平面坐标(厘米) */
                               twr.r, twr.p, twr.timestamp,          /* 姿态角和时间戳 */
                               twr.validity, twr.orientation,        /* 有效性和方向 */
                               twr.x, twr.y, twr.z);                 /* 三维坐标 */
                    }
                    break;
                default:
                    /* 未知行类型：忽略 */
                    break;
            }

            /* 无论什么类型，都尝试解析距离帧（存在TWR和距离在同一行的情况） */
            if (bu_uwb_parse_distance_line(line, &distance) && distance.valid) {
                printf("[BU_UWB][DISTANCE] %d mm %.3f m\n", distance.distance_mm, distance.distance_m);
            }
            
            /* 重置行缓冲，准备接收下一行 */
            line_pos = 0;
        } else {
            /* 正常字符：加入行缓冲区 */
            if (line_pos + 1 < sizeof(line)) {
                line[line_pos++] = ch;
            } else {
                /* 缓冲区溢出（行太长）——重置并警告 */
                line_pos = 0;
                printf("[BU_UWB][WARN] line too long, dropped\n");
            }
        }
    }
}

/* ================================================================
 * 传感器任务
 *
 * 每个传感器在独立的FreeRTOS任务中运行，互不阻塞。
 * 这样可以同时观察所有传感器的输出，判断每个传感器是否正常工作。
 *
 * 任务命名约定：task_<传感器名>
 * ================================================================ */

/* ---- A02YYUW #1 超声波传感器任务 (GPIO4, SW-UART) ---- */

/**
 * @brief 超声波传感器 #1 数据读取任务
 *
 * 使用软件模拟串口(GPIO位操作)读取A02YYUW的数据帧。
 * A02YYUW以约100ms间隔主动输出距离数据（9600波特率）。
 *
 * 核心0运行，优先级4（最高用户优先级，确保软件模拟串口时序准确）
 */
static void task_a02yyuw1(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        a02yyuw_reading_t r = {0};
        /* 阻塞读取直到收到有效数据或超时150ms */
        esp_err_t ret = a02yyuw_read_dev(&g_a02_1, &r, 150);
        
        if (ret == ESP_OK) {
            if (r.valid) {
                printf("[A02YYUW#1] distance=%d mm\n", r.distance_mm);
            } else {
                printf("[A02YYUW#1] no valid frame (RX=GPIO%d)\n", A02_1_RX_GPIO);
            }
        } else {
            printf("[A02YYUW#1] read error (RX=GPIO%d)\n", A02_1_RX_GPIO);
        }
        vTaskDelay(pdMS_TO_TICKS(500));  /* 500ms轮询间隔 */
    }
}

/* ---- A02YYUW #2 超声波传感器任务 (GPIO5, SW-UART) ---- */

/**
 * @brief 超声波传感器 #2 数据读取任务
 *
 * 与#1的功能完全相同，只是使用不同的GPIO引脚。
 * 两个超声波任务运行在同一个核心(Core 0)上，FreeRTOS会时间片轮转调度。
 *
 * 核心0运行，优先级4
 */
static void task_a02yyuw2(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        a02yyuw_reading_t r = {0};
        esp_err_t ret = a02yyuw_read_dev(&g_a02_2, &r, 150);
        
        if (ret == ESP_OK) {
            if (r.valid) {
                printf("[A02YYUW#2] distance=%d mm\n", r.distance_mm);
            } else {
                printf("[A02YYUW#2] no valid frame (RX=GPIO%d)\n", A02_2_RX_GPIO);
            }
        } else {
            printf("[A02YYUW#2] read error (RX=GPIO%d)\n", A02_2_RX_GPIO);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---- BU UWB 超宽带定位任务 (UART1, GPIO6/7) ---- */

/**
 * @brief BU UWB模块数据接收和解析任务
 *
 * 监听UART1接收的UWB模块数据流，将字节数据传递给
 * handle_bu_uwb_rx()函数进行逐行解析。
 *
 * UWB模块以中等频率（通常10~50Hz）输出TWR定位帧，
 * 波特率115200。
 *
 * 核心1运行，优先级2
 */
static void task_bu_uwb(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t no_data_count;  /* 无数据计数器（静态变量跨调用保持） */
    while (1) {
        uint8_t rx[128] = {0};     /* 接收缓冲区，每帧最多128字节 */
        int rx_len = 0;
        /* 从UWB模块串口读取字节，超时100ms */
        esp_err_t ret = bu_uwb_read_bytes(rx, sizeof(rx), &rx_len, 100);
        
        if (ret == ESP_OK && rx_len > 0) {
            no_data_count = 0;  /* 有数据，重置计数器 */
            handle_bu_uwb_rx(rx, rx_len);  /* 解析并打印 */
        } else {
            /* 无数据：累加计数器，达到阈值后打印调试提示 */
            no_data_count++;
            if (no_data_count >= 4) {  /* 连续4次无数据（约400ms） */
                no_data_count = 0;
                printf("[BU_UWB][WAIT] no data yet; check "
                       "PA2/TX -> ESP32 RX GPIO%d, common GND, "
                       "baud=%d, kit output mode\n",
                       BU_UWB_RX_GPIO, BU_UWB_BAUDRATE);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  /* 100ms轮询间隔 */
    }
}

/* ---- FSR 力敏电阻任务 (ADC1通道7, GPIO8) ---- */

/**
 * @brief FSR力敏电阻数据读取任务
 *
 * 通过ESP32的ADC(模数转换器)读取FSR的模拟电压，转换为
 * 原始ADC值、电压值和估算的压力重量。
 *
 * FSR是电阻式传感器：压力越大，电阻越低，电压越高。
 *
 * 核心1运行，优先级1
 */
static void task_fsr(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        fsr_adc_reading_t r = {0};
        esp_err_t ret = fsr_adc_read(&r);  /* 单次ADC采样 */
        
        if (ret == ESP_OK && r.valid) {
            printf("[FSR] raw=%d voltage=%.3fV force_est_kg=%.2f\n",
                   r.raw, r.voltage_v, r.weight_kg);
        }
        vTaskDelay(pdMS_TO_TICKS(500));  /* 500ms采样间隔 */
    }
}

/* ---- RPLIDAR C1 激光雷达任务 (UART2, GPIO17/18) ---- */

/**
 * @brief RPLIDAR C1激光雷达数据读取任务
 *
 * RPLIDAR C1以固定频率旋转（约5~10Hz），每个扫描周期输出
 * 数百个角度-距离点对。本任务读取这些点并实时打印。
 *
 * 任务逻辑：
 *   1. 扫描激活后，循环读取点数据
 *   2. 跳过无效点(distance_mm <= 0)
 *   3. 每找到一个有效点就打印并退出本次循环
 *   4. 如果长时间无有效点或扫描未启动，打印诊断提示
 *
 * 核心1运行，优先级3
 */
static void task_rplidar(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t no_point_count;  /* 无有效点计数器 */
    while (1) {
        rplidar_c1_point_t point = {0};
        int got_point = 0;  /* 本次是否找到有效点 */

        /* 最多尝试50次读取，防止无限循环阻塞其他任务 */
        for (int i = 0; g_lidar_scan_active && i < 50; ++i) {
            if (rplidar_c1_read_point(&g_lidar, &point) && point.distance_mm > 0.0f) {
                printf("[RPLIDAR] angle=%.2f distance=%.1f quality=%u start=%d\n",
                       point.angle_deg, point.distance_mm, point.quality, point.start_bit);
                got_point = 1;
                break; /* 找到有效点，跳出循环 */
            }
            vTaskDelay(pdMS_TO_TICKS(1));  /* 等待1ms再尝试 */
        }

        if (!g_lidar_scan_active) {
            /* 扫描未启动：可能是初始化失败或被中断 */
            no_point_count++;
            if (no_point_count >= 4) {
                no_point_count = 0;
                printf("[RPLIDAR][WAIT] scan did not start; "
                       "check 5V power, rotation, UART direction and baud=%d\n", 
                       RPLIDAR_BAUDRATE);
                vTaskDelay(pdMS_TO_TICKS(200));
                /* 尝试重新启动扫描 */
                g_lidar_scan_active = (rplidar_c1_start_scan(&g_lidar) == ESP_OK);
            }
        } else {
            if (got_point) {
                no_point_count = 0;  /* 有数据，重置计数器 */
            } else {
                no_point_count++;
                if (no_point_count >= 4) {
                    no_point_count = 0;
                    printf("[RPLIDAR][WAIT] no valid scan point; "
                           "check rotation, 5V current, ESP_RX=GPIO%d <- lidar TX\n",
                           RPLIDAR_RX_GPIO);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));  /* 50ms轮询间隔 */
    }
}

/* ---- IMU 惯性测量单元任务 (I2C0, 7位地址0x23) ---- */

/**
 * @brief IMU 9轴惯性测量单元数据读取任务
 *
 * 通过I2C总线高速读取IMU的数据（加速度 + 欧拉角）。
 * 加速度单位：g(重力加速度倍数)
 * 欧拉角单位：度
 * 欧拉角顺序：[0]=Roll(横滚), [1]=Pitch(俯仰), [2]=Yaw(偏航)
 *
 * 核心1运行，优先级2
 */
static void task_imu(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        imu_i2c_reading_t data = {0};
        esp_err_t ret = imu_i2c_read_all(&g_imu, &data);  /* 读取全量数据 */
        
        if (ret == ESP_OK && data.valid) {
            printf("[IMU] accel=%.3f %.3f %.3f g euler=%.2f %.2f %.2f deg\n",
                   data.accel_g[0], data.accel_g[1], data.accel_g[2],
                   data.euler_deg[0], data.euler_deg[1], data.euler_deg[2]);
        } else {
            printf("[IMU][WAIT] no valid data; check SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n",
                   HUB_I2C_SDA_GPIO, HUB_I2C_SCL_GPIO, IMU_I2C_ADDR);
        }
        vTaskDelay(pdMS_TO_TICKS(200));  /* 200ms采样间隔（约5Hz） */
    }
}

/* ---- VL53L1X ToF 飞行时间传感器任务 (I2C0, 8位地址0x52) ---- */

/**
 * @brief VL53L1X ToF(飞行时间)激光测距传感器任务
 *
 * VL53L1X是STMicroelectronics的ToF激光测距芯片，
 * 通过测量红外激光的飞行时间来计算距离（最高4米）。
 *
 * 与IMU共享同一I2C总线(I2C0)，但使用不同的设备地址(0x52)。
 * I2C总线支持多设备挂载，通过地址区分。
 *
 * 核心1运行，优先级2
 */
static void task_vl53l1x(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        vl53l1x_tof_reading_t r = {0};
        /* 阻塞读取，超时250ms */
        esp_err_t ret = vl53l1x_tof_read(&g_tof, &r, 250);
        
        if (ret == ESP_OK && r.valid) {
            printf("[VL53L1X] distance=%u mm\n", r.distance_mm);
        }
        vTaskDelay(pdMS_TO_TICKS(250));  /* 250ms采样间隔 */
    }
}

/* ================================================================
 * 系统启动入口 app_main
 *
 * 按顺序执行以下步骤：
 *   1. 初始化I2C总线
 *   2. 依次初始化所有7个传感器（带错误检查）
 *   3. 创建独立的FreeRTOS任务驱动每个传感器
 *
 * 任务分配策略（双核优化）：
 *   - Core 0：两个超声波任务（优先级4，保护SW-UART时序）
 *   - Core 1：所有其他任务（UART中断的上下文在核心间均匀分散）
 * ================================================================ */

void app_main(void)
{
    printf("\nAutobox sensor hub test start\n");
    printf("Default I2C: SDA=GPIO%d SCL=GPIO%d\n", HUB_I2C_SDA_GPIO, HUB_I2C_SCL_GPIO);

    /* ---- 1. 共享I2C总线初始化 ---- */
    /*
     * I2C是总线型协议，多个设备可以共享同一条SDA/SCL线。
     * 这里初始化一次总线，后续IMU和ToF都使用这个句柄。
     */
    const i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = HUB_I2C_PORT,                    /* I2C端口号 */
        .sda_io_num = HUB_I2C_SDA_GPIO,              /* SDA引脚 */
        .scl_io_num = HUB_I2C_SCL_GPIO,              /* SCL引脚 */
        .clk_source = I2C_CLK_SRC_DEFAULT,           /* 默认时钟源 */
        .glitch_ignore_cnt = 7,                      /* 硬件尖峰脉冲滤波：忽略<7个时钟周期 */
        .flags.enable_internal_pullup = true,        /* 启用内部上拉电阻 */
    };
    print_status("shared_i2c", i2c_new_master_bus(&i2c_bus_cfg, &g_shared_i2c));

    /* ---- 2. A02YYUW #1 超声波传感器初始化 (GPIO4, SW-UART) ---- */
    a02yyuw_config_t a02_cfg = a02yyuw_default_config(
        (uart_port_t)A02_1_UART_PORT,                /* UART端口(占位) */
        A02_1_RX_GPIO, A02_1_TX_GPIO);              /* RX/TX引脚 */
    a02_cfg.baudrate = A02_1_BAUDRATE;               /* 波特率 9600 */
    a02_cfg.use_sw_uart = A02_1_USE_SW_UART;         /* 启用软件模拟串口 */
    print_status("a02yyuw#1", a02yyuw_init_dev(&g_a02_1, &a02_cfg));
    printf("[A02YYUW#1] SW-UART RX=GPIO%d baud=%d\n", A02_1_RX_GPIO, a02_cfg.baudrate);

    /* ---- 3. A02YYUW #2 超声波传感器初始化 (GPIO5, SW-UART) ---- */
    a02yyuw_config_t a02b_cfg = a02yyuw_default_config(
        (uart_port_t)A02_2_UART_PORT,                /* UART端口(占位) */
        A02_2_RX_GPIO, A02_2_TX_GPIO);              /* RX/TX引脚 */
    a02b_cfg.baudrate = A02_2_BAUDRATE;               /* 波特率 9600 */
    a02b_cfg.use_sw_uart = A02_2_USE_SW_UART;         /* 启用软件模拟串口 */
    print_status("a02yyuw#2", a02yyuw_init_dev(&g_a02_2, &a02b_cfg));
    printf("[A02YYUW#2] SW-UART RX=GPIO%d baud=%d\n", A02_2_RX_GPIO, a02b_cfg.baudrate);

    /* ---- 4. BU UWB 超宽带定位模块初始化 (UART1, GPIO6/7) ---- */
    /*
     * UWB模块以"被动监听"模式工作——ESP32只接收UWB模块的输出，
     * 不发送任何指令。固件在UWB模块上独立运行，通过UART向外
     * 推送TWR定位数据。
     */
    bu_uwb_config_t bu_cfg = bu_uwb_default_config(
        (uart_port_t)BU_UWB_UART_PORT,               /* 使用硬件UART1 */
        BU_UWB_RX_GPIO, BU_UWB_TX_GPIO);             /* RX/TX引脚 */
    bu_cfg.baudrate = BU_UWB_BAUDRATE;                /* 波特率 115200 */
    print_status("bu_uwb", bu_uwb_init(&bu_cfg));
    printf("[BU_UWB] passive UART monitor mode: PA2/TX -> ESP32 RX GPIO%d, "
           "PA3/RX -> ESP32 TX GPIO%d optional, baud=%d\n",
           BU_UWB_RX_GPIO, BU_UWB_TX_GPIO, bu_cfg.baudrate);

    /* ---- 5. FSR 力敏电阻初始化 (ADC1通道7, GPIO8) ---- */
    /*
     * FSR通过ESP32的12位SAR ADC采集电压值。
     * 电压范围为0~参考电压(通常3.3V)，原始值范围0~4095(2^12-1)。
     */
    fsr_adc_config_t fsr_cfg = fsr_adc_default_config();
    fsr_cfg.adc_gpio = FSR_ADC_GPIO;                 /* ADC输入引脚 */
    fsr_cfg.adc_channel = (adc_channel_t)FSR_ADC_CHANNEL;  /* ADC通道号 */
    print_status("fsr_adc", fsr_adc_init(&fsr_cfg));

    /* ---- 6. RPLIDAR C1 激光雷达初始化 (UART2, GPIO17/18) ---- */
    /*
     * RPLIDAR C1是SLAMTEC的360度激光扫描测距雷达。
     * 初始化步骤：串口配置 -> 获取设备信息 -> 检查健康状态 -> 启动扫描。
     * 
     * 注意：
     *   - 雷达需要5V供电（USB或外部电源），逻辑电平3.3V兼容ESP32
     *   - 波特率460800是C1的固定值，不可更改
     *   - 雷达的马达需要约500ms达到稳定转速后才能正常输出数据
     */
    rplidar_c1_config_t lidar_cfg = rplidar_c1_default_config(
        (uart_port_t)RPLIDAR_UART_PORT,              /* 使用硬件UART2 */
        RPLIDAR_RX_GPIO, RPLIDAR_TX_GPIO);           /* RX/TX引脚 */
    lidar_cfg.baudrate = RPLIDAR_BAUDRATE;            /* 波特率 460800 */
    esp_err_t lidar_ret = rplidar_c1_init(&g_lidar, &lidar_cfg);
    print_status("rplidar_c1", lidar_ret);
    printf("[RPLIDAR] UART%d baud=%d ESP_RX=GPIO%d ESP_TX=GPIO%d\n",
           RPLIDAR_UART_PORT, lidar_cfg.baudrate, RPLIDAR_RX_GPIO, RPLIDAR_TX_GPIO);

    if (lidar_ret == ESP_OK) {
        /* 获取雷达的型号和固件版本信息 */
        rplidar_c1_info_t info = {0};
        esp_err_t ret_i = rplidar_c1_get_info(&g_lidar, &info);
        if (ret_i == ESP_OK) {
            printf("[RPLIDAR][INFO] model=%u.%u firmware=%u.%u hardware=%u sn=%s\n",
                   info.major_model, info.sub_model,           /* 主/子型号 */
                   info.firmware_major, info.firmware_minor,   /* 固件主/次版本 */
                   info.hardware, info.serial_num);            /* 硬件版本和序列号 */
        } else {
            printf("[RPLIDAR][WAIT] get info failed; check 5V/GND/TX/RX, baud=%d\n", lidar_cfg.baudrate);
        }

        /* 获取雷达健康状态：0=正常，1=警告，2=错误 */
        uint8_t health_status = 0xFF;
        uint16_t health_error = 0;
        esp_err_t ret_h = rplidar_c1_get_health(&g_lidar, &health_status, &health_error);
        if (ret_h == ESP_OK) {
            printf("[RPLIDAR][HEALTH] status=%u error=0x%04X\n", health_status, health_error);
        } else {
            printf("[RPLIDAR][WAIT] get health failed; check power/current and UART direction\n");
        }

        /* 启动激光扫描（雷达开始旋转并输出点云数据） */
        esp_err_t scan_ret = rplidar_c1_start_scan(&g_lidar);
        print_status("rplidar_start_scan", scan_ret);
        g_lidar_scan_active = (scan_ret == ESP_OK);  /* 记录扫描是否成功启动 */
    }

    /* ---- 7. IMU 惯性测量单元初始化 (I2C0, 7位地址0x23) ---- */
    /*
     * IMU芯片挂载在共享的I2C总线上。
     * 初始化后读取固件版本以确认通信正常。
     */
    imu_i2c_config_t imu_cfg = imu_i2c_default_config();
    imu_cfg.sda_gpio = HUB_I2C_SDA_GPIO;              /* SDA引脚 */
    imu_cfg.scl_gpio = HUB_I2C_SCL_GPIO;              /* SCL引脚 */
    imu_cfg.scl_speed_hz = HUB_I2C_SPEED_HZ;          /* I2C速率 400kHz */
    imu_cfg.device_address = IMU_I2C_ADDR;             /* 设备7位地址 */
    imu_cfg.external_bus = g_shared_i2c;               /* 使用共享I2C总线 */
    esp_err_t imu_ret = imu_i2c_init(&g_imu, &imu_cfg);
    print_status("imu_i2c", imu_ret);
    
    if (imu_ret == ESP_OK) {
        /* 读取IMU固件版本（3字节：主版本.次版本.修订号） */
        uint8_t version[3] = {0};
        esp_err_t ret_v = imu_i2c_read_version(&g_imu, version);
        if (ret_v == ESP_OK) {
            printf("[IMU] version=%u.%u.%u addr=0x%02X SDA=GPIO%d SCL=GPIO%d\n",
                   version[0], version[1], version[2],
                   imu_cfg.device_address, imu_cfg.sda_gpio, imu_cfg.scl_gpio);
        } else {
            printf("[IMU][WAIT] version read failed; check VCC/GND/SDA/SCL and addr=0x%02X\n",
                   imu_cfg.device_address);
        }
    }

    /* ---- 8. VL53L1X ToF 飞行时间传感器初始化 (I2C0, 8位地址0x52) ---- */
    /*
     * VL53L1X的8位地址为0x52（7位地址为0x29）。
     * 配置测量时间预算和测量间隔，直接影响精度和响应速度：
     *   - timing_budget_ms=50：快速模式，精度略低但响应快
     *   - inter_measurement_ms=55：测量间隔需 >= timing_budget
     */
    vl53l1x_tof_config_t tof_cfg = vl53l1x_tof_default_config();
    tof_cfg.sda_gpio = HUB_I2C_SDA_GPIO;              /* SDA引脚 */
    tof_cfg.scl_gpio = HUB_I2C_SCL_GPIO;              /* SCL引脚 */
    tof_cfg.scl_speed_hz = HUB_I2C_SPEED_HZ;          /* I2C速率 400kHz */
    tof_cfg.device_address_8bit = VL53L1X_ADDR_8BIT;   /* 设备8位地址 */
    tof_cfg.timing_budget_ms = VL53L1X_TIMING_BUDGET_MS;          /* 测量时间预算 */
    tof_cfg.inter_measurement_ms = VL53L1X_INTER_MEASUREMENT_MS;  /* 测量间隔 */
    tof_cfg.external_bus = g_shared_i2c;               /* 使用共享I2C总线 */
    print_status("vl53l1x_tof", vl53l1x_tof_init(&g_tof, &tof_cfg));

    /* ---- 9. 启动所有传感器任务 ---- */
    /*
     * 【双核调度优化】：
     * 
     * Core 0（核心0）：
     *   - 分配两个超声波任务（优先级4）
     *   - 理由：超声波使用软件模拟串口(GPIO bit-bang)，对CPU中断
     *     延迟极度敏感。将Core 0上的其他任务降到最少，
     *     避免被UART/SPI等中断抢占总线导致时序错误。
     * 
     * Core 1（核心1）：
     *   - 分配雷达(UART2)、UWB(UART1)、FSR(ADC)、IMU(I2C)、ToF(I2C)
     *   - 理由：这些外设都有专用的硬件控制器处理实时数据，
     *     任务本身以轮询/回调方式工作，无需精确到微秒级的时序。
     *   - 优先级从高到低：rplidar(3) > bu_uwb(2) = imu(2) = vl53(2) > fsr(1)
     */

    /* 将两个超声波任务固定在 Core 0，并且将优先级提高到 4 (最高用户优先级) */
    xTaskCreatePinnedToCore(task_a02yyuw1, "a02_1", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(task_a02yyuw2, "a02_2", 4096, NULL, 4, NULL, 0);

    /* 将高速的雷达和 UWB 固定在 Core 1，避免它们的中断干扰 Core 0 的时序 */
    xTaskCreatePinnedToCore(task_rplidar,  "rplidar", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(task_bu_uwb,   "bu_uwb", 4096, NULL, 2, NULL, 1);
    
    /* 其他 I2C/ADC 任务要求不高，放到 Core 1 */
    xTaskCreatePinnedToCore(task_fsr,      "fsr",    4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(task_imu,      "imu",    4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_vl53l1x,  "vl53",   4096, NULL, 2, NULL, 1);

    printf("All 7 sensor tasks launched.\n");
}


