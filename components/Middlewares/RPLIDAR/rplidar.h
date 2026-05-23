/**
 * @file rplidar_c1.h
 * @brief 基于 ESP-IDF 原生 UART 的思岚 RPLIDAR C1 驱动头文件 (不依赖 Arduino.h)
 * @date 2026
 */

#ifndef RPLIDAR_C1_H
#define RPLIDAR_C1_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 雷达协议常量定义
// ============================================================================
#define LIDAR_START_BYTE       0xA5  // 请求报文起始标志 [cite: 93]
#define LIDAR_CMD_STOP         0x25  // 停止扫描 [cite: 191]
#define LIDAR_CMD_RESET        0x40  // 软重启 [cite: 191]
#define LIDAR_CMD_SCAN         0x20  // 开始传统扫描 [cite: 191]
#define LIDAR_CMD_GET_HEALTH   0x52  // 获取健康状态 [cite: 191]
#define LIDAR_CMD_GET_INFO     0x50  // 获取设备信息 [cite: 191]
#define LIDAR_CMD_MOTOR_SPEED  0xA8  // 调节电机转速 [cite: 191, 718]

// ============================================================================
// 数据结构体
// ============================================================================

/**
 * @brief 解析后的单点激光点云数据
 */
typedef struct {
    float angle;         // 实际角度 (0.0 ~ 360.0 度) 
    float distance;      // 实际距离 (单位: 毫米 mm) 
    uint8_t quality;     // 信号强弱质量 
    bool start_bit;      // 是否为新一圈扫描的起始点 
} rplidar_point_t;

/**
 * @brief 雷达硬件固件信息
 */
typedef struct {
    uint8_t major_model;
    uint8_t sub_model;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t hardware;
    char serial_num[33]; // 16字节唯一序列号的Hex文本表示 [cite: 539]
} rplidar_info_t;

// ============================================================================
// 驱动核心 API 函数声明
// ============================================================================

/**
 * @brief 初始化 ESP32-S3 的底层硬件 UART
 * @param uart_num UART控制器端口号 (如 UART_NUM_1 或 UART_NUM_2)
 * @param tx_io_num 发送引脚 GPIO
 * @param rx_io_num 接收引脚 GPIO
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t rplidar_init(uart_port_t uart_num, int tx_io_num, int rx_io_num);

/**
 * @brief 开启雷达扫描模式
 */
esp_err_t rplidar_start_scan(uart_port_t uart_num);

/**
 * @brief 停止雷达扫描，进入低功耗
 */
void rplidar_stop(uart_port_t uart_num);

/**
 * @brief 软重启雷达核心
 */
void rplidar_reset(uart_port_t uart_num);

/**
 * @brief 获取当前雷达的健康状态
 */
esp_err_t rplidar_get_health(uart_port_t uart_num, uint8_t *status, uint16_t *error_code);

/**
 * @brief 获取雷达固件与序列号信息
 */
esp_err_t rplidar_get_device_info(uart_port_t uart_num, rplidar_info_t *info_out);

/**
 * @brief 在线控制雷达电机的目标转速
 */
esp_err_t rplidar_set_motor_speed(uart_port_t uart_num, uint16_t rpm);

/**
 * @brief 高频非阻塞读取单个点云数据（内部集成状态机滑窗字节对齐机制）
 * @return true 成功解析出一个有效点云；false 暂无完整包或数据校验失败
 */
bool rplidar_read_point(uart_port_t uart_num, rplidar_point_t *point_out);

#ifdef __cplusplus
}
#endif

#endif // RPLIDAR_C1_H