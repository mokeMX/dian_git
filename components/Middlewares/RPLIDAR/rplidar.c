/**
 * @file rplidar_c1.c
 * @brief 基于 ESP-IDF 原生 UART 的思岚 RPLIDAR C1 驱动实现文件
 * @date 2026
 */

#include "rplidar.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "RPLIDAR";

// 初始化 ESP32-S3 硬件串口
esp_err_t rplidar_init(uart_port_t uart_num, int tx_io_num, int rx_io_num) {
    uart_config_t uart_config = {
        .baud_rate = 460800,                    // C1M1 固定使用 460800 波特率 
        .data_bits = UART_DATA_8_BITS,          // 8位数据位 [cite: 955]
        .parity    = UART_PARITY_DISABLE,       // 无校验位 [cite: 955]
        .stop_bits = UART_STOP_BITS_1,          // 1位停止位 [cite: 955]
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 关闭流控
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 配置串口参数
    esp_err_t err = uart_param_config(uart_num, &uart_config);
    if (err != ESP_OK) return err;

    // 映射 GPIO 引脚
    err = uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // 分配大缓冲区 (2048字节) 保证 5KHz 测距不发生硬件溢出 [cite: 811, 888]
    err = uart_driver_install(uart_num, 2048, 0, 0, NULL, 0);
    return err;
}

// 内部底层命令发送封装
static void send_cmd(uart_port_t uart_num, uint8_t cmd) {
    uint8_t pkt[2] = { LIDAR_START_BYTE, cmd }; // 0xA5 + 命令码 [cite: 93]
    uart_write_bytes(uart_num, (const char*)pkt, 2);
}

// 停止扫描
void rplidar_stop(uart_port_t uart_num) {
    send_cmd(uart_num, LIDAR_CMD_STOP);
    vTaskDelay(pdMS_TO_TICKS(20)); // 发送 STOP 后需要延迟 10ms 以上 [cite: 200]
    uart_flush(uart_num);          // 强制刷空接收缓冲区
}

// 核心软重启
void rplidar_reset(uart_port_t uart_num) {
    send_cmd(uart_num, LIDAR_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(600)); // 软重启后建议延迟 500ms 以上 [cite: 212]
}

// 获取雷达健康状态
esp_err_t rplidar_get_health(uart_port_t uart_num, uint8_t *status, uint16_t *error_code) {
    rplidar_stop(uart_num); // 确保退出多次应答模式才能处理单次应答 [cite: 72]

    send_cmd(uart_num, LIDAR_CMD_GET_HEALTH);

    uint8_t resp[10]; // 7 字节头 + 3 字节状态负载 [cite: 230, 563]
    int len = uart_read_bytes(uart_num, resp, 10, pdMS_TO_TICKS(100));
    if (len < 10) return ESP_ERR_TIMEOUT;

    if (resp[0] != 0xA5 || resp[1] != 0x5A) {   // 校验起始标志 0xA5 0x5A [cite: 129]
        return ESP_FAIL;
    }

    *status = resp[7];                          // 0:良好, 1:警告, 2:错误 [cite: 581]
    *error_code = (resp[9] << 8) | resp[8];     // 小端格式拼装错误码 [cite: 574, 578]
    return ESP_OK;
}

// 获取设备序列号及固件信息
esp_err_t rplidar_get_device_info(uart_port_t uart_num, rplidar_info_t *info_out) {
    rplidar_stop(uart_num);

    send_cmd(uart_num, LIDAR_CMD_GET_INFO);

    uint8_t resp[27]; // 7 字节头 + 20 字节基本负载 [cite: 481, 485]
    int len = uart_read_bytes(uart_num, resp, 27, pdMS_TO_TICKS(100));
    if (len < 27) return ESP_ERR_TIMEOUT;

    if (resp[0] != 0xA5 || resp[1] != 0x5A) return ESP_FAIL;

    info_out->major_model    = resp[7];         // 主型号 [cite: 492]
    info_out->sub_model      = resp[8];         // 子型号 [cite: 493]
    info_out->firmware_minor = resp[9];         // 固件次版本 [cite: 497]
    info_out->firmware_major = resp[10];        // 固件主版本 [cite: 501]
    info_out->hardware       = resp[11];        // 硬件版本 [cite: 505]

    for (int i = 0; i < 16; i++) {
        sprintf(&info_out->serial_num[i * 2], "%02X", resp[12 + i]); // 16字节序列号 [cite: 509, 513]
    }
    info_out->serial_num[32] = '\0';
    return ESP_OK;
}

// 控制扫描电机目标转速
esp_err_t rplidar_set_motor_speed(uart_port_t uart_num, uint16_t rpm) {
    uint8_t pkt[6];
    pkt[0] = LIDAR_START_BYTE;
    pkt[1] = LIDAR_CMD_MOTOR_SPEED;
    pkt[2] = 0x02;                              // 负载长 2 字节 [cite: 720]
    pkt[3] = rpm & 0xFF;                        // 目标 RPM 低字节 [cite: 725]
    pkt[4] = (rpm >> 8) & 0xFF;                 // 目标 RPM 高字节

    uint8_t checksum = 0;                       // 协议要求的异或校验和
    for (int i = 0; i < 5; i++) {
        checksum ^= pkt[i];
    }
    pkt[5] = checksum;

    int written = uart_write_bytes(uart_num, (const char*)pkt, 6);
    return (written == 6) ? ESP_OK : ESP_FAIL;
}

// 进入扫描采样模式
esp_err_t rplidar_start_scan(uart_port_t uart_num) {
    send_cmd(uart_num, LIDAR_CMD_SCAN);

    uint8_t start_resp[7];                      // 期待标准的 7 字节应答头 [cite: 230]
    int len = uart_read_bytes(uart_num, start_resp, 7, pdMS_TO_TICKS(150));
    if (len < 7 || start_resp[0] != 0xA5 || start_resp[1] != 0x5A) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

// 核心解析器：使用无阻塞底层字节状态机实现精准对齐与解析
bool rplidar_read_point(uart_port_t uart_num, rplidar_point_t *point_out) {
    static uint8_t pkt_buf[5];
    static int state = 0;
    uint8_t ch;

    // 每次从硬件缓冲区非阻塞地读取 1 个字节
    while (uart_read_bytes(uart_num, &ch, 1, 0) > 0) {
        switch (state) {
            case 0:
                // 校验第 1 字节：根据协议，S 位与 !S 位必定取反，因此异或结果必为 1 
                if (((ch ^ (ch >> 1)) & 0x01) == 1) {
                    pkt_buf[0] = ch;
                    state = 1;
                }
                break;

            case 1:
                // 校验第 2 字节：校验位 C 必须恒为 1 [cite: 251, 268]
                if ((ch & 0x01) == 1) {
                    pkt_buf[1] = ch;
                    state = 2;
                } else {
                    // 校验失败，当前的 ch 有可能刚好是错位后的下一包的第 1 字节，重新判定
                    state = 0;
                    if (((ch ^ (ch >> 1)) & 0x01) == 1) {
                        pkt_buf[0] = ch;
                        state = 1;
                    }
                }
                break;

            case 2:
                pkt_buf[2] = ch;
                state = 3;
                break;

            case 3:
                pkt_buf[3] = ch;
                state = 4;
                break;

            case 4:
                pkt_buf[4] = ch;
                state = 0; // 成功集齐 5 字节，状态机复位 [cite: 234]

                // --- 触发解算 ---
                point_out->start_bit = (pkt_buf[0] & 0x01); // 提取新一圈开始标志 
                point_out->quality  = (pkt_buf[0] >> 2);   // 信号质量 

                // 解算角度：15位定点小数，实际角度 = angle_q6 / 64.0f [cite: 250, 255, 268]
                uint16_t angle_q6 = (pkt_buf[2] << 7) | (pkt_buf[1] >> 1);
                point_out->angle = (float)angle_q6 / 64.0f;

                // 解算距离：16位定点小数，实际距离 = distance_q2 / 4.0f mm [cite: 259, 263, 268]
                uint16_t distance_q2 = (pkt_buf[4] << 8) | pkt_buf[3];
                point_out->distance = (float)distance_q2 / 4.0f;

                return true; // 成功输出一个有效测距点
        }
    }
    return false; // 当前缓冲区暂时没有拼凑成一个完整的数据包
}