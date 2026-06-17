// #include "vl53l1x.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// void app_main(void) {
//     uint16_t distance = 0;
    
//     // 初始化传感器
//     if (vl53l1x_app_init(VL53L1X_DEFAULT_DEV_ADDR) == ESP_OK) {
//         while(1) {
//             // 获取距离
//             if(vl53l1x_get_single_distance(VL53L1X_DEFAULT_DEV_ADDR, &distance) == ESP_OK) {
//                 ESP_LOGI("APP", "Distance: %d mm", distance);
//             }
//             vTaskDelay(pdMS_TO_TICKS(500));
//         }
//     }
// }























// #include "vl53l1x.h"
// #include <stdio.h>          // 引入printf头文件
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// void app_main(void) {
//     uint16_t distance = 0;
    
//     // 初始化传感器
//     if (vl53l1x_app_init(VL53L1X_DEFAULT_DEV_ADDR) == ESP_OK) {
//         while(1) {
//             // 获取距离
//             if(vl53l1x_get_single_distance(VL53L1X_DEFAULT_DEV_ADDR, &distance) == ESP_OK) {
//                 printf("Distance: %d mm\n", distance);
//             }
//             vTaskDelay(pdMS_TO_TICKS(500));
//         }
//     }
// }


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rplidar.h"

// 定义串口号与引脚 (可根据你的实际接线修改)
#define LIDAR_UART_NUM  UART_NUM_1
#define LIDAR_TX_PIN    17  // ESP32 TX -> Lidar 绿线 (RX)
#define LIDAR_RX_PIN    18  // ESP32 RX -> Lidar 黄线 (TX)

static const char *TAG = "RPLIDAR_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Initializing RPLIDAR C1...");

    // 1. 初始化雷达底层的硬件 UART
    esp_err_t err = rplidar_init(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART Initialization failed!");
        return;
    }

    // 2. 停止当前动作并软重启，确保雷达状态干净
    ESP_LOGI(TAG, "Resetting Lidar...");
    rplidar_stop(LIDAR_UART_NUM);
    rplidar_reset(LIDAR_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待雷达核心重启完毕

    // 3. 读取雷达健康状态
    uint8_t health_status = 0;
    uint16_t error_code = 0;
    if (rplidar_get_health(LIDAR_UART_NUM, &health_status, &error_code) == ESP_OK) {
        ESP_LOGI(TAG, "Health Status: %d, Error Code: 0x%04X", health_status, error_code);
    } else {
        ESP_LOGW(TAG, "Failed to get health status. Check wiring!");
    }

    // 4. 读取设备序列号
    rplidar_info_t info;
    if (rplidar_get_device_info(LIDAR_UART_NUM, &info) == ESP_OK) {
        ESP_LOGI(TAG, "Device Model: %d.%d, Firmware: %d.%d", 
                 info.major_model, info.sub_model, info.firmware_major, info.firmware_minor);
        ESP_LOGI(TAG, "Serial Number: %s", info.serial_num);
    }

    // 5. 启动扫描模式
    ESP_LOGI(TAG, "Starting scan...");
    if (rplidar_start_scan(LIDAR_UART_NUM) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scan mode!");
        return;
    }

    // 6. 主循环：持续读取与解析点云
    rplidar_point_t point;
    ESP_LOGI(TAG, "Entering scanning loop. Waiting for data...");
    
    while (1) {
        // 调用底层 API 读取数据
        if (rplidar_read_point(LIDAR_UART_NUM, &point)) {
            // 解析成功，打印有效数据 (可根据质量过滤掉无效噪声点)
            if (point.quality > 0) {
                ESP_LOGI(TAG, "Angle: %6.2f°, Dist: %7.2f mm, Quality: %3d, NewScan: %d",
                         point.angle, point.distance, point.quality, point.start_bit);
            }
        } else {
            // 当前缓冲区无完整数据包，挂起任务 1 毫秒防看门狗超时
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}