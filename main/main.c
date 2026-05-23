/**
 ******************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       音乐播放器实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ******************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 ******************************************************************************
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "led.h"
#include "myiic.h"
#include "my_spi.h"
#include "xl9555.h"
#include "spilcd.h"
#include "spi_sd.h"
#include "sdmmc_cmd.h"
#include "text.h"
#include "fonts.h"
#include "key.h"
#include "text.h"
#include "fonts.h"
#include "es8388.h"
#include "myi2s.h"
#include "exfuns.h"
#include "audioplay.h"
#include <stdio.h>
#include "rplidar.h"



/**
 * @file main.c
 * @brief 纯 ESP-IDF 开发环境下驱动测试主程序
 */

// 选用硬件 UART 2，映射至 ESP32-S3 的普通 GPIO 上
#define LIDAR_UART_PORT   UART_NUM_2
#define LIDAR_TX_GPIO     17
#define LIDAR_RX_GPIO     18

void app_main(void) {
    ESP_LOGI("MAIN", "正在启动思岚 C1M1 原生硬件驱动层...");

    // 1. 初始化 UART 硬件
    if (rplidar_init(LIDAR_UART_PORT, LIDAR_TX_GPIO, LIDAR_RX_GPIO) != ESP_OK) {
        ESP_LOGE("MAIN", "错误: 串口驱动硬件安装失败！");
        return;
    }

    // 2. 读取硬件出厂信息
    rplidar_info_t dev_info;
    if (rplidar_get_device_info(LIDAR_UART_PORT, &dev_info) == ESP_OK) {
        ESP_LOGI("MAIN", "[雷达连接成功] 型号代码: %d, 固件版本: %d.%d, S/N: %s",
                 dev_info.major_model, dev_info.firmware_major, dev_info.firmware_minor, dev_info.serial_num);
    }

    // 3. 验证硬件健康状况
    uint8_t health_status;
    uint16_t error_code;
    if (rplidar_get_health(LIDAR_UART_PORT, &health_status, &error_code) == ESP_OK) {
        if (health_status == 0) {
            ESP_LOGI("MAIN", "健康自检通过，正在开启雷达并设置工作频率...");
            
            // // 在线设定电机转速为 600 RPM (典型 10Hz 扫描频) [cite: 813, 889]
            // rplidar_set_motor_speed(LIDAR_UART_PORT, 600); 
            
            // 启动扫描
            rplidar_start_scan(LIDAR_UART_PORT);
        } else {
            ESP_LOGE("MAIN", "雷达硬件故障！状态码: %d, 错误码: 0x%04X", health_status, error_code);
            return;
        }
    }

    // 4. 数据高频轮询业务循环
    rplidar_point_t p;
    while (1) {
        // 持续轮询底层的状态机
        if (rplidar_read_point(LIDAR_UART_PORT, &p)) {
            // 剔除雷达盲区或无反射点所导致的零距离无效数据 
            if (p.distance > 0.0f) {
                if (p.start_bit) {
                    printf("\n>>> [一圈 360 度点云数据刷新] <<<\n");
                }
                // 实时无阻塞打印获取到的距离和角位移
                printf("角度: %5.1f°, 距离: %5.0f mm (信号质量: %d)\n", p.angle, p.distance, p.quality);
            }
        }
        
        // 极短地切出 CPU 权限，防止喂狗机制报错 (在 FreeRTOS 下轮询任务必须加延时)
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}


























// /**
//  * @brief       程序入口
//  * @param       无
//  * @retval      无
//  */
// void app_main(void)
// {
//     esp_err_t ret;
//     uint8_t key = 0;

//     ret = nvs_flash_init();     /* 初始化NVS */
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }

//     led_init();                 /* LED初始化 */
//     my_spi_init();              /* SPI初始化 */
//     key_init();                 /* KEY初始化 */
//     myiic_init();               /* MYIIC初始化 */
//     xl9555_init();              /* XL9555初始化 */
//     spilcd_init();              /* SPILCD初始化 */

//     while (es8388_init())       /* ES8388初始化 */
//     {
//         spilcd_show_string(30, 110, 200, 16, 16, "ES8388 Error", RED);
//         vTaskDelay(pdMS_TO_TICKS(200));
//         spilcd_fill(30, 110, 239, 126, WHITE);
//         vTaskDelay(pdMS_TO_TICKS(200));
//     }

// //     xl9555_pin_write(SPK_EN_IO, 0);     /* 打开喇叭 */

// //     while (sd_spi_init())       /* 检测不到SD卡 */
// //     {
// //         spilcd_show_string(30, 110, 200, 16, 16, "SD Card Error!", RED);
// //         vTaskDelay(pdMS_TO_TICKS(500));
// //         spilcd_show_string(30, 130, 200, 16, 16, "Please Check! ", RED);
// //         vTaskDelay(pdMS_TO_TICKS(500));
// //     }
// //     ret = exfuns_init();    /* 为fatfs相关变量申请内存 */

// //     while (fonts_init())    /* 检查字库 */
// //     {
// //         spilcd_clear(WHITE);
// //         spilcd_show_string(30, 30, 200, 16, 16, "ESP32-S3", RED);
        
// //         key = fonts_update_font(30, 50, 16, (uint8_t *)"0:", RED);  /* 更新字库 */
        
// //         while (key)         /* 更新失败 */
// //         {
// //             spilcd_show_string(30, 50, 200, 16, 16, "Font Update Failed!", RED);
// //             vTaskDelay(pdMS_TO_TICKS(200));
// //             spilcd_fill(20, 50, 200 + 20, 90 + 16, WHITE);
// //             vTaskDelay(pdMS_TO_TICKS(200));
// //         }

// //         spilcd_show_string(30, 50, 200, 16, 16, "Font Update Success!   ", RED);
// //         vTaskDelay(pdMS_TO_TICKS(1000));
// //         spilcd_clear(WHITE);   
// //     }

// //   uint8_t hz1[] = "路";
// //     text_show_string(10, 10, 200, 16, "KEY0:NEXT  KEY1:PREV", 16, 0, RED);
// //     text_show_string(10, 30, 200, 16, "KEY2:PAUSE/PLAY", 16, 0, RED);
// //     //   text_show_font(80, 30, hz1, 16, 0, RED);
// //     vTaskDelay(pdMS_TO_TICKS(1000));

//     while (1)
//     {



//         audio_play();       /* 播放音乐 */
//     }
// }