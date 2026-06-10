#include "vl53l1x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    uint16_t distance = 0;
    
    // 初始化传感器
    if (vl53l1x_app_init(VL53L1X_DEFAULT_DEV_ADDR) == ESP_OK) {
        while(1) {
            // 获取距离
            if(vl53l1x_get_single_distance(VL53L1X_DEFAULT_DEV_ADDR, &distance) == ESP_OK) {
                ESP_LOGI("APP", "Distance: %d mm", distance);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}























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