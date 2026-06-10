/**
 * vl53l1x.h
 * ESP32-S3 Wrapper API for VL53L1X
 */

#ifndef _APP_VL53L1X_H_
#define _APP_VL53L1X_H_

#include <stdint.h>
#include "esp_err.h"

// 根据正点原子原理图提取的引脚
#define I2C_MASTER_SCL_IO           38
#define I2C_MASTER_SDA_IO           GPIO_NUM_39
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000 

// 官方默认 I2C 地址 (8位)
#define VL53L1X_DEFAULT_DEV_ADDR    0x52

/**
 * @brief 初始化 ESP32-S3 I2C 接口以及 VL53L1X 传感器
 * @param dev_addr 传感器 I2C 地址，默认填 VL53L1X_DEFAULT_DEV_ADDR
 * @return ESP_OK 表示成功，其他表示失败
 */
esp_err_t vl53l1x_app_init(uint16_t dev_addr);

/**
 * @brief 获取一次单次测距数据
 * @param dev_addr 传感器 I2C 地址
 * @param distance 指向保存距离变量的指针（单位：毫米）
 * @return ESP_OK 表示成功
 */
esp_err_t vl53l1x_get_single_distance(uint16_t dev_addr, uint16_t *distance);

#endif // _APP_VL53L1X_H_