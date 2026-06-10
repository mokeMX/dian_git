/**
 * vl53l1x.c
 * ESP32-S3 Wrapper Implementation
 */

#include "vl53l1x.h"
#include "VL53L1X_api.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "VL53L1X";

esp_err_t vl53l1x_app_init(uint16_t dev_addr) {
    // 1. 初始化 ESP32 I2C 主机
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    // 如果提示已经初始化，可以忽略错误
    i2c_param_config(I2C_MASTER_NUM, &conf);
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return err;
    }

  // 2. 等待传感器启动 (增加超时防卡死机制)
    uint8_t bootState = 0;
    int timeout = 100; // 最大等待 200ms
    while(bootState == 0 && timeout > 0){
        esp_err_t res = VL53L1X_BootState(dev_addr, &bootState);
        if(res != 0) {
            ESP_LOGE(TAG, "I2C 通信失败，请检查接线(SDA=41, SCL=42)或上拉电阻！");
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        timeout--;
    }
    
    if (timeout == 0) {
        ESP_LOGE(TAG, "传感器启动超时！(可能是 XSHUT 为低电平导致未开机)");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VL53L1X Booted!");

    // 3. 传感器参数初始化
    int8_t status = VL53L1X_SensorInit(dev_addr);
    if(status != 0) {
        ESP_LOGE(TAG, "Sensor Init failed!");
        return ESP_FAIL;
    }
    
    // 可选：设置测距模式为长距离 (1=Short, 2=Long)
    VL53L1X_SetDistanceMode(dev_addr, 2); 
    // 可选：设置测量定时预算(Timing Budget)，50ms 比较平衡
    VL53L1X_SetTimingBudgetInMs(dev_addr, 50);
    // 可选：设置两次测量之间的间隔，应大于预算
    VL53L1X_SetInterMeasurementInMs(dev_addr, 55); 

    ESP_LOGI(TAG, "VL53L1X Init Successfully");
    return ESP_OK;
}

esp_err_t vl53l1x_get_single_distance(uint16_t dev_addr, uint16_t *distance) {
    uint8_t dataReady = 0;
    int8_t status = 0;

    // 开启测距
    status = VL53L1X_StartRanging(dev_addr);
    if (status != 0) return ESP_FAIL;

    // 轮询等待数据就绪 (可以加一个超时保护)
    int timeout = 100; // 100 * 2ms = 200ms
    while (dataReady == 0 && timeout > 0) {
        VL53L1X_CheckForDataReady(dev_addr, &dataReady);
        vTaskDelay(pdMS_TO_TICKS(2));
        timeout--;
    }

    if (timeout == 0) {
        VL53L1X_StopRanging(dev_addr);
        return ESP_ERR_TIMEOUT;
    }

    // 读取距离并清理中断
    VL53L1X_GetDistance(dev_addr, distance);
    VL53L1X_ClearInterrupt(dev_addr);
    
    // 停止测距 (如果是单次测量)
    VL53L1X_StopRanging(dev_addr);

    return ESP_OK;
}