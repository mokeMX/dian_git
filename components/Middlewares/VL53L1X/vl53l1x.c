#include "vl53l1x.h"
#include "VL53L1X_api.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VL53L1X";

extern i2c_master_bus_handle_t bus_handle;

esp_err_t vl53l1x_app_init(uint16_t dev_addr) {
    uint8_t bootState = 0;
    int timeout = 100;

    while(bootState == 0 && timeout > 0){
        esp_err_t res = VL53L1X_BootState(dev_addr, &bootState);
        if(res != 0) {
            ESP_LOGE(TAG, "I2C communication failed, check wiring (SDA=41, SCL=42)!");
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        timeout--;
    }
    
    if (timeout == 0) {
        ESP_LOGE(TAG, "Sensor boot timeout! (XSHUT may be LOW)");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VL53L1X Booted!");

    int8_t status = VL53L1X_SensorInit(dev_addr);
    if(status != 0) {
        ESP_LOGE(TAG, "Sensor Init failed!");
        return ESP_FAIL;
    }
    
    VL53L1X_SetDistanceMode(dev_addr, 2);
    VL53L1X_SetTimingBudgetInMs(dev_addr, 50);
    VL53L1X_SetInterMeasurementInMs(dev_addr, 55);

    ESP_LOGI(TAG, "VL53L1X Init Successfully");
    return ESP_OK;
}

esp_err_t vl53l1x_get_single_distance(uint16_t dev_addr, uint16_t *distance) {
    uint8_t dataReady = 0;
    int8_t status = 0;

    status = VL53L1X_StartRanging(dev_addr);
    if (status != 0) return ESP_FAIL;

    int timeout = 100;
    while (dataReady == 0 && timeout > 0) {
        VL53L1X_CheckForDataReady(dev_addr, &dataReady);
        vTaskDelay(pdMS_TO_TICKS(2));
        timeout--;
    }

    if (timeout == 0) {
        VL53L1X_StopRanging(dev_addr);
        return ESP_ERR_TIMEOUT;
    }

    VL53L1X_GetDistance(dev_addr, distance);
    VL53L1X_ClearInterrupt(dev_addr);
    VL53L1X_StopRanging(dev_addr);

    return ESP_OK;
}
