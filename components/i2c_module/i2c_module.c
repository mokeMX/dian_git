#include "i2c_module.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2c_module";

extern i2c_master_bus_handle_t bus_handle;

esp_err_t i2c_module_init(void)
{
    ESP_LOGI(TAG, "I2C bus already initialized by BSP/MYIIC");
    return ESP_OK;
}

esp_err_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t *buffer = (uint8_t *)malloc(len + 1);
    if (buffer == NULL) return ESP_ERR_NO_MEM;
    
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    
    esp_err_t err = i2c_master_write_to_device(bus_handle, addr, buffer, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    free(buffer);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    esp_err_t err = i2c_master_write_read_device(bus_handle, addr, &reg, 1, buf, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
    }
    return err;
}
