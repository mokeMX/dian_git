#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern i2c_master_bus_handle_t bus_handle;

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t *buffer = (uint8_t *)malloc(count + 2);
    if (buffer == NULL) return -1;
    
    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    memcpy(&buffer[2], pdata, count);
    
    esp_err_t err = i2c_master_write_to_device(bus_handle, dev >> 1, buffer, count + 2, pdMS_TO_TICKS(100));
    free(buffer);
    
    return (err == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
    uint8_t reg[2];
    reg[0] = (uint8_t)(index >> 8);
    reg[1] = (uint8_t)(index & 0xFF);
    
    esp_err_t err = i2c_master_write_read_device(bus_handle, dev >> 1, reg, 2, pdata, count, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFF);
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)((data >> 16) & 0xFF);
    buf[2] = (uint8_t)((data >> 8) & 0xFF);
    buf[3] = (uint8_t)(data & 0xFF);
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t buf[2];
    int8_t status = VL53L1_ReadMulti(dev, index, buf, 2);
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t buf[4];
    int8_t status = VL53L1_ReadMulti(dev, index, buf, 4);
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return 0;
}
