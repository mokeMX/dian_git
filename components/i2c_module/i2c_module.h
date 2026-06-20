#ifndef __I2C_MODULE_H__
#define __I2C_MODULE_H__

#include "esp_err.h"
#include <stdint.h>

#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t i2c_module_init(void);
esp_err_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
esp_err_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
