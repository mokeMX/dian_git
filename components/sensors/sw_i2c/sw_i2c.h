#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SW_I2C_DEFAULT_SPEED_HZ 100000

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint32_t speed_hz;
    uint32_t half_period_us;
    bool initialized;
} sw_i2c_t;

esp_err_t sw_i2c_init(sw_i2c_t *i2c, int sda_gpio, int scl_gpio, uint32_t speed_hz);
void sw_i2c_deinit(sw_i2c_t *i2c);

esp_err_t sw_i2c_write(sw_i2c_t *i2c, uint8_t addr_7bit, const uint8_t *data, size_t len);
esp_err_t sw_i2c_read(sw_i2c_t *i2c, uint8_t addr_7bit, uint8_t *data, size_t len);
esp_err_t sw_i2c_write_read(sw_i2c_t *i2c, uint8_t addr_7bit,
                            const uint8_t *wdata, size_t wlen,
                            uint8_t *rdata, size_t rlen);

#ifdef __cplusplus
}
#endif
