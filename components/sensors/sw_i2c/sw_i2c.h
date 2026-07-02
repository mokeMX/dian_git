#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint32_t clk_speed_hz;
    uint8_t device_address;
} sw_i2c_config_t;

typedef struct {
    sw_i2c_config_t cfg;
    uint32_t half_period_us;
    bool initialized;
} sw_i2c_t;

sw_i2c_config_t sw_i2c_default_config(gpio_num_t sda, gpio_num_t scl);

esp_err_t sw_i2c_init(sw_i2c_t *bus, const sw_i2c_config_t *config);
void sw_i2c_deinit(sw_i2c_t *bus);

esp_err_t sw_i2c_write_reg(sw_i2c_t *bus, uint8_t reg, const uint8_t *data, size_t len);
esp_err_t sw_i2c_read_reg(sw_i2c_t *bus, uint8_t reg, uint8_t *data, size_t len);
bool sw_i2c_probe(sw_i2c_t *bus, uint8_t addr);
