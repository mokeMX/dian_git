#ifndef _APP_VL53L1X_H_
#define _APP_VL53L1X_H_

#include <stdint.h>
#include "esp_err.h"

#define VL53L1X_DEFAULT_DEV_ADDR    0x52

esp_err_t vl53l1x_app_init(uint16_t dev_addr);

esp_err_t vl53l1x_get_single_distance(uint16_t dev_addr, uint16_t *distance);

#endif
