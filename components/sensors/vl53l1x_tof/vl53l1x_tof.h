#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "esp_err.h"
#include "sw_i2c.h"
#else
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT 0x107
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L1X_TOF_DEFAULT_ADDR_8BIT 0x52
#define VL53L1X_TOF_DEFAULT_ADDR_7BIT 0x29

typedef enum {
    VL53L1X_TOF_DISTANCE_SHORT = 1,
    VL53L1X_TOF_DISTANCE_LONG = 2,
} vl53l1x_tof_distance_mode_t;

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint32_t scl_speed_hz;
    uint16_t device_address_8bit;
    uint16_t timing_budget_ms;
    uint16_t inter_measurement_ms;
    vl53l1x_tof_distance_mode_t distance_mode;
} vl53l1x_tof_config_t;

typedef struct {
    uint16_t distance_mm;
    bool valid;
} vl53l1x_tof_reading_t;

typedef struct {
    vl53l1x_tof_config_t config;
    sw_i2c_t sw_i2c;
    uint8_t device_address_7bit;
    bool initialized;
} vl53l1x_tof_t;

vl53l1x_tof_config_t vl53l1x_tof_default_config(void);
esp_err_t vl53l1x_tof_init(vl53l1x_tof_t *sensor,
                           const vl53l1x_tof_config_t *config);
esp_err_t vl53l1x_tof_read(vl53l1x_tof_t *sensor,
                           vl53l1x_tof_reading_t *out,
                           uint32_t timeout_ms);
void vl53l1x_tof_deinit(vl53l1x_tof_t *sensor);

#ifdef __cplusplus
}
#endif
