#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "sw_i2c.h"

#define IMU_I2C_DEFAULT_ADDR 0x23

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint32_t scl_speed_hz;
    uint8_t device_address;
} imu_i2c_config_t;

typedef struct {
    float accel_g[3];
    float gyro_rad_s[3];
    float mag_ut[3];
    float quat[4];
    float euler_deg[3];
    float baro[4];
    bool valid;
} imu_i2c_reading_t;

typedef struct {
    imu_i2c_config_t config;
    sw_i2c_t *i2c;
    bool owns_i2c;
    bool initialized;
} imu_i2c_t;

imu_i2c_config_t imu_i2c_default_config(void);
esp_err_t imu_i2c_init(imu_i2c_t *imu, const imu_i2c_config_t *config,
                       sw_i2c_t *external_i2c);
void imu_i2c_deinit(imu_i2c_t *imu);
esp_err_t imu_i2c_read_version(imu_i2c_t *imu, uint8_t version[3]);
esp_err_t imu_i2c_read_all(imu_i2c_t *imu, imu_i2c_reading_t *out);
bool imu_i2c_probe(imu_i2c_t *imu);
