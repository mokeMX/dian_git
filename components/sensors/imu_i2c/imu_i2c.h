#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "esp_err.h"
#include "sw_i2c.h"
#else
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT 0x107
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_I2C_DEFAULT_ADDR 0x23

typedef struct {
    int sda_gpio;
    int scl_gpio;
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
    sw_i2c_t sw_i2c;
    uint8_t device_address;
    bool initialized;
} imu_i2c_t;

imu_i2c_config_t imu_i2c_default_config(void);
esp_err_t imu_i2c_init(imu_i2c_t *imu, const imu_i2c_config_t *config);
void imu_i2c_deinit(imu_i2c_t *imu);
esp_err_t imu_i2c_read_version(imu_i2c_t *imu, uint8_t version[3]);
esp_err_t imu_i2c_read_all(imu_i2c_t *imu, imu_i2c_reading_t *out);

#ifdef __cplusplus
}
#endif
