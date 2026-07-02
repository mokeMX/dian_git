#include "imu_i2c.h"

#include <stdlib.h>
#include <string.h>

#define IMU_FUNC_VERSION 0x01
#define IMU_FUNC_RAW_ACCEL 0x04
#define IMU_FUNC_RAW_GYRO 0x0A
#define IMU_FUNC_RAW_MAG 0x10
#define IMU_FUNC_QUAT 0x16
#define IMU_FUNC_EULER 0x26
#define IMU_FUNC_BARO 0x32

imu_i2c_config_t imu_i2c_default_config(void)
{
    imu_i2c_config_t config = {
        .sda_gpio = GPIO_NUM_11,
        .scl_gpio = GPIO_NUM_12,
        .scl_speed_hz = 100000,
        .device_address = IMU_I2C_DEFAULT_ADDR,
    };
    return config;
}

static int16_t le_i16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

static float le_float(const uint8_t *bytes)
{
    float value = 0.0f;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

static esp_err_t read_reg(imu_i2c_t *imu, uint8_t reg, uint8_t *buf, size_t len)
{
    return sw_i2c_read_reg(imu->i2c, reg, buf, len);
}

esp_err_t imu_i2c_init(imu_i2c_t *imu, const imu_i2c_config_t *config,
                       sw_i2c_t *external_i2c)
{
    if (imu == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(imu, 0, sizeof(*imu));
    imu->config = *config;

    if (external_i2c != NULL) {
        imu->i2c = external_i2c;
        imu->owns_i2c = false;
    } else {
        imu->i2c = (sw_i2c_t *)malloc(sizeof(sw_i2c_t));
        if (imu->i2c == NULL) {
            return ESP_ERR_NO_MEM;
        }
        sw_i2c_config_t sw_cfg = sw_i2c_default_config(config->sda_gpio,
                                                        config->scl_gpio);
        sw_cfg.clk_speed_hz = config->scl_speed_hz;
        sw_cfg.device_address = config->device_address;
        esp_err_t ret = sw_i2c_init(imu->i2c, &sw_cfg);
        if (ret != ESP_OK) {
            free(imu->i2c);
            imu->i2c = NULL;
            return ret;
        }
        imu->owns_i2c = true;
    }

    imu->initialized = true;
    return ESP_OK;
}

void imu_i2c_deinit(imu_i2c_t *imu)
{
    if (imu == NULL || !imu->initialized) {
        return;
    }
    if (imu->owns_i2c && imu->i2c != NULL) {
        sw_i2c_deinit(imu->i2c);
        free(imu->i2c);
    }
    memset(imu, 0, sizeof(*imu));
}

bool imu_i2c_probe(imu_i2c_t *imu)
{
    if (imu == NULL || !imu->initialized) {
        return false;
    }
    return sw_i2c_probe(imu->i2c, imu->config.device_address);
}

esp_err_t imu_i2c_read_version(imu_i2c_t *imu, uint8_t version[3])
{
    if (imu == NULL || version == NULL || !imu->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_reg(imu, IMU_FUNC_VERSION, version, 3);
}

esp_err_t imu_i2c_read_all(imu_i2c_t *imu, imu_i2c_reading_t *out)
{
    if (imu == NULL || out == NULL || !imu->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    uint8_t buf[16] = {0};

    esp_err_t ret = read_reg(imu, IMU_FUNC_RAW_ACCEL, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    const float accel_ratio = 16.0f / 32767.0f;
    out->accel_g[0] = le_i16(&buf[0]) * accel_ratio;
    out->accel_g[1] = le_i16(&buf[2]) * accel_ratio;
    out->accel_g[2] = le_i16(&buf[4]) * accel_ratio;

    ret = read_reg(imu, IMU_FUNC_RAW_GYRO, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    const float gyro_ratio = (2000.0f / 32767.0f) * (3.1415926f / 180.0f);
    out->gyro_rad_s[0] = le_i16(&buf[0]) * gyro_ratio;
    out->gyro_rad_s[1] = le_i16(&buf[2]) * gyro_ratio;
    out->gyro_rad_s[2] = le_i16(&buf[4]) * gyro_ratio;

    ret = read_reg(imu, IMU_FUNC_RAW_MAG, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    const float mag_ratio = 800.0f / 32767.0f;
    out->mag_ut[0] = le_i16(&buf[0]) * mag_ratio;
    out->mag_ut[1] = le_i16(&buf[2]) * mag_ratio;
    out->mag_ut[2] = le_i16(&buf[4]) * mag_ratio;

    ret = read_reg(imu, IMU_FUNC_QUAT, buf, 16);
    if (ret != ESP_OK) {
        return ret;
    }
    out->quat[0] = le_float(&buf[0]);
    out->quat[1] = le_float(&buf[2]);
    out->quat[2] = le_float(&buf[8]);
    out->quat[3] = le_float(&buf[12]);

    ret = read_reg(imu, IMU_FUNC_EULER, buf, 12);
    if (ret != ESP_OK) {
        return ret;
    }
    const float rad_to_deg = 57.2957795f;
    out->euler_deg[0] = le_float(&buf[0]) * rad_to_deg;
    out->euler_deg[1] = le_float(&buf[4]) * rad_to_deg;
    out->euler_deg[2] = le_float(&buf[8]) * rad_to_deg;

    ret = read_reg(imu, IMU_FUNC_BARO, buf, 16);
    if (ret != ESP_OK) {
        return ret;
    }
    out->baro[0] = le_float(&buf[0]);
    out->baro[1] = le_float(&buf[4]);
    out->baro[2] = le_float(&buf[8]);
    out->baro[3] = le_float(&buf[12]);
    out->valid = true;
    return ESP_OK;
}
