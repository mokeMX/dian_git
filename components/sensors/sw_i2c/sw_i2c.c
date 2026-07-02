#include "sw_i2c.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sw_i2c";

#define SW_I2C_TIMEOUT_US 5000

static inline void sda_high(sw_i2c_t *i2c)
{
    gpio_set_level(i2c->sda_gpio, 1);
}

static inline void sda_low(sw_i2c_t *i2c)
{
    gpio_set_level(i2c->sda_gpio, 0);
}

static inline void scl_high(sw_i2c_t *i2c)
{
    gpio_set_level(i2c->scl_gpio, 1);
}

static inline void scl_low(sw_i2c_t *i2c)
{
    gpio_set_level(i2c->scl_gpio, 0);
}

static inline int sda_read(sw_i2c_t *i2c)
{
    return gpio_get_level(i2c->sda_gpio);
}

static void i2c_delay_us(sw_i2c_t *i2c, uint32_t us)
{
    esp_rom_delay_us(us);
}

static void i2c_start(sw_i2c_t *i2c)
{
    sda_high(i2c);
    scl_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);
    sda_low(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);
    scl_low(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);
}

static void i2c_stop(sw_i2c_t *i2c)
{
    sda_low(i2c);
    scl_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);
    sda_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);
}

static int i2c_write_byte(sw_i2c_t *i2c, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) {
            sda_high(i2c);
        } else {
            sda_low(i2c);
        }
        i2c_delay_us(i2c, i2c->half_period_us / 2);
        scl_high(i2c);
        i2c_delay_us(i2c, i2c->half_period_us);
        scl_low(i2c);
        i2c_delay_us(i2c, i2c->half_period_us / 2);
    }

    sda_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us / 2);
    scl_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us / 2);

    int ack = sda_read(i2c);
    i2c_delay_us(i2c, i2c->half_period_us / 2);
    scl_low(i2c);
    i2c_delay_us(i2c, i2c->half_period_us / 2);

    return ack;
}

static uint8_t i2c_read_byte(sw_i2c_t *i2c, int send_ack)
{
    uint8_t byte = 0;

    sda_high(i2c);
    for (int i = 7; i >= 0; i--) {
        scl_high(i2c);
        i2c_delay_us(i2c, i2c->half_period_us / 2);
        if (sda_read(i2c)) {
            byte |= (1 << i);
        }
        i2c_delay_us(i2c, i2c->half_period_us / 2);
        scl_low(i2c);
        i2c_delay_us(i2c, i2c->half_period_us);
    }

    if (send_ack) {
        sda_low(i2c);
    } else {
        sda_high(i2c);
    }
    i2c_delay_us(i2c, i2c->half_period_us / 2);
    scl_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);
    scl_low(i2c);
    i2c_delay_us(i2c, i2c->half_period_us / 2);
    sda_high(i2c);

    return byte;
}

esp_err_t sw_i2c_init(sw_i2c_t *i2c, int sda_gpio, int scl_gpio, uint32_t speed_hz)
{
    if (i2c == NULL || sda_gpio < 0 || scl_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(i2c, 0, sizeof(*i2c));
    i2c->sda_gpio = sda_gpio;
    i2c->scl_gpio = scl_gpio;
    i2c->speed_hz = (speed_hz > 0) ? speed_hz : SW_I2C_DEFAULT_SPEED_HZ;

    if (i2c->speed_hz > 400000) {
        i2c->speed_hz = 400000;
    }

    i2c->half_period_us = 500000 / i2c->speed_hz;
    if (i2c->half_period_us < 1) {
        i2c->half_period_us = 1;
    }

    gpio_config_t sda_cfg = {
        .pin_bit_mask = (1ULL << i2c->sda_gpio),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&sda_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    gpio_config_t scl_cfg = {
        .pin_bit_mask = (1ULL << i2c->scl_gpio),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&scl_cfg);
    if (ret != ESP_OK) {
        gpio_reset_pin(i2c->sda_gpio);
        return ret;
    }

    sda_high(i2c);
    scl_high(i2c);
    i2c_delay_us(i2c, i2c->half_period_us);

    i2c->initialized = true;
    ESP_LOGI(TAG, "SW-I2C SDA=GPIO%d SCL=GPIO%d speed=%lu Hz",
             i2c->sda_gpio, i2c->scl_gpio, i2c->speed_hz);
    return ESP_OK;
}

void sw_i2c_deinit(sw_i2c_t *i2c)
{
    if (i2c == NULL || !i2c->initialized) {
        return;
    }
    gpio_reset_pin(i2c->sda_gpio);
    gpio_reset_pin(i2c->scl_gpio);
    i2c->initialized = false;
}

esp_err_t sw_i2c_write(sw_i2c_t *i2c, uint8_t addr_7bit, const uint8_t *data, size_t len)
{
    if (i2c == NULL || !i2c->initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_start(i2c);
    if (i2c_write_byte(i2c, (uint8_t)(addr_7bit << 1)) != 0) {
        i2c_stop(i2c);
        ESP_LOGW(TAG, "write: NACK on addr 0x%02X", addr_7bit);
        return ESP_ERR_TIMEOUT;
    }

    for (size_t i = 0; i < len; i++) {
        if (i2c_write_byte(i2c, data[i]) != 0) {
            i2c_stop(i2c);
            ESP_LOGW(TAG, "write: NACK on data byte %d", i);
            return ESP_ERR_TIMEOUT;
        }
    }

    i2c_stop(i2c);
    return ESP_OK;
}

esp_err_t sw_i2c_read(sw_i2c_t *i2c, uint8_t addr_7bit, uint8_t *data, size_t len)
{
    if (i2c == NULL || !i2c->initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_start(i2c);
    if (i2c_write_byte(i2c, (uint8_t)((addr_7bit << 1) | 1)) != 0) {
        i2c_stop(i2c);
        ESP_LOGW(TAG, "read: NACK on addr 0x%02X", addr_7bit);
        return ESP_ERR_TIMEOUT;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(i2c, (i < len - 1) ? 1 : 0);
    }

    i2c_stop(i2c);
    return ESP_OK;
}

esp_err_t sw_i2c_write_read(sw_i2c_t *i2c, uint8_t addr_7bit,
                            const uint8_t *wdata, size_t wlen,
                            uint8_t *rdata, size_t rlen)
{
    if (i2c == NULL || !i2c->initialized || wdata == NULL || rdata == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_start(i2c);
    if (i2c_write_byte(i2c, (uint8_t)(addr_7bit << 1)) != 0) {
        i2c_stop(i2c);
        ESP_LOGW(TAG, "write_read: NACK on addr 0x%02X", addr_7bit);
        return ESP_ERR_TIMEOUT;
    }

    for (size_t i = 0; i < wlen; i++) {
        if (i2c_write_byte(i2c, wdata[i]) != 0) {
            i2c_stop(i2c);
            ESP_LOGW(TAG, "write_read: NACK on write byte %d", i);
            return ESP_ERR_TIMEOUT;
        }
    }

    i2c_start(i2c);
    if (i2c_write_byte(i2c, (uint8_t)((addr_7bit << 1) | 1)) != 0) {
        i2c_stop(i2c);
        ESP_LOGW(TAG, "write_read: NACK on addr 0x%02X (read phase)", addr_7bit);
        return ESP_ERR_TIMEOUT;
    }

    for (size_t i = 0; i < rlen; i++) {
        rdata[i] = i2c_read_byte(i2c, (i < rlen - 1) ? 1 : 0);
    }

    i2c_stop(i2c);
    return ESP_OK;
}
