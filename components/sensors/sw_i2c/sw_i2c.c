#include "sw_i2c.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

#define SW_I2C_TIMEOUT_US 5000

static void sda_high(sw_i2c_t *bus) {
    gpio_set_level(bus->cfg.sda_gpio, 1);
    gpio_set_direction(bus->cfg.sda_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
}

static void sda_low(sw_i2c_t *bus) {
    gpio_set_direction(bus->cfg.sda_gpio, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(bus->cfg.sda_gpio, 0);
}

static void scl_high(sw_i2c_t *bus) {
    gpio_set_level(bus->cfg.scl_gpio, 1);
}

static void scl_low(sw_i2c_t *bus) {
    gpio_set_level(bus->cfg.scl_gpio, 0);
}

static int sda_read(sw_i2c_t *bus) {
    return gpio_get_level(bus->cfg.sda_gpio);
}

static void i2c_delay_us(sw_i2c_t *bus, uint32_t us) {
    esp_rom_delay_us(us);
}

static void i2c_start(sw_i2c_t *bus) {
    sda_high(bus);
    i2c_delay_us(bus, bus->half_period_us);
    scl_high(bus);
    i2c_delay_us(bus, bus->half_period_us);
    sda_low(bus);
    i2c_delay_us(bus, bus->half_period_us);
    scl_low(bus);
    i2c_delay_us(bus, bus->half_period_us);
}

static void i2c_stop(sw_i2c_t *bus) {
    sda_low(bus);
    i2c_delay_us(bus, bus->half_period_us);
    scl_high(bus);
    i2c_delay_us(bus, bus->half_period_us);
    sda_high(bus);
    i2c_delay_us(bus, bus->half_period_us);
}

static bool i2c_write_byte(sw_i2c_t *bus, uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) {
            sda_high(bus);
        } else {
            sda_low(bus);
        }
        i2c_delay_us(bus, bus->half_period_us);
        scl_high(bus);
        i2c_delay_us(bus, bus->half_period_us);
        scl_low(bus);
        i2c_delay_us(bus, bus->half_period_us);
    }

    sda_high(bus);
    i2c_delay_us(bus, bus->half_period_us);
    scl_high(bus);

    int timeout = 0;
    while (sda_read(bus) == 1 && timeout < SW_I2C_TIMEOUT_US) {
        esp_rom_delay_us(1);
        timeout++;
    }
    bool ack = (sda_read(bus) == 0);

    scl_low(bus);
    i2c_delay_us(bus, bus->half_period_us);
    return ack;
}

static uint8_t i2c_read_byte(sw_i2c_t *bus, bool ack) {
    sda_high(bus);
    uint8_t data = 0;

    for (int i = 7; i >= 0; i--) {
        i2c_delay_us(bus, bus->half_period_us);
        scl_high(bus);
        i2c_delay_us(bus, bus->half_period_us);
        if (sda_read(bus)) {
            data |= (1 << i);
        }
        scl_low(bus);
    }

    if (ack) {
        sda_low(bus);
    } else {
        sda_high(bus);
    }
    i2c_delay_us(bus, bus->half_period_us);
    scl_high(bus);
    i2c_delay_us(bus, bus->half_period_us);
    scl_low(bus);
    i2c_delay_us(bus, bus->half_period_us);
    sda_high(bus);

    return data;
}

sw_i2c_config_t sw_i2c_default_config(gpio_num_t sda, gpio_num_t scl) {
    sw_i2c_config_t cfg = {
        .sda_gpio = sda,
        .scl_gpio = scl,
        .clk_speed_hz = 100000,
        .device_address = 0x23,
    };
    return cfg;
}

esp_err_t sw_i2c_init(sw_i2c_t *bus, const sw_i2c_config_t *config) {
    if (bus == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bus->cfg = *config;
    if (config->clk_speed_hz > 0) {
        bus->half_period_us = 500000 / config->clk_speed_hz;
    } else {
        bus->half_period_us = 5;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->sda_gpio) | (1ULL << config->scl_gpio),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    sda_high(bus);
    scl_high(bus);
    bus->initialized = true;
    return ESP_OK;
}

void sw_i2c_deinit(sw_i2c_t *bus) {
    if (bus == NULL || !bus->initialized) {
        return;
    }
    gpio_reset_pin(bus->cfg.sda_gpio);
    gpio_reset_pin(bus->cfg.scl_gpio);
    bus->initialized = false;
}

bool sw_i2c_probe(sw_i2c_t *bus, uint8_t addr) {
    if (bus == NULL || !bus->initialized) {
        return false;
    }

    i2c_start(bus);
    bool ack = i2c_write_byte(bus, (uint8_t)(addr << 1));
    i2c_stop(bus);
    return ack;
}

esp_err_t sw_i2c_write_reg(sw_i2c_t *bus, uint8_t reg, const uint8_t *data, size_t len) {
    if (bus == NULL || !bus->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_start(bus);
    if (!i2c_write_byte(bus, (uint8_t)(bus->cfg.device_address << 1))) {
        i2c_stop(bus);
        return ESP_ERR_TIMEOUT;
    }
    i2c_write_byte(bus, reg);

    for (size_t i = 0; i < len; i++) {
        i2c_write_byte(bus, data[i]);
    }

    i2c_stop(bus);
    return ESP_OK;
}

esp_err_t sw_i2c_read_reg(sw_i2c_t *bus, uint8_t reg, uint8_t *data, size_t len) {
    if (bus == NULL || !bus->initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len == 0) {
        return ESP_OK;
    }

    i2c_start(bus);
    if (!i2c_write_byte(bus, (uint8_t)(bus->cfg.device_address << 1))) {
        i2c_stop(bus);
        return ESP_ERR_TIMEOUT;
    }
    i2c_write_byte(bus, reg);

    i2c_start(bus);
    if (!i2c_write_byte(bus, (uint8_t)((bus->cfg.device_address << 1) | 1))) {
        i2c_stop(bus);
        return ESP_ERR_TIMEOUT;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(bus, i < len - 1);
    }

    i2c_stop(bus);
    return ESP_OK;
}
