#include "sw_uart.h"

#include <string.h>   /* memset */

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sw_uart";

static void sw_uart_schedule_next(sw_uart_t *uart, uint32_t delay_us);

static void sw_uart_timer_cb(void *arg)
{
    sw_uart_t *uart = (sw_uart_t *)arg;
    int level = gpio_get_level(uart->rx_gpio);

    switch (uart->state) {
    case SW_UART_START:
        if (level == 0) {
            uart->current_byte = 0;
            uart->bit_count = 0;
            uart->state = SW_UART_DATA;
            /* From the centre of the start bit, the centre of data bit 0 is
             * one full bit period away (not a half bit). */
            sw_uart_schedule_next(uart, uart->bit_us);
        } else {
            uart->state = SW_UART_IDLE;
            gpio_intr_enable(uart->rx_gpio);
        }
        break;

    case SW_UART_DATA:
        uart->current_byte >>= 1;
        if (level) {
            uart->current_byte |= 0x80;
        }
        uart->bit_count++;
        if (uart->bit_count >= 8) {
            uart->state = SW_UART_STOP;
        }
        /* Each subsequent sample is one full bit period later so we always
         * sample at the centre of each bit. */
        sw_uart_schedule_next(uart, uart->bit_us);
        break;

    case SW_UART_STOP:
        if (level) {
            int next_head = uart->rx_head + 1;
            if (next_head >= SW_UART_MAX_RX_BUF) {
                next_head = 0;
            }
            if (next_head != uart->rx_tail) {
                uart->rx_buf[uart->rx_head] = uart->current_byte;
                uart->rx_head = next_head;
            }
        }
        uart->state = SW_UART_IDLE;
        gpio_intr_enable(uart->rx_gpio);
        break;

    default:
        uart->state = SW_UART_IDLE;
        gpio_intr_enable(uart->rx_gpio);
        break;
    }
}

static void sw_uart_schedule_next(sw_uart_t *uart, uint32_t delay_us)
{
    /* The one-shot timer has already fired (we run from its callback), so it
     * is no longer armed and can simply be started again. */
    esp_timer_start_once(uart->timer, delay_us);
}

static void IRAM_ATTR sw_uart_gpio_isr(void *arg)
{
    sw_uart_t *uart = (sw_uart_t *)arg;
    int level = gpio_get_level(uart->rx_gpio);
    if (level != 0) {
        return;
    }

    gpio_intr_disable(uart->rx_gpio);

    uart->state = SW_UART_START;
    esp_timer_start_once(uart->timer, uart->half_bit_us);
}

sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio)
{
    sw_uart_config_t config = {
        .rx_gpio = rx_gpio,
        .tx_gpio = tx_gpio,
        .baudrate = 9600,
        .rx_buffer_size = 256,
    };
    return config;
}

esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config)
{
    if (uart == NULL || config == NULL || config->rx_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(uart, 0, sizeof(*uart));
    uart->rx_gpio = config->rx_gpio;
    uart->tx_gpio = config->tx_gpio;
    uart->baudrate = config->baudrate > 0 ? config->baudrate : 9600;
    uart->state = SW_UART_IDLE;
    uart->bit_us = 1000000 / uart->baudrate;
    if (uart->bit_us < 1) {
        uart->bit_us = 1;
    }
    uart->half_bit_us = uart->bit_us / 2;
    if (uart->half_bit_us < 1) {
        uart->half_bit_us = 1;
    }

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << uart->rx_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = sw_uart_timer_cb,
        .arg = uart,
        .name = "sw_uart_timer",
    };
    ret = esp_timer_create(&timer_args, &uart->timer);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        esp_timer_delete(uart->timer);
        return ret;
    }

    ret = gpio_isr_handler_add(uart->rx_gpio, sw_uart_gpio_isr, uart);
    if (ret != ESP_OK) {
        esp_timer_delete(uart->timer);
        return ret;
    }

    uart->initialized = true;
    ESP_LOGI(TAG, "SW UART RX=GPIO%d baud=%d half_bit=%luus",
             uart->rx_gpio, uart->baudrate, (unsigned long)uart->half_bit_us);
    return ESP_OK;
}

int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len, uint32_t timeout_ms)
{
    if (uart == NULL || buf == NULL || max_len <= 0) {
        return 0;
    }

    uint32_t waited = 0;
    while (waited < timeout_ms) {
        int avail;
        int tail = uart->rx_tail;
        int head = uart->rx_head;
        if (head >= tail) {
            avail = head - tail;
        } else {
            avail = SW_UART_MAX_RX_BUF - tail + head;
        }

        if (avail > 0) {
            int to_read = avail < max_len ? avail : max_len;
            for (int i = 0; i < to_read; i++) {
                buf[i] = uart->rx_buf[uart->rx_tail];
                int next_tail = uart->rx_tail + 1;
                if (next_tail >= SW_UART_MAX_RX_BUF) {
                    next_tail = 0;
                }
                uart->rx_tail = next_tail;
            }
            return to_read;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
        waited++;
    }

    return 0;
}

void sw_uart_flush(sw_uart_t *uart)
{
    if (uart == NULL) {
        return;
    }
    uart->rx_head = 0;
    uart->rx_tail = 0;
}

void sw_uart_deinit(sw_uart_t *uart)
{
    if (uart == NULL || !uart->initialized) {
        return;
    }

    gpio_isr_handler_remove(uart->rx_gpio);
    esp_timer_stop(uart->timer);
    esp_timer_delete(uart->timer);
    gpio_reset_pin(uart->rx_gpio);
    uart->initialized = false;
    ESP_LOGI(TAG, "SW UART deinitialized");
}
#else
sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio)
{
    sw_uart_config_t config = { .rx_gpio = rx_gpio, .tx_gpio = tx_gpio, .baudrate = 9600, .rx_buffer_size = 256 };
    (void)rx_gpio;
    (void)tx_gpio;
    return config;
}

esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config)
{
    (void)uart;
    (void)config;
    return ESP_ERR_INVALID_STATE;
}

int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len, uint32_t timeout_ms)
{
    (void)uart;
    (void)buf;
    (void)max_len;
    (void)timeout_ms;
    return 0;
}

void sw_uart_flush(sw_uart_t *uart)
{
    (void)uart;
}

void sw_uart_deinit(sw_uart_t *uart)
{
    (void)uart;
}
#endif
