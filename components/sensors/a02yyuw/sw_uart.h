#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "esp_err.h"
#include "esp_timer.h"
#else
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NO_MEM 0x101
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define SW_UART_MAX_RX_BUF 1024

typedef struct {
    int rx_gpio;
    int tx_gpio;
    int baudrate;
    int rx_buffer_size;
} sw_uart_config_t;

typedef enum {
    SW_UART_IDLE,
    SW_UART_START,
    SW_UART_DATA,
    SW_UART_STOP
} sw_uart_state_t;

typedef struct sw_uart_inst {
    int rx_gpio;
    int tx_gpio;
    int baudrate;
    uint8_t rx_buf[SW_UART_MAX_RX_BUF];
    volatile int rx_head;
    volatile int rx_tail;
    volatile sw_uart_state_t state;
    volatile uint8_t current_byte;
    volatile int bit_count;
    esp_timer_handle_t timer;
    uint32_t bit_us;
    uint32_t half_bit_us;
    bool initialized;
} sw_uart_t;

sw_uart_config_t sw_uart_default_config(int rx_gpio, int tx_gpio);
esp_err_t sw_uart_init(sw_uart_t *uart, const sw_uart_config_t *config);
int sw_uart_read_bytes(sw_uart_t *uart, uint8_t *buf, int max_len, uint32_t timeout_ms);
void sw_uart_flush(sw_uart_t *uart);
void sw_uart_deinit(sw_uart_t *uart);

#ifdef __cplusplus
}
#endif
