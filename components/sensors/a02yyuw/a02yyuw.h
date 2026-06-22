#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#include "esp_err.h"
#include "sw_uart.h"
#else
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_CRC 0x109
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_STATE 0x103
typedef int uart_port_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define A02YYUW_DEFAULT_BAUDRATE 9600
#define A02YYUW_DEFAULT_RX_BUF_SIZE 512
#define A02YYUW_MIN_DISTANCE_MM 30
#define A02YYUW_MAX_DISTANCE_MM 4500

typedef struct {
    uart_port_t uart_port;
    int tx_gpio;
    int rx_gpio;
    int baudrate;
    int rx_buffer_size;
    bool use_sw_uart;
} a02yyuw_config_t;

typedef struct {
    int distance_mm;
    bool valid;
} a02yyuw_reading_t;

a02yyuw_config_t a02yyuw_default_config(uart_port_t uart_port,
                                        int rx_gpio,
                                        int tx_gpio);
bool a02yyuw_parse_frame(const uint8_t *frame,
                         size_t len,
                         a02yyuw_reading_t *out);
bool a02yyuw_parse_latest(const uint8_t *buf,
                          size_t len,
                          a02yyuw_reading_t *out);
esp_err_t a02yyuw_init(const a02yyuw_config_t *config);
esp_err_t a02yyuw_read(a02yyuw_reading_t *out, uint32_t wait_ms);
void a02yyuw_deinit(void);

#ifdef __cplusplus
}
#endif
