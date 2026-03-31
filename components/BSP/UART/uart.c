#include "uart.h"
  


void usart_init(uint32_t baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(USART_UX, &uart_config);
    uart_set_pin(USART_UX, UART_TX_GPIO_PIN, UART_RX_GPIO_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(USART_UX, RX_BUF_SIZE * 2,RX_BUF_SIZE * 2, 0, NULL, 0);
     

    // ESP_ERROR_CHECK(uart_param_config(USART_UX, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(USART_UX, UART_TX_GPIO_PIN, UART_RX_GPIO_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_driver_install(USART_UX, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}