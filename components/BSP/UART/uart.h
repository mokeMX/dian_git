#ifndef __UART_H
#define __UART_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include <string.h>
#include "driver/uart_select.h"
#include "driver/gpio.h"


#define USART_UX UART_NUM_0
#define UART_TX_GPIO_PIN GPIO_NUM_43
#define UART_RX_GPIO_PIN GPIO_NUM_44

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024


void usart_init(uint32_t baud_rate);


#endif