#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include "led.h"
#include "key.h"
#include "uart.h"





/**
 * @brief 定时发送任务：1Hz 速率发送 Hello World
 */
void uart_tx_task(void *pvParameters)
{
    while (1) {
        const char *msg = "Hello World\r\n";
        uart_write_bytes(USART_UX, msg, strlen(msg));
        
        // 1Hz 速率，即每 1000 毫秒一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief 接收处理任务：监听用户输入，按下回车后输出特定字符串
 */
void uart_rx_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE);
    
    while (1) {
        // 读取串口数据，设置 20ms 等待超时
        int len = uart_read_bytes(USART_UX, data, RX_BUF_SIZE - 1, pdMS_TO_TICKS(20));
        
        if (len > 0) {
            data[len] = '\0'; // 字符串终止符
            
            // 检查接收到的数据中是否包含回车符 (\r 或 \n)
            if (strchr((char *)data, '\r') || strchr((char *)data, '\n')) {
                const char *resp = "GEL37KXHDU9G\r\nFXLKNKWHVURC\r\nCE4K7KEYCUPQ\r\n";
                uart_write_bytes(USART_UX, resp, strlen(resp));
            }
        }
        // 给系统一点喘息时间，防止看门狗复位
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}







/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;
    uint8_t key = 0;
    uint32_t len = 0;
    uint16_t times = 0;
    unsigned char data[RX_BUF_SIZE] = {0};

    ret = nvs_flash_init();     /* 初始化NVS */

    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ESP_ERROR_CHECK(nvs_flash_init());
    // }

    led_init();                 /* 初始化LED */
    key_init();                 /* 初始化KEY */
    usart_init(115200);        /* 初始化UART，波特率为115200 */      




// 4. 创建并行任务
    // 发送任务：优先级 5
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 5, NULL);
    // 接收任务：优先级 5（稍微高一点或相同即可）
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);




    // while(1)
    // {



    // //     key = key_scan(0);
    // //     if (key)
    // //     {
    // //         if (key == BOOT_PRES)
    // //         {
    // //             LED0_TOGGLE();
    // //         }
    // //     }
    // //     vTaskDelay(pdMS_TO_TICKS(10));
    // // }



    // //  uart_get_buffered_data_len(USART_UX, (size_t*)&len);  /* 获取UART接收缓冲区中的数据长度 */
    // //  if(len>0)
    // //  {
    // //     memset(data, 0, RX_BUF_SIZE);  /* 清空数据缓冲区 */
    // //     printf("你发送的数据是:\n");
    // //     uart_read_bytes(USART_UX, data, len, pdMS_TO_TICKS(100));  /* 从UART接收缓冲区读取数据 */
    // //     uart_write_bytes(USART_UX, (const char *)data, strlen((const char *)data));  /* 将接收到的数据原样发送回去 */  
    // //  }else{
    // //     times++;
    // //     if(times%100==0)
    // //     {
    // //         printf("没有接收到数据\n");
    // //         printf("len=%ld\n", len);
    // //         LED0_TOGGLE();  /* LED0翻转 */
    // //     }
    // //  }
    // //  vTaskDelay(pdMS_TO_TICKS(10));



    // }







    }