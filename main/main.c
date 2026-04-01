/**
 * @brief       WiFi-STA + Iperf测试 + 串口动态设置IP（紧凑优化版）
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "led.h"
#include "myiic.h"
#include "my_spi.h"
#include "spilcd.h"
#include "xl9555.h"
#include <stdio.h>
#include <string.h>

#include "iperf.h"
#include "esp_netif.h"
#include "esp_console.h"
#include "iperf_cmd.h"

#define DEFAULT_SSID        "train123"
#define DEFAULT_PWD         "linlin123"
#define TEST_TIME_SEC       30

char g_server_ip[20] = "192.168.1.100";
static EventGroupHandle_t wifi_event;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
static const char *TAG = "iperf_test";
char lcd_buff[100] = {0};

#define WIFICONFIG()   {                            \
    .sta = {                                        \
        .ssid = DEFAULT_SSID,                       \
        .password = DEFAULT_PWD,                    \
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,    \
    },                                              \
}

// LCD显示WiFi状态
void connet_display(uint8_t flag)
{
    if(flag == 2) {
        spilcd_fill(0,90,320,130,WHITE);
        sprintf(lcd_buff, "ssid:%s",DEFAULT_SSID);
        spilcd_show_string(0,90,240,16,16,lcd_buff,BLUE);
        sprintf(lcd_buff, "psw:%s",DEFAULT_PWD);
        spilcd_show_string(0,110,240,16,16,lcd_buff,BLUE);
    } else if (flag == 1) {
        spilcd_show_string(0,90,240,16,16,"wifi connecting fail",BLUE);
    } else {
        spilcd_show_string(0,90,240,16,16,"wifi connecting......",BLUE);
    }
}

// 启动iperf客户端
void start_iperf_client(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Starting Iperf Client... %s", g_server_ip);

    iperf_cfg_t cfg;
    memset(&cfg, 0, sizeof(iperf_cfg_t));
    cfg.flag = IPERF_FLAG_CLIENT | IPERF_FLAG_TCP;
    cfg.type = IPERF_TCP_CLIENT;
    cfg.destination_ip4 = esp_ip4addr_aton(g_server_ip);

    if (cfg.destination_ip4 == 0) {
        ESP_LOGE(TAG, "IP convert error");
        vTaskDelete(NULL);
        return;
    }

    cfg.sport = IPERF_DEFAULT_PORT;
    cfg.dport = IPERF_DEFAULT_PORT;
    cfg.interval = 3;
    cfg.time = TEST_TIME_SEC;

    esp_err_t ret = iperf_start(&cfg);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Iperf start fail: %s", esp_err_to_name(ret));

    vTaskDelete(NULL);
}

// setip命令处理
static int set_ip_cmd_handler(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: setip 192.168.x.x\n");
        return 1;
    }

    strncpy(g_server_ip, argv[1], sizeof(g_server_ip)-1);
    g_server_ip[sizeof(g_server_ip)-1] = '\0';
    printf("Success! Target IP: %s\n", g_server_ip);

    spilcd_fill(0,160,320,180,WHITE);
    sprintf(lcd_buff, "Target IP: %s", g_server_ip);
    spilcd_show_string(0,160,240,16,16,lcd_buff,MAGENTA);
    return 0;
}

// 启动串口控制台
void start_console_test(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    // 修改这里：将 stack_size 改为 task_stack_size
    repl_config.task_stack_size = 4096 * 2;
    repl_config.prompt = "iperf>";

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    app_register_iperf_commands();

    const esp_console_cmd_t setip_cmd = {
        .command = "setip",
        .help = "Set target PC IP",
        .hint = "<ip_address>",
        .func = set_ip_cmd_handler,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&setip_cmd));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

// WIFI事件回调
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t evt, void *data)
{
    static int retry = 0;

    if (base == WIFI_EVENT) {
        switch (evt) {
            case WIFI_EVENT_STA_START:
                connet_display(0); esp_wifi_connect(); break;
            case WIFI_EVENT_STA_CONNECTED:
                connet_display(2); break;
            case WIFI_EVENT_STA_DISCONNECTED:
                if (retry < 20) { esp_wifi_connect(); retry++; }
                else xEventGroupSetBits(wifi_event, WIFI_FAIL_BIT);
                break;
        }
    } else if (base == IP_EVENT && evt == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry = 0;

        spilcd_fill(0,130,320,150,WHITE);
        sprintf(lcd_buff, "Local IP: " IPSTR, IP2STR(&event->ip_info.ip));
        spilcd_show_string(0,130,240,16,16,lcd_buff,RED);
        sprintf(lcd_buff, "Target IP: %s", g_server_ip);
        spilcd_show_string(0,160,240,16,16,lcd_buff,MAGENTA);

        xEventGroupSetBits(wifi_event, WIFI_CONNECTED_BIT);
    }
}

// WIFI初始化
void wifi_sta_init(void)
{
    wifi_event = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_config = WIFICONFIG();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    EventBits_t bits = xEventGroupWaitBits(wifi_event, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) ESP_LOGI(TAG, "Connected to AP");
}

// 主函数
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_init();
    my_spi_init();
    myiic_init();
    xl9555_init();
    spilcd_init();

    spilcd_show_string(0,0,240,32,32,"ESP32-S3",RED);
    spilcd_show_string(0,40,240,24,24,"Iperf UART Test",RED);
    spilcd_show_string(0,70,240,16,16,"ATOM@ALIENTEK",RED);

    wifi_sta_init();
    start_console_test();

    uint32_t led_tick = 0;
    while (1) {
        if (++led_tick >= 50) {
            LED0_TOGGLE();
            led_tick = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}