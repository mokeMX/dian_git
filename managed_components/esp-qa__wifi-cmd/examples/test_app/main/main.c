/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_console.h"

#include "wifi_cmd.h"

/* Check definition during compile */
#if CONFIG_IDF_TARGET_ESP32 && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 0))
#if !CONFIG_ESP_WIFI_ENABLED
#error "CONFIG_ESP_WIFI_ENABLED should always be defined in header file when idf<=5.1"
#endif
#endif

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED || CONFIG_ESP_HOST_WIFI_ENABLED
    /* initialise wifi */
    wifi_cmd_initialize_wifi(NULL);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    repl_config.prompt = "test_app>";
    repl_config.max_history_len = 1;
    repl_config.task_priority = 24;
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED || CONFIG_ESP_HOST_WIFI_ENABLED
    /* Register commands */
    ESP_ERROR_CHECK(wifi_cmd_register_all());
#endif

    /* For CI: check basic-only build */
#if CONFIG_WIFI_CMD_BASIC_ONLY
#if CONFIG_WIFI_CMD_ENABLE_ITWT
#error "TEST BUILD error: itwt should be disable by basic-only config"
#endif
#else
#if CONFIG_IDF_TARGET_ESP32C6
    /* check this symbol in CI after the build */
    wifi_cmd_register_itwt();
#endif
#endif /* CONFIG_WIFI_CMD_BASIC_ONLY */

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
