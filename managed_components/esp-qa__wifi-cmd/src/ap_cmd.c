/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "wifi_cmd.h"

#ifndef APP_TAG
#define APP_TAG "WIFI"
#endif

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_str *authmode;
    struct arg_int *channel;
    struct arg_int *max_conn;
    struct arg_end *end;
} ap_set_args_t;
static ap_set_args_t ap_set_args;

static int cmd_do_ap_set(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &ap_set_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, ap_set_args.end, argv[0]);
        return 1;
    }

    wifi_config_t wifi_config = {};

    const char *ssid = ap_set_args.ssid->sval[0];
    if (strlen(ssid) >= sizeof(wifi_config.ap.ssid)) {
        memcpy((char *) wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    } else {
        strlcpy((char *) wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    }
    const char *pass = ap_set_args.password->sval[0];
    if (ap_set_args.password->count > 0) {
        if (strlen(pass) >= sizeof(wifi_config.ap.password)) {
            memcpy((char *) wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
        } else {
            strlcpy((char *) wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
        }
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK; // set default auth mode
    }
    if (ap_set_args.channel->count > 0) {
        wifi_config.ap.channel = (uint8_t)(ap_set_args.channel->ival[0]);
    }
    const char *auth_str = ap_set_args.authmode->sval[0];
    if (ap_set_args.authmode->count > 0) {
        wifi_config.ap.authmode = wifi_cmd_auth_mode_str2num(auth_str);
    }
    if (ap_set_args.max_conn->count > 0) {
        wifi_config.ap.max_connection = ap_set_args.max_conn->ival[0];
    } else {
        wifi_config.ap.max_connection = 2;
    }

    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    LOG_WIFI_CMD_DONE(err, "SET_AP_CONFIG");
    return 0;
}

#if CONFIG_WIFI_CMD_ENABLE_DEPRECATED
static int cmd_do_ap_deprecated(int argc, char **argv)
{
    ESP_LOGW(APP_TAG, "'ap' is deprecated, please use 'ap_set'.");
    return cmd_do_ap_set(argc, argv);
}
#endif

static void app_register_ap_set(void)
{
    ap_set_args.ssid = arg_str1(NULL, NULL, "<ssid>", "SSID of AP");
    ap_set_args.password = arg_str0(NULL, NULL, "<pass>", "password of AP");
    ap_set_args.authmode = arg_str0("a", "authmode", "<authmode>", "wifi auth type (ie. open | wep| wpa2 | wpa2_enterprise)");
    ap_set_args.channel = arg_int0("n", "channel", "<channel>", "channel of AP");
    ap_set_args.max_conn = arg_int0("m", "max_conn", "<max_conn>", "Max station number, default: 2");
    ap_set_args.end = arg_end(2);
    const esp_console_cmd_t ap_set_cmd = {
        .command = "ap_set",
        .help = "WiFi is ap mode, set ap config.",
        .hint = NULL,
        .func = &cmd_do_ap_set,
        .argtable = &ap_set_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ap_set_cmd));

#if CONFIG_WIFI_CMD_ENABLE_DEPRECATED
    esp_console_cmd_t ap_deprecated = ap_set_cmd;
    ap_deprecated.command = "ap";
    ap_deprecated.help = "Set ap config. Deprecated, please use 'ap_set'";
    ap_deprecated.func = &cmd_do_ap_deprecated;
    ESP_ERROR_CHECK(esp_console_cmd_register(&ap_deprecated));
#endif
}

static int cmd_do_ap_query(int argc, char **argv)
{
    wifi_mode_t mode = 0;
    wifi_config_t w_config = {};
    esp_wifi_get_mode(&mode);
    if (mode & WIFI_MODE_AP) {
        ESP_LOGI(APP_TAG, "wifi ap started.");
        esp_wifi_get_config(WIFI_IF_AP, &w_config);
        char tmp[65] = "";
        strncpy(tmp, (char *)w_config.ap.ssid, 32);
        ESP_LOGI(APP_TAG, "ap config ssid: %s", tmp);
        memset(tmp, 0, sizeof(tmp));
        strncpy(tmp, (char *)w_config.ap.password, 64);
        ESP_LOGI(APP_TAG, "ap config password: %s", tmp);
        ESP_LOGI(APP_TAG, "ap config channel: %d",  w_config.ap.channel);
    } else {
        ESP_LOGI(APP_TAG, "wifi ap stopped.");
    }
    return 0;
}

static void app_register_ap_query(void)
{
    const esp_console_cmd_t wifi_connect_cmd = {
        .command = "ap_query",
        .help = "WiFi is station mode, stop wifi connect.",
        .hint = NULL,
        .func = &cmd_do_ap_query,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_connect_cmd));
}

void wifi_cmd_register_ap_basic(void)
{
    app_register_ap_set();
    app_register_ap_query();
}
