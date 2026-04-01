/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "esp_wifi.h"
#include "esp_netif_ip_addr.h"
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "wifi_cmd.h"
#include "esp_system.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_wifi_ap_get_sta_list.h"
#else
#include "esp_netif_sta_list.h"
#define wifi_sta_mac_ip_list_t  esp_netif_sta_list_t
#define esp_wifi_ap_get_sta_list_with_ip  esp_netif_get_sta_list
#endif

#ifndef APP_TAG
#define APP_TAG "WIFI"
#endif

typedef struct {
    struct arg_str *action;
    struct arg_end *end;
} wifi_count_args_t;
static wifi_count_args_t wifi_count_args;

typedef struct {
    struct arg_str *mode;
    struct arg_end *end;
} wifi_mode_args_t;
static wifi_mode_args_t wifi_mode_args;

typedef struct {
    struct arg_int *ps_type;
    struct arg_end *end;
} wifi_ps_args_t;
static wifi_ps_args_t wifi_ps_args;

typedef struct {
    struct arg_str *code;
    struct arg_int *schan;
    struct arg_int *nchan;
    // struct arg_int *max_tx_power;
    struct arg_str *policy;
#if CONFIG_SOC_WIFI_SUPPORT_5G
    struct arg_int *chan_5g_bitmask;
    struct arg_str *chan_5g_list;
#endif
    struct arg_end *end;
} wifi_country_args_t;
static wifi_country_args_t wifi_country_args;

wifi_cmd_wifi_count_t g_wcount = {0};

static char* second_chan_t_to_str(wifi_second_chan_t second)
{
    switch (second) {
    case WIFI_SECOND_CHAN_ABOVE:
        return "40U";
    case WIFI_SECOND_CHAN_BELOW:
        return "40D";
    default:
        return "";
    }
}

static int get_cbw_from_ap_info(wifi_ap_record_t* ap_info)
{
    /* TODO: WIFIBUG-910 */
// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 1) /* IDF commit: f0365ba6b5d832a2e5999fda56b89c1f18c02858 */
#if 0
    if (ap_info->bandwidth == 2) {
        return 160;
    }
    if (ap_info->bandwidth == 1) {
        return 80;
    }
#endif /* v5.3.1 */
    if (ap_info->second == WIFI_SECOND_CHAN_ABOVE || ap_info->second == WIFI_SECOND_CHAN_BELOW) {
        return 40;
    }
    return 20;
}

void wifi_cmd_query_wifi_info(void)
{
    esp_err_t err = ESP_FAIL;
    ESP_LOGI(APP_TAG, "WIFI_QUERY:");
    /* Get wifi op mode first */
    wifi_mode_t mode = 0;
    err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "Failed to get wifi mode, err: %d", err);
        return;
    }
    ESP_LOGI(APP_TAG, "WIFI_MODE:%s", app_wifi_mode_num2str(mode));

#if CONFIG_SOC_WIFI_SUPPORT_5G || CONFIG_SOC_WIFI_HE_SUPPORT_5G
    wifi_protocols_t proto_config = {0};
    wifi_bandwidths_t cbw_config = {0};
#else
    uint32_t protocol = 0;
    wifi_bandwidth_t cbw = 0;
#endif
    uint8_t mac[6] = {0};
    wifi_config_t cfg = {0};
    esp_netif_ip_info_t ip_info = {0};
    char temp_ssid[33] = { 0 };

    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        if (g_is_sta_wifi_connected && g_is_sta_got_ip4) {
            ESP_LOGI(APP_TAG, " STA_CONNECTED!");
        } else if (g_is_sta_wifi_connected) {
            ESP_LOGI(APP_TAG, " STA_WIFI_CONNECTED_WITHOUT_IP!");
        }
        /* Get configs */
#if CONFIG_SOC_WIFI_SUPPORT_5G || CONFIG_SOC_WIFI_HE_SUPPORT_5G
        // TO compatible with the print of old apis, STA_PROTOCOL means sta protocol 2g, STA_CBW means sta cbw 2g
        ESP_ERROR_CHECK(esp_wifi_get_protocols(WIFI_IF_STA, &proto_config));
        ESP_LOGI(APP_TAG, " STA_PROTOCOL:%s", app_wifi_protocol_bitmap2str((uint32_t)proto_config.ghz_2g));
        ESP_LOGI(APP_TAG, " STA_PROTOCOL_5G:%s", app_wifi_protocol_bitmap2str((uint32_t)proto_config.ghz_5g));
        ESP_ERROR_CHECK(esp_wifi_get_bandwidths(WIFI_IF_STA, &cbw_config));
        ESP_LOGI(APP_TAG, " STA_CBW:%dMHz", cbw_config.ghz_2g == WIFI_BW_HT40 ? 40 : 20);
        ESP_LOGI(APP_TAG, " STA_CBW_5G:%dMHz", cbw_config.ghz_5g == WIFI_BW_HT40 ? 40 : 20);
#else
        ESP_ERROR_CHECK(esp_wifi_get_protocol(WIFI_IF_STA, (uint8_t *)&protocol));
        ESP_LOGI(APP_TAG, " STA_PROTOCOL:%s", app_wifi_protocol_bitmap2str(protocol));
        ESP_ERROR_CHECK(esp_wifi_get_bandwidth(WIFI_IF_STA, &cbw));
        ESP_LOGI(APP_TAG, " STA_CBW:%dMHz", cbw == WIFI_BW_HT40 ? 40 : 20);
#endif
        ESP_LOGI(APP_TAG, " STA_PMF:%d/%d", cfg.sta.pmf_cfg.capable, cfg.sta.pmf_cfg.required);
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
        ESP_LOGI(APP_TAG, " STA_MAC:"MACSTR, MAC2STR(mac));
        wifi_ap_record_t conn_ap_info = {0};
        if (g_is_sta_wifi_connected && (esp_wifi_sta_get_ap_info(&conn_ap_info) == ESP_OK)) {
            ESP_LOGI(APP_TAG, " CONN_SSID:%s", conn_ap_info.ssid); /* uint8_t ssid[33]; */
            ESP_LOGI(APP_TAG, " CONN_BSSID:"MACSTR, MAC2STR(conn_ap_info.bssid));
            ESP_LOGI(APP_TAG, " CONN_RSSI:%d", conn_ap_info.rssi);
            ESP_LOGI(APP_TAG, " CONN_CHANNEL:%d,%s", conn_ap_info.primary, second_chan_t_to_str(conn_ap_info.second));
            ESP_LOGI(APP_TAG, " CONN_AUTH:%s", wifi_cmd_auth_mode_num2str(conn_ap_info.authmode));
            ESP_LOGI(APP_TAG, " CONN_CBW:%d", get_cbw_from_ap_info(&conn_ap_info));
#if WIFI_PHY_MODE_SUPPORTED
            wifi_phy_mode_t phymode;
            esp_wifi_sta_get_negotiated_phymode(&phymode);
            ESP_LOGI(APP_TAG, " CONN_PHYMODE:%s", wifi_cmd_phy_mode_num2str(phymode));
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
            uint16_t aid = 0;
            ESP_ERROR_CHECK(esp_wifi_sta_get_aid(&aid));
            ESP_LOGI(APP_TAG, " STA_AID:%d", aid);
#endif
        }
        if (g_is_sta_got_ip4) {
            esp_netif_get_ip_info(g_netif_sta, &ip_info);
            ESP_LOGI(APP_TAG, " STA_IP:"IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(APP_TAG, " STA_IP_MASK:"IPSTR, IP2STR(&ip_info.netmask));
            ESP_LOGI(APP_TAG, " STA_IP_GW:"IPSTR, IP2STR(&ip_info.gw));
        }
    }
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        /* Get configs */
#if CONFIG_SOC_WIFI_SUPPORT_5G || CONFIG_SOC_WIFI_HE_SUPPORT_5G
        // TO compatible with the print of old apis, AP_PROTOCOL means sta protocol 2g, AP_CBW means sta cbw 2g
        ESP_ERROR_CHECK(esp_wifi_get_protocols(WIFI_IF_AP, &proto_config));
        ESP_LOGI(APP_TAG, " AP_PROTOCOL:%s", app_wifi_protocol_bitmap2str((uint32_t)proto_config.ghz_2g));
        ESP_LOGI(APP_TAG, " AP_PROTOCOL_5G:%s", app_wifi_protocol_bitmap2str((uint32_t)proto_config.ghz_5g));
        ESP_ERROR_CHECK(esp_wifi_get_bandwidths(WIFI_IF_AP, &cbw_config));
        ESP_LOGI(APP_TAG, " AP_CBW:%dMHz", cbw_config.ghz_2g == WIFI_BW_HT40 ? 40 : 20);
        ESP_LOGI(APP_TAG, " AP_CBW_5G:%dMHz", cbw_config.ghz_5g == WIFI_BW_HT40 ? 40 : 20);
#else
        ESP_ERROR_CHECK(esp_wifi_get_protocol(WIFI_IF_AP, (uint8_t *)&protocol));
        ESP_LOGI(APP_TAG, " AP_PROTOCOL:%s", app_wifi_protocol_bitmap2str(protocol));
        ESP_ERROR_CHECK(esp_wifi_get_bandwidth(WIFI_IF_AP, &cbw));
        ESP_LOGI(APP_TAG, " AP_CBW:%dMHz", cbw == WIFI_BW_HT40 ? 40 : 20);
#endif
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
        ESP_LOGI(APP_TAG, " AP_MAC:"MACSTR, MAC2STR(mac));
        ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, &cfg));
        strncpy(temp_ssid, (char *) cfg.ap.ssid, 32);
        ESP_LOGI(APP_TAG, " AP_SSID:%s", temp_ssid);
        ESP_LOGI(APP_TAG, " AP_CHANNEL:%d", cfg.ap.channel);
        esp_netif_get_ip_info(g_netif_ap, &ip_info);
        ESP_LOGI(APP_TAG, " AP_IP:"IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(APP_TAG, " AP_IP_MASK:"IPSTR, IP2STR(&ip_info.netmask));
        ESP_LOGI(APP_TAG, " AP_IP_GW:"IPSTR, IP2STR(&ip_info.gw));
        /* 2024/01/10
         * sizeof(wifi_sta_list_t): 124
         * sizeof(wifi_sta_mac_ip_list_t): 124
         */
        wifi_sta_list_t *p_sta_list = (wifi_sta_list_t *)malloc(sizeof(wifi_sta_list_t));
        if (p_sta_list == NULL) {
            ESP_LOGI(APP_TAG, "@EW: failed to malloc wifi_sta_list");
        } else {
            err = esp_wifi_ap_get_sta_list(p_sta_list);
            if (err == ESP_OK && p_sta_list->num) {
                wifi_sta_mac_ip_list_t wifi_sta_ip_mac_list;
                ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list_with_ip(p_sta_list, &wifi_sta_ip_mac_list));
                for (int i = 0; i < p_sta_list->num; i++) {
                    ESP_LOGI(APP_TAG, " AP_LSTA:"MACSTR","IPSTR"", MAC2STR(p_sta_list->sta[i].mac), IP2STR(&wifi_sta_ip_mac_list.sta[i].ip));
                }
            } else {
                ESP_LOGW(APP_TAG, "failed to get sta list, err:%d, num:%d", err, p_sta_list->num);
            }
            free(p_sta_list);
            p_sta_list = NULL;
        }
    }
}

static int cmd_do_wifi_count(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &wifi_count_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_count_args.end, argv[0]);
        return 1;
    }
    if (wifi_count_args.action->count > 0) {
        if (strcmp( wifi_count_args.action->sval[0], "clear") == 0) {
            memset(&g_wcount, 0, sizeof(g_wcount));
            ESP_LOGI(APP_TAG, "WIFI_COUNT_CLEAR,OK");
            return 0;
        }
    }
    ESP_LOGI(APP_TAG, "WiFi Counts:");
    ESP_LOGI(APP_TAG, "  BCN_TIMEOUT_CNT:%d", g_wcount.bcn_timeout);
    ESP_LOGI(APP_TAG, "  STA_DISCONNECTED_CNT:%d", g_wcount.sta_disconnected);
    return 0;
}

void wifi_cmd_register_wifi_count(void)
{
    wifi_count_args.action = arg_str0(NULL, NULL, "<action>", "query(default);clear;");
    wifi_count_args.end = arg_end(2);
    const esp_console_cmd_t wifi_count_cmd = {
        .command = "wifi_count",
        .help = "wifi count query/clear",
        .hint = NULL,
        .func = &cmd_do_wifi_count,
        .argtable = &wifi_count_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&wifi_count_cmd) );
}

static int cmd_do_wifi_mode(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &wifi_mode_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_mode_args.end, argv[0]);
        return 1;
    }
    esp_err_t err = ESP_FAIL;
    const char *mode_str = wifi_mode_args.mode->sval[0];
    ESP_LOGI(APP_TAG, "mode: %s", mode_str);
    wifi_mode_t mode = app_wifi_mode_str2num(mode_str);
    err = esp_wifi_set_mode(mode);
    LOG_WIFI_CMD_DONE(err, "SET_WIFI_MODE");
    return 0;
}

void app_register_wifi_mode(void)
{
    wifi_mode_args.mode = arg_str1(NULL, NULL, "<mode>", "wifi mode, 'ap', 'sta' or 'apsta'.");
    wifi_mode_args.end = arg_end(2);
    const esp_console_cmd_t wifi_mode_cmd = {
        .command = "wifi_mode",
        .help = "Set Wifi Mode <esp_wifi_set_mode()>",
        .hint = NULL,
        .func = &cmd_do_wifi_mode,
        .argtable = &wifi_mode_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&wifi_mode_cmd) );
}

static int cmd_do_set_wifi_ps(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &wifi_ps_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_ps_args.end, argv[0]);
        return 1;
    }

    esp_err_t err = ESP_FAIL;
    const int ps_type = wifi_ps_args.ps_type->ival[0];
    err = esp_wifi_set_ps((wifi_ps_type_t)ps_type);
    LOG_WIFI_CMD_DONE(err, "SET_WIFI_PS");
    return 0;
}

void app_register_wifi_ps(void)
{
    wifi_ps_args.ps_type = arg_int1(NULL, NULL, "<type>", "0: WIFI_PS_NONE, 1: WIFI_PS_MIN_MODEM, 2: WIFI_PS_MAX_MODEM");
    wifi_ps_args.end = arg_end(2);
    const esp_console_cmd_t wifi_ps_cmd = {
        .command = "wifi_ps",
        .help = "Set Wifi PS <esp_wifi_set_ps()>",
        .hint = NULL,
        .func = &cmd_do_set_wifi_ps,
        .argtable = &wifi_ps_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&wifi_ps_cmd) );
}


static int cmd_do_set_wifi_country(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &wifi_country_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_country_args.end, argv[0]);
        return 1;
    }

    esp_err_t err = ESP_FAIL;
    /* parse policy */
    wifi_country_policy_t policy = WIFI_COUNTRY_POLICY_AUTO;
    if (wifi_country_args.policy->count > 0) {
        if (strcmp(wifi_country_args.policy->sval[0], "manual") == 0) {
            policy = WIFI_COUNTRY_POLICY_MANUAL;
        }
    }
    if (wifi_country_args.code->count == 0) {
        /* no code given, get current country info */
        char code[3] = {0};
        wifi_country_t country = {0};
        err = esp_wifi_get_country_code(code);
        if (err == ESP_OK) {
            ESP_LOGI(APP_TAG, "CUR_COUNTRY_CODE:%c%c,", code[0], code[1]);
        } else {
            ESP_LOGE(APP_TAG, "error: %d", err);
        }
        err = esp_wifi_get_country(&country);
        if (err == ESP_OK) {
#if CONFIG_SOC_WIFI_SUPPORT_5G
            ESP_LOGI(APP_TAG, "GET_COUNTRY:%c%c,schan=%d,nchan=%d,policy=%s,5g_channel_mask:0x%x",
                country.cc[0], country.cc[1], country.schan, country.nchan,
                country.policy == WIFI_COUNTRY_POLICY_MANUAL? "manual" : "auto",
                country.wifi_5g_channel_mask);
            ESP_LOGI(APP_TAG, "CHANNEL_5G_LIST:%s", wifi_cmd_channel_bitmsk2str(WIFI_BAND_5G, country.wifi_5g_channel_mask));
#else
            ESP_LOGI(APP_TAG, "GET_COUNTRY:%c%c,schan=%d,nchan=%d,policy=%s",
                country.cc[0], country.cc[1], country.schan, country.nchan,
                country.policy == WIFI_COUNTRY_POLICY_MANUAL? "manual" : "auto");
#endif
        } else {
            ESP_LOGE(APP_TAG, "error: %d", err);
        }
        return 0;
    } else if (wifi_country_args.schan->count == 0
#if CONFIG_SOC_WIFI_SUPPORT_5G
               && wifi_country_args.chan_5g_bitmask == 0
               && wifi_country_args.chan_5g_list == 0
#endif
               ) {
        /* Call esp_wifi_set_country_code if schan not set */
        err = esp_wifi_set_country_code(wifi_country_args.code->sval[0], policy == WIFI_COUNTRY_POLICY_AUTO? true: false);
        LOG_WIFI_CMD_DONE(err, "SET_WIFI_COUNTRY_CODE");
        return 0;
    } else {
        wifi_country_t country = {.schan = 1,
                                  .nchan = 13,};
        country.policy = policy;
        memcpy(country.cc, wifi_country_args.code->sval[0], 2);
        if (wifi_country_args.schan->count > 0) {
            country.schan = wifi_country_args.schan->ival[0];
        }
        if (wifi_country_args.nchan->count > 0) {
            country.nchan = wifi_country_args.nchan->ival[0];
        }
#if CONFIG_SOC_WIFI_SUPPORT_5G
        if (wifi_country_args.chan_5g_bitmask->count > 0) {
            country.wifi_5g_channel_mask = wifi_country_args.chan_5g_bitmask->ival[0];
        } else if (wifi_country_args.chan_5g_list->count > 0) {
            country.wifi_5g_channel_mask = wifi_cmd_channel_str2bitmsk(WIFI_BAND_5G, wifi_country_args.chan_5g_list->sval[0]);
        }
#endif
        err = esp_wifi_set_country(&country);
        LOG_WIFI_CMD_DONE(err, "SET_WIFI_COUNTRY");
    }
    return 0;
}

void app_register_wifi_country(void)
{
    wifi_country_args.code = arg_str0(NULL, NULL, "<code>", "set country code cc, eg: CN, US, etc. if not given, get current country info.");
    wifi_country_args.schan = arg_int0("s", "schan", "<int>", "start channel");
    wifi_country_args.nchan = arg_int0("n", "nchan", "<int>", "total channel number");
    /* max_tx_power IDF doc: This field is used for getting WiFi maximum transmitting power */
    // wifi_country_args.nchan = arg_int0(NULL, "max_tx_power", "<int>", "total channel number");
    wifi_country_args.policy = arg_str0("p", "policy", "<str>", "policy, auto or manual, default: auto");
#if CONFIG_SOC_WIFI_SUPPORT_5G
    wifi_country_args.chan_5g_bitmask = arg_int0(NULL, "5g-chan-mask", "<int>", "5g support channel bitmask");
    wifi_country_args.chan_5g_list = arg_str0(NULL, "5g-chan", "<str>", "5g support channel, channels split with ','");
#endif
    wifi_country_args.end = arg_end(2);
    const esp_console_cmd_t wifi_country_cmd = {
        .command = "wifi_country",
        .help = "Set WiFi country <esp_wifi_set_country[_code]()>",
        .hint = NULL,
        .func = &cmd_do_set_wifi_country,
        .argtable = &wifi_country_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&wifi_country_cmd) );
}


void wifi_cmd_register_base_basic(void)
{
    wifi_cmd_register_wifi_init_deinit();
    wifi_cmd_register_wifi_count();
    app_register_wifi_mode();
    wifi_cmd_register_wifi_protocol();
    wifi_cmd_register_wifi_bandwidth();
    app_register_wifi_ps();
    app_register_wifi_country();
}

void wifi_cmd_register_all_basic(void)
{
    wifi_cmd_register_ap_basic();
    wifi_cmd_register_base_basic();
    wifi_cmd_register_sta_basic();
}

esp_err_t wifi_cmd_register_all(void)
{
    wifi_cmd_register_all_basic();

    wifi_cmd_register_wifi_txpower();
#if CONFIG_WIFI_CMD_ENABLE_ITWT
    wifi_cmd_register_itwt();
#endif
#if CONFIG_WIFI_CMD_ENABLE_WIFI_STATS
    wifi_cmd_register_stats();
#endif
#if CONFIG_WIFI_CMD_ENABLE_BAND
    wifi_cmd_register_wifi_band();
#endif
    return ESP_OK;
}
