/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_idf_version.h"

/*
 * CONFIG_ESP_WIFI_ENABLED added in idf v5.1
 * define it in header file to simplify the application code
 */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#define CONFIG_ESP_WIFI_ENABLED (1)
#elif ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 0)
#ifdef CONFIG_ESP32_WIFI_ENABLED
#define CONFIG_ESP_WIFI_ENABLED (CONFIG_ESP32_WIFI_ENABLED)
#endif
#endif

/* wifi_phy_mode_t supported at IDF commit: c681c92e4c125e5ba0599d4d51d3c423c8afe518, backports: v5.0.3, v4.4.7 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 1) || \
    (ESP_IDF_VERSION_MINOR == 0 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 3)) || \
    (ESP_IDF_VERSION_MAJOR == 4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 7))
#define WIFI_PHY_MODE_SUPPORTED (1)
#endif

#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED || CONFIG_ESP_HOST_WIFI_ENABLED

/* Avoid re-connect forever */
#define DEF_WIFI_CONN_MAX_RETRY_CNT (30)

/*
 * Default rssi offset value for 5G SSID when connect wifi
 * Make sta prefer to connect SSID on 5G band when do full scan connect
 * This value will affact SSID sort order when sort method is WIFI_CONNECT_AP_BY_SIGNAL
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0) && !CONFIG_WIFI_CMD_BASIC_ONLY
#define DEF_5G_CONN_RSSI_OFFSET     (10)
#endif

typedef struct {
    bool reconnect;
    uint32_t reconnect_max_retry;
} wifi_cmd_config_t;

#define WIFI_CMD_CONFIG_DEFAULT() { \
    .reconnect = true, \
    .reconnect_max_retry = DEF_WIFI_CONN_MAX_RETRY_CNT, \
}

#define LOG_WIFI_CMD_DONE(ret, desc) do { \
    if (ret == ESP_OK) { \
        ESP_LOGI(APP_TAG, "DONE.%s,OK.", desc); \
    } else { \
        ESP_LOGI(APP_TAG, "DONE.%s,FAIL.%d,%s", desc, ret, esp_err_to_name(ret)); \
    } \
} while(0)

#define WIFI_ERR_CHECK_LOG(ret, desc) do { \
    if (ret != ESP_OK) { \
        ESP_LOGW(APP_TAG, "@EW:failed:%s,%d,%s", desc, ret, esp_err_to_name(ret)); \
    } \
} while(0)

/* TODO: v1.0 use wifi_init_config_t directly */
typedef struct {
    wifi_init_config_t wifi_init_cfg;
    wifi_storage_t storage;
    wifi_ps_type_t ps_type;
    uint32_t magic;
} wifi_cmd_initialize_cfg_t;

#define app_wifi_initialise_config_t wifi_cmd_initialize_cfg_t
#define WIFI_CMD_INIT_MAGIC 0x00FF7161
#define WIFI_CMD_INIT_MAGIC_DEPRECATED_APP 0xA1A2A3A4

/* Deprecated, remove in v1.0 */
#define APP_WIFI_CONFIG_DEFAULT() { \
    .storage = WIFI_STORAGE_FLASH, \
    .ps_type = WIFI_PS_MIN_MODEM, \
    .magic = WIFI_CMD_INIT_MAGIC_DEPRECATED_APP, \
}

#define WIFI_CMD_INITIALISE_CONFIG_DEFAULT() { \
    .wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT(), \
    .magic = WIFI_CMD_INIT_MAGIC, \
}

typedef struct {
    int bcn_timeout;
    int sta_disconnected;
} wifi_cmd_wifi_count_t;
#define app_wifi_count_t  wifi_cmd_wifi_count_t

/**
 * Variables
 */
extern wifi_cmd_config_t g_wifi_cmd_config;
extern wifi_cmd_wifi_count_t g_wcount;
extern esp_netif_t *g_netif_ap;
extern esp_netif_t *g_netif_sta;
extern bool g_is_sta_wifi_connected;
extern bool g_is_sta_got_ip4;
extern bool g_is_scan_count_only;

/**
 * @brief This function is a combination of "wifi init" and "wifi start".
 *
 * @param[in] config Specifies the WiFi initialization configuration that differs from the sdkconfig default, can be NULL.
 *                   wifi-cmd will save this configuration globally and use it during "wifi restart".
 */
void wifi_cmd_initialize_wifi(wifi_cmd_initialize_cfg_t *config);

/**
 * @brief Register command: wifi [init|deinit|start|stop|status]
 */
void wifi_cmd_register_wifi_init_deinit(void);

/**
 * @brief Register: wifi_txpower
 */
void wifi_cmd_register_wifi_txpower(void);

/**
 * @brief Register commands: wifi, wifi_count, wifi_mode, wifi_protocol, wifi_bandwidth, wifi_ps.
 */
void wifi_cmd_register_base_basic(void);

/**
 * @brief Register commands: sta_connect, sta_disconnect, sta_scan
 */
void wifi_cmd_register_sta_basic(void);

/**
 * @brief Register commands: ap_set
 */
void wifi_cmd_register_ap_basic(void);

/**
 * @brief Register: base/sta/ap base commands
 */
void wifi_cmd_register_all_basic(void);

/**
 * @brief Register: all supported wifi commands
 */
esp_err_t wifi_cmd_register_all(void);

/* ----------------------------------------- Optional/Enhanced commands -------------------------------------------- */
#if CONFIG_WIFI_CMD_ENABLE_BAND
/**
 * @brief Register commands: wifi_band
 */
void wifi_cmd_register_wifi_band(void);
#endif

#if CONFIG_WIFI_CMD_ENABLE_ITWT
/**
 * @brief Register commands: itwt, probe
 */
void wifi_cmd_register_itwt(void);
#endif /* CONFIG_WIFI_CMD_ENABLE_ITWT */

#if CONFIG_WIFI_CMD_ENABLE_WIFI_STATS
/**
 * @brief Register commands: txstats, rxstats
 */
void wifi_cmd_register_stats(void);
#endif /* CONFIG_WIFI_CMD_ENABLE_WIFI_STATS */

#if CONFIG_WIFI_CMD_ENABLE_HE_DEBUG
/**
 * @brief Register: wifi he debug related commands.
 */
/**
 * include: he debug commands
 */
void wifi_cmd_register_he_debug(void);
#endif /* WIFI_CMD_ENABLE_HE_DEBUG */

/* --------------------------------------------- Internal ---------------------------------------------------------- */
extern int g_wifi_connect_retry_cnt;
uint8_t wifi_cmd_auth_mode_str2num(const char *auth_str);
char *wifi_cmd_auth_mode_num2str(uint8_t value);
wifi_interface_t app_wifi_interface_str2ifx(const char *interface);
uint32_t app_wifi_protocol_str2bitmap(const char *protocol_str);
const char *app_wifi_protocol_bitmap2str(const uint32_t bitmap);
wifi_mode_t app_wifi_mode_str2num(const char *mode_str);
const char *app_wifi_mode_num2str(const wifi_mode_t mode);
#if WIFI_PHY_MODE_SUPPORTED
const char *wifi_cmd_phy_mode_num2str(wifi_phy_mode_t phymode);
#endif
void wifi_cmd_query_wifi_info(void);
esp_err_t wifi_cmd_str2mac(const char *str, uint8_t *mac_addr);
#if CONFIG_SOC_WIFI_SUPPORT_5G
uint32_t wifi_cmd_channel_str2bitmsk(wifi_band_t band, const char *channel_list_str);
const char *wifi_cmd_channel_bitmsk2str(wifi_band_t band, uint32_t bitmap);
#endif
void wifi_cmd_register_wifi_protocol(void);
void wifi_cmd_register_wifi_bandwidth(void);
void wifi_cmd_register_wifi_count(void);

/* wifi handlers */
void app_handler_on_sta_disconnected(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_sta_connected(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_scan_done(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_sta_got_ipv6(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_bcn_timeout(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
#if CONFIG_SOC_WIFI_HE_SUPPORT && CONFIG_WIFI_CMD_ENABLE_ITWT
void app_handler_on_itwt_setup(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_itwt_teardown(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_itwt_suspend(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void app_handler_on_itwt_prob(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
#endif /* CONFIG_SOC_WIFI_HE_SUPPORT && CONFIG_WIFI_CMD_ENABLE_ITWT*/

/* Deprecated, remove in v1.0 */
#define app_initialise_wifi wifi_cmd_initialize_wifi
#define app_register_wifi_init_deinit wifi_cmd_register_wifi_init_deinit
#define app_register_wifi_basic_commands wifi_cmd_register_base_basic
#define app_register_sta_basic_commands wifi_cmd_register_sta_basic
#define app_register_ap_basic_commands wifi_cmd_register_ap_basic
#define app_register_all_wifi_commands wifi_cmd_register_all
#define app_register_wifi_band_command wifi_cmd_register_wifi_band
#define app_register_itwt_commands wifi_cmd_register_itwt
#define app_register_wifi_stats_commands wifi_cmd_register_stats
#define app_register_wifi_he_debug_commands wifi_cmd_register_he_debug

#endif  /* CONFIG_ESP_WIFI_ENABLED */
