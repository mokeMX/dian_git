/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "wifi_cmd.h"
#include "esp_log.h"
#if CONFIG_SOC_WIFI_HE_SUPPORT && CONFIG_WIFI_CMD_ENABLE_ITWT
#include "esp_wifi_he_types.h"
#endif

#ifndef APP_TAG
#define APP_TAG "WIFI"
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

bool g_is_sta_wifi_connected = false;
bool g_is_sta_got_ip4 = false;
bool g_is_scan_count_only = true;
wifi_cmd_config_t g_wifi_cmd_config = WIFI_CMD_CONFIG_DEFAULT();

void app_handler_on_sta_disconnected(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    g_wcount.sta_disconnected += 1;

    g_is_sta_wifi_connected = false;
    g_is_sta_got_ip4 = false;
    wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGI(APP_TAG, "WIFI_EVENT_STA_DISCONNECTED! reason: %d", event->reason);

    if (!g_wifi_cmd_config.reconnect) {
        return;
    }
    g_wifi_connect_retry_cnt++;
    if (g_wifi_connect_retry_cnt > g_wifi_cmd_config.reconnect_max_retry) {
        return;
    }
    ESP_LOGI(APP_TAG, "trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "WiFi Reconnect failed! (%s)", esp_err_to_name(err));
        return;
    }
}

void app_handler_on_sta_connected(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    g_is_sta_wifi_connected = true;
    ESP_LOGI(APP_TAG, "WIFI_EVENT_STA_CONNECTED!");
#if CONFIG_EXAMPLE_CONNECT_IPV6
    esp_netif_create_ip6_linklocal(esp_netif);
#endif // CONFIG_EXAMPLE_CONNECT_IPV6
}

void app_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    g_is_sta_got_ip4 = true;
    g_wifi_connect_retry_cnt = 0;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(APP_TAG, "IP_EVENT_STA_GOT_IP: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    ESP_LOGI(APP_TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));

}

#if CONFIG_LWIP_IPV6
void app_handler_on_sta_got_ipv6(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
    ESP_LOGI(APP_TAG, "IP_EVENT_GOT_IP6: Interface \"%s\" address: " IPV6STR ", type: %d", esp_netif_get_desc(event->esp_netif),
             IPV62STR(event->ip6_info.ip), ipv6_type);
    ESP_LOGI(APP_TAG, "- IPv6 address: " IPV6STR ", type: %d", IPV62STR(event->ip6_info.ip), ipv6_type);
}
#endif // CONFIG_LWIP_IPV6

void app_handler_on_scan_done(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint16_t sta_number = 0;
    uint8_t i;
    wifi_ap_record_t *ap_list_buffer;

    esp_wifi_scan_get_ap_num(&sta_number);
    if (!sta_number) {
        ESP_LOGE(APP_TAG, "SCAN_DONE: No AP found");
        return;
    }

    if (g_is_scan_count_only) {
        ESP_LOGI(APP_TAG, "SCAN_DONE: Found %d APs", sta_number);
#ifdef esp_wifi_clear_ap_list /* IDF >= v4.3, MR: 19441 */
        esp_wifi_clear_ap_list();
#endif
        return;
    }

    ap_list_buffer = malloc(sta_number * sizeof(wifi_ap_record_t));
    if (ap_list_buffer == NULL) {
        ESP_LOGE(APP_TAG, "SCAN_DONE: Failed to malloc buffer to print scan results");
#ifdef esp_wifi_clear_ap_list /* IDF >= v4.3, MR: 19441 */
        esp_wifi_clear_ap_list();
#endif
        return;
    }

    if (esp_wifi_scan_get_ap_records(&sta_number, (wifi_ap_record_t *)ap_list_buffer) == ESP_OK) {
        for (i = 0; i < sta_number; i++) {

            char phy_str[10] = "";
#if CONFIG_SOC_WIFI_HE_SUPPORT
            if (ap_list_buffer[i].phy_11ax) {
                sprintf(phy_str, "11ax");
            } else
#endif
                if (ap_list_buffer[i].phy_11n) {
                    sprintf(phy_str, "11n");
                } else if (ap_list_buffer[i].phy_11g) {
                    sprintf(phy_str, "11g");
                } else if (ap_list_buffer[i].phy_11b) {
                    sprintf(phy_str, "11b");
                }

            ESP_LOGI(APP_TAG,
                     "+SCAN:["MACSTR"][%s][rssi=%d][authmode=%d][ch=%d][second=%d][%s]"
#if CONFIG_SOC_WIFI_HE_SUPPORT
                     "[bssid-index=%d][bss_color=%d][disabled=%d]"
#endif
                     , MAC2STR(ap_list_buffer[i].bssid), ap_list_buffer[i].ssid, ap_list_buffer[i].rssi, ap_list_buffer[i].authmode,
                     ap_list_buffer[i].primary, ap_list_buffer[i].second, phy_str
#if CONFIG_SOC_WIFI_HE_SUPPORT
                     , ap_list_buffer[i].he_ap.bssid_index, ap_list_buffer[i].he_ap.bss_color, ap_list_buffer[i].he_ap.bss_color_disabled
#endif
                    );
        }
    }
    free(ap_list_buffer);
    ESP_LOGI(APP_TAG, "SCAN_DONE: Found %d APs", sta_number);
}

void app_handler_on_bcn_timeout(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    g_wcount.bcn_timeout += 1;
    /* WiFi will print "wifi:bcn_timeout,ap_probe_send_start" */
    // ESP_LOGI(APP_TAG, "WIFI_EVENT_STA_BEACON_TIMEOUT!");
}

#if CONFIG_SOC_WIFI_HE_SUPPORT && CONFIG_WIFI_CMD_ENABLE_ITWT

#define ITWT_SETUP_SUCCESS 1

static const char *itwt_probe_status_to_str(wifi_itwt_probe_status_t status)
{
    switch (status) {
    case ITWT_PROBE_FAIL:                 return "itwt probe fail";
    case ITWT_PROBE_SUCCESS:              return "itwt probe success";
    case ITWT_PROBE_TIMEOUT:              return "itwt probe timeout";
    case ITWT_PROBE_STA_DISCONNECTED:     return "Sta disconnected";
    default:                              return "Unknown status";
    }
}

void app_handler_on_itwt_setup(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_itwt_setup_t *setup = (wifi_event_sta_itwt_setup_t *) event_data;
    if (setup->status == ITWT_SETUP_SUCCESS) {
        /* TWT Wake Interval = TWT Wake Interval Mantissa * (2 ^ TWT Wake Interval Exponent) */
        ESP_LOGI(APP_TAG, "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, flow_id:%d, %s, %s, wake_dura:%d, wake_dura_unit:%d, wake_invl_e:%d, wake_invl_m:%d", setup->config.twt_id,
                 setup->config.flow_id, setup->config.trigger ? "trigger-enabled" : "non-trigger-enabled", setup->config.flow_type ? "unannounced" : "announced",
                 setup->config.min_wake_dura, setup->config.wake_duration_unit, setup->config.wake_invl_expn, setup->config.wake_invl_mant);
        ESP_LOGI(APP_TAG, "<WIFI_EVENT_ITWT_SETUP>target wake time:%lld, wake duration:%d us, service period:%d us", setup->target_wake_time, setup->config.min_wake_dura << (setup->config.wake_duration_unit == 1 ? 10 : 8),
                 setup->config.wake_invl_mant << setup->config.wake_invl_expn);
    } else {
        if (setup->status == ESP_ERR_WIFI_TWT_SETUP_TIMEOUT) {
            ESP_LOGE(APP_TAG, "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, timeout of receiving twt setup response frame", setup->config.twt_id);
        } else if (setup->status == ESP_ERR_WIFI_TWT_SETUP_TXFAIL) {
            ESP_LOGE(APP_TAG, "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, twt setup frame tx failed, reason: %d", setup->config.twt_id, setup->reason);
        } else if (setup->status == ESP_ERR_WIFI_TWT_SETUP_REJECT) {
            ESP_LOGE(APP_TAG, "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, twt setup request was rejected, setup cmd: %d", setup->config.twt_id, setup->config.setup_cmd);
        } else {
            ESP_LOGE(APP_TAG, "<WIFI_EVENT_ITWT_SETUP>twt_id:%d, twt setup failed, status: %d", setup->config.twt_id, setup->status);
        }
    }
}

void app_handler_on_itwt_teardown(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_itwt_teardown_t *teardown = (wifi_event_sta_itwt_teardown_t *) event_data;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)  // IDF MR: 30679
    // sta tx twt teardown action but not receive ack
    if (teardown->status == ITWT_TEARDOWN_FAIL) {
        ESP_LOGE(APP_TAG, "<WIFI_EVENT_ITWT_TEARDOWN>flow_id %d%s, twt teardown frame tx failed", teardown->flow_id, (teardown->flow_id == 8) ? "(all twt)" : "");
        return;
    }
#endif /* ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0) */
    ESP_LOGI(APP_TAG, "<WIFI_EVENT_ITWT_TEARDOWN>flow_id %d%s", teardown->flow_id, (teardown->flow_id == 8) ? "(all twt)" : "");
}

void app_handler_on_itwt_suspend(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_itwt_suspend_t *suspend = (wifi_event_sta_itwt_suspend_t *) event_data;
    ESP_LOGI(APP_TAG, "<WIFI_EVENT_ITWT_SUSPEND>status:%d, flow_id_bitmap:0x%x, actual_suspend_time_ms:[%lu %lu %lu %lu %lu %lu %lu %lu]",
             suspend->status, suspend->flow_id_bitmap,
             suspend->actual_suspend_time_ms[0], suspend->actual_suspend_time_ms[1], suspend->actual_suspend_time_ms[2], suspend->actual_suspend_time_ms[3],
             suspend->actual_suspend_time_ms[4], suspend->actual_suspend_time_ms[5], suspend->actual_suspend_time_ms[6], suspend->actual_suspend_time_ms[7]);
}

void app_handler_on_itwt_prob(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_itwt_probe_t *probe = (wifi_event_sta_itwt_probe_t *) event_data;
    ESP_LOGI(APP_TAG, "<WIFI_EVENT_ITWT_PROBE>status:%s, reason:0x%x", itwt_probe_status_to_str(probe->status), probe->reason);
}
#endif /* CONFIG_SOC_WIFI_HE_SUPPORT && CONFIG_WIFI_CMD_ENABLE_ITWT*/
