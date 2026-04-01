/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_wifi_types.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_idf_version.h"

#include "wifi_cmd.h"

#ifndef APP_TAG
#define APP_TAG "WIFI"
#endif

typedef struct {
    int8_t value;
    char *auth_mode;
} app_wifi_auth_mode_t;

static const app_wifi_auth_mode_t wifi_auth_mode_map[] = {
    {WIFI_AUTH_OPEN, "open"},
    {WIFI_AUTH_WEP, "wep"},
    {WIFI_AUTH_WPA_PSK, "wpa"},
    {WIFI_AUTH_WPA2_PSK, "wpa2"},
    {WIFI_AUTH_WPA_WPA2_PSK, "wpa_wpa2"},
    {WIFI_AUTH_WPA2_ENTERPRISE, "wpa2_enterprise"},
    {WIFI_AUTH_WPA3_PSK, "wpa3"},
    {WIFI_AUTH_WPA2_WPA3_PSK, "wpa2_wpa3"},
#ifdef WIFI_AUTH_WAPI_PSK
    {WIFI_AUTH_WAPI_PSK, "wapi"},
#endif
#ifdef WIFI_AUTH_OWE
    {WIFI_AUTH_OWE, "owe"},
#endif
};

#define WIFI_AUTH_TYPE_NUM  (sizeof(wifi_auth_mode_map) / sizeof(app_wifi_auth_mode_t))

uint8_t wifi_cmd_auth_mode_str2num(const char *auth_str)
{
    uint8_t auth_mode;
    for (auth_mode = 0; auth_mode < WIFI_AUTH_TYPE_NUM; auth_mode++) {
        if (strcmp(wifi_auth_mode_map[auth_mode].auth_mode, auth_str) == 0) {
            break;
        }
    }
    return wifi_auth_mode_map[auth_mode].value;
}

char *wifi_cmd_auth_mode_num2str(uint8_t value)
{
    uint8_t auth_mode;
    for (auth_mode = 0; auth_mode < WIFI_AUTH_TYPE_NUM; auth_mode++) {
        if (wifi_auth_mode_map[auth_mode].value == value) {
            break;
        }
    }
    assert(auth_mode != WIFI_AUTH_TYPE_NUM); /* If another authmode is added */
    return wifi_auth_mode_map[auth_mode].auth_mode;
}

wifi_interface_t app_wifi_interface_str2ifx(const char *interface)
{
    if (!strncmp(interface, "ap", 3)) {
        return WIFI_IF_AP;
    } else if (!strncmp(interface, "sta", 4)) {
        return WIFI_IF_STA;
    } else {
        ESP_LOGE(APP_TAG, "Can not get interface from str: %s", interface);
        /* Do not abort */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
        return WIFI_IF_MAX;
#else /* older IDF does not have IF_MAX*/
        return WIFI_IF_STA;
#endif
    }
    return WIFI_IF_STA;
}

typedef struct {
    char *mode_str;
    wifi_mode_t mode;
} app_wifi_mode_t;

static const app_wifi_mode_t s_wifi_mode_map[] = {
    {"null", WIFI_MODE_NULL},
    {"ap", WIFI_MODE_AP},
    {"sta", WIFI_MODE_STA},
    {"apsta", WIFI_MODE_APSTA},
};

#define WIFI_MODE_NUM  (sizeof(s_wifi_mode_map) / sizeof(app_wifi_mode_t))

wifi_mode_t app_wifi_mode_str2num(const char *mode_str)
{
    wifi_mode_t mode = 0;
    int i = 0;
    for (i = 0; i < WIFI_MODE_NUM; i++) {
        if (strcmp(s_wifi_mode_map[i].mode_str, mode_str) == 0) {
            mode = s_wifi_mode_map[i].mode;
            break;
        }
    }
    if (i == WIFI_MODE_NUM) {
        ESP_LOGE(APP_TAG, "Can not convert mode %s from str to value.", mode_str);
        /* Do not abort */
        return WIFI_MODE_NULL;
    }
    return mode;
}

const char *app_wifi_mode_num2str(const wifi_mode_t mode)
{
    char *mode_str = NULL;
    int i = 0;
    for (i = 0; i < WIFI_MODE_NUM; i++) {
        if (s_wifi_mode_map[i].mode == mode) {
            mode_str = s_wifi_mode_map[i].mode_str;
            break;
        }
    }
    if (i == WIFI_MODE_NUM) {
        ESP_LOGE(APP_TAG, "Can not convert mode %d to str", mode);
        /* Do not abort */
        return "unknown";
    }
    return mode_str;
}

esp_err_t wifi_cmd_str2mac(const char *str, uint8_t *mac_addr)
{
    unsigned int mac_tmp[6];
    if (6 != sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x%*c",
                    &mac_tmp[0], &mac_tmp[1], &mac_tmp[2],
                    &mac_tmp[3], &mac_tmp[4], &mac_tmp[5])) {
        return ESP_ERR_INVALID_MAC;
    }
    for (int i = 0; i < 6; i++) {
        mac_addr[i] = (uint8_t)mac_tmp[i];
    }
    return ESP_OK;

}

#if CONFIG_SOC_WIFI_SUPPORT_5G
#define MAX_CHANNEL_STR_LEN 128
uint32_t wifi_cmd_channel_str2bitmsk(wifi_band_t band, const char *channel_list_str)
{
    char input_copy[MAX_CHANNEL_STR_LEN];
    uint32_t channel_bitmap = 0;
    uint8_t channel = 0;
    strcpy(input_copy, channel_list_str);
    char *endptr;

    char *token = strtok(input_copy, ",");
    while (token != NULL) {

        channel = (uint8_t)strtol(token, &endptr, 10);
        // return 0 if input is not all numbers
        if (*endptr != '\0') {
            return 0;
        }

        if (band == WIFI_BAND_2G && channel > 14) {
            return 0;
        } else if (band == WIFI_BAND_5G && channel < 36) {
            return 0;
        }
        channel_bitmap |= CHANNEL_TO_BIT(channel);

        token = strtok(NULL, ",");
    }
    return channel_bitmap;
}

const char *wifi_cmd_channel_bitmsk2str(wifi_band_t band, uint32_t bitmap)
{
    static char channel_str[MAX_CHANNEL_STR_LEN];
    memset(channel_str, 0, sizeof(channel_str));

    int first_channel = 1;
    for (int bit_number = 0; bit_number < 32; bit_number++) {
        if (bitmap & (1 << bit_number)) {
            int channel = BIT_NUMBER_TO_CHANNEL(bit_number, band);
            if (channel != 0) {
                if (!first_channel) {
                    strcat(channel_str, ",");
                }
                char temp[8];
                snprintf(temp, sizeof(temp), "%d", channel);
                strcat(channel_str, temp);
                first_channel = 0;
            }
        }
    }

    if (strlen(channel_str) == 0) {
        return "UNKNOWN";
    }

    return channel_str;
}
#endif  /* CONFIG_SOC_WIFI_SUPPORT_5G */

#if WIFI_PHY_MODE_SUPPORTED
const char *wifi_cmd_phy_mode_num2str(wifi_phy_mode_t phymode)
{
    switch (phymode) {
    case WIFI_PHY_MODE_LR:
        return "lr";
    case WIFI_PHY_MODE_11B:
        return "b";
    case WIFI_PHY_MODE_11G:
        return "g";

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 1) /* IDF commit: f0365ba6b5d832a2e5999fda56b89c1f18c02858 */
    case WIFI_PHY_MODE_11A:
        return "a";
#endif /* v5.3.1 */

    case WIFI_PHY_MODE_HT20:
        return "ht20";
    case WIFI_PHY_MODE_HT40:
        return "ht40";
    case WIFI_PHY_MODE_HE20:
        return "he20";

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 1) /* IDF commit: f0365ba6b5d832a2e5999fda56b89c1f18c02858 */
    case WIFI_PHY_MODE_VHT20:
        return "vht20";
#endif /* v5.3.1 */

    default:
        return "?";
    }
    return "?";
}
#endif /* WIFI_PHY_MODE_SUPPORTED */
