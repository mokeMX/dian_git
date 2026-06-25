# WiFi AP+STA 共存模式 (SoftAP + Station)

WiFi AP+STA 双模式共存示例，基于 ESP-IDF 官方 `softap_sta` 示例改造。ESP32 同时作为 WiFi 热点和客户端运行，通过 NAPT 把 AP 侧设备转发上网——本质就是把 ESP32 当成一个 **WiFi 中继/小路由**。

> **这个分支是什么**：网络支线的进阶版（相对 `wifi` 分支的纯 STA）。当行李箱要既连上路由器、又给手机/上位机开一个热点接入时，用的就是这种 AP+STA 共存 + NAPT 转发的结构。代码主体是 Espressif 官方示例，改动主要是把 STA 的 SSID/密码写死成项目用的 `train123/linlin123`。

---

## 功能

- **AP 模式**: 创建 WiFi 热点，供其他设备连接（SSID/密码/信道/最大连接数由 **menuconfig** 配）
- **STA 模式**: 作为客户端连接外部路由器（**SSID/密码硬编码** `train123` / `linlin123`）
- **NAPT 转发**: 在 AP netif 上启用 `esp_netif_napt_enable`，AP 侧客户端可经 STA 上网
- **DNS 转发**: STA 拿到 IP 后，把上游 DNS 通过 `softap_set_dns_addr` 下发给 AP 的 DHCP

---

## 网络拓扑

```
[Internet] <--> [WiFi 路由器] <--> [ESP32-STA] <==NAPT==> [ESP32-AP] <--> [客户端设备(手机/PC)]
```

---

## WiFi 配置

**STA（连外部路由器）—— ⚠️ 硬编码，改要改源码**:
- SSID `train123`、密码 `linlin123`，在 `main/softap_sta.c` 的 `EXAMPLE_ESP_WIFI_STA_SSID/PASSWD` 宏。
- 注意：Kconfig 里**虽然有** `ESP_WIFI_REMOTE_AP_SSID/PASSWORD` 选项，但**代码没用它们**（被上面的宏覆盖），在 menuconfig 里改 STA 的 SSID 是**无效**的，必须改源码。

**AP（ESP32 自己的热点）—— 走 menuconfig**:
- `ESP_WIFI_AP_SSID`（默认 `myssid`）、`ESP_WIFI_AP_PASSWORD`（默认 `mypassword`）、`ESP_WIFI_AP_CHANNEL`（默认 1）、`ESP_MAX_STA_CONN_AP`（默认 4）。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py menuconfig   # ① 配 AP 参数  ② 务必启用 LWIP NAPT（见注意点1）
idf.py build
idf.py flash monitor
```

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/softap_sta.c` | 全部逻辑 | 顶部 STA 宏（硬编码 SSID/密码）；`app_main` 里 `esp_wifi_set_mode(WIFI_MODE_APSTA)` → `wifi_init_softap()` + `wifi_init_sta()` → 等连上 → `softap_set_dns_addr()`（DNS 下发）→ `esp_netif_set_default_netif(sta)` → `esp_netif_napt_enable(ap)`（**NAPT 开关就这一行**）。`#if IP_NAPT` 决定 NAPT 代码是否编进来。 |
| `main/Kconfig.projbuild` | AP/STA 菜单 | AP 的 SSID/密码/信道/最大连接、STA 的重试次数与扫描鉴权门限。**STA SSID/密码项实际未被代码使用**。 |
| `main/idf_component.yml` | 依赖 | 托管组件清单。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **必须在 menuconfig 启用 LWIP NAPT**，否则 NAPT 不生效、AP 侧设备上不了网：`Component config → LWIP → Enable NAPT`（对应 `CONFIG_LWIP_IPV4_NAPT`）。没开的话 `#if IP_NAPT` 整段不编译。
2. **AP 信道会被 STA「带跑」**：ESP32 单射频，AP+STA 共存时**两者必须同信道**。一旦 STA 连上路由器，AP 的实际工作信道会被强制对齐到路由器的信道——你在 menuconfig 配的 AP 信道可能不生效，属正常。
3. **STA 凭据硬编码**：改连接目标要改 `softap_sta.c` 源码并重编，menuconfig 改 STA 没用（见上）。
4. **2.4GHz only**：ESP32-S3 不支持 5GHz，上游路由器要有 2.4GHz。
5. **连不上会阻塞**：`xEventGroupWaitBits(..., portMAX_DELAY)` 等到「连上」或「重试到上限失败」才往下走，失败次数由 `ESP_MAXIMUM_STA_RETRY`（默认 5）控制。即使 STA 没连上，AP 仍会起来，只是没有上行网络。
6. **吞吐受限**：单射频做中继，AP 和 STA 抢同一个射频时隙，转发吞吐量会明显低于纯 STA，别拿它当主力路由。
7. **BSP 音频/LCD 组件在本分支挂着但不调用**，是同一套工程底座带来的。

---

## 调试与使用注意点

- 验证转发：手机连 ESP 的 AP（`myssid`），看能否上网/ping 通外网；不行先查 NAPT 是否启用、STA 是否真的连上路由器（串口 `connected to ap`）。
- 改 AP 名/密码后记得 `idf.py build flash` 重新烧。
- 看连接日志：`TAG_AP` / `TAG_STA` 两组日志分别对应两个接口。

---

## 目录结构

```
├── main/
│   ├── softap_sta.c            # 主程序：APSTA 双模式 + NAPT + DNS 下发
│   ├── CMakeLists.txt
│   ├── idf_component.yml       # 托管组件依赖
│   └── Kconfig.projbuild       # AP/STA 菜单（STA 项未被代码使用）
└── CMakeLists.txt
```
