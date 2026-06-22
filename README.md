# WiFi AP+STA 共存模式 (SoftAP + Station)

WiFi AP+STA 双模式共存示例，基于 ESP-IDF 官方 `softap_sta` 示例。ESP32 同时作为 WiFi 热点和客户端运行，支持 NAPT 网络转发。

## 功能

- **AP 模式**: 创建 WiFi 热点，供其他设备连接（SSID/密码通过 menuconfig 配置）
- **STA 模式**: 作为客户端连接到外部 WiFi AP（train123 / linlin123）
- **NAPT 转发**: 启用网络地址端口转换，使 AP 客户端可路由到互联网
- **DNS 转发**: 将 STA 获取的 DNS 服务器转发到 AP 的 DHCP 服务器

## 网络拓扑

```
[Internet] <--> [WiFi Router] <--> [ESP32-STA] <==NAPT==> [ESP32-AP] <--> [客户端设备]
```

## WiFi 配置

**STA 模式**（连接外部 AP）:
- SSID: `train123`
- 密码: `linlin123`

**AP 模式**（ESP32 创建的热点）:
- 通过 `menuconfig` 的 Kconfig 菜单配置
- 可配置项：SSID、密码、信道、最大连接数、STA 重试次数

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py menuconfig   # 配置 AP 参数
idf.py build
idf.py flash monitor
```

## 依赖组件

- ESP-IDF LWIP NAPT 支持（需在 menuconfig 中启用）

## 目录结构

```
├── main/
│   ├── softap_sta.c             # 主程序：AP+STA 双模式初始化
│   ├── CMakeLists.txt
│   ├── idf_component.yml        # 托管组件依赖
│   └── Kconfig.projbuild        # Kconfig 菜单配置
└── CMakeLists.txt
```
