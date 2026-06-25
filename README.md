# WiFi STA + iPerf 测试

WiFi 客户端（Station）模式 + iPerf 网络吞吐量测试，支持 SPI LCD 状态显示和 UART 控制台。

> **这个分支是什么**：网络支线的「基础联网 + 测吞吐」分支。让 ESP32-S3 作为 STA 连到路由器，再用 iPerf 量一量无线吞吐量，顺便练 `esp_console` 串口命令行。`wifi3.3` 分支是它的进阶版（AP+STA 共存 + NAPT 转发）。

---

## 功能

- **WiFi Station 模式**: 连接到指定 WiFi AP（WPA2-PSK）
- **iPerf 吞吐量测试**: 通过控制台 `iperf` 命令测到目标 PC
- **LCD 状态显示**: 显示 SSID/密码、本机 IP、目标 IP
- **UART 控制台**: `iperf>` 命令行（基于 `esp_console` REPL，走 UART0）
- **动态 IP 设置**: `setip` 命令修改并在 LCD 显示目标 PC 的 IP

---

## WiFi 配置（硬编码，按需修改）

- SSID: `train123`、密码: `linlin123`
- 改成你自己的：`main/main.c` 顶部 `DEFAULT_SSID` / `DEFAULT_PWD` 两个宏（鉴权模式写死 `WIFI_AUTH_WPA2_PSK`）。
- 默认目标 PC IP：`192.168.1.100`（`g_server_ip` 初值）。

---

## 硬件要求

- Alientek ATK-DNESP32S3 开发板（自带 WiFi）
- SPI LCD 显示屏、XL9555 GPIO 扩展器

---

## 使用说明（实际测吞吐流程）

1. 把 **ESP32 和你的电脑连到同一个路由器/热点**（电脑这边也连 `train123` 或同网段）。
2. 电脑上启动 iPerf **服务端**：`iperf -s`（⚠️ 见下方「iperf2 不是 iperf3」）。
3. ESP32 烧录后自动连 WiFi，LCD 显示本机 IP。
4. 串口终端里用 `iperf>` 提示符跑客户端：
   ```text
   iperf> setip 192.168.1.100     # 仅记录并在 LCD 显示目标 IP（见注意点5）
   iperf> iperf -c 192.168.1.100 -t 30   # 真正发起测试用 iperf 命令自带的 -c
   ```

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | 全部逻辑 | `wifi_sta_init()`：建 netif/event loop、设 STA 配置、`esp_wifi_set_ps(WIFI_PS_NONE)`（关省电保吞吐）、`xEventGroupWaitBits(..., portMAX_DELAY)` **阻塞等连上或失败**；`wifi_event_handler`：断线**重试 20 次**后置 FAIL 位；`start_console_test()`：起 `iperf>` REPL、`app_register_iperf_commands()` 注册 iperf 命令、再注册自定义 `setip`。 |
| `main/idf_component.yml` | 托管依赖 | iperf/wifi/ping 命令组件来源与版本（见下）。 |
| `components/BSP/SPILCD`、`XL9555`、`LED` | 板级 | 状态显示与心跳灯（其余 BSP 音频驱动本分支用不到）。 |

---

## 依赖组件（`idf_component.yml`）

- `espressif/iperf-cmd` (~0.1.1) — 提供 `iperf` 命令（**iperf v2 协议**）
- `esp-qa/wifi-cmd` (~0.1.8) — WiFi 命令
- `esp-qa/ping-cmd` (~1.0.0) — `ping` 命令
- `cmd_system` — IDF 自带系统命令（本地路径引入）
- `espressif/esp-extconn` (~0.1.0) — ⚠️ **仅在 `target==esp32p4` 时才拉取**（见 yml 的 `rules`），**ESP32-S3 上不会用到**，旧 README 把它列成普通依赖容易误解。

---

## ⚠️ 注意事项 / 容易踩的坑

1. **电脑要用 iperf 2.x，不是 iperf3！** Espressif 的 `iperf-cmd` 实现的是 **iperf v2** 协议，和 `iperf3` **互不兼容**。PC 端用 iperf3 会连不上/测不出。下载 iperf 2.x（`iperf` / `iperf2`）做服务端。
2. **凭据硬编码**：`train123`/`linlin123` 写在源码里，换网络要改宏重新编译；不要把含真实密码的固件随意外发。
3. **5GHz 连不上**：ESP32-S3 只支持 **2.4GHz**，AP 开 5GHz-only 会连不上。
4. **连不上会卡在哪**：`wifi_sta_init` 会阻塞等待；断线重试 20 次后才置 FAIL 位放行。SSID/密码错或信号弱时，前期会一直 `wifi connecting......`。
5. **`setip` 和实际测试是「半脱节」的**：`setip` 只更新全局 `g_server_ip` 和 LCD 显示，而真正用 `g_server_ip` 的 `start_iperf_client()` 函数**在本分支里定义了但从未被调用**（dead code）。所以发起测试请用控制台 `iperf -c <ip>` 命令**自带的 `-c` 参数**，别指望 `setip` 后直接开测。要让 `setip` 生效，得自己在某处调用 `start_iperf_client`。
6. **控制台占用 UART0**：`iperf>` REPL 和日志共用 UART0，输入命令和日志会混在一起，正常现象。

---

## 调试与使用注意点

- 测不出吞吐：先 `ping` 通目标 IP，再确认 PC 防火墙放行 iperf 端口（默认 5001），且 PC 用的是 iperf2。
- LCD 不显示 IP：说明还没拿到 IP（DHCP 失败/没连上），看串口 `Got IP` 日志。
- 心跳灯：LED0 约每 0.5s 翻转一次（`led_tick>=50` × 10ms），用来判断程序还活着。

---

## 目录结构

```
├── main/
│   ├── main.c                  # 主程序：WiFi STA + iperf REPL + setip
│   ├── CMakeLists.txt
│   └── idf_component.yml       # 托管组件依赖（iperf/wifi/ping/extconn）
├── components/BSP/
│   ├── SPILCD/                 # SPI LCD 驱动（状态显示）
│   ├── XL9555/                 # GPIO 扩展器
│   └── LED/                    # LED 驱动（心跳灯）
└── CMakeLists.txt
```
