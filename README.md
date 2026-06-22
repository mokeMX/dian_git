# WiFi STA + iPerf 测试

WiFi 客户端模式 + iPerf 网络吞吐量测试，支持 SPI LCD 状态显示和 UART 控制台。

## 功能

- **WiFi Station 模式**: 作为客户端连接到指定 WiFi AP
- **iPerf 吞吐量测试**: 通过控制台运行 iPerf 客户端测试到目标 PC
- **LCD 状态显示**: SPI LCD 实时显示 WiFi 连接状态和 IP
- **UART 控制台**: 提供 `iperf>` 命令行提示符
- **动态 IP 设置**: 通过 `setip` 命令动态修改目标 PC 的 IP 地址

## WiFi 配置

- SSID: `train123`
- 密码: `linlin123`

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- SPI LCD 显示屏
- XL9555 GPIO 扩展器

## 使用说明

1. 烧录程序后，ESP32 自动连接 WiFi
2. 连接成功后 LCD 显示 IP 地址
3. 通过串口终端输入命令：

```bash
iperf> setip 192.168.1.100    # 设置目标 PC IP
iperf> iperf -c <ip>          # 运行 iPerf 测试
```

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 依赖组件

- `espressif/iperf-cmd` - iPerf 命令行工具
- `esp-qa/wifi-cmd` - WiFi 命令行工具
- `esp-qa/ping-cmd` - Ping 命令行工具
- `espressif/esp-extconn` - 外部连接组件
- `cmd_system` - 系统命令组件

## 目录结构

```
├── main/
│   ├── main.c                  # 主程序
│   ├── CMakeLists.txt
│   └── idf_component.yml       # 托管组件依赖
├── components/BSP/
│   ├── SPILCD/                 # SPI LCD 驱动
│   ├── XL9555/                 # GPIO 扩展器
│   └── LED/                    # LED 驱动
└── CMakeLists.txt
```
