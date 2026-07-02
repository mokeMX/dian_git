# 引脚与接线建议 (YD-ESP32-S3 V1.4)

依据 YD-ESP32-S3 原理图 `YD-ESP32-S3-SCH-V1.4.pdf` 整理。该核心板为 ESP32-S3-WROOM-1 模块 + 两排 22 针排针引出全部 GPIO，板载 CH343P USB-UART、WS2812B RGB LED(GPIO48)、自动下载电路。

## YD-ESP32-S3 板载外设占用引脚

| 引脚 | 板载用途 | 说明 |
|---|---|---|
| GPIO19 | USB-OTG D+ | CH343P UD+ |
| GPIO20 | USB-OTG D- | CH343P UD- |
| GPIO43 | U0TXD | 串口 TX (CH343P RXD) |
| GPIO44 | U0RXD | 串口 RX (CH343P TXD) |
| GPIO0 | BOOT 按键 | 按下拉低，进入下载模式 |
| GPIO48 | RGB LED | WS2812B 数据输入 |
| EN | RST 按键 | 复位信号 |

> 除上表外，其余 GPIO 均通过 J1/J2 排针引出，可自由使用。

## 推荐默认引脚

| 用途 | 默认 GPIO | 接口类型 | 说明 |
|---|---|---|---|
| A02YYUW #1 RX | GPIO4 | SW UART | 软件串口 9600 baud，J1 排针 Pin 4 |
| A02YYUW #2 RX | GPIO5 | SW UART | 软件串口 9600 baud，J1 排针 Pin 5 |
| BU UWB RX | GPIO6 | HW UART1 | 115200 baud，J1 排针 Pin 6 |
| BU UWB TX | GPIO7 | HW UART1 | 115200 baud，J1 排针 Pin 7 |
| RPLIDAR C1 RX | GPIO17 | HW UART2 | 460800 baud，J1 排针 Pin 10 |
| RPLIDAR C1 TX | GPIO18 | HW UART2 | 460800 baud，J1 排针 Pin 11 |
| I2C SDA | GPIO11 | SW I2C | 软件 I2C 数据线，J1 排针 Pin 16 |
| I2C SCL | GPIO12 | SW I2C | 软件 I2C 时钟线，J1 排针 Pin 17 |
| FSR ADC | GPIO8 | ADC1 CH7 | 模拟输入，J1 排针 Pin 12 |

## SW-I2C 说明

YD-ESP32-S3 核心板无板载 I2C 外设，本项目使用 **软件 (bit-bang) I2C** 实现 I2C 通信。SDA 和 SCL 使用开漏输出 + 内部上拉，速率 100kHz（标准模式）。IMU 和 VL53L1X 共用同一对 SDA/SCL 引脚，通过不同的 7 位地址区分（IMU=0x23, VL53L1X=0x29）。

> 由于 SW-I2C 使用 GPIO bit-bang 方式，通信过程中可能被高优先级中断打断，因此建议 I2C 读取任务运行在较低中断负载的 Core 上（当前配置：Core 1）。提升 I2C 速率至 400kHz 可通过修改 `HUB_I2C_SPEED_HZ` 宏实现，但需确保从设备支持。

## 双 A02YYUW 超声波

- 超声波 #1 → 软件 UART，RX=GPIO4
- 超声波 #2 → 软件 UART，RX=GPIO5

两路均使用软件串口，各自独立的定时器和 GPIO 中断，不会相互干扰。A02YYUW 自主输出，ESP 端只需接 RX。

| A02YYUW #1 | ESP32-S3 | 说明 |
|---|---|---|
| VCC | 3.3V 或 5V | 按模块实物标称供电 |
| GND | GND | 必须与 ESP32 共地 |
| TX（模块输出） | GPIO4（ESP RX） | 软件 UART 输入 |

| A02YYUW #2 | ESP32-S3 | 说明 |
|---|---|---|
| VCC | 3.3V 或 5V | 按模块实物标称供电 |
| GND | GND | 必须与 ESP32 共地 |
| TX（模块输出） | GPIO5（ESP RX） | 软件 UART 输入 |

## BU03/BU04 UWB

| BU03/BU04 UART | ESP32-S3 |
|---|---|
| PA2 / TX | GPIO6 (ESP RX) |
| PA3 / RX | GPIO7 (ESP TX) |
| GND | GND |
| VCC | 按模块标称供电 |

## RPLIDAR C1 激光雷达

| RPLIDAR C1 | ESP32-S3 |
|---|---|
| VCC 5V | 5V 电源，建议独立供电 (>= 800mA) |
| GND | GND |
| TX | GPIO17 (ESP RX) |
| RX | GPIO18 (ESP TX) |

## I2C 设备: IMU + VL53L1X

| 设备 | SDA | SCL | VCC | GND | 7-bit 地址 |
|---|---|---|---|---|---|
| IMU | GPIO11 | GPIO12 | 3.3V | GND | 0x23 |
| VL53L1X | GPIO11 | GPIO12 | 3.3V | GND | 0x29 |

> IMU 和 VL53L1X 共享同一对 SW-I2C 引脚 (SDA=GPIO11, SCL=GPIO12)，通过不同 I2C 地址访问。必须外接 4.7kΩ ~ 10kΩ 上拉电阻到 3.3V（若使用内部上拉在长线情况下可能不够，推荐外加）。

## FSR 薄膜压力传感器

FSR 必须与固定电阻组成分压后接 ADC：

```
3.3V -- FSR -- ADC_IN(GPIO8) -- RM(10kΩ) -- GND
```

## YD-ESP32-S3 J1/J2 排针参考 (J1 左排 / J2 右排)

**J1 (左排，Pin 1 靠近 USB-C)：**

| Pin | 信号 | Pin | 信号 |
|---|---|---|---|
| 1 | 3.3V | 12 | GPIO8 |
| 2 | EN | 13 | GPIO3 |
| 3 | GPIO4 | 14 | GPIO46 |
| 4 | GPIO5 | 15 | GPIO10 |
| 5 | GPIO6 | 16 | GPIO11 |
| 6 | GPIO7 | 17 | GPIO12 |
| 7 | GPIO15 | 18 | GPIO13 |
| 8 | GPIO16 | 19 | GPIO14 |
| 9 | GPIO17 | 20 | GPIO21 |
| 10 | GPIO18 | 21 | GPIO45 |
| 11 | GPIO8 | 22 | GND |

**J2 (右排)：**

| Pin | 信号 | Pin | 信号 |
|---|---|---|---|
| 1 | GPIO1 | 12 | GPIO2 |
| 2 | GPIO42 | 13 | GPIO48 |
| 3 | GPIO41 | 14 | GPIO47 |
| 4 | GPIO40 | 15 | GPIO21 |
| 5 | GPIO39 | 16 | GPIO45 |
| 6 | GPIO38 | 17 | GPIO19 |
| 7 | GPIO37 | 18 | GPIO20 |
| 8 | GPIO36 | 19 | GPIO0 |
| 9 | GPIO35 | 20 | GPIO46 |
| 10 | U0RXD (GPIO44) | 21 | GPIO9 |
| 11 | U0TXD (GPIO43) | 22 | GPIO10 |

> 以上 J1/J2 排针定义依据原理图 `YD-ESP32-S3-SCH-V1.4.pdf` 中的网络标签提取。如接线后发现不一致，以实际板卡丝印为准。
