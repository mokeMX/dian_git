# 引脚与接线建议

依据 `DNESP32-S3 IO引脚分配表.xlsx`、`DNESP32S3 V1.0 硬件参考手册.pdf` 和原理图文本整理。

## 推荐默认引脚

| 用途 | 默认 GPIO | 依据与说明 |
|---|---|---|
| A02YYUW SW-UART RX | GPIO4 | 软件串口 9600 baud，引脚空闲无冲突；通过 `SENSOR_HUB_A02YYUW_RX_GPIO` 可改 |
| A02YYUW SW-UART TX | GPIO5 | 软件串口，A02YYUW 为自主输出模式，TX 可悬空；通过 `SENSOR_HUB_A02YYUW_TX_GPIO` 可改 |
| BU UWB UART1 RX | GPIO6 | 硬件 UART1 @ 115200 baud；通过 `SENSOR_HUB_BU_UWB_RX_GPIO` 可改 |
| BU UWB UART1 TX | GPIO7 | 硬件 UART1；通过 `SENSOR_HUB_BU_UWB_TX_GPIO` 可改 |
| RPLIDAR C1 UART2 RX | GPIO17 | 硬件 UART2 @ 460800 baud，原始源码默认引脚；通过 `SENSOR_HUB_RPLIDAR_RX_GPIO` 可改 |
| RPLIDAR C1 UART2 TX | GPIO18 | 硬件 UART2，原始源码默认引脚；通过 `SENSOR_HUB_RPLIDAR_TX_GPIO` 可改 |
| I2C SDA | GPIO11 | 共享 I2C，空闲无冲突；通过 `SENSOR_HUB_I2C_SDA_GPIO` 可改 |
| I2C SCL | GPIO12 | 共享 I2C，空闲无冲突；通过 `SENSOR_HUB_I2C_SCL_GPIO` 可改 |
| FSR ADC | P3 `ADC_IN` / GPIO8 / ADC1_CH7 | 原理图 `ADC&REMOTE_OUT` 四针座 P3 明确标注 `ADC_IN IO8`；可通过 `SENSOR_HUB_FSR_ADC_GPIO` 与 `SENSOR_HUB_FSR_ADC_CHANNEL` 改成其它 ADC 输入 |

## 引脚冲突解决说明

原代码存在以下引脚重叠问题，已全部修复：

| 冲突引脚 | 冲突设备 | 解决方案 |
|---|---|---|
| GPIO36 | A02YYUW RX, BU UWB RX, RPLIDAR C1 RX | A02YYUW→GPIO4, BU UWB→GPIO6, RPLIDAR→GPIO17 |
| GPIO37 | A02YYUW TX, BU UWB TX, RPLIDAR TX, IMU SCL | A02YYUW→GPIO5, BU UWB→GPIO7, RPLIDAR→GPIO18, I2C→GPIO12 |
| GPIO38 | IMU SDA, VL53L1X SCL | 统一共享 I2C 总线→GPIO11(SDA), GPIO12(SCL) |
| UART1 | A02YYUW 和 BU UWB 共用 UART1 | A02YYUW 改用软件串口(GPIO 模拟)，BU UWB 独占 UART1 |

## A02YYUW 软件串口超声波

A02YYUW 仅需 9600 baud 且每帧只有 4 字节，适合使用 GPIO 位检测软件串口。
通过 `SENSOR_HUB_A02YYUW_USE_SW_UART=y` 启用（默认开启），使用 **`esp_timer`** 高精度定时器
+ GPIO 中断采样 RX 信号（本分支已修正采样时序：起始位半位对齐、之后整位采样）。如需使用硬件 UART，关闭该选项即可。

> ⚠️ 本分支（传感器修改2）`sw_uart.c` 仍缺 `#include <string.h>`、组件 `REQUIRES` 仍缺 `esp_driver_gpio`，干净编译需到 `传感器修改3`。

| A02YYUW | ESP32-S3 |
|---|---|
| VCC | 3.3V 或 5V，优先按模块实物标称供电 |
| GND | GND |
| TX | GPIO4，ESP32 RX（软件串口输入） |
| RX | GPIO5，ESP32 TX；可悬空/拉高保持稳定输出模式 |

默认配置：软件串口 `9600 8N1`、`RX=GPIO4`、`TX=GPIO5`。

## BU03/BU04 UWB

| BU03/BU04 UART2 | ESP32-S3 |
|---|---|
| PA2 / TX | GPIO6，ESP32 RX |
| PA3 / RX | GPIO7，ESP32 TX |
| GND | GND |
| VCC | 按 BU03/BU04 开发板/模块标称供电 |

默认配置：`UART1`、`115200 8N1`、`RX=GPIO6`、`TX=GPIO7`。

## RPLIDAR C1 UART 激光雷达

| RPLIDAR C1 | ESP32-S3 |
|---|---|
| VCC / 5V | 5V 电源，建议使用能提供 800mA 启动电流的独立 5V |
| GND | GND，必须与 ESP32 共地 |
| TX | GPIO17，ESP32 RX |
| RX | GPIO18，ESP32 TX |

默认配置：`UART2`、`460800 8N1`、`RX=GPIO17`、`TX=GPIO18`。

注意：RPLIDAR C1 官方资料标称 5V 供电，启动电流较高；若板载 5V 或 USB 供电不足，可能会出现无法启动、转速不足或测距不准。GPIO17/GPIO18 为原始源码默认引脚。

## I2C 模块：VL53L1X 与 IMU

| IMU | ESP32-S3 |
|---|---|
| SDA | GPIO11 |
| SCL | GPIO12 |
| VCC | 3.3V |
| GND | GND |

| VL53L1X | ESP32-S3 |
|---|---|
| SDA | GPIO11 |
| SCL | GPIO12 |
| VCC | 3.3V |
| GND | GND |

IMU 和 VL53L1X 共享同一 I2C 总线（I2C0），通过 `SENSOR_HUB_I2C_SDA_GPIO` 和 `SENSOR_HUB_I2C_SCL_GPIO` 统一配置。VL53L1X 默认 8 位地址 `0x52`（7 位 `0x29`），IMU 默认 7 位地址 `0x23`。

## FSR 薄膜压力传感器

优先接开发板原理图中 `ADC&REMOTE_OUT` 四针座 P3 的 `ADC_IN`。原理图对应关系为：`P3-4=GND`、`P3-3=REMOTE_OUT`、`P3-2=ADC_IN/IO8`、`P3-1=VCC3.3`。

FSR 必须与固定电阻组成分压后接 ADC：

```text
3.3V(P3-1) -- FSR -- ADC_IN(P3-2 / IO8) -- RM -- GND(P3-4)
```

当前样例沿用原始公式 `U = 0.0004 * F + 0.0749`，驱动中转换为 `F = (U - 0.0749) / 0.0004`，并限幅到 0-6kg。这个公式必须用你的实物重新标定后才能作为算法输入。

## 避免默认冲突

- 所有传感器引脚已去重叠：每个 GPIO 仅分配给一个外设。
- A02YYUW 默认使用软件串口（`CONFIG_SENSOR_HUB_A02YYUW_USE_SW_UART=y`），不占用硬件 UART。
- I2C 总线统一使用 GPIO11(SDA) / GPIO12(SCL)，与 UART 和 ADC 无冲突。
- GPIO17/GPIO18 为原始 RPLIDAR 源码默认引脚，仅在启用 LCD/摄像头时需调整。
