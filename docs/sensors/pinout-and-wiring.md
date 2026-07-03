# 引脚与接线建议 (引脚与算法4一致版)

本分支引脚映射与 `算法4` (follow_robot) 分支完全一致。

## 引脚总览

| 传感器 | 接口 | GPIO | 备注 |
|--------|------|------|------|
| A02YYUW #1 (超声波左) | SW UART (9600) | RX=GPIO35 | 软件串口，仅接 RX |
| A02YYUW #2 (超声波右) | SW UART (9600) | RX=GPIO36 | 软件串口，仅接 RX |
| BU UWB | HW UART1 (115200) | RX=GPIO18, TX=GPIO37 | 独占 UART1 |
| RPLIDAR C1 | HW UART2 (460800) | RX=GPIO17, TX=GPIO9 | 独占 UART2 |
| IMU | I2C0 | SDA=GPIO39, SCL=GPIO38 | addr 0x23 |
| VL53L1X ToF | I2C0 共享 | SDA=GPIO39, SCL=GPIO38 | addr 0x52(8位) |
| FSR 压力 | ADC1 | GPIO8 (ADC1_CH7) | 模拟输入 |

## 与算法4 (follow_robot) 管脚对比

| 传感器 | 算法4 引脚 | 本分支引脚 | 是否一致 |
|--------|-----------|-----------|---------|
| 超声波左 RX | GPIO35 | GPIO35 | 一致 |
| 超声波右 RX | GPIO36 | GPIO36 | 一致 |
| UWB RX | GPIO18 | GPIO18 | 一致 |
| UWB TX | GPIO37 | GPIO37 | 一致 |
| RPLIDAR RX | GPIO17 | GPIO17 | 一致 |
| RPLIDAR TX | GPIO9 | GPIO9 | 一致 |
| I2C SDA | GPIO39 | GPIO39 | 一致 |
| I2C SCL | GPIO38 | GPIO38 | 一致 |
| IMU 地址 | 0x23 | 0x23 | 一致 |

## 与传感器修改6 原版管脚对比（变更对照）

| 传感器 | 原 传感器修改6 | 新 (与算法4一致) | 变更 |
|--------|---------------|-----------------|------|
| 超声波 #1 RX | GPIO4 | GPIO35 | GPIO4 → GPIO35 |
| 超声波 #2 RX | GPIO5 | GPIO36 | GPIO5 → GPIO36 |
| UWB RX | GPIO6 | GPIO18 | GPIO6 → GPIO18 |
| UWB TX | GPIO7 | GPIO37 | GPIO7 → GPIO37 |
| RPLIDAR TX | GPIO18 | GPIO9 | GPIO18 → GPIO9 |
| I2C SDA | GPIO39 | GPIO39 | 不变 |
| I2C SCL | GPIO38 | GPIO38 | 不变 |

## A02YYUW 超声波接线

| A02YYUW #1 | ESP32-S3 |
|---|---|
| VCC | 3.3V 或 5V |
| GND | GND |
| TX (模块输出) | **GPIO35** (ESP RX, SW UART) |
| RX (模块输入) | 不接 |

| A02YYUW #2 | ESP32-S3 |
|---|---|
| VCC | 3.3V 或 5V |
| GND | GND |
| TX (模块输出) | **GPIO36** (ESP RX, SW UART) |
| RX (模块输入) | 不接 |

## BU UWB 接线

| BU UWB | ESP32-S3 |
|---|---|
| PA2 / TX | **GPIO18** (ESP RX) |
| PA3 / RX | **GPIO37** (ESP TX) |
| GND | GND |
| VCC | 按模块标称供电 |

## RPLIDAR C1 接线

| RPLIDAR C1 | ESP32-S3 |
|---|---|
| VCC / 5V | 5V 电源 (≥800mA) |
| GND | GND |
| TX | **GPIO17** (ESP RX) |
| RX | **GPIO9** (ESP TX) |

## I2C 模块 (IMU + VL53L1X)

| IMU / VL53L1X | ESP32-S3 |
|---|---|
| SDA | **GPIO39** |
| SCL | **GPIO38** |
| VCC | 3.3V |
| GND | GND |

> GPIO39 在经典 ESP32 上为仅输入引脚，不可作 SDA。ESP32-S3 上 GPIO39 可作为双向 IO，因此 SDA=GPIO39 在 ESP32-S3 下合法。

## FSR 接线

| FSR | ESP32-S3 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| 信号 | **GPIO8** (ADC1_CH7) |
