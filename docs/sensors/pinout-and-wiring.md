# 引脚与接线建议

依据 `DNESP32-S3 IO引脚分配表.xlsx`、`DNESP32S3 V1.0 硬件参考手册.pdf` 和原理图文本整理。

## 开发板真实可用引脚（重要）

依据 `DNESP32-S3 IO引脚分配表.xlsx`，ATK-DNESP32S3 上**只有 IO35 / IO36 / IO37 三个 GPIO 被标记为「完全独立」**（不接任何板载外设，可随意使用）。其余所有 IO 均已被 RGB-LCD、摄像头(OV)、I2S 音频、SPI2、IIC0(触摸/IIC)、USB、UART0 控制台占用。因此本分支把两个超声波传感器放在这三个空闲引脚上，绝不与板载外设抢占引脚。

| 引脚 | 板载占用 | 本项目用途 |
|---|---|---|
| IO35 | 完全独立 | 超声波 #1 RX（硬件 UART1） |
| IO36 | 完全独立 | 超声波 #2 RX（软件 UART） |
| IO37 | 完全独立 | 预留（备用 RX / 第三传感器） |

> 注：原 `传感器修改3` 等分支默认的 GPIO4/5/6/7/8/11/12/17/18 在本开发板上实际被 LCD/摄像头/I2S/SPI/IIC0 占用，仅在不使用这些板载外设时才可借用。若你的硬件确实空出了那些引脚，可在 `menuconfig` 中改回。

## 推荐默认引脚

| 用途 | 默认 GPIO | 依据与说明 |
|---|---|---|
| A02YYUW #1 RX | GPIO35 | 硬件 UART1 @ 9600 baud，板载完全独立引脚；通过 `SENSOR_HUB_A02YYUW_RX_GPIO` 可改 |
| A02YYUW #1 TX | -1（不接） | A02YYUW 自主输出，ESP 仅需 RX；如需控制再分配，置 `SENSOR_HUB_A02YYUW_TX_GPIO` |
| A02YYUW #2 RX | GPIO36 | 软件串口 9600 baud，板载完全独立引脚；通过 `SENSOR_HUB_A02YYUW2_RX_GPIO` 可改 |
| A02YYUW #2 TX | -1（不接） | 同上，通过 `SENSOR_HUB_A02YYUW2_TX_GPIO` 可改 |
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

## 双 A02YYUW 超声波（两路 UART，引脚不冲突）

本项目支持**两个 A02YYUW 超声波同时工作**，每个传感器独占一路 UART、一个 RX 引脚：

- **超声波 #1 → 硬件 UART1，RX=IO35**
- **超声波 #2 → 软件 UART（GPIO 位检测），RX=IO36**

为什么一路硬件一路软件？ESP32-S3 除去作控制台的 UART0，只剩 UART1、UART2 两个硬件 UART。把第二个超声波放到软件 UART，可以把宝贵的 UART2 让给高波特率传感器（如 RPLIDAR 460800 baud），同时仍满足「两个超声波 = 两路独立 UART」。软件 UART 用 `esp_timer` 高精度定时 + GPIO 边沿中断在位中心采样，对 9600 baud / 4 字节帧完全够用。两路软件 UART 也可并存（各自独立的定时器与 GPIO 中断），需要时把 #1 也切到软件 UART 即可。

A02YYUW 为自主输出模式，上电后持续从其 TX 脚发送 4 字节距离帧，**ESP 端只需接 RX**，因此每个传感器只占用一个引脚，TX 默认置 -1 不接。

| A02YYUW #1 | ESP32-S3 | 说明 |
|---|---|---|
| VCC | 3.3V 或 5V | 按模块实物标称供电 |
| GND | GND | 必须与 ESP32 共地 |
| TX（模块输出） | **IO35**（ESP RX） | 硬件 UART1 输入 |
| RX（模块输入） | 不接 | A02YYUW 自主输出，无需控制 |

| A02YYUW #2 | ESP32-S3 | 说明 |
|---|---|---|
| VCC | 3.3V 或 5V | 按模块实物标称供电 |
| GND | GND | 必须与 ESP32 共地 |
| TX（模块输出） | **IO36**（ESP RX） | 软件 UART 输入 |
| RX（模块输入） | 不接 | A02YYUW 自主输出，无需控制 |

默认配置：
- #1：硬件 `UART1`，`9600 8N1`，`RX=IO35`，`TX=-1`
- #2：软件 UART，`9600 8N1`，`RX=IO36`，`TX=-1`

相关 Kconfig：`SENSOR_HUB_A02YYUW_*`（#1）与 `SENSOR_HUB_A02YYUW2_*`（#2），可在 `menuconfig` 中分别启用/改引脚/切换软硬件 UART。

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
