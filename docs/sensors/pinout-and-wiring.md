# 引脚与接线建议

依据 `DNESP32-S3 IO引脚分配表.xlsx`、`DNESP32S3 V1.0 硬件参考手册.pdf` 和原理图文本整理。

## 推荐默认引脚

| 用途 | 默认 GPIO | 依据与说明 |
|---|---:|---|
| IMU I2C SDA | GPIO38 | IMU 原始源码宏定义 `I2C_MASTER_SDA_IO=38`；可通过 `SENSOR_HUB_I2C_SDA_GPIO` 改 |
| IMU I2C SCL | GPIO37 | IMU 原始源码宏定义 `I2C_MASTER_SCL_IO=37`；可通过 `SENSOR_HUB_I2C_SCL_GPIO` 改 |
| VL53L1X I2C SDA | GPIO39 | VL53L1X 原始源码宏定义 `I2C_MASTER_SDA_IO=GPIO_NUM_39`；可通过 `SENSOR_HUB_I2C_SDA_GPIO` 改 |
| VL53L1X I2C SCL | GPIO38 | VL53L1X 原始源码宏定义 `I2C_MASTER_SCL_IO=38`；可通过 `SENSOR_HUB_I2C_SCL_GPIO` 改 |
| UART RX | GPIO36 | 表中无板载外设占用，适合作为默认测试 RX |
| UART TX | GPIO37 | 表中无板载外设占用，适合作为默认测试 TX |
| FSR ADC | P3 `ADC_IN` / GPIO8 / ADC1_CH7 | 原理图 `ADC&REMOTE_OUT` 四针座 P3 明确标注 `ADC_IN IO8`，比在扩展排针里找 IO8 更适合手工接线；可通过 `SENSOR_HUB_FSR_ADC_GPIO` 与 `SENSOR_HUB_FSR_ADC_CHANNEL` 改成其它 ADC 输入 |

## A02YYUW UART 超声波

| A02YYUW | ESP32-S3 |
|---|---|
| VCC | 3.3V 或 5V，优先按模块实物标称供电 |
| GND | GND |
| TX | GPIO36，ESP32 RX |
| RX | GPIO37，ESP32 TX；也可悬空/拉高保持稳定输出模式 |

默认配置：`UART1`、`9600 8N1`、`RX=GPIO36`、`TX=GPIO37`。

## BU03/BU04 UWB

| BU03/BU04 UART2 | ESP32-S3 |
|---|---|
| PA2 / TX | GPIO36，ESP32 RX |
| PA3 / RX | GPIO37，ESP32 TX |
| GND | GND |
| VCC | 按 BU03/BU04 开发板/模块标称供电 |

注意：Ai-Thinker 文档区分 AT 命令 UART 和测距输出 UART。当前测试程序按 `blink/main/2.c` 的原始思路监听 BU04 基站 UART2 输出，实测 PDOA/TWR 行格式为 `JSxxxx{"TWR": {...}}`，驱动已解析 `D`、`Xcm`、`Ycm` 等字段。

## RPLIDAR C1 UART 激光雷达

| RPLIDAR C1 | ESP32-S3 |
|---|---|
| VCC / 5V | 5V 电源，建议使用能提供 800mA 启动电流的独立 5V |
| GND | GND，必须与 ESP32 共地 |
| TX | GPIO36，ESP32 RX |
| RX | GPIO37，ESP32 TX |

默认配置：`UART2`、`460800 8N1`、`RX=GPIO36`、`TX=GPIO37`。

注意：RPLIDAR C1 官方资料标称 5V 供电，启动电流较高；若板载 5V 或 USB 供电不足，可能会出现无法启动、转速不足或测距不准。原始源码使用 `GPIO17/GPIO18`，但这两个引脚在开发板表中关联 LCD/摄像头资源，本轮测试不作为默认引脚。

## I2C 模块：VL53L1X 与 IMU

| IMU | ESP32-S3 |
|---|---|
| SDA | GPIO38 |
| SCL | GPIO37 |
| VCC | 3.3V |
| GND | GND |

| VL53L1X | ESP32-S3 |
|---|---|
| SDA | GPIO39 |
| SCL | GPIO38 |
| VCC | 3.3V |
| GND | GND |

VL53L1X 默认 8 位地址 `0x52`，等价 7 位地址 `0x29`。IMU 当前代码默认 7 位地址 `0x23`。

## FSR 薄膜压力传感器

优先接开发板原理图中 `ADC&REMOTE_OUT` 四针座 P3 的 `ADC_IN`。原理图对应关系为：`P3-4=GND`、`P3-3=REMOTE_OUT`、`P3-2=ADC_IN/IO8`、`P3-1=VCC3.3`。

FSR 必须与固定电阻组成分压后接 ADC：

```text
3.3V(P3-1) -- FSR -- ADC_IN(P3-2 / IO8) -- RM -- GND(P3-4)
```

当前样例沿用原始公式 `U = 0.0004 * F + 0.0749`，驱动中转换为 `F = (U - 0.0749) / 0.0004`，并限幅到 0-6kg。这个公式必须用你的实物重新标定后才能作为算法输入。

## 避免默认冲突

- GPIO17/GPIO18 在板卡表中关联 LCD/摄像头资源，本轮不作为默认 UART 测试引脚。
- 当前 I2C 测试引脚由 `SENSOR_HUB_I2C_SDA_GPIO` / `SENSOR_HUB_I2C_SCL_GPIO` 控制。IMU 原始源码默认 GPIO38/GPIO37；VL53L1X 原始源码默认 GPIO39/GPIO38。
- P3 的 `ADC_IN` 对应 GPIO8，同时在主芯片页复用 `LCD_G5`；当前单独测试 FSR 时不启用 LCD/红外输出。若后续要同时使用 LCD/红外，需要重新选择 ADC 输入并同步修改 `SENSOR_HUB_FSR_ADC_CHANNEL`。
