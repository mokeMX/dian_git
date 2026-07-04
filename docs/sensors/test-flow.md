# 传感器测试流程

测试工程：`/home/sp/auto-box/sensor-driver/examples/sensor_hub`

## 通用准备

```bash
cd /home/sp/auto-box/sensor-driver/examples/sensor_hub
source /home/sp/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig
```

进入 `Autobox sensor hub test`：

- 每次建议只启用一个待测传感器，避免 GPIO、UART 或 I2C 总线冲突。
- UART 模块可配置 UART 号、RX/TX GPIO 和波特率。
- I2C 模块可配置 SDA/SCL、I2C 速率、IMU 地址和 VL53L1X 地址/时序。
- FSR 可配置 ADC GPIO 说明值和 ADC1 channel。

构建、烧录、监视：

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

退出 monitor：`Ctrl+]`。

## A02YYUW

`menuconfig`：

- `SENSOR_HUB_A02YYUW_ENABLE=y`
- `UART=1`
- `RX_GPIO=36`
- `TX_GPIO=37`
- `BAUDRATE=9600`

接线：

```text
A02YYUW VCC -> 3.3V 或模块标称电源
A02YYUW GND -> GND
A02YYUW TX  -> ESP32 GPIO36
A02YYUW RX  -> ESP32 GPIO37 或悬空
```

期望输出：

```text
[A02YYUW] distance=360 mm
```

## BU03/BU04 UWB

`menuconfig`：

- `SENSOR_HUB_BU_UWB_ENABLE=y`
- `UART=1`
- `RX_GPIO=36`
- `TX_GPIO=37`
- `BAUDRATE=115200`

接线按 BU04 基站 UART2：

```text
BU04 PA2/TX -> ESP32 GPIO36
BU04 PA3/RX -> ESP32 GPIO37
BU04 GND    -> ESP32 GND
BU04        -> 独立供电
BU03 标签   -> 独立供电并加入基站网络
```

期望输出：

```text
[BU_UWB][TWR] frame=JS006F anchor=1081 D=16cm Xcm=-16 Ycm=10 ...
```

## FSR 薄膜压力

`menuconfig`：

- `SENSOR_HUB_FSR_ENABLE=y`
- `FSR_ADC_GPIO=8`
- `FSR_ADC_CHANNEL=7`

接线：

```text
P3-1 VCC3.3 -> FSR 一端
FSR 另一端  -> P3-2 ADC_IN/IO8
P3-2 ADC_IN -> 固定电阻 RM -> P3-4 GND
```

期望输出：

```text
[FSR] raw=120 voltage=0.097V force_est_kg=6.00
```

注意：`force_est_kg` 使用原始源码线性公式估算，只能作为趋势参考；要做实时重量显示必须用砝码重新标定。

## VL53L1X ToF

`menuconfig`：

- `SENSOR_HUB_VL53L1X_ENABLE=y`
- `I2C_SDA_GPIO=39`
- `I2C_SCL_GPIO=38`
- `I2C_SPEED_HZ=400000`
- `VL53L1X_ADDR_8BIT=0x52`

接线：

```text
VL53L1X VCC -> 3.3V
VL53L1X GND -> GND
VL53L1X SDA -> GPIO39
VL53L1X SCL -> GPIO38
```

期望输出：

```text
[VL53L1X] distance=123 mm
```

## IMU

`menuconfig`：

- `SENSOR_HUB_IMU_ENABLE=y`
- `I2C_SDA_GPIO=38`
- `I2C_SCL_GPIO=37`
- `I2C_SPEED_HZ=400000`
- `IMU_ADDR=0x23`

接线：

```text
IMU VCC -> 3.3V
IMU GND -> GND
IMU SDA -> GPIO38
IMU SCL -> GPIO37
```

期望输出：

```text
[IMU] accel=-0.072 -0.200 0.986 g euler=-11.67 4.05 6.02 deg
```

## RPLIDAR C1

`menuconfig`：

- `SENSOR_HUB_RPLIDAR_ENABLE=y`
- `UART=2`
- `RX_GPIO=36`
- `TX_GPIO=37`
- `BAUDRATE=460800`

接线：

```text
RPLIDAR 红线/VCC -> 5V 电源
RPLIDAR 黑线/GND -> ESP32 GND 和电源 GND
RPLIDAR 黄线/TX  -> ESP32 GPIO36
RPLIDAR 绿线/RX  -> ESP32 GPIO37
```

期望输出：

```text
[RPLIDAR] angle=35.14 distance=328.5 quality=40 start=0
```

注意：RPLIDAR 必须稳定 5V 供电并开始旋转后才会持续输出扫描点。只接 USB 转接板不能直接给 ESP32 提供 UART 数据；ESP32 应直接接雷达四线中的 TX/RX/GND，并共地。

## 主机侧协议测试

```bash
cd /home/sp/auto-box/sensor-driver
bash tests/protocol/run_tests.sh
```

期望输出：

```text
sensor parser tests passed
```
