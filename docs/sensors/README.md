# Autobox 传感器驱动整理说明

本目录说明 ESP-IDF 6.1 下整理后的传感器驱动接口、默认接线、测试流程与审查结论。驱动代码放在 `components/sensors/`，每个模块的 `.c` 和 `.h` 放在同一目录，可被同一个 ESP-IDF 工程通过 component 方式引入。

## 目录结构

| 模块 | 目录 | 接口状态 |
|---|---|---|
| A02YYUW UART 超声波 | `components/sensors/a02yyuw` | 可初始化 UART、解析帧、读取距离 mm |
| BU03/BU04 UWB | `components/sensors/bu_uwb` | 可发 AT 命令、读取行、解析 PDOA/TWR 坐标与距离 |
| 薄膜压力/FSR | `components/sensors/fsr_adc` | 可读 ADC、按线性标定估算质量 |
| RPLIDAR C1 | `components/sensors/rplidar_c1` | 可初始化 UART、获取信息、启动扫描、读点云 |
| 自定义 I2C IMU | `components/sensors/imu_i2c` | 可读加速度、角速度、磁场、四元数、欧拉角、气压 |
| VL53L1X ToF | `components/sensors/vl53l1x_tof` | 集成 ST ULD，可初始化和单次读距离 |

## 统一使用方式

每个模块遵循同一模式：

```c
sensor_config_t cfg = sensor_default_config(...);
sensor_init(&handle_or_cfg, &cfg);
sensor_read(&reading);
sensor_deinit(...);
```

A02YYUW、BU UWB、FSR 这类单实例模块用内部静态状态。RPLIDAR、IMU、VL53L1X 使用显式 handle，便于同一程序里管理多个状态对象。

详细示例见 [驱动接入与 API 使用](driver-usage.md)。

## 测试工程

测试工程位于 `examples/sensor_hub`，默认只启用 A02YYUW：

```bash
cd /home/sp/auto-box/sensor-driver/examples/sensor_hub
source /home/sp/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p <串口> flash monitor
```

打开 `menuconfig` 后进入 `Autobox sensor hub test`，可启用其它模块并修改 UART 引脚、波特率、I2C 地址、I2C 速率和 VL53L1X 时序参数。

## 主机侧测试

无需接开发板即可验证纯解析逻辑：

```bash
cd /home/sp/auto-box/sensor-driver
bash tests/protocol/run_tests.sh
```

当前覆盖：

- A02YYUW 4 字节 UART 帧校验与距离解析。
- BU03/BU04 `distance: 0.340000` 行解析与 `JSxxxx{"TWR": {...}}` PDOA/TWR 行解析。
- FSR 线性电压到重量换算。

## 实测状态

| 模块 | 实测状态 |
|---|---|
| A02YYUW | 通过，GPIO36 接模块 TX，可稳定输出 mm 距离 |
| BU03/BU04 | 通过，BU04 基站 UART2 输出 PDOA/TWR 行，驱动可解析 `D/Xcm/Ycm` |
| FSR | 通过，P3 `ADC_IN/IO8` 可读电压变化；质量换算需重新标定 |
| VL53L1X | 通过，按原始源码 SDA/SCL 引脚测试 |
| IMU | 通过，地址 `0x23` 可读加速度与欧拉角 |
| RPLIDAR C1 | 通过，5V 稳定供电并旋转后可读角度/距离/质量点 |
