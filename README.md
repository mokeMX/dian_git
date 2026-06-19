# Autobox Sensor Driver

ESP-IDF 6.1 / ESP32-S3 传感器驱动整理工程。当前已整理并完成单模块硬件测试的传感器：

- A02YYUW UART 超声波测距
- BU03/BU04 UWB PDOA/TWR 测距定位输出
- FSR 薄膜压力传感器 ADC 输入
- RPLIDAR C1 UART 激光雷达
- 自定义 I2C IMU 模块
- VL53L1X ToF 测距模块

驱动组件位于 `components/sensors/`，测试工程位于 `examples/sensor_hub/`。每个驱动都通过 `*_config_t` 注入 UART/I2C/ADC 参数；未来主算法工程只需要把对应 component 加入 `EXTRA_COMPONENT_DIRS` 或复制到工程 `components/` 下，即可 include 头文件并初始化使用。

## 快速构建

```bash
cd /home/sp/auto-box/sensor-driver/examples/sensor_hub
source /home/sp/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

`menuconfig` 路径：`Autobox sensor hub test`。在这里选择要测试的模块，并配置 UART 端口、GPIO、波特率、I2C 地址、I2C 速率等参数。

## 文档

- [驱动接入与 API 使用](docs/sensors/driver-usage.md)
- [引脚与接线建议](docs/sensors/pinout-and-wiring.md)
- [测试流程](docs/sensors/test-flow.md)
- [模块审查结论](docs/sensors/module-review.md)

## 验证

主机侧协议解析测试：

```bash
cd /home/sp/auto-box/sensor-driver
bash tests/protocol/run_tests.sh
```

ESP-IDF 构建：

```bash
cd /home/sp/auto-box/sensor-driver/examples/sensor_hub
source /home/sp/esp/esp-idf/export.sh
idf.py build
```
