# 传感器整合 (Sensor Driver Hub)

六传感器驱动统一框架项目，提供标准化的传感器驱动接口和 menuconfig 配置系统。

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 |
|--------|------|------|------|
| A02YYUW | 超声波测距 | UART | 毫米级距离测量（最大 4.5m） |
| BU03/BU04 | UWB 超宽带 | UART | 高精度室内定位（PDOA / TWR） |
| FSR | 薄膜压力 | ADC | 压力检测（模拟量转力值） |
| RPLIDAR C1 | 激光雷达 | UART | 360° 二维激光扫描 |
| IMU | 九轴惯性 | I2C | 加速度/陀螺/磁力计/欧拉角/气压 |
| VL53L1X | ToF 激光测距 | I2C | 毫米级距离测量（最大 4m） |

## 特性

- **模块化设计**: 每个传感器独立的驱动组件，位于 `components/sensors/`
- **menuconfig 配置**: 通过 Kconfig 菜单启用/禁用传感器、配置引脚
- **统一传感器中心**: `examples/sensor_hub/` 示例程序统一初始化所有已启用的传感器
- **完整文档**: `docs/sensors/` 包含驱动使用说明、引脚接线图、测试流程

## 硬件连接

详见 `docs/sensors/pinout-and-wiring.md`

> **注意**: 此分支存在引脚冲突（GPIO36/37 被多个传感器共享），建议使用 `传感器整合修改` 分支

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # 选择要启用的传感器
idf.py build
idf.py flash monitor
```

## 测试

项目包含 PC 端协议测试脚本：

```bash
cd tests/protocol
python a02yyuw_test.py       # A02YYUW 协议测试
python bu_uwb_test.py        # BU UWB 协议测试
python fsr_adc_test.py       # FSR ADC 测试
```

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/               # A02YYUW 超声波驱动
│   ├── bu_uwb/                # BU03/BU04 UWB 驱动
│   ├── fsr_adc/               # FSR 压力传感器驱动
│   ├── imu_i2c/               # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/            # RPLIDAR C1 驱动
│   └── vl53l1x_tof/           # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c             # 传感器中心示例程序
│       └── Kconfig.projbuild  # menuconfig 配置菜单
├── docs/sensors/              # 驱动使用文档
├── tests/protocol/            # PC 端协议测试
└── AGENTS.md                  # 工作区指南
```
