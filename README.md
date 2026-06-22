# 传感器整合修改 (引脚冲突修复版)

`传感器整合` 分支的改进版本，解决了所有 GPIO 引脚冲突，新增软件 UART 驱动，六传感器可同时运行。

## 相比原版的改进

### 引脚冲突修复

原版存在 GPIO36/37 被 3-4 个传感器共享的问题。此版本完全解决了所有冲突：

| 传感器 | 接口 | 原版引脚 | 修复后引脚 |
|--------|------|----------|------------|
| A02YYUW | SW UART | GPIO37(RX) | SW_RX=GPIO38 |
| BU UWB | UART1 | GPIO36(RX), GPIO37(TX) | GPIO13(RX), GPIO14(TX) |
| FSR | ADC | GPIO36 | GPIO15 |
| RPLIDAR | UART2 | GPIO17(TX), GPIO18(RX) | GPIO17(TX), GPIO18(RX) |
| IMU | I2C | SCL=42, SDA=41 | SCL=12, SDA=11 |
| VL53L1X | I2C | 共用 IMU 总线 | SCL=12, SDA=11 (共存) |

### 新增软件 UART 驱动

为 A02YYUW 超声波传感器实现了 GPIO 软件串口（`components/sensors/a02yyuw/sw_uart.c`）：
- 基于 `esp_timer` 高精度定时器和 GPIO 边沿中断
- 支持接收模式，最高 115200 波特率（针对 A02YYUW 的 9600 baud 优化）
- 起始位下降沿检测 + 半位宽中心采样
- 1024 字节环形缓冲区

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 |
|--------|------|------|------|
| A02YYUW | 超声波测距 | SW UART | 毫米级距离测量（最大 4.5m） |
| BU03/BU04 | UWB 超宽带 | UART1 | 高精度室内定位（PDOA / TWR） |
| FSR | 薄膜压力 | ADC1 | 压力检测（模拟量转力值） |
| RPLIDAR C1 | 激光雷达 | UART2 | 360° 二维激光扫描 |
| IMU | 九轴惯性 | I2C | 加速度/陀螺/磁力计/欧拉角/气压 |
| VL53L1X | ToF 激光测距 | I2C (共享) | 毫米级距离测量（最大 4m） |

## 特性

- **无引脚冲突**: 六传感器引脚完全独立，可同时运行
- **软件 UART**: 释放硬件 UART 资源给其他传感器
- **menuconfig 配置**: 可通过 Kconfig 菜单启用/禁用每个传感器并配置引脚
- **完整引脚文档**: `docs/sensors/pinout-and-wiring.md` 包含详细的接线图和冲突解决说明

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # 选择要启用的传感器
idf.py build
idf.py flash monitor
```

## 测试

```bash
cd tests/protocol
python a02yyuw_test.py       # A02YYUW 协议测试
python bu_uwb_test.py        # BU UWB 协议测试
python fsr_adc_test.py       # FSR ADC 测试
```

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（支持硬件/软件 UART 双模式）
│   │   ├── sw_uart.c/h         # 软件 UART 驱动（GPIO 位时序）
│   │   └── CMakeLists.txt      # 含 esp_timer 依赖
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动
│   ├── imu_i2c/                # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/             # RPLIDAR C1 驱动
│   └── vl53l1x_tof/            # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # 传感器中心示例（使用 SW UART 初始化 A02YYUW）
│       ├── Kconfig.projbuild   # menuconfig 配置菜单
│       └── sdkconfig.defaults  # 默认引脚配置
├── docs/sensors/
│   └── pinout-and-wiring.md    # 完整引脚接线图 + 冲突解决表
├── tests/protocol/             # PC 端协议测试
└── AGENTS.md
```
