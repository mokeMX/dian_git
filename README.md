# 传感器整合修改 (引脚冲突修复版)

`传感器整合` 分支的改进版本，解决了所有 GPIO 引脚冲突，新增软件 UART 驱动，六传感器可同时运行。

> **这个分支是什么（传感器修改2）**：在 `传感器整合修改` 基础上**修好软件串口的采样时序 bug**。`传感器整合修改` 的 SW-UART 每位只走半个位周期，导致 A02YYUW 字节错乱；本分支改为「起始位先用半位对齐到位中心，之后每位按整位周期采样」，A02YYUW 读数恢复正常。
>
> ⚠️ **本分支仍有一个遗留点**：`sw_uart.c` 还没补 `#include <string.h>`、组件 `REQUIRES` 还没加 `esp_driver_gpio`，**干净编译要到 `传感器修改3`**。要直接烧建议用 `传感器修改3/4`。

---

## 相比原版的改进

### 引脚冲突修复

原版存在 GPIO36/37 被 3-4 个传感器共享的问题。此版本完全解决了所有冲突。下表“修复后引脚”与 `examples/sensor_hub/main/Kconfig.projbuild` 默认值一致，可在 `menuconfig` 中修改：

| 传感器 | 接口 | 原版引脚 | 修复后引脚（Kconfig 默认值） |
|--------|------|----------|------------|
| A02YYUW | SW UART | GPIO37(RX) | SW_RX=GPIO4（TX=GPIO5，自主输出可不接） |
| BU UWB | UART1 | GPIO36(RX), GPIO37(TX) | RX=GPIO6, TX=GPIO7 |
| FSR | ADC | GPIO36 | GPIO8 (ADC1_CH7) |
| RPLIDAR | UART2 | GPIO17/18 | ESP_RX=GPIO17, ESP_TX=GPIO18 |
| IMU | I2C0 | SCL=37, SDA=38 | SCL=12, SDA=11 |
| VL53L1X | I2C0 | 共用 IMU 总线 | SCL=12, SDA=11 (共存) |

### 软件 UART 驱动（本分支重点修复）

A02YYUW 的 GPIO 软件串口（`components/sensors/a02yyuw/sw_uart.c`）：
- 基于 `esp_timer` 高精度定时器 + GPIO 边沿中断
- **采样时序（本分支已修正）**：起始位下降沿后先延半个位周期对齐到**位中心**，其后每个数据位按**整位周期**（`bit_us = 1e6/baud`）在位中心采样——不再是之前的「每位半周期」错位写法
- 1024 字节环形缓冲，针对 A02YYUW 的 9600 baud

---

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 |
|--------|------|------|------|
| A02YYUW | 超声波测距 | SW UART (RX=GPIO4) | 毫米级（最大 4.5m） |
| BU03/BU04 | UWB 超宽带 | UART1 (6/7) | 室内定位（PDOA / TWR 监听） |
| FSR | 薄膜压力 | ADC1 (GPIO8) | 压力（需标定） |
| RPLIDAR C1 | 激光雷达 | UART2 (17/18) | 360° 二维扫描 |
| IMU | 九轴惯性 | I2C0 (11/12) | 加速度(g)/陀螺/磁力/欧拉角/气压 |
| VL53L1X | ToF 激光测距 | I2C0 共享 (11/12) | 毫米级（最大 4m） |

---

## 特性

- **无引脚冲突**: 各传感器引脚独立，可同时运行
- **软件 UART**: 释放硬件 UART 给其它传感器，且采样时序已修正
- **menuconfig 配置**: Kconfig 菜单启用/禁用每个传感器并配置引脚
- **完整引脚文档**: `docs/sensors/pinout-and-wiring.md` 含接线图与冲突解决表

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # 选择要启用的传感器
idf.py build        # 若报 memset 未声明 / driver/gpio.h 找不到，见下注意点1（用 传感器修改3）
idf.py flash monitor
```

---

## 测试（PC 端，无需硬件）

协议解析层（A02YYUW / BU UWB / FSR）有纯 C 单元测试：

```bash
bash tests/protocol/run_tests.sh
```

测试源码见 `tests/protocol/test_sensor_parsers.c`。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `components/sensors/a02yyuw/sw_uart.c` | 软件串口（本分支修复点） | `bit_us`/`half_bit_us` 的计算与 `sw_uart_schedule_next(uart, uart->bit_us)`——注释明确「one full bit period away (not a half bit)」，对照看就懂时序修复。 |
| `components/sensors/a02yyuw/a02yyuw.c` | 超声波驱动 | 硬件/软件 UART 双模式、4 字节帧 + 校验和解析。 |
| `examples/sensor_hub/main/main.c` | 统一初始化 + 主循环 | 各 `#if CONFIG_SENSOR_HUB_*_ENABLE` 块、共享 I2C、500ms 轮询、`[WAIT]` 诊断。 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 配置 | 引脚默认值（A02=4/5、BU=6/7、FSR=8、I2C=11/12、RPLIDAR=17/18）。 |
| `docs/sensors/pinout-and-wiring.md` | 接线 | 权威接线表 + 冲突解决说明。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **干净编译用 `传感器修改3`**：本分支 `sw_uart.c` 仍缺 `#include <string.h>`（用了 `memset`），`CMakeLists` 的 `REQUIRES` 仍缺 `esp_driver_gpio`（用了 `driver/gpio.h`），可能报警告/错误。
2. **本分支的价值是「字节不再错乱」**：如果你在 `传感器整合修改` 上遇到 A02YYUW 数据乱跳，就是被这里修掉的时序 bug。
3. **FSR 公式 `U=0.0004F+0.0749` 未标定**，限幅 0–6kg，用前必须标定。
4. **RPLIDAR 需 5V/800mA 外部供电、TX/RX 交叉**；A02YYUW 自主输出，ESP 侧只接 RX。
5. **IMU 0x23 / VL53L1X 0x29(7位) 共享 I2C0(11/12)**，地址不冲突。
6. **工作目录是 `examples/sensor_hub`**；测试命令是 `run_tests.sh`（非 python）。

---

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式）
│   │   ├── sw_uart.c/h         # 软件 UART（esp_timer 位时序，本分支已修采样时序）
│   │   └── CMakeLists.txt      # 含 esp_timer 依赖（仍缺 esp_driver_gpio，见注意点1）
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动（需标定）
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
├── tests/protocol/             # PC 端协议测试（run_tests.sh + .c）
└── AGENTS.md
```
