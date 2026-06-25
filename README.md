# 传感器整合修改 (引脚冲突修复版)

`传感器整合` 分支的改进版本，解决了所有 GPIO 引脚冲突，新增软件 UART 驱动，六传感器可同时运行。

> **这个分支是什么（传感器修改3）**：在 `传感器修改2`（修好时序）基础上**补齐编译问题**，是**第一个能在 ESP-IDF v5.4 + ESP32-S3 上干净编译通过**的六传感器版本。如果你想要「单超声波 + 能直接 build 烧录」的稳定底座，用本分支即可；需要「双超声波」再上 `传感器修改4`。

---

## 修订记录

- **传感器修改3**：在 ESP32-S3 / ESP-IDF v5.4 实际编译中发现并修复 `a02yyuw` 组件的两处编译问题——`sw_uart.c` 补充 `#include <string.h>`（使用了 `memset`）；`CMakeLists.txt` 的 `REQUIRES` 补上 `esp_driver_gpio`（依赖 `driver/gpio.h`）。修复后该组件可干净通过编译。
- **传感器修改2**：修复软件 UART 采样时序错误——原实现每位只前进半个位周期，导致数据位采样点错位、收到字节错乱；改为起始位中心对齐后按整位周期采样。同时校正 README 引脚表与测试说明。

---

## 相比原版的改进

### 引脚冲突修复

原版存在 GPIO36/37 被 3-4 个传感器共享的问题。此版本完全解决了所有冲突。下表“修复后引脚”与 `examples/sensor_hub/main/Kconfig.projbuild` 默认值一致，可在 `menuconfig` 修改：

| 传感器 | 接口 | 原版引脚 | 修复后引脚（Kconfig 默认值） |
|--------|------|----------|------------|
| A02YYUW | SW UART | GPIO37(RX) | SW_RX=GPIO4（TX=GPIO5 可不接） |
| BU UWB | UART1 | GPIO36(RX), GPIO37(TX) | RX=GPIO6, TX=GPIO7 |
| FSR | ADC | GPIO36 | GPIO8 (ADC1_CH7) |
| RPLIDAR | UART2 | GPIO17/18 | ESP_RX=GPIO17, ESP_TX=GPIO18 |
| IMU | I2C0 | SCL=37, SDA=38 | SCL=12, SDA=11 |
| VL53L1X | I2C0 | 共用 IMU 总线 | SCL=12, SDA=11 (共存) |

### 软件 UART 驱动

A02YYUW 的 GPIO 软件串口（`components/sensors/a02yyuw/sw_uart.c`）：
- 基于 `esp_timer` 高精度定时器 + GPIO 边沿中断
- 起始位下降沿后先半位对齐到位中心，其后每个数据位按整位周期在位中心采样（修改2 修正）
- 1024 字节环形缓冲，针对 A02YYUW 9600 baud
- **本分支已补 `#include <string.h>` 和 `REQUIRES esp_driver_gpio`，可干净编译**

---

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 |
|--------|------|------|------|
| A02YYUW | 超声波测距 | SW UART (RX=GPIO4) | 毫米级（最大 4.5m），**单实例** |
| BU03/BU04 | UWB 超宽带 | UART1 (6/7) | 室内定位（PDOA / TWR 监听） |
| FSR | 薄膜压力 | ADC1 (GPIO8) | 压力（需标定） |
| RPLIDAR C1 | 激光雷达 | UART2 (17/18) | 360° 二维扫描 |
| IMU | 九轴惯性 | I2C0 (11/12) | 加速度(g)/陀螺/磁力/欧拉角/气压 |
| VL53L1X | ToF 激光测距 | I2C0 共享 (11/12) | 毫米级（最大 4m） |

> A02YYUW 仍是单实例 API（`a02yyuw_init` / `a02yyuw_read`）。需要同时接**两个**超声波要用 `传感器修改4` 的句柄式多实例 API。

---

## 特性

- **无引脚冲突 + 可干净编译**: 已在 ESP-IDF v5.4 + ESP32-S3 验证通过
- **软件 UART**: 释放硬件 UART 给其它传感器
- **menuconfig 配置**: Kconfig 菜单启用/禁用 + 配引脚
- **完整引脚文档**: `docs/sensors/pinout-and-wiring.md`

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # 选择要启用的传感器
idf.py build        # 本分支应可一次编译通过
idf.py flash monitor
```

> 各传感器组件已在 ESP-IDF v5.4 + ESP32-S3 上编译验证通过。

---

## 测试（PC 端，无需硬件）

```bash
bash tests/protocol/run_tests.sh
```

测试源码见 `tests/protocol/test_sensor_parsers.c`。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `components/sensors/a02yyuw/sw_uart.c` | 软件串口 | 顶部 `#include <string.h>`（本分支补的）；位时序（修改2 修正）。 |
| `components/sensors/a02yyuw/CMakeLists.txt` | 组件依赖 | `REQUIRES esp_driver_uart esp_driver_gpio esp_timer`（本分支补的 gpio）。 |
| `components/sensors/a02yyuw/a02yyuw.h` | A02 API | 单实例 `a02yyuw_init`/`a02yyuw_read`（对比 修改4 的多实例）。 |
| `examples/sensor_hub/main/main.c` | 统一初始化 + 主循环 | 各 `#if CONFIG_SENSOR_HUB_*_ENABLE` 块、共享 I2C、500ms 轮询、`[WAIT]` 诊断。 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 配置 | 引脚默认值。 |
| `docs/sensors/pinout-and-wiring.md` | 接线 | 权威接线表 + 冲突解决说明。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **这是「能直接编译烧录」的稳定单超声波版**；要双超声波用 `传感器修改4`。
2. **A02YYUW 是单实例**，重复调用 `a02yyuw_init` 不能驱动两个传感器。
3. **FSR 公式 `U=0.0004F+0.0749` 未标定**，限幅 0–6kg，用前必须标定。
4. **RPLIDAR 需 5V/800mA 外部供电、TX/RX 交叉**；A02YYUW 自主输出，ESP 侧只接 RX。
5. **IMU 0x23 / VL53L1X 0x29(7位) 共享 I2C0(11/12)**，地址不冲突。
6. **工作目录是 `examples/sensor_hub`**；测试命令是 `run_tests.sh`（非 python）。

---

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式，单实例）
│   │   ├── sw_uart.c/h         # 软件 UART（esp_timer 位时序，已补 string.h）
│   │   └── CMakeLists.txt      # REQUIRES 含 esp_timer / esp_driver_gpio / esp_driver_uart
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
