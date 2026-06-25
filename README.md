# 传感器整合修改 (引脚冲突修复版)

`传感器整合` 分支的改进版本，解决了所有 GPIO 引脚冲突，新增软件 UART 驱动，六传感器可同时运行。

## 智能跟随与避障（算法2 · 闭环版）

在传感器驱动之上新增了一套**跟随 + 避障**算法，把 UWB（跟随目标）、激光雷达
（前向障碍）、双超声波（近距兜底）、IMU、差速底盘整合为一台智能跟随行李箱。

> **算法2 相比算法1**：底盘从「H 桥 + 开环占空比」改成**你真实的硬件 —— APO-DL 电调 +
> RC 航模 PWM 驱动**，并用 **AB 正交编码器做轮速 PID 闭环**、用 **IMU 做航向闭环**，
> 指令 `(v, ω)` 会被实际跟踪而非开环估计。上层「跟随 + VFH 避障 + 状态机」沿用算法1。

- `components/control/chassis/`：**闭环**后轴差速底盘——电调 RC-PWM 输出 + AB 编码器轮速 PID +
  前馈 + 失效保护 + 里程计；`(v, ω)` → `chassis_set_velocity()` / `chassis_update()`。
- `components/control/follow_avoid/`：纯 C 算法，UWB 跟随 + VFH-lite 避障 + 速度调速器 +
  超声波急停 + 状态机（IDLE/SEARCH/FOLLOW/AVOID/ESTOP），可在 PC 上单元测试。
- `examples/follow_robot/`：整机示例，各传感器独立任务 + 50 Hz 控制循环 + IMU 航向闭环。
- 设计、坐标系、接线与调参详见 **[docs/algorithm/follow-and-avoidance.md](docs/algorithm/follow-and-avoidance.md)**。

```bash
cd examples/follow_robot
idf.py set-target esp32s3
idf.py menuconfig          # 配置电调/编码器引脚、TICKS_PER_METER、车距、避障阈值、PID
idf.py build flash monitor

bash tests/algorithm/run_tests.sh   # 算法 + 轮速 PID PC 单元测试（无需硬件）
```

## 修订记录

- **算法2**：跟随+避障**闭环版**。底盘改对接真实硬件 **APO-DL 电调（RC PWM，50 Hz/1500 µs 中位）+ AB 正交编码器**，
  新增**每轮速度 PID 闭环**（前馈 + 抗积分饱和 + 测量微分 + 斜率限幅 + 失效保护 + 里程计），控制循环新增 **IMU 航向闭环**
  （算法 ω 作前馈、IMU 偏航误差修正）。上层 VFH 跟随避障与状态机沿用算法1，并补充轮速 PID 的 PC 单元测试。
  引脚与 `动力轮代码` 对齐：电调 GPIO4/5、编码器 GPIO6/7 与 GPIO15/16。
- **算法1**：在传感器之上首次实现跟随 + VFH-lite 避障 + 状态机（开环 H 桥底盘假设）。
- **传感器修改4**：新增**第二个 A02YYUW 超声波**支持，两个超声波各占一路独立 UART、互不冲突。
  - 依据 `DNESP32-S3 IO引脚分配表.xlsx` 核对 ATK-DNESP32S3 引脚：全板**只有 IO35 / IO36 / IO37 为「完全独立」**可用引脚，其余均被 RGB-LCD / 摄像头 / I2S / SPI2 / IIC0 / USB / UART0 占用。两个超声波改用这组真正空闲的引脚：**#1 = IO35，#2 = IO36**（IO37 预留）。
  - `a02yyuw` 驱动重构为**句柄式多实例 API**（`a02yyuw_init_dev/read_dev/deinit_dev`），可同时驱动多个超声波；保留原单实例 API 作向后兼容。
  - 遵循「不复用引脚、引脚/硬件不够就用软件 IO 模拟」原则：**#1 走硬件 UART1，#2 走软件 UART（GPIO 位检测）**，把 UART2 留给高波特率传感器。A02YYUW 为自主输出，ESP 端仅需 RX，TX 默认 -1 不接，每个传感器仅占 1 个引脚。
- **传感器修改3**：在 ESP32-S3 / ESP-IDF v5.4 实际编译中发现并修复 `a02yyuw` 组件的两处编译问题——`sw_uart.c` 补充 `#include <string.h>`（使用了 `memset`）；`CMakeLists.txt` 的 `REQUIRES` 补上 `esp_driver_gpio`（依赖 `driver/gpio.h`）。修复后该组件可干净通过编译。
- **传感器修改2**：修复软件 UART 采样时序错误——原实现每位只前进半个位周期，导致数据位采样点错位、收到字节错乱；改为起始位中心对齐后按整位周期采样。同时校正 README 引脚表与测试说明。

## 相比原版的改进

### 引脚冲突修复

原版存在 GPIO36/37 被 3-4 个传感器共享的问题。此版本完全解决了所有冲突：

下表中的“修复后引脚”与 `examples/sensor_hub/main/Kconfig.projbuild` 的默认值保持一致，可在 `menuconfig` 中修改：

| 传感器 | 接口 | 原版引脚 | 修复后引脚（Kconfig 默认值） |
|--------|------|----------|------------|
| A02YYUW #1 | HW UART1 | GPIO37(RX) | RX=GPIO35（板载完全独立引脚） |
| A02YYUW #2 | SW UART | 无 | RX=GPIO36（板载完全独立引脚） |
| BU UWB | UART1 | GPIO36(RX), GPIO37(TX) | RX=GPIO6, TX=GPIO7 |
| FSR | ADC | GPIO36 | GPIO8 (ADC1_CH7) |
| RPLIDAR | UART2 | GPIO17(TX), GPIO18(RX) | ESP_RX=GPIO17, ESP_TX=GPIO18 |
| IMU | I2C | SCL=42, SDA=41 | SCL=12, SDA=11 |
| VL53L1X | I2C | 共用 IMU 总线 | SCL=12, SDA=11 (共存) |

### 新增软件 UART 驱动

为 A02YYUW 超声波传感器实现了 GPIO 软件串口（`components/sensors/a02yyuw/sw_uart.c`）：
- 基于 `esp_timer` 高精度定时器和 GPIO 边沿中断
- 支持接收模式，最高 115200 波特率（针对 A02YYUW 的 9600 baud 优化）
- 起始位下降沿检测：先用半位延时对齐到起始位中心，其后每个数据位按整位周期在位中心采样
- 1024 字节环形缓冲区

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 |
|--------|------|------|------|
| A02YYUW #1 | 超声波测距 | HW UART1 (RX=IO35) | 毫米级距离测量（最大 4.5m） |
| A02YYUW #2 | 超声波测距 | SW UART (RX=IO36) | 毫米级距离测量（最大 4.5m），软件串口 |
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

> 各传感器组件已在 ESP-IDF v5.4 + ESP32-S3 上编译验证通过。

## 测试

协议解析层（A02YYUW / BU UWB / FSR）提供 PC 端单元测试，使用 gcc 编译运行，无需硬件：

```bash
bash tests/protocol/run_tests.sh
```

测试源码见 `tests/protocol/test_sensor_parsers.c`。

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式 + 句柄式多实例）
│   │   ├── sw_uart.c/h         # 软件 UART 驱动（GPIO 位时序）
│   │   └── CMakeLists.txt      # 含 esp_timer / esp_driver_gpio / esp_driver_uart 依赖
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动
│   ├── imu_i2c/                # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/             # RPLIDAR C1 驱动
│   └── vl53l1x_tof/            # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # 传感器中心示例（初始化两个 A02YYUW：#1 硬件UART1 / #2 软件UART）
│       ├── Kconfig.projbuild   # menuconfig 配置菜单
│       └── sdkconfig.defaults  # 默认引脚配置
├── docs/sensors/
│   └── pinout-and-wiring.md    # 完整引脚接线图 + 冲突解决表
├── tests/protocol/             # PC 端协议测试
└── AGENTS.md
```
