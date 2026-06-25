# 传感器整合修改 (引脚冲突修复版)

`传感器整合` 分支的改进版本，解决了 GPIO 引脚冲突，新增软件 UART 驱动，目标是六传感器可同时运行。

> **这个分支是什么**：`传感器整合` 的「第一版修复」。把撞在 GPIO36/37 的串口传感器拆到互不冲突的引脚，并为 A02YYUW 写了软件串口以省出硬件 UART。
>
> ⚠️ **但本分支仍有两个已知缺陷**（见下「本分支已知问题」），**实际烧录请用 `传感器修改3`/`传感器修改4`**；本分支主要用于理解「怎么从冲突版改到不冲突版」。

---

## 🚧 本分支已知问题（重要，先看）

1. **可能编译不过**：`components/sensors/a02yyuw/sw_uart.c` 用了 `memset` 但**没 `#include <string.h>`**；其 `CMakeLists.txt` 的 `REQUIRES` 只有 `esp_driver_uart esp_timer`，**少了 `esp_driver_gpio`**（用到了 `driver/gpio.h`）。这两处在 `传感器修改3` 才补上。本分支构建可能报隐式声明/找不到头文件。
2. **软件 UART 采样时序有 bug**：`sw_uart.c` 每个位只前进**半个位周期**（`half_bit_us`，基于 `esp_timer`），导致数据位采样点错位、**A02YYUW 收到的字节会错乱**。此问题在 `传感器修改2` 修复（改为起始位中心对齐后按整位周期采样）。

---

## 相比原版的改进

### 引脚冲突修复（以实际 Kconfig 默认值为准）

原版 GPIO36/37 被 3–4 个传感器共享。本分支拆开如下：

| 传感器 | 接口 | 原版引脚 | 本分支修复后（Kconfig 默认值） |
|--------|------|----------|--------------------------------|
| A02YYUW | **软件 UART** | UART1 RX=36 | RX=GPIO4, TX=GPIO5 |
| BU UWB | UART1 | RX=36, TX=37 | RX=GPIO6, TX=GPIO7 |
| FSR | ADC | GPIO36 | GPIO8 (ADC1_CH7) |
| RPLIDAR | UART2 | RX=36, TX=37 | RX=GPIO17, TX=GPIO18 |
| IMU | I2C0 | SCL=37, SDA=38 | SCL=GPIO12, SDA=GPIO11 |
| VL53L1X | I2C0（共享） | 共用 IMU 总线 | SCL=GPIO12, SDA=GPIO11（共存） |

> ⚠️ 旧 README 这里曾写成 `SW_RX=38 / BU=13,14 / FSR=15`，那是更早一版的草稿值，**与本分支真实 Kconfig 不符**。请以上表 / `examples/sensor_hub/main/Kconfig.projbuild` / `docs/sensors/pinout-and-wiring.md` 为准（三者已一致：A02=4/5、BU=6/7、FSR=8、I2C=11/12、RPLIDAR=17/18）。

### 新增软件 UART 驱动

为 A02YYUW 实现 GPIO 软件串口（`components/sensors/a02yyuw/sw_uart.c`）：
- 基于 **`esp_timer`** 高精度定时器 + GPIO 边沿中断
- 接收模式，1024 字节环形缓冲，针对 A02YYUW 的 9600 baud
- 默认通过 `CONFIG_SENSOR_HUB_A02YYUW_USE_SW_UART=y` 启用；关掉则回退硬件 UART
- ⚠️ 本分支的采样时序见上「已知问题 2」。

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

> 默认 menuconfig 里**只有 A02YYUW 默认启用**（且走软件 UART），其余传感器默认关闭，需要时自行勾选。

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # 选择要启用的传感器
idf.py build        # 若报 memset/gpio.h 相关错误，见「本分支已知问题1」或换 传感器修改3
idf.py flash monitor
```

---

## 测试（PC 端，无需硬件）

```bash
bash tests/protocol/run_tests.sh
```

> ⚠️ 旧 README 写的 `python a02yyuw_test.py` 等在本分支不存在，实际是 `tests/protocol/test_sensor_parsers.c`（gcc 编译）。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `components/sensors/a02yyuw/sw_uart.c` | 软件串口 | `half_bit_us` 与 `esp_timer_start_once` 的步进逻辑（这里是时序 bug 所在）；`#include` 列表（缺 `string.h`）。 |
| `components/sensors/a02yyuw/a02yyuw.c` | 超声波驱动 | 硬件/软件 UART 双模式选择、帧解析。 |
| `components/sensors/a02yyuw/CMakeLists.txt` | 组件依赖 | `REQUIRES` 缺 `esp_driver_gpio`（已知问题 1）。 |
| `examples/sensor_hub/main/main.c` | 统一初始化 + 主循环 | 各 `#if CONFIG_SENSOR_HUB_*_ENABLE` 块、共享 I2C、500ms 轮询、`[WAIT]` 诊断。 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 配置 | **真实引脚默认值在此**（A02=4/5…）。 |
| `docs/sensors/pinout-and-wiring.md` | 接线 | 与 Kconfig 一致的权威接线表 + 冲突解决说明。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **优先用 `传感器修改3/4`**：本分支的编译与 SW-UART 时序问题都在后续分支修好。
2. **引脚以 Kconfig/pinout 文档为准**，别照 README 早期草稿值（38/13/14/15）。
3. **FSR 力值公式 `U=0.0004F+0.0749` 未标定**，限幅 0–6kg，用前必须用实物标定。
4. **RPLIDAR 需 5V/800mA 外部供电、TX/RX 交叉**；A02YYUW 自主输出，ESP 侧只接 RX 即可。
5. **IMU 0x23 / VL53L1X 0x29(7位) 共享一条 I2C0(11/12)**，地址不冲突。
6. **工作目录是 `examples/sensor_hub`**。

---

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式）
│   │   ├── sw_uart.c/h         # 软件 UART（esp_timer 位时序；本分支有时序 bug）
│   │   └── CMakeLists.txt      # REQUIRES 缺 esp_driver_gpio（本分支）
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动（需标定）
│   ├── imu_i2c/               # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/            # RPLIDAR C1 驱动
│   └── vl53l1x_tof/           # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c             # 传感器中心示例（默认用 SW UART 初始化 A02YYUW）
│       ├── Kconfig.projbuild  # menuconfig 配置菜单（真实引脚默认值）
│       └── sdkconfig.defaults # 默认引脚配置
├── docs/sensors/
│   └── pinout-and-wiring.md   # 引脚接线图 + 冲突解决表（与 Kconfig 一致）
├── tests/protocol/            # PC 端协议测试（run_tests.sh + .c）
└── AGENTS.md
```
