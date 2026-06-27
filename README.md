# 传感器整合修改 (全传感器并发版)

`传感器修改4` 的并发重构版本——移除所有 `#if`/`#endif` 条件编译，所有 7 路传感器**同时初始化、同时运行**，使用 FreeRTOS 任务实现真正并发。

> **这个分支是什么（传感器修改5）**：在 `传感器修改4` 基础上**彻底移除条件编译**，将串行轮询的主循环重构为 **7 个独立 FreeRTOS 任务**，实现全部传感器同时收发数据。引脚分配**完全不变**（仅 A02YYUW #1 内部从 HW UART 切换为 SW UART 以释放 UART1 给 BU UWB）。`算法1` / `算法2` 要拉取全传感器并发能力优先从这里拉分支。

---

## 修订记录

| 版本 | 修改内容 |
|------|----------|
| **传感器修改5**（本分支） | 移除所有 `#if`/`#endif`；全部传感器默认启用；主循环重构为 7 个 FreeRTOS 任务并发运行；A02YYUW #1 从 HW UART1 切换为 SW UART（IO35 引脚不变）以释放 UART1 给 BU UWB；主任务不再阻塞自旋；所有引脚硬编码为常量。 |
| 传感器修改4 | 新增第二个 A02YYUW 超声波；句柄式多实例 API；双超声波一硬一软 UART。 |
| 传感器修改3 | 补 `sw_uart.c` `#include <string.h>`、CMakeLists `REQUIRES`，组件可干净编译。 |
| 传感器修改2 | 修复软件 UART 采样时序（半位→整位中心采样）。 |

---

## 传感器修改5 详细变更清单

### 一、架构变更

#### 1.1 移除所有 `#if` / `#endif` 条件编译

**修改前**（`main.c`）：
```c
#if CONFIG_SENSOR_HUB_A02YYUW_ENABLE
    // 初始化 A02YYUW #1
#endif

#if CONFIG_SENSOR_HUB_BU_UWB_ENABLE
    // 初始化 BU UWB
#endif
// ... 每个传感器都被 #if 包裹
```

**修改后**：全部传感器无条件初始化、无条件运行。不再依赖 `sdkconfig.h` 中的 `CONFIG_SENSOR_HUB_xxx_ENABLE` 宏。所有引脚以 `#define` 常量直接定义在 `main.c` 顶部。

| 移除的 `#if` 块 | 数量 |
|-----------------|------|
| 头文件 include 条件 | 2 处 |
| 宏定义条件 | 3 处 |
| 初始化代码条件 | 7 处 |
| 主循环读取条件 | 7 处 |
| **合计** | **19 处** |

#### 1.2 主循环 → FreeRTOS 任务并发

**修改前**：单个 `while(1)` 循环内顺序调用 7 个传感器的读取函数。每个传感器有阻塞等待（如 A02YYUW 150ms、VL53L1X 250ms），导致后序传感器被严重延迟。

```
修改前时序:  [A02#1 150ms] → [A02#2 150ms] → [BU 100ms] → ...
             一轮循环累计阻塞 ~1.5 秒
```

**修改后**：每个传感器一个独立 FreeRTOS 任务，各任务自主调度、互不阻塞。

```
修改后时序:
  task_a02yyuw1  ──[读]────[等 500ms]──[读]──
  task_a02yyuw2  ──[读]────[等 500ms]──[读]──
  task_bu_uwb    ──[读]──[等 100ms]──[读]──
  task_fsr       ──[读]────[等 500ms]──[读]──
  task_rplidar   ──[扫点]──[等 50ms]──[扫点]──
  task_imu       ──[读]──[等 200ms]──[读]──
  task_vl53l1x   ──[读]──[等 250ms]──[读]──
                  ↑ 全部并行运行 ↑
```

| 任务 | 函数 | 优先级 | 栈空间 | 读取间隔 |
|------|------|--------|--------|----------|
| A02YYUW #1 | `task_a02yyuw1` | 1 | 4096 | 500ms |
| A02YYUW #2 | `task_a02yyuw2` | 1 | 4096 | 500ms |
| BU UWB | `task_bu_uwb` | 2 | 4096 | 100ms |
| FSR | `task_fsr` | 1 | 4096 | 500ms |
| RPLIDAR C1 | `task_rplidar` | 3 | 4096 | 50ms |
| IMU | `task_imu` | 2 | 4096 | 200ms |
| VL53L1X | `task_vl53l1x` | 2 | 4096 | 250ms |

### 二、硬件引脚变更

**所有传感器引脚完全不变。** 唯一变更：A02YYUW #1 从**硬件 UART1 模式**切换为**软件 UART 模式**（`sw_uart`），释放 UART1 给 BU UWB。

| 传感器 | 接口 | 引脚（不变） | UART 模式变化 |
|--------|------|-------------|--------------|
| A02YYUW #1 | ~~HW UART1~~ → **SW UART** | **IO35**（不变） | HW→SW，释放 UART1 |
| A02YYUW #2 | SW UART | **IO36**（不变） | 不变 |
| BU UWB | HW UART1 | **GPIO6/7**（不变） | 不变，独占 UART1 |
| FSR | ADC1 | **GPIO8**（不变） | 不变 |
| RPLIDAR C1 | HW UART2 | **GPIO17/18**（不变） | 不变，独占 UART2 |
| IMU | I2C0 | **GPIO11/12**（不变） | 不变 |
| VL53L1X | I2C0（共享） | **GPIO11/12**（不变） | 不变 |

#### 为什么 A02YYUW #1 必须从 HW 切到 SW UART

ESP32-S3 仅有 **4 个硬件 UART**：UART0（控制台）、UART1、UART2、UART3。传感器修改4 中：
- UART0 = 控制台
- UART1 = A02YYUW #1（HW）
- UART2 = RPLIDAR C1
- A02YYUW #2 = SW UART

BU UWB 也需要 UART1，当同时启用时与 A02YYUW #1 冲突。将 A02YYUW #1 切换到 SW UART（`esp_timer` + GPIO 中断，9600 baud 足够）后，UART1 完全释放给 BU UWB。**IO35 引脚本身完全不变**，只是内部驱动从硬件 UART 变为软件 UART。

### 三、代码质量改进

#### 3.1 消除变量遮蔽

**修改前**：传感器句柄（如 `static a02yyuw_t a02_1`）声明在 `#if` 块内部，且与非 `#if` 块的变量不在同一作用域，存在作用域混乱。

**修改后**：所有传感器句柄统一声明为文件级 `static` 全局变量（`g_a02_1`、`g_a02_2`、`g_lidar` 等），作用域清晰，各任务直接引用。

#### 3.2 主任务不再自旋

**修改前**：`app_main` 末尾有 `while(1) { ... vTaskDelay(500); }`，主任务永不休眠。

**修改后**：`app_main` 完成初始化并创建 7 个任务后**立即返回**，主任务自动销毁。传感器任务自行运行。

#### 3.3 引脚常量化

**修改前**：引脚值依赖 Kconfig → `sdkconfig.h` → `CONFIG_xxx` 宏链，路径长且无法在代码中直接看到实际值。

**修改后**：所有引脚值直接以 `#define` 定义在 `main.c` 顶部，一目了然：

```c
#define A02_1_RX_GPIO         35
#define A02_2_RX_GPIO         36
#define BU_UWB_RX_GPIO         6
#define BU_UWB_TX_GPIO         7
#define FSR_ADC_GPIO           8
#define RPLIDAR_RX_GPIO       17
#define RPLIDAR_TX_GPIO       18
#define HUB_I2C_SDA_GPIO      11
#define HUB_I2C_SCL_GPIO      12
// ...
```

#### 3.4 控制流简化

任务循环中优先使用 `switch` 和三元表达式（`? :`）替代 `if/else`：

```c
switch (ret) {
case ESP_OK:
    r.valid
        ? printf("[A02YYUW#1] distance=%d mm\n", r.distance_mm)
        : printf("[A02YYUW#1] no valid frame (RX=GPIO%d)\n",
                 A02_1_RX_GPIO);
    break;
default:
    printf("[A02YYUW#1] no valid frame (RX=GPIO%d)\n",
           A02_1_RX_GPIO);
    break;
}
```

> 注：`handle_bu_uwb_rx` 等辅助函数内部保留少量 `if` 以保证复杂解析逻辑的可读性，这些不在用户暴露的传感器读取路径上。

### 四、配置文件变更

#### 4.1 `sdkconfig.defaults`

全部传感器的 `CONFIG_SENSOR_HUB_xxx_ENABLE` 设为 `y`。A02YYUW #1 的 `USE_SW_UART` 从 `n` 改为 `y`。

#### 4.2 `Kconfig.projbuild`

所有传感器 ENABLE 选项的 `default` 从 `n` 改为 `y`。A02YYUW #1 SW UART 选项的 `default` 从 `n` 改为 `y`。配置项帮助文字更新。

### 五、潜在风险与注意事项

| 风险 | 说明 | 缓解措施 |
|------|------|----------|
| 内存占用 | 7 个任务 × 4096 字节栈 = 28KB + 其他 | ESP32-S3 有 512KB SRAM，充裕 |
| I2C 总线竞争 | IMU 和 VL53L1X 共享 I2C0 | ESP-IDF I2C 驱动内部有互斥锁，线程安全 |
| SW UART 精度 | 9600 baud 软件串口对定时器精度敏感 | `sw_uart.c` 已在 传感器修改2 修正时序 |
| RPLIDAR 高波特率 | 460800 baud 需要稳定供电 | 独立 5V/800mA 外部供电，与原版一致 |
| BU UWB 被动监听 | BU 模块主动输出，ESP 端只读 | 单向 RX 接线，与原版一致 |

### 六、文件变更摘要

| 文件 | 操作 | 说明 |
|------|------|------|
| `examples/sensor_hub/main/main.c` | **重写** | 移除 19 处 `#if`/`#endif`；拆分为 7 个 FreeRTOS 任务；引脚硬编码；A02YYUW #1 切 SW UART |
| `examples/sensor_hub/main/Kconfig.projbuild` | 修改 | 全部传感器 `default y`；A02YYUW #1 SW UART `default y` |
| `examples/sensor_hub/sdkconfig.defaults` | 修改 | 全部传感器 ENABLE=y；A02YYUW #1 USE_SW_UART=y |
| `README.md` | **重写** | 新增详细的 传感器修改5 变更记录 |

**未修改的文件**：所有 `components/sensors/` 下的传感器驱动代码（`a02yyuw.c/h`、`sw_uart.c/h`、`bu_uwb.c/h` 等）、构建系统文件（`CMakeLists.txt`）、测试和文档。

---

## 相比原版的改进 · 引脚

| 传感器 | 接口 | 原版引脚（传感器修改4） | 传感器修改5 引脚 |
|--------|------|------------------------|-----------------|
| A02YYUW #1 | **SW UART** | RX=IO35, TX=-1 (HW UART1) | RX=**IO35**, TX=-1 (**SW UART, 引脚不变**) |
| A02YYUW #2 | **SW UART** | RX=IO36, TX=-1 | RX=**IO36**, TX=-1（不变） |
| BU UWB | HW UART1 | RX=GPIO6, TX=GPIO7 | RX=GPIO6, TX=GPIO7（不变，独占 UART1） |
| FSR | ADC1 | GPIO8 (ADC1_CH7) | GPIO8 (ADC1_CH7)（不变） |
| RPLIDAR | HW UART2 | 17/18 | ESP_RX=GPIO17, ESP_TX=GPIO18（不变，独占 UART2） |
| IMU | I2C0 | SCL=12, SDA=11 | SCL=12, SDA=11（不变） |
| VL53L1X | I2C0（共享） | SCL=12, SDA=11 | SCL=12, SDA=11（不变） |

---

## 支持的传感器

| 传感器 | 类型 | 接口 | 并发方式 |
|--------|------|------|----------|
| A02YYUW #1 | 超声波测距 | SW UART (RX=IO35) | FreeRTOS 任务，500ms 间隔 |
| A02YYUW #2 | 超声波测距 | SW UART (RX=IO36) | FreeRTOS 任务，500ms 间隔 |
| BU03/BU04 | UWB 超宽带 | HW UART1 (6/7) | FreeRTOS 任务，100ms 间隔 |
| FSR | 薄膜压力 | ADC1 (GPIO8) | FreeRTOS 任务，500ms 间隔 |
| RPLIDAR C1 | 激光雷达 | HW UART2 (17/18) | FreeRTOS 任务，50ms 扫描 |
| IMU | 九轴惯性 | I2C0 (11/12) | FreeRTOS 任务，200ms 间隔 |
| VL53L1X | ToF 激光测距 | I2C0 共享 (11/12) | FreeRTOS 任务，250ms 间隔 |

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

> 不再需要 `menuconfig` 手动启用传感器——全部传感器默认启用，引脚已硬编码在 `main.c` 中。
> 如需修改引脚，直接编辑 `main.c` 顶部的 `#define` 常量。
> 各传感器组件已在 ESP-IDF v5.4 + ESP32-S3 上编译验证通过。

---

## 测试（PC 端，无需硬件）

```bash
bash tests/protocol/run_tests.sh
```

测试源码见 `tests/protocol/test_sensor_parsers.c`。

---

## 关键代码导读

| 文件 | 作用 | 传感器修改5 重点 |
|------|------|-----------------|
| `examples/sensor_hub/main/main.c` | **核心** | 7 个 FreeRTOS 任务并发；全部引脚常量；无 `#if`/`#endif` |
| `components/sensors/a02yyuw/a02yyuw.h` | A02YYUW 多实例 API | `a02yyuw_t` 句柄 + `a02yyuw_init_dev/read_dev/deinit_dev` |
| `components/sensors/a02yyuw/a02yyuw.c` | A02YYUW 实现 | 每个实例独立持有硬件/软件 UART 上下文 |
| `components/sensors/a02yyuw/sw_uart.c` | 软件串口 | `esp_timer` 位时序（已修正）；多路独立定时器/中断 |
| `components/sensors/bu_uwb/bu_uwb.h` | BU UWB API | `bu_uwb_init/read_bytes/parse_twr_line` — 独占 HW UART1 |
| `components/sensors/rplidar_c1/rplidar_c1.h` | RPLIDAR API | `rplidar_c1_init/start_scan/read_point` — 独占 HW UART2 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 配置 | 全部传感器 default y |
| `examples/sensor_hub/sdkconfig.defaults` | 默认值 | 全部 ENABLE=y，A02YYUW#1 USE_SW_UART=y |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **7 个任务并发，CPU 负载可控**：A02YYUW(9600baud/500ms)、FSR(500ms)、IMU(200ms)、VL53L1X(250ms) 均为低频读取；RPLIDAR 在 50ms 间隔内轮询 50 次点数据，优先级 3 保证实时性。ESP32-S3 双核 240MHz 完全胜任。
2. **A02YYUW #1 和 #2 都走 SW UART**：两个软件串口各有独立的 `esp_timer` 和 GPIO 中断，互不干扰。如需切回硬件 UART，修改 `A02_1_USE_SW_UART` / `A02_2_USE_SW_UART` 为 0，但需确保 UART 端口不冲突。
3. **每个超声波只接 RX**：A02YYUW 自主输出，TX 默认 -1 不接；接线只需 VCC/GND/模块TX→ESP RX。
4. **FSR 公式未标定**：`U=0.0004F+0.0749` 限幅 0–6kg，用前必须标定。
5. **RPLIDAR 需 5V/800mA 外部供电、TX/RX 交叉**。
6. **IMU 0x23 / VL53L1X 0x29(7位) 共享 I2C0(11/12)**。
7. **工作目录是 `examples/sensor_hub`**；测试命令是 `run_tests.sh`。
8. **引脚修改方式变更**：不再通过 `menuconfig` → Kconfig，直接编辑 `main.c` 顶部 `#define` 宏。

---

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式 + 句柄式多实例）
│   │   ├── sw_uart.c/h         # 软件 UART 驱动（esp_timer 位时序）
│   │   └── CMakeLists.txt
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动（需标定）
│   ├── imu_i2c/                # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/             # RPLIDAR C1 驱动
│   └── vl53l1x_tof/            # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # ★ 7 任务并发 + 无 #if/#endif + 引脚常量化
│       ├── Kconfig.projbuild   # 全部传感器 default y
│       └── sdkconfig.defaults  # 全部 ENABLE=y
├── docs/sensors/
│   └── pinout-and-wiring.md    # 引脚接线图
├── tests/protocol/             # PC 端协议测试
└── AGENTS.md
```
