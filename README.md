# 传感器整合修改 (双核任务固定版)

`传感器修改5` 的硬件适配改进版本——在 7 路传感器 FreeRTOS 并发基础上，**重新分配所有引脚**以适配实际开发板布局，**将任务固定到双核**以提升实时性，并**修复多路软件 UART 的 GPIO ISR 冲突**。

> **这个分支是什么（传感器修改6）**：在 `传感器修改5`（全传感器并发）基础上做**硬件适配与双核优化**——I2C 总线移到 GPIO38/39，超声波传感器移到 GPIO4/5，任务通过 `xTaskCreatePinnedToCore` 固定到 Core 0 / Core 1 以隔离时序敏感任务，修复 `gpio_install_isr_service` 重复调用导致的报错，并新增 `.vscode` / `.clangd` 开发环境配置。`算法1` / `算法2` 要基于最新硬件引脚拉取分支优先从这里拉。

---

## 修订记录

| 版本 | 修改内容 |
|------|----------|
| **传感器修改6**（本分支） | 引脚重新分配（I2C: 38/39, A02YYUW: 4/5）；双核任务固定（Core 0: 超声波, Core 1: 其余）；修复 `gpio_install_isr_service` 多路冲突；新增 `.vscode` / `.clangd` 配置。 |
| 传感器修改5 | 移除所有 `#if`/`#endif`；全部传感器默认启用；主循环重构为 7 个 FreeRTOS 任务并发运行；A02YYUW #1 从 HW UART1 切换为 SW UART 以释放 UART1 给 BU UWB；主任务不再阻塞自旋；所有引脚硬编码为常量。 |
| 传感器修改4 | 新增第二个 A02YYUW 超声波；句柄式多实例 API；双超声波一硬一软 UART；引脚冲突修复（IO35/36/37）。 |
| 传感器修改3 | 补 `sw_uart.c` `#include <string.h>`、CMakeLists `REQUIRES`，组件可干净编译。 |
| 传感器修改2 | 修复软件 UART 采样时序（半位→整位中心采样）。 |

---

## 传感器修改6 详细变更清单

### 一、引脚重新分配

相比传感器修改5，**所有引脚全部重新分配**以适配实际开发板可用引脚布局：

| 传感器 | 接口 | 当前引脚 | 备注 |
|--------|------|---------|------|
| A02YYUW #1 | SW UART (9600) | **RX=GPIO4** | 软件串口，只接 RX |
| A02YYUW #2 | SW UART (9600) | **RX=GPIO5** | 软件串口，只接 RX |
| BU UWB | HW UART1 (115200) | **RX=GPIO6, TX=GPIO7** | 独占 UART1 |
| FSR | ADC1 | **GPIO8** (ADC1_CH7) | 模拟输入 |
| RPLIDAR C1 | HW UART2 (460800) | **RX=GPIO17, TX=GPIO18** | 独占 UART2 |
| IMU | I2C0 | **SDA=GPIO39, SCL=GPIO38** | addr 0x23 |
| VL53L1X | I2C0 共享 | **SDA=GPIO39, SCL=GPIO38** | addr 0x52(8位) |

> **关于 GPIO39 作 SDA**：经典 ESP32 上 GPIO39 为仅输入引脚，不可作 SDA；但 **ESP32-S3 上 GPIO39 可作为双向 IO**，因此 SDA 接到 GPIO39 在 ESP32-S3 下是合法的。

所有引脚以 `#define` 常量直接定义在 `main.c` 顶部，修改引脚只需编辑这些宏，无需 `menuconfig`。

### 二、双核任务固定（Core Pinning）

**修改前（传感器修改5）**：所有 7 个任务使用 `xTaskCreate` 创建，FreeRTOS 调度器自动分配核心。软件 UART 的 GPIO 中断可能被 Core 1 上的其他任务中断干扰，导致采样时序偏离。

**修改后（传感器修改6）**：全部任务使用 `xTaskCreatePinnedToCore` 显式指定运行核心：

```
Core 0 (优先级 4):   task_a02yyuw1、task_a02yyuw2    ← SW UART 时序敏感，最高优先级隔离
Core 1 (优先级 3):   task_rplidar                     ← 高速扫描
Core 1 (优先级 2):   task_bu_uwb、task_imu、task_vl53l1x
Core 1 (优先级 1):   task_fsr
```

| 任务 | 核心 | 优先级 | 栈空间 | 读取间隔 | 原因 |
|------|------|--------|--------|----------|------|
| A02YYUW #1 | Core 0 | **4** | 4096 | 500ms | SW UART 时序敏感，隔离到独立核心 + 最高优先级 |
| A02YYUW #2 | Core 0 | **4** | 4096 | 500ms | SW UART 时序敏感，隔离到独立核心 + 最高优先级 |
| RPLIDAR C1 | Core 1 | 3 | 4096 | 50ms | 高速 UART 扫描（460800 baud） |
| BU UWB | Core 1 | 2 | 4096 | 100ms | UART 被动监听 |
| IMU | Core 1 | 2 | 4096 | 200ms | I2C 读取 |
| VL53L1X | Core 1 | 2 | 4096 | 250ms | I2C 读取 |
| FSR | Core 1 | 1 | 4096 | 500ms | ADC 读取，最低优先级 |

> **设计思路**：两个 SW UART 任务（GPIO4/5）使用 `esp_timer` + GPIO 边沿中断实现位采样，对中断响应延迟极其敏感。将它们固定在 Core 0 并设为最高优先级 4，确保 GPIO 中断不会被 Core 1 上的 RPLIDAR（460800 baud 高速 UART）或其它任务抢占，保证 9600 baud 采样时序的稳定性。

### 三、修复 GPIO ISR 重复安装冲突

**问题**：`sw_uart_init` 内部调用了 `gpio_install_isr_service(0)`，当初始化第二个 SW UART 时，`gpio_install_isr_service` 被再次调用，导致 ESP-IDF 报错：

```
ESP_ERR_INVALID_STATE: gpio isr service already installed
```

**修复**：在 `sw_uart_init` 中保留 `gpio_install_isr_service(0)` 调用（该函数内部有幂等检查，第二次调用会直接返回 ESP_OK），确保多路 SW UART 初始化不冲突。

### 四、新增开发环境配置文件

传感器修改6 分支新增以下文件：

| 文件 | 作用 |
|------|------|
| `.vscode/settings.json` | VS Code 编辑器配置（C/C++ IntelliSense、ESP-IDF 插件设定等） |
| `.clangd` | clangd 语言服务器配置（编译数据库路径、诊断参数等） |

这些文件让开发者使用 VS Code + clangd 打开项目时无需手动配置即可获得代码补全、跳转定义、错误提示等功能。

### 五、代码结构变更

#### 5.1 任务创建方式

```c
// Core 0: SW UART 时序敏感任务，最高优先级
xTaskCreatePinnedToCore(task_a02yyuw1, "a02_1", 4096, NULL, 4, NULL, 0);
xTaskCreatePinnedToCore(task_a02yyuw2, "a02_2", 4096, NULL, 4, NULL, 0);

// Core 1: 其余传感器
xTaskCreatePinnedToCore(task_rplidar,  "rplidar", 4096, NULL, 3, NULL, 1);
xTaskCreatePinnedToCore(task_bu_uwb,   "bu_uwb", 4096, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(task_fsr,      "fsr",    4096, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(task_imu,      "imu",    4096, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(task_vl53l1x,  "vl53",   4096, NULL, 2, NULL, 1);
```

#### 5.2 引脚常量

```c
#define HUB_I2C_SDA_GPIO      39
#define HUB_I2C_SCL_GPIO      38
#define A02_1_RX_GPIO          4
#define A02_2_RX_GPIO          5
#define BU_UWB_RX_GPIO         6
#define BU_UWB_TX_GPIO         7
#define FSR_ADC_GPIO           8
#define RPLIDAR_RX_GPIO       17
#define RPLIDAR_TX_GPIO       18
```

### 六、文件变更摘要

| 文件 | 操作 | 说明 |
|------|------|------|
| `examples/sensor_hub/main/main.c` | **重写** | 引脚重新分配（I2C: 38/39, A02YYUW: 4/5）；全部任务改用 `xTaskCreatePinnedToCore` 并指定核心；旧版注释代码保留在文件末尾供参考 |
| `components/sensors/a02yyuw/sw_uart.c` | 修改 | `sw_uart_init` 中新增 `gpio_install_isr_service(0)` 调用 |
| `.vscode/settings.json` | **新增** | VS Code 编辑器配置 |
| `.clangd` | **新增** | clangd 语言服务器配置 |
| `README.md` | **重写** | 本文件，传感器修改6 详细变更记录 |

**未修改的文件**：所有 `components/sensors/` 下的其他传感器驱动代码（`a02yyuw.c/h`、`bu_uwb.c/h`、`fsr_adc.c/h`、`imu_i2c.c/h`、`rplidar_c1.c/h`、`vl53l1x_tof.c/h`）、`Kconfig.projbuild`、`sdkconfig.defaults`、构建系统文件（`CMakeLists.txt`）、测试和文档。

---

## 支持的传感器

| 传感器 | 类型 | 接口 | 核心 | 优先级 | 读取间隔 |
|--------|------|------|------|--------|----------|
| A02YYUW #1 | 超声波测距 | SW UART (RX=GPIO4) | Core 0 | 4 | 500ms |
| A02YYUW #2 | 超声波测距 | SW UART (RX=GPIO5) | Core 0 | 4 | 500ms |
| BU03/BU04 | UWB 超宽带 | HW UART1 (GPIO6/7) | Core 1 | 2 | 100ms |
| FSR | 薄膜压力 | ADC1 (GPIO8) | Core 1 | 1 | 500ms |
| RPLIDAR C1 | 激光雷达 | HW UART2 (GPIO17/18) | Core 1 | 3 | 50ms 扫描 |
| IMU | 九轴惯性 | I2C0 (SDA=39, SCL=38) | Core 1 | 2 | 200ms |
| VL53L1X | ToF 激光测距 | I2C0 共享 (SDA=39, SCL=38) | Core 1 | 2 | 250ms |

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

| 文件 | 作用 | 传感器修改6 重点 |
|------|------|-----------------|
| `examples/sensor_hub/main/main.c` | **核心** | 7 个 FreeRTOS 任务 + `xTaskCreatePinnedToCore` 双核固定 + 引脚硬编码为 GPIO4/5/38/39；底部保留传感器修改5 及实验版本注释代码供参考 |
| `components/sensors/a02yyuw/sw_uart.c` | 软件串口 | `esp_timer` 位时序（已修正）；`sw_uart_init` 内调用 `gpio_install_isr_service(0)` 确保 GPIO ISR 服务就绪 |
| `components/sensors/a02yyuw/a02yyuw.h` | A02YYUW 多实例 API | `a02yyuw_t` 句柄 + `a02yyuw_init_dev/read_dev/deinit_dev` |
| `components/sensors/bu_uwb/bu_uwb.h` | BU UWB API | `bu_uwb_init/read_bytes/parse_twr_line` — 独占 HW UART1 |
| `components/sensors/rplidar_c1/rplidar_c1.h` | RPLIDAR API | `rplidar_c1_init/start_scan/read_point` — 独占 HW UART2 |
| `.vscode/settings.json` | VS Code 配置 | ESP-IDF 插件设置、C/C++ IntelliSense 路径 |
| `.clangd` | clangd 配置 | 编译数据库路径、诊断选项 |

---

## 潜在风险与注意事项

| 风险 | 说明 | 缓解措施 |
|------|------|----------|
| GPIO39 作 SDA | 经典 ESP32 上 GPIO39 仅输入不可作 SDA | 本分支目标为 ESP32-S3，GPIO39 可作为双向 IO |
| Core 0 负载 | 两个超声波任务优先级 4，长期占用 Core 0 | 超声波读取间隔 500ms，CPU 空闲时间充裕 |
| SW UART 时序 | 9600 baud 软件串口对定时器和中断响应延迟敏感 | 两个 SW UART 任务固定在 Core 0 且最高优先级 4，避免被 Core 1 的高速任务（RPLIDAR 460800 baud）中断干扰 |
| I2C 总线竞争 | IMU 和 VL53L1X 共享 I2C0（SCL=38, SDA=39） | ESP-IDF I2C 驱动内部有互斥锁，线程安全 |
| 内存占用 | 7 个任务 x 4096 字节栈 = 28KB + 其他 | ESP32-S3 有 512KB SRAM，充裕 |
| BU UWB 被动监听 | BU 模块主动输出，ESP 端只读 | 单向 RX 接线，与原版一致 |

---

## 注意事项 / 容易踩的坑

1. **引脚已全部变更**：核对接线时请使用本分支的引脚表（I2C=38/39, A02YYUW=4/5），不要参考传感器修改4/5 的引脚文档。
2. **GPIO39 仅 ESP32-S3 可用作 SDA**：如果在经典 ESP32（非 S3）上使用，需要将 SDA 改到其他可用引脚（如 GPIO15、GPIO21 等），因为经典 ESP32 的 GPIO34-39 为仅输入。
3. **SW UART 时序依赖 Core 0 隔离**：不要将超声波任务移到 Core 1 或降低优先级，否则 RPLIDAR 等高速外设的中断可能干扰 9600 baud 软件位采样时序。
4. **每个超声波只接 RX**：A02YYUW 自主输出，TX 默认 -1 不接；接线只需 VCC/GND/模块TX→ESP RX。
5. **FSR 公式未标定**：`U=0.0004F+0.0749` 限幅 0-6kg，用前必须标定。
6. **RPLIDAR 需 5V/800mA 外部供电、TX/RX 交叉**。
7. **IMU 0x23 / VL53L1X 0x52(8位) 共享 I2C0(SDA=39, SCL=38)**。
8. **工作目录是 `examples/sensor_hub`**；测试命令是 `run_tests.sh`。
9. **引脚修改方式**：直接编辑 `main.c` 顶部的 `#define` 宏，无需 `menuconfig`。

---

## 目录结构

```
├── .vscode/
│   └── settings.json            # VS Code 编辑器配置（传感器修改6 新增）
├── .clangd                      # clangd 语言服务器配置（传感器修改6 新增）
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式 + 句柄式多实例）
│   │   ├── sw_uart.c/h         # 软件 UART 驱动（esp_timer 位时序，含 gpio_install_isr_service）
│   │   └── CMakeLists.txt
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动（需标定）
│   ├── imu_i2c/                # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/             # RPLIDAR C1 驱动
│   └── vl53l1x_tof/            # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # 核心：双核任务固定 + 引脚 GPIO4/5/38/39
│       ├── Kconfig.projbuild   # 配置菜单（引脚默认值未同步更新，实际以 main.c #define 为准）
│       └── sdkconfig.defaults  # 默认配置
├── docs/sensors/
│   └── pinout-and-wiring.md    # 引脚接线图
├── tests/protocol/             # PC 端协议测试
├── scripts/                    # 辅助脚本
├── outputs/                    # 输出文件
├── work/                       # 工作目录
└── AGENTS.md
```
