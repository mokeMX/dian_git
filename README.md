# 传感器修改6 — 引脚与算法4一致版

`传感器修改6` 分支的引脚适配版本：**所有传感器引脚修改为与 `算法4` (follow_robot) 分支完全一致**，方便在同一套硬件上直接切换算法/传感器测试固件。

## 这个分支是什么

在 `传感器修改6`（7路传感器并发 + 双核任务固定）基础上，将所有引脚映射改为与 `算法4` (follow_robot) 分支相同的引脚：

```text
超声波 #1: RX=GPIO35          (原 GPIO4)
超声波 #2: RX=GPIO36          (原 GPIO5)
UWB:       RX=GPIO18, TX=GPIO37 (原 GPIO6/7)
RPLIDAR:   RX=GPIO17, TX=GPIO9  (原 TX=GPIO18)
IMU:       SDA=GPIO39, SCL=GPIO38 (不变)
FSR:       GPIO8               (不变)
VL53L1X:   I2C0, addr=0x52     (不变)
```

## 引脚总览

| 传感器 | 接口 | GPIO | 核心 | 优先级 |
|--------|------|------|------|--------|
| A02YYUW #1 | SW UART (9600) | RX=**GPIO35** | Core 0 | 4 |
| A02YYUW #2 | SW UART (9600) | RX=**GPIO36** | Core 0 | 4 |
| BU UWB | HW UART1 (115200) | RX=**GPIO18** TX=**GPIO37** | Core 1 | 2 |
| RPLIDAR C1 | HW UART2 (460800) | RX=GPIO17 TX=**GPIO9** | Core 1 | 3 |
| IMU | I2C0 | SDA=GPIO39 SCL=GPIO38 | Core 1 | 2 |
| VL53L1X | I2C0 共享 | SDA=GPIO39 SCL=GPIO38 | Core 1 | 2 |
| FSR | ADC1 | GPIO8 | Core 1 | 1 |

> 粗体为与原 `传感器修改6` 相比发生变化的引脚。

## 变更明细

### pin 定义变更 (main.c)

```c
// 变更前 (传感器修改6原版)
#define A02_1_RX_GPIO         4
#define A02_2_RX_GPIO         5
#define BU_UWB_RX_GPIO         6
#define BU_UWB_TX_GPIO         7
#define RPLIDAR_TX_GPIO       18

// 变更后 (与算法4一致)
#define A02_1_RX_GPIO         35
#define A02_2_RX_GPIO         36
#define BU_UWB_RX_GPIO         18
#define BU_UWB_TX_GPIO         37
#define RPLIDAR_TX_GPIO       9
```

### Kconfig.projbuild 默认值同步更新

| 配置项 | 旧默认 | 新默认 |
|--------|--------|--------|
| SENSOR_HUB_BU_UWB_RX_GPIO | 6 | 18 |
| SENSOR_HUB_BU_UWB_TX_GPIO | 7 | 37 |
| SENSOR_HUB_RPLIDAR_TX_GPIO | 18 | 9 |
| SENSOR_HUB_I2C_SDA_GPIO | 11 | 39 |
| SENSOR_HUB_I2C_SCL_GPIO | 12 | 38 |

### sdkconfig.defaults 同步更新

BU UWB、RPLIDAR、I2C 相关默认配置值已更新为与算法4一致。

## 代码功能（继承自 传感器修改6）

- 7 路传感器 FreeRTOS 任务并发运行
- `xTaskCreatePinnedToCore` 双核固定：Core 0 运行 SW UART 超声波（时序敏感），Core 1 运行其余传感器
- 引脚以 `#define` 硬编码在 `main.c` 顶部，修改直接编辑宏
- 全部传感器默认启用，无需 `menuconfig`

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 测试

```bash
bash tests/protocol/run_tests.sh
```

## 注意事项

1. **GPIO39 作 SDA**：经典 ESP32 上 GPIO39 仅输入不可作 SDA；ESP32-S3 上 GPIO39 可作为双向 IO，本分支目标为 ESP32-S3。
2. **与 算法4 共用同一套硬件接线**：烧录 算法4 固件和本分支固件时无需重新接线。
3. **SW UART 时序依赖 Core 0 隔离**：不要将超声波任务移到 Core 1 或降低优先级。
4. **修改引脚方式**：直接编辑 `main.c` 顶部的 `#define` 宏。
5. **A02YYUW 只接 RX**：模块自主输出，TX = -1。

## 修订记录

| 版本 | 修改内容 |
|------|----------|
| **本分支** | 引脚映射与算法4 (follow_robot) 一致：A02YYUW → GPIO35/36, UWB → GPIO18/37, RPLIDAR TX → GPIO9；Kconfig 和 sdkconfig.defaults 同步更新；更新 README 和 pinout-and-wiring.md |
| 传感器修改6 | 引脚重新分配（I2C: 38/39, A02YYUW: 4/5）；双核任务固定；修复 GPIO ISR 冲突 |
| 传感器修改5 | 7 路 FreeRTOS 并发；SW UART 适配 |

## 目录结构

```
├── .vscode/settings.json
├── .clangd
├── components/sensors/
│   ├── a02yyuw/        # 超声波驱动 (HW/SW UART 双模式)
│   ├── bu_uwb/         # UWB 驱动
│   ├── fsr_adc/        # FSR 压力传感器
│   ├── imu_i2c/        # IMU 驱动
│   ├── rplidar_c1/     # RPLIDAR C1 驱动
│   └── vl53l1x_tof/    # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # 核心：引脚定义 + 双核任务
│       ├── Kconfig.projbuild   # 配置菜单
│       └── sdkconfig.defaults  # 默认配置
├── docs/sensors/
│   └── pinout-and-wiring.md    # 引脚接线文档
├── tests/protocol/
└── README.md
```
