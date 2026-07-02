# 传感器整合修改 (YD-ESP32-S3 + SW-I2C 版)

`传感器修改6e` — 从 ATK-DNESP32S3 迁移到 **YD-ESP32-S3 V1.4** 核心板，硬件 I2C 替换为**软件 (bit-bang) I2C**，全部引脚重新适配。

> **本分支做了什么**：
> 1. 从 `传感器修改6` 分支迁移代码到 YD-ESP32-S3 V1.4 核心板
> 2. 将硬件 I2C (`driver/i2c_master.h`) 替换为自研软件 I2C 驱动 (`sw_i2c`)
> 3. 重新分配所有引脚以适配 YD-ESP32-S3 排针布局
> 4. IMU / VL53L1X 共用同一对 SW-I2C 引脚 (GPIO11/GPIO12)
> 5. 更新全部文档和配置

---

## 修订记录

| 版本 | 修改内容 |
|------|----------|
| **传感器修改6e**（本分支） | 迁移至 YD-ESP32-S3 V1.4；硬件 I2C → 软件 I2C (sw_i2c)；引脚适配 (A02YYUW: 4/5, UWB: 6/7, FSR: 8, RPLIDAR: 17/18, I2C: 11/12)；新增 sw_i2c 组件 |
| 传感器修改6 | 引脚重新分配 (I2C: 38/39, A02YYUW: 4/5)；双核任务固定；修复 GPIO ISR 冲突 |
| 传感器修改5 | 7 传感器 FreeRTOS 并发；SW UART for A02YYUW #1 |
| 传感器修改4 | 双超声波支持；句柄式多实例 API |
| 传感器修改3 | 构建系统修复 |
| 传感器修改2 | SW UART 采样时序修正 |

---

## 硬件适配: YD-ESP32-S3 V1.4

### 板载资源

| 资源 | 说明 |
|------|------|
| 主控 | ESP32-S3-WROOM-1 (双核 Xtensa LX7, 240MHz, 512KB SRAM) |
| USB-UART | CH343P (GPIO19=UD+, GPIO20=UD-) |
| RGB LED | WS2812B (GPIO48) |
| 按键 | BOOT (GPIO0), RST (EN) |
| 排针 | J1 (左排 22Pin) + J2 (右排 22Pin)，引出全部可用 GPIO |

### 引脚分配 (传感器修改6e)

| 传感器 | 接口 | 引脚 | 备注 |
|--------|------|------|------|
| A02YYUW #1 | SW UART (9600) | **RX=GPIO4** | 软件串口，J1 Pin 3 |
| A02YYUW #2 | SW UART (9600) | **RX=GPIO5** | 软件串口，J1 Pin 4 |
| BU UWB | HW UART1 (115200) | **RX=GPIO6, TX=GPIO7** | 独占 UART1，J1 Pin 5/6 |
| FSR | ADC1 | **GPIO8** (ADC1_CH7) | 模拟输入，J1 Pin 11 |
| RPLIDAR C1 | HW UART2 (460800) | **RX=GPIO17, TX=GPIO18** | 独占 UART2，J1 Pin 9/10 |
| IMU | SW-I2C | **SDA=GPIO11, SCL=GPIO12** | addr 0x23, J1 Pin 16/17 |
| VL53L1X | SW-I2C 共享 | **SDA=GPIO11, SCL=GPIO12** | addr 0x52(8位), J1 Pin 16/17 |

> 所有传感器均接在 J1 排针上，接线集中、方便。

### SW-I2C 技术细节

YD-ESP32-S3 核心板无板载 I2C 总线引出，因此本分支自研了软件 I2C 驱动 (`components/sensors/sw_i2c/`)：

- **实现方式**：GPIO 开漏输出 + 内部上拉，bit-bang 方式模拟 I2C 时序
- **速率**：默认 100kHz (标准模式)，可配置为 400kHz
- **API**：`sw_i2c_init` / `sw_i2c_write` / `sw_i2c_read` / `sw_i2c_write_read` / `sw_i2c_deinit`
- **可靠性**：每个字节操作后检查 ACK/NACK，通信失败返回错误码
- **共享总线**：IMU 和 VL53L1X 共用同一 sw_i2c 实例，通过不同 I2C 地址访问

> 若通信不稳定，建议在 SDA/SCL 引脚外接 4.7kΩ ~ 10kΩ 上拉电阻到 3.3V。长线 (>20cm) 建议降速到 50kHz。

---

## 双核任务固定 (Core Pinning)

```
Core 0 (优先级 4):   task_a02yyuw1、task_a02yyuw2    ← SW UART 时序敏感，最高优先级
Core 1 (优先级 3):   task_rplidar                     ← 高速扫描
Core 1 (优先级 2):   task_bu_uwb、task_imu、task_vl53l1x
Core 1 (优先级 1):   task_fsr
```

| 任务 | 核心 | 优先级 | 栈空间 | 读取间隔 | 说明 |
|------|------|--------|--------|----------|------|
| A02YYUW #1 | Core 0 | 4 | 4096 | 500ms | SW UART 时序敏感 |
| A02YYUW #2 | Core 0 | 4 | 4096 | 500ms | SW UART 时序敏感 |
| RPLIDAR C1 | Core 1 | 3 | 4096 | 50ms | 高速扫描 460800 baud |
| BU UWB | Core 1 | 2 | 4096 | 100ms | UART 被动监听 |
| IMU | Core 1 | 2 | 4096 | 200ms | SW-I2C 读取 |
| VL53L1X | Core 1 | 2 | 4096 | 250ms | SW-I2C 读取 |
| FSR | Core 1 | 1 | 4096 | 500ms | ADC 读取 |

---

## 项目结构

```
├── .vscode/
│   └── settings.json
├── .clangd
├── components/sensors/
│   ├── a02yyuw/               # A02YYUW 超声波驱动 (HW/SW UART 双模式)
│   │   ├── a02yyuw.c/h
│   │   ├── sw_uart.c/h        # 软件 UART 驱动
│   │   └── CMakeLists.txt
│   ├── bu_uwb/                # BU03/BU04 UWB 驱动
│   ├── fsr_adc/               # FSR 压力传感器驱动
│   ├── imu_i2c/               # I2C 九轴 IMU 驱动 (已适配 SW-I2C)
│   ├── rplidar_c1/            # RPLIDAR C1 驱动
│   ├── vl53l1x_tof/           # VL53L1X ToF 驱动 (已适配 SW-I2C)
│   └── sw_i2c/                # [新增] 软件 I2C 驱动
│       ├── sw_i2c.c/h
│       └── CMakeLists.txt
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # 核心: 双核任务 + SW-I2C + YD-ESP32-S3 引脚
│       ├── Kconfig.projbuild   # 配置菜单
│       └── sdkconfig.defaults  # 默认引脚配置
├── docs/sensors/
│   └── pinout-and-wiring.md    # 引脚接线图
├── tests/protocol/             # PC 端协议测试
└── README.md                   # 本文件
```

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

> 所有传感器默认启用，引脚硬编码在 `main.c` 顶部 `#define` 宏中。修改引脚直接编辑宏即可，无需 `menuconfig`。
> 已验证编译环境：ESP-IDF v5.4 + ESP32-S3。

---

## 测试 (PC 端)

```bash
bash tests/protocol/run_tests.sh
```

---

## 文件变更摘要

| 文件 | 操作 | 说明 |
|------|------|------|
| `components/sensors/sw_i2c/*` | **新增** | 软件 I2C 驱动 (sw_i2c.c/h + CMakeLists.txt)，GPIO bit-bang 实现 |
| `components/sensors/imu_i2c/imu_i2c.h` | **重写** | 移除硬件 I2C 依赖 (driver/i2c_master.h)，改用 sw_i2c；简化 config 结构 |
| `components/sensors/imu_i2c/imu_i2c.c` | **重写** | 所有 I2C 操作替换为 sw_i2c API；每个设备独立 sw_i2c 实例 |
| `components/sensors/imu_i2c/CMakeLists.txt` | 修改 | `REQUIRES esp_driver_i2c` → `REQUIRES sw_i2c` |
| `components/sensors/vl53l1x_tof/vl53l1x_tof.h` | **重写** | 移除硬件 I2C 依赖，改用 sw_i2c；简化 config |
| `components/sensors/vl53l1x_tof/vl53l1x_tof.c` | **重写** | VL53L1_WriteMulti/ReadMulti 改用 sw_i2c |
| `components/sensors/vl53l1x_tof/CMakeLists.txt` | 修改 | `REQUIRES esp_driver_i2c` → `REQUIRES sw_i2c` |
| `examples/sensor_hub/main/main.c` | **重写** | 移除硬件 I2C 总线；引脚改为 4/5/6/7/8/11/12/17/18；I2C 初始化改为 sw_i2c_init |
| `examples/sensor_hub/main/CMakeLists.txt` | 修改 | 新增 `sw_i2c` 依赖，移除 `esp_driver_i2c` |
| `examples/sensor_hub/main/Kconfig.projbuild` | 修改 | 引脚默认值更新；I2C 描述改为 SW-I2C |
| `examples/sensor_hub/sdkconfig.defaults` | 修改 | 引脚更新为 4/5/6/7/8/11/12/17/18；I2C 速率改为 100000 |
| `examples/sensor_hub/CMakeLists.txt` | 修改 | 新增 `sw_i2c` 组件路径 |
| `CMakeLists.txt` | 修改 | 新增 `sw_i2c` 组件路径 |
| `docs/sensors/pinout-and-wiring.md` | **重写** | 更新为 YD-ESP32-S3 V1.4 引脚表 + SW-I2C 接线说明 |
| `README.md` | **重写** | 本文件 |

**未修改的文件**：`a02yyuw.c/h`、`sw_uart.c/h`、`bu_uwb.c/h`、`fsr_adc.c/h`、`rplidar_c1.c/h`、`VL53L1X_api.c/h`、`VL53L1X_calibration.c/h`、`vl53l1_platform.h`、测试和脚本。

---

## 注意事项

1. **本分支仅适用于 YD-ESP32-S3 V1.4 核心板**。引脚定义与 DNESP32-S3 (ATK) 不同，请按本文档接线。
2. **SW-I2C 为 bit-bang 实现**，性能低于硬件 I2C。100kHz 速率下稳定可靠，如需 400kHz 可修改 `HUB_I2C_SPEED_HZ` 宏。
3. **SDA/SCL 建议外接上拉电阻** (4.7kΩ~10kΩ to 3.3V)。内部上拉 (~45kΩ) 在长线或高速场景下可能不足。
4. **SW UART 时序依赖 Core 0 隔离**，不要将超声波任务移到 Core 1 或降低优先级。
5. **每个超声波只接 RX**，A02YYUW 自主输出，TX 默认 -1 不接。
6. **FSR 公式未标定**，`U=0.0004F+0.0749` 限幅 0-6kg，用前必须标定。
7. **RPLIDAR 需 5V/800mA 外部供电**。
8. **工作目录是 `examples/sensor_hub`**。
9. **引脚修改方式**：直接编辑 `main.c` 顶部 `#define` 宏。
10. **ESP-IDF 版本**：推荐 v5.4+，ESP32-S3 target。

---

## 从传感器修改6 迁移指南

| 对比项 | 传感器修改6 | 传感器修改6e |
|--------|------------|-------------|
| 目标板 | ATK-DNESP32S3 | YD-ESP32-S3 V1.4 |
| I2C 实现 | 硬件 I2C (GPIO38/39) | 软件 bit-bang I2C (GPIO11/12) |
| I2C 速率 | 400kHz | 100kHz |
| A02YYUW #1 | GPIO4 | GPIO4 (不变) |
| A02YYUW #2 | GPIO5 | GPIO5 (不变) |
| BU UWB | GPIO6/7 | GPIO6/7 (不变) |
| FSR | GPIO8 | GPIO8 (不变) |
| RPLIDAR | GPIO17/18 | GPIO17/18 (不变) |
| IMU addr | 0x23 | 0x23 (不变) |
| VL53L1X addr | 0x52 (8-bit) | 0x52 (8-bit, 不变) |
