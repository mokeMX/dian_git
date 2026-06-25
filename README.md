# 传感器整合修改 (引脚冲突修复版)

`传感器整合` 分支的改进版本，解决了所有 GPIO 引脚冲突，新增软件 UART 驱动，并支持**双超声波**同时工作。

> **这个分支是什么（传感器修改4）**：传感器底座的**最新稳定版**——在 `传感器修改3`（可干净编译）基础上新增**第二个 A02YYUW 超声波**，并依据开发板真实引脚表把超声波放到板上**唯一真正空闲的 IO35/36/37**。`算法1` / `算法2` 就是在本分支之上加跟随避障与底盘控制。要做整机优先从这里拉分支。

---

## 修订记录

- **传感器修改4**：新增**第二个 A02YYUW 超声波**支持，两个超声波各占一路独立 UART、互不冲突。
  - 依据 `DNESP32-S3 IO引脚分配表.xlsx` 核对 ATK-DNESP32S3 引脚：全板**只有 IO35 / IO36 / IO37 为「完全独立」**，其余均被 RGB-LCD / 摄像头 / I2S / SPI2 / IIC0 / USB / UART0 占用。两个超声波改用真正空闲的 **#1 = IO35，#2 = IO36**（IO37 预留）。
  - `a02yyuw` 驱动重构为**句柄式多实例 API**（`a02yyuw_init_dev/read_dev/deinit_dev`），可同时驱动多个超声波；保留原单实例 API 作向后兼容。
  - 遵循「不复用引脚、引脚/硬件不够就用软件 IO 模拟」原则：**#1 走硬件 UART1，#2 走软件 UART（GPIO 位检测）**，把 UART2 留给高波特率传感器。A02YYUW 自主输出，ESP 端仅需 RX，TX 默认 -1 不接，每个传感器只占 1 个引脚。
- **传感器修改3**：补 `sw_uart.c` 的 `#include <string.h>`、`CMakeLists.txt` 的 `REQUIRES esp_driver_gpio`，组件可干净编译。
- **传感器修改2**：修复软件 UART 采样时序（半位→整位中心采样），A02YYUW 字节不再错乱。

---

## 相比原版的改进 · 引脚

| 传感器 | 接口 | 原版引脚 | 修复后引脚（Kconfig 默认值） |
|--------|------|----------|------------|
| A02YYUW #1 | **HW UART1** | GPIO37(RX) | RX=**IO35**，TX=-1（板载完全独立引脚） |
| A02YYUW #2 | **SW UART** | 无 | RX=**IO36**，TX=-1（板载完全独立引脚） |
| BU UWB | UART1 | 36/37 | RX=GPIO6, TX=GPIO7 |
| FSR | ADC | GPIO36 | GPIO8 (ADC1_CH7) |
| RPLIDAR | UART2 | 17/18 | ESP_RX=GPIO17, ESP_TX=GPIO18 |
| IMU | I2C0 | SCL=37,SDA=38 | SCL=12, SDA=11 |
| VL53L1X | I2C0（共享） | 共用 IMU 总线 | SCL=12, SDA=11 |

### 双 A02YYUW 为什么一硬一软

ESP32-S3 除作控制台的 UART0 外只剩 UART1/UART2 两个硬件 UART。把**第二个超声波放到软件 UART**（`esp_timer` 定时 + GPIO 中断，9600 baud/4 字节帧足够），就能把宝贵的 UART2 留给 RPLIDAR（460800）。两路都走软件 UART 也可以（各自独立定时器/中断）。

---

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 |
|--------|------|------|------|
| A02YYUW #1 | 超声波测距 | HW UART1 (RX=IO35) | 毫米级（最大 4.5m） |
| A02YYUW #2 | 超声波测距 | SW UART (RX=IO36) | 毫米级（最大 4.5m），软件串口 |
| BU03/BU04 | UWB 超宽带 | UART1 (6/7) | 室内定位（PDOA / TWR 监听） |
| FSR | 薄膜压力 | ADC1 (GPIO8) | 压力（需标定） |
| RPLIDAR C1 | 激光雷达 | UART2 (17/18) | 360° 二维扫描 |
| IMU | 九轴惯性 | I2C0 (11/12) | 加速度(g)/陀螺/磁力/欧拉角/气压 |
| VL53L1X | ToF 激光测距 | I2C0 共享 (11/12) | 毫米级（最大 4m） |

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # 分别启用 A02YYUW #1/#2、其它传感器，并按需改引脚
idf.py build
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
| `components/sensors/a02yyuw/a02yyuw.h` | 多实例 API | `a02yyuw_t` 句柄 + `a02yyuw_init_dev/read_dev/deinit_dev`（驱动两个超声波的关键）；旧单实例 API 仍保留。 |
| `components/sensors/a02yyuw/a02yyuw.c` | 实现 | 每个实例独立持有硬件/软件 UART 上下文。 |
| `components/sensors/a02yyuw/sw_uart.c` | 软件串口 | `esp_timer` 位时序（已修正）；多路软件 UART 各自独立定时器/中断。 |
| `examples/sensor_hub/main/main.c` | 示例 | `a02yyuw_init_dev(&a02_1,...)`（#1）与 `#if CONFIG_SENSOR_HUB_A02YYUW2_ENABLE` 下的 `a02yyuw_init_dev(&a02_2,...)`（#2）；其余传感器同 hub 范式。 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 配置 | `SENSOR_HUB_A02YYUW_*`（#1）与 `SENSOR_HUB_A02YYUW2_*`（#2）两组，各自可启用/改引脚/切软硬件 UART。 |
| `docs/sensors/pinout-and-wiring.md` | 接线 | 顶部「开发板真实可用引脚」表（IO35/36/37）是本分支权威依据。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **「六传感器同时跑」要谨慎看引脚**：本分支真正确认空闲的只有 **IO35/36/37**（两个超声波 + 1 预留）。其余传感器的默认引脚 **GPIO4–18 在本开发板上其实被 LCD/摄像头/I2S/SPI2/IIC0 等板载外设占用**（见 pinout 文档顶部说明）。**只有当你不使用那些板载外设时**才可借用这些引脚；否则要在 `menuconfig` 里把 BU/FSR/RPLIDAR/I2C 改到你硬件上确实空出的脚。
2. **每个超声波只接 RX**：A02YYUW 自主输出，TX 默认 -1 不接；接线只需 VCC/GND/模块TX→ESP RX。
3. **#1=硬件UART1(IO35)、#2=软件UART(IO36)**：想全软件可在 menuconfig 把 #1 也切 SW UART。
4. **FSR 公式 `U=0.0004F+0.0749` 未标定**，限幅 0–6kg，用前必须标定。
5. **RPLIDAR 需 5V/800mA 外部供电、TX/RX 交叉**。
6. **IMU 0x23 / VL53L1X 0x29(7位) 共享 I2C0(11/12)**。
7. **工作目录是 `examples/sensor_hub`**；测试命令是 `run_tests.sh`（非 python）。

---

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/
│   │   ├── a02yyuw.c/h         # A02YYUW 驱动（硬件/软件 UART 双模式 + 句柄式多实例）
│   │   ├── sw_uart.c/h         # 软件 UART 驱动（esp_timer 位时序）
│   │   └── CMakeLists.txt      # REQUIRES 含 esp_timer / esp_driver_gpio / esp_driver_uart
│   ├── bu_uwb/                 # BU03/BU04 UWB 驱动
│   ├── fsr_adc/                # FSR 压力传感器驱动（需标定）
│   ├── imu_i2c/                # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/             # RPLIDAR C1 驱动
│   └── vl53l1x_tof/            # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c              # 传感器中心示例（初始化两个 A02YYUW：#1 硬件UART1 / #2 软件UART）
│       ├── Kconfig.projbuild   # menuconfig 配置菜单（#1/#2 两组）
│       └── sdkconfig.defaults  # 默认引脚配置
├── docs/sensors/
│   └── pinout-and-wiring.md    # 引脚接线图（顶部 IO35/36/37 真实空闲引脚分析）
├── tests/protocol/             # PC 端协议测试（run_tests.sh + .c）
└── AGENTS.md
```
