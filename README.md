# 传感器整合修改 — Autobox Sensor Driver

ESP-IDF 6.1 / ESP32-S3 (正点原子 ATK-DNESP32S3) 传感器驱动整合工程。

> **分支说明**: 基于 `传感器整合` 分支，修复了所有引脚重叠冲突，A02YYUW 改用 GPIO 软件串口释放硬件 UART 资源。

---

## 工程结构

```
dian_git/
├── components/sensors/          # 传感器驱动组件
│   ├── a02yyuw/                 # A02YYUW 超声波 (支持硬件/软件串口)
│   │   ├── a02yyuw.c / .h       #   驱动主文件
│   │   └── sw_uart.c / .h       #   GPIO 软件串口驱动 (新增)
│   ├── bu_uwb/                  # BU03/BU04 UWB 测距定位
│   ├── fsr_adc/                 # FSR 薄膜压力 ADC
│   ├── imu_i2c/                 # 自定义 I2C IMU (9轴姿态)
│   ├── rplidar_c1/              # RPLIDAR C1 激光雷达
│   └── vl53l1x_tof/             # VL53L1X ToF 激光测距
├── examples/sensor_hub/         # 传感器联调测试工程
│   ├── main/
│   │   ├── main.c               #   测试主程序
│   │   └── Kconfig.projbuild    #   menuconfig 配置项
│   └── sdkconfig.defaults       #   默认配置
├── docs/sensors/                # 文档
│   └── pinout-and-wiring.md     #   引脚接线详细文档
└── tests/protocol/              # 主机侧协议解析测试
```

---

## 引脚分配 (完整总表)

**所有引脚已去重叠，每个 GPIO 仅分配给一个外设。**

| 传感器 | 接口类型 | ESP32-S3 引脚 | 速率/参数 | menuconfig 配置项 |
|---|---|---|---|---|
| **A02YYUW** | 软件串口 (GPIO 位检测) | **RX=GPIO4**, TX=GPIO5 | 9600 8N1 | `A02YYUW_USE_SW_UART=y` |
| **BU03/BU04 UWB** | 硬件 UART1 | **RX=GPIO6**, TX=GPIO7 | 115200 8N1 | `BU_UWB_UART=1` |
| **RPLIDAR C1** | 硬件 UART2 | **RX=GPIO17**, TX=GPIO18 | 460800 8N1 | `RPLIDAR_UART=2` |
| **FSR 压力** | ADC1_CH7 | **GPIO8** (P3 座 ADC_IN) | 0~3.3V 分压 | `FSR_ADC_CHANNEL=7` |
| **I2C 共享总线** | I2C0 | **SDA=GPIO11, SCL=GPIO12** | 400kHz | `I2C_SDA_GPIO=11`, `I2C_SCL_GPIO=12` |

> I2C 总线上挂载两个设备：IMU (7位地址 `0x23`) 和 VL53L1X (8位地址 `0x52` / 7位 `0x29`)，通过同一个 `i2c_master_bus_handle_t` 共享。

---

## 传感器逐一接线表

### A02YYUW 超声波测距

| A02YYUW | ESP32-S3 | 说明 |
|---|---|---|
| VCC | 3.3V 或 5V | 按模块标称供电 |
| GND | GND | 必须共地 |
| TX (模块输出) | **GPIO4** | ESP32 软件串口 RX |
| RX (模块输入) | GPIO5 (可悬空) | 自主输出模式下无需连接 |

- 协议：4 字节帧 `[0xFF][H][L][checksum]`，距离 = (H<<8)|L 单位 mm，范围 30~4500mm
- 软件串口原理：`esp_timer` 高精度定时器 + GPIO 下降沿中断，半位宽采样 (52µs@9600)

### BU03/BU04 UWB 测距/定位

| BU 模块 (UART2) | ESP32-S3 | 说明 |
|---|---|---|
| PA2 / TX (模块输出) | **GPIO6** | ESP32 UART1 RX |
| PA3 / RX (模块输入) | GPIO7 | ESP32 UART1 TX |
| GND | GND | 必须共地 |
| VCC | 按模块标称供电 | — |

- 监听 UWB 基站 UART2 输出，解析 PDOA/TWR JSON 行及 `distance: x.xxx` 文本行
- 支持 `TWR` 帧中 `D`, `Xcm`, `Ycm`, `R`, `P`, `T`, `V`, `O` 等字段解析

### RPLIDAR C1 激光雷达

| RPLIDAR C1 | ESP32-S3 | 说明 |
|---|---|---|
| VCC 5V | 5V 电源 | 启动电流可达 800mA，建议独立 5V 供电 |
| GND | GND | 必须与 ESP32 共地 |
| TX (黄色线, 雷达输出) | **GPIO17** | ESP32 UART2 RX |
| RX (绿色线, 雷达输入) | GPIO18 | ESP32 UART2 TX |

- 二进制协议：5 字节扫描点包 `[quality][angle_Q6_low][angle_Q6_high][dist_Q2_low][dist_Q2_high]`
- GPIO17/18 为原始源码默认引脚，若同时使用 RGB LCD 需调整

### FSR 薄膜压力传感器

| FSR 电路 | ESP32-S3 | 说明 |
|---|---|---|
| 分压点 | **GPIO8** (P3-2) | ADC1_CH7，开发板 P3 座 "ADC_IN" |
| 3.3V | P3-1 (VCC3.3) | 分压电阻上端 |
| GND | P3-4 | 固定电阻下端 |

分压电路：
```
3.3V(P3-1) ── FSR ──┬── ADC_IN(P3-2 / GPIO8) ── RM(固定电阻) ── GND(P3-4)
```

> 当前采用公式 `F = (U - 0.0749) / 0.0004` 估算压力 (kg)，限幅 0~6kg。**实际使用前必须用实物重新标定。**

### IMU (自定义 I2C 9 轴姿态模块)

| IMU | ESP32-S3 | 说明 |
|---|---|---|
| SDA | **GPIO11** | I2C 共享总线 SDA |
| SCL | **GPIO12** | I2C 共享总线 SCL |
| VCC | 3.3V | — |
| GND | GND | — |

- 7 位地址 `0x23`，读取加速度/陀螺仪/磁力计/四元数/欧拉角/气压计
- 寄存器映射：0x01=版本(3B), 0x04=加速度(6B), 0x0A=陀螺仪(6B), 0x10=磁力计(6B), 0x16=四元数(16B float), 0x26=欧拉角(12B float), 0x32=气压(16B float)

### VL53L1X ToF 激光测距

| VL53L1X | ESP32-S3 | 说明 |
|---|---|---|
| SDA | **GPIO11** | I2C 共享总线 SDA |
| SCL | **GPIO12** | I2C 共享总线 SCL |
| VCC | 3.3V | — |
| GND | GND | — |

- 8 位地址 `0x52` (等价 7 位 `0x29`)，使用 ST VL53L1X ULD v3.5.5 驱动
- 默认定时预算 50ms，测量间隔 55ms

---

## 引脚冲突修复详情

原始 `传感器整合` 分支存在严重的引脚重叠问题：

| 冲突引脚 | 原始分配 (冲突) | 修复后分配 |
|---|---|---|
| **GPIO36** | A02YYUW RX **=** BU UWB RX **=** RPLIDAR C1 RX | → A02=GPIO4, BU=GPIO6, RPLIDAR=GPIO17 |
| **GPIO37** | A02YYUW TX **=** BU UWB TX **=** RPLIDAR TX **=** IMU SCL | → A02=GPIO5, BU=GPIO7, RPLIDAR=GPIO18, I2C=GPIO12 |
| **GPIO38** | IMU SDA **=** VL53L1X SCL | → 统一 I2C SDA=GPIO11 |
| **UART1** | A02YYUW 和 BU UWB 共用同一 UART 端口 | → A02 改软件串口, BU 独占 UART1 |

---

## 软件串口 (sw_uart) 说明

为解决 3 个串口设备只有 2 个硬件 UART 的问题，A02YYUW 改用 GPIO 位检测软件串口。

### 工作原理

```
GPIO 下降沿中断 (检测起始位)
    ↓
esp_timer 一次性定时器 (半位宽 52μs → 采样起始位中心)
    ↓
esp_timer 连续定时器 (全位宽 104μs × 8 → 逐位采样数据位+停止位)
    ↓
环形缓冲区 → sw_uart_read_bytes() 读取
```

### 关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| 支持波特率 | ≤115200 | 受 esp_timer 精度限制，推荐 ≤38400 |
| 定时器分辨率 | 1μs | esp_timer 1MHz 时钟 |
| A02YYUW 半位宽 | 52μs | 500000/9600 |
| RX 缓冲区 | 1024 字节 | 环形缓冲，足够容纳约 250 帧 |
| 停止位校验 | 有 | 仅在停止位为高电平时存入字节 |

### 切换回硬件 UART

若需使用硬件 UART1 驱动 A02YYUW，在 `menuconfig` 中：
```
Autobox sensor hub test
  → [ ] Use software (GPIO bit-bang) UART for A02YYUW   # 取消勾选
  → A02YYUW hardware UART port = 1
```

---

## 快速构建与烧录

```bash
# 1. 进入测试工程目录
cd examples/sensor_hub

# 2. 加载 ESP-IDF 环境
source ~/esp/esp-idf/export.sh

# 3. 设置目标芯片
idf.py set-target esp32s3

# 4. 配置传感器 (可选，默认仅开启 A02YYUW)
idf.py menuconfig
#    路径: Autobox sensor hub test

# 5. 编译
idf.py build

# 6. 烧录并监控
idf.py -p /dev/ttyUSB0 flash monitor
```

### menuconfig 可配置项一览

| 分类 | 配置项 | 默认值 |
|---|---|---|
| **A02YYUW** | 启用 / 软件串口开关 / RX GPIO / TX GPIO / 波特率 | y / y / 4 / 5 / 9600 |
| **BU UWB** | 启用 / UART 端口 / RX GPIO / TX GPIO / 波特率 | n / 1 / 6 / 7 / 115200 |
| **FSR** | 启用 / ADC GPIO / ADC 通道 | n / 8 / 7 |
| **RPLIDAR C1** | 启用 / UART 端口 / RX GPIO / TX GPIO / 波特率 | n / 2 / 17 / 18 / 460800 |
| **IMU** | 启用 / I2C 地址 | n / 0x23 |
| **VL53L1X** | 启用 / I2C 地址 / 定时预算 / 测量间隔 | n / 0x52 / 50ms / 55ms |
| **I2C 共享** | SDA GPIO / SCL GPIO / 速率 | 11 / 12 / 400kHz |

---

## 协议解析测试 (主机侧)

无需 ESP32 硬件即可验证协议解析器：

```bash
cd tests/protocol
bash run_tests.sh
```

测试覆盖 A02YYUW、BU UWB、FSR 三个传感器的帧解析逻辑。

---

## 修改文件清单 (相对于 `传感器整合` 分支)

| 文件 | 修改内容 |
|---|---|
| `components/sensors/a02yyuw/sw_uart.h` | **新增** 软件串口头文件 |
| `components/sensors/a02yyuw/sw_uart.c` | **新增** 软件串口实现 (esp_timer + GPIO 中断) |
| `components/sensors/a02yyuw/a02yyuw.h` | 新增 `use_sw_uart` 配置字段，包含 sw_uart.h |
| `components/sensors/a02yyuw/a02yyuw.c` | 支持硬件/软件串口双模式 init/read/deinit |
| `components/sensors/a02yyuw/CMakeLists.txt` | 新增 sw_uart.c 源文件和 esp_timer 依赖 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 所有传感器默认引脚改为无冲突分配；新增软件串口开关 |
| `examples/sensor_hub/main/main.c` | A02YYUW 初始化传入 `use_sw_uart` 标志 |
| `examples/sensor_hub/sdkconfig.defaults` | 更新默认引脚和软件串口开关 |
| `docs/sensors/pinout-and-wiring.md` | 全面更新接线表、冲突解决方案、软件串口说明 |

---

## 注意事项

1. **A02YYUW 软件串口**：仅实现 RX 接收功能。A02YYUW 在自主模式下持续发送数据，无需主控发送命令，因此 TX 引脚可悬空。
2. **RPLIDAR C1 供电**：启动电流高达 800mA，USB 供电可能不足，建议使用独立 5V 电源并确保共地。
3. **FSR 标定**：当前 FSR 力-电压转换公式为示例值，必须用实际传感器和分压电阻重新标定。
4. **GPIO17/18 复用**：这两个引脚在 ATK-DNESP32S3 开发板上同时连接到 RGB LCD 接口。若需同时使用 LCD 和 RPLIDAR，请在 menuconfig 中改为其他空闲 GPIO。
5. **I2C 上拉**：代码已启用内部上拉 (`enable_internal_pullup=true`)，若 I2C 设备通信不稳定，可外加 2.2kΩ~4.7kΩ 上拉电阻到 3.3V。
6. **平台兼容**：所有驱动支持 `ESP_PLATFORM` 条件编译，非 ESP32 环境下返回 `ESP_ERR_INVALID_STATE`，便于在 PC 上进行协议解析单元测试。
