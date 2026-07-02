# 智能跟随行李箱 · 算法5e (YD-ESP32-S3 软件I2C + 闭环控制)

**算法5e = 算法4 + YD-ESP32-S3 V1.4 硬件适配 + 软件 I2C (bit-bang)**

以 `算法4` 的经验证传感器驱动和闭环控制逻辑为基础，适配 **YD-ESP32-S3 V1.4 核心板**。
核心变更：将 ESP-IDF 硬件 I2C 驱动替换为自研软件 bit-bang I2C (`sw_i2c`)，不再依赖 `driver/i2c_master.h`。

---

## 算法版本演变

| 版本 | 传感器基础 | 控制算法 | I2C 方式 | 目标硬件 | 状态 |
|------|-----------|---------|---------|---------|------|
| 算法1 | 传感器修改系列 | 开环 H 桥 + 跟随/避障 | HW I2C | 原板 | 历史 |
| 算法2 | 传感器修改系列 | 闭环 APO-DL ESC + AB 编码器 PID + IMU 航向 | HW I2C | 原板 | 参考 |
| 算法3 | 传感器驱动有 Bug | 闭环控制 (同算法2) | HW I2C | 原板 | 有问题 |
| 算法4 | 传感器修改6 (经验证) | 闭环控制 (同算法2/3) | HW I2C | 原板 | 上一个版本 |
| **算法5e** (本分支) | **传感器修改6 (经验证)** | **闭环控制 (同算法4)** | **软件 I2C (bit-bang)** | **YD-ESP32-S3 V1.4** | **当前** |

---

## 算法5e 迁移变更

### 1. 软件 I2C 驱动 (`sw_i2c`)

**原因**: YD-ESP32-S3 核心板上没有专用的硬件 I2C 外设引脚占用问题，但为保证最大的引脚灵活性，使用纯 GPIO 模拟 I2C 时序。

**新增组件**: `components/sensors/sw_i2c/`

| 文件 | 说明 |
|------|------|
| `sw_i2c.h` | 软件 I2C 主机接口定义 |
| `sw_i2c.c` | bit-bang I2C 实现 (START/STOP/读写/ACK检测) |
| `CMakeLists.txt` | 组件注册 (依赖 `driver`，仅需 GPIO) |

**特性**:
- 纯 GPIO 操作，无需 ESP-IDF I2C 驱动
- 支持标准模式 100kHz (可通过 Kconfig 配置)
- 内部上拉电阻使能
- `sw_i2c_probe()` 设备探测
- `sw_i2c_write_reg()` / `sw_i2c_read_reg()` 寄存器读写
- 超时保护 (5ms ACK 等待)

### 2. IMU 驱动适配

`imu_i2c.h/c` 从硬件 I2C 切换为软件 I2C:

| 变更项 | 算法4 (旧) | 算法5e (新) |
|--------|-----------|------------|
| I2C 初始化 | `i2c_new_master_bus()` + `i2c_master_bus_add_device()` | `sw_i2c_init()` |
| 寄存器读 | `i2c_master_transmit_receive()` | `sw_i2c_read_reg()` |
| 设备句柄 | `i2c_master_bus_handle_t` + `i2c_master_dev_handle_t` | 单个 `sw_i2c_t` |
| 外部总线共享 | `external_bus` 参数传入 | `external_i2c` 参数传入 |
| 头文件依赖 | `driver/i2c_master.h` | `sw_i2c.h` + `hal/gpio_types.h` |

### 3. 引脚映射 (YD-ESP32-S3 V1.4)

以下为 YD-ESP32-S3 核心板的 J1/J2 排针引脚对照。

#### J1 排针 (22pin, 左侧)

| 引脚 | 信号 | GPIO | 用途 |
|------|------|------|------|
| 1 | VDD33 | - | 3.3V 电源输出 |
| 2 | GND | - | 地 |
| 3 | CHIP_PU | - | EN (使能) |
| **4** | **GPIO4** | **4** | **左 ESC PWM (50Hz RC 脉冲)** |
| **5** | **GPIO5** | **5** | **右 ESC PWM (50Hz RC 脉冲)** |
| **6** | **GPIO6** | **6** | **左编码器 A 相 (4x 正交解码)** |
| **7** | **GPIO7** | **7** | **左编码器 B 相 (4x 正交解码)** |
| **8** | **GPIO15** | **15** | **右编码器 A 相 (4x 正交解码)** |
| **9** | **GPIO16** | **16** | **右编码器 B 相 (4x 正交解码)** |
| **10** | **GPIO17** | **17** | **Lidar RX (UART2)** |
| **11** | **GPIO18** | **18** | **UWB RX (UART1)** |
| **12** | **GPIO8** | **8** | **UWB TX (UART1)** |
| 13 | GPIO3 | 3 | 备用 (USB-JTAG 复用) |
| 14 | GPIO46 | 46 | 备用 |
| 15 | **GPIO9** | **9** | **Lidar TX (UART2)** |
| 16 | GPIO10 | 10 | 备用 |
| **17** | **GPIO11** | **11** | **IMU SDA (软件 I2C)** |
| **18** | **GPIO12** | **12** | **IMU SCL (软件 I2C)** |
| 19 | GPIO13 | 13 | 备用 |
| 20 | GPIO14 | 14 | 备用 |
| 21 | GND | - | 地 |
| 22 | GND | - | 地 |

#### J2 排针 (22pin, 右侧)

| 引脚 | 信号 | GPIO | 用途 |
|------|------|------|------|
| 1 | U0RXD | 43 | 调试串口 RX |
| 2 | U0TXD | 44 | 调试串口 TX |
| 3 | GPIO0 | 0 | BOOT 按钮 |
| 4 | GPIO35 | 35 | 备用 |
| 5 | GPIO36 | 36 | 备用 |
| 6 | GPIO42 | 42 | 备用 |
| 7 | GPIO41 | 41 | 备用 |
| 8 | GPIO40 | 40 | 备用 |
| 9 | GPIO39 | 39 | 备用 |
| 10 | GPIO38 | 38 | 备用 |
| **12** | **GPIO36** | **36** | **右超声波 RX (SW UART)** |
| **13** | **GPIO35** | **35** | **左超声波 RX (SW UART)** |
| 14 | GPIO1 | 1 | 备用 |
| 15 | GPIO45 | 45 | 备用 |
| 16 | GPIO48 | 48 | **板载 WS2812B RGB LED (已占用!)** |
| 17 | GPIO47 | 47 | 备用 |
| 18 | GPIO21 | 21 | 备用 |
| 19 | GPIO20 | 20 | 备用 |
| 20 | GPIO19 | 19 | 备用 |
| 21-22 | GND | - | 地 |

> **注意**: GPIO48 已连接板载 RGB LED (XL-5050RGBC-WS2812B)，不能用作普通 GPIO。

---

## 完整接线速查表

### 外设 -> YD-ESP32-S3 接线

```
左电机 ESC    -> J1 引脚 4  (GPIO4,  LEDC PWM 50Hz)
右电机 ESC    -> J1 引脚 5  (GPIO5,  LEDC PWM 50Hz)
左编码器 A    -> J1 引脚 6  (GPIO6,  GPIO 中断 4x 正交)
左编码器 B    -> J1 引脚 7  (GPIO7,  GPIO 中断)
右编码器 A    -> J1 引脚 8  (GPIO15, GPIO 中断)
右编码器 B    -> J1 引脚 9  (GPIO16, GPIO 中断)
Lidar C1 RX  -> J1 引脚 10 (GPIO17, UART2 460800)
Lidar C1 TX  -> J1 引脚 15 (GPIO9,  UART2 460800)
UWB RX       -> J1 引脚 11 (GPIO18, UART1 115200)
UWB TX       -> J1 引脚 12 (GPIO8,  UART1 115200)
IMU SDA      -> J1 引脚 17 (GPIO11, 软件 I2C)
IMU SCL      -> J1 引脚 18 (GPIO12, 软件 I2C)
左超声波 RX   -> J2 引脚 13 (GPIO35, SW UART 9600)
右超声波 RX   -> J2 引脚 12 (GPIO36, SW UART 9600)
```

```
                     YD-ESP32-S3 核心板
                     ┌──────────────────┐
          J1 (左侧)    │                  │    J2 (右侧)
      ┌───────────────┤                  ├───────────────┐
  1   │ VDD33 (3.3V)  │  ESP32-S3-WROOM-1│ U0RXD (调试)   │ 1
  2   │ GND           │                  │ U0TXD (调试)   │ 2
  3   │ EN            │                  │ GPIO0 (BOOT)   │ 3
  4   │ GPIO4  ← 左ESC│                  │ GPIO35 → 左超声│ 13
  5   │ GPIO5  ← 右ESC│                  │ GPIO36 → 右超声│ 12
  6   │ GPIO6  ← 左ENC│                  │ GPIO48 (LED)*  │ 16
  7   │ GPIO7  ← 左ENC│                  │ ...            │
  8   │ GPIO15 ← 右ENC│                  │                │
  9   │ GPIO16 ← 右ENC│                  │                │
  10  │ GPIO17 ← Lidar│                  │                │
  11  │ GPIO18 ← UWB  │                  │                │
  12  │ GPIO8  → UWB  │                  │                │
  15  │ GPIO9  → Lidar│                  │                │
  17  │ GPIO11 ↔ SDA  │                  │                │
  18  │ GPIO12 ↔ SCL  │                  │                │
      └───────────────┤                  ├───────────────┘
                      └──────────────────┘
```

> `*` GPIO48 已连接板载 WS2812B RGB LED，不可用作 IO。

---

## 软件 I2C 工作方式

```
SCL ──┐     ┌──┐  ┌──┐  ┌──┐  ┌──┐     ┌──┐
      └─────┘  └──┘  └──┘  └──┘  └─ ... ┘  └─────

SDA ──────┐     ┌───┐  ┌─────┐     ┌──────────
          └─────┘   └──┘     └─────┘
          START     D7  D6   D0 ACK  STOP

时序:
  - START: SDA↓ 在 SCL 高电平时
  - STOP:  SDA↑ 在 SCL 高电平时
  - 数据: 在 SCL 低电平时改变 SDA
  - 采样: 在 SCL 高电平时读取 SDA
  - ACK:  第 9 个 SCL 高电平时从机拉低 SDA

频率: 100kHz (可通过 Kconfig 配置)
实现: GPIO 直接寄存器操作 + esp_rom_delay_us() 延时
```

---

## 构建与烧录

### 主项目: follow_robot

```bash
# 在项目根目录
idf.py set-target esp32s3
idf.py menuconfig   # 确认引脚配置
idf.py build
idf.py flash monitor
```

Kconfig 菜单位于 `Follow-me suitcase 算法5e (YD-ESP32-S3 + SW I2C + closed-loop)` 菜单下。

### 传感器测试项目: sensor_hub

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build flash monitor
```

---

## 上电调试顺序

1. **垫高轮子空跑**: 先确认电调解锁 (上电后等待 2 秒 ESC 自检)
2. **标定编码器**: 手推 1m，看日志实测速度符号和增量
3. **标定满速轮速**: 给最大前进，量实际轮速
4. **整定速度 PID**: 先只留前馈 (KI=KD=0)，再加 KI 消静差、KD 抑超调
5. **IMU 验证**: 观察日志确认 IMU 读数正常，航向闭环方向正确
6. **校激光雷达零位**: `LIDAR_FORWARD_DEG` 对准机体正前方
7. **校 UWB 左右**: 目标方位方向反了则调整 `UWB_LEFT_IS_POS_X`
8. 确认 FOLLOW/AVOID/ESTOP 切换与转向方向都正确后再落地测试

---

## 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance_m` | 1.0m | 期望跟车距离 |
| `max_linear_mps` | 0.7 m/s | 最大前进速度 |
| `max_angular_rps` | 1.6 rad/s | 最大转向角速度 |
| `emergency_distance_m` | 0.35m | 急停触发距离 |
| `track_width_m` | 0.30m | 后轮距 |
| `ticks_per_meter` | 2000 | 编码器每米 4x 脉冲数 (必须标定) |
| `kp / ki / kd` | 200 / 300 / 5 | 速度 PID (us 每 m/s) |
| `heading_kp` | 1.5 rad/s per rad | 航向闭环增益 |
| `esc_min/mid/max_us` | 1000/1500/2000 | ESC 脉冲行程 |
| `control_hz` | 50 | 控制循环频率 |
| `i2c_speed_hz` | 100000 | 软件 I2C 时钟频率 |

---

## 目录结构

```
├── CMakeLists.txt                    # 根项目: follow_robot
├── README.md                         # 本文件
├── components/
│   ├── control/
│   │   ├── chassis/                  # 闭环底盘 (ESC PWM + 编码器 PID)
│   │   └── follow_avoid/            # 跟随 + VFH-lite 避障算法
│   └── sensors/
│       ├── a02yyuw/                  # A02YYUW 超声波 (SW UART)
│       ├── bu_uwb/                   # BU03/BU04 UWB (TWR JSON)
│       ├── fsr_adc/                  # FSR 压力传感器 (ADC)
│       ├── imu_i2c/                  # 九轴 IMU (软件 I2C)
│       ├── rplidar_c1/              # RPLIDAR C1 激光雷达
│       ├── sw_i2c/                   # ★ 新增: 软件 bit-bang I2C 驱动
│       └── vl53l1x_tof/             # VL53L1X ToF 激光测距
├── examples/
│   ├── follow_robot/                 # 主程序: 完整跟随机器人
│   │   ├── CMakeLists.txt
│   │   ├── sdkconfig.defaults
│   │   └── main/
│   │       ├── CMakeLists.txt
│   │       ├── Kconfig.projbuild    # 全部参数配置 (含 YD-ESP32-S3 引脚)
│   │       └── main.c               # 多任务 RTOS + 控制循环
│   └── sensor_hub/                   # 传感器测试程序
└── tests/
    ├── algorithm/
    └── protocol/
```

---

## 与算法4 的代码差异

```
 算法4 (原板)                      算法5e (YD-ESP32-S3)
 ─────────────────────────────────────────────────────
 main.c:
   #include "driver/i2c_master.h"   →  #include "sw_i2c.h"
   i2c_master_bus_handle_t          →  sw_i2c_t
   i2c_new_master_bus()             →  sw_i2c_init()
   i2c_master_bus_add_device()      →  (内置在 imu_i2c_init)

 imu_i2c.h:
   #include "driver/i2c_master.h"   →  #include "sw_i2c.h"
   i2c_master_bus_handle_t          →  sw_i2c_t *
   imu_i2c_init(imu, cfg)           →  imu_i2c_init(imu, cfg, ext_i2c)

 imu_i2c.c:
   i2c_master_transmit_receive()    →  sw_i2c_read_reg()
   (无)                              →  sw_i2c_write_reg()

 CMakeLists.txt (follow_robot):
   (无 sw_i2c)                      →  新增 sw_i2c 组件路径

 Kconfig.projbuild:
   (无 I2C 速度配置)                 →  新增 FOLLOW_ROBOT_I2C_SPEED_HZ
   YD-ESP32-S3 引脚提示 (无)         →  每个引脚都有 J1/J2 位置提示

 新增文件:
   components/sensors/sw_i2c/sw_i2c.h
   components/sensors/sw_i2c/sw_i2c.c
   components/sensors/sw_i2c/CMakeLists.txt
```

---

## 注意事项

1. **GPIO48 被占用**: YD-ESP32-S3 板载 WS2812B RGB LED 连接到 GPIO48，此引脚不可用作 IO
2. **波特率限制**: 软件 I2C 最高稳定 ~200kHz，推荐 100kHz
3. **内部上拉**: 软件 I2C 启用了 GPIO 内部上拉 (约 45kΩ)，长线建议外加 4.7kΩ 上拉
4. **软件 I2C 阻塞**: `sw_i2c_read_reg()` 是阻塞调用，读数期间不释放 CPU
5. **不后向感知**: 算法刻意不倒车，过近只停车
6. **编码器必须标定**: `TICKS_PER_METER` 推 1 米实测
7. **两个项目独立**: `follow_robot` 在根目录构建，`sensor_hub` 在 `examples/sensor_hub/` 构建
8. **RPLIDAR 需 5V/800mA 外部供电**

---

## 测试

传感器协议解析测试 (纯 C，无硬件依赖) 可在 PC 运行:

```bash
bash tests/protocol/run_tests.sh
```
