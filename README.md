# 智能跟随行李箱 · 算法4（传感器修复 + 闭环控制）

**算法4 = 传感器修改6（经验证可正常收数的传感器驱动） + 算法3（跟随避障 + 闭环底盘控制）**

在 `算法3` 基础上，用 `传感器修改6` 中经验证可正常接收数据的传感器驱动替换存在问题的传感器代码，保留算法3 的闭环控制逻辑（跟随 + VFH-lite 避障 + 状态机 + APO-DL 电调 + AB 编码器轮速 PID + IMU 航向闭环），并恢复 `#ifdef ESP_PLATFORM` 预编译条件以保持 PC 端单元测试兼容性。

---

## 算法版本演变

| 版本 | 传感器基础 | 控制算法 | 预编译条件 | 状态 |
|------|-----------|---------|-----------|------|
| 算法1 | 传感器修改系列 | 开环 H 桥 + 跟随/避障 | 有 `#if/#ifdef` | 历史 |
| 算法2 | 传感器修改系列 | 闭环 APO-DL ESC + AB 编码器 PID + IMU 航向 | 有 `#if/#ifdef` | 参考 |
| 算法3 | 传感器驱动有 Bug | 闭环控制（同算法2） | **全部移除**（导致传感器问题和 PC 测试不可编译） | 有问题 |
| **算法4**（本分支） | **传感器修改6（经验证可收数）** | 闭环控制（同算法2/3） | **恢复 ESP_PLATFORM 守卫** | ✅ 当前 |

---

## 算法4 修改清单

| 维度 | 算法3 | 算法4（本分支） |
|------|-------|----------------|
| 传感器代码 | 算法3 自带的传感器驱动（有问题） | **替代为传感器修改6 的经验证驱动** |
| 控制代码 | chassis.c 无 `#ifdef ESP_PLATFORM`（PC 测试不可编译） | **恢复算法2 的 ESP_PLATFORM 守卫**，PC 测试可用 |
| 传感器引脚 | 算法3 默认 | **沿用传感器修改6 的引脚分配**（I2C: 38/39, 超声波: 4/5, UWB: 6/7, RPLIDAR: 17/18） |
| 算法逻辑 | 跟随 + VFH-lite 避障 + 状态机 | **不变** |
| 底盘控制 | APO-DL ESC + AB 编码器 PID + IMU 航向 | **不变** |
| 预编译条件 | 无 | **恢复 `#ifdef ESP_PLATFORM`**（传感器 + 底盘） |
| 根项目 | sensor_hub（传感器测试） | **改为 follow_robot（完整跟随机器人）** |

---

## 硬件架构

```
  ┌──────────────────────────────────────────────────────────┐
  │                    ESP32-S3                               │
  │                                                          │
  │  UWB (BU0x)  ─── UART1 (GPIO6/7) ─── 跟随目标定位        │
  │  RPLIDAR C1  ─── UART2 (GPIO17/18) ── 激光避障场         │
  │  A02YYUW #1  ─── SW UART (GPIO4) ─── 前左近场安全        │
  │  A02YYUW #2  ─── SW UART (GPIO5) ─── 前右近场安全        │
  │  IMU          ─── I2C0 (SDA=39, SCL=38) ─ 航向闭环       │
  │                                                          │
  │  控制输出:                                                │
  │  左 ESC (APO-DL) ← GPIO4  RC PWM  50Hz                   │
  │  右 ESC (APO-DL) ← GPIO5  RC PWM  50Hz                   │
  │  左编码器 A/B   ← GPIO6/7   4x 正交解码 ISR              │
  │  右编码器 A/B   ← GPIO15/16 4x 正交解码 ISR              │
  └──────────────────────────────────────────────────────────┘
```

> **引脚说明**：算法4 的 `follow_robot` 项目默认使用上述引脚，可通过 `idf.py menuconfig` 修改。`sensor_hub` 项目仍使用传感器修改6 的引脚（超声波 GPIO4/5，I2C GPIO38/39）。

---

## 闭环底盘

- **电机驱动**：APO-DL ESC，RC PWM（50Hz，1000-2000us，1500us = 停止）
- **速度闭环**：每轮 AB 编码器 4x 解码 → 前馈 + PID 速度环（KP=200, KI=300, KD=5）
- **航向闭环**：IMU 偏航角积分参考航向，修正转弯指令（heading_kp=1500 milli）
- **安全机制**：0.3s 无指令超时自动停车，2s 上电 ESC 解锁延迟，加速度限幅

---

## 构建与烧录

### 主项目：follow_robot（完整跟随机器人）

```bash
# 在项目根目录
idf.py set-target esp32s3
idf.py menuconfig   # 配置引脚、PID、跟随参数等
idf.py build
idf.py flash monitor
```

Kconfig 菜单位于 `Follow-me suitcase 算法2` 菜单下，可配置：
- ESC 引脚/脉宽/反转
- AB 编码器引脚/反转/每米脉冲数
- 速度 PID 参数
- 跟随距离/停止带/最大速度
- 避障阈值（紧急/减速/安全距离）
- 传感器引脚（UWB/激光/超声波）
- IMU 航向闭环参数
- 控制循环频率

### 子项目：sensor_hub（传感器测试）

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build flash monitor
```

---

## PC 端测试（无需硬件）

```bash
# 传感器协议解析测试
bash tests/protocol/run_tests.sh

# 跟随避障算法 + PID + 运动学测试
bash tests/algorithm/run_tests.sh
```

测试覆盖：
- 差分驱动运动学（直行/原地旋转/饱和缩放）
- 轮速 PID（符号/限幅/积分抗饱和/闭环收敛/复位清零）
- 障碍物场（添加/最近保留/FOV 外忽略）
- 跟随行为（直行/转向目标/停止带保持）
- 避障（超声波急停/VFH 绕行）
- 搜索（目标丢失旋转/从未见过目标待机）

---

## 引脚对照表

### follow_robot 项目默认引脚（可通过 menuconfig 修改）

| 外设 | 接口 | 引脚 | 备注 |
|------|------|------|------|
| 左 ESC | LEDC PWM | GPIO4 | 50Hz RC 脉冲 |
| 右 ESC | LEDC PWM | GPIO5 | 50Hz RC 脉冲 |
| 左编码器 A | GPIO 中断 | GPIO6 | 4x 正交解码 |
| 左编码器 B | GPIO 中断 | GPIO7 | 4x 正交解码 |
| 右编码器 A | GPIO 中断 | GPIO15 | 4x 正交解码 |
| 右编码器 B | GPIO 中断 | GPIO16 | 4x 正交解码 |
| BU UWB | HW UART1 | RX=18, TX=8 | 115200 baud |
| RPLIDAR C1 | HW UART2 | RX=17, TX=9 | 460800 baud |
| 超声波左 | SW UART | RX=35 | 9600 baud |
| 超声波右 | SW UART | RX=36 | 9600 baud |
| IMU | I2C0 | SDA=11, SCL=12 | addr 0x23 |

### sensor_hub 项目引脚（传感器修改6 验证引脚，硬编码）

| 外设 | 接口 | 引脚 | 备注 |
|------|------|------|------|
| A02YYUW #1 | SW UART | RX=GPIO4 | 9600 baud |
| A02YYUW #2 | SW UART | RX=GPIO5 | 9600 baud |
| BU UWB | HW UART1 | RX=GPIO6, TX=GPIO7 | 115200 baud |
| FSR | ADC1 | GPIO8 (CH7) | 模拟输入 |
| RPLIDAR C1 | HW UART2 | RX=GPIO17, TX=GPIO18 | 460800 baud |
| IMU | I2C0 | SDA=GPIO39, SCL=GPIO38 | addr 0x23 |
| VL53L1X | I2C0 共享 | SDA=GPIO39, SCL=GPIO38 | addr 0x52(8位) |

---

## 目录结构

```
├── CMakeLists.txt                    # 根项目：follow_robot
├── README.md                         # 本文件
├── components/
│   ├── control/
│   │   ├── chassis/                  # 闭环底盘（ESC PWM + 编码器 PID）
│   │   │   ├── chassis.c/h
│   │   │   └── CMakeLists.txt
│   │   └── follow_avoid/            # 跟随 + VFH-lite 避障算法（纯 C）
│   │       ├── follow_avoid.c/h
│   │       └── CMakeLists.txt
│   └── sensors/
│       ├── a02yyuw/                  # A02YYUW 超声波（HW/SW UART 双模式）
│       ├── bu_uwb/                   # BU03/BU04 UWB 超宽带
│       ├── fsr_adc/                  # FSR 薄膜压力
│       ├── imu_i2c/                  # 九轴 IMU（I2C）
│       ├── rplidar_c1/              # RPLIDAR C1 激光雷达
│       └── vl53l1x_tof/             # VL53L1X ToF 激光测距
├── examples/
│   ├── follow_robot/                 # ★ 主程序：完整跟随机器人
│   │   ├── CMakeLists.txt
│   │   ├── sdkconfig.defaults
│   │   └── main/
│   │       ├── CMakeLists.txt
│   │       ├── Kconfig.projbuild
│   │       └── main.c
│   └── sensor_hub/                   # 传感器测试程序
│       ├── CMakeLists.txt
│       ├── sdkconfig.defaults
│       └── main/
│           ├── CMakeLists.txt
│           ├── Kconfig.projbuild
│           └── main.c
├── docs/
│   ├── algorithm/
│   │   └── follow-and-avoidance.md   # 算法设计文档
│   └── sensors/                      # 传感器使用文档
├── tests/
│   ├── algorithm/                    # 算法 PC 单元测试
│   │   ├── run_tests.sh
│   │   └── test_follow_avoid.c
│   └── protocol/                     # 传感器协议 PC 测试
│       ├── run_tests.sh
│       └── test_sensor_parsers.c
├── outputs/                          # 输出文件
├── scripts/                          # 辅助脚本
├── work/                             # 临时工作目录
└── AGENTS.md
```

---

## 关键代码导读

| 文件 | 作用 | 来源 |
|------|------|------|
| `components/sensors/*` | 传感器驱动（经验证可正常收数） | 传感器修改6 |
| `components/control/chassis/chassis.c` | 闭环底盘：ESC PWM + 编码器 PID + 前馈 | 算法2（恢复 ESP_PLATFORM 守卫） |
| `components/control/follow_avoid/follow_avoid.c` | 跟随 + VFH-lite 避障 + 状态机（纯 C） | 算法3 |
| `examples/follow_robot/main/main.c` | 主程序：多传感器 RTOS 任务 + 控制循环 | 算法3 |
| `examples/sensor_hub/main/main.c` | 传感器测试程序：7 路传感器并发读取 | 传感器修改6 |
| `tests/algorithm/test_follow_avoid.c` | 算法 + PID + 运动学 PC 单元测试 | 算法2 |
| `tests/protocol/test_sensor_parsers.c` | 传感器协议解析 PC 单元测试 | 传感器修改系列 |

---

## 注意事项

1. **引脚区分**：`follow_robot` 和 `sensor_hub` 是两个独立项目，引脚配置各自独立。`follow_robot` 默认使用 GPIO35/36（超声波）和 GPIO11/12（I2C），而 `sensor_hub` 使用传感器修改6 的 GPIO4/5（超声波）和 GPIO38/39（I2C）。
2. **RPLIDAR 需 5V/800mA 外部供电**，TX/RX 交叉接线。
3. **每个超声波只接 RX**，A02YYUW 自主输出，TX 不接。
4. **编码器每米脉冲数需标定**：推机器人恰好 1 米，记录计数差值。
5. **FSR 公式未标定**，使用前需实测标定。
6. **GPIO39 作 SDA** 仅 ESP32-S3 支持（经典 ESP32 的 GPIO34-39 仅为输入）。
7. **构建 follow_robot** 在项目根目录运行 `idf.py` 命令；构建 `sensor_hub` 需进入 `examples/sensor_hub/` 目录。
