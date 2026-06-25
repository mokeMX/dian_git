# 智能跟随行李箱 · 算法1（开环底盘版）

在 `传感器修改4` 的六传感器底座之上，新增一套**跟随 + 避障**算法，把 UWB（跟随目标）、激光雷达（前向障碍）、双超声波（近距兜底）、IMU、差速底盘整合为一台会自己跟着人走、并绕开障碍的智能行李箱。

> **这个分支是什么（算法1）**：**算法优先**版本。第一次把「跟随 + VFH-lite 避障 + 状态机」跑通，底盘按**开环 H 桥（TB6612/DRV8833 类）+ 占空比**假设来驱动。
>
> ⚠️ **底盘与真实硬件不完全一致**：项目真实动力是 **APO-DL 电调 + RC 航模 PWM 的无刷电机**（见 `动力轮代码` / `算法2`），不是 H 桥直流电机。所以本分支的 `chassis` 输出**不能直接驱动真实电调**——本分支重点是**算法逻辑**，硬件对接的闭环底盘在 `算法2`。要上真车请用 `算法2`。

---

## 三大件

- **`components/control/chassis/`** — 后轴差速底盘：`(v, ω)` → 左右轮占空比 → LEDC PWM。开环（无编码器），`max_speed_mps` 是把占空比换算成 m/s 的标定常数。面向 TB6612FNG / DRV8833 双 H 桥，引脚全部可配。
- **`components/control/follow_avoid/`** — **纯 C、不依赖 ESP/FreeRTOS** 的算法：UWB 跟随 + VFH-lite 避障 + 速度调速器 + 超声波急停 + 状态机（IDLE/SEARCH/FOLLOW/AVOID/ESTOP）。可在 PC 上单元测试。
- **`examples/follow_robot/`** — 整机示例：各传感器独立 FreeRTOS 任务 + 50Hz 控制循环。
- 设计、坐标系、接线与调参详见 **[docs/algorithm/follow-and-avoidance.md](docs/algorithm/follow-and-avoidance.md)**。

---

## 控制架构（要看的就是这套）

```
[uwb_task]   读 UWB → 目标 {距离, 方位, valid}        ┐
[lidar_task] 读雷达 → 前向障碍直方图 fa_obstacle_field ├→ 共享给
[ultra_l/r]  读双超声波 → 前角近距 fa_range            ┘   control_task
                                                          │  每 1/CONTROL_HZ 秒(默认50Hz)
                                  fa_update(target,field,ultraL,ultraR,dt)
                                                          │
                                  → (v_mps, ω_rps, state) → chassis_set_velocity()
```

- 控制循环用 `vTaskDelayUntil` 固定周期（默认 **50 Hz**，`CONFIG_FOLLOW_ROBOT_CONTROL_HZ`）。
- 传感器数据带 **0.5s 新鲜度**（`FIELD_FRESH_US`/`ULTRA_FRESH_US`）：过期即视为无效，避免拿旧数据开车。
- 任务优先级：control(7) > uwb/lidar(6) > ultra(5)。日志约 5Hz。

---

## 电机/接线默认引脚（`examples/follow_robot/main/Kconfig.projbuild`）

| 用途 | 默认 GPIO | 说明 |
|------|-----------|------|
| 左电机 PWM / IN1 / IN2 | GPIO4 / 5 / 6 | TB6612 A 路；`*_INVERT` 可反向 |
| 右电机 PWM / IN1 / IN2 | GPIO7 / 15 / 16 | TB6612 B 路 |
| PWM 频率 | 20kHz | 超出可听范围 |
| UWB RX / TX | GPIO18 / GPIO8 | 115200，给 follow 目标 |
| 跟随距离 | 1000 mm | `FOLLOW_DISTANCE_MM`，期望站在用户身后 1m |
| 其余阈值 | 见 Kconfig | 急停/减速/安全距离等，单位 mm，调参看算法文档 |

> 雷达/超声波/IMU 沿用 `传感器修改4` 的引脚与驱动；注意这些默认 GPIO 在本开发板上的占用情况（见传感器分支说明）。

---

## 构建、烧录与测试

```bash
cd examples/follow_robot
idf.py set-target esp32s3
idf.py menuconfig          # 配置电机引脚、跟随距离、避障阈值
idf.py build flash monitor

# 算法 PC 单元测试（无需硬件，gcc 编译纯 C 算法）
bash tests/algorithm/run_tests.sh
```

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `components/control/follow_avoid/follow_avoid.h` | 算法接口 + 设计注释 | **先读这个头文件的大段注释**：机体坐标系约定（+x 前、ω>0 左转）、状态机、`fa_config_t` 全部可调参数（跟随几何/速度加速度限/增益/障碍阈值/VFH 代价权重）。 |
| `components/control/follow_avoid/follow_avoid.c` | VFH-lite 实现 | `fa_update` 主流程：目标超时→SEARCH/IDLE、障碍直方图选最优航向、速度调速器、超声波/前锥急停。 |
| `components/control/chassis/chassis.h` | 底盘接口 | `chassis_set_velocity(v,ω)`、`chassis_diff_drive_mix`（过限时**整体缩放保转向几何**）、开环说明、`max_speed_mps` 标定、`min_duty` 死区补偿、`slew_per_call` 限幅。 |
| `examples/follow_robot/main/main.c` | 整机集成 | 各 `*_task` 怎么把传感器读数转成 `fa_*` 结构、`control_task` 的 50Hz 循环、新鲜度判断、`fa_update`→`chassis_set_velocity`。 |
| `examples/follow_robot/main/Kconfig.projbuild` | 配置 | 电机引脚、控制频率、跟随/避障阈值默认值。 |
| `tests/algorithm/test_follow_avoid.c` | PC 单测 | 不接硬件就能验证算法行为，改算法后先跑它。 |
| `docs/algorithm/follow-and-avoidance.md` | 设计/调参文档 | 坐标系、状态机、每个参数怎么调。 |

---

## ⚠️ 注意事项 / 安全（这是会自己跑的车，务必先看）

1. **第一次必须「架空轮子」测试**：把箱体垫起来让驱动轮离地，确认方向、转向、急停都对，再落地。开环 + 真人跟随，调错方向会直接撞上去。
2. **底盘不匹配真实电调**：本分支 `chassis` 是 H 桥占空比驱动；真车是 **APO-DL 电调 + RC PWM**。直接拿本分支驱动真实无刷电机**不会动或行为异常**——真车用 `算法2`。
3. **开环没有速度反馈**：电池电量、地面摩擦、负载都会改变实际速度，`max_speed_mps` 只是标定值。坡道/电量低时实际速度会偏离指令，跟随距离会漂。要精确请上 `算法2` 的编码器 PID 闭环。
4. **UWB 要能给「方位」不只是距离**：`fa_target_t` 需要 `bearing_rad`。单纯测距的 UWB 给不出方位，本项目靠 BU0x 的 PDOA/TWR 输出 X/Y 反算方位；UWB 数据质量直接决定跟随是否乱转。
5. **ESTOP / 新鲜度**：前锥或超声波过近会进 ESTOP 硬停；传感器数据超过 0.5s 没更新就当无效。如果车「无故停住」，先看是不是某路传感器掉数据。
6. **先标定再上路**：`max_speed_mps`、`min_duty`（死区）、`track_width_m`（轮距）必须按实车测量/标定，否则转向半径和速度都不准。
7. **调参全在算法文档**：不要散乱改源码常量，改 `fa_config_t` / Kconfig，对照 `docs/algorithm/follow-and-avoidance.md`。

---

## 目录结构（算法相关）

```
├── components/control/
│   ├── chassis/                # 差速底盘（开环 v,ω → 占空比 → LEDC PWM）
│   └── follow_avoid/           # 纯 C 跟随+VFH避障+状态机（可 PC 单测）
├── components/sensors/         # 六传感器驱动（同 传感器修改4）
├── examples/follow_robot/
│   └── main/
│       ├── main.c              # 传感器任务 + 50Hz 控制循环 + 底盘
│       └── Kconfig.projbuild   # 电机引脚 / 控制频率 / 跟随避障阈值
├── docs/algorithm/
│   └── follow-and-avoidance.md # 设计 / 坐标系 / 调参
├── tests/algorithm/            # 算法 PC 单元测试
└── docs/sensors/, tests/protocol/  # 传感器文档与协议测试（同 传感器修改4）
```
