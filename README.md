# 智能跟随行李箱 · 算法2（闭环版 · 对接真实硬件）

在 `传感器修改4` 六传感器底座 + `算法1` 跟随避障算法之上，把底盘换成**项目真实的执行硬件**并做成**闭环**：APO-DL 电调（RC 航模 PWM）驱动无刷电机，AB 正交编码器做轮速 PID 闭环，IMU 做航向闭环。指令 `(v, ω)` 会被**真正跟踪**，而不再是开环估计。

> **这个分支是什么（算法2）**：整条线的**集大成 / 上真车版本**。上层「UWB 跟随 + VFH-lite 避障 + 状态机」沿用算法1（仍是纯 C、可 PC 单测），**底层执行换成闭环**。要把行李箱真正跑起来，用这个分支。底盘引脚与电机驱动方式与独立的 `动力轮代码` 分支一致。

---

## 算法1 → 算法2 改了什么

| 维度 | 算法1 | 算法2（本分支） |
|------|-------|----------------|
| 电机驱动 | H 桥 + 开环占空比（假设） | **APO-DL 电调 + RC PWM**（50Hz，1000–2000µs，1500=停，LEDC 14位） |
| 速度控制 | 无反馈 | **AB 编码器 + 每轮速度 PID**（前馈 + 抗积分饱和 + 测量微分 + 斜率限幅） |
| 航向 | 直接用算法 ω | **IMU 航向闭环**：ω 作前馈，IMU 偏航误差修正 |
| 安全 | — | 失效保护（0.3s 无指令回中位）、上电 2s 解锁、里程计 |
| 上层算法 | 跟随 + VFH 避障 + 状态机 | **不变**（沿用，且补轮速 PID 的 PC 单测） |

---

## 闭环底盘（本分支核心）

```
follow_avoid 给出 (v, ω)
        │
        ▼  差速运动学
 每轮目标轮速 (m/s)
   │                          ┌── 前馈 ff = (2000-1500)/Vmax · 目标轮速 ──┐
   ├──────────────────────────┤                                          ├─► 电调脉冲 = 1500 + 限幅(ff+pid)
   │   AB 编码器 4×计数/dt → 实测轮速 → 误差 → PID(kp,ki,kd) → 修正 µs ──┘        │
   └────────────────────────────────────────────────────────────────────────► APO-DL 电调 → 无刷电机
```

- **前馈**保证编码器掉线也能走（优雅降级），**PID** 修掉误差/负载/左右不一致。
- 目标为 0 → 强制中位 + 清积分（电调不蠕动）；脉冲经斜率限幅护齿轮/限流。
- **IMU 航向闭环**只在 FOLLOW/AVOID 启用；SEARCH/ESTOP 需自由旋转时自动关闭并重置参考。

---

## 电机/编码器接线（与 `动力轮代码` 对齐，`examples/follow_robot/main/Kconfig.projbuild`）

| 用途 | 默认 GPIO | 说明 |
|------|-----------|------|
| 左电调信号 | GPIO4 | RC PWM 输出（50Hz，1000–2000µs） |
| 右电调信号 | GPIO5 | 同上 |
| 左编码器 A / B | GPIO6 / GPIO7 | 4× 正交解码（GPIO 中断） |
| 右编码器 A / B | GPIO15 / GPIO16 | 同上 |
| ESC 中位/下限/上限 | 1500 / 1000 / 2000 µs | 按电调实际行程改 |
| ESC 解锁时长 | 2000 ms | 上电先保持中位让电调「滴滴」自检解锁 |
| 每米脉冲 `TICKS_PER_METER` | 2000 | **必须标定**（推 1m 读编码器增量） |
| 速度 PID `SPEED_KP/KI/KD` | 200 / 300 / 5 | µs per (m/s)；调参见算法文档第7/10节 |

> 传感器（UWB/雷达/双超声波/IMU）沿用 `传感器修改4` 的驱动与引脚。

---

## 构建、烧录与测试

```bash
cd examples/follow_robot
idf.py set-target esp32s3
idf.py menuconfig        # 电调/编码器引脚、TICKS_PER_METER、车距、避障阈值、PID、航向增益
idf.py build flash monitor

# 算法 + 轮速 PID 的 PC 单元测试（无需硬件）
bash tests/algorithm/run_tests.sh
```

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `components/control/chassis/chassis.h` | 闭环底盘接口 + 设计注释 | **先读头部大段注释**：电调 RC-PWM 约定、AB 编码器 4× 解码、`chassis_pid_t`（kp/ki/kd 单位 µs/(m/s)）、`chassis_set_velocity`+`chassis_update(dt)` 两段式用法、失效保护、`chassis_get_odometry`。 |
| `components/control/chassis/chassis.c` | 实现 | ESC 脉冲生成（LEDC 14位@50Hz）、编码器 ISR 查表、PID step、前馈映射、arm/failsafe。 |
| `components/control/follow_avoid/follow_avoid.h/.c` | 上层算法（同算法1） | 坐标系约定、状态机、`fa_update`、`fa_config_t` 调参项。 |
| `examples/follow_robot/main/main.c` | 整机集成 | 传感器任务 + 互斥快照 + 50Hz 控制循环 + **IMU 航向闭环**（`yaw_ref += ω·dt`，误差经 `HEADING_KP` 修正）→ `chassis_set_velocity` → `chassis_update`。 |
| `examples/follow_robot/main/Kconfig.projbuild` | 全部参数 | 电调/编码器引脚、TICKS_PER_METER、PID、航向增益、各取反开关。 |
| `tests/algorithm/test_follow_avoid.c` | PC 单测 | 算法 + 轮速 PID 不接硬件即可验证。 |
| `docs/algorithm/follow-and-avoidance.md` | **完整设计/调参/调试文档** | 第6节闭环底盘、第7节参数表、**第10节上电调试顺序**、第11节安全限制——上车前必读。 |

---

## ⚠️ 注意事项 / 安全（真车闭环，务必按顺序来）

1. **垫高轮子空跑 + 可随时断电**：上电 2s 内**不要给指令**（让电调解锁），先确认方向/转向/急停。
2. **必标定三件事**：`TICKS_PER_METER`（推 1m 读增量）、`MAX_WHEEL_SPEED_MMPS`（满脉冲实测轮速）、速度 PID。不标定则速度跟踪不准。
3. **编码器方向**：前进时计数应变大；不对就 `*_ENC_INVERT`。**取反电机后编码器方向通常也要跟着改**。
4. **电机转反**：若已像 `动力轮代码` 那样对调电机蓝白线，保持 `MOTOR_*_INVERT=关`；否则用取反开关，别两处都改。
5. **优雅降级要心里有数**：编码器掉线→退化为纯前馈开环（仍会动但不精确）；IMU 掉线→跳过航向闭环。所以「能动」不代表「闭环在工作」，看日志确认 `meas v` / 航向修正是否正常。
6. **不倒车**：车尾无传感器，过近只停车不后退——这是有意的安全设计。
7. **失效保护**：控制任务卡死会在 0.3s 内回中位停车；首测仍请手放断电开关。
8. **调参改 Kconfig / `fa_config`，别散改常量**，对照算法文档第7/10节。

---

## 目录结构（算法相关）

```
├── components/control/
│   ├── chassis/                # 闭环底盘：电调RC-PWM + AB编码器轮速PID + 前馈 + 失效保护 + 里程计
│   └── follow_avoid/           # 纯 C 跟随+VFH避障+状态机（同算法1，可 PC 单测）
├── components/sensors/         # 六传感器驱动（同 传感器修改4）
├── examples/follow_robot/
│   └── main/
│       ├── main.c              # 传感器任务 + 50Hz 控制循环 + IMU 航向闭环 + 闭环底盘
│       └── Kconfig.projbuild   # 电调/编码器引脚、TICKS_PER_METER、PID、航向增益、跟随避障阈值
├── docs/algorithm/
│   └── follow-and-avoidance.md # 设计 / 坐标系 / 闭环底盘 / 调参 / 上电调试顺序 / 安全（详尽）
├── tests/algorithm/            # 算法 + 轮速 PID PC 单元测试
└── docs/sensors/, tests/protocol/  # 传感器文档与协议测试（同 传感器修改4）
```
