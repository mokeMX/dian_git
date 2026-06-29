# 智能跟随行李箱 · 算法3（无预编译条件版）

在 `算法2` 基础上，**移除所有 `#if` / `#ifdef` / `#ifndef` / `#endif` 预编译指令**，代码无条件包含全部传感器驱动和闭环控制逻辑。算法逻辑与算法2完全一致：上层 UWB 跟随 + VFH-lite 避障 + 状态机，底层 APO-DL 电调（RC PWM）+ AB 正交编码器轮速 PID + IMU 航向闭环。

---

## 算法2 → 算法3 改了什么

| 维度 | 算法2 | 算法3（本分支） |
|------|-------|----------------|
| 预编译条件 | 大量 `#ifdef ESP_PLATFORM` / `#if CONFIG_*` / `#ifdef __cplusplus` | **全部移除**，代码无条件编译 |
| ESP_PLATFORM 分叉 | ESP 实现 + PC stub（`#else` 分支） | 仅保留 ESP 实现，删除 PC stub |
| Kconfig 条件 | `#if CONFIG_*` 可选传感器 | 所有传感器驱动无条件包含 |
| C++ 守卫 | `#ifdef __cplusplus` / `extern "C"` | 移除 |
| 头文件守卫 | `#ifndef _HEADER_H_` 包含守卫 | 统一使用 `#pragma once` |
| 算法逻辑 | — | **不变**（跟随 + VFH 避障 + 状态机 + PID + IMU 航向闭环） |

---

## 闭环底盘

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

## 电机/编码器接线

| 用途 | 默认 GPIO | 说明 |
|------|-----------|------|
| 左电调信号 | GPIO4 | RC PWM 输出（50Hz，1000–2000µs） |
| 右电调信号 | GPIO5 | 同上 |
| 左编码器 A / B | GPIO6 / GPIO7 | 4× 正交解码（GPIO 中断） |
| 右编码器 A / B | GPIO15 / GPIO16 | 同上 |
| ESC 中位/下限/上限 | 1500 / 1000 / 2000 µs | 按电调实际行程改 |
| ESC 解锁时长 | 2000 ms | 上电先保持中位让电调自检解锁 |
| 每米脉冲 `TICKS_PER_METER` | 2000 | **必须标定**（推 1m 读编码器增量） |
| 速度 PID `SPEED_KP/KI/KD` | 200 / 300 / 5 | µs per (m/s) |

---

## 构建、烧录与测试

```bash
cd examples/follow_robot
idf.py set-target esp32s3
# 参数直接在代码中以 #define 定义，无需 menuconfig
idf.py build flash monitor

# 算法 + 轮速 PID 的 PC 单元测试（无需硬件）
bash tests/algorithm/run_tests.sh
```

---

## 关键代码导读

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `components/control/chassis/chassis.h` | 闭环底盘接口 + 设计注释 | 电调 RC-PWM 约定、AB 编码器 4× 解码、`chassis_pid_t`、`chassis_set_velocity`+`chassis_update` 两段式用法 |
| `components/control/chassis/chassis.c` | 实现 | ESC 脉冲生成（LEDC 14位@50Hz）、编码器 ISR 查表、PID step、前馈映射 |
| `components/control/follow_avoid/follow_avoid.h/.c` | 上层算法 | 坐标系约定、状态机、`fa_update`、`fa_config_t` 调参项 |
| `examples/follow_robot/main/main.c` | 整机集成 | 传感器任务 + 互斥快照 + 50Hz 控制循环 + IMU 航向闭环 → `chassis_set_velocity` → `chassis_update` |
| `tests/algorithm/test_follow_avoid.c` | PC 单测 | 算法 + 轮速 PID 不接硬件即可验证 |
| `docs/algorithm/follow-and-avoidance.md` | 完整设计/调参/调试文档 | 闭环底盘、参数表、上电调试顺序、安全限制 |

---

## 注意事项 / 安全

1. **垫高轮子空跑 + 可随时断电**：上电 2s 内不要给指令（让电调解锁），先确认方向/转向/急停。
2. **必标定三件事**：`TICKS_PER_METER`（推 1m 读增量）、`MAX_WHEEL_SPEED_MMPS`（满脉冲实测轮速）、速度 PID。
3. **编码器方向**：前进时计数应变大；不对就 `*_ENC_INVERT`。
4. **不倒车**：车尾无传感器，过近只停车不后退。
5. **失效保护**：控制任务卡死会在 0.3s 内回中位停车。
6. **调参改代码中常量 / `fa_config`**，对照算法文档。

---

## 目录结构

```
├── components/control/
│   ├── chassis/                # 闭环底盘：电调RC-PWM + AB编码器轮速PID + 前馈 + 失效保护 + 里程计
│   └── follow_avoid/           # 纯 C 跟随+VFH避障+状态机（可 PC 单测）
├── components/sensors/         # 六传感器驱动（UWB/雷达/双超声波/IMU/ToF/FSR）
├── examples/follow_robot/      # 整机集成示例
├── examples/sensor_hub/        # 传感器测试集线器
├── docs/algorithm/             # 设计文档
└── tests/algorithm/            # 算法 + 轮速 PID PC 单元测试
```
