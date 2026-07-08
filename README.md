# 智能跟随行李箱 · follow_only（跟随代码 1.4 · 脉宽直接控制版）

UWB 跟随 + IMU 航向闭环（可选）+ **ESC 脉宽直接控制** + WiFi 调试网页 + Flash CSV 日志，无避障。

> 本分支 `跟随代码1.4脉宽修改` 在 1.3 基础上把底盘输出改为 **直接下发 ESC 脉宽（PWM）**，
> 绕过轮速 PID 闭环，专门用于在 `meas_v_mps` 长期为 0（编码器无反馈）时验证 ESC / 电机链路，
> 并修复了一批 HTTP 服务器并发、UWB 任务饿死、E-STOP 竞态等稳定性问题。

---

## 相比 1.3 的改动（本分支要点）

| 模块 | 变更 | 目的 |
|------|------|------|
| `control_task` | `FR_USE_DIRECT_PULSE_CONTROL=1`：调用 `chassis_set_pulse_us()` 直接写脉宽，不走速度 PID | 编码器无反馈时也能驱动电机 |
| `live_handler` | JSON 缓冲区由全局 `static` 改为**栈分配** | 消除 `/live` 与 `/status` 并发共享缓冲的竞态 |
| `uwb_task` | 连续解析错误 10 次后 `vTaskDelay(1)`；UART 硬错误也让出一个 tick | 防止垃圾帧刷屏时占满 CPU 饿死 HTTP 服务器（prio5） |
| `estop_handler` | 先 `remote_estop()` 再置 `s_estop_pending` 标志 | 消除抢占竞态导致的“多动一拍”（~20ms 误动作） |
| `flash_log_task` | 每次写入立即 `fflush()` | 保证 `/log` 下载看到最新数据 |
| 前端 JS | `fetch` 超时保护（POST 3s / hb 2s / poll 5s），轮询间隔 300→500ms | 弱网/断连下网页不卡死 |
| FOLLOW 状态 | 合并 `stop_band` 冗余 `else` 分支 | 逻辑简化：过近一律停车、不倒车 |
| `FR_LIVE_JSON_BUF_SIZE` | 8192 → 4096 | 减小 HTTP 任务栈压力（实际 JSON ~1.3KiB） |
| 新增 `FR_STARTUP_AUTO_ARM` | 默认 0 | 台架调试时可跳过手动 CLEAR/ARM |
| 新增 `examples/follow_only1/` | 纯 UWB 开环示例（无 IMU、无 PID） | 对照工程 |

---

## 系统架构

```
ESP32-S3
├── uwb_task (优先级6)      ← UART1 接收 UWB 目标定位，EMA 滤波 + 跳点剔除
├── control_task (优先级7)  ← 50Hz: 跟随算法 → 差速运动学 → ESC 脉宽直发
├── flash_log_task (优先级2)← 异步写 CSV 到 SPIFFS（每写即 flush）
└── HTTP server (WiFi SoftAP, 优先级5)
    ├── GET  /             ← 调试网页
    ├── GET  /live /status ← 实时遥测 JSON
    ├── POST /hb           ← 心跳（500ms）
    ├── POST /estop        ← 急停（锁存）
    ├── POST /clear        ← 解锁并 ARM
    ├── POST /motion_off   ← 仅停运动，继续采数
    ├── POST /clear_log    ← 清空 CSV
    └── GET  /log          ← 下载 CSV 日志
```

**启动时序（安全优先）：**
1. 先起 SoftAP + HTTP，**必须**手机连上并产生第一个心跳，才继续初始化机器人；
2. 再挂载 SPIFFS 日志、初始化底盘（ESC 保持中位 arm）、UWB、IMU；
3. 默认 `estop_latched=true / motion_armed=false`，需在网页按 **CLEAR/ARM** 才允许运动。

**传感器 / 执行器：**
- UWB (BU0x) — UART1 115200，TWR 坐标 → 距离 + 方位；EMA（τ=0.35s）平滑 + 跳点剔除（连续 5 次跳变后重置滤波）
- IMU (I2C) — 航向角闭环修正（本分支默认 `FR_ENABLE_IMU=0` 关闭）
- 编码器 (GPIO 中断) — 4x 正交解码，仅用于测量 `meas_v/meas_w`（脉宽直控模式下不参与闭环）
- APO-DL ESC × 2 — RC PWM 50Hz（GPIO4/5），1000/1500/2000us = 倒/停/正

---

## 脉宽直接控制原理

控制环每周期（50Hz）执行：

```
UWB(距离,方位) ─→ 跟随算法 ─→ (v, ω) ─→ 差速运动学 ─→ 左/右轮速 ─→ 脉宽映射 ─→ ESC
```

1. **跟随算法**（`FOLLOW` 状态）
   - 线速度：`v = kp_dist * (range − follow_distance)`，仅当过远时前进，过近只停不倒车；
   - 角速度：`ω = kp_bear * bearing`，并按转角做 `turn_scale` 减速；
   - 可选 IMU 航向闭环：`ω += heading_kp * (yaw_ref − yaw_meas)`。
2. **差速运动学**（`velocity_to_direct_pulses`）
   - `l = v − ω·半轴距`，`r = v + ω·半轴距`；
   - 超过 `max_wheel_mps` 时按比例缩放，保持曲率。
3. **脉宽映射**（`wheel_speed_to_pulse_us`）
   - 轮速线性映射到 `[esc_min, esc_mid, esc_max]`，支持左右反转；
   - 未 ARM / STOP / 心跳门控不通过时，强制中位（`esc_mid`）。
4. **下发**：`chassis_update()` 先刷新编码器测量，再 `chassis_set_pulse_us()` 直接写脉宽。

> 切回速度闭环：将 `FR_USE_DIRECT_PULSE_CONTROL` 置 0，即改用 `chassis_set_velocity()` + 轮速 PID。

---

## 跟随状态机

```
IDLE ──目标有效──→ FOLLOW ──丢失>0.5s──→ SEARCH ──搜索超时(6s)──→ IDLE
  ▲                   ▲                     │
  └──────目标重捕──────┴─────────────────────┘
```

- **FOLLOW**：距离 P 控线速度 + 方位 P 控角速度 + 转向减速 +（可选）IMU 航向闭环
- **SEARCH**：朝最后已知方向旋转搜索
- **IDLE**：停车

---

## 安全机制

| 保护 | 行为 |
|------|------|
| STOP 按钮 | 锁存 `estop_latched=true`，需重新 CLEAR/ARM |
| Fast-path E-STOP | `s_estop_pending` 标志，控制环在任何算法逻辑前优先处理 |
| 未 ARM / 已 STOP | 应用速度强制 0，脉宽输出中位 |
| 命令超时 | 目标速度按减速斜率归零 |
| 启动门控 | 手机未连接 + 无心跳前不启动机器人 |
| ESC arm | 上电保持中位 2s（`ESC_ARM_MS`）等待 ESC 自检 |

> 注意：本分支按需求**不再**因心跳超时 / WiFi 断连自动解除 ARM，只有 STOP 才会 disarm。
> 心跳超时阈值 `FR_REMOTE_HB_TIMEOUT_US` 被设得很大，字段仅用于遥测显示。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py menuconfig    # 配置引脚、PID、跟随参数
idf.py build
idf.py flash monitor
```

烧录后：
1. 手机连接热点 **SSID `FollowRobot-UWB` / 密码 `12345678`**；
2. 浏览器打开 **http://192.168.4.1**；
3. 网页显示实时遥测；按 **CLEAR/ARM** 解锁电机，**STOP** 急停。

### 关键编译开关（main.c 顶部宏）

| 宏 | 默认 | 说明 |
|----|------|------|
| `FR_USE_DIRECT_PULSE_CONTROL` | 1 | 脉宽直发（本分支核心）；0 = 速度 PID 闭环 |
| `FR_ENABLE_IMU` | 0 | 是否启用 IMU 航向闭环 |
| `FR_ENABLE_MOTION` | 1 | 总运动使能（0 = 只采数不动） |
| `FR_STARTUP_AUTO_ARM` | 0 | 台架调试自动 ARM（**现场必须 0**） |
| `FR_USE_MOTION_DEBUG_LIMITS` | 0 | 额外硬限速（台架早期用） |
| `FR_LEFT_INVERT / FR_RIGHT_INVERT` | true | ESC 方向反转 |

### 关键参数 (Kconfig)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance_mm` | 1000 | 期望跟车距离 |
| `max_linear_mmps` | 700 | 最大前进速度 |
| `max_wheel_speed_mmps` | 800 | 单轮最高速（脉宽映射满量程） |
| `kp_dist` / `kp_bear` | 900 / 1600 | 距离/方位 P 增益 (milli) |
| `heading_kp_milli` | 1500 | IMU 航向闭环增益 |
| `esc_min/mid/max_us` | 1000/1500/2000 | ESC 脉宽标定 |
| `ticks_per_meter` | 2000 | 编码器每米脉冲数（测量用，**须标定**） |
| `speed_kp/ki/kd` | 200/300/5 | 轮速 PID（仅闭环模式生效） |
| `control_hz` | 50 | 控制循环频率 |
| `target_fresh_ms` | 700 | UWB 数据新鲜度阈值 |

---

## 引脚

| 外设 | GPIO | 说明 |
|------|------|------|
| 左 ESC | 4 | RC PWM 50Hz |
| 右 ESC | 5 | RC PWM 50Hz |
| 左编码器 A/B | 6/7 | 4x 正交解码 |
| 右编码器 A/B | 15/16 | 4x 正交解码 |
| UWB RX/TX | 18/37 | UART1 115200 |
| IMU SDA/SCL | 39/38 | I2C0（GPIO39 作 SDA 仅 ESP32-S3 支持） |

---

## 实时遥测 / 日志字段

网页 `/live` 与 CSV 日志包含：目标距离/方位、UWB 原始值与滤波值、帧计数（TWR/range/parse_error/outlier）、
算法输出 `algo_v/w`、斜率后 `ramp_v/w`、实际下发 `applied_v/w`、**直发脉宽 `direct_left/right_us`**、
底盘返回码 `chassis_update_ret / chassis_pulse_ret`、编码器测量 `meas_v/w`、遥控状态与心跳。

日志写入 SPIFFS `logs` 分区 `/spiffs/follow_log.csv`（5Hz），可从网页 `/log` 下载、`/clear_log` 清空。

---

## 目录结构

```
├── CMakeLists.txt                         # 根项目 (project=follow_only)
├── partitions.csv                         # 含 logs SPIFFS 分区
├── sdkconfig.defaults                     # esp32s3 / 自定义分区 / HTTP 头缓冲
├── components/
│   ├── control/chassis/                   # 底盘：ESC PWM + 编码器 + 轮速 PID + set_pulse_us
│   ├── control/web_control/               # 2D 遥控组件
│   ├── debug/web_debug/                   # WiFi 调试组件
│   └── sensors/{bu_uwb, imu_i2c}/         # UWB / IMU 驱动
└── examples/
    ├── follow_only/                       # ★ 本分支主固件（脉宽直控 + 调试网页）
    └── follow_only1/                      # 纯 UWB 开环对照（无 IMU / 无 PID）
```

---

## 注意事项

1. **脉宽直控模式下编码器不参与闭环**，只用于遥测；`meas_v/w` 仅供观察。
2. ESC 方向 / 中位务必标定：先在 STOP 状态确认电机不动，再逐步 ARM 测试。
3. 编码器 `TICKS_PER_METER` 若要用于测量须物理标定：推机器人 1m，读日志 tick delta。
4. 无后向感知：算法不倒车，过近只停车。
5. `FR_STARTUP_AUTO_ARM` 现场务必保持 0，否则上电见到 UWB 目标即自动运动。
6. HTTP/WiFi 任务绝不直接操作底盘，所有运动仲裁集中在 `control_task`。
</content>
</invoke>
