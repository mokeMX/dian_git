# 智能跟随行李箱 · 算法4（传感器修复 + 闭环控制 + 无预编译条件）

**算法4 = 传感器修改6（经验证可正常收数） + 算法2/3 闭环控制逻辑 + 全部移除 #if/#ifdef**

以 `传感器修改6` 的经验证传感器驱动为基础，整合 `算法2/3` 的闭环控制逻辑，并**移除全部 `#if` / `#ifdef` / `#ifndef` / `#else` / `#endif` 预编译指令**，代码无条件包含全部传感器驱动和闭环控制逻辑。

---

## 算法版本演变

| 版本 | 传感器基础 | 控制算法 | 预编译条件 | 状态 |
|------|-----------|---------|-----------|------|
| 算法1 | 传感器修改系列 | 开环 H 桥 + 跟随/避障 | 有 `#if/#ifdef` | 历史 |
| 算法2 | 传感器修改系列 | 闭环 APO-DL ESC + AB 编码器 PID + IMU 航向 | 有 `#if/#ifdef` | 参考 |
| 算法3 | 传感器驱动有 Bug | 闭环控制（同算法2） | 全部移除（导致传感器问题） | 有问题 |
| **算法4**（本分支） | **传感器修改6（经验证可收数） + 全部 #if 已清理** | 闭环控制（同算法2/3） | **全部移除** | ✅ 当前 |

---

## 算法3 → 算法4 修改清单

| 维度 | 算法3 | 算法4（本分支） |
|------|-------|----------------|
| 传感器代码 | 算法3 自带（有问题） | **替换为传感器修改6 经验证驱动，并移除其 #if 指令** |
| 控制代码 | chassis.c 无 `#ifdef` | **同样无 `#ifdef`**（底盘代码逻辑不变） |
| 预编译条件 | 全部移除 | **全部移除**（传感器 + 底盘 + 示例代码，无任何 `#if`） |
| PC stub | 已移除 | **已移除**（仅保留 ESP 实现） |
| 算法逻辑 | 跟随 + VFH-lite 避障 + 状态机 | **不变** |
| 底盘控制 | APO-DL ESC + AB 编码器 PID + IMU 航向 | **不变** |
| 根项目 | sensor_hub | **改为 follow_robot**（完整跟随机器人） |

---

## 系统总体架构

```
                          ┌─────────────────────────┐
                          │      ESP32-S3            │
                          │                          │
   UWB (BU0x) ───────────►│ UART1 (GPIO6/7)          │
                          │   ↓ uwb_task (优先级6)    │
   RPLIDAR C1 ───────────►│ UART2 (GPIO17/18)        │
                          │   ↓ lidar_task (优先级6)  │
   A02YYUW #1 ───────────►│ SW UART (GPIO4)          │
                          │   ↓ ultra_task_L (优先5)  │
   A02YYUW #2 ───────────►│ SW UART (GPIO5)          │
                          │   ↓ ultra_task_R (优先5)  │
   IMU ──────────────────►│ I2C0 (GPIO38/39)         │
                          │   ↓ (control_task 内读取) │
                          │                          │
                          │   ┌─────── 共享快照 ──────┐│
                          │   │ g_shared (互斥锁保护) ││
                          │   │ · 目标距离/方位 + 时间戳││
                          │   │ · 障碍物直方图 + 时间戳 ││
                          │   │ · 左/右超声波距离 + 时间戳││
                          │   └───────────────────────┘│
                          │          ↓                  │
                          │   control_task (50Hz, 优先7)│
                          │   ┌───────────────────────┐│
                          │   │ 1. 快照传感器数据      ││
                          │   │ 2. fa_update() 跟随避障││
                          │   │ 3. IMU 航向闭环修正    ││
                          │   │ 4. chassis_set_velocity││
                          │   │ 5. chassis_update()    ││
                          │   └───────────────────────┘│
                          │          ↓                  │
   左 ESC (APO-DL) ◄──────│ GPIO4  RC PWM 50Hz        │
   右 ESC (APO-DL) ◄──────│ GPIO5  RC PWM 50Hz        │
   左编码器 A/B ◄─────────│ GPIO6/7  (4x 正交解码 ISR) │
   右编码器 A/B ◄─────────│ GPIO15/16 (4x 正交解码 ISR) │
                          └─────────────────────────┘
```

---

## 传感器层详解

### 1. UWB 超宽带定位 —— 跟随目标

| 属性 | 值 |
|------|-----|
| 型号 | BU03 / BU04（Ai-Thinker） |
| 接口 | HW UART1，RX=GPIO6，TX=GPIO7 |
| 波特率 | 115200 |
| 协议 | JSxxxx{"TWR":...} JSON 帧，或 "distance: X.XX" 纯距离行 |
| 运行任务 | `uwb_task`，栈 4096，优先级 6 |
| 输出 | 目标距离 `tgt_distance_m`、方位 `tgt_bearing_rad`，带时间戳 |

UWB 模块持续输出定位数据。`uwb_task` 逐字符读取 UART 行，调用 `bu_uwb_parse_twr_line()` 解析完整
TWR JSON 帧，从中提取 `Xcm`（横向坐标）、`Ycm`（纵向坐标）和 `distance_cm`（距离），换算为机体坐标系
下的距离与方位角（`atan2(Xcm, Ycm)` + 左右符号修正）。如果模块输出的是简化的 `distance:` 行，则仅更新
距离，保留上次已知方位。解析结果写入互斥锁保护的全局快照 `g_shared`，附带微秒时间戳。

**新鲜度窗口**：`TARGET_FRESH_US = 700ms`。超过 0.7 秒未收到新数据，目标判为丢失，算法进入 SEARCH 状态。

### 2. RPLIDAR C1 激光雷达 —— 障碍物感知

| 属性 | 值 |
|------|-----|
| 型号 | Slamtec RPLIDAR C1 |
| 接口 | HW UART2，RX=GPIO17，TX=GPIO18 |
| 波特率 | 460800 |
| 协议 | 5 字节扫描数据包（角度 Q6、距离 Q2、品质、起始标志） |
| 运行任务 | `lidar_task`，栈 4096，优先级 6 |
| 输出 | 前向 180° 极坐标障碍物直方图 `fa_obstacle_field_t`（36 扇区，5°/扇区） |

`lidar_task` 初始化雷达后发送 `SCAN` 命令，进入连续扫描。每收到一个 5 字节数据点，调用
`rplidar_c1_read_point()` 解析出角度（Q6 格式，÷64 换算为度）、距离（Q2 格式，÷4 换算为 mm）
和品质。角度经过机体坐标系转换（`lidar_angle_to_body_rad()`：偏移前向零位 + 旋向翻转），
然后调用 `fa_obstacle_add()` 填入极坐标直方图。当检测到 `start_bit`（新一圈起始标志）时，
将刚完成的一圈直方图发布到 `g_shared.field`，同时复位直方图开始下一圈。

**新鲜度窗口**：`FIELD_FRESH_US = 500ms`。超过 0.5 秒未收到新一圈数据，算法退化为仅靠超声波避障。

**数据格式**：
- 角度：0–360°（CW），通过 `LIDAR_FORWARD_DEG` 偏移和 `LIDAR_CW` 翻转映射到机体坐标系
- 距离：0–12000mm，品质 0 的点被过滤
- 仅前向 ±90°（180° FOV）内的点参与构建障碍物直方图

### 3. A02YYUW 超声波传感器 —— 近场安全兜底

| 属性 | 值 |
|------|-----|
| 型号 | A02YYUW（30–4500mm） |
| 数量 | 2 个（前左 + 前右） |
| 接口 | 软件 UART（bit-bang），RX=GPIO4（左）/ GPIO5（右），TX 不接 |
| 波特率 | 9600 |
| 协议 | 4 字节帧：0xFF 头 + DATA_H + DATA_L + 校验和 |
| 运行任务 | `ultra_task`（左/右各一个），栈 3072，优先级 5 |
| 输出 | `ul_m` / `ur_m`（前方角落净空距离，米），带时间戳 |

两个超声波安装于箱体前方左右两角，向外偏一定角度，弥补激光雷达在低矮/薄障碍（如桌腿、电线）和
极近距（雷达盲区）的不足。每个超声波使用 **软件 UART（bit-bang）** 而非硬件 UART——因为 ESP32-S3
只有 2 个硬件 UART（UART1 被 UWB 占用，UART2 被雷达占用），而 A02YYUW 只需 9600 波特率，
软件 GPIO 位采样完全满足精度。

软件 UART 实现：GPIO 下降沿中断触发 → `esp_timer` 单次定时器在每位中心采样 → 8N1 解码 →
环形缓冲区。两个 SW UART 任务在 `传感器修改6` 中已被验证能稳定收数。

**新鲜度窗口**：`ULTRA_FRESH_US = 500ms`。

### 4. IMU 惯性测量单元 —— 航向闭环

| 属性 | 值 |
|------|-----|
| 型号 | 定制 I2C 九轴 IMU |
| 接口 | I2C0，SDA=GPIO39，SCL=GPIO38 |
| 地址 | 0x23（7 位） |
| 读取 | 加速度计（16g）、陀螺仪（2000dps）、磁力计（800uT）、四元数、欧拉角、气压计 |
| 用途 | 航向闭环：用偏航角误差修正转向指令 |

IMU 不在独立任务中运行，而是在 `control_task` 中同步读取（每次控制循环调用 `imu_read_yaw()`）。
读取频率跟随控制循环（默认 50Hz）。欧拉角的 yaw 分量经过符号修正（`FR_IMU_YAW_SIGN = -1.0`）
后作为航向闭环的测量值。

### 5. FSR 薄膜压力传感器（sensor_hub 测试用）

| 属性 | 值 |
|------|-----|
| 接口 | ADC1，GPIO8（CH7） |
| 模型 | 线性 `U = 0.0004F + 0.0749`（需标定） |
| 用途 | 传感器测试程序中的体重/压力测量，follow_robot 主程序未使用 |

### 6. VL53L1X ToF 激光测距（sensor_hub 测试用）

| 属性 | 值 |
|------|-----|
| 接口 | I2C0 共享，SDA=GPIO39，SCL=GPIO38 |
| 地址 | 0x52（8 位） |
| 用途 | 传感器测试程序中的近距离精确测距，follow_robot 主程序未使用 |

---

## 软件架构：多任务 RTOS + 共享快照

### 任务布局

```
Core 0:                                   Core 1:
  (空闲)                                    uwb_task       优先级6  栈4096  UART 目标定位
                                            lidar_task     优先级6  栈4096  UART 激光扫描
                                            ultra_task_L   优先级5  栈3072  SW UART 左超声波
                                            ultra_task_R   优先级5  栈3072  SW UART 右超声波
                                            control_task   优先级7  栈4096  50Hz 控制循环
```

### 共享快照（g_shared）

所有传感器任务将数据写入一个全局结构体，由互斥锁 `g_shared.lock` 保护：

```c
typedef struct {
    SemaphoreHandle_t lock;
    float tgt_distance_m;     // UWB: 目标距离（米）
    float tgt_bearing_rad;    // UWB: 目标方位（弧度，机体坐标系）
    uint64_t tgt_ts_us;       // UWB: 最后更新时间戳
    fa_obstacle_field_t field;// 雷达: 障碍物直方图（36扇区）
    uint64_t field_ts_us;     // 雷达: 最后更新时间戳
    float ul_m;               // 左超声: 净空距离
    uint64_t ul_ts_us;
    float ur_m;               // 右超声: 净空距离
    uint64_t ur_ts_us;
} shared_t;
```

### 数据新鲜度判定

控制任务每次循环读取快照后，用当前时间减去各传感器时间戳，与新鲜度阈值比较：

| 传感器 | 新鲜度阈值 | 超时行为 |
|--------|-----------|----------|
| UWB 目标 | 700ms | 目标判为无效 → SEARCH / IDLE |
| 激光雷达 | 500ms | 障碍场判为无效 → 仅靠超声波避障 |
| 超声波 | 500ms | 对应侧判为无效 → 仅用另一侧 |

### 传感器故障降级

- **无激光雷达** → 仅靠两路超声波做前向急停（ESTOP），不进行 VFH 绕行
- **无 UWB** → 无法定位用户，进入 SEARCH 旋转找回，超时后退回 IDLE 停车
- **编码器掉线** → 退化为纯前馈开环驱动（前馈项直接将目标速度映射为 ESC 脉冲）
- **IMU 掉线** → 跳过航向闭环，直接用算法输出的 ω 指令

---

## 跟随避障算法详解（follow_avoid）

### 坐标系

```
       +x （前进方向）
        ^
        |
  +y <--+      角度 0   = 正前方 (+x)
 （左侧）        角度 > 0 = 左侧（逆时针 CCW）
                角度 < 0 = 右侧（顺时针 CW）

  ω > 0 使机体左转（CCW），与底盘差速驱动一致。
```

后两轮为电机驱动轮（差速），前两轮为万向随动脚轮。瞬时旋转中心在后轮轴上，万向轮自由跟随。
**不倒车** —— 车尾没有任何传感器，盲倒不安全。

### 状态机

```
          ┌──────────────────────────────────────┐
          │                                      │
          ▼                                      │
     ┌─────────┐   目标丢失 > 重捕超时    ┌──────────┐
     │  IDLE   │ ◄────────────────────── │ SEARCH   │
     │  停车    │    搜索超时(6s)无目标    │ 原地旋转  │
     └─────────┘                         └──────────┘
          ▲                                    │
          │  目标有效且畅通                      │ 目标重捕
          │                                    ▼
     ┌─────────┐   前向有障碍    ┌──────────┐
     │ FOLLOW  │ ──────────────► │  AVOID   │
     │ 直线跟随 │                 │ 绕行跟随  │
     └─────────┘                 └──────────┘
          │                            │
          │   净空 ≤ 急停距离            │   净空 ≤ 急停距离
          ▼                            ▼
     ┌──────────────────────────────────────┐
     │               ESTOP                   │
     │  刹停 + 朝较空侧原地旋转脱困            │
     └──────────────────────────────────────┘
```

| 状态 | 触发条件 | 行为 |
|------|---------|------|
| **IDLE** | 从未见过目标，或 SEARCH 超时（6s） | 停车，v=0，ω=0 |
| **SEARCH** | 目标丢失超过重捕时间（0.7s），且之前见过目标 | 朝 `last_known_bearing` 方向原地旋转（0.8 rad/s），尝试重捕 UWB 信号 |
| **FOLLOW** | 目标有效，且前向路径畅通（无阻塞扇区） | 朝向目标前进，距离近时降速，落在停止带内停车保持 |
| **AVOID** | 目标有效，但正前方扇区有障碍物阻塞 | VFH-lite 选最优绕行方向，降速通过 |
| **ESTOP** | 前向净空 ≤ 急停距离（雷达前锥或任一路超声波） | 线速度刹停，朝净空较大的一侧原地旋转脱困 |

**优先级**：ESTOP > SEARCH/IDLE > AVOID/FOLLOW。急停最高优先，确保安全。

### 跟随控制（FOLLOW / AVOID 状态下的速度指令）

输入：UWB 解析出的目标 `{距离 d（米）, 方位 β（弧度）}`。

**线速度**（保持车距）：
```
若 d > follow_distance_m（1.0m）：
    v = kp_dist × (d − follow_distance_m)    // 向前追，kp_dist=0.9
若 d 落在 [follow_distance_m − stop_band_m, follow_distance_m]（0.75–1.0m）：
    v = 0                                     // 停止带内保持静止，避免反复进退抖动
若 d < follow_distance_m − stop_band_m：
    v = 0                                     // 太近，不倒车
```
线速度上限：`max_linear_mps = 0.7 m/s`。

**角速度**（对准目标）：
```
ω = kp_bear × β + kd_bear × dβ/dt
```
`kp_bear = 1.6`（比例），`kd_bear = 0.25`（阻尼，抑制震荡）。
角速度上限：`max_angular_rps = 1.6 rad/s`。

**转向-速度耦合**：`v_effective = v × (1 − |β| / 90°)`，转角越大前进越慢，避免边急转边猛冲。

### VFH-lite 避障（AVOID 状态下的航向选择）

1. **构建极坐标直方图**：将激光雷达点按机体角度装入前向 FOV（默认 ±90°，即前方 180°），
   划分为 36 个扇区（每扇区 5°），每个扇区记录最近障碍距离。

2. **阻塞判定 + 膨胀**：某扇区最近障碍距离 < `safe_distance_m`（0.6m）即判为阻塞。
   再按机体半宽在该距离下张成的角宽 `asin(robot_half_width / distance)` 向两侧膨胀，
   防止算法选择一条比车还窄的缝隙。

3. **航向选择**：在所有非阻塞扇区中，选择代价值最小的扇区角度作为目标航向：
   ```
   cost = w_goal × |扇区角 − 目标方位| + w_smooth × |扇区角 − 上次选中航向|
   ```
   `w_goal = 1.0`（趋向目标），`w_smooth = 0.35`（避免来回打舵）。
   角速度指令 = `kp_bear × heading_error + kd_bear × d(heading_error)/dt`。

4. **速度调速器**：用前向锥（±40°）内和两路超声波的最近净空 `clearance` 线性压速：
   ```
   若 clearance < slow_distance_m（1.2m）：
       v_scaled = v × (clearance − emergency_distance_m) / (slow_distance_m − emergency_distance_m)
   若 clearance ≤ emergency_distance_m（0.35m）：
       触发 ESTOP
   ```
   这是避障的「软」层——越靠近障碍物，前进越慢。

5. **超声波兜底 / 急停**：`clearance ≤ emergency_distance_m`（雷达前锥或任一路超声波）→
   ESTOP：线速度立即刹停（减速限幅 2.0 m/s²，快于加速的 0.8 m/s²），朝空侧原地旋转脱困。

### 加速度限幅

所有速度指令经过加速度限幅平滑处理：
- 升速限幅：`max_lin_accel_mps2 = 0.8 m/s²`
- 降速限幅：`max_lin_decel_mps2 = 2.0 m/s²`（刹车比加速快，安全优先）
- 角加速度限幅：`max_ang_accel_rps2 = 6.0 rad/s²`

---

## 闭环底盘详解（chassis）

### 1. 电机驱动：APO-DL ESC + RC PWM

两个 APO-DL 电调各接收一路 50Hz 航模舵机脉冲（LEDC 生成，14 位分辨率）：

| 脉冲宽度 | 含义 |
|---------|------|
| 1000 µs | 全速后退 |
| 1500 µs | 中位 / 停止 |
| 2000 µs | 全速前进 |

输出引脚：左 ESC = GPIO4，右 ESC = GPIO5。方向可通过 `left_invert` / `right_invert` 翻转。

### 2. 速度闭环：AB 编码器 + 轮速 PID

```
目标轮速 (m/s) ──► 前馈 ff_us = (2000−1500)/Vmax × 目标轮速 ──┐
                                                               ├─► 电调脉冲 = 1500 + 限幅(ff + pid)
实测轮速 (m/s) ──► 误差 ──► PID(kp,ki,kd) ──► 修正 us ────────┘
   ▲
   │
AB 编码器 4x 正交解码 → 计数差 / ticks_per_meter / dt
```

- **编码器引脚**：左 A=GPIO6，左 B=GPIO7，右 A=GPIO15，右 B=GPIO16
- **正交解码**：两路 GPIO 边沿中断 → 4x 解码查找表（16 状态 → ±1 步）→ `volatile int64_t` 累计计数
- **前馈**：`(esc_max − esc_mid) / max_speed_mps × 目标轮速`，将目标速度直接映射为脉冲偏移量，
  保证「即使编码器掉线也能走」（优雅降级）
- **PID**：`kp=200, ki=300, kd=5`（us 每 m/s）
  - 比例项：瞬时速度误差修正
  - 积分项：消除稳态误差，同输出限幅（抗积分饱和 `anti-windup`），`pid_out_limit_us=400`
  - 微分项：取在「测量值」而非误差上（`derivative-on-measurement`），避免设定值突变导致脉冲尖峰
- **停止处理**：目标速度为 0 时强制输出中位 1500 µs 并清零积分器，防止电调蠕动
- **斜率限幅**：脉冲变化率 `slew_us_per_s = 1500 us/s`，保护减速齿轮和电池电流

### 3. 差速驱动运动学

`chassis_diff_drive_mix(v, ω, track, vmax)` 将机体指令 (v, ω) 分解为左右轮速：

```
v_left  = (v − ω × track/2) / vmax     // 归一化到 [−1, 1]
v_right = (v + ω × track/2) / vmax
```

若任一轮速超过 ±1，两者**等比例缩放**以保持转弯几何不变（`proportional saturation`），
再乘以 `vmax` 得到实际 m/s 目标值。

**轮距 `track_width_m`**：两后轮间距，默认 0.30m，需按实车修改。

### 4. IMU 航向闭环

```
yaw_ref += ω · dt                          // 积分算法输出的期望角速度，形成参考航向
ω_cmd = ω + heading_kp × wrap(yaw_ref − yaw_meas)   // IMU 实测偏航角误差修正
```

- **前馈**：`follow_avoid` 输出的 ω 作为期望转向意图
- **IMU 反馈**：用 IMU 实测偏航角 `yaw_meas` 计算航向误差，乘以 `heading_kp = 1.5 rad/s per rad` 修正 ω
- **仅在工作时启用**：仅在 FOLLOW / AVOID 状态启用；SEARCH / ESTOP 需要自由旋转，自动关闭并重置参考
- **效果**：即便万向轮拖滞、轮子打滑，机体也能真正转到位、直线段能压住航向不跑偏

### 5. 安全机制

| 机制 | 参数 | 说明 |
|------|------|------|
| 上电解锁延迟 | `ESC_ARM_MS = 2000ms` | 上电后保持中位 2 秒等待电调自检完成 |
| 失控保护 | `failsafe_timeout_s = 0.3s` | 控制任务卡死 0.3 秒无新指令 → 自动拉回中位停车 |
| 里程计 | `chassis_get_odometry()` | 编码器积分航位推算 (x, y, yaw)，供调试扩展 |

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
- 速度 PID 参数（KP/KI/KD/限幅）
- 跟随距离/停止带/最大速度
- 避障阈值（紧急/减速/安全距离）+ 机体半宽
- 传感器引脚（UWB/激光/超声波/IMU）
- IMU 航向闭环开关/增益/偏航取反
- 控制循环频率

### 子项目：sensor_hub（传感器测试）

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py build flash monitor
```

传感器测试程序运行 7 个 FreeRTOS 任务并发读取全部传感器，引脚已硬编码在 `main.c` 中
（超声波 GPIO4/5，I2C GPIO38/39 等），用于验证硬件接线和传感器驱动是否正常。

---

## 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_distance_m` | 1.0m | 期望跟车距离 |
| `stop_band_m` | 0.25m | 停止带（0.75–1.0m 内保持静止） |
| `max_linear_mps` | 0.7 m/s | 最大前进速度 |
| `max_angular_rps` | 1.6 rad/s | 最大转向角速度 |
| `emergency_distance_m` | 0.35m | 急停触发距离 |
| `slow_distance_m` | 1.2m | 开始减速的距离 |
| `safe_distance_m` | 0.6m | 扇区阻塞判定距离 |
| `robot_half_width_m` | 0.22m | 机体半宽（膨胀阻塞扇区用） |
| `track_width_m` | 0.30m | 后轮距 |
| `ticks_per_meter` | 2000 | 编码器每米 4x 脉冲数（**必须标定**） |
| `max_speed_mps` | 0.8 m/s | 满速轮速 |
| `kp / ki / kd` | 200 / 300 / 5 | 速度 PID（us 每 m/s） |
| `pid_out_limit_us` | 400 µs | PID 单轮最大修正 |
| `heading_kp` | 1.5 | 航向闭环增益 |
| `esc_min/mid/max_us` | 1000/1500/2000 | ESC 脉冲行程 |
| `control_hz` | 50 | 控制循环频率 |

更深层参数（`kp_dist / kp_bear / kd_bear`、加速度限幅、VFH 权重 `w_goal / w_smooth`）定义在
`follow_avoid.c` 的 `fa_default_config()` 函数中，可在 `main.c` 的 `build_fa_config()` 中覆盖。

---

## 测试

算法4 已移除全部 PC stub。传感器协议解析测试（纯 C 解析逻辑，无硬件依赖）仍可在 PC 运行：

```bash
bash tests/protocol/run_tests.sh
```

测试内容：A02YYUW 帧解析、BU UWB 距离行/TWR JSON 解析、FSR 线性标定公式。

---

## 引脚对照表

### follow_robot 项目默认引脚（可通过 menuconfig 修改）

| 外设 | 接口 | 引脚 | 备注 |
|------|------|------|------|
| 左 ESC | LEDC PWM | GPIO4 | 50Hz RC 脉冲，1000–2000µs |
| 右 ESC | LEDC PWM | GPIO5 | 50Hz RC 脉冲 |
| 左编码器 A | GPIO 中断 | GPIO6 | 4x 正交解码 |
| 左编码器 B | GPIO 中断 | GPIO7 | 4x 正交解码 |
| 右编码器 A | GPIO 中断 | GPIO15 | 4x 正交解码 |
| 右编码器 B | GPIO 中断 | GPIO16 | 4x 正交解码 |
| BU UWB | HW UART1 | RX=18, TX=8 | 115200 baud |
| RPLIDAR C1 | HW UART2 | RX=17, TX=9 | 460800 baud |
| 超声波左 | SW UART | RX=35 | 9600 baud，只接 RX |
| 超声波右 | SW UART | RX=36 | 9600 baud，只接 RX |
| IMU | I2C0 | SDA=11, SCL=12 | addr 0x23 |

### sensor_hub 项目引脚（传感器修改6 验证引脚，硬编码在 main.c）

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
│   │   ├── chassis/                  # 闭环底盘（ESC PWM + 编码器 PID + 前馈 + IMU 航向）
│   │   │   ├── chassis.c/h
│   │   │   └── CMakeLists.txt
│   │   └── follow_avoid/            # 跟随 + VFH-lite 避障算法（纯 C，无平台依赖）
│   │       ├── follow_avoid.c/h
│   │       └── CMakeLists.txt
│   └── sensors/
│       ├── a02yyuw/                  # A02YYUW 超声波（HW/SW UART 双模式 + 句柄式多实例）
│       │   ├── a02yyuw.c/h
│       │   ├── sw_uart.c/h          # 软件 UART（GPIO 位采样 + esp_timer）
│       │   └── CMakeLists.txt
│       ├── bu_uwb/                   # BU03/BU04 UWB 超宽带（TWR JSON 解析）
│       ├── fsr_adc/                  # FSR 薄膜压力（ADC 线性标定）
│       ├── imu_i2c/                  # 九轴 IMU（I2C 寄存器读取 + 四元数/欧拉角）
│       ├── rplidar_c1/              # RPLIDAR C1 激光雷达（5 字节扫描包状态机解析）
│       └── vl53l1x_tof/             # VL53L1X ToF 激光测距（ST ULD API 封装）
├── examples/
│   ├── follow_robot/                 # ★ 主程序：完整跟随机器人
│   │   ├── CMakeLists.txt
│   │   ├── sdkconfig.defaults
│   │   └── main/
│   │       ├── CMakeLists.txt
│   │       ├── Kconfig.projbuild    # 全部参数配置菜单
│   │       └── main.c               # 多任务 RTOS + 控制循环
│   └── sensor_hub/                   # 传感器测试程序（7 路并发读取）
│       ├── CMakeLists.txt
│       ├── sdkconfig.defaults
│       └── main/
│           ├── CMakeLists.txt
│           ├── Kconfig.projbuild
│           └── main.c
├── docs/
│   ├── algorithm/
│   │   └── follow-and-avoidance.md   # 算法设计详细文档
│   └── sensors/                      # 传感器使用/接线/测试文档
├── tests/
│   ├── algorithm/                    # 算法 + PID 单元测试（参考用）
│   └── protocol/                     # 传感器协议解析测试（PC 可运行）
├── outputs/                          # 输出文件
├── scripts/                          # 辅助脚本
├── work/                             # 临时工作目录
└── AGENTS.md
```

---

## 上电调试顺序

1. **垫高轮子空跑**：先确认电调解锁（上电后等待 2 秒 ESC 自检）
2. **标定编码器方向与每米脉冲**：手推 1m，看日志 `meas v` 符号是否为正、增量是否合理，
   不对就调 `*_ENC_INVERT`，据增量设 `TICKS_PER_METER`
3. **标定满速轮速**：给最大前进，量实际轮速，填 `MAX_WHEEL_SPEED_MMPS`
4. **整定速度 PID**：先只留前馈（KI=KD=0）看跟随误差，再加 KI 消静差、KD 抑超调
5. **校航向闭环**：直线跑若往一侧偏，调 `HEADING_KP_MILLI`；方向修反则开 `IMU_YAW_INVERT`
6. **校激光雷达零位**：`LIDAR_FORWARD_DEG` 对准机体正前方，`LIDAR_CW` 匹配扫描旋向
7. **校 UWB 左右**：目标方位方向反了则开 `UWB_LEFT_IS_POS_X`
8. 确认 FOLLOW/AVOID/ESTOP 切换与转向方向都正确后再落地测试

---

## 注意事项

1. **引脚区分**：`follow_robot` 和 `sensor_hub` 是两个独立项目，引脚配置各自独立
2. **RPLIDAR 需 5V/800mA 外部供电**，TX/RX 交叉接线
3. **每个超声波只接 RX**，A02YYUW 自主输出，TX 不接
4. **编码器 `TICKS_PER_METER` 必须标定**：推机器人恰好 1 米，记录日志中编码器计数差值
5. **FSR 公式未标定**，使用前需实测标定
6. **GPIO39 作 SDA** 仅 ESP32-S3 支持（经典 ESP32 的 GPIO34–39 仅为输入）
7. **构建 follow_robot** 在项目根目录运行 `idf.py` 命令；构建 `sensor_hub` 需进入 `examples/sensor_hub/` 目录
8. **无后向感知**：算法刻意不倒车，过近只停车
9. **UWB 坐标约定**：跟随方位由 `Xcm/Ycm` 算得，若左右颠倒则打开 `UWB_LEFT_IS_POS_X`
10. **失效保护**：控制任务卡死会在 0.3 秒内自动停车，但首测时请在可断电环境下进行
