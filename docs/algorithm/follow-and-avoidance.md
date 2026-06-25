# 智能跟随 + 避障算法（算法1）

本文件描述行李箱的「跟随用户 + 自主避障」算法的设计、坐标约定、参数与调参方法。
代码位于：

```
components/control/chassis/         # 差速底盘驱动（v, ω → 左右轮 PWM）
components/control/follow_avoid/     # 纯算法（跟随 + 避障 + 状态机），可在 PC 上单测
examples/follow_robot/              # 整机示例：传感器任务 + 控制循环 + 底盘
tests/algorithm/                    # 算法 PC 单元测试
```

> ⚠️ **安全 & 硬件提醒（先读）**：
> - 这是会自驱行进的车，**首次调试务必把驱动轮架空离地**，确认方向/转向/急停正确再落地。
> - 本分支（算法1）的 `chassis` 按**开环 H 桥 + 占空比**驱动，**与项目真实的 APO-DL 电调 + RC PWM 无刷动力不一致**；真车请用 `算法2`（电调 RC-PWM + 编码器轮速 PID + IMU 航向闭环）。
> - `max_speed_mps`（占空比→速度标定）、`min_duty`（死区）、`track_width_m`（轮距）必须按实车标定，否则速度与转向半径不准。

## 1. 硬件与坐标系

| 部件 | 位置 | 作用 |
|------|------|------|
| 后两轮（电机轮） | 底盘后方 | 差速驱动，提供前进速度 v 与转向角速度 ω |
| 前两轮（万向轮） | 底盘前方 | 被动随动脚轮 |
| RPLIDAR C1 | 底盘前方 | 360° 扫描 → 前向 180° 障碍物直方图 |
| 超声波 ×2（A02YYUW） | 箱体前方左右两角 | 近距离安全兜底，补激光雷达盲区/低矮薄障碍 |
| IMU | 底盘中部 | 偏航角遥测（可选，用于航向保持扩展） |
| UWB（BU0x） | 箱体 | 跟随目标：用户随身标签的距离 + 方位 |

**机体坐标系**（贯穿整个算法）：

```
        +x  （前进方向）
         ^
         |
   +y <--+      角度 0   = 正前方 (+x)
  （左）  |      角度 > 0 = 左侧（逆时针 CCW）
         |      角度 < 0 = 右侧（顺时针 CW）

   ω > 0 使机体左转（CCW），与 chassis_set_velocity 一致。
```

后轮差速、前轮万向 → 这是标准的「后轴差速驱动」：瞬时旋转中心在后轮轴上，
万向轮自由跟随。

## 2. 软件架构

每个传感器各跑一个 FreeRTOS 任务，把结果写入一个互斥量保护的快照；固定频率
的控制任务读取快照、运行纯算法、驱动底盘：

```
 uwb_task   ─┐                        ┌─► chassis_set_velocity(v, ω)
 lidar_task ─┤  互斥量保护的共享快照   │
 ultra_task ─┼──► g_shared ──► control_task ──► fa_update() ─┘
 (imu 遥测) ─┘     (含时间戳)        (50 Hz)
```

- 任何传感器初始化失败 → 其数据保持「过期/无效」，算法自动降级：
  - 没有激光雷达 → 仅靠超声波做安全停障；
  - 没有 UWB → 进入 SEARCH 旋转找回，超时后 IDLE 停车。
- 快照带时间戳，控制任务按新鲜度判断数据是否可用（`*_FRESH_US`）。

`follow_avoid` 不依赖任何 ESP/FreeRTOS API（纯 C + math.h），因此与 PC 单测
共用同一份逻辑（见 `tests/algorithm/`）。

## 3. 跟随控制

输入：UWB 解析出的目标 `{距离 d, 方位 β}`。

- **线速度**（保持车距）
  - `d > 期望车距`：`v = kp_dist · (d − 期望车距)`，向前追；
  - 落在 `[期望车距 − 停止带, 期望车距]`：`v = 0`，原地保持，避免抖动；
  - 比停止带还近：`v = 0`。**不倒车** —— 车尾没有任何传感器，盲倒不安全。
- **角速度**（对准目标）：`ω = kp_bear · heading + kd_bear · d(heading)/dt`。
- **耦合**：转角越大，前进越慢（`turn_scale = 1 − |heading| / 90°`），避免边急转边猛冲。

## 4. 避障控制（VFH-lite + 速度调速器 + 超声波兜底）

1. **极坐标直方图**：把激光雷达点按机体角度装进前向 FOV（默认 ±90°，36 个扇区，
   每扇区 5°），每个扇区记录最近障碍距离。
2. **阻塞判定 + 膨胀**：某扇区最近障碍 < `安全距离` 即判为阻塞；再按机体半宽在
   该距离下张成的角宽（`asin(半宽/距离)`）向两侧膨胀，确保不会去钻比车还窄的缝。
3. **航向选择（VFH-lite）**：在所有「空闲」扇区里挑代价最小者：
   `cost = w_goal·|扇区角 − 目标方位| + w_smooth·|扇区角 − 上次航向|`。
   即「尽量朝向用户、又尽量不来回打舵」。选中的扇区角作为转向目标 heading。
4. **速度调速器**：用前向锥内（含两超声波）的最近净空 `clearance` 线性压速 ——
   `clearance` 从 `减速距离` 降到 `急停距离` 时，v 从满速线性压到 0。这是避障的「软」层。
5. **超声波兜底 / 急停**：`clearance ≤ 急停距离`（雷达前锥或任一超声波）→ ESTOP：
   线速度刹停，并朝较空的一侧原地旋转脱困，绝不向障碍物前进。

## 5. 状态机

| 状态 | 触发 | 行为 |
|------|------|------|
| IDLE | 从未见过目标且已超时 | 停车 |
| SEARCH | 目标丢失超过重捕时间 | 朝最后已知方位原地旋转找回；超时退回 IDLE |
| FOLLOW | 目标有效且正前方畅通 | 朝目标跟随 |
| AVOID | 目标有效但正前方被挡 | 绕开（VFH 选侧），减速通过 |
| ESTOP | 前向/超声波进入急停距离，或四面被堵 | 刹停 + 朝空侧旋转脱困 |

急停优先级最高，其次目标丢失处理，最后才是正常跟随/避障。
所有输出都经过**加速度限幅**（升速、降速分别限幅，刹车比加速更快），输出平滑、保护齿轮。

## 6. 参数与调参（menuconfig）

全部参数在 `examples/follow_robot/main/Kconfig.projbuild`，`idf.py menuconfig` 可改。

| 参数 | 默认 | 说明 / 调参方向 |
|------|------|------|
| 期望车距 `FOLLOW_DISTANCE_MM` | 1000 | 跟得更近就调小 |
| 停止带 `STOP_BAND_MM` | 250 | 抖动就调大 |
| 最大线速度 `MAX_LINEAR_MMPS` | 700 | 场地小就调小 |
| 最大角速度 `MAX_ANGULAR_MRADPS` | 1600 | 转向太猛就调小 |
| 急停距离 `EMERGENCY_DIST_MM` | 350 | 撞了就调大 |
| 减速距离 `SLOW_DIST_MM` | 1200 | 想更早减速就调大 |
| 安全距离 `SAFE_DIST_MM` | 600 | 判定阻塞的阈值 |
| 机体半宽 `ROBOT_HALF_WIDTH_MM` | 220 | 改成实际车宽的一半 |
| 轮距 `TRACK_WIDTH_MM` | 300 | **必须**改成实际两后轮间距 |
| 满占空轮速 `MAX_WHEEL_SPEED_MMPS` | 800 | 开环标定：满 PWM 时实测轮速 |
| 死区占空 `MIN_DUTY_PCT` | 12 | 电机不转就调大 |

更深的增益（`kp_dist / kp_bear / kd_bear`、加速度限幅、VFH 权重）在
`fa_default_config()` 里，可在 `build_fa_config()` 覆盖。

## 7. 电机接线（TB6612FNG / DRV8833 类双 H 桥）

每个电机：两个方向脚（IN1/IN2）+ 一个 PWM 脚。前进 IN1=1,IN2=0；后退反之；
停车 IN1=IN2=0；刹车 IN1=IN2=1。引脚全部在 Kconfig 配置：

```
左电机: LEFT_PWM_GPIO / LEFT_IN1_GPIO / LEFT_IN2_GPIO   (+ LEFT_INVERT)
右电机: RIGHT_PWM_GPIO / RIGHT_IN1_GPIO / RIGHT_IN2_GPIO (+ RIGHT_INVERT)
```

> ⚠️ ATK-DNESP32S3 板上绝大多数 IO 已被 LCD/摄像头/I2S/SPI 占用，可用 IO 很少。
> 6 个电机信号请分配到板上**确实空闲**的 GPIO，通常需要电机驱动扩展板或独立控制板。
> 若某个轮子转反，把对应的 `*_INVERT` 打开即可，无需改线。

换成单 DIR 脚的驱动？只需改 `chassis.c` 里 `apply_motor()` 一处方向逻辑，其余不变。

## 8. 构建、烧录、测试

```bash
# 整机固件
cd examples/follow_robot
idf.py set-target esp32s3
idf.py menuconfig        # 配置电机引脚、车距、避障阈值
idf.py build flash monitor

# 算法 PC 单元测试（无需硬件，gcc 即可）
bash tests/algorithm/run_tests.sh
```

## 9. 安全与限制

- **无后向感知**：算法刻意不倒车，过近只停车。
- **开环调速**：本版无轮速编码器，`MAX_WHEEL_SPEED_MMPS` 为标定常数；若日后加编码器，
  在 `chassis_set_velocity()` 外包一层 PID 即可，上层算法不变。
- **UWB 坐标约定**：跟随方位由 `Xcm/Ycm` 算得，若左右颠倒，打开
  `UWB_LEFT_IS_POS_X`；激光雷达零位/旋向用 `LIDAR_FORWARD_DEG / LIDAR_CW` 校准。
- 首次上电请垫高轮子空跑，确认 FOLLOW/AVOID/ESTOP 状态切换与转向方向正确后再落地。
