# ESP32-S3 跟随车：算法6

算法6以 `算法4` 为基础，保留 BU UWB、RPLIDAR C1、双 A02YYUW、双编码器、APO-DL ESC、FSR、闭环跟随与避障，移除不再使用的惯性姿态传感器和 VL53L1X。控制方向来自 UWB 目标方位，雷达/超声波负责避障修正，编码器闭合左右轮速度环。

实机参考分支为 `跟随代码2脉宽可用`。迁移的是已验证的 ESC 参数、GPIO4/5 LEDC 输出、上电中立、方向约定、停车/超时保护和跟随行为；没有整分支覆盖 `算法4`。详细对比见 [结构审计](docs/algorithm6-audit.md)，失效场景见 [验证矩阵](docs/validation-matrix.md)。

## 算法6相对算法4的变化

- 建立唯一板级引脚表 `components/board/include/board_pin_config.h`。
- 删除两个不再使用的传感器组件、任务、Kconfig、CMake 依赖、结构字段和旧文档。
- 修复算法4已提交的雷达冲突标记。
- UWB 固定 UART1 TX47/RX48；RPLIDAR 固定 UART2 TX17/RX18。
- 双 A02YYUW 固定 GPIO38/39，使用两个软件 UART 接收，不占硬件 UART。
- FSR 恢复到主程序，固定 GPIO8/ADC1_CH7、ADC oneshot。
- 增加网页 AUTO/MANUAL、停止、急停、心跳超时和实时状态。
- 编码器持续 0.6 s 无反馈时锁存硬件故障并输出中立。
- 增加独立 `examples/sensor_check`，默认绝不自动转动电机。
- 把纯数学和协议解析拆出，使 PC 测试不依赖 ESP-IDF 头文件。

## 硬件与板卡前提

目标芯片为 ESP32-S3；仓库的 `sdkconfig.defaults` 未给出具体开发板/模组料号，构建得到的默认 Flash 配置为 2 MB，PSRAM 未在仓库中明确启用。必须按实际模组修正 Flash/PSRAM 配置。

软件层面没有发现固定引脚复用。硬件层面仍须核对实际原理图：部分 ESP32-S3 板的 GPIO48 连接 RGB LED，部分 Octal Flash/PSRAM 板型可能占用高编号 GPIO。GPIO47/48、38/39 是否真正引出只能由实际板卡料号和原理图确认；本仓库不能提供该硬件证据。

## 固定 GPIO 表

| 外设 | 功能 | GPIO | 接口 |
|---|---|---:|---|
| 左 ESC | PWM | GPIO4 | LEDC low-speed channel 0 |
| 右 ESC | PWM | GPIO5 | LEDC low-speed channel 1 |
| 左编码器 | A 相 | GPIO6 | GPIO 任意边沿中断，4x 解码 |
| 左编码器 | B 相 | GPIO7 | GPIO 任意边沿中断，4x 解码 |
| 右编码器 | A 相 | GPIO15 | GPIO 任意边沿中断，4x 解码 |
| 右编码器 | B 相 | GPIO16 | GPIO 任意边沿中断，4x 解码 |
| RPLIDAR | UART2 TX | GPIO17 | ESP32 TX → 雷达 RX |
| RPLIDAR | UART2 RX | GPIO18 | ESP32 RX ← 雷达 TX |
| BU UWB | UART1 TX | GPIO47 | ESP32 TX → UWB RX |
| BU UWB | UART1 RX | GPIO48 | ESP32 RX ← UWB TX |
| 左 A02YYUW | RX | GPIO38 | 软件 UART 数字接收 |
| 右 A02YYUW | RX | GPIO39 | 软件 UART 数字接收 |
| FSR | ADC | GPIO8 | ADC1_CH7 |

TX/RX 必须交叉连接，不能 TX 接 TX。

## 通信和执行参数

| 项目 | 参数 |
|---|---|
| UWB | UART1，115200，TX47/RX48，驱动独占安装和读取 |
| RPLIDAR C1 | UART2，460800，TX17/RX18，5 字节点解析 |
| A02YYUW | 9600 8N1、4 字节 `FF HH LL checksum`、RX-only 软件 UART，30–4500 mm |
| FSR | ADC1_CH7，12-bit 默认位宽，12 dB 衰减，10 次平均，100 ms 周期 |
| ESC | LEDC 50 Hz，20 ms 周期，14 bit，1000/1500/2000 us |
| ESC 解锁 | 初始化立即 1500 us，中立保持 2000 ms |
| 控制循环 | 50 Hz |
| 数据超时 | UWB 700 ms；雷达/超声/FSR 500 ms；网页 1000 ms |
| 底盘看门狗 | 300 ms 无新设定值自动中立 |
| 编码器失反馈 | ≥0.12 m/s 指令且反馈 <0.01 m/s 持续 600 ms，锁存停车 |

FSR 的 `0.0004 V/kg` 与 `0.0749 V` 偏置是原项目占位标定，必须结合实际分压、参考电压、噪声和载荷重新标定。电机电源噪声可能耦合进 GPIO8，应使用合理布线、RC 滤波和共地。

## 软件架构与目录

```text
components/board/                 固定引脚和总线参数
components/control/chassis/       ESC、编码器、PID、看门狗、故障锁存
components/control/follow_avoid/  纯 C 跟随/VFH-lite 避障
components/control/web_control/   SoftAP、HTTP、控制权和遥测
components/sensors/               UWB、雷达、A02YYUW、FSR
examples/follow_robot/            主固件（根项目也指向该 main）
examples/sensor_check/            独立安全自检固件
tests/algorithm/                   PC 运动学/PID/跟随/避障测试
tests/protocol/                    PC 协议解析测试
docs/                              审计与逻辑验证记录
```

共享传感器快照由互斥锁保护；每条数据带微秒时间戳。网页组件有独立互斥锁。业务代码不创建阻塞队列或事件组。只有 `control` 任务能调用底盘速度/急停接口，HTTP handler 不直接写 PWM。

## FreeRTOS 任务

| 任务 | 栈（byte） | 优先级 | 周期/阻塞方式 | 用途 |
|---|---:|---:|---|---|
| `control` | 6144 | 7 | 20 ms | 安全仲裁、算法、PID、遥测 |
| `uwb` | 4096 | 6 | UART 200 ms 超时 | 目标距离和方位 |
| `lidar` | 4096 | 6 | UART 点流；空读延时 1 ms | 障碍扇区 |
| `ultra_left` | 3072 | 5 | 120 ms 读超时 + 20 ms | 左 A02YYUW |
| `ultra_right` | 3072 | 5 | 120 ms 读超时 + 20 ms | 右 A02YYUW |
| `fsr` | 3072 | 3 | 100 ms | 压力采样 |

任务使用 `xTaskCreate`，由 ESP-IDF SMP 调度；未强行绑核。任务创建和驱动初始化返回值均检查，初始化失败的传感器不会创建依赖任务。

## 数据流和控制状态机

```text
UWB 距离/方位 ─┐
RPLIDAR 扇区 ───┼→ 时间戳快照 → FOLLOW/AVOID/SEARCH/IDLE → 权限/安全仲裁
左右超声波 ────┘                                      ↓
网页心跳/模式/急停 ─────────────────────────────→ 左右轮目标速度
                                                        ↓
编码器 4x 计数 ───────────────────────────────→ 双轮 PID + 前馈
                                                        ↓
                                               LEDC GPIO4/GPIO5
```

目标距离误差生成线速度；目标方位生成角速度。目标方向被雷达扇区阻挡时，VFH-lite 选择代价最低的可通行方向。超声波覆盖侧前方盲区；小于 0.35 m 进入障碍停车。UWB 丢失先向最后方位短时搜索，随后 IDLE。没有后向感知，因此自动模式不倒车。

## 无绝对姿态传感器控制方案

- 绝对转向输入：UWB 的目标方位角。
- 短时执行反馈：左右编码器速度差，通过差速运动学形成实测角速度。
- 环境修正：雷达 VFH-lite 和左右超声波限制/改变期望方向。
- 局限：不能长期保持世界坐标绝对航向；轮胎打滑、脚轮摩擦和编码器标定误差会造成里程计累计误差。编码器仅闭合轮速和短时相对运动，不会伪装成绝对姿态。

代码中的 `odo_yaw_rad` 仅表示由轮差积分的相对里程计角度，不来自已删除传感器，也不参与绝对航向闭环。

## 控制权和网页

优先级固定为：

```text
网页急停 / 编码器硬件故障
  > 网页断线或心跳超时
  > 手动模式
  > 自动跟随
```

上电急停默认锁存。手机连接 `Algorithm6-Control` 后打开 `http://192.168.4.1/`，必须点击 `CLEAR / ARM` 才允许运动。模式切换会清零旧手动命令；手动按钮松开即发送零速度；Wi-Fi station 离开或 1000 ms 无心跳会中立停车。非法数值、缺失参数、非手动模式的手动命令均返回 HTTP 400。

默认 AP 为空密码，仅适合台架。用 `idf.py menuconfig` 设置本地密码；不要把私人密码提交到 Git，日志也不会打印密码。

## 安全停车与急停

- 上电、控制超时、网页掉线、HTTP 急停、LEDC 写入错误或编码器失反馈都会走统一中立输出。
- 所有脉宽在 1000–2000 us 内限幅，并使用 1500 us 停车。
- 正常目标变化受 `1500 us/s` 斜率限制；急停跳过斜率限制立即中立。
- 编码器故障锁存后网页 CLEAR 不能恢复电机，必须检查接线并重启。
- 台架首次测试必须垫高驱动轮，旁边保留物理断电手段。

## 传感器自检

`examples/sensor_check` 检查 UWB、RPLIDAR、左右 A02YYUW、双编码器、FSR 和左右 ESC。输出统一为 `[PASS]`、`[WARN]`、`[FAIL]` 及汇总。ESC 只初始化并保持 1500 us；没有自动转动测试。

没有连接硬件时，固件构建成功只证明编译和静态接口正确，不能声称设备实机通过。烧录自检固件后才可根据真实串口输出填写硬件结果。

## VS Code / ESP-IDF 5.4

仓库 `.vscode/settings.json` 使用 `${workspaceFolder}/build`，目标为 `esp32s3`，ESP-IDF 路径为 `D:/Espressif/frameworks/esp-idf-v5.4/`。换电脑后在 VS Code 命令面板运行 `ESP-IDF: Configure ESP-IDF extension` 更新本机路径，不要提交个人 COM 口或密码。

主工程：

```powershell
idf.py set-target esp32s3
idf.py reconfigure
idf.py build
idf.py -p COMx flash monitor
```

自检工程：

```powershell
cd examples/sensor_check
idf.py set-target esp32s3
idf.py reconfigure
idf.py build
idf.py -p COMx flash monitor
```

VS Code 可直接使用状态栏的 Build、Flash、Monitor。若 clangd 索引不更新，先成功构建一次以生成 `build/compile_commands.json`。

## menuconfig

`Algorithm 6 follow vehicle` 菜单可调整轮距、每米脉冲、PID、跟随距离、速度、避障阈值、雷达正前方角度、控制频率、AP 名称/本地密码和网页超时。固定 GPIO、UART 编号与 ESC 实机参数不放入 Kconfig，避免不同配置静默破坏接线约束。

## PC 逻辑测试

在 Git Bash 中运行：

```bash
bash tests/algorithm/run_tests.sh
bash tests/protocol/run_tests.sh
```

覆盖项和 24 个失效场景的预期/实际行为见 `docs/validation-matrix.md`。

## 正常日志示例

```text
I algorithm6: ESC neutral 1500us; arming for 2000ms
I web_control: control AP ready: SSID=Algorithm6-Control (credentials are not logged)
I algorithm6: FOLLOW target=1 d=1.82 bearing=-0.22 v=+0.31 w=-0.18 pulses=1692/1698
```

## 故障排查

- 无法编译：确认 VS Code 选择 ESP-IDF 5.4、ESP32-S3、Ninja 和 Xtensa GCC；删除损坏的 `build/` 后重新配置。
- UWB 无数据：确认 UART1、115200、ESP TX47→模块 RX、ESP RX48←模块 TX；核对 GPIO48 板载负载。
- 雷达无数据：确认独立 5 V/足够电流、460800 和交叉 TX/RX。
- 超声无数据：A02YYUW 必须是 9600 8N1 自主输出模式，GPIO38/39 只接传感器 TX。
- FSR 近零/满量程：检查断线、分压、3.3 V 范围、12 dB 衰减、滤波和电机噪声。
- 编码器故障：垫高轮子，手转检查 A/B 相和正负方向，再标定每米 4x 脉冲；锁存后重启。
- ESC 不解锁：确认共同地、50 Hz、1500 us 中立和 2 s 等待；不要把信号接 APO-DL 电源正端。
- GPIO 冲突：只修改 `board_pin_config.h` 前先核对板卡原理图；全仓搜索裸 GPIO，禁止在模块内再定义副本。
- UART 冲突：UWB 与雷达各自只能初始化一次；双超声不能占 UART1/2。

## 已知限制与后续改进

- 未提供实际开发板/模组型号和原理图，特殊 GPIO 的板载冲突尚待确认。
- 未连接实体车辆，本次没有传感器、ESC 方向、编码器方向、Flash/PSRAM 或网页无线范围的实机验收。
- 编码器仍是 GPIO ISR 4x 解码；高脉冲率场景建议迁移 ESP-IDF PCNT 并重新验证方向和滤波。
- FSR 标定、轮距、轮径/每米脉冲、最高轮速和 PID 必须实车标定。
- 后续可增加双编码器不一致检测、轮滑诊断、认证网页和物理急停输入。

## 算法4到算法6迁移摘要

保留 `算法4` 的组件化驱动、VFH-lite、双轮 PID 和任务快照架构；删除未使用传感器及注释旧代码；采用参考分支实机 ESC 参数和方向；固定用户指定引脚；增加网页控制权、自检和安全故障锁存。原 `算法4` 与参考分支均未被改写。
