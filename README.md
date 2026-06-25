# RPLIDAR C1 激光雷达扫描

基于思岚 RPLIDAR C1 (C1M1) 的独立激光雷达扫描测试程序，读取设备信息并实时输出 360° 扫描点云数据。纯 ESP-IDF 原生 UART 实现，不依赖 Arduino。

> **这个分支是什么**：激光雷达单体调试分支，是「跟随行李箱」感知前方障碍/做避障的核心传感器。本分支把 RPLIDAR 的「初始化 → 读设备信息 → 健康自检 → 启动扫描 → 持续读点」完整跑通，后续 `imu_vl53l1x_rplidar_merged`、`传感器整合`、`算法1/2` 都复用这套驱动（在那些分支里组件名是 `rplidar_c1`）。

---

## 功能

- **设备识别**: 读取型号、固件版本、序列号（应答以 `0xA5 0x5A` 开头）
- **健康检查**: `health_status==0` 才算正常，否则打印错误码并退出
- **连续扫描**: 启动传统 360° 连续扫描模式
- **点云输出**: 实时输出每个点的角度(°)、距离(mm)、信号质量；`start_bit` 标记新一圈起点

---

## 硬件连接（接线最容易接反，看清方向）

| RPLIDAR C1 引脚 | 接到 ESP32-S3 | 说明 |
|------------------|----------------|------|
| **TX**（雷达发） | **GPIO18**（ESP 的 RX） | 雷达数据进 ESP |
| **RX**（雷达收） | **GPIO17**（ESP 的 TX） | ESP 指令给雷达 |
| 5V | 5V（**外部供电**） | 见下电源注意 |
| GND | GND（与 ESP 共地） | 必须共地 |

> 即代码里 `LIDAR_TX_GPIO=17`（ESP 发）、`LIDAR_RX_GPIO=18`（ESP 收），UART 收发必须**交叉**接：雷达 TX↔ESP RX、雷达 RX↔ESP TX。用的是 `UART_NUM_2`。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 运行现象（烧录后应该看到什么）

- 先打印一行连接成功信息（型号/固件/序列号），随后健康自检通过，开始刷点：`角度: xx.x°, 距离: xxxx mm (信号质量: n)`。
- 每转完一圈会插一行 `>>> [一圈 360 度点云已刷新] <<<`。
- ⚠️ **串口里的中文会是乱码**：`main.c` 是 GBK 编码，所有中文 `printf`/`ESP_LOGI` 在 `idf.py monitor`（按 UTF-8 解码）下显示为乱码，**但角度/距离这些数字是正常的**，不影响判断。介意的话把日志字符串改成英文或把文件转 UTF-8。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | 启动流程 | 顶部 `LIDAR_UART_PORT/TX/RX` 三个宏（改端口/引脚在这）；`app_main` 顺序：`rplidar_init` → `rplidar_get_device_info` → `rplidar_get_health`（不为 0 直接 return）→ `rplidar_start_scan` → `while` 里 `rplidar_read_point` 轮询。注意 `rplidar_set_motor_speed(...,600)` 被注释了，**默认用雷达自身转速**，要定转速就放开它。 |
| `components/Middlewares/RPLIDAR/rplidar.h` | 协议/API | 命令字（`0xA5` 起始、`0x20` 扫描、`0x25` 停、`0x52` 健康、`0x50` 信息、`0xA8` 调速）、`rplidar_point_t`（angle/distance/quality/start_bit）、`rplidar_info_t`。 |
| `components/Middlewares/RPLIDAR/rplidar.c` | 驱动实现 | `rplidar_init` 里波特率**固定 460800**、无校验、RX 缓冲 2048；`rplidar_read_point` 是**非阻塞滑窗字节对齐状态机**：校验第 1 字节 S/!S 取反必为 1、第 2 字节 C 位恒为 1、再做异或校验，错位时自动重新对齐。这段是理解雷达数据怎么从字节流里「对齐」出来的关键。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **电源是头号问题**：RPLIDAR C1 必须 **5V 外部供电**，电机启动瞬时电流可达 ~800mA。**别用开发板的 3.3V 或 USB 弱供电直接带电机**，否则电机转不起来/反复重启/数据乱。信号线是 3.3V TTL，可直连 ESP，但电源要单独给足。
2. **接线交叉**：TX/RX 必须交叉（见上表），接成「直连」会完全收不到数据。
3. **健康自检不过会直接退出**：`health_status != 0` 时 `app_main` 打印错误码后 `return`，程序就停了。看不到点云先确认这一步。
4. **要等电机转稳**：上电到稳定旋转有零点几秒，刚开始可能没点或点很少，属正常。
5. **距离为 0 的点被丢弃**：`if (p.distance > 0.0f)` 过滤了雷达盲区/无回波点，所以打印的点数会少于理论值，不是 bug。
6. **轮询要让出 CPU**：主循环 `vTaskDelay(1ms)` 是为了喂狗/让出 CPU；`rplidar_read_point` 本身非阻塞，删掉延时容易触发看门狗。
7. **main.c 残留一堆音频/LCD 头文件**（`es8388.h`/`spilcd.h`/`audioplay.h`…）和文件底部被注释的旧 `app_main`——是从 `audioplay` 分支拉出来改的残留，**与雷达功能无关**，别被带偏。

---

## 调试与使用注意点

- 完全收不到数据：① 查交叉接线；② 查 5V 供电是否够；③ 确认波特率 460800、用的是 GPIO17/18 / UART2。
- 点云稀疏/跳变：多半是供电不稳或电机未转稳。
- 要停雷达省电：调用 `rplidar_stop`；要软复位：`rplidar_reset`。

---

## 目录结构

```
├── main/
│   └── main.c                  # 主程序：RPLIDAR 初始化 + 健康自检 + 扫描轮询（GBK 编码）
├── components/
│   └── Middlewares/RPLIDAR/
│       ├── rplidar.c           # RPLIDAR C1 UART 驱动（460800、滑窗对齐解析）
│       └── rplidar.h           # 协议命令字 + 5字节点云包结构
└── CMakeLists.txt
```
