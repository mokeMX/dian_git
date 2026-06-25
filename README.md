# imu_vl53l1x_rplidar_merged - 三传感器融合

IMU + VL53L1X + RPLIDAR C1 三传感器同时采集程序，在单个 ESP32-S3 上并行读取三种传感器数据。

> **这个分支是什么**：把前面单体调通的 `IMU`、`vl53l1x`、`rplidar` 三个分支**合到一个工程里同时跑**，是迈向 `传感器整合` / `算法1·2`（完整跟随行李箱）的第一步「集成验证」分支。核心看点是**多传感器如何共用资源**——尤其是 IMU 和 VL53L1X 怎么共用一条 I2C 总线。

---

## 功能

- **VL53L1X ToF 测距**: 单次测距，毫米级（地址 0x52/8 位，即 0x29/7 位）
- **IMU 9轴 + 气压计**: 加速度(g)、陀螺(rad/s)、磁力计、欧拉角(deg)、四元数、气压/温度/高度（地址 0x23）
- **RPLIDAR C1 激光扫描**: 360° 点云（角度+距离+质量+新圈标志），UART1
- **统一串口输出**: 每帧分段打印三种传感器数据

---

## 硬件连接

| 传感器 | 接口类型 | 引脚 / 地址 |
|--------|----------|-------------|
| IMU | I2C0（共享总线） | SCL=GPIO42, SDA=GPIO41，地址 0x23 |
| VL53L1X | I2C0（与 IMU **共用同一条总线**） | SCL=GPIO42, SDA=GPIO41，地址 0x52(8位)/0x29(7位) |
| RPLIDAR C1 | UART1，460800 | ESP TX=GPIO17, ESP RX=GPIO18（与雷达 RX/TX 交叉），5V 外部供电 |

> 本分支的 I2C 引脚（42/41）和**独立 `IMU` 分支（37/38）、独立 `vl53l1x` 分支（38/39）都不一样**——因为这里统一改成了 BSP/MYIIC 的 IIC0（42/41）。接线请以本分支为准。

---

## 🔑 I2C 共享总线架构（本分支最该理解的点）

三个 I2C 调用看着像各init各的，其实是**一条总线、三处复用**：

1. `myiic_init()`（`components/BSP/MYIIC/myiic.c`）用**新版 `i2c_master` 驱动**（`i2c_new_master_bus`）在 I2C0 / SDA41 / SCL42 上创建**全局总线句柄 `bus_handle`**。
2. `i2c_module_init()`（IMU）在本分支**已不再自己初始化总线**，只打印 `I2C bus already initialized by BSP/MYIIC`，读写时 `extern bus_handle` 走 `i2c_master_write_read_device`。
3. `vl53l1x.c` 同样 `extern bus_handle`，复用同一条总线。

⚠️ **所以初始化顺序不能乱**：必须**先 `myiic_init()` 建好 `bus_handle`**，再调用 VL53L1X / IMU 的接口；否则 `bus_handle` 为 NULL 直接崩。`main.c` 里的顺序（myiic → vl53l1x → imu → rplidar）就是按这个约束排的。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 运行现象

- 串口先打印各传感器初始化日志（哪个失败会 `WARN` 并继续，见下）。
- 之后每 100ms 打印一帧 `===== Frame N =====`，含 `[VL53L1X]` / `[IMU]` / `[RPLIDAR]` 各段。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | 三传感器初始化 + 主循环 | **初始化顺序**（先 myiic 建总线）、各传感器**失败只 WARN 不退出**的「优雅降级」、主循环 100ms 一帧、RPLIDAR 用 `UART_NUM_1` 且带 stop+reset+1s。 |
| `components/BSP/MYIIC/myiic.c` | 共享 I2C 总线 | `i2c_new_master_bus` 建 `bus_handle`，引脚 IIC_SCL=42/IIC_SDA=41。这是总线的**唯一**真正初始化处。 |
| `components/i2c_module/i2c_module.c` | IMU I2C 读写 | 注意它 `extern bus_handle`、init 是**空操作**，用 `i2c_master_write_to_device/write_read_device`（新 API，不是老的 `i2c_cmd_link`）。 |
| `components/imu_module/imu_i2c_driver.c` | IMU 换算 | 同 `IMU` 分支：accel×16/32767、gyro×(2000/32767)(π/180)、euler×57.2958。 |
| `components/Middlewares/VL53L1X/vl53l1x.c` | ToF 封装 | `extern bus_handle`，`vl53l1x_app_init` / `vl53l1x_get_single_distance`。 |
| `components/Middlewares/RPLIDAR/rplidar.c` | 雷达驱动 | 460800、滑窗对齐解析。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **I2C 顺序依赖**：见上，`myiic_init()` 必须最先。改初始化顺序前先想清楚 `bus_handle` 谁建的。
2. **地址别冲突**：IMU=0x23、VL53L1X=0x29(7 位)，本来不冲突；若你再加 I2C 设备，先确认地址唯一。VL53L1X 上电默认 0x29，用 I2C 扫描能看到的就是 0x29。
3. **雷达在 100ms 循环里「每帧只读 1 个点」严重跟不上数据率**：RPLIDAR 每秒产生几千个点，而这里 10Hz 只取 1 点，UART 的 2048 字节 RX 缓冲很快被刷满/丢弃，导致雷达点稀疏甚至对齐困难。**这是「集成演示」写法，不是高吞吐写法**。真要用雷达，应把它放到**独立任务里高频 `rplidar_read_point`**（后续 `传感器整合`/`算法` 分支就是各传感器独立任务的结构）。
4. **优雅降级**：VL53L1X / RPLIDAR 初始化失败只 `WARN` 继续，所以「能跑起来」不代表三个传感器都在工作——看每段是不是 `Read failed` 或没数据。
5. **`main/IIC.c` 是备用的 IMU 单测**，未被编译（`main/CMakeLists.txt` 的 `SRC_DIRS` 只含 `APP`/`APP/AUDIO`，不含 IIC.c），所以不会和 `main.c` 的 `app_main` 重复定义；但你若手动把它加进编译会**重复定义 `app_main`**，注意。
6. **`main/APP/AUDIO/` 下的音频代码会被编译但本分支用不到**，是从音频分支带来的，属"挂着但不调用"。
7. **RPLIDAR 仍需 5V 外部供电**、TX/RX 交叉接线。

---

## 调试与使用注意点

- 某个传感器没数据：先看初始化日志那一段是 OK 还是 WARN；I2C 设备用扫描确认 0x23/0x29 都在；雷达确认 5V 与交叉接线。
- 想提高雷达可用性：把雷达读取移到独立 FreeRTOS 任务里持续读，别和 100ms 主循环绑一起。

---

## 目录结构

```
├── main/
│   ├── main.c                  # 主程序：三传感器初始化（共享 I2C）+ 100ms 主循环
│   └── IIC.c                   # IMU 单独测试代码（备用，未编译）
├── components/
│   ├── imu_module/             # IMU 换算驱动
│   ├── i2c_module/             # IMU I2C 读写（复用 BSP 的 bus_handle）
│   ├── BSP/MYIIC/              # 共享 I2C 总线的唯一初始化处（i2c_master 新 API）
│   └── Middlewares/
│       ├── VL53L1X/            # VL53L1X ToF 驱动 + ST ULD API（复用 bus_handle）
│       └── RPLIDAR/            # RPLIDAR C1 UART 驱动
└── CMakeLists.txt
```
