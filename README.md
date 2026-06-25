# IMU 9轴传感器数据读取

基于 I2C 通信的九轴 IMU 传感器数据采集程序，持续读取并打印加速度、陀螺仪、磁力计、欧拉角、四元数和气压计数据。

> **这个分支是什么**：姿态传感器练习分支，也是「跟随行李箱」获取自身朝向/运动状态的基础。注意这里的 IMU **不是裸的 MPU6050/ICM 芯片**，而是一块**自带姿态解算的「智能 IMU 模块」**——ESP32 通过 I2C 按「功能码寄存器」直接读取已经算好的欧拉角/四元数/气压，不需要自己跑融合算法。后续 `imu_vl53l1x_rplidar_merged`、`传感器整合`、`算法1/2` 都复用这套 IMU 驱动。

---

## 功能

- **加速度读取**: X/Y/Z 三轴加速度（单位 **g**，量程 ±16g）
- **陀螺仪读取**: X/Y/Z 三轴角速度（单位 **rad/s**，量程 ±2000°/s 换算）
- **磁力计读取**: X/Y/Z 三轴磁场强度（单位 **µT**，量程 ±800µT）
- **姿态解算**: 欧拉角 Roll/Pitch/Yaw（模块输出弧度，驱动里已 ×57.2958 转成**度**）和四元数（w,x,y,z，IEEE754 float）
- **气压计读取**: 高度(m)、温度(°C)、气压(Pa)、气压差
- **定时输出**: 每 100ms 通过串口打印全部数据

> ⚠️ 旧 README 把加速度写成 m/s²、角速度写成 °/s，**与代码不符**。以 `components/imu_module/imu_i2c_driver.c` 里每个 `ReadXxx` 的换算系数为准（见下方代码导读）。

---

## 硬件连接（以代码为准）

| IMU 模块 | ESP32-S3 | 出处 |
|----------|----------|------|
| SCL      | **GPIO37** | `components/i2c_module/i2c_module.h` 的 `I2C_MASTER_SCL_IO` |
| SDA      | **GPIO38** | `components/i2c_module/i2c_module.h` 的 `I2C_MASTER_SDA_IO` |
| VCC      | 3.3V     | — |
| GND      | GND      | — |

- I2C 端口 `I2C_NUM_0`，时钟 **400kHz**，从机 7 位地址 **0x23**，片上已开 SDA/SCL 上拉。
- ⚠️ **引脚冲突提醒**：旧 README 写的是 SCL=GPIO42 / SDA=GPIO41，但**代码里实际是 37/38**。请按你的真实接线为准——要么照代码接 37/38，要么改 `i2c_module.h` 里的两个宏。另外 `i2c_module.c` 注释里还残留 `GPIO21/GPIO22` 字样，那是复制来的旧注释，无效，别被误导。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 运行现象（烧录后应该看到什么）

- 先打印一行固件版本 `Version:x.y.z`（`IMU_I2C_ReadVersion`）。
- 之后每 100ms 刷一整块数据：Acceleration[g] / Gyroscope[rad/s] / Magnetometer[uT] / Euler Angle[deg] / Quaternion / Barometer。
- 若版本读不出、数据全 0 或卡住：基本是 I2C 没通（接线/上拉/地址/引脚不对），看下方排查。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/IIC.c` | **主程序**（注意是 IIC.c 不是 main.c） | `app_main` 流程：`i2c_module_init()` → `IMU_I2C_ReadVersion()` → `while` 里每 100ms `IMU_I2C_ReadAll(&imu_data)` 后 `printf`。想改打印格式/频率就改这里。 |
| `components/imu_module/imu_i2c_driver.h` | 功能码 + 数据结构 | 顶部一堆 `IMU_FUNC_*` 宏就是**模块的「寄存器/功能码」**：0x01 版本、0x04 加速度、0x0A 陀螺、0x10 磁力、0x16 四元数、0x26 欧拉角、0x32 气压、0x70~0x73 校准。`imu_measurement_t` 是统一数据结构。 |
| `components/imu_module/imu_i2c_driver.c` | 解析 + 换算 | **单位换算就在这里**：accel `×16/32767`、gyro `×(2000/32767)×(π/180)`、mag `×800/32767`、euler `×57.2958`（弧度转度）。`to_int16` 是**小端**拼接、`to_float` 直接 `memcpy` 4 字节。还有 3 个 `IMU_I2C_Calibration*` 校准函数。 |
| `components/i2c_module/i2c_module.c/.h` | I2C 底层 | 引脚/频率/地址宏在 `.h`；`.c` 用的是 **ESP-IDF 传统 I2C API**（`i2c_cmd_link_create`/`i2c_master_cmd_begin`），不是新版 `i2c_master` 驱动，移植到别的 IDF 版本时注意。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **主文件是 `main/IIC.c`，不是 `main.c`**：找入口别找错文件。
2. **引脚以代码为准（37/38）**：见上。接线和宏定义对不上是「读不到数据」最常见的原因。
3. **单位别照旧 README**：加速度是 **g**、角速度是 **rad/s**、欧拉角已是**度**。要 m/s² 自己乘 9.8。
4. **校准函数默认没被调用**，但如果你要用：
   - `IMU_I2C_CalibrationMag()` 的等待超时传的是 **0 = 永不超时**，磁校准没成功会**一直阻塞**，调用前要有心理准备（通常需要绕 8 字转动）。
   - `IMU_I2C_CalibrationImu()` 超时 7s、`CalibrationTemp()` 超时 2s 且要求温度在 ±50°C。
5. **没有错误重试**：任意一项 I2C 读失败，`IMU_I2C_ReadAll` 直接返回 -1，但 `IIC.c` **没检查返回值**，会照打印上一帧/零值。调试时如果数据不动，先看串口有没有 `I2C read failed` 日志。
6. **`ReadVersion` 的判断有点小问题**：用 `uint8_t == -1` 比较恒不成立，所以即使读失败也可能照打印版本号——版本能打印不代表 I2C 一定 OK，要结合实际数据判断。

---

## 调试与使用注意点

- 读不到数据排查顺序：① 万用表确认 SCL/SDA 接到 GPIO37/38；② 确认模块供电 3.3V、共地；③ 用 I2C 扫描确认能看到 0x23；④ 线太长/无上拉时把 400kHz 降到 100kHz 试试。
- 想加第二个 I2C 设备（如后续分支的 VL53L1X）：它们**共用这条总线**，确认地址不冲突（VL53L1X 默认 0x29/0x52）。
- 改采样频率：调 `IIC.c` 里 `delay_ms(100)`。

---

## 目录结构

```
├── main/
│   ├── IIC.c               # 主程序：I2C 初始化 + 循环读取并打印 IMU
│   └── Kconfig.projbuild   # 残留的 blink-LED 示例菜单（与本工程无关）
├── components/
│   ├── imu_module/         # IMU 模块驱动（功能码、换算、校准）
│   └── i2c_module/         # I2C 总线底层（传统 i2c_cmd API）
└── CMakeLists.txt
```
