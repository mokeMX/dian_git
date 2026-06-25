# VL53L1X / RPLIDAR 分支

> ⚠️ **分支名有误导**：分支叫 `VL53L1X`，但**当前编译运行的其实是 RPLIDAR C1 代码**。VL53L1X 的驱动（ST ULD API）已经备好，但它的 `app_main` 在 `main/main.c` 顶部被**整段注释**了。本分支的真正用途是「同一套工程里，RPLIDAR 与 VL53L1X 二选一，注释切换」。

此分支用来在「激光雷达」和「ToF 单点测距」之间快速切换验证，两者都是「跟随行李箱」的测距/避障候选传感器。

---

## 当前功能

- **RPLIDAR C1 扫描**（✅ 已激活，UART1）:
  - UART 初始化 → **先 stop+reset 复位** → 健康自检 → 读设备信息 → 启动扫描 → 循环读点
  - 只打印 `quality > 0` 的有效点
- **VL53L1X ToF 测距**（💤 已注释，可启用）:
  - I2C 初始化 + 传感器初始化 `vl53l1x_app_init`
  - 单次测距 `vl53l1x_get_single_distance`，毫米级

---

## 硬件连接

| 传感器 | 接口 | 引脚（以代码为准） |
|--------|------|--------------------|
| RPLIDAR C1（当前激活） | **UART1**，460800 | ESP TX=GPIO17、ESP RX=GPIO18（与雷达 RX/TX **交叉**接），5V 外部供电 + 共地 |
| VL53L1X（注释待启用） | **I2C0**，400kHz | SCL=GPIO38、SDA=GPIO39，地址 **0x52**（8 位） |

> 注意：RPLIDAR 在本分支用的是 `UART_NUM_1`（不是 `rplidar` 分支的 UART2）；VL53L1X 的 I2C 引脚（38/39）也和 `IMU` 分支（37/38）不一样，后续做整合时要统一，别照搬。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 🔁 如何切换到 VL53L1X（重点）

`main/main.c` 里**只能有一个 `app_main`**。切换步骤：

1. 把**当前激活的 RPLIDAR `app_main`**（文件下半部分，`#include "rplidar.h"` 那段）整体注释掉。
2. 把**文件顶部两段被注释的 VL53L1X `app_main` 之一**取消注释（一段用 `ESP_LOGI` 打印、一段用 `printf` 打印，**只放开一个**，否则重复定义 `app_main` 编译报错）。
3. 确认 VL53L1X 接到 SCL=38 / SDA=39、供电 3.3V、共地，重新 `idf.py build flash monitor`，串口应每 500ms 打印 `Distance: xxx mm`。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | RPLIDAR 主流程 + VL53L1X 备用入口 | 顶部两段注释掉的 VL53L1X `app_main`（切换用）；下半部分激活的 RPLIDAR 流程，比 `rplidar` 分支多了 `rplidar_stop`+`rplidar_reset`+1s 等待，复位更稳；用 `UART_NUM_1`。 |
| `components/Middlewares/VL53L1X/vl53l1x.h/.c` | ToF 应用层封装 | 引脚/地址宏（SCL=38、SDA=39、`VL53L1X_DEFAULT_DEV_ADDR=0x52`）、`vl53l1x_app_init`、`vl53l1x_get_single_distance`。 |
| `components/Middlewares/VL53L1X/VL53L1X_api.c/.h` | ST ULD 官方 API (v3.5.5) | 底层寄存器操作，一般不用改；要改测距模式/时序在这。 |
| `components/Middlewares/VL53L1X/VL53L1X_calibration.c/.h` | 官方校准例程 | 需要偏移/串扰校准时参考。 |
| `components/Middlewares/RPLIDAR/rplidar.c/.h` | RPLIDAR 驱动 | 同 `rplidar` 分支（460800、滑窗对齐解析）。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **别被分支名骗了**：默认跑的是 RPLIDAR，不是 VL53L1X。
2. **两个 `app_main` 只能留一个**：切换时务必把另一个注释掉，否则链接期重复定义。
3. **RPLIDAR 用 UART1、VL53L1X 用 I2C0(38/39)**：和其它分支的端口/引脚不一致，整合时统一规划。
4. **RPLIDAR 仍需 5V 外部供电**（启动电流 ~800mA），接线 TX/RX 交叉；VL53L1X 是 3.3V I2C。
5. **VL53L1X 地址 0x52 是 8 位写法**（= 7 位 0x29），用 I2C 扫描工具看到的是 0x29，别以为不匹配。
6. **中文注释是 GBK 编码**：在 UTF-8 编辑器里会乱码，但日志本身是英文（`ESP_LOGI`），`idf.py monitor` 显示正常。

---

## 调试与使用注意点

- RPLIDAR 无数据：查 5V 供电、TX/RX 交叉、UART1/460800。
- VL53L1X 读不到：查 SCL=38/SDA=39 接线、3.3V 供电、I2C 扫描能否看到 0x29、是否取消注释了对应 `app_main`。

---

## 目录结构

```
├── main/
│   └── main.c                          # 顶部=注释的 VL53L1X 入口；下半=激活的 RPLIDAR 入口
├── components/
│   └── Middlewares/
│       ├── VL53L1X/                    # VL53L1X 驱动（ST ULD API v3.5.5）
│       │   ├── vl53l1x.c/h             # 应用层封装（引脚/地址/单次测距）
│       │   ├── VL53L1X_api.c/h         # ST ULD 官方 API
│       │   └── VL53L1X_calibration.c/h # 校准例程
│       └── RPLIDAR/                    # RPLIDAR C1 驱动
└── CMakeLists.txt
```
