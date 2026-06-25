# 传感器整合 (Sensor Driver Hub)

六传感器驱动统一框架项目，提供标准化的传感器驱动接口和 menuconfig 配置系统。

> **这个分支是什么**：把前面零散调试的各传感器（超声波/UWB/压力/激光雷达/IMU/ToF）**重构成统一风格的独立组件**，用一个 `examples/sensor_hub` 例程按 menuconfig 开关挨个初始化、统一打印。它是「跟随行李箱」整机软件的**传感器底座**，`算法1/2` 就是在它之上加跟随避障。
>
> ⚠️ **这是「原始整合版」，已知存在引脚冲突，仅供理解架构。要实际跑请用 `传感器修改4` 或 `算法2` 分支。**

---

## 支持的传感器

| 传感器 | 类型 | 接口 | 功能 | 默认地址/参数 |
|--------|------|------|------|----------------|
| A02YYUW | 超声波测距 | UART1 9600 | 毫米级（最大 4.5m） | RX=36 TX=37 |
| BU03/BU04 | UWB 超宽带 | UART1 115200 | 室内定位（被动监听 TWR/PDOA） | RX=36 TX=37 |
| FSR | 薄膜压力 | ADC1 | 压力（模拟量转力值） | GPIO8/CH7 |
| RPLIDAR C1 | 激光雷达 | UART2 460800 | 360° 二维扫描 | RX=36 TX=37 |
| IMU | 九轴惯性 | I2C0 | 加速度(g)/陀螺/磁力/欧拉角(deg)/气压 | 0x23 |
| VL53L1X | ToF 激光测距 | I2C0（共享） | 毫米级（最大 4m） | 0x52(8位)/0x29(7位) |

---

## 🚨 已知引脚冲突（本分支最重要的事）

看上表就能发现：**A02YYUW、BU_UWB、RPLIDAR 三者的默认 RX/TX 全是 GPIO36/37**，而且 **I2C 默认 SCL 也落在 GPIO37**。后果：

- 这三个串口传感器**默认引脚完全撞车**，同时启用必然抢占同一对 GPIO；
- 即使只开一个串口传感器，再开 IMU/VL53L1X，I2C 的 SCL=37 仍会和 UART TX=37 冲突。

所以**默认配置下最多只能稳妥地单独启用一类传感器**。要六个一起跑，必须手动在 menuconfig 把每个传感器的引脚改到互不冲突——这正是后续 `传感器整合修改` → `传感器修改2/3/4` 在做的事（推荐直接用那些分支）。

---

## 特性

- **模块化设计**: 每个传感器是 `components/sensors/` 下独立组件，接口风格统一（`xxx_default_config` → `xxx_init` → `xxx_read`）
- **menuconfig 配置**: `CONFIG_SENSOR_HUB_<sensor>_ENABLE` 开关 + 每个传感器的引脚/UART/波特率/地址都可配
- **共享 I2C 总线**: IMU 与 VL53L1X 用 `i2c_new_master_bus` 建一条总线（新版 `i2c_master` 驱动），各自挂不同地址
- **自带接线诊断**: 每个传感器读不到数据时打印 `[XXX][WAIT] check ...` 提示该查什么

---

## 硬件连接

详见 `docs/sensors/pinout-and-wiring.md`（已在文件顶部补充冲突警告）。

---

## 构建与烧录

```bash
cd examples/sensor_hub
idf.py set-target esp32s3
idf.py menuconfig   # Sensor Hub 菜单里勾选要启用的传感器、改引脚避免冲突
idf.py build
idf.py flash monitor
```

---

## 测试（PC 端，无需硬件）

协议解析层（A02YYUW / BU UWB / FSR）有纯 C 单元测试，用 gcc 编译运行：

```bash
bash tests/protocol/run_tests.sh
```

> ⚠️ 旧 README 写的 `python a02yyuw_test.py` 等**在本分支不存在**；实际测试是 `tests/protocol/test_sensor_parsers.c`，由 `run_tests.sh` 用 gcc 编译。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `examples/sensor_hub/main/main.c` | 统一初始化 + 主循环 | 每个 `#if CONFIG_SENSOR_HUB_*_ENABLE` 块就是一个传感器的「初始化+读取」范式；共享 I2C `i2c_new_master_bus` 建总线后传给 IMU/VL53L1X 的 `external_bus`；主循环 500ms 轮询打印；大量 `[WAIT] check...` 是接线诊断。 |
| `examples/sensor_hub/main/Kconfig.projbuild` | 所有可配项 | **引脚默认值都在这**（看 `default 36/37` 就知道冲突来源）；改引脚、UART 号、波特率、I2C 地址都在此菜单。 |
| `components/sensors/a02yyuw/a02yyuw.c` | 超声波 | UART 帧解析（帧头 + 校验和），`a02yyuw_default_config`/`a02yyuw_read`。 |
| `components/sensors/bu_uwb/bu_uwb.c` | UWB | 按行解析 `JSxxxx{"TWR":{...}}`，`bu_uwb_parse_twr_line`/`parse_distance_line`，被动监听模式。 |
| `components/sensors/fsr_adc/fsr_adc.c` | 压力 | ADC 读数 → 电压 → 力换算，公式 `U=0.0004F+0.0749`（**必须用实物重新标定**）。 |
| `components/sensors/imu_i2c/imu_i2c.c` | IMU | 复用外部 I2C 总线、换算系数同 IMU 分支。 |
| `components/sensors/rplidar_c1/rplidar_c1.c` | 雷达 | 句柄式 `rplidar_c1_t`、滑窗对齐解析。 |
| `components/sensors/vl53l1x_tof/vl53l1x_tof.c` | ToF | timing budget / inter-measurement 可配。 |
| `tests/protocol/test_sensor_parsers.c` | PC 单测 | 不依赖硬件，验证协议解析逻辑。 |
| `AGENTS.md` | 工作区说明 | Codex/智能体协作规则（WSL 路径等），不是固件代码。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **默认引脚冲突**（见上）——这是本分支的头号问题，实际使用请改引脚或换 `传感器修改4`。
2. **工作目录是 `examples/sensor_hub`**，不是仓库根目录；`idf.py` 要在那里执行。
3. **FSR 力值公式未标定**：`U=0.0004F+0.0749` 是示例值，限幅 0–6kg，必须用你的实物压力标定后才能当算法输入。
4. **A02YYUW / RPLIDAR 供电**：A02YYUW 按模块标称 3.3V 或 5V；RPLIDAR C1 必须 5V 且能给 ~800mA 启动电流，否则不转/测不准。
5. **BU UWB 是被动监听**：ESP 只接它的 TX→ESP RX，解析 UWB 套件输出的 TWR 行；接 AT 命令口或没设成输出模式会收不到。
6. **I2C 地址**：IMU 0x23、VL53L1X 0x29(7位)，扫描看到的是这两个。
7. **测试命令以 `run_tests.sh` 为准**（不是 python，见上）。

---

## 目录结构

```
├── components/sensors/
│   ├── a02yyuw/               # A02YYUW 超声波驱动
│   ├── bu_uwb/                # BU03/BU04 UWB 驱动（TWR 行解析）
│   ├── fsr_adc/               # FSR 压力传感器驱动（需标定）
│   ├── imu_i2c/              # I2C 九轴 IMU 驱动
│   ├── rplidar_c1/           # RPLIDAR C1 驱动
│   └── vl53l1x_tof/          # VL53L1X ToF 驱动
├── examples/sensor_hub/
│   └── main/
│       ├── main.c            # 传感器中心示例（按 Kconfig 开关初始化）
│       └── Kconfig.projbuild # menuconfig 配置菜单（引脚默认值在此）
├── docs/sensors/             # 驱动使用、引脚接线、测试流程文档
├── tests/protocol/           # PC 端协议单元测试（run_tests.sh + .c）
└── AGENTS.md                 # 工作区/智能体协作指南
```
