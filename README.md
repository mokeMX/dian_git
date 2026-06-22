# VL53L1X / RPLIDAR 分支

此分支名为 VL53L1X，但实际运行的是 RPLIDAR C1 代码。VL53L1X 驱动代码已就位但被注释，可直接切换使用。

## 当前功能

- **RPLIDAR C1 扫描**（已激活）:
  - 设备初始化与自检
  - 读取设备信息（型号、固件、序列号）
  - 健康状态检查
  - 连续扫描模式 + 点云输出

- **VL53L1X ToF 测距**（已注释，可启用）:
  - I2C 初始化
  - 传感器校准
  - 毫米级距离测量

## 硬件连接

| 传感器 | 接口 | 引脚 |
|--------|------|------|
| RPLIDAR C1 | UART1 | TX=GPIO17, RX=GPIO18 |
| VL53L1X | I2C | （代码中已预留） |

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 目录结构

```
├── main/
│   └── main.c                          # 主程序（VL53L1X 部分注释，RPLIDAR 活跃）
├── components/
│   └── Middlewares/
│       ├── VL53L1X/                    # VL53L1X 驱动（ST ULD API v3.5.5）
│       │   ├── vl53l1x.c/h            # 应用层驱动
│       │   ├── VL53L1X_api.c/h        # ST ULD API
│       │   └── VL53L1X_calibration.c/h # 校准例程
│       └── RPLIDAR/                    # RPLIDAR C1 驱动
└── CMakeLists.txt
```

## 注意事项

- 波特率：460800（RPLIDAR）
- 要使用 VL53L1X，取消 main.c 中相关代码注释即可
