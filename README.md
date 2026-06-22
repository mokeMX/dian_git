# imu_vl53l1x_rplidar_merged - 三传感器融合

IMU + VL53L1X + RPLIDAR C1 三传感器同时采集程序，在单个 ESP32-S3 上并行读取三种传感器数据。

## 功能

- **VL53L1X ToF 测距**: 读取毫米级距离数据
- **IMU 9轴 + 气压计**: 加速度、陀螺仪、磁力计、欧拉角、四元数、气压/温度/高度
- **RPLIDAR C1 激光扫描**: 360° 二维激光扫描点云（角度 + 距离 + 质量）
- **统一串口输出**: 三种传感器数据分帧打印至串口

## 硬件连接

| 传感器 | 接口类型 | 引脚 |
|--------|----------|------|
| IMU | I2C | SCL=GPIO42, SDA=GPIO41 |
| VL53L1X | I2C | 与 IMU 共用 I2C 总线 |
| RPLIDAR C1 | UART | TX=GPIO17, RX=GPIO18 |

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 目录结构

```
├── main/
│   ├── main.c                  # 主程序：三传感器初始化 + 循环读取
│   └── IIC.c                   # IMU 单独测试代码（备用）
├── components/
│   ├── imu_module/             # IMU I2C 驱动
│   ├── i2c_module/             # I2C 总线驱动
│   ├── BSP/MYIIC/             # 另一套 I2C 初始化
│   └── Middlewares/
│       ├── VL53L1X/           # VL53L1X ToF 驱动 + ST ULD API
│       └── RPLIDAR/           # RPLIDAR C1 UART 驱动
└── CMakeLists.txt
```

## 注意事项

- VL53L1X 和 IMU 共享同一 I2C 总线，需确认地址不冲突（IMU: 0x23, VL53L1X: 0x52）
- RPLIDAR C1 需要 5V 供电，启动电流可达 800mA
- 波特率 460800
