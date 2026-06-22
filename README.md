# IMU 9轴传感器数据读取

基于 I2C 通信的九轴 IMU 传感器数据采集程序，持续读取并打印加速度、陀螺仪、磁力计、欧拉角、四元数和气压计数据。

## 功能

- **加速度读取**: X/Y/Z 三轴加速度（m/s²）
- **陀螺仪读取**: X/Y/Z 三轴角速度（°/s）
- **磁力计读取**: X/Y/Z 三轴磁场强度
- **姿态解算**: 欧拉角（Roll/Pitch/Yaw）和四元数
- **气压计读取**: 高度、温度、气压值
- **定时输出**: 每 100ms 通过串口打印全部传感器数据

## 硬件连接

| IMU 模块 | ESP32-S3 |
|----------|----------|
| SCL      | GPIO42   |
| SDA      | GPIO41   |
| VCC      | 3.3V     |
| GND      | GND      |

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 目录结构

```
├── main/
│   └── IIC.c               # 主程序：I2C 初始化 + 循环读取 IMU
├── components/
│   ├── imu_module/          # IMU I2C 驱动（寄存器读写）
│   └── i2c_module/          # I2C 总线底层驱动
└── CMakeLists.txt
```
