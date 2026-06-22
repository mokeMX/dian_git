# RPLIDAR C1 激光雷达扫描

基于 RPLIDAR C1 的独立激光雷达扫描测试程序，读取设备信息并实时输出扫描点云数据。

## 功能

- **设备识别**: 读取 RPLIDAR 型号、固件版本、序列号
- **健康检查**: 检查激光雷达健康状态
- **连续扫描**: 启动 360° 连续扫描模式
- **点云输出**: 实时输出每个扫描点的角度、距离（mm）和质量

## 硬件连接

| RPLIDAR C1 | ESP32-S3 |
|------------|----------|
| TX         | GPIO18   |
| RX         | GPIO17   |
| 5V         | 5V (外部供电) |
| GND        | GND      |

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 目录结构

```
├── main/
│   └── main.c                  # 主程序：RPLIDAR 初始化 + 扫描循环
├── components/
│   └── Middlewares/RPLIDAR/
│       ├── rplidar.c           # RPLIDAR C1 UART 驱动
│       └── rplidar.h           # 二进制协议解析（5字节响应包）
└── CMakeLists.txt
```

## 注意事项

- 波特率：460800
- 需要 5V 外部供电，启动电流可达 800mA
- 上电后需等待电机稳定旋转
