# dian_git - 嵌入式招新题目

基于 ESP32-S3 (Alientek ATK-DNESP32S3) 的嵌入式入门练习项目，使用 ESP-IDF 框架开发。

## 功能

- **UART 定时发送**: 以 1Hz 速率从串口发送 "Hello World\r\n"
- **UART 命令响应**: 监听串口输入，当用户按下回车时，立即输出预设字符串
- **LED 指示**: 板载 LED 控制（BSP 驱动）
- **按键检测**: 板载 BOOT 按键扫描

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- USB-UART 串口连接

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 目录结构

```
├── main/
│   ├── main.c          # 主程序：UART 收发、LED、按键初始化
│   └── CMakeLists.txt
├── components/
│   ├── BSP/            # 板级支持包（LED、KEY、UART 驱动）
│   └── Middlewares/    # 中间件
├── CMakeLists.txt      # 顶层 CMake 配置
├── sdkconfig           # ESP-IDF 5.4.0 项目配置
└── partitions-16MiB.csv
```
