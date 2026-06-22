# audioplay - SD卡 WAV 音乐播放器

基于 ESP32-S3 + ES8388 + SPI LCD 的 WAV 音频播放器，支持从 SD 卡读取 WAV 文件播放，并通过 SPI LCD 显示曲目信息和状态。

## 功能

- **WAV 音频播放**: 支持标准 PCM WAV 格式文件
- **SD 卡文件系统**: 使用 FATFS 读取 SD 卡中的音频文件
- **曲目切换**: KEY0 下一首，KEY1 上一首，KEY2 暂停/播放
- **LCD 显示**: SPI LCD 屏幕显示播放状态和曲目信息
- **中文字库**: 支持中文/英文文本显示

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- ES8388 音频编解码器
- SPI LCD 显示屏
- SPI SD 卡槽（含 FAT32 格式的 SD 卡）
- XL9555 GPIO 扩展器

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 目录结构

```
├── main/
│   ├── main.c                # 主程序：外设初始化 + 播放循环
│   ├── CMakeLists.txt
│   └── APP/AUDIO/
│       ├── audioplay.c/h     # 播放状态机
│       └── wavplay.c/h       # WAV 文件解析器
├── components/
│   ├── BSP/
│   │   ├── ES8388/           # 音频编解码器驱动
│   │   ├── MYI2S/            # I2S 音频输出驱动
│   │   ├── SPILCD/           # SPI LCD 显示驱动
│   │   ├── SPI_SD/           # SPI SD 卡驱动
│   │   ├── MYSPI/            # SPI 总线初始化
│   │   ├── MYIIC/            # I2C 总线初始化
│   │   ├── XL9555/           # GPIO 扩展器驱动
│   │   └── LED/              # LED 驱动
│   └── Middlewares/
│       ├── MYFATFS/          # FATFS 文件系统中间件
│       └── TEXT/             # 文本/字库显示中间件
└── partitions-16MiB.csv
```
