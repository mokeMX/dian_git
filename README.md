# I2S 音频波形生成

基于 ESP32-S3 + ES8388 音频编解码器的 I2S 音频测试信号发生器。

## 功能

- **锯齿波输出**: 500Hz 锯齿波
- **差拍正弦波**: 左声道 1001Hz + 右声道 999Hz，可听差拍效果
- **立体声正弦波**: 左声道 1kHz + 右声道 4kHz
- **声道切换**: 通过按键切换仅左声道/仅右声道/双声道输出
- **波形切换**: 通过 KEY0/KEY1/KEY3 按键切换不同波形模式

## 按键功能

| 按键 | 功能 |
|------|------|
| KEY0 | 切换到锯齿波（500Hz） |
| KEY1 | 切换到差拍正弦波 |
| KEY3 | 切换到立体声正弦波 |
| BOOT | 声道切换（左/右/立体声） |

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- ES8388 音频编解码器（板载）
- I2S 音频输出接口
- XL9555 GPIO 扩展芯片（板载）

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 关键驱动

- `components/BSP/MYI2S/` - I2S 音频输出驱动
- `components/BSP/ES8388/` - ES8388 音频编解码器驱动（I2C 配置）
- `components/BSP/XL9555/` - GPIO 扩展器驱动
- `components/BSP/KEY/` - 按键驱动
- `components/BSP/LED/` - LED 驱动
