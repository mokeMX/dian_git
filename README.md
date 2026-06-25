# I2S 音频波形生成

基于 ESP32-S3 + ES8388 音频编解码器的 I2S 音频测试信号发生器。

> **这个分支是什么**：音频外设练习分支。用 ESP32-S3 的 I2S 外设实时合成几种测试波形（锯齿波 / 拍频正弦 / 双频立体声），通过板载 ES8388 编解码器输出，用按键切换波形和声道。和 `audioplay` 分支属于同一条「音频」支线——本分支是「自己用数学公式生成波形」，`audioplay` 是「从 SD 卡读 WAV 播放」。

---

## 功能

- **锯齿波输出**: 500Hz 锯齿波（默认上电模式，仅右声道）
- **差拍正弦波**: 左声道 1001Hz + 右声道 999Hz，能听到约 2Hz 的「嗡——嗡——」拍频
- **立体声正弦波**: 左声道 1kHz + 右声道 4kHz
- **声道切换**: BOOT 键循环切换 仅右 → 左+右 → 仅左（软件置零实现，不是硬件静音）
- **波形切换**: KEY0/KEY1/KEY3 切换不同波形模式

---

## 按键功能（以代码 `main/main.c` 为准）

> ⚠️ 这张表按**实际代码**填写。若你看到的旧 README 写的是「KEY0=锯齿波」，那是笔误——代码里 KEY0 其实是拍频。

| 按键 | 实际触发的模式 | 同时设定的声道 |
|------|----------------|----------------|
| KEY0 | 差拍正弦波（L:1001Hz / R:999Hz） | 强制 L+R |
| KEY1 | 立体声正弦波（L:1kHz / R:4kHz） | 强制 L+R |
| KEY3 | 锯齿波 500Hz | 强制仅右声道 |
| BOOT | 仅切换声道：右 → 左+右 → 左（循环），并翻转 LED0 | — |

KEY0/1/3 是 **XL9555 扩展 IO 上的按键**（`xl9555_key_scan`），BOOT 是芯片 GPIO0（`key_scan`）。

---

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- ES8388 音频编解码器（板载，I2C 控制 + I2S 数据）
- XL9555 GPIO 扩展芯片（板载，KEY0~3 和喇叭使能 SPK_EN 都挂在它上面）
- 耳机或喇叭

### I2S 引脚（定义在 `components/BSP/MYI2S/myi2s.h`）

| 信号 | 引脚 | 说明 |
|------|------|------|
| MCLK | GPIO3 | 主时钟 → ES8388_MCLK |
| BCLK | GPIO46 | 位时钟 → ES8388_SCLK |
| WS/LRCK | GPIO9 | 声道选择 |
| DOUT | GPIO10 | ESP→ES8388 数据（播放） |
| DIN | GPIO14 | ES8388→ESP 数据（录音，本分支没用到） |

采样率 44100Hz、16bit 立体声、MCLK = 256×fs。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

烧录后串口会打印 `I2S Audio Lab Started!`，默认从右声道输出 500Hz 锯齿波。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | 波形合成 + 按键状态机 | `audio_task()`：每帧算 128 点 × 左右声道，用 `sin()`/`fmod()` 生成波形，`amplitude=12000`（音量，最大 32767），写进 `i2s_tx_write`。`app_main()` 里 `while(1)` 是按键状态机，改波形/声道映射就改这里。 |
| `components/BSP/MYI2S/myi2s.c` | I2S 驱动 | `myi2s_init()` 配置标准 Philips 模式、采样率、引脚；`i2s_tx_write()` 是阻塞写（超时 1000ms）。改采样率/引脚看这里和 `myi2s.h`。 |
| `components/BSP/ES8388/es8388.c/.h` | 编解码器驱动 | `es8388_init`、`es8388_adda_cfg`、`es8388_output_cfg`、`es8388_hpvol_set`/`spkvol_set`——**控制实际出不出声的就是这些函数**（见下方「易踩的坑」）。 |
| `components/BSP/XL9555/xl9555.c/.h` | 扩展 IO | `xl9555_key_scan` 读 KEY0~3；`SPK_EN_IO`(0x0004) 是喇叭功放使能位。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **没声音？先查 ES8388 有没有初始化！** 本分支 `app_main()` 只调了 `myi2s_init()`，**并没有调用 `es8388_init()` 配置编解码器，也没有打开喇叭功放 `SPK_EN`**。I2S 只是把数字信号送到 ES8388 的脚上，要真正出声还得用 I2C 把 ES8388 的 DAC/输出通道/音量配好。如果上电没声音，参考 `audioplay` 分支 `main/APP/AUDIO/wavplay.c` 里的初始化序列补上：
   ```c
   es8388_adda_cfg(1,0);      // 开 DAC，关 ADC
   es8388_output_cfg(1,1);    // 开喇叭 + 耳机通道
   es8388_hpvol_set(20);      // 耳机音量
   es8388_spkvol_set(20);     // 喇叭音量
   // 喇叭还需 XL9555 拉高 SPK_EN
   ```
2. **`main/APP/AUDIO/` 里的 wavplay.c / audioplay.c 在本分支是「死代码」**：`main/CMakeLists.txt` 只编译 `main.c`，那两个文件不参与编译，是从 `audioplay` 分支带过来的残留。别误以为本分支在放 WAV。
3. **波形/声道映射和直觉不一致**：按键不是「按一下播一种」，而是会**同时强制设定声道**（KEY0/1 设成 L+R，KEY3 设成仅右）。BOOT 只改声道不改波形。调试听感前先看清 `main.c` 的 `switch`。
4. **CPU 占用**：`audio_task` 用 `double` 精度逐点算三角函数，钉在核心 1、优先级 10。如果你把采样率或频率调很高，注意 `i2s_tx_write` 是阻塞写，算不过来会卡音/出现欠载（underrun）。
5. **拍频 t 复位**：`if (t > 1.0) t -= 1.0;` 会在 1 秒处把相位拉回，999/1001Hz 不是 1 的整数倍，复位瞬间有极小相位跳变（基本听不出），知道即可。

---

## 调试与使用注意点

- 想验证 I2S 波形是否正确，最稳的是接示波器看 DOUT/BCLK，或先把波形换成单一 1kHz 正弦确认链路。
- 改音量：调 `main.c` 的 `amplitude`，或在补上 ES8388 后用 `es8388_hpvol_set/spkvol_set`。
- 切到「录音/回环」需要用到 `DIN`(GPIO14) 和 `i2s_rx_read`，本分支没接，需自行扩展。

---

## 关键驱动一览

- `components/BSP/MYI2S/` - I2S 音频输出驱动（采样率/引脚配置）
- `components/BSP/ES8388/` - ES8388 音频编解码器驱动（I2C 配置，决定出不出声）
- `components/BSP/XL9555/` - GPIO 扩展器驱动（KEY0~3 + SPK_EN）
- `components/BSP/KEY/` - 板载 BOOT 按键驱动
- `components/BSP/LED/` - LED 驱动
