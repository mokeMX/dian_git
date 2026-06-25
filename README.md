# audioplay - SD卡 WAV 音乐播放器

基于 ESP32-S3 + ES8388 + SPI LCD 的 WAV 音频播放器，支持从 SD 卡读取 WAV 文件播放，并通过 SPI LCD 显示曲目信息和状态。

> **这个分支是什么**：音频支线里「完整应用」分支——把 I2S + ES8388 + SPI LCD + SPI SD 卡 + FATFS 文件系统 + 中文字库串成一台能放歌的小播放器。相比 `I2S` 分支（自己合成波形、且没初始化 ES8388），本分支的 `main.c` 把 **ES8388、喇叭使能、SD 卡、字库**都正确初始化好了，是学这几样外设协同的好样板。

---

## 功能

- **WAV 音频播放**: 解析标准 PCM WAV（读取 fmt/data 块，支持任意采样率/位宽/声道，串口会打印解析结果）
- **SD 卡文件系统**: FATFS 读取 SD 卡 `0:/MUSIC` 目录下的音频文件
- **曲目切换**: KEY0 下一首，KEY1 上一首，KEY2 暂停/播放
- **断点记忆**: 每首歌记录上次播放位置（`song_breakpoints` 数组）
- **LCD 显示**: 曲目序号、文件名、播放时间、码率、状态
- **中文字库**: TEXT 中间件 + 字库，支持中文/英文显示

---

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- ES8388 音频编解码器（I2C 控制 + I2S 数据）
- SPI LCD 显示屏
- SPI SD 卡槽 + **FAT32 格式 SD 卡**
- XL9555 GPIO 扩展器（KEY0~3 + 喇叭使能 SPK_EN）
- 耳机或喇叭

> 引脚分散在各 BSP 头文件里：I2S 见 `components/BSP/MYI2S/myi2s.h`，LCD/SD 共用 SPI 见 `components/BSP/MYSPI/`、`SPILCD/`、`SPI_SD/`，I2C 见 `MYIIC/`。

---

## ⭐ SD 卡准备（最关键，不弄好就一直卡在初始化）

SD 卡必须是 **FAT32**，并且**同时**包含两类文件：

1. **音乐**：放在 `0:/MUSIC` 目录下的 WAV 文件（没有这个文件夹会一直显示 `MUSIC文件夹错误!`，文件夹空会显示 `没有音乐文件!`）。
2. **字库**：`fonts_init()` / `fonts_update_font()` 会**从 SD 卡的 `0:` 根加载/更新字库**。如果卡里没有正点原子的字库文件，开机会一直循环 `Font Update Failed!`——这一步过不去就到不了播放界面。第一次用需要按正点原子例程把字库（SYSTEM/FONT 等）拷进卡里。

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 运行现象（开机流程）

LCD 依次完成：ES8388 自检 → 打开喇叭 → SD 卡自检 → 字库初始化 → 显示按键提示 → 进入 `0:/MUSIC` 播放第一首。任一外设失败会**停在对应错误界面循环闪烁**（见下「易踩坑」），不会崩溃但也不往下走。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | 开机初始化链 | 初始化顺序：NVS→LED→SPI→KEY→IIC→XL9555→LCD→**ES8388**（带重试）→`SPK_EN`→**SD 卡**（带重试）→`exfuns_init`→**字库**→`while(1){ audio_play(); }`。每一步失败都在 LCD 上提示。 |
| `main/APP/AUDIO/audioplay.c` | 播放列表/翻页状态机 | `audio_play()`：扫描 `0:/MUSIC`、统计有效音频数(`audio_get_tnum`)、建立偏移表 `wavoffsettbl`、断点表 `song_breakpoints`，循环里根据返回的 `KEY0/KEY1` 切歌。**只在屏上列前 4 个文件名**（`if(i>=3)break`）。 |
| `main/APP/AUDIO/wavplay.c` | WAV 解析 + 出声 | `wav_decode_init` 解析 WAV 头并 `printf` 各字段；`music()` 任务做实际播放：先配 **ES8388（`es8388_adda_cfg`/`output_cfg`/`hpvol_set`/`spkvol_set`）** 再 `i2s_tx_write` 双缓冲送数；`status & 0x0F == 0x03` 播放 / `0x00` 暂停（死等循环）。 |
| `components/Middlewares/MYFATFS/` | FATFS 适配 | `exfuns_init`、`exfuns_file_type`（靠扩展名判断 `T_WAV` 等）。 |
| `components/Middlewares/TEXT/` | 字库/文本 | `fonts_init`、`text_show_string`，中文显示依赖 SD 卡字库。 |
| `components/BSP/ES8388/` | 编解码器 | 音量、通道、DAC 配置都在这。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **所有初始化都是「失败就卡在 LCD 循环」**：ES8388/SD/字库任一不就绪，程序会停在该步闪烁报错。调试时**先看 LCD 停在哪一行**，就知道是哪个外设的问题。
2. **字库依赖 SD 卡**（见上 SD 准备）——这是「新卡放不出歌」最常见的坑，很多人以为只放 WAV 就行。
3. **WAV 路径写死 `0:/MUSIC`**，不是根目录。要改目录就改 `audioplay.c` 里的字符串。
4. **只支持 WAV**：`audio_play_song` 里 `T_MP3` 分支是空的（`mp3_play_song` 没实现），放 MP3 会被跳过并 `printf("can't play")`。
5. **每首歌结束会 `i2s_deinit()` 并 `vTaskDelete` 掉 music 任务**：即每曲一个播放任务、切歌时重建 I2S。改播放逻辑时注意这套生命周期，别在任务删除后还访问其资源。
6. **音量默认 20**（`es8388_hpvol_set(20)`/`spkvol_set(20)`），太小/太大都在 `wavplay.c:168-169` 改。
7. **源码编码**：`main.c`、`audioplay.c` 是 **GBK 编码**（在 UTF-8 编辑器里中文注释会乱码，属正常，设成 GBK/GB2312 即可正常看）；`wavplay.c` 是 UTF-8。改文件时注意保持各自编码，别让中文字符串变成乱码烧进去。
8. **开机先送一段静音**（`wavplay.c` 里 `i2s_tx_write(tbuf, ...)`）是为了压掉 ES8388 上电的「沙沙」声，别删。

---

## 调试与使用注意点

- 放不出声但 LCD 正常：查 ES8388 音量、`SPK_EN` 是否拉低使能、耳机/喇叭是否插对。
- 切歌没反应：KEY0/1/2 是 XL9555 上的扩展键，确认 `xl9555_key_scan` 正常。
- 想验证 WAV 是否被正确解析：看串口里 `wavx->samplerate/bitrate/bps...` 那几行打印。

---

## 目录结构

```
├── main/
│   ├── main.c                # 主程序：外设初始化链 + while(audio_play())
│   ├── CMakeLists.txt
│   └── APP/AUDIO/
│       ├── audioplay.c/h     # 播放列表/翻页/断点状态机（GBK 编码）
│       └── wavplay.c/h       # WAV 解析 + music 播放任务（UTF-8 编码）
├── components/
│   ├── BSP/
│   │   ├── ES8388/           # 音频编解码器驱动（音量/通道/DAC）
│   │   ├── MYI2S/            # I2S 音频输出驱动
│   │   ├── SPILCD/           # SPI LCD 显示驱动
│   │   ├── SPI_SD/           # SPI SD 卡驱动
│   │   ├── MYSPI/            # SPI 总线初始化（LCD 与 SD 共用）
│   │   ├── MYIIC/            # I2C 总线初始化
│   │   ├── XL9555/           # GPIO 扩展器（KEY0~3 + SPK_EN）
│   │   └── LED/              # LED 驱动
│   └── Middlewares/
│       ├── MYFATFS/          # FATFS 文件系统中间件
│       └── TEXT/             # 文本/字库显示中间件（依赖 SD 卡字库）
└── partitions-16MiB.csv
```
