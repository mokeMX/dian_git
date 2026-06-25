# dian_git - 嵌入式招新题目

基于 ESP32-S3 (Alientek ATK-DNESP32S3) 的嵌入式入门练习项目，使用 ESP-IDF 框架开发。

> **这个分支是什么**：整个仓库的「起点」分支，只验证最基础的外设（UART 收发 + LED + 按键），用来确认开发板、工具链、烧录链路是否打通。后续 `IMU` / `rplidar` / `I2S` / `传感器整合` / `算法1·2` 等分支都是在打通本分支后逐步往上叠加的功能。第一次拿到板子，建议先跑通本分支。

---

## 功能

- **UART 定时发送**: 以 1Hz 速率从串口发送 `Hello World\r\n`
- **UART 命令响应**: 监听串口输入，当用户按下回车时，立即输出一段预设字符串（招新题目的「暗号」答案）
- **LED 指示**: 板载 LED 控制（BSP 驱动）
- **按键检测**: 板载 BOOT 按键扫描（驱动已就绪，主流程当前未调用）

---

## 硬件要求

- Alientek ATK-DNESP32S3 开发板
- 一根 USB 线（板载 USB-UART，直接接电脑即可，无需额外串口模块）

### 引脚一览（写死在 BSP 头文件里，改之前先看清楚）

| 外设 | 端口/引脚 | 定义位置 |
|------|-----------|----------|
| UART | `UART_NUM_0`，TX=GPIO43，RX=GPIO44，115200-8-N-1 | `components/BSP/UART/uart.h` |
| LED0 | GPIO1 | `components/BSP/LED/led.h` |
| BOOT 按键 | GPIO0 | `components/BSP/KEY/key.h` |

---

## 构建与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 运行现象（烧录后应该看到什么）

- 打开 `idf.py monitor`（或任意串口终端，115200），**每秒会刷一行 `Hello World`**。
- 在终端里随便输入几个字符后**按回车**，会立刻回吐三行预设字符串（`GEL37KXHDU9G` / `FXLKNKWHVURC` / `CE4K7KEYCUPQ`）——这是招新题目的「校验答案」，正常现象，不是乱码。
- 如果只看到 `Hello World` 而回车没有反应，先确认终端是不是把回车吞掉了（需要真正发出 `\r` 或 `\n`）。

---

## 关键代码导读（要看哪些文件、重点看什么）

| 文件 | 作用 | 重点看什么 |
|------|------|-----------|
| `main/main.c` | 程序主体 | `app_main()` 里初始化顺序：`nvs_flash_init` → `led_init` → `key_init` → `usart_init(115200)`，然后用 `xTaskCreate` 起两个并行任务。重点理解 `uart_tx_task`（1Hz 发送）和 `uart_rx_task`（`uart_read_bytes` 超时 20ms，收到 `\r`/`\n` 才回包）的写法。 |
| `components/BSP/UART/uart.c` | 串口初始化 | `usart_init()` 里 `uart_config_t` 的参数、`uart_set_pin`、`uart_driver_install`（装了 2×1024 的收发缓冲）。注意函数**没有用 `ESP_ERROR_CHECK` 包裹**，初始化失败不会报错，调试时要留意。 |
| `components/BSP/UART/uart.h` | 串口宏 | 改引脚/串口号/缓冲区大小都在这里（`USART_UX`、`UART_TX_GPIO_PIN`、`RX_BUF_SIZE`）。 |
| `components/BSP/LED/led.c/.h` | LED 驱动 | `LED0_TOGGLE()`、`LED0(x)` 宏，GPIO1。 |
| `components/BSP/KEY/key.c/.h` | 按键驱动 | `key_scan(mode)` 与 `BOOT_PRES` 的用法，方便后续分支复用。 |

---

## ⚠️ 注意事项 / 容易踩的坑

1. **UART0 与 USB 串口是同一路**：`USART_UX = UART_NUM_0`、TX=GPIO43/RX=GPIO44 正是 `idf.py monitor` 用的那一路。所以 `printf` 的日志、`Hello World`、你的输入、回包**全挤在同一个串口**上，调试时不要奇怪它们混在一起。如果要把数据口和日志口分开，得换成 UART1/UART2 并改引脚。
2. **回车响应是「招新暗号」**：`uart_rx_task` 里那串 `GEL37KXHDU9G...` 是题目答案，不要当成业务逻辑删掉；要改成你自己的回显逻辑，看 `main.c` 中 `uart_rx_task` 里拼 `resp` 的那一行。
3. **`app_main` 末尾的 `while(1)` 整段被注释掉了**：原本的按键翻转 LED、串口回显代码都在注释里（`main.c` 后半段）。当前真正生效的是上面两个 task，`key_init()` 虽然调用了但 `key_scan` 没人调，**按键此刻不起作用**，需要的话把注释逻辑挪进任务里。
4. **NVS 容错被注释**：`nvs_flash_init()` 的返回值没有处理，`ESP_ERR_NVS_NO_FREE_PAGES` / `NEW_VERSION_FOUND` 时不会自动 `erase`（`main.c` 里 `nvs_flash_init` 下方的注释块）。换芯片或改分区表后如果起不来，先把这段容错放开。
5. **任务栈大小**：rx 任务用了 4096、tx 任务 2048。如果在 rx 里加 `printf` 或更大的处理逻辑，注意别栈溢出。

---

## 调试与使用注意点

- 烧录后没反应：先确认 `idf.py set-target esp32s3` 已执行、选对串口、波特率 115200。
- 看不到日志：ESP32-S3 部分板子需要按住 BOOT 再按 RESET 进下载模式；`idf.py monitor` 里 `Ctrl+]` 退出。
- 改了引脚编译不过：检查 `uart.h` / `led.h` / `key.h` 的宏与板子实际丝印是否一致。

---

## 目录结构

```
├── main/
│   ├── main.c          # 主程序：UART 收发任务、LED、按键初始化
│   └── CMakeLists.txt
├── components/
│   ├── BSP/            # 板级支持包（LED、KEY、UART 驱动）
│   └── Middlewares/    # 中间件（本分支暂空，后续分支扩展）
├── CMakeLists.txt      # 顶层 CMake 配置
├── sdkconfig           # ESP-IDF 5.4.0 项目配置
└── partitions-16MiB.csv  # 16MB Flash 分区表
```
