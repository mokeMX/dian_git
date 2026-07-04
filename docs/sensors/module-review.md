# 模块审查结论

## A02YYUW

来源：`/home/sp/auto-box/sensor_test/main/a02yyuw.c` 与外部 A02YYUW/A0221AU UART 文档对照。

结论：协议清晰，适合直接整理为驱动。已抽出 `a02yyuw_parse_frame` 和 `a02yyuw_parse_latest`，算法层可直接读取 `distance_mm`。UART 号、RX/TX 引脚、波特率和缓冲区均通过 `a02yyuw_config_t` 配置。

风险：超声波测距受温度、入射角、材质和气流影响，算法层应做滤波；驱动只负责原始有效帧解析。

## BU03/BU04

来源：`blink/main/2.c` 和 Ai-Thinker BU03/BU04 #2717 AT 文档。

结论：原代码是 UART 打印 demo，不是完整驱动，而且 `blink` 工程默认并未编译 `2.c`。现整理为 `bu_uwb_read_bytes`、`bu_uwb_read_line`、`bu_uwb_classify_line`、`bu_uwb_parse_twr_line` 与可选 `bu_uwb_send_command`，测试程序默认按原代码先被动监听 BU04 基站 UART2 输出。

风险：BU03/BU04 有 AT 命令口和测距输出口之分。当前实测基站输出为 `JSxxxx{"TWR": {...}}`，驱动解析 `D/Xcm/Ycm` 等字段；如果后续更换固件或输出模式，需要按新行格式扩展解析器。

## FSR 薄膜压力

来源：`blink/main/blink_example_main.c`。

结论：原代码混合 ADC、OLED 和字体显示，现已拆出 ADC/标定驱动。ADC channel、采样次数、参考电压和线性标定参数均通过 `fsr_adc_config_t` 配置。

风险：原始线性公式不是通用传感器模型，必须用实际机械结构标定。FSR 不适合做高精度重量传感器，适合作为压力趋势或阈值输入。

## RPLIDAR C1

来源：GitHub `rplidar` 分支与 Slamtec RPLIDAR C1 手册。

结论：原解析状态机可用，但原实现使用静态状态，不支持多个实例。现改为 `rplidar_c1_t` handle 内保存解析状态，并对设备信息、健康状态和扫描响应描述符做长度与类型校验。

风险：RPLIDAR C1 需要 5V 供电，UART 电平手册标注典型 3.5V；接 ESP32-S3 前确认电平安全和共地。

## VL53L1X / VL53L0X

来源：GitHub 分支名为 `vl53l1x`，代码也使用 ST VL53L1X ULD。

结论：本轮整理的是 `vl53l1x_tof`，不是 VL53L0X。VL53L0X 与 VL53L1X API/寄存器不同，不能直接共用。I2C 引脚、速率、8 位地址、timing budget 和 inter-measurement period 均通过 `vl53l1x_tof_config_t` 配置。

风险：ST ULD 平台函数是全局符号，本组件当前按单个 VL53L1X 实例适配。若后续需要多个 VL53L1X，需要扩展地址到设备 handle 的映射。

## IMU

来源：GitHub `IMU` 分支。

结论：代码体现为地址 `0x23` 的自定义 I2C 寄存器协议，可读加速度、角速度、磁场、四元数、欧拉角和气压。I2C 引脚、速率、地址和外部共享总线均通过 `imu_i2c_config_t` 配置。

风险：未给出 IMU 型号和官方协议文档。当前只能按已有代码整理，硬件测试时必须先确认版本寄存器可读，再信任其它数据。

## 最终接口审查

通过项：

- 六个驱动都以 ESP-IDF component 形式注册，头文件位于组件 include 根目录，可由主算法工程 `REQUIRES <component>` 后直接 `#include`。
- 关键硬件参数均不需要修改驱动源码即可配置：UART 端口、RX/TX GPIO、波特率、I2C SDA/SCL、I2C 速率、I2C 地址、ADC channel、采样次数、ToF 时序。
- 驱动输出已经转换为算法更容易消费的结构体：距离 `mm/cm`、FSR 电压和估算质量、RPLIDAR 角度/距离/质量、IMU 物理量数组、UWB PDOA/TWR 字段。
- 测试工程 `examples/sensor_hub` 的配置项集中在 `menuconfig -> Autobox sensor hub test`。

限制项：

- A02YYUW、BU UWB、FSR 当前是单实例驱动；若同一主程序需要多个同类设备，应扩展为 handle 风格。
- VL53L1X 使用 ST ULD 的全局平台函数，当前按单实例适配；多 VL53L1X 需要增加地址到 handle 的映射。
- FSR 质量换算不是可靠标定结果，只能作为当前电路的趋势值或占位估算。
