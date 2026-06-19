# 驱动接入与 API 使用

本文说明如何在未来主算法 ESP-IDF 工程中引入 `components/sensors` 下的传感器驱动。驱动层目标是只做硬件初始化、数据传输、协议解码和基础单位换算；滤波、融合、地图、控制等算法逻辑应放在上层。

## 组件引入

方式一：把需要的目录复制到主工程 `components/` 下：

```text
your_app/
  components/
    a02yyuw/
    bu_uwb/
    fsr_adc/
    rplidar_c1/
    imu_i2c/
    vl53l1x_tof/
```

方式二：在主工程 `CMakeLists.txt` 中指定外部组件路径：

```cmake
set(EXTRA_COMPONENT_DIRS
    "/home/sp/auto-box/sensor-driver/components/sensors/a02yyuw"
    "/home/sp/auto-box/sensor-driver/components/sensors/bu_uwb"
    "/home/sp/auto-box/sensor-driver/components/sensors/fsr_adc"
    "/home/sp/auto-box/sensor-driver/components/sensors/rplidar_c1"
    "/home/sp/auto-box/sensor-driver/components/sensors/imu_i2c"
    "/home/sp/auto-box/sensor-driver/components/sensors/vl53l1x_tof"
)
```

主程序组件按需声明依赖：

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES a02yyuw bu_uwb fsr_adc rplidar_c1 imu_i2c vl53l1x_tof
)
```

## 统一配置原则

- 不在驱动源码中修改引脚、波特率、I2C 地址或 ADC 通道。
- 调用 `*_default_config()` 得到默认配置，再在应用层覆盖需要修改的字段。
- UART 传感器配置 `uart_port`、`rx_gpio`、`tx_gpio`、`baudrate`、`rx_buffer_size`。
- I2C 传感器配置 `i2c_port`、`sda_gpio`、`scl_gpio`、`scl_speed_hz`、`device_address`。
- 如果多个 I2C 设备共用总线，应用层创建 `i2c_master_bus_handle_t` 后传给各驱动的 `external_bus`。

## A02YYUW

```c
#include "a02yyuw.h"

a02yyuw_config_t cfg = a02yyuw_default_config(UART_NUM_1, 36, 37);
cfg.baudrate = 9600;
ESP_ERROR_CHECK(a02yyuw_init(&cfg));

a02yyuw_reading_t reading = {0};
if (a02yyuw_read(&reading, 150) == ESP_OK && reading.valid) {
    printf("distance=%d mm\n", reading.distance_mm);
}
```

输出单位为 `mm`。驱动同时暴露 `a02yyuw_parse_frame()` 和 `a02yyuw_parse_latest()`，可用于离线协议测试。

## BU03/BU04 UWB

```c
#include "bu_uwb.h"

bu_uwb_config_t cfg = bu_uwb_default_config(UART_NUM_1, 36, 37);
cfg.baudrate = 115200;
ESP_ERROR_CHECK(bu_uwb_init(&cfg));

char line[BU_UWB_LINE_MAX] = {0};
if (bu_uwb_read_line(line, sizeof(line), 1000) == ESP_OK) {
    bu_uwb_twr_reading_t twr = {0};
    if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
        printf("D=%d cm X=%d cm Y=%d cm\n",
               twr.distance_cm, twr.x_cm, twr.y_cm);
    }
}
```

当前实测 BU04 基站输出 `JSxxxx{"TWR": {...}}`，驱动解析 `D`、`Xcm`、`Ycm`、`R`、`P`、`V` 等字段。`bu_uwb_send_command()` 保留给需要 AT 命令交互的固件。

## FSR ADC

```c
#include "fsr_adc.h"

fsr_adc_config_t cfg = fsr_adc_default_config();
cfg.adc_gpio = 8;
cfg.adc_channel = ADC_CHANNEL_7;
cfg.sample_count = 16;
ESP_ERROR_CHECK(fsr_adc_init(&cfg));

fsr_adc_reading_t reading = {0};
if (fsr_adc_read(&reading) == ESP_OK && reading.valid) {
    printf("raw=%d voltage=%.3f force_est_kg=%.2f\n",
           reading.raw, reading.voltage_v, reading.weight_kg);
}
```

`weight_kg` 只表示按当前线性公式估算的质量。FSR 必须按实物结构重新标定后才适合做实时重量显示。

## RPLIDAR C1

```c
#include "rplidar_c1.h"

static rplidar_c1_t lidar;
rplidar_c1_config_t cfg = rplidar_c1_default_config(UART_NUM_2, 36, 37);
cfg.baudrate = 460800;
ESP_ERROR_CHECK(rplidar_c1_init(&lidar, &cfg));

rplidar_c1_info_t info = {0};
rplidar_c1_get_info(&lidar, &info);
ESP_ERROR_CHECK(rplidar_c1_start_scan(&lidar));

rplidar_c1_point_t point = {0};
if (rplidar_c1_read_point(&lidar, &point) && point.distance_mm > 0.0f) {
    printf("angle=%.2f distance=%.1f quality=%u\n",
           point.angle_deg, point.distance_mm, point.quality);
}
```

雷达必须 5V 稳定供电并开始旋转后才会持续输出扫描点。

## IMU

```c
#include "imu_i2c.h"

static imu_i2c_t imu;
imu_i2c_config_t cfg = imu_i2c_default_config();
cfg.sda_gpio = 38;
cfg.scl_gpio = 37;
cfg.device_address = 0x23;
ESP_ERROR_CHECK(imu_i2c_init(&imu, &cfg));

imu_i2c_reading_t reading = {0};
if (imu_i2c_read_all(&imu, &reading) == ESP_OK && reading.valid) {
    printf("accel=%.3f %.3f %.3f g euler=%.2f %.2f %.2f deg\n",
           reading.accel_g[0], reading.accel_g[1], reading.accel_g[2],
           reading.euler_deg[0], reading.euler_deg[1], reading.euler_deg[2]);
}
```

输出包含加速度、角速度、磁场、四元数、欧拉角和气压数组。

## VL53L1X ToF

```c
#include "vl53l1x_tof.h"

static vl53l1x_tof_t tof;
vl53l1x_tof_config_t cfg = vl53l1x_tof_default_config();
cfg.sda_gpio = 39;
cfg.scl_gpio = 38;
cfg.device_address_8bit = 0x52;
cfg.timing_budget_ms = 50;
cfg.inter_measurement_ms = 55;
ESP_ERROR_CHECK(vl53l1x_tof_init(&tof, &cfg));

vl53l1x_tof_reading_t reading = {0};
if (vl53l1x_tof_read(&tof, &reading, 250) == ESP_OK && reading.valid) {
    printf("distance=%u mm\n", reading.distance_mm);
}
```

本组件基于 ST VL53L1X ULD。平台函数是 ST API 要求的全局符号，当前按单个 VL53L1X 实例适配。

## 接口审查结论

六个模块均可在不修改驱动源码的情况下配置关键硬件参数。A02YYUW、BU UWB、FSR 为单实例驱动；RPLIDAR、IMU、VL53L1X 使用显式 handle。若后续需要同类多实例，优先扩展 A02YYUW、BU UWB、FSR 的接口为 handle 风格。
