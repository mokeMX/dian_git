// #include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <float.h>
#include "freertos/task.h"

// #include "a02yyuw.h"
#include "board_pin_config.h"
// #include "bu_uwb.h"
// #include "chassis.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "fsr_adc.h"
// #include "rplidar_c1.h"


#define FSR_ADC_GPIO       8
#define FSR_ADC_CHANNEL    ADC_CHANNEL_7
typedef struct {
    unsigned passed;
    unsigned warnings;
    unsigned failed;
} summary_t;

static summary_t s_summary;

static void pass(const char *message)
{
    ++s_summary.passed;
    printf("[PASS] %s\n", message);
}

static void warn(const char *message)
{
    ++s_summary.warnings;
    printf("[WARN] %s\n", message);
}

static void fail(const char *message, esp_err_t error)
{
    ++s_summary.failed;
    printf("[FAIL] %s: %s\n", message, esp_err_to_name(error));
}

// static void check_uwb(void)
// {
//     bu_uwb_config_t config = bu_uwb_default_config(UWB_UART_PORT,
//                                                    PIN_UWB_RX, PIN_UWB_TX);
//     config.baudrate = UWB_BAUD_RATE;
//     esp_err_t ret = bu_uwb_init(&config);
//     if (ret != ESP_OK) {
//         fail("UWB UART1 initialization", ret);
//         return;
//     }
//     pass("UWB UART initialized: UART1, TX=47, RX=48, baud=115200");
//     char line[BU_UWB_LINE_MAX];
//     ret = bu_uwb_read_line(line, sizeof(line), 1200);
//     if (ret != ESP_OK) {
//         warn("UWB timeout; check crossed TX/RX wiring and module power");
//         return;
//     }
//     bu_uwb_twr_reading_t twr = {0};
//     bu_uwb_distance_t distance = {0};
//     if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
//         char message[160];
//         const float bearing = atan2f((float)twr.x_cm, (float)twr.y_cm) *
//                               180.0f / 3.14159265358979323846f;
//         snprintf(message, sizeof(message),
//                  "UWB frame received: distance=%.2f m, bearing=%.1f deg",
//                  (float)twr.distance_cm / 100.0f, bearing);
//         pass(message);
//     } else if (bu_uwb_parse_distance_line(line, &distance) && distance.valid) {
//         char message[128];
//         snprintf(message, sizeof(message),
//                  "UWB range-only frame received: distance=%.2f m; bearing unavailable",
//                  distance.distance_m);
//         warn(message);
//     } else {
//         warn("UWB data arrived but frame validation failed");
//     }
// }

#if 0  // ── LiDAR ──
static void check_lidar(void)
{
    static rplidar_c1_t lidar;
    char message[192];

    /*
     * rplidar_c1_default_config() 的参数顺序：
     * UART 端口、RX GPIO、TX GPIO
     *
     * 实际接线：
     * ESP32 GPIO17 TX -> RPLIDAR RX
     * ESP32 GPIO18 RX <- RPLIDAR TX
     */
    rplidar_c1_config_t config = rplidar_c1_default_config(
        RPLIDAR_UART_PORT,
        PIN_RPLIDAR_RX,
        PIN_RPLIDAR_TX);

    config.baudrate = RPLIDAR_BAUD_RATE;

    /*
     * 雷达数据量较大，建议 RX 缓冲区至少为 4096 字节。
     * 如果宏定义中的默认值已经大于等于 4096，也可以删除这一行。
     */
    if (config.rx_buffer_size < 4096) {
        config.rx_buffer_size = 4096;
    }

    /* 1. 初始化 UART 和 RPLIDAR 驱动 */
    esp_err_t ret = rplidar_c1_init(&lidar, &config);
    if (ret != ESP_OK) {
        fail("RPLIDAR UART2 initialization", ret);
        return;
    }

    snprintf(message,
             sizeof(message),
             "RPLIDAR UART initialized: UART%d, TX=%d, RX=%d, baud=%d",
             (int)RPLIDAR_UART_PORT,
             PIN_RPLIDAR_TX,
             PIN_RPLIDAR_RX,
             config.baudrate);
    pass(message);

    /*
     * 2. 停止原有扫描并软复位。
     *
     * rplidar_c1_reset() 内部已经包含约 600 ms 延时，
     * 并会清空 UART 接收缓冲区、复位解析状态。
     */
    rplidar_c1_stop(&lidar);
    rplidar_c1_reset(&lidar);

    pass("RPLIDAR stop and reset command completed");

    /* 额外等待雷达内部系统和电机控制部分稳定 */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 3. 读取健康状态 */
    uint8_t health_status = 0;
    uint16_t error_code = 0;

    ret = rplidar_c1_get_health(
        &lidar,
        &health_status,
        &error_code);

    if (ret != ESP_OK) {
        fail("RPLIDAR get health status", ret);
        rplidar_c1_stop(&lidar);
        rplidar_c1_deinit(&lidar);
        return;
    }

    snprintf(message,
             sizeof(message),
             "RPLIDAR health: status=%u, error_code=0x%04X",
             (unsigned)health_status,
             (unsigned)error_code);

    switch (health_status) {
    case 0:
        pass(message);
        break;

    case 1:
        warn(message);
        break;

    case 2:
        fail(message, ESP_FAIL);
        rplidar_c1_stop(&lidar);
        rplidar_c1_deinit(&lidar);
        return;

    default:
        warn("RPLIDAR returned an unknown health status");
        break;
    }

    /* 4. 读取设备信息 */
    rplidar_c1_info_t info = {0};

    ret = rplidar_c1_get_info(&lidar, &info);
    if (ret == ESP_OK) {
        snprintf(message,
                 sizeof(message),
                 "RPLIDAR device: model=%u.%u, firmware=%u.%u, "
                 "hardware=%u, SN=%s",
                 (unsigned)info.major_model,
                 (unsigned)info.sub_model,
                 (unsigned)info.firmware_major,
                 (unsigned)info.firmware_minor,
                 (unsigned)info.hardware,
                 info.serial_num);
        pass(message);
    } else {
        /*
         * 获取设备信息失败不一定代表扫描一定失败，
         * 因此这里只警告，继续尝试启动扫描。
         */
        snprintf(message,
                 sizeof(message),
                 "RPLIDAR device information unavailable, error=%s",
                 esp_err_to_name(ret));
        warn(message);
    }

    /* 5. 启动标准扫描模式 */
    ret = rplidar_c1_start_scan(&lidar);
    if (ret != ESP_OK) {
        fail("RPLIDAR start scan", ret);
        rplidar_c1_stop(&lidar);
        rplidar_c1_deinit(&lidar);
        return;
    }

    pass("RPLIDAR standard scan mode started");

    /*
     * 6. 在规定时间内持续读取扫描点。
     *
     * 建议至少测试 2～3 秒，因为雷达电机启动以及第一圈数据
     * 可能需要一定时间。
     */
    const int64_t test_duration_us = 3000000;
    const int64_t deadline =
        esp_timer_get_time() + test_duration_us;

    unsigned valid_points = 0;
    unsigned invalid_points = 0;
    unsigned new_scan_count = 0;

    uint8_t max_quality = 0;
    float minimum_distance_mm = 0.0f;
    float maximum_distance_mm = 0.0f;
    bool has_distance = false;

    while (esp_timer_get_time() < deadline) {
        rplidar_c1_point_t point = {0};

        if (!rplidar_c1_read_point(&lidar, &point)) {
            /*
             * 当前 UART 缓冲区还没有完整的 5 字节节点。
             * 让出 CPU，避免任务持续空转触发看门狗。
             */
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /*
         * 基本合法性判断：
         * 1. 角度必须处于 [0, 360)；
         * 2. 距离必须大于 0；
         * 3. 距离设置一个合理上限，避免错位数据被当成有效点；
         * 4. 质量必须大于 0。
         *
         * 12000 mm 可根据具体 C1 型号参数调整。
         */
        const bool angle_valid =
            point.angle_deg >= 0.0f &&
            point.angle_deg < 360.0f;

        const bool distance_valid =
            point.distance_mm > 0.0f &&
            point.distance_mm <= 12000.0f;

        const bool quality_valid =
            point.quality > 0;

        if (angle_valid &&
            distance_valid &&
            quality_valid) {

            ++valid_points;

            if (point.start_bit) {
                ++new_scan_count;
            }

            if (point.quality > max_quality) {
                max_quality = point.quality;
            }

            if (!has_distance) {
                minimum_distance_mm = point.distance_mm;
                maximum_distance_mm = point.distance_mm;
                has_distance = true;
            } else {
                if (point.distance_mm < minimum_distance_mm) {
                    minimum_distance_mm = point.distance_mm;
                }

                if (point.distance_mm > maximum_distance_mm) {
                    maximum_distance_mm = point.distance_mm;
                }
            }

            /*
             * 不建议打印每一个扫描点。
             * 460800 波特率下点数较多，逐点打印可能阻塞串口，
             * 导致雷达 UART 缓冲区溢出。
             *
             * 这里只间隔打印少量样本。
             */
            if ((valid_points % 500U) == 1U) {
                ESP_LOGI(
                    "RPLIDAR_CHECK",
                    "Sample: angle=%6.2f deg, distance=%7.2f mm, "
                    "quality=%u, new_scan=%u",
                    point.angle_deg,
                    point.distance_mm,
                    (unsigned)point.quality,
                    point.start_bit ? 1U : 0U);
            }
        } else {
            ++invalid_points;
        }
    }

    /* 7. 停止扫描 */
    rplidar_c1_stop(&lidar);

    /* 8. 根据接收到的数据给出检测结果 */
    if (valid_points > 0) {
        snprintf(message,
                 sizeof(message),
                 "RPLIDAR scan data received: valid=%u, invalid=%u, "
                 "new_scans=%u, max_quality=%u, "
                 "distance_range=%.1f~%.1f mm",
                 valid_points,
                 invalid_points,
                 new_scan_count,
                 (unsigned)max_quality,
                 minimum_distance_mm,
                 maximum_distance_mm);

        pass(message);

        /*
         * 正常旋转情况下，3 秒内通常应该出现至少一个新一圈标志。
         * 有测距点但没有 start_bit，说明数据解析、丢包或同步可能有问题。
         */
        if (new_scan_count == 0) {
            warn("RPLIDAR data received, but no complete scan boundary detected");
        }
    } else {
        warn(
            "RPLIDAR scan timeout: no valid point received; "
            "check 5V power, common GND, motor rotation, "
            "and crossed TX/RX wiring");
    }

    /* 9. 释放 UART 驱动资源 */
    rplidar_c1_deinit(&lidar);
}
#endif  // ── LiDAR ──

// static void check_ultrasonic(const char *name, gpio_num_t pin)
// {
//     a02yyuw_t device;
//     memset(&device, 0, sizeof(device));
//     a02yyuw_config_t config = a02yyuw_default_config(UART_NUM_0, pin, -1);
//     config.use_sw_uart = true;
//     config.baudrate = ULTRASONIC_BAUD_RATE;
//     esp_err_t ret = a02yyuw_init_dev(&device, &config);
//     if (ret != ESP_OK) {
//         char message[96];
//         snprintf(message, sizeof(message), "Ultrasonic %s GPIO%d initialization",
//                  name, pin);
//         fail(message, ret);
//         return;
//     }
//     a02yyuw_reading_t reading = {0};
//     ret = a02yyuw_read_dev(&device, &reading, 500);
//     if (ret == ESP_OK && reading.valid) {
//         char message[96];
//         snprintf(message, sizeof(message),
//                  "Ultrasonic %s: GPIO%d, distance=%.2f m", name, pin,
//                  (float)reading.distance_mm / 1000.0f);
//         pass(message);
//     } else {
//         char message[128];
//         snprintf(message, sizeof(message),
//                  "Ultrasonic %s GPIO%d timeout/frame error; check A02YYUW 9600 8N1 RX",
//                  name, pin);
//         warn(message);
//     }
//     a02yyuw_deinit_dev(&device);
// }

static void check_fsr(void)
{
    fsr_adc_config_t config = fsr_adc_default_config();
    config.adc_gpio = FSR_ADC_GPIO;
    config.adc_channel = (adc_channel_t)FSR_ADC_CHANNEL;

    esp_err_t ret = fsr_adc_init(&config);
    if (ret != ESP_OK) {
        fail("FSR initialization", ret);
        return;
    }

    pass("FSR initialized: GPIO8, ADC1_CH7");

    while (1) {
        fsr_adc_reading_t reading = {0};

        ret = fsr_adc_read(&reading);

        if (ret == ESP_OK && reading.valid) {
            printf("[FSR] raw=%d voltage=%.3fV force_est_kg=%.2f\n",
                   reading.raw,
                   reading.voltage_v,
                   reading.weight_kg);
        } else {
            printf("[FSR] read failed: %s\n", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
// static void check_chassis(void)
// {
//     static chassis_t chassis;
//     chassis_config_t config = chassis_default_config();
//     esp_err_t ret = chassis_init(&chassis, &config);
//     if (ret != ESP_OK) {
//         fail("ESC/encoder initialization", ret);
//         return;
//     }
//     ret = chassis_emergency_stop(&chassis);
//     if (ret != ESP_OK) {
//         fail("ESC safe neutral output", ret);
//         return;
//     }
//     pass("Left ESC PWM initialized: GPIO4, 50Hz, neutral=1500us");
//     pass("Right ESC PWM initialized: GPIO5, 50Hz, neutral=1500us");
//     pass("Encoder LEFT initialized: A=6, B=7");
//     pass("Encoder RIGHT initialized: A=15, B=16");
//     const int64_t left_start = chassis.enc_l.count;
//     const int64_t right_start = chassis.enc_r.count;
//     vTaskDelay(pdMS_TO_TICKS(1000));
//     const int64_t left_delta = chassis.enc_l.count - left_start;
//     const int64_t right_delta = chassis.enc_r.count - right_start;
//     if (left_delta == 0 && right_delta == 0) {
//         warn("Encoder static check saw no pulses; rotate each wheel manually to verify direction");
//     } else {
//         char message[128];
//         snprintf(message, sizeof(message), "Encoder pulse check: left=%lld right=%lld",
//                  (long long)left_delta, (long long)right_delta);
//         pass(message);
//     }
//     chassis_deinit(&chassis);
// }

void app_main(void)
{
    printf("\n================ SENSOR SELF TEST ================\n\n");
    // check_chassis();
    check_fsr();
    // check_uwb();
    // check_lidar();
    // check_ultrasonic("LEFT", PIN_ULTRASONIC_LEFT_RX);
    // check_ultrasonic("RIGHT", PIN_ULTRASONIC_RIGHT_RX);
    printf("\nSummary: %u passed, %u warning, %u failed\n",
           s_summary.passed, s_summary.warnings, s_summary.failed);
    printf("==================================================\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
