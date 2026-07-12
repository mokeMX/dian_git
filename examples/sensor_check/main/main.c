#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "a02yyuw.h"
#include "board_pin_config.h"
#include "bu_uwb.h"
#include "chassis.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "fsr_adc.h"
#include "rplidar_c1.h"

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

static void check_uwb(void)
{
    bu_uwb_config_t config = bu_uwb_default_config(UWB_UART_PORT,
                                                   PIN_UWB_RX, PIN_UWB_TX);
    config.baudrate = UWB_BAUD_RATE;
    esp_err_t ret = bu_uwb_init(&config);
    if (ret != ESP_OK) {
        fail("UWB UART1 initialization", ret);
        return;
    }
    pass("UWB UART initialized: UART1, TX=47, RX=48, baud=115200");
    char line[BU_UWB_LINE_MAX];
    ret = bu_uwb_read_line(line, sizeof(line), 1200);
    if (ret != ESP_OK) {
        warn("UWB timeout; check crossed TX/RX wiring and module power");
        return;
    }
    bu_uwb_twr_reading_t twr = {0};
    bu_uwb_distance_t distance = {0};
    if (bu_uwb_parse_twr_line(line, &twr) && twr.valid) {
        char message[160];
        const float bearing = atan2f((float)twr.x_cm, (float)twr.y_cm) *
                              180.0f / 3.14159265358979323846f;
        snprintf(message, sizeof(message),
                 "UWB frame received: distance=%.2f m, bearing=%.1f deg",
                 (float)twr.distance_cm / 100.0f, bearing);
        pass(message);
    } else if (bu_uwb_parse_distance_line(line, &distance) && distance.valid) {
        char message[128];
        snprintf(message, sizeof(message),
                 "UWB range-only frame received: distance=%.2f m; bearing unavailable",
                 distance.distance_m);
        warn(message);
    } else {
        warn("UWB data arrived but frame validation failed");
    }
}

static void check_lidar(void)
{
    static rplidar_c1_t lidar;
    rplidar_c1_config_t config = rplidar_c1_default_config(
        RPLIDAR_UART_PORT, PIN_RPLIDAR_RX, PIN_RPLIDAR_TX);
    config.baudrate = RPLIDAR_BAUD_RATE;
    esp_err_t ret = rplidar_c1_init(&lidar, &config);
    if (ret != ESP_OK) {
        fail("RPLIDAR UART2 initialization", ret);
        return;
    }
    pass("RPLIDAR UART initialized: UART2, TX=17, RX=18, baud=460800");
    rplidar_c1_stop(&lidar);
    ret = rplidar_c1_start_scan(&lidar);
    if (ret != ESP_OK) {
        fail("RPLIDAR start scan", ret);
        return;
    }
    const int64_t deadline = esp_timer_get_time() + 1500000;
    unsigned valid_points = 0;
    unsigned invalid_points = 0;
    while (esp_timer_get_time() < deadline) {
        rplidar_c1_point_t point = {0};
        if (rplidar_c1_read_point(&lidar, &point)) {
            if (point.distance_mm > 0.0f && point.angle_deg >= 0.0f &&
                point.angle_deg < 360.0f) {
                ++valid_points;
            } else {
                ++invalid_points;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    if (valid_points > 0) {
        char message[128];
        snprintf(message, sizeof(message),
                 "RPLIDAR scan data received: valid=%u invalid=%u",
                 valid_points, invalid_points);
        pass(message);
    } else {
        warn("RPLIDAR scan timeout; check 5V supply and crossed TX/RX wiring");
    }
    rplidar_c1_stop(&lidar);
}

static void check_ultrasonic(const char *name, gpio_num_t pin)
{
    a02yyuw_t device;
    memset(&device, 0, sizeof(device));
    a02yyuw_config_t config = a02yyuw_default_config(UART_NUM_0, pin, -1);
    config.use_sw_uart = true;
    config.baudrate = ULTRASONIC_BAUD_RATE;
    esp_err_t ret = a02yyuw_init_dev(&device, &config);
    if (ret != ESP_OK) {
        char message[96];
        snprintf(message, sizeof(message), "Ultrasonic %s GPIO%d initialization",
                 name, pin);
        fail(message, ret);
        return;
    }
    a02yyuw_reading_t reading = {0};
    ret = a02yyuw_read_dev(&device, &reading, 500);
    if (ret == ESP_OK && reading.valid) {
        char message[96];
        snprintf(message, sizeof(message),
                 "Ultrasonic %s: GPIO%d, distance=%.2f m", name, pin,
                 (float)reading.distance_mm / 1000.0f);
        pass(message);
    } else {
        char message[128];
        snprintf(message, sizeof(message),
                 "Ultrasonic %s GPIO%d timeout/frame error; check A02YYUW 9600 8N1 RX",
                 name, pin);
        warn(message);
    }
    a02yyuw_deinit_dev(&device);
}

static void check_fsr(void)
{
    fsr_adc_config_t config = fsr_adc_default_config();
    esp_err_t ret = fsr_adc_init(&config);
    if (ret != ESP_OK) {
        fail("FSR ADC1_CH7 initialization", ret);
        return;
    }
    pass("FSR ADC initialized: GPIO8, ADC1_CH7");
    fsr_adc_reading_t reading = {0};
    ret = fsr_adc_read(&reading);
    if (ret != ESP_OK || !reading.valid) {
        fail("FSR ADC sample", ret == ESP_OK ? ESP_FAIL : ret);
    } else {
        char message[128];
        snprintf(message, sizeof(message), "FSR raw=%d, voltage=%.0f mV",
                 reading.raw, reading.voltage_v * 1000.0f);
        pass(message);
        if (reading.raw < 10) {
            warn("FSR reading is close to zero; check sensor connection");
        } else if (reading.raw > 4085) {
            warn("FSR reading is saturated; check divider and ADC attenuation");
        }
    }
    fsr_adc_deinit();
}

static void check_chassis(void)
{
    static chassis_t chassis;
    chassis_config_t config = chassis_default_config();
    esp_err_t ret = chassis_init(&chassis, &config);
    if (ret != ESP_OK) {
        fail("ESC/encoder initialization", ret);
        return;
    }
    ret = chassis_emergency_stop(&chassis);
    if (ret != ESP_OK) {
        fail("ESC safe neutral output", ret);
        return;
    }
    pass("Left ESC PWM initialized: GPIO4, 50Hz, neutral=1500us");
    pass("Right ESC PWM initialized: GPIO5, 50Hz, neutral=1500us");
    pass("Encoder LEFT initialized: A=6, B=7");
    pass("Encoder RIGHT initialized: A=15, B=16");
    const int64_t left_start = chassis.enc_l.count;
    const int64_t right_start = chassis.enc_r.count;
    vTaskDelay(pdMS_TO_TICKS(1000));
    const int64_t left_delta = chassis.enc_l.count - left_start;
    const int64_t right_delta = chassis.enc_r.count - right_start;
    if (left_delta == 0 && right_delta == 0) {
        warn("Encoder static check saw no pulses; rotate each wheel manually to verify direction");
    } else {
        char message[128];
        snprintf(message, sizeof(message), "Encoder pulse check: left=%lld right=%lld",
                 (long long)left_delta, (long long)right_delta);
        pass(message);
    }
    chassis_deinit(&chassis);
}

void app_main(void)
{
    printf("\n================ SENSOR SELF TEST ================\n\n");
    check_chassis();
    check_fsr();
    check_uwb();
    check_lidar();
    check_ultrasonic("LEFT", PIN_ULTRASONIC_LEFT_RX);
    check_ultrasonic("RIGHT", PIN_ULTRASONIC_RIGHT_RX);
    printf("\nSummary: %u passed, %u warning, %u failed\n",
           s_summary.passed, s_summary.warnings, s_summary.failed);
    printf("==================================================\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
