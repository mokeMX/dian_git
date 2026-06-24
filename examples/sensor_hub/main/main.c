#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "a02yyuw.h"
#include "bu_uwb.h"
#include "fsr_adc.h"
#include "imu_i2c.h"
#include "rplidar_c1.h"
#include "vl53l1x_tof.h"

#if CONFIG_SENSOR_HUB_IMU_ENABLE || CONFIG_SENSOR_HUB_VL53L1X_ENABLE
#include "driver/i2c_master.h"
#endif

#define HUB_I2C_PORT 0
#if CONFIG_SENSOR_HUB_IMU_ENABLE || CONFIG_SENSOR_HUB_VL53L1X_ENABLE
#define HUB_I2C_SDA_GPIO CONFIG_SENSOR_HUB_I2C_SDA_GPIO
#define HUB_I2C_SCL_GPIO CONFIG_SENSOR_HUB_I2C_SCL_GPIO
#define HUB_I2C_SPEED_HZ CONFIG_SENSOR_HUB_I2C_SPEED_HZ
#endif

static void print_status(const char *name, esp_err_t ret)
{
    printf("%s init: %s\n", name, ret == ESP_OK ? "OK" : "FAIL");
}

#if CONFIG_SENSOR_HUB_BU_UWB_ENABLE
static void print_hex(const uint8_t *data, int len)
{
    printf("[BU_UWB][HEX]");
    for (int i = 0; i < len; ++i) {
        printf(" %02X", data[i]);
    }
    printf("\n");
}

static void handle_bu_uwb_rx(const uint8_t *data, int len)
{
    static char line[BU_UWB_LINE_MAX];
    static size_t line_pos;

    printf("\n========== BU_UWB DATA RECEIVED ==========\n");
    printf("[BU_UWB] RX len=%d bytes\n", len);
    printf("[BU_UWB][RAW] %.*s\n", len, (const char *)data);
    print_hex(data, len);

    for (int i = 0; i < len; ++i) {
        const char ch = (char)data[i];
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            line[line_pos] = '\0';
            if (line_pos > 0) {
                const bu_uwb_line_type_t type = bu_uwb_classify_line(line);
                const char *payload = bu_uwb_line_payload(line);
                bu_uwb_distance_t distance = {0};
                bu_uwb_twr_reading_t twr = {0};

                printf("[BU_UWB][LINE] %s\n", line);
                if (type == BU_UWB_LINE_DATA) {
                    printf("[BU_UWB][DATA] %s\n", payload);
                } else if (type == BU_UWB_LINE_ERROR) {
                    printf("[BU_UWB][ERR] %s\n", payload);
                } else if (type == BU_UWB_LINE_TWR &&
                           bu_uwb_parse_twr_line(line, &twr) &&
                           twr.valid) {
                    printf("[BU_UWB][TWR] frame=%s anchor=%s D=%dcm Xcm=%d Ycm=%d R=%d P=%d T=%d V=%d O=%d xyz=%d,%d,%d\n",
                           twr.frame_id,
                           twr.anchor_id,
                           twr.distance_cm,
                           twr.x_cm,
                           twr.y_cm,
                           twr.r,
                           twr.p,
                           twr.timestamp,
                           twr.validity,
                           twr.orientation,
                           twr.x,
                           twr.y,
                           twr.z);
                }
                if (bu_uwb_parse_distance_line(line, &distance) && distance.valid) {
                    printf("[BU_UWB][DISTANCE] %d mm %.3f m\n",
                           distance.distance_mm,
                           distance.distance_m);
                }
            }
            line_pos = 0;
        } else if (line_pos + 1 < sizeof(line)) {
            line[line_pos++] = ch;
        } else {
            line_pos = 0;
            printf("[BU_UWB][WARN] line too long, dropped\n");
        }
    }
}
#endif

void app_main(void)
{
    printf("\nAutobox sensor hub test start\n");
#if CONFIG_SENSOR_HUB_IMU_ENABLE || CONFIG_SENSOR_HUB_VL53L1X_ENABLE
    printf("Default I2C: SDA=GPIO%d SCL=GPIO%d\n", HUB_I2C_SDA_GPIO, HUB_I2C_SCL_GPIO);
#endif

#if CONFIG_SENSOR_HUB_IMU_ENABLE || CONFIG_SENSOR_HUB_VL53L1X_ENABLE
    i2c_master_bus_handle_t shared_i2c = NULL;
    const i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = HUB_I2C_PORT,
        .sda_io_num = HUB_I2C_SDA_GPIO,
        .scl_io_num = HUB_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    print_status("shared_i2c",
                 i2c_new_master_bus(&i2c_bus_cfg, &shared_i2c));
#endif

#if CONFIG_SENSOR_HUB_A02YYUW_ENABLE
    static a02yyuw_t a02_1;
    a02yyuw_config_t a02_cfg = a02yyuw_default_config(
        (uart_port_t)CONFIG_SENSOR_HUB_A02YYUW_UART,
        CONFIG_SENSOR_HUB_A02YYUW_RX_GPIO,
        CONFIG_SENSOR_HUB_A02YYUW_TX_GPIO);
    a02_cfg.baudrate = CONFIG_SENSOR_HUB_A02YYUW_BAUDRATE;
#ifdef CONFIG_SENSOR_HUB_A02YYUW_USE_SW_UART
    a02_cfg.use_sw_uart = true;
#endif
    print_status("a02yyuw#1", a02yyuw_init_dev(&a02_1, &a02_cfg));
    printf("[A02YYUW#1] %s%d RX=GPIO%d baud=%d\n",
           a02_cfg.use_sw_uart ? "SW-UART" : "HW-UART",
           a02_cfg.use_sw_uart ? 0 : CONFIG_SENSOR_HUB_A02YYUW_UART,
           CONFIG_SENSOR_HUB_A02YYUW_RX_GPIO,
           a02_cfg.baudrate);
#endif

#if CONFIG_SENSOR_HUB_A02YYUW2_ENABLE
    static a02yyuw_t a02_2;
    a02yyuw_config_t a02b_cfg = a02yyuw_default_config(
        (uart_port_t)CONFIG_SENSOR_HUB_A02YYUW2_UART,
        CONFIG_SENSOR_HUB_A02YYUW2_RX_GPIO,
        CONFIG_SENSOR_HUB_A02YYUW2_TX_GPIO);
    a02b_cfg.baudrate = CONFIG_SENSOR_HUB_A02YYUW2_BAUDRATE;
#ifdef CONFIG_SENSOR_HUB_A02YYUW2_USE_SW_UART
    a02b_cfg.use_sw_uart = true;
#endif
    print_status("a02yyuw#2", a02yyuw_init_dev(&a02_2, &a02b_cfg));
    printf("[A02YYUW#2] %s%d RX=GPIO%d baud=%d\n",
           a02b_cfg.use_sw_uart ? "SW-UART" : "HW-UART",
           a02b_cfg.use_sw_uart ? 0 : CONFIG_SENSOR_HUB_A02YYUW2_UART,
           CONFIG_SENSOR_HUB_A02YYUW2_RX_GPIO,
           a02b_cfg.baudrate);
#endif

#if CONFIG_SENSOR_HUB_BU_UWB_ENABLE
    bu_uwb_config_t bu_cfg = bu_uwb_default_config(
        (uart_port_t)CONFIG_SENSOR_HUB_BU_UWB_UART,
        CONFIG_SENSOR_HUB_BU_UWB_RX_GPIO,
        CONFIG_SENSOR_HUB_BU_UWB_TX_GPIO);
    bu_cfg.baudrate = CONFIG_SENSOR_HUB_BU_UWB_BAUDRATE;
    print_status("bu_uwb", bu_uwb_init(&bu_cfg));
    printf("[BU_UWB] passive UART monitor mode: PA2/TX -> ESP32 RX GPIO%d, PA3/RX -> ESP32 TX GPIO%d optional, baud=%d\n",
           CONFIG_SENSOR_HUB_BU_UWB_RX_GPIO,
           CONFIG_SENSOR_HUB_BU_UWB_TX_GPIO,
           bu_cfg.baudrate);
#endif

#if CONFIG_SENSOR_HUB_FSR_ENABLE
    fsr_adc_config_t fsr_cfg = fsr_adc_default_config();
    fsr_cfg.adc_gpio = CONFIG_SENSOR_HUB_FSR_ADC_GPIO;
    fsr_cfg.adc_channel = (adc_channel_t)CONFIG_SENSOR_HUB_FSR_ADC_CHANNEL;
    print_status("fsr_adc", fsr_adc_init(&fsr_cfg));
#endif

#if CONFIG_SENSOR_HUB_RPLIDAR_ENABLE
    static rplidar_c1_t lidar;
    static bool lidar_scan_active;
    rplidar_c1_config_t lidar_cfg = rplidar_c1_default_config(
        (uart_port_t)CONFIG_SENSOR_HUB_RPLIDAR_UART,
        CONFIG_SENSOR_HUB_RPLIDAR_RX_GPIO,
        CONFIG_SENSOR_HUB_RPLIDAR_TX_GPIO);
    lidar_cfg.baudrate = CONFIG_SENSOR_HUB_RPLIDAR_BAUDRATE;
    esp_err_t lidar_ret = rplidar_c1_init(&lidar, &lidar_cfg);
    print_status("rplidar_c1", lidar_ret);
    printf("[RPLIDAR] UART%d baud=%d ESP_RX=GPIO%d ESP_TX=GPIO%d\n",
           CONFIG_SENSOR_HUB_RPLIDAR_UART,
           lidar_cfg.baudrate,
           CONFIG_SENSOR_HUB_RPLIDAR_RX_GPIO,
           CONFIG_SENSOR_HUB_RPLIDAR_TX_GPIO);
    if (lidar_ret == ESP_OK) {
        rplidar_c1_info_t info = {0};
        esp_err_t info_ret = rplidar_c1_get_info(&lidar, &info);
        if (info_ret == ESP_OK) {
            printf("[RPLIDAR][INFO] model=%u.%u firmware=%u.%u hardware=%u sn=%s\n",
                   info.major_model,
                   info.sub_model,
                   info.firmware_major,
                   info.firmware_minor,
                   info.hardware,
                   info.serial_num);
        } else {
            printf("[RPLIDAR][WAIT] get info failed; check 5V/GND/TX/RX, baud=%d\n",
                   lidar_cfg.baudrate);
        }

        uint8_t health_status = 0xFF;
        uint16_t health_error = 0;
        esp_err_t health_ret = rplidar_c1_get_health(&lidar, &health_status, &health_error);
        if (health_ret == ESP_OK) {
            printf("[RPLIDAR][HEALTH] status=%u error=0x%04X\n",
                   health_status,
                   health_error);
        } else {
            printf("[RPLIDAR][WAIT] get health failed; check power/current and UART direction\n");
        }

        esp_err_t scan_ret = rplidar_c1_start_scan(&lidar);
        print_status("rplidar_start_scan", scan_ret);
        lidar_scan_active = (scan_ret == ESP_OK);
    }
#endif

#if CONFIG_SENSOR_HUB_IMU_ENABLE
    static imu_i2c_t imu;
    imu_i2c_config_t imu_cfg = imu_i2c_default_config();
    imu_cfg.sda_gpio = HUB_I2C_SDA_GPIO;
    imu_cfg.scl_gpio = HUB_I2C_SCL_GPIO;
    imu_cfg.scl_speed_hz = HUB_I2C_SPEED_HZ;
    imu_cfg.device_address = CONFIG_SENSOR_HUB_IMU_ADDR;
    imu_cfg.external_bus = shared_i2c;
    esp_err_t imu_ret = imu_i2c_init(&imu, &imu_cfg);
    print_status("imu_i2c", imu_ret);
    if (imu_ret == ESP_OK) {
        uint8_t version[3] = {0};
        esp_err_t version_ret = imu_i2c_read_version(&imu, version);
        if (version_ret == ESP_OK) {
            printf("[IMU] version=%u.%u.%u addr=0x%02X SDA=GPIO%d SCL=GPIO%d\n",
                   version[0],
                   version[1],
                   version[2],
                   imu_cfg.device_address,
                   imu_cfg.sda_gpio,
                   imu_cfg.scl_gpio);
        } else {
            printf("[IMU][WAIT] version read failed; check VCC/GND/SDA/SCL and addr=0x%02X\n",
                   imu_cfg.device_address);
        }
    }
#endif

#if CONFIG_SENSOR_HUB_VL53L1X_ENABLE
    static vl53l1x_tof_t tof;
    vl53l1x_tof_config_t tof_cfg = vl53l1x_tof_default_config();
    tof_cfg.sda_gpio = HUB_I2C_SDA_GPIO;
    tof_cfg.scl_gpio = HUB_I2C_SCL_GPIO;
    tof_cfg.scl_speed_hz = HUB_I2C_SPEED_HZ;
    tof_cfg.device_address_8bit = CONFIG_SENSOR_HUB_VL53L1X_ADDR_8BIT;
    tof_cfg.timing_budget_ms = CONFIG_SENSOR_HUB_VL53L1X_TIMING_BUDGET_MS;
    tof_cfg.inter_measurement_ms = CONFIG_SENSOR_HUB_VL53L1X_INTER_MEASUREMENT_MS;
    tof_cfg.external_bus = shared_i2c;
    print_status("vl53l1x_tof", vl53l1x_tof_init(&tof, &tof_cfg));
#endif

    while (1) {
#if CONFIG_SENSOR_HUB_A02YYUW_ENABLE
        a02yyuw_reading_t a02 = {0};
        if (a02yyuw_read_dev(&a02_1, &a02, 150) == ESP_OK && a02.valid) {
            printf("[A02YYUW#1] distance=%d mm\n", a02.distance_mm);
        } else {
            printf("[A02YYUW#1] no valid frame (RX=GPIO%d)\n",
                   CONFIG_SENSOR_HUB_A02YYUW_RX_GPIO);
        }
#endif

#if CONFIG_SENSOR_HUB_A02YYUW2_ENABLE
        a02yyuw_reading_t a02b = {0};
        if (a02yyuw_read_dev(&a02_2, &a02b, 150) == ESP_OK && a02b.valid) {
            printf("[A02YYUW#2] distance=%d mm\n", a02b.distance_mm);
        } else {
            printf("[A02YYUW#2] no valid frame (RX=GPIO%d)\n",
                   CONFIG_SENSOR_HUB_A02YYUW2_RX_GPIO);
        }
#endif

#if CONFIG_SENSOR_HUB_BU_UWB_ENABLE
        static uint32_t bu_no_data_count;
        uint8_t bu_rx[128] = {0};
        int bu_len = 0;
        if (bu_uwb_read_bytes(bu_rx, sizeof(bu_rx), &bu_len, 100) == ESP_OK) {
            bu_no_data_count = 0;
            handle_bu_uwb_rx(bu_rx, bu_len);
        } else if (++bu_no_data_count >= 4) {
            bu_no_data_count = 0;
            printf("[BU_UWB][WAIT] no data yet; check PA2/TX -> ESP32 RX GPIO%d, common GND, baud=%d, kit output mode\n",
                   CONFIG_SENSOR_HUB_BU_UWB_RX_GPIO,
                   CONFIG_SENSOR_HUB_BU_UWB_BAUDRATE);
        }
#endif

#if CONFIG_SENSOR_HUB_FSR_ENABLE
        fsr_adc_reading_t fsr = {0};
        if (fsr_adc_read(&fsr) == ESP_OK && fsr.valid) {
            printf("[FSR] raw=%d voltage=%.3fV force_est_kg=%.2f\n",
                   fsr.raw,
                   fsr.voltage_v,
                   fsr.weight_kg);
        }
#endif

#if CONFIG_SENSOR_HUB_RPLIDAR_ENABLE
        static uint32_t lidar_no_point_count;
        rplidar_c1_point_t point = {0};
        bool got_point = false;

        if (lidar_scan_active) {
            for (int i = 0; i < 50; ++i) {
                if (rplidar_c1_read_point(&lidar, &point) && point.distance_mm > 0.0f) {
                    printf("[RPLIDAR] angle=%.2f distance=%.1f quality=%u start=%d\n",
                           point.angle_deg,
                           point.distance_mm,
                           point.quality,
                           point.start_bit);
                    got_point = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        } else if (++lidar_no_point_count >= 4) {
            lidar_no_point_count = 0;
            printf("[RPLIDAR][WAIT] scan did not start; check 5V power, rotation, UART direction and baud=%d\n",
                   lidar_cfg.baudrate);
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_err_t scan_ret = rplidar_c1_start_scan(&lidar);
            print_status("rplidar_start_scan_retry", scan_ret);
            lidar_scan_active = (scan_ret == ESP_OK);
        }

        if (got_point) {
            lidar_no_point_count = 0;
        } else if (lidar_scan_active && ++lidar_no_point_count >= 4) {
            lidar_no_point_count = 0;
            printf("[RPLIDAR][WAIT] no valid scan point; check rotation, 5V current, ESP_RX=GPIO%d <- lidar TX\n",
                   CONFIG_SENSOR_HUB_RPLIDAR_RX_GPIO);
        }
#endif

#if CONFIG_SENSOR_HUB_IMU_ENABLE
        imu_i2c_reading_t imu_data = {0};
        if (imu_i2c_read_all(&imu, &imu_data) == ESP_OK && imu_data.valid) {
            printf("[IMU] accel=%.3f %.3f %.3f g euler=%.2f %.2f %.2f deg\n",
                   imu_data.accel_g[0],
                   imu_data.accel_g[1],
                   imu_data.accel_g[2],
                   imu_data.euler_deg[0],
                   imu_data.euler_deg[1],
                   imu_data.euler_deg[2]);
        } else {
            printf("[IMU][WAIT] no valid data; check SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n",
                   HUB_I2C_SDA_GPIO,
                   HUB_I2C_SCL_GPIO,
                   IMU_I2C_DEFAULT_ADDR);
        }
#endif

#if CONFIG_SENSOR_HUB_VL53L1X_ENABLE
        vl53l1x_tof_reading_t tof_reading = {0};
        if (vl53l1x_tof_read(&tof, &tof_reading, 250) == ESP_OK &&
            tof_reading.valid) {
            printf("[VL53L1X] distance=%u mm\n", tof_reading.distance_mm);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
