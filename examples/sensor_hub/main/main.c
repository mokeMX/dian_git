#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#include "a02yyuw.h"
#include "bu_uwb.h"
#include "fsr_adc.h"
#include "imu_i2c.h"
#include "rplidar_c1.h"
#include "vl53l1x_tof.h"

/* ================================================================
 * Pin / hardware constants  (unchanged from original defaults)
 *
 * Key resolution: A02YYUW #1 moved from HW UART1 → SW UART (same
 * IO35 pin) to free UART1 for BU UWB.  All other pins identical to
 * 传感器修改4.
 * ================================================================ */
#define HUB_I2C_PORT           0
#define HUB_I2C_SDA_GPIO      11
#define HUB_I2C_SCL_GPIO      12
#define HUB_I2C_SPEED_HZ      400000

#define A02_1_RX_GPIO         35
#define A02_1_TX_GPIO         (-1)
#define A02_1_UART_PORT        1
#define A02_1_BAUDRATE        9600
#define A02_1_USE_SW_UART      1

#define A02_2_RX_GPIO         36
#define A02_2_TX_GPIO         (-1)
#define A02_2_UART_PORT        2
#define A02_2_BAUDRATE        9600
#define A02_2_USE_SW_UART      1

#define BU_UWB_UART_PORT       1
#define BU_UWB_RX_GPIO         6
#define BU_UWB_TX_GPIO         7
#define BU_UWB_BAUDRATE        115200

#define FSR_ADC_GPIO           8
#define FSR_ADC_CHANNEL        7

#define RPLIDAR_UART_PORT      2
#define RPLIDAR_RX_GPIO       17
#define RPLIDAR_TX_GPIO       18
#define RPLIDAR_BAUDRATE       460800

#define IMU_I2C_ADDR           0x23

#define VL53L1X_ADDR_8BIT             0x52
#define VL53L1X_TIMING_BUDGET_MS      50
#define VL53L1X_INTER_MEASUREMENT_MS  55

/* ---- shared resources ------------------------------------------*/
static i2c_master_bus_handle_t g_shared_i2c;

static a02yyuw_t g_a02_1;
static a02yyuw_t g_a02_2;
static rplidar_c1_t g_lidar;
static imu_i2c_t g_imu;
static vl53l1x_tof_t g_tof;
static volatile bool g_lidar_scan_active;

/* ---- helpers ---------------------------------------------------*/
static void print_status(const char *name, esp_err_t ret)
{
    printf("%s init: %s\n", name,
           (ret == ESP_OK) ? "OK" : "FAIL");
}

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
        switch (ch) {
        case '\r':
            continue;
        case '\n':
            line[line_pos] = '\0';
            {
                const bu_uwb_line_type_t type =
                    (line_pos > 0) ? bu_uwb_classify_line(line)
                                   : BU_UWB_LINE_UNKNOWN;
                const char *payload = bu_uwb_line_payload(line);
                bu_uwb_distance_t distance = {0};
                bu_uwb_twr_reading_t twr = {0};

                printf("[BU_UWB][LINE] %s\n", line);

                switch (type) {
                case BU_UWB_LINE_DATA:
                    printf("[BU_UWB][DATA] %s\n", payload);
                    break;
                case BU_UWB_LINE_ERROR:
                    printf("[BU_UWB][ERR] %s\n", payload);
                    break;
                case BU_UWB_LINE_TWR:
                    bu_uwb_parse_twr_line(line, &twr) &&
                        twr.valid &&
                        printf("[BU_UWB][TWR] frame=%s anchor=%s D=%dcm "
                               "Xcm=%d Ycm=%d R=%d P=%d T=%d V=%d O=%d "
                               "xyz=%d,%d,%d\n",
                               twr.frame_id, twr.anchor_id,
                               twr.distance_cm, twr.x_cm, twr.y_cm,
                               twr.r, twr.p, twr.timestamp,
                               twr.validity, twr.orientation,
                               twr.x, twr.y, twr.z);
                    break;
                default:
                    break;
                }

                bu_uwb_parse_distance_line(line, &distance) &&
                    distance.valid &&
                    printf("[BU_UWB][DISTANCE] %d mm %.3f m\n",
                           distance.distance_mm, distance.distance_m);
            }
            line_pos = 0;
            break;
        default:
            (line_pos + 1 < sizeof(line))
                ? (line[line_pos++] = ch)
                : ((line_pos = 0),
                   printf("[BU_UWB][WARN] line too long, "
                          "dropped\n"));
            break;
        }
    }
}

/* ================================================================
 * Sensor tasks  — each runs on its own FreeRTOS task so all
 * sensors output data simultaneously without blocking each other.
 * ================================================================ */

/* ---- A02YYUW #1 (IO35, SW-UART) --------------------------------*/
static void task_a02yyuw1(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        a02yyuw_reading_t r = {0};
        esp_err_t ret = a02yyuw_read_dev(&g_a02_1, &r, 150);
        switch (ret) {
        case ESP_OK:
            r.valid
                ? printf("[A02YYUW#1] distance=%d mm\n", r.distance_mm)
                : printf("[A02YYUW#1] no valid frame "
                         "(RX=GPIO%d)\n",
                         A02_1_RX_GPIO);
            break;
        default:
            printf("[A02YYUW#1] no valid frame (RX=GPIO%d)\n",
                   A02_1_RX_GPIO);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---- A02YYUW #2 (IO36, SW-UART) --------------------------------*/
static void task_a02yyuw2(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        a02yyuw_reading_t r = {0};
        esp_err_t ret = a02yyuw_read_dev(&g_a02_2, &r, 150);
        switch (ret) {
        case ESP_OK:
            r.valid
                ? printf("[A02YYUW#2] distance=%d mm\n", r.distance_mm)
                : printf("[A02YYUW#2] no valid frame "
                         "(RX=GPIO%d)\n",
                         A02_2_RX_GPIO);
            break;
        default:
            printf("[A02YYUW#2] no valid frame (RX=GPIO%d)\n",
                   A02_2_RX_GPIO);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---- BU UWB (UART1, GPIO6/7) -----------------------------------*/
static void task_bu_uwb(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t no_data_count;
    while (1) {
        uint8_t rx[128] = {0};
        int rx_len = 0;
        esp_err_t ret = bu_uwb_read_bytes(rx, sizeof(rx),
                                           &rx_len, 100);
        switch (ret) {
        case ESP_OK:
            no_data_count = 0;
            handle_bu_uwb_rx(rx, rx_len);
            break;
        default:
            (++no_data_count >= 4) &&
                (no_data_count = 0,
                 printf("[BU_UWB][WAIT] no data yet; check "
                        "PA2/TX -> ESP32 RX GPIO%d, common GND, "
                        "baud=%d, kit output mode\n",
                        BU_UWB_RX_GPIO, BU_UWB_BAUDRATE));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---- FSR (ADC1, GPIO8) -----------------------------------------*/
static void task_fsr(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        fsr_adc_reading_t r = {0};
        esp_err_t ret = fsr_adc_read(&r);
        (ret == ESP_OK) &&
            r.valid &&
            printf("[FSR] raw=%d voltage=%.3fV "
                   "force_est_kg=%.2f\n",
                   r.raw, r.voltage_v, r.weight_kg);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---- RPLIDAR C1 (UART2, GPIO17/18) -----------------------------*/
static void task_rplidar(void *pvParameters)
{
    (void)pvParameters;
    static uint32_t no_point_count;
    while (1) {
        rplidar_c1_point_t point = {0};
        int got_point = 0;

        for (int i = 0; g_lidar_scan_active && i < 50; ++i) {
            int ok = rplidar_c1_read_point(&g_lidar, &point) &&
                     (point.distance_mm > 0.0f);
            ok &&
                printf("[RPLIDAR] angle=%.2f distance=%.1f "
                       "quality=%u start=%d\n",
                       point.angle_deg, point.distance_mm,
                       point.quality, point.start_bit) &&
                (got_point = 1, i = 50);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        g_lidar_scan_active ||
            ((++no_point_count >= 4) &&
             (no_point_count = 0,
              printf("[RPLIDAR][WAIT] scan did not start; "
                     "check 5V power, rotation, UART direction "
                     "and baud=%d\n", RPLIDAR_BAUDRATE),
              vTaskDelay(pdMS_TO_TICKS(200)),
              (g_lidar_scan_active =
                   (rplidar_c1_start_scan(&g_lidar) == ESP_OK))));

        got_point
            ? (no_point_count = 0)
            : (g_lidar_scan_active && (++no_point_count >= 4) &&
               (no_point_count = 0,
                printf("[RPLIDAR][WAIT] no valid scan point; "
                       "check rotation, 5V current, "
                       "ESP_RX=GPIO%d <- lidar TX\n",
                       RPLIDAR_RX_GPIO)));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---- IMU (I2C0, GPIO11/12, addr 0x23) --------------------------*/
static void task_imu(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        imu_i2c_reading_t data = {0};
        esp_err_t ret = imu_i2c_read_all(&g_imu, &data);
        switch (ret) {
        case ESP_OK:
            data.valid
                ? printf("[IMU] accel=%.3f %.3f %.3f g "
                         "euler=%.2f %.2f %.2f deg\n",
                         data.accel_g[0], data.accel_g[1],
                         data.accel_g[2],
                         data.euler_deg[0], data.euler_deg[1],
                         data.euler_deg[2])
                : printf("[IMU][WAIT] no valid data; "
                         "check SDA=GPIO%d SCL=GPIO%d "
                         "addr=0x%02X\n",
                         HUB_I2C_SDA_GPIO, HUB_I2C_SCL_GPIO,
                         IMU_I2C_ADDR);
            break;
        default:
            printf("[IMU][WAIT] no valid data; "
                   "check SDA=GPIO%d SCL=GPIO%d addr=0x%02X\n",
                   HUB_I2C_SDA_GPIO, HUB_I2C_SCL_GPIO,
                   IMU_I2C_ADDR);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ---- VL53L1X ToF (shared I2C0, addr 0x52) ----------------------*/
static void task_vl53l1x(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        vl53l1x_tof_reading_t r = {0};
        esp_err_t ret = vl53l1x_tof_read(&g_tof, &r, 250);
        (ret == ESP_OK) &&
            r.valid &&
            printf("[VL53L1X] distance=%u mm\n", r.distance_mm);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ================================================================
 * app_main  — init everything, then spawn tasks.
 * No #if / #endif.  No conditional-compilation.
 * ================================================================ */
void app_main(void)
{
    printf("\nAutobox sensor hub test start\n");
    printf("Default I2C: SDA=GPIO%d SCL=GPIO%d\n",
           HUB_I2C_SDA_GPIO, HUB_I2C_SCL_GPIO);

    /* ---- shared I2C bus ----------------------------------------*/
    const i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = HUB_I2C_PORT,
        .sda_io_num = HUB_I2C_SDA_GPIO,
        .scl_io_num = HUB_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    print_status("shared_i2c",
                 i2c_new_master_bus(&i2c_bus_cfg, &g_shared_i2c));

    /* ---- A02YYUW #1 (IO35, SW-UART) ----------------------------*/
    a02yyuw_config_t a02_cfg = a02yyuw_default_config(
        (uart_port_t)A02_1_UART_PORT, A02_1_RX_GPIO,
        A02_1_TX_GPIO);
    a02_cfg.baudrate = A02_1_BAUDRATE;
    a02_cfg.use_sw_uart = A02_1_USE_SW_UART;
    print_status("a02yyuw#1",
                 a02yyuw_init_dev(&g_a02_1, &a02_cfg));
    printf("[A02YYUW#1] SW-UART RX=GPIO%d baud=%d\n",
           A02_1_RX_GPIO, a02_cfg.baudrate);

    /* ---- A02YYUW #2 (IO36, SW-UART) ----------------------------*/
    a02yyuw_config_t a02b_cfg = a02yyuw_default_config(
        (uart_port_t)A02_2_UART_PORT, A02_2_RX_GPIO,
        A02_2_TX_GPIO);
    a02b_cfg.baudrate = A02_2_BAUDRATE;
    a02b_cfg.use_sw_uart = A02_2_USE_SW_UART;
    print_status("a02yyuw#2",
                 a02yyuw_init_dev(&g_a02_2, &a02b_cfg));
    printf("[A02YYUW#2] SW-UART RX=GPIO%d baud=%d\n",
           A02_2_RX_GPIO, a02b_cfg.baudrate);

    /* ---- BU UWB (UART1, GPIO6/7) -------------------------------*/
    bu_uwb_config_t bu_cfg = bu_uwb_default_config(
        (uart_port_t)BU_UWB_UART_PORT, BU_UWB_RX_GPIO,
        BU_UWB_TX_GPIO);
    bu_cfg.baudrate = BU_UWB_BAUDRATE;
    print_status("bu_uwb", bu_uwb_init(&bu_cfg));
    printf("[BU_UWB] passive UART monitor mode: "
           "PA2/TX -> ESP32 RX GPIO%d, "
           "PA3/RX -> ESP32 TX GPIO%d optional, baud=%d\n",
           BU_UWB_RX_GPIO, BU_UWB_TX_GPIO, bu_cfg.baudrate);

    /* ---- FSR (ADC1, GPIO8) -------------------------------------*/
    fsr_adc_config_t fsr_cfg = fsr_adc_default_config();
    fsr_cfg.adc_gpio = FSR_ADC_GPIO;
    fsr_cfg.adc_channel = (adc_channel_t)FSR_ADC_CHANNEL;
    print_status("fsr_adc", fsr_adc_init(&fsr_cfg));

    /* ---- RPLIDAR C1 (UART2, GPIO17/18) -------------------------*/
    rplidar_c1_config_t lidar_cfg = rplidar_c1_default_config(
        (uart_port_t)RPLIDAR_UART_PORT, RPLIDAR_RX_GPIO,
        RPLIDAR_TX_GPIO);
    lidar_cfg.baudrate = RPLIDAR_BAUDRATE;
    esp_err_t lidar_ret = rplidar_c1_init(&g_lidar, &lidar_cfg);
    print_status("rplidar_c1", lidar_ret);
    printf("[RPLIDAR] UART%d baud=%d ESP_RX=GPIO%d "
           "ESP_TX=GPIO%d\n",
           RPLIDAR_UART_PORT, lidar_cfg.baudrate,
           RPLIDAR_RX_GPIO, RPLIDAR_TX_GPIO);

    lidar_ret == ESP_OK &&
        ({
            rplidar_c1_info_t info = {0};
            esp_err_t ret_i = rplidar_c1_get_info(&g_lidar, &info);
            (ret_i == ESP_OK)
                ? printf("[RPLIDAR][INFO] model=%u.%u "
                         "firmware=%u.%u hardware=%u sn=%s\n",
                         info.major_model, info.sub_model,
                         info.firmware_major, info.firmware_minor,
                         info.hardware, info.serial_num)
                : printf("[RPLIDAR][WAIT] get info failed; "
                         "check 5V/GND/TX/RX, baud=%d\n",
                         lidar_cfg.baudrate);

            uint8_t health_status = 0xFF;
            uint16_t health_error = 0;
            esp_err_t ret_h = rplidar_c1_get_health(
                &g_lidar, &health_status, &health_error);
            (ret_h == ESP_OK)
                ? printf("[RPLIDAR][HEALTH] status=%u "
                         "error=0x%04X\n",
                         health_status, health_error)
                : printf("[RPLIDAR][WAIT] get health failed; "
                         "check power/current and UART "
                         "direction\n");

            esp_err_t scan_ret = rplidar_c1_start_scan(&g_lidar);
            print_status("rplidar_start_scan", scan_ret);
            g_lidar_scan_active = (scan_ret == ESP_OK);
        });

    /* ---- IMU (I2C0, addr 0x23) ---------------------------------*/
    imu_i2c_config_t imu_cfg = imu_i2c_default_config();
    imu_cfg.sda_gpio = HUB_I2C_SDA_GPIO;
    imu_cfg.scl_gpio = HUB_I2C_SCL_GPIO;
    imu_cfg.scl_speed_hz = HUB_I2C_SPEED_HZ;
    imu_cfg.device_address = IMU_I2C_ADDR;
    imu_cfg.external_bus = g_shared_i2c;
    esp_err_t imu_ret = imu_i2c_init(&g_imu, &imu_cfg);
    print_status("imu_i2c", imu_ret);
    imu_ret == ESP_OK &&
        ({
            uint8_t version[3] = {0};
            esp_err_t ret_v = imu_i2c_read_version(&g_imu, version);
            (ret_v == ESP_OK)
                ? printf("[IMU] version=%u.%u.%u addr=0x%02X "
                         "SDA=GPIO%d SCL=GPIO%d\n",
                         version[0], version[1], version[2],
                         imu_cfg.device_address,
                         imu_cfg.sda_gpio, imu_cfg.scl_gpio)
                : printf("[IMU][WAIT] version read failed; "
                         "check VCC/GND/SDA/SCL and "
                         "addr=0x%02X\n",
                         imu_cfg.device_address);
        });

    /* ---- VL53L1X ToF (shared I2C0, addr 0x52) ------------------*/
    vl53l1x_tof_config_t tof_cfg = vl53l1x_tof_default_config();
    tof_cfg.sda_gpio = HUB_I2C_SDA_GPIO;
    tof_cfg.scl_gpio = HUB_I2C_SCL_GPIO;
    tof_cfg.scl_speed_hz = HUB_I2C_SPEED_HZ;
    tof_cfg.device_address_8bit = VL53L1X_ADDR_8BIT;
    tof_cfg.timing_budget_ms = VL53L1X_TIMING_BUDGET_MS;
    tof_cfg.inter_measurement_ms =
        VL53L1X_INTER_MEASUREMENT_MS;
    tof_cfg.external_bus = g_shared_i2c;
    print_status("vl53l1x_tof",
                 vl53l1x_tof_init(&g_tof, &tof_cfg));

    /* ---- launch all sensor tasks -------------------------------*/
    xTaskCreate(task_a02yyuw1, "a02_1", 4096, NULL, 1, NULL);
    xTaskCreate(task_a02yyuw2, "a02_2", 4096, NULL, 1, NULL);
    xTaskCreate(task_bu_uwb,   "bu_uwb", 4096, NULL, 2, NULL);
    xTaskCreate(task_fsr,      "fsr",    4096, NULL, 1, NULL);
    xTaskCreate(task_rplidar,  "rplidar", 4096, NULL, 3, NULL);
    xTaskCreate(task_imu,      "imu",    4096, NULL, 2, NULL);
    xTaskCreate(task_vl53l1x,  "vl53",   4096, NULL, 2, NULL);

    printf("All 7 sensor tasks launched.\n");
}
