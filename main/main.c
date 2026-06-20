#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "myiic.h"
#include "vl53l1x.h"
#include "imu_i2c_driver.h"
#include "rplidar.h"

static const char *TAG = "MAIN";

#define LIDAR_UART_NUM  UART_NUM_1
#define LIDAR_TX_PIN    17
#define LIDAR_RX_PIN    18

static imu_measurement_t imu_data;

void app_main(void)
{
    ESP_LOGI(TAG, "=== Initializing I2C bus ===");
    if (myiic_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed!");
        return;
    }
    ESP_LOGI(TAG, "I2C bus ready (SCL=42, SDA=41)");

    ESP_LOGI(TAG, "=== Initializing VL53L1X ToF sensor ===");
    if (vl53l1x_app_init(VL53L1X_DEFAULT_DEV_ADDR) != ESP_OK) {
        ESP_LOGW(TAG, "VL53L1X init failed, continuing without it");
    }

    ESP_LOGI(TAG, "=== Initializing IMU sensor ===");
    if (i2c_module_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU I2C init failed!");
        return;
    }
    IMU_I2C_ReadVersion();

    ESP_LOGI(TAG, "=== Initializing RPLIDAR C1 ===");
    esp_err_t err = rplidar_init(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RPLIDAR UART init failed, continuing without it");
    } else {
        ESP_LOGI(TAG, "Resetting RPLIDAR...");
        rplidar_stop(LIDAR_UART_NUM);
        rplidar_reset(LIDAR_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint8_t health_status = 0;
        uint16_t error_code = 0;
        if (rplidar_get_health(LIDAR_UART_NUM, &health_status, &error_code) == ESP_OK) {
            ESP_LOGI(TAG, "RPLIDAR Health: %d, Error: 0x%04X", health_status, error_code);
        }

        rplidar_info_t info;
        if (rplidar_get_device_info(LIDAR_UART_NUM, &info) == ESP_OK) {
            ESP_LOGI(TAG, "RPLIDAR Model: %d.%d, FW: %d.%d, SN: %s",
                     info.major_model, info.sub_model,
                     info.firmware_major, info.firmware_minor,
                     info.serial_num);
        }

        ESP_LOGI(TAG, "Starting RPLIDAR scan...");
        if (rplidar_start_scan(LIDAR_UART_NUM) != ESP_OK) {
            ESP_LOGW(TAG, "RPLIDAR scan start failed");
        }
    }

    ESP_LOGI(TAG, "=== All sensors initialized, entering main loop ===");

    uint32_t loop_count = 0;
    uint16_t vl53_distance = 0;
    rplidar_point_t lidar_point;

    while (1) {
        loop_count++;

        printf("\n========== Frame %lu ==========\n", (unsigned long)loop_count);

        if (vl53l1x_get_single_distance(VL53L1X_DEFAULT_DEV_ADDR, &vl53_distance) == ESP_OK) {
            printf("[VL53L1X] Distance: %d mm\n", vl53_distance);
        } else {
            printf("[VL53L1X] Read failed\n");
        }

        IMU_I2C_ReadAll(&imu_data);
        printf("[IMU] Accel(g):  x=%.3f y=%.3f z=%.3f\n", imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        printf("[IMU] Gyro(r/s): x=%.3f y=%.3f z=%.3f\n", imu_data.gyro[0],  imu_data.gyro[1],  imu_data.gyro[2]);
        printf("[IMU] Euler(deg): roll=%.3f pitch=%.3f yaw=%.3f\n", imu_data.euler[0], imu_data.euler[1], imu_data.euler[2]);
        printf("[IMU] Baro: height=%.3fm temp=%.3fC press=%.3fPa\n", imu_data.baro[0], imu_data.baro[1], imu_data.baro[2]);

        if (rplidar_read_point(LIDAR_UART_NUM, &lidar_point)) {
            if (lidar_point.quality > 0) {
                printf("[RPLIDAR] Angle: %6.2f deg, Dist: %7.2f mm, Quality: %3d, NewScan: %d\n",
                       lidar_point.angle, lidar_point.distance, lidar_point.quality, lidar_point.start_bit);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
