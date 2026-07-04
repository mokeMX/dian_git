idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        a02yyuw
        bu_uwb
        fsr_adc
        rplidar_c1
        imu_i2c
        vl53l1x_tof
        esp_driver_i2c
)
