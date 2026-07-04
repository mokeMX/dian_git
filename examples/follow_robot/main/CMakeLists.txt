idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        a02yyuw
        bu_uwb
        rplidar_c1
        imu_i2c
        chassis
        follow_avoid
        esp_driver_i2c
        esp_timer
        esp_driver_ledc
        esp_driver_gpio
)
