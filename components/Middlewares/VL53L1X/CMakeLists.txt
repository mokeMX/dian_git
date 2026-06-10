list(APPEND srcs   vl53l1x.c
VL53L1X_api.c
VL53L1X_calibration.c
vl53l1_platform.c
                   )

idf_component_register(SRCS  "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES esp_partition
                             spi_flash
                             fatfs
                             BSP
                             driver hal)