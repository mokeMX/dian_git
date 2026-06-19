cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/a02yyuw"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/bu_uwb"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/fsr_adc"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/rplidar_c1"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/imu_i2c"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/vl53l1x_tof"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_hub)
