cmake_minimum_required(VERSION 3.16)

set(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(EXTRA_COMPONENT_DIRS
    "${PROJECT_ROOT}/components/board"
    "${PROJECT_ROOT}/components/sensors/a02yyuw"
    "${PROJECT_ROOT}/components/sensors/bu_uwb"
    "${PROJECT_ROOT}/components/sensors/fsr_adc"
    "${PROJECT_ROOT}/components/sensors/rplidar_c1"
    "${PROJECT_ROOT}/components/control/chassis"
)

set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_check)
