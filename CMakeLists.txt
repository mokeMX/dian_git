cmake_minimum_required(VERSION 3.16)

# Fix UnicodeDecodeError on Chinese Windows: kconfgen reads sdkconfig with GBK
# encoding but the file contains UTF-8 (Kconfig help text / Kconfig.projbuild).
# PYTHONUTF8=1 forces Python 3.7+ to use UTF-8 for all file I/O.
set(ENV{PYTHONUTF8} "1")

# Point to the example's sdkconfig.defaults (target, WiFi, etc.)
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/examples/follow_robot/sdkconfig.defaults")

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/a02yyuw"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/bu_uwb"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/fsr_adc"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/rplidar_c1"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/imu_i2c"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/vl53l1x_tof"
    "${CMAKE_CURRENT_LIST_DIR}/components/control/chassis"
    "${CMAKE_CURRENT_LIST_DIR}/components/control/follow_avoid"
    "${CMAKE_CURRENT_LIST_DIR}/examples/follow_robot/main"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(follow_robot)
