idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES board a02yyuw bu_uwb fsr_adc rplidar_c1 chassis esp_timer
)
