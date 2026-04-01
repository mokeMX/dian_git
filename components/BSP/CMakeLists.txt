set(src_dirs
            LED
            KEY
            MYIIC
            XL9555
            MYSPI
            SPILCD
            SPI_SD
            ES8388
            MYI2S)

set(include_dirs
            LED
            KEY
            MYIIC
            XL9555
            MYSPI
            SPILCD
            SPI_SD
            ES8388
            MYI2S)

set(requires
            driver
            esp_lcd
            fatfs)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)