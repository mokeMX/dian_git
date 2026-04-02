set(src_dirs
            LED
            KEY
            MYIIC
            XL9555
            MYSPI
            MYI2S
            SPILCD)

set(include_dirs
            LED
            KEY
            MYIIC
            XL9555
            MYSPI
            MYI2S
            SPILCD)

set(requires
            driver
            esp_lcd)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)