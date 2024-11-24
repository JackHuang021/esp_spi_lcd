set(src_dirs
            iic
            xl9555
            lcd)

set(include_dirs
            iic
            xl9555
            lcd)

set(requires
            driver
            esp_lcd
            lvgl)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)

