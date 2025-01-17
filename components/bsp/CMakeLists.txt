set(src_dirs
            i2c
            xl9555
            lcd
            spi)

set(include_dirs
            i2c
            xl9555
            lcd
            spi)

set(requires
            driver
            esp_lcd
            lvgl
            esp_lcd_touch_ft5x06)

idf_component_register(SRC_DIRS ${src_dirs}
                       INCLUDE_DIRS ${include_dirs} 
                       REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)

