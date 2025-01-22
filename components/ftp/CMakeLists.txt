set(src_dirs ftp.c)

set(include_dirs ".")

set(requires driver nvs_flash esp_wifi)

idf_component_register(SRCS ${src_dirs}
                       INCLUDE_DIRS ${include_dirs} 
                       REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)

