set(srcs 
         "main.c"
         "lvgl_demo_ui.c"
         "smartconfig.c"
         "lvgl_test.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "")

idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
idf_component_get_property(freetype_lib espressif__freetype COMPONENT_LIB)
target_link_libraries(${lvgl_lib} PUBLIC ${freetype_lib})

# littlefs_create_partition_image(storage ../font FLASH_IN_PROJECT)
