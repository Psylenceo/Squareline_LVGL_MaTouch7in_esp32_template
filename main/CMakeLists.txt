idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "." "ui"
                       REQUIRES lvgl
                       REQUIRES esp_lcd
                       )

add_subdirectory(ui)
target_link_libraries(${COMPONENT_LIB} PRIVATE ui)