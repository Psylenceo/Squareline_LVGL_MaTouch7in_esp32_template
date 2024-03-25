idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "." "ui"
                       #REQUIRES lvgl
                       )

add_subdirectory(ui)
target_link_libraries(${COMPONENT_LIB} PRIVATE ui)