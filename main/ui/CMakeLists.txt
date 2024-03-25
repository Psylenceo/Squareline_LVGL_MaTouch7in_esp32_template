SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c)

add_library(ui ${SOURCES})

target_include_directories(ui PUBLIC .)
target_link_libraries(ui PRIVATE idf::main)