aux_source_directory(src serial_DIRS)
add_library(serial STATIC ${serial_DIRS})
target_include_directories(serial PUBLIC include)
target_link_libraries(serial param)