find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

aux_source_directory(src reader_DIRS)
aux_source_directory(src/RsReader reader_DIRS)
# aux_source_directory(src/AstraReader reader_DIRS)

add_library(reader STATIC ${reader_DIRS})
target_include_directories(reader PUBLIC include include/RsReader)
target_link_libraries(reader serial)