aux_source_directory(src detector_DIRS)
aux_source_directory(src/face detector_DIRS)
aux_source_directory(src/body detector_DIRS)
aux_source_directory(src/object detector_DIRS)

add_library(detector STATIC ${detector_DIRS})
target_include_directories(detector PUBLIC include include/face include/body include/object)
target_link_libraries(detector dis)