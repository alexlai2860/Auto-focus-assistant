find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

aux_source_directory(src param_DIRS)
add_library(processor STATIC ${param_DIRS})
target_include_directories(processor PUBLIC include)
target_link_libraries(processor serial motor)