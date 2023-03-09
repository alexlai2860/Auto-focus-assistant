find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

aux_source_directory(src motor_DIRS)
aux_source_directory(src/stepping_motor motor_DIRS)
add_library(motor STATIC ${motor_DIRS})
target_include_directories(motor PUBLIC include include/stepping_motor)
target_link_libraries(motor serial)