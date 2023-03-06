find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# 生成名为calibrator的静态库
aux_source_directory(src calibrator_DIRS)
add_library(calibrator STATIC ${calibrator_DIRS})
target_include_directories(calibrator PUBLIC include)
target_link_libraries(calibrator processor)