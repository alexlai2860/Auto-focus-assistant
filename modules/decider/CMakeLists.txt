find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# 生成名为calibrator的静态库
aux_source_directory(src decider_DIRS)
add_library(decider STATIC ${decider_DIRS})
target_include_directories(decider PUBLIC include)
target_link_libraries(decider dis detector reader)