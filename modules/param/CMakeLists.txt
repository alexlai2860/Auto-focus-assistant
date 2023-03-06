find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

aux_source_directory(src param_DIRS)
add_library(param STATIC ${param_DIRS})
target_include_directories(param PUBLIC include ${OpenCV_INCLUDE_DIR})
target_link_libraries(param ${OpenCV_LIBS} ${realsense2_LIBRARY})