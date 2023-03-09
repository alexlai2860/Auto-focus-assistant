set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set (CMAKE_BUILD_TYPE "Release" )
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set (CMAKE_BUILD_TYPE Debug)

aux_source_directory(src SRC_LIST)
add_library(controller STATIC ${SRC_LIST})
target_include_directories(controller PUBLIC include)
target_link_libraries(controller processor calibrator motor)

# find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)

add_executable(AF_assistant main.cpp)
# link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(AF_assistant controller)
