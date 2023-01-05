cmake_minimum_required (VERSION 3.1)

project (RM_energy_identify)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

set( CMAKE_BUILD_TYPE "Release" )

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set (CMAKE_BUILD_TYPE Debug)

aux_source_directory(src SRC_LIST)

include_directories(include)

include_directories( ${realsense2_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)

add_executable(main ${SRC_LIST})

link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(main ${OpenCV_LIBS} ${realsense2_LIBRARY})
