cmake_minimum_required (VERSION 3.1)

project (AutoFocusAssistant)

# set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
# set (CMAKE_BUILD_TYPE "Release" )
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# set (CMAKE_BUILD_TYPE Debug)

add_subdirectory(param)
add_subdirectory(serial)
add_subdirectory(modules)
add_subdirectory(controller)