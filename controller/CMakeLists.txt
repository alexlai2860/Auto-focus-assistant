set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set (CMAKE_BUILD_TYPE "Release" )
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set (CMAKE_BUILD_TYPE Debug)

# 此处的target_include_directories有点混乱和玄学，后续改进

aux_source_directory(src/FocusController SRC_LIST)
add_library(FocusController STATIC ${SRC_LIST})
target_include_directories(FocusController PUBLIC include/FocusController include/CalController include)
target_link_libraries(FocusController dis motor reader calibrator decider)

aux_source_directory(src/CalController SRC_LIST)
add_library(CalController STATIC ${SRC_LIST})
target_include_directories(CalController PUBLIC include/CalController  include/FocusController include)
target_link_libraries(CalController dis motor reader calibrator decider)

aux_source_directory(src/Capturer SRC_LIST)
add_library(Capturer STATIC ${SRC_LIST})
target_include_directories(Capturer PUBLIC include/Capturer include/FocusController include/CalController include)
target_link_libraries(Capturer dis motor reader calibrator decider)

aux_source_directory(src/SleepPoseController SRC_LIST)
add_library(SleepPoseController STATIC ${SRC_LIST})
target_include_directories(SleepPoseController PUBLIC include/Capturer include/FocusController include/CalController include/SleepPoseController include)
target_link_libraries(SleepPoseController dis motor reader calibrator decider)

aux_source_directory(src SRC_LIST)
add_library(controller STATIC ${SRC_LIST})
target_include_directories(controller PUBLIC include)
target_link_libraries(controller dis motor FocusController CalController Capturer SleepPoseController reader)

# find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)

add_executable(AF_assistant main.cpp)
# link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(AF_assistant controller)
