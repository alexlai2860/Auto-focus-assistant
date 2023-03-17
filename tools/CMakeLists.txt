# find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)

aux_source_directory(src tools_DIRS)
aux_source_directory(src/LogicTools tools_DIRS)
add_library(tools STATIC ${tools_DIRS})
target_include_directories(tools PUBLIC include include/LogicTools)
target_link_libraries(tools param)

# set(kalman_INCLUDE_DIR)
# list(APPEND kalman_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
# list(APPEND kalman_INCLUDE_DIR "${OpenCV_INCLUDE_DIRS}")

# set(kalman_INCLUDE_DIRS ${kalman_INCLUDE_DIR}
#                         CACHE PATH "kalman filter include directories")