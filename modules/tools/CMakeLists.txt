# find_package(OpenCV REQUIRED)
# find_package(realsense2 REQUIRED)

# aux_source_directory(tools tools_DIRS)
# add_library(tools STATIC ${tools_DIRS})
# target_include_directories(tools PUBLIC include)
# target_link_libraries(tools serial motor)

set(kalman_INCLUDE_DIR)
list(APPEND kalman_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
list(APPEND kalman_INCLUDE_DIR "${OpenCV_INCLUDE_DIRS}")

set(kalman_INCLUDE_DIRS ${kalman_INCLUDE_DIR}
                        CACHE PATH "kalman filter include directories")