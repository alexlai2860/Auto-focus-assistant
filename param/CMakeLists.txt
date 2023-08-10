find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

#onnxruntime
set(ONNXRUNTIME_ROOT_PATH /usr/local)
set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_ROOT_PATH}/include)
set(ONNXRUNTIME_LIB ${ONNXRUNTIME_ROOT_PATH}/lib/libonnxruntime.so.1.14.1)
include_directories(${ONNXRUNTIME_INCLUDE_DIRS})

aux_source_directory(src param_DIRS)
add_library(param STATIC ${param_DIRS})
target_include_directories(param PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(param ${OpenCV_LIBS} ${realsense2_LIBRARY} ${ONNXRUNTIME_LIB})