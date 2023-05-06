set(detector_INCLUDE_DIR)
list(APPEND detector_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")

set(detector_INCLUDE_DIRS ${detector_INCLUDE_DIR} CACHE PATH "detector include directories")

# face
aux_source_directory(src FACE_DIRS)
aux_source_directory(src/face FACE_DIRS)
add_library(FaceProcessor STATIC ${FACE_DIRS})
target_include_directories(FaceProcessor PUBLIC ${detector_INCLUDE_DIRS}
                                         PUBLIC include/face)
target_link_libraries(FaceProcessor dis)

# object
aux_source_directory(src OBJECT_DIRS)
aux_source_directory(src/object OBJECT_DIRS)
add_library(ObjectProcessor STATIC ${OBJECT_DIRS})
target_include_directories(ObjectProcessor PUBLIC ${detector_INCLUDE_DIRS}
                                         PUBLIC include/object)
target_link_libraries(ObjectProcessor dis)

# object-light
aux_source_directory(src OBJECT_LIGHT_DIRS)
aux_source_directory(src/ObjectLight OBJECT_LIGHT_DIRS)
add_library(ObjectProcessorLight STATIC ${OBJECT_LIGHT_DIRS})
target_include_directories(ObjectProcessorLight PUBLIC ${detector_INCLUDE_DIRS}
                                         PUBLIC include/ObjectLight)
target_link_libraries(ObjectProcessorLight dis)

# face-light
aux_source_directory(src FACE_LIGHT_DIRS)
aux_source_directory(src/FaceLight FACE_LIGHT_DIRS)
add_library(FaceProcessorLight STATIC ${FACE_LIGHT_DIRS})
target_include_directories(FaceProcessorLight PUBLIC ${detector_INCLUDE_DIRS}
                                         PUBLIC include/FaceLight)
target_link_libraries(FaceProcessorLight dis)

# aux_source_directory(src detector_DIRS)
# aux_source_directory(src/face detector_DIRS)
# aux_source_directory(src/body detector_DIRS)
# aux_source_directory(src/object detector_DIRS)

# add_library(detector STATIC ${detector_DIRS})
# target_include_directories(detector PUBLIC include include/face include/body include/object)
# target_link_libraries(detector dis)