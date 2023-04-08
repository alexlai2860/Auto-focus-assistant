aux_source_directory(src decider_DIRS)
add_library(decider STATIC ${decider_DIRS})
target_include_directories(decider PUBLIC include)
target_link_libraries(decider dis FaceProcessor ObjectProcessor ObjectProcessorLight reader)