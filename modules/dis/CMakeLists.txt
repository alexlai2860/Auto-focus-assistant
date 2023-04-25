aux_source_directory(src dis_DIRS)
add_library(dis STATIC ${dis_DIRS})
target_include_directories(dis PUBLIC include)
target_link_libraries(dis serial SteppingMotor NucleusN)