set(motor_INCLUDE_DIR)
list(APPEND motor_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
set(motor_INCLUDE_DIRS ${motor_INCLUDE_DIR} CACHE PATH "motor include directories")

# SteppingMotor
aux_source_directory(src motor_DIRS)
aux_source_directory(src/SteppingMotor motor_DIRS)
add_library(SteppingMotor STATIC ${motor_DIRS})
target_include_directories(SteppingMotor PUBLIC ${motor_INCLUDE_DIRS}
                                         PUBLIC include/SteppingMotor)
target_link_libraries(SteppingMotor serial tools)

# NucleusN
aux_source_directory(src motor_DIRS)
aux_source_directory(src/NucleusN motor_DIRS)
add_library(NucleusN STATIC ${motor_DIRS})
target_include_directories(NucleusN PUBLIC ${motor_INCLUDE_DIRS}
                                         PUBLIC include/NucleusN include/SteppingMotor)
target_link_libraries(NucleusN serial tools)

# aux_source_directory(src motor_DIRS)
# aux_source_directory(src/SteppingMotor motor_DIRS)
# add_library(motor STATIC ${motor_DIRS})
# target_include_directories(motor PUBLIC include include/SteppingMotor)
# target_link_libraries(motor serial tools)