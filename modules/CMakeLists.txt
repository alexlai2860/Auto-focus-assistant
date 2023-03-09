add_subdirectory(param)
add_subdirectory(serial) # depends on param
add_subdirectory(motor) # depends on serial
add_subdirectory(processor) # depends on motor
add_subdirectory(calibrator) # depends on processor