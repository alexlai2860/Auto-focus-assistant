add_subdirectory(param)
add_subdirectory(serial) # depends on param
add_subdirectory(processor) # depends on serial
add_subdirectory(calibrator) # depends on processor