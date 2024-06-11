# CMake for BMP5 common build

SET(libname BMP5_common_lib)

# Create lib
add_library(${libname}
    bmp5_common.h
    bmp5_common.c
)

# Includes
target_include_directories(${libname} PRIVATE
    ../BMP5_SensorAPI
)

# Link lib
target_link_libraries(${libname}
    pico_stdlib
    hardware_i2c
)