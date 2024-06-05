# CMake for common build

SET(BMP3CommonLibName BMP3_common_lib)

# Create common library
add_library(${BMP3CommonLibName}
    bmp3_common.h
    bmp3_common.c
)

# Add BMP3 includes
target_include_directories(${BMP3CommonLibName} PRIVATE
    ../BMP3_SensorAPI
)

# Link libraries
target_link_libraries(${BMP3CommonLibName}
    pico_stdlib
    hardware_i2c
)
