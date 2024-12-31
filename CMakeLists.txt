# Root CMakeLists.txt file --- run 'cmake' on this root directory to begin build generation!

# Minimum CMake version
cmake_minimum_required(VERSION 3.5)

# Set project name variable (must init project after freertos cmake imports)
SET(ProjectName freertos-ace3)

# Set variables required for library imports
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)	# local freertos kernel path

# Import necessary libraries
include(pico_sdk_import.cmake)	# pico sdk cmake locator (official from freertos)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)	# freertos kernel locator (official from freertos)


##################### BEGIN PROJECT INIT

# Define project
project(${ProjectName})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Init rpi sdk
pico_sdk_init()	# (fn obtained from pico_sdk_import.cmake)

# Add subdirectories for CMake to search during build generation
#add_subdirectory(lib)
add_subdirectory(src)
