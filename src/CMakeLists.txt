# bottom level cmake file for source building

# Set source and header files
set(SOURCES 
	main.c
	bmp5_internal.c
)
set(HEADERFILES 
	bmp5_internal.h	
)
set(HEADERS)
FOREACH(FILE ${HEADERFILES})
	list(APPEND HEADERS ${CMAKE_SOURCE_DIR}/inc/${FILE})
ENDFOREACH()

# Add an executable target to be generated
add_executable(${ProjectName}
	${SOURCES} ${HEADERS}
)

# Include directories to search for includes for this (main) target
target_include_directories(${ProjectName} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_SOURCE_DIR}/lib/BMP5_SensorAPI
	${CMAKE_SOURCE_DIR}/lib/BMP5_common
	${CMAKE_SOURCE_DIR}/inc
)

# Include libraries to link for this (main) target
target_link_libraries(${ProjectName}
	pico_stdlib	# standard pico sdk lib
	FreeRTOS-Kernel-Heap4	# heap 4 method for FreeRTOS kernel
	BMP5_lib
	BMP5_common_lib
)

###### Target specific config
# Enable strict compilation checks
target_compile_options(${ProjectName} PUBLIC
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wall>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wextra>
	$<$<COMPILE_LANG_AND_ID:C,Clang>:-Weverything>
	## No warning below (loosen check strictness) ##
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-sign-compare>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-unused-function>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-unused-parameter>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-type-limits>			# rp2040 port has too many of these...
)

# Enable pico uart output (from pico sdk)
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 1)

# ACE 3.0 specific board configuration
target_compile_definitions(${ProjectName} PRIVATE
	# Change default pico uart tx/rx pins (from pico sdk)
	PICO_DEFAULT_UART_TX_PIN=16
	PICO_DEFAULT_UART_RX_PIN=17

	# Change default i2c pins
	# PICO_DEFAULT_I2C_INSTANCE=i2c1
)

# Add extra useful outputs from pico sdk
pico_add_extra_outputs(${ProjectName})