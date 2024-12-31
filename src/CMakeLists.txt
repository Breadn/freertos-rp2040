# bottom level cmake file for source building

# Set source and header files
set(SOURCES 
	main.c
)

# Add an executable target to be generated
add_executable(${ProjectName}
	${SOURCES}
)

# Include directories to search for includes for this (main) target
target_include_directories(${ProjectName} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_SOURCE_DIR}/inc
)

# Include libraries to link for this (main) target
target_link_libraries(${ProjectName}
	pico_stdlib	# standard pico sdk lib
	FreeRTOS-Kernel
	FreeRTOS-Kernel-Heap4	# heap 4 method for FreeRTOS kernel (unused if not dynamic)
)

###### Target specific config
# Enable strict compilation checks
target_compile_options(${ProjectName} PUBLIC
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wall>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wextra>
	$<$<COMPILE_LANG_AND_ID:C,Clang>:-Weverything>
	$<$<COMPILE_LANG_AND_ID:C,Clang>:-Werror>
	## No warning below (loosen check strictness) ##
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-sign-compare>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-unused-function>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-unused-parameter>
	$<$<COMPILE_LANG_AND_ID:C,Clang,GNU>:-Wno-type-limits>			# rp2040 port has too many of these...
)

# Enable pico uoutputs (from pico sdk)
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 1)

# ACE 3.0 specific board configuration
target_compile_definitions(${ProjectName} PRIVATE
	# Default pico uart tx/rx pins (from pico sdk)
	#PICO_DEFAULT_UART_TX_PIN=1
	#PICO_DEFAULT_UART_RX_PIN=2
	#PICO_DEFAULT_UART_BAUD_RATE=115200
	
	# Change default i2c pins
	# PICO_DEFAULT_I2C_INSTANCE=i2c1
)

# Add extra useful outputs from pico sdk
pico_add_extra_outputs(${ProjectName})

add_subdirectory(littlebookofsemaphores/)
