# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)
 
# Set the name and version of the project
project(PicoSWD VERSION 1.0.0)
 
# Link the Project to a source file (step 4.6)
add_executable(PicoSWD source.c led.c)
 
add_subdirectory(littlefs-lib)

target_include_directories(PicoSWD PRIVATE ${../../wipe/})

# Link the Project to an extra library (pico_stdlib)
target_link_libraries(PicoSWD pico_stdlib hardware_pio littlefs-lib)

# Initalise the SDK
pico_sdk_init()

set(DBG_PIN_COUNT=4)

# pico_generate_pio_header(PicoTest ${CMAKE_CURRENT_LIST_DIR}/probe.pio)
 
# Enable USB, UART output
#pico_enable_stdio_usb(PicoTest 1)
#pico_enable_stdio_uart(PicoTest 1)
 
# Enable extra outputs (SWD?)
pico_add_extra_outputs(PicoSWD)

pico_enable_stdio_rtt(PicoSWD 1)

pico_set_linker_script(PicoSWD  ${CMAKE_SOURCE_DIR}/memmap_rtt.ld)
