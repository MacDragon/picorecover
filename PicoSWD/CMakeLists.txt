# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Set the name and version of the project
project(PicoSWD VERSION 1.0.0)

include(pimoroni_pico_import.cmake)

# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
 
# Link the Project to a source file (step 4.6)
add_executable(PicoSWD source.cpp led.c)
 
add_subdirectory(littlefs-lib)
add_subdirectory(msc)

target_include_directories(PicoSWD PRIVATE ${../../wipe/})
target_include_directories(PicoSWD PRIVATE .)

# Link the Project to an extra library (pico_stdlib)
target_link_libraries(PicoSWD pico_stdlib hardware_pio littlefs-lib tinyusb_device tinyusb_board hardware_spi hardware_pwm hardware_dma rgbled pico_display pico_graphics st7789 button)

# Initalise the SDK
pico_sdk_init()

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
include(common/pimoroni_i2c)
#include(libraries/pico_explorer/pico_explorer)
include(libraries/pico_display/pico_display)
include(libraries/bitmap_fonts/bitmap_fonts)
include(libraries/hershey_fonts/hershey_fonts)
include(common/pimoroni_bus)
include(libraries/pico_graphics/pico_graphics)
include(drivers/st7789/st7789)
include(drivers/rgbled/rgbled)
include(drivers/button/button)

set(DBG_PIN_COUNT=4)

# pico_generate_pio_header(PicoTest ${CMAKE_CURRENT_LIST_DIR}/probe.pio)
 
# Enable USB, UART output
#pico_enable_stdio_usb(PicoTest 1)
#pico_enable_stdio_uart(PicoTest 1)
 
# Enable extra outputs (SWD?)
pico_add_extra_outputs(PicoSWD)

pico_enable_stdio_rtt(PicoSWD 1)

pico_set_linker_script(PicoSWD  ${CMAKE_SOURCE_DIR}/memmap_rtt.ld)
