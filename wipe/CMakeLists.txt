# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)
 
# Set the name and version of the project
project(wipe VERSION 1.0.0)

# Link the Project to a source file (step 4.6)
add_executable(wipe wipe.c)
 
add_subdirectory(littlefs-lib)
# add_subdirectory(msc)

# Link the Project to an extra library (pico_stdlib)
target_link_libraries(wipe pico_stdlib hardware_flash hardware_adc littlefs-lib)
 
# Initalise the SDK
pico_sdk_init()

# It doesn't make sense to run this program from flash. Always build a
# RAM-only binary.
 
# Enable extra outputs (SWD?)
pico_add_extra_outputs(wipe)

pico_set_binary_type(wipe no_flash)

pico_set_linker_script(wipe ${CMAKE_SOURCE_DIR}/memmap_no_flash_wipe.ld)

target_compile_definitions(hardware_timer INTERFACE PICO_TIME_DEFAULT_ALARM_POOL_DISABLED=1)

add_custom_command(TARGET wipe POST_BUILD
    COMMAND xxd -n helper --include ${CMAKE_BINARY_DIR}/wipe.bin > ${CMAKE_SOURCE_DIR}/helper.h
    COMMENT "Creating include file ${CMAKE_SOURCE_DIR}/helper.h"
)

