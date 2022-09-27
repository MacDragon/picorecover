target_include_directories(PicoSWD INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_sources(PicoSWD PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}/board_flash.c
   ${CMAKE_CURRENT_SOURCE_DIR}/boards.c
   ${CMAKE_CURRENT_SOURCE_DIR}/ghostfat.c
   ${CMAKE_CURRENT_SOURCE_DIR}/msc.c
   ${CMAKE_CURRENT_SOURCE_DIR}/usbload.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/board_api.h
   ${CMAKE_CURRENT_SOURCE_DIR}/board.h
   ${CMAKE_CURRENT_SOURCE_DIR}/boards.h
   ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c
)