target_include_directories(PicoSWD INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_sources(PicoSWD PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT.c
   ${CMAKE_CURRENT_SOURCE_DIR}/SEGGER_RTT_printf.c
)