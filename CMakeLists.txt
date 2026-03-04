cmake_minimum_required(VERSION 3.20)

project(CUBE NONE)

set(CUBE_TARGET "" CACHE STRING "Firmware target to configure")

if(CUBE_TARGET STREQUAL "LTD_MAIN_CPU2")
    add_subdirectory(LTD_MAIN_CPU2)
elseif(CUBE_TARGET STREQUAL "LTD_DISPLAY_CPU3")
    add_subdirectory(LTD_DISPLAY_CPU3)
else()
    message(FATAL_ERROR "Unknown CUBE_TARGET='${CUBE_TARGET}'. Expected LTD_MAIN_CPU2 or LTD_DISPLAY_CPU3.")
endif()
