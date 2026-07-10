cmake_minimum_required(VERSION 3.20)

project(LTD_DISPLAY_CPU3 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(TARGET ${PROJECT_NAME}.elf)

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/STM32F429ZGTX_FLASH.ld)
set(APP_VERSION_HEADER ${CMAKE_CURRENT_LIST_DIR}/Application/app_version.h)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${APP_VERSION_HEADER})

file(STRINGS
    ${APP_VERSION_HEADER}
    CPU3_VERSION_STRING_LINE
    REGEX "^#define[ \t]+CPU3_APP_VERSION_STRING[ \t]+\"V[^\"]+\""
)
string(REGEX REPLACE ".*\"([^\"]+)\".*" "\\1" CPU3_FIRMWARE_VERSION "${CPU3_VERSION_STRING_LINE}")
message(STATUS "CPU3 firmware version: ${CPU3_FIRMWARE_VERSION}")
set(CPU3_VERSIONED_HEX_NAME ${PROJECT_NAME}_${CPU3_FIRMWARE_VERSION}.hex)
set(CPU3_HEX ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(CPU3_VERSIONED_HEX ${CMAKE_CURRENT_BINARY_DIR}/${CPU3_VERSIONED_HEX_NAME})
set(CPU3_BIN ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin)

file(GLOB CORE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
)

file(GLOB_RECURSE APPLICATION_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Application/*.c
)

file(GLOB_RECURSE COMMUNICATION_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Communication/*.c
)

file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c
)

set(STARTUP_SOURCE
    ${CMAKE_CURRENT_LIST_DIR}/Core/Startup/startup_stm32f429zgtx.s
)

add_executable(${TARGET}
    ${CORE_SOURCES}
    ${APPLICATION_SOURCES}
    ${COMMUNICATION_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_SOURCE}
)

target_compile_definitions(${TARGET} PRIVATE
    USE_HAL_DRIVER
    STM32F429xx
)

target_include_directories(${TARGET} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/Application
    ${CMAKE_CURRENT_LIST_DIR}/Application/display
    ${CMAKE_CURRENT_LIST_DIR}/Application/system_param
    ${CMAKE_CURRENT_LIST_DIR}/Communication/common
    ${CMAKE_CURRENT_LIST_DIR}/Communication/external/DSM_modbus
    ${CMAKE_CURRENT_LIST_DIR}/Communication/external/si_modbus
    ${CMAKE_CURRENT_LIST_DIR}/Communication/external/wartsila_modbus
    ${CMAKE_CURRENT_LIST_DIR}/Communication/internal/main_board_modbus
)

target_compile_options(${TARGET} PRIVATE
    -Wall
    -ffunction-sections
    -fdata-sections
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-Os -g0>
)

target_link_options(${TARGET} PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${TARGET} PRIVATE --specs=nosys.specs)
    target_link_libraries(${TARGET} PRIVATE c m nosys)
else()
    target_link_libraries(${TARGET} PRIVATE c m)
endif()

set(POST_BUILD_COMMANDS
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CPU3_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CPU3_VERSIONED_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${CPU3_BIN}
)

if(CMAKE_SIZE)
    list(PREPEND POST_BUILD_COMMANDS COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>)
endif()

add_custom_command(TARGET ${TARGET} POST_BUILD
    ${POST_BUILD_COMMANDS}
    BYPRODUCTS
        ${CPU3_HEX}
        ${CPU3_VERSIONED_HEX}
        ${CPU3_BIN}
    COMMENT "Generating ${PROJECT_NAME}.hex, ${CPU3_VERSIONED_HEX_NAME}, and ${PROJECT_NAME}.bin"
    VERBATIM
)
