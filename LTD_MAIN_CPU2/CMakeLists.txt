cmake_minimum_required(VERSION 3.20)

project(LTD_MAIN_CPU2 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(TARGET ${PROJECT_NAME}.elf)

set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/STM32F429ZGTX_FLASH.ld)
set(APP_VERSION_HEADER ${CMAKE_CURRENT_LIST_DIR}/Application/Inc/app_version.h)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${APP_VERSION_HEADER})

file(STRINGS
    ${APP_VERSION_HEADER}
    CPU2_VERSION_STRING_LINE
    REGEX "^#define[ \t]+CPU2_APP_VERSION_STRING[ \t]+\"V[^\"]+\""
)
string(REGEX REPLACE ".*\"([^\"]+)\".*" "\\1" CPU2_FIRMWARE_VERSION "${CPU2_VERSION_STRING_LINE}")
message(STATUS "CPU2 firmware version: ${CPU2_FIRMWARE_VERSION}")
set(CPU2_VERSIONED_HEX ${PROJECT_NAME}_${CPU2_FIRMWARE_VERSION}.hex)

file(GLOB CORE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/*.c
)

file(GLOB_RECURSE SERVICE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Services/*.c
)

file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c
)

file(GLOB BSP_PERIPHERAL_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/BSP/Peripherals/src/*.c
)

set(STARTUP_SOURCE
    ${CMAKE_CURRENT_LIST_DIR}/Core/Startup/startup_stm32f429zgtx.s
)

add_executable(${TARGET}
    ${CORE_SOURCES}
    ${APP_SOURCES}
    ${SERVICE_SOURCES}
    ${HAL_SOURCES}
    ${BSP_PERIPHERAL_SOURCES}
    ${STARTUP_SOURCE}
)

target_compile_definitions(${TARGET} PRIVATE
    USE_HAL_DRIVER
    STM32F429xx
)

target_include_directories(${TARGET} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Application/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/BSP/Peripherals/inc
    ${CMAKE_CURRENT_LIST_DIR}/Services
    ${CMAKE_CURRENT_LIST_DIR}/Services/Encoder
    ${CMAKE_CURRENT_LIST_DIR}/Services/Hart
    ${CMAKE_CURRENT_LIST_DIR}/Services/Modbus
    ${CMAKE_CURRENT_LIST_DIR}/Services/MotorControl
    ${CMAKE_CURRENT_LIST_DIR}/Services/ParamStorage
    ${CMAKE_CURRENT_LIST_DIR}/Services/Sensor
    ${CMAKE_CURRENT_LIST_DIR}/Services/Utilities
    ${CMAKE_CURRENT_LIST_DIR}/Services/Weight
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
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CPU2_VERSIONED_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${PROJECT_NAME}.bin
)

if(CMAKE_SIZE)
    list(PREPEND POST_BUILD_COMMANDS COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>)
endif()

add_custom_command(TARGET ${TARGET} POST_BUILD
    ${POST_BUILD_COMMANDS}
    COMMENT "Generating ${PROJECT_NAME}.hex, ${CPU2_VERSIONED_HEX}, and ${PROJECT_NAME}.bin"
)
