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
set(CPU2_BUILD_VARIANT "debug" CACHE STRING "CPU2 build variant: debug or release")
set_property(CACHE CPU2_BUILD_VARIANT PROPERTY STRINGS debug release)
string(TOLOWER "${CPU2_BUILD_VARIANT}" CPU2_BUILD_VARIANT_LOWER)
if(CPU2_BUILD_VARIANT_LOWER STREQUAL "debug")
    set(CPU2_VARIANT_SUFFIX "DEBUG")
    set(CPU2_COMMAND_RX_ENABLED 1)
elseif(CPU2_BUILD_VARIANT_LOWER STREQUAL "release")
    set(CPU2_VARIANT_SUFFIX "RELEASE")
    set(CPU2_COMMAND_RX_ENABLED 0)
else()
    message(FATAL_ERROR "CPU2_BUILD_VARIANT must be debug or release")
endif()
set(CPU2_VERSIONED_HEX_NAME ${PROJECT_NAME}_${CPU2_FIRMWARE_VERSION}_${CPU2_VARIANT_SUFFIX}.hex)
set(CPU2_HEX ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_${CPU2_VARIANT_SUFFIX}.hex)
set(CPU2_VERSIONED_HEX ${CMAKE_CURRENT_BINARY_DIR}/${CPU2_VERSIONED_HEX_NAME})
set(CPU2_BIN ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_${CPU2_VARIANT_SUFFIX}.bin)

file(GLOB CORE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
)

set(APP_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/app/app_main.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/commands/serial_command.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/commands/serial_command_parser.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/diagnostics/fault_manager.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/diagnostics/fault_recovery.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/diagnostics/part_diagnostics.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/measure.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/measure_composite_commands.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/measure_level_commands.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/measure_maintenance_commands.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/density/measure_density.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/density/measure_density_acquisition.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/density/measure_density_commands.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/density/measure_density_si.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/density/measure_density_single_point.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/density/wartsila_density_measurement.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_calibration.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_common.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_density.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_follow.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_frequency.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_refresh_policy.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_reference_refresh.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_position.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_runtime.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_search.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_step_search.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/oil_level/oil_level_strategy.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/tank_height/measure_tank_height.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/water_level/water_level_calibration.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/water_level/water_level_common.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/water_level/water_level_fast_search.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/water_level/water_level_follow.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/water_level/water_level_search.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/measurement/zero/measure_zero.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_command.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_demo.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_fixed_frequency.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_motor.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_sensor_protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_storage.c
    ${CMAKE_CURRENT_LIST_DIR}/Application/Src/service_debug/service_debug_water_profile.c
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
    CPU2_USART1_COMMAND_RX_ENABLED=${CPU2_COMMAND_RX_ENABLED}
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
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CPU2_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CPU2_VERSIONED_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${CPU2_BIN}
)

if(CMAKE_SIZE)
    list(PREPEND POST_BUILD_COMMANDS COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>)
endif()

add_custom_command(TARGET ${TARGET} POST_BUILD
    ${POST_BUILD_COMMANDS}
    BYPRODUCTS
        ${CPU2_HEX}
        ${CPU2_VERSIONED_HEX}
        ${CPU2_BIN}
    COMMENT "Generating ${PROJECT_NAME}_${CPU2_VARIANT_SUFFIX}.hex, ${CPU2_VERSIONED_HEX_NAME}, and ${PROJECT_NAME}_${CPU2_VARIANT_SUFFIX}.bin"
    VERBATIM
)
