set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
<<<<<<< ours

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

find_program(ARM_GCC NAMES arm-none-eabi-gcc)
if(ARM_GCC)
  set(CMAKE_C_COMPILER   ${ARM_GCC})
  set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
  set(CMAKE_ASM_COMPILER ${ARM_GCC})
else()
  find_program(CLANG_COMPILER NAMES clang REQUIRED)
  set(CMAKE_C_COMPILER   ${CLANG_COMPILER})
  set(CMAKE_CXX_COMPILER clang++)
  set(CMAKE_ASM_COMPILER ${CLANG_COMPILER})
  add_compile_options(--target=arm-none-eabi)
  add_link_options(--target=arm-none-eabi -fuse-ld=lld)
endif()

find_program(CMAKE_AR NAMES arm-none-eabi-ar llvm-ar REQUIRED)
find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy llvm-objcopy REQUIRED)
find_program(CMAKE_OBJDUMP NAMES arm-none-eabi-objdump llvm-objdump REQUIRED)
find_program(CMAKE_SIZE NAMES arm-none-eabi-size llvm-size)
=======
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi)

find_program(ARM_GCC NAMES ${TOOLCHAIN_PREFIX}-gcc REQUIRED)
find_program(ARM_GXX NAMES ${TOOLCHAIN_PREFIX}-g++ REQUIRED)
find_program(ARM_AR NAMES ${TOOLCHAIN_PREFIX}-ar REQUIRED)
find_program(ARM_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}-objcopy REQUIRED)
find_program(ARM_OBJDUMP NAMES ${TOOLCHAIN_PREFIX}-objdump REQUIRED)
find_program(ARM_SIZE NAMES ${TOOLCHAIN_PREFIX}-size REQUIRED)

set(CMAKE_C_COMPILER   ${ARM_GCC})
set(CMAKE_CXX_COMPILER ${ARM_GXX})
set(CMAKE_ASM_COMPILER ${ARM_GCC})
set(CMAKE_AR           ${ARM_AR})
set(CMAKE_OBJCOPY      ${ARM_OBJCOPY})
set(CMAKE_OBJDUMP      ${ARM_OBJDUMP})
set(CMAKE_SIZE         ${ARM_SIZE})

set(MCPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
>>>>>>> theirs

set(CMAKE_C_FLAGS_INIT "${MCPU_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${MCPU_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${MCPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCPU_FLAGS}")
