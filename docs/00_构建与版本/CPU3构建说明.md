# LTD_DISPLAY_CPU3 build guide (GCC + CMake + Ninja)

## Prerequisites
- `cmake` (>= 3.20)
- `ninja`
- Arm GNU Toolchain binaries available in `PATH`:
  - `arm-none-eabi-gcc`
  - `arm-none-eabi-g++`
  - `arm-none-eabi-objcopy`
  - `arm-none-eabi-size`

## Manual build
Run from repository root:

```bash
cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build/LTD_DISPLAY_CPU3
```

## Source collection scope
`LTD_DISPLAY_CPU3/CMakeLists.txt` currently compiles:
- `Core/Src/*.c`
- `Application/**/*.c`
- `Communication/**/*.c`
- `Drivers/STM32F4xx_HAL_Driver/Src/*.c`
- `Core/Startup/startup_stm32f429zgtx.s`

## Build outputs
After the standalone build, `build/LTD_DISPLAY_CPU3/` will contain:
- `LTD_DISPLAY_CPU3.elf`
- `LTD_DISPLAY_CPU3.hex`
- `LTD_DISPLAY_CPU3_V<major>.<minor>.<patch>.<build>.hex`
- `LTD_DISPLAY_CPU3.bin`
- `LTD_DISPLAY_CPU3.map`

## Release clean and artifact verification

Use a clean-first build before release:

```bash
cmake --build build/LTD_DISPLAY_CPU3 --clean-first
```

The fixed-name HEX, current-version HEX, and BIN are declared as CMake `POST_BUILD BYPRODUCTS`. `clean` / `clean-first` removes the current configuration's outputs first, preventing a failed compile or link from being mistaken for a successful build because stale files still exist. Historical versioned HEX files are retained and are not removed automatically.

Do not treat HEX existence alone as proof of success. Confirm an exit code of 0, fresh ELF/current-version artifact timestamps, and matching SHA-256 values for the fixed-name and versioned HEX files.

## One-command build
```bash
./tools/build_cpu3.sh
```
