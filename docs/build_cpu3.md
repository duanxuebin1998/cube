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

## Preset build
If you want to use the repository-root preset, run:

```bash
cmake --preset cpu3-debug
cmake --build --preset cpu3-debug
```

Notes:
- `build/LTD_DISPLAY_CPU3/` is reserved for the standalone subproject build above.
- `build/presets/LTD_DISPLAY_CPU3/` is reserved for the repository-root preset flow.
- Do not reuse one build directory across those two source layouts, or CMake will report a source mismatch.

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
- `LTD_DISPLAY_CPU3.bin`
- `LTD_DISPLAY_CPU3.map`

Preset artifacts are generated under `build/presets/LTD_DISPLAY_CPU3/`.

## One-command build
```bash
./tools/build_cpu3.sh
```
