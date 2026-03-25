# LTD_MAIN_CPU2 构建说明（GCC + CMake + Ninja）

## 前置依赖
- `cmake` (>= 3.20)
- `ninja`
- Arm GNU Toolchain（可执行文件需在 `PATH` 中）：
  - `arm-none-eabi-gcc`
  - `arm-none-eabi-g++`
  - `arm-none-eabi-objcopy`
  - `arm-none-eabi-size`

> 说明：该工程按 GCC 工具链构建，不再回退到 clang。

## 手动构建
在仓库根目录执行：

```bash
cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build/LTD_MAIN_CPU2
```

## Preset 构建
如果使用仓库根目录的 preset，请执行：

```bash
cmake --preset cpu2-debug
cmake --build --preset cpu2-debug
```

说明：
- `build/LTD_MAIN_CPU2/` 仅用于上面的“子工程手动构建”。
- `build/presets/LTD_MAIN_CPU2/` 仅用于根目录 preset。
- 不要让两种 source layout 复用同一个 build 目录，否则 CMake 会报 source mismatch。

## 源码收集范围
`LTD_MAIN_CPU2/CMakeLists.txt` 当前会编译：
- `Core/Src/*.c`
- `Application/Src/*.c`
- `Services/**/*.c`
- `Drivers/STM32F4xx_HAL_Driver/Src/*.c`
- `Drivers/Peripherals/src/*.c`（补齐 TMC5130 与 FRAM 驱动符号，解决 `stpr_* / stepper / WriteMultiData / ReadMultiData` 链接错误）
- `Core/Startup/startup_stm32f429zgtx.s`
## 产物
手动构建完成后，`build/LTD_MAIN_CPU2/` 下会生成：
- `LTD_MAIN_CPU2.elf`
- `LTD_MAIN_CPU2.hex`
- `LTD_MAIN_CPU2.bin`
- `LTD_MAIN_CPU2.map`

Preset 构建完成后，对应产物位于 `build/presets/LTD_MAIN_CPU2/`。

## 一键构建脚本
可直接运行：

```bash
./tools/build_cpu2.sh
```
