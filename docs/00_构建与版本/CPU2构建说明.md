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

## 源码收集范围
`LTD_MAIN_CPU2/CMakeLists.txt` 当前会编译：
- `Core/Src/*.c`
- `Application/Src/*.c`
- `Services/**/*.c`
- `Drivers/STM32F4xx_HAL_Driver/Src/*.c`
- `BSP/Peripherals/src/*.c`（补齐 TMC5130、AS5145、AD5421、FRAM、CH9141K 等板级外设驱动符号）
- `Core/Startup/startup_stm32f429zgtx.s`
## 产物
手动构建完成后，`build/LTD_MAIN_CPU2/` 下会生成：
- `LTD_MAIN_CPU2.elf`
- `LTD_MAIN_CPU2.hex`
- `LTD_MAIN_CPU2_V<major>.<minor>.<patch>.<build>.hex`
- `LTD_MAIN_CPU2.bin`
- `LTD_MAIN_CPU2.map`

## 发布前清理与产物确认

发布前使用全量清理构建：

```bash
cmake --build build/LTD_MAIN_CPU2 --clean-first
```

固定名 HEX、当前版本名 HEX 和 BIN 已声明为 CMake `POST_BUILD BYPRODUCTS`，执行 `clean` / `clean-first` 时会删除本配置当前产物，避免编译或链接失败后误用同名旧文件。构建目录中的历史版本 HEX 不会自动删除，属于保留归档。

不能只凭 HEX 文件存在判断构建成功；应同时确认构建命令退出码为 0、ELF 和当前版本产物生成时间为本轮时间，并核对固定名与版本名 HEX 的 SHA-256 一致。

## 一键构建脚本
可直接运行：

```bash
./tools/build_cpu2.sh
```
