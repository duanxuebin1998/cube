# 升级日志

记录 CPU2/CPU3 固件版本变更。使用 `tools/bump_version.py` 升级版本时会自动追加记录；提交前应补充到与 Git 提交信息同等详细。

## 2026-05-11

版本：
- CPU2: 初始版本 -> V1.0.0.0
- CPU3: 初始版本 -> V1.0.0.0

本次修改：
- 引入 CPU2/CPU3 固件版本号。
- 引入 CPU2/CPU3 兼容契约。
- 引入自动升级日志。

## 2026-05-11

版本：
- CPU2: V1.0.0.0 -> V1.1.0.0
- CPU3: V1.0.0.0 -> V1.1.0.0

兼容性：
- CPU2 与 CPU3 使用 major.minor 作为兼容契约，当前均为 V1.1.x，版本匹配。
- CPU3 读取 CPU2 软件版本后会提示版本不匹配，便于现场识别不兼容组合。

本次修改：
- 新增 CPU2/CPU3 固件版本头文件，统一 32 位版本编码和版本字符串。
- CPU2 参数存储运行时写入当前固件版本，避免旧 FRAM 参数覆盖软件版本。
- CPU3 本机版本改为 32 位版本，并显示 CPU2/CPU3 程序版本。
- 新增 CPU2/CPU3 兼容宏和 CPU3 侧版本匹配提示。
- 新增版本升级脚本、提交前版本检查脚本和升级日志。
- CMake 构建输出固定名 hex 与带版本号 hex。
- 移除 preset 构建入口和相关文档，统一使用单独构建目录。

验证：
- `py tools\check_version_bumped.py`
- `py -m py_compile tools\bump_version.py tools\check_version_bumped.py`
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`

## 2026-05-11

版本：
- CPU2: V1.1.0.0 -> V1.2.0.0
- CPU3: 未变化，保持 V1.1.0.0

兼容性：
- 本次仅新增 CPU2 串口 B 类测试指令，不修改 Modbus、参数存储布局或 CPU2/CPU3 共享数据结构。
- CPU2 兼容协议 minor 升至 2；CPU3 未同步升级时，版本兼容检查可能提示 CPU2/CPU3 minor 不一致。

本次修改：
- 保留 `B<mm>` 原有电机模型往返测试行为。
- 新增 `BE<mm>` 编码器反馈往返测试，参数单位为 mm，运行中按编码器计数判断目标位置。
- `BE` 测试使用启动时编码器位置作为固定原点，每轮下行到固定目标、上行回固定原点，避免目标随循环累计漂移。
- `BE` 过程日志同时打印编码值和相对原点的 mm，便于现场比对距离误差。
- 串口接收 `\r\n` 完整命令时清除 `\r`，避免短命令解析到上一条长命令残留后缀。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --check`
- `py tools\check_version_bumped.py`
