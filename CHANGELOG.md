# 升级日志

记录 CPU2/CPU3 固件版本变更。使用 `tools/bump_version.py` 升级版本时会自动追加记录；提交前应补充到与 Git 提交信息同等详细。

## CPU2 程序版本与参数存储版本

CPU2 参数加载会校验 FRAM 中的 `magic`、`struct_size`、`param_version` 和 `crc`。A/B 两份分区都连续读取失败时，会恢复出厂默认参数并重新写入 FRAM。程序版本和参数存储版本是两套信息：程序版本用于发布追踪，`DEVICE_PARAM_VERSION` 用于判断旧 FRAM 参数是否能按当前结构读取。

| CPU2 程序版本 | CPU2 参数存储版本 | 参数存储兼容性 |
| --- | ---: | --- |
| V1.0.0.0 | 2 | 引入程序版本体系时，CPU2 当前参数存储版本已为 2；从未带当前 `LTDM` 元信息或版本 1 旧固件升级时，旧 FRAM 可能因元信息/版本/结构不匹配恢复出厂参数 |
| V1.1.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.2.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.2.1.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.3.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.4.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.4.1.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.5.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.5.1.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.6.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.7.0.0 | 2 | 存储版本不变；原 `reserved23` 改为“探底修正罐高”，旧存储默认按 0 处理并继续沿用液位罐高 |
| V1.7.1.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.7.1.1 | 2 | 存储版本不变，通常保留旧参数 |
| V1.7.1.2 | 2 | 存储版本不变，通常保留旧参数 |
| V1.7.2.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.7.3.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.8.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.8.1.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.9.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.9.1.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.10.0.0 | 2 | 存储版本不变；原 `reserved2` 改为故障自动恢复重跑上限，旧存储升级到协议版本 6 时补默认值 `3`；`empty_weight` 按 `int32_t` 有符号解释 |
| V1.11.0.0 | 2 | 存储版本不变，通常保留旧参数 |
| V1.11.0.1 | 2 | 存储版本不变，通常保留旧参数 |
| V1.12.0.0 | 3 | 存储版本从 2 升到 3，`DeviceParameters` 结构大小变化；从 V1.11.0.1 及更早存储版本 2 固件升级会恢复出厂参数，升级前必须备份或记录现场参数，升级后重新下发并核对 |
| V1.12.0.1 | 3 | 存储版本不变，通常保留旧参数 |
| V1.12.0.2 | 3 | 存储版本不变，通常保留旧参数 |
| V1.12.1.0 | 3 | 存储版本不变，通常保留旧参数 |
| V1.12.1.1 | 3 | 存储版本不变；仅调整参数、打印和文档中的称重/探底等中文口径，通常保留旧参数 |
| V1.12.1.2 | 3 | 存储版本不变；同步 11 项恢复出厂默认值，旧 FRAM 参数通常保留，新默认值仅在恢复出厂或 FRAM 无效时生效 |

历史说明：建立 CPU2 程序版本号前，2025-12-16 引入当前参数元信息时使用 `DEVICE_PARAM_VERSION=1`；2026-03-05 系统参数增加时提升到 `DEVICE_PARAM_VERSION=2`，从版本 1 升级到版本 2 会因旧参数版本不匹配恢复出厂参数。

## 2026-05-11 - 引入 CPU2/CPU3 版本与兼容契约

版本：
- CPU2: 初始版本 -> V1.0.0.0
- CPU3: 初始版本 -> V1.0.0.0

本次修改：
- 引入 CPU2/CPU3 固件版本号。
- 引入 CPU2/CPU3 兼容契约。
- 引入自动升级日志。

## 2026-05-11 - CPU3 显示 CPU2 版本并提示兼容性

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

## 2026-05-11 - 新增 CPU2 串口 B 类测试指令

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

## 2026-05-11 - 优化液位跟随和频率异常恢复

版本：
- CPU2: V1.2.0.0 -> V1.2.1.0
- CPU3: 未变化，保持 V1.1.0.0

兼容性：
- 本次仅调整 CPU2 液位跟随和液位频率异常恢复流程，不修改 Modbus、参数存储布局或 CPU2/CPU3 共享寄存器映射。
- CPU2/CPU3 兼容契约不变，仍按现有 major.minor 规则识别兼容性。

本次修改：
- 液位跟随稳定时不再重复刷新液位值，只打印已保存的液位寄存器值并标明单位为 0.1mm。
- 液位频率发生变化时，先进入寻找液位状态重新精找，成功后再恢复液位跟随状态并更新液位值。
- 液位频率连续 3 次为 0 或大于 6500Hz 时，增加现场恢复流程：电机静止时先上行 1mm，再切密度模式等待 3 秒，最后切回液位模式等待 10 秒稳定后继续读取。
- 电机仍在运行时不执行 1mm 上行动作，仅执行模式恢复，避免抢占正在进行的电机运动。
- 新增液位频率异常恢复流程图，便于现场说明和复核。

验证：
- `git diff --check`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`

## 2026-05-12 - 完善测量异常自动恢复和驱动重初始化

版本：
- CPU2: V1.2.1.0 -> V1.3.0.0
- CPU3: 未变化，保持 V1.1.0.0

兼容性：
- 本次仅调整 CPU2 测量异常恢复、电机驱动复位后重初始化、传感器通信重试和部件参数检查逻辑，不修改 Modbus 寄存器映射、参数存储布局或 CPU2/CPU3 共享数据结构。
- CPU2 兼容协议 minor 升至 3；CPU3 未同步升级时，版本兼容检查可能提示 CPU2/CPU3 minor 不一致。

本次修改：
- 新增测量命令异常后的自动恢复流程：保留原错误状态，每 1 秒读取部件参数，全部部件恢复正常后重新执行完整测量命令。
- 传感器通信重试统一调整为 10 次、间隔 300ms，并在 UART6/无线通信等待和重试阶段支持有效命令切换打断。
- 读取部件参数拆分为正常命令和恢复检查两种入口；恢复检查不切换设备状态，读取部件参数时称重只判断自身通信是否超时。
- TMC5130 检测到复位标志、驱动状态异常或 SPI 读写失败时，使电机驱动参数初始化失效，后续重新初始化会完整下发驱动配置。
- 编码器 SSI 错误记录最近一次错误码，部件参数恢复检查可识别编码器通信异常。
- 新增测量失败自动恢复需求文档和流程图，便于现场复核恢复策略。

验证：
- `git diff --check`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`

## 2026-05-13 - 完善错误日志和故障状态传递

版本：
- CPU2: V1.3.0.0 -> V1.4.0.0
- CPU3: 未变化，保持 V1.1.0.0

兼容性：
- 本次仅调整 CPU2 错误日志、错误码传递、故障出口去重和现场诊断文档，不修改 Modbus 寄存器映射、参数存储布局或 CPU2/CPU3 共享数据结构。
- CPU2 兼容协议 minor 升至 4；CPU3 未同步升级时，版本兼容检查可能提示 CPU2/CPU3 minor 不一致。

本次修改：
- 新增 CPU2 统一错误日志模块，统一输出 `错误重试`、`重试成功`、`错误报警`、`最终报错` 四类现场日志。
- 调整故障最终出口，由 `fault_manager` 统一打印最终报错，并将最终报错操作名改为 `置错误状态`。
- 补充电机、编码器、传感器、滑环通信、称重、参数存储和测量流程中的重试、恢复和告警日志。
- 优化错误码名称、模块名和原因映射，补齐滑环主机/从机、TMC5130、编码器、称重等错误含义。
- 修正多处调用链错误码传递，避免底层具体错误被通用失败码覆盖。
- 将错误日志中的八进制转义中文还原为可维护的中文字符串，并保持 GBK 源码编码。
- 新增和更新 UART6 异常包、故障重试日志整理、近期固件工作计划等中文文档。

验证：
- `rg '\\[0-7]{3}' LTD_MAIN_CPU2 LTD_DISPLAY_CPU3 docs tools`
- `git diff --check`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`

## 2026-05-14 - 调整版本兼容检查和协议版本规则

版本：
- CPU2: V1.4.0.0 -> V1.4.1.0
- CPU3: V1.1.0.0 -> V1.1.1.0

兼容性：
- 本次不修改 Modbus 寄存器映射、参数存储布局、命令枚举数值或 CPU2/CPU3 共享字段含义。
- CPU2/CPU3 固件版本继续独立递增；CPU3 显示 CPU2 程序版本时仅按主版本判断兼容，minor/patch/build 不再触发版本不匹配提示。

本次修改：
- 新增长等待公共入口 `AbortableDelay_CommandSwitch()`，让固定点监测、液位/密度模式稳定等待、称重稳定等待等流程可响应命令切换。
- 调整 `STATE_SWITCH` 日志语义，命令切换不再作为错误重试或最终报错打印。
- 将串口 `I/G/K/R/W/O/P/Q` 正式业务命令统一映射到 `CommandType`，交给主循环正式入口执行，避免绕过 `current_command` 和自动恢复。
- 增强 TMC5130 健康检查，识别 `CHOPCONF=0` 配置丢失、充电泵欠压、功率级电流未建立和运行期 XACTUAL 异常。
- 抽出 `MotorDriver_SyncPositionOrCheckHealth()` 和 `MotorDriver_ReturnAfterTemporarySpeed()`，收敛运动等待和临时速度恢复中的重复错误处理。
- 将自动恢复状态机拆分为 `fault_recovery.c/.h`，主循环只负责调度和重跑命令，恢复模块不直接调用测量命令。
- 保持找零点粗找原有重试/退让语义，仅修正粗找失败时错误码被“零点偏差超限”覆盖的问题。
- 新增传感器通信失败后的轻量滑环探测，区分传感器、滑环主机和滑环从机无响应。
- 修正固定点监测中密度读取失败后不置错误状态的问题。
- 更新《电机24V断电与命令切换改动整理》文档，整理最终方案、验证清单和后续建议。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_version_bumped.py`
- `git diff --check`
- `git diff --cached --check`

## 2026-05-14 - 扩展密度分布测量点和瓦锡兰寄存器

版本：
- CPU2: V1.4.1.0 -> V1.5.0.0
- CPU3: V1.1.1.0 -> V1.2.0.0

兼容性：
- 本次扩展 CPU2/CPU3 共享密度分布测量点数组和内部输入寄存器范围，两侧需要同步升级。
- 外部 Wärtsilä 密度点寄存器由 100 点扩展到 200 点，前 100 点地址保持不变，新增点位向后追加。
- 原 `reserved1` 预留位正式替换为 CPU2/CPU3 协议版本字段；CPU2 启动后写入当前协议，旧 CPU2 默认协议版本为 0，当前 200 点协议版本为 1。

本次修改：
- 将 CPU2 密度分布测量最大点数 `MAX_MEASUREMENT_POINTS` 从 100 提高到 200。
- 将 CPU3 同步缓存和内部主板通信的密度分布点数组从 100 点提高到 200。
- 将 CPU3 “分布测点数”参数输入上限从 99 提高到 200，避免界面或参数校验挡住新增点数。
- 将 CPU3 Wärtsilä 外部 Modbus 密度点数量 `REG_DENSITY_POINT_COUNT` 从 100 提高到 200。
- 新增 CPU2/CPU3 协议版本文档，CPU3 显示 CPU2 固件版本时只做显示，设备状态页统一提示协议是否匹配；只有两侧协议版本相同才兼容，避免 CPU2 V1.4.1.0 被误判为支持 200 点。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --check`
- `py tools\check_version_bumped.py` 当前未暂存文件，脚本提示无暂存内容可检查；提交前暂存后需再执行一次。

## 2026-05-15 - 补齐 CPU3 本地错误码显示

版本：
- CPU2: 未变化，保持 V1.5.0.0
- CPU3: V1.2.0.0 -> V1.3.0.0

兼容性：
- 本次仅修改 CPU3 本地显示逻辑和 CPU3 本地错误码枚举补齐，不新增 Modbus 寄存器，不改变 `REG_DEVICE_STATUS_ERROR_CODE` 含义。
- CPU2/CPU3 共享协议版本不变，CPU3 继续用现有 32 位错误码在本地解释故障原因。

本次修改：
- CPU3 设备状态栏保持原有故障代码显示，不把故障原因拼入状态栏。
- CPU3 设备详情分页在故障态新增一行中文故障原因，按 `device_status.error_code` 显示短文本原因。
- CPU3 中文故障原因一行显示不下时自动占用下一详情行继续显示，后续详情项随分页顺延。
- CPU3 故障原因文案改为更贴近现场含义的描述，例如通信无响应、寄存器通信失败、测量流程超时等。
- CPU3 显示字库补充故障原因所需汉字，避免中文原因出现空字。
- 补齐 CPU3 缺失的 CPU2 错误码定义，覆盖电机、传感器、滑环、测量、参数等新增错误码。

验证：
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --check`
- `py tools\check_version_bumped.py` 当前未暂存文件，脚本提示无暂存内容可检查；提交前暂存后需再执行一次。

## 2026-05-15 - 修复 UART 和传感器通信异常处理

- CPU2: V1.5.0.0 -> V1.5.1.0 (patch)

兼容性：
- 本次仅修改 CPU2 内部 UART5/Modbus 中断路径打印、UART6 传感器 DMA 收发和异常帧处理，不改变 CPU2/CPU3 Modbus 寄存器地址、字段含义或协议版本。
- CPU3 固件无需同步升级。

本次修改：
- 将 UART5 0x10 写保持寄存器路径中的 `command` 打印和逐寄存器 `HoldingRegisterArray` 打印移出默认路径，避免在 UART5 中断上下文走 USART1 阻塞打印。
- DSM 文本协议 UART6 收发改为单次 DMA 流程，先启动接收 DMA、再用发送 DMA 发命令，按换行终止符识别帧结束，保留命令切换打断和 UART 硬件错误检测。
- DSM 通信重试日志改为带详情输出，记录命令、接收长度、UART 错误标志和原始 HEX，便于区分超时、短帧、BCC 和硬件错误。
- 水电容 `Cl` 响应恢复固定帧校验，严格检查 11 字节长度、帧头、`\r\n` 结尾、数值字段和 BCC，设备异常帧不再继续解析为有效电容。
- LTD/V2 与无线 8 字节 UART6 链路改为单次 DMA 收发，先打开 8 字节接收窗口、再用发送 DMA 发命令，收满 8 字节后执行原有校验和、功能码、地址和参数码检查。
- UART6 DMA 接收过程中遇到硬件错误时立即停止 DMA 并返回格式错误，避免继续等待并误判为普通超时。
- 初始化传感器协议识别改为读取传感器编号：LTD/V2 使用 `DSM_V2_Read_SensorID()`，DSM 一代使用 `CN` 振动管编号并提取数字部分写入 `sensorID`。
- 新增 `SensorWireless_CommTest()` 手动通信测试入口，并接入 `SC` 串口调试命令，覆盖无线主机/从机探测、传感器协议识别，以及 DSM 一代或 LTD/V2 关键参数读取。
- 回零点流程仅在 DSM 一代传感器下读取零点电容和陀螺仪零点基准；LTD/V2 不支持这些辅助通道时跳过对应步骤，避免把不支持的辅助通道误报为回零点故障。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --check`

## 2026-05-15 - 增加取消测量命令和命令切换保护

- CPU2: V1.5.1.0 -> V1.6.0.0 (minor)
- CPU3: V1.3.0.0 -> V1.4.0.0 (minor)

兼容性：
- 本次新增 CPU2/CPU3 共享命令 `CMD_CANCEL_MEASUREMENT = 16`，协议版本从 1 升级到 2。
- CPU2 V1.6.0.0 与 CPU3 V1.4.0.0 配套使用；旧 CPU2 不识别命令 16，协议版本不匹配时应按状态页提示处理。

本次修改：
- CPU3 设备状态显示界面支持长按返回键进入“是否停止测量?”确认页；菜单界面返回键行为保持不变。
- 取消测量确认页复用罐上操作按键逻辑，返回键取消，确认键直接下发取消测量命令。
- CPU3 处于故障状态时，长按返回键进入故障原因查看页；测量结果详情页不再自动插入故障原因行。
- CPU3 长按返回只进入确认页或故障原因查看页；取消测量命令在确认页按确认后直接下发。
- CPU2 新增取消测量命令处理：取消故障自动恢复、慢停电机、清除当前命令和错误码，并将设备状态切到待机。
- 同步 CPU2/CPU3 `CommandType` 枚举、`DEVICE_PROTOCOL_VERSION` 和协议变更记录。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --check`
- `py tools\check_version_bumped.py` 当前未暂存文件，脚本提示无暂存内容可检查；提交前暂存后需再执行一次。

## 2026-05-15 - 增加探底修正罐高和电机记步诊断

版本：
- CPU2: V1.6.0.0 -> V1.7.0.0
- CPU3: V1.4.0.0 -> V1.5.0.0

协议版本/兼容性：
- CPU2/CPU3 共享协议版本从 2 升至 3。
- 原 `reserved23` 正式替换为“探底修正罐高”，保持寄存器地址不移动，旧协议存储升级时默认清零并沿用液位罐高。
- 旧 CPU2/CPU3 与协议 3 固件混用时，共享参数含义不完整，应按协议版本不匹配处理。

本次修改：
- 新增“探底修正罐高”参数，用于罐底测量完成后的编码器修正；参数为 0 时兼容旧逻辑，继续使用液位罐高。
- 瓦锡兰分布测量完成并达到测后探底频率后，先移动到固定点监测位置，且只移动不读取密度。
- 瓦锡兰测后固定点移动或后续探底失败时，退出后续流程并切回单点监测，不通过 `SET_ERROR` 置错误状态。
- 罐底测量实高与编码器修正目标罐高的偏差大于实高最大偏差时，跳过编码器修正。
- CPU3 同步新增参数表、菜单分组、内部 Modbus 读写和参数同步映射。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_version_bumped.py`
- `git diff --check`
- `py LTD_DISPLAY_CPU3\font_check.py`

## 2026-05-15 - 增加罐底离底确认

- CPU2: V1.7.0.0 -> V1.7.1.0 (patch)
- 说明：粗找罐底前增加离底确认，精找罐底离底上行改为无称重检测，避免已触底状态直接误判完成或误触发称重碰撞报错

## 2026-05-15 - 限制自动恢复重跑次数

- CPU2: V1.7.1.0 -> V1.7.1.1 (build)
- 兼容性：不改变 CPU2/CPU3 共享协议，不影响读取部件参数命令。
- 本次修改：
  - 自动恢复在部件参数读取恢复后，最多自动重跑原测量命令 3 次。
  - 同一条命令跨多轮“恢复成功 -> 重跑测量 -> 再失败”时保留重跑计数，达到上限后保持错误态并停止继续重入测量。
  - 部件参数读取仍作为恢复确认点持续执行，不纳入测量重跑次数限制。
- 验证：
  - `cmake --build build\LTD_MAIN_CPU2`

## 2026-05-16 - 修复 LTD 模式切换和瓦锡兰分布测量流程

- CPU2: V1.7.1.1 -> V1.7.1.2 (build)
- 说明：修复LTD/V2模式切换和部件参数读取误报13-13；DSM首字母E/e仅提示传感器电压过低；优化固定点监测退出、瓦锡兰分布测首点定位和测后探底日志；找罐底固定距离上行改为无检测上行，避免脱离罐底时误报称重错误；更新LTD故障代码表和测试记录。

## 2026-05-16 - 完善液位同步负位置钳位和瓦锡兰回位

版本：
- CPU2: V1.7.1.2 -> V1.7.2.0
- CPU3: 未变化，保持当前版本

兼容性：
- 本次仅修改 CPU2 内部测量流程、位置计算钳位、测量结果上报钳位、瓦锡兰测后探底前回固定点重试和文档记录。
- 不修改 CPU2/CPU3 共享寄存器、命令码、数据结构字段含义，不升级 `DEVICE_PROTOCOL_VERSION`。
- 不修改本设备和瓦锡兰设备对应关系，CPU3 无需同步升级。

本次修改：
- 液位修正后使用 `int64_t` 计算罐高并钳位，按当前记步源刷新当前位置，立即同步 `oil_measurement.oil_level` 和 `density_distribution.Density_oil_level`；电机记步模式下刷新位置使用 TMC5130 `XACTUAL`，不再强制使用外部编码轮。
- `MotorCtrl_SnapshotSensorPositionMm()` 改为保留 `int32_t` 负位置；`MotorCtrl_MoveToPosition()` 增加当前位置快照范围保护，负位置可正常参与绝对位置移动，仅拦截超出罐高量级的极端快照。
- 找罐底罐高记录、传感器温度位置、水位计算、瓦锡兰测点位置和液位结果写入前增加有符号计算或钳位。
- 普通单点/固定点、普通密度分布液位、液位测量/跟随和水位结果写入无符号上报字段前增加负值钳位，负测量结果按 0 上报；液位跟随负位置不再按下限错误处理。
- 液位修正计算出的罐高为负或超出 `uint32_t` 范围时，取消本次修正，保留原罐高和修正参数，避免误覆盖有效罐高。
- 新增按当前记步源刷新业务位置的接口；罐高变化、编码器修正和电机局部周长标定路径避免在电机记步模式下用编码轮覆盖 `sensor_position`。
- 瓦锡兰测后需要探底时，探底前回固定点监测位置最多尝试 3 次；三次失败后跳过探底并切回固定点监测，不置错误状态。
- 合并“液位修正后液位值不刷新改进方案”和“有符号位置转无符号风险检查”两个新增 md 到 `docs/05_测试记录/26.05.16_CPU2_V1.7.2.0液位同步负位置钳位与瓦锡兰回位整改记录.md`。
- 新增 `26.05.16_CPU2_V1.7.1.2详细改动方案.md/.pdf`，并更新 `LNG计量仪屏幕菜单.docx`，补齐暂存区内实际包含的文档变更。

当前仍存在的风险：
- `measure_waterLevel.c` 的水位精找减速距离和水位标定罐高计算仍建议后续统一改用 `int64_t`。
- `MoveToPosition()` 在 `tankHeight` 为 0 或异常时使用 `500m` 兜底范围，仍建议结合现场恢复和校准边界确认是否足够。
- `MoveToPosition()` 当前仅校验当前位置快照，固定点、单点和运行到指定位置等目标位置参数仍建议后续增加统一范围校验。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`
- `git diff --check`

## 2026-05-19 - 增加罐底下行保护和回零称重判定

版本：
- CPU2: V1.7.2.0 -> V1.7.3.0
- CPU3: 未变化，保持 V1.5.0.0

协议版本/兼容性：
- 本次仅修改 CPU2 罐底测量保护和称重碰撞判定逻辑。
- 不修改 CPU2/CPU3 共享协议、寄存器映射、命令码或参数布局，`DEVICE_PROTOCOL_VERSION` 不变。
- `maxDownDistance` 继续沿用原参数含义，不新增参数，不改变已有参数地址。

本次修改：
- 罐底粗找和精找每轮下行前增加最大尺带长度保护，限制值为 `tankHeight + maxDownDistance`。
- 当尺带长度超过最大允许位置且仍未识别到罐底时，立即快速停机并返回 `MEASUREMENT_WEIGHT_DOWN_FAIL`，避免罐底测量无限下行。
- 罐底粗找和精找在传感器位置低于 1m 时，将下行速度上限压到 0.50m/min，并打印传感器位置、原速度和实际下发速度。
- 回零/标零上行时，称重超过零点阈值视为正常到零点信号，不再被通用防撞逻辑抢先误报 18-3。
- 新增《罐底测量下行保护改动说明》文档，记录问题背景、修改点、现场日志和验证建议。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE_temp_bottom_zero_commit/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`
- `git diff --check`
- `git diff --cached --check`

<## 2026-05-16 - 适配 SI7000 协议和共享协议版本 4

版本：
- CPU2: V1.7.3.0 -> V1.8.0.0
- CPU3: V1.5.0.0 -> V1.6.0.0

兼容性：
- 本次将 SI7000 所需补充状态融合进 CPU2/CPU3 既有测量结构，`DEVICE_PROTOCOL_VERSION` 从 3 升级到 4。
- CPU2/CPU3 必须同为协议版本 4，CPU3 才能完整获得外部协议转换所需的 profile 完成、探底参考、液位到达和偏差报警状态。

本次修改：
- 合并 `wip/si7000-protocol-assist` 中的外部协议辅助状态、CPU2 测量状态维护、CPU3 外部 SI7000 Modbus 从站基础实现和协议映射文档。
- 收紧 CPU2/CPU3 职责边界：CPU2 只保留通用业务状态和命令，SI7000 地址、线圈、缩放、影子寄存器和异常响应集中在 CPU3 转换层。
- CPU3 主分发路径和 `com_manager` 兼容路径统一调用 `si7000_modbus_process_for_dispatch()`，避免异常响应帧处理语义重复。
- CPU3 新增 SI7000 协议选择项，并接入 COM1/COM2/COM3 协议分发。
- CPU3 新增本地 RTC 时钟接口，SI7000 `30011-30013` 返回当前时分秒，profile 完成计数变化时锁存 `30007-30010` 月日时分。
- SI7000 `FC05` 写线圈增加模式/动作影子区互斥，避免连续写入后读回多个互斥命令位。
- SI7000 `FC05` 对协议保留线圈写入返回非法地址，避免 PLC 误写保留位时收到成功 echo。
- SI7000 profile 时间戳在 RTC 暂不可读时允许后续读寄存器继续尝试锁存，避免偶发初始化窗口丢失时间戳。
- SI7000 `FC06` 对自动 profile 使能、小时、分钟做基础值域检查，非法值返回 Modbus 异常而不进入影子寄存器。
- SI7000 `FC06` 对 `40004-40009` 保留寄存器写入返回非法地址，读取仍保持 0。
- SI7000 profile 点阵只输出 `Number Of Points` 范围内的有效测点，范围外保持 0，避免 PLC 读到旧 profile 残留数据。
- SI7000 输入寄存器按协议单位输出位置、液位、温度和密度；密度由内部 `kg/m3 x10` 转为协议 `kg/m3 x100`，超出 16 位时钳位。
- SI7000 `10012` 和阈值报警位由 `40011`、`40014-40021` 影子寄存器合成，阈值为 0 时视为未启用。
- 修正 CPU3 串口有校验位时的 WordLength 配置，支持 SI7000 要求的 9600 8O1，并在 SI7000 协议选中后自动锁定端口配置。
- 更新 SI7000 执行计划、兼容映射表、CPU2 暂存区逐文件改动整理和 CPU2/CPU3 协议变更记录。

验证：
- `git diff --check`
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_si7000_modbus_frames.py`
- `py tools\check_si7000_protocol_contract.py`
- `py tools\check_version_bumped.py`

## 2026-05-23 - 调整串口 B/BE 低检测执行流程

版本：
- CPU2: V1.8.0.0 -> V1.8.1.0
- CPU3: 未变化，保持 V1.6.0.0

协议版本/兼容性：
- 本次仅调整 CPU2 串口 B/BE 调试指令执行流程，不修改 Modbus、SI7000、CPU2/CPU3 共享寄存器映射、参数存储布局或协议版本。
- B/BE 属于现场调试入口，兼容既有命令格式：`B<mm>`、`BE<mm>`、`S` 后缀和 `,1` 后缀仍可使用。

本次修改：
- `process_command()` 中 B/BE 分支前置到通用 `MeasureStart()` 之前，避免 B/BE 被测量初始化、称重、电机健康检查或历史错误码拦截。
- B/BE 新增专用低检测执行路径：初始化阶段只尽量写 TMC5130 基础配置，运动阶段直接写 `RAMPMODE`、`VMAX`、`XTARGET`，不再调用 `MotorCtrl_Init()`、`MotorCtrl_MoveNoWait()` 或 `stpr_waitMove()`。
- B 指令只负责按电机模型下发往返运动；运行期下发失败、停止状态读取失败等只打印并重试，不检测编码器、称重、过热等业务错误。
- B/BE 电机运行函数改为直接写 TMC5130 `RAMPMODE`、`VMAX`、`XTARGET` 等寄存器，不再根据 `stpr_moveBy()`、`stpr_moveTo()`、`stpr_setVelocity()`、`stpr_setPos()` 返回错误码决定流程。
- BE 指令进入后固定记录起始编码点和下行目标点，循环过程中不重新计算；下行结束后记录上行起点，上行始终回到最初起始点。
- BE 单段运动下发后，只按固定编码目标或 `RAMPSTAT.VZERO` 电机停转结束本阶段；不再读取编码器错误码或其它全局错误码。
- 电机停转判断改为二次确认：第一次读到 `RAMPSTAT.VZERO` 后延时 200ms，再次读取仍为 `VZERO` 才认为停稳，避免换向瞬间速度为 0 被误判。
- BE 不重复调用编码器采集定时器启动函数，编码器定时器仍由上电初始化负责，避免把“已启动”误打印成启动失败。
- S 后缀通信检查和 `SC` 通信测试的传感器参数读取都改为按 `sensorType` 区分 DSM 一代和 LTD/V2 传感器，每次只发一次读命令；DSM 一代只读频率/密度/温度，LTD/V2 只读密度，不切液位模式、不读液位频率。S 后缀调用前后保存并恢复 `device_state` 和 `error_code`，不参与退出条件或错误状态。
- 每段运动后统一只等待 `TMC5130_RAMPSTAT.VZERO`，速度归零后才下发下一段运动，避免电机拟合目标不准导致等待不到 `POSREACHED`。
- 新增《串口 B / BE 指令详细执行过程》HTML 文档和本版本改动与测试方案，便于现场按最终流程复核。

验证：
- `git diff --check`
- `cmake --build build\LTD_MAIN_CPU2`
- 已用 GBK/936 解码检查 `measure.c`、`test.c` 无替换字符。
- 待硬件现场验证：B/BE 连续往返、没插称重时 BE 仍持续运行、S 后缀通信打印不置错、运动停稳后再下发下一段。

## 2026-05-23 - 完善无线滑环最近匹配正式命令与MAC显示（CPU2 V1.9.0.0 / CPU3 V1.7.0.0）

版本：
- CPU2: V1.8.1.0 -> V1.9.0.0
- CPU3: V1.6.0.0 -> V1.7.0.0

协议版本/兼容性：
- 无线滑环分支中的 `V1.8.0.0` / CPU3 `V1.6.0.0` 是阶段性开发口径；合并到 MAIN 后，因 MAIN 已使用协议版本 4，最终以协议版本 5、CPU3 V1.7.0.0 发布。
- 新增 CPU2/CPU3 共享命令 `CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117`，并在输入寄存器末尾追加无线滑环匹配结果状态，协议版本从 4 升级到 5。
- CPU2 V1.9.0.0 与 CPU3 V1.7.0.0 配套使用；旧 CPU2 不识别命令 117，协议版本不匹配时应按状态页提示处理。
- 不新增保持寄存器地址，不改变 `command` 寄存器宽度；MAC 仅作为运行态输入寄存器状态发布，不新增无线滑环 MAC/RSSI/名称持久化参数。

本次修改：
- 新增 `ch9141_at.c/.h`，封装 UART6 AT 指令收发、空闲等待、响应收集以及 `OK`、`ERR`、`LINK OK`、`PAIR ERR`、`SCAN END`、RSSI 上报判定。
- 新增 `wireless_pairing.c/.h`，实现 CH9141K 主机模式确认、扫描候选解析、RSSI 近距离选择、名称字段选择、连接、默认连接保存和当前连接状态查询。
- 新增串口调试命令 `SPS`、`SPR`、`SPN=<name>`、`SPC`：分别用于扫描打印、RSSI 近距离匹配、按扫描名称匹配、读取当前连接状态。
- 修正 CH9141K 软件 AT 入口为 `AT...`，修正旧 `CH9141EVT.c` 初始化路径裸 `AT`，并修正 RSSI 异步等待、`BLEMODE?` 主机模式判断、扫描 MAC 边界解析和 UART6 残留污染问题。
- `SPR` 只在最强 RSSI 不低于 `-55 dB` 且多候选时领先第二名不少于 `8 dB` 时自动匹配；`SPN=<name>` 只接受显式名称字段，避免误配。
- 新增固定点监测样机宏 `ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE`，默认启用；固定点监测移动到设定位置后使用虚拟温度和 `999.5kg/m3` 水密度，不访问真实密度传感器。
- 增强 `BE<mm>` 编码器闭环往返测试日志，并将 `BE/B` 的 `S` 通信检查改为按 `g_deviceParams.sensorType` 分支，避免向 DSM 一代误发 LTD/V2 参数帧。
- 统一 `process_command()` 串口调试提示为中文字段格式，便于现场日志检索。
- CPU2 新增正式命令入口，收到命令 117 时跳过电机 `MeasureStart()` 初始化，直接复用 `WirelessPairing_RunByRssi()` 执行 CH9141K RSSI 最近匹配。
- CPU2/CPU3 新增设备状态 `STATE_WIRELESS_PAIRING = 0x0032` 和 `STATE_WIRELESS_PAIRING_OVER = 0x8032`，用于正式发布无线滑环匹配中和匹配完成。
- CPU2 在匹配开始、成功、失败时发布 `WirelessPairingStatus`，包含结果码、MAC 是否有效、`AA:BB`/`CC:DD`/`EE:FF` 三段 MAC、失败错误码和更新计数。
- CPU3 维护/调试菜单新增“匹配无线滑环”无参菜单项，并通过现有 `send_cpu2_command()` 写入 CPU2 `command` 寄存器。
- CPU3 新增菜单操作码使用显式值 `1000`，不插入既有连续枚举区间，避免改变已有菜单和参数操作码数值。
- CPU3 正常轮询新增的 `WirelessPairingStatus` 输入寄存器区间，避免匹配结果和 MAC 状态未刷新。
- CPU3 下发匹配命令后退出罐上操作覆盖页；收到匹配中/完成设备状态后由普通设备状态页显示“无线滑环匹配中/完成”，完成状态下显示两行 MAC。
- 匹配失败时 CPU2 将 `device_status.error_code` 置为失败错误码，并将 `device_status.device_state` 置为 `STATE_ERROR`，CPU3 按现有故障页显示错误码。
- 同步 CPU2/CPU3 `CommandType` 枚举和 `DEVICE_PROTOCOL_VERSION`，保持命令码一致。
- 更新《无线滑环最近匹配正式命令需求》和 CPU2/CPU3 协议变更记录，明确触发状态、兼容影响、非目标和验收标准。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE=0U` 临时编译通过，恢复 `1U` 后最终编译通过。
- `BE/B` 的 `S` 通信检查按传感器类型分流后，`cmake --build build\LTD_MAIN_CPU2` 通过。
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `py tools\check_version_bumped.py`
- `git diff --check`（仅提示 Git 下次触碰部分 LF 文件时会转换为 CRLF，未发现空白错误）
- `git diff --cached --check`
- CPU2 GBK 源码抽检通过，未发现替换字符或 `??`
- 待台架验证：CPU3 菜单触发最近匹配、单从机匹配成功后显示从机 MAC、多从机 RSSI 差值不足拒绝并显示失败错误码、匹配后主/从节点透传探测、断电重启自动连接；`SPS` 扫描输出、`SPR` 单近距离从机匹配、多候选拒绝、`SPN=<name>` 名称字段匹配、`SPC` 已连接/未连接状态查询；固定点样机模式下显示虚拟温度和 `999.5kg/m3` 水密度且不访问真实传感器；`BE<mm>` 停止时输出完整退出原因。

## 2026-06-02 - 支持 DSM V1.228 外部协议兼容需求（CPU3 V1.8.0.0）

版本：
- CPU2: 未变化，保持 V1.9.0.0
- CPU3: V1.7.0.0 -> V1.8.0.0

协议版本/兼容性：
- 本次仅调整 CPU3 外部 DSM 协议适配层，不修改 CPU2/CPU3 共享命令、共享参数语义、寄存器映射或 `DEVICE_PROTOCOL_VERSION`。
- DSM V1.228 的第 6 段保持寄存器按外部兼容占位处理，未落地到 CPU2 共享协议或参数存储。
- CPU3 固件版本升至 V1.8.0.0，用于追踪外部 DSM 兼容行为变化；共享协议版本保持不变。

本次修改：
- 增加 DSM V1.228 外部状态翻译，把 CPU2 内部状态映射为 DSM 外部状态、自检占位状态和兼容状态码。
- 增加第 6 段保持寄存器零值占位响应，满足 DSM V1.228 对新增保持寄存器区间的读取兼容。
- 调整 DSM 线圈命令映射、读写数量校验和无效地址返回，避免占位指令误下发到 CPU2。
- 更新 DSM V1.228 协议需求方案文档、CHANGELOG 和升级日志 PDF。

验证：
- `py tools\check_version_bumped.py`（使用本次 amend 临时暂存区验证）
- `git diff --cached --check`（使用本次 amend 临时暂存区验证）
- `cmake --build build\LTD_DISPLAY_CPU3`
- 未做现场联调：DSM V1.228 主站读取第 6 段保持寄存器、自检占位状态和异常地址返回仍需台架复测。

## 2026-06-02 - 限制单点测量和固定点监测目标位置范围（CPU2 V1.9.1.0）

版本：
- CPU2: V1.9.0.0 -> V1.9.1.0
- CPU3: 未变化，保持 V1.8.0.0

协议版本/兼容性：
- 本次仅调整 CPU2 单点测量、固定点监测和瓦锡兰测后回固定点的执行前位置校验。
- 不修改 CPU2/CPU3 共享命令、共享参数语义、寄存器映射或 `DEVICE_PROTOCOL_VERSION`。
- 固定点/单点位置参数仍可按原有寄存器写入；执行命令时如果目标位置超出允许范围，将以 `PARAM_RANGE_ERROR` 进入错误状态并阻止电机移动。

本次修改：
- 新增单点目标位置校验，统一要求目标位置满足 `blindZone <= target <= tankHeight`。
- 单点测量 `singlePointMeasurementPosition` 在移动前执行校验，超过零点或进入罐底盲区时直接报参数超限。
- 固定点监测 `singlePointMonitoringPosition` 在移动前执行校验，避免零点以上继续上行或进入罐底盲区。
- 瓦锡兰密度分布测后回固定点位置前复用同一校验，目标非法时不再先移动。
- 目标非法报错前打印流程名、原因、目标位置、零点、罐底盲区、当前位置、尺带长度和错误码，便于现场定位。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`
- `cmake --build build\LTD_MAIN_CPU2`
- GBK 源码抽检通过，未发现替换字符或 `??`
- `git diff --check`：本次修改无空白错误；当前工作区存在未提交文件的行尾转换 warning。
- 未做现场联调：仍需台架验证单点测量、固定点监测和瓦锡兰测后回固定点在正常边界、超过零点、进入罐底盲区三类目标下的电机禁止动作和错误码上报。

## 2026-06-03 - 完善启动安全、串口调试状态、电机日志和参数同步

版本：
- CPU2: V1.9.1.0 -> V1.10.0.0
- CPU3: V1.8.0.0 -> V1.9.0.0
- `DEVICE_PROTOCOL_VERSION`: `5u -> 6u`

兼容性：
- 本次新增 CPU2/CPU3 共享 `DeviceState` 语义 `STATE_DEBUG_MODE = 0x0033`，旧 CPU3 无法正确显示该状态，因此提升共享协议版本。
- 不新增保持寄存器或输入寄存器地址，不改变共享寄存器打包顺序；原 `reserved2` 参数槽复用为 `fault_auto_recovery_retry_limit`，后续字段地址不移动。
- `empty_weight` 空载重量寄存器地址和长度不变，32 位内容按 `int32_t` 有符号值解释，负值仍按二进制补码传输。
- CPU2/CPU3 需要成对升级到协议版本 6；协议不匹配时仍由现有兼容检查拦截。

本次修改：
- CPU2 上电默认安全态改为电机驱动禁用、SPI 片选空闲高电平，启动早期执行 `MotorCtrl_BootSafeStop()` 清理 TMC5130 残留运动目标。
- CPU2 AS5145 编码器链路新增首帧有效门控，编码轮记步模式下普通运动和上电默认命令会在位置源未就绪时被拦截；强制调试运动只绕过编码器首帧门控，不绕过驱动安全检查。
- CPU2 串口调试命令中，除调用正式测量流程的 `F/H/J`、正式命令映射、无线滑环 `SP*` 和演示 `X` 之外，执行期间临时显示调试模式，退出时恢复进入前业务状态。
- `B/BE` 诊断进入时切换到 `STATE_DEBUG_MODE`，退出时恢复进入前 `device_state`/`error_code`；诊断期间清除临时错误后仍保持调试模式显示。
- CPU2 整理 B/BE 诊断打印和错误隔离，阶段名改为 `B电机测试`、`BE编码器测试`、`B下行`、`B上行回零`、`BE下行`、`BE上行`，并补充编码器触发偏差、停稳偏差和通信检查中文字段。
- CPU2 按现场反馈撤回 B/BE 专用 10 倍加减速度调整，当前 B/BE 不再读写 `A1/AMAX/D1/DMAX` 斜坡寄存器，继续使用正常程序斜坡配置。
- CPU2 电机初始化日志改为单条 `电机初始化完成` 摘要，合并初始化方式、速度、尺带长度、卷筒周长、VMAX、电流档位和等效微步速度；底层速度换算和 TMC5130 电流写入不再重复打印。
- CPU2 手动速度/电流设置保留单条中文结果打印；急停、慢停和临时速度恢复改走静默路径，避免正常运动流程额外刷屏。
- CPU2 故障自动恢复重跑次数改为参数控制：`0` 关闭自动恢复重跑，`1~10` 为最多自动重跑次数，默认 `3`；旧协议存储升级到协议版本 6 时自动补默认值。
- CPU2/CPU3 将空载重量按 `int32_t` 有符号参数同步和显示，避免负空载称重在保持寄存器和参数打印中被解释为无符号大数。
- CPU3 设备状态页新增 `STATE_DEBUG_MODE` 显示为“调试模式中”，英文为 `Debug Mode`；字库已有“调试模式中”和“自动恢复次数”所需汉字，无需新增点阵。
- CPU3 参数页将原 `reserved2` 菜单项显示为“自动恢复次数”，范围 `0~10`，写入同一保持寄存器地址。
- CPU3 外部 DSM 状态转换层将内部调试模式对外映射为既有维护模式，避免外部 DSM 主站收到未知 `0x0033`。
- 同步补充 CPU2/CPU3 协议版本 6 记录、CPU3 参数菜单与故障自动恢复次数说明、串口调试指令梳理、SI7000 指令映射资料、重启旧运动问题分析、SIL/MISRA 准备资料和 Markdown 转 PDF 工具。

验证：
- `py tools\check_si7000_protocol_contract.py`
- `py tools\check_si7000_modbus_frames.py`
- `cmake --build build\LTD_MAIN_CPU2`，生成 `LTD_MAIN_CPU2_V1.10.0.0.hex`
- `cmake --build build\LTD_DISPLAY_CPU3`，生成 `LTD_DISPLAY_CPU3_V1.9.0.0.hex`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `git diff --check`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

## 2026-06-04 - BE 编码器测试增加速度 m/min 控制、加速度倍率和连续换向重启（CPU2 V1.11.0.0）

版本：
- CPU2: V1.10.0.0 -> V1.11.0.0
- CPU3: 未变化，保持 V1.9.0.0

协议版本/兼容性：
- 本次调整 CPU2 串口 `BE<mm>` 编码器闭环往返调试流程，并同步调整普通运动入口和自动恢复中的 TMC5130 掉电/复位重初始化逻辑。
- 不修改 CPU2/CPU3 共享命令、共享状态、共享参数语义、寄存器映射或 `DEVICE_PROTOCOL_VERSION`。
- `BE<mm>`、`BE<mm>S`、旧式 `BE<mm>,1` 保持兼容；旧式 `BE<mm>,1` 仍表示开启运动中传感器通信。
- 新增 `BE<mm>,<速度m/min>,<加速度倍率>`；速度按 m/min 解析，内部转成 `0.01m/min` 后复用现有 `MotorDriver_ComputeUniformVelocityFromLength()` 换算 `VMAX`，`0` 或省略时沿用当前速度配置；加速度倍率按 `1~20` 限幅。需要运动中传感器通信时可用 `S` 后缀或第三个 `,1`，例如 `BE50,1.5,3,1`。

本次修改：
- 根据现场 DAT 日志复核，`BE300`/`BE150` 多轮下行在未到编码器目标时先出现 `保护行程停稳`，随后往返起点逐轮变大，导致运行区间整体向下漂移。
- `BE` 下行/上行改为速度模式连续换向：到编码器目标后只打印当前位置和偏差，不写 `VMAX=0`，由上层立即下发下一段反向运动。
- `BE` 增加速度 m/min 参数，非 0 时按线速度设置临时换算 `VMAX`，速度限幅使用既有 `MOTOR_LINEAR_SPEED_MIN_X100`/`MOTOR_LINEAR_SPEED_MAX_X100`。
- `BE` 增加加速度倍率，进入测试时读取当前 `A1/AMAX/D1/DMAX` 作为 1 档，按倍率临时写入；命令切换退出时先通过 `MotorCtrl_Init()` 重新初始化回位置模式，再恢复进入前斜坡参数。
- `A/B/BE` 调试命令切换退出时，命令切换检查先等待电机停稳并对齐 `XTARGET=XACTUAL`、切回 `TMC5130_MODE_POSITION`；退出恢复阶段直接调用 `MotorCtrl_Init()` 刷新电机配置，避免 BE 速度模式或减速过程残留影响下一条回零命令。
- `MotorCtrl_Init()` 的非首次初始化从“只刷新电流和 VMAX”改为完整重写 TMC5130 寄存器；非首次不恢复 FRAM 中的 `XACTUAL/XTARGET`，也不恢复位置源，由 `stpr_initStepper()` 重建默认位置寄存器。
- 运行期 TMC5130 掉电/复位调用 `MotorCtrl_InvalidateDriverInit()` 时，只让当前初始化有效性失效，不清掉已初始化标志；后续自动恢复或 BE 重启仍走非首次刷新配置分支，避免误恢复 FRAM 位置和位置源。
- 运行过程中 TMC5130 掉电/复位导致驱动初始化状态失效后，下一次运动入口会先尝试 `MotorCtrl_Init()` 自动重新初始化；初始化成功后继续原业务重试，避免粗找零点等流程直接卡在“电机被禁止”。
- 自动恢复轮询不再只按保存的错误码判断是否初始化电机；只要当前驱动初始化/上电安全状态失效，就先调用 `MotorCtrl_Init()`。错误态停机也不再反复慢停已经失效的驱动，避免持续刷 `停止电机 | 电机被禁止`。
- `MotorCtrl_Init()` 初始化期增加 `DRV_STATUS.CS_ACTUAL` 等待窗口，TMC5130 reset 清除后会等待实际电流建立；超时后再返回 `MOTOR_DISABLED`，交给自动恢复继续等待。
- 回零点粗找/精找内部遇到 TMC5130 通信异常、电机被禁止、充电泵欠压或运行超时后，不再继续执行退让运动，直接退出本轮命令并交给外层自动恢复重新初始化。
- 传感器通信从下行后/上行后的停机点读取，改为在运动过程中按周期读取并打印，通信结果不改变 BE 往返流程。
- 固定初始原点和固定下行目标，往返过程中不因超时、提前停稳或重启重新计算运动区间。
- 单段按距离和有效线速度估算超时；一直未到编码器目标时打印 `到位超时重启`，提前停稳时打印 `提前停稳重启`，重启前先调用 `MotorCtrl_Init()` 成功并重新应用 BE 加速度倍率，再按固定编码器区间重新下发当前方向。

验证：
- 解析 `G:\SAVE2026_06_04_8-54-35.DAT`：确认异常表现为保护行程先于编码器目标停稳，`BE150` 长跑中上行回原点偏差从约 `0.07mm` 增长到约 `83.33mm`。
- `cmake --build build\LTD_MAIN_CPU2`
- GBK 源码按 936 编码读写和抽检，UTF-8 文档抽检未发现替换字符或 `??`。
- `git diff --check`：仅提示 Markdown 工作区后续会被 Git 转为 CRLF，未发现空白错误。
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- 未做现场联调：仍需台架复测 `BE50`、`BE50,1.5,3`、`BE150,0,1,1`、`BE300` 连续往返，确认到位后不停车、区间不漂移、超时/提前停稳只重启当前阶段。

## 2026-06-06 - CPU2 外设驱动迁移到 BSP 目录（CPU2 V1.11.0.1）

版本：
- CPU2: V1.11.0.0 -> V1.11.0.1
- CPU3: 未变化，保持 V1.9.0.0

协议版本/兼容性：
- 本次不修改 CPU2/CPU3 共享命令、共享状态、共享参数语义、寄存器映射或 `DEVICE_PROTOCOL_VERSION`。
- 外设驱动文件内容保持不变，仅从 `LTD_MAIN_CPU2/Drivers/Peripherals` 迁移到 `LTD_MAIN_CPU2/BSP/Peripherals`，头文件名和对外 API 不变。
- 迁移后 `Drivers/` 仅保留 ST HAL/CMSIS 等 CubeMX/CubeIDE 带入内容，项目板级外设驱动归入 `BSP/`，降低后续 CubeMX 再生成和代码统计时的边界混淆。

本次修改：
- 将 CPU2 的 TMC5130、AS5145、CH9141EVT、MB85RS2M 和 AD5421 底层外设驱动整体移动到 `BSP/Peripherals/inc` 与 `BSP/Peripherals/src`。
- 更新 CPU2 CMake 构建脚本，使外设驱动从 `BSP/Peripherals/src` 编译，并从 `BSP/Peripherals/inc` 引入头文件。
- 同步更新 CPU2 相关设计文档、解耦计划和无线滑环匹配记录中的旧路径。
- 按提交检查脚本要求，将 CPU2 build 版本升级到 `V1.11.0.1`，用于追踪本次构建组织调整。

验证：
- `rg` 检查源码、CMake 和 CubeIDE 配置中的 `Drivers/Peripherals` 旧路径，除 `Debug/Release` 历史构建产物和迁移说明文本外无活动引用残留。
- `cmake --build build\LTD_MAIN_CPU2`，确认 `BSP/Peripherals/src/*.c` 已参与编译并生成 `LTD_MAIN_CPU2_V1.11.0.1.hex`。
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

## 2026-06-06 - 新增四路继电器报警输出（CPU2 V1.12.0.0，CPU3 V1.10.0.0）

版本：
- CPU2: V1.11.0.1 -> V1.12.0.0
- CPU3: V1.9.0.0 -> V1.10.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION`: 6 -> 7。
- 本次在 `DeviceParameters` 元信息前追加四路 `RelayAlarmConfig`，每路 13 个 32 位字段，保持寄存器从 `HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE` 开始顺延。
- 同步删除旧 DO 报警 3 个 32 位字段，AO 及后续保持寄存器前移 6 个寄存器；协议版本 7 相对协议版本 6 的保持寄存器净增量为 98 个寄存器。
- 本次在 `MeasurementResult` 末尾追加四路 `RelayAlarmRuntimeState`，输入寄存器从无线滑环匹配状态后顺延，每路 10 个寄存器用于发布 `alarm_value`、HH/H/HH_H/L/LL/LL_L/any/clear 运行态。
- CPU2/CPU3 必须同为协议版本 7 才能正确同步四路继电器配置和运行态；旧协议 CPU3 不具备新增寄存器和菜单，旧协议 CPU2 不执行继电器报警输出配置。
- 原 `AlarmHighDO`、`AlarmLowDO`、`ThirdStateThreshold` 旧 DO 报警接口直接删除；AO、指令参数、尺带补偿、继电器报警输出配置和元信息寄存器整体前移，由协议版本 7 和 `DEVICE_PARAM_VERSION=3` 覆盖布局变化。
- 参数存储版本：CPU2 `DEVICE_PARAM_VERSION` 从 2 升级到 3，`DeviceParameters` 结构大小变化。
- 参数存储兼容性影响：从 CPU2 V1.11.0.1 及更早 `DEVICE_PARAM_VERSION=2` 固件升级到 V1.12.0.0 时，旧 FRAM 参数版本/结构不匹配，会恢复出厂默认参数。
- 现场升级动作：升级到 CPU2 V1.12.0.0 前必须备份或记录现场参数，升级后重新下发并核对继电器报警输出、液位、称重、分布测量和通信参数。

本次修改：
- CPU2 新增 `Services/Relay/relay_output.c/.h`，按参考程序继电器报警输出实现四路继电器报警输出，支持工作模式、输出报警位、接点类型、报警模式、报警取值源、HH/H/L/LL 阈值、滞回、无效值策略和锁存清除。
- CPU2 程序侧按 PG9/PG10/PG11/PG12 驱动 RELAY1~RELAY4；ULN2001D 低边驱动按 GPIO 高电平吸合处理，第四路 MCU IO 已预留，外部驱动和端子待后续原理图补齐。
- CPU2 在 `App_Init` 初始化继电器报警输出；TIM4 中断只置位继电器刷新请求，主循环统一执行继电器报警输出计算和 GPIO 输出，避免中断中读取参数、测量值和执行浮点比较。
- CPU2 参数默认值将四路继电器报警输出配置设为禁用、常开、储罐液位源、阈值/滞回为 `0.0`，并将参数结构版本提升到 `DEVICE_PARAM_VERSION=3`。
- CPU2 将“清报警”字段按运行期命令处理：单独写清报警不触发 FRAM 保存，保存镜像和上电归一化都会清零 `clear_alarm`，批量写配置时也不会把清锁存命令持久化。
- CPU2 继电器刷新前复制单路配置和测量快照，继电器报警输出计算使用同一时刻的数据；每路运行态在本地计算完成后通过短临界区一次提交，避免输入寄存器读取到半更新状态；手动报警抑制期间只释放输出，不清除已锁存的运行态。
- CPU2 将每路继电器报警输出运行态写入 `g_measurement.relay_alarm_runtime[]`，字段和参考程序 `alarm_para_onlyread`/`alarm_para_no_storage` 对齐。
- CPU3 同步新增操作码、参数元数据、保持寄存器映射、参数指针和枚举文字表；参数菜单调整为 `输出配置 -> 继电器报警输出 -> R1/R2/R3/R4 -> 通道设置/报警配置`，`AO输出` 下沉到输出配置页。
- CPU3 删除旧 DO 报警菜单入口、操作码、参数元数据、结构体字段、寄存器打包读写和参数同步映射，不再保留兼容接口。
- CPU3 上电和参数刷新时补读 AO/指令/尺带段以及继电器报警输出配置与元信息段；正常轮询读取无线滑环匹配状态与继电器报警输出运行态尾段，避免新增寄存器未同步。

验证：
- `cmake --build build\LTD_MAIN_CPU2`：通过，已编译新增 `Services/Relay/relay_output.c`，生成 `LTD_MAIN_CPU2_V1.12.0.0.hex`。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过，生成 `LTD_DISPLAY_CPU3_V1.10.0.0.hex`。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过。
- 复核参考程序 `MEASURE/alarm.c/.h`：参考程序报警状态机、无效值策略、HH/H/L/LL 组合、锁存清报警和只读运行态字段已对齐到四路实现。
- 按 GBK/UTF-8 实际编码抽检本次修改源码和文档：通过，未发现替换字符。
- `git diff --check`：未发现空白错误，仅有工作区 LF 后续转换为 CRLF 的 Git warning。
- `py tools\check_version_bumped.py`：提交前暂存后通过。
- 尚未做台架/实物联调：需要现场验证 PG9~PG12 对 RELAY1~RELAY4 的吸合极性、常开/常闭配置、无效值策略和锁存清除；第四路外部驱动/端子需等待原理图补齐后实测。

## 2026-06-06 - 清理正式固件未用代码（CPU2 V1.12.0.1，CPU3 V1.10.0.1）

版本：
- CPU2: V1.12.0.0 -> V1.12.0.1
- CPU3: V1.10.0.0 -> V1.10.0.1

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享寄存器地址、字段含义、命令码、参数解释口径或外部协议响应语义。
- 本次删除的是已确认未用、未进入正式调用链的旧接口、备用调试入口和历史兼容路径；CPU2 写入测量结果、CPU3 读取 CPU2 输入寄存器、SI7000 主分发和 DSM 主协议路径保留。
- 删除源码后 CPU2/CPU3 `.hex` 哈希发生变化，按构建产物变化升级 CPU2/CPU3 build 版本，用于交付追踪和回溯。

本次修改：
- 删除 CPU3 旧 `com_manager.c/.h` 兼容分发模块，保留 `app_main` 现有主分发路径。
- 删除 CPU2 旧 `CH9141EVT.c/.h` AT 初始化备用入口。
- 删除 CPU2 `fault_manager` 旧全局故障接口、水位辅助接口、`AS5145_GetLastOkTick()`、`TMC5130 stpr_readInt()`、输入寄存器反向解析链路、`MotorCtrl_SetSpeed()`、`WIRELESS_Read_IntParam()` 和 `ErrorLog_Report()`。
- 删除 CPU3 OLED 未用接口、CPU3 时钟设置入口、设备状态错误设置、DSM 备用发送/输入寄存器单读写接口、SI7000 站号 get/set、CPU2 通信初始化、测量结果写输入寄存器和单参数同步入口。
- 调整 `tools/check_si7000_protocol_contract.py`，继续强制校验保留的 CPU2 写入测量结果和 CPU3 读取 CPU2 输入寄存器方向；对已删除的 CPU2 反向读取和 CPU3 写入死接口改为仅在源码存在时校验。
- 同步记录当前文档和资料整理改动，包括需求计划 PDF、自动恢复需求 PDF 替换，以及 SIL 功能安全资料 PDF。

验证：
- `rg` 检查确认删除符号在 `.c/.h` 中清零；仅保留 CPU2 正常写输入寄存器和 CPU3 正常读取 CPU2 输入寄存器接口。
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_si7000_modbus_frames.py`
- `py tools\check_si7000_protocol_contract.py`
- `git diff --check`
- 暂存后运行 `py tools\check_version_bumped.py`
- 已对比删除前后 CPU2/CPU3 `.hex` 哈希，确认构建产物变化，因此本次升级 build 版本；尚未做现场实物联调。

## 2026-06-08 - 优化电机停止判定并整理继电器报警输出文档（CPU2 V1.12.0.2，CPU3 V1.10.0.2）

版本：
- CPU2: V1.12.0.1 -> V1.12.0.2
- CPU3: V1.10.0.1 -> V1.10.0.2

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享寄存器地址、命令码、参数字段顺序或参数解释口径。
- CPU2 电机停止判定由布尔返回调整为错误码加输出参数，TMC5130 通信失败会向上返回具体错误码，不再把读取失败误判为已停止。
- CPU3 仅调整继电器报警输出菜单、注释和文档命名口径，不改变协议布局和菜单层级入口语义。

本次修改：
- CPU2 统一 `MotorCtrl_IsDriverMoving()` / `MotorDriver_ReadMovingState()` 运动状态读取接口，综合 `RAMPSTAT.vzero`、二次确认、`VACTUAL` 和目标位置差值判断停止状态。
- CPU2 在 Wartsila 密度测量、阻塞移动、目标停止等待、运行期位置轮询和液位传感器电机停止判断中传递运动状态读取错误，避免通信异常被吞掉。
- CPU2 停止等待和显示状态刷新复用统一运动状态读取逻辑，减少重复推断路径。
- CPU2/CPU3 将“四路继电器方式2”相关注释、菜单文案、协议记录和改动方案统一整理为“四路继电器报警输出”口径。
- 新增 CPU2 电机程序与函数梳理 HTML，补充 SIL 功能安全认证资料和 README 索引，整理相关文档引用。

验证：
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- 尚未做现场实物联调；需后续验证 TMC5130 通信异常、目标位置容差停止判定、继电器报警输出菜单文案和 SIL 文档资料引用。

## 2026-06-08 - 优化CPU3菜单选择型参数编辑（CPU3 V1.10.1.0）

版本：
- CPU2: 保持 V1.12.0.2
- CPU3: V1.10.0.2 -> V1.10.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享寄存器地址、字段数值编码、参数存储结构或外部协议响应语义。
- 本次只改变 CPU3 OLED 参数菜单的显示和编辑交互：有限枚举、开关和模式类参数由数字逐位输入优化为上下键候选选择。
- “非法配置”仅作为异常值显示占位，不作为可写候选项；水位测量方式因未确认独立枚举语义，仍保持原数字编辑路径。

本次修改：
- CPU3 将故障自动回零、故障停止测量、位置源自动切换、记步模式、是否测罐底/水位/单点密度、分布测顺序/模式、数据源、密度手输上传、是否息屏、更新罐高标志和罐底后编码器修正等参数挂接到选择型文字显示和编辑流程。
- CPU3 新增位置源自动切换和记步模式候选文字表，分别显示为禁用/启用、编码器/电机。
- CPU3 优化选择页进入时的高亮位置，默认选中当前参数值；保存或返回后清理选择状态，避免下次进入沿用上一次菜单位置。
- CPU3 为枚举候选表异常增加空指针和候选数防护，避免 `dtm_disarr()` 无映射时进入空菜单。

验证：
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，未发现新增显示文字缺字。
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --check`：未发现空白错误，仅有工作区 LF 后续转换为 CRLF 的 Git warning。
- 尚未做实物按键联调；需现场确认进入选择型参数时当前值高亮、上下键循环、确认写回、返回取消，以及 COM 协议可选择到 SI7000 但不能选择“非法配置”。

## 2026-06-09 - 修复电机停止等待被目标位置差值卡死（CPU2 V1.12.1.0）

版本：
- CPU2: V1.12.0.2 -> V1.12.1.0
- CPU3: 保持 V1.10.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享寄存器地址、字段数值编码、参数存储结构或外部协议响应语义。
- 本次只修复 CPU2 电机停止等待逻辑：普通运动完成判定继续检查 `XTARGET/XACTUAL` 差值，停止命令后的刹停等待改为只依据 `RAMPSTAT.vzero` 与 `VACTUAL` 判断物理停止。

本次修改：
- 新增停止专用 `MotorDriver_ReadStoppingState()`，用于停止命令后的减速等待，避免旧 `XTARGET` 与减速滑行后的 `XACTUAL` 差值导致一直判定运动中。
- `MotorDriver_StopAndMarkStopped()` 在确认物理停止后重新把 `XTARGET` 对齐当前 `XACTUAL`，避免恢复速度或重新使能后旧目标继续生效。
- `MotorMotion_WaitStoppedAfterStopCommand()` 改用停止专用状态读取；普通移动等待仍使用带目标位置差值的运动判定，保留防提前到位保护。

验证：
- `git diff --check`
- `py tools\check_version_bumped.py`
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- 尚未做现场实物联调；需复测零点测量到达零点后的刹停、命令切换停止、TMC5130 通信异常和普通长距离运动到位判定。

## 2026-06-09 - 优化 CPU3 参数菜单、协议联动和状态页显示（CPU2 V1.12.1.1，CPU3 V1.11.0.0）

版本：
- CPU2: V1.12.1.0 -> V1.12.1.1
- CPU3: V1.10.1.0 -> V1.11.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享寄存器地址、字段顺序、命令码、参数存储结构或外部协议响应语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和字段顺序不变，不触发旧 FRAM 参数恢复出厂。
- 本次改变 CPU3 OLED 菜单组织、选择项显示、CPU3 本机串口参数归一化、保存前确认流程和状态页显示条件；旧 CPU2 固件只要协议版本仍为 7，寄存器交互保持兼容。
- CPU3 本机 FRAM 参数结构未新增字段；旧存储读取后会把串口协议预留值收敛为计量仪协议，并按协议自动套用推荐串口参数。

本次修改：
- CPU2 同步调整参数注释、串口打印和称重/探底相关中文口径，将“重量/罐底”等容易混淆的名称统一为“称重/探底”等现场口径；不改变计算逻辑、默认值、参数布局或保存格式。
- 参数配置首页调整为“测量参数、输出配置、通信设置、显示设置、维护设置”，运行策略并入测量参数，设备信息和参数校验并入维护设置。
- 标定液位值、修正液位值、标定水位值、标定罐高值、单点测量位置、单点监测位置和电机调试位置继续随指令输入，不再作为参数配置菜单项展示。
- 带参指令菜单项和下发确认页只显示指令名，不在指令名后追加参数值；参数值仅在输入页和参数写入确认页显示。
- 水位测量方式改为选择式编辑，屏幕显示为“低速模式/快速模式”，写入范围收敛为 0/1；CPU2 侧仍兼容原 0/非0 语义。
- COM 协议菜单只显示计量仪、瓦锡兰、LTD、SI7000 四个有效选项；SI7000 仍保存为真实协议值 5，不再向现场暴露预留协议项。
- 写任一 COM 串口参数后按协议自动收敛串口配置：计量仪/DSM 4800 8N1，瓦锡兰 4800 8N1，LTD 115200 8N1，SI7000 9600 8O1。
- 对运行策略、电机/编码换算、罐高/罐底、水位关键阈值、通信串口参数和恢复出厂设置增加“参数保护”额外确认页；不新增维护权限或更高等级密码。
- 继电器 R1~R4 子页新增只读“报警状态”入口，显示报警值、HH/H/HH-H/L/LL/LL-L、任意报警和清锁存运行态；数据来自 CPU2 输入寄存器快照，不触发参数写入。
- CPU3 状态页新增显示上下文归类，密度、温度、液位、水位、频率、电容和罐高按设备状态选择数据源，避免待机、运动、维护或故障状态混入旧业务结果。
- 位置固定取 `debug_data.sensor_position`，称重固定取 `debug_data.current_weight`，允许位置或称重为 `0` 时仍在正常状态页路径显示。
- CPU3 菜单列表、参数详情标题、下发确认页、参数保护页和选择项列表按 OLED 单行宽度使用短名或裁剪，避免长中文/英文名称越过单行显示边界。
- 同步更新 CPU3 状态页不同状态显示信息确认表。

验证：
- 新增屏幕文字字库检查通过：参数保护、请确认、确认保存、低速模式、快速模式等均已覆盖。
- 继电器报警状态页新增屏幕文字字库检查通过，未发现缺字。
- CPU3 菜单显示裁剪和带参指令确认页调整后重新构建通过。
- `param_meta[]` 可见参数分组覆盖检查通过，非保留、非内部设备指令参数未发现漏分组。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，StockMap 与点阵顺序一致，OLED 显示缺字数为 0。
- `git diff --check`
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_version_bumped.py`
- 尚未做实物联调；需现场确认新菜单层级、危险参数返回/确认流程、COM 协议切换后串口参数刷新、水位方式 0/1 写回，以及状态页在待机、故障、初始化、读参数、恢复出厂、运动完成和各测量完成状态下的分页与显示内容。

## 2026-06-09 - 修正 CPU3 只读版本与校验值显示格式（CPU3 V1.11.1.0）

版本：
- CPU2: 保持 V1.12.1.1
- CPU3: V1.11.0.0 -> V1.11.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享寄存器地址、字段顺序、命令码、参数存储结构或外部协议响应语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和字段顺序不变，不触发旧 FRAM 参数恢复出厂。
- 本次只修正 CPU3 OLED 对既有只读字段的显示格式，参数读写、保存、同步和 CPU2 侧运行逻辑均不改变。

本次修改：
- CPU3 参数配置列表页和详情页中，传感器软件版本、CPU2 程序版本、CPU3 程序版本统一按 `Vx.x.x.x` 显示，不再按十进制整数或旧字符串路径混用显示。
- 魔术字和参数 CRC32 统一按 `0xXXXXXXXX` 大写 HEX 显示，便于现场与 CPU2 调试打印、FRAM 元信息和 CRC 校验结果核对。
- 补充设备信息、参数校验和 CPU3 本机版本字段的短名映射，避免 `传感器软件版本:Vx.x.x.x`、`Magic:0xXXXXXXXX` 等组合超过 OLED 单行宽度。
- 同类问题排查结论：协议版本、参数版本号和结构体大小仍保持十进制显示；未发现其他版本编码或 32 位校验/魔术字类只读字段需要 HEX/版本格式化。

验证：
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- 尚未做实物按键联调；需现场进入维护设置/设备信息和维护设置/参数校验，确认列表页、详情页均按版本号和 HEX 口径显示且不越界。

## 2026-06-09 - 同步系统参数出厂默认值（CPU2 V1.12.1.2）

版本：
- CPU2: V1.12.1.1 -> V1.12.1.2
- CPU3: 保持 V1.11.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION`: 保持 7。
- 不改变保持寄存器地址、输入寄存器地址、字段顺序、命令码、`DeviceParameters` 结构大小或参数存储格式。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`struct_size` 不变，从 V1.12.1.1 升级到 V1.12.1.2 不会因参数存储版本触发恢复出厂。
- 新默认值只在恢复出厂、FRAM A/B 均无效或首次写入参数区时生效；已有现场参数会正常保留。

本次修改：
- 根据《LNG计量仪屏幕菜单.docx》同步 11 项 CPU2 恢复出厂默认值：故障自动回零、位置源自动切换、碰撞上下限比率、液位探头距差、液位盲区、探底称重阈值、实测罐高最大偏差、区间测量上下限和瓦锡兰探底间隔。
- 位置源自动切换非法值和瓦锡兰探底间隔越界值的运行期回退值同步改为新默认值。
- 修正 `LNG计量仪屏幕菜单.docx` 中瓦锡兰探底间隔默认值说明为 `0（不探底）`。
- 同步更新系统参数出厂默认值、参数存储升级清单和版本改动与测试方案。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build\LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`：通过，配置显示 CPU2 固件版本 `V1.12.1.2`。
- `cmake --build build\LTD_MAIN_CPU2`：通过，生成 `LTD_MAIN_CPU2_V1.12.1.2.hex`。
- `git diff --check`：通过。
- `python-docx` 读回《LNG计量仪屏幕菜单.docx》瓦锡兰探底间隔行，默认值为 `0（不探底）`。

## 2026-06-09 - CPU3 继电器报警阈值按取值源显示单位（CPU3 V1.11.1.1）

版本：
- CPU2: 保持 V1.12.1.2
- CPU3: V1.11.1.0 -> V1.11.1.1

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不改变保持寄存器地址、字段顺序、命令码、`DeviceParameters` 结构大小、CPU2 参数存储格式或范围校验规则。
- 本次仅改变 CPU3 OLED 参数菜单的单位显示，不改变参数写入值、倍率或 CPU2 执行逻辑。

本次修改：
- CPU3 参数列表、参数详情页和参数输入页显示继电器报警阈值、报警滞回时，按同一路“报警取值源”动态选择单位。
- 报警取值源为储罐液位、水位、浮子位置时显示 `mm`；报警取值源为液相温度时显示 `℃`；报警取值源为无或非法值时不显示静态单位。
- 保持 R1~R4 阈值和滞回的 `param_meta[]` 静态单位为空，避免把源相关参数误固定为单一物理单位。
- 同步更新 CPU3 参数单位与范围补充清单、界面文档索引和版本改动与测试方案。

验证：
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，未发现缺字。
- `cmake -S LTD_DISPLAY_CPU3 -B build\LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`：通过，配置显示 CPU3 固件版本 `V1.11.1.1`。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过，生成 `LTD_DISPLAY_CPU3_V1.11.1.1.hex`。
- `git diff --cached --check`：通过。
- `py tools\check_version_bumped.py`：通过，确认 CPU3 已升版。

## 2026-06-09 - CPU3 参数菜单单位倍率与手输值存储修正（CPU3 V1.11.1.2）

版本：
- CPU2: 保持 V1.12.1.2
- CPU3: V1.11.1.1 -> V1.11.1.2

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不改变 CPU2/CPU3 共享寄存器地址、命令码、CPU2 `DeviceParameters` 结构、CPU2 参数存储版本或 CPU2 执行逻辑。
- CPU3 本地显示/通信参数结构变更，`CPU3_PARAM_VERSION` 从 `0x0002` 升至 `0x0003`；升级后 CPU3 本地显示/通信参数会按默认值重建，避免旧结构误读。

本次修改：
- CPU3 参数菜单补充电机限速 `m/min`、液位找液/滞后阈值 `Hz`、水位电容阈值 `pF`、零点电容 `pF`、探底角度阈值 `°` 等已确认单位。
- 修正液位找液阈值和液位滞后阈值的小数位：两者在 CPU2 算法中直接按整数 Hz 频率差比较，CPU3 菜单不再按 1 位小数显示。
- 将“磁通量D/磁通量T”文案改为“密度修正/温度修正”，并按算法口径显示 `kg/m3`、`℃`，小数位保持 1 位。
- 修正密度手输值口径为 `kg/m3 x10`，范围改为 `0.0~2000.0 kg/m3`，不再显示为 `0.000~2.000 kg/m3`。
- 将 CPU3 本地液位/水位/密度/温度手输值存储字段从 `uint8_t` 改为 `int32_t`，避免菜单允许的大范围输入被截断。
- 同步更新 CPU3 参数单位与范围补充清单、系统参数出厂默认值和版本改动与测试方案。

验证：
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --check`
- `py tools\check_version_bumped.py`
- 尚未做实物按键联调；需现场确认参数列表页、详情页、输入页单位倍率显示，以及 CPU3 本地参数版本升级后的默认值重建行为。

## 2026-06-10 - 同步系统参数默认值和 CPU3 状态页矩阵（CPU2 V1.12.1.3 / CPU3 V1.11.1.3）

版本：
- CPU2: V1.12.1.2 -> V1.12.1.3
- CPU3: V1.11.1.2 -> V1.11.1.3

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不改变 CPU2/CPU3 共享寄存器地址、输入寄存器地址、字段顺序、命令码、`DeviceParameters` 结构大小或参数语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`struct_size` 不变，从 V1.12.1.2 升级到 V1.12.1.3 不会因参数存储版本触发恢复出厂。
- CPU3 本地参数结构和 `CPU3_PARAM_VERSION` 不变，从 V1.11.1.2 升级到 V1.11.1.3 不会触发 CPU3 本地显示/通信参数重建。
- CPU2 新默认值只在恢复出厂、FRAM A/B 均无效或首次写入参数区时生效；已有现场参数会正常保留。

本次修改：
- 根据《LNG计量仪屏幕菜单.docx》当前工作区默认值，再同步 9 项 CPU2 恢复出厂默认值：空载称重上限、满载称重下限、液位盲区、水位盲区、水位跟随电容阈值、水位寻找电容阈值、水位滞后电容阈值、最高点距液面和最低点距罐底。
- `water_lag_cap_threshold == 0` 的旧存储补默认值从 `80000` 同步为 `30000`，与恢复出厂默认值一致；非零现场值不被覆盖。
- 按 `CPU3状态页显示参数矩阵.xlsx` 同步状态页显示逻辑：`STATE_WARTSILA_DENSITY_MEASURING` 显示平均密度和平均温度，但不显示分布液位。
- `STATE_READPARAMETEROVER` 新增读取参数完成上下文，按有效数据分页显示液位、水位、平均密度、平均温度、频率、电容、X/Y 角和罐高。
- 将频率和电容拆为两个独立状态页参数项，读取参数完成时两者可同时显示。
- `STATE_ERROR` 状态页除状态、错误码、位置、称重外，新增故障详情行；故障原因过长时沿用既有两行拆分逻辑。
- 同步更新系统参数出厂默认值、CPU2 参数存储升级清单、CPU3 参数单位与范围补充清单、CPU3 状态页确认表、状态页显示参数矩阵工作簿、界面/协议/版本文档索引和合并后的版本改动与测试方案。
- 《LNG计量仪屏幕菜单.docx》中“尺带厚度”行仅带“改成选项尺带类型”备注，未给出新的数值或字段设计，本次不改变程序默认值和菜单结构。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`：通过，配置显示 CPU2 固件版本 `V1.12.1.3`。
- `cmake --build build\LTD_MAIN_CPU2`：通过，生成 `LTD_MAIN_CPU2_V1.12.1.3.hex`。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，未发现缺字。
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`：通过，配置显示 CPU3 固件版本 `V1.11.1.3`。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过，生成 `LTD_DISPLAY_CPU3_V1.11.1.3.hex`。
- `git diff --cached --check`：通过。
- `py tools\check_version_bumped.py`：通过。
- 已通过 `python-docx` 对比《LNG计量仪屏幕菜单.docx》当前工作区与 `HEAD` 的出厂默认值变化，确认本次程序同步范围。
- 尚未做实物恢复出厂、旧 FRAM 升级、水位实测和 OLED 翻页联调；需现场确认读取参数完成、LTD 密度分布测量中、故障态三类页面的实际显示顺序和翻页体验。

## 2026-06-10 - 恢复磁通量D/T菜单显示并补充菜单文档选项含义（CPU3 V1.11.1.4）

版本：
- CPU2: 保持 V1.12.1.3
- CPU3: V1.11.1.3 -> V1.11.1.4

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不改变 CPU2/CPU3 共享寄存器地址、命令码、参数存储结构、字段倍率或算法计算口径。
- CPU3 仅恢复两个参数菜单显示名：`密度修正/温度修正` 改回现场确认的 `磁通量D/磁通量T`。
- CPU3 本地参数结构和 `CPU3_PARAM_VERSION` 不变，从 V1.11.1.3 升级到 V1.11.1.4 不会触发 CPU3 本地显示/通信参数重建。

本次修改：
- 将 CPU3 参数元数据中的 `COM_NUM_DEVICEPARAM_DENSITYCORRECTION` 和 `COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION` 显示名恢复为 `磁通量D`、`磁通量T`。
- 优化《LNG计量仪屏幕菜单.docx》：选项型参数默认值在数字后补充选项含义，例如 `0（否）`、`0（禁用）`、`0（8位）`。
- 同步《LNG计量仪屏幕菜单.docx》中的 CPU2/CPU3 版本、单位/小数位、找油/找水阈值、修正倍率说明和密度手输值口径。

验证：
- `python-docx` 结构化读取《LNG计量仪屏幕菜单.docx》，确认 `磁通量D/T`、选项型默认值含义和 CPU3 版本号已更新。
- `git diff --check -- LTD_DISPLAY_CPU3/Application/system_param/system_parameter.c`：通过。
- 尚未重新构建 CPU3 固件；本次代码改动仅涉及菜单参数显示名，提交前已通过版本检查脚本约束升版范围。
