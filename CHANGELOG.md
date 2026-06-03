# 升级日志

记录 CPU2/CPU3 固件版本变更。使用 `tools/bump_version.py` 升级版本时会自动追加记录；提交前应补充到与 Git 提交信息同等详细。

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
