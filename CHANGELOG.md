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
| V1.12.1.1 | 3 | 存储版本不变；仅调整参数、打印和文档中的扭力/探底等中文口径，通常保留旧参数 |
| V1.12.1.2 | 3 | 存储版本不变；同步 11 项恢复出厂默认值，旧 FRAM 参数通常保留，新默认值仅在恢复出厂或 FRAM 无效时生效 |
| V1.12.1.3 | 3 | 存储版本不变；继续同步恢复出厂默认值和状态页矩阵，旧 FRAM 参数通常保留 |
| V1.12.2.0 | 3 | 存储版本不变；读取部件参数完成态改为持续刷新，`debug_data` 水位字段改为水位电容快照，旧 FRAM 参数通常保留 |
| V1.13.0.0 | 3 | 存储版本不变；新增点动长距离运动接口和 BJ/BJP 本地串口测试命令，旧 FRAM 参数通常保留 |
| V1.13.0.1 | 3 | 存储版本不变；拆分串口测试命令到 `test.c` 并整理构建流程，旧 FRAM 参数通常保留 |
| V1.13.1.0 | 3 | 存储版本不变；优化瓦锡兰分布测量点间移动、空气点液位识别和失败写回语义，旧 FRAM 参数通常保留 |
| V1.13.2.0 | 3 | 存储版本不变；修复电机绝对目标越界保护和参数错误自动恢复过滤，旧 FRAM 参数通常保留 |
| V1.14.0.0 | 3 | 存储版本不变；新增密度连续、连续相对频率和连续定频找液位方式，旧 FRAM 参数通常保留 |
| V1.14.0.1 | 3 | 存储版本不变；优化电机点动与位置模式运动流程，旧 FRAM 参数通常保留 |
| V1.15.0.0 | 3 | 存储版本不变；新增读取部件参数蓝牙 RSSI 快照、协议版本 9 和 CPU3 菜单显示优化，旧 FRAM 参数通常保留 |
| V1.16.0.0 | 3 | 存储版本不变；新增 AO 模拟电流输出运行态和 AO 输出使能，原 `reserved26` 语义改为 `AoOutputEnable`；旧存储升级到协议版本 10 时默认关闭 AO 输出，并保留启动阶段 AO/电机初始化错误 |
| V1.16.0.1 | 3 | 存储版本不变；收紧罐底检测模式为 `0/1`，旧 FRAM 中异常值运行期归零，通常保留旧参数 |
| V1.16.1.0 | 3 | 存储版本不变；修复继电器报警 float 阈值写入并补全参数打印，旧 FRAM 参数通常保留 |
| V1.16.2.0 | 3 | 存储版本不变；固定点、密度和瓦锡兰移动改用 Jog 两段减速定位，旧 FRAM 参数通常保留 |
| V1.17.0.0 | 3 | 存储版本不变；旧无线主机/从机探测改为 CH9141K 蓝牙主从机状态查询并优化 CPU3 菜单，旧 FRAM 参数通常保留 |
| V1.18.0.0 | 3 | 存储版本不变；修复零点标定脱离和 AO 正式回读刷新，旧 FRAM 参数通常保留 |
| V1.18.1.0 | 3 | 存储版本不变；修复继电器液位/温度报警数据源有效性和 CPU3 息屏开关运行期同步，旧 FRAM 参数通常保留 |
| V1.18.1.1 | 3 | 存储版本不变；协议版本升至 11，AO 液位量程、正常电流端点、独立报警阈值和特殊 AO 电流范围按新语义运行期归一化，旧 FRAM 参数通常保留并补齐 AO 液位量程默认值 |
| V1.19.0.0 | 3 | 存储版本不变；命令 115 改为保留、修复液位/水位标定固定偏差、读取部件参数增加连接 MAC 快照并统一液位/扭力口径，旧 FRAM 参数通常保留 |
| V1.20.0.0 | 3 | 存储版本不变；协议版本升至 13，内部密度 raw 升级到 `kg/m3 x100`，旧协议 FRAM 密度参数按运行期迁移，旧 FRAM 参数通常保留 |
| V1.20.1.0 | 3 | 存储版本不变；参数打印、Modbus 写参差异、分布密度悬停单位和初始化日志整理，旧 FRAM 参数通常保留 |
| V1.20.2.0 | 3 | 存储版本不变；编码器通信诊断、传感器上电 AT 干扰过滤、全局错误归因和中断优先级调整，旧 FRAM 参数通常保留 |
| V1.20.3.0 | 3 | 存储版本不变；AS5145 SSI 错误重试、持续故障一次性上报和有效帧门控调整，旧 FRAM 参数通常保留 |
| V1.21.0.0 | 3 | 存储版本不变；协议版本升至 14，原 `reserved30~reserved33` 复用为 SI profile 首点、步距、停留和探底频次参数，旧 FRAM 参数通常保留并在协议迁移或运行期归一化补默认值 |
| V1.21.1.0 | 3 | 存储版本不变；修复 SI Profile 点位输出、空气点判定和无检测运行通信策略，旧 FRAM 参数通常保留 |
| V1.21.2.0 | 3 | 存储版本不变；SI Profile 成功完成后自动排队找液位并恢复液位跟随，旧 FRAM 参数通常保留 |
| V1.21.3.0 | 3 | 存储版本不变；优化液位跟随滞后确认、方法 0/1 跟随重找保持跟随态和方法 5 定频速度闭环边界，旧 FRAM 参数通常保留 |
| V1.21.4.0 | 3 | 存储版本不变；修复编码轮模式运行期速度补偿参考长度，旧 FRAM 参数通常保留 |
| V1.21.5.0 | 3 | 存储版本不变；修复 AD5421 断环重接恢复与 AO 运行态错误处理，旧 FRAM 参数通常保留 |
| V1.21.6.0 | 3 | 存储版本不变；修复 DSM 传感器编号响应解析、无符号 32 位溢出和上电蓝牙持续收包永久等待，并清理未用代码，旧 FRAM 参数通常保留 |
| V1.22.0.0 | 3 | 存储版本不变；新增 CPU2 串口命令严格收帧、解析、查询和停止入口，未改变 `DeviceParameters` 布局、元信息或 CRC 范围，旧 FRAM 参数通常保留 |
| V1.23.0.0 | 3 | 存储版本不变；故障码协议升级到 15 并完善底层诊断，未改变 `DeviceParameters` 布局、元信息或 CRC 范围，旧 FRAM 参数通常保留 |
| V1.24.0.0 | 3 | 存储版本不变；新增 `SAFE_SENSOR=14` 和设备侧安全传感器协议栈，复用既有 u32 `sensorType` 存储位置，未改变结构大小、元信息或 CRC 范围，旧 FRAM 参数保留；降级前需把传感器类型恢复为旧固件支持值 |
| V1.25.0.0 | 3 | 存储版本不变；共享故障码按二代计量仪责任域重新分类、删减并重编号，未改变 `DeviceParameters` 布局、元信息或 CRC 范围，升级不会清除原有参数 |
| V1.26.0.0 | 3 | 存储版本不变；协议18只在输入寄存器追加SI Profile生命周期字段，不进入`DeviceParameters`，升级不会清除原有参数 |
| V1.27.0.0 | 3 | 存储版本不变；协议20的AO配置原位复用既有连续13个32位槽并按旧协议标记迁移，协议21代际字段只追加在输入寄存器尾部；`DeviceParameters`大小、元信息和CRC范围不变，升级不清其它现场参数 |
| V1.28.0.0 | 3 | 存储版本不变；协议23继续原位复用AO 13个槽，把旧故障模式迁移为两种故障动作、原上电电流解释为非跟随电流，`DeviceParameters`大小、元信息和CRC范围不变，升级不清其它现场参数 |
| V1.29.0.0 | 3 | 存储版本不变；协议24把输出源1改为传感器位置、非跟随电流改为初始电流，并增加仅驻留RAM的成功过程目标缓存；结构、元信息和CRC范围不变，升级不清其它现场参数，但需人工复核旧输出源1及量程 |
| V1.29.1.0 | 3 | 存储版本不变；修复分布点阵原子发布和来源锁存，未改变`DeviceParameters`结构、默认值、元信息或CRC范围，升级不清现场参数 |
| V1.30.0.0 | 3 | 存储版本不变；协议25把原`reserved3`槽位定义为水位滞后时间预留参数，结构、元信息和CRC范围不变；协议24及更早升级时把该槽归零，不清其它现场参数 |

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
- 读取部件参数拆分为正常命令和恢复检查两种入口；恢复检查不切换设备状态，读取部件参数时扭力只判断自身通信是否超时。
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
- 补充电机、编码器、传感器、滑环通信、扭力、参数存储和测量流程中的重试、恢复和告警日志。
- 优化错误码名称、模块名和原因映射，补齐滑环主机/从机、TMC5130、编码器、扭力等错误含义。
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
- 新增长等待公共入口 `AbortableDelay_CommandSwitch()`，让固定点监测、液位/密度模式稳定等待、扭力稳定等待等流程可响应命令切换。
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
- 说明：粗找罐底前增加离底确认，精找罐底离底上行改为无扭力检测，避免已触底状态直接误判完成或误触发扭力碰撞报错

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
- 说明：修复LTD/V2模式切换和部件参数读取误报13-13；DSM首字母E/e仅提示传感器电压过低；优化固定点监测退出、瓦锡兰分布测首点定位和测后探底日志；找罐底固定距离上行改为无检测上行，避免脱离罐底时误报扭力错误；更新LTD故障代码表和测试记录。

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

## 2026-05-19 - 增加罐底下行保护和回零扭力判定

版本：
- CPU2: V1.7.2.0 -> V1.7.3.0
- CPU3: 未变化，保持 V1.5.0.0

协议版本/兼容性：
- 本次仅修改 CPU2 罐底测量保护和扭力碰撞判定逻辑。
- 不修改 CPU2/CPU3 共享协议、寄存器映射、命令码或参数布局，`DEVICE_PROTOCOL_VERSION` 不变。
- `maxDownDistance` 继续沿用原参数含义，不新增参数，不改变已有参数地址。

本次修改：
- 罐底粗找和精找每轮下行前增加最大尺带长度保护，限制值为 `tankHeight + maxDownDistance`。
- 当尺带长度超过最大允许位置且仍未识别到罐底时，立即快速停机并返回 `MEASUREMENT_WEIGHT_DOWN_FAIL`，避免罐底测量无限下行。
- 罐底粗找和精找在传感器位置低于 1m 时，将下行速度上限压到 0.50m/min，并打印传感器位置、原速度和实际下发速度。
- 回零/标零上行时，扭力超过零点阈值视为正常到零点信号，不再被通用防撞逻辑抢先误报 18-3。
- 新增《罐底测量下行保护改动说明》文档，记录问题背景、修改点、现场日志和验证建议。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE_temp_bottom_zero_commit/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`
- `git diff --check`
- `git diff --cached --check`

<## 2026-05-16 - 适配 SI协议和共享协议版本 4

版本：
- CPU2: V1.7.3.0 -> V1.8.0.0
- CPU3: V1.5.0.0 -> V1.6.0.0

兼容性：
- 本次将 SI 所需补充状态融合进 CPU2/CPU3 既有测量结构，`DEVICE_PROTOCOL_VERSION` 从 3 升级到 4。
- CPU2/CPU3 必须同为协议版本 4，CPU3 才能完整获得外部协议转换所需的 profile 完成、探底参考、液位到达和偏差报警状态。

本次修改：
- 合并 `wip/si-protocol-assist` 中的外部协议辅助状态、CPU2 测量状态维护、CPU3 外部 SI Modbus 从站基础实现和协议映射文档。
- 收紧 CPU2/CPU3 职责边界：CPU2 只保留通用业务状态和命令，SI 地址、线圈、缩放、影子寄存器和异常响应集中在 CPU3 转换层。
- CPU3 主分发路径和 `com_manager` 兼容路径统一调用 `si_modbus_process_for_dispatch()`，避免异常响应帧处理语义重复。
- CPU3 新增 SI协议选择项，并接入 COM1/COM2/COM3 协议分发。
- CPU3 新增本地 RTC 时钟接口，SI `30011-30013` 返回当前时分秒，profile 完成计数变化时锁存 `30007-30010` 月日时分。
- SI `FC05` 写线圈增加模式/动作影子区互斥，避免连续写入后读回多个互斥命令位。
- SI `FC05` 对协议保留线圈写入返回非法地址，避免 PLC 误写保留位时收到成功 echo。
- SI profile 时间戳在 RTC 暂不可读时允许后续读寄存器继续尝试锁存，避免偶发初始化窗口丢失时间戳。
- SI `FC06` 对自动 profile 使能、小时、分钟做基础值域检查，非法值返回 Modbus 异常而不进入影子寄存器。
- SI `FC06` 对 `40004-40009` 保留寄存器写入返回非法地址，读取仍保持 0。
- SI profile 点阵只输出 `Number Of Points` 范围内的有效测点，范围外保持 0，避免 PLC 读到旧 profile 残留数据。
- SI 输入寄存器按协议单位输出位置、液位、温度和密度；密度由内部 `kg/m3 x10` 转为协议 `kg/m3 x100`，超出 16 位时钳位。
- SI `10012` 和阈值报警位由 `40011`、`40014-40021` 影子寄存器合成，阈值为 0 时视为未启用。
- 修正 CPU3 串口有校验位时的 WordLength 配置，支持 SI协议要求的 9600 8O1，并在 SI协议选中后自动锁定端口配置。
- 更新 SI协议执行计划、兼容映射表、CPU2 暂存区逐文件改动整理和 CPU2/CPU3 协议变更记录。

验证：
- `git diff --check`
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_si_modbus_frames.py`
- `py tools\check_si_protocol_contract.py`
- `py tools\check_version_bumped.py`

## 2026-05-23 - 调整串口 B/BE 低检测执行流程

版本：
- CPU2: V1.8.0.0 -> V1.8.1.0
- CPU3: 未变化，保持 V1.6.0.0

协议版本/兼容性：
- 本次仅调整 CPU2 串口 B/BE 调试指令执行流程，不修改 Modbus、SI、CPU2/CPU3 共享寄存器映射、参数存储布局或协议版本。
- B/BE 属于现场调试入口，兼容既有命令格式：`B<mm>`、`BE<mm>`、`S` 后缀和 `,1` 后缀仍可使用。

本次修改：
- `process_command()` 中 B/BE 分支前置到通用 `MeasureStart()` 之前，避免 B/BE 被测量初始化、扭力、电机健康检查或历史错误码拦截。
- B/BE 新增专用低检测执行路径：初始化阶段只尽量写 TMC5130 基础配置，运动阶段直接写 `RAMPMODE`、`VMAX`、`XTARGET`，不再调用 `MotorCtrl_Init()`、`MotorCtrl_MoveNoWait()` 或 `stpr_waitMove()`。
- B 指令只负责按电机模型下发往返运动；运行期下发失败、停止状态读取失败等只打印并重试，不检测编码器、扭力、过热等业务错误。
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
- 待硬件现场验证：B/BE 连续往返、没插扭力时 BE 仍持续运行、S 后缀通信打印不置错、运动停稳后再下发下一段。

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
- `empty_weight` 空载扭力寄存器地址和长度不变，32 位内容按 `int32_t` 有符号值解释，负值仍按二进制补码传输。
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
- CPU2/CPU3 将空载扭力按 `int32_t` 有符号参数同步和显示，避免负空载扭力在保持寄存器和参数打印中被解释为无符号大数。
- CPU3 设备状态页新增 `STATE_DEBUG_MODE` 显示为“调试模式中”，英文为 `Debug Mode`；字库已有“调试模式中”和“自动恢复次数”所需汉字，无需新增点阵。
- CPU3 参数页将原 `reserved2` 菜单项显示为“自动恢复次数”，范围 `0~10`，写入同一保持寄存器地址。
- CPU3 外部 DSM 状态转换层将内部调试模式对外映射为既有维护模式，避免外部 DSM 主站收到未知 `0x0033`。
- 同步补充 CPU2/CPU3 协议版本 6 记录、CPU3 参数菜单与故障自动恢复次数说明、串口调试指令梳理、SI 指令映射资料、重启旧运动问题分析、SIL/MISRA 准备资料和 Markdown 转 PDF 工具。

验证：
- `py tools\check_si_protocol_contract.py`
- `py tools\check_si_modbus_frames.py`
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
- 现场升级动作：升级到 CPU2 V1.12.0.0 前必须备份或记录现场参数，升级后重新下发并核对继电器报警输出、液位、扭力、分布测量和通信参数。

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
- 本次删除的是已确认未用、未进入正式调用链的旧接口、备用调试入口和历史兼容路径；CPU2 写入测量结果、CPU3 读取 CPU2 输入寄存器、SI 主分发和 DSM 主协议路径保留。
- 删除源码后 CPU2/CPU3 `.hex` 哈希发生变化，按构建产物变化升级 CPU2/CPU3 build 版本，用于交付追踪和回溯。

本次修改：
- 删除 CPU3 旧 `com_manager.c/.h` 兼容分发模块，保留 `app_main` 现有主分发路径。
- 删除 CPU2 旧 `CH9141EVT.c/.h` AT 初始化备用入口。
- 删除 CPU2 `fault_manager` 旧全局故障接口、水位辅助接口、`AS5145_GetLastOkTick()`、`TMC5130 stpr_readInt()`、输入寄存器反向解析链路、`MotorCtrl_SetSpeed()`、`WIRELESS_Read_IntParam()` 和 `ErrorLog_Report()`。
- 删除 CPU3 OLED 未用接口、CPU3 时钟设置入口、设备状态错误设置、DSM 备用发送/输入寄存器单读写接口、SI 站号 get/set、CPU2 通信初始化、测量结果写输入寄存器和单参数同步入口。
- 调整 `tools/check_si_protocol_contract.py`，继续强制校验保留的 CPU2 写入测量结果和 CPU3 读取 CPU2 输入寄存器方向；对已删除的 CPU2 反向读取和 CPU3 写入死接口改为仅在源码存在时校验。
- 同步记录当前文档和资料整理改动，包括需求计划 PDF、自动恢复需求 PDF 替换，以及 SIL 功能安全资料 PDF。

验证：
- `rg` 检查确认删除符号在 `.c/.h` 中清零；仅保留 CPU2 正常写输入寄存器和 CPU3 正常读取 CPU2 输入寄存器接口。
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_si_modbus_frames.py`
- `py tools\check_si_protocol_contract.py`
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
- 尚未做实物按键联调；需现场确认进入选择型参数时当前值高亮、上下键循环、确认写回、返回取消，以及 COM 协议可选择到 SI 但不能选择“非法配置”。

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
- CPU2 同步调整参数注释、串口打印和扭力/探底相关中文口径，将“扭力/罐底”等容易混淆的名称统一为“扭力/探底”等现场口径；不改变计算逻辑、默认值、参数布局或保存格式。
- 参数配置首页调整为“测量参数、输出配置、通信设置、显示设置、维护设置”，运行策略并入测量参数，设备信息和参数校验并入维护设置。
- 标定液位值、修正液位值、标定水位值、标定罐高值、单点测量位置、单点监测位置和电机调试位置继续随指令输入，不再作为参数配置菜单项展示。
- 带参指令菜单项和下发确认页只显示指令名，不在指令名后追加参数值；参数值仅在输入页和参数写入确认页显示。
- 水位测量方式改为选择式编辑，屏幕显示为“低速模式/快速模式”，写入范围收敛为 0/1；CPU2 侧仍兼容原 0/非0 语义。
- COM 协议菜单只显示计量仪、瓦锡兰、LTD、SI 四个有效选项；SI 仍保存为真实协议值 5，不再向现场暴露预留协议项。
- 写任一 COM 串口参数后按协议自动收敛串口配置：计量仪/DSM 4800 8N1，瓦锡兰 4800 8N1，LTD 115200 8N1，SI 9600 8O1。
- 对运行策略、电机/编码换算、罐高/罐底、水位关键阈值、通信串口参数和恢复出厂设置增加“参数保护”额外确认页；不新增维护权限或更高等级密码。
- 继电器 R1~R4 子页新增只读“报警状态”入口，显示报警值、HH/H/HH-H/L/LL/LL-L、任意报警和清锁存运行态；数据来自 CPU2 输入寄存器快照，不触发参数写入。
- CPU3 状态页新增显示上下文归类，密度、温度、液位、水位、频率、电容和罐高按设备状态选择数据源，避免待机、运动、维护或故障状态混入旧业务结果。
- 位置固定取 `debug_data.sensor_position`，扭力固定取 `debug_data.current_weight`，允许位置或扭力为 `0` 时仍在正常状态页路径显示。
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
- 根据《LNG计量仪屏幕菜单.docx》同步 11 项 CPU2 恢复出厂默认值：故障自动回零、位置源自动切换、碰撞上下限比率、液位探头距差、液位盲区、探底扭力阈值、实测罐高最大偏差、区间测量上下限和瓦锡兰探底间隔。
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
- 根据《LNG计量仪屏幕菜单.docx》当前工作区默认值，再同步 9 项 CPU2 恢复出厂默认值：空载扭力上限、满载扭力下限、液位盲区、水位盲区、水位跟随电容阈值、水位寻找电容阈值、水位滞后电容阈值、最高点距液面和最低点距罐底。
- `water_lag_cap_threshold == 0` 的旧存储补默认值从 `80000` 同步为 `30000`，与恢复出厂默认值一致；非零现场值不被覆盖。
- 按 `CPU3状态页显示参数矩阵.xlsx` 同步状态页显示逻辑：`STATE_WARTSILA_DENSITY_MEASURING` 显示平均密度和平均温度，但不显示分布液位。
- `STATE_READPARAMETEROVER` 新增读取参数完成上下文，按有效数据分页显示液位、水位、平均密度、平均温度、频率、电容、X/Y 角和罐高。
- 将频率和电容拆为两个独立状态页参数项，读取参数完成时两者可同时显示。
- `STATE_ERROR` 状态页除状态、错误码、位置、扭力外，新增故障详情行；故障原因过长时沿用既有两行拆分逻辑。
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

## 2026-06-11 - 优化读取部件参数持续刷新与 DSM 调试区兼容（CPU2 V1.12.2.0 / CPU3 V1.11.2.0）

版本：
- CPU2: V1.12.1.3 -> V1.12.2.0
- CPU3: V1.11.1.4 -> V1.11.2.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不新增 CPU2/CPU3 共享命令码、状态码或寄存器地址；读取部件参数继续复用 `CMD_READ_PART_PARAMS`、`STATE_READPARAMETERING` 和 `STATE_READPARAMETEROVER`。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.12.1.3 升级到 V1.12.2.0 不会因参数存储版本触发恢复出厂。
- `debug_data.water_level_voltage` 更名为 `debug_data.water_capacitance_x10`，内部语义明确为水位电容快照，单位 0.1pF；CPU2/CPU3 调试数据寄存器位置保持不变。
- DSM 外部调试区保留历史寄存器地址，`0x0104` X 角度按一代兼容口径输出 `angle_x + 0x8000`，`0x010C~0x010D` 沿用“水位传感器电压”地址输出水位电容快照。

本次修改：
- CPU2 `CMD_ReadPartParams()` 首次读取完成后，在 `STATE_READPARAMETEROVER` 内每 1 秒刷新一次部件参数；等待期间通过 `AbortableDelay_CommandSwitch(..., 100U)` 支持新命令打断。
- 刷新逻辑保留在读取部件参数命令函数内部，不放入 `App_MainLoop()` 空闲轮询。
- CPU2/CPU3 内部调试数据字段统一改为 `water_capacitance_x10`，同步内部 Modbus 寄存器宏、打印和 CPU3 解析口径。
- CPU3 状态页读取参数完成态显示最新部件参数快照，不混用液位、水位、罐高等历史业务结果；频率和电容读取 `debug_data`。
- CPU3 DSM 外部输入寄存器修正 X 角度一代偏移兼容，并将水位电容快照写入历史水位传感器电压地址。
- 同步更新读取部件参数需求记录、状态页确认表、LNG 设备说明书、DSM V1.228 兼容方案、版本方案和文档索引。
- 新增读取部件参数刷新、DSM 兼容、CPU3 故障详情显示的契约检查脚本。
- 整理 LTD 传感器 S002-2026-V1.1 原理图资料和阅读版文档；该资料整理不影响固件协议和版本兼容性。

验证：
- `py -3 tools\check_read_part_params_refresh_contract.py`
- `py -3 tools\check_dsm_compat_contract.py`
- `py -3 tools\check_cpu3_fault_reason_visibility.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- 已做源码/文档关键字复查，确认不再残留 `debug_data.water_level_voltage` 旧字段引用；目标文档不再描述主循环轮询或跳过外层初始化旧方案。
- 尚未做实物 OLED 翻页、DSM 主站读取和读取部件参数长时间运行联调；现场需确认完成态持续刷新周期、新命令打断和外部寄存器数值倍率。

## 2026-06-11 - 新增 CPU2 点动长距离运动接口和 BJ 测试命令（CPU2 V1.13.0.0）

版本：
- CPU2: V1.12.2.0 -> V1.13.0.0
- CPU3: 保持 V1.11.2.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不新增 CPU2/CPU3 共享命令码、状态码、寄存器地址或参数字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.12.2.0 升级到 V1.13.0.0 不会因参数存储版本触发恢复出厂。
- `BJ+`、`BJ-`、`BJP` 仅为 CPU2 本地串口调试命令，不改变内部 Modbus、DSM 或 Wartsila 外部协议。
- 既有 `MotorCtrl_MoveToPosition()` 和 `MotorCtrl_MoveAndWait()` 保持原有位置模式方案；新增点动接口供长距离移动场景单独调用。

本次修改：
- TMC5130 BSP 暴露速度点动接口 `stpr_rotate()`，并在 `stpr_stop()` 停机后对齐 `XTARGET` 和 `RAMPMODE`，避免速度模式停机后目标位置状态残留。
- 新增 `MotorCtrl_JogMoveAndWait()` 和 `MotorCtrl_JogMoveToPosition()`，使用速度点动模式按方向运行，持续读取有效位置源，并在接近目标前提前降速和提前停机。
- 点动方案根据当前速度、驱动加速度和毫米换算系数计算降速距离和低速停机距离，低速停机后再次刷新位置；若最终位置超过目标容差，返回 `MEASUREMENT_POSITION_ERROR`，不再把超限当作成功。
- 点动运行过程中保留命令切换、驱动健康检查、位置源有效性检查、驱动状态刷新、扭力碰撞保护、丢步检测、运行超时和临时速度恢复。
- 新增 `motor_jog_text()` 和 `motor_jog_to_position_text()` 测试封装，串口 `BJ+<mm>[,<速度m/min>]`、`BJ-<mm>[,<速度m/min>]`、`BJP<target_mm>[,<速度m/min>]` 可直接验证相对和绝对点动运行。
- 同步新增 CPU2 V1.13.0.0 改动与测试方案，记录兼容性、参数存储影响、重点测试、回归范围和现场验证结果。

验证：
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- 功能分支已通过 ST-LINK 下载同一电机点动控制改动并做现场 BJ/BJP 测试：`BJP19450,1.5` 终点约 `19450.500mm`，未再出现超过目标位置后仍返回成功的问题。

## 2026-06-11 - 拆分 CPU2 串口测试命令并整理构建流程（CPU2 V1.13.0.1）

版本：
- CPU2: V1.13.0.0 -> V1.13.0.1
- CPU3: 保持 V1.11.2.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不新增 CPU2/CPU3 共享命令码、状态码、寄存器地址或参数字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.13.0.0 升级到 V1.13.0.1 不会因参数存储版本触发恢复出厂。
- CPU2 本地串口测试命令字符、参数格式和执行入口保持兼容；本次只把测试命令实现从 `measure.c` 拆到 `test.c`，暂不区分正式/测试构建。
- CPU3 屏幕驱动与刷新资料整理只新增分析文档和屏幕资料归档，不改变 CPU3 固件代码或协议行为。

本次修改：
- `measure.c` 保留正式业务串口命令映射和主循环调度，测试串口命令统一转交 `Test_ProcessSerialCommand()`。
- `test.c` 承接 `SC`、`SP*`、`B/BE/BJ/BJP`、`A/C/D/E/F/H/J/L/M/N/T*/Y*/X` 等本地串口测试命令，并增加空命令保护。
- `test.h` 新增 `Test_ProcessSerialCommand()` 声明，避免 `measure.c` 继续保存大量测试实现。
- GitHub Actions CPU2/CPU3 构建流程改为显式 `source_dir`/`build_dir`，使用项目固定构建目录并按 `.elf/.hex/.bin/.map` 上传产物。
- `.gitignore` 忽略 `.superpowers/` 本地工具目录。
- 新增 CPU3 屏幕驱动与刷新问题分析文档，归档当前 HGS128645-Y-EH-LV / SSD1325 OLED 屏资料，并同步问题分析和界面文档索引。
- 同步新增 CPU2 V1.13.0.1 改动与测试方案和 CPU2 参数存储升级说明。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --check -- LTD_MAIN_CPU2/Application/Src/measure.c LTD_MAIN_CPU2/Application/Src/test.c LTD_MAIN_CPU2/Application/Inc/test.h`
- GBK 编码检查：`measure.c`、`test.c`、`test.h` 均未出现替换字符或 `??`。
- 结构检查：`measure.c` 调用 `Test_ProcessSerialCommand()`，`test.c` 提供测试命令入口，`test.c` 不包含 `ProcessMeasureCmd()`。
- `Get-FileHash -Algorithm SHA256 docs\04_界面与菜单\00_屏幕资料\HGS128645-Y-EH-LV_SSD1325_OLED_当前液位计屏幕_中文版.pdf`：与索引记录一致。

## 2026-06-11 - 新增 CPU3 屏幕亮度配置并优化 OLED 刷新恢复（CPU3 V1.12.0.0）

版本：
- CPU2: 保持 V1.13.0.1
- CPU3: V1.11.2.0 -> V1.12.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享命令、状态、输入寄存器和 CPU2 设备参数语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，CPU2 `DeviceParameters` 结构和升级清参规则不变。
- CPU3 本地显示/通信参数版本 `CPU3_PARAM_VERSION` 从 `0x0003` 升至 `0x0004`，新增屏幕亮度字段。
- 从 CPU3 本地参数 V3 升级时保留原有显示/通信配置，把屏幕亮度补为默认“中低”挡并写回 V4；本地参数魔术字或版本异常时仍按默认值重建。
- 新增 `HOLDREGISTER_CPU3_BRIGHTNESS` 使用 CPU3 本地保持寄存器预留地址 `CPU3_BASE + 0x1C`，只用于 CPU3 屏幕亮度配置，不要求提升共享协议版本。

本次修改：
- 在 `显示设置 -> 显示基础` 菜单新增“屏幕亮度”，支持“低/中低/中/中高/高”五挡，默认保持原等效亮度“中低”。
- OLED 初始化改为统一命令表和亮度接口，初始化、清屏恢复、屏幕重开和异常恢复都使用当前配置的亮度。
- OLED 驱动新增影子缓冲、刷新序号、SPI 错误统计和健康诊断字段，便于判断花屏或总线异常后的恢复状态。
- 中断上下文只置刷新请求，实际 OLED 绘制在主循环中完成，降低中断内 SPI 操作和页面状态竞争风险。
- 状态页从菜单等前景页面返回时强制全屏重绘，避免非全屏局部刷新导致乱码残留。
- 状态页数值变化时仅清理并重画变化区域，变化值和单位反显高亮 0.5 秒，到期后自动请求一次局部刷新恢复普通显示。
- 屏幕恢复入口优先重画当前页面；状态页、菜单页和页面刷新标志统一走恢复后的主循环刷新路径。
- 同步更新 CPU3 屏幕驱动与刷新问题分析文档、界面索引、版本改动与测试方案，并新增 OLED 刷新边界和亮度配置检查脚本。

验证：
- `py tools\check_cpu3_display_isr_boundaries.py`
- `py tools\check_cpu3_oled_brightness_config.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

## 2026-06-11 - 优化 CPU3 长按确认进入配置菜单稳定性（CPU3 V1.12.1.0）

版本：
- CPU2: 保持 V1.13.0.1
- CPU3: V1.12.0.0 -> V1.12.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7，不改变 CPU2/CPU3 共享命令、状态、寄存器或 CPU2 参数语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，CPU2 参数存储结构和升级清参规则不变。
- CPU3 本地显示/通信参数结构和 `CPU3_PARAM_VERSION` 保持 `0x0004`，不触发 CPU3 本地参数重建。
- 本次只影响 CPU3 本地按键识别、长按进入配置菜单和长按返回确认页的 UI 操作体验。

本次修改：
- 长按确认进入配置菜单的阈值从约 3.0 秒调整为约 1.5 秒，修正旧注释与实际计数不一致的问题。
- EXTI 按键入口增加 50ms 软件消抖，降低确认键抖动导致长按计数反复清零的概率。
- 状态页长按检测增加目标键锁定，确认长按开始后返回/上下误触不再覆盖当前长按目标。
- 旧目标键已经释放但 TIM1 尚未清锁时，新按下的确认/返回可以清理旧目标并接管长按检测，修正先按其它键再长按确认无法进入配置菜单的问题。
- 启动 TIM1 长按采样前清零计数器并清除更新标志，使首次采样时间更一致。
- 长按触发后清空普通按键队列并进入释放保护，必须松开确认键后再短按确认，避免二次确认页被残留确认键自动越过。
- 新增长按入口回归检查脚本，并同步 CPU3 长按确认不灵敏问题分析文档和问题索引。

验证：
- `py tools\check_cpu3_long_press_entry.py`
- `py tools\check_cpu3_display_isr_boundaries.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

## 2026-06-11 - 优化瓦锡兰分布测量点间移动与空气点液位识别（CPU2 V1.13.1.0）

版本：
- CPU2: V1.13.0.1 -> V1.13.1.0
- CPU3: 保持 V1.12.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不修改 CPU2/CPU3 共享命令、状态、输入寄存器、共享结构体或设备参数字段语义。
- 不修改外部 Wartsila 寄存器映射和 CPU3 外部 Wartsila 主站状态码表达。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.13.0.1 升级到 V1.13.1.0 不会因参数存储版本触发恢复出厂。

本次修改：
- 瓦锡兰分布测量起始点定位和点间移动改为只按位置运行，不再在点间运动过程中切液位模式或做液位模式空气检测。
- 到达测点后使用密度模式读取的实际密度值和浮点频率判定空气：实际密度 `< 100`、浮点频率 `> oilLevelFrequency` 或密度读取 5 分钟超时输出 `0` 均按空气点处理。
- 空气点不写入分布测结果；遇空气点后切液位模式慢速下行，首次识别到液体时把当前位置作为液位位置。
- 结果裁剪改为只保留 `point.position_mm < level_mm - wartsila_max_height_above_surface` 的液体点，不再设置最多删除两个点的硬限制。
- 瓦锡兰分布测失败时不把临时结果覆盖到 `g_measurement.density_distribution`，避免失败后对 CPU3/外部读取暴露清零结果。
- 新增瓦锡兰流程规则检查脚本和配套需求、流程、对比、验证及版本测试文档。

验证：
- `py -B tools\check_wartsila_distribution_logic.py`
- `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --check`
- `py tools\check_version_bumped.py`

未验证风险：
- 尚未执行现场 Wartsila 主站读写、CPU3 页面读取、真实液面慢速下行识别和裁剪边界实测。
- `EnableLevelMode()` 已有 10 秒模式稳定等待，但瓦锡兰慢速下行运动中液位判定仍使用单次频率读取，现场需验证切换后频率稳定性和误判风险。

## 2026-06-11 - 修复电机绝对目标越界保护和参数错误自动恢复（CPU2 V1.13.2.0）

版本：
- CPU2: V1.13.1.0 -> V1.13.2.0
- CPU3: 保持 V1.12.1.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 7。
- 不新增 CPU2/CPU3 共享命令、状态、寄存器、结构体或设备参数字段。
- 电机绝对目标越界继续使用已有 `PARAM_RANGE_ERROR` (`0x00110003`)，不新增错误码。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.13.1.0 升级到 V1.13.2.0 不会因参数存储版本触发恢复出厂。

本次修改：
- `MotorCtrl_MoveByTicksAndWait()` 拆分为起步等待和到位等待两个内部助手，保留编码器 tick 运动的起步、超时、误差和用户中断判定。
- `MotorCtrl_MoveAndWait()` 删除不可达重试框架，改为单路径正式运动，入口统一刷新当前位置、计算目标位置并做目标范围检查。
- 新增 `MotorMotionSpeedScope` 速度保护作用域，集中处理正式运动和点动运动的速度切换、失败退出和恢复默认速度。
- 新增 `MotorMotion_CheckAbsoluteTargetRange()`，在正式绝对目标、点动绝对目标和由相对距离换算出的目标位置上统一拦截小于 0 或超过罐高的目标。
- 点动位置刷新改为 `MotorMotion_RefreshJogPositionChecked()`，保留快照有效性、运动方向和越界检查。
- 自动故障恢复入口增加不可恢复错误过滤，`PARAM_RANGE_ERROR`、`PARAM_ADDRESS_OVERFLOW` 和 `PARAM_ERROR` 不再进入自动恢复重跑，避免越界目标反复重试。
- 同步整理电机运动函数优化计划、台架验证记录、本次改动点 HTML 和电机与编码器文档索引。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- COM12 115200 GBK + ST-LINK 台架验证同逻辑固件：T01~T14 通过，越界 1100mm 正式目标和 `BJP1100,1.0` 均返回 `PARAM_RANGE_ERROR` 且不运动、不触发自动恢复；`BJP700,1.0` 合法目标可正常执行。

未验证风险：
- T15/T16 大负距离和长时间软停止场景未在本轮台架覆盖。
- 现场机械限位要求传感器位置不低于 200mm，本次固件只按 0 到罐高做通用目标保护，未把 200mm 作为固件硬限位。

## 2026-06-12 - 新增连续找液位方式并同步协议版本8（CPU2 V1.14.0.0 / CPU3 V1.13.0.0）

版本：
- CPU2: V1.13.2.0 -> V1.14.0.0
- CPU3: V1.12.1.0 -> V1.13.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION`: 7 -> 8。
- `liquidLevelMeasurementMethod` 仍使用原保持寄存器和原 32 位字段，不新增或移动寄存器。
- 液位测量方式语义扩展为 `0=相对频率`、`1=定频`、`2=密度连续找液位`、`3=超声预留`、`4=连续相对频率`、`5=连续定频`。
- CPU2 和 CPU3 必须同为协议版本 8 才能正确显示、写入并执行新的连续找液位方式；协议版本 7 的 CPU2/CPU3 不应混用。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.13.2.0 升级到 V1.14.0.0 不会因参数存储版本触发恢复出厂。

本次修改：
- CPU2 新增密度连续找液位/跟随闭环，按目标密度偏差和死区计算速度，稳定后记录液位并同步分布测液位字段。
- CPU2 新增连续相对频率和连续定频找液位/跟随闭环，保留原 0/1 步进式频率精找路径不变。
- CPU2 新增 `MotorCtrl_StartVelocity()` 速度模式启动接口，供连续液位闭环按方向和速度更新电机运动。
- 水位精找前如果粗找停稳后仍处于水中，先上行脱离水区，再进入低速下行细找，避免第一次采样直接结束。
- CPU3 液位测量方式菜单新增“连相对频率”和“连定频”，参数范围同步为 `0..5`。
- CPU3 参数列表页补充和收敛 OLED 短名，菜单文档同步标注屏幕短名，避免名称挤占数值列。
- 同步协议变更记录、系统参数默认值、版本测试方案和契约检查脚本。

验证：
- `py -3 tools\check_density_level_control_contract.py`
- `py -3 tools\check_water_precise_search_contract.py`
- `py -3 tools\check_cpu3_menu_name_width.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

未验证风险：
- 未做实物联调；需要现场验证密度连续、连续相对频率、连续定频三种方案的方向、速度响应、稳定判定、命令切换退出和异常停机。
- 水位精找前脱水逻辑未做真实水位工况验证，需现场确认粗找后仍在水中时的上行脱离距离和零点附近保护。

## 2026-06-13 - 优化电机点动与位置模式运动流程（CPU2 V1.14.0.1）

版本：
- CPU2: V1.14.0.0 -> V1.14.0.1
- CPU3: 保持 V1.13.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 8。
- 不新增 CPU2/CPU3 共享命令、状态、寄存器、结构体字段或设备参数字段。
- BJ/BJP 本地串口测试命令格式不变；本次只收敛 CPU2 电机运动内部目标规划、运行守护和停止后同步流程。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.14.0.0 升级到 V1.14.0.1 不会因参数存储版本触发恢复出厂。

本次修改：
- 新增 `MotorMotionTargetPlan`，将相对运动、绝对位置运动和点动位置模式的目标、方向、距离、提前到达判断统一为同一规划口径。
- 收敛运动前准备、命令切换中断、临时速度恢复、起步观察、运行期驱动健康检查和停止后位置同步流程，减少正式运动与点动运动的重复分支。
- 点动运行期守护统一检查驱动健康、扭力碰撞、丢步检测和命令切换，结束后统一同步 TMC5130 位置或健康状态。
- 新增 `tools/check_motor_motion_target_plan.py`，静态检查目标规划 helper 和点动/位置模式入口仍接入统一范围检查。
- 同步整理电机点动与位置模式分层优化方案、剩余台架验证规程、当前改动与待测项汇总、计划审计、详细流程图和第三轮策略评估记录。

验证：
- `py tools\check_motor_motion_target_plan.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- 功能 worktree 前序台架验证：COM12 115200 GBK + ST-LINK 烧写同逻辑 CPU2 固件后，`BJ-20,0.3`、`BJ+20,0.3`、`BJP880,0.3`、`BJP1100,0.3`、`A-10`、`A+10` 均符合预期；`BJP1100,0.3` 返回 `PARAM_RANGE_ERROR` 且未运动。

未验证风险：
- 合入当前 `MAIN` 并升级到 V1.14.0.1 后，尚未重新烧写最终合并固件做完整台架回归。
- 前序台架中出现过两次可恢复的 AS5145 编码器校验失败重试，后续静止 24 次 `YS/YC` 和 5mm 往返复测未复现；现场仍需观察运动后首帧采样和线束抗干扰。
- 长距离软停止、大负距离越界和更多速度组合仍建议按《电机运动剩余台架验证执行规程》补测。
- 台架安全要求传感器位置不低于 200mm、不高于 1000mm；固件通用目标范围仍按 0 到罐高判断，没有把 200mm 作为软件硬限位。

## 2026-06-13 - 新增读取部件参数蓝牙RSSI并优化CPU3菜单显示（CPU2 V1.15.0.0 / CPU3 V1.14.0.0）

版本：
- CPU2: V1.14.0.1 -> V1.15.0.0
- CPU3: V1.13.0.0 -> V1.14.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION`: 8 -> 9。
- `WirelessPairingStatus` 输入寄存器尾部追加当前连接有效、RSSI 有效、RSSI 值、查询错误码和 RSSI 更新计数；`REG_ENG` 随新增尾部字段顺延。
- CPU2/CPU3 必须同为协议版本 9，才能正确读取和显示当前蓝牙连接 RSSI；协议版本 8 的旧端不应解释新增尾部输入寄存器。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.14.0.1 升级到 V1.15.0.0 不会因参数存储版本触发恢复出厂。
- CPU3 本机参数版本保持 `0x0004`，不改变本机显示/通信参数存储布局。

本次修改：
- CPU2 读取部件参数流程增加 CH9141K 当前蓝牙连接状态和 RSSI 快照查询；查询失败时发布错误码和无效 RSSI，不重新配对、不保存默认连接。
- CPU2/CPU3 同步协议版本 9、输入寄存器映射和无线滑环 RSSI 契约检查脚本。
- CPU3 读取部件参数完成态持续显示位置、扭力、温度、频率、电容、X/Y 角和 RSSI；RSSI 有效时显示数值，无效时显示 `RSSI:N/A`。
- CPU3 菜单按现场口径重组：读取部件参数前移，测量命令拆分密度单点测量和密度分布测量，水位命令归组，分布类密度命令集中到密度分布测量，传感器运动命令集中到浮子运动控制。
- CPU3 调试指令显示完整指令名称，空载/满载扭力获取时进入独立等待页，完成后返回扭力配置菜单；补充“控”字模。
- CPU2 瓦锡兰密度读取在频率无效或超时保持无有效频率时返回频率异常，避免把无效频率误归类为有效空气点。
- 同步更新菜单 Markdown/Word 文档、状态页显示确认表、协议变更记录、系统参数默认值、程序流程导航、版本改动与测试方案和相关检查脚本。

验证：
- `py tools\check_wireless_rssi_contract.py`
- `py tools\check_si_protocol_contract.py`
- `py tools\check_density_level_control_contract.py`
- `py tools\check_read_part_params_refresh_contract.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `py tools\update_flow_navigation.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

未验证风险：
- 未做实物联调；需要现场验证 CH9141K 已连接、未连接、RSSI 查询超时、命令切换打断和 UART6 透传恢复。
- CPU3 菜单回退路径、扭力等待页返回路径和 OLED 分页显示仍建议在实机按键上逐项确认。
- 瓦锡兰频率异常保护需在现场确认无效频率、空气点和液体点的边界表现。

## 2026-06-15 - 新增 AO 电流输出运行态与使能参数（CPU2 V1.16.0.0 / CPU3 V1.15.0.0）

版本：
- CPU2: V1.15.0.0 -> V1.16.0.0
- CPU3: V1.14.0.0 -> V1.15.0.0

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION`: 9 -> 10。
- 协议版本 10 在无线 RSSI 运行态后追加 `AoOutputRuntime` 输入寄存器尾段，发布 AO 目标电流、最近写入电流、来源、AD5421 故障标志、READFAULT 原始值、最近错误码和更新时间。
- 协议版本 10 将原 `reserved26` 正式替换为 `AoOutputEnable`，保持地址 `0x00C2-0x00C3` 不后移，语义为 `0=关闭`、`1=启用`。
- CPU2/CPU3 必须同为协议版本 10 才能正确显示、写入和执行 AO 输出使能；协议版本 9 的旧端不应与本版本混用。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因参数存储版本触发恢复出厂参数；旧协议存储升级时将 `AoOutputEnable` 默认置 0，避免未接电流环时误启用 AO 输出。

本次修改：
- CPU2 新增 AO 模拟电流输出服务，统一处理液位电流计算、报警电流覆盖、故障/调试电流选择、AD5421 写入和运行态维护。
- CPU2 AD5421 驱动增加有限 SPI 超时、控制寄存器回读、FAULT 管脚/READFAULT 诊断和专用错误码。
- CPU2 在主循环和液位测量刷新 AO 输出，HART Command 2/3 改为读取 AO 运行态电流和 4-20mA 百分比。
- CPU2 `AoOutputEnable=0` 时不初始化、不诊断、不写入 AD5421，并发布禁用运行态；恢复出厂和旧协议升级默认关闭 AO 输出。
- CPU2 `AoOutputEnable=1` 时对 AD5421 初始化重试、诊断轮询和重复写失败做 1 秒节流，避免硬件异常时每轮主循环阻塞访问 SPI/GPIO。
- CPU2 启动阶段 AO/电机初始化失败后，在 `fault_info_init()` 清理故障信息之后恢复启动错误码，并阻止上电默认命令继续下发。
- CPU3 同步解析 AO 运行态、补充 AD5421 故障文案，并在 AO 输出配置菜单新增 `AO使能`，范围限制为 0..1。
- 同步更新协议记录、默认值文档、CPU3 菜单/状态页文档、AO 契约检查脚本和版本改动与测试方案。

验证：
- `py -3 tools\check_ao_output_enable_contract.py`
- `py -3 tools\check_si_protocol_contract.py`
- `py -3 tools\check_density_level_control_contract.py`
- `py -3 tools\check_wireless_rssi_contract.py`
- `py -3 tools\check_read_part_params_refresh_contract.py`
- `py -3 LTD_DISPLAY_CPU3\font_check.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --check`

未验证风险：
- 未做实物联调；需要现场验证 `AoOutputEnable=0` 且电流环未接时不上报 AO 故障。
- `AoOutputEnable=1` 时仍需用电流表/HART 主站验证 4-20mA 输出、电流百分比、报警/故障/调试电流和 AD5421 故障诊断边界。
- 提交前需要在暂存目标文件后运行 `git diff --cached --check` 和 `py -3 tools\check_version_bumped.py`。
- AO 输出只作为测量结果后的辅助输出刷新，不新增 CPU2/CPU3 命令入口或改变测量状态机；启动流程图已同步 AO 初始化、启动错误保留和上电默认命令拦截，后续若把 AO 输出做成独立命令或闭环控制再补对应业务流程图。

## 2026-06-17 - 修复状态页刷新节流和参数范围校验（CPU2 V1.16.0.1 / CPU3 V1.15.0.1）

版本：
- CPU2: V1.16.0.0 -> V1.16.0.1
- CPU3: V1.15.0.0 -> V1.15.0.1

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增 CPU2/CPU3 共享寄存器、命令码或输入寄存器尾段。
- 协议版本 10 继续同时覆盖 `AoOutputRuntime` 输入寄存器尾段和 `AoOutputEnable` 保持寄存器语义，不再拆分为两个协议版本。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，从 V1.16.0.0 升级到 V1.16.0.1 不会因参数存储版本触发恢复出厂参数。
- 旧 FRAM 中 `bottom_detect_mode` 若为非 `0/1` 异常值，运行期归一化为 0；保持寄存器读取路径按 `0=扭力`、`非0=陀螺仪角度` 归一化显示。

本次修改：
- CPU3 状态页增加运行数据 2 秒采样节流，非采样周期只恢复反白区域，减少状态页频繁重绘带来的闪烁和数据跳动。
- CPU3 参数详情页用数字绘制范围分隔符和小数点，避免使用字库外字符造成范围显示异常。
- CPU3 参数元数据收紧罐底检测模式范围为 `0..1`，缩短尺带伸缩率和继电器报警阈值输入位宽，降低 OLED 输入显示越界风险。
- CPU2/CPU3 读取保持寄存器时将 `bottom_detect_mode` 统一归一化为 `0/1`，CPU2 运行期加载旧 FRAM 参数时对异常值归零。
- 同步修正协议版本 10 的文档口径、系统参数默认值、界面菜单索引和版本改动与测试方案。

验证：
- `git diff --cached --check`
- `py -3 tools\check_version_bumped.py`
- `py -3 tools\check_ao_output_enable_contract.py`
- `py -3 tools\check_cpu3_ret_arr_word_contract.py`
- `py -3 LTD_DISPLAY_CPU3\font_check.py`

未验证风险：
- 未做实物联调；需要在 OLED 实机上确认状态页 2 秒采样节流、反白恢复、菜单范围显示和参数输入位宽不会造成可见闪烁或截断。
- 未重新运行 CPU2/CPU3 完整固件构建；提交前如需要发布固件，仍建议补跑 `cmake --build build\LTD_MAIN_CPU2` 和 `cmake --build build\LTD_DISPLAY_CPU3`。
- 本次不改变测量状态机、命令入口、协议尾段布局或 AO 输出执行链路，程序流程图无需更新。

## 2026-06-17 - 增加 CPU3 RTC LSE 电池保持和屏幕校时（CPU3 V1.15.0.2）

版本：
- CPU2: 保持 V1.16.0.1。
- CPU3: V1.15.0.1 -> V1.15.0.2。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增 CPU2/CPU3 共享寄存器、命令码或输入寄存器尾段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次 CPU3 RTC 改动触发恢复出厂参数。
- CPU3 本机参数版本保持 `0x0004`，不改变 CPU3 FRAM 本机显示/通信参数布局。
- 新 CPU3 板优先使用外部 32.768 kHz LSE 和 VBAT 电池保持 RTC；旧板或 LSE 起振失败时自动回退内部 LSI，启动不因 LSE 缺失卡死。
- RTC 只服务外部协议时间显示和 profile 完成时间锁存，不参与 CPU2 测量控制和调度。

本次修改：
- CPU3 CubeMX 工程启用 RTC/HAL RTC 相关文件，并在 `.ioc` 中配置 RTC 时钟优先为 LSE。
- CPU3 系统时钟配置增加 LSE 失败兜底逻辑，旧板未装 LSE 时会关闭 LSE 配置并继续启动。
- `Cpu3Clock_Init()` 改为 LSE 优先、LSI 兜底；仅在备份标记无效、RTC 未初始化或时间非法时写入默认时间，避免每次上电覆盖电池保持时间。
- 新增 `Cpu3Clock_SetDateTime()`、`Cpu3Clock_GetState()` 和 `Cpu3Clock_GetSource()`，用于屏幕校时和显示 RTC 状态。
- CPU3 维护设置菜单新增 `RTC设置` 页面，可编辑年、月、日、时、分、秒，保存成功后写入已校时备份标记。
- SI 现有 profile 完成时间锁存和当前时分秒实时输出逻辑保持不变，第一阶段不新增对外校时寄存器。
- 同步更新 CPU3 程序流程文档和本版本改动与测试方案。
- 本次提交按用户要求包含当前工作区全部改动，其中 CPU2 现有差异为中文注释编码形式变化，不改变 CPU2 版本号和运行逻辑。

验证：
- `py -3 tools\generate_cpu3_flow_docs.py`
- `py -3 LTD_DISPLAY_CPU3\font_check.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py -3 tools\check_version_bumped.py`

未验证风险：
- 未做 RTC 实物联调；需要在新板验证 LSE 起振、VBAT 断主电保持、菜单校时保存和重新上电不覆盖时间。
- 旧板兼容需要实机确认未装 LSE 时系统时钟配置不会卡死，并且 RTC 状态显示为 LSI 兜底。
- LSI 兜底模式长期时间精度有限，现场若看到 `RTC LSI` 应按硬件配置或 LSE 起振问题排查。

## 2026-06-18 - 修复 CPU3 串口协议默认值与手动参数覆盖逻辑（CPU3 V1.15.1.0）

版本：
- CPU2: 保持 V1.16.0.1。
- CPU3: V1.15.0.2 -> V1.15.1.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增 CPU2/CPU3 共享寄存器、命令码或输入寄存器尾段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会触发 CPU2 参数恢复出厂。
- CPU3 本机参数版本保持 `0x0004`，不改变 CPU3 FRAM 本机通信/显示参数存储布局。
- 旧 FRAM 中已有的合法 COM1/COM2/COM3 串口参数会原样保留；只有协议字段被用户修改时，才按新协议带出默认串口参数。

本次修改：
- CPU3 COM1/COM2/COM3 协议字段修改时，自动带出对应协议默认串口参数：DSM/Wartsila 为 `4800 8N1`，LTD 为 `115200 8N1`，SI 为 `9600 8O1`。
- CPU3 波特率、数据位、校验和停止位允许后续单独修改，保存和上电加载时只修正非法值，不再被当前协议持续覆盖。
- CPU3 COM 配置菜单顺序调整为“协议、波特率、数据位、校验、停止位”，让现场先选协议再确认或覆盖物理串口参数。
- 同步更新 SI协议枚举注释和本版本改动与测试方案。

验证：
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py -3 tools\check_version_bumped.py`

未验证风险：
- 未做实物联调；需要在 OLED 菜单中验证 COM1/COM2/COM3 修改协议后默认值立即带出，随后单独修改波特率、校验和停止位能够保存并重启后保持。
- 如果现场依赖“选择 SI 后始终强制 9600 8O1”的旧行为，需要升级说明中明确新版本允许人工覆盖，避免误判为配置异常。
- 本次不改变主循环、状态机、命令分发、CPU2 内部通信链路或外部协议寄存器映射，程序流程图无需更新。

## 2026-06-23 - 修复继电器报警限值写入和参数打印（CPU2 V1.16.1.0 / CPU3 V1.15.2.0）

版本：
- CPU2: V1.16.0.1 -> V1.16.1.0。
- CPU3: V1.15.1.0 -> V1.15.2.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增 CPU2/CPU3 共享寄存器、命令码或输入寄存器尾段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数版本保持 `0x0004`，不改变 CPU3 FRAM 本机通信/显示参数存储布局。
- 继电器报警限值仍按既有保持寄存器和 IEEE754 float 原始位传输，本次只修正 CPU3 写入时的字序组织错误，不改变外部寄存器地址和字段语义。

本次修改：
- 修复 CPU3 菜单写入 CPU2 浮点参数时先按字节重组再交给发送层换字导致的字序反转问题，继电器报警 HH/H/L/LL 阈值、滞回等 float 参数可按菜单值正确写入 CPU2。
- CPU3 本机参数写入时只对整型参数应用 `offset`，避免浮点/双精度参数路径被无关偏移处理影响。
- CPU2/CPU3 参数打印补全继电器报警配置中的无效值、HH/H/L/LL 阈值、滞回、阻尼和清锁存字段，便于现场确认报警限值是否落库。
- 版本升级脚本支持按 UTF-8 或 GBK 读取并按原编码写回版本头，避免 CPU2 GBK 源码版本头阻塞后续升版。
- 同步新增本版本改动与测试方案，并更新版本测试方案索引。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py -3 tools\check_version_bumped.py`

未验证风险：
- 未做实物联调；需要在 OLED 菜单实机写入 R1~R4 继电器报警 HH/H/L/LL 阈值和滞回后，读取 CPU2 参数打印确认数值一致。
- 需要现场覆盖禁用、无源输出、不同报警源/报警位和常开/常闭组合，确认新增打印字段不会被串口工具截断或误判。
- 本次不新增命令入口、状态机分支、流程跳转或跨 CPU 导航关系，只修正既有参数写入载荷编码并补全打印字段，程序流程图无需更新。

## 2026-06-24 - 修复电机固定点定位点动两段减速（CPU2 V1.16.2.0）

版本：
- CPU2: V1.16.1.0 -> V1.16.2.0。
- CPU3: 保持 V1.15.2.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增或变更 CPU2/CPU3 共享寄存器、命令码或输入寄存器尾段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- 不改变 `MotorCtrl_MoveToPosition()` 原有位置模式方案；正式测量、密度分布和瓦锡兰固定点移动调用处改为 `MotorCtrl_JogMoveToPosition()`。
- Jog 最终误差仅在超过 10mm 时上报 `MEASUREMENT_POSITION_ERROR`；0.1mm 仍用于入口已到位判断。

本次修改：
- 正式单点测量、固定点监测、密度分布逐点移动、运行到指定位置、瓦锡兰测点和测后回监测点改用 `MotorCtrl_JogMoveToPosition()`。
- 恢复点动绝对定位两段速度策略：高速段按计算刹车距离 * 1.3 且最小 2mm、最大 1000mm 切到 0.10m/min 低速；低速同向爬行到 0.45mm 触发慢停。
- 移除低速段再次估算停机距离的提前停机路径，避免低速段过早刹车。
- 最终停稳复核阈值放宽为 10mm，超过阈值才上报 `MEASUREMENT_POSITION_ERROR`；日志保留高低速切换、停止触发和刹车距离诊断。
- 同步新增本版本改动与测试方案，并更新密度/单点测量、电机位置模型流程文档。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `python tools\check_motor_motion_target_plan.py`
- `git diff --check`
- `git diff --cached --check`
- `python tools\check_version_bumped.py`

未验证风险：
- 未在本次提交流程重新接串口实物联调；需现场用 BJP 和正式单点、固定点、密度分布流程确认上下行到位、无反向修正和最终误差阈值。
- 最终误差 10mm 内不再报 `MEASUREMENT_POSITION_ERROR`，上层流程可能把 2~10mm 误差视为成功；现场需确认该容差符合业务允许范围。
- 本次改变正式测量入口的运动策略，程序流程文档已同步更新密度/单点测量和电机位置模型页面。

## 2026-06-24 - 改用蓝牙主从机链路并优化 CPU3 菜单（CPU2 V1.17.0.0 / CPU3 V1.16.0.0）

版本：
- CPU2: V1.16.2.0 -> V1.17.0.0。
- CPU3: V1.15.2.0 -> V1.16.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增或变更 CPU2/CPU3 共享寄存器、命令码、输入寄存器尾段或保持寄存器地址。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- `WIRELESS_HOST_COMM_TIMEOUT` 和 `WIRELESS_SLAVE_COMM_TIMEOUT` 错误码数值保持不变，仅把现场语义和显示文案调整为蓝牙主机/蓝牙从机连接异常，避免破坏既有错误码传递链路。
- 旧无线主机/从机 8 字节自身通信模块移除后，传感器测量透传仍继续使用 UART6；蓝牙链路状态统一通过 CH9141K AT 状态查询判断。

本次修改：
- CPU2 删除旧 `wireless_communication.c/.h`，上电传感器识别、传感器通信超时诊断、配对完成链路确认和 `SC` 串口测试统一改为 `WirelessPairing_CheckBluetoothLink()`。
- 蓝牙链路检查进入 CH9141K AT 模式后读取 `AT+BLEMODE?`、`AT+BLESTA?`、`AT+CCADD?`，并按需读取 RSSI，启动识别日志可打印从机 MAC 和 RSSI 快照。
- 读取部件参数流程继续保留 RSSI 刷新；刷新被新命令打断时返回 `STATE_SWITCH`，普通 RSSI 刷新失败只保留上次快照，不中断部件参数主流程。
- CH9141K AT 入口失败或被命令切换打断时增加透明模式恢复和 UART6 清理，降低后续传感器透传被半截 AT 状态影响的风险。
- CPU3 修复继电器参数四级菜单返回路径，通道设置返回对应 `menu_relayN_channel`，报警配置返回对应 `menu_relayN_alarm`。
- CPU3 尺带厚度参数增加 PET 0.300、PEEK 0.500、ETFE 1.100 和手输入口，常用材料厚度可直接选择写入。
- CPU2 继电器输出刷新去掉 pending 机制，TIM4 中断中直接调用 `RelayOutput_Update()` 并依赖模块内部重入保护。
- CPU2/CPU3 错误日志、屏幕错误原因、串口调试说明、无线滑环匹配文档和菜单文档同步改为蓝牙主从机语义。
- 同步新增本版本改动与测试方案，并更新版本索引、CPU2 参数存储升级清单和相关流程文档说明。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py -3 LTD_DISPLAY_CPU3\font_check.py`
- `git diff --cached --check`
- `py -3 tools\check_version_bumped.py`

未验证风险：
- 未做实物联调；需要现场验证蓝牙主机断电、蓝牙从机未连接、RSSI 查询超时、命令切换打断 AT 查询和 UART6 透传恢复。
- 需要在 OLED 实机上逐项确认继电器 R1~R4 通道设置、报警配置和报警状态页的返回路径，以及尺带厚度型号选择和手输路径。
- TIM4 中断直接执行继电器输出计算会增加中断内工作量，现场需重点观察 TIM4 周期、看门狗刷新、UART6 传感器通信和电机控制是否受影响。
- `CHANGELOG.pdf` 已包含当前暂存区原有 PDF 更新；本次提交未重新生成 PDF。

## 2026-06-25 - 补充 CPU2 宏定义中文注释（CPU2/CPU3 版本不变）

版本：
- CPU2: 保持 V1.17.0.0。
- CPU3: 保持 V1.16.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增或变更 CPU2/CPU3 共享寄存器、命令码、输入寄存器尾段或保持寄存器地址。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次提交触发恢复出厂参数。
- 本次仅补充 `#define` 宏定义中文块注释，不修改宏值、函数逻辑、状态机、协议字段或构建配置。

本次修改：
- 为 CPU2 自编代码中的应用层、服务层和 BSP 外设层宏定义补充中文说明，覆盖测量流程、Modbus 寄存器映射、电机/编码器控制、错误日志文本、TMC5130、AD5421、FRAM、AO 输出和扭力等定义。
- 对多行宏使用前置块注释，对普通宏使用行尾块注释，保持 C/C++ 注释风格统一使用 `/* ... */`。
- 暂存范围按注释行选择性暂存，未纳入工作区中已有的 AO 功能代码、Core 配置改动、文档改动或 PDF 改动。

验证：
- `git diff --cached --check`
- 暂存 diff 只包含 `#define` 行和块注释变化
- `git grep --cached -n "//" -- LTD_MAIN_CPU2/Application LTD_MAIN_CPU2/Services LTD_MAIN_CPU2/BSP/Peripherals` 无输出
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`

未验证风险：
- 注释-only 改动未做实物联调；不影响固件行为、对外协议或参数存储。
- 本次不改变入口、状态机、命令分发、测量流程或结果上报，程序流程图无需更新。
- 按项目 Markdown 规则，本次未重新生成或暂存 `CHANGELOG.pdf`。

## 2026-06-25 - 修复零点标定脱离误报并完善 AO 正式回读刷新（CPU2 V1.18.0.0）

版本：
- CPU2: V1.17.0.0 -> V1.18.0.0。
- CPU3: 保持 V1.16.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增或变更 CPU2/CPU3 共享寄存器、命令码、输入寄存器尾段或保持寄存器地址。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- 本次新增 CPU2 本地串口 AO 正式测试命令 `AO400/AO1200/AO2000/AO2200/AOS`，不影响 CPU3 外部协议和菜单契约。

本次修改：
- 修复丢步后零点标定脱离零点期间的扭力误报：回零/标零内部退让和脱离动作改用专用 `Zero_MoveDownWithoutWeightGuard()`，调用 `MotorCtrl_MoveBlockingNoDetectQuiet()` 跳过通用扭力碰撞误判，并在动作完成后用 `Weight_RebaseStableWeight()` 对齐稳定扭力基准。
- 为零点退让/脱离路径保留业务日志，只打印阶段名、下行距离、当前扭力和位置参考，不恢复底层无检测运行的密度/温度/频率刷屏。
- AD5421 正式读写路径增加访问互斥、控制寄存器回读、NOOP 读寄存器、诊断打印抑制和 READFAULT 诊断；硬件 PB8 未接入时不再读取悬空 FAULT 引脚。
- AO 自动刷新从主循环直接刷新改为 TIM4 置位请求、PendSV 最低优先级延后处理；TIM4 中断继续刷新继电器和看门狗，不直接访问 AD5421 SPI。
- 串口 AO 测试命令改为正式初始化、写电流和回读诊断路径，并在测试期间暂停定时 AO 自动刷新，避免测试和周期刷新争用 AD5421。
- 同步 CubeMX 配置：SysTick 优先级为 4，PendSV 优先级为 15，并保留生成代码中一处 PendSV 优先级配置。
- 修复 `tools/bump_version.py` 对带行尾注释版本宏的解析和替换能力，避免后续版本升级脚本因宏注释失败。
- 更新 AO 正式回读方案、硬件原理图问题清单、版本测试方案、参数存储升级清单和相关流程文档。

验证：
- `git diff --cached --check`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`

未验证风险：
- 未在本次提交流程重新做完整实物联调；需要现场覆盖丢步后零点标定、正常回零、罐底搜索、普通下行扭力保护、AO 4/12/20/22mA 电流输出和 HART/万用表回读。
- 当前现场日志曾出现 AD5421 写电流成功但 READFAULT 回读 `0xFFFF`；测试命令会保持输出便于测量，但正式运行仍会把 READFAULT 非 0 当故障处理，需继续结合电流环负载和硬件回读链路确认。
- AO 延后刷新使用 PendSV 处理 SPI 和少量状态写入，已经避开 TIM4 直接阻塞，但仍属于异常上下文，需要现场观察主循环阻塞、电机运动、UART6 传感器通信和看门狗刷新是否受影响。

## 2026-06-25 - 修复继电器报警数据源和息屏开关立即生效（CPU2 V1.18.1.0 / CPU3 V1.16.1.0）

版本：
- CPU2: V1.18.0.0 -> V1.18.1.0。
- CPU3: V1.16.0.0 -> V1.16.1.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 10，不新增或变更 CPU2/CPU3 共享寄存器、命令码、输入寄存器尾段或保持寄存器地址。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机显示参数存储结构和寄存器地址不变，只修复保存后的运行期同步时机。

本次修改：
- 修复修正液位和正常跟随更新液位后未恢复 `probe_at_liquid_level/liquid_stable` 的问题，避免继电器液位报警数据源被误判为 `99999` 无效值并长期不恢复。
- 继电器温度报警源新增初始值 `0` 无效判定，避免上电或温度尚未刷新时把 `0.00` 当作真实温度参与报警。
- CPU3 本机显示类参数写入后立即调用 `Cpu3Local_ApplyDisplayRuntimeParams()`，使息屏开关保存后立即更新 `screen_parameter.screenoff`，无需重启生效；亮度也统一走同一运行期同步路径。
- 按用户要求，本次提交不纳入当前工作区里的 `CHANGELOG.pdf`，也不纳入 AO/电流输出专题改动。
- 同步新增本版本改动与测试方案，并更新版本索引和 CPU2 参数存储升级清单。

验证：
- `git diff --check -- LTD_MAIN_CPU2\Services\Relay\relay_output.c LTD_MAIN_CPU2\Application\Src\measure_oilLevel.c`
- GBK 编码检查：`relay_output.c`、`measure_oilLevel.c` 无替换字符和双问号。
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `py tools\check_version_bumped.py`

未验证风险：
- 未做实物联调；需要现场确认液位修正后继电器液位报警值可恢复为真实液位，温度未刷新时不会触发低温误报警。
- 需要现场确认 CPU3 菜单修改息屏开关后无需重启即可按新值息屏/唤醒。
- 本次不改变报警策略中“非液位跟随状态是否强制报警”的业务语义，后续如要调整需要单独确认维护模式、手动运动和读参数场景。

## 2026-06-26 - 完善 AO 参数量程和协议文档（CPU2 V1.18.1.1 / CPU3 V1.16.1.1）

版本：
- CPU2: V1.18.1.0 -> V1.18.1.1。
- CPU3: V1.16.1.0 -> V1.16.1.1。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 从 10 升级到 11；不新增保持寄存器地址，但将原 `reserved24/reserved25` 参数位置重命名为 `AOStartLevel_01mm/AOEndLevel_01mm`，作为 AO 正常输出起点/终点液位，并修正 `AlarmHighAO/AlarmLowAO` 为 AO 独立报警液位阈值。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- 旧协议参数升级到协议版本 11 时，CPU2 运行期补齐 `AOStartLevel_01mm=0`、`AOEndLevel_01mm=tankHeight`、`AlarmHighAO=tankHeight`、`AlarmLowAO=0`，避免旧保留值或旧电流口径误触发 AO 报警。
- CPU3 本机参数存储结构不变；菜单和操作号名称按 AO 新语义同步，地址和值保持不变。

本次修改：
- AO 输出对齐参考程序：`CurrentRangeStart_mA/CurrentRangeEnd_mA` 保留为正常输出起点/终点电流，`AOStartLevel_01mm/AOEndLevel_01mm` 作为正常输出起点/终点液位，`AlarmHighAO/AlarmLowAO` 作为 AO 独立报警液位阈值；正常电流端点支持正向和反向映射。
- AO 相关保持寄存器宏和 CPU3 操作号改为 AO 液位、正常电流和报警液位口径，移除 `RESERVED24/RESERVED25` 和旧 `ALARM_HIGH_AO/ALARM_LOW_AO` 命名在业务代码中的继续使用；寄存器地址和值保持不变。
- AO 报警覆盖不再读取继电器 `HH/H` 或 `L/LL` 运行态；继电器报警阈值和 AO 报警阈值分开配置，AO 高/低报警触发后分别输出 `AOHighCurrent_mA/AOLowCurrent_mA`，重叠或越界 AO 报警阈值写参后关闭 AO 报警覆盖。
- AO 初始/高位/低位/故障/调试电流菜单范围和 CPU2 写参归一化统一限制为 `3.20..24.00mA`，与 AD5421 硬件输出范围一致；Modbus 写参后回写保持寄存器，便于读回确认。
- 同步 CPU3 菜单名称、参数范围、协议记录、说明书、程序流程文档和契约检查脚本；按用户要求，本次提交不纳入当前工作区里的 `CHANGELOG.pdf`。

验证：
- `git diff --cached --check`
- `python tools/check_version_bumped.py`
- `python tools/check_ao_output_enable_contract.py`
- `python tools/check_si_protocol_contract.py`
- `python tools/check_density_level_control_contract.py`
- `python tools/check_wireless_rssi_contract.py`
- `python tools/check_cpu3_menu_name_width.py`
- `cmake --build build/LTD_MAIN_CPU2`
- `cmake --build build/LTD_DISPLAY_CPU3`

未验证风险：
- 未做实物联调；需要现场确认 AO 正向/反向电流映射、非法 AO 参数写入后的归一化读回，以及特殊 AO 电流 `3.20..24.00mA` 边界输出。
- 需要现场确认 AO 高/低报警阈值与继电器 HH/H/L/LL 阈值互不影响，重叠或越界阈值写入后不会误触发报警覆盖。
- 本次保持寄存器地址不变但改变保留字段和报警字段语义，上位机和现场文档必须按协议版本 11 解释 AO 参数。

## 2026-06-26 - 汇总测量流程、显示参数和协议语义调整（CPU2 V1.19.0.0 / CPU3 V1.17.0.0）

版本：
- CPU2: V1.18.1.1 -> V1.19.0.0。
- CPU3: V1.16.1.1 -> V1.17.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 从 11 升级到 12；共享命令 115 从原强制提零点执行命令改为 `CMD_RESERVED_CMD7`，设备状态 `0x002F/0x802F` 改为保留状态。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0004`，存储结构不变；Wartsila 旧默认串口参数在加载时迁移到 2400 8E1，保存后沿用新默认值。
- CPU2/CPU3 必须同为协议版本 12；协议版本 11 的 CPU2/CPU3 可能仍存在命令 115 强制提零点语义或菜单入口，不建议混用。

本次修改：
- 删除 CPU2 强制提零点执行入口，CPU3 移除对应菜单映射和状态页文案，命令 115 仅作为保留命令存在，并新增契约脚本覆盖该语义。
- 修复液位和水位标定的 `0.1mm` 固定偏差：修正液位和水罐高度标定不再追加固定 `+1`，文档同步更新。
- CPU3 Wartsila 默认串口参数调整为 2400 8E1，并兼容旧默认 4800 8N1/8N2 参数迁移。
- 读取部件参数增加当前蓝牙连接 MAC 显示；CPU2 发布连接 MAC 快照，CPU3 菜单显示 `MAC:...` 或 `MAC N/A`。
- CPU3 增加电机电流 IRUN 档位选择和 RMS 电流显示，便于现场按电机驱动档位调整。
- 中文口径统一：当前业务文案和文档统一使用“液位”“扭力”，保留 `oilLevel`、`weight`、寄存器名等协议/代码标识符，CPU3 字库补充“扭”字。
- 梳理浮子运动和停机流程，删除电机运行监控中特定“停止当前运动?”二次确认分支，保留通用取消测量入口。
- 同步更新程序流程文档、协议文档、参数菜单文档、说明书资料、构建版本索引和相关检查脚本；本次按用户要求提交当前工作区全部改动。

验证：
- `py -3 tools\check_reserved_cmd7_contract.py`
- `py -3 tools\check_wireless_rssi_contract.py`
- `py -3 tools\check_ao_output_enable_contract.py`
- `py -3 tools\check_cpu3_menu_name_width.py`
- `py -3 LTD_DISPLAY_CPU3\font_check.py`
- `py -3 tools\check_water_precise_search_contract.py`
- `py -3 tools\check_version_bumped.py`
- `git diff --cached --check`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`

未验证风险：
- 未做实物联调；需要现场覆盖命令 115 不再触发电机动作、CPU3 菜单无强制提零点入口、取消测量仍能停机、液位/水位标定无固定 `+1` 偏差。
- 需要现场确认 Wartsila 2400 8E1 默认通信、旧 FRAM 默认串口参数迁移、读取部件参数 MAC 显示、电机电流档位设置和实际运行电流匹配。
- 本次提交包含大量口径统一和文档重生成，后续如上位机或外部资料仍引用“重量/油高/强制提零点”，需要按协议版本 12 继续清理。

## 2026-06-26 - 密度两位精度、参数菜单排序和外部协议兼容整理（CPU2 V1.20.0.0 / CPU3 V1.18.0.0）

版本：
- CPU2: V1.19.0.0 -> V1.20.0.0。
- CPU3: V1.17.0.0 -> V1.18.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 从 12 升级到 13；内部密度 raw 从 `kg/m3 x10` 升级为 `kg/m3 x100`。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小不变，不会因本次升级触发恢复出厂参数；旧 FRAM 中 `protocolVersion < 13` 时会一次性迁移 `oilLevelDensity`、`oilLevelThreshold`、`oilLevelHysteresisThreshold` 和 `densityCorrection`。
- CPU3 本机参数存储版本升级到 `0x0005`，加载 `0x0003` 或 `0x0004` 时迁移本机手输密度 `screen_input_d`，并保留旧屏幕亮度默认值迁移路径。
- CPU2/CPU3 必须同为协议版本 13；协议版本 12 和 13 的内部密度字段倍率不同，不建议混用。
- DSM、Wartsila、SI协议边界继续保持原对外密度口径，主站不需要随内部 raw 倍率调整。

本次修改：
- CPU2/CPU3 密度计算、参数存储、状态页显示和参数菜单统一支持两位小数密度口径。
- CPU2 液位找液/滞后阈值兼容旧频率路径：内部按 `kg/m3 x100` 保存，频率找液方案按旧倍率折算回历史 Hz 阈值使用。
- 修复液位频率跟随稳定判断中对 `frequency_difference` 使用整数 `abs()` 的问题，改用浮点绝对值比较。
- CPU3 参数菜单按导出的拖拽排序结果重排，新增本地 `tools/cpu3_menu_sorter.html` 辅助后续菜单整理。
- CPU3 继电器相关菜单和文档显示名由 R1~R4 统一为 K1~K4，内部 `RELAY*` 协议符号保持不变。
- 默认 AO 高报警液位改为罐高，写参归一化时超罐高钳位到罐高，低报警异常或高低重叠时恢复高报警罐高、低报警 0。
- DSM、Wartsila、SI协议密度边界转换和协议文档同步整理，补充密度精度、综合测量密度模式和 SI协议契约检查脚本。
- 将密度两位小数改造方案从“未实现”移动到“已实现”，同步 README、默认值表、协议变更记录、菜单说明、设备说明书和相关工具文档。

验证：
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_cpu3_menu_name_width.py`
- `py tools\check_reserved_cmd7_contract.py`
- `py tools\check_density_precision_contract.py`
- `py tools\check_synthetic_density_mode_contract.py`
- `py tools\check_si_modbus_frames.py`
- `py tools\check_si_protocol_contract.py`
- `git diff --check`

未验证风险：
- 未做实物联调；需要现场确认密度两位小数显示、LTD 自有协议密度读写、旧 FRAM 密度参数迁移和频率找液兼容折算。
- 需要现场遍历 CPU3 参数菜单拖拽排序结果，确认跨菜单移动、K1~K4 命名和只读状态页路径符合最终操作习惯。
- 需要现场确认 AO 高报警默认罐高和高低报警归一化策略符合输出电流报警预期。

## 2026-06-27 - 修复参数打印、分布悬停和 CPU3 菜单空闲退出（CPU2 V1.20.1.0 / CPU3 V1.18.1.0）

版本：
- CPU2: V1.20.0.0 -> V1.20.1.0。
- CPU3: V1.18.0.0 -> V1.18.1.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 13，不新增共享寄存器、命令码或输入寄存器长度。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小、FRAM A/B 分区地址、`magic/struct_size/param_version/crc` 校验规则不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0005`，不改变本机 FRAM 布局。
- 本次会改变参数写入后的串口诊断输出、分布密度悬停时间执行口径和 CPU3 菜单空闲退出行为，建议 CPU2/CPU3 成对升级以便现场日志和菜单行为一致。

本次修改：
- CPU2 参数打印改为统一场景入口：上电、恢复出厂、人工打印、保存摘要和保存跳过分别输出结构化日志；Modbus 写持久化参数前捕获旧参数快照，延后保存时打印变更差异。
- CPU2 0x10 写设备参数的持久化判定收敛：只写 command 或继电器清锁存命令不触发 FRAM 保存；真正覆盖持久化参数区时才请求保存并打印差异。
- 分布密度逐点悬停统一按菜单配置的 `spreadPointHoverTime` 秒解释，进入 `HAL_Delay()` 前换算为毫秒，和单点稳定窗口口径一致。
- CPU2 单点展示与调试密度 raw 统一到 `kg/m3 x100`，DSM V2 调试读取使用 `DENSITY_TO_RAW()`，演示打印显示两位密度。
- CPU2 编码器、电机初始化和位置源恢复日志改为结构化中文输出，默认关闭上电位置详细诊断，减少启动日志刷屏。
- CPU3 菜单 120 秒无有效操作后自动退出到状态页；电机运行监控页和扭力等待页不参与普通菜单空闲退出，避免中断业务等待。
- CPU3 显示菜单将语言和屏幕密码提升到显示设置页，参数返回映射按实际入口修正；保留项过滤改为完整 UTF-8 前缀比较。
- CPU3 液位找液阈值、液位滞后阈值菜单显示恢复为 `Hz` 且保留 1 位小数，水位稳定距离和罐底后编码器修正等参数顺序/短名同步调整。
- 同步密度流程页、协议记录、默认值说明、CPU3 菜单单位清单、版本测试方案和相关契约检查脚本。

验证：
- `git diff --cached --check`
- `py tools\check_density_level_control_contract.py`
- `py tools\check_density_precision_contract.py`
- `py tools\check_version_bumped.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`

未验证风险：
- 未做实物联调；需要现场通过 Modbus/CPU3 菜单写入多类参数，确认参数差异打印、保存跳过、继电器清锁存不保存和 FRAM A/B 保存摘要符合预期。
- 需要现场执行分布密度测量，确认 `spreadPointHoverTime` 从原误按毫秒执行修正为按秒执行后，节拍符合工艺要求。
- 需要 CPU3 实机确认菜单 120 秒空闲自动退出不会误退出电机运行监控和扭力等待页，语言/屏幕密码入口和返回路径符合操作习惯。

## 2026-06-29 - 修复编码器通信诊断和传感器上电 AT 干扰（CPU2 V1.20.2.0 / CPU3 V1.18.2.0）

版本：
- CPU2: V1.20.1.0 -> V1.20.2.0。
- CPU3: V1.18.1.0 -> V1.18.2.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 13，不新增共享命令、共享寄存器、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 参数校验规则不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0005`，不改变本机 FRAM 布局。
- 本次改变 CPU2 编码器错误上报、CH9141K 进入 AT 前的启动 `%` 过滤、全局错误归因以及 CPU3 轮询节流，建议 CPU2/CPU3 成对升级。

本次修改：
- CPU2 AS5145 编码器中断路径取消单帧错误立即重试和日志刷屏，改为累计错误计数、稳定有效帧门控和连续错误阈值后再进入全局错误状态。
- CPU2 记录编码器 parity、OCF、COF、空帧、SPI busy 和 DMA 启动失败计数，保留最后一次错误原始帧，便于现场诊断编码器校验失败和 OCF 未完成。
- CPU2 调整 DMA、TIM4、USART1/2/6 等中断优先级，降低串口和低优先级 DMA 对编码器采样链路的抢占压力。
- CPU2 编码器启动就绪等待从 300 ms 延长到 1500 ms，配合连续有效帧门控降低刚上电未稳定时的误判。
- CPU2 CH9141K 软件进入 AT 逻辑只在设备启动或传感器重新上电后的第一次响应中识别单独 `%` 或 `%\r\n`，丢弃后立即重试一次，不增加普通 AT 查询、RSSI 扫描和运行中通信等待。
- CPU2 故障管理新增全局错误状态归因路径，`CHECK_ERROR` 和运动速度范围检查遇到已有全局错误时记录“来源：全局错误状态”，避免把底层异步错误误归因为当前函数返回值。
- CPU3 空闲态 CPU2 轮询延时从 10 ms 调整为 100 ms，降低 CPU2/CPU3 空闲通信压力。
- 同步编码器和无线滑环问题分析 HTML、版本测试方案，以及程序流程文档生成工具中的正式 `docs/` 路径。

验证：
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`

未验证风险：
- 未做实物联调；需要现场长时间运行确认编码器校验失败、OCF 未完成和蓝牙主机无响应是否收敛。
- 需要现场覆盖传感器断电后首次上电 `%` 透传场景，确认首次进入 AT 会立即重试且不影响 RSSI 扫描、普通 AT 查询和运行中通信时效。
- 需要示波器或现场日志复核中断优先级调整后编码器采样、串口 DMA、TIM4 电流刷新和 CPU2/CPU3 轮询之间没有新的实时性回归。

## 2026-07-01 - 调整 AS5145 编码器 SSI 错误重试和持续故障上报（CPU2 V1.20.3.0 / CPU3 V1.18.2.0）

版本：
- CPU2: V1.20.2.0 -> V1.20.3.0。
- CPU3: V1.18.2.0 保持不变。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 13，不新增共享命令、共享寄存器、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 参数校验规则不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0005`，不改变本机 FRAM 布局。
- 本次只调整 CPU2 AS5145 编码器 SSI 错误重试、持续故障日志和有效帧门控策略；CPU3 固件行为不变，可继续使用 V1.18.2.0。

本次修改：
- CPU2 AS5145 SSI 错误处理恢复前 3 次立即重试，超过阈值后只记录一次持续故障详情，不停止 TIM1，也不阻断后续定时采样。
- CPU2 有效帧门控改为单帧完整解析且无协议错误后即标记首个可信位置，避免旧连续有效帧门控导致上电后可用位置迟滞。
- CPU2 清理编码器错误诊断中的未使用统计计数和原始帧缓存，减少中断路径共享状态。
- CPU2 SPI busy 和 DMA 启动失败路径改为记录编码器超时错误和一次性诊断输出，不再递归进入 SSI 错误重试流程。
- 同步提交标题版本号门禁脚本、本地 commit-msg 钩子、AGENTS 和版本测试 README 的提交流程说明。
- 整理 SI协议进一步兼容需求、CPU2 全局错误响应滞后分析、LNG 菜单核对资料和 SIL 外部资料索引。

验证：
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- `py -m unittest tools.test_check_commit_subject_versions`
- `py tools\check_commit_subject_versions.py --subject "fix: 调整 AS5145 编码器 SSI 错误重试和持续故障上报（CPU2 V1.20.3.0 / CPU3 V1.18.2.0）"`
- `cmake --build build\LTD_MAIN_CPU2`

未验证风险：
- 未做实物联调；需要现场覆盖 AS5145 断线、空帧、OCF 未完成、COF 和偶发校验错误，确认 3 次重试后日志不刷屏且通信可在有效帧恢复后自恢复。
- 未重新构建 CPU3；本次没有 CPU3 源码行为改动，CPU3 仅作为提交标题和文档资料的版本参照。
- 本次包含文档和提交门禁工具整理，需要推送前按仓库要求运行 `py tools\check_commit_subject_versions.py --range origin/MAIN..HEAD`。

## 2026-07-01 - 落地 SI协议独立 Profile 兼容（CPU2 V1.21.0.0 / CPU3 V1.19.0.0）

版本：
- CPU2: V1.20.3.0 -> V1.21.0.0。
- CPU3: V1.18.2.0 -> V1.19.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION`: 13 -> 14。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 参数校验规则不变，不会因本次升级触发恢复出厂参数。
- CPU2 复用原 `reserved30~reserved33` 作为 SI profile 首点、步距、停留和探底频次；旧协议存储会按 `protocolVersion < 14` 或运行期归一化补默认值。
- CPU3 本机参数版本升至 `0x0006`，新增 SI 自动 profile 和报警限值参数，旧 V3/V4/V5 本机参数按迁移逻辑补默认值。
- CPU2/CPU3 必须同为协议版本 14 才能正确识别 `CMD_SI_PROFILE`、SI profile 参数、`profile_source` 和输入寄存器偏移；协议 13 不能混用。

本次修改：
- 新增 `CMD_SI_PROFILE = 20`，SI `00004 Profile`、屏幕 SI Profile 和自动调度均下发独立 SI profile 命令，不再复用普通分布测量。
- CPU2 新增 SI profile 执行流程，按探底频次处理上电首次探底、旧底部位置回退、Point0、最多 200 点和液面以上点停止。
- CPU2 在普通分布、国标、每米、间隔、Wärtsilä 和 SI profile 完成后写入 `profile_source`，CPU3 SI 协议只认 `PROFILE_SOURCE_SI` 的完成态和点阵。
- 修复 SI profile 逐点采样过程中命令切换仍可能用部分有效点进入完成态的问题，命令切换统一返回 `STATE_SWITCH`。
- CPU3 将 `40001~40003` 桥接到 CPU2 SI profile 参数，将 `40010~40023` 保存为 CPU3 SI 本机参数，并落地自动 profile 调度。
- CPU3 菜单新增 `SI参数`，包含 Profile 参数、自动 Profile 和报警限值；`SI首点/SI步距/SI停留/SI探底` 均启用屏幕侧范围校验。
- SI 温度无效值统一输出 `-200.00°C`，密度无效值保持 `0`；温度限值按 signed int16 解释，报警阈值 `0` 作为有效值参与判断。
- 同步 SI 协议映射、PLC 联调检查表、协议变更记录、程序流程页、版本测试方案和 Wartsila 原始资料 README。

验证：
- `py tools\check_si_protocol_contract.py`
- `py tools\check_si_modbus_frames.py`
- `py tools\check_cpu3_menu_name_width.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `py tools\check_docs_structure.py`
- `py tools\check_markdown_links.py docs\01_协议与寄存器\SI协议适配`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

未验证风险：
- 未做真实 SI PLC 或科学仪器主机联调；需要现场覆盖 `00004 Profile`、屏幕 SI Profile、自动 profile、报警限值和负温度限值。
- 需要现场验证探底失败、无旧底部位置、旧底部位置回退、液面以上点停止和命令切换打断的完整测量过程。
- SI 输入寄存器仍按 DCS 表输出 200 个 profile 点；超过 200 点需要另行设计扩展地址或分页机制。

## 2026-07-02 - 修复 SI Profile 点位和无检测运行通信（CPU2 V1.21.1.0 / CPU3 V1.19.1.0）

版本：
- CPU2: V1.21.0.0 -> V1.21.1.0。
- CPU3: V1.19.0.0 -> V1.19.1.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14，不新增共享命令、共享寄存器、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 参数校验规则不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0006`，不改变本机 FRAM 布局。
- 仍要求 CPU2/CPU3 同为协议版本 14 才能正确使用独立 SI Profile 命令、参数和输入寄存器点阵。

本次修改：
- CPU2 SI Profile 首点按 `40001` 配置的绝对位置执行，不再叠加罐底位置。
- CPU2 SI Profile 点 0 上报位置固定为 0，后续点上报实际绝对运动位置，并同步到存储结果中的温度位置字段。
- CPU2 SI Profile 到点后先按低密度频率判定空气点，再等待无效频率，避免空气点因零频或无效频率等待被误判为失败。
- CPU2 普通无检测阻塞运动不再启用带密度传感器通信的运行日志；只有强制调试无检测运行继续启用该通信路径。串口测试命令 `B/BE` 的 `S`/`,1` 单独控制路径保持独立。
- CPU3 本机参数写入绕过 CPU2 测量状态限制，CPU2 状态禁止普通参数写入时仍允许保存 CPU3 本机参数。
- CPU3 将 SI 自动 Profile 使能显示为布尔选项，并在探底、Wärtsilä 和 SI Profile 相关菜单中显示探底修正罐高参数。
- 同步 SI 协议契约检查脚本、SI 官方资料中文整理、故障码统一表、LNG 菜单文档和相关 README/索引。

验证：
- `git diff --cached --check`
- `py tools\check_si_protocol_contract.py`
- `py tools\check_version_bumped.py`
- `py tools\check_docs_structure.py`
- `py tools\check_markdown_links.py docs\01_协议与寄存器\SI协议适配`
- `py tools\check_cpu3_menu_name_width.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `cmake --build build\LTD_MAIN_CPU2`
- `cmake --build build\LTD_DISPLAY_CPU3`

未验证风险：
- 未做真实 SI PLC 或科学仪器主机联调；需要现场覆盖 `00004 Profile`、屏幕 SI Profile、自动 Profile、点 0 输出、绝对位置输出和空气点停止。
- 未做真实密度传感器运行抓包；需要现场确认普通无检测运行不再产生密度通信，强制调试无检测和串口测试命令仍按预期通信。
- 未做 CPU3 实机 FRAM 保存和 OLED 菜单逐项验证；需要现场确认 CPU2 忙态下 CPU3 本机参数可保存，新增菜单项显示不截断。

## 2026-07-06 - 补齐 SI Profile 完成后自动回液位跟随（CPU2 V1.21.2.0 / CPU3 V1.19.1.0）

版本：
- CPU2: V1.21.1.0 -> V1.21.2.0。
- CPU3: V1.19.1.0 保持不变。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14，不新增共享命令、共享寄存器、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 参数校验规则不变，不会因本次升级触发恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0006`，不改变本机 FRAM 布局。
- 仍要求 CPU2/CPU3 同为协议版本 14 才能正确使用独立 SI Profile 命令、参数和输入寄存器点阵。

本次修改：
- CPU2 SI Profile 成功锁存并进入完成态后，如没有其它待执行命令，自动排队 `CMD_FIND_OIL`，下一轮主循环恢复找液位和液位跟随。
- CPU2 SI Profile 失败或执行期间检测到有效命令切换时，不覆盖外部待执行命令。
- SI 协议契约脚本新增静态断言，检查 `CMD_SiProfile()` 完成后只在无命令切换时排队 `CMD_FIND_OIL`。
- 同步 SI 协议需求总览、兼容映射表、PLC 联调检查表和程序流程页，补齐 Profile 完成后回 Auto 的生命周期说明。
- 整理传感器新一代安全通信协议卷、SI 官方资料中文整理、SIL 功能安全外部资料、CPU3 当前屏幕菜单树和相关 README/索引。

验证：
- `git diff --cached --check`
- `py tools\check_si_protocol_contract.py`
- `py tools\check_si_modbus_frames.py`
- `py tools\check_docs_structure.py`
- `py tools\check_markdown_links.py docs\01_协议与寄存器\SI协议适配`
- `py tools\check_version_bumped.py`
- `cmake --build build\LTD_MAIN_CPU2`

未验证风险：
- 未做真实 SI PLC 或科学仪器主机联调；需要现场覆盖 `00004 Profile`、屏幕 SI Profile 和自动 Profile 完成后自动回液位跟随。
- 未做真实液位跟随长时间运行；需要现场确认 SI Profile 完成态可被外部读取到，随后自动找液位不会覆盖新的手动/维护命令。
- 本次包含既有暂存区中的 PDF、docx、xmind、opml 等资料整理，二进制文档未在本轮逐页视觉复核。

## 2026-07-08 - 优化液位跟随滞后确认与定频速度闭环（CPU2 V1.21.3.0 / CPU3 V1.19.1.0）

版本：
- CPU2: V1.21.2.0 -> V1.21.3.0。
- CPU3: V1.19.1.0 保持不变。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14，不新增共享命令、寄存器地址、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 校验规则不变，从 V1.21.2.0 升级不会因参数存储版本恢复出厂参数。
- `oilLevelHysteresisTime` 保留为液位滞后时间参数；本版本明确仅用于方法 0/1 跟随变化确认，速度闭环死区等待改为固定 1000 ms。旧 FRAM 参数值保留，现场如原先依赖该值影响速度闭环等待，需要重新核对参数配置。

本次修改：
- 方法 0/1 液位跟随在频率偏差超滞后阈值后，按 `oilLevelHysteresisTime` 连续确认液位变化；参数为 0 时立即重找，超过 60 s 时按 60 s 生效。
- 方法 0/1 跟随重找期间保持 `STATE_FLOWOIL`，不再切回 `STATE_FINDOIL`；中间液位按既有设计继续发布到 AO 和继电器。
- 方法 5 固定频率方式直接校验 `oilLevelFrequency` 并进入定频速度闭环；方法 1 保持固定频率步进精找，避免复用速度闭环。
- 拆分 `SearchOilLevel()`、`SearchOilPrecise()` 和液位发布相关 helper，统一方法 0/1/2/3/4/5 的找液位和跟随边界。
- 同步液位流程 HTML、软件详细设计、函数调用关系和系统参数默认值文档。

验证：
- `git diff --cached --check`
- `py tools\check_version_bumped.py`
- `cmake --build build\LTD_MAIN_CPU2`

未验证风险：
- 未做真实液位长时间跟随和 AO/继电器硬件联调；需要现场覆盖方法 0/1 滞后确认、方法 5 定频速度闭环和跟随重找期间输出状态。

## 2026-07-08 - 修复编码轮模式速度补偿参考长度（CPU2 V1.21.4.0 / CPU3 V1.19.1.0）

版本：
- CPU2: V1.21.3.0 -> V1.21.4.0。
- CPU3: V1.19.1.0 保持不变。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14，不新增共享命令、寄存器地址、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 校验规则不变，从 V1.21.3.0 升级不会因参数存储版本恢复出厂参数。
- 本次只修正 CPU2 电机速度补偿计算使用的运行期长度来源，不改变参数含义、存储布局、CPU3 菜单或外部协议契约。

本次修改：
- CPU2 运行中速度刷新统一通过参考长度 helper 计算 VMAX，编码轮模式先刷新编码轮业务位置，再使用 `g_measurement.debug_data.cable_length` 作为补偿长度。
- 修复编码轮模式下运行刷新误用 `drum.motor_distance_01mm` 的问题，避免长距离下行分段后把本段局部电机长度当成实际尺带长度，导致 `VMAX` 被压低、速度越来越慢。
- 运行中重新下发速度也复用同一参考长度 helper，避免改速、恢复速度和慢速切换路径保留同类取值问题。
- 速度刷新日志新增 `模式`、`业务尺带`、`电机尺带` 和 `使用尺带`，便于现场确认编码轮模式下实际参与计算的是业务尺带长度。

验证：
- `git diff --check -- LTD_MAIN_CPU2\Services\MotorControl\motor_ctrl_driver_param.c`
- `cmake --build build\LTD_MAIN_CPU2`
- `py tools\check_version_bumped.py`

未验证风险：
- 未做真实 55 m 下行复测；现场需要确认 `模式=编码轮` 时 `使用尺带` 跟随 `业务尺带`，不再等于接近 0 的 `电机尺带`。
- 未做长时间连续速度模式实机复测；本次覆盖运行刷新和运行中改速路径，连续速度模式仍需现场按实际命令入口确认刷新节奏。

## 2026-07-08 - 修复 AD5421 断环重接恢复与 AO 运行态错误处理（CPU2 V1.21.5.0 / CPU3 V1.19.1.0）

版本：
- CPU2: V1.21.4.0 -> V1.21.5.0。
- CPU3: V1.19.1.0 保持不变。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14，不新增共享命令、寄存器地址、输入寄存器长度或跨 CPU 状态字段。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` 结构大小和 FRAM 校验规则不变，从 V1.21.4.0 升级不会因参数存储版本恢复出厂参数。
- 本次只改变 CPU2 AO/AD5421 运行期恢复和错误处理，不改变 CPU3 菜单、外部寄存器契约或参数语义。

本次修改：
- AD5421 驱动新增按目标电流恢复接口，恢复序列保持复位、控制寄存器写入与回读校验、目标电流写入和 `READFAULT` 诊断。
- AO 运行期发现 AD5421 诊断异常后，按 1 秒节流尝试恢复到当前目标或最近一次有效目标；恢复成功后不再写入故障电流。
- AO 运行期 AD5421 读写或恢复异常只记录 AO 运行态和驱动故障标志，不再由后台刷新直接触发整机最终错误。
- 启动期 AO 初始化异常只打印并保留 AO 运行态，不再阻塞整机测量流程。
- 修正 CubeMX 生成标签中 PB4/PB5、PB14/PB15、PD5/PD6 的 MISO/MOSI/TX/RX 命名，保留 AD5421 CS 独立初始置高，避免 PB6 PinState 生成合并写影响 AD5421 回读。
- 同步 AD5421 断环重接问题分析文档，记录手册依据、代码落地、PB6 配置根因定位和 `AO400` 台架验证结果。

验证：
- `cmake --build build\LTD_MAIN_CPU2`
- `STM32_Programmer_CLI -c port=SWD mode=UR -w D:\CUBE\build\LTD_MAIN_CPU2\LTD_MAIN_CPU2_V1.21.5.0.hex -v -rst`
- 串口 `COM12` 自动发送 `AO400`，0ms 到 4500ms 持续 `fault=0x0000`、`flags=0x00000000`，未出现 AD5421 控制寄存器回读失败或 `0xFFFF`。
- `V1.21.5.0` 烧写校验后，提交前串口复测因 `COM12` 被占用未重跑；`AO400` 结果来自同一代码逻辑升版前的台架验证。
- `cmd /c fc /b D:\CUBE\build\debug-logs\LTD_MAIN_CPU2_V1.21.4.0_ao_logic_only_working.hex D:\CUBE\build\LTD_MAIN_CPU2\LTD_MAIN_CPU2_V1.21.4.0.hex`
- `git diff --cached --check`
- `py tools\check_version_bumped.py`

未验证风险：
- 已验证 `AO400` 正常回读和输出刷新，尚未做真实断环、重接、自动恢复到断环前目标电流的连续 10 次台架循环。
- 尚未接入 HART 通信场景验证断环恢复瞬态对 HART 通信的影响。
- 尚未用逻辑分析仪抓取断环恢复周期内 `WRITECONTROL`、`READCONTROL`、`WRITEDAC`、`READFAULT` 帧。

## 2026-07-10 - 修复传感器编号和上电蓝牙等待并闭环 CPU2 通信与外部写失败反馈（CPU2 V1.21.6.0 / CPU3 V1.20.0.0）

版本：
- CPU2: V1.21.5.0 -> V1.21.6.0。
- CPU3: V1.19.1.0 -> V1.20.0.0。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14；本次不新增或调整 CPU2/CPU3 共享命令、寄存器地址、结构体字段、字段长度或内部参数语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters` / `struct_size`、FRAM A/B 地址和校验范围不变；从 V1.21.5.0 升级不会因本次版本变化恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0006`，本机 FRAM 布局不变；从 V1.19.1.0 升级不会因本次升版重建本机参数，无需重新下发，升级后回读核对 `40010~40023`。
- CPU3 依赖 CPU2 的普通命令和参数写入口新增完整状态快照、完整参数快照及当前连接协议快照三重门禁；只有合法 `0x03` 实际覆盖协议字段并读回版本 14 后，协议快照才有效。协议不匹配时仍允许轮询和状态显示，但禁止共享命令或参数写入。参数刷新期间仅屏幕 `CMD_CANCEL_MEASUREMENT` 停止/取消入口保持可达，且仍要求状态快照、当前连接协议快照、协议 14 和无通信故障；SI `00009 Stop` 仍映射维护模式，不使用该绕过。
- DSM、SI、Wartsila 对 CPU2 相关写入在链路未就绪、协议不匹配或 CPU2 未确认时返回 Modbus 异常 `0x06`；DSM FC03 第 1～5 段在参数快照无效期间同样返回 `0x06`，第 6 段全零占位继续正常响应。这是 CPU3 外部适配层行为变化，不提升 CPU2/CPU3 共享协议版本；外部主站需等待补读完成后重试并回读确认。

本次修改：
- CPU2 DSM 文本响应接受 `N`、`E`、`e` 编号前缀，修复低电压响应在上电阶段误报 `13-11`；十进制编号解析增加 `uint32_t` 上界保护，接受 `4294967295`，拒绝 `4294967296` 和超长数字串。
- CPU2 清理无调用的 Modbus 异常封装、AO 旧包装入口、Wartsila 旧运动入口和液位状态旧导出包装，保留仍用于台架测试的水位电容曲线入口。
- CPU2 CH9141 UART6 清空同时使用连续空闲和固定总时限：上电/退出准备要求连续空闲 `500 ms`、总时限 `1000 ms`，每条 AT 命令前及失败恢复要求连续空闲 `30 ms`、总时限 `200 ms`；从机持续透传或命令切换时返回既有状态码，不再把 `DetectSensorType()` 永久卡在蓝牙链路检查，也不新增错误码或 CPU2/CPU3 协议字段。
- CPU3 使用独立连续失败计数归一统计响应超时、非法长度/CRC/地址/功能码/写回显、UART 错误和 TX DMA 启动失败；连续第 10 次失败锁存本机故障 `CPU2_COMM_TIMEOUT (0x0013000A)`。
- CPU3 任一合法 `0x03`/`0x04`/`0x10` 响应均清连续失败计数；但只有完整覆盖设备状态和错误码的合法 `0x04` 才建立首次状态快照并解除已锁存通信故障，其它 `0x04` 不得解除故障。通讯尝试页要等状态快照和当前连接协议快照都建立后才退出，避免协议字段尚未读回时短暂显示伪 `ProtoErr` 或掉线前兼容结果。通信故障会同时清除参数快照和协议快照；状态 `0x04` 恢复后仍需重建完整参数快照，并重新收到覆盖协议字段且值为 14 的合法 `0x03`，才重新开放普通写入口。
- CPU3 参数刷新期间临时关闭普通写门禁，补读完成后同步刷新 Wartsila 寄存器镜像；任何已发起的非命令 FC16 未获得合法响应时，都立即使参数快照失效并强制全量补读，覆盖 CPU2 已生效但 ACK 丢失、DSM/SI/菜单单参数失败和多字段部分成功场景。屏幕 `CMD_CANCEL_MEASUREMENT` 在状态快照、当前连接协议快照有效、协议兼容且无通信故障时仍可下发，避免参数刷新阻断停止运动。批量参数同步永久跳过命令字段，禁止旧命令跨协议重放。
- CPU3 外部 COM 持续有流量时仍按主循环 100 ms 调度门限检查 CPU2 轮询，UART5 ISR 只记录错误并由主循环统一处理。
- CPU3 菜单命令、参数下发和运动停止请求失败时不再进入成功页或运动监控页；参数读回失败走失败出口，已经处于取消/空闲状态的取消请求按幂等成功收口。
- CPU3 同时清理未使用的 AUBON Logo 位图、旧调试命令判定 helper 和旧共享通信计数变量；OLED CRC/刷新序号注释按实际语义修正，不改变运行行为。
- CPU3 `cpu2_communicate.h` 显式引入 `CommandType` 所在的 `system_parameter.h`，修复新增命令门禁原型在部分编译单元中先被包含时的 `unknown type name` clean-first 编译失败。
- CPU2/CPU3 CMake 把固定名 HEX、当前版本 HEX 和 BIN 声明为 `POST_BUILD BYPRODUCTS`；`clean`/`clean-first` 会先移除本配置当前产物，避免本轮编译或链接失败后误拿同名旧固件。历史版本 HEX 不在自动清理范围内，仍按版本归档保留。
- DSM FC05 动作线圈 ON、非第 6 段占位的 FC16、SI FC05 ON 与 `40001~40003`、Wartsila `0x0006` 与 `0x005A~0x005D` 均依赖 CPU2 ACK；DSM FC16 写前保存已确认参数和本次寄存器镜像，中途失败时先回滚 CPU3 本地影子、关闭参数快照并强制补读，补读完成前 FC03 第 1～5 段返回 `0x06`，不得把未确认请求值或分组混合值伪装成成功回读。CPU2 端多字段提交仍非原子，最终以补读后的实际值为准。SI 影子值只在 ACK 后提交，CPU3 本机参数 `40010~40023` 仍可独立保存。
- DSM 分发层在地址和 CRC 通过后、进入无长度参数的响应函数前校验实际 RTU 帧长：FC01/03/04/05 必须为 8 字节，FC16 必须满足实际长度等于 `9 + byteCount`；截短、超长或声明长度不一致的合法 CRC 帧返回 `0x03`，不得把 CRC 或尾随字节误作参数数据。FC06 仍保持非法功能 `0x86/0x01`。
- SI 自动 Profile 下发失败后按 5 s 节流重试，失败不锁存本分钟成功标记；只有 CPU2 接受命令后才对本分钟去重。
- Wartsila 跨区 FC16 先同步参数再下发命令，任一步失败返回 `0x06`；参数阶段失败时恢复 CPU3 写前四参数镜像、清除命令影子、立即关闭普通命令门禁并强制补读 CPU2 实际值，补读完成前读取 `0x005A~0x005D` 也返回 `0x06`，不得把未确认目标值伪装成成功回读。命令尝试结束后同样清除非零命令影子，CPU2 拒绝后写 0 或其它协议参数同步均不会重放旧命令。多字段同步仍为非原子过程，失败前已确认字段不自动回滚，最终以补读后的 CPU2 实际值为准。
- 同步故障码 Excel、LNG 菜单维护版 Word、DSM/SI/Wartsila 协议资料、状态页确认表、问题整改记录、CPU2/CPU3 程序流程页和版本改动与测试方案；修正 CPU2 传感器流程页中仍沿用的密度修正 `x10` 旧公式，使其与当前 `x100` 源码口径一致。
- 流程文档工具链改为可复现生成：CPU3 生成器直接产出统一网站包装、稳定页面/图形 ID、嵌入标记、figure/viewport 结构及 SVG title/desc/ARIA，并动态解析关键函数源码行号；每个 SVG `desc` 使用独立业务摘要，不得只重复 `title` 或 `figcaption`。导航工具增加幂等安全更新、重复 ARIA 属性治理和 Windows 瞬时文件占用下的原子替换有界重试，新增流程 HTML 契约检查、生成器摘要契约测试、格式规范及单元测试并接入文档总门禁。同步修复 CPU2 页面重复 `aria-labelledby`，将 CPU3 FRAM 流程口径更正为 V3/V4/V5 迁移至当前 V6（`0x0006`）。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`
- `py tools\check_wireless_rssi_contract.py`
- `py tools\test_dsm_cn_low_voltage_response.py`
- `py tools\check_dsm_compat_contract.py`（覆盖 DSM FC03 参数快照门禁、第 6 段豁免、FC16 写前双快照与失败回滚顺序、实际 RTU 帧长及 FC06 非法功能边界）
- `py tools\check_density_precision_contract.py`
- `py tools\check_cpu3_cpu2_comm_timeout_fault_contract.py`
- `py tools\check_cpu3_display_isr_boundaries.py`
- `py tools\check_cpu3_fault_reason_visibility.py`
- `py tools\check_si_protocol_contract.py`
- `py tools\check_si_modbus_frames.py`
- `py tools\check_wartsila_distribution_logic.py`
- `py tools\check_cpu3_menu_name_width.py`
- `py LTD_DISPLAY_CPU3\font_check.py`
- `py tools\test_generate_cpu3_flow_docs.py`
- `py tools\test_check_flow_docs.py`
- `py tools\check_flow_docs.py`
- `py tools\test_update_flow_navigation.py`
- `py tools\test_check_docs.py`
- `py tools\check_docs.py`
- `py tools\check_docs_structure.py`
- `py tools\check_markdown_links.py`
- CPU3 流程生成器与导航连续执行两遍，44 个目标文件 SHA-256 变化为 0；流程契约覆盖 32 页、178 张 SVG 和 47 张表格。
- `git diff HEAD --check`、`git diff --check`、`git diff --cached --check`
- 通过本地 HTTP 预览复核 CPU3“CPU2 内部通信与轮询”“DSM 协议与命令映射”“本机参数 FRAM 时钟与外设恢复”和“跨 CPU 业务链路”页面；桌面布局、文字显示、SVG 溢出、重复 ID、导航锚点及控制台均未发现异常。故障码 Excel 15 个工作表公式错误扫描为 0 并完成分段渲染检查；LNG 菜单 Word 已完成 29 页逐页渲染检查。

未验证风险：
- 尚未完成真实 DSM `E/e` 低电压响应、首次烧写冷启动、蓝牙从机持续透传、CPU2 延迟上线、持续外部流量和 UART 干扰实测；CH9141 有界退出需台架按 `10/100/400 ms` 上报周期确认实际时延和恢复状态。
- 尚未使用真实 PLC 验证 DSM/SI/Wartsila 对 `0x06` 的重试与回读策略，也未通过真实串口注入 CRC 正确的 DSM 截短/超长帧；DSM FC16 和 Wartsila 多字段写会回滚 CPU3 本地影子并门禁未确认读，但 CPU2 端仍可能在失败前部分生效，最终值需等强制补读后确认。
- CPU2 单次同步请求最长可阻塞约 1000 ms，外部请求峰值延迟仍需实机评估。
- 主循环空闲延时由约 100 ms 缩短为 1 ms 后，显示任务和 SI 自动调度检查频率提高，需在实机观察 CPU 占用、OLED 刷新和 RTC 访问负载。
- Modbus RTU 独立 t1.5/t3.5 帧间隔处理未在本版本实现，后续应按专项方案整改和验证。
- `BYPRODUCTS` 只清理当前配置的固定名和当前版本固件，构建目录中的历史版本 HEX 不会自动删除；发布仍应按版本名、链接成功状态、生成时间和哈希选择产物。
- Wartsila 点表 100/200 点及完成态等既有兼容问题不在本轮范围。
- 暂存区尚未按最终清单重建；正式提交前需重新运行 `py tools\check_version_bumped.py` 和暂存区差异检查。

## 2026-07-11 - 新增 LTD 共享 Modbus、CPU2 串口命令严检和流程影响门禁（CPU2 V1.22.0.0 / CPU3 V1.21.0.0）

版本：
- CPU2: V1.21.6.0 -> V1.22.0.0（MINOR）。
- CPU3: V1.20.0.0 -> V1.21.0.0（MINOR）。

协议版本/兼容性：
- `DEVICE_PROTOCOL_VERSION` 保持 14；CPU2/CPU3 共享命令、保持寄存器、输入寄存器、字段长度、32 位线序和字段语义不变。CPU3 的外部 `COM_PROTO_LTD` 从不回包占位改为响应既有共享协议，不属于板间共享契约变更。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters`、`struct_size`、FRAM A/B 地址、magic 和 CRC 范围不变；从 V1.21.6.0 升级不会因本次改动恢复出厂参数。
- CPU3 本机参数存储版本保持 `0x0006`，COM 口配置和本机 FRAM 布局不变；原已配置为 LTD 的端口升级后会开始响应 FC03/FC04/FC10，外部主机需要按共享寄存器 32 位高字在前的标准 Modbus 口径访问。

本次修改：
- CPU3 新增独立 LTD Modbus 从站，接入 COM1/COM2/COM3 协议分发表；支持 FC03 读保持寄存器、FC04 读输入寄存器和 FC10 写多个保持寄存器，地址不匹配或 CRC 错误静默，非法功能/地址/值分别返回 `0x01/0x02/0x03`。
- LTD FC03/FC04 只读取 CPU2 已确认快照；状态、参数、协议快照未建立、协议不匹配或 CPU2 通信故障时返回 `0x06`，不把默认值或旧缓存伪装为实时数据。
- LTD FC10 强制偶数起始地址、偶数寄存器数量、标准 `byteCount=qty*2` 和实际帧长一致；CPU3 将高字在前的寄存器对还原为本机 `uint32_t` 后转发 CPU2，只有合法 ACK 才回写成功。
- 参数 FC10 成功后关闭 CPU3 参数快照并主动全量补读，补读完成前外部读取返回设备忙；命令字段单独写入成功后只更新命令镜像，避免把请求影子当作 CPU2 已确认参数。
- CPU2 USART1 IDLE 中断不再拼接和执行文本命令，只把 DMA 字节交给定长收帧器；支持 CRLF/LF，正文上限 63 字节，超长帧整行丢弃并在主循环报告，不在 ISR 打印或阻塞。
- CPU2 主循环先复制完整串口帧、取消待执行自动恢复，再开放下一帧接收；正式业务命令仍只写入 `g_deviceParams.command`，由统一命令调度执行。
- 新增严格串口命令解析器，正式单字母、固定测试命令和数值命令都要求完整匹配；拒绝 `Qxxxx`、`Ixxxx`、`T1abc`、`A+10abc` 等前缀误执行，并校验距离、速度、倍率、AO 电流和蓝牙名称范围。
- 新增 `STOP/HELP/VER?/STAT?/ERR?`；STOP 映射统一取消测量命令，四个查询入口只读取版本、共享协议、参数版本、状态和错误信息。
- 旧串口测试动作继续由 `Test_ProcessSerialCommand()` 执行，但进入旧分发前必须由新解析器判定为合法测试命令；`measure.c` 仅保留兼容包装并转交新模块。
- 新增 10 组 67 个按钮的 CPU2 串口助手配置生成器、导入配置、维护说明和主机侧 C 回归测试，覆盖合法命令、尾随垃圾、越界参数、CRLF/LF、超长丢弃和恢复。
- 新增 LTD 协议契约脚本，校验分发表连接、CPU2 快照/ACK/补读闭环、共享协议版本 14 和 11 组 golden frame。
- 新增流程影响反向索引与检查脚本，根据暂存固件源码定位受影响流程页、业务链路和验证资料；版本门禁自动调用暂存区影响分析。
- 流程 HTML 契约增加表格可聚焦横向滚动区、caption/提示 ARIA 关联、SVG 视觉层隐藏和统一键盘滚动脚本；CPU3 流程生成器直接输出同一 SVG/viewport/脚本位置契约，导航生成器与测试同步更新并保持幂等生成，避免重新生成后回退旧属性。
- 同步 CPU2 串口协议卷、LTD 共享 Modbus 协议卷、流程证据清单、跨 CPU 流程页、索引和本版本改动与测试方案。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，生成 `LTD_MAIN_CPU2_V1.22.0.0.hex`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：通过，生成 `LTD_DISPLAY_CPU3_V1.21.0.0.hex`。
- `py -X utf8 tools\check_ltd_modbus_contract.py`：通过。
- `gcc -std=c11 -Wall -Wextra -Werror -I LTD_MAIN_CPU2\Application\Inc tools\test_serial_command_parser.c LTD_MAIN_CPU2\Application\Src\serial_command_parser.c -lm -o build\test_serial_command_parser.exe` 及生成程序：通过。
- `py -X utf8 tools\generate_cpu2_serial_command_panel.py --check`：通过，10 组 67 个按钮一致。
- `py -X utf8 -m unittest tools.test_check_docs tools.test_check_flow_docs tools.test_check_flow_impact tools.test_check_version_bumped tools.test_update_flow_navigation tools.test_generate_cpu3_flow_docs`：57 项通过。
- `py -X utf8 tools\check_docs.py`、`py -X utf8 tools\update_flow_navigation.py --check`、`py -X utf8 tools\check_flow_docs.py`：通过；流程契约覆盖 32 页、178 张 SVG、47 张表格。
- `py -X utf8 tools\check_flow_impact.py --staged --strict`：通过；命中 6 个核心流程、8 条业务链路，待复核流程 0 个。
- `py tools\check_version_bumped.py`、`git diff --cached --check`：通过。

未验证风险：
- 尚未使用真实 PLC/RS485 验证 LTD 长帧、CPU2 掉线、协议不匹配、ACK 丢失、参数补读窗口和主站重试策略。
- 尚未在真实 USART1 DMA/IDLE 中断环境验证连续粘贴、多行高速输入、63/64 字节边界和长时间测试中的 STOP 响应时延；当前接收器为单帧槽，上一帧未被主循环取走时后续字节会被丢弃。
- 串口查询命令仅证明解析和打印路径可构建，未在实机核对串口工具编码、换行设置和现场状态值。

## 2026-07-12 - 发布故障码协议15、外部协议切换与诊断治理

版本：
- CPU2：`V1.22.0.0 -> V1.23.0.0`（minor）。
- CPU3：`V1.21.0.0 -> V1.22.0.0`（minor）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION` 由14升级为15；共享故障码按9个责任域重新编号，协议15与14及更早版本的故障码数值解释不兼容，CPU2和CPU3必须成对升级。
- CPU2 `DEVICE_PARAM_VERSION` 保持3，`DeviceParameters` 结构大小和 `struct_size` 不变；本次升级不会因参数布局变化触发恢复出厂参数。

本次修改：
- CPU2新增TMC5130故障快照、SPI阶段与配置丢失诊断，补齐运行期复位、欠压和关键寄存器异常的故障传播与恢复边界。
- CPU2新增AD5421总线访问、寄存器回读和主动故障状态快照；AO将中断/PendSV中的诊断延后到主循环打印，并对持续相同故障去重。
- CPU2细化传感器、无线、参数、测量和串口传输错误归因，统一错误码目录、名称、原因和最终故障出口。
- CPU3新增功能码 `0x46` 的统一外部协议切换帧；ACK在旧串口参数下发送完成后，由主循环保存FRAM并只重配目标COM口，保存增加写后读回校验和失败回滚。
- CPU3同步协议15故障码显示与CPU2通信超时归因；同协议切换请求只返回ACK，不触发无意义重配。
- 流程证据、验证工作包、运行台账、验证批次、治理事项、交付基线和跨版本知识治理工具纳入文档总门禁；修复Windows管道输出编码和缺少tzdata时的上海时区回退。
- 同步协议记录、故障码资料、流程HTML/SVG及流程证据固件基线，正式文档继续统一维护在 `docs/`。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，原构建验证的代码范围不变；修正版本后产物名应为 `LTD_MAIN_CPU2_V1.23.0.0.hex`，`text=284460 data=2336 bss=27768`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：通过，原构建验证的代码范围不变；修正版本后产物名应为 `LTD_DISPLAY_CPU3_V1.22.0.0.hex`，`text=164072 data=37380 bss=60420`。
- 故障码、参数测量、传感器、TMC5130、AD5421和CPU2通信超时契约脚本通过；CPU3字库454字符一致、缺字0。
- Python全量257项单测、文档总门禁和 `git diff --check` 通过；暂存区版本门禁和提交标题门禁以本次提交记录为准。

未验证风险：
- 尚未在实机验证协议切换ACK后的串口重配时序、FRAM写失败回滚、同协议重复请求及三路COM并发切换。
- 尚未完成TMC5130掉电/运行期复位、AD5421主动报警/断环恢复和协议14/15交叉烧写的台架故障注入。

## 2026-07-13 - 新增 CPU2 设备侧安全传感器协议并保留旧协议回退

版本：
- CPU2：`V1.23.0.0 -> V1.24.0.0`（MINOR）。
- CPU3：保持 `V1.22.0.0`，本次不修改 CPU3 固件行为。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION` 保持 15。`SAFE_SENSOR=14` 只在 CPU2 驱动选择中解释；CPU3 对既有 u32 `sensorType` 仅原样缓存和打印，不改变共享字段地址、宽度、布局、命令、状态或双方消费语义。
- CPU2 `DEVICE_PARAM_VERSION` 保持 3，`DeviceParameters`、`struct_size`、FRAM A/B 地址、元信息和 CRC 范围不变；从 V1.23.0.0 升级不会恢复出厂参数。
- 传感器安全通信协议保持 V1；新增的 `READ_PARAM_PAGE(0x63)` 由 `CAP_PARAM_RW` 单独声明。没有该能力的原 V1 传感器仍可完成 HELLO 和既有测量，完整参数读取返回不支持且不会退化为 115 次单参数读取。
- 旧 DSM、LTD/V2 探测和测量路径保留；安全协议 HELLO 失败会释放 UART6，再继续 LTD/V2 和 DSM 探测。降级到不识别 `SAFE_SENSOR=14` 的旧 CPU2 固件前，应先恢复旧固件支持的传感器类型。

本次修改：
- 新增 `Services/Sensor/SafeProtocol/` 独立子目录，实现 CRC32C、固定偏移帧编解码、HELLO/会话/事务连续性、55 个控制帧契约、7 个固定 44 字节快报契约和严格错误分类。
- 新增 UART6 DMA 传输、SOF/长度重同步、超时/取消、`PE/NE/FE/ORE/DMA/BREAK` 诊断和所有异常出口 busy 清理；CH9141 进入 AT 模式前主动终止安全协议传输，避免共享 UART6 状态残留。
- 新增 FRAM 启动计数 A/B 双副本、写后回读、硬件 RNG/后备 nonce、配置摘要复核、安全快照双缓冲、过期撤销和只读事务单次受控恢复；副作用命令的模糊结果禁止跨会话自动重放。
- 新增旧业务适配层，将安全协议结果映射到现有错误码、恢复日志、密度/液位/水位电容/姿态接口；传感器自动识别顺序固定为蓝牙链路、安全协议、LTD/V2、DSM，并把三种探测结果都纳入最终错误归因。
- 液位测量、回零能力判断、读取部件参数和串口通信诊断新增 `SAFE_SENSOR` 分支；不支持的水位电容或姿态能力不会误阻断旧流程。
- 新增完整 LTD 参数表读取入口：上层一次调用由 client 自动读取 39 页共 115 项，并校验参数表版本、总数、游标、末页、页内/跨页顺序、逐项 CRC，以及固定 `0x1000..0x1072` 地址、索引、类型、权限、倍率和单位；容量不足时不发布部分表并返回所需容量。
- CMake 和 Eclipse 工程增加安全协议子目录；协议卷、落地方案、golden frame、残余危险失效率论证和验证记录同步到当前设备侧实现。
- 按安全代码审计要求为 21 个 `SafeProtocol` 文件补齐中文模块、函数和关键分支注释，明确同序号重试、防重放、FRAM 双副本、参数分页、周期静默预算、副作用恢复和旧协议回退约束；C/C++ 注释仅使用 `/* ... */`。

验证：
- `py -X utf8 tools\check_sensor_safe_protocol_contract.py`：通过，55 个控制向量、7 个快报向量与 CPU2 C 常量一致。
- `py -X utf8 tools\check_sensor_safe_host_tests.py`：通过，7 组主机测试和 4 项 GCC `-fanalyzer` 单元全部通过，包含 115 项、39 页完整规模读取。
- `py -X utf8 tools\check_sensor_safe_cpu2_integration.py`：通过，19 个冻结命令入口、115 项服务校验、完整参数表适配入口、`SAFE_SENSOR=14`、安全→LTD/V2→DSM 回退及设备侧分发一致。
- 注释规范复核：全部 `SafeProtocol` 函数定义均有中文用途或约束注释，未出现 `//`；严格 UTF-8 检查通过。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，生成 `LTD_MAIN_CPU2_V1.24.0.0.hex`，`text=296860 data=2336 bss=28984`。

未验证风险：
- 尚未用真实安全传感器完成 200 米、9600 bps、噪声/断线/粘连/重复/乱序、控制窗口、硬件 RNG、FRAM 掉电和 CH9141 切换故障注入。
- 传感器端 115 项参数描述表、取值回调和分页响应分发尚待按冻结协议实现并与 CPU2 对拍；39 页在 9600 bps 下的实际总时长和对实时测量调度的影响仍需台架验证。
- 目标 SIL 尚不能由协议和主机测试单独声明满足；仍需整机 SRS、FTTI、安全状态、硬件失效率、诊断覆盖率、系统级验证和独立评审证据。

## 2026-07-14 - 按二代计量仪责任域重构故障码并同步双端显示

版本：
- CPU2：`V1.24.0.0 -> V1.25.0.0`（MINOR）。
- CPU3：`V1.22.0.0 -> V1.23.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION` 由15升级为17；协议16是13、14类别互换的开发过渡基线，未单独发布。协议17按二代计量仪责任域重新分类、删减并重编号故障码，故障状态寄存器地址、宽度和32位传递方式不变，但数值解释与协议16及更早版本不兼容，CPU2和CPU3必须成对升级。
- 现场查询CPU2产生的共享故障时按CPU2程序版本选择故障码表；共享协议版本只用于CPU2与CPU3配套检查。CPU3本机 `20-7` 按CPU3程序版本确认。
- CPU2 `DEVICE_PARAM_VERSION` 保持3，`DeviceParameters` 结构大小、`struct_size`、FRAM A/B地址、元信息和CRC范围均不变；从V1.24.0.0升级不会恢复出厂参数，不需要重新下发设备参数。
- CPU3本机参数存储版本保持 `0x0006`，本机FRAM布局不变。

本次修改：
- CPU2和CPU3同步65项 `ErrorCode` 定义，其中2项为非故障状态、63项为共享故障；CPU3另保留本机通信故障 `CPU2_COMM_TIMEOUT = 0x00140007`，屏幕显示 `20-7`。
- 删除12项低价值或已被具体故障替代的当前枚举：电机设置失败、电机复位失败、滑环校验错误、滑环丢包、传感器温度异常、传感器电压异常、密度不稳定、密度范围无效、AD5421故障管脚报警、未知故障、地址读取错误和电源波动；旧编号继续保存在CPU2 V1.24.0.0历史表，不复用于新含义。
- 仅保留 `12-6 编码器上电值变化`、`15-16 实高偏差过大`、`21-5 扭力传感器饱和` 3项未启用预留码；正式表当前共64个故障项目，其中61项使用中、3项保留。
- CPU2错误日志名称、责任模块和原因映射同步新编号；CPU3故障详情和类别兜底显示同步新责任域。AO运行期判定及密度测量注释清除已删除枚举引用。
- 正式工作簿补充硬件、机构、设置、操作、运行环境和数据异常等中文故障原因，历史代码按设备代际和CPU2程序版本独立归档；内部技术工作簿、当前清单、协议记录、LTD/DSM/SI资料和CPU3状态页说明同步协议17口径。
- 本次只改变故障码数值、名称映射和显示解释，不改变测量步骤、状态机、命令分发、故障恢复条件或通信寄存器布局，因此程序流程HTML/SVG无需更新。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，生成 `LTD_MAIN_CPU2_V1.25.0.0.hex`；`text=296412 data=2336 bss=28984`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：通过，生成 `LTD_DISPLAY_CPU3_V1.23.0.0.hex`；`text=163600 data=37380 bss=60420`。
- 故障码目录、CPU3原因显示、CPU2通信超时、传感器、参数与测量、TMC5130和AD5421专项契约检查通过；CPU3字库检查缺字0个。
- 正式工作簿6张工作表完成公式错误扫描和视觉检查；文档结构、Markdown链接、版本门禁及Git空白检查以本次提交前最终结果为准。

未验证风险：
- 尚未完成FI-01～FI-08实机故障注入、协议16/17交叉烧写拦截和全部恢复路径台架验证。
- 外部主站、历史日志分析工具和售后资料若按故障码数值解析，必须按设备代际及对应CPU2程序版本选择代码表，不能直接套用协议16及更早版本编号。

## 2026-07-15 - 落地SI协议18生命周期、站2寄存器兼容与最终快照发布

版本：
- CPU2：`V1.25.0.0 -> V1.26.0.0`（MINOR）。
- CPU3：`V1.23.0.0 -> V1.24.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由17升级为18；在输入寄存器尾部新增SI Profile阶段、周期号和有效点进度三个32位运行态字段。协议17与18不兼容，双端必须成对升级并保持严格相等。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`、`struct_size`、字段顺序、FRAM A/B地址和CRC范围不变；从CPU2 V1.25.0.0升级不会因本次改动恢复出厂参数。
- CPU3本机参数存储由V6 / `0x0006`升级为V7 / `0x0007`，新增`40004~40009`六个原始兼容槽；正常V6迁移保留原SI参数及三路串口配置。V6迁移写失败可能破坏旧镜像的风险已记录为R18-01，本次未修复。

本次修改：
- CPU2新增独立SI Profile生命周期运行态。Point0有效样本写入时建立新周期并把有效点数置为1，后续点数逐步增加；预期最终点数无需在测量前给出。
- SI采点成功后先形成候选并进入回液位阶段，只有液位稳定且电机停止后才提交Complete、完成计数、来源、最终N和点阵；取消、失败或命令切换不发布本轮最终结果。
- CPU3按周期号锁存Point0时间，活动期点阵保持0；最终候选通过头部前后代际复核和24个点阵分块读取后，生成独立不可变SI快照，避免后续普通分布、国标、每米、间隔或Wärtsilä结果覆盖。
- 按站2抓包实现`30014~30016`对FC01/FC02的状态镜像，`30017~30020`保持0；实现`40004~40009`原值读写和掉电保存，不赋予未知业务含义，也不参与控制或报警。
- `40004~40023` CPU3本机参数写入采用写后读回校验、失败运行态回滚和`0x06`异常；`FC05=ON`与`40001~40003`继续依赖CPU2 ACK、协议匹配和完整快照，失败不提交影子并强制补读。
- 自动Profile按RTC绝对分钟和RAM锚点跨日运行，命令失败按5秒节流重试；正常连续运行支持61/1000分钟等非整日周期。
- 同步SI协议方案、映射、站2抓包分析、PLC联调检查表、协议18变更记录和本版本改动与测试方案；已知问题统一记录在`SI协议待确认与后续清单.md`第7节。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，生成`LTD_MAIN_CPU2_V1.26.0.0.hex`；`text=297396 data=2336 bss=33864`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：通过，生成`LTD_DISPLAY_CPU3_V1.24.0.0.hex`；`text=167144 data=37396 bss=61764`。
- `py tools\check_si_protocol_contract.py`、`py tools\check_si_modbus_frames.py --dump`、`py tools\check_wireless_rssi_contract.py`通过。
- 版本门禁、编码、注释、冲突标记和Git空白检查在精确暂存后按提交结果复核。

已知问题和未验证风险：
- R18-01（P1）：CPU3 V6→V7迁移写入或读回校验失败后没有恢复并复核原V6镜像，部分写可能导致下次启动恢复默认值。
- R18-02（P1）：CPU2快速重启时周期号归零可能被CPU3误判为Point0；若中间的0未被观察到，新周期号复用还可能漏掉新周期。
- R18-03（P1）：CPU3重启会按当天起始时间重建RAM调度锚点，61/1000分钟等非整日周期可能丢失原跨日相位。
- R18-04（P2）：200点最终候选同步拉取会在外部COM请求处理前执行，可能阻塞PLC响应数百毫秒，内部单帧超时还可能额外等待约1秒。
- R18-05（P2）：完成边沿的分布头和生命周期尾分两帧刷新，可能短暂产生新阶段与旧完成计数/来源/点数的非原子组合。
- LEGACY-01：既有探底频次存在off-by-one，N=2会每轮探底，N=5实际为第1、5、9轮；默认N=1不受影响。
- LEGACY-02：协议17/18混搭能够fail-closed，但当前上电读取顺序通常先报告`CPU2_COMM_TIMEOUT`，未提供明确版本不匹配诊断。
- 本次按用户决定只记录以上问题，不修改固件；尚未完成真实站2 RS485、PLC响应时延、V6 FRAM故障注入、CPU2快速重启、CPU3跨日重启和完成边沿高频轮询测试。

## 2026-07-16 - 发布协议21故障码、AO配置、固定点快照与外部协议一致性（CPU2 V1.27.0.0 / CPU3 V1.25.0.0）

版本：
- CPU2：`V1.26.0.0 -> V1.27.0.0`（MINOR）。
- CPU3：`V1.24.0.0 -> V1.25.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由18正式升级到21。协议19完成共享故障码拆分，协议20完成AO配置和运行态扩展，两者均未形成独立发布组合，统一随协议21发布；双端必须成对升级，协议18/21、19/21或20/21混搭均由严格版本门禁阻止运行。
- 协议19改变故障码数值和解释，故障状态寄存器地址及32位传递方式不变；历史故障查询必须按设备代际和CPU2程序版本选择冻结表。
- 协议20把既有连续13个AO参数槽原位解释为`AoOutputConfig`，追加AO仿真开关和6个运行态字段；CPU2按旧协议标记迁移合法的旧使能、液位量程、初始/故障/调试电流，其余字段补默认值。CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、FRAM A/B地址、magic、元信息和CRC范围不变，从V1.26.0.0升级不会因本次改动清除其它现场参数。
- 协议21在输入寄存器尾部追加`0x0A44`单点测量完成代际和`0x0A46`固定点监测样本代际，`REG_ENG=0x0A48`。CPU3本机参数存储保持V7 / `0x0007`，不改变既有SI兼容槽和三路串口配置布局。

本次修改：
- 协议19故障码治理随本次正式发布：CPU2/CPU3同步当前共享故障定义、具体产生路径、名称、原因、自动恢复分类和CPU3详情显示；无线扫描、名称冲突、传感器安全协议、参数存储、测量计划、电机到位/停止以及AD5421/TMC5130等通用码拆为可定位原因，历史编号永久冻结。
- 自动恢复只在被重跑的业务命令真实返回`NO_ERROR`后记录“重试成功”并清理上下文；部件健康检查只证明具备重跑条件，`STATE_SWITCH`只取消恢复，不再被误记为恢复成功。
- CPU2 AO重构为工作模式、电流模式、输出源、固定电流、0%/100%对应值、阻尼、故障模式/电流/等级、上电电流和仿真电流13字段配置；支持储罐液位、空高和水位过程源，NE/US/普通/固定电流、反向量程、阻尼、故障回退和仿真旁路。
- AO运行态改为短临界区一致发布，HART只在HART从站模式应答并读取最后一次成功下发电流及真实量程百分数；AD5421访问增加上下文限流、初始化时序、控制寄存器写后校验、故障寄存器读取和诊断恢复，AO输入样本在测量失败或切换时显式失效。
- CPU3把AO配置整理为五组菜单，补齐选项文案、值域、保护确认、成对写入、CPU2 ACK后提交、失败回滚和强制补读；状态页显示AO过程值、百分比和最近一次成功下发电流，并按有效性显示仿真状态；该电流不是DAC回读或物理环路实测值。
- CPU2固定点测量和固定点监测分别把每组六个结果字段在短临界区整体发布，字段完成后再递增各自代际；通用`Read_Density()`不再提前改写公开固定点结果。
- CPU3固定点同步改为“代际前读—24个结果寄存器私有读取—代际后读”，只有前后两个代际完全一致才在短临界区提交公开寄存器缓存和`g_measurement`；任一读取失败或代际变化都丢弃候选并从前读重来。
- CPU2内部通信首次请求失败即关闭状态、参数、协议和固定点快照新鲜度，保留连续10次置`CPU2_COMM_TIMEOUT`的报警阈值；恢复时从状态、协议、参数和固定点快照完整重同步，外部协议在此期间返回Busy而不是旧数据。
- DSM和SI按每个读取范围区分CPU3本地静态字段与CPU2派生运行态；混合范围只要包含CPU2字段且快照无效，整帧返回Busy。DSM特殊响应映射到具体传感器故障，不再统一折叠为设备错误。
- Wärtsilä FC03按运行态、参数和本地静态字段分类门禁，可靠点表固定为100点，长结果受控截断、短结果先清完整点表再填有效点；版本签名固定要求20个寄存器，FC10严格校验`byteCount=2*quantity`和PDU长度，命令只接受0、3、4、5。
- LTD外部Modbus新增CPU3本机`0x0200`保持寄存器段：FC03读取当前有效本机参数，FC10每次只写一个完整32位且可写的非UART字段，执行值域校验、FRAM写后读回和失败异常；CPU2参数段仍使用CPU2快照与ACK闭环。
- CPU3 UART DMA+IDLE接收按同一SR快照分类，`PE/FE/NE/ORE`优先于IDLE；错误组合事件交HAL统一中止和恢复，不再把带错误帧长度发布给协议解析器。
- 相邻测量与存储边界同步细化：罐高/水位标定、密度计划、位置到位、参数缺失、缓冲容量和调用条件返回具体错误码；点动越界容差由0.1 mm调整为1.0 mm；电机持久位置恢复区分未初始化与通信失败，并继续校验FRAM A/B快照；无线名称匹配区分未发现、重复和非法名称。
- 同步README、版本总览、参数存储清单、共享Modbus/DSM/SI/AO/故障码协议资料、CPU3菜单和状态页资料、工作簿、Word/OPML维护件及本版本改动与测试方案。PDF按仓库发布规则保留在工作区，不纳入本次Git提交。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，生成`LTD_MAIN_CPU2_V1.27.0.0.hex`，`text=306452 data=2328 bss=33992`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：通过，生成`LTD_DISPLAY_CPU3_V1.25.0.0.hex`，`text=176840 data=37940 bss=60852`。
- 故障码、CPU2通信超时/详情、传感器/参数/测量、电机、AD5421、无线和AO契约检查：通过。
- 协议21固定点代际、DSM/SI/Wärtsilä/LTD和CPU3外部读取新鲜度契约检查：通过；外部新鲜度主机组合检查通过66872项。
- 传感器安全协议、CPU2集成、7组主机测试和4个fanalyzer单元：通过；CPU3字库、显示ISR边界、菜单短名宽度检查：通过。
- 本机程序流程导航已同步18个核心流程、8条业务链路，严格暂存影响检查待判断0项、待复核0项；文档结构、链接、暂存一致性、版本和Git空白门禁通过。

未验证风险：
- 尚未执行协议18/21、19/21和20/21交叉烧写，以及协议19新增拆分故障的完整实机注入、停机、显示、日志和恢复验证。
- 尚未以真实FRAM旧镜像验证协议18/19到协议21的AO迁移、A/B掉电中断与回滚，也未在真实4~20 mA负载、HART主站和AD5421断环/过温/低压环境验证输出及恢复。
- 尚未用真实PLC/RS485验证固定点发布期间高频轮询、CPU2首次通信失败后的Busy窗口、DSM/SI/Wärtsilä新鲜度门禁、Wärtsilä100点截断/短轮次清尾、LTD本机参数段和UART错误+IDLE组合事件。
- 安全传感器相关静态契约只能证明接口和错误映射一致，不能单独作为SIL满足性证据；仍需整机SRS、FTTI、故障注入、硬件失效率和独立评审。

## 2026-07-16 - 发布协议23故障码重排、AO语义收敛、分布同步与现场协议兼容（CPU2 V1.28.0.0 / CPU3 V1.26.0.0）

版本：
- CPU2：`V1.27.0.0 -> V1.28.0.0`（MINOR）。
- CPU3：`V1.25.0.0 -> V1.26.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由21升级为23；协议22完成32项共享故障码重排但未单独发布，协议23在相同13个AO参数槽上收敛故障动作、非跟随电流和运行态来源语义。协议21/22/23不能混搭，双端必须成对升级。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`、`struct_size`、FRAM A/B地址、magic和CRC范围不变。协议20～22的旧AO值由CPU2按原槽位迁移和归一化，不因本次升级恢复出厂或清除其它现场参数。
- CPU3本机参数存储保持V7 / `0x0007`，SI兼容槽、三路串口参数和FRAM布局不变。

本次修改：
- 现行故障码按一代同义子码优先、二代新增项使用最低可用子码的规则重排32项；CPU2 V1.27.0.0 / CPU3 V1.25.0.0 / 协议21完整归档为历史页。正式表现行页共131项，包含129项共享故障和CPU3本机`20-7`、`20-13`。
- AO故障配置由五种模式收敛为“输出故障电流/保持上次有效过程电流”两种动作，原错误等级槽隐藏预留，原上电电流槽解释为3.40～22.60mA非跟随电流。只有匹配的液位、空高或水位跟随状态使用过程电流，待机、搜索、其它测量和过程无效状态使用非跟随电流。
- AO最近有效过程电流只在正常过程目标成功写入AD5421后更新；保持动作无历史值时回退非跟随电流。CPU3菜单保持基本、量程、故障、运行和模拟五组，运行页显示输出状态、过程输入、比例和最近成功下发电流。
- CPU3上电或通信恢复后先独立探测CPU2协议版本；协议快照未知、已确认不匹配和真实通信超时分开处理。协议不匹配时只保留版本心跳，禁止访问扩展寄存器和业务命令，不再永久停在“通讯尝试中”或误报`20-7`。
- 分布结果改为按实际`N`点、每帧最多8点、帧间隔至少50ms的非阻塞拉取；单块按100/300/1000ms重试，整轮最多3轮且总上限15秒。前后头部一致后才原子发布；最终失败进入CPU3本机`20-13`，不会无限保持测量中。
- CPU2重启、通信会话变化或完成锁存清零时，CPU3作废旧分布缓存并取消旧同步；SI本地投影按会话代际清除旧点阵、完成态和时间，避免计数复用或跨会话拼接。
- Wärtsilä FC10同时兼容现场`byteCount=quantity`和标准`byteCount=2*quantity`，两种格式仍严格要求实际数据区长度为`2*quantity`；DSM、LTD和内部Modbus继续保持标准长度校验。
- CH9141K上电首次AT入口收到非空、非`OK/ERR`残包并超时时，恢复透明模式和UART6后有界重试一次；明确`ERR`、状态切换和运行期普通AT不重试。
- CPU3字库按确认点阵修正“尼、组、跑、锁、隔、策、亮”，并增加AO菜单所需“站、普”；同步故障码工作簿、协议卷、菜单树、问题闭环和版本资料。PDF、本机`tools/`、程序流程导航、`docs-site/`和`outputs/`不进入Git提交。

验证：
- 双端clean-first构建、协议/故障/AO/分布/SI/Wärtsilä/字库/编码/文档/Excel和Git门禁结果记录在本版本《改动与测试方案》中。
- 真实4～20mA负载、AD5421断环/HART、协议21/22/23交叉烧写、CPU2快速重启、分布同步中立即开始新测量、Wärtsilä现场主机和完整故障注入仍需台架验证。

## 2026-07-17 - 协议24 AO过程保持与CPU2通讯健康

- CPU2: V1.28.0.0 -> V1.29.0.0 (minor)
- CPU3: V1.26.0.0 -> V1.27.0.0 (minor)
- 说明：发布协议24 AO过程保持与CPU2通讯健康。

协议和兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`同步从23升级到24；协议24重新解释AO输出源1、初始电流和运行态来源，双端必须成对升级。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、13个AO槽位、FRAM A/B地址、元信息和CRC范围不变，不因本次升级恢复出厂参数。
- CPU3本机参数版本保持V7 / `0x0007`；通信健康计数只保存在RAM，上电清零，不进入共享协议或FRAM。
- 旧FRAM中的`output_source=1`会由“空高”直接改按“传感器位置”解释；升级前后必须人工复核输出源、0%/100%量程、初始电流和故障动作。

CPU2 AO：
- 输出源固定为储罐液位、传感器位置和水位；位置源直接使用传感器位置，编码轮/电机记步模式分别按各自就绪条件门控，0.0mm允许有效。
- 原非跟随电流改为初始电流；首次有效过程目标成功写入AD5421前使用初始状态，成功后建立电流、同轮输入值和比例缓存。
- 命令切换、重新搜索、自动恢复等待新样本或过程样本暂时无效时保持上次成功过程目标，并在保持期间冻结阻尼时间。
- 禁用、输出源/电流模式/量程变化清除缓存；传感器位置源下罐高或记步方式变化也清除缓存。真实设备故障优先于输出仿真，AD5421写失败不更新缓存。

CPU3通信和界面：
- 公开快照改为连续第3次请求失败时失效并完整重同步；第10次连续失败仍进入CPU2通信超时故障。
- 新增成功、超时、CRC、地址、功能码、长度、UART错误、事务失败、TX DMA启动失败、当前/最大连续失败和最近失败原因等RAM统计。
- UART错误中断向通信层传递实际`ErrorCode`；维护设置新增三页“CPU2通讯”页面，显示总次数、失败率、错误分类和连续失败。
- AO菜单和运行页同步传感器位置源、初始/过程/保持状态及最近成功下发快照。

验证：
- AO协议24、CPU3通信超时、分布快照、AD5421、SI协议、菜单宽度、字库、文档和版本门禁均纳入本版静态验证。
- 双端clean-first构建结果和最终ELF尺寸记录在本版本《改动与测试方案》中。
- 真实4～20mA负载、0.0mm位置源、命令切换/重新搜索保持、AD5421写失败、通信错误注入和三页维护界面仍需台架验证。

## 2026-07-18 - CPU3异步通信诊断、串口时序修复与双端点阵同步恢复

版本：
- CPU2：`V1.29.0.0 -> V1.29.1.0`（PATCH）。
- CPU3：`V1.27.0.0 -> V1.28.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持24；本次不改变共享寄存器地址、长度、功能码、字段类型、单位、倍率、命令码或异常响应，V1.29.1.0与V1.28.0.0仍按协议24严格配套运行。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构、`struct_size`、FRAM A/B地址、元信息和CRC范围均不变；本次升级不会恢复出厂参数，也不需要重新下发现场参数。
- CPU3本机参数存储保持V7 / `0x0007`；新增通信日志队列、失败分类和点阵同步上下文只驻留RAM，不改变FRAM布局。

本次修改：
- CPU2六种分布测量统一通过原子发布入口写入点阵数据、来源和完成锁存，完成计数最后递增；Wartsila与综合测量开始时清旧锁存，综合测量按实际模式发布来源，避免CPU3读取到跨代组合。
- CPU3点阵同步改为分块推进状态机：单块按100/300/1000ms重试，耗尽后等待500ms并从前头部开启下一整轮，最多三轮；任一时刻20秒无进展直接进入完整失败。会话或代际变化时全量重启，重试期间不穿插普通轮询；完整失败代际记录为20-13并在2秒后自动重试，同代成功后自动清除，CPU2真实故障保持优先。
- CPU3在完整同代点阵提交前，把Wartsila、普通、国标、综合、每米和区间密度的CPU2完成态临时映射回对应测量中态；候选完整提交后才恢复原始完成态，避免外部看到“完成态+旧/部分点阵”。同一同步入口同时覆盖SI来源快照。
- CPU3新增4KB环形日志队列与USART1 TX DMA异步输出，统一CPU2、COM1/2/3、DSM、Wartsila、LTD和SI通信日志口径；队列满时只丢日志并累计，不阻塞业务，HardFault紧急轮询输出保留。
- 删除COM1/2/3发送完成中断中的18000次忙等待；UART5及其DMA保持抢占优先级0，COM1/2/3及其DMA调整为1，调试USART1及其DMA保持5，并同步CubeMX配置。若实测收发器需要DE尾保持，只允许按注释使用TIM5微秒级非阻塞方案。
- 新增CPU3通信健康、失败帧和UART/DMA分类统计；发送统计区分提交、排队和真实完成，ISR只记录事件，主循环统一格式化输出。
- 纳入4-20mA电流输出逻辑PDF、全工程问题总账HTML阅读版及两个问题索引；Markdown继续作为问题总账权威维护源。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过，生成`LTD_MAIN_CPU2_V1.29.1.0.hex`，`text=307516 data=2328 bss=34064`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：通过，生成`LTD_DISPLAY_CPU3_V1.28.0.0.hex`，`text=189688 data=37932 bss=84964`。
- CPU2分布点阵发布、综合测量来源、CPU3分布快照、CPU2通信超时、外部新鲜度66877项、DSM、Wartsila、SI、LTD及CPU3显示ISR边界契约通过。
- Markdown链接检查通过286个文件；`git diff --check`通过。全库文档结构仍仅报告既有`docs/05_测试记录/协议台架验收`缺少`README.md`，与本次提交无关。

未验证风险：
- 尚未完成点阵同步期间单帧超时、连续3次超时、会话/代际变化、20-13后同代恢复及CPU2真实故障优先级的实机故障注入。
- 尚未用逻辑分析仪验证UART5接收切换、COM1/2/3 DE释放时序和三路外部COM并发压力；COM12短时监测不能替代1小时、24小时或100万帧测试。
- 新增日志队列仍需在真实CPU3上验证USART1输出、队列溢出提示、HardFault紧急输出以及DEBUG高流量下的丢弃策略。

## 2026-07-20 - 发布协议25水位滞后参数、菜单对齐与多协议资料收口

版本：
- CPU2：`V1.29.1.0 -> V1.30.0.0`（MINOR）。
- CPU3：`V1.28.0.0 -> V1.29.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由24升级为25；原`reserved3` 32位槽位正式定义为`water_level_hysteresis_time_s`，单位s、范围0～3600、默认0，双端必须成对升级，协议24及更早程序不得与协议25混用。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、`struct_size`、FRAM A/B地址、magic、元信息和CRC范围不变；协议24及更早存储升级时强制把该槽归零，不恢复出厂、不清其它现场参数。
- CPU3本机参数版本保持V7 / `0x0007`，新增共享参数菜单和多入口快捷项不改变CPU3本机FRAM布局。
- Wärtsilä FC10改为仅接受莆田现场`byteCount=quantity`且实际数据区为`2*quantity`的格式；标准Modbus `byteCount=2*quantity`返回`0x03`且不得写寄存器或下发CPU2。FC03响应仍使用标准`byteCount=2*quantity`。

固件与菜单：
- CPU2、CPU3同步复用保留槽并更新寄存器/操作号别名、参数解析、写入、启动迁移、恢复默认和参数打印；水位滞后时间当前仅支持菜单、持久化和双端同步，不参与水位流程控制。
- CPU2出厂默认值按已确认文档调整：液位固定频率5500改为5200，水位滞后电容阈值30.000pF改为15.000pF，SI Profile首点1000改为5000；对应启动归一化和恢复出厂保持一致。
- CPU3长按确认时间由约1.5s改为约3s；罐底、Wärtsilä和SI Profile菜单使用显式排序，并复用罐底编码器修正入口；水位菜单改为11项显式顺序，依次为水位方式、罐高、盲区、跳变/跟随/寻找/滞后电容阈值、稳定距离、修正值、最大下行距离、零点电容和滞后时间，确保协议25新增的滞后时间位于末尾。
- CPU3正式菜单树、OPML和《LNG计量仪屏幕菜单》Word同步当前实现，本轮Word改写内容保留红字供人工审核，参考副本单独归档。

协议、审查与资料：
- 同步DSM CPU1传统UART协议正文、15个版本/快照矩阵、20组golden frames和哈希绑定清单；适配复查按V5.4工作区重新核对，补充`CX`主动掉电能力、`Ch`冷唤醒时序以及正常首轮超时被打印为错误日志的`DSM-CUBE-09`，原`DSM-CUBE-01～08`仍为尚未整改问题。
- 新增参数/协议无效值审计和二代故障码逐项核查，明确FC10值域、零值冲突、温度哨兵、HART、Wärtsilä状态、12-11不可达、13-2命名和13-26密度上限等后续事项；这些报告不表示对应固件问题已经修复。
- 收口CPU3点阵同步发布说明、Wärtsilä抓包/兼容文档、全工程问题总账、协议台架验收README、工具catalog及命令闭环矩阵；静态状态仍保留`Unverifiable/BenchPending`，不替代真实台架。
- 新增双安全继电器冗余输出电路HTML分析，区分已确认接线、逻辑推导和待确认安全边界。
- 复核并统一现行文档基线：协议25及CPU2 V1.30.0.0 / CPU3 V1.29.0.0组合、129项共享故障加2项CPU3本机故障、SI沿用协议18契约、AO沿用协议24语义，以及安全传感器资料的SIL证据边界；历史快照继续保留原版本口径。

验证：
- 双端clean-first构建、协议25/参数迁移、菜单宽度与字库、Wärtsilä格式、DSM同步制品、文档结构/链接、编码和Git门禁结果记录在本版本《改动与测试方案》中。
- 菜单顺序调整后的CPU3 clean-first构建通过，生成`LTD_DISPLAY_CPU3_V1.29.0.0.hex`，ELF为`text=190144`、`data=37932`、`bss=86780`。
- 《LNG计量仪屏幕菜单》Word已检查全部28页；DSM适配复查PDF与Markdown同步并检查全部11页，未见明显截断、重叠或乱码。
- 真实CPU2/CPU3交叉烧写、旧FRAM升级、Wärtsilä现场主机、DSM传感器、菜单按键、Word/PDF人工审阅、继电器硬件和完整故障注入仍需台架或人工验证。

## 2026-07-21 - UART连续流量中断活锁与双端通信恢复整改

版本：
- CPU2：`V1.30.0.0 -> V1.30.1.0`（PATCH）。
- CPU3：`V1.29.0.0 -> V1.29.1.0`（PATCH）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持25；共享寄存器、功能码、命令、参数含义、合法帧响应内容和外部DSM/Wärtsilä/SI/LTD协议格式均不变，两端仍按协议25配套运行。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`、`struct_size`、FRAM A/B地址、magic、元信息和CRC范围不变；升级不恢复出厂、不清除现场参数。
- CPU3本机参数版本保持V7 / `0x0007`，三路COM配置和FRAM布局不变；新增恢复状态、计数和看门狗健康代际只驻留RAM。

本次修改：
- CPU3 COM1/COM2/COM3及板间UART5补齐DMA收满和UART错误接管：中断先关闭IDLEIE、清除错误触发并丢弃不完整候选帧，只投递恢复事件；RX DMA启动成功后才重新开启IDLEIE，避免`DMA已停+IDLE/ORE未清`导致UART IRQ无限重入。
- CPU3按端口执行100 ms退避恢复，并把启动参数重载、运行期重配置和协议切换中的UART重初始化失败纳入失败端口独立的完整重初始化队列；COM摘要增加收满、恢复和失败计数，异常事件按端口限频输出。
- CPU3 TIM4改为仅在启动、主循环关键阶段或CPU2事务边界的前台健康代际推进后刷新IWDG，响应等待循环内部不重复续票，使持续UART中断活锁不再被无条件喂狗永久掩盖。
- CPU2 USART1/USART2/UART4的512字节接收长度由8位改为16位；四路普通DMA+IDLE接收统一处理DMA收满、UART错误和启动失败，TIM4只检查状态并触发最低优先级PendSV恢复，不依赖不可更改的阻塞式主循环。
- CPU2 HART和板间UART5 Modbus从硬件UART IRQ移到PendSV解析；进入原业务解析前增加HART声明长度及Modbus FC03/04、FC10帧形校验，合法帧的分发、寄存器访问和响应内容保持不变，畸形或超长帧直接丢弃并恢复接收。
- CPU2中断上下文不再执行阻塞式`printf`，线程态调试串口单字符发送等待上限改为10 ms；TIM4仅在没有其它活动ISR时刷新IWDG，降低中断活锁被看门狗掩盖的风险。

验证：
- `py -X utf8 tools\check_cpu3_external_freshness_contract.py`：通过，`66877 checks`。
- `py -X utf8 tools\check_cpu2_uart_resilience_contract.py`：通过，`42 checks`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：61步通过，生成`LTD_DISPLAY_CPU3_V1.29.1.0.hex`；ELF为`text=191408`、`data=37932`、`bss=86876`。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：92步通过，生成`LTD_MAIN_CPU2_V1.30.1.0.hex`；ELF为`text=308492`、`data=2328`、`bss=34088`。
- 整改前COM3连续流量已实机复现并由ST-Link确认USART3中断活锁；较早P0修复固件完成约150秒COM3短回归。新版本完整闭环代码仍需执行COM1/2/3单路与并发压力、A-B短接、UART错误/HAL失败注入、CPU3看门狗冻结和CPU2阻塞业务并发台架验证。

## 2026-07-22 - 发布协议27固定地址、LH适配与AO继电器维护闭环

版本：
- CPU2：`V1.30.1.0 -> V1.31.0.0`（MINOR）。
- CPU3：`V1.29.1.0 -> V1.30.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由25提升到27；协议26的AO电流修正能力未单独形成固件组合，随协议27首个正式组合交付。协议27改为固定功能块地址直接索引，与旧紧凑地址程序不兼容，双端必须严格使用CPU2 V1.31.0.0 / CPU3 V1.30.0.0配套运行。
- CPU2共享Holding范围固定为`0x0000～0x0FFF`，共享Input范围固定为`0x0000～0x29FF`，CPU3本机Holding范围固定为`0x7000～0x706B`；继续只使用标准Modbus FC03、FC04和FC10。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、字段偏移、`struct_size`、FRAM A/B地址、magic、元信息和CRC范围均不变；AO修正复用原预留槽，旧协议存储升级时归零，不恢复出厂、不清除其它现场参数。
- CPU3本机参数存储版本保持V7 / `0x0007`；新增LH协议模板和运行态只复用现有配置结构及RAM上下文，不改变CPU3本机FRAM布局。

本次修改：
- CPU2/CPU3共享寄存器改为固定PDU地址直接索引，CPU3按公共、参数、运行态和分布数据分组轮询并保持原子快照；新增协议能力掩码、非阻塞维护、继电器最终逻辑动作/实际屏蔽状态、AO设备故障电流屏蔽及命令118/119。
- AO第4个32位预留槽正式定义为有符号电流修正值，单位0.01mA、范围-1.00～+1.00mA、默认0；启用态在最终输出阶段对基础目标只修正一次，保持路径不累计，禁用输出仍固定3.40mA。
- 维护模式由阻塞等待改为主循环状态驱动；继电器按报警、正反逻辑、延时、锁存和维护屏蔽计算最终逻辑动作。当前输入寄存器发布逻辑动作而非NO/NC物理触点反馈，清锁存命令按一次性请求消费。
- CPU3新增LH Modbus RTU从站，默认9600 8N1，支持FC03、FC04、FC05和FC10；命令等待CPU2合法ACK，参数写在ACK后继续等待FRAM完成代次并回读目标字段一致，异常按标准Modbus响应。
- LTD外部FC10继续以CPU2合法ACK作为成功条件，随后异步失效并补读参数快照，不误用LH的FRAM强确认语义；LTD外部默认串口统一为4800 8N1。屏幕协议字段和远程 `0x46` 都先在旧串口参数下完成ACK，再应用并保存目标协议整组默认模板；同协议切换也重新套用模板，不保留旧协议下的自定义串口值。
- SI Stop改为发送取消测量命令，不再隐式进入维护模式；同步DSM、LH、LTD、SI、AO、系统参数、CPU3菜单/状态页和版本索引资料。
- DSM二代Modbus手册停止位修正为1；新增LH协议手册和索引；《LNG计量仪屏幕菜单》同步CPU2 V1.31.0.0、CPU3 V1.30.0.0及协议27并消除重复空白页。

关键源码与边界：
- 固定地址链路位于双端 `stateformodbus.h`、CPU2 `hostcommu_modbus.c` 和CPU3 `cpu2_communicate.c`；CPU3通过 `poweron_groups`、`runtime_groups`、`refresh_hold_groups`分块读取，并只在完整候选满足代际条件后提交公开快照。必须覆盖功能块首末地址、空洞、32位对齐、125寄存器上限及协议25/27混搭。
- AO修正统一由CPU2 `AoOutput_ApplyCurrentCorrection()`在最终输出阶段执行；保持缓存只保存基础目标，禁用3.40mA不修正。必须覆盖 `-1.00/0/+1.00mA`、各输出模式、上下限幅、重复保持和AD5421写失败。
- 维护与继电器闭环由 `ProcessMeasureCmd()`、`CMD_EnterMaintenanceMode()`、`CMD_ExitMaintenanceMode()`、`CMD_ClearAllRelayLatchedAlarms()`和 `RelayOutput_BuildStateMask()`组成；寄存器发布的是屏蔽后的逻辑动作，不是NO/NC物理触点反馈。
- LH入口为 `lh_modbus_process()`；命令写以CPU2合法ACK为完成条件，持久参数还要经过 `lh_wait_parameter_persisted()`确认保存代次变化和目标字段回读一致。FRAM失败、超时、回读不一致和CPU2忙均不得返回成功。
- 协议切换由 `ProtocolSwitchFrame_Process()`识别、`cpu3_apply_ready_protocol_switches()`延后应用；ACK使用旧串口配置，随后 `Cpu3Local_WriteValueChecked()`应用目标默认模板。CRC错误静默、非法目标返回异常、FRAM或重初始化失败必须回滚或进入UART恢复队列。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：92步通过，生成V1.31.0.0产物；ELF为`text=309452`、`data=2328`、`bss=57832`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：62步通过，生成V1.30.0.0产物；ELF为`text=195832`、`data=37940`、`bss=100716`。
- `py -X utf8 tools\check_ao_output_enable_contract.py`、`check_ad5421_diagnostic_contract.py`和`check_fixed_point_generation_contract.py`通过，覆盖AO基础/修正分层、AD5421诊断及固定地址快照。
- `py -X utf8 tools\check_cpu2_uart_resilience_contract.py`通过42项；`py -X utf8 tools\check_cpu3_external_freshness_contract.py`通过66877项；CPU2通信故障、DSM、SI、Wärtsilä、参数/故障和协议切换专项检查通过。
- `py -m unittest tools.test_cpu3_protocol_switch`通过6项，覆盖LH 9600 8N1、同协议重套模板和非法目标；菜单短名与字库检查覆盖1075个显示字符串且缺字为0。
- 文档结构、Markdown链接、流程影响/流程资料和 `git diff --check` 通过；这些均为静态或构建证据，不替代本节未验证台架。
- DSM、LH和LNG三份Word分别完成17页、16页和29页渲染检查，未见明显乱码、重叠、截断或空白页。

未验证风险：
- 尚未完成协议27交叉烧写、真实RS485错波特率/校验、A-B短接、三路并发压力、FRAM A/B失败及掉电恢复、AO真实4～20mA电流环、继电器物理触点、维护/清锁存时序、LH现场主机和SI Stop实机回归。
- 固定地址数组使双端BSS明显增加，链接通过不能替代启动、栈余量和长稳验证；LH参数强确认最长同步等待约1.5秒，其对其它COM、OLED和看门狗的影响仍需并发台架记录。
- FRAM持久化失败时CPU2已更新的RAM参数不会回滚；LH会对外返回失败，但同次上电期间RAM读回可能仍是新值。
- 本机旧协议事实提取工具仍固化协议25紧凑地址和旧命令集合，当前失败不列为本版通过项，需后续单独升级工具基线。

## 2026-07-23 - CPU3维护显示闭环、LH状态投影与文档治理

版本：
- CPU2：保持 `V1.31.0.0`。
- CPU3：`V1.30.0.0 -> V1.31.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持27；固定功能块地址、命令118/119、AO修正、维护及继电器运行态的共享字段布局不变，双端仍必须使用协议27配套运行。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构、字段偏移、CRC范围和FRAM A/B地址不变；本次不改CPU2固件产物，不触发清参。
- CPU3本机参数版本保持V7 / `0x0007`；新增屏幕操作码、徽标状态和LH投影均不进入CPU3本机FRAM布局。

本次修改：
- CPU3系统维护菜单根据完整CPU2运行快照切换“进入维护模式”/“退出维护模式”；快照无效时显示维护状态未知且禁止下发，避免把未知误判为可进入状态。
- 补齐退出维护和清除全部继电器锁存报警的菜单名称、二次确认、命令映射、发送回执及返回路径；“指令已发送”与CPU2最终执行结果明确分层。
- 状态栏统一显示维护、AO仿真及叠加徽标；已确认维护开启后遇快照失效显示“维护?”，直到新快照明确关闭，并清理切行和长度变化时的残留区域。
- K1～K4继电器运行态页增加“动作禁用”和“最终动作”；快照无效时字段统一显示 `N/A`。最终动作仍是维护/人工禁用处理后的逻辑动作，不是NO/NC反相后的物理触点反馈。
- CPU3字库按用户提供的14×14点阵补入“求”字，同步 `StockMap` 和 `WordStock2`索引。
- LH设备状态只发布LH手册明确支持的内部状态及水位/罐高标定映射；其它协议或屏幕专用内部状态统一投影为待机，避免对LH主机暴露未定义码值。
- 整理CPU2串口助手说明、电机方案/台架资料和2026-07-23全量问题双格式总账；迁移重复资料到 `docs/02_需求与计划/` 与 `docs/05_测试记录/`，删除旧位置重复副本，并更新正式索引。
- 增加仓库根目录、固件构建目录与临时产物门禁；`tools/`、`docs/00_程序流程导航/`、`docs-site/`和临时/构建产物仍仅本机维护，不纳入本次提交。

关键源码与边界：
- 维护菜单闭环位于 `display_tankopera.c` 的 `menu_debug_system()`、`screen_operation_is_no_para_command()`、`ifsendcmd()`和`cmd_nopara_process()`；必须覆盖普通/维护/快照无效、确认/取消、发送失败和状态刷新竞态，CPU3“已发送”不能替代CPU2最终执行结果。
- 状态栏由 `display.c` 的 `DIS_Equipment()`统一维护徽标及清屏范围；继电器页由 `relay_status_state_of()`和`display_relay_status_row()`读取Block/Action。必须覆盖维护+仿真、维护后断链、字符串长短切换、K1～K4及快照失效 `N/A`。
- “求”字同时追加到 `StockMap` 与 `WordStock2`同一索引；静态字库检查只能证明索引和缺字，不证明OLED位序、偏移、笔画和残影。
- LH状态由 `lh_translate_device_state()`显式白名单投影；水位/罐高标定保留专用映射，其它协议、屏幕、保留和未知内部态统一为待机，不改变CPU2原始状态或其它协议。
- 本提交精确包含61个Git文件：4个CPU3业务源码、1个CPU3版本头、`AGENTS.md`、`CHANGELOG.md`及54项正式资料变更/迁移/删除；新PDF、`tools/`、流程导航、`docs-site/`和临时构建产物均排除。

验证：
- `py -X utf8 LTD_DISPLAY_CPU3/font_check.py`通过，475个字索引一致、1100个OLED字符串缺字为0；`check_cpu3_menu_name_width.py`、`check_cpu3_display_command_contract.py`和`check_cpu3_display_isr_boundaries.py`通过。
- `check_ao_output_enable_contract.py`、`check_wireless_rssi_contract.py`、`check_fixed_point_generation_contract.py`和`check_cpu3_cpu2_comm_timeout_fault_contract.py`通过；LH状态白名单完成针对性源码/主机契约检查。
- 文档结构、305份Markdown链接、XMind/OPML结构和Git差异门禁通过；静态结果不替代按键、OLED残影、RS485和继电器实测。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：92步通过，生成`LTD_MAIN_CPU2_V1.31.0.0.hex`；ELF为`text=309452`、`data=2328`、`bss=57832`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：62步通过，生成`LTD_DISPLAY_CPU3_V1.31.0.0.hex`；ELF为`text=197280`、`data=37964`、`bss=100716`。
- 《LNG计量仪屏幕菜单》Word已同步CPU3 V1.31.0.0并重新渲染检查；不为本次提交新增PDF。

未验证风险：
- 尚未完成OLED真机显示、按键确认/返回、维护快照失效、进入/退出/清锁存时序、继电器物理触点和LH现场主机状态码回归。
- 静态契约、Word渲染和构建成功不能替代真实CPU2/CPU3交叉烧写、RS485并发/故障注入、FRAM掉电、AO电流环或继电器台架验收。

## 2026-07-23 - 缩短CPU2看门狗监督节拍并收紧双端持久参数写门禁

版本：
- CPU2：`V1.31.0.0 -> V1.31.1.0`（PATCH）。
- CPU3：`V1.31.0.0 -> V1.31.1.0`（PATCH）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持27；共享寄存器地址、字段布局、命令值和外部合法帧格式均不变，两端仍按协议27配套运行。
- CPU2 `DEVICE_PARAM_VERSION`保持3；`DeviceParameters`大小、字段偏移、`struct_size`、FRAM A/B地址、magic、元信息和CRC范围不变，升级不恢复出厂、不清除现场参数。
- CPU3本机参数版本保持V7 / `0x0007`；本次只增加运行态写权限预检查，不改变CPU3本机FRAM布局。

本次修改：
- CPU2 TIM4从约2秒监督一次调整为250毫秒监督一次；继电器更新、AO刷新请求和UART恢复检查通过8分频继续保持约2秒周期，避免改变原业务时序，同时缩短瞬时ISR重叠后补喂IWDG的等待窗口。
- CPU2在板间FC10持久参数入口增加最终运行上下文门禁：只允许待机和明确白名单完成态；活动命令、待执行命令、持续态、预留态、带错误码的普通完成态以及自动故障恢复期间均返回标准Modbus设备忙`0x06`。错误态只有在命令队列和自动恢复均空闲时才允许修正持久参数。
- CPU3把相同状态白名单用于屏幕参数修改、通用CPU2持久参数桥接和LH FC10预检查；底层发送统一经过公共门禁，持续态、未知态和未来新增状态默认拒绝。CPU2仍保留最终裁决，避免CPU3快照滞后形成越权写窗口。
- LH参数写等待不再因单纯处于错误态提前失败；只要CPU2最终接受写入，LH继续等待持久化完成代次和目标字段回读一致。超时、通信失败、最终回读不一致仍返回失败。
- 同步全量问题核对清单、专项问题报告和问题索引，统一C-01～C-45、R-01～R-05、B-01～B-10、F-01～F-21的当前状态、证据边界和历史映射；这些文档更新不把静态核查冒充台架闭环。

关键源码与边界：
- TIM4由 `MX_TIM4_Init()`把Period改为1249，`TIM4_IRQHandler()`以 `CPU2_TIM4_BUSINESS_DIVIDER=8`分离250毫秒监督节拍和约2秒继电器/AO/UART业务节拍；必须覆盖计数边界、LSI偏差、瞬时/持续ISR占用及三类业务周期不漂移。
- CPU2 `DeviceState_AllowsPersistentParamWrite()`和`DeviceContext_AllowsPersistentParamWrite()`组合状态、当前命令、待执行命令、错误码及 `FaultRecovery_IsActive()`；`Response10Process()`拒绝时返回 `0x90/0x06`且不得应用候选或请求保存。
- CPU3屏幕 `state_allows_param_write()`、通用 `CPU2_CombinatePackage_Send()`和LH入口复用显式白名单，只做保守预检查；CPU3快照可能滞后，因此所有放行仍必须经过CPU2最终裁决，未知/未来状态默认拒绝。
- LH `lh_write_holding_field()`只在CPU2 ACK后进入 `lh_wait_parameter_persisted()`；错误态本身不再提前判失败，但恢复活动、完成代次超时、通信失败或字段回读不一致仍不得成功。
- 本提交精确包含32个Git文件：11个双端业务源码/工程文件、2个版本头、`CHANGELOG.md`、1份版本方案及17份问题资料/索引；未跟踪PDF和本机流程/工具产物不纳入。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：92步通过，生成`LTD_MAIN_CPU2_V1.31.1.0.hex`；ELF为`text=309836`、`data=2328`、`bss=57832`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：62步通过，生成`LTD_DISPLAY_CPU3_V1.31.1.0.hex`；ELF为`text=197944`、`data=37964`、`bss=100716`。
- `py -X utf8 tools/check_cpu2_uart_resilience_contract.py`通过，覆盖250毫秒节拍、8分频和UART恢复预算；`check_cpu3_cpu2_comm_timeout_fault_contract.py`通过板间通信超时契约。
- `check_dsm_compat_contract.py`、`check_wartsila_distribution_logic.py`、`check_si_protocol_contract.py`、`check_cpu3_display_command_contract.py`和`check_parameter_measurement_fault_contract.py`通过。
- `py -X utf8 tools/check_docs_structure.py`通过，统计600个正式文档；`check_markdown_links.py`检查305个Markdown，缺失链接为0；`git diff --check`通过。

未验证风险：
- 尚未在目标板测量TIM4实际250毫秒节拍、IWDG的LSI偏差、瞬时/持续ISR占用下的复位时间，以及继电器、AO和UART恢复仍保持约2秒周期。
- 尚未用CPU2/CPU3实机逐项验证待机、完成态、持续态、活动命令、待执行命令、自动故障恢复和错误态的屏幕/LH/DSM/Wärtsilä/SI/LTD持久参数写入矩阵。
- 静态契约和clean-first构建不能替代FRAM写失败/掉电、RS485并发、真实LH主机和现场参数修正流程验证。

## 2026-07-23 - 新增固定频率找液位、LH FC01并修复区间测量与传感器失败传播

版本：
- CPU2：`V1.31.1.0 -> V1.32.0.0`（MINOR）。
- CPU3：`V1.31.1.0 -> V1.32.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持27；本次不改变板间共享寄存器地址、字段布局、命令值或参数语义。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、字段偏移、`struct_size`、CRC范围和FRAM A/B地址不变；升级不恢复出厂、不清除现场参数。
- CPU3本机参数版本保持V7 / `0x0007`；LH新增FC01属于CPU3外部协议能力，不改变CPU3本机FRAM布局。

本次修改：
- CPU2新增`LF`、`LF=<频率>[,<死区>]`和`LF?`串口调试命令：执行一次固定频率找液位闭环，按频差决定方向和速度，并以位置、扭力、传感器通信、丢步、命令切换和10分钟超时作为停机保护；临时覆盖值不写参数或FRAM。
- CPU2区间密度点规划把点数和索引统一为32位无符号类型，显式传入输出容量；0点、超过200点或超过容量时直接失败并保持输出点数为0，不再静默钳位。区间测量失败后不再发布空/部分点阵，也不进入完成态。
- LTD/V2响应第7字节为`0xFF`时映射到现有`SENSOR_REMOTE_INTERNAL_ERROR`并进入有限重试，失败帧不再被当作成功解析；重试恢复日志记录最后一次具体错误原因。
- CPU3 LH从站新增FC01读线圈兼容：合法`0x0000～0x0010`范围统一返回全0位图，不回显历史命令或运行状态；数量0、超过2000或越界分别返回标准Modbus异常。
- 同步CPU2串口调试协议卷、LH协议索引和16页LH Word手册；更新高价值问题分批方案、需求索引和故障码复核口径。

关键源码与边界：
- LF链路为 `SerialCommandParser_Parse()` → `Test_ProcessSerialCommand()` → `FixedFrequencyLevelSearch_Run()`；默认目标/死区来自现有参数但本次覆盖仅驻留单次运行。必须覆盖语法、0/6500边界、方向切换、连续3次稳定、STOP/新命令、位置/扭力/通信/丢步和10分钟超时，所有退出都要先安全停机。
- `BuildPoints_Interval_Exact()`把点数、索引和容量统一为32位无符号，入口先清 `out_n`；必须覆盖0、127、128、129、199、200、201和canary。`CMD_MeasureDensitySpread_Interval()`失败后立即返回，不发布空/部分点阵或完成态。
- `DSM_V2_CheckReply()`在功能码校验后把应答第7字节 `0xFF`映射为 `SENSOR_REMOTE_INTERNAL_ERROR`；模式、浮点和整数读取只提交完整成功帧，有限重试耗尽后保留最后一次具体失败原因。
- LH `lh_handle_read_coils()`只接受1～2000且不越过17线圈范围的8字节FC01请求，合法响应按位打包全0；该全0口径只用于兼容探测，不是命令历史或状态回读。
- 本提交精确包含22个Git文件：10个CPU2/CPU3业务源码与工程文件、2个版本头、`CHANGELOG.md`、1份版本方案及8项正式协议/需求/问题资料；3个未跟踪PDF及本机工具、流程、构建产物均排除。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：通过；最终V1.32.0.0产物为`text=312420`、`data=2328`、`bss=57832`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：62步通过；最终V1.32.0.0产物为`text=198112`、`data=37964`、`bss=100716`。
- 主机编译并运行 `tools/test_serial_command_parser.c`，LF合法/非法/边界及CRLF/LF用例通过。
- `py -X utf8 tools/check_sensor_fault_contract.py`、`check_density_profile_publish_contract.py`和`check_parameter_measurement_fault_contract.py`通过；C/C++新增/修改行未新增`//`注释。
- LH Word手册完成16页逐页渲染检查，未见裁切、重叠或乱码。

未验证风险：
- 尚未用真实电机、传感器和扭力输入验证LF闭环方向、速度、稳定确认、机械边界、STOP打断、丢步和10分钟超时。
- 尚未完成128～200点区间测量canary/目标板回归、LTD/V2 `0xFF`真实或golden frame注入、LH FC01现场主机和RS485异常帧测试。
- clean-first构建和静态契约不能替代CPU2/CPU3交叉烧写、真实机械运动、传感器故障注入及长时间并发测试。

## 2026-07-25 - 发布协议28命令参数事务、继电器配置校验与CPU3快照门禁解耦

版本：
- CPU2：`V1.32.0.0 -> V1.33.0.0`（MINOR）。
- CPU3：`V1.32.0.0 -> V1.33.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由27提升到28。协议28改变七个命令前置参数的运行态生命周期和跨CPU确认语义，CPU2/CPU3必须使用V1.33.0.0配套运行；协议27及更早组合由严格相等门禁拦截。
- 不新增、删除或移动共享寄存器，不改变字段宽度、字序、缩放、命令值、设备状态和故障码。
- CPU2 `DEVICE_PARAM_VERSION`保持3；`DeviceParameters`大小、字段偏移、`struct_size`、magic、CRC范围、FRAM A/B地址和单槽容量不变，升级不恢复出厂、不清除现场参数。
- CPU3本机参数版本保持V7 / `0x0007`；CPU2的pending/active快照、字段代次和CPU3逐字段ACK确认位图均只存在RAM，不改变本机FRAM布局。

本次修改：
- CPU2为七个命令前置参数建立pending/active快照和逐字段代次。参数写成功后更新pending，命令被主循环接受时原子绑定active；命令执行和自动恢复使用已绑定值，后续参数写只供下一条命令使用。
- 活动态只放开这七个命令相关参数，普通持久配置仍沿用既有状态门禁。协议28不增加端口所有权、来源ID或事务租约；屏幕与上位机交错写同一字段时，以命令到达前最后一次CPU2 ACK成功值生效。
- CPU3在七字段FC10合法ACK后立即提交局部镜像并记录逐字段确认资格，随后后台补读完整参数；参数刷新期间仅允许实际所需字段均已确认的带参命令凭运行态下发。命令ACK后只消费该命令对应资格，不改变原有命令打断、自中断白名单、重复命令和单命令槽规则。
- 拆分CPU3快照门禁：普通运行态只依赖状态、当前连接协议、通信会话和严格协议匹配；完整参数、固定点结果和Profile分别使用各自门禁。LTD FC04普通字段不再等待完整参数或固定点，`0x0030～0x0033`及`0x1100～0x113F`固定点字段单独检查固定点快照，`0x1140`起分布汇总继续按运行态处理；DSM、SI、Wärtsilä固定点派生字段同步使用专用门禁。
- 区分失败类型：参数写结果不确定仍关闭完整参数快照并立即请求全量刷新；普通命令超时、坏帧或UART失败只消费对应命令参数确认资格；恢复出厂继续保留立即关闭参数读取并等待参数代次后全量刷新。TX DMA未启动不消费确认资格，CPU2标准Modbus异常按确定失败处理且不累计物理通信失败。
- 继电器报警配置改为禁用通道可逐字段配置、启用时完整校验；已启用通道不允许写入破坏阈值顺序或物理范围的单字段，非法历史浮点逐字段归零，纯顺序冲突保值但禁用通道。四路阻尼预留字段继续固定为0。
- 参数打印和浮点寄存器转换复用统一辅助函数；CPU3屏幕、LH、LTD、DSM、SI和Wärtsilä继续以CPU2合法ACK作为成功依据，不增加自动重发。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`通过，生成`LTD_MAIN_CPU2_V1.33.0.0.hex`；ELF为`text=314116`、`data=2328`、`bss=57992`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`通过，生成`LTD_DISPLAY_CPU3_V1.33.0.0.hex`；ELF为`text=201896`、`data=37964`、`bss=100716`。
- `check_command_argument_transaction_contract.py`通过，确认协议28、七字段和“最后一次CPU2 ACK成功值”语义；`check_cpu3_external_freshness_contract.py`通过66892项。
- `check_cpu3_cpu2_comm_timeout_fault_contract.py`、`check_cpu3_display_command_contract.py`、`check_dsm_compat_contract.py`、`check_relay_alarm_config_contract.py`、`check_si_protocol_contract.py`和`check_wartsila_distribution_logic.py`通过。

未验证风险：
- 尚未执行真实CPU2/CPU3交叉烧写、屏幕与上位机并发写同字段、ACK丢失/坏帧/超时、FRAM掉电和恢复出厂时序测试。
- 尚未用真实RS485主站逐项验证DSM、SI、Wärtsilä、LTD和LH的读写Busy边界，也未用电机和传感器验证带参命令的实际动作值。
- 静态契约和clean-first构建证明源码契约与可编译性，不替代台架通信压力、物理继电器输出和真实运动验收。

## 2026-07-25 - 发布协议29故障码、远程协议保持串口参数与CPU3非阻塞启动保护

版本：
- CPU2：`V1.33.0.0 -> V1.34.0.0`（MINOR）。
- CPU3：`V1.33.0.0 -> V1.34.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由28提升到29。协议29删除12-11并交换18/21故障类别，CPU2/CPU3必须使用V1.34.0.0配套运行；不提供协议28旧故障码别名、运行时翻译或规避。
- 13-2统一为“震动管频率异常”；13-26密度有效上限由2000.0 kg/m³调整为3000.0 kg/m³，3000.0合法，大于3000.0形成故障。
- 12-6、15-16、21-5按产品决定继续作为预留项冻结保留；一代和协议28历史资料保持历史口径，不用于解释协议29设备。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、字段偏移、`struct_size`、magic、CRC范围、FRAM A/B地址和单槽容量不变，升级不恢复出厂、不清除现场参数。
- CPU3本机参数版本保持V7 / `0x0007`；远程协议切换继续复用现有端口配置字段，不改变本机FRAM布局。

本次修改：
- CPU2/CPU3共享故障码同步取消12-11，把扭力检测故障调整为18类、模拟输出与自检故障调整为21类；同步CPU2日志、CPU3显示、当前清单、协议资料和两份工作簿。
- CPU2密度有效性判断把13-26上限调整为3000.0 kg/m³；13-2双端文字统一为“震动管频率异常”。
- CPU3 OLED字库新增“震”字14×16点阵，显示13-2时不再以“振”代替。
- CPU3远程`0x46`协议切换只修改目标端口协议，保持当前波特率、校验、数据位、停止位和地址；屏幕配置仍使用目标协议默认串口模板。
- 切换ACK使用旧协议和旧串口参数完整发送，发送完成后由主循环保存并逐口重初始化；FRAM保存/读回失败恢复旧端口配置，成功或回滚都恢复目标口RS485接收方向和RX DMA。
- 删除CPU3首屏前两个固定1000 ms延时；`App_Init()`末尾原阻塞1秒改为1000 ms非阻塞外部COM保护窗口。保护期继续显示、CPU2轮询、板间UART5恢复和日志，暂停COM1/2/3外部UART恢复、协议切换应用、UART重配、SI周期任务及外部帧处理，窗口结束后恢复原有外部任务顺序。
- 正式版本方案、版本总览、问题台账HTML/Markdown、各协议说明、菜单资料、`CHANGELOG.pdf`和CPU2/CPU3自编代码规模报告同步归档。

验证：
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`通过，生成`LTD_MAIN_CPU2_V1.34.0.0.hex`；ELF为`text=313996`、`data=2328`、`bss=57992`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`通过，生成`LTD_DISPLAY_CPU3_V1.34.0.0.hex`；ELF为`text=202432`、`data=37972`、`bss=100732`。
- 字库、故障码、密度控制、远程协议切换、SI、显示命令、CPU3协议快照、固定点代际和显示ISR边界等静态/主机契约通过；CPU3外部新鲜度契约通过66892项。
- 本机流程资料已从生成源重建，33页流程HTML契约检查通过。

未验证风险：
- 尚未执行真实CPU2/CPU3交叉烧写、协议28/29错配、升级不清参和真实OLED“震”字显示测试。
- 尚未用真实RS485验证旧参数ACK、三口并发、持续流量、屏幕/远程交错切换以及FRAM/UART失败回滚。
- 尚未用真实传感器验证2999.99/3000.00/3000.01边界，也未测量上电1000 ms保护窗口、SI分钟边界、CPU2轮询和看门狗实际时序。
- 静态契约、主机测试和clean-first构建证明源码契约与可编译性，不替代台架通信、显示、掉电和故障恢复验收。

## 2026-07-28 - 发布协议30编码器故障锁存、FRAM仲裁与24V掉电恢复

版本：
- CPU2：`V1.34.0.0 -> V1.35.0.0`（MINOR）。
- CPU3：`V1.34.0.0 -> V1.35.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由29提升到30，新增23-1～23-6整机供电与电源监控共享故障；协议29及更早CPU3不能解释新编号，双端必须使用协议30严格配套。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构、大小、`struct_size`、magic、CRC范围和参数FRAM A/B槽均不变；CPU3本机参数版本保持V7 / `0x0007`，升级不恢复出厂、不清除现场参数。
- 编码器独立位置记录只写V2并兼容读取V1；未发布的中间格式不保留兼容分支。

本次修改：
- 编码器SPI5中断只采集并投递事件，由PendSV处理连续帧判定；任意连续3帧异常锁存编码器故障，正常帧、通信恢复、内部重试和取消命令不自动清锁存，新的顶层正式测量或回零命令允许清锁存。
- SPI4 FRAM访问改为单所有者、有限超时和状态检查；编码器位置采用A/B记录、代次、提交标记、CRC与读回验证，运行期每累计256 counts投递周期保存。
- 24 V监控采用20 V欠压、22 V恢复和100 ms稳定窗口；真实低压立即禁止运动并投递紧急位置保存，紧急保存最多尝试3次，可信位置在启动恢复、回零或人工修正后可自动重新布防。
- ADC溢出或DMA停止后立即进入安全禁止，主循环最多执行3次局部恢复；恢复成功保留并报告故障快照，新的正式流程可在电源和监控链路重新可信后继续，连续3次恢复失败保持禁止并报告23-5。
- 位置不可信时继续禁止依赖绝对位置的运动和测量，但允许明确回零及标定零点；`set_encoder_zero()`向上传播FRAM持久化结果，标零保存失败时不继续首圈周长标定。
- 参数Modbus写成功仍表示已接受到RAM，持久化由主循环延后执行；连续保存失败通过存储错误和读回暴露，不把协议成功响应解释为已经落盘。
- CPU3同步协议30故障定义和23类故障原因显示；故障码工作簿、当前清单、共享协议、整改方案、菜单资料和版本资料同步归档。

验证：
- CPU2和CPU3按最终源码执行clean-first构建，生成`LTD_MAIN_CPU2_V1.35.0.0.hex`和`LTD_DISPLAY_CPU3_V1.35.0.0.hex`。
- 故障码、共享协议、参数/传感器安全契约、文档结构、编码、行尾空白和版本门禁按提交前最终暂存区执行检查。
- 既有PDF只做可打开性和页数核查，未在本轮重新生成。

未验证风险：
- 尚未执行真实20 V／22 V阈值、掉电保持时间、随机断电、FRAM阶段故障注入、ADC OVR/DMA停止及三次恢复故障注入。
- 尚未执行最大电机速度下保存队列与FRAM压力、CRC关中断时长、PendSV最坏占用、RS485通信压力、OLED故障显示和真实电机动作测试。
- 静态契约和clean-first构建只证明源码一致性与可编译性，不替代台架供电、存储、通信、显示和运动验收。

## 2026-07-29 - 发布协议31扭力模块温度显示并修复标定与Newhall帧门禁

版本：
- CPU2：`V1.35.0.0 -> V1.36.0.0`（MINOR）。
- CPU3：`V1.35.0.0 -> V1.36.0.0`（MINOR）。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`由30提升到31。输入寄存器`0x0116~0x0117`地址和32位宽度不变，原未使用“扭力参数”槽改为Newhall扭力模块温度IEEE754 float32原始位；协议30及更早程序不能按新语义解释该槽，双端必须使用V1.36.0.0严格配套。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、字段偏移、`struct_size`、magic、CRC范围和参数FRAM A/B地址不变；本次升级不恢复出厂、不清除现场参数。
- CPU3本机参数版本保持V7 / `0x0007`，不改变CPU3本机FRAM布局。

本次修改：
- 归档Newhall称重主控UART协议卷，记录19200 8N1、`0x81/0x82/0x83`帧、字段、BCC、主动上报、来源哈希和联调边界。
- CPU2在UART4入口统一校验精确22字节、`CC 03 83`帧头和前21字节异或BCC；只有完整合法帧才能更新重量、刷新通信接收时间并解析偏移15～18的扭力模块温度，坏帧不能清除扭力通信超时。
- 双端把原`weight_param`占位字段和`REG_DEBUG_WEIGHT_PARAM`统一改名为`torque_temperature_bits`和`REG_DEBUG_TORQUE_TEMPERATURE_BITS`，寄存器地址与轮询块长度不变；未收到有效温度时发布安静NaN。
- CPU3在空载/满载扭力获取等待页显示温度，并在`STATE_READPARAMETEROVER`读取部件参数结果分页新增“扭温”项；NaN、无穷或超出显示范围时显示`--.--`。
- 水位标定收到`STATE_SWITCH`后立即退出，不再继续修正和保存旧结果；水位罐高变化后同步归一化AO量程。
- FC10候选AO校验增加写地址和寄存器数量感知：未触及AO且动态上限未变化时允许无关写入，触及AO的同值写仍严格校验历史非法配置。
- 零值罐高标定先保留候选罐高，安全抬升成功后才提交罐高、AO归一化和位置刷新；编码器罐底修正显式使用候选罐高，保持原目标语义。

验证：
- 提交前执行Newhall帧门禁、参数/测量和AO候选配置针对性静态契约，检查协议31、寄存器地址、命名、标定退出与延迟提交。
- 提交前执行`py tools\check_docs.py`、编码/换行、无新增`//`注释、`git diff --check`、版本门禁和根目录布局检查。
- 提交前按最终隔离源码执行CPU2/CPU3 clean-first构建，并核对固定名与版本名HEX哈希一致。

未验证风险：
- 尚未用真实UART4抓包确认Newhall温度float32字节序、首帧0 ℃和温度落后一轮行为；未执行坏帧、粘连、断线恢复及OLED负温/无效占位实机验证。
- 扭力温度通信超时后仍会保留最后有效值；读取部件参数也不保证取得命令开始后的新温度帧，这两项已记录并按用户决定暂缓。
- 尚未执行真实`STATE_SWITCH`命令切换、电机安全抬升失败注入、FRAM写失败、AO物理电流和水位/罐高标定台架回归；静态检查和构建不能替代硬件证据。

## 2026-07-29 - 修复CH9141K RSSI半ACK清理与透传恢复

版本：
- CPU2：`V1.36.0.0 -> V1.36.1.0`（PATCH）。
- CPU3：保持`V1.36.0.0`。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持31，不新增或改变共享寄存器、命令、状态及字段语义。
- CPU2 `DEVICE_PARAM_VERSION`保持3，参数结构、FRAM布局和升级清参行为均不变。

本次修改：
- RSSI查询在发送`AT+RSSI=ON`前即记录“已尝试开启”，即使模块执行命令后只返回半个ACK，收尾阶段仍发送`AT+RSSI=OFF`。
- 普通`AT+RSSI=OFF`或`AT+EXIT`遇到短ACK、UART/DMA异常或命令切换时，使用有界强制清理依次发送`AT+RSSI=OFF\r\n`和`AT+EXIT\r\n`，随后终止DMA、清除UART错误并排空残留数据，避免异步`RSSI:-xxdB`污染DSM透传响应。
- 连接状态读取显式透传RSSI阶段的`STATE_SWITCH`，维护连接状态命令同步使用相同清理边界。

验证：
- 无线RSSI契约、传感器故障契约、GBK/936与CRLF、无新增`//`注释及`git diff --check`通过。
- CPU2按隔离提交源码执行clean-first构建，固定名与`V1.36.1.0`版本名HEX哈希一致。

未验证风险：
- 尚未在真实CH9141K上注入`AT+RSSI=ON`半ACK、`AT+RSSI=OFF`/`AT+EXIT`丢ACK、UART FE及DMA异常。
- 尚未在真实DSM透传链路验证恢复后首条`Cl`响应；静态契约和构建不能替代台架故障注入。

## 2026-08-04 - 修复液位标定后AO量程归一化并归档近期设计资料

版本：
- CPU2：`V1.36.1.0 -> V1.36.2.0`（PATCH）。
- CPU3：保持`V1.36.0.0`。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持31；不新增或改变共享寄存器、命令、状态、字段宽度或字段语义，CPU3无需升版。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`大小、字段偏移、`struct_size`、CRC范围和FRAM A/B地址不变；升级不恢复出厂、不清除现场参数。
- 罐高变化后只把已有AO端点归一化到新动态上限，不新增参数，也不改变AO寄存器地址、单位或持久化布局。

本次修改：
- `CorrectOilLevelProcess()`在液位标定成功写入新`tankHeight`后立即调用`normalize_ao_params_after_write()`，使储罐液位源的AO高低端点同步受新罐高约束，避免旧端点阻塞后续参数命令或继续参与输出换算。
- 归档并同步近期正式资料：故障码工作簿与现场文案、共享/外部协议和菜单口径、DSM `Cl`未发出问题分析、传感器安全通信协议资料、CPU2/CPU3自编代码规模报告，以及相关README和版本索引。
- 建立CPU2软件详细设计工作包和2026-08-03中期验收冻结包，纳入总册、8个专业分册、Visio源图、撰写标准、哈希清单和验收说明；中期包仍明确为待审批文档，不代表目标板、台架、现场、故障注入或SIL认证通过。
- 重整SIL资料目录，归档DM4 V6.0四块原理图及四板架构/测量链路/安全证据边界分析；原理图静态分析不等于硬件、PCB、故障覆盖率或SIL验证。

验证：
- CPU2按最终V1.36.2.0源码执行clean-first构建；参数/测量故障契约、文档结构与链接、版本门禁、UTF-8/乱码、OOXML/PDF可读性、`git diff --check`和根目录布局检查通过。
- CPU2软件详细设计中期包当前哈希与清单一致，既有验收报告197/197项通过；10份冻结Word合计297页渲染证据完整，289页现行设计已完成逐页复核。
- 故障码工作簿8个页签/8张表可导入，公式错误扫描为0，8个页签均完成可视化抽查。

未验证风险：
- 尚未在真实液位标定流程中验证罐高变化后的AO物理电流、FC10后续写入、FRAM保存/掉电恢复以及上位机和屏幕显示。
- 尚未执行CPU2 V1.36.2.0与CPU3 V1.36.0.0交叉烧写；静态检查和clean-first构建不替代RS485、OLED、传感器、电机、AO电流环或现场验收。
- 软件详细设计和DM4资料仅形成可审查的文档/静态证据，不构成需求批准、硬件验证、故障注入、FMEDA或SIL认证结论。

## 2026-08-07 - 修复电机目标运动固定一小时超时

版本：
- CPU2：`V1.36.2.0 -> V1.36.3.0`（PATCH）。
- CPU3：保持`V1.36.0.0`。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持31；不新增或改变共享命令、寄存器、状态、字段宽度或字段语义，CPU3无需升版。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构、大小、字段偏移、`struct_size`、CRC范围和FRAM A/B地址不变；升级不恢复出厂、不清除现场参数。
- `MOTOR_RUN_TIMEOUT`错误码、命令切换、驱动健康、碰撞检测和既有故障恢复策略保持不变。

本次修改：
- `MotorCtrl_MoveAndWait()`在临时速度已生效、相对目标已规划后，按`理论时间 = 距离(mm) × 60 ÷ 生效线速度(m/min)`计算单次等待预算；允许时间取`max(3600000 ms, 理论时间 × 2 + 60000 ms)`，不再对所有目标运动统一使用固定1小时上限。
- `MotorMotion_WaitUntilStopWithTarget()`改为接收本次允许时间；超时时保留原`MOTOR_RUN_TIMEOUT`出口，并在结构化日志和现场打印中分别记录实际已运行时间与允许时间。
- 本次不增加丢步检测、不修改自动恢复、故障码、协议、参数和CPU3行为。

验证：
- `py tools\check_motor_motion_target_plan.py`和`git diff --check`通过；目标源码保持CP936、CRLF且未新增`//`注释。
- CPU2按最终V1.36.3.0源码执行`cmake --build build\LTD_MAIN_CPU2 --clean-first`通过，生成`LTD_MAIN_CPU2_V1.36.3.0.hex`；ELF为`text=326596`、`data=2344`、`bss=58352`。

未验证风险：
- 尚未在真实电机上执行短距离、长距离、最低/最高速度、临界等待预算、命令切换和真实超时验证。
- 静态检查与clean-first构建只证明源码契约、编码和可编译性，不替代目标板、电机机构、编码器、故障恢复或现场验收。

## 2026-08-07 - 修复DSM定长收帧与UART6透明链路交接

版本：
- CPU2：`V1.36.3.0 -> V1.36.4.0`（PATCH）。
- CPU3：保持`V1.36.0.0`。

协议版本与兼容性：
- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION`保持31；不新增共享命令、寄存器、状态或字段，CPU3无需升版。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构、大小、字段偏移、`struct_size`、CRC范围和FRAM地址均不变化；升级不恢复出厂、不清除现场参数。
- DSM传感器线上命令和响应字节保持兼容：`CL/CB/CD`仍使用`25 0D 0A` ACK，`CN/CK/Cb/Cd/Ch/Cl`仍使用既有BCC和CRLF，`CV/Cv`仅补齐既有六字节无CRLF响应的事务层支持，未加入自动探测顺序。
- 保留测水电容小于`30.0 pF`映射为`99999.9 pF`的产品规则。

本次修改：
- 为DSM命令建立统一响应规格，按命令和首字节确定正常帧或19字节错误帧的精确长度；取消遇到首个`0x0A`即截帧的历史逻辑，避免BCC为`0x00/0x0A/0x0D`时出现短帧、格式错误或校验错误。
- `CL/CB/CD`只接受精确`25 0D 0A`；`Cd`频率字段兼容六位整数和小数，密度、温度继续要求小数；所有七字节数值字段先做格式校验并限长解析，禁止把后续BCC当成数字。
- UART6停止DMA前后锁存FE/NE/ORE/PE错误位，并在协议解析前消费本事务快照；错误6、7明确映射为远端内部错误，禁止把`666.66`或`777.88`作为真实倾角。
- DSM重试日志按命令输出真实操作名；合法低电压帧继续解析，短帧、长帧、坏BCC、坏CRLF、未知命令和AT文本污染进入明确错误出口。
- CH9141K新增UART6链路状态机；AT退出、模块复位和RSSI清理必须完成不可中断的透明链路交接，失败时保持`NOT_READY`并向上返回具体错误。新DSM事务在`NOT_READY`时只执行一次有界恢复，其它AT占用状态直接拒绝发送，避免永久模式未就绪或AT异步文本污染DSM响应。
- 无线连接状态查询和调试扫描检查RSSI关闭、AT退出、恢复及交接结果；部件参数读取在透明传输未确认时停止后续传感器访问，不再吞掉错误。
- 无丢步运行日志的密度读取周期从本次阻塞读取结束后重新计时，避免慢事务结束后立即再次占用UART6。

验证：
- DSM UART6帧、DSM兼容性、DSM CN低电压和无线RSSI契约检查通过；覆盖正常帧、全长度短帧、长帧、坏BCC、坏CRLF、精确ACK、额外`%`、BCC特殊值、错误1至9、现场整数频率帧、版本帧、AT文本污染和透明链路恢复形状。`check_sensor_fault_contract.py`依赖另一并行任务尚未提交的多参数协议源码，不登记为本隔离候选证据。
- 目标源码按CP936严格解码，无UTF-8 BOM、无孤立LF、无`U+FFFD`且未新增`//`注释；`git diff --check`和仓库根目录门禁通过。
- CPU2按最终V1.36.4.0范围提交候选执行clean-first构建并生成版本HEX；具体ELF规模和HEX哈希以本版本测试方案中的最终记录为准。

未验证风险：
- 尚未使用真实CH9141K执行半ACK、无ACK、异步RSSI残留和AT退出失败台架验证。
- 尚未在目标板注入UART FE/NE/ORE、DMA启动/停止失败，也未主动构造传感器错误6、7。
- 尚未完成真实DSM传感器的冷启动、连续`SC`、密度/油位/水位/倾角流程和长时间运行；静态契约与clean-first构建不替代目标板、传感器和现场验收。
