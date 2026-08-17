# SI协议适配改动整理

## 1. 基本信息

| 项目 | 内容 |
| --- | --- |
| 整理日期 | 2026-05-16 |
| 当前分支 | `codex/si-protocol-adapter` |
| 合并来源 | `wip/si-protocol-assist` |
| 协议依据 | `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000协议.docx` |
| CPU2版本 | `V1.7.3.0` -> `V1.8.0.0` |
| CPU3版本 | `V1.5.0.0` -> `V1.6.0.0` |
| CPU2/CPU3共享协议版本 | `DEVICE_PROTOCOL_VERSION` 从 `3` 升级到 `4` |
| 当前暂存快照 | 2026-05-21 复核：46 个文件，5118 行新增，90 行删除 |
| 最近复核内容 | 暂存区文件清单、改动统计、SI协议地址表检查、CPU2/CPU3 共享协议契约检查 |

本次改动目标是在保留当前主分支既有测量、取消、错误恢复、日志和电机封装逻辑的基础上，合并前期外部协议适配辅助改动，并依据原始协议资料继续完成 CPU3 外部 Modbus RTU 从站适配。CPU3 新增 SI协议入口和本地时钟，CPU2 只新增通用协议适配辅助状态字段，用于补足 PLC 协议需要但原有 CPU2/CPU3 数据结构中没有直接表达的状态。

文档目录整理已单独提交为 `a5a65cb docs: 整理项目文档目录`。本文件记录该提交之后剩余的 SI协议适配改动，包括代码、版本、验证脚本、SI协议专项文档和从 `docs/SI协议.docx` 迁移到 `../00_原始资料/SI7000协议.docx` 的原始协议文档。

阅读口径：第1～14节保留2026-05第一版适配过程；第15节保留协议14引入SI Profile独立兼容时的阶段快照，第16节记录2026-07-10 CPU2通信门禁和写失败反馈；这些历史章节中的“当前”只表示当时基线。SI功能变更以第17节的协议18实现为准；最新正式组合为协议34、CPU2 V1.40.0.0 / CPU3 V1.40.0.0。协议27重排CPU2/CPU3共享地址，并把SI Stop改为取消当前测量、不再隐式进入维护模式；协议28～34继续沿用SI对外地址归属、Point0时间、递增点数、最终发布和CPU3 FRAM V7。

## 2. 改动范围概览

本轮剩余改动主要分为以下几类：

| 类别 | 主要内容 |
| --- | --- |
| 分支合并 | 合并 `wip/si-protocol-assist`，解决 CPU2 测量流程冲突 |
| CPU2 协议辅助 | 将 SI 所需状态融合进既有测量结构，并同步到底层 Modbus 输入寄存器 |
| CPU2 测量状态 | 在底部基准、液位搜索、手动运行、密度剖面流程中维护外部协议适配所需通用状态 |
| CPU3 SI协议从站 | 新增 SI Modbus RTU 从站，支持 FC01/02/03/04/05/06 |
| CPU3 时钟 | 新增 CPU3 RTC/LSI 时钟模块，输出当前时间和剖面时间戳 |
| CPU3 通信配置 | 新增协议枚举、显示菜单选项、串口参数范围和 8O1 字长修正 |
| 中文注释 | 对 CPU2 测量状态维护、CPU3 SI协议转换、RTC、串口归一化和验证脚本补充中文注释 |
| 版本和文档 | 升级 CPU2/CPU3 版本，更新协议变更记录、CHANGELOG、SI协议专项文档和 PLC 联调表 |

截至 2026-05-21 对当前暂存区复核，文件分组如下：

| 分组 | 文件数 | 说明 |
| --- | ---: | --- |
| CPU2 固件代码 | 7 | 版本、共享结构、输入寄存器、测量状态维护和 profile 生命周期 |
| CPU3 固件代码/构建 | 17 | 外部协议分发、串口参数、RTC、SI 从站、共享结构和 CMake 接入 |
| 文档和资料 | 20 | CHANGELOG、README、CPU2/CPU3 协议记录、SI协议原始资料、映射表、计划、改动记录和联调表 |
| 验证工具 | 2 | SI Modbus 参考帧检查、CPU2/CPU3 共享协议契约检查 |

## 3. 合并与冲突处理

从 `wip/si-protocol-assist` 合并前期 SI协议相关工作时，主要冲突集中在 CPU2 测量流程：

| 文件 | 处理方式 |
| --- | --- |
| `LTD_MAIN_CPU2/Application/Src/measure.c` | 保留当前主分支的命令取消、错误恢复、日志、电机控制封装和流程边界，合入外部协议适配辅助状态维护 |
| `LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c` | 保留当前液位测量逻辑，补充探头到液面、液位稳定等协议辅助状态 |

处理原则：

- 不回退当前主分支已有的测量、恢复和日志改动。
- SI 只新增协议需要的状态表达，不改变原有测量命令的核心执行路径。
- CPU2 与 CPU3 共享结构变化同步升级 `DEVICE_PROTOCOL_VERSION`，避免新旧固件误配。

## 4. SI协议要求提取

从 `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000协议.docx` 提取到的关键要求如下：

| 项目 | 协议要求 | 本次适配 |
| --- | --- | --- |
| 物理链路 | RS-485 | 复用 CPU3 外部串口协议框架 |
| 串口格式 | 9600 bps，8 data bits，Odd parity，1 stop bit | 修正 CPU3 串口字长处理；选择 SI 后保存/加载均强制端口为 9600 8O1 |
| 通信角色 | PLC 主站，仪表从站 | CPU3 实现 Modbus RTU slave |
| 站号 | Tank ID，且每台唯一 | 使用 `SlaveAddress`，有效范围 1..247 |
| 请求间隔 | 不小于 1s | 从站无主动限流，按请求响应 |
| 功能码 | FC01、FC02、FC03、FC04、FC05、FC06 | 已实现 |
| Coil 地址 | 00001-00016 | 运行模式、停止和手动上下行命令 |
| Discrete Input 地址 | 10001-10032 | 状态、报警和协议辅助状态 |
| Holding Register 地址 | 40001-40023 | 当前 `40001~40003` 桥接CPU2 SI profile参数；`40004~40009`为CPU3本地原始兼容槽；`40010~40023`为CPU3本机持久化参数 |
| Input Register 地址 | 30001-30620 | 位置、温度、密度、液位、时间和剖面点 |

协议文档中密度缩放写为 `0.01 kg/m3`，示例 `450.00 -> 45000`。协议 13 前适配层曾将内部 `kg/m3 x10` 转为协议 `kg/m3 x100` 输出；协议 13 后 CPU2/CPU3 内部密度 raw 已统一为 `kg/m3 x100`，SI 层不再额外乘 10。由于 16 位寄存器最高只能表示 `655.35 kg/m3`，超过范围时钳位到 `65535`，该限制仍需 PLC 侧确认。

## 5. CPU2改动明细

### 5.1 共享协议结构

涉及文件：

- `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`
- `LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h`
- `LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c`
- `LTD_MAIN_CPU2/Application/Inc/app_version.h`

改动内容：

- 在 `MeasurementResult` 的既有子结构中新增 SI 所需状态字段：设备状态、液位测量、实高测量和密度分布各自承载对应状态。
- 新增跨 CPU 协议寄存器地址，分别归入 `REG_DEVICE_STATUS_*`、`REG_OIL_MEASUREMENT_*`、`REG_HEIGHT_MEASUREMENT_*` 和 `REG_DENSITY_DIST_*` 段。
- CPU2 内部 Modbus 数据分析逻辑新增协议辅助状态读写。
- `DEVICE_PROTOCOL_VERSION` 升级为 `4u`。
- CPU2 固件版本升级到 `V1.8.0.0`。

这些融合后的字段用于承载外部协议转换需要的通用补充状态，包括底部基准有效、探头处于液面、液位稳定、剖面完成锁存、剖面完成计数、过程联锁、手动期间报警/液位更新抑制，以及温度/密度偏差报警等。CPU2 不保存 SI 地址、线圈或缩放语义，这些都由 CPU3 转换层处理。

### 5.2 测量流程状态维护

涉及文件：

- `LTD_MAIN_CPU2/Application/Src/measure.c`
- `LTD_MAIN_CPU2/Application/Src/measure_density.c`
- `LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c`

改动内容：

- `MeasureStart()` 启动新测量时清理瞬态协议状态，避免上一轮测量残留影响 PLC 读数。
- `CMD_MeasureBottom()` 在底部基准成功或接受回退结果后置位 `bottom_reference_valid`。
- 液位搜索流程维护 `probe_at_liquid_level` 和 `liquid_stable`。
- 手动/维护移动期间维护 `manual_alarm_inhibit` 和 `manual_level_update_inhibit`，供 PLC 判断手动运行期间的报警和液位更新语义。
- 分布、国标、米间隔等密度剖面流程启动时清理剖面状态，成功结束时锁存 `profile_complete_latched` 并递增 `profile_complete_counter`。
- 保持原有错误恢复、测量取消、状态切换和电机停止流程的行为边界不变。

### 5.3 维护模式命令循环调整

`CMD_EnterMaintenanceMode()` 从宏式命令循环调整为显式 `switch` 命令循环。这样可以在所有返回路径上更明确地清理 SI 手动抑制标志，同时保持原有维护模式命令语义。

## 6. CPU3改动明细

### 6.1 SI协议入口

涉及文件：

- `LTD_DISPLAY_CPU3/Application/app_main.c`
- `LTD_DISPLAY_CPU3/Application/system_param/com_port_config.h`
- `LTD_DISPLAY_CPU3/Application/display/display_tankopera.c`
- `LTD_DISPLAY_CPU3/Application/system_param/system_parameter.c`
- `LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c`

改动内容：

- 新增 `COM_PROTO_SI = 5`。
- CPU3 协议菜单新增 `SI协议` / `SI`。
- COM1/COM2/COM3 协议参数范围上限从 `4` 扩展到 `5`。
- 新增 `proto_si_process()`，在 `app_main.c` 的协议分发中接入 SI，并统一调用 `si_modbus_process_for_dispatch()`。
- SI 异常响应帧只要生成 `tx_len > 0` 即视为已处理，确保 PLC 可以收到标准 Modbus 异常帧。
- 增加协议参数归一化逻辑，写入协议参数时限定在 `COM_PROTO_DSM..COM_PROTO_SI` 范围内。

### 6.2 串口8O1适配

涉及文件：

- `LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c`
- `LTD_DISPLAY_CPU3/Communication/common/com_manager.c`
- `LTD_DISPLAY_CPU3/Communication/common/com_manager.h`

改动内容：

- CPU3 串口配置中，当数据位为 8 且校验位不是 none 时，HAL UART `WordLength` 需要使用 9B 才能形成实际 8 data bits + parity。
- 原逻辑只在 `databits == 9` 时使用 9B，无法正确覆盖 SI协议要求的 8O1。
- 本次同步修正显示侧通信参数应用路径和 common `com_manager` 兼容路径；两处 SI 分发入口均调用同一个转换层适配函数，避免异常响应语义漂移。

### 6.3 CPU3时钟模块

新增文件：

- `LTD_DISPLAY_CPU3/Application/system_param/cpu3_clock.c`
- `LTD_DISPLAY_CPU3/Application/system_param/cpu3_clock.h`

接入文件：

- `LTD_DISPLAY_CPU3/Application/app_main.c`

实现内容：

- CPU3 启动时调用 `Cpu3Clock_Init()`。
- 使用 CMSIS RTC 寄存器直接初始化和读写时间，避免依赖当前工程没有启用的 HAL RTC 模块。
- 当前实现优先使用 LSE，启动失败或旧板缺少 LSE 时回退 LSI。
- 使用 `RTC->BKP0R` 作为初始化标记。
- RTC 未初始化时写入默认时间 `2026-01-01 00:00:00`。
- 提供 `Cpu3Clock_GetDateTime()` 和 `Cpu3Clock_SetDateTime()`，供SI当前时间读取；协议18在Point0新周期事件到达时使用同一RTC接口锁存Profile时间。

注意事项：

- 回退 LSI 时长期计时仍会有漂移；状态和联调需区分当前 LSE/LSI 时钟源。
- CPU3 已提供屏幕 RTC 设置入口；SI 当前未定义 PLC 写时间寄存器。

### 6.4 CPU3共享协议结构

涉及文件：

- `LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h`
- `LTD_DISPLAY_CPU3/Application/system_param/stateformodbus.h`
- `LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/dataanalysis_modbus.c`
- `LTD_DISPLAY_CPU3/Application/app_version.h`

改动内容：

- 与 CPU2 同步新增 `SI 状态字段`。
- 与 CPU2 同步新增 `REG_DEVICE_STATUS_*` / `REG_OIL_MEASUREMENT_*` / `REG_HEIGHT_MEASUREMENT_*` / `REG_DENSITY_DIST_*` 寄存器映射。
- CPU3 从 CPU2 内部 Modbus 同步协议辅助状态。
- `DEVICE_PROTOCOL_VERSION` 升级为 `4u`。
- CPU3 固件版本升级到 `V1.6.0.0`。

## 7. SI Modbus从站实现

新增文件：

- `LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.c`
- `LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.h`

构建接入：

- `LTD_DISPLAY_CPU3/CMakeLists.txt`

### 7.1 通用处理

- 支持 Modbus RTU CRC 校验。
- 使用 `SlaveCheckCRC()` 校验请求。
- 响应异常码遵循 Modbus 标准：
  - `0x01`：非法功能码
  - `0x02`：非法地址
  - `0x03`：非法数据值
- 从站地址优先使用全局 `SlaveAddress`，范围为 `1..247`；无效时使用模块内默认地址。
- 所有 SI 地址按协议文档的 1-based 地址转换为 Modbus PDU 中的 0-based 地址处理。

### 7.2 FC01 / FC05 Coil

协议范围：`00001-00016`

实现方式：

- FC01 从 CPU2 的 `device_state`、`motor_state` 和 SI 影子状态生成线圈状态。
- FC05 支持单线圈写入，并桥接到 CPU2 命令；`00005~00008` 和 `00016` 保留线圈写入返回 Illegal Data Address。`FC05=ON` 在 CPU2 不可用或 ACK 失败时返回设备忙 `0x06`，动作影子只在 ACK 成功后提交。

主要命令映射：

| SI Coil | 含义 | CPU2命令 |
| --- | --- | --- |
| `00001` | Manual | `CMD_MAINTENANCE_MODE` |
| `00002` | Calibrate | `CMD_CALIBRATE_ZERO` |
| `00003` | Auto | `CMD_FIND_OIL` |
| `00004` | Profile | `CMD_SI_PROFILE` |
| `00009` | Stop | `CMD_MAINTENANCE_MODE` |
| `00010/00011/00012` | Up Slow/Medium/Fast | `CMD_FORCE_MOVE_UP` |
| `00013/00014/00015` | Down Slow/Medium/Fast | `CMD_FORCE_MOVE_DOWN` |

处理细节：

- `00001-00004` 模式线圈在影子状态中互斥。
- `00009-00015` 停止/方向线圈在影子状态中互斥。
- 写入 `OFF` 时只清除对应影子状态，不主动派生命令。
- 写入 `ON` 时只有 CPU2 ACK 成功才提交对应影子；状态/参数/当前连接协议快照未完成、共享协议不兼容、参数刷新中、通信故障锁存或本次请求失败均返回 `0x06`。
- 当前 CPU2 只有统一的强制上/下行命令，SI 的 slow/medium/fast 速度档位暂映射为同一方向命令。

### 7.3 FC02 Discrete Input

协议范围：`10001-10032`

关键映射：

| SI 地址 | 含义 | 数据来源 |
| --- | --- | --- |
| `10001` | Bottom Reference | `既有测量子结构状态字段.bottom_reference_valid` |
| `10002/10003` | Lower/Upper Level Sensor | CPU3 按液位跟随稳定状态合成，不接入真实双液位传感器硬件 |
| `10004` | Interlock | CPU2 错误码或 `profile_blocked_by_process` |
| `10005` | Profile Complete | `profile_complete_latched` 且 `profile_source == PROFILE_SOURCE_SI` |
| `10006` | Unit Is Metric | 固定为 1 |
| `10009` | Reel Alarm | CPU2 错误码非 `NO_ERROR` |
| `10010` | Probe Un-calibrated | 兼容固定返回 `0`，不能作为物理位置可信证明 |
| `10013` | Probe At Liquid Level | `oil_measurement.probe_at_liquid_level` |
| `10025` | Temperature Deviation Alarm | `profile_temp_deviation_alarm` |
| `10026` | Density Deviation Alarm | `profile_density_deviation_alarm` |

历史记录：早期版本在没有可靠本机数据源时 Lower/Upper Level Sensor 保持为 0，并把报警阈值 0 视为未启用。当前实现已由 2026-07-01 后续优化覆盖：Lower/Upper 由 CPU3 按 SI 显示语义合成，报警阈值 `0` 是有效值，不能再作为禁用条件。

### 7.4 FC03 / FC06 Holding Register

协议范围：`40001-40023`

关键映射：

| SI 地址 | 含义 | 实现 |
| --- | --- | --- |
| `40001` | Profile First Point | 写入 CPU2 `si_profile_first_point`，协议 mm 转 CPU2 0.1mm；CPU2 ACK 后提交影子 |
| `40002` | Profile Increment | 写入 CPU2 `si_profile_increment`，协议 mm 转 CPU2 0.1mm；CPU2 ACK 后提交影子 |
| `40003` | Profile Dwell Time | 写入 CPU2 `si_profile_dwell_time`，单位秒；CPU2 ACK 后提交影子 |
| `40004-40009` | 2026-05阶段仍按保留处理；协议18改为原始兼容槽 | 当前默认 `0、50、1000、0、5、1`，支持FC03/FC06和FRAM V7持久化，不参与控制 |
| `40010-40023` | 自动 profile 参数和报警设定 | CPU3 本机参数，FRAM 持久化，不依赖 CPU2 在线状态 |

处理细节：

- `40011` 仅接受 `0/1`。
- `40012` 仅接受 `0..23`。
- `40013` 仅接受 `0..59`。
- `40014/40015` 不直接写入瓦锡兰密度参数，避免 SI协议写入影响既有瓦锡兰协议行为。
- 历史行为：协议18以前 `40004~40009` 为文档保留区，FC06写入返回Illegal Data Address、读取为0；协议18已由第17节所述原始兼容槽取代。
- FC06 写响应使用显式 echo 构造，不依赖对请求缓冲区的反向写入。
- `40001~40003` 在 CPU2 不可用或 ACK 失败时返回设备忙 `0x06` 并保留原影子；协议18的 `40004~40023` 合法写入不受CPU2通信门禁影响，但FRAM事务失败同样返回 `0x06`并恢复整份写前运行态。

### 7.5 FC04 Input Register

协议范围：`30001-30620`

关键映射：

| SI 地址 | 含义 | 实现 |
| --- | --- | --- |
| `30001` | Current Probe Position | CPU2 0.1mm 转 mm，负值钳位为 0 |
| `30002` | Current Temperature | 内部温度原始值换算为有符号 0.01摄氏度 |
| `30003` | Current Density | 协议 13 后内部与 SI协议均按 `kg/m3 x100`，超过 16 位时钳位 |
| `30004` | Liquid Level | CPU2 0.1mm 转 mm，无效值输出 0 |
| `30006` | Number Of Points | 协议14阶段仅在完成后输出；协议18改为Point0前保留旧N、Point0后按有效液体点递增、最终发布后为完整点数 |
| `30007-30010` | Profile Timestamp | 协议14阶段在命令触发时锁存；协议18改为Point0新周期事件时锁存，无法重建时全0 |
| `30011-30013` | Current Time | 每次读取 CPU3 RTC 当前时间 |
| `30021+3n` | Profile Point | 剖面点位置、温度、密度 |

处理细节：

- 历史行为：协议14阶段在命令触发入口锁存时间；协议18由CPU3观察Point0周期事件后锁存。
- 如果 RTC 暂时读取失败，本次时间戳保持无效；后续触发重新尝试，不能把旧完成时间冒充本次开始时间。
- 历史行为：协议14阶段未完成时 `Number Of Points` 和点阵为0；协议18在Point0前保留旧结果、Point0后仅递增N并隐藏点阵，最终代际复核后一次开放新点阵。

## 8. 复查后修正和优化

本次在执行后又做了问题核对和优化，已纳入当前剩余改动：

| 优化点 | 说明 |
| --- | --- |
| FC05 线圈互斥 | 模式线圈和方向线圈在影子状态中互斥，减少 PLC 读到多个模式同时有效的风险 |
| FC05 保留地址 | 保留线圈写入返回 Illegal Data Address，避免 PLC 误写后收到成功 echo |
| FC06 参数校验 | 对自动剖面开关、小时、分钟等协议参数做范围校验，非法值返回异常 |
| FC06 保留地址 | 2026-05历史基线中 `40004~40009` 写入返回Illegal Data Address、读取为0；协议18已改为CPU3本地兼容槽 |
| 密度协议缩放 | `30003` 和剖面密度点按协议示例输出 `0.01 kg/m3`，超出 16 位时钳位 |
| Interval Timer/报警位 | `10012` 和阈值报警位由 `40011`、`40014~40021` 影子寄存器合成 |
| 写响应构造 | 移除依赖 `pdu[-1]` 的响应构造方式，改为显式 echo，降低缓冲区边界风险 |
| 剖面时间戳 | RTC 暂时不可读时不立即标记为已锁存，后续读取继续重试 |
| 剖面点有效范围 | 只在剖面完成锁存后输出有效点数量内的数据，超出部分清零，避免旧数据泄漏 |
| 无效温度 | 历史版本曾输出 0；当前实现统一输出 `-200.00°C`，寄存器补码 `0xB1E0` |
| 串口字长 | 修正 8O1 场景下 HAL UART `WordLength` 配置 |
| 兼容分发枚举 | 将旧 `com_manager` 兼容路径的协议/校验枚举改为 `COM_MANAGER_*` 专用前缀，避免与 CPU3 本机参数 `ComProtocolType` 混用 |
| 待确认与后续清单 | 将需要现场或 PLC 确认的事项单独整理到 `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议待确认与后续清单.md`，当前分支只记录不处理 |
| 验证基线 | 新增 SI Modbus 参考帧检查和 CPU2/CPU3 共享协议契约检查，不改变固件行为 |
| PLC 联调表 | 新增 `docs/01_协议与寄存器/SI协议适配/04_联调测试/SI协议_PLC联调检查表.md`，明确当前可验收项和未确认需求暂停项 |
| 中文注释 | 对 SI协议转换边界、CPU2 通用状态维护、CPU3 RTC 和工具脚本补充中文注释，便于后续维护 |
| 文档一致性 | 同步更新执行计划、映射表、协议变更记录、总文档索引、联调表和 CHANGELOG |

## 9. 文档和版本文件

新增或更新的文档：

| 文件 | 内容 |
| --- | --- |
| `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议需求与实施总览.md` | SI协议官方资料要点、当前实现基线、进一步兼容需求、实施方案和验证计划 |
| `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议待确认与后续清单.md` | 已确认项、实现前仍需确认的问题、P0/P1/P2 后续清单和建议回复格式 |
| `docs/01_协议与寄存器/SI协议适配/02_协议映射/SI协议兼容映射表.md` | SI协议地址与当前系统数据/命令映射 |
| `docs/01_协议与寄存器/SI协议适配/03_改动记录/CPU2暂存区改动整理.md` | 当前暂存区中 CPU2 侧 7 个文件的逐文件改动、协议影响、运行期语义、回归重点和风险说明 |
| `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000_PLC协议及软件需求说明.docx` | 前期分支带入的 PLC 协议/需求说明文档 |
| `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方操作手册.pdf` | 官方操作手册，作为测量流程和状态语义依据 |
| `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方资料包_含Modbus规范.pdf` | 官方资料包，包含 DCS Modbus Interface Specification |
| `docs/01_协议与寄存器/SI协议适配/04_联调测试/SI协议_PLC联调检查表.md` | PLC/主机侧联调检查表、参考帧和当前暂不验收项 |
| `docs/01_协议与寄存器/SI协议适配/README.md` | SI协议专项文档索引和维护规则 |
| `docs/README.md` | 根文档索引重新纳入 SI 专项资料入口 |
| `docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md` | 记录共享协议版本 `4` 的 CPU2/CPU3 兼容关系 |
| `CHANGELOG.md` | 记录 CPU2 `V1.8.0.0` 和 CPU3 `V1.6.0.0` 的适配内容与验证 |

当前暂存区只包含 `03_改动记录` 下两份 Markdown 说明文件，未包含对应 PDF 导出版；如后续交付需要 PDF，可在最终审查通过后再单独生成并暂存。

新增或更新的验证工具：

| 文件 | 内容 |
| --- | --- |
| `tools/check_si_modbus_frames.py` | 检查 SI 功能码、地址表常量和主机侧参考帧，覆盖 FC01/02/03/04/05/06 与典型异常响应 |
| `tools/check_si_protocol_contract.py` | 检查 CPU2/CPU3 `SI 状态字段` 字段顺序、`REG_DEVICE_STATUS_*` / `REG_OIL_MEASUREMENT_*` / `REG_HEIGHT_MEASUREMENT_*` / `REG_DENSITY_DIST_*` 寄存器链和 `DEVICE_PROTOCOL_VERSION` 一致性 |

版本相关改动：

- `LTD_MAIN_CPU2/Application/Inc/app_version.h`
- `LTD_DISPLAY_CPU3/Application/app_version.h`
- CPU2/CPU3 共享协议版本同步升级到 `4u`。
- CPU2/CPU3 固件版本均按新增协议能力升级 minor 版本。

## 10. 已执行验证

本次改动已执行以下验证：

| 验证项 | 结果 |
| --- | --- |
| `git ls-files -u` | 无未解决冲突 |
| 冲突标记搜索 | 未发现 `<<<<<<<` / `>>>>>>>` |
| CPU2 配置 | 通过 |
| CPU2 构建 | 通过，生成 `LTD_MAIN_CPU2_V1.8.0.0.hex` |
| CPU3 构建 | 通过，生成 `LTD_DISPLAY_CPU3_V1.6.0.0.hex` |
| CPU3 字库检查 | `py LTD_DISPLAY_CPU3\font_check.py` 通过 |
| 版本检查 | `py tools\check_version_bumped.py` 通过 |
| SI 参考帧检查 | `py tools\check_si_modbus_frames.py --dump` 通过 |
| SI 共享协议契约检查 | `py tools\check_si_protocol_contract.py` 通过 |
| `git diff --check` | 通过 |
| 本轮涉及文件尾随空白检查 | 通过 |
| CPU2 GBK 解码检查 | 修改过的 CPU2 源文件未发现替换字符 |
| CPU3/工具 UTF-8 解码检查 | 修改过的 CPU3 源文件和工具脚本未发现替换字符 |
| 协议版本搜索 | CPU2/CPU3 均为 `DEVICE_PROTOCOL_VERSION 4u` |

已运行的关键命令：

```powershell
cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/CUBE/cmake/toolchain-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build build\LTD_MAIN_CPU2
cmake --build build\LTD_DISPLAY_CPU3
py LTD_DISPLAY_CPU3\font_check.py
py tools\check_si_modbus_frames.py
py tools\check_si_protocol_contract.py
git diff --check
py tools\check_version_bumped.py
```

### 10.1 2026-05-21审查复核

本次更新本文档时，没有重新运行 CPU2/CPU3 CMake 构建；构建结果仍以上表实现阶段记录为准。为确认当前暂存快照和说明一致，已复跑以下轻量检查：

| 复核项 | 结果 |
| --- | --- |
| `git diff --cached --stat` | 当前暂存区为 46 个文件，5118 行新增，90 行删除 |
| `git diff --cached --check` | 通过 |
| `py tools\check_si_modbus_frames.py` | 通过 |
| `py tools\check_si_protocol_contract.py` | 通过 |

## 11. 第一版限制和后续状态（历史）

下表原用于记录 2026-05 第一版限制。已在协议 14 或后续工作中解决的项目直接标注当前结果；仍未解决的项目继续由 `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议待确认与后续清单.md` 维护。

| 项目 | 当前处理 | 后续建议 |
| --- | --- | --- |
| 从站地址/Tank ID | 优先使用现有 `SlaveAddress`，无效时使用 SI 默认地址 | 确认 PLC 的 Tank ID 是否等同现有从站地址，必要时新增 SI 专用地址参数 |
| Profile 参数语义 | 已改为 CPU2 独立 `si_profile_*` 参数，不再复用普通分布参数；协议18最终Point0输出本轮实际采用底部参考 | 继续按绝对首点、步距、秒单位和实际Point0位置口径联调 |
| 密度缩放 | 已按协议示例输出 `0.01 kg/m3`，超过 16 位时钳位 | 与 PLC 确认实际密度范围是否会超过 `655.35 kg/m3`，必要时另定兼容倍率 |
| 手动速度档位 | slow/medium/fast 均映射为 CPU2 统一强制上/下行命令 | 如现场需要速度差异，CPU2 需提供可指定速度的命令或参数 |
| 报警设定寄存器 | `40010~40023` 已作为 CPU3 SI 本机参数落盘保存 | 继续验证掉电恢复和报警合成，不映射到 Wartsila 或继电器报警参数 |
| 报警语义细分 | Reel Alarm、Probe Un-calibrated 和 profile 偏差报警为第一版简化/通用字段映射 | 确认 PLC 是否需要更细的卷尺、电机、校准和 profile 偏差算法 |
| Lower/Upper Level Sensor | 当前按液位跟随稳定状态合成 | 真实双液位传感器硬件仍未接入，保持受控偏差 |
| Interval Timer | `10012` 当前由 `40011` 影子使能合成 | 确认现场是否还需要独立“计时器正在运行/到时”状态 |
| 液位/profile 刷新 | 协议18在Point0前保留旧SI快照，Point0后活动点阵为0，最终同代开放；已发布SI快照与后续非SI分布测量隔离 | 验证主机按新周期沿判断完成，并覆盖普通/国标/每米/间隔/Wärtsilä测量后的旧SI快照保持 |
| CPU3 RTC 精度 | 优先 LSE，失败回退 LSI，默认时间为 `2026-01-01 00:00:00` | LSI 回退场景仍需评估长期精度；PLC 写时接口未定义 |
| 时间设置入口 | 已提供 CPU3 屏幕 RTC 设置入口 | 如 PLC 要求远程校时，需要另行定义 Holding Register 或命令入口 |
| 现场协议节奏 | 从站按请求响应，不主动限制请求频率 | PLC 侧需遵守请求间隔不小于 1s |
| 协议诊断 | 当前只按 Modbus 响应返回异常，不保存统计 | 后续增加 CRC 错误、地址不匹配、非法地址/值等 CPU3 本地诊断计数 |
| 主机侧帧测试 | 已新增 `tools/check_si_modbus_frames.py` 作为参考帧检查入口 | 后续在需求确认后继续扩展真实状态快照、缩放边界和报警合成用例 |
| SI 模块结构 | 协议从站逻辑集中在一个实现文件 | 后续可拆分地址表、缩放、状态快照、功能码处理和诊断统计 |
| 共享协议回归检查 | 已新增 `tools/check_si_protocol_contract.py` | 后续协议变更时纳入提交前检查 |
| 兼容分发路径 | 主路径使用 `app_main.c`，旧 `com_manager` 已加专用枚举前缀 | 后续确认保留、删除或收敛为主分发薄封装 |
| 官方 profile 时序 | 协议18已按Point0新周期锁存时间；Point0前保留旧结果，活动期只递增N，最终同代开放点阵 | CPU3冷启动/重连无法重建Point0事件时四项必须全0，不得用重启时刻代替 |
| 官方点数口径 | 当前按 DCS 地址表输出 200 个点 | 若 PLC 要求 250+ 或更多点，需要定义扩展地址或分页机制 |

## 12. 建议后续执行顺序

1. 先和 PLC/现场确认 P0 项：Tank ID、`40001~40003` profile 参数语义、密度缩放范围。
2. 再确认 P1 项：报警启用语义、Manual 期间的自动剖面调度与报警抑制、Cal 完整回 Auto、上下液位传感器来源和官方默认值/完整值域。
3. 主机侧参考帧检查和共享协议静态检查已经具备，需求口径确认后先扩展测试快照，再实现对应代码。
4. 官方资料复核后，把上下液位传感器、profile 首点时间戳、profile 第 0 点罐底语义和 Manual 抑制作为下一轮 P0/P1 优先项。
5. 最后做结构性优化：SI协议模块拆分、协议诊断计数、`com_manager` 兼容路径取舍。

## 13. 关键文件清单

### CPU2

```text
LTD_MAIN_CPU2/Application/Inc/app_version.h
LTD_MAIN_CPU2/Application/Src/measure.c
LTD_MAIN_CPU2/Application/Src/measure_density.c
LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c
LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c
LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h
LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h
```

### CPU3

```text
LTD_DISPLAY_CPU3/Application/app_main.c
LTD_DISPLAY_CPU3/Application/app_version.h
LTD_DISPLAY_CPU3/Application/display/display_tankopera.c
LTD_DISPLAY_CPU3/Application/system_param/com_port_config.h
LTD_DISPLAY_CPU3/Application/system_param/cpu3_clock.c
LTD_DISPLAY_CPU3/Application/system_param/cpu3_clock.h
LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c
LTD_DISPLAY_CPU3/Application/system_param/stateformodbus.h
LTD_DISPLAY_CPU3/Application/system_param/system_parameter.c
LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h
LTD_DISPLAY_CPU3/CMakeLists.txt
LTD_DISPLAY_CPU3/Communication/common/com_manager.c
LTD_DISPLAY_CPU3/Communication/common/com_manager.h
LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.c
LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.h
LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/dataanalysis_modbus.c
```

### 文档

```text
CHANGELOG.md
README.md
docs/README.md
docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md
docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议需求与实施总览.md
docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议待确认与后续清单.md
docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000_PLC协议及软件需求说明.docx
docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000协议.docx
docs/01_协议与寄存器/SI协议适配/02_协议映射/SI协议兼容映射表.md
docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方操作手册.pdf
docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方资料包_含Modbus规范.pdf
docs/01_协议与寄存器/SI协议适配/README.md
docs/01_协议与寄存器/SI协议适配/03_改动记录/SI协议适配改动整理.md
docs/01_协议与寄存器/SI协议适配/03_改动记录/CPU2暂存区改动整理.md
docs/01_协议与寄存器/SI协议适配/04_联调测试/SI协议_PLC联调检查表.md
```

### 工具

```text
tools/check_si_modbus_frames.py
tools/check_si_protocol_contract.py
```

## 14. 总结

本次 SI协议适配已经完成从协议需求拆解、前期分支合并、CPU2 状态补齐、CPU3 外部从站协议实现、CPU3 时钟引入、协议/版本文档同步，到构建、静态检查和参考帧验证的完整闭环。`40010~40023` 已作为 CPU3 本机参数持久化，不再属于待确认项。当前仍需现场或 PLC 侧进一步确认和验证的重点是 SI 官方默认值及完整非法值范围、报警启用方式和默认阈值、Manual 期间的报警与自动 Profile 抑制、Cal 完整回 Auto，以及 Probe Un-calibrated 与探底失败口径；真实双液位传感器、Interlock/Reel Alarm 细分和首点时间戳继续按受控偏差评估。工程侧后续建议扩展真实状态快照测试、诊断计数和 SI 模块结构拆分。

## 15. 2026-07-01 进一步兼容实施更新

本节记录协议版本 14 对 SI协议进一步兼容需求的落地结果。历史章节保留第一版适配过程，本节以当前源码为准。

### 15.1 共享协议和 CPU2 profile 流程

| 项目 | 当前结果 |
| --- | --- |
| 共享协议版本 | `DEVICE_PROTOCOL_VERSION = 14` |
| 新命令 | `CMD_SI_PROFILE = 20` |
| CPU2 profile 参数 | `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval` |
| CPU2 参数版本 | 复用原预留槽，`DEVICE_PARAM_VERSION` 保持 `3` |
| 默认值 | 首点 100mm、步距 1000mm、停留 10s、探底频次 1 |
| 测量入口 | `CMD_SiProfile()` |

CPU2 SI profile 流程已从普通分布测量解耦。执行时直接读取 CPU2 本机 SI profile 参数，按探底频次决定是否真实探底；探底频次按进入 CPU2 SI profile 入口的触发次数计数，不按 profile 成功完成次数计数。不探底时仍运行到保存的底部位置采集 Point0。探底失败时优先使用旧底部位置继续，没有旧底部位置则用当前位置作为底部采样运动目标，但不刷新底部参考、不标记底部可信、不清首次探底状态。SI profile 不在测量前单独执行 `SearchOilLevel()`；协议14阶段曾把Point0位置固定写0，协议18已改为最终输出本轮实际采用的底部参考。后续候选停点从 `first_point` 绝对位置开始，按 `increment` 向上生成，即 Point1=`40001`、Point2=`40001+40002`、PointN=`40001+(N-1)*40002`。即使实际底部位置高于 `40001`，也不跳过 Point1。最多 200 点；逐点采样时如果当前候选点被判定为液面以上点，则停止本轮 profile且不把该点写入有效点阵，达到罐高或点数上限也会停止。

### 15.2 CPU3 SI 参数、菜单和调度

| 项目 | 当前结果 |
| --- | --- |
| `40001~40003` | 桥接 CPU2 SI profile 参数，不再写普通分布参数 |
| `40010~40013` | CPU3 本机 SI 自动 profile 参数 |
| `40014~40023` | CPU3 本机 SI 报警限值参数 |
| CPU3 本机参数版本 | `0x0005 -> 0x0006`，支持 V3/V4/V5 迁移 |
| 菜单入口 | `参数配置 -> 测量参数 -> SI参数` |
| 子菜单 | `Profile参数`、`自动Profile`、`报警限值` |
| 自动调度 | `si_modbus_periodic_task()` 基于 `40010~40013` 和 CPU3 RTC 周期触发 |

写入边界：`FC05=ON` 和 `40001~40003` 依赖 CPU2 ACK，失败返回设备忙 `0x06`，相关影子只在 ACK 成功后提交；协议18的 `40004~40023` 是CPU3本机持久化参数，不依赖CPU2在线状态，FRAM写后校验失败时整份回滚并返回 `0x06`。

协议14阶段CPU3写 `00004 Profile ON` 时先锁存profile开始时间再下发 `CMD_SI_PROFILE`；协议18已经把对外时间改到Point0新周期事件。自动 profile按起始时分和interval周期跨天触发；到点下发失败时按5 s门限节流重试，不记录本分钟成功状态，只有下发成功后才在同一分钟去重。到点后仍按已确认口径直接覆盖当前命令并执行SI profile。

### 15.3 状态、报警和时间戳

| 地址 | 当前结果 |
| --- | --- |
| `10001` | 保持现有 `bottom_reference_valid` 映射逻辑 |
| `10002` | Lower Level Sensor：液位跟随稳定时显示液体，其它状态显示液体 |
| `10003` | Upper Level Sensor：液位跟随稳定时显示空气，其它状态显示液体 |
| `10010` | 固定 `0`，SI协议对外显示探针可信 |
| `10017~10024` | CPU3 按当前值和 `40014~40021` 合成 |
| `10025~10026` | CPU3 按 profile 相邻点温差/密差和 `40022/40023` 合成 |
| `10029~10032` | CPU3 按 profile 点阵和上下限阈值合成 |
| `30007~30010` | 协议14阶段在命令触发时锁存；协议18按Point0新周期锁存，无法重建时全0 |

### 15.4 文档和脚本

已同步更新：

- `tools/check_si_modbus_frames.py`
- `tools/check_si_protocol_contract.py`
- `docs/00_构建与版本/版本改动与测试/2026-07-01_CPU2_V1.21.0.0_CPU3_V1.19.0.0_SI协议独立Profile兼容_改动与测试方案.md`
- `docs/00_程序流程导航/CPU2/06_密度与单点测量.html`
- `docs/00_程序流程导航/CPU3/05_Wartsila与SI协议适配.html`
- `docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md`
- `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议需求与实施总览.md`
- `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议进一步兼容改动方案.html`
- `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议待确认与后续清单.md`
- `docs/01_协议与寄存器/SI协议适配/02_协议映射/SI协议兼容映射表.md`
- `docs/01_协议与寄存器/SI协议适配/04_联调测试/SI协议_PLC联调检查表.md`

### 15.5 当前仍需复核

- SI协议官方手册逐项默认值仍需现场或资料复核；当前代码采用保守默认值。
- `10009 Reel Alarm` 仍按设备错误简化合成，若 PLC 需要卷尺/电机/校准细分，需要下一轮补充。
- `10002/10003` 当前按液位跟随稳定状态合成，不接入真实双液位传感器硬件。
- 超过 200 点 profile 扩展、独立 Tank ID、三档真实手动速度仍不在本轮实现范围。

## 16. 2026-07-10 CPU2 通信门禁和失败反馈更新

CPU3 `V1.20.0.0` 对 SI 写入口补齐以下行为，`DEVICE_PROTOCOL_VERSION` 保持 `14`：

- `FC05=ON` 在状态/参数/当前连接协议快照未完成、共享协议不兼容、参数刷新中、CPU2 通信故障锁存或本次命令 ACK 失败时返回 Slave Device Busy `0x06`，不提交模式或动作线圈影子。
- `FC05=OFF` 只清对应本地影子，不派生 CPU2 命令，合法请求正常回显。
- `FC06 40001~40003` 只有 CPU2 参数 ACK 成功后才更新 `g_deviceParams` 和 SI 保持寄存器影子；失败返回 `0x06` 并保留原值。成功写和已发起后失败都会让内部参数快照立即失效并强制补读四组；补读完成前，与CPU2参数段相交的FC03、后续CPU2参数写和普通命令保持关闭，`40004~40023`本机参数不受影响。
- `FC06 40004~40023` 只写CPU3本机参数和FRAM，合法写入不依赖CPU2在线状态；FRAM写入或写后读回失败返回 `0x06`并恢复整份写前运行态。
- PLC 收到 `0x06` 后应等待 CPU2 状态、参数和当前连接协议快照完整恢复，重试原目标值并通过 FC01/FC03 回读确认。

本轮已执行 `py tools\check_si_modbus_frames.py` 和 `py tools\check_si_protocol_contract.py`，结果通过；真实 PLC 离线重试、延迟上线和串口干扰仍需硬件联调。

## 17. 2026-07-14 协议18进一步兼容落地

本节记录2026-07-14开发工作区落地协议18时的差异；这些SI行为已由当前协议34沿用。2026-07-13抓包章节仍按当时的CUBE行为保留，不倒改为“当时已兼容”。协议18随后已形成CPU2 V1.26.0.0 / CPU3 V1.24.0.0正式组合；当前协议34正式组合为CPU2 V1.40.0.0 / CPU3 V1.40.0.0，协议19至34的后续共享协议变化均不改变本节SI行为。

### 17.1 CPU2/CPU3共享生命周期

| 项目 | 已实现结果 |
| --- | --- |
| 共享协议 | `DEVICE_PROTOCOL_VERSION 17 -> 18`，CPU2/CPU3同步 |
| 新运行态 | 独立 `SiProfileRuntime`：`phase`、`cycle_counter`、`progress_points` |
| 地址边界 | 三个32位字段追加在AO运行态之后、`REG_ENG`之前，不移动既有点阵和前序寄存器 |
| 阶段 | `IDLE/PREPARING/MEASURING/RETURNING_LEVEL/COMPLETE/ABORTED/FAILED` |
| Point0事件 | 第一个有效样本提交时清候选Complete、进度置1、写阶段，最后递增周期计数 |
| 进度 | 每个有效液体点提交后递增；空气点、失败样本和重试不计数 |
| 最终候选 | 回液位成功、液位稳定且电机停止后复制完整候选，最后递增完成计数 |

CPU2在Point0前不清上一轮已发布结果。Point0有效样本建立本轮周期后，旧Complete、旧N和旧点阵失效；活动期只发布进度，不发布半成品点阵。Point0后显式取消或真实失败都会清本轮Complete、N和候选点阵且不恢复旧Complete；显式取消不置Interlock，真实失败继续走现有错误/Interlock出口。探底失败时优先沿用可信旧底；没有可信底部时使用当前探头位置作为本轮Point0参考继续，不按失败处理。

### 17.2 CPU3本地SI投影和最终发布

- CPU3先读取紧凑Profile头和协议18生命周期字段，再按内部帧长分块读取点阵，最后重读头部和生命周期字段。
- 只有周期、完成计数、最终N、`PROFILE_SOURCE_SI`和`COMPLETE`阶段前后一致，候选才进入本地SI投影；任一分块或代际复核失败时保持对外Complete=0，并按1秒节流从头重试。
- Point0前保留上一轮Complete、N、时间、点阵和Profile报警；Point0新周期到达时清旧发布结果并锁存CPU3 RTC。
- 活动期 `30006` 显示1～200的有效点进度，`30021~30620`全部为0。
- 回液位后同时满足AtLevel、Stable、电机停止、Interlock=0和候选代际复核通过，CPU3才一次发布Profile=0、Auto=1、Stop=1、Complete=1、AtLevel=1、最终N、完整点阵和Profile报警。
- CPU3首次上线或断线重连不会把当前时间冒充Point0时间；无法重建时当前周期 `30007~30010`保持0。首次上线时若CPU2已经处于同代COMPLETE，仍允许拉取并发布最终点阵，但时间保持0。
- 已发布SI快照存放在CPU3本地不可变投影中；后续普通分布、国标、每米、间隔或Wärtsilä测量不会清除或改写其Complete、N、时间、点阵和Profile报警。旧SI的COMPLETE/ABORTED/FAILED终态只有处于SI收尾上下文时才投影终态线圈，不能遮住当前非SI动作。

### 17.3 私有地址兼容和CPU3 FRAM V7

| 地址/存储 | 已实现结果 |
| --- | --- |
| `30014` | `(FC01 byte1 << 8) | FC01 byte0` |
| `30015` | `(FC02 byte1 << 8) | FC02 byte0` |
| `30016` | `(FC02 byte3 << 8) | FC02 byte2` |
| `30017~30020` | 保持0，不赋予业务语义 |
| `40004~40009` | CPU3本地原始 `uint16_t` 兼容槽；默认 `0、50、1000、0、5、1`；支持FC03批读和FC06单写 |
| 写失败 | `40004~40023`均通过事务式写入口；FRAM写入或写后读回失败时恢复整份写前运行态、尽力恢复旧镜像，并向FC06返回 `0x06` |
| CPU3参数存储 | `V6 / 0x0006 -> V7 / 0x0007` |
| V6迁移 | 使用显式V6旧结构和CRC校验，完整保留旧SI参数与合法三路串口参数，只补六项兼容默认值；V6迁移和V7重载不再按值猜测并改写合法Wartsila `4800 8N1/8N2` |

`40004~40009`仍没有厂家公开名称、单位和值域，因此只作为原始兼容槽保存和回读，不参与测量、报警、自动调度或其它控制。

### 17.4 验证状态和发布限制

已完成：

- `py tools\check_si_protocol_contract.py`：通过，覆盖协议18、阶段枚举、共享字段顺序、Point0/进度/取消失败/最终提交顺序、CPU3候选分块及V6→V7迁移契约。
- `py tools\check_si_modbus_frames.py`：通过，覆盖完整 `40001~40023`、六项FC06和 `30014~30020` 镜像参考帧。
- CPU2、CPU3 clean-first目标构建：通过。

尚待真实硬件验证：

- RS485站2上位机完整两轮联调和协议17/18混搭拦截。
- V6真实FRAM镜像迁移、`40004~40023`掉电保持及FRAM写入/写后读回失败回滚。
- Point0前后显式取消、采样失败、回液位失败、CPU3在各阶段重启、CPU2通信中断和分块重试。
- 最终1点、现场约30点、200点以及空气点/失败重试不计数边界。
- 普通分布、国标、每米、间隔和Wärtsilä测量与已发布SI快照隔离，以及旧SI终态不遮住当前非SI动作。

发布基线：协议18首个正式匹配组合为CPU2 `V1.26.0.0`、CPU3 `V1.24.0.0`；双端版本、CHANGELOG、协议变更记录和版本改动与测试资料已经同步。提交前只读审查确认的未修复风险统一见`../01_计划与需求/SI协议待确认与后续清单.md`第7节。
