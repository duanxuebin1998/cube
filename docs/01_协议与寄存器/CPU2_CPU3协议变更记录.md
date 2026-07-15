# CPU2/CPU3 协议变更记录

本文记录 CPU2 与 CPU3 之间共享寄存器、共享结构体和能力字段的协议版本。固件版本只表示各 CPU 自身软件版本，不能单独作为跨 CPU 兼容依据。

## 协议版本字段

- 字段位置：`HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION`
- 当前语义：CPU2/CPU3 共享协议版本
- 旧程序语义：保留字段，默认值为 `0`
- 当前开发程序语义：协议版本 `18`

该字段由原 `reserved1` 预留位正式替换而来，寄存器地址不移动，不新增存储字段。

## 版本记录

| 协议版本 | 首个 CPU2 版本 | 首个 CPU3 版本 | 密度分布最大点数 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 旧程序，或未写入协议版本字段 | 旧程序 | 100 | 原 `reserved1` 未定义协议语义；当前CPU3读到0时提示协议不兼容。 |
| 1 | V1.5.0.0 | V1.2.0.0 | 200 | 原 `reserved1` 正式替换为协议版本，密度分布测量、内部输入寄存器和 Wärtsilä 外部密度点扩展到 200 点。 |
| 2 | V1.6.0.0 | V1.4.0.0 | 200 | 新增 `CMD_CANCEL_MEASUREMENT = 16`，用于 CPU3 状态显示界面长按返回键取消当前测量并让 CPU2 进入待机。 |
| 3 | V1.7.0.0 | V1.5.0.0 | 200 | 原 `reserved23` 正式替换为探底修正罐高，用于罐底测量后编码器修正；瓦锡兰分布测后探底前先回固定点监测位置。 |
| 4 | V1.8.0.0 | V1.6.0.0 | 200 | 将 SI 所需补充状态融合进既有测量结构，并通过共享输入寄存器发布给 CPU3 外部协议转换层。 |
| 5 | V1.9.0.0 | V1.7.0.0 | 200 | 新增 `CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117`，用于 CPU3 菜单或共享命令通道触发 CPU2 执行无线滑环 RSSI 最近匹配；新增无线滑环匹配中/完成设备状态；输入寄存器末尾追加无线滑环匹配结果和从机 MAC 状态。 |
| 6 | V1.10.0.0 | V1.9.0.0 | 200 | 新增 `STATE_DEBUG_MODE = 0x0033`，用于 CPU2 串口调试指令执行期间通过 CPU3 显示“调试模式中”；原 `reserved2` 参数槽复用为故障自动恢复重跑上限；`empty_weight` 空载扭力按 `int32_t` 有符号 32 位解释，寄存器地址和后续字段不移动。 |
| 7 | V1.12.0.0 | V1.10.0.0 | 200 | 新增四路继电器报警输出配置和运行态共享区；CPU3 可显示、写入四路继电器报警输出配置，CPU2 执行 HH/H/L/LL、滞回、锁存清除和无效值策略。 |
| 8 | V1.14.0.0 | V1.13.0.0 | 200 | `liquidLevelMeasurementMethod` 液位测量方式语义扩展：保留 0/1 原相对频率和定频步进方案，补实 2=密度连续找液位，新增 4=连续相对频率、5=连续定频；CPU3 菜单和参数范围同步允许 0..5。 |
| 9 | V1.15.0.0 | V1.14.0.0 | 200 | 读取部件参数增加当前蓝牙连接 RSSI 快照；扩展 `WirelessPairingStatus` 并在输入寄存器尾部追加连接有效、RSSI 有效、RSSI 值、查询错误码和 RSSI 更新计数。 |
| 10 | V1.16.0.0 | V1.15.0.0 | 200 | 新增 AO 模拟电流输出服务和 AD5421 诊断；在 RSSI 运行态后追加 `AoOutputRuntime`；原 `reserved26` 正式替换为 `AoOutputEnable`，地址保持 `0x00C2-0x00C3` 不后移，CPU2/CPU3 同步支持 AO 输出启停，默认关闭。 |
| 11 | V1.18.1.0 | V1.16.1.0 | 200 | 复用 AO 相关保留字段为 AO 正常输出液位量程端点和独立报警液位阈值；同步 CPU2/CPU3 参数菜单、保持寄存器和运行期归一化语义。 |
| 12 | V1.19.0.0 | V1.17.0.0 | 200 | 删除共享命令 115 的强制提零点执行语义；命令码 115、状态码 `0x002F/0x802F` 改为保留，不再由 CPU3 菜单下发或由 CPU2 执行。 |
| 13 | V1.20.0.0 | V1.18.0.0 | 200 | 内部密度 raw 从 `kg/m3 x10` 升级为 `kg/m3 x100`；CPU3 状态页、密度参数菜单和 LTD 自有协议支持两位小数；DSM/Wartsila/SI协议在边界保持原对外口径。 |
| 14 | V1.21.0.0 | V1.19.0.0 | 200 | 新增 `CMD_SI_PROFILE = 20`、CPU2 SI profile 执行参数和 `density_distribution.profile_source` 共享状态；`40001~40003` 从普通分布参数解耦；CPU3 新增 SI 菜单、`40010~40023` 本机参数、自动 profile 调度、开始时间戳、状态/报警合成；SI profile 完成态和点阵只认 `PROFILE_SOURCE_SI`，并修正温度无效值、报警 0 阈值、负温度阈值、自动调度真实日历换算和 SI profile 不预先找液位的流程语义。 |
| 15 | V1.23.0.0 | V1.22.0.0 | 200 | 共享故障码按9个责任域重新分类和编号，新增TMC配置丢失、串口传输异常、无线响应格式异常和AD5421主动报警4项故障；CPU3本机主控通信故障调整为14-12。故障寄存器地址和宽度不变，但数值解释与协议14不兼容。 |
| 16 | 未单独发布（CPU2 V1.24.0.0开发过渡） | 未单独发布（CPU3 V1.22.0.0开发过渡） | 200 | 互换故障类别13和14：通信链路改为13类，传感器与密度改为14类；两类内部子码和故障语义不变。CPU3本机主控通信故障由14-12调整为13-12。故障寄存器地址和宽度不变，但数值解释与协议15及更早版本不兼容。 |
| 17 | V1.25.0.0 | V1.23.0.0 | 200 | 按二代计量仪责任域重新分类、删减并重编号故障码：CPU2保留63项共享故障，CPU3另有1项本机故障；正式表61项使用中、3项保留。CPU3本机主控通信故障由13-12调整为20-7。故障寄存器地址和宽度不变，但数值解释与协议16及更早版本不兼容。 |
| 18 | V1.26.0.0 | V1.24.0.0 | 200 | 在既有输入寄存器尾部、`REG_ENG` 前追加 `si_profile_runtime.phase`、`cycle_counter`、`progress_points` 三个32位运行态字段；用于 Point0 建立周期、有效点进度和回液位后最终快照门禁。SI 外部兼容同时增加 `30014~30016` 镜像、`40004~40009` CPU3 本地持久化槽和 CPU3 FRAM V7。 |

## 兼容判断规则

1. CPU2 启动时把本机 `DeviceParameters.protocolVersion` 修正为当前协议版本，并发布到保持寄存器。
2. CPU3 读取 CPU2 发布的协议版本字段。
3. 只有 CPU2 协议版本与 CPU3 当前 `DEVICE_PROTOCOL_VERSION` 完全相同时，才判定协议兼容。
4. 固件版本只用于显示和追踪发布，不参与 CPU2/CPU3 协议兼容判断。
5. 协议不匹配属于整机状态问题，由设备状态页统一提示；CPU2固件版本参数页只显示版本号。

## 变更记录

### 协议版本 1

关联改动：
- 密度分布测量最大点数从 100 扩展到 200。
- CPU2 `MAX_MEASUREMENT_POINTS` 改为 200。
- CPU3 `MAX_MEASUREMENT_POINTS` 改为 200。
- CPU3 “分布测点数”参数上限改为 200。
- CPU3 Wärtsilä 外部 Modbus `REG_DENSITY_POINT_COUNT` 改为 200。

兼容影响：
- CPU3 V1.2.0.0 与 CPU2 V1.4.1.0 组合时，CPU2 协议版本为 0，不能认为完整兼容。
- CPU3 V1.2.0.0 与 CPU2 V1.5.0.0 组合时，CPU2 协议版本为 1，满足 200 点密度分布协议。

### 协议版本 2

关联改动：
- 新增共享命令 `CMD_CANCEL_MEASUREMENT = 16`，CPU2 与 CPU3 的 `CommandType` 枚举保持一致。
- CPU3 状态显示界面长按返回键时，不进入菜单，改为向 CPU2 写入取消测量命令。
- CPU2 收到取消测量命令后，取消故障自动恢复，停止电机，清除当前命令和错误码，并切换到待机状态。

兼容影响：
- CPU3 V1.4.0.0 与 CPU2 V1.6.0.0 组合时，双方协议版本为 2，支持状态界面长按返回取消测量。
- CPU3 V1.4.0.0 与旧 CPU2 组合时，旧 CPU2 不识别命令 16；协议版本不匹配时应按设备状态页提示处理，不建议混用。

### 协议版本 3

关联改动：
- 原 `reserved23` 正式替换为“探底修正罐高”，保持寄存器地址不移动，单位沿用 0.1mm。
- CPU2 罐底测量后的编码器修正优先使用“探底修正罐高”；该参数为 0 时兼容旧逻辑，回退使用液位罐高。
- 瓦锡兰分布测量完成并达到测后探底频率后，先移动到固定点监测位置，仅移动不读取密度，再执行罐底测量。
- 瓦锡兰测后固定点移动或后续探底失败时，退出后续流程并切回单点监测，不通过 `SET_ERROR` 置错误状态。
- 罐底测量实高与编码器修正目标罐高的偏差大于实高最大偏差时，跳过编码器修正。

兼容影响：
- CPU2/CPU3 必须同为协议版本 3 才能正确同步“探底修正罐高”字段。
- 旧协议存储升级到协议版本 3 时，CPU2 将新增“探底修正罐高”参数恢复为 0，保持旧逻辑兼容。

### 协议版本 4

关联改动：
<- 将 SI 所需补充状态按业务含义融合到 CPU2/CPU3 既有测量结构：`DeviceStatus`、`OilMeasurement`、`ActualHeightMeasurement` 和 `DensityDistribution`。
- CPU2 发布探底参考有效、探头位于液位、液位稳定、profile 完成锁存、profile 完成计数、profile 被工况阻止、装卸液状态、手动报警抑制、手动液位更新抑制、profile 温度偏差报警、profile 密度偏差报警。
- CPU3 读取该状态区后供外部协议转换层使用，SI 地址、线圈、缩放和异常响应只存在于 CPU3 外部协议模块，避免外部协议直接反推或污染 CPU2 内部流程状态。
- 没有新增独立 `ProtocolAssistStatus` 结构，避免在 `MeasurementResult` 里再维护一套外部协议专用镜像；所有字段都落在原业务结构尾部，并由 CPU2/CPU3 两侧同名结构同步。

新增共享字段明细：

| 所属结构 | 新增字段 | 共享寄存器 | 语义 |
| --- | --- | --- | --- |
| `DeviceStatus` | `loading_unloading_active` | `REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE` | 当前是否处于装卸液过程，供外部协议判断工况。 |
| `DeviceStatus` | `manual_alarm_inhibit` | `REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT` | 手动/强制动作期间抑制自动报警语义，避免 CPU3 将手动过程误判为自动测量结果。 |
| `OilMeasurement` | `probe_at_liquid_level` | `REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL` | 找液位成功后置位，SI 可映射为 Probe At Liquid Level。 |
| `OilMeasurement` | `liquid_stable` | `REG_OIL_MEASUREMENT_LIQUID_STABLE` | 找液位成功后置位，表示当前液位结果可认为稳定。 |
| `OilMeasurement` | `manual_level_update_inhibit` | `REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT` | 手动/强制动作期间抑制液位自动更新语义。 |
| `ActualHeightMeasurement` | `bottom_reference_valid` | `REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID` | 探底成功或使用有效回退罐高后置位，SI 可映射为 Bottom Reference。 |
| `DensityDistribution` | `profile_complete_latched` | `REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED` | 分布测量成功后锁存完成状态，失败或命令切换不置位。 |
| `DensityDistribution` | `profile_complete_counter` | `REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER` | 每次 profile 成功完成后递增，CPU3 用计数变化锁存 SI profile 时间戳。 |
| `DensityDistribution` | `profile_blocked_by_process` | `REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS` | profile 被当前工况阻止时置位，供 CPU3 转换为外部协议互锁/阻止状态。 |
| `DensityDistribution` | `profile_temp_deviation_alarm` | `REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM` | 分布温度偏差报警状态。 |
| `DensityDistribution` | `profile_density_deviation_alarm` | `REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM` | 分布密度偏差报警状态。 |

寄存器布局影响：
- `DeviceStatus` 尾部新增 2 个 `uint32_t`，`REG_DEBUG_BASE` 随之顺延。
- `OilMeasurement` 尾部新增 3 个 `uint32_t`，`REG_WATER_MEASUREMENT_WATER_LEVEL` 随之顺延。
- `ActualHeightMeasurement` 尾部新增 1 个 `uint32_t`，`REG_SINGLE_POINT_MEAS_TEMP` 随之顺延。
- `DensityDistribution` 在 `REG_DENSITY_DIST_OIL_LEVEL` 后新增 5 个 `uint32_t`，`REG_DENSITY_DIST_POINT_BASE` 随之顺延。
- CPU2 与 CPU3 的结构字段顺序、寄存器宏表达式和读写打包顺序必须完全一致，否则后续寄存器会错位。

兼容影响：
- CPU2/CPU3 必须同为协议版本 4 才能正确同步融合后的 SI 状态字段。
- 旧 CPU3 不识别新增状态寄存器，不能完整支持 SI 的离散输入状态映射。
- 旧 CPU2 不发布这些新增状态字段，CPU3 V1.6.0.0 不能依赖旧协议数据生成完整 SI 状态。
- 由于多个分组尾部寄存器顺延，协议版本 4 与协议版本 3 不能混用；必须依赖 `DEVICE_PROTOCOL_VERSION` 严格相等检查拦截。

验证结果：
- `py tools\check_si_protocol_contract.py`：确认 CPU2/CPU3 的结构字段、寄存器宏和读写打包顺序一致。
- `py tools\check_si_modbus_frames.py`：确认 SI 外部地址常量和 golden frame 一致。
- `py tools\check_version_bumped.py`：确认 CPU2 V1.8.0.0、CPU3 V1.6.0.0 已匹配本次协议升级。
- `cmake --build build\LTD_MAIN_CPU2`、`cmake --build build\LTD_DISPLAY_CPU3`：两端构建通过。

### 协议版本 5

关联改动：
- 新增共享命令 `CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117`，CPU2 与 CPU3 的 `CommandType` 枚举保持一致。
- CPU3 维护/调试菜单新增“匹配无线滑环”无参入口，通过现有 `command` 保持寄存器写入 CPU2。
- CPU2 收到该命令后不执行电机 `MeasureStart()` 初始化，直接复用 `WirelessPairing_RunByRssi()` 执行 CH9141K 扫描、RSSI 最近候选选择、连接、默认连接保存、模块复位和主/从节点透传探测。
- 保留 CPU2 调试串口 `SPR`，正式命令与调试命令复用同一 RSSI 匹配流程。
- `DeviceState` 新增 `STATE_WIRELESS_PAIRING = 0x0032` 和 `STATE_WIRELESS_PAIRING_OVER = 0x8032`，用于 CPU3 设备状态页显示无线滑环匹配中和匹配完成。
- 在输入寄存器末尾追加 `WirelessPairingStatus`，不移动协议版本 4 既有输入寄存器地址。字段包括：
  - `result`：`0` 未执行或无结果，`1` 正在匹配，`2` 匹配成功，`3` 匹配失败。
  - `mac_valid`：成功 MAC 是否有效。
  - `mac_high` / `mac_mid` / `mac_low`：分别保存 `AA:BB`、`CC:DD`、`EE:FF` 三段 MAC。
  - `error_code`：匹配失败时的 CPU2 错误码。
  - `update_counter`：CPU2 每次匹配状态变化时递增，CPU3 用于识别新结果。
- CPU3 收到完成设备状态和成功结果后，在设备状态页显示两行从机 MAC；匹配失败时 CPU2 将 `device_status.device_state` 置为 `STATE_ERROR` 并写入 `device_status.error_code`，CPU3 按现有故障页显示错误码。该 MAC 状态为运行态共享数据，不写入 FRAM 参数。

兼容影响：
- CPU2/CPU3 必须同为协议版本 5 才能通过 CPU3 菜单触发无线滑环最近匹配并正确读取 MAC 状态。
- 协议版本 4 仅包含 SI 辅助状态，不包含命令 117 和 `WirelessPairingStatus`；CPU3 V1.7.0.0 不应与协议版本 4 的 CPU2 混用。
- 旧 CPU3 不提供该菜单入口，但不影响 CPU2 新版本通过调试串口 `SPR` 执行匹配。
- 旧协议 CPU3 不知道追加的 `WirelessPairingStatus` 字段；协议版本不匹配时不应继续解释匹配结果。

### 协议版本 6

关联改动：
- `DeviceState` 新增 `STATE_DEBUG_MODE = 0x0033`，CPU2 与 CPU3 枚举值保持一致。
- CPU2 串口调试命令中，除调用正式测量流程的 `F/H/J`、正式命令映射、无线滑环 `SP*` 和演示 `X` 之外，执行期间临时发布 `STATE_DEBUG_MODE`。
- CPU2 串口 `B/BE` 诊断进入时切换到 `STATE_DEBUG_MODE`，退出时恢复进入前的业务状态和错误码快照；诊断期间清除临时错误时仍保持调试模式显示。
- 原 `reserved2` 正式替换为 `fault_auto_recovery_retry_limit`，用于控制故障自动恢复确认成功后最多自动重跑原命令次数：`0` 关闭，`1~10` 为上限，默认 `3`。
- `empty_weight` 空载扭力按 `int32_t` 有符号 32 位参数解释，寄存器仍占 2 个 word，负值按二进制补码传输。
- CPU3 设备状态页新增 `STATE_DEBUG_MODE` 显示文案“调试模式中”，英文文案为 `Debug Mode`。
- CPU3 参数页将原 `COM_NUM_DEVICEPARAM_RESERVED2` 显示为“故障自动恢复重跑次数”，仍写入 `HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION + REG_STRIDE` 对应地址。
- CPU3 外部 DSM 状态转换层将内部 `STATE_DEBUG_MODE` 对外映射为既有维护模式状态，避免 DSM 主站收到未知 `0x0033`。

寄存器布局影响：
- 不新增保持寄存器或输入寄存器地址，不移动后续参数地址。
- `DeviceParameters` 原 `reserved2` 字段语义替换为 `fault_auto_recovery_retry_limit`；CPU2/CPU3 Modbus 打包顺序不变。
- `HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT` 地址和长度不变，仅将 32 位寄存器内容解释为有符号值。
- CPU2 旧协议存储升级到协议版本 6 时，将该字段补为默认 `3`，避免旧预留值 `0` 被误解释为关闭自动恢复。
- 本次提升协议版本的原因是共享 `DeviceState` 语义新增和共享参数语义新增，旧 CPU3 无法正确显示新状态，也无法展示/写入新参数语义。

兼容影响：
- CPU2/CPU3 必须同为协议版本 6 才能正确显示串口调试模式状态，并正确同步故障自动恢复重跑上限。
- CPU2 协议版本 6 与旧 CPU3 混用时，旧 CPU3 不识别 `STATE_DEBUG_MODE`；应由协议版本不匹配检查拦截。
- CPU3 协议版本 6 与旧 CPU2 混用时，旧 CPU2 不会发布 `STATE_DEBUG_MODE`，原 `reserved2` 也没有故障恢复次数语义；仍应由协议版本不匹配检查拦截。

### 协议版本 7

关联改动：
- 在 CPU2/CPU3 共享 `DeviceParameters` 的 `reserved33` 后、元信息字段前追加 `RelayAlarmConfig relayAlarm[4]`。
- 每路继电器配置占 13 个 32 位字段，字段顺序为：`operating_mode`、`digital_source`、`contact_type`、`alarm_mode`、`error_value`、`alarm_source`、`HH_alarm_value`、`H_alarm_value`、`L_alarm_value`、`LL_alarm_value`、`alarm_hysteresis`、`damping_factor`、`clear_alarm`。
- `HH/H/L/LL_alarm_value` 与 `alarm_hysteresis` 按 IEEE754 float 原始位通过两个保持寄存器传输；CPU3 菜单按 1 位小数显示/输入。
- 新增保持寄存器基址 `HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE`，四路配置结束地址为 `HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_END`；`param_version`、`struct_size`、`magic`、`crc` 顺延到新配置之后。
- CPU3 新增四路继电器报警输出配置菜单和枚举文字表；原 `AlarmHighDO`、`AlarmLowDO`、`ThirdStateThreshold` 旧 DO 报警接口直接删除，后续 AO、指令参数、尺带补偿、继电器报警输出配置和元信息寄存器整体前移。
- 在 CPU2/CPU3 共享 `MeasurementResult` 的末尾追加 `RelayAlarmRuntimeState relay_alarm_runtime[4]`，用于发布每路当前报警值、HH/H/L/LL、组合报警、任意报警和清锁存运行态。
- `RelayAlarmRuntimeState` 字段顺序与参考程序只读区/不存储区一致：`alarm_value`、`HH_alarm`、`H_alarm`、`HH_H_alarm`、`L_alarm`、`LL_alarm`、`LL_L_alarm`、`any_error`、`clear_alarm`。
- CPU2 参数结构版本同步提升到 `DEVICE_PARAM_VERSION=3`，旧 FRAM 参数不会按新结构误读。

寄存器布局影响：
- 保持寄存器在 `HOLDREGISTER_DEVICEPARAM_RESERVED33 + REG_STRIDE` 后新增 104 个寄存器（四路 * 13 字段 * 2 寄存器）。
- 同时删除旧 DO 报警 3 个 32 位字段，AO 及其后续保持寄存器前移 6 个寄存器；协议版本 7 相对协议版本 6 的保持寄存器净增量为 98 个寄存器。
- 元信息与 CRC 寄存器整体后移，`HOLEREGISTER_STOP` 随之增大。
- 输入寄存器在无线滑环匹配状态后追加四路继电器运行态：`REG_RELAY_ALARM_RUNTIME_BASE = REG_WIRELESS_PAIRING_UPDATE_COUNTER + REG_SIZE_U32`。
- 每路继电器报警输出运行态占 10 个输入寄存器：`alarm_value` 按 IEEE754 float 占 2 个寄存器，其余 8 个状态字段各占 1 个寄存器；四路共追加 40 个输入寄存器，`REG_ENG` 随之后移。
- CPU3 上电和运行期轮询组 3 从 `REG_WIRELESS_PAIRING_RESULT` 读到新的 `REG_ENG`，同时同步无线滑环匹配状态和继电器运行态。

兼容影响：
- CPU2/CPU3 必须同为协议版本 7 才能正确显示、写入和执行四路继电器报警输出配置，并正确读取四路运行态。
- 协议版本 6 的 CPU3 不知道新增继电器配置寄存器、输入运行态寄存器和菜单，不能配置或解释继电器。
- 协议版本 6 的 CPU2 不执行新增 `relayAlarm[4]` 配置，也不会发布四路继电器运行态和顺延后的元信息地址；协议版本不匹配应由 CPU3 严格相等检查拦截。

验证结果：
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过。
- 未做实物联调；需要现场验证 RELAY1~RELAY4 输出极性、锁存清除、无效值故障策略和 CPU3 运行态显示/读取。

### 协议版本 8

关联改动：
- `DeviceParameters.liquidLevelMeasurementMethod` 仍使用原保持寄存器和原 32 位字段，不新增或移动寄存器。
- 液位测量方式语义调整为：`0=相对频率`、`1=定频`、`2=密度连续找液位`、`3=超声预留`、`4=连续相对频率`、`5=连续定频`。
- CPU2 在 `SearchOilLevel()` / `FollowOilLevel()` 中保留 0/1 原步进式频率方案；方法 2 进入密度阈值速度闭环；方法 4/5 进入频率偏差速度闭环。
- CPU3 “液位测量方式”菜单新增“连相对频率”和“连定频”，并将参数元数据有效范围同步为 `0..5`。

寄存器布局影响：
- 不新增保持寄存器、输入寄存器、命令码或共享结构体字段。
- `DeviceParameters` 结构大小、`DEVICE_PARAM_VERSION`、`param_version`、`struct_size`、`magic` 和 `crc` 字段位置不变。
- 本次提升协议版本的原因是已有字段 `liquidLevelMeasurementMethod` 的跨 CPU 参数语义发生变化；旧协议 7 的 CPU3 或外部工具可能不知道 2/4/5 的新含义。

兼容影响：
- CPU2/CPU3 必须同为协议版本 8，才能正确显示、写入并执行新的连续找液位方式。
- 协议版本 7 的 CPU3 只知道旧液位测量方式语义，不应继续向协议版本 8 的 CPU2 写入或解释该字段。
- 协议版本 7 的 CPU2 不支持新的 2/4/5 连续找液位执行路径，协议版本不匹配应由 CPU3 严格相等检查拦截。

验证结果：
- `py -3 tools\check_density_level_control_contract.py`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证密度连续、连续相对频率、连续定频三种新方案的电机速度闭环方向、速度响应、稳定判定和命令切换退出。

### 协议版本 9

关联改动：
- `CMD_READ_PART_PARAMS` / `STATE_READPARAMETEROVER` 读取部件参数流程增加当前 CH9141K 蓝牙连接 RSSI 只读查询。
- CPU2 新增 `WirelessPairing_ReadConnectionStatus()` 和 `WirelessPairing_UpdateConnectionStatusSnapshot()`，复用 `BLEMODE`、`BLESTA`、`CCADD` 和 `AT+RSSI=ON/OFF` 查询链路；不扫描、不重新配对、不保存默认连接。
- `WirelessPairingStatus` 在原匹配结果字段后新增 `connection_valid`、`rssi_valid`、`rssi`、`connection_error_code`、`rssi_update_counter`。
- CPU3 读取参数完成态新增 RSSI 显示项：有效时显示 `RSSI:<value>dB`，无效时显示 `RSSI:N/A`。

寄存器布局影响：
- 保留协议版本 8 既有无线滑环匹配状态和四路继电器运行态地址。
- RSSI 运行态字段追加在继电器运行态之后：
  - `REG_WIRELESS_PAIRING_CONNECTION_VALID`
  - `REG_WIRELESS_PAIRING_RSSI_VALID`
  - `REG_WIRELESS_PAIRING_RSSI`
  - `REG_WIRELESS_PAIRING_CONNECTION_ERROR_CODE`
  - `REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER`
- 协议版本 9 中 `REG_ENG` 顺延到 `REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER + REG_SIZE_U32`。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 9，才能正确读取和显示当前蓝牙连接 RSSI。
- 协议版本 8 的 CPU3 不知道 RSSI 尾部输入寄存器，不应解释新增字段。
- 协议版本 8 的 CPU2 不发布 RSSI 快照，协议版本不匹配应由 CPU3 严格相等检查拦截。

验证结果：
- `py -3 tools\check_si_protocol_contract.py`：通过。
- `py -3 tools\check_density_level_control_contract.py`：通过。
- `py -3 tools\check_read_part_params_refresh_contract.py`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证已连接、未连接、RSSI 异步超时、命令切换打断和 UART6 透传恢复。

### 协议版本 10

关联改动：
- CPU2 新增 AO 模拟电流输出服务，统一负责液位电流计算、报警电流覆盖、故障/调试电流选择、AD5421 写入和运行态维护。
- AD5421 驱动新增有限 SPI 超时、控制寄存器回读、FAULT 管脚/READFAULT 诊断和专用错误码：`AD5421_INIT_ERROR`、`AD5421_WRITE_CURRENT_ERROR`、`AD5421_FAULT_PIN_ERROR`、`AD5421_READFAULT_ERROR`、`AD5421_READBACK_ERROR`。
- HART Command 2/3 的电流值改为读取 AO 运行态，百分比按标准 4-20mA 量程 `(mA - 4.0) / 16.0` 计算并钳位。
- CPU2/CPU3 共享输入寄存器在 RSSI 运行态后追加 `AoOutputRuntime`，用于发布 AO 目标电流、最近成功写入电流、来源、AD5421 故障标志、READFAULT 原始值、最近错误码和更新时间。
- 原 `reserved26` 正式替换为 `AoOutputEnable`，语义为 `0=关闭`、`1=启用`。
- CPU2 恢复出厂默认值将 `AoOutputEnable` 置为 `0`；旧协议存储升级到协议版本 10 时也强制置为 `0`，避免旧预留值误启用电流输出。
- CPU2 AO 服务在关闭时不初始化、不诊断、不写入 AD5421，并将运行态来源置为 `AO_OUTPUT_SOURCE_DISABLED`，用于未接电流环场景避免 AD5421 环路故障上报。
- CPU3 同步解析新增 AO 运行态字段，补充 AD5421 相关错误码的屏幕故障文案，并通过参数表、保持寄存器读写、参数同步指针和 AO 菜单支持 `AoOutputEnable`。
- 同步收紧 `bottom_detect_mode` 的显示与写入范围为 `0..1`，与 CPU2 实际枚举 `0=扭力`、`1=陀螺仪角度` 一致；异常值运行期归零，避免 CPU3 菜单显示“非法配置”或探底准备流程误判。

寄存器布局影响：
- 输入寄存器在协议版本 9 的 `REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER` 后追加 AO 运行态：
  - `REG_AO_OUTPUT_RUNTIME_TARGET_MA_X100`
  - `REG_AO_OUTPUT_RUNTIME_LAST_SENT_MA_X100`
  - `REG_AO_OUTPUT_RUNTIME_SOURCE`
  - `REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_FLAGS`
  - `REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_REGISTER`
  - `REG_AO_OUTPUT_RUNTIME_LAST_ERROR_CODE`
  - `REG_AO_OUTPUT_RUNTIME_UPDATE_COUNTER`
  - `REG_AO_OUTPUT_RUNTIME_LAST_UPDATE_TICK`
  - `REG_AO_OUTPUT_RUNTIME_LAST_SENT_TICK`
- `REG_ENG` 和 `INPUTREGISTER_AMOUNT` 随新增 AO 运行态顺延；旧输入寄存器地址不移动。

- 保持寄存器不新增地址，`HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE = HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA + REG_STRIDE`。
- `HOLDREGISTER_DEVICEPARAM_RESERVED26` 作为兼容别名等于 `HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE`。
- `HOLDREGISTER_DEVICEPARAM_RESERVED27` 仍为 `HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE + REG_STRIDE`，后续保持寄存器地址不移动。
- `DeviceParameters` 结构体大小和 `DEVICE_PARAM_VERSION` 不变；本次只改变原预留字段语义并追加输入运行态。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 10，才能正确读取 AO 运行态、识别 AD5421 专用错误码，并正确显示、写入和执行 AO 输出使能。
- 协议版本 9 的 CPU3 不知道新增 AO 输入寄存器尾部、AD5421 错误码文案和 `AoOutputEnable` 参数语义，不应与协议版本 10 的 CPU2 混用。
- 协议版本 9 的 CPU2 不发布 AO 运行态，也不按 `AoOutputEnable` 控制 AO 服务，协议版本 10 的 CPU3 应由严格协议版本相等检查拦截。

验证结果：
- `py -3 tools\check_ao_output_enable_contract.py`：通过。
- `py -3 tools\check_si_protocol_contract.py`：通过。
- `py -3 tools\check_density_level_control_contract.py`：通过。
- `py -3 tools\check_wireless_rssi_contract.py`：通过。
- `py -3 tools\check_read_part_params_refresh_contract.py`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证 `AoOutputEnable=0` 且电流环未接时不上报 AO 故障，`AoOutputEnable=1` 时 4-20mA 输出、HART 电流值和 AD5421 故障诊断恢复正常。

### 协议版本 11

关联改动：
- 保持寄存器地址不新增、不移动，`AOStartLevel_01mm/AOEndLevel_01mm` 复用原 `reserved24/reserved25` 参数位置作为 AO 正常输出液位量程端点。
- `AOStartLevel_01mm` 复用原 `reserved24` 参数位置，语义为 AO 起点液位，单位 `0.1mm`，对应 `CurrentRangeStart_mA` 正常起点电流。
- `AOEndLevel_01mm` 复用原 `reserved25` 参数位置，语义为 AO 终点液位，单位 `0.1mm`，对应 `CurrentRangeEnd_mA` 正常终点电流。
- `CurrentRangeStart_mA/CurrentRangeEnd_mA` 保留为正常输出起点/终点电流，单位 `0.01mA`；允许起点电流大于终点电流以支持反向 20mA->4mA 输出，二者相等时 CPU2 归一化为默认 4.00mA/20.00mA。
- `AlarmHighAO/AlarmLowAO` 语义改为 AO 独立高/低报警液位阈值，单位 `0.1mm`。
- `AOHighCurrent_mA/AOLowCurrent_mA` 继续表示 AO 高/低报警触发后的输出电流，单位 `0.01mA`；`InitialCurrent_mA/AOHighCurrent_mA/AOLowCurrent_mA/FaultCurrent_mA/DebugCurrent_mA` 统一按 AD5421 硬件输出范围限制为 `3.20..24.00mA`。
- CPU2 AO 输出不再读取继电器 `HH/H` 或 `L/LL` 运行态覆盖电流，改为按 `AlarmHighAO/AlarmLowAO` 独立判断 AO 报警；继电器报警继续使用各路 `relayAlarm[]` 阈值。
- AO 高低报警阈值中任一项为 `0` 表示关闭对应报警；两者均非 `0` 时应保持低报警液位低于高报警液位。如果高报警超出罐高，CPU2 写参后钳位到罐高；如果低报警超出罐高或高低报警重叠，CPU2 写参后恢复为出厂默认高报 `tankHeight`、低报 `0`。

寄存器布局影响：
- 保持寄存器地址和数量不变。
- `HOLDREGISTER_DEVICEPARAM_AO_START_LEVEL` 和 `HOLDREGISTER_DEVICEPARAM_AO_END_LEVEL` 从保留字段变为 AO 液位量程端点。
- `HOLDREGISTER_DEVICEPARAM_AO_NORMAL_CURRENT_START_mA`、`HOLDREGISTER_DEVICEPARAM_AO_NORMAL_CURRENT_END_mA` 地址不变，仅明确为电流端点。
- `HOLDREGISTER_DEVICEPARAM_AO_HIGH_ALARM_LEVEL`、`HOLDREGISTER_DEVICEPARAM_AO_LOW_ALARM_LEVEL` 地址不变，字段单位从旧电流口径修正为液位阈值口径。
- CPU3 操作号同步改为 `COM_NUM_DEVICEPARAM_AO_START_LEVEL`、`COM_NUM_DEVICEPARAM_AO_END_LEVEL`、`COM_NUM_DEVICEPARAM_AO_NORMAL_CURRENT_START_mA`、`COM_NUM_DEVICEPARAM_AO_NORMAL_CURRENT_END_mA`、`COM_NUM_DEVICEPARAM_AO_HIGH_ALARM_LEVEL`、`COM_NUM_DEVICEPARAM_AO_LOW_ALARM_LEVEL`。
- `DeviceParameters` 结构体大小和 `DEVICE_PARAM_VERSION` 不变。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 11，才能正确显示和写入 AO 液位量程端点、AO 独立报警液位阈值和状态电流值。
- 协议版本 10 的 CPU3 会把 `reserved24/reserved25` 显示为保留字段，并把 `AlarmHighAO/AlarmLowAO` 显示为报警电流，不能与协议版本 11 的 CPU2 混用。
- CPU2 从旧协议存储升级到协议版本 11 时，会把 `AOStartLevel_01mm=0`、`AOEndLevel_01mm=tankHeight`、`AlarmHighAO=tankHeight`、`AlarmLowAO=0` 写入运行参数，避免旧保留值或旧电流口径被误解释成液位报警阈值；运行期写参后也会把 AO 液位端点归一化到 `0..tankHeight` 且保持起点小于终点，并把特殊 AO 电流参数归一化到 `320..2400` 后回读。
- `DEVICE_PARAM_VERSION` 保持不变，不会因本次升级触发恢复出厂参数；但协议语义变化后需要现场复核 AO起点/终点液位点、正常起点/终点电流方向和 AO 报警液位阈值。

验证结果：
- `git diff --check`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证 AO起点/终点液位量程、正向和反向正常电流映射、AO 独立高/低报警电流覆盖，以及继电器报警阈值不再影响 AO 报警输出。

### 协议版本 12

关联改动：
- 共享命令 `115` 从原强制提零点执行命令改为 `CMD_RESERVED_CMD7`。
- CPU2 删除 `CMD_ForceLiftZero()` 执行入口；收到命令 115 时按保留/未知指令处理，只打印暂不支持，不启动电机。
- CPU3 删除“强制提零点”菜单项、无参命令映射、电机运行监控归类和状态页文案。
- 设备状态 `0x002F`、`0x802F` 改为保留状态，不再作为强制提零点运行/完成状态发布或显示。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 12，才能确保命令 115 不再被解释为可执行电机动作。
- 协议版本 11 的 CPU3 仍可能下发命令 115；协议版本 12 的 CPU2 会拒绝执行，但应通过协议不匹配提示避免混用。
- 协议版本 11 的 CPU2 仍可能执行命令 115；协议版本 12 的 CPU3 不再提供该菜单入口，但仍必须依赖严格协议版本相等检查避免旧 CPU2/新 CPU3 混用。
- 本次不新增、不移动寄存器字段，不改变 `DeviceParameters` 存储结构；协议升级原因是跨 CPU 共享命令语义发生变化。

验证结果：
- `py -3 tools\check_reserved_cmd7_contract.py`：通过。
- `py -3 tools\check_ao_output_enable_contract.py`：通过。
- `py -3 tools\check_wireless_rssi_contract.py`：通过。
- `py -3 tools\check_version_bumped.py`：通过。
- `git diff --cached --check`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。

### 协议版本 13

关联改动：
- CPU2 `DENSITY_TO_RAW()` / `RAW_TO_DENSITY()` 统一改为 `kg/m3 x100`。
- CPU2 `DEVICE_PARAM_VERSION` 保持 `3`，通过旧 FRAM 中 `protocolVersion < 13` 识别一次性迁移 `oilLevelDensity`、`oilLevelThreshold`、`oilLevelHysteresisThreshold` 和 `densityCorrection`。
- `densityCorrection` 零点从 `10000` 迁移到 `100000`，修正量从 `(raw - 10000) / 10` 改为 `(raw - 100000) / 100`。
- CPU3 状态页密度显示小数位改为 2，尾零裁剪逻辑保留。
- CPU3 参数菜单中 `液位跟随密度`、`磁通量D`、`密度手输值` 改为两位密度口径；`液位找液阈值`、`液位滞后阈值` 继续面向频率液位显示为 `Hz`，并通过 `point=1` 保持默认 `150/200` 对应 `15.0/20.0 Hz`。
- CPU3 本机 FRAM 参数版本升级到 `0x0005`，读取 `0x0003` 或 `0x0004` 时迁移本机手输密度 `screen_input_d`。
- DSM 外部协议输出密度和密度修正时保持原 `x10` 口径；外部写入密度修正时转换回内部 `x100`。
- Wartsila 外部协议密度继续保持 `scale = 10` / `x10`。
- SI协议继续保持既有 `0.01 kg/m3` 口径，内部升级后取消原先从 `x10` 到 `x100` 的额外乘 10。

兼容影响：
- 协议版本 13 改变内部密度倍率语义，但不移动共享寄存器地址和字段长度。
- CPU2/CPU3 必须同为协议版本 13 才能正确解释内部密度字段。
- 旧 FRAM 参数由 CPU2 按旧协议版本标记迁移后写回 `protocolVersion = 13`，CPU3 本机 FRAM 写回 `0x0005`，避免按新倍率误读旧参数。
- 除 LTD 自有协议外，DSM、Wartsila、SI 主站不需要修改密度倍率解析。

验证结果：
- `py tools\check_density_precision_contract.py`：通过。
- `py tools\check_synthetic_density_mode_contract.py`：通过。
- `py tools\check_si_modbus_frames.py`：通过。
- `py tools\check_si_protocol_contract.py`：通过。

### 协议版本 14

关联改动：
- 新增共享命令 `CMD_SI_PROFILE = 20`，CPU3 的 SI `00004 Profile` 写 ON 后锁存 profile 开始时间，并通过 CPU2 命令保持寄存器下发该命令。
- CPU2 新增 SI profile 执行参数：`si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval`。
- CPU2 复用原 `reserved30~reserved33` 预留参数槽，`DEVICE_PARAM_VERSION` 保持 `3`，但 `protocolVersion < 14` 或运行期归一化时会补默认值：首点 `1000`、步距 `10000`、停留 `10`、探底频次 `1`。
- CPU2 新增 `CMD_SiProfile()` 独立测量入口，执行时直接读取本机 SI profile 参数，支持探底频次、旧底部位置回退、Point0、最多 200 点、候选点运行中判定液面以上后停止、完成锁存和完成计数。
- 探底失败且无旧底部位置时，CPU2 使用当前位置采集 Point0 并继续本轮 profile，但不刷新 SI 内部底部位置、不置底部位置有效、不清首次探底状态。
- CPU3 参数同步链路新增 SI profile 参数读写，`40001~40003` 不再桥接 `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime`。
- CPU3 本机参数新增 `40010~40023` 对应的 SI 自动 profile 和报警限值参数，CPU3 本机 FRAM 参数版本从 `0x0005` 升级到 `0x0006`，并提供 V3/V4/V5 迁移。
- CPU3 菜单新增 `SI参数` 入口，并拆分 `Profile参数`、`自动Profile`、`报警限值` 三个子页。
- CPU3 SI 层新增自动 profile 调度任务 `si_modbus_periodic_task()`，按 `40010~40013` 和 CPU3 RTC 从起始时间起周期触发，可跨天，同一分钟去重。
- CPU3 SI协议状态口径调整：`10010 Probe Un-calibrated` 固定 `0`，`10002/10003` 按液位跟随稳定状态合成，`10001 Bottom Reference` 保持既有 `bottom_reference_valid` 映射。
- CPU3 SI协议报警口径调整：`40014~40023` 独立保存，CPU3 合成当前值报警、profile 上下限报警和相邻点温度/密度偏差报警。
- CPU2/CPU3 `DensityDistribution` 共享结构新增 `profile_source` 字段，位置在 `profile_complete_counter` 之后、`profile_blocked_by_process` 之前。
- CPU2 在普通分布、国标、每米、间隔、Wärtsilä 和 SI profile 完成后分别写入 `PROFILE_SOURCE_STANDARD/GB/METER/INTERVAL/WARTSILA/SI`。
- CPU3 SI 协议层读取 `10005 Profile Complete`、`30006 Number of Points` 和 `30021~30620` 点阵时，只认 `profile_source == PROFILE_SOURCE_SI`。
- CPU3 SI 协议层读回 `00004 Profile` 运行态时，同时要求 CPU2 当前命令为 `CMD_SI_PROFILE`，普通分布和 Wärtsilä 分布运行中不再误置 SI Profile 模式线圈。
- SI 温度无效值对外统一为 `-200.00°C`，即寄存器补码 `0xB1E0`；密度无效值保持 `0`。
- SI 报警阈值 `0` 改为有效阈值，不再作为禁用条件；无效温度和无效密度仍不参与报警判断。
- `40016/40017` 按有符号 `0.01°C` 保存和比较，Modbus 写入按 int16 补码解释，CPU3 菜单允许负温度限值。
- `10025 Profile Temp Deviation Alarm` 的相邻点温差按 int16 温度值计算，避免负温度跨零时按无符号补码差值误报警。
- 自动 profile 调度从近似日期索引改为真实日历分钟数，避免跨月或长 interval 漂移。
- `30007~30010 Profile Timestamp` 改为 CPU3 收到 SI `00004 Profile`、屏幕 SI profile 或自动 profile 触发时锁存的测量开始时间。

寄存器布局影响：
- 共享保持寄存器数量不减少；原 `reserved30~reserved33` 对应地址变为 SI profile 参数地址，继电器等后续参数基址保持顺延关系不变。
- 共享输入寄存器在密度分布结果区新增 `REG_DENSITY_DIST_PROFILE_SOURCE`，后续 `profile_blocked_by_process`、profile 偏差报警和点阵基址顺延。
- SI 外部 Modbus 地址窗口不变，仍为 `00001~00016`、`10001~10032`、`40001~40023`、`30001~30620`。
- CPU2 `DeviceParameters` 结构体大小和 `DEVICE_PARAM_VERSION` 不变，但字段语义从预留改为 SI profile 执行参数；旧协议存储通过 `protocolVersion < 14` 补默认值并写回当前协议版本。
- CPU3 本机参数结构新增 SI 自动 profile 和报警限值字段；该变化只影响 CPU3 本机 FRAM 参数版本，不改变 CPU2/CPU3 共享输入寄存器点数上限；`40016/40017` 的 CPU3 存储和菜单解释改为有符号温度阈值。

兼容影响：
- CPU2/CPU3 必须同为协议版本 14，才能正确识别 `CMD_SI_PROFILE`、SI profile 参数字段、`profile_source` 共享状态和 `40001~40003` 的新语义。
- 协议版本 13 的 CPU3 会把 `00004 Profile` 桥接到普通分布测量，并把 `40001~40003` 写入普通分布参数，不能与协议版本 14 的 CPU2 混用。
- 协议版本 13 的 CPU2 不识别 `CMD_SI_PROFILE`，也不会按 SI Point0/探底频次执行 profile；协议版本不匹配应由 CPU3 严格相等检查拦截。
- CPU2 参数存储不会因本次升级恢复出厂参数；但原预留槽会被赋予 SI profile 参数语义，现场升级后需要复核 SI 首点、步距、停留和探底频次。
- CPU3 本机参数升级到 `0x0006` 后会新增自动 profile 和报警限值默认值；旧 V3/V4/V5 本机参数迁移后应复核 SI 自动调度和报警阈值。
- 协议版本 13 的 CPU3 会把普通分布或 Wärtsilä 分布结果误认为 SI profile 结果；协议版本 14 修正为只认 SI profile 来源。
- 协议版本 13 的 CPU3 菜单和 Modbus 侧不能正确保存负温度限值，并且报警阈值 0 会被视为未启用；协议版本 14 修正为 signed 温度和 0 有效阈值。

验证结果：
- `py tools\check_si_modbus_frames.py --dump`：通过。
- `py tools\check_si_protocol_contract.py`：通过。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，未发现缺字。
- `cmake --build build\LTD_MAIN_CPU2`：通过，Ninja 无需重建。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过，Ninja 无需重建。

### 协议版本 15

关联改动：
- CPU2/CPU3共享故障码按责任域重新分类，类别代码固定为：11电机驱动、12编码器、13传感器与密度、14通信链路、15测量流程、16模拟量输出、17参数存储、18扭力检测、19系统与软件。
- CPU2共享 `ErrorCode` 由73项调整为77项，其中包含2个非故障状态和75个共享故障码。
- 新增 `MOTOR_TMC_CONFIG_LOST = 0x000B000B`，用于TMC关键配置为空或未保持，屏幕显示 `11-11`。
- 新增 `COMM_UART_TRANSFER_ERROR = 0x000E0004`，用于UART或DMA启动、发送、接收硬件错误，屏幕显示 `14-4`。
- 新增 `WIRELESS_RESP_FORMAT_ERROR = 0x000E000B`，用于无线模块响应字段缺失或解析失败，屏幕显示 `14-11`。
- 新增 `AD5421_FAULT_STATUS_ERROR = 0x00100005`，用于AD5421故障寄存器报告非零状态，屏幕显示 `16-5`。
- CPU3本机 `CPU2_COMM_TIMEOUT` 调整为 `0x000E000C`，屏幕显示 `14-12`；该码仍只由CPU3产生，不加入CPU2共享枚举。
- 测量流程码消除历史跳号，连续使用 `15-1` 至 `15-15`。
- AD5421相关故障由19类调整到16类；传感器物理量与通信链路分开；参数地址和内部调用条件调整到19类。
- CPU2错误日志名称、原因、责任域和类别兜底已同步新版编号；CPU3枚举、类别显示和具体原因已同步新版编号。

寄存器布局影响：
- 不新增、不删除、不移动故障状态寄存器，错误码仍按32位值传递。
- 本次改变故障码数值和类别解释，因此属于共享协议语义变更。
- `DeviceParameters` 结构大小、CPU2 `DEVICE_PARAM_VERSION` 和CPU3本机参数存储布局不因本次故障码调整而变化。

兼容性影响：
- 协议版本15与14及更早版本的故障码数值不兼容，CPU2和CPU3必须成对升级。
- 协议版本14的CPU3会按旧值显示错误类别和原因，不能与协议版本15的CPU2混用；协议版本15的CPU3也不能按新版表解释协议版本14的CPU2错误值。
- 上位机、日志分析工具或售后资料如果按数值解析故障码，必须同步新版编号表。
- 旧日志继续按产生时的CPU2程序版本和旧编号表解释，不对历史记录做数值转换；CPU3本机故障按CPU3程序版本单独确认。
- 首个使用协议版本15的CPU2固件为 `V1.23.0.0`，CPU3固件为 `V1.22.0.0`；协议14及更早版本不得与协议15交叉烧写或混用。

验证结果：
- `py tools\check_fault_code_catalog_contract.py`：通过，共享码77个、CPU3本机码1个、正式表使用中61项、保留15项。
- `py tools\check_sensor_fault_contract.py`、`check_tmc5130_diagnostic_contract.py`、`check_ad5421_diagnostic_contract.py`、`check_parameter_measurement_fault_contract.py`：通过。
- `py tools\check_cpu3_fault_reason_visibility.py`、`check_cpu3_cpu2_comm_timeout_fault_contract.py`：通过。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，缺字0个。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`、`git diff --check`：通过。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：协议 15 代码范围构建通过；升版前产物为 `LTD_MAIN_CPU2_V1.22.0.0.hex`，版本修正后应为 `LTD_MAIN_CPU2_V1.23.0.0.hex`，本记录不把升版前文件名写成最终产物证据。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：协议 15 代码范围构建通过；升版前产物为 `LTD_DISPLAY_CPU3_V1.21.0.0.hex`，版本修正后应为 `LTD_DISPLAY_CPU3_V1.22.0.0.hex`，本记录不把升版前文件名写成最终产物证据。
- 实物故障注入和协议14/15交叉烧写拦截仍需台架验证。

### 协议版本 16

关联改动：
- 共享故障码13类和14类互换：13类改为通信链路，14类改为传感器与密度。
- 通信链路完整码由 `0x000E0001`～`0x000E000B` 调整为 `0x000D0001`～`0x000D000B`；各枚举的低16位子码、名称、原因和处理语义不变。
- 传感器与密度完整码由 `0x000D0001`～`0x000D0006` 调整为 `0x000E0001`～`0x000E0006`；各枚举的低16位子码、名称、原因和处理语义不变。
- CPU3本机 `CPU2_COMM_TIMEOUT` 由 `0x000E000C`（屏幕 `14-12`）调整为 `0x000D000C`（屏幕 `13-12`），产生条件和恢复口径不变。
- CPU2错误日志的类别模块和兜底原因映射随高16位类别号同步；CPU3类别显示同步为 `0x000D` 通信故障、`0x000E` 传感器故障。

寄存器布局影响：
- 不新增、不删除、不移动故障状态寄存器，错误码仍按32位值传递。
- 不改变类别内部子码、故障语义、触发条件和恢复策略。
- `DeviceParameters` 结构大小、CPU2 `DEVICE_PARAM_VERSION` 和CPU3本机参数存储布局不变。

兼容性影响：
- 协议版本16与15及更早版本的故障码数值解释不兼容，CPU2和CPU3必须成对升级。
- 上位机、日志分析工具和资料如果按数值解析故障码，必须同步协议16编号表。
- 旧日志继续按产生时的CPU2程序版本选择对应代码表；旧版 `13-x`、`14-x` 不做原地改写。CPU3本机故障按CPU3程序版本单独确认。
- `LTD故障代码统一表.xlsx` 的“历史故障代码_CPU2V1.24.0.0”工作表冻结CPU2 V1.24.0.0开发基线编号。该工作表按CPU2程序版本归档；`DEVICE_PROTOCOL_VERSION` 仍用于CPU2/CPU3通信兼容判断，但不作为正式故障码表的现场选择依据。后续删除、合并或重编号不得回写该历史表，旧日志按产生时的CPU2程序版本查询；CPU3本机13-12按CPU3程序版本单独确认。
- 协议16仅作为故障类别互换的开发过渡基线保留，未形成独立发布固件；正式发布直接使用协议17。

验证结果：
- `py tools\check_fault_code_catalog_contract.py`：通过，共享码77个、CPU3本机码1个，正式表使用中61项、保留15项。
- `py tools\check_cpu3_cpu2_comm_timeout_fault_contract.py`、`check_cpu3_fault_reason_visibility.py`：通过，CPU3本机故障为 `13-12 / 0x000D000C`，13类和14类显示原因完整。
- `py tools\check_sensor_fault_contract.py`、`check_parameter_measurement_fault_contract.py`、`check_tmc5130_diagnostic_contract.py`、`check_ad5421_diagnostic_contract.py`：通过。
- `py tools\check_sensor_safe_protocol_contract.py`、`check_ltd_modbus_contract.py`、`check_dsm_compat_contract.py`、`check_si_protocol_contract.py`：通过，共享协议版本和相关外部映射一致。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，缺字0个。
- CPU2、CPU3 clean-first构建曾在协议16开发基线通过，分别生成 `LTD_MAIN_CPU2_V1.24.0.0.hex`、`LTD_DISPLAY_CPU3_V1.22.0.0.hex`；该结果只作为过渡阶段证据，不作为协议17发布产物。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`、`git diff --check`：通过。
- 实物故障注入和协议15/16交叉烧写拦截仍需台架验证。

### 协议版本 17

关联改动：
- 二代计量仪故障码按一代类别延续关系和二代新增责任域重新归档；当前CPU2共享故障为63项，CPU3同步共享定义并额外保留1项本机通信故障。
- 正式表共64个故障项目，其中61项标记“使用中”，仅保留 `12-6 编码器上电值变化`、`15-16 实高偏差过大`、`21-5 扭力传感器饱和` 3项未启用预留码。
- 从当前枚举移出12项低价值或被具体故障替代的项目：电机设置失败、电机复位失败、滑环校验错误、滑环丢包、传感器温度异常、传感器电压异常、密度不稳定、密度范围无效、AD5421故障管脚报警、未知错误、地址读取错误和电源波动。
- 这12项仅保留在 `LTD故障代码统一表.xlsx` 的“历史故障代码_CPU2V1.24.0.0”工作表中，不再进入当前CPU2/CPU3枚举、日志映射和CPU3显示映射。
- CPU3本机 `CPU2_COMM_TIMEOUT` 由 `0x000D000C`（屏幕 `13-12`）调整为 `0x00140007`（屏幕 `20-7`）；产生条件、连续失败门槛和恢复口径不变。

寄存器与参数存储影响：
- 不新增、不删除、不移动故障状态寄存器，错误码仍按32位值传递，高16位为类别、低16位为类别内编号。
- CPU2 `DeviceParameters` 结构大小、`DEVICE_PARAM_VERSION = 3`、元信息和CRC覆盖范围均不变；本次升级不会因为参数结构变化而恢复出厂参数。
- CPU3本机参数存储布局和版本不变。

兼容性影响：
- 协议版本17与16及更早版本的故障码数值解释不兼容，CPU2和CPU3必须使用相同 `DEVICE_PROTOCOL_VERSION`；外部主站、日志分析工具和资料应同步当前编号表。
- 现场选择共享故障码表时仍按CPU2程序版本，不按CPU2/CPU3共享协议版本选表；共享协议版本只用于CPU2与CPU3严格配套判断。
- CPU3本机故障不属于CPU2共享故障码表，应按CPU3程序版本单独确认。
- 首个正式使用协议17的固件为CPU2 `V1.25.0.0`、CPU3 `V1.23.0.0`。本次参数存储结构兼容且不会清参，因此按功能级变更提升次版本，不提升软件主版本。

验证状态：
- `check_fault_code_catalog_contract.py`、`check_cpu3_cpu2_comm_timeout_fault_contract.py`、`check_cpu3_fault_reason_visibility.py`、`check_sensor_fault_contract.py`、`check_parameter_measurement_fault_contract.py`、`check_tmc5130_diagnostic_contract.py`、`check_ad5421_diagnostic_contract.py` 全部通过。
- `check_sensor_safe_protocol_contract.py`、`check_ltd_modbus_contract.py`、`check_dsm_compat_contract.py`、`check_si_protocol_contract.py` 全部通过，故障码调整未改变相关外部协议地址和数据格式。
- `py LTD_DISPLAY_CPU3\font_check.py` 通过，显示缺字为0。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`、`cmake --build build\LTD_DISPLAY_CPU3 --clean-first` 通过；发布产物分别为CPU2 `V1.25.0.0`、CPU3 `V1.23.0.0`。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`、`git diff --check` 通过。
- 实物故障注入、协议16/17交叉烧写拦截和现场恢复路径仍需台架验证。

### 协议版本 18

关联改动：

- 新增独立 `SiProfileRuntime` 运行态，字段为 `phase`、`cycle_counter` 和 `progress_points`。阶段值固定为 `0=IDLE`、`1=PREPARING`、`2=MEASURING`、`3=RETURNING_LEVEL`、`4=COMPLETE`、`5=ABORTED`、`6=FAILED`。
- 三个字段追加在 AO 运行态之后、`REG_ENG` 之前；现有 Profile 点阵、无线、继电器、AO 及其它前序寄存器地址均不移动。
- CPU2 在 Point0 有效样本提交后最后递增 `cycle_counter`，并从 `progress_points=1` 开始按有效液体点递增；空气点、失败样本和重试不计数，不预设最终点数。
- CPU2 在 Point0 前保留上一轮已发布结果；Point0 后活动期以及取消/失败出口不发布半成品点阵。回液位成功、液位稳定且电机停止后，CPU2提交候选结果并最后递增原有完成计数。
- CPU3根据新运行态维护本地SI投影：完整分块读取候选点阵，复核周期、完成计数、点数、来源和阶段，再一次开放最终 `Complete/N/点阵/Profile报警` 组合；CPU2候选Complete不再直接穿透SI侧。
- CPU3首次启动或重连时不能把当前RTC冒充Point0时间；无法重建时，当前周期 `30007~30010` 返回0。若CPU2已处于同代SI COMPLETE，CPU3仍允许恢复最终N、点阵和报警，但时间保持0。
- CPU3已发布SI快照独立保存在本地投影中；后续普通分布、国标、每米、间隔或Wärtsilä分布不会清除或改写旧SI的Complete、N、时间、点阵和Profile报警。旧SI COMPLETE/ABORTED/FAILED只有在SI收尾上下文中才覆盖FC01终态线圈，不得遮住当前非SI动作。

SI外部兼容和存储影响：

- `30014=(FC01 byte1<<8)|FC01 byte0`；`30015=(FC02 byte1<<8)|FC02 byte0`；`30016=(FC02 byte3<<8)|FC02 byte2`。`30017~30020`继续固定为0。
- `40004~40009`作为CPU3本地原始 `uint16_t` 兼容槽，默认值依次为 `0、50、1000、0、5、1`；支持FC03连续读取和FC06单寄存器写入。这六个槽不参与测量、报警或控制。
- `40004~40023`统一使用CPU3本机参数事务写入口：只有FRAM写入及写后读回校验成功才向FC06正常回显；失败时恢复整份写前运行态、尽力恢复旧FRAM镜像，并返回设备故障 `0x06`，避免局部参数或运行态/持久化不一致。
- CPU3本机参数存储版本从 `V6 / 0x0006` 提升到 `V7 / 0x0007`。V6迁移使用显式旧结构和CRC校验，完整保留V6的SI自动调度、报警阈值和三路串口参数，只为六个兼容槽补默认值。
- CPU2 `DeviceParameters` 结构、`DEVICE_PARAM_VERSION = 3`、元信息和CRC范围不因这三个运行态字段变化；本次共享运行态扩展不会触发CPU2参数恢复出厂。

SI Profile外部生命周期：

1. `PREPARING`：Profile进入但Point0尚未建立，保留上一轮Complete、最终点数、Point0时间、点阵和Profile报警。
2. Point0有效样本：建立新周期，Complete清0，`30006=1`，锁存Point0时间；Point0位置为本轮实际采用的底部参考。
3. 活动采集：`30006`只按有效液体点单调递增，`30021~30620`全部为0。
4. 回液位：保持Profile=1、Complete=0，最终点数冻结但不提前发布点阵。
5. 最终发布：CPU3完成分块读取和代际复核，且满足AtLevel、Stable、电机停止和Interlock=0后，一次发布Profile=0、Auto=1、Stop=1、Complete=1、AtLevel=1、最终N、点阵和Profile报警。
6. Point0前显式取消保留上一轮结果；Point0后取消或真实失败均清Complete、N和点阵且不恢复旧Complete。显式取消不置Interlock，真实失败沿用既有错误/Interlock链路。

兼容性和发布门禁：

- 协议18与17及更早版本的输入寄存器总长度和SI生命周期契约不同，CPU2/CPU3必须成对使用协议18；严格相等检查会拦截混搭。
- 首个正式使用协议18的固件为CPU2 `V1.26.0.0`、CPU3 `V1.24.0.0`。CPU2参数存储版本保持3且结构大小不变；CPU3本机参数存储由V6升级为V7。协议18与17及更早版本不得交叉烧写或混用。
- 提交前审查确认3项P1、2项P2和2项既有问题，本次按用户决定只记录、不修改。详细失效机制见`SI协议适配/01_计划与需求/SI协议待确认与后续清单.md`第7节；其中V6→V7迁移写失败、CPU2快速重启和CPU3非整日周期重启属于优先修复项。

验证状态：

- 已实现并通过静态契约检查：`py tools\check_si_protocol_contract.py`、`py tools\check_si_modbus_frames.py --dump`、`py tools\check_wireless_rssi_contract.py`。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`通过，生成`LTD_MAIN_CPU2_V1.26.0.0.hex`，`text=297396 data=2336 bss=33864`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`通过，生成`LTD_DISPLAY_CPU3_V1.24.0.0.hex`，`text=167144 data=37396 bss=61764`。
- 尚未完成真实RS485台架、协议17/18交叉烧写拦截、V6真实FRAM镜像迁移及写失败注入、`40004~40023`掉电/故障注入、CPU2快速重启、CPU3跨日重启、200点PLC响应时延、完成边沿高频轮询、取消/失败、通信中断、非SI分布与已发布SI快照隔离以及1点、30点、200点边界验证。

## 后续维护要求

- 2026-06-10 CPU2 `V1.12.1.3` / CPU3 `V1.11.1.3` 同步 CPU2 系统参数出厂默认值和 CPU3 状态页显示参数矩阵，复用现有共享测量数据、设备状态和参数寄存器，不新增共享寄存器、命令码或参数语义，因此 `DEVICE_PROTOCOL_VERSION` 保持 `7`。
- 每次修改 CPU2/CPU3 共享寄存器地址、共享结构体容量、字段含义或跨 CPU 命令语义时，都要先判断是否需要提升协议版本。
- 只修改单 CPU 内部实现、不改变共享数据含义时，不提升协议版本。
- 新协议必须在本文追加记录，并说明旧协议默认行为。
