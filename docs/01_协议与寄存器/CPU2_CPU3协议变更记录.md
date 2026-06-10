# CPU2/CPU3 协议变更记录

本文记录 CPU2 与 CPU3 之间共享寄存器、共享结构体和能力字段的协议版本。固件版本只表示各 CPU 自身软件版本，不能单独作为跨 CPU 兼容依据。

## 协议版本字段

- 字段位置：`HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION`
- 当前语义：CPU2/CPU3 共享协议版本
- 旧程序语义：保留字段，默认值为 `0`
- 当前程序语义：协议版本 `7`

该字段由原 `reserved1` 预留位正式替换而来，寄存器地址不移动，不新增存储字段。

## 版本记录

| 协议版本 | 首个 CPU2 版本 | 首个 CPU3 版本 | 密度分布最大点数 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 旧程序，或未写入协议版本字段 | 旧程序 | 100 | 原 `reserved1` 未定义协议语义；当前CPU3读到0时提示协议不兼容。 |
| 1 | V1.5.0.0 | V1.2.0.0 | 200 | 原 `reserved1` 正式替换为协议版本，密度分布测量、内部输入寄存器和 Wärtsilä 外部密度点扩展到 200 点。 |
| 2 | V1.6.0.0 | V1.4.0.0 | 200 | 新增 `CMD_CANCEL_MEASUREMENT = 16`，用于 CPU3 状态显示界面长按返回键取消当前测量并让 CPU2 进入待机。 |
| 3 | V1.7.0.0 | V1.5.0.0 | 200 | 原 `reserved23` 正式替换为探底修正罐高，用于罐底测量后编码器修正；瓦锡兰分布测后探底前先回固定点监测位置。 |
| 4 | V1.8.0.0 | V1.6.0.0 | 200 | 将 SI7000 所需补充状态融合进既有测量结构，并通过共享输入寄存器发布给 CPU3 外部协议转换层。 |
| 5 | V1.9.0.0 | V1.7.0.0 | 200 | 新增 `CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117`，用于 CPU3 菜单或共享命令通道触发 CPU2 执行无线滑环 RSSI 最近匹配；新增无线滑环匹配中/完成设备状态；输入寄存器末尾追加无线滑环匹配结果和从机 MAC 状态。 |
| 6 | V1.10.0.0 | V1.9.0.0 | 200 | 新增 `STATE_DEBUG_MODE = 0x0033`，用于 CPU2 串口调试指令执行期间通过 CPU3 显示“调试模式中”；原 `reserved2` 参数槽复用为故障自动恢复重跑上限；`empty_weight` 空载称重按 `int32_t` 有符号 32 位解释，寄存器地址和后续字段不移动。 |
| 7 | V1.12.0.0 | V1.10.0.0 | 200 | 新增四路继电器报警输出配置和运行态共享区；CPU3 可显示、写入四路继电器报警输出配置，CPU2 执行 HH/H/L/LL、滞回、锁存清除和无效值策略。 |

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
<- 将 SI7000 所需补充状态按业务含义融合到 CPU2/CPU3 既有测量结构：`DeviceStatus`、`OilMeasurement`、`ActualHeightMeasurement` 和 `DensityDistribution`。
- CPU2 发布探底参考有效、探头位于液位、液位稳定、profile 完成锁存、profile 完成计数、profile 被工况阻止、装卸液状态、手动报警抑制、手动液位更新抑制、profile 温度偏差报警、profile 密度偏差报警。
- CPU3 读取该状态区后供外部协议转换层使用，SI7000 地址、线圈、缩放和异常响应只存在于 CPU3 外部协议模块，避免外部协议直接反推或污染 CPU2 内部流程状态。
- 没有新增独立 `ProtocolAssistStatus` 结构，避免在 `MeasurementResult` 里再维护一套外部协议专用镜像；所有字段都落在原业务结构尾部，并由 CPU2/CPU3 两侧同名结构同步。

新增共享字段明细：

| 所属结构 | 新增字段 | 共享寄存器 | 语义 |
| --- | --- | --- | --- |
| `DeviceStatus` | `loading_unloading_active` | `REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE` | 当前是否处于装卸液过程，供外部协议判断工况。 |
| `DeviceStatus` | `manual_alarm_inhibit` | `REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT` | 手动/强制动作期间抑制自动报警语义，避免 CPU3 将手动过程误判为自动测量结果。 |
| `OilMeasurement` | `probe_at_liquid_level` | `REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL` | 找液位成功后置位，SI7000 可映射为 Probe At Liquid Level。 |
| `OilMeasurement` | `liquid_stable` | `REG_OIL_MEASUREMENT_LIQUID_STABLE` | 找液位成功后置位，表示当前液位结果可认为稳定。 |
| `OilMeasurement` | `manual_level_update_inhibit` | `REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT` | 手动/强制动作期间抑制液位自动更新语义。 |
| `ActualHeightMeasurement` | `bottom_reference_valid` | `REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID` | 探底成功或使用有效回退罐高后置位，SI7000 可映射为 Bottom Reference。 |
| `DensityDistribution` | `profile_complete_latched` | `REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED` | 分布测量成功后锁存完成状态，失败或命令切换不置位。 |
| `DensityDistribution` | `profile_complete_counter` | `REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER` | 每次 profile 成功完成后递增，CPU3 用计数变化锁存 SI7000 profile 时间戳。 |
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
- CPU2/CPU3 必须同为协议版本 4 才能正确同步融合后的 SI7000 状态字段。
- 旧 CPU3 不识别新增状态寄存器，不能完整支持 SI7000 的离散输入状态映射。
- 旧 CPU2 不发布这些新增状态字段，CPU3 V1.6.0.0 不能依赖旧协议数据生成完整 SI7000 状态。
- 由于多个分组尾部寄存器顺延，协议版本 4 与协议版本 3 不能混用；必须依赖 `DEVICE_PROTOCOL_VERSION` 严格相等检查拦截。

验证结果：
- `py tools\check_si7000_protocol_contract.py`：确认 CPU2/CPU3 的结构字段、寄存器宏和读写打包顺序一致。
- `py tools\check_si7000_modbus_frames.py`：确认 SI7000 外部地址常量和 golden frame 一致。
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
- 协议版本 4 仅包含 SI7000 辅助状态，不包含命令 117 和 `WirelessPairingStatus`；CPU3 V1.7.0.0 不应与协议版本 4 的 CPU2 混用。
- 旧 CPU3 不提供该菜单入口，但不影响 CPU2 新版本通过调试串口 `SPR` 执行匹配。
- 旧协议 CPU3 不知道追加的 `WirelessPairingStatus` 字段；协议版本不匹配时不应继续解释匹配结果。

### 协议版本 6

关联改动：
- `DeviceState` 新增 `STATE_DEBUG_MODE = 0x0033`，CPU2 与 CPU3 枚举值保持一致。
- CPU2 串口调试命令中，除调用正式测量流程的 `F/H/J`、正式命令映射、无线滑环 `SP*` 和演示 `X` 之外，执行期间临时发布 `STATE_DEBUG_MODE`。
- CPU2 串口 `B/BE` 诊断进入时切换到 `STATE_DEBUG_MODE`，退出时恢复进入前的业务状态和错误码快照；诊断期间清除临时错误时仍保持调试模式显示。
- 原 `reserved2` 正式替换为 `fault_auto_recovery_retry_limit`，用于控制故障自动恢复确认成功后最多自动重跑原命令次数：`0` 关闭，`1~10` 为上限，默认 `3`。
- `empty_weight` 空载称重按 `int32_t` 有符号 32 位参数解释，寄存器仍占 2 个 word，负值按二进制补码传输。
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

## 后续维护要求

- 2026-06-10 CPU2 `V1.12.1.3` / CPU3 `V1.11.1.3` 同步 CPU2 系统参数出厂默认值和 CPU3 状态页显示参数矩阵，复用现有共享测量数据、设备状态和参数寄存器，不新增共享寄存器、命令码或参数语义，因此 `DEVICE_PROTOCOL_VERSION` 保持 `7`。
- 每次修改 CPU2/CPU3 共享寄存器地址、共享结构体容量、字段含义或跨 CPU 命令语义时，都要先判断是否需要提升协议版本。
- 只修改单 CPU 内部实现、不改变共享数据含义时，不提升协议版本。
- 新协议必须在本文追加记录，并说明旧协议默认行为。
