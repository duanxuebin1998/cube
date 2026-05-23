# CPU2暂存区改动整理

本文整理当前暂存区中 `LTD_MAIN_CPU2` 目录下的全部改动。范围只覆盖 CPU2 侧文件，不包含 CPU3、公共文档、工具脚本和 CHANGELOG 的其他改动。

## 1. 改动概览

本次 CPU2 改动共涉及 7 个文件：

| 文件 | 暂存改动规模 | 主要目的 |
| --- | ---: | --- |
| `LTD_MAIN_CPU2/Application/Inc/app_version.h` | 3 行新增 / 3 行删除 | CPU2 固件版本从 `V1.7.3.0` 升级到 `V1.8.0.0`。 |
| `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h` | 多处结构字段调整 | 协议版本升到 4，并把 SI7000 需要的补充状态融合进既有测量结构。 |
| `LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h` | 多处寄存器地址调整 | 按设备状态、液位、实高和密度分布分配共享输入寄存器地址。 |
| `LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c` | 31 行新增 | 将协议辅助状态写入/回读共享输入寄存器。 |
| `LTD_MAIN_CPU2/Application/Src/measure.c` | 87 行新增 / 11 行删除 | 在测量入口、探底、维护模式和手动/强制电机动作中维护协议辅助状态。 |
| `LTD_MAIN_CPU2/Application/Src/measure_density.c` | 40 行新增 | 在分布/profile 测量生命周期中维护完成锁存和完成计数。 |
| `LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c` | 15 行新增 | 在找液位流程中维护探头到达液位和液体稳定状态。 |

整体目标是让 CPU2 只发布通用、可复用的测量生命周期状态，CPU3 再把这些状态转换成 SI7000 外部协议的线圈、离散输入和输入寄存器语义。CPU2 不引入 SI7000 地址、功能码、缩放规则或异常响应逻辑。

## 2. 协议和版本改动

### 2.1 CPU2固件版本

`LTD_MAIN_CPU2/Application/Inc/app_version.h` 将 CPU2 版本从：

- `CPU2_APP_VERSION_MINOR`: `7` -> `8`
- `CPU2_APP_VERSION_PATCH`: `3` -> `0`
- `CPU2_APP_VERSION_STRING`: `"V1.7.3.0"` -> `"V1.8.0.0"`

这次属于 CPU2/CPU3 共享协议扩展和外部协议适配支撑改动，版本级别按 minor 升级处理。

### 2.2 共享协议版本

`LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h` 将：

- `DEVICE_PROTOCOL_VERSION`: `3u` -> `4u`

原因是 `MeasurementResult` 的既有子结构新增了 SI7000 需要的补充状态字段，CPU2/CPU3 共享输入寄存器布局发生变化。CPU3 必须按协议版本 4 读取对应地址，否则后续测量数据会错位。

### 2.3 状态字段融合方式

本轮不再新增独立 `ProtocolAssistStatus` 结构体，而是把补充状态按业务含义融合进原有变量。

字段如下：

| 字段 | 类型 | CPU2侧语义 | CPU3/SI7000用途 |
| --- | --- | --- | --- |
| `bottom_reference_valid` | `uint32_t` | 已经获得有效罐底参考。 | 映射 SI7000 Bottom Reference 状态。 |
| `probe_at_liquid_level` | `uint32_t` | 探头当前已经到达有效液位点。 | 映射 Probe At Liquid Level。 |
| `liquid_stable` | `uint32_t` | 液位结果当前可视为稳定。 | 支撑外部协议液位稳定/间隔状态判断。 |
| `profile_complete_latched` | `uint32_t` | 本轮分布/profile 测量结果已完成并锁存。 | 控制 CPU3 是否向 PLC 开放 profile 点阵。 |
| `profile_complete_counter` | `uint32_t` | 每次 profile 完成后递增。 | CPU3 用计数变化锁存 profile 完成时间戳。 |
| `profile_blocked_by_process` | `uint32_t` | profile 被工况阻止。 | 预留给 SI7000 工况阻止或流程不可用状态。 |
| `loading_unloading_active` | `uint32_t` | 当前处于装卸液过程。 | 预留给 SI7000 装卸液相关状态。 |
| `manual_alarm_inhibit` | `uint32_t` | 手动/强制动作期间抑制自动报警语义。 | 避免 CPU3 把手动动作误映射成自动测量报警。 |
| `manual_level_update_inhibit` | `uint32_t` | 手动/强制动作期间抑制液位自动更新语义。 | 避免外部协议在手动运行中误读自动液位更新。 |
| `profile_temp_deviation_alarm` | `uint32_t` | profile 温度偏差报警。 | 映射 SI7000 profile 温度偏差报警位。 |
| `profile_density_deviation_alarm` | `uint32_t` | profile 密度偏差报警。 | 映射 SI7000 profile 密度偏差报警位。 |

当前暂存改动已经维护了部分字段的运行期赋值；`profile_blocked_by_process`、`loading_unloading_active`、`profile_temp_deviation_alarm`、`profile_density_deviation_alarm` 主要作为后续扩展或由其他流程继续补齐。

## 3. 共享寄存器布局改动

`LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h` 按业务结构新增 11 个共享输入寄存器地址：

1. `REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE`
2. `REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT`
3. `REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL`
4. `REG_OIL_MEASUREMENT_LIQUID_STABLE`
5. `REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT`
6. `REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID`
7. `REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED`
8. `REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER`
9. `REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS`
10. `REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM`
11. `REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM`

每个字段按 `REG_SIZE_U32` 占用两个 16 位 Modbus 输入寄存器。新增后，后续输入寄存器地址会随对应业务段自然后移。

这个变化会影响 CPU2/CPU3 共享输入寄存器布局，因此必须和 CPU3 侧 `stateformodbus.h`、读写顺序和 `DEVICE_PROTOCOL_VERSION` 保持完全一致。

## 4. 共享寄存器读写改动

`LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c` 在两个路径中加入 `SI7000 状态字段`：

### 4.1 写入输入寄存器

`write_measurement_result_to_InputRegisters()` 在写完 `height_measurement` 后，将 11 个 既有测量子结构状态字段 字段按固定顺序写入 `REG_DEVICE_STATUS_*` / `REG_OIL_MEASUREMENT_*` / `REG_HEIGHT_MEASUREMENT_*` / `REG_DENSITY_DIST_*` 地址。

关键约束：

- 写入顺序必须和 CPU3 侧读取顺序一致。
- 每个字段以 `u32` 形式写入，保持和 `MeasurementResult` 结构体字段类型一致。
- 该状态区位于单点测量数据之前，因此单点测量寄存器整体后移。

### 4.2 回读输入寄存器

`read_measurement_result_from_InputRegisters()` 按相同顺序从 `REG_DEVICE_STATUS_*` / `REG_OIL_MEASUREMENT_*` / `REG_HEIGHT_MEASUREMENT_*` / `REG_DENSITY_DIST_*` 地址读回到 `g_measurement` 既有子结构状态字段。

这条路径主要用于调试、镜像或本地寄存器恢复场景，目的是避免 CPU2/CPU3 对照排查时出现字段错位。

## 5. 测量主流程改动

`LTD_MAIN_CPU2/Application/Src/measure.c` 负责在通用测量入口、探底和手动动作中维护协议辅助状态。

### 5.1 MeasureStart入口清理

`MeasureStart()` 在清除故障码后，新增清理以下状态：

- `probe_at_liquid_level = 0`
- `liquid_stable = 0`
- `profile_blocked_by_process = 0`
- `manual_alarm_inhibit = 0`
- `manual_level_update_inhibit = 0`
- `profile_temp_deviation_alarm = 0`
- `profile_density_deviation_alarm = 0`

目的：每个新测量命令开始时重新计算外部协议辅助状态，避免 CPU3/SI7000 读到上一轮测量残留。

注意：当前入口没有清理 `bottom_reference_valid` 和 `profile_complete_latched/profile_complete_counter`。这表示罐底参考和 profile 结果锁存具备跨命令保留语义，是否符合现场期望需要在联调时确认。

### 5.2 探底成功置位罐底参考

`CMD_MeasureBottom()` 在两个成功路径中新增：

- 快速返回成功路径：置位 `bottom_reference_valid = 1`
- 普通探底完成且 `ret == NO_ERROR`：置位 `bottom_reference_valid = 1`

目的：CPU3 可据此映射 SI7000 Bottom Reference 状态，表示 CPU2 当前已有有效罐底参考。

### 5.3 维护模式改动

`CMD_EnterMaintenanceMode()` 从原来的无限循环 `CHECK_COMMAND_SWITCH_NO_RETURN()` 改为显式检查 `HasEffectiveCommandSwitchRequest()`。

新增行为：

- 进入维护模式时置位：
  - `manual_alarm_inhibit = 1`
  - `manual_level_update_inhibit = 1`
- 检测到命令切换时打印提示，清除两个抑制位并返回。

目的：维护模式作为 SI7000 Manual/Stop 的保守映射，手动期间抑制自动报警和液位自动更新语义；退出时必须恢复，避免 CPU3 长时间保持手动抑制状态。

行为影响：原逻辑依赖 `CHECK_COMMAND_SWITCH_NO_RETURN()` 在循环内处理命令切换；新逻辑改为函数返回。需要确认调用链对维护模式返回后的状态处理符合预期。

### 5.4 手动上行和下行

`CMD_MoveUp()`、`CMD_MoveDown()` 新增运行期抑制位维护：

- 动作开始前置位：
  - `manual_alarm_inhibit = 1`
  - `manual_level_update_inhibit = 1`
- 如果 `ret == STATE_SWITCH`，先清除抑制位再返回。
- 如果 `ret != NO_ERROR`，先清除抑制位再 `SET_ERROR(ret)`。
- 正常完成后清除抑制位，再置完成状态。

目的：手动上行/下行不应被外部协议误判成自动测量动作，也不应在动作期间触发自动报警和液位更新语义。

行为影响：原逻辑无条件 `SET_ERROR(ret)`；新逻辑对 `STATE_SWITCH` 和 `NO_ERROR` 做了显式分支。需要确认 `SET_ERROR(NO_ERROR)` 原本是否有副作用；如果没有副作用，新逻辑更清晰。如果原宏承担额外状态更新，则需要复核。

### 5.5 强制上行和强制下行

`CMD_ForceMoveUp()`、`CMD_ForceMoveDown()` 同样新增手动抑制位维护：

- 强制动作开始前置位手动抑制。
- `STATE_SWITCH` 时清除抑制位并返回。
- 非 `NO_ERROR` 时清除抑制位并进入 `SET_ERROR(ret)`。
- 正常完成后清除抑制位。

目的：强制运行无检测，外部协议侧只能获得“当前为手动/强制运行，不应更新自动液位或报警”的通用语义，不能把它当成有效自动测量结果。

### 5.6 注释修正

`CMD_ForceLiftZero()` 附近注释从“强制提零点：长距离上行（无检测称重/丢步）”调整为“强制回零点：只控制电机上行，无检测重量/液位等”。

该项不改变行为，只让注释更贴近实际动作含义。

## 6. 分布/profile测量改动

`LTD_MAIN_CPU2/Application/Src/measure_density.c` 修改了 4 个分布测量入口：

- `CMD_MeasureDensitySpread_Spread()`
- `CMD_MeasureDensitySpread_GB()`
- `CMD_MeasureDensitySpread_Meter()`
- `CMD_MeasureDensitySpread_Interval()`

每个入口在开始测量前清除：

- `profile_complete_latched = 0`
- `profile_blocked_by_process = 0`
- `profile_temp_deviation_alarm = 0`
- `profile_density_deviation_alarm = 0`

每个入口在测量成功、写入 `g_measurement.density_distribution = temp` 并打印结果后设置：

- `profile_complete_latched = 1`
- `profile_complete_counter++`

运行期语义：

- `profile_complete_latched` 表示当前 profile 结果已经完成并可被 CPU3 读取。
- `profile_complete_counter` 用于区分不同完成事件，CPU3 可在计数变化时锁存 profile 完成时间戳。
- 只有成功走到结果写入后的路径才置位完成锁存；中途失败不会把旧 profile 当成新结果开放。

需要注意：

- `profile_complete_counter` 是 `uint32_t`，长时间运行后会自然回绕。CPU3 侧应按“是否变化”而不是“是否递增大于旧值”来判断新完成事件。
- 当前暂存 diff 未在失败路径显式清理 `profile_complete_latched` 之外的结果数据，CPU3 需要继续以 `profile_complete_latched` 作为开放 profile 点阵的门控。

## 7. 找液位流程改动

`LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c` 在找液位流程中维护液位命中和稳定状态。

### 7.1 SearchOilLevel开始清理

`SearchOilLevel()` 开始时新增：

- `probe_at_liquid_level = 0`
- `liquid_stable = 0`

目的：每次找液位开始时先清除上一轮命中状态，防止 CPU3 读到旧液位稳定结果。

### 7.2 找液位成功置位

找液位成功并打印液位结果后新增：

- `probe_at_liquid_level = 1`
- `liquid_stable = 1`

目的：只有确认得到有效液位后，外部协议才可认为探头处于液位点且液体稳定。

### 7.3 上下限和盲区失败路径清理

`determineTheSensorPositionAndUpdateTheLevelValue()` 中：

- 到达位置上限时清除 `probe_at_liquid_level` 和 `liquid_stable`
- 处于盲区内下限时清除 `probe_at_liquid_level` 和 `liquid_stable`

目的：上下限和盲区不是有效液位点，不能让 CPU3/SI7000 误判为液位命中。

### 7.4 等待脱离盲区期间清理

`waitForTheLiquidLevelToExceedTheBlindZone()` 进入等待前清除：

- `probe_at_liquid_level = 0`
- `liquid_stable = 0`

目的：等待脱离盲区时液位尚未确认，外部协议侧应保持未命中和不稳定。

## 8. CPU2职责边界

本次暂存区中的 CPU2 改动遵循以下边界：

- CPU2 只发布通用状态，不发布 SI7000 专用地址语义。
- CPU2 不处理 SI7000 Modbus 功能码、CRC、异常码、线圈互斥和寄存器缩放。
- CPU2 不持久化 SI7000 影子寄存器或报警阈值。
- CPU2 现有测量命令、状态机和参数存储只做最小必要补充。
- CPU3 负责把融合后的测量状态字段映射到 SI7000 的 `000xx`、`100xx`、`300xx`、`400xx` 语义。

这样的分层可以避免外部 PLC 协议细节反向污染 CPU2 测量流程。

## 9. 兼容性影响

### 9.1 CPU2/CPU3必须协议匹配

由于新增共享输入寄存器区并后移后续测量寄存器，CPU2 协议版本 4 必须和 CPU3 协议版本 4 配套使用。

不兼容组合风险：

- CPU2 V1.8.0.0 + 旧 CPU3：旧 CPU3 会按旧地址读取后续单点/分布测量数据，可能字段错位。
- 旧 CPU2 + CPU3 V1.6.0.0：CPU3 读不到融合后的新增状态字段，SI7000 状态位无法完整映射，且协议检查应提示不兼容。

### 9.2 外部协议行为影响

CPU2 新增状态会影响 CPU3 对 SI7000 的外部表现：

- 探底成功后才有 Bottom Reference。
- 找液位成功后才有 Probe At Liquid Level 和 Liquid Stable。
- profile 完成后才开放新的 profile 完成状态和时间戳触发条件。
- 手动/强制运行期间，外部协议可识别报警和液位更新应被抑制。

### 9.3 原有测量流程影响

大部分改动只写 `g_measurement` 既有子结构中的状态字段，不直接改变原测量算法。但以下点需要重点回归：

- 维护模式从宏循环改为检测命令切换后返回。
- 手动/强制电机动作对 `STATE_SWITCH` 和错误返回的处理分支更明确。
- `SET_ERROR(ret)` 不再对 `NO_ERROR` 无条件调用。

## 10. 建议验证

### 10.1 静态一致性

建议至少运行：

```powershell
py tools\check_si7000_protocol_contract.py
py tools\check_version_bumped.py
git diff --cached --check
```

检查重点：

- CPU2/CPU3 `DEVICE_PROTOCOL_VERSION` 一致。
- CPU2/CPU3 新增状态字段的归属结构一致。
- CPU2/CPU3 `REG_DEVICE_STATUS_*`、`REG_OIL_MEASUREMENT_*`、`REG_HEIGHT_MEASUREMENT_*`、`REG_DENSITY_DIST_*` 地址顺序一致。
- `dataanalysis_modbus.c` 写入/读取顺序一致。

### 10.2 编译验证

建议运行：

```powershell
cmake --build build\LTD_MAIN_CPU2
```

如果 CPU3 同步改动也在同一提交中，应同时运行：

```powershell
cmake --build build\LTD_DISPLAY_CPU3
```

### 10.3 运行期回归

建议联调或仿真覆盖：

1. 探底成功后，CPU3 读到 `bottom_reference_valid = 1`。
2. 找液位开始时，`probe_at_liquid_level/liquid_stable` 清零。
3. 找液位成功后，`probe_at_liquid_level/liquid_stable` 置 1。
4. 到达上限、下限、盲区等待期间，液位命中和稳定状态保持 0。
5. 四类分布测量开始时 `profile_complete_latched` 清零。
6. 四类分布测量成功后 `profile_complete_latched = 1` 且 `profile_complete_counter` 递增。
7. 维护模式、手动上/下、强制上/下运行期间手动抑制位为 1，退出后恢复 0。
8. 命令切换 `STATE_SWITCH` 路径不会残留手动抑制位。

## 11. 风险和后续事项

1. 新增状态字段已融合进 `MeasurementResult` 的既有子结构，仍属于共享协议布局变化；提交时必须和 CPU3 对应改动、协议版本、协议变更记录放在同一提交。
2. CPU2 当前只维护部分辅助状态，装卸液、profile 被工况阻止、profile 温度/密度偏差报警的真实来源还需要后续按业务流程补齐。
3. `bottom_reference_valid` 当前探底成功后置 1，但没有在新测量入口清零；如果现场需要“每轮探底重新确认”，应增加清零策略。
4. `profile_complete_counter` 回绕需要 CPU3 侧按变化判断，不能按单调递增判断。
5. 维护模式返回路径发生变化，建议重点检查命令调度层对 `CMD_EnterMaintenanceMode()` 返回后的处理是否和旧宏行为一致。
6. 手动/强制动作不再无条件调用 `SET_ERROR(NO_ERROR)`，如果 `SET_ERROR` 宏在成功路径有隐藏副作用，需要进一步确认。
