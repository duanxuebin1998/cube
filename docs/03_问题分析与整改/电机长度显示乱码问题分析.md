# 电机尺带长度异常显示问题整理

## 1. 问题背景

关联提交：`86918a0 修复电机状态显示和电流实时生效`。

现场现象是电机尺带长度偶发显示异常，表现为突然变成不合理的大数、0，或短时间跳变后恢复。结合代码链路看，核心风险不是显示格式本身，而是状态显示/运行期轮询过程中读取到异常 `XACTUAL` 后，把它当作真实电机位置写入了调试数据和电机记步位置。

初始风险链路如下：

```text
MotorCtrl_GetDisplayState() / MotorCtrl_PollRuntimePosition()
    -> MotorDriver_InferDisplayStateFromDriver()
    -> 读取 VACTUAL / XACTUAL / XTARGET 推断方向
    -> 某些运行期分支同步 MotorPosition_SyncDebugDrumState()
    -> 读取 XACTUAL 并换算 motor_distance
    -> 电机记步模式下更新 cable_length / sensor_position
    -> 尺带长度显示异常
```

`MotorPosition_SyncDebugDrumState()` 不能直接删除。它负责把 TMC5130 的 `XACTUAL` 同步成电机卷筒模型长度，并在电机记步模式下刷新业务位置；真正要控制的是调用时机和异常读数过滤。

## 2. 当前代码核对结果

### 2.1 已解决：底层 XACTUAL 稳定读取

位置：`LTD_MAIN_CPU2/Drivers/Peripherals/src/TMC5130.c`

当前已经增加了以下保护：

- `tmc5130_delayCsGuard()`：在 SPI 访问前、CS 拉低后、CS 拉高后加入短 NOP 延时，降低连续读时状态字混入数据区的概率。
- `tmc5130_readXactualStable()`：`XACTUAL` 要求连续两次读数在容差内才接受；前两次不稳定时补读第三次，仍不稳定则返回失败。
- `stpr_tryReadInt()`：读取 `TMC5130_XACTUAL` 时统一走稳定读取流程，其它寄存器保持原来的读取方式。
- 不稳定读数会打印 `TMC5130 XACTUAL连续读取不稳定`，并写入电机错误日志，便于现场回溯。

结论：底层“单次错位读数直接上抛”的问题已经处理。

### 2.2 已解决：上层 XACTUAL 异常跳变过滤

位置：`LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_position_model.c`

当前 `MotorPosition_TryUpdateDrumStateFromXactual()` 已统一调用 `MotorPosition_FilterXactualGlitch()`，主要保护包括：

- 非零缓存位置偶发读到 0 时会重读；若仍为 0，再用 `CHOPCONF` 判断是否疑似 SPI 全 0。
- 读数相对缓存位置发生超过 `MOTOR_XACTUAL_SUSPECT_JUMP_TICKS` 的大跳变时会重读确认。
- 重读恢复正常时采用重读值，并打印 `XACTUAL读数修正`。
- 重读仍不一致时丢弃本次读数，并打印 `XACTUAL读数丢弃`。
- 读取失败或判定异常时保留旧缓存，不再把异常值写入 `motor_step`、`motor_distance`、`cable_length` 或 FRAM 持久化快照。

结论：异常 `XACTUAL` 污染尺带长度缓存的问题已经明显收口。

### 2.3 已解决：显示状态查询不直接同步位置

位置：`LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_motion_api.c`

当前 `MotorCtrl_GetDisplayState()` 只做显示状态判断：

- 优先通过 `MotorDriver_TryReadMovingState()` 读取 `RAMPSTAT` 判断是否运动。
- 失败或需要方向时调用 `MotorDriver_InferDisplayStateFromDriver()` 推断上行/下行。
- 函数内没有直接调用 `MotorPosition_SyncDebugDrumState()`，也没有直接刷新 `cable_length`。

结论：原先“查询显示状态直接改尺带长度”的风险已经处理。

### 2.4 已缓解：运行期轮询仍会同步位置，但读数已受保护

位置：`LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_position_model.c`

当前 `MotorCtrl_PollRuntimePosition()` 仍会在以下场景同步位置：

- 运行中周期刷新位置。
- 停止状态从上行/下行切回停止时做收尾同步。
- `RAMPSTAT` 读取失败但根据驱动状态推断仍在运动时，同步位置或进入健康检查。

同步入口是 `MotorDriver_SyncPositionOrCheckHealth()`，内部先调用 `MotorPosition_SyncDebugDrumState()`；同步失败时再走驱动健康检查，用于区分 SPI 抖动、驱动掉电或复位。

结论：这部分不是纯显示路径，属于运行期位置维护，不能简单删除。由于底层稳定读取和上层跳变过滤已经加上，风险已缓解；但仍需要现场验证是否还有异常显示。

## 3. 目前已经解决的问题

1. `XACTUAL` 单次异常读数直接污染尺带长度：已通过底层多次稳定读取和上层跳变过滤处理。
2. SPI 连续读帧边界太紧导致状态字混入数据区：已通过 CS guard 延时缓解。
3. `MotorCtrl_GetDisplayState()` 查询显示状态时直接同步位置：当前代码中已不存在直接同步位置调用。
4. 读到疑似全 0 或大跳变时覆盖旧位置/持久化数据：当前会丢弃或重读确认，失败时保留旧缓存。
5. 排查日志缺失：已增加 `TMC5130 XACTUAL连续读取不稳定`、`XACTUAL读数丢弃`、`XACTUAL读数修正` 等现场日志。

## 4. 还需要做或验证的问题

1. 现场验证异常是否彻底消失。
   - 需要重点观察电机运行、停止瞬间、改速、位置源切换、驱动掉电/恢复等场景。
   - 如果仍出现异常，优先抓取 `TMC5130 XACTUAL连续读取不稳定`、`XACTUAL读数丢弃`、`XACTUAL读数修正`、驱动健康检查日志。

2. 继续评估 `MotorCtrl_PollRuntimePosition()` 的同步触发范围。
   - 当前运行期轮询仍会在 `RAMPSTAT` 读取失败但推断为运动时同步位置。
   - 如果现场仍能复现异常，可进一步收口该分支：先只更新显示状态或先做健康检查，再决定是否同步位置。

3. 评估 `MotorDriver_InferDisplayStateFromDriver()` 是否需要减少 `XACTUAL` 读取。
   - 当前逻辑是先看 `VACTUAL`，低速或刚启动时再用 `XACTUAL/XTARGET` 推断方向。
   - 由于 `XACTUAL` 已走稳定读取，当前风险较低；若现场日志显示状态查询仍触发大量不稳定读，可考虑只在确认运动时读取 `XACTUAL/XTARGET`，或用最近运动方向兜底。

4. 现场调参。
   - `TMC5130_XACTUAL_READ_TOLERANCE_TICKS` 当前为 `8192`。
   - `MOTOR_XACTUAL_SUSPECT_JUMP_TICKS` 当前按两圈步数判定可疑跳变。
   - 这两个阈值需要结合实际速度、轮径、最大轮询周期确认是否过严或过松。

5. 日志频率控制。
   - 当前诊断日志便于定位问题。
   - 如果现场频繁打印影响串口或实时性，需要在问题稳定后增加限频或条件开关。

## 5. 建议后续执行顺序

1. 先做现场复测，确认尺带长度是否还会跳到 0、大数或乱码。
2. 如果不再复现，保留当前保护逻辑，只评估是否需要降低日志频率。
3. 如果仍复现，先根据日志区分是 `XACTUAL` 不稳定、驱动健康异常，还是运行期同步时机过宽。
4. 若确认是同步时机问题，再收口 `MotorCtrl_PollRuntimePosition()` 中 `RAMPSTAT` 失败后的同步分支。
5. 若确认是误判阈值问题，再调整 `XACTUAL` 稳定读取容差和跳变过滤阈值。

## 6. 当前结论

本问题的主要代码风险已经处理：显示状态查询不再直接刷新尺带长度，`XACTUAL` 读取也已经增加了底层稳定读取和上层异常过滤。

剩余工作主要是现场验证和必要时收口运行期轮询分支。只有在现场仍能复现异常时，才建议继续改 `MotorCtrl_PollRuntimePosition()` 或 `MotorDriver_InferDisplayStateFromDriver()`，否则当前逻辑更适合保持不动，避免影响运动中位置刷新和停机收尾。
