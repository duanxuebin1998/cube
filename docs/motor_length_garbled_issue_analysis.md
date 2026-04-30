# 电机尺带长度异常显示问题分析

## 背景

问题关联提交：

```text
86918a0 修复电机状态显示和电流实时生效
```

该提交主要修改了电机状态显示、电流实时写入和运行状态轮询逻辑，涉及文件：

- `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl.c`
- `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl.h`
- `LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c`
- `LTD_MAIN_CPU2/Application/Src/wartsila_density_measurement.c`

现场现象：提交后电机尺带长度偶发显示异常值，表现为“乱码”或明显不合理的大数。

## 相关函数职责

### `Motor_SyncDebugDrumState()`

该函数不是单纯刷新显示状态，而是带有位置同步副作用。它会：

1. 读取 `TMC5130_XACTUAL`。
2. 将电机步数换算为卷筒圈数、角度和 `motor_distance`。
3. 写入 `g_measurement.debug_data.motor_step`。
4. 写入 `g_measurement.debug_data.motor_distance`。
5. 在电机计数模式下，通过 `Motor_UpdatePositionFromMotorSource()` 更新 `cable_length` 和 `sensor_position`。
6. 执行 TFIT 自动采样。
7. 触发位置寄存器持久化检查。

因此它可以用于“位置同步”，但不适合被普通“显示状态查询”无条件调用。

### `Motor_InferDisplayStateFromDriver()`

该函数在问题提交中新增，用于通过驱动寄存器推断电机显示状态。它会读取：

- `TMC5130_VACTUAL`
- `TMC5130_XACTUAL`
- `TMC5130_XTARGET`

它的目标是判断电机上行、下行或停止。但如果显示状态查询路径因此频繁触发 `XACTUAL` 读取，就会增加异常读数进入位置同步链路的概率。

### `motorPollRuntimePosition()`

问题提交后，该函数不再只在明确运动时同步位置，而是新增了若干“状态推断失败/停止状态修正”分支。这些分支中会调用 `Motor_SyncDebugDrumState()`。

这会导致一个风险：状态显示或轮询逻辑本来只应该确认电机是否运动，却可能顺手更新尺带长度。

## 初步定位

最可疑的引入点不是电流实时生效，而是“状态显示修复”中新增的驱动状态推断和位置同步调用。

风险链路如下：

```text
motorGetDisplayState() / motorPollRuntimePosition()
    -> Motor_TryReadMovingState()
    -> Motor_InferDisplayStateFromDriver()
    -> 读取 VACTUAL / XACTUAL / XTARGET
    -> 某些分支调用 Motor_SyncDebugDrumState()
    -> 读取 XACTUAL 并换算 motor_distance
    -> 电机计数模式下更新 cable_length
    -> 尺带长度显示异常
```

底层 `stpr_tryReadInt()` 当前对 `XACTUAL` 的读取依赖 TMC5130 两帧流水线读法。如果 SPI 读数偶发串帧、状态字串入数据或读到异常高位值，上层会把错误的 `XACTUAL` 当成真实电机位置。

问题提交提高了这些读寄存器路径的调用频率，因此更容易暴露这个底层读数保护不足的问题。

## `Motor_SyncDebugDrumState()` 是否可以删除

不建议直接删除。

原因：

- 电机计数模式下，`cable_length` 需要从 `XACTUAL` 推算。
- 运动过程中需要同步 `motor_step`、`motor_distance` 和业务层位置。
- 停止、切换位置源、位置持久化等流程也依赖该函数。

但它应该被限制在明确需要“同步电机位置”的路径中使用，不应该被普通显示状态查询函数随意调用。

建议原则：

- `motorGetDisplayState()` 应只返回显示状态，不应修改 `cable_length`。
- 状态推断函数不应触发位置同步。
- `motorPollRuntimePosition()` 只有在确认电机正在运动，且 `XACTUAL` 读数可信时才同步位置。
- 停止状态修正可以更新 `motor_state`，但不应为了显示静止而同步尺带长度。

## 建议验证方向

### 方向一：收敛位置同步调用点

检查并减少这些路径中的 `Motor_SyncDebugDrumState()` 调用：

- `motorGetDisplayState()`
- `motorPollRuntimePosition()` 中“仅用于显示状态修正”的分支
- `Motor_InferDisplayStateFromDriver()` 间接触发的位置同步路径

重点判断：调用该函数时是否真的需要更新 `cable_length`。

### 方向二：增强 `XACTUAL` 读取可信度

可以在底层 `TMC5130.c` 中增加保护：

- SPI CS 拉高/拉低前后增加短延时。
- 对 `TMC5130_XACTUAL` 做多次读取一致性判断。
- 如果多次读数差异超过容差，则返回失败，不更新旧位置。

当前工作区中存在一份未提交的实验性修改，属于这个方向。该修改已经通过编译，但还需要结合现场日志验证，不能直接视为最终结论。

### 方向三：增加现场定位日志

建议临时增加日志，区分异常来自哪里：

- `Motor_InferDisplayStateFromDriver()` 读取到的 `VACTUAL/XACTUAL/XTARGET`。
- `Motor_SyncDebugDrumState()` 准备写入的 `motor_step/motor_distance/cable_length`。
- `Motor_TryUpdateDrumState_FromXACTUAL()` 拒绝更新的位置读数。

日志要避免高频刷屏，可以只在读数跳变超过阈值时打印。

## 暂定结论

`Motor_SyncDebugDrumState()` 本身有必要保留，但问题提交让它进入了状态显示相关路径，导致“显示状态查询”可能间接更新尺带长度。

修复思路不应是简单删除该函数，而是拆清职责：

- 显示状态查询只读状态，不改位置。
- 位置同步只在运动轮询、停止确认、位置源切换等明确场景执行。
- `XACTUAL` 读数异常时保持旧位置，不覆盖 `cable_length`。
