/**
 * @file measure_commands_internal.h
 * @brief measurement 目录内部命令处理接口，不属于 Application 公共契约。
 */

#ifndef APPLICATION_MEASUREMENT_MEASURE_INTERNAL_H
/* 包含保护：防止模块私有接口重复声明。 */
#define APPLICATION_MEASUREMENT_MEASURE_INTERNAL_H

/* 命令处理函数：由 ProcessMeasureCmd 完成统一初始化后调用。 */
/* 取消当前测量，停止电机并恢复可接收命令状态。 */
void CMD_CancelMeasurement(void);
/* 标定罐高：先测出原始实高，再用标定罐高值修正“当前实高”显示链路。 */
void CMD_CalibrateTankHeight(void);
/* 执行单次水位搜索命令：初始化测量并发布寻找状态，运行 SearchWaterLevel 后统一写入错误码和完成态。 */
void CMD_MeasurWater(void);
/* 水位跟随主函数（命令入口）。 */
void CMD_FollowWaterLevel(void);
/* 执行零点测量命令并统一发布完成或故障状态。 */
void CMD_MeasureZero(void);
/* 启动测量并搜索机械零点，成功后在零点标定电机首圈尺带周长。 */
void CMD_CalibrateZeroPoint(void);
/* 启动罐底搜索流程，并按搜索结果发布完成、命令切换或故障状态。 */
void CMD_MeasureBottom(void);
/* 按需回零并搜索液位；切换到电机记步后重新定位，完成 SI Profile 回液位发布，再进入持续液位跟随。 */
void CMD_MeasureAndFollowOilLevel(void);
/* 启动液位标定；跟随态直接修正后继续跟随，其他状态先探底再重新搜索液位。 */
void CMD_CalibrateOilLevel(void);
/* 液位跟随中直接应用液位修正并恢复跟随；非跟随态转入液位标定流程。 */
void CMD_CorrectOilLevel(void);
/* 进入维护模式并停止当前自动测量动作。 */
void CMD_EnterMaintenanceMode(void);
/* 退出非阻塞维护模式。 */
void CMD_ExitMaintenanceMode(void);
/* 请求清除四路继电器锁存报警。 */
void CMD_ClearAllRelayLatchedAlarms(void);
/* 按命令参数指定的距离和默认速度执行手动上行，并在动作期间抑制自动报警与液位更新。 */
void CMD_MoveUp(void);
/* 按命令参数指定的距离和默认速度执行手动下行，并在动作期间抑制自动报警与液位更新。 */
void CMD_MoveDown(void);
/* 电机强制上行指令（无检测）。 */
void CMD_ForceMoveUp(void);
/* 电机强制下行指令（无检测）。 */
void CMD_ForceMoveDown(void);
/* 切换至空载扭力采集状态，完成稳定等待、范围校验和参数保存后发布完成状态。 */
void CMD_SetEmptyWeight(void);
/* 切换至满载扭力采集状态，完成稳定等待、范围校验和参数保存后发布完成状态。 */
void CMD_SetFullWeight(void);
/* 运行到指定绝对位置（mm） */
void CMD_RunToPosition(void);
/* 执行并发布瓦锡兰点阵，保留结果供 CPU3 读取，并按配置周期探底后恢复固定点监测。 */
void CMD_WartsilaDensitySpread(void);
/* 搜索液位并切换密度模式，按配置执行分布测量，随后发布与实际内核一致的点阵来源。 */
void CMD_SyntheticMeasurement(void);
/* 执行普通密度分布测量，并仅在完整成功后打印和发布新的标准点阵。 */
void CMD_MeasureDensitySpread_Spread(void);
/* 执行国标密度分布测量；仅在完整成功后发布点阵和完成态。 */
void CMD_MeasureDensitySpread_GB(void);
/* 执行每米密度分布测量；仅在完整成功后发布点阵和完成态。 */
void CMD_MeasureDensitySpread_Meter(void);
/* 执行区间密度分布测量；仅在完整成功后发布点阵和完成态。 */
void CMD_MeasureDensitySpread_Interval(void);
/* 执行 SI 独立 profile 测量。 */
void CMD_SiProfile(void);
/* 单点测量命令：移动到指定高度 -> 单点稳定读取。 */
void CMD_SinglePointMeasurement(void);
/* 单点监测命令：移动到监测高度 -> 循环单点稳定读取（直到命令切换）。 */
void CMD_SinglePointMonitoring(void);
/* 按用户给出的水位真值修正水罐高度；非跟随态先重找水位，跟随态修正后恢复原跟随模式。 */
void CMD_CalibrateWaterLevel(void);

#endif /* APPLICATION_MEASUREMENT_MEASURE_INTERNAL_H */
