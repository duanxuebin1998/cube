/*
 * motor_ctrl.h
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
 *
 * 说明：
 *  - 步进电机控制模块（TMC5130）
 *  - 支持：精确长度换算下发、阻塞等待、碰撞/异常检测、丢步检测、卷筒圈数/角度计算、第一圈周长标定
 *
 * 关键口径（必须全工程一致）：
 *  1) 方向定义（重要）：
 *     - MOTOR_DIRECTION_DOWN = 0：下行放带，cable_length 增加
 *     - MOTOR_DIRECTION_UP   = 1：上行收带，cable_length 减少
 *
 *  2) 单位约定（与 g_measurement/g_deviceParams 保持一致）：
 *     - cable_length：0.1mm
 *     - sensor_position：0.1mm
 *     - first_loop_circumference_mm：0.1mm
 *     - tape_thickness_mm：0.001mm
 */

#ifndef INC_MOTOR_CTRL_H_
#define INC_MOTOR_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include "TMC5130.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 使用说明（建议先看） ===================== */

/*
 * 1) motorSetSpeed(speed_x100)
 *    持久修改当前速度设定值；后续运动会继续沿用该速度。
 *
 * 2) xxxWithSpeed(..., speed_x100)
 *    为本次命令附带速度参数。
 *    - 阻塞型接口：命令结束后自动恢复到 g_deviceParams.max_motor_speed
 *    - 非阻塞型接口：函数返回时电机可能仍在运行，因此不会自动恢复
 *
 * 3) 推荐优先使用的上层接口：
 *    - 按距离阻塞移动：motorMoveAndWaitUntilStop()
 *    - 按绝对位置移动：motorMoveToPositionOneShot()
 *    - 按条件停止：motorMoveUntilCondition()
 *    - 直接改速：motorSetSpeed()
 */

/* ===================== 方向定义 ===================== */

/**
 * @brief 电机方向：下行（放带）
 * 约定：下行时 cable_length 增加
 */
#define MOTOR_DIRECTION_DOWN   0

/**
 * @brief 电机方向：上行（收带）
 * 约定：上行时 cable_length 减少
 */
#define MOTOR_DIRECTION_UP     1

/* ===================== 全局变量 ===================== */

/**
 * @brief 当前电机运行速度参数（由 motor_ctrl.c 定义）
 * 说明：该速度最终传入 stpr_moveBy()/stpr_moveTo() 等驱动接口
 */
extern uint32_t velocity;

/**
 * @brief 条件检测回调
 *
 * @param user_ctx 用户自定义上下文
 * @return true  条件满足，应立即停止运动
 * @return false 条件未满足，继续运动
 */
typedef bool (*MotorConditionCheckFn)(void *user_ctx);

/* ===================== 基础初始化/停止 ===================== */

/**
 * @brief 电机初始化：初始化 TMC5130、使能驱动
 * @return NO_ERROR 或错误码
 */
uint32_t motor_Init(void);

/**
 * @brief 电机快速停止：急停 + 临时关闭驱动 + 延时 + 重新使能
 * @return NO_ERROR 或错误码
 */
uint32_t motorQuickStop(void);

/**
 * @brief 电机慢速停止：仅发送 stop 指令（不关驱动）
 * @return NO_ERROR 或错误码
 */
uint32_t motorSlowStop(void);

/* ===================== 运动控制接口 ===================== */

/**
 * @brief 电机长距离上行（测试/调试用）：下发一个很大的上行距离（不等待）
 *
 * 用法：
 *  - 常用于“粗找”流程，每次调用推动继续上行
 *  - 使用当前已生效速度
 *  - 返回时电机通常仍在运行
 *  - 若需要等待停止，请不要用这个接口
 *
 * @return NO_ERROR 或错误码
 */
uint32_t motorMove_up(void);

/**
 * @brief 电机长距离上行（测试/调试用），并为本次下发指定速度
 *
 * 用法：
 *  - speed_x100 单位为 0.01m/min
 *  - 这是非阻塞接口，不会自动恢复默认速度
 */
uint32_t motorMove_upWithSpeed(uint32_t speed_x100);

/**
 * @brief 电机长距离下行（测试/调试用）：下发一个很大的下行距离（不等待）
 *
 * 用法：
 *  - 常用于“粗找”流程，每次调用推动继续下行
 *  - 使用当前已生效速度
 *  - 返回时电机通常仍在运行
 *  - 若需要等待停止，请不要用这个接口
 *
 * @return NO_ERROR 或错误码
 */
uint32_t motorMove_down(void);

/**
 * @brief 电机长距离下行（测试/调试用），并为本次下发指定速度
 *
 * 用法：
 *  - speed_x100 单位为 0.01m/min
 *  - 这是非阻塞接口，不会自动恢复默认速度
 */
uint32_t motorMove_downWithSpeed(uint32_t speed_x100);

/**
 * @brief 精确长度换算下发（不等待）
 *
 * 输入：
 *  - mm：移动距离（单位：mm，必须 >=0）
 *  - dir：MOTOR_DIRECTION_UP / MOTOR_DIRECTION_DOWN
 *
 * 说明：
 *  - 本函数内部会读取当前 cable_length（物理长度基准）
 *  - 结合卷筒模型计算应走 ticks，并调用 stpr_moveBy() 下发
 *  - 本函数不等待停止；需要上层调用 wait 或监测
 *  - 是最基础的“发运动命令”接口
 *
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveNoWait(float mm, int dir);

/**
 * @brief 精确长度换算下发（不等待），并为本次下发指定速度
 *
 * 用法：
 *  - 本函数会先切换到指定速度，再下发运动
 *  - 因为不等待结束，所以不会自动恢复默认速度
 */
uint32_t motorMoveNoWaitWithSpeed(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 电机移动并等待停止（阻塞）
 *
 * 说明：
 *  - 内部会调用 motorMoveNoWait() 下发
 *  - 阻塞等待期间执行碰撞检测、驱动异常检测等
 *  - 允许补偿重试（按 motor_ctrl.c 的重试策略）
 *  - 是目前最推荐的按距离主入口
 *
 * @param mm  移动距离（mm，必须 >=0）
 * @param dir 方向（MOTOR_DIRECTION_UP / MOTOR_DIRECTION_DOWN）
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveAndWaitUntilStop(float mm, int dir);

/**
 * @brief 电机移动并等待停止（阻塞），并为本次命令临时指定速度
 *
 * 用法：
 *  - speed_x100 单位为 0.01m/min
 *  - 命令开始前切到该速度
 *  - 命令结束后自动恢复到 g_deviceParams.max_motor_speed
 */
uint32_t motorMoveAndWaitUntilStopWithSpeed(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 设置电机速度设定值，可在运行前或运行中调整
 *
 * 说明：
 *  - speed_x100 单位为 0.01m/min，100 表示 1.00m/min
 *  - 内部会自动限制到有效范围，超出范围将被钳制到最小/最大值
 *  - 若当前未运行，则仅更新后续运动使用的速度设定值
 *  - 若当前正在运行，则会立即重算 VMAX 并写入 TMC5130
 *  - 该接口属于“持久改速”，不会自动恢复
 *
 * @param speed_x100 目标速度，单位 0.01m/min
 * @return NO_ERROR 或错误码
 */
uint32_t motorSetSpeed(uint32_t speed_x100);

/**
 * @brief 按 ticks 运动并等待结束（阻塞）
 *
 * 说明：
 *  - ticks 为 TMC5130 的位置增量（microstep）
 *  - ticks 的正方向是否等于“下行放带”，建议通过 Motor_DebugCheckTicksDirection() 确认一次
 *  - 若发现正方向相反，建议只在 motorMoveWaitByTicks() 的实现中统一翻转
 *  - 常用于标定和底层调试
 *
 * @param ticks 运动步数（可正可负）
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveWaitByTicks(int32_t ticks);

/**
 * @brief 按 ticks 运动并等待结束（阻塞），并为本次命令临时指定速度
 *
 * 用法：
 *  - 命令开始前切到该速度
 *  - 命令结束后自动恢复到 g_deviceParams.max_motor_speed
 */
uint32_t motorMoveWaitByTicksWithSpeed(int32_t ticks, uint32_t speed_x100);

/**
 * @brief 移动到绝对目标位置（阻塞，单次调用）
 *
 * 说明：
 *  - target_mm 为绝对位置（mm）
 *  - 内部会根据当前位置与目标位置计算方向与距离，调用 motorMoveAndWaitUntilStop()
 *  - 适合业务层“跑到绝对位置”
 *
 * @param target_mm 目标绝对位置（mm）
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveToPositionOneShot(float target_mm);

/**
 * @brief 移动到绝对目标位置（阻塞），并为本次命令临时指定速度
 *
 * 用法：
 *  - 命令开始前切到该速度
 *  - 命令结束后自动恢复到 g_deviceParams.max_motor_speed
 */
uint32_t motorMoveToPositionOneShotWithSpeed(float target_mm, uint32_t speed_x100);

/**
 * @brief 相对位移一次下发（阻塞）
 *
 * 说明：
 *  - mm 正负含义需与你工程“位置坐标定义”一致
 *  - 建议与 motor_ctrl.c 内实现保持口径一致（例如正数=下行，负数=上行）
 *  - 适合做相对补偿和微调
 *
 * @param mm 相对位移（mm）
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveRelativeOneShot(float mm);

/**
 * @brief 相对位移一次下发（阻塞），并为本次命令临时指定速度
 *
 * 用法：
 *  - 命令开始前切到该速度
 *  - 命令结束后自动恢复到 g_deviceParams.max_motor_speed
 */
uint32_t motorMoveRelativeOneShotWithSpeed(float mm, uint32_t speed_x100);

/**
 * @brief 电机运行直到满足用户条件或触发保护停止
 *
 * 用法：
 *  - 按 max_mm 和 dir 先下发一段“足够长”的运动
 *  - 运行中周期性调用 condition_fn(user_ctx)
 *  - 当回调返回 true 时，立即停止并返回
 *  - 期间仍执行速度补偿、碰撞检测、驱动异常检测、超时保护
 *  - 若运动自然结束但条件未满足，则返回 NO_ERROR，同时 *condition_met = false
 *
 * 速度语义：
 *  - speed_x100 = 0：沿用当前速度
 *  - speed_x100 > 0：本次命令临时使用该速度
 *  - 命令结束后自动恢复到 g_deviceParams.max_motor_speed
 */
uint32_t motorMoveUntilCondition(float max_mm,
                                 int dir,
                                 uint32_t speed_x100,
                                 uint32_t poll_ms,
                                 uint32_t timeout_ms,
                                 MotorConditionCheckFn condition_fn,
                                 void *user_ctx,
                                 bool *condition_met);

/* ===================== 驱动状态/运动状态 ===================== */

/**
 * @brief 判断电机是否正在运动（基于 TMC5130_RAMPSTAT）
 *
 * @param tmc5130 TMC5130 句柄
 * @return true 仍在运动；false 已停止
 */
bool stpr_isMoving(TMC5130TypeDef *tmc5130);

/**
 * @brief 检查 TMC5130 的 GSTAT 状态寄存器并处理错误（如过热/欠压等）
 *
 * 说明：
 *  - 函数内部通常会读取 GSTAT 并清除状态位
 *  - 若检测到异常，返回对应错误码并可触发统一故障处理
 *
 * @param tmc5130 TMC5130 句柄
 * @return NO_ERROR 或错误码
 */
uint32_t stpr_checkGstat(TMC5130TypeDef *tmc5130);

/* ===================== 丢步检测 ===================== */

/**
 * @brief 丢步检测初始化：每次新的运动阶段开始前调用
 */
void MotorLostStep_Init(void);

/**
 * @brief 自动定时丢步检测（建议高频调用，但内部每 1 秒执行一次判断）
 *
 * 输入：
 *  - currentPos：当前位置（单位 0.1mm），一般使用 sensor_position
 *
 * 输出：
 *  - NO_ERROR：正常
 *  - ENCODER_LOST_STEP：检测到疑似丢步
 */
uint32_t Motor_CheckLostStep_AutoTiming(int32_t currentPos);

/* ===================== 位置快照 ===================== */

/**
 * @brief 读取当前 sensor_position 快照（单位：mm）
 *
 * @param pos_mm 输出：当前位置（mm）
 */
void snapshot_sensor_pos_mm(float *pos_mm);

/* ===================== 参数标定（尺带机构） ===================== */

/**
 * @brief 标定第一圈周长（电机在零点位置时下行旋转一圈）
 *
 * 原理：
 *  - 测得 dL = L1 - L0
 *  - 卷筒模型在 n=0->1：dL = C0 + π*t
 *  - C0 = dL - π*t
 *
 * 前提：
 *  - 当前位于零点，且 cable_length 基准正确
 *  - 下行一圈不会碰撞/越界
 *
 * @return NO_ERROR 或错误码
 */
uint32_t CalibrateFirstLoopCircumference_OneTurnAtZero(void);



/* ===================== 调试/显示：XACTUAL -> 圈数/角度/预测长度 ===================== */

/**
 * @brief 卷筒状态结构体（用于显示/调试）
 *
 * 注意：
 *  - motor_distance_01mm 为电机侧预测长度，不等同于 cable_length（物理长度）
 */
typedef struct
{
    int32_t motor_step;          ///< TMC5130_XACTUAL（微步 ticks）
    double  turns_total;         ///< 输出轴总圈数（含小数）
    int32_t turns_int;           ///< 整圈数（floor）
    double  angle_deg;           ///< 圈内角度 [0,360)
    int32_t motor_distance_01mm; ///< 由模型预测的长度（0.1mm）
} MotorDrumState;

/**
 * @brief 从 TMC5130_XACTUAL 计算卷筒总圈数、圈内角度、预测长度
 *
 * 说明：
 *  - 若希望“相对零点”的 XACTUAL，可在回零后调用 stpr_setPos(&stepper, 0)
 *
 * @param tmc5130 TMC5130 句柄
 * @param out 输出状态结构体
 */
void Motor_UpdateDrumState_FromXACTUAL(TMC5130TypeDef *tmc5130,
                                       MotorDrumState *out);

/**
 * @brief 无检测阻塞运动（调试/维护用）
 *
 * 用法：
 *  - 不做碰撞检测/丢步检测，只做基础运动与运行日志
 *  - 更适合调试期或维护模式，不建议直接用于关键测量流程
 */
void motorMoveBlocking_NoDetect(float mm, int dir);

/**
 * @brief 无检测阻塞运动（调试/维护用），并为本次命令临时指定速度
 *
 * 用法：
 *  - 命令开始前切到该速度
 *  - 命令结束后自动恢复到 g_deviceParams.max_motor_speed
 */
void motorMoveBlocking_NoDetectWithSpeed(float mm, int dir, uint32_t speed_x100);
#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_CTRL_H_ */

