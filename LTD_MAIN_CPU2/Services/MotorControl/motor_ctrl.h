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
 *    - 按距离阻塞移动：motorMoveAndWaitUntilStopWithSpeed()
 *    - 按绝对位置移动：motorMoveToPositionOneShotWithSpeed()
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
 * @brief 长距离上行命令（非阻塞）
 *
 * 说明：
 *  - 主要用于粗找、调试或手动连续上行
 *  - 函数返回时电机通常仍在运行
 *  - speed_x100 单位为 0.01m/min
 *  - 非阻塞接口不会自动恢复默认速度
 *
 * @param speed_x100 本次命令使用的速度；通常传 motorGetDefaultSpeedX100()
 * @return NO_ERROR 或错误码
 */
uint32_t motorMove_upWithSpeed(uint32_t speed_x100);

/**
 * @brief 长距离下行命令（非阻塞）
 *
 * 说明：
 *  - 主要用于粗找、调试或手动连续下行
 *  - 函数返回时电机通常仍在运行
 *  - speed_x100 单位为 0.01m/min
 *  - 非阻塞接口不会自动恢复默认速度
 *
 * @param speed_x100 本次命令使用的速度；通常传 motorGetDefaultSpeedX100()
 * @return NO_ERROR 或错误码
 */
uint32_t motorMove_downWithSpeed(uint32_t speed_x100);

/**
 * @brief 按距离下发运动命令（非阻塞）
 *
 * 说明：
 *  - mm 为移动距离，单位 mm，必须 >= 0
 *  - dir 为 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN
 *  - 内部会读取当前 cable_length，并结合卷筒模型换算成 ticks 下发
 *  - 适合“发出命令后由上层自己等待或监控”的场景
 *  - 非阻塞接口不会自动恢复默认速度
 *
 * @param mm 移动距离，单位 mm
 * @param dir 方向
 * @param speed_x100 本次命令使用的速度；通常传 motorGetDefaultSpeedX100()
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveNoWaitWithSpeed(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 按距离运动并等待停止（阻塞）
 *
 * 说明：
 *  - 是当前最推荐的按距离移动主入口
 *  - 内部会调用 motorMoveNoWaitWithSpeed() 下发
 *  - 等待期间保留碰撞检测、驱动异常检测、速度补偿和补偿重试
 *  - 阻塞接口结束后会自动恢复到 g_deviceParams.max_motor_speed
 *
 * @param mm 移动距离，单位 mm
 * @param dir 方向
 * @param speed_x100 本次命令临时使用的速度；通常传 motorGetDefaultSpeedX100()
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveAndWaitUntilStopWithSpeed(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 设置当前速度设定值
 *
 * 说明：
 *  - speed_x100 单位为 0.01m/min，100 表示 1.00m/min
 *  - 若电机空闲，只更新后续运动使用的速度设定
 *  - 若电机正在运行，会立即重算 VMAX 并写入驱动
 *  - 这是“持久改速”接口，不会自动恢复
 *
 * @param speed_x100 目标速度，单位 0.01m/min
 * @return NO_ERROR 或错误码
 */
uint32_t motorSetSpeed(uint32_t speed_x100);

/**
 * @brief 获取默认最高速度
 *
 * 说明：
 *  - 返回值直接来自 g_deviceParams.max_motor_speed
 *  - 推荐业务层默认传这个值，保持“按参数最大速度运行”的统一口径
 *
 * @return speed_x100，单位 0.01m/min
 */
uint32_t motorGetDefaultSpeedX100(void);

/**
 * @brief 按 ticks 运动并等待停止（阻塞）
 *
 * 说明：
 *  - ticks 为 TMC5130 的位置增量，单位 microstep
 *  - 常用于标定、调试或需要直接控制步数的场景
 *  - 若正方向与“下行放带”口径不一致，应只在本实现内部统一翻转
 *  - 阻塞接口结束后会自动恢复到 g_deviceParams.max_motor_speed
 *
 * @param ticks 运动步数，可正可负
 * @param speed_x100 本次命令临时使用的速度
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveWaitByTicksWithSpeed(int32_t ticks, uint32_t speed_x100);

/**
 * @brief 移动到绝对位置（阻塞）
 *
 * 说明：
 *  - target_mm 为目标绝对位置，单位 mm
 *  - 内部会先计算当前位置、方向和距离，再调用 motorMoveAndWaitUntilStopWithSpeed()
 *  - 阻塞接口结束后会自动恢复到 g_deviceParams.max_motor_speed
 *
 * @param target_mm 目标绝对位置，单位 mm
 * @param speed_x100 本次命令临时使用的速度
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveToPositionOneShotWithSpeed(float target_mm, uint32_t speed_x100);

/**
 * @brief 相对位移一次到位（阻塞）
 *
 * 说明：
 *  - mm 为相对位移，单位 mm
 *  - 正负方向语义以 motor_ctrl.c 当前实现口径为准
 *  - 适合相对补偿和小范围微调
 *  - 阻塞接口结束后会自动恢复到 g_deviceParams.max_motor_speed
 *
 * @param mm 相对位移，单位 mm
 * @param speed_x100 本次命令临时使用的速度
 * @return NO_ERROR 或错误码
 */
uint32_t motorMoveRelativeOneShotWithSpeed(float mm, uint32_t speed_x100);

/**
 * @brief 持续运行直到满足条件或触发保护停止
 *
 * 说明：
 *  - 先按 max_mm 和 dir 下发一段足够长的运动
 *  - 运行中周期性调用 condition_fn(user_ctx)
 *  - 回调返回 true 时立即停机，并按“条件满足”返回
 *  - 若触发碰撞、驱动异常、超时等保护，则按错误码返回
 *  - 命令结束后会自动恢复到 g_deviceParams.max_motor_speed
 *
 * @param max_mm 最大允许运动距离，单位 mm
 * @param dir 方向
 * @param speed_x100 本次命令临时使用的速度
 * @param poll_ms 条件轮询周期；传 0 使用默认值
 * @param timeout_ms 总超时；传 0 使用默认值
 * @param condition_fn 条件回调，返回 true 表示应立即停止
 * @param user_ctx 传给回调的用户上下文
 * @param condition_met 输出参数；返回 true 表示因条件满足而停止
 * @return NO_ERROR 或错误码
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

