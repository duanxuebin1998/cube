/*
 * weight.h
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_WEIGHT_H_
/* INC_WEIGHT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_WEIGHT_H_

#include "AS5145.h"
#include "system_parameter.h"
/* #include "motor_ctrl.h" */

#define IMPACT_WEIGHT_THRESHOLD 4000.0 /* 扭力处理参数：冲击 扭力 阈值。 */
typedef enum {
	/* 称重采样经过阈值和状态机判定后的业务状态。 */
	NORMAL, /* 称重处于正常稳定范围。 */ IMPACT, /* 检测到碰撞或瞬时冲击。 */ ZERO, /* 称重结果处于零点判定范围。 */ BOTTOM /* 称重变化满足探底判定条件。 */
} Weight_StateTypeDef;

typedef struct {
	int stable_weight; /* 稳态基准扭力（单位：0.1克） */
	int current_weight; /* 实时采样扭力（不带滤波处理） */
	int empty_weight; /* 空载扭力 */
	int full_weight; /* 满载扭力 */
} Weight_ParamentTypeDef;

extern Weight_ParamentTypeDef weight_parament;

/**
 * @brief 从设备参数加载空载和满载扭力，并初始化扭力通信有效性状态。
 *
 * @return 固定返回 NO_ERROR；当前初始化过程只复制参数并复位运行态，不包含可失败的硬件访问。
 */
uint32_t weight_init() ;
/**
 * @brief 获取空载扭力
 *        延时5秒后，将当前扭力值记录为空载扭力
 *        并通过串口打印空载扭力
 *
 * uint32_t get_stable_weight(void);
 *
 * @return NO_ERROR 表示获取空载扭力已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t get_empty_weight(void);
/**
 * @brief 获取满载扭力
 *        延时5秒后，计算当前扭力与空载扭力的差值，作为满载扭力
 *        并通过串口打印满载扭力
 *
 * @return NO_ERROR 表示获取满载扭力已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t get_full_weight(void);
/**
 * @brief 零点状态检测（基于扭力）
 *
 * Weight_StateTypeDef determine_weight_status(void);
 *
 * @return ZERO（到达零点）或 NORMAL（未到零点）
 */
Weight_StateTypeDef check_zero_point_status(void);
/**
 * @brief 罐底状态检测（不锁存）。
 * @param status 输出检测状态，检测成功时写入 NORMAL 或 BOTTOM。
 * @return NO_ERROR 表示本次检测有效；其他错误码表示检测链路异常，需要上层停止探底并进入最终报错。
 *
 * 判定方式：
 *  - g_deviceParams.bottom_detect_mode == 0 : 扭力阈值
 *  - g_deviceParams.bottom_detect_mode != 0 : 陀螺仪角度阈值
 *
 * 阈值来源：
 *  - 扭力阈值：g_deviceParams.bottom_weight_threshold
 *  - 角度阈值：g_deviceParams.bottom_angle_threshold
 */
uint32_t check_bottom_status(Weight_StateTypeDef *status); /* 检测罐底状态，返回错误码并通过参数输出 NORMAL/BOTTOM */
/**
 * @brief 扭力统一碰撞/极限检测
 *        上行: 先判零点阈值，再判变重阈值
 *        下行: 先判罐底阈值，再判变轻阈值
 *
 * 规则：
 *  1) 碰撞阈值（相对变化）：
 *     - 上行变重阈值 = full_weight * g_deviceParams.weight_upper_limit_ratio / 100
 *     - 下行变轻阈值 = full_weight * g_deviceParams.weight_lower_limit_ratio / 100
 *
 *  2) 零点/罐底阈值：
 *     - 零点：g_deviceParams.zero_weight_threshold_ratio（百分比，加在 full_weight 上）
 *     - 罐底：g_deviceParams.bottom_detect_mode==0 用扭力阈值 g_deviceParams.bottom_weight_threshold
 *            g_deviceParams.bottom_detect_mode!=0 用角度阈值 g_deviceParams.bottom_angle_threshold（不在此函数判定）
 *
 *  3) 保护：尺带长度 < g_deviceParams.max_zero_deviation_distance 时，不进行碰撞检测（直接 NO_ERROR）
 *
 * @return WEIGHT_COLLISION_DETECTED 表示检测到碰撞/到达极限；NO_ERROR 表示正常
 */
uint32_t CheckWeightCollision(void);
/**
 * @brief 更新扭力模块当前扭力并维护稳定/空载/满载状态。
 * @param currWeight 当前扭力采样值。
 */
void Weight_Update(int32_t currWeight);
/**
 * @brief 回零退让成功后，把当前扭力设为新的稳定基线。
 */
void Weight_RebaseStableWeight(void);
/**
 * @brief 接收扭力数据中的 Weight_MarkFrameReceived 逻辑。
 */
void Weight_MarkFrameReceived(void);
/**
 * @brief 检查共享扭力通信是否超过允许静默时间。
 * @return 返回共享扭力通信超时检查结果；NO_ERROR 表示链路未超时，其他值为对应通信故障码。
 */
uint32_t Weight_CheckCommunicationTimeout(void);
/**
 * @brief 检查本机扭力通信链路是否超过允许静默时间。
 * @return 返回本机扭力通信链路超时检查结果；NO_ERROR 表示链路未超时，其他值为对应通信故障码。
 */
uint32_t Weight_CheckOwnCommunicationTimeout(void);
#endif /* INC_WEIGHT_H_ */
