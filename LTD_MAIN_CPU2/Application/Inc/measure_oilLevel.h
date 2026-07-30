/*
 * measure_oilLevel.h
 *
 *  Created on: Mar 28, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_OILLEVEL_H_
/* INC_MEASURE_OILLEVEL_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MEASURE_OILLEVEL_H_

#include "system_parameter.h"
/* 液位探头当前介质判定；用于在油相与气相之间切换搜索方向和液面确认逻辑。 */
typedef enum {
	/* 液位探头所在介质状态。 */
	OIL, /* 探头判定处于油相。 */ AIR /* 探头判定处于气相。 */
} Level_StateTypeDef;


#define MAX_TIMES_WHEN_FRE_FOLLOW				15 /* 频率跟随时的最大加速次数 */

/**
 * @brief 根据运动过程中的传感器频率判断浮子位于空气还是油中。
 *
 * @param state_out 用于返回液位频率相对上下阈值的判定状态。
 * @return NO_ERROR 表示已读取运动过程频率并通过 state_out 写入 AIR 或 OIL；空输出指针、命令切换、传感器通信或读数错误返回对应错误码。
 */
uint32_t determine_level_status_motion(Level_StateTypeDef *state_out);
/**
 * @brief 搜索液位高度
 *
 * 该函数通过粗找和精找两个阶段完成液位高度的测量。粗找阶段判断传感器当前状态（在油中或在空气中），
 * 并执行相应的搜索操作；精找阶段根据设定的目标频率进行精确液位定位。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 液位测量成功
 *                  - MEASUREMENT_WEIGHT_DOWN_FAIL: 粗找液位失败（超过最大尝试次数）
 *                  - 其他错误码: 具体错误状态
 *
 * @note 函数执行流程：
 *       1. 方法 2/3/5 先直接分流：密度速度闭环、超声预留、固定频率速度闭环
 *       2. 方法 0/1/4/未知配置复位运行态并启用液位模式
 *       3. 粗找阶段：判断传感器在油中或空气中，执行 SearchAir() 或 SearchOil()
 *       4. 解析目标频率：相对法取端点中点，固定频率步进法取参数频率
 *       5. 执行搜索：方法 4 走频率速度闭环，方法 0/1 走 SearchOilPrecise() 离散步进
 *       6. 步进路径记录最终液位位置；速度闭环路径由闭环函数记录结果
 *
 * @note 依赖函数：
 *       - OilLevel_TryRunDirectSearchMethod(): 处理方法 2/3/5 直接入口
 *       - OilLevel_EnableLevelModeWithRetry(): 启用液位模式并保留重试
 *       - OilLevel_RunFrequencyCoarseSearch(): 执行空气/油中端点粗找
 *       - OilLevel_ResolveSearchTargetFrequency(): 解析本次目标频率
 *       - OilLevel_RunResolvedSearchMethod(): 执行步进精找或方法 4 速度闭环
 *       - OilLevel_RecordSearchResult(): 记录步进精找成功结果
 */
uint32_t SearchOilLevel(void); /* 寻找油面但不跟随 */
/**
 * @brief 液位跟随函数，用于持续监测并跟踪液位变化
 *
 * 该函数通过超声波频率信号实现液位的实时监测与跟随。当液位稳定时，电机保持静止；
 * 当检测到液位变动时，重新执行精确搜索并调整传感器位置，确保液位值准确更新。
 *
 * @return uint32_t 返回操作状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部包含无限循环，持续监测液位状态，仅在发生错误时退出
 * @note 液位稳定判断基于频率波动阈值（oilLevelHysteresisThreshold）
 * @note 当液位过低进入盲区时，会调用 waitForTheLiquidLevelToExceedTheBlindZone 等待液位恢复
 */
uint32_t FollowOilLevel(void); /* 在油面附近跟随油面 */
/**
 * @brief 搜索并跟随液位
 *
 * 该函数执行液位测量的完整流程，包括回零点检查、液位搜索和液位跟随。
 * 首先检查设备是否需要回零点，如需要则先执行回零点操作。
 * 然后进行液位搜索，最多尝试3次。搜索成功后，根据设备状态执行液位位置修正。
 * 最后进入液位跟随状态，同样最多尝试3次，完成液位的持续跟踪。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 操作成功完成
 *                  - 其他错误码: 操作失败，具体错误码由 SearchZero、SearchOilLevel 或 FollowOilLevel 返回
 *
 * @note 函数执行流程：
 *       1. 检查并执行回零点操作（如果设备需要）
 *       2. 搜索液位（最多3次尝试）
 *       3. 修正液位位置（如果处于校准状态）
 *       4. 跟随液位（最多3次尝试）
 *
 * @note 每次失败后会延迟1秒后重试，如果检测到状态切换（STATE_SWITCH）则中止当前步骤
 */
uint32_t SearchAndFollowOilLevel(void); /* 寻找并跟随油面 */
/**
 * @brief 处理液位测量中的 CorrectOilLevelProcess 逻辑。
 */
void CorrectOilLevelProcess(void); /* 修正液位 */
#endif /* INC_MEASURE_OILLEVEL_H_ */
