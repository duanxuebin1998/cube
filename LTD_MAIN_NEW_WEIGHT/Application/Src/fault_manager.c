/*
 * fault_manager.c
 *
 *  Created on: Mar 13, 2025
 *      Author: 1
 */
#include "fault_manager.h"

// 全局故障信息结构体
volatile FaultInfo g_faultInfo =
{ .severity = FAULT_SEVERITY_NONE };

/* 故障信息初始化函数（系统启动时调用） */
void fault_info_init(void)
{
	memset((void*) &g_faultInfo, 0, sizeof(FaultInfo));
	g_faultInfo.severity = FAULT_SEVERITY_NONE; // 初始化为无故障状态
}

/**
 * @brief 复合型故障处理函数
 * @param category 故障类别
 * @param error_code 诊断故障码
 * @param location 故障位置标识
 * @param severity 故障严重等级
 * @param retry_count 故障恢复尝试次数
 * @return 32位故障寄存器值
 */
uint32_t update_and_encode_fault(FaultCategory category, uint8_t error_code,
		uint8_t location, FaultSeverity severity)
{
	// 更新全局故障信息（符合ISO 14229-1标准）
	g_faultInfo.category = category;
	g_faultInfo.error_code = error_code;
	g_faultInfo.location = location;
	g_faultInfo.severity = severity;
	g_faultInfo.retry_count += 1;

	// 实时寄存器编码
	uint32_t register_value = ((uint32_t) (g_faultInfo.category & 0xFF) << 16)
			| ((uint32_t) (g_faultInfo.location & 0xFF) << 8)
			| (uint32_t) (g_faultInfo.error_code & 0xFF);

	return register_value;
}

/* 带阈值的故障状态检测函数 */
uint8_t check_fault_with_threshold(FaultSeverity threshold)
{
	// 原子操作读取当前严重级别（防止中断干扰）
	FaultSeverity current_severity = g_faultInfo.severity;

	/* 故障等级映射规则：
	 * 1. 当系统存在更高或等于阈值的故障时返回1
	 * 2. 故障等级按严重程度递增排列（4>3>2>1>0）
	 * 3. 支持动态阈值配置（1-4级）
	 */
	return (current_severity >= threshold) ? 1 : 0;
}
