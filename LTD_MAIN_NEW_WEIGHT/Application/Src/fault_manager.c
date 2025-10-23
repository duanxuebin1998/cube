/*
 * fault_manager.c
 *
 *  Created on: Mar 13, 2025
 *      Author: 1
 */
#include "fault_manager.h"
#include "system_parameter.h"
#include <stdio.h>

ErrorInfo err;// 全局错误信息变量
// 错误处理函数
/**
 * @brief 错误记录与紧急停机函数
 * @param err 错误信息结构体指针
 * @note 任何错误都停机；STATE_SWITCH 只停机不修改错误状态
 */
void LogError(const ErrorInfo* err)
{
    if (err == NULL) return; // 空指针保护

    // ⚠️ 任何错误都必须立即停机
    motorQuickStop();

    // 如果是状态切换错误，仅停机，不记录
    if (err->error_code == STATE_SWITCH)
    {
        return;
    }

    // 若当前无错误，记录错误信息
    if (g_measurement.device_status.error_code == NO_ERROR)
    {
        g_measurement.device_status.error_code = err->error_code;  // 保存错误码
        g_measurement.device_status.zero_point_status = 1;         // 标记需回零点
        // g_measurement.device_status.device_state = STATE_ERROR;  // 可选：切换状态为错误
        // g_measurement.device_status.current_command = COMMAND_NONE; // 可选：清空当前命令
    }
    // 打印错误信息
    printf("【故障】CODE: 0x%X | FILE: %s | LINE: %lu | FUNC: %s\r\n",
           err->error_code, err->file, err->line, err->func);
}

//void LogError(const ErrorInfo* err) {
//	motorQuickStop(); // 紧急停止电机
//	if (g_measurement.device_status.error_code == NO_ERROR && g_measurement.device_status.error_code != STATE_SWITCH&&err->error_code!= STATE_SWITCH) {
////		g_measurement.device_status.device_state = STATE_ERROR; // 设置设备状态为错误
//		g_measurement.device_status.error_code = err->error_code; // 更新错误码
//		g_measurement.device_status.zero_point_status = 1;// 设置零点状态为需要回零点
////		g_measurement.device_status.current_command = COMMAND_NONE; // 清除当前命令
//		// 打印错误信息到控制台
//		printf("故障\tCODE: 0x%X | FILE: %s | LINE: %lu | FUNC: %s \r\n", err->error_code, err->file, err->line, err->func);
//
//	}
//}
// 提取短文件名 (从路径中提取)
const char* GetShortFilename(const char* fullpath) {
    const char* p = fullpath + strlen(fullpath);
    while (p > fullpath) {
        if (*(p-1) == '/' || *(p-1) == '\\') break;
        p--;
    }
    return p;
}
// 全局故障信息结构体
volatile FaultInfo g_faultInfo = { .severity = FAULT_SEVERITY_NONE };

/* 故障信息初始化函数（系统启动时调用） */
void fault_info_init(void) {
	motorQuickStop(); // 到达零点后快速停止电机
	g_measurement.device_status.error_code = NO_ERROR; // 清除设备状态错误码;

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
uint32_t update_errorcode(uint8_t error_code) {
	// 更新全局故障信息（符合ISO 14229-1标准）
	g_faultInfo.error_code = error_code;

	// 实时寄存器编码
	uint32_t register_value = ((uint32_t) (g_faultInfo.location & 0xFF) << 8) | (uint32_t) (g_faultInfo.error_code & 0xFF);

	return register_value;
}

/* 带阈值的故障状态检测函数 */
uint8_t check_fault_with_threshold(FaultSeverity threshold) {
	// 原子操作读取当前严重级别（防止中断干扰）
	FaultSeverity current_severity = g_faultInfo.severity;

	/* 故障等级映射规则：
	 * 1. 当系统存在更高或等于阈值的故障时返回1
	 * 2. 故障等级按严重程度递增排列（4>3>2>1>0）
	 * 3. 支持动态阈值配置（1-4级）
	 */
	return (current_severity >= threshold) ? 1 : 0;
}
