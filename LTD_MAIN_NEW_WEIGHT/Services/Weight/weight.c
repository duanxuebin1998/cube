/*
 * weight.c
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 *  该文件实现了重量传感器的相关操作，包括获取空载重量、获取稳定重量、判断重量状态等功能。
 */

#include "main.h"
#include "weight.h"
#include "stdio.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "AS5145.h"

#define MAX_WEIGHT 20000 // 最大重量限制
#define MIN_WEIGHT 1000 // 最小重量限制
#define MAX_EMPTY_WEIGHT 20000 // 最大重量限制
// 全局变量，存储当前称重传感器的原始重量值
int16_t g_weight;

// 全局结构体，存储称重相关参数（空载、稳定、当前重量等）
Weight_ParamentTypeDef weight_parament = { 0 };

//初始化称重
uint32_t weight_init() {
	weight_parament.empty_weight = g_deviceParams.empty_weight;// 从设备参数中获取空载重量
	weight_parament.full_weight = g_deviceParams.full_weight;// 从设备参数中获取满载重量
	printf("empty_weight\t%d\r\n", weight_parament.empty_weight); // 打印空载重量
	printf("full_weight\t%d\r\n", weight_parament.full_weight); // 打印满载重量
	return NO_ERROR;
}

/**
 * @brief 获取空载重量
 *        延时5秒后，将当前重量值记录为空载重量
 *        并通过串口打印空载重量
 */
uint32_t get_empty_weight(void) {
	printf("正在获取空载称重值...\r\n"); // 打印获取空载重量信息
	HAL_Delay(5000); // 等待5秒，确保重量稳定
	weight_parament.empty_weight = g_weight; // 记录空载重量
	printf("空载称重值获取成功\t空载称重：\t%d\r\n", weight_parament.empty_weight); // 打印空载重量
	if (abs(weight_parament.empty_weight > MAX_EMPTY_WEIGHT)) { // 检查空载重量是否在合理范围内
		printf("空载称重值异常，请检查传感器或重新校准\r\n");
		CHECK_ERROR(WEIGHT_DRIFT_ERROR); // 如果空载重量不在合理范围内，打印错误信息并返回
	} else {
		g_deviceParams.empty_weight = weight_parament.empty_weight; // 更新设备参数中的空载重量
		printf("空载称重值获取成功\t空载称重：\t%d\r\n", weight_parament.empty_weight); // 打印空载重量
		save_device_params(); // 保存设备参数
	}
	return NO_ERROR; // 获取空载重量成功，返回无错误状态
}
/**
 * @brief 获取满载重量
 *        延时5秒后，计算当前重量与空载重量的差值，作为满载重量
 *        并通过串口打印满载重量
 */
uint32_t get_full_weight(void) {
    printf("正在等待称重值稳定ing...\r\n"); // 打印等待信息
    HAL_Delay(5000); // 等待5秒，确保重量稳定

    weight_parament.full_weight = weight_parament.empty_weight - g_weight; // 计算满载重量

    if (weight_parament.full_weight < MIN_WEIGHT) {
        printf("满载重量小于最小限制，可能需要重新校准\r\n");
        CHECK_ERROR(WEIGHT_UNDER_RANGE); // 小于最小限制，报错
    } else if (weight_parament.full_weight > MAX_WEIGHT) {
        printf("满载重量超出范围，可能需要重新校准\r\n");
        CHECK_ERROR(WEIGHT_OUT_OF_RANGE); // 超出范围，报错
    }
    else {
		g_deviceParams.full_weight = weight_parament.full_weight; // 更新设备参数中的满载重量
		printf("满载称重值获取成功\t满载的称重：\t%d\r\n", weight_parament.full_weight); // 打印满载重量
		save_device_params(); // 保存设备参数
	}

    printf("满载称重值获取成功\t满载的称重：\t%d\r\n", weight_parament.full_weight); // 打印满载重量
    return NO_ERROR; // 获取成功
}

/**
 * @brief 获取稳定重量
 *        延时5秒后，计算空载重量与当前重量的差值，作为稳定重量
 *        并通过串口打印稳定重量
 */
uint32_t get_stable_weight(void) {
	printf("正在等待称重值稳定ing...\r\n"); // 打印等待信息
	HAL_Delay(5000); // 等待5秒，确保重量稳定
	weight_parament.stable_weight = weight_parament.empty_weight - g_weight; // 计算稳定重量
	if (weight_parament.stable_weight < MIN_WEIGHT) {
		printf("稳定重量小于最小限制，可能需要重新校准\r\n");
		CHECK_ERROR(WEIGHT_UNDER_RANGE); // 如果稳定重量小于最小限制，打印错误信息并返回
	} else if (weight_parament.stable_weight > MAX_WEIGHT) { // 检查稳定重量是否在合理范围内
		printf("稳定重量超出范围，可能需要重新校准\r\n");
		CHECK_ERROR(WEIGHT_OUT_OF_RANGE); // 如果稳定重量小于0，打印错误信息并返回
	}
	printf("稳态称重值获取成功\t稳定的称重：\t%d\r\n", weight_parament.stable_weight); // 打印稳定重量
	return NO_ERROR; //获取稳态称重成功，返回
}

/**
 * @brief 判断当前重量状态
 *        计算当前重量与空载重量的差值，根据阈值判断重量状态
 *        并通过串口打印当前重量及状态
 * @return ZERO   - 重量过大
 *         BOTTOM - 重量过小
 *         NORMAL - 重量正常
 */
Weight_StateTypeDef determine_weight_status(void) {
	// 计算当前重量
	weight_parament.current_weight = weight_parament.empty_weight - g_weight;
	weight_parament.stable_weight = weight_parament.stable_weight*0.8+weight_parament.current_weight*0.2; // 稳定重量
	printf("称重状态检测\t当前称重值\t%d\t", weight_parament.current_weight);

	// 判断是否超过上限阈值
	if (weight_parament.current_weight > weight_parament.full_weight * (100 + ZERO_WEIGHT_THRESHOLD) / 100) {
		printf("称重值过大\r\n");
		return ZERO; // 返回重量过大状态
	}
	// 判断是否低于下限阈值
	else if (weight_parament.current_weight < weight_parament.full_weight * (100 - BOTTOM_WEIGHT_THRESHOLD) / 100) {
		printf("称重值过小\r\n");
		return BOTTOM; // 返回重量过小状态
	}
	// // 判断是否有剧烈波动（
	// else if(fabs(weight_parament.current_weight-weight_parament.stable_weight)>IMPACT_WEIGHT_THRESHOLD)
	// {
	//     printf("稳定的称重\t%d\r\n",weight_parament.stable_weight);
	//     printf("称重值波动超限\r\n");
	//     return IMPACT;
	// }
	// 重量正常
	else {
		printf("称重值正常\r\n");
		return NORMAL;	// 返回正常状态
	}
}
