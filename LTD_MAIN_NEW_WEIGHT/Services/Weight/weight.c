/*
 * weight.c
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 *  该文件实现了重量传感器的相关操作，包括获取空载重量、获取稳定重量、判断重量状态等功能。
 */

#include "main.h"
#include "weight.h"
#include "measure_zero.h"
#include "stdio.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "AS5145.h"
#include "measure_tank_height.h"

#define MAX_WEIGHT 20000 // 最大重量限制
#define MIN_WEIGHT -2000 // 最小重量限制
#define MAX_EMPTY_WEIGHT 20000 // 最大重量限制
// 采样和更新周期定义
#define WEIGHT_SAMPLE_INTERVAL 200    // 每200ms采样一次

// 全局变量，存储当前称重传感器的原始重量值
int16_t g_weight;

// 全局结构体，存储称重相关参数（空载、稳定、当前重量等）
Weight_ParamentTypeDef weight_parament = { 0 };

//初始化称重
uint32_t weight_init() {
	weight_parament.empty_weight = g_deviceParams.empty_weight;           // 从设备参数中获取空载重量
	weight_parament.full_weight = g_deviceParams.full_weight;           // 从设备参数中获取满载重量
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
		RETURN_ERROR(WEIGHT_DRIFT_ERROR); // 如果空载重量不在合理范围内，打印错误信息并返回
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

	weight_parament.full_weight = weight_parament.current_weight; // 计算满载重量

	if (weight_parament.full_weight < MIN_WEIGHT) {
		printf("满载重量小于最小限制，可能需要重新校准\r\n");
		RETURN_ERROR(WEIGHT_UNDER_RANGE); // 小于最小限制，报错
	} else if (weight_parament.full_weight > MAX_WEIGHT) {
		printf("满载重量超出范围，可能需要重新校准\r\n");
		RETURN_ERROR(WEIGHT_OUT_OF_RANGE); // 超出范围，报错
	} else {
		g_deviceParams.full_weight = weight_parament.full_weight; // 更新设备参数中的满载重量
		printf("满载称重值获取成功\t满载的称重：\t%d\r\n", weight_parament.full_weight); // 打印满载重量
		save_device_params(); // 保存设备参数
	}

	printf("满载称重值获取成功\t满载的称重：\t%d\r\n", weight_parament.full_weight); // 打印满载重量
	return NO_ERROR; // 获取成功
}


///**
// * @brief 判断当前重量状态
// *        计算当前重量与空载重量的差值，根据阈值判断重量状态
// *        并通过串口打印当前重量及状态
// * @return ZERO   - 重量过大
// *         BOTTOM - 重量过小
// *         NORMAL - 重量正常
// */
//Weight_StateTypeDef determine_weight_status(void)
//{
//    int upper_limit = weight_parament.full_weight * (100 + ZERO_WEIGHT_THRESHOLD) / 100;
//    int lower_limit = weight_parament.full_weight * (100 - BOTTOM_WEIGHT_THRESHOLD) / 100;
//    int current     = weight_parament.current_weight;
//    const char *result_str;
//
//    Weight_StateTypeDef state;
//
//    if (current > upper_limit) {
//        state = ZERO;
//        result_str = "到达零点";
//    }
//    else if (current < lower_limit) {
//        state = BOTTOM;
//        result_str = "到达罐底";
//    }
//    else {
//        state = NORMAL;
//        result_str = "正常";
//    }
//
//    printf("称重状态检测 | 当前:%d | 零点阈值:%d(+%d%%) | 罐底阈值:%d(-%d%%) | 结果:%s\r\n",
//           current, upper_limit, ZERO_WEIGHT_THRESHOLD,
//           lower_limit, BOTTOM_WEIGHT_THRESHOLD, result_str);
//
//    return state;
//}
/**
 * @brief 零点状态检测
 * @return ZERO（到达零点）或 NORMAL（未到零点）
 */
Weight_StateTypeDef check_zero_point_status(void)
{
    int upper_limit = weight_parament.full_weight * (100 + ZERO_WEIGHT_THRESHOLD) / 100;
    int current     = weight_parament.current_weight;
    const char *result_str;
    Weight_StateTypeDef state;

    if (current > upper_limit) {
        state = ZERO;
        result_str = "到达零点";
    } else {
        state = NORMAL;
        result_str = "正常";
    }

    printf("零点状态检测 | 当前:%d | 零点阈值:%d(+%d%%) | 结果:%s\r\n",
           current, upper_limit, ZERO_WEIGHT_THRESHOLD, result_str);

    return state;
}

/**
 * @brief 罐底状态检测
 * @return BOTTOM（到达罐底）或 NORMAL（未到罐底）
 */
Weight_StateTypeDef check_bottom_status(void)
{
    int lower_limit = weight_parament.full_weight * (100 - BOTTOM_WEIGHT_THRESHOLD) / 100;
    int current     = weight_parament.current_weight;
    const char *result_str;
    Weight_StateTypeDef state;

    if (current < lower_limit) {
        state = BOTTOM;
        result_str = "到达罐底";
    } else {
        state = NORMAL;
        result_str = "正常";
    }

    printf("罐底状态检测 | 当前:%d | 罐底阈值:%d(-%d%%) | 结果:%s\r\n",
           current, lower_limit, BOTTOM_WEIGHT_THRESHOLD, result_str);

    return state;
}
/**
 * @brief 称重碰撞检测函数（检测30%称重变化）
 * @return WEIGHT_COLLISION_DETECTED（1）表示检测到碰撞；NO_ERROR（0）表示正常
 */
uint32_t CheckWeightCollision(void)
{
    float cable_mm  = (float)g_measurement.debug_data.cable_length / 10.0f;
    float sensor_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
    float diff      = fabs(weight_parament.current_weight - weight_parament.stable_weight);
    float threshold = weight_parament.full_weight * 0.5f;

    if ((diff > threshold) &&
        (g_measurement.debug_data.cable_length > 1000) &&
        (g_measurement.debug_data.cable_length < bottom_value - g_deviceParams.blindZone))
    {
        printf("\r\n================= ⚠️ 碰撞检测报警 =================\r\n");
        printf("称重状态：波动超限\r\n");
        printf("当前重量：%d\r\n", weight_parament.current_weight);
        printf("稳定重量：%d\r\n", weight_parament.stable_weight);
        printf("重量差值：%.1f（阈值：%.1f）\r\n", diff, threshold);
        printf("尺带长度：%.1f mm\r\n", cable_mm);
        printf("距离罐底：%.1f mm\r\n", sensor_mm);
        printf("盲区高度：%ld mm\r\n", g_deviceParams.blindZone);
        printf("---------------------------------------------------\r\n");
        return WEIGHT_COLLISION_DETECTED;
    }
    else
    {
        printf("称重检测：正常 | 当前:%d | 稳定:%d | 稳定:%d | 差值:%.1f | 阈值:%.1f | 状态:OK\r\n",
               weight_parament.current_weight,
               weight_parament.stable_weight,
			   weight_parament.full_weight,
               diff,
               threshold);
        return NO_ERROR;
    }
}

/**
 * @brief 称重碰撞检测函数
 * @return IMPACT（1）表示检测到碰撞；NORMAL（0）表示正常
 */
//uint32_t CheckWeightCollision(void)
//{
//    float cable_mm  = (float)g_measurement.debug_data.cable_length / 10.0f;
//    float sensor_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
//    float diff      = fabs(weight_parament.current_weight - weight_parament.stable_weight);
//
//    // 判断条件
//    if ((diff > IMPACT_WEIGHT_THRESHOLD) &&
//        (g_measurement.debug_data.cable_length > 1000) &&
//        (g_measurement.debug_data.cable_length < bottom_value - g_deviceParams.waterBlindZone))
//    {
//        printf("\r\n================= ⚠️ 碰撞检测报警 =================\r\n");
//        printf("称重状态：波动超限\r\n");
//        printf("当前重量：%d\r\n", weight_parament.current_weight);
//        printf("稳定重量：%d\r\n", weight_parament.stable_weight);
//        printf("重量差值：%.1f（阈值：%.1f）\r\n", diff, IMPACT_WEIGHT_THRESHOLD);
//        printf("尺带长度：%.1f mm\r\n", cable_mm);
//        printf("距离罐底：%.1f mm\r\n", sensor_mm);
//        printf("盲区高度：%ld mm\r\n", g_deviceParams.waterBlindZone);
//        printf("---------------------------------------------------\r\n");
//        return WEIGHT_COLLISION_DETECTED;
//    }
//    else
//    {
//        printf("碰撞检测：正常（当前差值 %.1f ≤ 阈值 %.1f）\r\n", diff, IMPACT_WEIGHT_THRESHOLD);
//        return NO_ERROR;
//    }
//}

/**
 * @brief 称重稳态更新（整数版，无浮点）
 * @param currWeight 当前称重原始值（例如ADC或整数克重）
 * @return 稳态称重值（整数）
 */
void Weight_Update(int32_t currWeight) {
	static uint32_t lastTime = 0;
	uint32_t now = HAL_GetTick();

	if (now - lastTime < WEIGHT_SAMPLE_INTERVAL)
		return;  // 每200ms更新一次
	lastTime = now;

	weight_parament.stable_weight = weight_parament.stable_weight * 0.8f + currWeight * 0.2f;

	// 可选调试输出
	// printf("稳态称重: %ld\n", weightFilter.stableWeight);

	return;
}
