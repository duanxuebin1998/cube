/*
 * weight.c
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
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
#include "motor_ctrl.h"
#define WEIGHT_DEBUG
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


/**
 * @brief 零点状态检测（基于称重）
 * @return ZERO（到达零点）或 NORMAL（未到零点）
 */
Weight_StateTypeDef check_zero_point_status(void)
{
    int threshold_ratio = g_deviceParams.zero_weight_threshold_ratio;  /* % */
    int full_weight    = weight_parament.full_weight;
    int current        = weight_parament.current_weight;

    int upper_limit;
    Weight_StateTypeDef state;

    /* ---------- 防御性处理 ---------- */
    if (threshold_ratio < 0) {
        threshold_ratio = 0;
    }
    if (threshold_ratio > 100) {
        threshold_ratio = 100;
    }

    /* ---------- 阈值计算 ---------- */
    upper_limit = full_weight * (100 + threshold_ratio) / 100;

    /* ---------- 状态判断 ---------- */
    if (current > upper_limit) {
        state = ZERO;
    } else {
        state = NORMAL;
    }

    /* ---------- 日志输出 ---------- */
    printf("零点状态检测 | 当前:%d | 满载:%d | 阈值:+%d%% | 判定:%s\r\n",
           current,
           full_weight,
           threshold_ratio,
           (state == ZERO) ? "到达零点" : "正常");

    return state;
}

/**
 * @brief 称重统一碰撞/极限检测
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
 *     - 罐底：g_deviceParams.bottom_detect_mode==0 用称重阈值 g_deviceParams.bottom_weight_threshold
 *            g_deviceParams.bottom_detect_mode!=0 用角度阈值 g_deviceParams.bottom_angle_threshold（不在此函数判定）
 *
 *  3) 保护：尺带长度 < g_deviceParams.max_zero_deviation_distance 时，不进行碰撞检测（直接 NO_ERROR）
 *
 * @return WEIGHT_COLLISION_DETECTED 表示检测到碰撞/到达极限；NO_ERROR 表示正常
 */
uint32_t CheckWeightCollision(void)
{
	int32_t cur_weight	= (int32_t)weight_parament.current_weight;
	int32_t stable_weight	= (int32_t)weight_parament.stable_weight;
	int32_t full_weight	= (int32_t)weight_parament.full_weight;

	int32_t diff = cur_weight - stable_weight;

	uint32_t motor_dir = (uint32_t)g_measurement.debug_data.motor_state;

	/* 尺带长度/位置（debug_data: 0.1mm） */
	float cable_mm	= g_measurement.debug_data.cable_length / 10.0f;
	float sensor_mm	= g_measurement.debug_data.sensor_position / 10.0f;

	/* ==========================
	 *  保护：零点附近不做碰撞检测
	 * ========================== */
	if (cable_mm < (float)g_deviceParams.max_zero_deviation_distance/10.0) {
#ifdef WEIGHT_DEBUG
		printf("称重跳过 | 原因:零点保护 | dir=%lu cur=%ld stable=%ld diff=%+ld full=%ld cable=%.1f pos=%.1f | maxZeroDev=%lu\r\n",
				(unsigned long)motor_dir,
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)full_weight,
				cable_mm,
				sensor_mm,
				(unsigned long)g_deviceParams.max_zero_deviation_distance);
#endif
		return NO_ERROR;
	}

	/* ==========================
	 *    电机状态检查
	 * ========================== */
	if ((motor_dir != MOTOR_DIRECTION_UP) && (motor_dir != MOTOR_DIRECTION_DOWN)) {
#ifdef WEIGHT_DEBUG
		printf("称重检测 | dir=%lu(无效) cur=%ld stable=%ld diff=%+ld full=%ld cable=%.1f pos=%.1f\r\n",
				(unsigned long)motor_dir,
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)full_weight,
				cable_mm,
				sensor_mm);
#endif
		return NO_ERROR;
	}

	/* ==========================
	 *   相对碰撞阈值（整数）
	 * ========================== */
	int32_t upper_threshold =
		(int32_t)(((int64_t)full_weight * (int64_t)g_deviceParams.weight_upper_limit_ratio) / 100);

	int32_t lower_threshold =
		(int32_t)(((int64_t)full_weight * (int64_t)g_deviceParams.weight_lower_limit_ratio) / 100);

	/* 零点阈值：full_weight * (100 + ratio)% / 100 */
	int32_t zero_limit =
		(int32_t)(((int64_t)full_weight * (int64_t)(100 + g_deviceParams.zero_weight_threshold_ratio)) / 100);

	/* 罐底阈值：仅在“称重找底(mode==0)”时判定 */
	int32_t bottom_limit = (int32_t)g_deviceParams.bottom_weight_threshold;

	/* ==========================
	 *       上行检测
	 * ========================== */
	if (motor_dir == MOTOR_DIRECTION_UP) {

		/* 1) 零点阈值 */
		if (cur_weight > zero_limit) {
			printf("\r\n====== 称重碰撞报警(上行) ======\r\n");
			printf("原因: 超过零点阈值(认为已到零点)\r\n");
			printf("当前重量 : %ld\r\n", (long)cur_weight);
			printf("零点阈值 : %ld (full:%ld +%d%%)\r\n",
					(long)zero_limit,
					(long)full_weight,
					(int)g_deviceParams.zero_weight_threshold_ratio);
			printf("稳定重量 : %ld\r\n", (long)stable_weight);
			printf("diff     : %ld\r\n", (long)diff);
			printf("上限阈值 : %ld (ratio:%ld%%)\r\n",
					(long)upper_threshold,
					(long)g_deviceParams.weight_upper_limit_ratio);
			printf("尺带长度 : %.1f mm\r\n", cable_mm);
			printf("传感器位置 : %.1f mm\r\n", sensor_mm);
			printf("================================\r\n");
			return WEIGHT_COLLISION_DETECTED;
		}

		/* 2) 相对变重阈值 */
		if (diff > upper_threshold) {
			printf("\r\n====== 称重碰撞报警(上行) ======\r\n");
			printf("原因: 重量增加超过上限阈值\r\n");
			printf("当前重量 : %ld\r\n", (long)cur_weight);
			printf("稳定重量 : %ld\r\n", (long)stable_weight);
			printf("重量增加 : %ld (阈值:%ld)\r\n", (long)diff, (long)upper_threshold);
			printf("满载重量 : %ld (ratio:%ld%%)\r\n",
					(long)full_weight,
					(long)g_deviceParams.weight_upper_limit_ratio);
			printf("零点阈值 : %ld (+%d%%)\r\n",
					(long)zero_limit,
					(int)g_deviceParams.zero_weight_threshold_ratio);
			printf("尺带长度 : %.1f mm\r\n", cable_mm);
			printf("传感器位置 : %.1f mm\r\n", sensor_mm);
			printf("================================\r\n");
			return WEIGHT_COLLISION_DETECTED;
		}

#ifdef WEIGHT_DEBUG
		printf("称重正常 | 方向:上行 | 当前:%ld | 稳定:%ld | 变化:%+ld | 零点阈值:%ld | 上限阈值:%ld | 满载:%ld | 尺带:%.1fmm | 位置:%.1fmm\r\n",
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)zero_limit,
				(long)upper_threshold,
				(long)full_weight,
				cable_mm,
				sensor_mm);
#endif
		return NO_ERROR;
	}

	/* ==========================
	 *       下行检测
	 * ========================== */
	{
		/* 1) 罐底阈值：仅称重找底时有效 */
		if (g_deviceParams.bottom_detect_mode == 0) {
			if (cur_weight < bottom_limit) {
				printf("\r\n====== 称重碰撞报警(下行) ======\r\n");
				printf("原因: 低于触底阈值(认为已到罐底)\r\n");
				printf("当前重量 : %ld\r\n", (long)cur_weight);
				printf("触底阈值 : %ld\r\n", (long)bottom_limit);
				printf("稳定重量 : %ld\r\n", (long)stable_weight);
				printf("diff     : %ld\r\n", (long)diff);
				printf("下限阈值 : %ld (ratio:%ld%%)\r\n",
						(long)lower_threshold,
						(long)g_deviceParams.weight_lower_limit_ratio);
				printf("尺带长度 : %.1f mm\r\n", cable_mm);
				printf("传感器位置 : %.1f mm\r\n", sensor_mm);
				printf("================================\r\n");
				return WEIGHT_COLLISION_DETECTED;
			}
		}

		/* 2) 相对变轻阈值：diff 为负数，比较 -diff */
		if (-diff > lower_threshold) {
			printf("\r\n====== 称重碰撞报警(下行) ======\r\n");
			printf("原因: 重量减少超过下限阈值\r\n");
			printf("当前重量 : %ld\r\n", (long)cur_weight);
			printf("稳定重量 : %ld\r\n", (long)stable_weight);
			printf("重量减少 : %ld (阈值:%ld)\r\n", (long)(-diff), (long)lower_threshold);
			printf("满载重量 : %ld (ratio:%ld%%)\r\n",
					(long)full_weight,
					(long)g_deviceParams.weight_lower_limit_ratio);
			printf("尺带长度 : %.1f mm\r\n", cable_mm);
			printf("传感器位置 : %.1f mm\r\n", sensor_mm);
			printf("================================\r\n");
			return WEIGHT_COLLISION_DETECTED;
		}

#ifdef WEIGHT_DEBUG
		printf("称重正常 | 方向:下行 | 当前:%ld | 稳定:%ld | 变化:%+ld | 触底阈值:%ld | 下限阈值:%ld | 满载:%ld | 尺带:%.1fmm | 位置:%.1fmm | bottomMode=%lu\r\n",
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)bottom_limit,
				(long)lower_threshold,
				(long)full_weight,
				cable_mm,
				sensor_mm,
				(unsigned long)g_deviceParams.bottom_detect_mode);
#endif
		return NO_ERROR;
	}
}


///**
// * @brief 称重统一碰撞/极限检测
// *        上行: 先判零点阈值，再判变重阈值
// *        下行: 先判罐底阈值，再判变轻阈值
// * @return WEIGHT_COLLISION_DETECTED 表示检测到碰撞/到达极限；NO_ERROR 表示正常
// */
//uint32_t CheckWeightCollision(void)
//{
//    int32_t cur_weight    = weight_parament.current_weight;
//    int32_t stable_weight = weight_parament.stable_weight;
//    int32_t full_weight   = weight_parament.full_weight;
//
//    /* diff > 0 表示变重, diff < 0 表示变轻 */
//    int32_t diff = cur_weight - stable_weight;
//
//    /* 方向: 0 下行, 1 上行 (以你当前 motor_state 的定义为准) */
//    uint32_t motor_dir = g_measurement.debug_data.motor_state;
//
//    /* 阈值 = 满载重量 * 比例 / 100，全部用整数计算 */
//    int32_t upper_threshold =
//        (int32_t)(((int64_t)full_weight * (int64_t)g_deviceParams.weight_upper_limit_ratio) / 100);
//
//    int32_t lower_threshold =
//        (int32_t)(((int64_t)full_weight * (int64_t)g_deviceParams.weight_lower_limit_ratio) / 100);
//
//    /* 零点阈值：满载 * (100 + g_deviceParams.zero_weight_threshold_ratio)% / 100 */
//    int32_t zero_limit =
//        (int32_t)(((int64_t)full_weight * (int64_t)(100 + g_deviceParams.zero_weight_threshold_ratio)) / 100);
//
//    /* 罐底阈值：绝对值阈值 */
//    int32_t bottom_limit = (int32_t)BOTTOM_WEIGHT_THRESHOLD-2000;
//
//    float cable_mm  = g_measurement.debug_data.cable_length / 10.0f;
//    float sensor_mm = g_measurement.debug_data.sensor_position / 10.0f;
//
//    /* ==========================
//     *    电机停止或状态异常
//     * ========================== */
//    if ((motor_dir != MOTOR_DIRECTION_UP) && (motor_dir != MOTOR_DIRECTION_DOWN))
//    {
//#ifdef WEIGHT_DEBUG
//        /* 正常时一行输出（这里属于“非检测态”提示） */
//        printf("称重检测 | dir=%lu(无效) cur=%ld stable=%ld diff=%ld full=%ld cable=%.1f pos=%.1f\r\n",
//               (unsigned long)motor_dir,
//               (long)cur_weight,
//               (long)stable_weight,
//               (long)diff,
//               (long)full_weight,
//               cable_mm,
//               sensor_mm);
//#endif
//        /* 这里按“非运动不判碰撞”处理，避免误触发 */
//        return NO_ERROR;
//    }
//
//    /* ==========================
//     *       上行检测
//     * ========================== */
//    if (motor_dir == MOTOR_DIRECTION_UP)
//    {
//        /* 1) 零点阈值（统一在此处判断，不调用外部函数） */
//        if (cur_weight > zero_limit)
//        {
//            printf("\r\n====== 称重碰撞报警(上行) ======\r\n");
//            printf("原因: 超过零点阈值(认为已到零点)\r\n");
//            printf("当前重量 : %ld\r\n", (long)cur_weight);
//            printf("零点阈值 : %ld (full:%ld +%d%%)\r\n",
//                   (long)zero_limit, (long)full_weight, g_deviceParams.zero_weight_threshold_ratio);
//            printf("稳定重量 : %ld\r\n", (long)stable_weight);
//            printf("diff     : %ld\r\n", (long)diff);
//            printf("上限阈值 : %ld (ratio:%ld%%)\r\n",
//                   (long)upper_threshold, (long)g_deviceParams.weight_upper_limit_ratio);
//            printf("尺带长度 : %.1f mm\r\n", cable_mm);
//            printf("传感器位置 : %.1f mm\r\n", sensor_mm);
//            printf("================================\r\n");
//            return WEIGHT_COLLISION_DETECTED;
//        }
//
//        /* 2) 相对变重阈值 */
//        if (diff > upper_threshold)
//        {
//            printf("\r\n====== 称重碰撞报警(上行) ======\r\n");
//            printf("原因: 重量增加超过上限阈值\r\n");
//            printf("当前重量 : %ld\r\n", (long)cur_weight);
//            printf("稳定重量 : %ld\r\n", (long)stable_weight);
//            printf("重量增加 : %ld (阈值:%ld)\r\n", (long)diff, (long)upper_threshold);
//            printf("满载重量 : %ld (ratio:%ld%%)\r\n",
//                   (long)full_weight, (long)g_deviceParams.weight_upper_limit_ratio);
//            printf("零点阈值 : %ld (+%d%%)\r\n", (long)zero_limit, g_deviceParams.zero_weight_threshold_ratio);
//            printf("尺带长度 : %.1f mm\r\n", cable_mm);
//            printf("传感器位置 : %.1f mm\r\n", sensor_mm);
//            printf("================================\r\n");
//            return WEIGHT_COLLISION_DETECTED;
//        }
//
//#ifdef WEIGHT_DEBUG
//        printf("称重正常 | 方向:上行 | 当前:%ld | 稳定:%ld | 变化:%+ld | 零点阈值:%ld | 上限阈值:%ld | 满载:%ld | 尺带:%.1fmm | 位置:%.1fmm\r\n",
//               (long)cur_weight,
//               (long)stable_weight,
//               (long)diff,
//               (long)zero_limit,
//               (long)upper_threshold,
//               (long)full_weight,
//               cable_mm,
//               sensor_mm);
//
//#endif
//        return NO_ERROR;
//    }
//
//    /* ==========================
//     *       下行检测
//     * ========================== */
//    /* motor_dir == MOTOR_DIRECTION_DOWN */
//    {
//        /* 1) 罐底阈值（统一在此处判断，不调用外部函数） */
//        if (cur_weight < bottom_limit)
//        {
//            printf("\r\n====== 称重碰撞报警(下行) ======\r\n");
//            printf("原因: 低于触底阈值(认为已到罐底)\r\n");
//            printf("当前重量 : %ld\r\n", (long)cur_weight);
//            printf("触底阈值 : %ld\r\n", (long)bottom_limit);
//            printf("稳定重量 : %ld\r\n", (long)stable_weight);
//            printf("diff     : %ld\r\n", (long)diff);
//            printf("下限阈值 : %ld (ratio:%ld%%)\r\n",
//                   (long)lower_threshold, (long)g_deviceParams.weight_lower_limit_ratio);
//            printf("尺带长度 : %.1f mm\r\n", cable_mm);
//            printf("传感器位置 : %.1f mm\r\n", sensor_mm);
//            printf("================================\r\n");
//            return WEIGHT_COLLISION_DETECTED;
//        }
//
//        /* 2) 相对变轻阈值：diff 为负数，比较 -diff */
//        if (-diff > lower_threshold)
//        {
//            printf("\r\n====== 称重碰撞报警(下行) ======\r\n");
//            printf("原因: 重量减少超过下限阈值\r\n");
//            printf("当前重量 : %ld\r\n", (long)cur_weight);
//            printf("稳定重量 : %ld\r\n", (long)stable_weight);
//            printf("重量减少 : %ld (阈值:%ld)\r\n", (long)(-diff), (long)lower_threshold);
//            printf("满载重量 : %ld (ratio:%ld%%)\r\n",
//                   (long)full_weight, (long)g_deviceParams.weight_lower_limit_ratio);
//            printf("触底阈值 : %ld\r\n", (long)bottom_limit);
//            printf("尺带长度 : %.1f mm\r\n", cable_mm);
//            printf("传感器位置 : %.1f mm\r\n", sensor_mm);
//            printf("================================\r\n");
//            return WEIGHT_COLLISION_DETECTED;
//        }
//
//#ifdef WEIGHT_DEBUG
//        printf("称重正常 | 方向:下行 | 当前:%ld | 稳定:%ld | 变化:%+ld | 触底阈值:%ld | 下限阈值:%ld | 满载:%ld | 尺带:%.1fmm | 位置:%.1fmm\r\n",
//               (long)cur_weight,
//               (long)stable_weight,
//               (long)diff,
//               (long)bottom_limit,
//               (long)lower_threshold,
//               (long)full_weight,
//               cable_mm,
//               sensor_mm);
//#endif
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
	g_measurement.debug_data.current_weight = currWeight; // 更新当前重量到调试数据
	// 可选调试输出
	// printf("稳态称重: %ld\n", weightFilter.stableWeight);

	return;
}
