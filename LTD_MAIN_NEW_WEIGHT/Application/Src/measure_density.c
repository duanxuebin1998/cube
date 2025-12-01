/*
 * MeasureDensity.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */

#include "measure_density.h"
#include "motor_ctrl.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "system_parameter.h"

static uint32_t Density_SpreadMeasurement(DensityDistribution *dist);
// 分布测量主函数
void MeasureDensitySpread(void) {
	uint32_t ret = 0;
	// 设置设备状态：分布测量中
	g_measurement.device_status.device_state = STATE_SPREADPOINTING;

	ret = SearchOilLevel();

	if (ret != NO_ERROR) {
		printf("液位流程\t液位搜索失败，错误码:0x%lX\r\n", ret);
		return;
	} else
		printf("液位流程\t液位搜索成功\r\n");
// 切换传感器到密度测量模式（如果你已经在其他地方统一处理，这行可以删掉）
	EnableDensityMode();

// 执行一次分布密度测量，结果写入 g_measurement 中的分布结构体
	ret = Density_SpreadMeasurement(&g_measurement.density_distribution);

// 记录/上报错误码（和零点测量一样用 SET_ERROR）
	SET_ERROR(ret);

	HAL_Delay(1000);
	Print_DensitySpreadResult(&g_measurement.density_distribution);
// 测量结束，回到待机状态
	g_measurement.device_status.device_state = STATE_SPREADPOINTOVER;

	return;
}

/**
 * @brief  进行一次密度分布测量
 * @param  dist   [out] 测量结果输出到 DensityDistribution
 * @return 错误码，NO_ERROR 表示成功
 */
static uint32_t Density_SpreadMeasurement(DensityDistribution *dist) {
	if (dist == NULL) {
		return PARAM_ADDRESS_OVERFLOW;
	}

	memset(dist, 0, sizeof(DensityDistribution));

	uint32_t count = g_deviceParams.spreadMeasurementCount;      // 测量点数量
	uint32_t distance = g_deviceParams.spreadMeasurementDistance;   // 点间距 (mm)
	uint32_t top_limit = g_deviceParams.spreadTopLimit;              // 距液面的上限 (mm)
	uint32_t bottom_lim = g_deviceParams.spreadBottomLimit;           // 距罐底的下限 (mm)
	uint32_t order = g_deviceParams.spreadMeasurementOrder;      // 测量顺序
	uint32_t hover_time = g_deviceParams.spreadPointHoverTime;        // 单点悬停时间 (ms)

	if (count == 0 || count > MAX_MEASUREMENT_POINTS) {
		printf("分布测量 参数错误：count=%lu\n", (unsigned long) count);
		return PARAM_RANGE_ERROR;
	}

	/* TODO: 根据你的 MeasurementResult 结构体，确认油位字段名称和单位 */
	float oil_level_mm = (float) g_measurement.oil_measurement.oil_level / 10.0;  // 当前油位，单位 mm，从罐底向上
//	float oil_level_mm = (float) g_deviceParams.tankHeight / 10.0;  // 当前油位，单位 mm，从罐底向上

	if (oil_level_mm <= 0.0f) {
		printf("分布测量 错误：油位无效 oil_level=%.2f\n", oil_level_mm);
		return PARAM_RANGE_ERROR;
	}

	/* 计算允许的测量高度范围：
	 * h_bottom = bottom_lim
	 * h_top    = oil_level_mm - top_limit
	 */
	float h_bottom = (float) bottom_lim;             // 离罐底的最下测量位置
	float h_top = oil_level_mm - (float) top_limit; // 离罐底的最上测量位置

	if (h_top <= h_bottom) {
		printf("分布测量 范围错误：h_top<=h_bottom  h_top=%.2f  h_bottom=%.2f\n", h_top, h_bottom);
		return PARAM_RANGE_ERROR;
	}

	printf("分布测量 开始：油位=%.1fmm, 上限=%.1fmm, 下限=%.1fmm, 点数=%lu, 间距=%lumm\n", oil_level_mm, h_top, h_bottom, (unsigned long) count, (unsigned long) distance);

	float step = (float) distance;   // 步长（正数），方向后面根据 order 决定

	/* 计算起始高度和步进方向 */
	float pos_first = 0.0f;
	float step_signed = 0.0f;

	if (order == 0) {
		/* 从上往下：
		 * 第一个点靠近液面：pos_first = h_top
		 * 往下走：负步长
		 */
		pos_first = h_top;
		step_signed = -step;
	} else {
		/* 从下往上：
		 * 第一个点靠近罐底：pos_first = h_bottom
		 * 往上走：正步长
		 */
		pos_first = h_bottom;
		step_signed = step;
	}

	/* 开始循环测量 */
	uint32_t valid_points = 0;
	uint32_t sum_temp = 0.0;
	uint32_t sum_density = 0.0;
// 可选：如果你要统计频率、标准密度、vcf20 等，也可以在这里加累计

	for (uint32_t i = 0; i < count; i++) {

		float cur_pos = pos_first + step_signed * (float) i;

		/* 超出允许范围则直接停止 */
		if (cur_pos < h_bottom - 0.01f || cur_pos > h_top + 0.01f) {
			printf("分布测量 第%lu个点超出范围，停止测量。cur_pos=%.2f\n", (unsigned long) i, cur_pos);
			break;
		}

		printf("分布测量 移动到位置 %.2f mm\n", cur_pos);

		/* 移动到指定高度（绝对位置，单位 mm） */
		uint32_t ret = motorMoveToPositionOneShot(cur_pos);
		if (ret != NO_ERROR) {
			printf("分布测量 电机移动失败: pos=%.2f, err=%lu\n", cur_pos, (unsigned long) ret);
			return ret;  // 如果更温和，可以 break，然后用已有数据算平均
		}

		/* 悬停等待液体稳定 */
		if (hover_time > 0) {
			HAL_Delay(hover_time);
		}

		ret = SinglePoint_ReadSensor(&g_measurement.density_distribution.single_density_data[valid_points]);
//		ret = Read_Density(&freq, &density, &temp);
		if (ret != NO_ERROR) {
			printf("分布测量 读取密度失败: pos=%.2f, err=%lu\n", cur_pos, (unsigned long) ret);
			return ret;
		}

//		printf("分布测量 点%lu: pos=%.2fmm, dens=%.4f, temp=%.3f, freq=%.3f\n", (unsigned long) i, (float)g_measurement.debug_data.sensor_position/10.0, density, temp, freq);

		/* 将结果存入 DensityDistribution.single_density_data 中 */
		if (valid_points < MAX_MEASUREMENT_POINTS) {
			sum_temp += g_measurement.density_distribution.single_density_data[valid_points].temperature;
			sum_density += g_measurement.density_distribution.single_density_data[valid_points].density;
			valid_points++;
		}
	}

	/* 没有有效测点 */
	if (valid_points == 0) {
		printf("分布测量 未得到任何有效测点\n");
		return OTHER_UNKNOWN_ERROR;   // 或者自定义错误码
	}

	/* 统计平均值 */
	dist->measurement_points = valid_points;
	dist->Density_oil_level = (uint32_t) roundf(oil_level_mm);

	dist->average_temperature = (uint32_t) (sum_temp / valid_points); // 你按需要处理小数
	dist->average_density = (uint32_t) (sum_density / valid_points);

	dist->average_standard_density = dist->average_density;  //
	dist->average_vcf20 = 0;
	dist->average_weight_density = dist->average_density;

	printf("分布测量 完成：有效点数=%lu, AvgDensity=%lu(%.1f), AvgTemp=%.2f\n", (unsigned long) valid_points, (unsigned long) dist->average_density,
			RAW_TO_DENSITY(dist->average_density), RAW_TO_TEMP(dist->average_temperature));

	return NO_ERROR;
}
void Print_DensitySpreadResult(const DensityDistribution *dist) {
	if (dist == NULL) {
		printf("分布测量结果为空！\r\n");
		return;
	}

	printf("\r\n========== 密度分布测量结果 ==========\r\n");

	/* 总体信息 */
	printf("测量点数量         : %lu\r\n", (unsigned long) dist->measurement_points);
	printf("测量油位/罐高 (mm) : %lu\r\n", (unsigned long) dist->Density_oil_level);

	/* 平均值（注意：average_* 已经是编码后的 RAW） */
	printf("平均温度 RAW       : %lu  =>  实际: %.2f ℃\r\n", (unsigned long) dist->average_temperature, RAW_TO_TEMP(dist->average_temperature));

	printf("平均密度 RAW       : %lu  =>  实际: %.1f\r\n", (unsigned long) dist->average_density, RAW_TO_DENSITY(dist->average_density));

	printf("标准密度 RAW       : %lu  =>  实际: %.1f\r\n", (unsigned long) dist->average_standard_density, RAW_TO_DENSITY(dist->average_standard_density));

	printf("VCF20 RAW          : %lu\r\n", (unsigned long) dist->average_vcf20);

	printf("计重密度 RAW       : %lu  =>  实际: %.1f\r\n", (unsigned long) dist->average_weight_density, RAW_TO_DENSITY(dist->average_weight_density));

	printf("\r\n------ 单点数据列表 ------\r\n");
	printf("序号  位置(mm)  密度RAW  密度(实测)  温度RAW   温度(℃)\r\n");

	uint32_t n = dist->measurement_points;
	if (n > MAX_MEASUREMENT_POINTS) {
		n = MAX_MEASUREMENT_POINTS;  // 防御一下
	}

	for (uint32_t i = 0; i < n; i++) {
		const DensityMeasurement *p = &dist->single_density_data[i];

		float dens_f = RAW_TO_DENSITY(p->density);
		float temp_f = RAW_TO_TEMP(p->temperature);
		float temp_position = (float) (p->temperature_position) / 10.0;
		printf("%3lu   %.1f   %7lu   %8.1f   %7lu   %7.2f\r\n", (unsigned long) i, temp_position, (unsigned long) p->density, dens_f,
				(unsigned long) p->temperature, temp_f);
	}

	printf("=====================================\r\n\r\n");
}

//static uint32_t SinglePoint_ReadSensor(void) {
//	float density, viscosity, temp;
//
//	// 读取密度、温度
//	if (Read_Density(&density, &viscosity, &temp) == 0) {
//		printf("密度: %.3f  粘度: %.3f  温度: %.3f ℃\r\n", density, viscosity, temp);
//		g_measurement.single_point_monitoring.temperature_position = g_measurement.debug_data.sensor_position;
//		g_measurement.single_point_measurement.temperature_position = g_measurement.debug_data.sensor_position;
//	} else {
//		printf("读取密度/温度失败！\r\n");
//	}
//}
#include <math.h>   // 确保有这个

#define SINGLE_POINT_TIMEOUT_ERR   ((uint32_t)-1)  // -1 错误码

uint32_t SinglePoint_ReadSensor(DensityMeasurement *result) {
	uint32_t ret = 0;

	/* 稳定判定时间窗口：g_deviceParams.spreadPointHoverTime 秒 */
	uint32_t hover_time_s = g_deviceParams.spreadPointHoverTime;  // 单位：秒
	uint32_t stable_win_ms = hover_time_s * 1000U;

	if (stable_win_ms == 0) {
		// 防止设置为 0，给个默认值 5s
		stable_win_ms = 5000U;
	}

	/* 最大等待时间：5 分钟 */
	const uint32_t MAX_WAIT_MS = 5U * 60U * 1000U;  // 300000 ms

	/* 稳定阈值（根据现场调整） */
	const float FREQ_EPS = 1.0f;    // 频率允许波动范围 Hz
	const float DENSITY_EPS = 0.1f;  // 密度允许波动范围（按你的单位调）
	const float TEMP_EPS = 0.2f;   // 温度允许波动范围 ℃

	/* 采样间隔 */
	const uint32_t SAMPLE_INTERVAL_MS = 200U;   // 每 200ms 采集一次，可根据需要改

	uint32_t t_start = HAL_GetTick();   // 函数开始时间
	uint32_t stable_start = 0;               // 开始“稳定”的起点时间
	uint8_t first_sample = 1;               // 是否为第一帧数据

	float ref_freq = 0.0f;
	float ref_density = 0.0f;
	float ref_temp = 0.0f;

	float cur_freq = 0.0f;
	float cur_density = 0.0f;
	float cur_temp = 0.0f;

	while (1) {
		uint32_t now = HAL_GetTick();

		/* 整体超时判断：5 分钟 */
		if (now - t_start >= MAX_WAIT_MS) {
			printf("单点测量 5分钟内未得到稳定数据，超时退出。\r\n");
			return SINGLE_POINT_TIMEOUT_ERR;  // 返回 -1
		}

		/* 读取传感器：频率 / 密度 / 温度 */
//		 ret = Read_Density_text(&cur_freq, &cur_density, &cur_temp);
		ret = Read_Density(&cur_freq, &cur_density, &cur_temp);
		if (ret != NO_ERROR) {
			printf("读取密度/温度/频率失败：err=%lu\r\n", (unsigned long) ret);
			HAL_Delay(SAMPLE_INTERVAL_MS);
			continue;   // 通信失败当作不稳定，继续尝试
		}
		CHECK_COMMAND_SWITCH(ret);
		printf("单点读数: f=%.3f Hz  dens=%.4f  temp=%.3f ℃\r\n", cur_freq, cur_density, cur_temp);

		if (first_sample) {
			// 第一帧数据，作为参考值
			ref_freq = cur_freq;
			ref_density = cur_density;
			ref_temp = cur_temp;
			stable_start = now;
			first_sample = 0;
		} else {
			float df = fabsf(cur_freq - ref_freq);
			float dd = fabsf(cur_density - ref_density);
			float dt = fabsf(cur_temp - ref_temp);

			/* 只要有一项超出阈值，就认为“不稳定”，重新计时和更新参考值 */
			if (df > FREQ_EPS || dd > DENSITY_EPS || dt > TEMP_EPS) {
				ref_freq = cur_freq;
				ref_density = cur_density;
				ref_temp = cur_temp;
				stable_start = now;
			}
		}

		/* 如果已经有参考值，并且从 stable_start 到现在，持续稳定超过 stable_win_ms，则认为达到稳定 */
		if (!first_sample && (now - stable_start >= stable_win_ms)) {
			printf("单点测量 数据稳定，稳定窗口 = %lu ms\r\n", (unsigned long) stable_win_ms);
			printf("稳定结果: f=%.3f Hz  dens=%.4f  temp=%.3f ℃\r\n", ref_freq, ref_density, ref_temp);

			/* 保存稳定的结果到测量结构体中（根据你的结构体字段名调整） */
			result->temperature_position = g_measurement.debug_data.sensor_position;
			result->density = DENSITY_TO_RAW(ref_density);
			result->temperature = TEMP_TO_RAW(ref_temp);
			result->standard_density = DENSITY_TO_RAW(ref_density);
			result->vcf20 = 1;
			result->weight_density = DENSITY_TO_RAW(ref_density);

			return NO_ERROR;
		}

		HAL_Delay(SAMPLE_INTERVAL_MS);
	}
}

uint32_t SinglePointMeasurement() {
	uint32_t ret = 0;
	ret = motorMoveToPositionOneShot((float) g_deviceParams.singlePointMeasurementPosition / 10.0);
	CHECK_ERROR(ret);
	g_measurement.device_status.device_state = STATE_SPTESTING;
	EnableDensityMode();
	ret = SinglePoint_ReadSensor(&g_measurement.single_point_measurement);
	CHECK_ERROR(ret);
	return NO_ERROR;
}
uint32_t SinglePointMonitoring() {
	uint32_t ret = 0;
	ret = motorMoveToPositionOneShot((float) g_deviceParams.singlePointMonitoringPosition / 10.0);
	CHECK_ERROR(ret);
	g_measurement.device_status.device_state = STATE_SPTESTING;
	EnableDensityMode();
	while (1) {
		ret = SinglePoint_ReadSensor(&g_measurement.single_point_monitoring); // 传感器测试
		CHECK_ERROR(ret);
	}
}
