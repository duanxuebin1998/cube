/*
 * wartsila_density_measurement.c
 * 瓦西莱密度梯度测量
 *  Created on: 2025年12月1日
 *      Author: admin
 */


#include "wartsila_density_measurement.h"

#include "motor_ctrl.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "system_parameter.h"

static uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist);
// 分布测量主函数
void WartsilaDensitySpread(void) {
	uint32_t ret = 0;
	// 设置设备状态：分布测量中
	g_measurement.device_status.device_state = STATE_SPREADPOINTING;

	ret = Wartsila_Density_SpreadMeasurement(&g_measurement.density_distribution);
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
static uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist) {
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
