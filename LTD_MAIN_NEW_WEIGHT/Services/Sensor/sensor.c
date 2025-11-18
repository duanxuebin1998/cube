/*
 * @FilePath     : \KEILe:\03CodeRepository\DSM_MCB\HARDWARE\SENSOR\sensor.c
 * @Description  : 传感器通信相关函数
 * @Author       : Aubon
 * @Date         : 2024-02-23 10:20:27
 * @LastEditors  : Duan
 * @LastEditTime : 2024-11-15 15:36:58
 * Copyright 2024 Aubon, All Rights Reserved.
 * 2024-02-23 10:20:27
 */

#include "sensor.h"

uint32_t EnableDensityMode(void) {
	if (g_deviceParams.sensorType == DSM_SENSOR) {
		return DSM_EnableDensityMode();
	} else {
		return DSM_V2_SwitchToDensityMode();
	}
}
uint32_t EnableLevelMode(void) {
	if (g_deviceParams.sensorType == DSM_SENSOR) {
		return DSM_EnableLevelMode();
	} else {
		return DSM_V2_SwitchToLevelMode();
	}
}
// 读取一次并以整数 Hz 返回
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency_out) {
	uint32_t ret;
	if (!frequency_out)
		return SENSOR_COMM_TIMEOUT;

	uint32_t hz = 0;
	if (g_deviceParams.sensorType == DSM_SENSOR) {
		ret = Read_Level_Frequency(&hz);
	} else {
		ret = DSM_V2_Read_LevelFrequency(&hz);
	}
	if (ret != NO_ERROR)
		return ret;

	*frequency_out = hz;

	printf("液位频率: %ld Hz\r\n", *frequency_out);
	return NO_ERROR;
}

/**
 * @brief 获取液位跟随频率的平均值（10 次采样，2s 间隔，去 2 大 2 小，取中间 6 次均值）
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency_out) {
	if (!frequency_out)
		return SENSOR_COMM_TIMEOUT;

	uint32_t values[10];
	uint32_t ret;

	for (int i = 0; i < 10; i++) {
		ret = DSM_Get_LevelMode_Frequence(&values[i]);
		if (ret != NO_ERROR) {
			printf("读取液位频率失败，第 %d 次\r\n", i + 1);
			return ret;
		}
		printf("第 %d 次液位频率: %ld Hz\r\n", i + 1, values[i]);
		HAL_Delay(2000); // 2 秒间隔
	}

	// 冒泡排序（升序）
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9 - i; j++) {
			if (values[j] > values[j + 1]) {
				float tmp = values[j];
				values[j] = values[j + 1];
				values[j + 1] = tmp;
			}
		}
	}

	// 去掉两个最大与两个最小
	float sum = 0.0f;
	for (int i = 2; i < 8; i++)
		sum += (float) values[i];

	float avg = sum / 6.0f;
	uint32_t avg_u32 = (avg >= 0.0f) ? (uint32_t) (avg + 0.5f) : 0u;
	*frequency_out = avg_u32;

	printf("液位频率平均值(去极值): %ld Hz\r\n", *frequency_out);
	return NO_ERROR;
}
uint32_t Read_Density(float *density, float *viscosity, float *temp) {
	uint32_t ret;
	if (g_deviceParams.sensorType == DSM_SENSOR) {
		ret = Read_Density_Temp(density, viscosity, temp);
	} else {
		ret = Read_Density_SIL(density, viscosity, temp);
	}
	// 读取密度、温度
	if (ret == 0) {
		printf("密度: %.2f  粘度: %.3f  温度: %.3f ℃\r\n", *density, *viscosity, *temp);
		g_measurement.single_point_monitoring.density = (uint32_t) (*density * 10.0f);
		g_measurement.single_point_monitoring.temperature = (uint32_t) (*temp * 100.0f + 20000.0f);
		g_measurement.single_point_monitoring.temperature_position = g_measurement.debug_data.sensor_position;
		g_measurement.single_point_measurement.density = (uint32_t) (*density * 10.0f);
		g_measurement.single_point_measurement.temperature = (uint32_t) (*temp * 100.0f + 20000.0f);
		g_measurement.single_point_measurement.temperature_position = g_measurement.debug_data.sensor_position;
	} else {
		printf("读取密度/温度失败！\r\n");
	}
	return ret;
}
