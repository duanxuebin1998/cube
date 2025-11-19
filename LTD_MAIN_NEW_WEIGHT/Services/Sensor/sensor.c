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
/**
 * @brief 自动识别传感器类型（DSM 一代 / DSM_V2 / SIL）
 *
 * @return uint32_t 错误码或 NO_ERROR
 */
uint32_t DetectSensorType(void)
{

}

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
uint32_t DSM_Get_LevelMode_Frequence(uint32_t *frequency_out) {
    if (frequency_out == NULL) {
        return PARAM_ADDRESS_OVERFLOW;   // 比 SENSOR_COMM_TIMEOUT 更合理
    }

    uint32_t ret;
    uint32_t hz = 0;

    if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = Read_Level_Frequency(&hz);
    } else {
        ret = DSM_V2_Read_LevelFrequency(&hz);
    }

    if (ret != NO_ERROR) {
        return ret;
    }

    *frequency_out = hz;

    printf("液位频率: %lu Hz\r\n", (unsigned long)*frequency_out);
    return NO_ERROR;
}

/**
 * @brief 获取液位跟随频率的平均值
 *        （10 次采样，2s 间隔，去 2 大 2 小，取中间 6 次均值）
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(uint32_t *frequency_out) {
    if (frequency_out == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    uint32_t values[10];
    uint32_t ret;

    for (int i = 0; i < 10; i++) {
        ret = DSM_Get_LevelMode_Frequence(&values[i]);
        if (ret != NO_ERROR) {
            printf("读取液位频率失败，第 %d 次\r\n", i + 1);
            return ret;
        }
        printf("第 %d 次液位频率: %lu Hz\r\n", i + 1, (unsigned long)values[i]);
        HAL_Delay(2000); // 2 秒间隔（标定场景可接受）
    }

    // 冒泡排序（升序）
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9 - i; j++) {
            if (values[j] > values[j + 1]) {
                uint32_t tmp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = tmp;
            }
        }
    }

    // 去掉两个最大与两个最小
    float sum = 0.0f;
    for (int i = 2; i < 8; i++) {
        sum += (float)values[i];
    }

    float avg = sum / 6.0f;
    uint32_t avg_u32 = (avg >= 0.0f) ? (uint32_t)(avg + 0.5f) : 0u;
    *frequency_out = avg_u32;

    printf("液位频率平均值(去极值): %lu Hz\r\n", (unsigned long)*frequency_out);
    return NO_ERROR;
}

#define TEMP_TO_RAW(t)  ((uint32_t)((t) * 100.0f + 20000.0f))
#define DENSITY_TO_RAW(d) ((uint32_t)((d) * 10.0f))

uint32_t Read_Density(float *density, float *viscosity, float *temp) {
    if (density == NULL || viscosity == NULL || temp == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    uint32_t ret;
    if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = Read_Density_Temp(density, viscosity, temp);
    } else {
        ret = Read_Density_SIL(density, viscosity, temp);
    }

    if (ret == NO_ERROR) {
        printf("密度: %.2f  粘度: %.3f  温度: %.3f ℃\r\n",
               *density, *viscosity, *temp);

        uint32_t density_raw = DENSITY_TO_RAW(*density);
        uint32_t temp_raw    = TEMP_TO_RAW(*temp);
        uint32_t pos         = g_measurement.debug_data.sensor_position;

        g_measurement.single_point_monitoring.density             = density_raw;
        g_measurement.single_point_monitoring.temperature         = temp_raw;
        g_measurement.single_point_monitoring.temperature_position = pos;

        g_measurement.single_point_measurement.density             = density_raw;
        g_measurement.single_point_measurement.temperature         = temp_raw;
        g_measurement.single_point_measurement.temperature_position = pos;
    } else {
        printf("读取密度/温度失败！\r\n");
    }

    return ret;
}
