/*
 * dsm_sensor_communication.h
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */

#ifndef SENSOR_DSM_SENSOR_COMMUNICATION_H_
#define SENSOR_DSM_SENSOR_COMMUNICATION_H_

#include <stddef.h>
#include "sensor.h"

#define DSM_WATER_CAP "Cl"		 /* 读取测水电容数据 */

#define DSM_SENSORSTART "CD"		 /* 切换密度模式 */
#define DSM_SENSORGET "Cd"			 /* 读取传感器数据 */

#define DSM_VERSIONSTART "CV"		 /* 读取传感器版本 */
#define DSM_DENSITYPARAMETER "CM"	 /* 读取密度分析参数 */
#define DSM_NUMBER "CN"				 /* 读取震动管编号 */
#define DSM_POWER "CK"				 /* 读取供电电压 */

#define DSM_GET_FREQUENCE_START "CB" /* 切换液位模式 */
#define DSM_GET_FREQUENCE "Cb"		 /* 读取零点空气密度计频率 */

#define DSM_CMDREPLY "%\r\n"		 /* 开启命令返回 */

#define RCVBUFFLEN 32 /* 传感器通信参数：接收缓冲长度。 */


typedef struct
{
	float Sensor_Frequency; /* 频率 */
	float Sensor_Temperature; /* 温度 */
	float Density; /* 密度 */
	float Dynamic_Viscosity; /* 动力粘度 */
	float MeanSquareOf225DegreeSweepPeriod; /* 225度扫频均方根 */
	float SquareMeanOf45degreeSweepPeriod; /* 45度扫频均方根 */
	float Power_Voltage; /* 电压 */
	float level_frequency; /* 液位频率 */
	float capacitance; /* 电容 */
}DSMSENSOR_DATA;
extern DSMSENSOR_DATA dsmsensor_data;
/* int DSMSendcommand3times(uint8_t *pCommand, uint16_t commandLen); */

int DSM_EnableLevelMode(void);
/**
 * @brief 将 DSM 传感器切换到密度测量模式。
 * @return 0 表示成功，非 0 表示通信或模式切换失败。
 */
int DSM_EnableDensityMode(void);
/**
 * @brief 读取 DSM 传感器供电电压。
 * @param voltage_out 电压输出指针。
 * @return NO_ERROR 表示读取成功，其他值表示通信异常。
 */
uint32_t Read_Sensor_Voltage(float *voltage_out);
/**
 * @brief 读取DSM 传感器通信中的 Read_Level_Frequency 逻辑。
 *
 * @param frequency_out 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Read_Level_Frequency(uint32_t *frequency_out);
/**
 * @brief 执行DSM 传感器通信中的 Probe_EnableWaterSensor 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int Probe_EnableWaterSensor(void);
/**
 * @brief 读取DSM 传感器通信中的 DSM_Read_Frequency_Density_Temp 逻辑。
 *
 * @param frequency 业务参数。
 * @param density 业务参数。
 * @param temp 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_Read_Frequency_Density_Temp(float *frequency,float *density, float *temp);
/**
 * @brief 读取DSM 传感器通信中的 Read_VibrationTube_ID 逻辑。
 *
 * @param id_out 业务参数。
 * @param id_out_size 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Read_VibrationTube_ID(char *id_out, size_t id_out_size);
/**
 * @brief 读取DSM 传感器通信中的 Read_Water_Capacitance 逻辑。
 *
 * @param cap_out 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Read_Water_Capacitance(float *cap_out);
/**
 * @brief 读取DSM 传感器通信中的 Read_Gyro_Angle 逻辑。
 *
 * @param angle_x_deg 业务参数。
 * @param angle_y_deg 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Read_Gyro_Angle(float *angle_x_deg, float *angle_y_deg);
#endif /* SENSOR_DSM_SENSOR_COMMUNICATION_H_ */
