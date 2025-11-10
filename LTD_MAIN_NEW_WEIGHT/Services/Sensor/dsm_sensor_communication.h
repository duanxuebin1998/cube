/*
 * dsm_sensor_communication.h
 *
 *  Created on: Nov 10, 2025
 *      Author: admin
 */

#ifndef SENSOR_DSM_SENSOR_COMMUNICATION_H_
#define SENSOR_DSM_SENSOR_COMMUNICATION_H_

#include "sensor.h"

#define DSM_WATER_CAP "Cl"		 // 读取测水电容数据

#define DSM_SENSORSTART "CD"		 // 切换密度模式
#define DSM_SENSORGET "Cd"			 // 读取传感器数据

#define DSM_VERSIONSTART "CV"		 // 读取传感器版本
#define DSM_DENSITYPARAMETER "CM"	 // 读取密度分析参数
#define DSM_NUMBER "CN"				 // 读取震动管编号
#define DSM_POWER "CK"				 // 读取供电电压

#define DSM_GET_FREQUENCE_START "CB" // 切换液位模式
#define DSM_GET_FREQUENCE "Cb"		 // 读取零点空气密度计频率

#define DSM_CMDREPLY "%\r\n"		 // 开启命令返回

#define RCVBUFFLEN 32


typedef struct
{
	float Sensor_Frequency;// 频率
	float Sensor_Temperature;// 温度
	float Density;// 密度
	float Dynamic_Viscosity;// 动力粘度
	float MeanSquareOf225DegreeSweepPeriod;// 225度扫频均方根
	float SquareMeanOf45degreeSweepPeriod;// 45度扫频均方根
	float Power_Voltage;// 电压
	float level_frequency;// 液位频率
	float capacitance;// 电容
}DSMSENSOR_DATA;
extern DSMSENSOR_DATA dsmsensor_data;
int DSMSendcommand3times(uint8_t *pCommand, uint16_t commandLen);

int EnableLevelMode(void);
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency);
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency);
int Probe_EnableWaterSensor(void);
int Read_Density_Temp(float *density, float *viscosity, float *temp);
int Read_Density_SIL(float *density, float *viscosity, float *temp);// 读取超声小管的密度、温度

#endif /* SENSOR_DSM_SENSOR_COMMUNICATION_H_ */
