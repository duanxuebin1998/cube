/*
 * sensor.h
 *
 *  Created on: Mar 19, 2025
 *      Author: 1
 */

#ifndef SENSOR_SENSOR_H_
#define SENSOR_SENSOR_H_

#include <stdint.h>

//#define DSM_DATAARRAY_OVER 1
//#define DSM_ANGLE_UNVALIDVALUE 0
//#define DSM_RESISTANCE_UNVALIDVALUE 0
//#define DSM_SENSOR_UNVALID 0
//#define DSM_ANGLEDMA 19
//#define DSM_ANGLEGETONTIME_LENGTHMAX 50


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
//#define DSM_UNVALIDDATA 0x00
//#define DSM_ANGLEGETONTIME_LENGTHMAX 50

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
//int DSM_GetSonicverage(float *sonic_a_average, float *sonic_b_average,
//		float *sonic_c_average, int count); // V1.118
//int DSM_AngleStart(void);
//int DSM_AngleGet(float *gyroscopevalue_x, float *gyroscopevalue_y);
//int DSM_ResistanceStart(void);
//int DSM_ResistanceGet(float *resistancevalue);
//
//int DSM_ZeroStart(void);
//int DSM_ZeroGet(float *resistancevalue);
//int DSM_GetZeroeAverage(float *resistance_average, int count);
//int DSM_SensorStart(void);
//int DSM_SensorGet(struct DSMSENSOR_DATA *sensorvalue);
//
//int DSM_AngleGetDataLoop_Start(unsigned int ms, unsigned int count);
//int DSM_AngleSendCMDData(void);
//
//int DSM_GetAngleXverage(float *gyro_anglex_average, int count);
//int DSM_GetAngleYverage(float *gyro_angley_average, int count);
//int DSM_GetResistanceAverage(float *resistance_average, int count);
//
//int DSM_Sensorfast_Start(void);								// 启动快速读取频率值
//int DSM_Sensorfast_Get(struct DSMSENSOR_DATA *sensorvalue); // 读取频率值
//int DSM_GetFreverage(float *fre, int count);				// 读取平均值，最大不超过100
//int water_version(char *command, unsigned int commandlen);	// 判断水位版本
//int DSM_WaterStart(void);									// 读取水位版本号
//int DSM_SonicFrequency_Get(float *sonicFrequency1, float *sonicFrequency2);
//int DSM_SonicAD_Get_SIL(float *Sonic_A, float *Sonic_B, float *Sonic_C);
//int DSM_SensorGet_SIL(struct DSMSENSOR_DATA *sensorvalue);
//int DSM_DensityParameterGet_SIL(struct DSMSENSOR_DATA *sensorvalue); /* 获取密度分析参数 */
//int DSM_SenserNumGet(void);
//int DSM_SenserVoltageGet(void);
//int DSM_Switch_LevelMode(void);
//int DSM_Get_LevelMode_Frequence(float *Frequency);
//int DSM_CapacitanceGet(float *capacitance);

#endif /* SENSOR_SENSOR_H_ */
