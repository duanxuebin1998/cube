/*
 * sensor.h
 *
 *  Created on: Mar 19, 2025
 *      Author: 1
 */

#ifndef SENSOR_SENSOR_H_
#define SENSOR_SENSOR_H_

#include <stdint.h>

#define DSM_DATAARRAY_OVER 1
#define DSM_ANGLE_UNVALIDVALUE 0
#define DSM_RESISTANCE_UNVALIDVALUE 0
#define DSM_SENSOR_UNVALID 0
#define DSM_ANGLEDMA 19
#define DSM_ANGLEGETONTIME_LENGTHMAX 50

#define DSM_RESISTANCESTART "CL"	 // 开启读取电阻
#define DSM_RESISTANCEGET "Cl"		 // 读取电阻数据
#define DSM_SENSORSTART "CD"		 // 开启传感器
#define DSM_SENSORGET "Cd"			 // 读取传感器数据
#define DSM_SENSORFASTSTART "CF"	 // 开启传感器
#define DSM_SENSORFASTGET "Cf"		 // 读取传感器数据
#define DSM_ZEROSTART "CZ"			 // 开启零点读取
#define DSM_ZEROGET "Cz"			 // 读取零点数据
#define DSM_VERSIONSTART "CV"		 // 读取传感器版本
#define DSM_CMDREPLY "%\r\n"		 // 开启命令返回
#define DSM_SWEEP_AB "CS"			 // 读取扫频数据
#define DSM_DENSITYPARAMETER "CM"	 // 读取密度分析参数
#define DSM_NUMBER "CN"				 // 读取震动管编号
#define DSM_POWER "CK"				 // 读取供电电压
#define DSM_GET_FREQUENCE_START "CB" // 准备读取零点空气密度计频率
#define DSM_GET_FREQUENCE "Cb"		 // 读取零点空气密度计频率

#define DSM_SONIC_AD_ERROR "A+111.11B+111.11" // 超声无谐振V1.130
#define DSM_SONIC_AD_ERROR_COMTIMEOUT    	"A+222.22B+222.22" //超声无谐振V1.130
#define DSM_SONIC_GYO_ERROR_COMTIMEOUT "A+333.33B+333.33"		// 超声无谐振V1.130
#define DSM_SONIC_ERROR_COMTIMEOUT "A+666.66B+666.66"			// 读取超声波传感器无数据//V1.118
#define DSM_SONIC_ERROR_COMBCC "A+777.77B+777.77"				// 读取超声波传感器校验错误//V1.118
#define DSM_ANGLE_ERROR_COMTIMEOUT "A+888.88B+888.88"			// 传感器读取陀螺仪失败
#define DSM_ANGLE_ERROR_COMBCC "A+999.99B+999.99"				// 传感器读取陀螺仪校验错误
#define DSM_CPU0_ERROR_COMTIMEOUT "A+888.88B+888.88"			/* 与CPU0通讯超时 */
#define DSM_CPU0_ERROR_COMBCC "A+999.99B+999.99"				/* 与CPU0通讯校验错误 */
#define DSM_SENSOR_ERROR_7915TIMEOUT "F8888.88V88.8888T+888.88" // 传感器读取频率错误(与AD通讯不上)第二代数字驱动无此故障；
#define DSM_DENSITY_ERROR_COMTIMEOUT "F8888.88V88.8888T+888.88" // 传感器STM32和430通讯不上（第二代数字驱动V2.0以上程序版本）
#define DSM_DENSITY_ERROR_COMBCC "F9999.99V99.9999T+999.99"		// 传感器STM32和430通讯校验错误（第二代数字驱动V2.0以上程序版本）

#define RCVBUFFLEN 32
#define DSM_UNVALIDDATA 0x00
#define DSM_ANGLEGETONTIME_LENGTHMAX 50

struct DSMSENSOR_DATA
{
	float Sensor_Frequency;
	float Sensor_Temperature;
	float Density;
	float Dynamic_Viscosity;
	float MeanSquareOf225DegreeSweepPeriod;
	float SquareMeanOf45degreeSweepPeriod;
	float Power_Voltage;
	float level_frequency;
	float capacitance;
};

int DSMSendcommand3times(uint8_t *pCommand, uint16_t commandLen);
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
