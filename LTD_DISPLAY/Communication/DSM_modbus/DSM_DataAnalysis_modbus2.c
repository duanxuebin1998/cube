#include "DSM_DataAnalysis_modbus2.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "DSM_comm.h"
#include "spi.h"
#include "DSM_communication.h"
#include "DSM_stateformodbus2.h"
#include "DSM_system_parameter.h"
#include "system_parameter.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*
通讯的应用层，对数据进行接卸或者写入。
*/
//#define DEBUG_DA
#ifdef DEBUG_DA
	#include "usart1.h"
#endif

extern int SlaveAddress;           //下位机地址

int StateofAverageTemperature;
int ValueofAverageTemperature;
int gloabalindex;

/******************************************************
函数功能： 清空线圈数组

函 数 名： ResetCoil
参    数：

返 回 值：
******************************************************/
int ResetCoil(void)
{
	unsigned int i;

	for(i = 0; i < COILAMOUNT; ++i)
	{
		CoilArray[i] = 0;
	}

	return RETURN_OK;
}
/******************************************************
函数功能： 设置寄存器初始值

函 数 名： Parameter_Init
参    数：

返 回 值：
******************************************************/
void Parameter_Init(void)
{
	u8 i;
//#define UNVALID_LEVEL 999999u //液位无效值
//#define LEVEL_DOWNLIMIT 100u //盲区液位值
//#define UNVALID_TEMPERATURE -2000//温度无效值
//#define UNVALID_DENSITY 0 //密度无效值
//#define LEVEL_DOWNLIMITWATER 0//水位无效值
//#define UNVALID_QUALITY 0//质量无效值
//#define UNVALID_VOLUME 0//体积无效值
//#define UNVALID_X 0//暂用于流速流量无效值
	WriteOneHoldingRegister(HOLDREGISTER_SP_POSITION, 2, 0);			//单点测量位置 默认值
	WriteOneHoldingRegister(HOLDREGISTER_SPT_POSITION, 2, 0);			//单点监测位置 默认值
	WriteOneHoldingRegister(HOLDREGISTER_SRREAD_POSITION, 2, 0);			//分布测量位置 默认值
	WriteOneHoldingRegister(HOLDREGISTER_CALIBRATIONLIQUIDLEVEL, 2, 0);

	WriteOneInputRegister(INPUTREGISTER_PATTERNOFWORK, 1, 0);				 					//工作模式
	WriteOneInputRegister(INPUTREGISTER_ERRORNUM, 2, 0);											//故障代码
	WriteOneInputRegister(INPUTREGISTER_SP_TEMPERATURE, 1, 0);									//单点测量温度
	WriteOneInputRegister(INPUTREGISTER_SP_DENSITY, 1, 0);									//单点测量密度
	WriteOneInputRegister(INPUTREGISTER_SP_POSITION, 2, 0xFFFFFFFF);										//单点测量密度点位置
	WriteOneInputRegister(INPUTREGISTER_SP_STANDARDDENSITY, 1, 0);								//单点测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SPT_TEMPERATURE, 1, 0);									//单点检测温度
	WriteOneInputRegister(INPUTREGISTER_SPT_DENSITY, 1, 0);										//单点检测密度
	WriteOneInputRegister(INPUTREGISTER_SPT_POSITION, 2, 0xFFFFFFFF);										//单点检测密度点位置
	WriteOneInputRegister(INPUTREGISTER_SPT_STANDARDDENSITY, 1, 0);								//单点检测标准密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGETEMPERATURE, 1, 0);					//平均温度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGEDENSITY, 1, 0);							//平均密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY, 1, 0);							//分布测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF20, 1, 0);									//分布测量VCF
	WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY, 1, 0);								//分布测量计重密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_NUMOFDENSITY, 1, 0);								//分布测量密度点数
	WriteOneInputRegister(INPUTREGISTER_LIQUIDLEVEL, 2, 0xFFFFFFFF);			//分布测量液位值 默认值 孟
	WriteOneInputRegister(INPUTREGISTER_OILLEVEL, 2, 0xFFFFFFFF);			//液位值 默认值
	WriteOneInputRegister(INPUTREGISTER_WATERLEVEL, 2, 0xFFFFFFFF);			//水位值 默认值
	WriteOneInputRegister(INPUTREGISTER_BOTTOMHIGH, 2, 0xFFFFFFFF);			//罐底值 默认值
	WriteOneInputRegister(INPUTREGISTER_VALUE_P1, 2, 0);			//液体压力值 赋值 孟
	WriteOneInputRegister(INPUTREGISTER_VALUE_P3, 2, 0);
	WriteOneInputRegister(INPUTREGISTER_REALTIMEPOSITION, 2, 0xFFFFFFFF);			//传感器实时 默认值

	for(i = 0; i < 16; i++)
	{
		WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE1 + 8 * i, 1, UNVALID_TEMPERATURE);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY1 + 8 * i, 1, UNVALID_DENSITY);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION1 + 8 * i, 2, UNVALID_POSITION);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY1 + 8 * i, 1, UNVALID_DENSITY);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF1 + 8 * i, 2, UNVALID_VCF);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY1 + 8 * i, 1, UNVALID_DENSITY);
	}

	WriteOneInputRegister(INPUTREGISTER_ZEROCIRCLE, 1, 0);										//零点圈数
	WriteOneInputRegister(INPUTREGISTER_ZEROANGLE, 1, 0);											//零点角度
	WriteOneInputRegister(INPUTREGISTER_TANKHIGHT, 2, 200000);									//罐高
	WriteOneInputRegister(INPUTREGISTER_CIRCLE, 1, 0);											//当前圈数
	WriteOneInputRegister(INPUTREGISTER_ANGLE, 1, 0);												//当前角度
	WriteOneInputRegister(INPUTREGISTER_SENSORPOSITION, 2, 0xFFFFFFFF);							//传感器位置
	WriteOneInputRegister(INPUTREGISTER_FREQUENCE, 2, 0);											//传感器频率
	WriteOneInputRegister(INPUTREGISTER_TEMPERATURE, 1, UNVALID_TEMPERATURE);									//传感器温度
	WriteOneInputRegister(INPUTREGISTER_STATEOFEINDUCTION, 1, 0);									//干簧管状态
	WriteOneInputRegister(INPUTREGISTER_FREQUENCEINAIR, 2, 0);									//空气中频率
}



/******************************************************
函数功能： 设置寄存器赋值

函 数 名： Input_Write
参    数：

返 回 值：
******************************************************/
void Input_Write(void)
{
	int i;
	WriteOneInputRegister(INPUTREGISTER_PATTERNOFWORK, 1, Measure_One.PatternOfWork);				 					//工作模式
	WriteOneInputRegister(INPUTREGISTER_SYSTEMSTATE, 1, Measure_One.EquipmentState);									//工作状态
	WriteOneInputRegister(INPUTREGISTER_ERRORNUM, 2, Measure_One.EquipmentErrorCode);											//故障代码
	WriteOneInputRegister(INPUTREGISTER_SP_TEMPERATURE, 1, Measure_One.Measure_TempOnePoint);									//单点测量温度
	WriteOneInputRegister(INPUTREGISTER_SP_DENSITY, 1, Measure_One.Measure_DensityOnePoint);									//单点测量密度
	WriteOneInputRegister(INPUTREGISTER_SP_POSITION, 2, Measure_One.Measure_OnePointPosition);										//单点测量密度点位置
	WriteOneInputRegister(INPUTREGISTER_SP_STANDARDDENSITY, 1, Measure_One.Measure_Density20OnePoint);								//单点测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SP_VCF20, 2, Measure_One.Measure_VCF20OnePoint);											//单点测量的VCF
	WriteOneInputRegister(INPUTREGISTER_SP_WEIGHTDENSITY, 1, Measure_One.Measure_DensityTOnePoint);						//单点测量的计重密度

	WriteOneInputRegister(INPUTREGISTER_SPT_TEMPERATURE, 1, Measure_One.Monitor_TempOnePoint);									//单点检测温度
	WriteOneInputRegister(INPUTREGISTER_SPT_DENSITY, 1, Measure_One.Monitor_DensityOnePoint);										//单点检测密度
	WriteOneInputRegister(INPUTREGISTER_SPT_POSITION, 2, Measure_One.Monitor_OnePointPosition);										//单点检测密度点位置
	WriteOneInputRegister(INPUTREGISTER_SPT_STANDARDDENSITY, 1, Measure_One.Monitor_Density20OnePoint);								//单点检测标准密度
	WriteOneInputRegister(INPUTREGISTER_SPT_VCF20, 2, Measure_One.Monitor_VCF20OnePoint);
	WriteOneInputRegister(INPUTREGISTER_SPT_WEIGHTDENSITY, 1, Measure_One.Monitor_DensityTOnePoint);


	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGETEMPERATURE, 1, Measure_One.Spread_TempAverage);					//平均温度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGEDENSITY, 1, Measure_One.Spread_DensityAverage);							//平均密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY, 1, Measure_One.Spread_Density20Average);							//分布测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF20, 2, Measure_One.Spread_Vcf20Average);									//分布测量VCF
	WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY, 1, Measure_One.Spread_DensityTAverage);								//分布测量计重密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_NUMOFDENSITY, 1, Measure_One.Spread_Points);								//分布测量密度点数
	WriteOneInputRegister(INPUTREGISTER_LIQUIDLEVEL, 2, Measure_One.Spread_OilLevel);				//分布测量液位值
	WriteOneInputRegister(INPUTREGISTER_OILLEVEL, 2, Measure_One.MeasureOil_Level);			//液位值 赋值
	WriteOneInputRegister(INPUTREGISTER_WATERLEVEL, 2, Measure_One.MeasureWater_Level);			//水位值 赋值
	WriteOneInputRegister(INPUTREGISTER_BOTTOMHIGH, 2, Measure_One.MeasureBottom_Level);			//罐底值 赋值
	WriteOneInputRegister(INPUTREGISTER_VALUE_P1, 2, Measure_One.PressValue1);			//液体压力值 赋值 孟
	WriteOneInputRegister(INPUTREGISTER_REALTIMEPOSITION, 2, Measure_One.Read_AM4096_Position);			//传感器实饰恢帽 赋值

	if(Measure_One.Spread_Points > 16) //V1.116 dq2020.4.2
	{
		for(i = 0; i < 16; i++)
		{
			WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE1 + 8 * i, 1, Result.Temperature[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY1 + 8 * i, 1, Result.Density[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION1 + 8 * i, 2, Result.Postion[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY1 + 8 * i, 1, Result.Density20[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF1 + 8 * i, 2, Result.Vcf[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY1 + 8 * i, 1, Result.DensityT[i + 1]);
		}

		for(i = 0; i < Measure_One.Spread_Points - 16; i++)
		{
			WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE17 + 8 * i, 1, Result.Temperature[i + 17]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY17 + 8 * i, 1, Result.Density[i + 17]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION17 + 8 * i, 2, Result.Postion[i + 17]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY17 + 8 * i, 1, Result.Density20[i + 17]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF17 + 8 * i, 2, Result.Vcf[i + 17]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY17 + 8 * i, 1, Result.DensityT[i + 17]);
		}
	}
	else
	{
		for(i = 0; i < Measure_One.Spread_Points; i++)
		{
			WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE1 + 8 * i, 1, Result.Temperature[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY1 + 8 * i, 1, Result.Density[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION1 + 8 * i, 2, Result.Postion[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY1 + 8 * i, 1, Result.Density20[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF1 + 8 * i, 2, Result.Vcf[i + 1]);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY1 + 8 * i, 1, Result.DensityT[i + 1]);
		}
	}

	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGETEMPERATURE_M, 1, Measure_Meter.Spread_TempAverage);					//每米平均温度 V1.116 dq2020.4.2
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGEDENSITY_M, 1, Measure_Meter.Spread_DensityAverage);							//每米平均密度 V1.116 dq2020.4.2
	WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY_M, 1, Measure_Meter.Spread_Density20Average);							//每米测量标准密度 V1.116 dq2020.4.2
	WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF20_M, 2, Measure_Meter.Spread_Vcf20Average);									//每米测量VCF V1.116 dq2020.4.2
	WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY_M, 1, Measure_Meter.Spread_DensityTAverage);								//每米测量计重密度 V1.116 dq2020.4.2
	WriteOneInputRegister(INPUTREGISTER_SPREAD_NUMOFDENSITY_M, 1, Measure_Meter.Spread_Points);								//每米测量密度点数 V1.116 dq2020.4.2
	WriteOneInputRegister(INPUTREGISTER_LIQUIDLEVEL_M, 2, Measure_Meter.Spread_OilLevel);				//每米分布测量液位值 V1.116 dq2020.4.2

	for(i = 0; i < Measure_Meter.Spread_Points; i++) //V1.116 dq2020.4.2
	{
		WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE_M1 + 8 * i, 1, Educt.Temperature[i + 1]);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY_M1 + 8 * i, 1, Educt.Density[i + 1]);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION_M1 + 8 * i, 2, Educt.Postion[i + 1]);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY_M1 + 8 * i, 1, Educt.Density20[i + 1]);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF_M1 + 8 * i, 2, Educt.Vcf[i + 1]);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY_M1 + 8 * i, 1, Educt.DensityT[i + 1]);
	}

	WriteOneInputRegister(INPUTREGISTER_ZEROCIRCLE, 1, 0);									//零点圈数
	WriteOneInputRegister(INPUTREGISTER_ZEROANGLE, 1, 0);										//零点角度
	WriteOneInputRegister(INPUTREGISTER_TANKHIGHT, 2, TankHigh_LevelCorrection);							//修正罐高
	WriteOneInputRegister(INPUTREGISTER_CIRCLE, 1, measure_debug_area.X_Angele + 0X8000);										//当前圈数(0X7F9C是软件里面做的运算) 改为X轴角度
	WriteOneInputRegister(INPUTREGISTER_ANGLE, 1, measure_debug_area.Y_Angele);											//当前角度  改为Y轴角度
	WriteOneInputRegister(INPUTREGISTER_SENSORPOSITION, 2, measure_debug_area.Am4096_Position);			//传感器位置
	WriteOneInputRegister(INPUTREGISTER_FREQUENCE, 2, measure_debug_area.Sensor_Fre);										//传感器频率
	WriteOneInputRegister(INPUTREGISTER_TEMPERATURE, 1, measure_debug_area.Sensor_Temp);								//传感器温度()
	WriteOneInputRegister(INPUTREGISTER_STATEOFEINDUCTION, 1, measure_debug_area.Amplitude);						//霍尔开关状态(改为带宽)														//干簧管状态
	WriteOneInputRegister(INPUTREGISTER_FREQUENCEINAIR, 2, measure_debug_area.Water_Voltage);							//空气中频率
	WriteOneInputRegister(INPUTREGISTER_AMPLITUDE, 1, 0);										//传感器幅值
	WriteOneInputRegister(INPUTREGISTER_SENSORX_ANGLE, 1, 0);								//传感器X角度
	WriteOneInputRegister(INPUTREGISTER_SENSORY_ANGLE, 1, 0);								//传感器Y角度
	WriteOneInputRegister(INPUTREGISTER_WARTER_VOLTAGE, 1, 0);							//水位传感器电压
	WriteOneInputRegister(INPUTREGISTER_REAL_TANKHIGH_ORIGIN, 2, systemunion.systemparameter.real_tankhigh_origin);							
	WriteOneInputRegister(INPUTREGISTER_REAL_TANKHIGH_NOW, 2, systemunion.systemparameter.real_tankhigh_now);							
//WriteOneInputRegister(INPUTREGISTER_SELF_CHECK, 2, Measure_One.SelfCheck_Rank);							//V1.119 一体机自检
}


/******************************************************
函数功能： 设置线圈数组

函 数 名： ProcessWriteCoil
参    数：

返 回 值：
RETURN_OK 			0  指令正常
RETURN_SLAVEFAIL 		-1  从设备故障对应105(恢复配置文件失败)+0X04
RETURN_UNSUPPORTED 	-2无效数据地址对应103（超出地址范围/写线圈指令错误）+0X02
RETURN_UNDEFADDRESS -3无效功能码对应102  +0X01
RETURN_UNDEFDATEORDER -4无效数据或指令对应104  0X03
RETURN_SLAVEBUSY -5     从设备忙碌对应106   0X05
******************************************************/
int ProcessWriteCoil(void)
{
	unsigned int com1, com2, com3;
	com1 = STARTADDRESS1_COM;//0X000A
	com2 = STARTADDRESS2_COM - (ENDADDRESS1_COM - STARTADDRESS1_COM + 1);//0x0100-D10
	com3 = STARTADDRESS3_COM - (ENDADDRESS2_COM - com2 + 1);//0x0200-D18



	return RETURN_OK;
}
