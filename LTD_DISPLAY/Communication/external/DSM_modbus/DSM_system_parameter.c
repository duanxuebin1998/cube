#include "DSM_system_parameter.h"
#include "DSM_comm.h"
#include "usart.h"
#include "DSM_DataAnalysis_modbus2.h"
#include "DSM_SlaveModbus_modbus2.h"



char SoftOfversion6 = 'F'; // 版本号最低位









/**********************************************************************************************
**函数名称：	SystemParameterSet()
**函数功能：	设置保持寄存器读取系统参数
**参数:			无
**返回值:		int：
				0:	成功!
				-1:	失败!
**********************************************************************************************/
void SystemParameterSet(void)
{
	/*无需权限读取的*/
	WriteOneHoldingRegister(HOLDREGISTER_SP_POSITION, 1, 0);						 // 固定点监测测量位置V1.106
	WriteOneHoldingRegister(HOLDREGISTER_SPT_POSITION, 1, 0);						 // 单点测量的测量位置V1.106
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM_FREE, 1, g_deviceParams.requireBottomMeasurement);			 // 无需权限综合指令是否需要测罐底默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER_FREE, 1, g_deviceParams.requireWaterMeasurement);			 // 无需权限综合指令是否需要测水位默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE, 1, g_deviceParams.requireSinglePointDensity);	 // 无需权限综合指令是否需要测水位默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_DSM_STATE_FREE, 1, g_deviceParams.spreadMeasurementMode);							 // 无需权限分布测量时测量模式V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_NUM_FREE, 1, g_deviceParams.spreadMeasurementCount);								 // 无需权限分布测量点数	V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE_FREE, 2,g_deviceParams.spreadMeasurementDistance);						 // 无需权限分布测量点之间间距V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT_FREE, 2, g_deviceParams.spreadTopLimit);						 // 无需权限分布测量最高点距液面间距V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT_FREE, 2, g_deviceParams.spreadBottomLimit);					 // 无需权限分布测量最低点距罐底间距V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPSYNTHETIC_POSITION, 2, 0);		 // 无需权限综合指令单点测量的位置V1.106
	WriteOneHoldingRegister(HOLDREGISTER_MEASREMENT_METER, 1, g_deviceParams.spreadMeasurementOrder);						 // 密度每米测量方向
	WriteOneHoldingRegister(HOLDREGISTER_INTERVAL_POINT, 1, g_deviceParams.spreadMeasurementCount);							 // 密度液位区间测量点数
	WriteOneHoldingRegister(HOLDREGISTER_INTERVAL_DIREDION, 1, g_deviceParams.spreadMeasurementOrder);						 // 密度液位区间测量方向
	WriteOneHoldingRegister(HOLDREGISTER_INTERVAL_OIL_A, 2, 0);							 // 密度液位区间测量液位点A
	WriteOneHoldingRegister(HOLDREGISTER_INTERVAL_OIL_B, 2, 0);							 // 密度液位区间测量液位点B
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM1, 1, 0);	 // 密度分段修正温度阈值1
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM2, 1, 0);	 // 密度分段修正温度阈值2
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM3, 1, 0);	 // 密度分段修正温度阈值3
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM4, 1, 0);	 // 密度分段修正温度阈值4
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM5, 1, 0);	 // 密度分段修正温度阈值5
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM6, 1, 0);	 // 密度分段修正温度阈值6
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM7, 1, 0);	 // 密度分段修正温度阈值7
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM8, 1, 0);	 // 密度分段修正温度阈值8
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM9, 1, 0);	 // 密度分段修正温度阈值9
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM10, 1, 0);	 // 密度分段修正温度阈值10
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM11, 1, 0); // 密度分段修正温度阈值11
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_1, 1, 0);			 // 密度分段修正值1
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_2, 1, 0);			 // 密度分段修正值2
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_3, 1, 0);			 // 密度分段修正值3
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_4, 1, 0);			 // 密度分段修正值4
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_5, 1,0);			 // 密度分段修正值5
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_6, 1, 0);			 // 密度分段修正值6
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_7, 1, 0);			 // 密度分段修正值7
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_8, 1, 0);			 // 密度分段修正值8
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_9, 1, 0);			 // 密度分段修正值9
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_10, 1, 0);			 // 密度分段修正值10
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_11, 1, 0);			 // 密度分段修正值11
	WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_12, 1, 0);			 // 密度分段修正值12

	/*权限读取的*/
	WriteOneHoldingRegister(HOLDREGISTER_TANKHIGHT, 2, g_deviceParams.tankHeight);						   // 罐高
	WriteOneHoldingRegister(HOLDREGISTER_SRREAD_MEASURETURN, 1, g_deviceParams.spreadMeasurementOrder);		   // 分布测量顺序
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_STATE, 1, g_deviceParams.spreadMeasurementMode);					   // 分布测量模式
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_NUM, 1, g_deviceParams.spreadMeasurementCount);						   // 分布测量数量
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE, 2, g_deviceParams.spreadMeasurementDistance);			   // 分布测量间距
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT, 2, g_deviceParams.spreadTopLimit);			   // 分布测量上限（距液面）
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT, 2, g_deviceParams.spreadBottomLimit);		   // 分布测量下限（距罐底）
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_1POINTTHRESHOLD, 2, 0); // 分布测量一个点时的液位阈值
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_5POINTTHRESHOLD, 2, 0); // 分布测量五个点时的液位阈值
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_OTHERNUM, 1, 0);			   // 分布测量其他点数时的液位阈值

	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDDISTANCE, 2, g_deviceParams.spreadMeasurementDistance); // 固定间距分布测量的间距
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDTOP, 2, g_deviceParams.spreadTopLimit);			   // 固定间距分布测量最高点距液面间距
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDBASE, 2, g_deviceParams.spreadBottomLimit);		   // 固定间距分布测量最低点距罐底间距
	WriteOneHoldingRegister(HOLDREGISTER_THRESHOLD_A, 1,0);				   // 密度分层加测密度点A
	WriteOneHoldingRegister(HOLDREGISTER_THRESHOLD_B, 1,0);				   // 密度分层加测密度点B
	WriteOneHoldingRegister(HOLDREGISTER_THRESHOLD_STANDARD, 1,0);		   // 国标密度测量阈值
	WriteOneHoldingRegister(HOLDREGISTER_ZEROCIRCLE, 1, 0);				   // 预设零点编码圈数
	WriteOneHoldingRegister(HOLDREGISTER_ZEROANGLE, 1, 0);					   // 预设零点编码角度
	// WriteOneHoldingRegister(HOLDREGISTER_GIRTH_USELESS,1,systemunion.systemparameter.Girth);				//导论周长

	WriteOneHoldingRegister(HOLDREGISTER_WATER_IS_REAL_HIGH, 1, 0);
	WriteOneHoldingRegister(HOLDREGISTER_REAL_WATER_LEVEL_CORRECT, 1, g_deviceParams.waterLevelCorrection);
	WriteOneHoldingRegister(HOLDREGISTER_IF_REFRESH_TANKHIGH, 1, 0);
	WriteOneHoldingRegister(HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF, 1, 0);
	WriteOneHoldingRegister(HOLDREGISTER_LEVEL_MEASURE_METHOD, 1, g_deviceParams.liquidLevelMeasurementMethod);
	WriteOneHoldingRegister(HOLDREGISTER_TEMPERATURE_MODE, 1, 0);
	WriteOneHoldingRegister(HOLDREGISTER_TEMPERATURE_MODE_D, 1, 0);

	WriteOneHoldingRegister(HOLDREGISTER_REDUCTIONRATIO, 1,23);	  // 减速比
	WriteOneHoldingRegister(HOLDREGISTER_TYPEOFSENSOR, 1, 0);		  // 传感器类型
	WriteOneHoldingRegister(HOLDREGISTER_LENGTHOFSENSOR, 1, 0);	  // 传感器长度
	WriteOneHoldingRegister(HOLDREGISTER_FADEZERO, 2, g_deviceParams.blindZone);				  // 盲区
	WriteOneHoldingRegister(HOLDREGISTER_GIRTH_YITI, 2, g_deviceParams.encoder_wheel_circumference_mm);		 // 一体机导论周长
	WriteOneHoldingRegister(HOLDREGISTER_LEVELTOWATER_HIGH, 1, 0); // 液位传感器到水位传感器距离
	WriteOneHoldingRegister(HOLDREGISTER_MAXDOWN_DIS, 1, g_deviceParams.maxDownDistance);			  // 罐底/水位测量时的最大下行值

	WriteOneHoldingRegister(HOLDREGISTER_TYPEOFEINDUCTION, 1, 0);			 // 霍尔器件类型
	WriteOneHoldingRegister(HOLDREGISTER_NUMOFEINDUCTION, 1, 0);				 // 霍尔器件数量
	WriteOneHoldingRegister(HOLDREGISTER_USEFULNUMOFEINDUCTION, 1, 0);	 // 霍尔器件作用数量
	WriteOneHoldingRegister(HOLDREGISTER_DIATANCEFROMEINDUCTION, 1, 0); // 脱离配重步进数
	WriteOneHoldingRegister(HOLDREGISTER_CORRECTIONFACTOR, 1,0); // 脱离配重步进数
	WriteOneHoldingRegister(HOLDREGISTER_THRESHOLDOFFREQUENCE, 2, 0);	 // 频率阈值
	WriteOneHoldingRegister(HOLDREGISTER_RANGE_FREQUENCE, 1, 0); // 频率阈值范围
	WriteOneHoldingRegister(HOLDREGISTER_RANGE_POSITION, 1, 0);			 // 位置范围
	WriteOneHoldingRegister(HOLDREGISTER_RANGE_STABLEFREQUENCE, 1, 0);	  // 频率稳定范围
	WriteOneHoldingRegister(HOLDREGISTER_RANGE_STABLETEMPERATURE, 1, 0); // 温度稳定范围
	WriteOneHoldingRegister(HOLDREGISTER_NUMOFSTABLEHITS, 1, 0);			  // 测量时数据稳定数
	WriteOneHoldingRegister(HOLDREGISTER_NUMOFHITS, 1, 0);						  // 测量时总采样数

	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM, 1, g_deviceParams.requireBottomMeasurement);		// 综合指令是否需要测罐底默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER, 1, g_deviceParams.requireWaterMeasurement);			// 综合指令是否需要测水位默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT, 1, g_deviceParams.requireSinglePointDensity); // 综合指令是否需要测单点密度

	WriteOneHoldingRegister(HOLDREGISTER_NUMOFDECIMALS, 1, 0);		// 温度的有效点数默认是2位V1.105
	WriteOneHoldingRegister(HOLDREGISTER_TEMCORRECTCALUE, 1, g_deviceParams.temperatureCorrection);		// 温度的修正系数 温度+(修正-1000)V1.105
	WriteOneHoldingRegister(HOLDREGISTER_DENSITYCORRECTCALUE, 1, g_deviceParams.densityCorrection);		// 密度的修正系数 密度+(修正-10000)V1.105
	WriteOneHoldingRegister(HOLDREGISTER_DEVICENUM, 2, 0);						// 传感器系数V1.105
	WriteOneHoldingRegister(HOLDREGISTER_OIL_MEASUR_POSITION, 2, 0); // 综合指令发油口密度测量位置

	WriteOneHoldingRegister(HOLDREGISTER_WATER_ZERO_MIN_DISTANCE, 1, 0); // 水位与零点最小距离
	WriteOneHoldingRegister(HOLDREGISTER_IF_FINDOIL_POWERON, 1, 0);		  // 上电是否自动找液位
	WriteOneHoldingRegister(HOLDREGISTER_DENSITY_TIME, 1, 0);				  // 密度测量前提出油面时间

	WriteOneHoldingRegister(HOLDREGISTER_SOFTOFVERSION, 2,0);	 // 程序版本
	WriteOneHoldingRegister(HOLDREGISTER_SOFTOFVERSION + 2, 2,0); // 程序版本
	WriteOneHoldingRegister(HOLDREGISTER_K0, 2,0);							 // K0_H
	WriteOneHoldingRegister(HOLDREGISTER_K0 + 2, 2, 0);						 // K0_L
	WriteOneHoldingRegister(HOLDREGISTER_K1, 2,0);							 // K1_H
	WriteOneHoldingRegister(HOLDREGISTER_K1 + 2, 2, 0);						 // K1_L
	WriteOneHoldingRegister(HOLDREGISTER_K2, 2,0);							 // K2_H
	WriteOneHoldingRegister(HOLDREGISTER_K2 + 2, 2, 0);						 // K2_L
	WriteOneHoldingRegister(HOLDREGISTER_K3, 2, 0);							 // K3_H
	WriteOneHoldingRegister(HOLDREGISTER_K3 + 2, 2, 0);						 // K3_L
	WriteOneHoldingRegister(HOLDREGISTER_K18, 2, 0);						 // K18_H
	WriteOneHoldingRegister(HOLDREGISTER_K18 + 2, 2, 0);					 // K18_L
	WriteOneHoldingRegister(HOLDREGISTER_K19, 2, 0);						 // K19_H
	WriteOneHoldingRegister(HOLDREGISTER_K19 + 2, 2, 0);					 // K19_L
}

/**********************************************************************************************
**函数名称：	SystemParameterSaveMul()
**函数功能：	配置保持寄存器，修改为判断参数的地址是否在本次设置的范围内，如果在则对测量参数或者系统参数进行修改
**参数:			无
**返回值:		int：
				0:	成功!
				-1:	失败!
**********************************************************************************************/
int SystemParameterSave(int startadd, int reamount)
{
	u32 temp;

	// 配置保持寄存器，修改为判断参数的地址是否在本次设置的范围内，如果在则对测量参数或者系统参数进行修改
	/***************罐高***************************************************************************************************/
//	if ((HOLDREGISTER_TANKHIGHT >= startadd) && ((HOLDREGISTER_TANKHIGHT + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_TANKHIGHT, 2);
//
//		if (temp > (systemunion.systemparameter.FadeZero + systemunion.systemparameter.DiatanceFromEinduction)) // 罐高大于盲区+脱离零点上行距离；
//		{
//			systemunion.systemparameter.TankHigh = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_TANKHIGH, systemunion.systemparameter.TankHigh, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_TANKHIGHT, 2, systemunion.systemparameter.TankHigh);
//			return PARAMETER_ERROR;
//		}
//	}
//	/***************分布测量时测量顺序***************************************************************************************************/
//	if ((HOLDREGISTER_SRREAD_MEASURETURN >= startadd) && ((HOLDREGISTER_SRREAD_MEASURETURN) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SRREAD_MEASURETURN, 1);
//		if ((temp == 0) || (temp == 1))
//		{
//			systemunion.systemparameter.SpredMeasureTurn = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SRREAD_MEASURETURN, systemunion.systemparameter.SpredMeasureTurn, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SRREAD_MEASURETURN, 1, systemunion.systemparameter.SpredMeasureTurn);
//			return PARAMETER_ERROR;
//		}
//	}
//	/***************分布测量时测量模式***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_STATE >= startadd) && ((HOLDREGISTER_SPREAD_STATE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_STATE, 1);
//
//		if (temp >= 1 && temp <= 3)
//		{
//			systemunion.systemparameter.SpredState = temp;
//			System_measure_parameter.SpredState = systemunion.systemparameter.SpredState; // 分布测量模式V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_STATE, systemunion.systemparameter.SpredState, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_STATE, 1, systemunion.systemparameter.SpredState);
//			return PARAMETER_ERROR;
//		}
//	}
//	/***************分布测量点数***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_NUM >= startadd) && ((HOLDREGISTER_SPREAD_NUM) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_NUM, 1);
//
//		if (temp >= 1 && temp <= 100) //
//		{
//			systemunion.systemparameter.SpredNum = temp;
//			System_measure_parameter.SpredNum = systemunion.systemparameter.SpredNum; // 分布测量点数V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_NUM, systemunion.systemparameter.SpredNum, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_NUM, 1, systemunion.systemparameter.SpredNum);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************分布测量点之间间距***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_DISTANCE >= startadd) && ((HOLDREGISTER_SPREAD_DISTANCE + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE, 2);
//
//		if (temp > 0)
//		{
//			systemunion.systemparameter.SpredDistance = temp;
//			System_measure_parameter.SpredDistance = systemunion.systemparameter.SpredDistance; // 分布测量间距V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_DISTANCE, systemunion.systemparameter.SpredDistance, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE, 2, systemunion.systemparameter.SpredDistance);
//			return PARAMETER_ERROR;
//		}
//	}
//	/***************分布测量最高点距液面间距***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_TOPLIMIT >= startadd) && ((HOLDREGISTER_SPREAD_TOPLIMIT + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT, 2);
//
//		if (temp >= 1000)
//		{
//			systemunion.systemparameter.SpredTopLimit = temp;
//			System_measure_parameter.SpredTopLimit = systemunion.systemparameter.SpredTopLimit; // 分布测量上限（距液面）V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_TOPLIMIT, systemunion.systemparameter.SpredTopLimit, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT, 2, systemunion.systemparameter.SpredTopLimit);
//			return PARAMETER_ERROR;
//		}
//	}
//	/***************分布测量最低点距罐底间距***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_FLOORLIMIT >= startadd) && ((HOLDREGISTER_SPREAD_FLOORLIMIT + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero)
//		{
//			systemunion.systemparameter.SpredFloorLimit = temp;
//			System_measure_parameter.SpredFloorLimit = systemunion.systemparameter.SpredFloorLimit; // 分布测量下限（距罐底）V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_FLOORLIMIT, systemunion.systemparameter.SpredFloorLimit, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT, 2, systemunion.systemparameter.SpredFloorLimit);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************分布测量一个点的高度阈值***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_1POINTTHRESHOLD >= startadd) && ((HOLDREGISTER_SPREAD_1POINTTHRESHOLD + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_1POINTTHRESHOLD, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero)
//		{
//			systemunion.systemparameter.Spred1PointThreshold = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_1POINTTHRESHOLD, systemunion.systemparameter.Spred1PointThreshold, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_1POINTTHRESHOLD, 2, systemunion.systemparameter.Spred1PointThreshold);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************分布测量五个点的高度阈值***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_5POINTTHRESHOLD >= startadd) && ((HOLDREGISTER_SPREAD_5POINTTHRESHOLD + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_5POINTTHRESHOLD, 2);
//
//		if (temp > systemunion.systemparameter.Spred1PointThreshold)
//		{
//			systemunion.systemparameter.Spred5PointThreshold = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_5POINTTHRESHOLD, systemunion.systemparameter.Spred5PointThreshold, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_5POINTTHRESHOLD, 2, systemunion.systemparameter.Spred5PointThreshold);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************分布测量其他情况的测量点数***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_OTHERNUM >= startadd) && ((HOLDREGISTER_SPREAD_OTHERNUM) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_OTHERNUM, 1);
//
//		if (temp >= 6 && temp <= 100)
//		{
//			systemunion.systemparameter.SpredOtherNum = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_OTHERNUM, systemunion.systemparameter.SpredOtherNum, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_OTHERNUM, 1, systemunion.systemparameter.SpredOtherNum);
//			return PARAMETER_ERROR;
//		}
//	}
//	/***************固定间距分布测量的间距***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_FIXEDDISTANCE >= startadd) && ((HOLDREGISTER_SPREAD_FIXEDDISTANCE + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDDISTANCE, 2);
//
//		if (temp > 0)
//		{
//			systemunion.systemparameter.SpredFixedDistance = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_FIXEDDISTANCE, systemunion.systemparameter.SpredFixedDistance, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDDISTANCE, 2, systemunion.systemparameter.SpredFixedDistance);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************固定间距分布测量最高点距液面间距***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_FIXEDTOP >= startadd) && ((HOLDREGISTER_SPREAD_FIXEDTOP + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDTOP, 2);
//
//		if (temp > 1000)
//		{
//			systemunion.systemparameter.SpredFixTop = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_FIXEDTOP, systemunion.systemparameter.SpredFixTop, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDTOP, 2, systemunion.systemparameter.SpredFixTop);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************固定间距分布测量最低点距罐底间距***************************************************************************************************/
//	if ((HOLDREGISTER_SPREAD_FIXEDBASE >= startadd) && ((HOLDREGISTER_SPREAD_FIXEDBASE + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDBASE, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero)
//		{
//			systemunion.systemparameter.SpredFixBase = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_SPREAD_FIXEDBASE, systemunion.systemparameter.SpredFixBase, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDBASE, 2, systemunion.systemparameter.SpredFixBase);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************确定间距分层加测A点阈值**************************************************************************************************/
//	if ((HOLDREGISTER_THRESHOLD_A >= startadd) && ((HOLDREGISTER_THRESHOLD_A) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR))
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_THRESHOLD_A, 1);
//
//		if (temp > 0)
//		{
//			systemunion.systemparameter.Oil_Add_A = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_OILADD_A, systemunion.systemparameter.Oil_Add_A, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_THRESHOLD_A, 1, systemunion.systemparameter.Oil_Add_A);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************确定间距分层加测B点阈值**************************************************************************************************/
//	if ((HOLDREGISTER_THRESHOLD_B >= startadd) && ((HOLDREGISTER_THRESHOLD_B) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_THRESHOLD_B, 1);
//
//		if (temp > 0)
//		{
//			systemunion.systemparameter.Oil_Add_B = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_OILADD_B, systemunion.systemparameter.Oil_Add_B, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_THRESHOLD_B, 1, systemunion.systemparameter.Oil_Add_B);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************国标密度测量阈值**************************************************************************************************/
//	//
//	if ((HOLDREGISTER_THRESHOLD_STANDARD >= startadd) && ((HOLDREGISTER_THRESHOLD_STANDARD) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_THRESHOLD_STANDARD, 1);
//
//		if (temp > 0)
//		{
//			systemunion.systemparameter.Oil_Standard = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_OIL_STANDARD, systemunion.systemparameter.Oil_Standard, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_THRESHOLD_STANDARD, 1, systemunion.systemparameter.Oil_Standard);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************水位是否为实高测量**************************************************************************************************/
//	if ((HOLDREGISTER_WATER_IS_REAL_HIGH >= startadd) && ((HOLDREGISTER_WATER_IS_REAL_HIGH) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_WATER_IS_REAL_HIGH, 1);
//
//		if ((temp == 0) || (temp == 1))
//		{
//			systemunion.systemparameter.water_is_realhigh = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_WATER_IS_REAL_HIGH, systemunion.systemparameter.water_is_realhigh, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_WATER_IS_REAL_HIGH, 1, systemunion.systemparameter.water_is_realhigh);
//			return PARAMETER_ERROR;
//		}
//	}
//	/****************实高水位测量修正值**************************************************************************************************/
//	if ((HOLDREGISTER_REAL_WATER_LEVEL_CORRECT >= startadd) && ((HOLDREGISTER_REAL_WATER_LEVEL_CORRECT) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_REAL_WATER_LEVEL_CORRECT, 1);
//
//		if ((temp > 0) && (temp < 2000))
//		{
//			systemunion.systemparameter.real_water_level_correct = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_REAL_WATER_LEVEL_CORRECT, systemunion.systemparameter.real_water_level_correct, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_REAL_WATER_LEVEL_CORRECT, 1, systemunion.systemparameter.real_water_level_correct);
//			return PARAMETER_ERROR;
//		}
//	}
//	/****************测量是否更新罐高**************************************************************************************************/
//	if ((HOLDREGISTER_IF_REFRESH_TANKHIGH >= startadd) && ((HOLDREGISTER_IF_REFRESH_TANKHIGH) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_IF_REFRESH_TANKHIGH, 1);
//
//		if ((temp == 0) || (temp == 1))
//		{
//			systemunion.systemparameter.if_refresh_tankhigh = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_IF_REFRESH_TANKHIGH, systemunion.systemparameter.if_refresh_tankhigh, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_IF_REFRESH_TANKHIGH, 1, systemunion.systemparameter.if_refresh_tankhigh);
//			return PARAMETER_ERROR;
//		}
//	}
//	/****************实高最大偏差阈值**************************************************************************************************/
//	if ((HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF >= startadd) && ((HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF, 1);
//
//		if (temp > 0)
//		{
//			systemunion.systemparameter.real_tankhigh_max_diff = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_REAL_TANKHIGH_MAX_DIFF, systemunion.systemparameter.real_tankhigh_max_diff, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF, 1, systemunion.systemparameter.real_tankhigh_max_diff);
//			return PARAMETER_ERROR;
//		}
//	}
//	/****************液位测量方式**************************************************************************************************/
//	if ((HOLDREGISTER_LEVEL_MEASURE_METHOD >= startadd) && ((HOLDREGISTER_LEVEL_MEASURE_METHOD) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_LEVEL_MEASURE_METHOD, 1);
//
//		if ((temp == 0) || (temp == 1))
//		{
//			systemunion.systemparameter.level_masure_method = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_LEVEL_MEASURE_METHOD, systemunion.systemparameter.level_masure_method, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_LEVEL_MEASURE_METHOD, 1, systemunion.systemparameter.level_masure_method);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************综合测量单温度模式**************************************************************************************************/
//	if ((HOLDREGISTER_TEMPERATURE_MODE >= startadd) && ((HOLDREGISTER_TEMPERATURE_MODE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_TEMPERATURE_MODE, 1);
//
//		if ((temp == 0) || (temp == 1))
//		{
//			systemunion.systemparameter.synthetic_temperature_mode = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_TEMPERATURE_MODE, systemunion.systemparameter.synthetic_temperature_mode, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_TEMPERATURE_MODE, 1, systemunion.systemparameter.synthetic_temperature_mode);
//			return PARAMETER_ERROR;
//		}
//	}
//	/****************温度模式密度值**************************************************************************************************/
//	if ((HOLDREGISTER_TEMPERATURE_MODE_D >= startadd) && ((HOLDREGISTER_TEMPERATURE_MODE_D) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_TEMPERATURE_MODE_D, 1);
//
//		if (temp < 20000)
//		{
//			systemunion.systemparameter.synthetic_temperature_mode_density = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_TEMPERATURE_MODE_D, systemunion.systemparameter.synthetic_temperature_mode_density, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_TEMPERATURE_MODE_D, 1, systemunion.systemparameter.synthetic_temperature_mode_density);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************零点圈数***************************************************************************************************/
//	if ((HOLDREGISTER_ZEROCIRCLE >= startadd) && ((HOLDREGISTER_ZEROCIRCLE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_ZEROCIRCLE, 1);
//
//		if (temp <= 65535)
//		{
//			systemunion.systemparameter.ZeroCircle = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_ZEROCIRCLE, systemunion.systemparameter.ZeroCircle, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_ZEROCIRCLE, 1, systemunion.systemparameter.ZeroCircle);
//			return PARAMETER_ERROR;
//		}
//
//		// break;
//	}
//
//	/***************零点角度***************************************************************************************************/
//	if ((HOLDREGISTER_ZEROANGLE >= startadd) && ((HOLDREGISTER_ZEROANGLE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_ZEROANGLE, 1);
//
//		if (temp <= 4095)
//		{
//			systemunion.systemparameter.ZeroAngle = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_ZEROANGLE, systemunion.systemparameter.ZeroAngle, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_ZEROANGLE, 1, systemunion.systemparameter.ZeroAngle);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************导轮周长***************************************************************************************************/
//	if ((HOLDREGISTER_GIRTH_USELESS >= startadd) && ((HOLDREGISTER_GIRTH_USELESS) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_GIRTH_USELESS, 1);
//
//		if (temp >= 1 && temp <= 65535)
//		{
//			//	systemunion.systemparameter.Girth = temp;
//			//	AT24CXX_Write16Or32Bit(ADDRESS_GIRTH,systemunion.systemparameter.Girth,2);
//		}
//		else
//		{
//			// WriteOneHoldingRegister(HOLDREGISTER_GIRTH_USELESS,1,systemunion.systemparameter.Girth);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************减速比***************************************************************************************************/
//	if ((HOLDREGISTER_REDUCTIONRATIO >= startadd) && ((HOLDREGISTER_REDUCTIONRATIO) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_REDUCTIONRATIO, 1);
//
//		if (temp >= 26 && temp <= 32)
//		{
//			systemunion.systemparameter.ReductionRatio = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_REDUCTIONRATIO, systemunion.systemparameter.ReductionRatio, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_REDUCTIONRATIO, 1, systemunion.systemparameter.ReductionRatio);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************传感器类型***************************************************************************************************/
//	if ((HOLDREGISTER_TYPEOFSENSOR >= startadd) && ((HOLDREGISTER_TYPEOFSENSOR) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_TYPEOFSENSOR, 1);
//
//		if (temp >= 1 && temp <= 10)
//		{
//			systemunion.systemparameter.TypeOfSensor = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_TYPEOFSENSOR, systemunion.systemparameter.TypeOfSensor, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_TYPEOFSENSOR, 1, systemunion.systemparameter.TypeOfSensor);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************传感器长度***************************************************************************************************/
//	if ((HOLDREGISTER_LENGTHOFSENSOR >= startadd) && ((HOLDREGISTER_LENGTHOFSENSOR) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_LENGTHOFSENSOR, 1);
//
//		if (temp >= 1 && temp <= 1000)
//		{
//			systemunion.systemparameter.LengthOfSensor = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_LENGTHOFSENSOR, systemunion.systemparameter.LengthOfSensor, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_LENGTHOFSENSOR, 1, systemunion.systemparameter.LengthOfSensor);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************盲区**************************************************************************************************/
//	if ((HOLDREGISTER_FADEZERO >= startadd) && ((HOLDREGISTER_FADEZERO + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_FADEZERO, 2);
//
//		if ((temp > 0) && (temp < (systemunion.systemparameter.TankHigh - systemunion.systemparameter.DiatanceFromEinduction))) // 盲区<罐高-脱离传感器运行距离
//		{
//			systemunion.systemparameter.FadeZero = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_FADEZERO, systemunion.systemparameter.FadeZero, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_FADEZERO, 2, systemunion.systemparameter.FadeZero);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************一体机导轮周长***************************************************************************************************/
//	if ((HOLDREGISTER_GIRTH_YITI >= startadd) && ((HOLDREGISTER_GIRTH_YITI + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_GIRTH_YITI, 2);
//
//		if (temp >= 1)
//		{
//			systemunion.systemparameter.Girth = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_GIRTH, systemunion.systemparameter.Girth, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_GIRTH_YITI, 2, systemunion.systemparameter.Girth);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************水位传感器到液位传感器距离***************************************************************************************************/
//	if ((HOLDREGISTER_LEVELTOWATER_HIGH >= startadd) && ((HOLDREGISTER_LEVELTOWATER_HIGH) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_LEVELTOWATER_HIGH, 1);
//
//		if (temp >= 1)
//		{
//			systemunion.systemparameter.Dis_Leveltowater = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_LEVELTOWATER, systemunion.systemparameter.Dis_Leveltowater, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_LEVELTOWATER_HIGH, 1, systemunion.systemparameter.Dis_Leveltowater);
//			return PARAMETER_ERROR;
//		}
//
//		// break;
//	}
//
//	/***************罐底和水位测量时最大下行值***************************************************************************************/
//	if ((HOLDREGISTER_MAXDOWN_DIS >= startadd) && ((HOLDREGISTER_MAXDOWN_DIS) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_MAXDOWN_DIS, 1);
//
//		if (temp < 5000) // 设置参数要小于500mm
//		{
//			systemunion.systemparameter.Dis_Maxdown = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_MAXDOWN, systemunion.systemparameter.Dis_Maxdown, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_MAXDOWN_DIS, 1, systemunion.systemparameter.Dis_Maxdown);
//			return PARAMETER_ERROR;
//		}
//
//		// break;
//	}
//
//	/****************霍尔器件类型**************************************************************************************************/
//	if ((HOLDREGISTER_TYPEOFEINDUCTION >= startadd) && ((HOLDREGISTER_TYPEOFEINDUCTION) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_TYPEOFEINDUCTION, 1);
//
//		if (temp >= 1 && temp <= 10)
//		{
//			systemunion.systemparameter.TypeOfEinduction = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_TYPEOFEINDUCTION, systemunion.systemparameter.TypeOfEinduction, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_TYPEOFEINDUCTION, 1, systemunion.systemparameter.TypeOfEinduction);
//			return PARAMETER_ERROR;
//		}
//
//		// break;
//	}
//
//	/***************霍尔器件数量***************************************************************************************************/
//	if ((HOLDREGISTER_NUMOFEINDUCTION >= startadd) && ((HOLDREGISTER_NUMOFEINDUCTION) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_NUMOFEINDUCTION, 1);
//
//		if (temp >= 1 && temp <= 8)
//		{
//			systemunion.systemparameter.NumOfEinduction = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_NUMOFEINDUCTION, systemunion.systemparameter.NumOfEinduction, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_NUMOFEINDUCTION, 1, systemunion.systemparameter.NumOfEinduction);
//			return PARAMETER_ERROR;
//		}
//
//		//	break;
//	}
//
//	/***************霍尔器件作用数量***************************************************************************************************/
//	if ((HOLDREGISTER_USEFULNUMOFEINDUCTION >= startadd) && ((HOLDREGISTER_USEFULNUMOFEINDUCTION) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_USEFULNUMOFEINDUCTION, 1);
//
//		if (temp >= 1 && temp <= systemunion.systemparameter.NumOfEinduction)
//		{
//			systemunion.systemparameter.UsefulNumOfEinduction = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_USEFULNUMOFEINDUCTION, systemunion.systemparameter.UsefulNumOfEinduction, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_USEFULNUMOFEINDUCTION, 1, systemunion.systemparameter.UsefulNumOfEinduction);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/***************脱离配重步进数***************************************************************************************************/
//	if ((HOLDREGISTER_DIATANCEFROMEINDUCTION >= startadd) && ((HOLDREGISTER_DIATANCEFROMEINDUCTION) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_DIATANCEFROMEINDUCTION, 1);
//
//		if ((temp >= 1) && (temp <= 65535) && (temp < (systemunion.systemparameter.TankHigh - systemunion.systemparameter.FadeZero))) // 脱离传感器运行距离<罐高-盲区
//		{
//			systemunion.systemparameter.DiatanceFromEinduction = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_DIATANCEFROMEINDUCTION, systemunion.systemparameter.DiatanceFromEinduction, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_DIATANCEFROMEINDUCTION, 1, systemunion.systemparameter.DiatanceFromEinduction);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************位置修正系数步进数**************************************************************************************************/
//	if ((HOLDREGISTER_CORRECTIONFACTOR >= startadd) && ((HOLDREGISTER_CORRECTIONFACTOR) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_CORRECTIONFACTOR, 1);
//
//		if (temp >= 500 && temp <= 5000)
//		{
//			systemunion.systemparameter.CorrectionFactor = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_CORRECTIONFACTOR, systemunion.systemparameter.CorrectionFactor, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_CORRECTIONFACTOR, 1, systemunion.systemparameter.CorrectionFactor);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************频率阈值**************************************************************************************************/
//	if ((HOLDREGISTER_THRESHOLDOFFREQUENCE >= startadd) && ((HOLDREGISTER_THRESHOLDOFFREQUENCE + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_THRESHOLDOFFREQUENCE, 2);
//
//		if (temp >= 350000 && temp <= 650000)
//		{
//			systemunion.systemparameter.FrequenceThreshold = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_FREQUENCETHRESHOLD, systemunion.systemparameter.FrequenceThreshold, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_THRESHOLDOFFREQUENCE, 2, systemunion.systemparameter.FrequenceThreshold);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************精找液位频率阈值范围**************************************************************************************************/
//	if ((HOLDREGISTER_RANGE_FREQUENCE >= startadd) && ((HOLDREGISTER_RANGE_FREQUENCE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_RANGE_FREQUENCE, 1);
//
//		if (temp >= 1 && temp <= 50000)
//		{
//			systemunion.systemparameter.RangeOfFrequenceThreshold = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_RANGEOFFREQUENCETHRESHOLD, systemunion.systemparameter.RangeOfFrequenceThreshold, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_RANGE_FREQUENCE, 1, systemunion.systemparameter.RangeOfFrequenceThreshold);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************位置阈值范围**************************************************************************************************/
//	if ((HOLDREGISTER_RANGE_POSITION >= startadd) && ((HOLDREGISTER_RANGE_POSITION) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_RANGE_POSITION, 1);
//
//		if (temp >= 1 && temp <= 1000)
//		{
//			systemunion.systemparameter.RangeOfPosition = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_RANGEOFPOSITION, systemunion.systemparameter.RangeOfPosition, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_RANGE_POSITION, 1, systemunion.systemparameter.RangeOfPosition);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/****************测量时频率稳定范围**************************************************************************************************/
//	if ((HOLDREGISTER_RANGE_STABLEFREQUENCE >= startadd) && ((HOLDREGISTER_RANGE_STABLEFREQUENCE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_RANGE_STABLEFREQUENCE, 1);
//
//		if (temp >= 1 && temp <= 10000)
//		{
//			systemunion.systemparameter.RangeOfFrequence = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_RANGEOFFREQUENCE, systemunion.systemparameter.RangeOfFrequence, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_RANGE_STABLEFREQUENCE, 1, systemunion.systemparameter.RangeOfFrequence);
//			return PARAMETER_ERROR;
//		}
//
//		// break;
//	}
//
//	/****************测量时温度稳定范围**************************************************************************************************/
//	if ((HOLDREGISTER_RANGE_STABLETEMPERATURE >= startadd) && ((HOLDREGISTER_RANGE_STABLETEMPERATURE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_RANGE_STABLETEMPERATURE, 1);
//
//		if (temp >= 1 && temp <= 100)
//		{
//			systemunion.systemparameter.RangeOfTemperature = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_RANGEOFTEMPERATURE, systemunion.systemparameter.RangeOfTemperature, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_RANGE_STABLETEMPERATURE, 1, systemunion.systemparameter.RangeOfTemperature);
//			return PARAMETER_ERROR;
//		}
//
//		// break;
//	}
//
//	/*****************测量时数据稳定数*************************************************************************************************/
//	if ((HOLDREGISTER_NUMOFSTABLEHITS >= startadd) && ((HOLDREGISTER_NUMOFSTABLEHITS) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_NUMOFSTABLEHITS, 1);
//
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		if (temp >= 1)
//		{
//			systemunion.systemparameter.NumOfStableHits = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_NUMOFSTABLEHITS, systemunion.systemparameter.NumOfStableHits, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_NUMOFSTABLEHITS, 1, systemunion.systemparameter.NumOfStableHits);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************测量时总采样数*************************************************************************************************/
//	if ((HOLDREGISTER_NUMOFHITS >= startadd) && ((HOLDREGISTER_NUMOFHITS) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_NUMOFHITS, 1);
//
//		if (temp >= 1 && temp <= 100)
//		{
//			systemunion.systemparameter.NumOfHits = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_NUMOFHITS, systemunion.systemparameter.NumOfHits, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_NUMOFHITS, 1, systemunion.systemparameter.NumOfHits);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************综合指令是否需要测量罐底V1.105**********************************************************************/
//	if ((HOLDREGISTER_SYNTHETIC_BOTTOM >= startadd) && ((HOLDREGISTER_SYNTHETIC_BOTTOM) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM, 1);
//
//		if (temp == 0 || temp == 1)
//		{
//			systemunion.systemparameter.Synthetic_Need_FindBottom = temp;
//			System_measure_parameter.Synthetic_Need_FindBottom = systemunion.systemparameter.Synthetic_Need_FindBottom; // 综合指令是否需要测量罐底V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SYNTHETIC_BOTTOM, systemunion.systemparameter.Synthetic_Need_FindBottom, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM, 1, systemunion.systemparameter.Synthetic_Need_FindBottom);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************综合指令是否需要测量水位V1.105**********************************************************************/
//	if ((HOLDREGISTER_SYNTHETIC_WATER >= startadd) && ((HOLDREGISTER_SYNTHETIC_WATER) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER, 1);
//
//		if (temp == 0 || temp == 1)
//		{
//			systemunion.systemparameter.Synthetic_Need_FindWater = temp;
//			System_measure_parameter.Synthetic_Need_FindWater = systemunion.systemparameter.Synthetic_Need_FindWater; // 综合指令是否需要测量水位V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SYNTHETIC_WATER, systemunion.systemparameter.Synthetic_Need_FindWater, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER, 1, systemunion.systemparameter.Synthetic_Need_FindWater);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************综合指令是否需要测量单点密度V1.105**********************************************************************/
//	if ((HOLDREGISTER_SYNTHETIC_SINGLEPOINT >= startadd) && ((HOLDREGISTER_SYNTHETIC_SINGLEPOINT) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT, 1);
//
//		if (temp == 0 || temp == 1)
//		{
//			systemunion.systemparameter.Synthetic_Need_SinglePoint = temp;
//			System_measure_parameter.Synthetic_Need_SinglePoint = systemunion.systemparameter.Synthetic_Need_SinglePoint; // 综合指令是否需要测量单点密度V1.106
//			AT24CXX_Write16Or32Bit(ADDRESS_SYNTHETIC_SINGLEPOINT, systemunion.systemparameter.Synthetic_Need_SinglePoint, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT, 1, systemunion.systemparameter.Synthetic_Need_SinglePoint);
//			return PARAMETER_ERROR;
//		}
//	}
//	/*****************水位与零点最小距离**********************************************************************/
//	if ((HOLDREGISTER_WATER_ZERO_MIN_DISTANCE >= startadd) && ((HOLDREGISTER_WATER_ZERO_MIN_DISTANCE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // 待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_WATER_ZERO_MIN_DISTANCE, 1);
//
//		if (temp <= 65535)
//		{
//			systemunion.systemparameter.Water_zero_min_disance = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_WATER_ZERO_MIN_DISTANCE, systemunion.systemparameter.Water_zero_min_disance, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_WATER_ZERO_MIN_DISTANCE, 1, systemunion.systemparameter.Water_zero_min_disance);
//			return PARAMETER_ERROR;
//		}
//	}
//	/*****************上电是否自动找液位**********************************************************************/
//	if ((HOLDREGISTER_IF_FINDOIL_POWERON >= startadd) && ((HOLDREGISTER_IF_FINDOIL_POWERON) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // 待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_IF_FINDOIL_POWERON, 1);
//
//		if (temp <= 1)
//		{
//			systemunion.systemparameter.If_findoil_poweron = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_IF_FINDOIL_POWERON, systemunion.systemparameter.If_findoil_poweron, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_IF_FINDOIL_POWERON, 1, systemunion.systemparameter.If_findoil_poweron);
//			return PARAMETER_ERROR;
//		}
//	}
//	/*****************上电是否自动找液位**********************************************************************/
//	if ((HOLDREGISTER_DENSITY_TIME >= startadd) && ((HOLDREGISTER_DENSITY_TIME) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // 待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_DENSITY_TIME, 1);
//
//		if (temp <= 60)
//		{
//			systemunion.systemparameter.Density_air_time = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_DENSITY_TIME, systemunion.systemparameter.Density_air_time, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_DENSITY_TIME, 1, systemunion.systemparameter.Density_air_time);
//			return PARAMETER_ERROR;
//		}
//	}
//	/*****************温度显示的有效位数V1.105**********************************************************************/
//	if ((HOLDREGISTER_NUMOFDECIMALS >= startadd) && ((HOLDREGISTER_NUMOFDECIMALS) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//		temp = ReadOneHoldingRegister(HOLDREGISTER_NUMOFDECIMALS, 1);
//
//		if (temp == 1 || temp == 2)
//		{
//			systemunion.systemparameter.Temperature_Decimals = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_NUMOFDECIMALS, systemunion.systemparameter.Temperature_Decimals, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_NUMOFDECIMALS, 1, systemunion.systemparameter.Temperature_Decimals);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************温度修正值V1.105**********************************************************************/
//	if ((HOLDREGISTER_TEMCORRECTCALUE >= startadd) && ((HOLDREGISTER_TEMCORRECTCALUE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_TEMCORRECTCALUE, 1);
//
//		if (temp > 0 && temp <= 2000)
//		{
//			systemunion.systemparameter.Temperature_Correct = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_TEMCORRECTCALUE, systemunion.systemparameter.Temperature_Correct, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_TEMCORRECTCALUE, 1, systemunion.systemparameter.Temperature_Correct);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************密度修正值V1.105**********************************************************************/
//	if ((HOLDREGISTER_DENSITYCORRECTCALUE >= startadd) && ((HOLDREGISTER_DENSITYCORRECTCALUE) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_DENSITYCORRECTCALUE, 1);
//
//		if (temp > 0 && temp <= 20000)
//		{
//			systemunion.systemparameter.Density_Correct = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_DENSITYCORRECTCALUE, systemunion.systemparameter.Density_Correct, 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_DENSITYCORRECTCALUE, 1, systemunion.systemparameter.Density_Correct);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************传感器系数V1.105**********************************************************************/
//	if ((HOLDREGISTER_DEVICENUM >= startadd) && ((HOLDREGISTER_DEVICENUM + 1) <= (startadd + reamount - 1)))
//	{
//		return PARAMETER_ERROR; // 传感器系数不允许设置
//	}
//
//	/*****************综合指令发油口密度测量位置**********************************************************************/
//	if ((HOLDREGISTER_OIL_MEASUR_POSITION >= startadd) && ((HOLDREGISTER_OIL_MEASUR_POSITION + 1) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		temp = ReadOneHoldingRegister(HOLDREGISTER_OIL_MEASUR_POSITION, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero && temp <= systemunion.systemparameter.TankHigh)
//		{
//			systemunion.systemparameter.Oil_Measure_Position = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_OIL_MEASUR_POSITION, systemunion.systemparameter.Oil_Measure_Position, 4);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_OIL_MEASUR_POSITION, 2, systemunion.systemparameter.Oil_Measure_Position);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	/*****************K0*************************************************************************************************/
//	if ((HOLDREGISTER_K0 >= startadd) && ((HOLDREGISTER_K0 + 3) <= (startadd + reamount - 1)))
//	{
//
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.K0_H = ReadOneHoldingRegister(HOLDREGISTER_K0, 2);
//		systemunion.systemparameter.K0_L = ReadOneHoldingRegister(HOLDREGISTER_K0 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_K0, systemunion.systemparameter.K0_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_K0 + 4, systemunion.systemparameter.K0_L, 4);
//	}
//
//	///****************K1**************************************************************************************************/
//	if ((HOLDREGISTER_K1 >= startadd) && ((HOLDREGISTER_K1 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.K1_H = ReadOneHoldingRegister(HOLDREGISTER_K1, 2);
//		systemunion.systemparameter.K1_L = ReadOneHoldingRegister(HOLDREGISTER_K1 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_K1, systemunion.systemparameter.K1_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_K1 + 4, systemunion.systemparameter.K1_L, 4);
//	}
//
//	//
//	///****************K2**************************************************************************************************/
//	if ((HOLDREGISTER_K2 >= startadd) && ((HOLDREGISTER_K2 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.K2_H = ReadOneHoldingRegister(HOLDREGISTER_K2, 2);
//		systemunion.systemparameter.K2_L = ReadOneHoldingRegister(HOLDREGISTER_K2 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_K2, systemunion.systemparameter.K2_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_K2 + 4, systemunion.systemparameter.K2_L, 4);
//	}
//	///****************K3**************************************************************************************************/
//	if ((HOLDREGISTER_K3 >= startadd) && ((HOLDREGISTER_K3 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.K3_H = ReadOneHoldingRegister(HOLDREGISTER_K3, 2);
//		systemunion.systemparameter.K3_L = ReadOneHoldingRegister(HOLDREGISTER_K3 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_K3, systemunion.systemparameter.K3_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_K3 + 4, systemunion.systemparameter.K3_L, 4);
//	}
//	///****************K18**************************************************************************************************/
//	if ((HOLDREGISTER_K18 >= startadd) && ((HOLDREGISTER_K18 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.K18_H = ReadOneHoldingRegister(HOLDREGISTER_K18, 2);
//		systemunion.systemparameter.K18_L = ReadOneHoldingRegister(HOLDREGISTER_K18 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_K18, systemunion.systemparameter.K18_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_K18 + 4, systemunion.systemparameter.K18_L, 4);
//	}
//	///****************K19**************************************************************************************************/
//	if ((HOLDREGISTER_K19 >= startadd) && ((HOLDREGISTER_K19 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.K19_H = ReadOneHoldingRegister(HOLDREGISTER_K19, 2);
//		systemunion.systemparameter.K19_L = ReadOneHoldingRegister(HOLDREGISTER_K19 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_K19, systemunion.systemparameter.K19_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_K19 + 4, systemunion.systemparameter.K19_L, 4);
//	}
//
//	///****************V1**************************************************************************************************/
//	if ((HOLDREGISTER_V1 >= startadd) && ((HOLDREGISTER_V1 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.v1_H = ReadOneHoldingRegister(HOLDREGISTER_V1, 2);
//		systemunion.systemparameter.v1_L = ReadOneHoldingRegister(HOLDREGISTER_V1 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_V1, systemunion.systemparameter.v1_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_V1 + 4, systemunion.systemparameter.v1_L, 4);
//	}
//
//	///****************V2**************************************************************************************************/
//	if ((HOLDREGISTER_V2 >= startadd) && ((HOLDREGISTER_V2 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.v2_H = ReadOneHoldingRegister(HOLDREGISTER_V2, 2);
//		systemunion.systemparameter.v2_L = ReadOneHoldingRegister(HOLDREGISTER_V2 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_V2, systemunion.systemparameter.v2_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_V2 + 4, systemunion.systemparameter.v2_L, 4);
//	}
//
//	///****************V3**************************************************************************************************/
//	if ((HOLDREGISTER_V3 >= startadd) && ((HOLDREGISTER_V3 + 3) <= (startadd + reamount - 1)))
//	{
//		if ((Measure_One.EquipmentState != DSM_STATE_STANDBY) && (Measure_One.EquipmentState != DSM_STATE_ERROR)) // V1.106待机和故障时允许写
//		{
//			return PARAMETER_WRITE_FAIL;
//		}
//
//		systemunion.systemparameter.v3_H = ReadOneHoldingRegister(HOLDREGISTER_V3, 2);
//		systemunion.systemparameter.v3_L = ReadOneHoldingRegister(HOLDREGISTER_V3 + 2, 2);
//		AT24CXX_Write16Or32Bit(ADDRESS_V3, systemunion.systemparameter.v3_H, 4);
//		AT24CXX_Write16Or32Bit(ADDRESS_V3 + 4, systemunion.systemparameter.v3_L, 4);
//	}
//
//	/************不写入EEPROM的,单点监测的位置******************/
//	if ((HOLDREGISTER_SPT_POSITION >= startadd) && ((HOLDREGISTER_SPT_POSITION + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.Density_Single_St_High = ReadOneHoldingRegister(HOLDREGISTER_SPT_POSITION, 2);
//	}
//
//	/**不写入EEROM,运行记录**/
//	if ((HOLDREGISTER_RUNTODISTANCE >= startadd) && ((HOLDREGISTER_RUNTODISTANCE + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.UpOrDown_Need_Run = ReadOneHoldingRegister(HOLDREGISTER_RUNTODISTANCE, 2);
//	}
//
//	if ((HOLDREGISTER_CORRECTION_OIL >= startadd) && ((HOLDREGISTER_CORRECTION_OIL + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.Calibration_Oil_Level = ReadOneHoldingRegister(HOLDREGISTER_CORRECTION_OIL, 2);
//	}
//
//	if ((HOLDREGISTER_SP_POSITION >= startadd) && ((HOLDREGISTER_SP_POSITION + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.Density_Single_Mm_High = ReadOneHoldingRegister(HOLDREGISTER_SP_POSITION, 2);
//	}
//
//	if ((HOLDREGISTER_CALIBRATIONLIQUIDLEVEL >= startadd) && ((HOLDREGISTER_CALIBRATIONLIQUIDLEVEL + 1) <= (startadd + reamount - 1)))
//	{
//		LevelCorrection = ReadOneHoldingRegister(HOLDREGISTER_CALIBRATIONLIQUIDLEVEL, 2);
//	}
//
//	// case HOLDREGISTER_SYNTHETIC_BOTTOM_FREE://无需权限综合指令是否需要测罐底默认为0不测V1.105
//	if ((HOLDREGISTER_SYNTHETIC_BOTTOM_FREE >= startadd) && ((HOLDREGISTER_SYNTHETIC_BOTTOM_FREE) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.Synthetic_Need_FindBottom = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM_FREE, 1);
//	}
//
//	// case HOLDREGISTER_SYNTHETIC_WATER_FREE://无需权限综合指令是否需要测水位默认为0不测V1.105
//	if ((HOLDREGISTER_SYNTHETIC_WATER_FREE >= startadd) && ((HOLDREGISTER_SYNTHETIC_WATER_FREE) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.Synthetic_Need_FindWater = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER_FREE, 1);
//	}
//
//	// case HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE://无需权限综合指令是否需要测水位默认为0不测V1.105
//	if ((HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE >= startadd) && ((HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.Synthetic_Need_SinglePoint = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE, 1);
//	}
//
//	// case HOLDREGISTER_SPREAD_DSM_STATE_FREE://无需权限分布测量时测量模式V1.105
//	if ((HOLDREGISTER_SPREAD_DSM_STATE_FREE >= startadd) && ((HOLDREGISTER_SPREAD_DSM_STATE_FREE) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.SpredState = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_DSM_STATE_FREE, 1);
//	}
//
//	// case HOLDREGISTER_SPREAD_NUM_FREE://无需权限分布测量点数	V1.105
//	if ((HOLDREGISTER_SPREAD_NUM_FREE >= startadd) && ((HOLDREGISTER_SPREAD_NUM_FREE) <= (startadd + reamount - 1))) //
//	{
//		System_measure_parameter.SpredNum = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_NUM_FREE, 1);
//	}
//
//	// case HOLDREGISTER_SPREAD_DISTANCE_FREE://无需权限分布测量点之间间距V1.105
//	if ((HOLDREGISTER_SPREAD_DISTANCE_FREE >= startadd) && ((HOLDREGISTER_SPREAD_DISTANCE_FREE + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.SpredDistance = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE_FREE, 2);
//	}
//
//	// case HOLDREGISTER_SPREAD_TOPLIMIT_FREE://无需权限分布测量最高点距液面间距V1.105
//	if ((HOLDREGISTER_SPREAD_TOPLIMIT_FREE >= startadd) && ((HOLDREGISTER_SPREAD_TOPLIMIT_FREE + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.SpredTopLimit = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT_FREE, 2);
//	}
//
//	// case HOLDREGISTER_SPREAD_FLOORLIMIT_FREE://无需权限分布测量最低点距罐底间距V1.105
//	if ((HOLDREGISTER_SPREAD_FLOORLIMIT_FREE >= startadd) && ((HOLDREGISTER_SPREAD_FLOORLIMIT_FREE + 1) <= (startadd + reamount - 1)))
//	{
//		System_measure_parameter.SpredFloorLimit = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT_FREE, 2);
//	}
//
//	// case HOLDREGISTER_SPSYNTHETIC_POSITION://无需权限分布测量最低点距罐底间距V1.105
//	if ((HOLDREGISTER_SPSYNTHETIC_POSITION >= startadd) && ((HOLDREGISTER_SPSYNTHETIC_POSITION + 1) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_SPSYNTHETIC_POSITION, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero && temp <= systemunion.systemparameter.TankHigh)
//		{
//			System_measure_parameter.Density_Single_Synthetic_High = temp;
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_SPSYNTHETIC_POSITION, 2, System_measure_parameter.Density_Single_Synthetic_High);
//			return PARAMETER_ERROR;
//		}
//	}
//
//	// case HOLDREGISTER_MEASREMENT_METER://无需分配权限密度每米测量方向
//	if ((HOLDREGISTER_MEASREMENT_METER >= startadd) && ((HOLDREGISTER_MEASREMENT_METER) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_MEASREMENT_METER, 1);
//
//		if (temp == 0 || temp == 1)
//		{
//			System_measure_parameter.Measrement_Meter = temp;
//		}
//		else
//		{
//			return PARAMETER_ERROR;
//		}
//	}
//
//	// case HOLDREGISTER_INTERVAL_POINT://密度液位区间测量点数
//	if ((HOLDREGISTER_INTERVAL_POINT >= startadd) && ((HOLDREGISTER_INTERVAL_POINT) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_INTERVAL_POINT, 1);
//
//		if (temp > 0 && temp <= 100)
//		{
//			System_measure_parameter.Interval_Point = temp;
//		}
//		else
//		{
//			return PARAMETER_ERROR;
//		}
//	}
//
//	// case HOLDREGISTER_INTERVAL_DIREDION://密度液位区间测量方向
//	if ((HOLDREGISTER_INTERVAL_DIREDION >= startadd) && ((HOLDREGISTER_INTERVAL_DIREDION) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_INTERVAL_DIREDION, 1);
//
//		if (temp == 0 || temp == 1)
//		{
//			System_measure_parameter.Interval_Diredion = temp;
//		}
//		else
//		{
//			return PARAMETER_ERROR;
//		}
//	}
//
//	// case HOLDREGISTER_INTERVAL_OIL_A://密度液位区间测量液位值A
//	if ((HOLDREGISTER_INTERVAL_OIL_A >= startadd) && ((HOLDREGISTER_INTERVAL_OIL_A + 1) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_INTERVAL_OIL_A, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero && temp < systemunion.systemparameter.TankHigh)
//		{
//			System_measure_parameter.Interval_Oil_A = temp;
//		}
//		else
//		{
//			return PARAMETER_ERROR;
//		}
//	}
//
//	// case HOLDREGISTER_INTERVAL_OIL_B://密度液位区间测量液位值B
//	if ((HOLDREGISTER_INTERVAL_OIL_B >= startadd) && ((HOLDREGISTER_INTERVAL_OIL_B + 1) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_INTERVAL_OIL_B, 2);
//
//		if (temp > systemunion.systemparameter.FadeZero && temp < systemunion.systemparameter.TankHigh)
//		{
//			System_measure_parameter.Interval_Oil_B = temp;
//		}
//		else
//		{
//			return PARAMETER_ERROR;
//		}
//	}
//	// 密度分段修正温度阈值
//	if ((HOLDREGISTER_D_CORRECTION_TEM1 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM1) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM1, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[0] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM1, systemunion.systemparameter.Density_Correction_Temperature[0], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM1, 1, systemunion.systemparameter.Density_Correction_Temperature[0]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM2 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM2) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM2, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[1] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM2, systemunion.systemparameter.Density_Correction_Temperature[1], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM2, 1, systemunion.systemparameter.Density_Correction_Temperature[1]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM3 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM3) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM3, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[2] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM3, systemunion.systemparameter.Density_Correction_Temperature[2], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM3, 1, systemunion.systemparameter.Density_Correction_Temperature[2]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM4 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM4) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM4, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[3] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM4, systemunion.systemparameter.Density_Correction_Temperature[3], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM4, 1, systemunion.systemparameter.Density_Correction_Temperature[3]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM5 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM5) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM5, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[4] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM5, systemunion.systemparameter.Density_Correction_Temperature[4], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM5, 1, systemunion.systemparameter.Density_Correction_Temperature[4]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM6 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM6) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM6, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[5] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM6, systemunion.systemparameter.Density_Correction_Temperature[5], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM6, 1, systemunion.systemparameter.Density_Correction_Temperature[5]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM7 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM7) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM7, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[6] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM7, systemunion.systemparameter.Density_Correction_Temperature[6], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM7, 1, systemunion.systemparameter.Density_Correction_Temperature[6]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM8 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM8) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM8, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[7] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM8, systemunion.systemparameter.Density_Correction_Temperature[7], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM8, 1, systemunion.systemparameter.Density_Correction_Temperature[7]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM9 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM9) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM9, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[8] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM9, systemunion.systemparameter.Density_Correction_Temperature[8], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM9, 1, systemunion.systemparameter.Density_Correction_Temperature[8]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM10 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM10) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM10, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[9] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM10, systemunion.systemparameter.Density_Correction_Temperature[9], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM10, 1, systemunion.systemparameter.Density_Correction_Temperature[9]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_TEM11 >= startadd) && ((HOLDREGISTER_D_CORRECTION_TEM11) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM11, 1);
//		if (temp <= 400)
//		{
//			systemunion.systemparameter.Density_Correction_Temperature[10] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_TEM11, systemunion.systemparameter.Density_Correction_Temperature[10], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_TEM11, 1, systemunion.systemparameter.Density_Correction_Temperature[10]);
//			return PARAMETER_ERROR;
//		}
//	}
//	// 密度分段修正值
//	if ((HOLDREGISTER_D_CORRECTION_1 >= startadd) && ((HOLDREGISTER_D_CORRECTION_1) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_1, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[0] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_1, systemunion.systemparameter.Density_Correction_Value[0], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_1, 1, systemunion.systemparameter.Density_Correction_Value[0]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_2 >= startadd) && ((HOLDREGISTER_D_CORRECTION_2) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_2, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[1] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_2, systemunion.systemparameter.Density_Correction_Value[1], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_2, 1, systemunion.systemparameter.Density_Correction_Value[1]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_3 >= startadd) && ((HOLDREGISTER_D_CORRECTION_3) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_3, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[2] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_3, systemunion.systemparameter.Density_Correction_Value[2], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_3, 1, systemunion.systemparameter.Density_Correction_Value[2]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_4 >= startadd) && ((HOLDREGISTER_D_CORRECTION_4) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_4, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[3] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_4, systemunion.systemparameter.Density_Correction_Value[3], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_4, 1, systemunion.systemparameter.Density_Correction_Value[3]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_5 >= startadd) && ((HOLDREGISTER_D_CORRECTION_5) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_5, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[4] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_5, systemunion.systemparameter.Density_Correction_Value[4], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_5, 1, systemunion.systemparameter.Density_Correction_Value[4]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_6 >= startadd) && ((HOLDREGISTER_D_CORRECTION_6) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_6, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[5] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_6, systemunion.systemparameter.Density_Correction_Value[5], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_6, 1, systemunion.systemparameter.Density_Correction_Value[5]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_7 >= startadd) && ((HOLDREGISTER_D_CORRECTION_7) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_7, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[6] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_7, systemunion.systemparameter.Density_Correction_Value[6], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_7, 1, systemunion.systemparameter.Density_Correction_Value[6]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_8 >= startadd) && ((HOLDREGISTER_D_CORRECTION_8) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_8, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[7] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_8, systemunion.systemparameter.Density_Correction_Value[7], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_8, 1, systemunion.systemparameter.Density_Correction_Value[7]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_9 >= startadd) && ((HOLDREGISTER_D_CORRECTION_9) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_9, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[8] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_9, systemunion.systemparameter.Density_Correction_Value[8], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_9, 1, systemunion.systemparameter.Density_Correction_Value[8]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_10 >= startadd) && ((HOLDREGISTER_D_CORRECTION_10) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_10, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[9] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_10, systemunion.systemparameter.Density_Correction_Value[9], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_10, 1, systemunion.systemparameter.Density_Correction_Value[9]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_11 >= startadd) && ((HOLDREGISTER_D_CORRECTION_11) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_11, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[10] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_11, systemunion.systemparameter.Density_Correction_Value[10], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_11, 1, systemunion.systemparameter.Density_Correction_Value[10]);
//			return PARAMETER_ERROR;
//		}
//	}
//	if ((HOLDREGISTER_D_CORRECTION_12 >= startadd) && ((HOLDREGISTER_D_CORRECTION_12) <= (startadd + reamount - 1)))
//	{
//		temp = ReadOneHoldingRegister(HOLDREGISTER_D_CORRECTION_12, 1);
//		if (temp <= 2000)
//		{
//			systemunion.systemparameter.Density_Correction_Value[11] = temp;
//			AT24CXX_Write16Or32Bit(ADDRESS_D_CORRECTION_12, systemunion.systemparameter.Density_Correction_Value[11], 2);
//		}
//		else
//		{
//			WriteOneHoldingRegister(HOLDREGISTER_D_CORRECTION_12, 1, systemunion.systemparameter.Density_Correction_Value[11]);
//			return PARAMETER_ERROR;
//		}
//	}
	return NO_ERROR;
}

