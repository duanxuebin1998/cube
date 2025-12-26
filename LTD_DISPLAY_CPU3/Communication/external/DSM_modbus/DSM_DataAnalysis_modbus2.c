#include "DSM_DataAnalysis_modbus2.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "DSM_comm.h"
#include "spi.h"
#include "DSM_communication.h"
#include "DSM_stateformodbus2.h"
#include "system_parameter.h"
#include "stateformodbus.h"
#include "address.h"
#include "usart.h"
#include "DSM_SlaveModbus_modbus2.h"
#include "device_param_sync.h"


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
	WriteOneHoldingRegister(HOLDREGISTER_SP_POSITION, 1,g_deviceParams.);						 // 固定点监测测量位置
	WriteOneHoldingRegister(HOLDREGISTER_SPT_POSITION, 1, 0);						 // 单点测量的测量位置
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM_FREE, 1, g_deviceParams.requireBottomMeasurement);			 // 无需权限综合指令是否需要测罐底默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER_FREE, 1, g_deviceParams.requireWaterMeasurement);			 // 无需权限综合指令是否需要测水位默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE, 1, g_deviceParams.requireSinglePointDensity);	 // 无需权限综合指令是否需要测水位默认为0不测V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_DSM_STATE_FREE, 1, g_deviceParams.spreadMeasurementMode);							 // 无需权限分布测量时测量模式V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_NUM_FREE, 1, g_deviceParams.spreadMeasurementCount);								 // 无需权限分布测量点数	V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE_FREE, 2,g_deviceParams.spreadMeasurementDistance);						 // 无需权限分布测量点之间间距V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT_FREE, 2, g_deviceParams.spreadTopLimit);						 // 无需权限分布测量最高点距液面间距V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT_FREE, 2, g_deviceParams.spreadBottomLimit);					 // 无需权限分布测量最低点距罐底间距V1.105
	WriteOneHoldingRegister(HOLDREGISTER_SPSYNTHETIC_POSITION, 2, 0);		 // 无需权限综合指令单点测量的位置
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

	WriteOneHoldingRegister(HOLDREGISTER_REDUCTIONRATIO, 1,30);	  // 减速比
	WriteOneHoldingRegister(HOLDREGISTER_TYPEOFSENSOR, 1, 0);		  // 传感器类型
	WriteOneHoldingRegister(HOLDREGISTER_LENGTHOFSENSOR, 1, 0);	  // 传感器长度
	WriteOneHoldingRegister(HOLDREGISTER_FADEZERO, 2, g_deviceParams.blindZone);				  // 盲区
	WriteOneHoldingRegister(HOLDREGISTER_GIRTH_YITI, 2, g_deviceParams.encoder_wheel_circumference_mm);		 // 一体机导论周长
	WriteOneHoldingRegister(HOLDREGISTER_LEVELTOWATER_HIGH, 1, 0); // 液位传感器到水位传感器距离/水位修正值
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

	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM, 1, g_deviceParams.requireBottomMeasurement);		// 综合指令是否需要测罐底默认为0不测
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER, 1, g_deviceParams.requireWaterMeasurement);			// 综合指令是否需要测水位默认为0不测
	WriteOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT, 1, g_deviceParams.requireSinglePointDensity); // 综合指令是否需要测单点密度

	WriteOneHoldingRegister(HOLDREGISTER_NUMOFDECIMALS, 1, 1);		// 温度的有效点数默认是2位
	WriteOneHoldingRegister(HOLDREGISTER_TEMCORRECTCALUE, 1, g_deviceParams.temperatureCorrection);		// 温度的修正系数 温度+(修正-1000)
	WriteOneHoldingRegister(HOLDREGISTER_DENSITYCORRECTCALUE, 1, g_deviceParams.densityCorrection);		// 密度的修正系数 密度+(修正-10000)
	WriteOneHoldingRegister(HOLDREGISTER_DEVICENUM, 2, 0);						// 传感器系数
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

/**
 * @brief  上位机写保持寄存器后的处理：
 *         1) 按当前 DSM_HoldingRegisterArray 刷新 g_deviceParams
 *         2) 把新的 g_deviceParams 与 param_meta / CPU2 对比
 *         3) 通过 CPU2_CombinatePackage_Send 下发差异参数
 *
 * @param  startadd  本次写入的起始地址
 * @param  reamount  本次写入的寄存器数量
 * @retval 0              成功
 *         PARAMETER_ERROR 数据越界/非法
 *         PARAMETER_WRITE_FAIL 设备忙等（你需要时可保留）
 */

int UpdateDeviceParamsFromLegacyRegs(int startadd, int reamount)
{
    u32 temp;
    int end = startadd + reamount - 1;
    /*************** 固定点测量位置 -> g_deviceParams.tankHeight ****************/
    if ((HOLDREGISTER_TANKHIGHT >= startadd) &&
        ((HOLDREGISTER_TANKHIGHT + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_TANKHIGHT, 2);
        g_deviceParams.tankHeight = temp;
    }
    /*************** 罐高 -> g_deviceParams.tankHeight ****************/
    if ((HOLDREGISTER_TANKHIGHT >= startadd) &&
        ((HOLDREGISTER_TANKHIGHT + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_TANKHIGHT, 2);
        g_deviceParams.tankHeight = temp;
    }
    /*************** 标定液位/修正液位 -> g_deviceParams.calibrateOilLevel ****************/
    if ((HOLDREGISTER_CALIBRATIONLIQUIDLEVEL >= startadd) &&
        ((HOLDREGISTER_CALIBRATIONLIQUIDLEVEL + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_CALIBRATIONLIQUIDLEVEL, 2);
        g_deviceParams.calibrateOilLevel = temp;
    }
    if ((HOLDREGISTER_CORRECTION_OIL >= startadd) &&
        ((HOLDREGISTER_CORRECTION_OIL + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_CORRECTION_OIL, 2);
        g_deviceParams.calibrateOilLevel = temp;
    }
    /*************** 分布测量顺序 -> spreadMeasurementOrder ***********/
    if ((HOLDREGISTER_SRREAD_MEASURETURN >= startadd) &&
        (HOLDREGISTER_SRREAD_MEASURETURN <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SRREAD_MEASURETURN, 1);
        g_deviceParams.spreadMeasurementOrder = temp & 0xFFFF;
    }

    /* 每米测量方向（与顺序共用） */
    if ((HOLDREGISTER_MEASREMENT_METER >= startadd) &&
        (HOLDREGISTER_MEASREMENT_METER <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_MEASREMENT_METER, 1);
        g_deviceParams.spreadMeasurementOrder = temp & 0xFFFF;
    }

    /* 区间密度测量方向（与顺序共用） */
    if ((HOLDREGISTER_INTERVAL_DIREDION >= startadd) &&
        (HOLDREGISTER_INTERVAL_DIREDION <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_INTERVAL_DIREDION, 1);
        g_deviceParams.spreadMeasurementOrder = temp & 0xFFFF;
    }

    /*************** 分布测量模式 -> spreadMeasurementMode ************/
    if ((HOLDREGISTER_SPREAD_STATE >= startadd) &&
        (HOLDREGISTER_SPREAD_STATE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_STATE, 1);
        g_deviceParams.spreadMeasurementMode = temp & 0xFFFF;
    }

    /* 无需权限的模式 */
    if ((HOLDREGISTER_SPREAD_DSM_STATE_FREE >= startadd) &&
        (HOLDREGISTER_SPREAD_DSM_STATE_FREE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_DSM_STATE_FREE, 1);
        g_deviceParams.spreadMeasurementMode = temp & 0xFFFF;
    }

    /*************** 分布测点数 -> spreadMeasurementCount *************/
    if ((HOLDREGISTER_SPREAD_NUM >= startadd) &&
        (HOLDREGISTER_SPREAD_NUM <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_NUM, 1);
        g_deviceParams.spreadMeasurementCount = temp & 0xFFFF;
    }

    /* 无需权限分布点数 */
    if ((HOLDREGISTER_SPREAD_NUM_FREE >= startadd) &&
        (HOLDREGISTER_SPREAD_NUM_FREE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_NUM_FREE, 1);
        g_deviceParams.spreadMeasurementCount = temp & 0xFFFF;
    }

    /* 区间密度测量点数 */
    if ((HOLDREGISTER_INTERVAL_POINT >= startadd) &&
        (HOLDREGISTER_INTERVAL_POINT <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_INTERVAL_POINT, 1);
        g_deviceParams.spreadMeasurementCount = temp & 0xFFFF;
    }

    /*************** 分布测间距 -> spreadMeasurementDistance **********/
    if ((HOLDREGISTER_SPREAD_DISTANCE >= startadd) &&
        ((HOLDREGISTER_SPREAD_DISTANCE + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE, 2);
        g_deviceParams.spreadMeasurementDistance = temp;
    }

    /* 无需权限间距 */
    if ((HOLDREGISTER_SPREAD_DISTANCE_FREE >= startadd) &&
        ((HOLDREGISTER_SPREAD_DISTANCE_FREE + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_DISTANCE_FREE, 2);
        g_deviceParams.spreadMeasurementDistance = temp;
    }

    /* 固定间距分布测量的间距 */
    if ((HOLDREGISTER_SPREAD_FIXEDDISTANCE >= startadd) &&
        ((HOLDREGISTER_SPREAD_FIXEDDISTANCE + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDDISTANCE, 2);
        g_deviceParams.spreadMeasurementDistance = temp;
    }

    /*************** 顶点距液面 -> spreadTopLimit *********************/
    if ((HOLDREGISTER_SPREAD_TOPLIMIT >= startadd) &&
        ((HOLDREGISTER_SPREAD_TOPLIMIT + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT, 2);
        g_deviceParams.spreadTopLimit = temp;
    }

    if ((HOLDREGISTER_SPREAD_TOPLIMIT_FREE >= startadd) &&
        ((HOLDREGISTER_SPREAD_TOPLIMIT_FREE + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_TOPLIMIT_FREE, 2);
        g_deviceParams.spreadTopLimit = temp;
    }

    if ((HOLDREGISTER_SPREAD_FIXEDTOP >= startadd) &&
        ((HOLDREGISTER_SPREAD_FIXEDTOP + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDTOP, 2);
        g_deviceParams.spreadTopLimit = temp;
    }

    /*************** 底点距罐底 -> spreadBottomLimit ******************/
    if ((HOLDREGISTER_SPREAD_FLOORLIMIT >= startadd) &&
        ((HOLDREGISTER_SPREAD_FLOORLIMIT + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT, 2);
        g_deviceParams.spreadBottomLimit = temp;
    }

    if ((HOLDREGISTER_SPREAD_FLOORLIMIT_FREE >= startadd) &&
        ((HOLDREGISTER_SPREAD_FLOORLIMIT_FREE + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FLOORLIMIT_FREE, 2);
        g_deviceParams.spreadBottomLimit = temp;
    }

    if ((HOLDREGISTER_SPREAD_FIXEDBASE >= startadd) &&
        ((HOLDREGISTER_SPREAD_FIXEDBASE + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SPREAD_FIXEDBASE, 2);
        g_deviceParams.spreadBottomLimit = temp;
    }

    /*************** 水位修正 -> waterLevelCorrection *****************/
    if ((HOLDREGISTER_REAL_WATER_LEVEL_CORRECT >= startadd) &&
        (HOLDREGISTER_REAL_WATER_LEVEL_CORRECT <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_REAL_WATER_LEVEL_CORRECT, 1);
        g_deviceParams.waterLevelCorrection = (temp & 0xFFFF);
    }

    /*************** 液位测量方式 -> liquidLevelMeasurementMethod *****/
    if ((HOLDREGISTER_LEVEL_MEASURE_METHOD >= startadd) &&
        (HOLDREGISTER_LEVEL_MEASURE_METHOD <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_LEVEL_MEASURE_METHOD, 1);
        g_deviceParams.liquidLevelMeasurementMethod = temp & 0xFFFF;
    }

    /*************** 盲区 -> blindZone ********************************/
    if ((HOLDREGISTER_FADEZERO >= startadd) &&
        ((HOLDREGISTER_FADEZERO + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_FADEZERO, 2);
        g_deviceParams.blindZone = temp;
    }

    /*************** 导轮周长 -> encoder_wheel_circumference_mm *******/
    if ((HOLDREGISTER_GIRTH_YITI >= startadd) &&
        ((HOLDREGISTER_GIRTH_YITI + 1) <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_GIRTH_YITI, 2);
        g_deviceParams.encoder_wheel_circumference_mm = temp;
    }

    /*************** 最大下行距离 -> maxDownDistance ******************/
    if ((HOLDREGISTER_MAXDOWN_DIS >= startadd) &&
        (HOLDREGISTER_MAXDOWN_DIS <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_MAXDOWN_DIS, 1);
        g_deviceParams.maxDownDistance = temp & 0xFFFF;
    }

    /*************** 综合指令测罐底 / 水位 / 单点密度 ******************/
    if ((HOLDREGISTER_SYNTHETIC_BOTTOM >= startadd) &&
        (HOLDREGISTER_SYNTHETIC_BOTTOM <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM, 1);
        g_deviceParams.requireBottomMeasurement = temp & 0xFFFF;
    }

    if ((HOLDREGISTER_SYNTHETIC_WATER >= startadd) &&
        (HOLDREGISTER_SYNTHETIC_WATER <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER, 1);
        g_deviceParams.requireWaterMeasurement = temp & 0xFFFF;
    }

    if ((HOLDREGISTER_SYNTHETIC_SINGLEPOINT >= startadd) &&
        (HOLDREGISTER_SYNTHETIC_SINGLEPOINT <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT, 1);
        g_deviceParams.requireSinglePointDensity = temp & 0xFFFF;
    }

    /* 无需权限综合指令对应的三项 */
    if ((HOLDREGISTER_SYNTHETIC_BOTTOM_FREE >= startadd) &&
        (HOLDREGISTER_SYNTHETIC_BOTTOM_FREE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_BOTTOM_FREE, 1);
        g_deviceParams.requireBottomMeasurement = temp & 0xFFFF;
    }

    if ((HOLDREGISTER_SYNTHETIC_WATER_FREE >= startadd) &&
        (HOLDREGISTER_SYNTHETIC_WATER_FREE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_WATER_FREE, 1);
        g_deviceParams.requireWaterMeasurement = temp & 0xFFFF;
    }

    if ((HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE >= startadd) &&
        (HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE, 1);
        g_deviceParams.requireSinglePointDensity = temp & 0xFFFF;
    }

    /*************** 温度/密度修正 -> temperatureCorrection/densityCorrection ****/
    if ((HOLDREGISTER_TEMCORRECTCALUE >= startadd) &&
        (HOLDREGISTER_TEMCORRECTCALUE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_TEMCORRECTCALUE, 1);
        g_deviceParams.temperatureCorrection = (temp & 0xFFFF);
    }

    if ((HOLDREGISTER_DENSITYCORRECTCALUE >= startadd) &&
        (HOLDREGISTER_DENSITYCORRECTCALUE <= end))
    {
        temp = ReadOneHoldingRegister(HOLDREGISTER_DENSITYCORRECTCALUE, 1);
        g_deviceParams.densityCorrection = (temp & 0xFFFF);
    }

    /* 其它旧寄存器如果在新 DeviceParameters 中没有对应，就不处理 */

    /* 同步修改过的 DeviceParameters → CPU2 */
    DeviceParams_SyncAllToCPU2();
    return 0;
}


/******************************************************
 函数功能： 设置寄存器赋值

 函 数 名： Input_Write
 参    数：

 返 回 值：
 ******************************************************/
void Input_Write(void) {
	int i;
	WriteOneInputRegister(INPUTREGISTER_PATTERNOFWORK, 1, 1);				 					//工作模式固定为调试模式
	WriteOneInputRegister(INPUTREGISTER_SYSTEMSTATE, 1, g_measurement.device_status.device_state);									//工作状态
	WriteOneInputRegister(INPUTREGISTER_ERRORNUM, 2, g_measurement.device_status.error_code);
	//故障代码
	WriteOneInputRegister(INPUTREGISTER_SP_TEMPERATURE, 1, g_measurement.single_point_measurement.temperature);									//单点测量温度
	WriteOneInputRegister(INPUTREGISTER_SP_DENSITY, 1, g_measurement.single_point_measurement.density);									//单点测量密度
	WriteOneInputRegister(INPUTREGISTER_SP_POSITION, 2, g_measurement.single_point_measurement.temperature_position);								//单点测量密度点位置
	WriteOneInputRegister(INPUTREGISTER_SP_STANDARDDENSITY, 1, g_measurement.single_point_measurement.standard_density);							//单点测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SP_VCF20, 2, g_measurement.single_point_measurement.vcf20);											//单点测量的VCF
	WriteOneInputRegister(INPUTREGISTER_SP_WEIGHTDENSITY, 1, g_measurement.single_point_measurement.weight_density);						//单点测量的计重密度

	WriteOneInputRegister(INPUTREGISTER_SPT_TEMPERATURE, 1, g_measurement.single_point_monitoring.temperature);									//单点检测温度
	WriteOneInputRegister(INPUTREGISTER_SPT_DENSITY, 1, g_measurement.single_point_monitoring.density);										//单点检测密度
	WriteOneInputRegister(INPUTREGISTER_SPT_POSITION, 2, g_measurement.single_point_monitoring.temperature_position);								//单点检测密度点位置
	WriteOneInputRegister(INPUTREGISTER_SPT_STANDARDDENSITY, 1, g_measurement.single_point_monitoring.standard_density);							//单点检测标准密度
	WriteOneInputRegister(INPUTREGISTER_SPT_VCF20, 2, g_measurement.single_point_monitoring.vcf20);
	WriteOneInputRegister(INPUTREGISTER_SPT_WEIGHTDENSITY, 1, g_measurement.single_point_monitoring.weight_density);

	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGETEMPERATURE, 1, g_measurement.density_distribution.average_temperature);					//平均温度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGEDENSITY, 1, g_measurement.density_distribution.average_density);							//平均密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY, 1, g_measurement.density_distribution.average_standard_density);					//分布测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF20, 2, g_measurement.density_distribution.average_vcf20);									//分布测量VCF
	WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY, 1, g_measurement.density_distribution.average_weight_density);						//分布测量计重密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_NUMOFDENSITY, 1, g_measurement.density_distribution.measurement_points);								//分布测量密度点数
	WriteOneInputRegister(INPUTREGISTER_LIQUIDLEVEL, 2, g_measurement.density_distribution.Density_oil_level);				//分布测量液位值

	WriteOneInputRegister(INPUTREGISTER_OILLEVEL, 2, g_measurement.oil_measurement.oil_level);			//液位值 赋值
	WriteOneInputRegister(INPUTREGISTER_WATERLEVEL, 2, g_measurement.water_measurement.water_level);			//水位值 赋值
	WriteOneInputRegister(INPUTREGISTER_BOTTOMHIGH, 2, g_measurement.height_measurement.current_real_height);			//罐底值 赋值
	WriteOneInputRegister(INPUTREGISTER_REALTIMEPOSITION, 2, g_measurement.debug_data.sensor_position);			//传感器
	WriteOneInputRegister(INPUTREGISTER_VALUE_P1, 2, 0);			//液体压力值
	WriteOneInputRegister(INPUTREGISTER_VALUE_P3, 2, 0);			//传感值

	if (g_measurement.density_distribution.measurement_points > 16) {
		for (i = 0; i < 16; i++) {
			WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].temperature);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].density);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION1 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].temperature_position);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].standard_density);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF1 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].vcf20);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].weight_density);
		}

		for (i = 0; i < 100 - 16; i++) {
			WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE17 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].temperature);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY17 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].density);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION17 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].temperature_position);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY17 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].standard_density);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF17 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].vcf20);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY17 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].weight_density);
		}
	} else {
		for (i = 0; i < g_measurement.density_distribution.measurement_points; i++) {
			WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].temperature);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].density);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION1 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].temperature_position);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].standard_density);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF1 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].vcf20);
			WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].weight_density);
		}
	}
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGETEMPERATURE_M, 1, g_measurement.density_distribution.average_temperature);					//平均温度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGEDENSITY_M, 1, g_measurement.density_distribution.average_density);							//平均密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY_M, 1, g_measurement.density_distribution.average_standard_density);					//分布测量标准密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF20_M, 2, g_measurement.density_distribution.average_vcf20);									//分布测量VCF
	WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY_M, 1, g_measurement.density_distribution.average_weight_density);						//分布测量计重密度
	WriteOneInputRegister(INPUTREGISTER_SPREAD_NUMOFDENSITY_M, 1, g_measurement.density_distribution.measurement_points);								//分布测量密度点数
	WriteOneInputRegister(INPUTREGISTER_LIQUIDLEVEL_M, 2, g_measurement.density_distribution.Density_oil_level);				//分布测量液位值

	for (i = 0; i < 100; i++) //V1.116 dq2020.4.2
			{
		WriteOneInputRegister(INPUTREGISTER_SPREAD_TEMPERATURE_M1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].temperature);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_DENSITY_M1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].density);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_POSITION_M1 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].temperature_position);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_STANDARDDENSITY_M1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].standard_density);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_VCF_M1 + 8 * i, 2, g_measurement.density_distribution.single_density_data[i].vcf20);
		WriteOneInputRegister(INPUTREGISTER_SPREAD_WEIGHTDENSITY_M1 + 8 * i, 1, g_measurement.density_distribution.single_density_data[i].weight_density);
	}
	WriteOneInputRegister(INPUTREGISTER_ZEROCIRCLE, 1, 0);									//零点圈数
	WriteOneInputRegister(INPUTREGISTER_ZEROANGLE, 1, 0);										//零点角度
	WriteOneInputRegister(INPUTREGISTER_TANKHIGHT, 2, 0);							//修正罐高
	WriteOneInputRegister(INPUTREGISTER_CIRCLE, 1, 0);										//当前圈数(0X7F9C是软件里面做的运算) 改为X轴角度
	WriteOneInputRegister(INPUTREGISTER_ANGLE, 1, 0);											//当前角度  改为Y轴角度
	WriteOneInputRegister(INPUTREGISTER_SENSORPOSITION, 2, g_measurement.debug_data.sensor_position);			//传感器位置
	WriteOneInputRegister(INPUTREGISTER_FREQUENCE, 2, g_measurement.debug_data.frequency);										//传感器频率
	WriteOneInputRegister(INPUTREGISTER_TEMPERATURE, 1, g_measurement.debug_data.temperature);								//传感器温度()
	WriteOneInputRegister(INPUTREGISTER_STATEOFEINDUCTION, 1, 0);						//霍尔开关状态(改为带宽)														//干簧管状态
	WriteOneInputRegister(INPUTREGISTER_FREQUENCEINAIR, 2, g_measurement.debug_data.air_frequency);							//空气中频率
	WriteOneInputRegister(INPUTREGISTER_AMPLITUDE, 1, 0);										//传感器幅值
	WriteOneInputRegister(INPUTREGISTER_SENSORX_ANGLE, 1, 0);								//传感器X角度
	WriteOneInputRegister(INPUTREGISTER_SENSORY_ANGLE, 1, 0);								//传感器Y角度
	WriteOneInputRegister(INPUTREGISTER_WARTER_VOLTAGE, 1, g_measurement.debug_data.water_level_voltage);							//水位传感器电压
	WriteOneInputRegister(INPUTREGISTER_REAL_TANKHIGH_ORIGIN, 2, g_measurement.height_measurement.calibrated_liquid_level);
	WriteOneInputRegister(INPUTREGISTER_REAL_TANKHIGH_NOW, 2,  g_measurement.height_measurement.current_real_height);
}


