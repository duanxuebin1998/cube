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
#include "stateformodbus.h"
#include "address.h"
/*
 通讯的应用层，对数据进行接卸或者写入。
 */
//#define DEBUG_DA
#ifdef DEBUG_DA
	#include "usart1.h"
#endif


/******************************************************
 函数功能： 设置输入寄存器初始值

 函 数 名： Parameter_Init
 参    数：

 返 回 值：
 ******************************************************/
void Parameter_Init(void) {
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

	for (i = 0; i < 16; i++) {
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
	WriteOneInputRegister(INPUTREGISTER_REALTIMEPOSITION, 2, g_measurement.debug_data.sensor_position);			//传感器实饰恢帽 赋值
	WriteOneInputRegister(INPUTREGISTER_VALUE_P1, 2, 0);			//液体压力值 赋值 孟
	WriteOneInputRegister(INPUTREGISTER_VALUE_P3, 2, 0);			//传感器实饰恢帽 赋值

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
int ProcessWriteCoil(void) {
	uint32_t cmd = 1;
//	static const uint32_t nopara_cmd_map[][2] = {
//	    { COM_NUM_BACK_ZERO,           CMD_BACK_ZERO },           // 返回零点
//	    { COM_NUM_FIND_ZERO,           CMD_CALIBRATE_ZERO },        // 标定零点
//	    { COM_NUM_SPREADPOINTS_AI,     CMD_MEASURE_DISTRIBUTED },// 自动分布测量
//	    { COM_NUM_FIND_OIL,            CMD_FIND_OIL },        // 寻找液位
//	    { COM_NUM_FIND_WATER,          CMD_FIND_WATER },      // 寻找水位
//	    { COM_NUM_FIND_BOTTOM,         CMD_FIND_BOTTOM },           // 寻找罐底
//	    { COM_NUM_SYNTHETIC,           CMD_SYNTHETIC },     // 综合测量
//	    { COM_NUM_METER_DENSITY,       CMD_MEASURE_DENSITY_METER }, // 每米测量
//	    { COM_NUM_INTERVAL_DENSITY,    CMD_MEASURE_DENSITY_RANGE }, // 区间测量
//	    { COM_NUM_READPARAMETER,       CMD_READ_PARAM },       // 读取参数
//	    { COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },    // 恢复出厂设置
//	};

//	    int mapamount = sizeof(nopara_cmd_map) / sizeof(nopara_cmd_map[0]);
//	    int i;
//	    //发送指令
//
//	    for(i = 0;i < mapamount;i++)
//	    {
//	        if(now_Opera_Num == nopara_cmd_map[i][0])
//	        {
//	           CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,HOLDREGISTER_DEVICEPARAM_COMMAND,2,&nopara_cmd_map[i][1]);
//	           printf("send command to cpu2 %lu\r\n", cmd);
//	           break;
//	        }
//	    }
	CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, 2, &cmd); //发送命令给CPU2

	return RETURN_OK;
}
