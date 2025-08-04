#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dataanalysis_modbus.h"
#include "usart.h"
#include "stateformodbus.h"	 
#include <ctype.h>
#include <float.h>
#include "weight.h"

static int Rounding(int value); //四舍五入

#define WRITE_UINT32_TO_HOLDING(addr, val)                  \
		HoldingRegisterArray[(addr)]     = ((val) >> 16) & 0xFFFF; \
    HoldingRegisterArray[(addr) + 1] = (val) & 0xFFFF;
#define READ_UINT32_FROM_HOLDING(addr, dest)                             \
    dest = (((uint32_t)HoldingRegisterArray[(addr)] << 16) |             \
             (uint32_t)HoldingRegisterArray[(addr) + 1])

void WriteDeviceParamsToHoldingRegisters(int *HoldingRegisterArray) {
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_TANKHEIGHT, g_deviceParams.tankHeight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_BLINDZONE, g_deviceParams.blindZone);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, g_deviceParams.encoder_wheel_circumference_mm);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SENSORTYPE, g_deviceParams.sensorType);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION, g_deviceParams.softwareVersion);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT, g_deviceParams.empty_weight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION, g_deviceParams.densityCorrection);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION, g_deviceParams.temperatureCorrection);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, g_deviceParams.requireBottomMeasurement);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT, g_deviceParams.requireWaterMeasurement);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, g_deviceParams.requireSinglePointDensity);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER, g_deviceParams.spreadMeasurementOrder);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE, g_deviceParams.spreadMeasurementMode);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT, g_deviceParams.spreadMeasurementCount);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, g_deviceParams.spreadMeasurementDistance);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT, g_deviceParams.spreadTopLimit);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT, g_deviceParams.spreadBottomLimit);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION, g_deviceParams.waterLevelCorrection);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE, g_deviceParams.maxDownDistance);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG, g_deviceParams.refreshTankHeightFlag);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION, g_deviceParams.maxTankHeightDeviation);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT, g_deviceParams.initialTankHeight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT, g_deviceParams.currentTankHeight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD, g_deviceParams.oilLevelThreshold);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, g_deviceParams.liquidLevelMeasurementMethod);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_CRC, g_deviceParams.crc);
}

void ReadDeviceParamsFromHoldingRegisters(int *HoldingRegisterArray) {
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_TANKHEIGHT, g_deviceParams.tankHeight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_BLINDZONE, g_deviceParams.blindZone);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, g_deviceParams.encoder_wheel_circumference_mm);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SENSORTYPE, g_deviceParams.sensorType);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION, g_deviceParams.softwareVersion);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT, g_deviceParams.empty_weight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION, g_deviceParams.densityCorrection);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION, g_deviceParams.temperatureCorrection);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, g_deviceParams.requireBottomMeasurement);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT, g_deviceParams.requireWaterMeasurement);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, g_deviceParams.requireSinglePointDensity);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER, g_deviceParams.spreadMeasurementOrder);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE, g_deviceParams.spreadMeasurementMode);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT, g_deviceParams.spreadMeasurementCount);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, g_deviceParams.spreadMeasurementDistance);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT, g_deviceParams.spreadTopLimit);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT, g_deviceParams.spreadBottomLimit);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION, g_deviceParams.waterLevelCorrection);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE, g_deviceParams.maxDownDistance);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG, g_deviceParams.refreshTankHeightFlag);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION, g_deviceParams.maxTankHeightDeviation);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT, g_deviceParams.initialTankHeight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT, g_deviceParams.currentTankHeight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD, g_deviceParams.oilLevelThreshold);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, g_deviceParams.liquidLevelMeasurementMethod);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_CRC, g_deviceParams.crc);
}

void ResetInputRegister(int *p_inputregister) {
//    int ave_tmp,state_tmp;
//    int i,j;
//    int tmp;
//    union DoubleU32 data;
//    union FolatInt floatdata;
//    //液位
//    p_inputregister[INPUTREGISTER_VALUE_LEVEL] = result.OilLevel >> 16;
//    p_inputregister[INPUTREGISTER_VALUE_LEVEL + 1] = result.OilLevel & 0x0000FFFF;
//    //新协议段
//    p_inputregister[INPUTREGISTER_NEW_VALUE_LEVEL] = result.OilLevel >> 16;
//    p_inputregister[INPUTREGISTER_NEW_VALUE_LEVEL + 1] = result.OilLevel & 0x0000FFFF;
//    //报警状态
//    p_inputregister[INPUTREGISTER_STATE_ALARM] = result.Alarm & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_STATE_ALARM] = result.Alarm & 0x0000FFFF;
//    /*液位计状态*/
//    p_inputregister[INPUTREGISTER_STATE_LEVEL] = EquipmentState & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_STATE_LEVEL] = EquipmentState & 0x0000FFFF;
//    /*错误码*/
//    p_inputregister[INPUTREGISTER_ERRORCODE] = EquipmentErrorCode & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_ERRORCODE] = EquipmentErrorCode & 0x0000FFFF;
//    /*称重值(维护模式下)*/
//    p_inputregister[INPUTREGISTER_VALUE_GETWEIGHT] = (result.CurrentWeight / 10) & 0x0000FFFF;
//    /*步进数(维护模式下)*/
//    p_inputregister[INPUTREGISTER_VALUE_MOTORSTEPS] = ((result.CurrentMotorSteps + 5000000) >>16) & 0xFFFF;
//    p_inputregister[INPUTREGISTER_VALUE_MOTORSTEPS + 1] = (result.CurrentMotorSteps + 5000000) & 0x0000FFFF;
//    /*罐高*/
//    p_inputregister[INPUTREGISTER_VALUE_TANKHEIGHT] = result.BottomPosition >> 16;
//    p_inputregister[INPUTREGISTER_VALUE_TANKHEIGHT + 1] = (result.BottomPosition) & 0xFFFF;
//    /*温度计类型*/
//    p_inputregister[INPUTREGISTER_TYPEOFTHERMOMETER] = systemparameter.TempType & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_TYPEOFTHERMOMETER] = systemparameter.TempType & 0x0000FFFF;
//    /*平均温度*/
//    CalculateAverageTemp(&ave_tmp,&state_tmp);
//    ave_tmp = Rounding(ave_tmp);
//    //ave_tmp = result.TemperatureValueAverage;//测试用
//    p_inputregister[INPUTREGISTER_VALUE_AVERAGETEMPERATURE] = (ave_tmp + 1000) & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_VALUE_AVERAGETEMPERATURE] = (ave_tmp + 1000) & 0x0000FFFF;
//    result.TemperatureValueAverage = ave_tmp;//用于计量换算
//    /*平均温度状态*/
//    p_inputregister[INPUTREGISTER_STATE_AVERAGETEMPERATURE] = state_tmp & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_STATE_AVERAGETEMPERATURE] = state_tmp & 0x0000FFFF;
//    /*平均密度*/
//    p_inputregister[INPUTREGISTER_VALUE_AVERAGEDENSITY] = (result.DensityValueAverage) & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_VALUE_AVERAGEDENSITY] = (result.DensityValueAverage) & 0x0000FFFF;
//    /*平均密度状态*/
//    p_inputregister[INPUTREGISTER_STATE_AVERAGEDENSITY] = (result.DensityAveState) & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_STATE_AVERAGEDENSITY] = (result.DensityAveState) & 0x0000FFFF;
//    /*实时温度与位置-共16个点*/
//    for(i = 0;i<REALTEMPMAXSPOT;i++)
//    {
//        p_inputregister[INPUTREGISTER_VALUE_TEMPERATURE1 + i * 3] = (Rounding(result.ThermometerValue[i]) + 1000) & 0x0000FFFF;
//        p_inputregister[INPUTREGISTER_VALUE_TEMPERATURE1 + i * 3 + 1] = (result.ThermometerPosition[i] >> 16) & 0xFFFF;
//        p_inputregister[INPUTREGISTER_VALUE_TEMPERATURE1 + i * 3 + 2] = (result.ThermometerPosition[i]) & 0xFFFF;
//    }
//    /*无线温度值(维护)*/
//    p_inputregister[INPUTREGISTER_VALUE_TEMPERATURE] = (result.CurrentWireTemperatur + 1000) & 0x0000FFFF;
//    /*无线温度*/
//    for(i = 0;i<3;i++)
//    {
//        p_inputregister[INPUTREGISTER_VALUE_WIRELESSTEMPERATURE1 + i * 3] = ((result.WireTempValue[i]) + 1000) & 0x0000FFFF;
//        p_inputregister[INPUTREGISTER_VALUE_WIRELESSTEMPERATURE1 + i * 3 + 1] = (result.DensityPosition[i] >> 16) & 0xFFFF;
//        p_inputregister[INPUTREGISTER_VALUE_WIRELESSTEMPERATURE1 + i * 3 + 2] = (result.DensityPosition[i]) & 0xFFFF;
//    }
//    for(i = 3,j = 0;i<WIRELESSMAXSPOT;i++,j++)
//    {
//        p_inputregister[INPUTREGISTER_VALUE_WIRELESSTEMPERATURE4 + j * 3] = ((result.WireTempValue[i]) + 1000) & 0x0000FFFF;
//        p_inputregister[INPUTREGISTER_VALUE_WIRELESSTEMPERATURE4 + j * 3 + 1] = (result.DensityPosition[i] >> 16) & 0xFFFF;
//        p_inputregister[INPUTREGISTER_VALUE_WIRELESSTEMPERATURE4 + j * 3 + 2] = (result.DensityPosition[i]) & 0xFFFF;
//    }
//    /*指定高度无线温度*/
//    p_inputregister[INPUTREGISTER_VALUE_TEMPERATUREASSIGNHEIGHT] = ((result.SingleTemperature) + 1000) & 0x0000FFFF;
//    /*指定高度密度*/
//    p_inputregister[INPUTREGISTER_VALUE_DENSITYASSIGNHEIGHT] = (result.SingleDensity) & 0x0000FFFF;
//    /*指定位置*/
//    p_inputregister[INPUTREGISTER_POSITION_DENSITYASSIGNHEIGHT] = (result.SinglePosition >> 16) & 0xFFFF;
//    p_inputregister[INPUTREGISTER_POSITION_DENSITYASSIGNHEIGHT + 1] = (result.SinglePosition) & 0xFFFF;
//    /*密度*/
//    p_inputregister[INPUTREGISTER_VALUE_SPOTSOFDENSITY] = result.ActualDensityPoints & 0x0000FFFF;
//    for(i = 0;i<WIRELESSMAXSPOT;i++)
//    {
//        p_inputregister[INPUTREGISTER_VALUE_DENSITY1 + i * 3] = (result.DensityValue[i]) & 0x0000FFFF;
//        p_inputregister[INPUTREGISTER_VALUE_DENSITY1 + i * 3 + 1] = (result.DensityPosition[i] >> 16) & 0xFFFF;
//        p_inputregister[INPUTREGISTER_VALUE_DENSITY1 + i * 3 + 2] = (result.DensityPosition[i]) & 0xFFFF;
//    }
//    /*水位值*/
//    p_inputregister[INPUTREGISTER_VALUE_WATER] = result.WaterValue >> 16;
//    p_inputregister[INPUTREGISTER_VALUE_WATER + 1] = (result.WaterValue) & 0xFFFF;
//    p_inputregister[INPUTREGISTER_NEW_VALUE_WATER] = result.WaterValue >> 16;
//    p_inputregister[INPUTREGISTER_NEW_VALUE_WATER + 1] = (result.WaterValue) & 0xFFFF;
//    /*水位状态*/
//    p_inputregister[INPUTREGISTER_STATE_WATER] = result.WaterState & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_STATE_WATER] = result.WaterState & 0x0000FFFF;
//    /*压力*/
//    p_inputregister[INPUTREGISTER_PRESSUREP1] = result.PressureValue >> 16;
//    p_inputregister[INPUTREGISTER_PRESSUREP1 + 1] = (result.PressureValue) & 0xFFFF;
//    p_inputregister[INPUTREGISTER_PRESSUREP2] = result.Pressure2Value >> 16;
//    p_inputregister[INPUTREGISTER_PRESSUREP2 + 1] = (result.Pressure2Value) & 0xFFFF;
//    p_inputregister[INPUTREGISTER_NEW_PRESSUREP1] = result.PressureValue >> 16;
//    p_inputregister[INPUTREGISTER_NEW_PRESSUREP1 + 1] = (result.PressureValue) & 0xFFFF;
//    p_inputregister[INPUTREGISTER_NEW_PRESSUREP2] = result.Pressure2Value >> 16;
//    p_inputregister[INPUTREGISTER_NEW_PRESSUREP2 + 1] = (result.Pressure2Value) & 0xFFFF;
//    /*100个密度点*/
//    //有效点数
//    p_inputregister[INPUTREGISTER_VALIDPOINTS] = result.ActualDensityPoints & 0x0000FFFF;
//    //密度模式
//    p_inputregister[INPUTREGISTER_DENSITYTYPE] = TypeofDensityMode & 0x0000FFFF;
//    //100个密度点数据
//    for(i = 0;i < INTERVALDENSITYMAXSPOT;i++)
//    {
//        //密度
//        p_inputregister[INPUTREGISTER_DENSITY_1 + i * 8] = result.Distribute_Nation_Arr[i].density & 0x0000FFFF;
//        //温度
//        p_inputregister[INPUTREGISTER_WIRELESSTEMP_1 + i * 8] = (result.Distribute_Nation_Arr[i].temperature + 1000) & 0x0000FFFF;
//        //位置
//        p_inputregister[INPUTREGISTER_POSITION_1 + i * 8] = result.Distribute_Nation_Arr[i].position >> 16;
//        p_inputregister[INPUTREGISTER_POSITION_1 + 1 + i * 8] = (result.Distribute_Nation_Arr[i].position) & 0xFFFF;
//        //标密
//        p_inputregister[INPUTREGISTER_DENSITY20_1 + i * 8] = result.Distribute_Nation_Arr[i].density20 & 0x0000FFFF;
//        //VCF
//        p_inputregister[INPUTREGISTER_VCF_1 + i * 8] = ((int)(result.Distribute_Nation_Arr[i].vcf * 100000)) >> 16;
//        p_inputregister[INPUTREGISTER_VCF_1 + 1 + i * 8] = ((int)(result.Distribute_Nation_Arr[i].vcf * 100000)) & 0xFFFF;
//        //计重密度
//        p_inputregister[INPUTREGISTER_DENSITYGW_1 + i * 8] = result.Distribute_Nation_Arr[i].density_gw & 0x0000FFFF;
//    }
//    //单点数据
//    //密度
//    p_inputregister[INPUTREGISTER_DENSITY_SINGLE] = result.SingleData.density & 0x0000FFFF;
//    //温度
//    p_inputregister[INPUTREGISTER_WIRELESSTEMP_SINGLE] = (result.SingleData.temperature + 1000) & 0x0000FFFF;
//    //位置
//    p_inputregister[INPUTREGISTER_POSITION_SINGLE] = result.SingleData.position >> 16;
//    p_inputregister[INPUTREGISTER_POSITION_SINGLE + 1] = (result.SingleData.position) & 0xFFFF;
//    //标密
//    p_inputregister[INPUTREGISTER_DENSITY20_SINGLE] = result.SingleData.density20 & 0x0000FFFF;
//    //VCF
//    p_inputregister[INPUTREGISTER_VCF_SINGLE] = ((int)(result.SingleData.vcf * 100000)) >> 16;
//    p_inputregister[INPUTREGISTER_VCF_SINGLE + 1] = ((int)(result.SingleData.vcf * 100000)) & 0xFFFF;
//    //计重密度
//    p_inputregister[INPUTREGISTER_DENSITYGW_SINGLE] = result.SingleData.density_gw & 0x0000FFFF;
//    /////////////////////////////////////新轮询协议段///////////////////////////////////////////////////
//    //浮子位置
//    tmp = systemparameter.YeildCalibration - ( GetDistanceFromCode(Current_Code) * 10 );
//    p_inputregister[INPUTREGISTER_NEW_DISPLACERPOSI] = tmp >> 16;
//    p_inputregister[INPUTREGISTER_NEW_DISPLACERPOSI + 1] = (tmp) & 0xFFFF;
//    //平均标密
//    p_inputregister[INPUTREGISTER_NEW_DENSITY20] = (int)(Convert.Density20 * 10) & 0x0000FFFF;
//    //平均VCF
//    p_inputregister[INPUTREGISTER_NEW_VCF] = (int)(Convert.VCF20 * 100000) >> 16;
//    p_inputregister[INPUTREGISTER_NEW_VCF + 1] = (int)(Convert.VCF20 * 100000) & 0xFFFF;
//    //平均计重密度
//    p_inputregister[INPUTREGISTER_NEW_DENSITYGW] = (int)(Convert.GWDensity * 10) & 0x0000FFFF;
//    //水体积
//    data.d = Convert.FOV;
//    p_inputregister[INPUTREGISTER_NEW_FOV] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_FOV + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_FOV + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_FOV + 3] = data.u[0] & 0x0000FFFF;
//    //净容积
//    data.d = Convert.GOV;
//    p_inputregister[INPUTREGISTER_NEW_GOV] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_GOV + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_GOV + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_GOV + 3] = data.u[0] & 0x0000FFFF;
//    //油水总体积
//    data.d = Convert.TOV;
//    p_inputregister[INPUTREGISTER_NEW_TOV] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_TOV + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_TOV + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_TOV + 3] = data.u[0] & 0x0000FFFF;
//    //质量
//    data.d = Convert.GSW;
//    p_inputregister[INPUTREGISTER_NEW_GSW] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_GSW + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_GSW + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_GSW + 3] = data.u[0] & 0x0000FFFF;
//    //罐壁修正系数
//    data.d = Convert.CTSh;
//    p_inputregister[INPUTREGISTER_NEW_CTSH] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_CTSH + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_CTSH + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_CTSH + 3] = data.u[0] & 0x0000FFFF;
//    //毛标准体积
//    data.d = Convert.GSV;
//    p_inputregister[INPUTREGISTER_NEW_GSV] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_GSV + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_GSV + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_GSV + 3] = data.u[0] & 0x0000FFFF;
//    //表观质量换算系数
//    data.d = Convert.WCF;
//    p_inputregister[INPUTREGISTER_NEW_WCF] = data.u[1] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_WCF + 1] = data.u[1] & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_WCF + 2] = data.u[0] >> 16;
//    p_inputregister[INPUTREGISTER_NEW_WCF + 3] = data.u[0] & 0x0000FFFF;
//    //称重
//    p_inputregister[INPUTREGISTER_NEW_WEIGHT] = WeightValue >> 16;
//    p_inputregister[INPUTREGISTER_NEW_WEIGHT + 1] = (WeightValue) & 0xFFFF;
//    //管道流速
//    floatdata.f = result.PipeFlowVelocity;
//    p_inputregister[INPUTREGISTER_NEW_PIPEVELOCITY] = floatdata.i >> 16;
//    p_inputregister[INPUTREGISTER_NEW_PIPEVELOCITY + 1] = floatdata.i & 0x0000FFFF;
//    //管道流量
//    floatdata.f = result.PipeFlux;
//    p_inputregister[INPUTREGISTER_NEW_PIPEFLUX] = floatdata.i >> 16;
//    p_inputregister[INPUTREGISTER_NEW_PIPEFLUX + 1] = floatdata.i & 0x0000FFFF;
//    /*拟合*/
//    //拟合点数
//    p_inputregister[INPUTREGISTER_NEW_SINEPOINTS] = FITTINGARRAYLENGTH & 0x0000FFFF;
//    //拟合数组
//    for(i=0;i<FITTINGARRAYLENGTH;i++)
//    {
//        p_inputregister[INPUTREGISTER_NEW_SINESTART + i * 2] = systemparameter.FittingArray[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_SINESTART + 1 + i * 2] = systemparameter.FittingArray[i] & 0xFFFF;
//    }
//    /*重复性*/
//    for(i=0;i<REPEATMAX;i++)
//    {
//        //零点重复性
//        p_inputregister[INPUTREGISTER_NEW_ORIGINPOS_START + i * 2] = result.repeat.originposition[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_ORIGINPOS_START + 1 + i * 2] = result.repeat.originposition[i] & 0xFFFF;
//        //液位重复性
//        p_inputregister[INPUTREGISTER_NEW_OILLEVEL_START + i * 2] = result.repeat.oillevel[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_OILLEVEL_START + 1 + i * 2] = result.repeat.oillevel[i] & 0xFFFF;
//        //罐底重复性
//        p_inputregister[INPUTREGISTER_NEW_BOTTOM_START + i * 2] = result.repeat.bottomposition[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_BOTTOM_START + 1 + i * 2] = result.repeat.bottomposition[i] & 0xFFFF;
//        //水位重复性
//        p_inputregister[INPUTREGISTER_NEW_WATER_START + i * 2] = result.repeat.waterlevel[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_WATER_START + 1 + i * 2] = result.repeat.waterlevel[i] & 0xFFFF;
//        //密度温度重复性
//        p_inputregister[INPUTREGISTER_NEW_DENSITY_TEMP_START + i * 2] = result.repeat.deadzonedensity[i] & 0x0000FFFF;
//        p_inputregister[INPUTREGISTER_NEW_DENSITY_TEMP_START + 1 + i * 2] = (result.repeat.deadzonetemperature[i] + 1000) & 0x0000FFFF;
//        //运行带宽重复性
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + i * 8] = result.repeat.runMaxWeightInAir[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 1 + i * 8] = result.repeat.runMaxWeightInAir[i] & 0xFFFF;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 2 + i * 8] = result.repeat.bandwidthInAir[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 3 + i * 8] = result.repeat.bandwidthInAir[i] & 0xFFFF;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 4 + i * 8] = result.repeat.runMaxWeightInOil[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 5 + i * 8] = result.repeat.runMaxWeightInOil[i] & 0xFFFF;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 6 + i * 8] = result.repeat.bandwidthInOil[i] >> 16;
//        p_inputregister[INPUTREGISTER_NEW_BANDWIDTH_START + 7 + i * 8] = result.repeat.bandwidthInOil[i] & 0xFFFF;
//    }
//    //正重置阈值到第几点
//    p_inputregister[INPUTREGISTER_NEW_SINEPOINT] = GetResetWeightPoint() & 0x0000FFFF;
//    //电流
//    p_inputregister[INPUTREGISTER_NEW_CURRENT] = (int)(result.CurrentValue * 1000) & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_NOWV] = (result.VoltageCurrent) & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_MAXV] = (result.VoltageMax) & 0x0000FFFF;
//    p_inputregister[INPUTREGISTER_NEW_MINV] = (result.VoltageMin) & 0x0000FFFF;
//    //零点位置
//    p_inputregister[INPUTREGISTER_NEW_ORIGINEPOSITION] = (result.OriginPosition) & 0x0000FFFF;
//    //超范围参数的序号
//    p_inputregister[INPUTREGISTER_NEW_PARA_ERR_NUM] = (result.para_err_num) & 0x0000FFFF;
}

//static int Rounding(int value)
//{
//    if(value == UNVALID_TEMPERATURE_REALTIME)
//    {
//        return value / 10;
//    }
//    else if(value >= 0)
//    {
//        return (value + 5) / 10;
//    }
//    else
//    {
//        return (value - 5) / 10;
//    }
//}

/*执行命令*/
int ProcessWriteCoil(int const *p_coilarray) {
//    int ret = 0;

//    /*标定零点*/
//    if(p_coilarray[COIL_CALIBRATIONORIGIN - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_ORIGINCALIBRATION:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_ORIGINCALIBRATION;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_ORIGINCALIBRATION_START;
//                break;
//            }
//        }
//    }
//    /*液位跟随*/
//    else if(p_coilarray[COIL_FINDLEVEL - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_LEVELFIND:
//            case STATE_LEVELFOLLOWWING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_LEVELFIND;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREOILFOLLOW_START;
//                break;
//            }
//        }
//    }
//    /*修正液位*/
//    else if(p_coilarray[COIL_CORRECTLEVEL - STARTADDRESS_COIL] != 0)
//    {
//        if((EquipmentState == STATE_LEVELFOLLOWWING)
//            && (result.OilLevel != LEVEL_DOWNLIMIT))
//        {
//            unsigned int tempvalue;
//            tempvalue = systemparameter.YeildCalibration + (LevelCorrection - result.OilLevel);
//            if((tempvalue <= 15)||(tempvalue >= 3000000))
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            else
//            {
//                FlagofCorrectLevel = true;
//                return RETURN_OK;
//            }
//        }
//        else
//        {
//            return RETURN_UNSUPPORTED;
//        }
//    }
//    /*提浮子至零点*/
//    else if(p_coilarray[COIL_BRINGUPDISPLACERTOORIGIN - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DISPLACERRUNUP:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DISPLACERRUNUP;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_PULLDISPLACERTOORIGIN;
//                break;
//            }
//        }
//    }
//    /*读取称重值*/
//    else if(p_coilarray[COIL_GETVALUEOFWEIGHT - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_WEIGHTGETTING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_WEIGHTGETTING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_GET_WEIGHT;
//                break;
//            }
//        }
//    }
//    /*读取步进数*/
//    else if(p_coilarray[COIL_GETVALUEOFMOTORSTEPS - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_MOTORSTEPSGETTING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_MOTORSTEPSGETTING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MOTOR_GETSTEPS;
//                break;
//            }
//        }
//    }
//    /*读取无线温度*/
//    else if(p_coilarray[COIL_GETVALUEOFTEMPERATURE - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_TEMPERATUREGETTING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_TEMPERATUREGETTING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_GET_TEMPERATURE;
//                break;
//            }
//        }
//    }
//    /*恢复出厂设置*/
//    else if(p_coilarray[COIL_RESTOREFACTORYSETTINGS - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_STANDBY:
//            case STATE_FAIL:
//            case STATE_CONFIGMODE:
//            {
//                ret = FactoryDataReset();
//                if(ret != 0)
//                {
//                    return RETURN_EXEFAIL;
//                }
//                else
//                {
//                    EquipmentState = STATE_STANDBY;
//                    EquipmentErrorCode = ERR_NO_ERROR;
//                    return RETURN_OK;
//                }
//            }
//            default:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//        }
//    }
//    /*重置称重阈值*/
//    else if(p_coilarray[COIL_RESETSCOPEOFWEIGHT - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_WEIGHTSCOPERESETTING:
//            {
//                break;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_WEIGHTSCOPERESETTING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_GETWEIGHSCOPE;
//                break;
//            }
//        }
//    }
//    /*恢复配置文件*/
//    else if(p_coilarray[COIL_RESTORECONFIGURATIONFILE - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_STANDBY:
//            case STATE_FAIL:
//            case STATE_CONFIGMODE:
//            {
//                ret = ConfigurationFileReset();
//                if(ret != 0)
//                {
//                    return RETURN_EXEFAIL;
//                }
//                else
//                {
//                    EquipmentState = STATE_STANDBY;
//                    EquipmentErrorCode = ERR_NO_ERROR;
//                    return RETURN_OK;
//                }
//            }
//            default:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//        }
//    }
//    /*密度温度测量*/
//    else if(p_coilarray[COIL_MEASUREDENSITYANDTEMPERATURE - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DENSITYTEMPERATUREMEASURING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_AUTO;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYTEMPERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITYANDTEMP_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_AUTO;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYTEMPERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITYANDTEMP_START;
//                break;
//            }
//        }
//    }
//    /*指定高度密度温度测量*/
//    else if(p_coilarray[COIL_MEASUREDENSITYANDTEMPERATURE_ASSIGNHEIGHT - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DENSITYTEMPERATUREMEASURING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_MANUAL;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYTEMPERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITYANDTEMP_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_MANUAL;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYTEMPERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITYANDTEMP_START;
//                break;
//            }
//        }
//    }
//    /*密度校正*/
//    else if(p_coilarray[COIL_CALIBRATIONDENSITY - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DENSITYCALIBRATION:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYCALIBRATION;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_DENSITYCALIBRATION_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYCALIBRATION;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_DENSITYCALIBRATION_START;
//                break;
//            }
//        }
//    }
//    /*密度测量*/
//    else if(p_coilarray[COIL_MEASUREDENSITY - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DENSITYMEASURING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_AUTO;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITY_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_AUTO;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITY_START;
//                break;
//            }
//        }
//    }
//    /*指定高度密度测量*/
//    else if(p_coilarray[COIL_MEASUREDENSITY_ASSIGNHEIGHT - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DENSITYMEASURING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_MANUAL;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITY_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_MANUAL;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITYMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREDENSITY_START;
//                break;
//            }
//        }
//    }
//    /*水位测量*/
//    else if(p_coilarray[COIL_MEASUREWATER - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_WATERMEASUREING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_WATERMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREWATER_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_WATERMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MEASUREWATER_START;
//                break;
//            }
//        }
//    }
//    /*仪表配置模式*/
//    else if(p_coilarray[COIL_CONFIG - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_CONFIGMODE:
//            {
//                break;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_CONFIGMODE;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_ENTERCONFIGMODE;
//                break;
//            }
//        }
//    }
//    /*设置空载阈值*/
//    else if(p_coilarray[COIL_SETNOLOAD - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_SETNOLOADING:
//            {
//                break;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_SETNOLOADING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_SETNOLOAD;
//                break;
//            }
//        }
//    }
//    /*设置满载阈值*/
//    else if(p_coilarray[COIL_SETFULLLOAD - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_SETFULLLOADING:
//            {
//                break;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_SETFULLLOADING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_SETFULLLOAD;
//                break;
//            }
//        }
//    }
//    /*上行*/
//    else if(p_coilarray[COIL_MOTORRUNUP - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_MOTORRUNUP:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_MOTORRUNUP;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MOTOR_RUNUP;
//                break;
//            }
//        }
//    }
//    /*下行*/
//    else if(p_coilarray[COIL_MOTORRUNDOWN - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_MOTORRUNDOWN:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_MOTORRUNDOWN;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_MOTOR_RUNDOWN;
//                break;
//            }
//        }
//    }
//    /*罐底测量*/
//    else if(p_coilarray[COIL_MEASUREBOTTOM - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_BOTTOMTESTING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_BOTTOMTESTING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_BOTTOMTEST;
//                break;
//            }
//        }
//    }
//    /*国标测量*/
//    else if(p_coilarray[COIL_DENSITY_NATION - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DENSITY_NATION_ERATUREMEASURING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_NATIOM;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITY_NATION_ERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_DESITY_NATION_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_NATIOM;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DENSITY_NATION_ERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_DESITY_NATION_START;
//                break;
//            }
//        }
//    }
//    /*综合测量*/
//    else if(p_coilarray[COIL_RESERCOIL_INTEGRATEDVE6 - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_INTEGRATED_ERATUREMEASURING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_INTEGRATED_ERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_INTEGRATED_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_INTEGRATED_ERATUREMEASURING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_INTEGRATED_START;
//                break;
//            }
//        }
//    }
//    /***************系统重启*******************************************************/
//    else if(p_coilarray[COIL_SYSTEMRESTART - STARTADDRESS_COIL] != 0)
//    {
//        SCB->AIRCR =0X05FA0000|(u32)0x04;
//    }
//    /*区间测量*/
//    else if(p_coilarray[COIL_INTERVALMEASURE - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_INTERVALMEASUREING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_INTERVAL;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_INTERVALMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_INTERVALMEASURE;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_INTERVAL;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_INTERVALMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_INTERVALMEASURE;
//                break;
//            }
//        }
//    }
//    /*每米测量*/
//    else if(p_coilarray[COIL_PERMETERMEASURE - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_PERMETERMEASUREING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                TypeofDensityMode = DENSITYMEASURE_PERMETER;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_PERMETERMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_PERMETERMEASURE;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                TypeofDensityMode = DENSITYMEASURE_PERMETER;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_PERMETERMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_PERMETERMEASURE;
//                break;
//            }
//        }
//    }
//    /*设备自检*/
//    else if(p_coilarray[COIL_SELFINSPECTION - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_SELFINSPECTION_ORIGIN:
//            case STATE_SELFINSPECTION_OILLEVEL:
//            case STATE_SELFINSPECTION_BOTTOM:
//            case STATE_SELFINSPECTION_WATER:
//            case STATE_SELFINSPECTION_DENSITY:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_SELFINSPECTION_ORIGIN;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_SELFINSPECTION;
//                break;
//            }
//        }
//    }
//    /*读实时温度*/
//    else if(p_coilarray[COIL_REDAREALTMP - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_REALTIMETEMPERATURE:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_REALTIMETEMPERATURE;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_REALTIMETEMPERATURE;
//                break;
//            }
//        }
//    }
//    /*无拟合找液位*/
//    else if(p_coilarray[COIL_NOSINEFINDOIL - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_SELFINSPECTION_NOSINEFITTING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_SELFINSPECTION_NOSINEFITTING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_NOSINEFITTING;
//                break;
//            }
//        }
//    }
//    /*无拟合回零点*/
//    else if(p_coilarray[COIL_NOSINEZERO - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_DISPLACER_SINE:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_DISPLACER_SINE;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_DISPLACER_NOSINE;
//                break;
//            }
//        }
//    }
//    /*标定液位*/
//    else if(p_coilarray[COIL_CALIBRATIONLEVEL - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_CORRECTLEVELING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofCorrectLevel = true;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_CORRECTLEVELING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_CORRECTLEVEL;
//                break;
//            }
//        }
//    }
//    /*电压检测 - 静止*/
//    else if(p_coilarray[COIL_VCHECK_STATIC - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_VOLTAGEMONITOR_STATIC:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_VOLTAGEMONITOR_STATIC;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_VOLTAGEMONITOR_STATIC;
//                break;
//            }
//        }
//    }
//    /*电压检测 - 运动*/
//    else if(p_coilarray[COIL_VCHECK_RUN - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_VOLTAGEMONITOR_MOVE:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_VOLTAGEMONITOR_MOVE;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_VOLTAGEMONITOR_MOVE;
//                break;
//            }
//        }
//    }
//    /*水位跟随*/
//    else if(p_coilarray[COIL_WATERFOLLOWING - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_WATERFOLLOWING:
//            case STATE_WATERMEASUREING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            case STATE_LEVELFOLLOWWING:
//            {
//                FlagofLevelFollowingBefore = true;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_WATERMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_WATERFOLLOWING_START;
//                break;
//            }
//            default:
//            {
//                FlagofLevelFollowingBefore = false;
//                FlagofContinue   = false;
//                EquipmentState   = STATE_WATERMEASUREING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_WATERFOLLOWING_START;
//                break;
//            }
//        }
//    }
//    /*修正水位*/
//    else if(p_coilarray[COIL_CORRECTWATERLEVEL - STARTADDRESS_COIL] != 0)
//    {
//        if((EquipmentState == STATE_WATERFOLLOWING)
//            && (result.WaterValue != LEVEL_DOWNLIMITWATER))
//        {
//            unsigned int tempvalue;
//            tempvalue = systemparameter.WaterTankHeight + (WaterLevelCorrection - result.WaterValue);
//            if((tempvalue <= 15)||(tempvalue >= 3000000))
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            else
//            {
//                FlagofCorrectWaterLevel = true;
//                return RETURN_OK;
//            }
//        }
//        else
//        {
//            return RETURN_UNSUPPORTED;
//        }
//    }
//    /*设备磨合*/
//    else if(p_coilarray[COIL_EQUIPRUNNINGIN - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_RUNNING_IN_ING:
//            {
//                break;
//            }
//            case STATE_WEIGHTSCOPERESETTING:
//            case STATE_SETFULLLOADING:
//            case STATE_SETNOLOADING:
//            {
//                return RETURN_UNSUPPORTED;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_RUNNING_IN_ING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_RUNNING_IN;
//                break;
//            }
//        }
//    }
//    /*检修模式*/
//    else if(p_coilarray[COIL_CHECKMODE - STARTADDRESS_COIL] != 0)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_CHECKMODING:
//            {
//                break;
//            }
//            default:
//            {
//                FlagofContinue   = false;
//                EquipmentState   = STATE_CHECKMODING;
//                FlagofCommand    = true;
//                MeasureCommand   = COMMAND_CHECKMODE;
//                break;
//            }
//        }
//    }
//    else
//    {
//        return RETURN_UNDEFADDRESS;
//    }

	return RETURN_OK;
}

/*重置线圈*/
void ResetCoil(int *p_coilarray, int coilamount) {
	int i;

	for (i = 0; i < coilamount; ++i) {
		p_coilarray[i] = 0;
	}
}

/*更新保持寄存器参数,若需存储则存入铁电*/
int UpdateHoldRegisterParameter(int const *p_holdingregister, bool iswrite) {
	int ret = RETURN_OK;
//    int tmpvalue;
//    int lastvalue;
//    int i;
//    union FolatInt tmpvalue_f;
//
//    if(iswrite == true)
//    {
//        switch(EquipmentState)
//        {
//            case STATE_STANDBY:
//            case STATE_FAIL:
//            case STATE_ORIGINCALIBRATIONOVER:
//            case STATE_WATERMEASUREOVER:
//            case STATE_BOTTOMTESTOVER:
//            case STATE_INTEGRATED_OVER:
//            case STATE_DENSITY_NATION_OVER:
//            case STATE_INTERVALMEASUREOVER:
//            case STATE_PERMETERMEASUREOVER:
//            case STATE_CONFIGMODE:
//            {
//                /*转毂周长*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DRUMCIRCUMFERENCE] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DRUMCIRCUMFERENCE + 1] & 0x0000FFFF);
//                if(tmpvalue <= 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DrumCircle = tmpvalue;
//                /*罐高*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_TANKHEIGHT] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_TANKHEIGHT + 1] & 0x0000FFFF);
//                if(tmpvalue <= 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.YeildCalibration = tmpvalue;
//                /*高液位报警值*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_LEVELOFHIGHALARM] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_LEVELOFHIGHALARM + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DOHighAlarmLevel = tmpvalue;
//                /*低液位报警值*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_LEVELOFLOWALARM] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_LEVELOFLOWALARM + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DOLowAlarmLevel = tmpvalue;
//                /*盲区*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_OFFSETOFDEADZONE] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_OFFSETOFDEADZONE + 1] & 0x0000FFFF);
//                if((tmpvalue - 30000) < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DeadZone = tmpvalue - 30000;
//                /*减速比*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_REDUCTIONRATIO] & 0x0000FFFF;
//                systemparameter.ReductionRatio = tmpvalue;
//                /*4_20mA量程*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_VALUEOFSTRIKINGFORK] & 0x0000FFFF;
//                systemparameter.CurrentRange = tmpvalue;
//                /*浮子重量*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_COMMCONVERTTIME] & 0x0000FFFF;
//                if(tmpvalue > 1000)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DisplacerWeight = tmpvalue;
//                /*最高密度点至液位距离*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_HIGHESTPOINTOFDENSITY] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_HIGHESTPOINTOFDENSITY + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DensityHighestDistance = tmpvalue;
//                /*最低密度点至罐底距离*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_LOWESTPOINTOFDENSITY] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_LOWESTPOINTOFDENSITY + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DensityLowestDistance = tmpvalue;
//                /*密度测量点间距*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DENSITYINTERVAL] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DENSITYINTERVAL + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DensityIntervalDistance = tmpvalue;
//                /*密度测量点数*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_DENSITYSPOTS] & 0x0000FFFF;
//                if(tmpvalue < 0|| tmpvalue > WIRELESSMAXSPOT)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DensitySpotsAmount = tmpvalue;
//                /*实时温度计测量点数*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_THERMOMETERSPOTS] & 0x0000FFFF;
//                if(tmpvalue > REALTEMPMAXSPOT)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.RealTempSpot = tmpvalue;
//                /*实时温度计测量范围*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_THERMOMETERMEASUREDISTANCE] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_THERMOMETERMEASUREDISTANCE + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.RealTempRange = tmpvalue;
//                /*最低温度探头距罐底距离*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_THERMOMETERLOWESTPOSITION] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_THERMOMETERLOWESTPOSITION + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.RealTempLowestDistance = tmpvalue;
//                /*温度探头浸没最小距离*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_THERMOMETERLOWESTIMMERSEDHEIGHT] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_THERMOMETERLOWESTIMMERSEDHEIGHT + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.RealTempImmerMinDistance = tmpvalue;
//                /*标定零点后下行距离*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_STOPTIME] & 0x0000FFFF;
//                if(tmpvalue <= 0 || tmpvalue > 2000)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                lastvalue = systemparameter.DownDistanceAfterOrigin;
//                systemparameter.DownDistanceAfterOrigin = tmpvalue;
//                if(lastvalue != systemparameter.DownDistanceAfterOrigin)
//                {
//                    systemparameter.FlagOfFirstPowerOn = ISFIRSTPOWERON;
//                }
//                /*浮子类型*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_DISPLACERHEIGHT] & 0x0000FFFF;
//                if(tmpvalue >= (FLOATTYPEAMOUNT * 10))
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DisplacerType = tmpvalue;
//                /*上层介质密度*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_CORRECTTHRESHOLD] & 0x0000FFFF;
//                if(tmpvalue <= 0 || tmpvalue > 2000)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.MediumDensity = tmpvalue;
//                /*浮子体积*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DISPLACERVOLUME] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DISPLACERVOLUME + 1] & 0x0000FFFF);
//                if(tmpvalue < 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.DisplacerVolume = tmpvalue * 100;
//                /*温度计类型*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_TYPEOFTHERMOMETER] & 0x0000FFFF;
//                if(tmpvalue >= Temperature_END)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.TempType = tmpvalue;
//                /*水位罐高*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_WATERTANKHEIGHT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_WATERTANKHEIGHT + 1] & 0x0000FFFF);
//                if(tmpvalue <= 0)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.WaterTankHeight = tmpvalue;
//                /*水位盲区*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_WATERDEADZONE] & 0x0000FFFF;
//                if(tmpvalue > systemparameter.WaterTankHeight)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.WaterDeadZone = tmpvalue;
//                /*综合测量*/
//                /*综合测量_密度测量模式*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_DENSITYCHOOSE] & 0x0000FFFF;
//                systemparameter.compre_DensityMode = tmpvalue;
//                /*综合测量_密度测量方向*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_SIGLEDENSITY_CHOOSE] & 0x0000FFFF;
//                if(tmpvalue != MOTOR_DIRECTION_UP && tmpvalue != MOTOR_DIRECTION_DOWN)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.compre_DensityDir = tmpvalue;
//                /*综合测量_是否测量罐底*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_BOTTOM_CHOOSE] & 0x0000FFFF;
//                if(tmpvalue != CONPRE_NO && tmpvalue != CONPRE_YES)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.compre_FlagofMeasureBottom = tmpvalue;
//                /*综合测量_是否测量水位*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_WATERCHOOSE] & 0x0000FFFF;
//                if(tmpvalue != CONPRE_NO && tmpvalue != CONPRE_YES)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.compre_FlagofMeasureWater = tmpvalue;
//                /*综合测量_是否测量发油口*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_OUTOIL_POSI] & 0x0000FFFF;
//                if(tmpvalue != CONPRE_NO && tmpvalue != CONPRE_YES)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.compre_FlagofMeasureHairOil = tmpvalue;
//                /*零点重复性范围*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_UNSTABLEVALUE] & 0x0000FFFF;
//                systemparameter.ScopeOriginRepeat = tmpvalue;
//                /*测量出的罐高*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_FIRSTWEIGHTCHKVALUE] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_FIRSTWEIGHTCHKVALUE + 1] & 0x0000FFFF);
//                systemparameter.MeasuredTankHeight = tmpvalue;
//                /*罐高变化范围*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_TANKHEIGHTSCOPE] & 0x0000FFFF;
//                systemparameter.TankHeightScope = tmpvalue;
//                /*是否更新罐高*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_SWITCHUPDATETANK] & 0x0000FFFF;
//                systemparameter.Switch_Bottom = tmpvalue;
//                /*区间测量点数*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_RESERVED6] & 0x0000FFFF;
//                if(tmpvalue > 100)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.Interval_Spots = tmpvalue;
//                /*AO高报*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_AOH] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_AOH + 1] & 0x0000FFFF);
//                systemparameter.AOHighAlarmLevel = tmpvalue;
//                /*AO低报*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_AOL] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_AOL + 1] & 0x0000FFFF);
//                systemparameter.AOLowAlarmLevel = tmpvalue;
//                /*从液面上行到空气距离*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_DETERMINEDISTANCE] & 0x0000FFFF;
//                systemparameter.DetermineDistance = tmpvalue;
//                /*称重检测上限比例*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_WEIGHTCHECKMAXRATIO] & 0x0000FFFF;
//                if(tmpvalue <= 0 || tmpvalue >= 100)
//                    return RETURN_UNSUPPORTED;
//                systemparameter.WeightCheckMaxRatio = tmpvalue;
//                /*称重检测下限比例*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_WEIGHTCHECKMINRATIO] & 0x0000FFFF;
//                if(tmpvalue <= 0 || tmpvalue >= 100)
//                    return RETURN_UNSUPPORTED;
//                systemparameter.WeightCheckMinRatio = tmpvalue;
//                /*磁通量D*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_MAGNETICFLUXD] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_MAGNETICFLUXD + 1] & 0x0000FFFF);
//                systemparameter.MagneticFluxD.i = tmpvalue;
//                /*磁通量T*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_MAGNETICFLUXT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_MAGNETICFLUXT + 1] & 0x0000FFFF);
//                systemparameter.MagneticFluxT.i = tmpvalue;
//                /*设备速度*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SPEED] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SPEED + 1] & 0x0000FFFF);
//                tmpvalue_f.i = tmpvalue;
//                if(tmpvalue_f.f < 0 || tmpvalue_f.f > 7)
//                    return RETURN_UNSUPPORTED;
//                systemparameter.VE.i = tmpvalue;
//                /*空载*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_NOLOAD] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_NOLOAD + 1] & 0x0000FFFF);
//                if(tmpvalue < MINWEIGHT || (systemparameter.WeightofFullLoad - tmpvalue) < 20000)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                lastvalue = systemparameter.WeightofNoLoad;
//                systemparameter.WeightofNoLoad = tmpvalue;
//                if(lastvalue != systemparameter.WeightofNoLoad)
//                {
//                    systemparameter.FlagofGetNoLoad = GETNOLOAD;
//                    //更新称重上限、下限
//                    systemparameter.WeightMaxWhenOrigin = systemparameter.WeightInTheAir + ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * 0.2;
//                    systemparameter.WeightMinWhenOrigin = systemparameter.WeightInTheAir - ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * 0.2;
//                    systemparameter.WeightTooMax = systemparameter.WeightInTheAir + ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * (systemparameter.WeightCheckMaxRatio / 100.0);
//                    systemparameter.WeightTooMin = systemparameter.WeightInTheAir - ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * (systemparameter.WeightCheckMinRatio / 100.0);
//                    if((int)systemparameter.WeightTooMax < MINWEIGHT || (int)systemparameter.WeightTooMax > MAXWEIGHT)
//                        return RETURN_UNSUPPORTED;
//                    if((int)systemparameter.WeightTooMin < MINWEIGHT || (int)systemparameter.WeightTooMin > MAXWEIGHT)
//                        return RETURN_UNSUPPORTED;
//                }
//                /*满载*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_FULLLOAD] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_FULLLOAD + 1] & 0x0000FFFF);
//                if((tmpvalue - (int)systemparameter.WeightofNoLoad) < 20000 || tmpvalue > MAXWEIGHT)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                lastvalue = systemparameter.WeightofFullLoad;
//                systemparameter.WeightofFullLoad = tmpvalue;
//                if(lastvalue != systemparameter.WeightofFullLoad)
//                {
//                    systemparameter.WeightInTheAir = systemparameter.WeightofFullLoad;
//                    systemparameter.FlagofGetFullLoad = GETFULLLOAD;
//                    //更新称重上限、下限
//                    systemparameter.WeightMaxWhenOrigin = systemparameter.WeightInTheAir + ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * 0.2;
//                    systemparameter.WeightMinWhenOrigin = systemparameter.WeightInTheAir - ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * 0.2;
//                    systemparameter.WeightTooMax = systemparameter.WeightInTheAir + ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * (systemparameter.WeightCheckMaxRatio / 100.0);
//                    systemparameter.WeightTooMin = systemparameter.WeightInTheAir - ((int)systemparameter.WeightofFullLoad - (int)systemparameter.WeightofNoLoad) * (systemparameter.WeightCheckMinRatio / 100.0);
//                    if((int)systemparameter.WeightTooMax < MINWEIGHT || (int)systemparameter.WeightTooMax > MAXWEIGHT)
//                        return RETURN_UNSUPPORTED;
//                    if((int)systemparameter.WeightTooMin < MINWEIGHT || (int)systemparameter.WeightTooMin > MAXWEIGHT)
//                        return RETURN_UNSUPPORTED;
//                }
//                /*传感器系数K0*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K0] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K0 + 1] & 0x0000FFFF);
//                systemparameter.K0.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K0 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K0 + 3] & 0x0000FFFF);
//                systemparameter.K0.u[0] = tmpvalue;
//                /*传感器系数K1*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K1] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K1 + 1] & 0x0000FFFF);
//                systemparameter.K1.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K1 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K1 + 3] & 0x0000FFFF);
//                systemparameter.K1.u[0] = tmpvalue;
//                /*传感器系数K2*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K2 + 1] & 0x0000FFFF);
//                systemparameter.K2.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K2 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K2 + 3] & 0x0000FFFF);
//                systemparameter.K2.u[0] = tmpvalue;
//                /*传感器系数K3*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K3] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K3 + 1] & 0x0000FFFF);
//                systemparameter.K3.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K3 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K3 + 3] & 0x0000FFFF);
//                systemparameter.K3.u[0] = tmpvalue;
//                /*传感器系数K4*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K4] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K4 + 1] & 0x0000FFFF);
//                systemparameter.K4.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K4 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K4 + 3] & 0x0000FFFF);
//                systemparameter.K4.u[0] = tmpvalue;
//                /*传感器系数K5*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K5] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K5 + 1] & 0x0000FFFF);
//                systemparameter.K5.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K5 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K5 + 3] & 0x0000FFFF);
//                systemparameter.K5.u[0] = tmpvalue;
//                /*传感器系数K18*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K18] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K18 + 1] & 0x0000FFFF);
//                systemparameter.K18.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K18 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K18 + 3] & 0x0000FFFF);
//                systemparameter.K18.u[0] = tmpvalue;
//                /*传感器系数K19*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K19] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K19 + 1] & 0x0000FFFF);
//                systemparameter.K19.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_K19 + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_K19 + 3] & 0x0000FFFF);
//                systemparameter.K19.u[0] = tmpvalue;
//                /*传感器系数DeviceNum*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_DEVICENUM] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_DEVICENUM + 1] & 0x0000FFFF);
//                systemparameter.DeviceNum.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_DEVICENUM + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_DEVICENUM + 3] & 0x0000FFFF);
//                systemparameter.DeviceNum.u[0] = tmpvalue;
//                /*传感器系数detT*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_DETT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_DETT + 1] & 0x0000FFFF);
//                systemparameter.detT.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_SENSOR_DETT + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SENSOR_DETT + 3] & 0x0000FFFF);
//                systemparameter.detT.u[0] = tmpvalue;
//                /*油品类型*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_CONVER_OILTYPE] & 0x0000FFFF;
//                systemparameter.OilType = tmpvalue;
//                /*油罐类型*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_CONVER_TANKTYPE] & 0x0000FFFF;
//                systemparameter.TankType = tmpvalue;
//                /*油罐是否保温*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_CONVER_TANKINSULATED] & 0x0000FFFF;
//                systemparameter.TankInsulated = tmpvalue;
//                /*热膨胀系数*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_TANKWALLEXPANSIONCOEF] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_TANKWALLEXPANSIONCOEF + 1] & 0x0000FFFF);
//                systemparameter.TankWallExpansionCoefficient.u[1] = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_TANKWALLEXPANSIONCOEF + 2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_TANKWALLEXPANSIONCOEF + 3] & 0x0000FFFF);
//                systemparameter.TankWallExpansionCoefficient.u[0] = tmpvalue;
//                /*浮顶罐_浮盘重量*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_PLATEWEIGHT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_PLATEWEIGHT + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_FloatingPlateWeight = tmpvalue;
//                /*浮顶罐_支撑高度*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_SUPPORTHEIGHT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_SUPPORTHEIGHT + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_SupportHeight = tmpvalue;
//                /*浮顶罐_起浮高度*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_FLOATHEIGHT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_FLOATHEIGHT + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_FloatingHeight = tmpvalue;
//                /*浮顶罐_不计量区间H1*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_H1] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_H1 + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_UnmeasuredIntervalH1 = tmpvalue;
//                /*浮顶罐_不计量区间L1*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_L1] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_L1 + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_UnmeasuredIntervalL1 = tmpvalue;
//                /*浮顶罐_不计量区间H2*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_H2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_H2 + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_UnmeasuredIntervalH2 = tmpvalue;
//                /*浮顶罐_不计量区间L2*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_L2] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_FLOATROOF_L2 + 1] & 0x0000FFFF);
//                systemparameter.FloatRoofTank_UnmeasuredIntervalL2 = tmpvalue;
//                /*含水率*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_MOISTURECONTENT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_MOISTURECONTENT + 1] & 0x0000FFFF);
//                systemparameter.MoistureContent.i = tmpvalue;
//                /*球罐_球面半径*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_SPHERICALTANK_SPHERICALRADIUS] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_SPHERICALTANK_SPHERICALRADIUS + 1] & 0x0000FFFF);
//                systemparameter.SphericalTank_SphericalRadius = tmpvalue;
//                /*球罐_校正高度*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_SPHERICALTANK_CORRECTIONHEIGHT] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_SPHERICALTANK_CORRECTIONHEIGHT + 1] & 0x0000FFFF);
//                systemparameter.SphericalTank_ZeroCorrectionHeight = tmpvalue;
//                /*球罐_罐总体积*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_CONVER_SPHERICALTANK_TOTALVOLUMETANK] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_CONVER_SPHERICALTANK_TOTALVOLUMETANK + 1] & 0x0000FFFF);
//                systemparameter.SphericalTank_TotalVolumeofTank = tmpvalue;
//                /*液位分段修正*/
//                for(i=0;i<SUBSECTION_OIL_AMOUNT;i++)
//                {
//                    tmpvalue = (p_holdingregister[HOLDREGISTER_SUBSECTION_OIL_POSI_1 + i * 3] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SUBSECTION_OIL_POSI_1 + 1 + i * 3] & 0x0000FFFF);
//                    systemparameter.Subsection_oil_Posi[i] = tmpvalue;
//                    tmpvalue = p_holdingregister[HOLDREGISTER_SUBSECTION_OIL_POSI_1 + 2 + i * 3] & 0x0000FFFF;
//                    systemparameter.Subsection_oil_value[i] = tmpvalue - 10000;
//                }
//                /*水位分段修正*/
//                for(i=0;i<SUBSETTION_WATER_AMOUNT;i++)
//                {
//                    tmpvalue = (p_holdingregister[HOLDREGISTER_SUBSECTION_WATER_POSI_1 + i * 3] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_SUBSECTION_WATER_POSI_1 + 1 + i * 3] & 0x0000FFFF);
//                    systemparameter.Subsection_water_Posi[i] = tmpvalue;
//                    tmpvalue = p_holdingregister[HOLDREGISTER_SUBSECTION_WATER_POSI_1 + 2 + i * 3] & 0x0000FFFF;
//                    systemparameter.Subsection_water_value[i] = tmpvalue - 10000;
//                }
//                /*管道直径*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_PIPEDIAMETER] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_PIPEDIAMETER + 1] & 0x0000FFFF);
//                systemparameter.pipeDiameter.i = tmpvalue;
//                /*管道流速采样速率*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_PIPESAMPLERATIO] & 0x0000FFFF;
//                systemparameter.pipeSampleRatio = tmpvalue;
//                /*钢丝直径*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_WIERDIAMETER] & 0x0000FFFF;
//                if(tmpvalue != WIREDIAMETER_20 && tmpvalue != WIREDIAMETER_15)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.wireDiameter = tmpvalue;
//                /*电流相关参数*/
//                tmpvalue = (p_holdingregister[HOLDREGISTER_4_20RANGESTART] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_4_20RANGESTART + 1] & 0x0000FFFF);
//                systemparameter.CurrentRange_start = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_4_20_INIT] & 0x0000FFFF;
//                systemparameter.CURRENT_INIT = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_4_20_AOH] & 0x0000FFFF;
//                systemparameter.CURRENT_GREATERAOH = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_4_20_AOL] & 0x0000FFFF;
//                systemparameter.CURRENT_UNDERAOL = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_4_20_ERR] & 0x0000FFFF;
//                systemparameter.CURRENT_MEASUREERROR = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_4_20_DEBUG] & 0x0000FFFF;
//                systemparameter.CURRENT_CONFIGMODE = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DOSTATE] & 0x0000FFFF;
//                systemparameter.DO_state = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_POWERONTODO] & 0x0000FFFF;
//                systemparameter.poweronTodo = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_1PDELAY] & 0x0000FFFF;
//                systemparameter.onepointdelay = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SINE] & 0x0000FFFF;
//                systemparameter.switch_Sine0use = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DECIMALPLACES] & 0x0000FFFF;
//                systemparameter.decimal = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_PASSWARD] & 0x0000FFFF;
//                systemparameter.password = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SOURCE_OIL] & 0x0000FFFF;
//                systemparameter.source_oil = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SOURCE_WATER] & 0x0000FFFF;
//                systemparameter.source_water = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SOURCE_DENSITY] & 0x0000FFFF;
//                systemparameter.source_d = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SOURCE_TEMP] & 0x0000FFFF;
//                systemparameter.source_t = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_INPUTVAL_OIL] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_INPUTVAL_OIL + 1] & 0x0000FFFF);
//                systemparameter.inputval_oil = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_INPUTVAL_WATER] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_INPUTVAL_WATER + 1] & 0x0000FFFF);
//                systemparameter.inputval_water = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_INPUTVAL_D] & 0x0000FFFF;
//                systemparameter.inputval_d = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_INPUTVAL_T] & 0x0000FFFF;
//                systemparameter.inputval_t = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SELF_D] & 0x0000FFFF;
//                systemparameter.switch_selfcheck_density = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SELF_WATER] & 0x0000FFFF;
//                systemparameter.switch_selfcheck_water = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SELF_BOTTOM] & 0x0000FFFF;
//                systemparameter.switch_selfcheck_bottom = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_PRESSURE_BOTADD] & 0x0000FFFF;
//                systemparameter.Presure1ShortAdd = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_PRESSURE_AIRADD] & 0x0000FFFF;
//                systemparameter.Presure2ShortAdd = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_PRESSURE_BOTSWITCH] & 0x0000FFFF;
//                systemparameter.Presure1Switch = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_PRESSURE_AIRSWITCH] & 0x0000FFFF;
//                systemparameter.Presure2Switch = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_CHEMICAL_WAY] & 0x0000FFFF;
//                systemparameter.chemical_way = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_CHEMICAL_VALUE] & 0x0000FFFF;
//                systemparameter.Chemical_density20 = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_MIX_SWITCH] & 0x0000FFFF;
//                systemparameter.Mix_Switch = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_MIX_INPUTD] & 0x0000FFFF;
//                systemparameter.Mix_Density = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_MIX_BOTTOMDIS] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_MIX_BOTTOMDIS + 1] & 0x0000FFFF);
//                systemparameter.Mix_P1distance = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_MIX_AIRDIS] << 16)
//                                        + (p_holdingregister[HOLDREGISTER_MIX_AIRDIS + 1] & 0x0000FFFF);
//                systemparameter.Mix_P3distance = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_MIX_G] & 0x0000FFFF;
//                systemparameter.Mix_g = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_MIX_STEAMD] & 0x0000FFFF;
//                systemparameter.Mix_Dsteam = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_MIX_AIRD] & 0x0000FFFF;
//                systemparameter.Mix_Dair = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_LANGUAGE] & 0x0000FFFF;
//                systemparameter.language = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DENSITYINPUTVALUE_SWITCH] & 0x0000FFFF;
//                systemparameter.switch_density = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SWITCH_LOWPOSI] & 0x0000FFFF;
//                systemparameter.switch_lowposi = tmpvalue;
//                /*下层介质密度*/
//                tmpvalue = p_holdingregister[HOLDREGISTER_LOWERMEDIUMDENSITY] & 0x0000FFFF;
//                if(tmpvalue <= 100 || tmpvalue > 2000)
//                {
//                    return RETURN_UNSUPPORTED;
//                }
//                systemparameter.LowerMediumDensity = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_LAGDOMAIN] & 0x0000FFFF;
//                systemparameter.LagDomain_distance = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SWITCH_INITCURRENT] & 0x0000FFFF;
//                systemparameter.switch_initcurrent = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_WATERTHRESHOLD] & 0x0000FFFF;
//                systemparameter.probelength = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SWITCH_BTMFLWING] & 0x0000FFFF;
//                systemparameter.switch_btmflwing = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_SWITCH_DOWNFLWING] & 0x0000FFFF;
//                systemparameter.switch_downflwing = tmpvalue;
//
//                tmpvalue = p_holdingregister[HOLDREGISTER_DRUM_TOTALLAYERS] & 0x0000FFFF;
//                systemparameter.drum_totallayers = tmpvalue;
//
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_2] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_2 + 1] & 0x0000FFFF);
//                systemparameter.drum_circle_2 = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_3] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_3 + 1] & 0x0000FFFF);
//                systemparameter.drum_circle_3 = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_4] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_4 + 1] & 0x0000FFFF);
//                systemparameter.drum_circle_4 = tmpvalue;
//                tmpvalue = (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_5] << 16)
//                                      + (p_holdingregister[HOLDREGISTER_DRUM_CIRCLE_5 + 1] & 0x0000FFFF);
//                systemparameter.drum_circle_5 = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DRUM_TURNSNUM_1] & 0x0000FFFF;
//                systemparameter.drum_turnsnum_1 = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DRUM_TURNSNUM_2] & 0x0000FFFF;
//                systemparameter.drum_turnsnum_2 = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DRUM_TURNSNUM_3] & 0x0000FFFF;
//                systemparameter.drum_turnsnum_3 = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DRUM_TURNSNUM_4] & 0x0000FFFF;
//                systemparameter.drum_turnsnum_4 = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_DRUM_TURNSNUM_5] & 0x0000FFFF;
//                systemparameter.drum_turnsnum_5 = tmpvalue;
//
//                tmpvalue = p_holdingregister[HOLDREGISTER_MOTOR_R] & 0x0000FFFF;
//                systemparameter.motor_r = tmpvalue;
//                tmpvalue = p_holdingregister[HOLDREGISTER_MOTOR_VREF] & 0x0000FFFF;
//                systemparameter.motor_vref = tmpvalue;
//
//                //更新实时温度点位置计算
//                TempPositionCalculate();
//                //刷新综合测量单次参数为存储值
//                Conpre_RefreshPara();
//
//                /*全部参数存入铁电*/
//                if(SystemParameterWriteAll() != 0)
//                {
//                    return RETURN_EXEFAIL;
//                }
//                break;
//            }
//            default:
//            {
//                ret = RETURN_UNSUPPORTED;
//                break;
//            }
//        }
//    }
//    else
//    {
//        /*液位修正值*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_VALUEOFLEVELCALIBRATION] << 16)
//                                + (p_holdingregister[HOLDREGISTER_VALUEOFLEVELCALIBRATION + 1] & 0x0000FFFF);
//        LevelCorrection = tmpvalue;
//        /*水位修正值*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_WATERCALIBRATIONVALUE] << 16)
//                                + (p_holdingregister[HOLDREGISTER_WATERCALIBRATIONVALUE + 1] & 0x0000FFFF);
//
//        WaterLevelCorrection = tmpvalue;
//
//        /*密度温度测量指定高度*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_ASSIGNHEIGHTOFDENSITY] << 16)
//                                + (p_holdingregister[HOLDREGISTER_ASSIGNHEIGHTOFDENSITY + 1] & 0x0000FFFF);
//        if(tmpvalue < systemparameter.DeadZone)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        DensitySpecifiedHeight = tmpvalue;
//        /*标定密度的高度参数值*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_HEIGHTOFDENSITYCORRECTION] << 16)
//                                + (p_holdingregister[HOLDREGISTER_HEIGHTOFDENSITYCORRECTION + 1] & 0x0000FFFF);
//        if(tmpvalue < systemparameter.DeadZone)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        DensityCalHeight = tmpvalue;
//
//        /*标定密度的密度参数值*/
//        tmpvalue = p_holdingregister[HOLDREGISTER_VALUEOFDENSITYCORRECTION] & 0x0000FFFF;
//        if(tmpvalue < 4000 || tmpvalue > 15000)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        DensityCalValue = tmpvalue;
//
//        /*浮子运行距离*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_DISTANCEOFMOTORRUN] << 16)
//                                + (p_holdingregister[HOLDREGISTER_DISTANCEOFMOTORRUN + 1] & 0x0000FFFF);
//        if(tmpvalue < 0)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        DistanceofMotorRun = tmpvalue;
//
//        /*综合测量_单次*/
//        /*综合测量_密度测量模式_单次*/
//        tmpvalue = p_holdingregister[HOLDREGISTER_DENSITYCHOOSE_N] & 0x0000FFFF;
//        if(tmpvalue != DENSITYMEASURE_AUTO && tmpvalue != DENSITYMEASURE_NATIOM && tmpvalue != DENSITYMEASURE_PERMETER)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        ConpreMea.densitymode = tmpvalue;
//        /*综合测量_密度测量方向_单次*/
//        tmpvalue = p_holdingregister[HOLDREGISTER_SIGLEDENSITY_CHOOSE_N] & 0x0000FFFF;
//        if(tmpvalue != MOTOR_DIRECTION_UP && tmpvalue != MOTOR_DIRECTION_DOWN)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        ConpreMea.densitydir = tmpvalue;
//        /*综合测量_是否测量罐底_单次*/
//        tmpvalue = p_holdingregister[HOLDREGISTER_BOTTOM_CHOOSE_N] & 0x0000FFFF;
//        if(tmpvalue != CONPRE_NO && tmpvalue != CONPRE_YES)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        ConpreMea.flag_bottom = tmpvalue;
//        /*综合测量_是否测量水位_单次*/
//        tmpvalue = p_holdingregister[HOLDREGISTER_WATERCHOOSE_N] & 0x0000FFFF;
//        if(tmpvalue != CONPRE_NO && tmpvalue != CONPRE_YES)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        ConpreMea.flag_water = tmpvalue;
//        /*综合测量_是否测量发油口_单次*/
//        tmpvalue = p_holdingregister[HOLDREGISTER_DENSITYNUM] & 0x0000FFFF;
//        if(tmpvalue != CONPRE_NO && tmpvalue != CONPRE_YES)
//        {
//            return RETURN_UNSUPPORTED;
//        }
//        ConpreMea.flag_hairoil = tmpvalue;
//        /*区间A*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_INTERVAL_A] << 16)
//                                + (p_holdingregister[HOLDREGISTER_INTERVAL_A + 1] & 0x0000FFFF);
//        Interval_OillevelA = tmpvalue;
//        /*区间B*/
//        tmpvalue = (p_holdingregister[HOLDREGISTER_INTERVAL_B] << 16)
//                                + (p_holdingregister[HOLDREGISTER_INTERVAL_B + 1] & 0x0000FFFF);
//        Interval_OillevelB = tmpvalue;
//
//        tmpvalue = (p_holdingregister[HOLDREGISTER_INPUT_P1] << 16)
//                                + (p_holdingregister[HOLDREGISTER_INPUT_P1 + 1] & 0x0000FFFF);
//        result.PressureValue = tmpvalue;
//        tmpvalue = (p_holdingregister[HOLDREGISTER_INPUT_P2] << 16)
//                                + (p_holdingregister[HOLDREGISTER_INPUT_P2 + 1] & 0x0000FFFF);
//        result.Pressure2Value = tmpvalue;
//
//
//
//    }

	return ret;
}

