/*
 * system_parameter.c
 *  CPU2参数读取与管理
 *  Created on: Feb 27, 2025
 *      Author: 1
 */
#include "system_parameter.h"
#include "display_tankopera.h"
#include "cpu2_communicate.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
int cnt_commutoCPU2 = COMMU_ERROR_MAX;
volatile MeasurementResult g_measurement = { 0 }; // 测量结果
volatile DeviceParameters g_deviceParams = { 0 }; // 设备参数

//参数元数据
struct ParameterMetadata param_meta[] = {
	/* 名称	数值	数据号	起始地址	寄存器数	是否检范围	最小	最大	单位	小数	偏移	写权	类型	显示	隐藏	英文 */
	{(uint8_t*)"设备指令",	0,	COM_NUM_DEVICEPARAM_COMMAND,	HOLDREGISTER_DEVICEPARAM_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	8,	NULL,	(uint8_t*)"Cmd"},

	/* 基础参数 */
	{(uint8_t*)"罐高",	0,	COM_NUM_DEVICEPARAM_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_TANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TankHeight"},
	{(uint8_t*)"液位盲区",	0,	COM_NUM_DEVICEPARAM_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_BLINDZONE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"BlindZone"},
	{(uint8_t*)"水位盲区",	0,	COM_NUM_DEVICEPARAM_WATER_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterBlind"},
	{(uint8_t*)"编码轮周长mm",	0,	COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"EncWheelCirc"},

	/* 电机与机械参数 */
	{(uint8_t*)"电机最大速度",	0,	COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED,	HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED,	2,	false,	0,	0,	NULL,	2,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"MaxMotorSpd"},
	{(uint8_t*)"首圈周长mm",	0,	COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FirstLoop"},
	{(uint8_t*)"尺带厚度mm",	0,	COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM,	HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"TapeThick"},

	/* 传感器信息 */
	{(uint8_t*)"传感器类型",	0,	COM_NUM_DEVICEPARAM_SENSORTYPE,	HOLDREGISTER_DEVICEPARAM_SENSORTYPE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	3,	NULL,	(uint8_t*)"SensorType"},
	{(uint8_t*)"传感器编号",	0,	COM_NUM_DEVICEPARAM_SENSORID,	HOLDREGISTER_DEVICEPARAM_SENSORID,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"SensorID"},
	{(uint8_t*)"传感器软件版本",	0,	COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"SenSWVer"},

	/* 软件信息 */
	{(uint8_t*)"软件版本",	0,	COM_NUM_DEVICEPARAM_SOFTWAREVERSION,	HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"FWVersion"},
	{(uint8_t*)"上电默认指令",	0,	COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,	HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"PwrOnCmd"},

	/* 称重参数 */
	{(uint8_t*)"空载重量",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyWt"},
	{(uint8_t*)"满载重量",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullWt"},
	{(uint8_t*)"上限比率",	0,	COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,	HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,	2,	false,	0,	0,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"UpperRatio"},
	{(uint8_t*)"下限比率",	0,	COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,	HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,	2,	false,	0,	0,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"LowerRatio"},
	{(uint8_t*)"空载重量上限",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyWtHi"},
	{(uint8_t*)"空载重量下限",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyWtLo"},
	{(uint8_t*)"满载重量上限",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullWtHi"},
	{(uint8_t*)"满载重量下限",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullWtLo"},
	{(uint8_t*)"找零下行距离",	0,	COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,	HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"FindZeroDown"},

	/* 指令参数 */
	{(uint8_t*)"标定液位(油)",	0,	COM_NUM_CAL_OIL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalOilLvl"},
	{(uint8_t*)"修正液位(油)",	0,	COM_NUM_CORRECTION_OIL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CorrectLvl"},
	{(uint8_t*)"标定液位(水)",	0,	COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalWaterLvl"},
	{(uint8_t*)"单点测量位置",	0,	COM_NUM_SINGLE_POINT,	HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MeasPos"},
	{(uint8_t*)"单点监测位置",	0,	COM_NUM_SP_TEST,	HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MonPos"},
	{(uint8_t*)"电机上行距离",	0,	COM_NUM_RUNUP,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"MotorUp"},
	{(uint8_t*)"电机下行距离",	0,	COM_NUM_RUNDOWN,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"MotorDown"},

	/* 修正参数 */
	{(uint8_t*)"密度修正",	0,	COM_NUM_DEVICEPARAM_DENSITYCORRECTION,	HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION,	2,	false,	-10000,	10000,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DensityCorr"},
	{(uint8_t*)"温度修正",	0,	COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION,	HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION,	2,	false,	-1000,	1000,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"TempCorr"},

	/* 分布测量参数 */
	{(uint8_t*)"是否测罐底",	0,	COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,	HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,	2,	false,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"NeedBottom"},
	{(uint8_t*)"是否测水位",	0,	COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT,	HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT,	2,	false,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"NeedWater"},
	{(uint8_t*)"是否测单点密度",	0,	COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,	HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,	2,	false,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"NeedSingleDens"},
	{(uint8_t*)"分布测顺序",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SpreadOrder"},
	{(uint8_t*)"分布测模式",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SpreadMode"},
	{(uint8_t*)"分布测点数",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT,	2,	false,	1,	99,	NULL,	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"SpreadCount"},
	{(uint8_t*)"分布点间距",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SpreadDist"},
	{(uint8_t*)"顶点距液面",	0,	COM_NUM_DEVICEPARAM_SPREADTOPLIMIT,	HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"Top2Level"},
	{(uint8_t*)"底点距罐底",	0,	COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT,	HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"Bot2Tank"},
	{(uint8_t*)"分布点悬停(s)",	0,	COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,	HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,	2,	false,	0,	600,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"PtHoverTime"},

	/* 瓦锡兰密度区间测量 */
	{(uint8_t*)"密度点上限",	0,	COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"WUpperDen"},
	{(uint8_t*)"密度点下限",	0,	COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"WLowerDen"},
	{(uint8_t*)"密度点间隔",	0,	COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"WDenStep"},
	{(uint8_t*)"最高点与液面差",	0,	COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WMaxHeight"},

	/* 水位测量 */
	{(uint8_t*)"水位修正值",	0,	COM_NUM_DEVICEPARAM_WATERLEVELCORRECTION,	HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION,	2,	false,	-1000,	1000,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"WaterCorr"},
	{(uint8_t*)"最大下行距离",	0,	COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE,	HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"MaxDown"},

	/* 实高测量 */
	{(uint8_t*)"更新罐高标志",	0,	COM_NUM_DEVICEPARAM_REFRESHTANKHEIGHTFLAG,	HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG,	2,	false,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"UpdTankHeight"},
	{(uint8_t*)"实高最大偏差",	0,	COM_NUM_DEVICEPARAM_MAXTANKHEIGHTDEVIATION,	HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"MaxDev"},
	{(uint8_t*)"初始罐高",	0,	COM_NUM_DEVICEPARAM_INITIALTANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"InitTankH"},
	{(uint8_t*)"当前罐高",	0,	COM_NUM_DEVICEPARAM_CURRENTTANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"CurrTankH"},

	/* 液位测量 */
	{(uint8_t*)"找油阈值",	0,	COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD,	HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"OilLvlTh"},
	{(uint8_t*)"滞后阈值",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"HysTh"},
	{(uint8_t*)"液位测量方式",	0,	COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,	HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"LvlMeasMode"},

	/* 报警（DO） */
	{(uint8_t*)"高液位报警(DO)",	0,	COM_NUM_DEVICEPARAM_ALARM_HIGH_DO,	HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_DO,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"AlarmHiDO"},
	{(uint8_t*)"低液位报警(DO)",	0,	COM_NUM_DEVICEPARAM_ALARM_LOW_DO,	HOLDREGISTER_DEVICEPARAM_ALARM_LOW_DO,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"AlarmLoDO"},
	{(uint8_t*)"第三状态阈值",	0,	COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_THIRD_STATE_THRESHOLD,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ThirdStateTh"},

	/* 4–20mA AO */
	{(uint8_t*)"输出范围起点mA",	0,	COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA,	HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_START_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"RangeStart"},
	{(uint8_t*)"输出范围终点mA",	0,	COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA,	HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_END_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"RangeEnd"},
	{(uint8_t*)"高限报警(AO)",	0,	COM_NUM_DEVICEPARAM_ALARM_HIGH_AO,	HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_AO,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AlarmHiAO"},
	{(uint8_t*)"低限报警(AO)",	0,	COM_NUM_DEVICEPARAM_ALARM_LOW_AO,	HOLDREGISTER_DEVICEPARAM_ALARM_LOW_AO,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AlarmLoAO"},
	{(uint8_t*)"初始电流mA",	0,	COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_INITIAL_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"InitCurrent"},
	{(uint8_t*)"高位电流mA",	0,	COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_AO_HIGH_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"HighCurrent"},
	{(uint8_t*)"低位电流mA",	0,	COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_AO_LOW_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"LowCurrent"},
	{(uint8_t*)"故障电流mA",	0,	COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_FAULT_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FaultCurrent"},
	{(uint8_t*)"调试电流mA",	0,	COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DebugCurrent"},

	/* 结构信息 */
	{(uint8_t*)"参数版本号",	0,	COM_NUM_DEVICEPARAM_PARAM_VERSION,	HOLDREGISTER_DEVICEPARAM_PARAM_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"ParamVer"},
	{(uint8_t*)"结构体大小",	0,	COM_NUM_DEVICEPARAM_STRUCT_SIZE,	HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"StructSize"},
	{(uint8_t*)"魔术字",	0,	COM_NUM_DEVICEPARAM_MAGIC,	HOLDREGISTER_DEVICEPARAM_MAGIC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Magic"},
	{(uint8_t*)"参数CRC32",	0,	COM_NUM_DEVICEPARAM_CRC,	HOLDREGISTER_DEVICEPARAM_CRC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"ParamCRC"},

	/* ==================== CPU3 本机参数（独立寄存器段 0x0200 起） ==================== */
	{(uint8_t*)"屏幕程序版本",	0,	COM_NUM_PARA_LOCAL_LEDVERSION,	HOLDREGISTER_CPU3_LED_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"LedVer"},
	{(uint8_t*)"语言",	0,	COM_NUM_PARA_LANG,	HOLDREGISTER_CPU3_LANGUAGE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"Lang"},

	{(uint8_t*)"液位数据源",	0,	COM_NUM_SCREEN_SOURCE_OIL,	HOLDREGISTER_CPU3_SRC_OIL,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SrcOil"},
	{(uint8_t*)"水位数据源",	0,	COM_NUM_SCREEN_SOURCE_WATER,	HOLDREGISTER_CPU3_SRC_WATER,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SrcWater"},
	{(uint8_t*)"密度数据源",	0,	COM_NUM_SCREEN_SOURCE_D,	HOLDREGISTER_CPU3_SRC_D,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SrcD"},
	{(uint8_t*)"温度数据源",	0,	COM_NUM_SCREEN_SOURCE_T,	HOLDREGISTER_CPU3_SRC_T,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SrcT"},

	{(uint8_t*)"液位手输值",	0,	COM_NUM_SCREEN_INPUT_OIL,	HOLDREGISTER_CPU3_IN_OIL,	2,	true,	-999999,	999999,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InOil"},
	{(uint8_t*)"水位手输值",	0,	COM_NUM_SCREEN_INPUT_WATER,	HOLDREGISTER_CPU3_IN_WATER,	2,	true,	-999999,	999999,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InWater"},
	{(uint8_t*)"密度手输值",	0,	COM_NUM_SCREEN_INPUT_D,	HOLDREGISTER_CPU3_IN_D,	2,	true,	0,	2000,	(uint8_t*)"kg/m3",	3,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InD"},
	{(uint8_t*)"密度手输上传",	0,	COM_NUM_SCREEN_INPUT_D_SWITCH,	HOLDREGISTER_CPU3_IN_D_SW,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"InDSw"},
	{(uint8_t*)"温度手输值",	0,	COM_NUM_SCREEN_INPUT_T,	HOLDREGISTER_CPU3_IN_T,	2,	true,	-500,	2000,	(uint8_t*)"℃",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InT"},

	{(uint8_t*)"小数点位数",	0,	COM_NUM_SCREEN_DECIMAL,	HOLDREGISTER_CPU3_DECIMAL,	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"Decimal"},
	{(uint8_t*)"屏幕密码",	0,	COM_NUM_SCREEN_PASSWARD,	HOLDREGISTER_CPU3_PASSWORD,	2,	true,	0,	9999,	NULL,	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ScrPwd"},
	{(uint8_t*)"是否息屏",	0,	COM_NUM_SCREEN_OFF,	HOLDREGISTER_CPU3_OFF_TIME,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"ScreenOff"},

	{(uint8_t*)"COM1波特率",	0,	COM_NUM_CPU3_COM1_BAUDRATE,	HOLDREGISTER_CPU3_COM1_BAUD,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Baud"},
	{(uint8_t*)"COM1数据位",	0,	COM_NUM_CPU3_COM1_DATABITS,	HOLDREGISTER_CPU3_COM1_DATABITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Data"},
	{(uint8_t*)"COM1校验",	0,	COM_NUM_CPU3_COM1_PARITY,	HOLDREGISTER_CPU3_COM1_PARITY,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Parity"},
	{(uint8_t*)"COM1停止位",	0,	COM_NUM_CPU3_COM1_STOPBITS,	HOLDREGISTER_CPU3_COM1_STOPBITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Stop"},
	{(uint8_t*)"COM1协议",	0,	COM_NUM_CPU3_COM1_PROTOCOL,	HOLDREGISTER_CPU3_COM1_PROTO,	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Proto"},

	{(uint8_t*)"COM2波特率",	0,	COM_NUM_CPU3_COM2_BAUDRATE,	HOLDREGISTER_CPU3_COM2_BAUD,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Baud"},
	{(uint8_t*)"COM2数据位",	0,	COM_NUM_CPU3_COM2_DATABITS,	HOLDREGISTER_CPU3_COM2_DATABITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Data"},
	{(uint8_t*)"COM2校验",	0,	COM_NUM_CPU3_COM2_PARITY,	HOLDREGISTER_CPU3_COM2_PARITY,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Parity"},
	{(uint8_t*)"COM2停止位",	0,	COM_NUM_CPU3_COM2_STOPBITS,	HOLDREGISTER_CPU3_COM2_STOPBITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Stop"},
	{(uint8_t*)"COM2协议",	0,	COM_NUM_CPU3_COM2_PROTOCOL,	HOLDREGISTER_CPU3_COM2_PROTO,	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Proto"},

	{(uint8_t*)"COM3波特率",	0,	COM_NUM_CPU3_COM3_BAUDRATE,	HOLDREGISTER_CPU3_COM3_BAUD,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Baud"},
	{(uint8_t*)"COM3数据位",	0,	COM_NUM_CPU3_COM3_DATABITS,	HOLDREGISTER_CPU3_COM3_DATABITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Data"},
	{(uint8_t*)"COM3校验",	0,	COM_NUM_CPU3_COM3_PARITY,	HOLDREGISTER_CPU3_COM3_PARITY,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Parity"},
	{(uint8_t*)"COM3停止位",	0,	COM_NUM_CPU3_COM3_STOPBITS,	HOLDREGISTER_CPU3_COM3_STOPBITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Stop"},
	{(uint8_t*)"COM3协议",	0,	COM_NUM_CPU3_COM3_PROTOCOL,	HOLDREGISTER_CPU3_COM3_PROTO,	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Proto"},
};




const int param_metaAmount = sizeof(param_meta) / sizeof(param_meta[0]);

/* 根据操作获取当前保持寄存器具体信息的索引 */
int getHoldValueNum(int operanum)
{
    int i;
    for(i = 0;i < param_metaAmount;i++)
    {
        if(operanum == param_meta[i].operanum)
            break;
    }
    if(i >= param_metaAmount)
        return -1;
    else
        return i;
}

void InputValueInit(void)
{
	g_measurement.oil_measurement.oil_level = UNVALID_LEVEL;
	g_measurement.water_measurement.water_level = LEVEL_DOWNLIMITWATER;
	g_measurement.device_status.device_state = STATE_INIT;
	g_measurement.single_point_monitoring.density = UNVALID_DENSITY;
	g_measurement.single_point_monitoring.temperature = UNVALID_TEMPERATURE_WIRELESS;
}
/* 未与CPU2通讯成功时，需设置设备的故障状态 */
void setEquipStateError(void)
{
//    static uint32_t const comerrcode = 0x000A0001;
//    /* 设备状态 */
//    input_register_value[INPUT_EQUIPMENTSTATE << 1] = 0xFF;
//    input_register_value[(INPUT_EQUIPMENTSTATE << 1) + 1] = 0xFF;
//    /* 故障代码 */
//    input_register_value[INPUT_ERRORCODE << 1] = (comerrcode >> 24) & 0xFF;
//    input_register_value[(INPUT_ERRORCODE << 1) + 1] = (comerrcode >> 16) & 0xFF;
//    input_register_value[(INPUT_ERRORCODE << 1) + 2] = (comerrcode >> 8) & 0xFF;
//    input_register_value[(INPUT_ERRORCODE << 1) + 3] = comerrcode & 0xFF;
//
//    //更新数据
//    Analysis04Register();
}

/* 新增函数：打印所有设备参数 */
void print_device_params(void)
{
    DeviceParameters params;
    memcpy(&params, (void *)&g_deviceParams, sizeof(DeviceParameters));

    printf("========== Device Parameters ==========\r\n");

    /* 指令 */
    printf("[Command]\r\n");
    printf("  command                    : %u\r\n", (unsigned)params.command);
    printf("  powerOnDefaultCommand      : %u\r\n", (unsigned)params.powerOnDefaultCommand);

    /* 基础参数 */
    printf("[Basic]\r\n");
    printf("  sensorType                 : %lu\r\n", (unsigned long)params.sensorType);
    printf("  sensorID                   : %lu\r\n", (unsigned long)params.sensorID);
    printf("  sensorSoftwareVersion      : 0x%08lX\r\n", (unsigned long)params.sensorSoftwareVersion);
    printf("  softwareVersion            : 0x%08lX\r\n", (unsigned long)params.softwareVersion);
    printf("  error_auto_back_zero       : %lu\r\n", (unsigned long)params.error_auto_back_zero);
    printf("  error_stop_measurement     : %lu\r\n", (unsigned long)params.error_stop_measurement);

    /* 电机与编码器 */
    printf("[Motor / Encoder]\r\n");
    printf("  encoder_circ(0.001mm)      : %lu\r\n", (unsigned long)params.encoder_wheel_circumference_mm);
    printf("  max_motor_speed(r/s)       : %lu\r\n", (unsigned long)params.max_motor_speed);
    printf("  first_loop_circ(0.1mm)     : %lu\r\n", (unsigned long)params.first_loop_circumference_mm);
    printf("  tape_thickness(0.001mm)    : %lu\r\n", (unsigned long)params.tape_thickness_mm);

    /* 称重 */
    printf("[Weight]\r\n");
    printf("  empty_weight               : %lu\r\n", (unsigned long)params.empty_weight);
    printf("  empty_weight_upper_limit   : %lu\r\n", (unsigned long)params.empty_weight_upper_limit);
    printf("  empty_weight_lower_limit   : %lu\r\n", (unsigned long)params.empty_weight_lower_limit);
    printf("  full_weight                : %lu\r\n", (unsigned long)params.full_weight);
    printf("  full_weight_upper_limit    : %lu\r\n", (unsigned long)params.full_weight_upper_limit);
    printf("  full_weight_lower_limit    : %lu\r\n", (unsigned long)params.full_weight_lower_limit);
    printf("  weight_upper_limit_ratio   : %lu\r\n", (unsigned long)params.weight_upper_limit_ratio);
    printf("  weight_lower_limit_ratio   : %lu\r\n", (unsigned long)params.weight_lower_limit_ratio);

    /* 零点 */
    printf("[Zero]\r\n");
    printf("  zero_weight_threshold_ratio: %lu\r\n", (unsigned long)params.zero_weight_threshold_ratio);
    printf("  weight_ignore_zone(0.1mm)  : %lu\r\n", (unsigned long)params.weight_ignore_zone);
    printf("  max_zero_deviation(0.1mm)  : %lu\r\n", (unsigned long)params.max_zero_deviation_distance);
    printf("  findZeroDownDistance(0.1mm): %lu\r\n", (unsigned long)params.findZeroDownDistance);

    /* 液位 */
    printf("[Oil Level]\r\n");
    printf("  tankHeight(0.1mm)          : %lu\r\n", (unsigned long)params.tankHeight);
    printf("  liquid_sensor_diff(0.1mm)  : %lu\r\n", (unsigned long)params.liquid_sensor_distance_diff);
    printf("  blindZone(0.1mm)           : %lu\r\n", (unsigned long)params.blindZone);
    printf("  oilLevelThreshold          : %lu\r\n", (unsigned long)params.oilLevelThreshold);
    printf("  oilLevelHysteresis         : %lu\r\n", (unsigned long)params.oilLevelHysteresisThreshold);
    printf("  liquidLevelMethod          : %lu\r\n", (unsigned long)params.liquidLevelMeasurementMethod);

    /* 水位 */
    printf("[Water Level]\r\n");
    printf("  water_tank_height(0.1mm)   : %lu\r\n", (unsigned long)params.water_tank_height);
    printf("  water_sensor_diff(0.1mm)   : %lu\r\n", (unsigned long)params.water_level_sensor_distance_diff);
    printf("  waterBlindZone(0.1mm)      : %lu\r\n", (unsigned long)params.waterBlindZone);
    printf("  water_cap_threshold        : %lu\r\n", (unsigned long)params.water_cap_threshold);
    printf("  water_cap_hysteresis       : %lu\r\n", (unsigned long)params.water_cap_hysteresis);
    printf("  maxDownDistance(0.1mm)     : %lu\r\n", (unsigned long)params.maxDownDistance);

    /* 罐底/罐高 */
    printf("[Bottom / Tank Height]\r\n");
    printf("  bottom_detect_mode         : %lu\r\n", (unsigned long)params.bottom_detect_mode);
    printf("  bottom_angle_threshold     : %lu\r\n", (unsigned long)params.bottom_angle_threshold);
    printf("  bottom_weight_threshold    : %lu\r\n", (unsigned long)params.bottom_weight_threshold);
    printf("  refreshTankHeightFlag      : %lu\r\n", (unsigned long)params.refreshTankHeightFlag);
    printf("  maxTankHeightDeviation     : %lu\r\n", (unsigned long)params.maxTankHeightDeviation);
    printf("  initialTankHeight          : %lu\r\n", (unsigned long)params.initialTankHeight);
    printf("  currentTankHeight          : %lu\r\n", (unsigned long)params.currentTankHeight);

    /* 修正 */
    printf("[Correction]\r\n");
    printf("  densityCorrection          : %lu\r\n", (unsigned long)params.densityCorrection);
    printf("  temperatureCorrection      : %lu\r\n", (unsigned long)params.temperatureCorrection);

    /* 分布/区间 */
    printf("[Spread / Interval]\r\n");
    printf("  requireBottomMeasurement   : %lu\r\n", (unsigned long)params.requireBottomMeasurement);
    printf("  requireWaterMeasurement    : %lu\r\n", (unsigned long)params.requireWaterMeasurement);
    printf("  requireSinglePointDensity  : %lu\r\n", (unsigned long)params.requireSinglePointDensity);
    printf("  spreadMeasurementOrder     : %lu\r\n", (unsigned long)params.spreadMeasurementOrder);
    printf("  spreadMeasurementMode      : %lu\r\n", (unsigned long)params.spreadMeasurementMode);
    printf("  spreadMeasurementCount     : %lu\r\n", (unsigned long)params.spreadMeasurementCount);
    printf("  spreadMeasurementDistance  : %lu\r\n", (unsigned long)params.spreadMeasurementDistance);
    printf("  spreadTopLimit(0.1mm)      : %lu\r\n", (unsigned long)params.spreadTopLimit);
    printf("  spreadBottomLimit(0.1mm)   : %lu\r\n", (unsigned long)params.spreadBottomLimit);
    printf("  spreadPointHoverTime       : %lu\r\n", (unsigned long)params.spreadPointHoverTime);
    printf("  intervalTopLimit(0.1mm)    : %lu\r\n", (unsigned long)params.intervalMeasurementTopLimit);
    printf("  intervalBottomLimit(0.1mm) : %lu\r\n", (unsigned long)params.intervalMeasurementBottomLimit);

    /* Wartsila */
    printf("[Wartsila]\r\n");
    printf("  upper_density_limit        : %lu\r\n", (unsigned long)params.wartsila_upper_density_limit);
    printf("  lower_density_limit        : %lu\r\n", (unsigned long)params.wartsila_lower_density_limit);
    printf("  density_interval           : %lu\r\n", (unsigned long)params.wartsila_density_interval);
    printf("  max_height_above_surface   : %lu\r\n", (unsigned long)params.wartsila_max_height_above_surface);

    /* DO */
    printf("[Alarm DO]\r\n");
    printf("  AlarmHighDO                : %lu\r\n", (unsigned long)params.AlarmHighDO);
    printf("  AlarmLowDO                 : %lu\r\n", (unsigned long)params.AlarmLowDO);
    printf("  ThirdStateThreshold        : %lu\r\n", (unsigned long)params.ThirdStateThreshold);

    /* AO */
    printf("[4-20mA / AO]\r\n");
    printf("  CurrentRangeStart_mA       : %lu\r\n", (unsigned long)params.CurrentRangeStart_mA);
    printf("  CurrentRangeEnd_mA         : %lu\r\n", (unsigned long)params.CurrentRangeEnd_mA);
    printf("  AlarmHighAO                : %lu\r\n", (unsigned long)params.AlarmHighAO);
    printf("  AlarmLowAO                 : %lu\r\n", (unsigned long)params.AlarmLowAO);
    printf("  InitialCurrent_mA          : %lu\r\n", (unsigned long)params.InitialCurrent_mA);
    printf("  AOHighCurrent_mA           : %lu\r\n", (unsigned long)params.AOHighCurrent_mA);
    printf("  AOLowCurrent_mA            : %lu\r\n", (unsigned long)params.AOLowCurrent_mA);
    printf("  FaultCurrent_mA            : %lu\r\n", (unsigned long)params.FaultCurrent_mA);
    printf("  DebugCurrent_mA            : %lu\r\n", (unsigned long)params.DebugCurrent_mA);

    /* 指令参数 */
    printf("[Command Params]\r\n");
    printf("  calibrateOilLevel          : %lu\r\n", (unsigned long)params.calibrateOilLevel);
    printf("  calibrateWaterLevel        : %lu\r\n", (unsigned long)params.calibrateWaterLevel);
    printf("  singlePointMeasurePos      : %lu\r\n", (unsigned long)params.singlePointMeasurementPosition);
    printf("  singlePointMonitorPos      : %lu\r\n", (unsigned long)params.singlePointMonitoringPosition);
    printf("  densityDistributionOilLevel: %lu\r\n", (unsigned long)params.densityDistributionOilLevel);
    printf("  motorCommandDistance       : %lu\r\n", (unsigned long)params.motorCommandDistance);

    /* 元信息/CRC */
    printf("[Meta]\r\n");
    printf("  param_version              : %lu\r\n", (unsigned long)params.param_version);
    printf("  struct_size                : %lu\r\n", (unsigned long)params.struct_size);
    printf("  magic                      : 0x%08lX\r\n", (unsigned long)params.magic);
    printf("  crc                        : 0x%08lX\r\n", (unsigned long)params.crc);

    printf("======================================\r\n");
}


