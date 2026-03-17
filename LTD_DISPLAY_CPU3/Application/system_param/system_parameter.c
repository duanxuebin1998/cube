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

/* 名称	数值	数据号	起始地址	寄存器数	是否检范围	最小	最大	单位	小数	偏移	写权	类型	显示	隐藏	英文 */
//参数元数据
struct ParameterMetadata param_meta[] = {

{(uint8_t*)"设备指令",	0,	COM_NUM_DEVICEPARAM_COMMAND,	HOLDREGISTER_DEVICEPARAM_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Cmd"},

{(uint8_t*)"传感器类型",	0,	COM_NUM_DEVICEPARAM_SENSORTYPE,	HOLDREGISTER_DEVICEPARAM_SENSORTYPE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	3,	NULL,	(uint8_t*)"SensorType"},
{(uint8_t*)"传感器编号",	0,	COM_NUM_DEVICEPARAM_SENSORID,	HOLDREGISTER_DEVICEPARAM_SENSORID,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"SensorID"},
{(uint8_t*)"传感器软件版本",	0,	COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"SenSWVer"},
{(uint8_t*)"软件版本",	0,	COM_NUM_DEVICEPARAM_SOFTWAREVERSION,	HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"FWVersion"},
{(uint8_t*)"上电默认指令",	0,	COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,	HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	3,	ret_arr_word,	(uint8_t*)"PwrOnCmd"},
{(uint8_t*)"故障自动回零",	0,	COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO,	HOLDREGISTER_DEVICEPARAM_ERROR_AUTO_BACK_ZERO,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"ErrAutoBackZero"},
{(uint8_t*)"故障停止测量",	0,	COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT,	HOLDREGISTER_DEVICEPARAM_ERROR_STOP_MEASUREMENT,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"ErrStopMeas"},

{(uint8_t*)"保留1",	0,	COM_NUM_DEVICEPARAM_RESERVED1,	HOLDREGISTER_DEVICEPARAM_RESERVED1,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv1"},
{(uint8_t*)"保留2",	0,	COM_NUM_DEVICEPARAM_RESERVED2,	HOLDREGISTER_DEVICEPARAM_RESERVED2,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv2"},
{(uint8_t*)"保留3",	0,	COM_NUM_DEVICEPARAM_RESERVED3,	HOLDREGISTER_DEVICEPARAM_RESERVED3,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv3"},
{(uint8_t*)"保留4",	0,	COM_NUM_DEVICEPARAM_RESERVED4,	HOLDREGISTER_DEVICEPARAM_RESERVED4,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv4"},
{(uint8_t*)"保留5",	0,	COM_NUM_DEVICEPARAM_RESERVED5,	HOLDREGISTER_DEVICEPARAM_RESERVED5,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv5"},

{(uint8_t*)"编码轮周长mm",	0,	COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EncWheelCirc"},
{(uint8_t*)"电机最大速度",	0,	COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED,	HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED,	2,	false,	0,	0,	NULL,	2,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"MaxMotorSpd"},
{(uint8_t*)"首圈周长mm",	0,	COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FirstLoop"},
{(uint8_t*)"尺带厚度mm",	0,	COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM,	HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"TapeThick"},
{(uint8_t*)"保留6",	0,	COM_NUM_DEVICEPARAM_RESERVED6,	HOLDREGISTER_DEVICEPARAM_RESERVED6,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv6"},
{(uint8_t*)"保留7",	0,	COM_NUM_DEVICEPARAM_RESERVED7,	HOLDREGISTER_DEVICEPARAM_RESERVED7,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv7"},

{(uint8_t*)"空载重量",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyWt"},
{(uint8_t*)"空载重量上限",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyWtHi"},
{(uint8_t*)"空载重量下限",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyWtLo"},
{(uint8_t*)"满载重量",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullWt"},
{(uint8_t*)"满载重量上限",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullWtHi"},
{(uint8_t*)"满载重量下限",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullWtLo"},
{(uint8_t*)"碰撞上限比率",	0,	COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,	HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,	2,	false,	0,	0,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"UpperRatio"},
{(uint8_t*)"碰撞下限比率",	0,	COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,	HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,	2,	false,	0,	0,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"LowerRatio"},
{(uint8_t*)"保留8",	0,	COM_NUM_DEVICEPARAM_RESERVED8,	HOLDREGISTER_DEVICEPARAM_RESERVED8,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv8"},
{(uint8_t*)"保留9",	0,	COM_NUM_DEVICEPARAM_RESERVED9,	HOLDREGISTER_DEVICEPARAM_RESERVED9,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv9"},

{(uint8_t*)"零点阈值比例",	0,	COM_NUM_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO,	HOLDREGISTER_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO,	2,	true,	0,	100,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"ZeroThRatio"},
{(uint8_t*)"称重忽略区",	0,	COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE,	HOLDREGISTER_DEVICEPARAM_WEIGHT_IGNORE_ZONE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WeightIgnore"},
{(uint8_t*)"零点最大偏差",	0,	COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE,	HOLDREGISTER_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"MaxZeroDev"},
{(uint8_t*)"找零下行距离",	0,	COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,	HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"FindZeroDown"},
{(uint8_t*)"保留10",	0,	COM_NUM_DEVICEPARAM_RESERVED10,	HOLDREGISTER_DEVICEPARAM_RESERVED10,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv10"},
{(uint8_t*)"保留11",	0,	COM_NUM_DEVICEPARAM_RESERVED11,	HOLDREGISTER_DEVICEPARAM_RESERVED11,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv11"},

{(uint8_t*)"液位罐高",	0,	COM_NUM_DEVICEPARAM_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_TANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TankHeight"},
{(uint8_t*)"液位探头距差",	0,	COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF,	HOLDREGISTER_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF,	2,	false,	-999999,	999999,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"LvlSenDiff"},
{(uint8_t*)"液位盲区",	0,	COM_NUM_DEVICEPARAM_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_BLINDZONE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"BlindZone"},
{(uint8_t*)"找油阈值",	0,	COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD,	HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"OilLvlTh"},
{(uint8_t*)"滞后阈值",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"HysTh"},
{(uint8_t*)"液位测量方式",	0,	COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,	HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"LvlMeasMode"},
{(uint8_t*)"液位跟随频率",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_FREQUENCY,	2,	false,	0,	0,	(uint8_t*)"HZ",	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"FollowFreq"},
{(uint8_t*)"液位跟随密度",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_DENSITY,	2,	false,	0,	0,	(uint8_t*)"kg/m3",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"FollowDens"},
{(uint8_t*)"液位滞后时间",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME,	2,	false,	0,	0,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"OilHysTime"},

{(uint8_t*)"水位罐高",	0,	COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT,	HOLDREGISTER_DEVICEPARAM_WATER_TANK_HEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WaterTankH"},
{(uint8_t*)"水位测量方式",	0,	COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE,	HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_MODE,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"WaterSenDiff"},
{(uint8_t*)"水位盲区",	0,	COM_NUM_DEVICEPARAM_WATER_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterBlind"},
{(uint8_t*)"水位电容阈值",	0,	COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_WATER_CAP_THRESHOLD,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterCapTh"},
{(uint8_t*)"水位电容滞回",	0,	COM_NUM_DEVICEPARAM_WATER_CAP_HYSTERESIS,	HOLDREGISTER_DEVICEPARAM_WATER_CAP_HYSTERESIS,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterCapHys"},
{(uint8_t*)"最大下行距离",	0,	COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE,	HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"MaxDown"},
{(uint8_t*)"零点电容",	0,	COM_NUM_DEVICEPARAM_ZERO_CAP,	HOLDREGISTER_DEVICEPARAM_ZERO_CAP,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ZeroCap"},
{(uint8_t*)"水位稳定阈值",	0,	COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_WATER_STABLE_THRESHOLD,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"Rsv15"},
{(uint8_t*)"水位修正值",	0,	COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION,	HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_CORRECTION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WaterCorr"},

{(uint8_t*)"罐底检测模式",	0,	COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE,	HOLDREGISTER_DEVICEPARAM_BOTTOM_DETECT_MODE,	2,	true,	0,	3,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"BottomMode"},
{(uint8_t*)"罐底角度阈值",	0,	COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BottomAngTh"},
{(uint8_t*)"罐底称重阈值",	0,	COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BottomWtTh"},
{(uint8_t*)"更新罐高标志",	0,	COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,	HOLDREGISTER_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"UpdTankHeight"},
{(uint8_t*)"实高最大偏差",	0,	COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION,	HOLDREGISTER_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"MaxDev"},
{(uint8_t*)"初始罐高",	0,	COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_INITIAL_TANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"InitTankH"},
{(uint8_t*)"当前罐高",	0,	COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_CURRENT_TANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"CurrTankH"},
{(uint8_t*)"保留16",	0,	COM_NUM_DEVICEPARAM_RESERVED16,	HOLDREGISTER_DEVICEPARAM_RESERVED16,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv16"},
{(uint8_t*)"保留17",	0,	COM_NUM_DEVICEPARAM_RESERVED17,	HOLDREGISTER_DEVICEPARAM_RESERVED17,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv17"},

{(uint8_t*)"磁通量D",	0,	COM_NUM_DEVICEPARAM_DENSITYCORRECTION,	HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION,	2,	false,	-10000,	10000,	NULL,	1,	-10000,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DensityCorr"},
{(uint8_t*)"磁通量T",	0,	COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION,	HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION,	2,	false,	-1000,	1000,	NULL,	1,	-1000,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"TempCorr"},
{(uint8_t*)"保留18",	0,	COM_NUM_DEVICEPARAM_RESERVED18,	HOLDREGISTER_DEVICEPARAM_RESERVED18,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv18"},
{(uint8_t*)"保留19",	0,	COM_NUM_DEVICEPARAM_RESERVED19,	HOLDREGISTER_DEVICEPARAM_RESERVED19,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv19"},

{(uint8_t*)"是否测罐底",	0,	COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,	HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"NeedBottom"},
{(uint8_t*)"是否测水位",	0,	COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT,	HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"NeedWater"},
{(uint8_t*)"是否测单点密度",	0,	COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,	HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"NeedSingleDens"},
{(uint8_t*)"分布测顺序",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SpreadOrder"},
{(uint8_t*)"分布测模式",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SpreadMode"},
{(uint8_t*)"分布测点数",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT,	2,	true,	1,	99,	NULL,	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"SpreadCount"},
{(uint8_t*)"分布点间距",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SpreadDist"},
{(uint8_t*)"顶点距液面",	0,	COM_NUM_DEVICEPARAM_SPREADTOPLIMIT,	HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Top2Level"},
{(uint8_t*)"底点距罐底",	0,	COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT,	HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Bot2Tank"},
{(uint8_t*)"分布点悬停(s)",	0,	COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,	HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,	2,	true,	0,	600,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"PtHoverTime"},
{(uint8_t*)"区间测量上限",	0,	COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT,	HOLDREGISTER_DEVICEPARAM_INTERVAL_TOPLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"IntTop"},
{(uint8_t*)"区间测量下限",	0,	COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT,	HOLDREGISTER_DEVICEPARAM_INTERVAL_BOTTOMLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"IntBottom"},
{(uint8_t*)"保留20",	0,	COM_NUM_DEVICEPARAM_RESERVED20,	HOLDREGISTER_DEVICEPARAM_RESERVED20,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv20"},
{(uint8_t*)"保留21",	0,	COM_NUM_DEVICEPARAM_RESERVED21,	HOLDREGISTER_DEVICEPARAM_RESERVED21,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv21"},

{(uint8_t*)"密度点上限",	0,	COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WUpperDen"},
{(uint8_t*)"密度点下限",	0,	COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WLowerDen"},
{(uint8_t*)"密度点间距",	0,	COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WDenStep"},
{(uint8_t*)"最高点距液面",	0,	COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WMaxHeight"},
{(uint8_t*)"保留22",	0,	COM_NUM_DEVICEPARAM_RESERVED22,	HOLDREGISTER_DEVICEPARAM_RESERVED22,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv22"},
{(uint8_t*)"保留23",	0,	COM_NUM_DEVICEPARAM_RESERVED23,	HOLDREGISTER_DEVICEPARAM_RESERVED23,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv23"},

{(uint8_t*)"高液位报警(DO)",	0,	COM_NUM_DEVICEPARAM_ALARM_HIGH_DO,	HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_DO,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"AlarmHiDO"},
{(uint8_t*)"低液位报警(DO)",	0,	COM_NUM_DEVICEPARAM_ALARM_LOW_DO,	HOLDREGISTER_DEVICEPARAM_ALARM_LOW_DO,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"AlarmLoDO"},
{(uint8_t*)"第三状态阈值",	0,	COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_THIRD_STATE_THRESHOLD,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ThirdStateTh"},
{(uint8_t*)"保留24",	0,	COM_NUM_DEVICEPARAM_RESERVED24,	HOLDREGISTER_DEVICEPARAM_RESERVED24,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv24"},
{(uint8_t*)"保留25",	0,	COM_NUM_DEVICEPARAM_RESERVED25,	HOLDREGISTER_DEVICEPARAM_RESERVED25,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv25"},

{(uint8_t*)"输出范围起点mA",	0,	COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA,	HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_START_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"RangeStart"},
{(uint8_t*)"输出范围终点mA",	0,	COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA,	HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_END_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"RangeEnd"},
{(uint8_t*)"高限报警(AO)",	0,	COM_NUM_DEVICEPARAM_ALARM_HIGH_AO,	HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_AO,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AlarmHiAO"},
{(uint8_t*)"低限报警(AO)",	0,	COM_NUM_DEVICEPARAM_ALARM_LOW_AO,	HOLDREGISTER_DEVICEPARAM_ALARM_LOW_AO,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AlarmLoAO"},
{(uint8_t*)"初始电流mA",	0,	COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_INITIAL_CURRENT_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"InitCurrent"},
{(uint8_t*)"高位电流mA",	0,	COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_AO_HIGH_CURRENT_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"HighCurrent"},
{(uint8_t*)"低位电流mA",	0,	COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_AO_LOW_CURRENT_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"LowCurrent"},
{(uint8_t*)"故障电流mA",	0,	COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_FAULT_CURRENT_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FaultCurrent"},
{(uint8_t*)"调试电流mA",	0,	COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA,	2,	true,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DebugCurrent"},
{(uint8_t*)"保留26",	0,	COM_NUM_DEVICEPARAM_RESERVED26,	HOLDREGISTER_DEVICEPARAM_RESERVED26,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv26"},
{(uint8_t*)"保留27",	0,	COM_NUM_DEVICEPARAM_RESERVED27,	HOLDREGISTER_DEVICEPARAM_RESERVED27,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv27"},

{(uint8_t*)"标定液位值",	0,	COM_NUM_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalOilLvl"},
{(uint8_t*)"标定水位值",	0,	COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalWaterLvl"},
{(uint8_t*)"单点测量位置",	0,	COM_NUM_DEVICEPARAM_SP_MEAS_POSITION,	HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MeasPos"},
{(uint8_t*)"单点监测位置",	0,	COM_NUM_DEVICEPARAM_SP_MEAS_POSITION,	HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MonPos"},
{(uint8_t*)"电机运行位置",	0,	COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"DistOilLvl"},
{(uint8_t*)"电机上行距离",	0,	COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"保留28",	0,	COM_NUM_DEVICEPARAM_RESERVED28,	HOLDREGISTER_DEVICEPARAM_RESERVED28,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv28"},
{(uint8_t*)"保留29",	0,	COM_NUM_DEVICEPARAM_RESERVED29,	HOLDREGISTER_DEVICEPARAM_RESERVED29,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv29"},

{(uint8_t*)"上次液位修正液位",	0,	COM_NUM_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL,	HOLDREGISTER_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"LastOilCorrLvl"},
{(uint8_t*)"气相温度",	0,	COM_NUM_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE,	HOLDREGISTER_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE,	2,	false,	0,	0,	(uint8_t*)"C",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"GasTemp"},
{(uint8_t*)"尺带伸缩率",	0,	COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT,	HOLDREGISTER_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT,	2,	false,	0,	0,	NULL,	6,	0,	true,	TYPE_INT,	8,	NULL,	(uint8_t*)"TapeExpCoeff"},
{(uint8_t*)"标定尺带温度",	0,	COM_NUM_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE,	HOLDREGISTER_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE,	2,	false,	0,	0,	(uint8_t*)"C",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"TapeCalTemp"},
{(uint8_t*)"保留30",	0,	COM_NUM_DEVICEPARAM_RESERVED30,	HOLDREGISTER_DEVICEPARAM_RESERVED30,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv30"},
{(uint8_t*)"保留31",	0,	COM_NUM_DEVICEPARAM_RESERVED31,	HOLDREGISTER_DEVICEPARAM_RESERVED31,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv31"},
{(uint8_t*)"保留32",	0,	COM_NUM_DEVICEPARAM_RESERVED32,	HOLDREGISTER_DEVICEPARAM_RESERVED32,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv32"},
{(uint8_t*)"保留33",	0,	COM_NUM_DEVICEPARAM_RESERVED33,	HOLDREGISTER_DEVICEPARAM_RESERVED33,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv33"},


{(uint8_t*)"参数版本号",	0,	COM_NUM_DEVICEPARAM_PARAM_VERSION,	HOLDREGISTER_DEVICEPARAM_PARAM_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"ParamVer"},
{(uint8_t*)"结构体大小",	0,	COM_NUM_DEVICEPARAM_STRUCT_SIZE,	HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"StructSize"},
{(uint8_t*)"魔术字",	0,	COM_NUM_DEVICEPARAM_MAGIC,	HOLDREGISTER_DEVICEPARAM_MAGIC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Magic"},
{(uint8_t*)"参数CRC32",	0,	COM_NUM_DEVICEPARAM_CRC,	HOLDREGISTER_DEVICEPARAM_CRC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"ParamCRC"},


/* ==================== 下发指令参数（保持你原来的寄存器宏）==================== */
{(uint8_t*)"标定液位值",	0,	COM_NUM_CAL_OIL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalOilLvl"},
{(uint8_t*)"修正液位值",	0,	COM_NUM_CORRECTION_OIL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalOilLvl"},
{(uint8_t*)"标定水位值",	0,	COM_NUM_CALIBRATE_WATER,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalWaterLvl"},
{(uint8_t*)"单点测量位置",	0,	COM_NUM_SINGLE_POINT,	HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MeasPos"},
{(uint8_t*)"单点监测位置",	0,	COM_NUM_SP_TEST,	HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MonPos"},
{(uint8_t*)"电机运行位置",	0,	COM_NUM_RUN_TO_POSITION,	HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"DistOilLvl"},
{(uint8_t*)"电机上行距离",	0,	COM_NUM_RUNUP,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"电机下行距离",	0,	COM_NUM_RUNDOWN,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"电机上行距离",	0,	COM_NUM_FORCE_RUNUP,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"电机下行距离",	0,	COM_NUM_FORCE_RUNDOWN,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},

/* ==================== CPU3 本机参数（保持你原来的寄存器宏）==================== */
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
    printf("  max_motor_speed(0.01m/min): %lu\r\n", (unsigned long)params.max_motor_speed);
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
    printf("  oilLevelFrequency          : %lu\r\n", (unsigned long)params.oilLevelFrequency);
    printf("  oilLevelDensity            : %lu\r\n", (unsigned long)params.oilLevelDensity);
    printf("  oilLevelHysteresisTime     : %lu\r\n", (unsigned long)params.oilLevelHysteresisTime);

    /* 水位 */
    printf("[Water Level]\r\n");
    printf("  water_tank_height(0.1mm)   : %lu\r\n", (unsigned long)params.water_tank_height);
    printf("  water_level_mode           : %lu\r\n", (unsigned long)params.water_level_mode);
    printf("  waterBlindZone(0.1mm)      : %lu\r\n", (unsigned long)params.waterBlindZone);
    printf("  water_cap_threshold        : %lu\r\n", (unsigned long)params.water_cap_threshold);
    printf("  water_cap_hysteresis       : %lu\r\n", (unsigned long)params.water_cap_hysteresis);
    printf("  maxDownDistance(0.1mm)     : %lu\r\n", (unsigned long)params.maxDownDistance);
    printf("  waterLevel_zero_cap        : %lu\r\n", (unsigned long)params.zero_cap);
    printf("  water_stable_threshold     : %lu\r\n", (unsigned long)params.water_stable_threshold);
    printf("  waterLevelCorrection       : %lu\r\n", (unsigned long)params.waterLevelCorrection);

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

    printf("[Tape Compensation]\r\n");
    printf("  lastOilCorrectionLevel     : %lu\r\n", (unsigned long)params.lastOilCorrectionLevel);
    printf("  tankGasPhaseTemperature    : %lu\r\n", (unsigned long)params.tankGasPhaseTemperature);
    printf("  tapeExpansionCoefficient   : %lu\r\n", (unsigned long)params.tapeExpansionCoefficient);
    printf("  tapeCalibrationTemperature : %lu\r\n", (unsigned long)params.tapeCalibrationTemperature);

    /* 元信息/CRC */
    printf("[Meta]\r\n");
    printf("  param_version              : %lu\r\n", (unsigned long)params.param_version);
    printf("  struct_size                : %lu\r\n", (unsigned long)params.struct_size);
    printf("  magic                      : 0x%08lX\r\n", (unsigned long)params.magic);
    printf("  crc                        : 0x%08lX\r\n", (unsigned long)params.crc);

    printf("======================================\r\n");
}
/*========================= 测量结果打印（可选） =========================*/
/* 注: 该部分与参数结构无强耦合，仅保留你现有打印习惯；如果不需要可移除 */

void PrintDensity(const char *title, const DensityMeasurement *d)
{
    if (!d) return;

    printf("  [%s]\r\n", title);
    printf("    温度: %lu\r\n", (unsigned long)d->temperature);
    printf("    密度: %lu\r\n", (unsigned long)d->density);
    printf("    标准密度: %lu\r\n", (unsigned long)d->standard_density);
    printf("    VCF20: %lu\r\n", (unsigned long)d->vcf20);
    printf("    计重密度: %lu\r\n", (unsigned long)d->weight_density);
    printf("    温度位置: %lu\r\n", (unsigned long)d->temperature_position);
}
void PrintMeasurementResult(const MeasurementResult *m)
{
    if (!m) return;

    printf("\r\n=====================【设备实时测量结果】=====================\r\n");

    /* 1. 设备状态 */
    printf("【设备状态】\r\n");
    printf("  工作模式: %lu\r\n", (unsigned long)m->device_status.work_mode);
    printf("  设备状态: %d\r\n",  (int)m->device_status.device_state);
    printf("  错误代码: %lu\r\n", (unsigned long)m->device_status.error_code);
    printf("  当前指令: %d\r\n",  (int)m->device_status.current_command);
    printf("  零点状态: %lu\r\n", (unsigned long)m->device_status.zero_point_status);

    printf("--------------------------------------------------------------\r\n");

    /* 2. 调试数据 */
    printf("【调试数据 DebugData】\r\n");
    printf("  编码值: %ld\r\n",        (long)m->debug_data.current_encoder_value);
    printf("  传感器位置: %ld mm\r\n", (long)m->debug_data.sensor_position);
    printf("  尺带长度: %ld mm\r\n",   (long)m->debug_data.cable_length);
    printf("  电机步进: %ld\r\n",      (long)m->debug_data.motor_step);
    printf("  电机距离: %ld (0.1mm)\r\n",(long)m->debug_data.motor_distance);

    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->debug_data.frequency);
    printf("  温度: %.2f ℃\r\n",       (float)m->debug_data.temperature / 100.0f);
    printf("  空气中频率: %lu Hz\r\n", (unsigned long)m->debug_data.air_frequency);
    printf("  幅值: %lu\r\n",          (unsigned long)m->debug_data.current_amplitude);
    printf("  水位电压/电容值: %lu\r\n",(unsigned long)m->debug_data.water_level_voltage);

    printf("  当前称重值: %lu\r\n",    (unsigned long)m->debug_data.current_weight);
    printf("  称重参数: %lu\r\n",      (unsigned long)m->debug_data.weight_param);

    printf("  X角度: %ld\r\n", (long)m->debug_data.angle_x);
    printf("  Y角度: %ld\r\n", (long)m->debug_data.angle_y);

    printf("  motor_speed(0.01m/min): %lu\r\n", (unsigned long)m->debug_data.motor_speed);
    printf("  电机状态: %lu (%s)\r\n",
           (unsigned long)m->debug_data.motor_state,
           (m->debug_data.motor_state == 0) ? "停止" :
           (m->debug_data.motor_state == 1) ? "上行" :
           (m->debug_data.motor_state == 2) ? "下行" : "未知");

    printf("--------------------------------------------------------------\r\n");

    /* 3. 液位测量 */
    printf("【液位测量 OilMeasurement】\r\n");
    printf("  跟随液位: %lu mm\r\n",   (unsigned long)m->oil_measurement.oil_level);
    printf("  空气频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.air_frequency);
    printf("  油中频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.oil_frequency);
    printf("  跟随频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.follow_frequency);
    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.current_frequency);

    printf("--------------------------------------------------------------\r\n");

    /* 4. 水位测量 */
    printf("【水位测量 WaterMeasurement】\r\n");
    printf("  水位值: %lu mm\r\n", (unsigned long)m->water_measurement.water_level);
    printf("  零点电容: %.3f\r\n",(float) m->water_measurement.zero_capacitance);
    printf("  油区电容: %.3f\r\n",(float) m->water_measurement.oil_capacitance);
    printf("  当前电容: %.3f\r\n",(float) m->water_measurement.current_capacitance);

    printf("--------------------------------------------------------------\r\n");

    /* 5. 实高测量 */
    printf("【实高测量 ActualHeight】\r\n");
    printf("  标定液位实高: %lu mm\r\n", (unsigned long)m->height_measurement.calibrated_liquid_level);
    printf("  当前实高: %lu mm\r\n",     (unsigned long)m->height_measurement.current_real_height);

    printf("--------------------------------------------------------------\r\n");

    /* 6. 单点密度测量 */
    printf("【单点密度测量 SinglePoint】\r\n");
    PrintDensity("单点测量", &m->single_point_measurement);

    /* 7. 单点监测 */
    printf("【单点监测 SinglePoint Monitoring】\r\n");
    PrintDensity("单点监测", &m->single_point_monitoring);

    printf("--------------------------------------------------------------\r\n");

    /* 8. 密度分布（概要） */
    printf("【密度分布 DensityDistribution】\r\n");
    printf("  平均温度: %lu\r\n",     (unsigned long)m->density_distribution.average_temperature);
    printf("  平均密度: %lu\r\n",     (unsigned long)m->density_distribution.average_density);
    printf("  平均计重密度: %lu\r\n", (unsigned long)m->density_distribution.average_weight_density);
    printf("  测量点数: %lu\r\n",     (unsigned long)m->density_distribution.measurement_points);
    printf("  测量时液位: %lu mm\r\n",(unsigned long)m->density_distribution.Density_oil_level);

    printf("  --- 单点数据（仅打印前10个）---\r\n");
    for (uint32_t i = 0; i < 10 && i < m->density_distribution.measurement_points; i++)
    {
        const DensityMeasurement *d = &m->density_distribution.single_density_data[i];
        printf("    [%02lu] T=%lu ρ=%lu ρ15=%lu VCF=%lu WD=%lu Pos=%lu\r\n",
               (unsigned long)i,
               (unsigned long)d->temperature,
               (unsigned long)d->density,
               (unsigned long)d->standard_density,
               (unsigned long)d->vcf20,
               (unsigned long)d->weight_density,
               (unsigned long)d->temperature_position);
    }

    printf("========================【打印结束】========================\r\n");
}
