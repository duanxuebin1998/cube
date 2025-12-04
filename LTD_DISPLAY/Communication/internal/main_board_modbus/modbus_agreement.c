#include "modbus_agreement.h"
#include "usart.h"
#include "display_tankopera.h"
#include <math.h>
#include "communicate.h"
#include "system_parameter.h"

int cnt_commutoCPU2 = COMMU_ERROR_MAX;

static int ywj_hold_analysis_data(int startadd,int rgscnt);

struct HoldRegisterData holdValue[] = {
	/* 名称	默认	数据号	起始地址	寄存器数	是否检	最小	最大	单位	小数	偏移	写权	类型	显示	隐藏	英文 */
	{(uint8_t*)"设备指令",	0,	COM_NUM_DEVICEPARAM_COMMAND,	HOLDREGISTER_DEVICEPARAM_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	8,	NULL,	(uint8_t*)"Cmd"},

/* 基础参数 */
	{(uint8_t*)"罐高",	0,	COM_NUM_DEVICEPARAM_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_TANKHEIGHT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TankHeight"},
	{(uint8_t*)"液位盲区",	0,	COM_NUM_DEVICEPARAM_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_BLINDZONE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"BlindZone"},
	{(uint8_t*)"水位盲区",	0,	COM_NUM_DEVICEPARAM_WATER_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterBlind"},
	{(uint8_t*)"编码轮周长mm",	0,	COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"EncWheelCirc"},

	/* 新增：最大电机速度、首圈周长、尺带厚度 */
	{(uint8_t*)"电机最大速度",	0,	COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED,	HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED,	2,	false,	0,	0,	NULL,	2,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"MaxMotorSpd"},
	{(uint8_t*)"首圈周长mm",	0,	COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FirstLoop"},
	{(uint8_t*)"尺带厚度mm",	0,	COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM,	HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"TapeThick"},

	{(uint8_t*)"传感器类型",	0,	COM_NUM_DEVICEPARAM_SENSORTYPE,	HOLDREGISTER_DEVICEPARAM_SENSORTYPE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	3,	NULL,	(uint8_t*)"SensorType"},
	{(uint8_t*)"传感器编号",	0,	COM_NUM_DEVICEPARAM_SENSORID,	HOLDREGISTER_DEVICEPARAM_SENSORID,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"SensorID"},

	/* 新增：传感器软件版本 */
	{(uint8_t*)"传感器软件版本",	0,	COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"SenSWVer"},

	{(uint8_t*)"软件版本",	0,	COM_NUM_DEVICEPARAM_SOFTWAREVERSION,	HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"FWVersion"},

	/* 新增：上电默认指令 */
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
//	{(uint8_t*)"分布测液位",	0,	COM_NUM_SPREADPOINTS,	HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"DistLvl"},
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

/* 瓦锡兰密度区间测量参数（新增） */
	{(uint8_t*)"密度上限",	0,	COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"WUpperDen"},
	{(uint8_t*)"密度下限",	0,	COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"WLowerDen"},
	{(uint8_t*)"密度步进",	0,	COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"WDenStep"},
	{(uint8_t*)"最高点距液面",	0,	COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WMaxHeight"},

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

/* 4–20mA 输出与报警（AO） */
	{(uint8_t*)"输出范围起点mA",	0,	COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA,	HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_START_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"RangeStart"},
	{(uint8_t*)"输出范围终点mA",	0,	COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA,	HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_END_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"RangeEnd"},
	{(uint8_t*)"高限报警(AO)",	0,	COM_NUM_DEVICEPARAM_ALARM_HIGH_AO,	HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_AO,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AlarmHiAO"},
	{(uint8_t*)"低限报警(AO)",	0,	COM_NUM_DEVICEPARAM_ALARM_LOW_AO,	HOLDREGISTER_DEVICEPARAM_ALARM_LOW_AO,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AlarmLoAO"},
	{(uint8_t*)"初始电流mA",	0,	COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_INITIAL_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"InitCurrent"},
	{(uint8_t*)"高位电流mA",	0,	COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_AO_HIGH_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"HighCurrent"},
	{(uint8_t*)"低位电流mA",	0,	COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_AO_LOW_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"LowCurrent"},
	{(uint8_t*)"故障电流mA",	0,	COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_FAULT_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FaultCurrent"},
	{(uint8_t*)"调试电流mA",	0,	COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA,	HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA,	2,	false,	0,	3000,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DebugCurrent"},

/* 结构版本信息（新增，一般只读） */
	{(uint8_t*)"参数版本号",	0,	COM_NUM_DEVICEPARAM_PARAM_VERSION,	HOLDREGISTER_DEVICEPARAM_PARAM_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"ParamVer"},
	{(uint8_t*)"结构体大小",	0,	COM_NUM_DEVICEPARAM_STRUCT_SIZE,	HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"StructSize"},
	{(uint8_t*)"魔术字",	0,	COM_NUM_DEVICEPARAM_MAGIC,	HOLDREGISTER_DEVICEPARAM_MAGIC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Magic"},

/* CRC */
	{(uint8_t*)"参数CRC32",	0,	COM_NUM_DEVICEPARAM_CRC,	HOLDREGISTER_DEVICEPARAM_CRC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"ParamCRC"},
};


static const int holdValueAmount = sizeof(holdValue) / sizeof(holdValue[0]);

/* 根据操作获取当前保持寄存器具体信息的索引 */
int getHoldValueNum(int operanum)
{
    int i;
    for(i = 0;i < holdValueAmount;i++)
    {
        if(operanum == holdValue[i].operanum)
            break;
    }
    if(i >= holdValueAmount)
        return -1;
    else
        return i;
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


void InputValueInit(void)
{
	g_measurement.oil_measurement.oil_level== UNVALID_LEVEL;
	g_measurement.water_measurement.water_level = LEVEL_DOWNLIMITWATER;
	g_measurement.device_status.device_state = STATE_INIT;
	g_measurement.single_point_monitoring.density = UNVALID_DENSITY;
	g_measurement.single_point_monitoring.temperature = UNVALID_TEMPERATURE_WIRELESS;
}
/* 解析03功能码保持寄存器数据 */
void AnalysisHoldRegister(void)
{
    int index = 0;

    for(index = 0;index < holdValueAmount;index++)
    {
        if(holdValue[index].data_type == TYPE_INT)
        {
            holdValue[index].val = ywj_hold_analysis_data(holdValue[index].startadd,holdValue[index].rgstcnt);
            holdValue[index].val += holdValue[index].offset;
//            printf("Hold Reg %s: %d\n",holdValue[index].name,holdValue[index].val);
        }
        else if(holdValue[index].data_type == TYPE_FLOAT)
        {
            union utof tmp_f;
            tmp_f.u = ywj_hold_analysis_data(holdValue[index].startadd,holdValue[index].rgstcnt);
            tmp_f.f *= pow(10,holdValue[index].point);
            holdValue[index].val = tmp_f.f;
        }
        else if(holdValue[index].data_type == TYPE_DOUBLE)
        {
            union utod tmp_d;
            tmp_d.u[1] = ywj_hold_analysis_data(holdValue[index].startadd,holdValue[index].rgstcnt / 2);
            tmp_d.u[0] = ywj_hold_analysis_data(holdValue[index].startadd + 2,holdValue[index].rgstcnt / 2);
            tmp_d.d *= pow(10,holdValue[index].point);
            holdValue[index].val = tmp_d.d;
        }
    }
}

/* 单个数据解析 - 保持寄存器 */
static int ywj_hold_analysis_data(int startadd,int rgscnt)
{
    int value = 0;
    int i;

//    startadd *= 2;
    //解析数据
    for(i = 0;i < rgscnt;i++)
    {
        value <<= 16;
        value += HoldingRegisterArray[startadd + i];
    }
    return value;
}


