#include "modbus_agreement.h"
#include "usart.h"
#include "display_tankopera.h"
#include <math.h>
#include "communicate.h"
#include "system_parameter.h"

int cnt_commutoCPU2 = COMMU_ERROR_MAX;



struct HoldRegisterData holdValue[] = {              /* 保持寄存器数据 */
    /* 名称						数据	操作码						起始地址	寄存器数量	是否范围检查	最小值	最大值	单位	小数点个数	偏移量	写权限	数据类型	要显示的位数	隐藏信息数组	英文
 */
	{(uint8_t*)"工作模式密码",	0,	COM_NUM_WORKPATTER_PASSWORD,	HOLDREGISTER_WORKPATTER_PASSWORD,	6,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WorkModePwd"},
	{(uint8_t*)"固定点测量位置",	0,	COM_NUM_SINGLE_POINT,	HOLDREGISTER_SP_POSITION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FixPtMeasPos"},
	{(uint8_t*)"固定点监测位置",	0,	COM_NUM_SP_TEST,	HOLDREGISTER_SPT_POSITION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FixPtMonPos"},
	{(uint8_t*)"密度分布测高度",	0,	COM_NUM_SPREADPOINTS,	HOLDREGISTER_SRREAD_POSITION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"DensityMeasHt"},
	{(uint8_t*)"综合测是否测罐底",	0,	COM_NUM_SYNTHETIC_BOTTOM_FREE,	HOLDREGISTER_SYNTHETIC_BOTTOM_FREE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"IntgMeasTankBot"},
	{(uint8_t*)"综合测是否测水位",	0,	COM_NUM_SYNTHETIC_WATER_FREE,	HOLDREGISTER_SYNTHETIC_WATER_FREE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"IntgMeasWaterLvl"},
	{(uint8_t*)"综合测是否固定点",	0,	COM_NUM_SYNTHETIC_SINGLEPOINT_FREE,	HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"IntgUseFixPt"},
	{(uint8_t*)"分布测模式",	0,	COM_NUM_SPREAD_STATE_FREE,	HOLDREGISTER_SPREAD_STATE_FREE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"DistMeasMode"},
	{(uint8_t*)"分布测测量点数",	0,	COM_NUM_SPREAD_NUM_FREE,	HOLDREGISTER_SPREAD_NUM_FREE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"DistMeasPts"},
	{(uint8_t*)"分布测量密度点间距",	0,	COM_NUM_SPREAD_DISTANCE_FREE,	HOLDREGISTER_SPREAD_DISTANCE_FREE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DistDensPtDist"},
	{(uint8_t*)"顶密距液位",	0,	COM_NUM_SPREAD_TOPLIMIT_FREE,	HOLDREGISTER_SPREAD_TOPLIMIT_FREE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"TopDensToLvl"},
	{(uint8_t*)"底密距罐底",	0,	COM_NUM_SPREAD_FLOORLIMIT_FREE,	HOLDREGISTER_SPREAD_FLOORLIMIT_FREE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BotDensToBot"},
	{(uint8_t*)"综合测单点位置",	0,	COM_NUM_SPSYNTHETIC_POSITION,	HOLDREGISTER_SPSYNTHETIC_POSITION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"IntgSinglePos"},
	{(uint8_t*)"每米测方向",	0,	COM_NUM_MEASREMENT_METER,	HOLDREGISTER_MEASREMENT_METER,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"MeterMeasDir"},
	{(uint8_t*)"区间密度测量点数",	0,	COM_NUM_INTERVAL_POINT,	HOLDREGISTER_INTERVAL_POINT,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"SectDensPts"},
	{(uint8_t*)"区间密度测量方向",	0,	COM_NUM_INTERVAL_DIREDION,	HOLDREGISTER_INTERVAL_DIREDION,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"SectDensDir"},
	{(uint8_t*)"区间测量液位A",	0,	COM_NUM_INTERVAL_OIL_A,	HOLDREGISTER_INTERVAL_OIL_A,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SectMeasLvlA"},
	{(uint8_t*)"区间测量液位B",	0,	COM_NUM_INTERVAL_OIL_B,	HOLDREGISTER_INTERVAL_OIL_B,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SectMeasLvlB"},
																
	{(uint8_t*)"罐高",	0,	COM_NUM_TANKHIGHT,	HOLDREGISTER_TANKHIGHT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TankHeight"},
	{(uint8_t*)"标定液位液位值",	0,	COM_NUM_CAL_OIL,	HOLDREGISTER_CALIBRATIONLIQUIDLEVEL,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalibLvlVal"},
	{(uint8_t*)"综合测量顺序",	0,	COM_NUM_SRREAD_MEASURETURN,	HOLDREGISTER_SRREAD_MEASURETURN,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"IntgMeasOrder"},
	{(uint8_t*)"综合测量分布模式",	0,	COM_NUM_SPREAD_STATE,	HOLDREGISTER_SPREAD_STATE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"IntgMeasDistMode"},
	{(uint8_t*)"密度分布测量点数",	0,	COM_NUM_SPREAD_NUM,	HOLDREGISTER_SPREAD_NUM,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"DensDistPts"},
	{(uint8_t*)"密度点间距",	0,	COM_NUM_SPREAD_DISTANCE,	HOLDREGISTER_SPREAD_DISTANCE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"DensPtDist"},
	{(uint8_t*)"顶密距液位",	0,	COM_NUM_SPREAD_TOPLIMIT,	HOLDREGISTER_SPREAD_TOPLIMIT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"TopDensToLvl"},
	{(uint8_t*)"底密距罐底",	0,	COM_NUM_SPREAD_FLOORLIMIT,	HOLDREGISTER_SPREAD_FLOORLIMIT,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BotDensToBot"},
	{(uint8_t*)"分层加测阈值点A",	0,	COM_NUM_THRESHOLD_A,	HOLDREGISTER_THRESHOLD_A,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"LayerExtraThA"},
	{(uint8_t*)"分层加测阈值点B",	0,	COM_NUM_THRESHOLD_B,	HOLDREGISTER_THRESHOLD_B,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"LayerExtraThB"},
	{(uint8_t*)"国标测点数阈值",	0,	COM_NUM_THRESHOLD_STANDARD,	HOLDREGISTER_THRESHOLD_STANDARD,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"StdMeasPtsTh"},
	{(uint8_t*)"水位测量方式",	0,	COM_NUM_WATER_IS_REAL_HIGH,	HOLDREGISTER_WATER_IS_REAL_HIGH,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"WaterLvlMode"},
	{(uint8_t*)"实高水位修正值",	0,	COM_NUM_REAL_WATER_LEVEL_CORRECT,	HOLDREGISTER_REAL_WATER_LEVEL_CORRECT,	1,	false,	0,	0,	NULL,	1,	-1000,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ActHeightCorr"},
	{(uint8_t*)"综合测是否更罐高",	0,	COM_NUM_IF_REFRESH_TANKHIGH,	HOLDREGISTER_IF_REFRESH_TANKHIGH,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"UpdTankHeight"},
	{(uint8_t*)"实高最大偏差阈值",	0,	COM_NUM_REAL_TANKHIGH_MAX_DIFF,	HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF,	1,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"MaxDevThresh"},
	{(uint8_t*)"液位测量方式",	0,	COM_NUM_LEVEL_MEASURE_METHOD,	HOLDREGISTER_LEVEL_MEASURE_METHOD,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"LvlMeasMode"},
	{(uint8_t*)"综合测单温度模式",	0,	COM_NUM_TEMPERATURE_MODE,	HOLDREGISTER_TEMPERATURE_MODE,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"IntgSingleTempMode"},
	{(uint8_t*)"单温度模式密度值",	0,	COM_NUM_TEMPERATURE_MODE_D,	HOLDREGISTER_TEMPERATURE_MODE_D,	1,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SingleTempDensVal"},
	{(uint8_t*)"水位与零点最小距",	0,	COM_NUM_WATER_ZERO_MIN_DISTANCE,	HOLDREGISTER_WATER_ZERO_MIN_DISTANCE,	1,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"MinLvlZeroDist"},
	{(uint8_t*)"上电找液位",	0,	COM_NUM_IF_FINDOIL_POWERON,	HOLDREGISTER_IF_FINDOIL_POWERON,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"AutoLvlSearch"},
	{(uint8_t*)"测密前提出时间",	0,	COM_NUM_DENSITY_TIME,	HOLDREGISTER_DENSITY_TIME,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	2,	NULL,	(uint8_t*)"PreMeasDelay"},
	{(uint8_t*)"运行距离",	0,	COM_NUM_RUNUP,	HOLDREGISTER_RUNTODISTANCE,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"RunDistance"},
	{(uint8_t*)"运行距离",	0,	COM_NUM_RUNDOWN,	HOLDREGISTER_RUNTODISTANCE,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"RunDistance"},
	{(uint8_t*)"零点编码器圈数",	0,	COM_NUM_ZEROCIRCLE,	HOLDREGISTER_ZEROCIRCLE,	1,	false,	0,	0,	NULL,	0,	-32768,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"ZeroEncTurns"},
	{(uint8_t*)"零点编码器编码值",	0,	COM_NUM_ZEROANGLE,	HOLDREGISTER_ZEROANGLE,	1,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"ZeroEncVal"},
	{(uint8_t*)"导轮周长",	0,	COM_NUM_GIRTH_USELESS,	HOLDREGISTER_GIRTH_USELESS,	1,	false,	0,	0,	NULL,	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"PulleyCirc"},
	{(uint8_t*)"减速比",	0,	COM_NUM_REDUCTION_RATIO,	HOLDREGISTER_REDUCTION_RATIO,	1,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	2,	NULL,	(uint8_t*)"GearRatio"},
	{(uint8_t*)"传感器类型",	0,	COM_NUM_TYPEOFSENSOR,	HOLDREGISTER_TYPEOFSENSOR,	1,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	2,	NULL,	(uint8_t*)"SensorType"},
	{(uint8_t*)"传感器头长度",	0,	COM_NUM_LENGTHOFSENSOR,	HOLDREGISTER_LENGTHOFSENSOR,	1,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"SensorHeadLen"},
	{(uint8_t*)"盲区",	0,	COM_NUM_FADEZERO,	HOLDREGISTER_FADEZERO,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BlindZone"},
	{(uint8_t*)"程序版本",	0,	COM_NUM_SOFTOFVERSION,	HOLDREGISTER_SOFTOFVERSION,	1,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"FWVersion"},
	{(uint8_t*)"修正的高度",	0,	COM_NUM_CORRECTION_OIL,	HOLDREGISTER_CORRECTION_OIL,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CorrHeight"},
	{(uint8_t*)"标定的高度",	0,	COM_NUM_CAL_OIL,	HOLDREGISTER_CALIBRATIONLIQUIDLEVEL,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CorrHeight"},
	{(uint8_t*)"导轮周长",	0,	COM_NUM_GIRTH_YITI,	HOLDREGISTER_GIRTH_YITI,	2,	false,	0,	0,	NULL,	3,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"PulleyCirc"},
	{(uint8_t*)"液水传感距离差",	0,	COM_NUM_LEVELTOWATER_HIGH,	HOLDREGISTER_LEVELTOWATER_HIGH,	1,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"LiqWaterDistDiff"},
	{(uint8_t*)"测罐底最大下行值",	0,	COM_NUM_MAXDOWN_DIS,	HOLDREGISTER_MAXDOWN_DIS,	1,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"MaxDownTankMeas"},
	{(uint8_t*)"综合测是否测罐底",	0,	COM_NUM_SYNTHETIC_BOTTOM,	HOLDREGISTER_SYNTHETIC_BOTTOM,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"IntgMeasTankBot"},
	{(uint8_t*)"综合测是否测水位",	0,	COM_NUM_SYNTHETIC_WATER,	HOLDREGISTER_SYNTHETIC_WATER,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"IntgMeasWaterLvl"},
	{(uint8_t*)"综合测是否固定点",	0,	COM_NUM_SYNTHETIC_SINGLEPOINT,	HOLDREGISTER_SYNTHETIC_SINGLEPOINT,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"IntgUseFixPt"},
	{(uint8_t*)"温度有效位数",	0,	COM_NUM_NUMOFDECIMALS,	HOLDREGISTER_NUMOFDECIMALS,	1,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"TempValidDigits"},
	{(uint8_t*)"磁通量T",	0,	COM_NUM_MAGNETIC_T,	HOLDREGISTER_TEMCORRECTCALUE,	1,	false,	0,	0,	NULL,	1,	-1000,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"MAGNETIC_T"},
	{(uint8_t*)"磁通量D",	0,	COM_NUM_MAGNETIC_D,	HOLDREGISTER_DENSITYCORRECTCALUE,	1,	false,	0,	0,	NULL,	1,	-10000,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"MAGNETIC_D"},
	{(uint8_t*)"传感器编号",	0,	COM_NUM_DEVICENUM,	HOLDREGISTER_DEVICENUM,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"SensorID"},
	{(uint8_t*)"综合测单点测位置",	0,	COM_NUM_OIL_MEASUR_POSITION,	HOLDREGISTER_OIL_MEASUR_POSITION,	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"IntgSinglePtPos"},

    
    
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





