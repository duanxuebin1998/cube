#ifndef _DSM_SYSTEM_PARAMETER_H
#define _DSM_SYSTEM_PARAMETER_H
#include "main.h"
#include "system_parameter.h"
extern int LevelCorrection;
extern int TankHigh_LevelCorrection;

extern char SoftOfversion6;
#define SOFTOFVERSION_0 1213 // 程序版本
#define SOFTOFVERSION_1 'P'	 // 程序版本
#define SOFTOFVERSION_2 'B'	 // 程序版本
#define SOFTOFVERSION_3 'C'	 // 程序版本
#define SOFTOFVERSION_4 'D'	 // 程序版本
#define SOFTOFVERSION_5 'E'	 // 程序版本
// 第六位是数据定义
#define PARAMETER_ERROR -1
#define PARAMETER_WRITE_FAIL -2


#define UNIT_CONVER_MM 10		   // 过程高度数据与mm单位转换关系
#define GIRTH_CONVER_MM_TIMES 1000 // 导轮周长到mm之间的倍数关系
#define ALARM_ANGLECHANGE 30.0
#define ALARM_ANGLECHANGE1 10.0	  // V1.106 dq0911
#define RESVALUE_WATER 1.5		  // 2018.01.18W:由2.0改为1.5
#define RESVALUE_ZERO_UP 1.72	  // V1.119
#define RESVALUE_ZERO_DOWN 1.56	  // V1.119
#define FIND_MAGZERO_UP 500		  // 磁性原理找到零点后上行的距离
#define RESVALUE_LEMES_WATER 1.7  // 到达下限3次前阈值
#define RESVALUE_LEMES_WATERK 3.0 //
#define RESVALUE_LEMES_WATERL 3.0 //
#define TANKBOTTOM_ANGLECHANGE 1.0
#define ANGLECHANGE_TANKBOTTOM 5.0

//#define ADDRESS_AREA0_START 0x00000				 // EEPROM区域 0 起始地址
//#define ADDRESS_BEGAIN 0x00000					 // 开始
//#define ADDRESS_TANKHIGH 0x00004				 // 罐高
//#define ADDRESS_SRREAD_MEASURETURN 0x00008		 // 分布测量顺序
//#define ADDRESS_SPREAD_STATE 0x0000C			 // 分布测量模式
//#define ADDRESS_SPREAD_NUM 0x00010				 // 分布测量数量
//#define ADDRESS_SPREAD_DISTANCE 0x00014			 // 分布测量间距
//#define ADDRESS_SPREAD_TOPLIMIT 0x00018			 // 分布测量上限（距液面）
//#define ADDRESS_SPREAD_FLOORLIMIT 0x0001C		 // 分布测量下限（距罐底）
//#define ADDRESS_SPREAD_1POINTTHRESHOLD 0x00020	 // 分布测量一个点时的液位阈值
//#define ADDRESS_SPREAD_5POINTTHRESHOLD 0x00024	 // 分布测量五个点时的液位阈值
//#define ADDRESS_SPREAD_OTHERNUM 0x00028			 // 分布测量其他点数时的液位阈值
//#define ADDRESS_SPREAD_FIXEDDISTANCE 0x0002C	 // 固定间距分布测量的间距
//#define ADDRESS_SPREAD_FIXEDTOP 0x00030			 // 固定间距分布测量最高点距液面间距
//#define ADDRESS_SPREAD_FIXEDBASE 0x00034		 // 固定间距分布测量最低点距罐底间距
//#define ADDRESS_ZEROCIRCLE 0x00038				 // 预设零点编码圈数
//#define ADDRESS_ZEROANGLE 0x0003C				 // 预设零点编码角度
//#define ADDRESS_GIRTH 0x00040					 // 导论周长
//#define ADDRESS_REDUCTIONRATIO 0x00044			 // 减速比
//#define ADDRESS_TYPEOFSENSOR 0x00048			 // 传感器类型
//#define ADDRESS_LENGTHOFSENSOR 0x0004C			 // 传感器长度
//#define ADDRESS_FADEZERO 0x00050				 // 盲区
//#define ADDRESS_ANGLEINAIR 0x00054				 // 空气中的角度Y
//#define ADDRESS_ANGLEXAIR 0x00058				 // 液体中的角度X
//#define ADDRESS_LEVELTOWATER 0x0005C			 // 水位传感器到液位传感器距离
//#define ADDRESS_MAXDOWN 0x00060					 // 水位和罐底测量时最大下行位置
//#define ADDRESS_OILTHRESHOLD 0X00064			 // 液位取得的阈值
//#define ADDRESS_OILADD_A 0x00068				 // 分层加测阈值点A
//#define ADDRESS_OILADD_B 0x0006C				 // 分层加测阈值点B
//#define ADDRESS_OIL_STANDARD 0x00070			 // 国标密度测量阈值
//#define ADDRESS_OIL_MEASUR_POSITION 0x00074		 // 综合指令密度发油口位置
//#define ADDRESS_WATER_IS_REAL_HIGH 0x00078		 /*水位是否为实高测量*/
//#define ADDRESS_REAL_WATER_LEVEL_CORRECT 0x0007C /*实高水位测量修正值*/
//#define ADDRESS_IF_REFRESH_TANKHIGH 0x00080		 /*测量是否更新罐高*/
//#define ADDRESS_REAL_TANKHIGH_MAX_DIFF 0x00084	 /*实高最大偏差阈值*/
//#define ADDRESS_REAL_TANKHIGH_ORIGIN 0x00088	 /*初始实高*/
//#define ADDRESS_REAL_TANKHIGH_NOW 0x0008C		 /*当前实高*/
//#define ADDRESS_LEVEL_MEASURE_METHOD 0x00090	 /*液位测量方式*/
//#define ADDRESS_TEMPERATURE_MODE 0x00094		 /*单温度模式*/
//#define ADDRESS_TEMPERATURE_MODE_D 0x00098		 /*温度模式密度值*/
///*************/
//#define ADDRESS_AREA1_START 0x01000				  // EEPROM区域 1 起始地址
//#define ADDRESS_TYPEOFEINDUCTION 0x01000		  // 霍尔器件类型
//#define ADDRESS_NUMOFEINDUCTION 0x01004			  // 霍尔器件数量
//#define ADDRESS_USEFULNUMOFEINDUCTION 0x01008	  // 霍尔器件作用数量
//#define ADDRESS_DIATANCEFROMEINDUCTION 0x0100C	  // 脱离配重步进数
//#define ADDRESS_CORRECTIONFACTOR 0x01010		  // 修正系数步进数
//#define ADDRESS_FREQUENCETHRESHOLD 0x01014		  // 频率阈值
//#define ADDRESS_RANGEOFFREQUENCETHRESHOLD 0x01018 // 频率阈值范围
//#define ADDRESS_RANGEOFPOSITION 0x0101C			  // 位置范围
//#define ADDRESS_RANGEOFFREQUENCE 0x01020		  // 频率稳定范围
//#define ADDRESS_RANGEOFTEMPERATURE 0x01024		  // 温度稳定范围
//#define ADDRESS_NUMOFSTABLEHITS 0x01028			  // 温度平衡是需要的点数
//#define ADDRESS_NUMOFHITS 0x0102C				  // 第一点平衡判断前的有效时间
//#define ADDRESS_DEVICENUM 0x01030				  // 传感器系数      v0.013
//#define ADDRESS_NUMOFDECIMALS 0x01034			  // 温度的有效点数默认是2位
//#define ADDRESS_TEMCORRECTCALUE 0x01038			  // 温度的修正系数 温度+(修正-1000)
//#define ADDRESS_DENSITYCORRECTCALUE 0x0103C		  // 密度的修正系数 密度+(修正-10000)
//#define ADDRESS_SYNTHETIC_BOTTOM 0x01040		  // 综合指令是否需要测罐底默认为0不测
//#define ADDRESS_SYNTHETIC_WATER 0x01044			  // 综合指令是否需要测水位默认为0不测
//#define ADDRESS_SYNTHETIC_SINGLEPOINT 0x01048	  // 综合指令是否需要测水位默认为0不测
//#define ADDRESS_D_CORRECTION_TEM1 0x0104C		  // 密度分段修正温度区间值1
//#define ADDRESS_D_CORRECTION_TEM2 0x01050		  // 密度分段修正温度区间值2
//#define ADDRESS_D_CORRECTION_TEM3 0x01054		  // 密度分段修正温度区间值3
//#define ADDRESS_D_CORRECTION_TEM4 0x01058		  // 密度分段修正温度区间值4
//#define ADDRESS_D_CORRECTION_TEM5 0x0105C		  // 密度分段修正温度区间值5
//#define ADDRESS_D_CORRECTION_TEM6 0x01060		  // 密度分段修正温度区间值6
//#define ADDRESS_D_CORRECTION_TEM7 0x01064		  // 密度分段修正温度区间值7
//#define ADDRESS_D_CORRECTION_TEM8 0x01068		  // 密度分段修正温度区间值8
//#define ADDRESS_D_CORRECTION_TEM9 0x0106C		  // 密度分段修正温度区间值9
//#define ADDRESS_D_CORRECTION_TEM10 0x01070		  // 密度分段修wo正温度区间值10
//#define ADDRESS_D_CORRECTION_TEM11 0x01074		  // 密度分段修正温度区间值11
//#define ADDRESS_D_CORRECTION_1 0x01078			  // 密度修正值1
//#define ADDRESS_D_CORRECTION_2 0x0107C			  // 密度修正值2
//#define ADDRESS_D_CORRECTION_3 0x01080			  // 密度修正值3
//#define ADDRESS_D_CORRECTION_4 0x01084			  // 密度修正值4
//#define ADDRESS_D_CORRECTION_5 0x01088			  // 密度修正值5
//#define ADDRESS_D_CORRECTION_6 0x0108C			  // 密度修正值6
//#define ADDRESS_D_CORRECTION_7 0x01090			  // 密度修正值7
//#define ADDRESS_D_CORRECTION_8 0x01094			  // 密度修正值8
//#define ADDRESS_D_CORRECTION_9 0x01098			  // 密度修正值9
//#define ADDRESS_D_CORRECTION_10 0x0109C			  // 密度修正值10
//#define ADDRESS_D_CORRECTION_11 0x010A0			  // 密度修正值11
//#define ADDRESS_D_CORRECTION_12 0x010A4			  // 密度修正值12
//#define ADDRESS_WATER_ZERO_MIN_DISTANCE 0x010A8  /*水位与零点最小距离*/
//#define ADDRESS_IF_FINDOIL_POWERON 0x010AC       /*上电是否自动找液位*/
//#define ADDRESS_DENSITY_TIME 0x010B0	             /*密度测量前提出油面时间*/
///*************/
//
//#define ADDRESS_AREA2_START 0x02000	  // EEPROM区域 2 起始地址
//#define ADDRESS_SOFTOFVERSION 0x02000 // 程序版本
//#define ADDRESS_K0 0x02008			  // K0
//#define ADDRESS_K1 0x02010			  // K1
//#define ADDRESS_K2 0x02018			  // K2
//#define ADDRESS_K3 0x02020			  // K3
//#define ADDRESS_K18 0x02028			  // K18
//#define ADDRESS_K19 0x02030			  // K19
//#define ADDRESS_DetT 0x02038		  // Det
//#define ADDRESS_A2 0x02040			  // A2
//#define ADDRESS_A1 0x02048			  // A1
//#define ADDRESS_A0 0x02050			  // A0
//#define ADDRESS_K4 0x02058			  // K4
//#define ADDRESS_K5 0x02060			  // K5
//#define ADDRESS_V1 0x02068			  // v1
//#define ADDRESS_V2 0x02070			  // v2
//#define ADDRESS_V3 0x02078			  // v3
//
//#define NUMOFAREA0 39 // 区域0 地址数
//#define NUMOFAREA1 45 // 区域1 地址数
//#define NUMOFAREA2 32 // 区域2 地址数

typedef struct // 不需要写入EEPROM的测量参数
{
	int Density_Single_St_High;		   // 单点监测的高度
	int Density_Single_Mm_High;		   // 单点测量的高度
	int UpOrDown_Need_Run;			   // 上行或者下行的高度
	int Calibration_Oil_Level;		   // 修正液位的高度
	int Synthetic_Need_FindBottom;	   // 综合指令是否需要测量罐底
	int Synthetic_Need_FindWater;	   // 综合指令是否需要测量水位
	int Synthetic_Need_SinglePoint;	   // 综合指令是否需要测量单点密度
	int SpredState;					   // 分布测量模式
	int SpredNum;					   // 分布测量数量
	int SpredDistance;				   // 分布测量间距
	int SpredTopLimit;				   // 分布测量上限（距液面）
	int SpredFloorLimit;			   // 分布测量下限（距罐底）
	int Density_Single_Synthetic_High; // 综合指令固定点测量的位置
	int Measrement_Meter;			   // 密度每米测量方向
	int Interval_Point;				   // 密度液位区间密度测量点数
	int Interval_Diredion;			   // 密度液位区间测量方向
	int Interval_Oil_A;				   // 密度液位区间测量液位阈值点A
	int Interval_Oil_B;				   // 密度液位区间测量液位阈值点B
} System_Measure_Parameter;
extern System_Measure_Parameter System_measure_parameter;

// v1.1252021.06.17液位分段修正
/*
 定义一个修正区间，10个数组可以将一个罐分为9个区间，数组的第一个变量
 */
typedef struct {
	int Level_Correct_Section[10];
	int Level_Correct_Data[10];

} System_Measure_Correct;
extern System_Measure_Correct System_measure_correct;

typedef struct {
	int BeginBlank;			  // 开头空白
	int TankHigh;			  // 罐高
	int SpredMeasureTurn;	  // 分布测量顺序
	int SpredState;			  // 分布测量模式
	int SpredNum;			  // 分布测量数量
	int SpredDistance;		  // 分布测量间距
	int SpredTopLimit;		  // 分布测量上限（距液面）
	int SpredFloorLimit;	  // 分布测量下限（距罐底）
	int Spred1PointThreshold; // 分布测量一个点时的液位阈值
	int Spred5PointThreshold; // 分布测量五个点时的液位阈值
	int SpredOtherNum;		  // 分布测量其他点数时的液位阈值
	int SpredFixedDistance;	  // 固定间距分布测量的间距
	int SpredFixTop;		  // 固定间距分布测量最高点距液面间距
	int SpredFixBase;		  // 固定间距分布测量最低点距罐底间距
	int ZeroCircle;			  // 预设零点编码圈数
	int ZeroAngle;			  // 预设零点编码角度
	int Girth;				  // 导论周长

	int ReductionRatio;						// 减速比
	int TypeOfSensor;						// 传感器类型
	int LengthOfSensor;						// 传感器长度
	int FadeZero;							// 盲区
	int gyro_angleinair;					// 空气中的角度
	int gyro_angleinoil;					// 液体中的角度
	int Dis_Leveltowater;					// 液位传感器到水位传感器的距离
	int Dis_Maxdown;						// 最大下行值，寻找水位/罐底时的最大下行
	int Oil_Threshold;						// 液位跟随阈值
	int Oil_Add_A;							// 密度分层测量点A阈值
	int Oil_Add_B;							// 密度分层测量点B阈值
	int Oil_Standard;						// 国标密度测量阈值
	int Oil_Measure_Position;				// 综合指令发油口测量位置
	int water_is_realhigh;					// 水位是否为实高测量
	int real_water_level_correct;			// 实高水位修正值
	int if_refresh_tankhigh;				// 实高测量是否修正罐高
	int real_tankhigh_max_diff;				// 实高测量最大偏差阈值
	int real_tankhigh_origin;				// 初始实高
	int real_tankhigh_now;					// 当前实高
	int level_masure_method;				// 液位测量方式
	int synthetic_temperature_mode;			// 综合测量单温度模式
	int synthetic_temperature_mode_density; // 综合测量单温度模式密度值

	int TypeOfEinduction;					// 霍尔器件类型
	int NumOfEinduction;					// 霍尔器件数量
	int UsefulNumOfEinduction;				// 霍尔器件作用数量
	int DiatanceFromEinduction;				// 找到磁零点后的上行距离
	int CorrectionFactor;					// 位置修正步进数
	int FrequenceThreshold;					// 频率阈值
	int RangeOfFrequenceThreshold;			// 频率阈值范围
	int RangeOfPosition;					// 位置范围
	int RangeOfFrequence;					// 频率稳定范围
	int RangeOfTemperature;					// 温度稳定范围
	int NumOfStableHits;					// 温度平衡时的点数
	int NumOfHits;							// 第一点等待时间
	int DeviceNum;							// 传感器系数
	int Temperature_Decimals;				// 温度小数点后位数
	int Temperature_Correct;				// 温度修正值correct
	int Density_Correct;					// 密度修正值
	int Synthetic_Need_FindBottom;			// 综合指令是否需要测量罐底
	int Synthetic_Need_FindWater;			// 综合指令是否需要测量水位
	int Synthetic_Need_SinglePoint;			// 综合指令是否需要测量单点密度
	int Density_Correction_Temperature[11]; // 密度分段修正温度区间值
	int Density_Correction_Value[12];		// 密度修正值
	int Water_zero_min_disance;			// 水位与零点最小距离
	int If_findoil_poweron; // 上电是否自动找液位
	int Density_air_time;		// 密度测量提出油面时间

	int SoftOfversion_H; // 程序版本
	int SoftOfversion_L; // 程序版本
	int K0_H;			 // K0_高32位
	int K0_L;			 // K0_低32位
	int K1_H;			 // K1_高32位
	int K1_L;			 // K1_低32位
	int K2_H;			 // K2_高32位
	int K2_L;			 // K2_低32位
	int K3_H;			 // K3_高32位
	int K3_L;			 // K3_低32位
	int K18_H;			 // K18_高32位
	int K18_L;			 // K18_低32位
	int K19_H;			 // K19_高32位
	int K19_L;			 // K19_低32位
	int DET_H;			 // DET_高32位
	int DET_L;			 // DET_低32位
	int A2_H;			 // A2_高32位
	int A2_L;			 // A2_低32位
	int A1_H;			 // A1_高32位
	int A1_L;			 // A1_低32位
	int A0_H;			 // A0_高32位
	int A0_L;			 // A0_低32位
	int K4_H;			 // K4_高32位
	int K4_L;			 // K4_低32位
	int K5_H;			 // K5_高32位
	int K5_L;			 // K5_低32位
	int v1_H;			 // V1_高32位
	int v1_L;			 // V1_低32位
	int v2_H;			 // V2_高32位
	int v2_L;			 // V2_低32位
	int v3_H;			 // V3_高32位
	int v3_L;			 // V3_低32位

} SystemParameter;

union SystemUnion {
	u32 Sysremparameter[116];
	SystemParameter systemparameter;
};
union Coefficient_K {
	double coefficient;
	u32 arr[2];
};
extern union Coefficient_K coefficient_k0;
extern union Coefficient_K coefficient_k1;
extern union Coefficient_K coefficient_k2;
extern union Coefficient_K coefficient_k3;
extern union Coefficient_K coefficient_k18;
extern union Coefficient_K coefficient_k19;
extern union Coefficient_K coefficient_detT;
extern union Coefficient_K coefficient_k4;
extern union Coefficient_K coefficient_k5;

extern union Coefficient_K coefficient_v1;
extern union Coefficient_K coefficient_v2;
extern union Coefficient_K coefficient_v3;

union Coefficient_A {
	double coefficient;
	u32 arr[2];
};
extern union Coefficient_A coefficient_a2;
extern union Coefficient_A coefficient_a1;
extern union Coefficient_A coefficient_a0;

extern union SystemUnion systemunion;

int SystemParameterInit(void);
void SystemParameterSet(void);
int SystemParameterSaveMul(int startadd, int reamount);

#endif
