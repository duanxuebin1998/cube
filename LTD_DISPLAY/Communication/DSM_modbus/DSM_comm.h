#ifndef _DSM_COMM_H__
#define _DSM_COMM_H__

#include "DSM_stateformodbus2.h"
#include "DSM_SlaveModbus_modbus2.h"
#include "main.h"


struct RESULT // V1.116 dq2020.4.2
{
	int Density[102];	  // 视密度
	int Temperature[102]; // 温度
	int Density20[102];	  // 标密
	int Vcf[102];		  // 体积修正系数
	int DensityT[102];
	int Postion[102];
	int Frequence;
	int FrequenceInAir;
	int NumOfPoints;
};

extern struct RESULT Result;

struct EDUCT // V1.116 dq2020.4.2
{
	int Density[102];	  // 视密度
	int Temperature[102]; // 温度
	int Density20[102];	  // 标密
	int Vcf[102];		  // 体积修正系数
	int DensityT[102];
	int Postion[102];
	int Frequence;
	int FrequenceInAir;
	int NumOfPoints;
};

extern struct EDUCT Educt; // V1.116 dq2020.4.2

// #pragma pack (2)
typedef volatile struct
{
	u16 PatternOfWork;			   // 0
	u16 EquipmentState;			   // 1
	u32 EquipmentErrorCode;		   // 2-3
	u16 Measure_TempOnePoint;	   // 4
	u16 Measure_DensityOnePoint;   // 5
	u16 Measure_OnePointState;	   // 6
	u32 Measure_OnePointPosition;  // 7-8
	u16 Measure_Density20OnePoint; // 9
	u32 Measure_VCF20OnePoint;	   // 10-11
	u16 Measure_DensityTOnePoint;  // 12

	u16 Monitor_TempOnePoint;	   // 13
	u16 Monitor_DensityOnePoint;   // 14
	u16 Monitor_OnePointState;	   // 15
	u32 Monitor_OnePointPosition;  // 16-17
	u16 Monitor_Density20OnePoint; // 18
	u32 Monitor_VCF20OnePoint;	   // 19-20
	u16 Monitor_DensityTOnePoint;  // 21

	u16 Spread_TempAverage;				// 22
	u16 Spread_DensityAverage;			// 23
	u16 Spread_TempDensityAverageState; // 24
	u16 Spread_Density20Average;		// 25
	u32 Spread_Vcf20Average;			// 26-27
	u16 Spread_DensityTAverage;			// 28
	u32 Spread_Points;					// 29
	u32 Spread_OilLevel;				// 30-31

	int MeasureOil_Level;	 // 32-33
	int MeasureWater_Level;	 // 34-35
	int MeasureBottom_Level; // 36-37

	u32 Read_AM4096_Position; // 38-39传感器对应的位置，距离罐底；
	u32 PressValue1;		  // 40-41  液体压力 meng
	u32 PressValue3;		  // 42-43	气相压力 meng
	int SelfCheck_Rank;
} Measure_One_Area; // 输入寄存器第一区段数据
// #pragma pack ()
typedef volatile struct // V1.116 dq2020.4.2
{
	u16 Spread_TempAverage;				// 22
	u16 Spread_DensityAverage;			// 23
	u16 Spread_TempDensityAverageState; // 24
	u16 Spread_Density20Average;		// 25
	u32 Spread_Vcf20Average;			// 26-27
	u16 Spread_DensityTAverage;			// 28
	u32 Spread_Points;					// 29
	u32 Spread_OilLevel;				// 30-31
} Measure_Meter_D;

extern Measure_Meter_D Measure_Meter; // V1.116 dq2020.4.2
extern Measure_One_Area Measure_One;

// #pragma pack (2)
typedef struct
{
	u16 Am4096_Setzero_Circle; // 0
	u16 Am4096_Setzero_Angle;  // 1
	u32 Set_Level_TankHigh;	   // 2-3
	u16 Am4096_Circle;		   // 4
	u16 Am4096_Angle;		   // 5
	u32 Am4096_Position;	   // 6-7
	u32 Sensor_Fre;			   // 8-9
	u16 Sensor_Temp;		   // 10
	u16 Hall_State;			   // 11
	u32 Air_Fre;			   // 12-13
	u16 Amplitude;			   // 14
	short X_Angele;			   // 15
	short Y_Angele;			   // 16
	u16 Water_Voltage;		   // 17
} Measure_Debug_Area;
// #pragma pack ()
extern Measure_Debug_Area measure_debug_area;

/*****************************命令*****************************************/
#define COMMAND_NONE 0
#define COMMAND_BACKZERO_START 1					// 开始回零点
#define COMMAND_BACKZERO_STOP 2						// 停止回零点
#define COMMAND_FINDZERO_START 3					// 开始找零点
#define COMMAND_FINDZERO_STOP 4						// 停止找零点
#define COMMAND_SINGLEPOINT_START 5					// 开始单点测量
#define COMMAND_SINGLEPOINT_STOP 6					// 停止单点测量
#define COMMAND_SPTEST_START 7						// 开始单点监测
#define COMMAND_SPTEST_STOP 8						// 停止单点监测
#define COMMAND_SPREADPOINTS_START 9				// 开始分布测量（带液位高度）
#define COMMAND_SPREADPOINTS_STOP 10				// 停止分布测量（带液位高度）
#define COMMAND_SPREADPOINTSAI_START 11				// 开始自动分布测量
#define COMMAND_SPREADPOINTSAI_STOP 12				// 停止自动分布测量
#define COMMAND_CALOIL_START 13						// 开始标定液位
#define COMMAND_CALOIL_STOP 14						// 停止标定液位
#define COMMAND_SETWORKPATTER 15					// 设置工作模式
#define COMMAND_READPARAMETER 16					// 读取参数
#define COMMAND_RUNUP 17							// 上行
#define COMMAND_RUNDOWN 18							// 下行
#define COMMAND_SETZEROCIRCLE 19					// 设置零点圈数
#define COMMAND_SETZEROANGLE 20						// 设置零点圈数
#define COMMAND_RESTOREFACTORYSETTING 21			// 恢复出厂设置
#define COMMAND_BACKUPFILE 22						// 备份配置文件
#define COMMAND_RESTORYFILE 23						// 恢复配置文件
#define COMMAND_FINDOIL_START 24					// 开始寻找液位
#define COMMAND_FINDOIL_STOP 25						// 停止寻找液位
#define COMMAND_FINDWATER_START 26					// 开始寻找水位
#define COMMAND_FINDWATER_STOP 27					// 停止寻找水位
#define COMMAND_FINDBOTTOM_START 28					// 开始寻找罐底
#define COMMAND_FINDBOTTOM_STOP 29					// 停止寻找罐底
#define COMMAND_FORCEZERO_START 30					// 开始设置零点
#define COMMAND_FORCEZERO_STOP 31					// 停止设置零点
#define COMMAND_ONTANKOPERATION_BACKZERO 32			// 罐上操作回零点中
#define COMMAND_ONTANKOPERATION_RUN_BACKZEROOK 33	// 罐上操作完成回零点
#define COMMAND_ONTANKOPERATION_RUN_BEGAINUP 34		// 罐上操作上行
#define COMMAND_ONTANKOPERATION_RUN_UPCOMPLATE 35	// 罐上操作上行完成
#define COMMAND_ONTANKOPERATION_RUN_BEGAINDOWN 36	// 罐上操作下行
#define COMMAND_ONTANKOPERATION_RUN_DOWNCOMPLATE 37 // 罐上操作下行完成
#define COMMAND_SYNTHETIC_START 38					// 综合指令开始synthetic instruct 2019.08.07V1.105
#define COMMAND_SYNTHETIC_STOP 39					// 综合指令停止synthetic instruct 2019.08.07V1.105
#define COMMAND_METERDENSITY_START 40				// 开始密度每米测量V1.116 dq2020.4.2
#define COMMAND_METERDENSITY_STOP 41				// 停止密度每米测量V1.116 dq2020.4.2
#define COMMAND_INTERVALDENSITY_START 42			// 开始液位区间测量V1.116 dq2020.4.2
#define COMMAND_INTERVALDENSITY_STOP 43				// 停止液位区间测量V1.116 dq2020.4.2

#define COMMAND_UNKNOW 44 // 未知指令

#define DSM_STATE_SWITCH 0x00000002 // 状态切换



/*****************************状态*****************************************/
#define DSM_STATE_STANDBY 0x0000				   // 待机状态
#define DSM_STATE_INIT 0x0001					   // 初始化状态
#define DSM_STATE_BACKZEROING 0x0002			   // 回零点中
#define DSM_STATE_FINDZEROING 0x0010			   // 标定零点中
#define DSM_STATE_SINGLEPOINTING 0x0011			   // 单点测量中
#define DSM_STATE_RUNTOPOINTING 0x0012			   // 运行到测量点中
#define DSM_STATE_SPREADPOINTING 0x0013			   // 分布测量中
#define DSM_STATE_AI_SPREADPOINTING 0x0014		   // 无参分布测量中
#define DSM_STATE_CALIBRATIONOILING 0x0015		   // 标定液位
#define DSM_STATE_READPARAMETERING 0x0016		   // 读取参数中
#define DSM_STATE_RUNUPING 0x0017				   // 向上运行中
#define DSM_STATE_RUNDOWNING 0x0018				   // 向下运行中
#define DSM_STATE_SETZEROCIRCLING 0x0019		   // 设置零点编码值
#define DSM_STATE_SETZEROANGLING 0x001A			   // 设置零点编码值
#define DSM_STATE_EFACTORYSETTING_RESTORING 0x001B // 恢复出厂设置中
#define DSM_STATE_BACKUPING 0x001C				   // 备份配置文件中
#define DSM_STATE_RESTORYING 0x001D				   // 恢复配置文件中
#define DSM_STATE_FINDOIL 0x001E				   // 寻找液位中
#define DSM_STATE_FINDWATER 0x001F				   // 寻找水位中
#define DSM_STATE_FINDBOTTOM 0x0020				   // 寻找罐底中
#define DSM_STATE_FORCEZERO 0x0021				   // 设置电机零点中
#define DSM_STATE_ONTANKOPRATIONING 0x0022		   // 罐上仪表操作中
#define DSM_STATE_SYNTHETICING 0x0023			   // 综合指令synthetic instruct 2019.08.07V1.105
#define DSM_STATE_METER_DENSITY 0x0024			   // 密度每米测量中 V1.116 dq2020.4.2
#define DSM_STATE_INTERVAL_DENSITY 0x0025		   // 液位区间测量中 V1.116 dq2020.4.2

#define DSM_STATE_FINDZEROOVER 0x8010				// 标定零点完成
#define DSM_STATE_SINGLEPOINTOVER 0x8011			// 单点测量完成
#define DSM_STATE_SPTESTING 0x8012					// 正在单点检测
#define DSM_STATE_SPREADPOINTOVER 0x8013			// 分布测量完成
#define DSM_STATE_AI_SPREADPOINTOVER 0x8014			// 无参分布测量完成
#define DSM_STATE_FINDOILOVER 0x8015				// 标定液位完成
#define DSM_STATE_READPARAMETEROVER 0x8016			// 读取参数完成
#define DSM_STATE_RUNUPOVER 0x8017					// 向上运行完成
#define DSM_STATE_RUNDOWNOVER 0x8018				// 向下运行完成
#define DSM_STATE_SETZEROCIRCLOVER 0x8019			// 设置零点编码值
#define DSM_STATE_SETZEROANGLOVER 0x801A			// 设置零点编码值
#define DSM_STATE_EFACTORYSETTING_RESTOROVER 0x801B // 恢复出厂设置完成
#define DSM_STATE_BACKUPOVER 0x801C					// 备份配置文件完成
#define DSM_STATE_RESTORYOVER 0x801D				// 恢复配置文件完成
#define DSM_STATE_FLOWOIL 0x801E					// 液位跟随中
#define DSM_STATE_FINDWATER_OVER 0x801F				// 寻找水位中
#define DSM_STATE_FINDBOTTOM_OVER 0x8020			// 寻找罐底中
#define DSM_STATE_FORCEZERO_OVER 0x8021				// 设置电机零点完成
#define DSM_STATE_ONTANKOPRATIONCOMPLATE 0x8022		// 罐上仪表操作完成
#define DSM_STATE_SYNTHETICING_OVER 0x8023			// 综合指令完成synthetic instruct 2019.08.07V1.105
#define DSM_STATE_COM_METER_DENSITY_OVER 0x8024		// 密度每米测量完成 V1.116 dq2020.4.2
#define DSM_STATE_INTERVAL_DENSITY_OVER 0x8025		// 液位区间测量完成 V1.116 dq2020.4.2

#define DSM_STATE_ERROR 0xFFFF // 故障




//extern volatile int EquipmentState;
//extern volatile int MeasureCommand;
//
//extern volatile bool Power_Flag;
//extern volatile bool XY_Initialize;
//extern float Gyro_Angle_X_Zero;
//extern float Gyro_Angle_Y_Zero;
//extern float Gyro_Angle_Oil;
//extern float Sonic_A_Zero;
//extern float Sonic_B_Zero;
//extern float Sonic_A_Oil;
//extern float Sonic_B_Oil;
//extern float Gyro_Angle_Threshold;
//extern float X_Zero;
//extern float Y_Zero;
//extern char NumOfPoints;
#endif
