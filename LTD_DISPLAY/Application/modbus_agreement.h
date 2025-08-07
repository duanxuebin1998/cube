#ifndef __MODBUS_AGREEMENT_H
#define __MODBUS_AGREEMENT_H 
#include "main.h"

typedef enum{/* 数据源取自 */
    SOURCE_FROM_MEA,
    SOURCE_FROM_INPUT,
}SOURCE;




#define REPEATMAX 3//重复性测试次数
#define COMMU_ERROR_MAX 10//通讯连续错误最多次数
#define REALTEMPMAXSPOT 16//实时温度计最多测量点数
/*液位盲区值*/
#define OILLEVELDOWNLIMIT 100
/*无效值*/
#define UNVALID_LEVEL 999999u //液位无效值
#define LEVEL_DOWNLIMIT 100u //盲区液位值
#define UNVALID_TEMPERATURE_WIRELESS -2000//温度无效值
#define UNVALID_DENSITY 0 //密度无效值
#define LEVEL_DOWNLIMITWATER 0//水位无效值
#define UNVALID_QUALITY 0//质量无效值
#define UNVALID_VOLUME 0//体积无效值
#define UNVALID_X 0//暂用于流速流量无效值
/*报警状态*/
#define ALARMSTATE_NONE 0
#define ALARMSTATE_LOW 1
#define ALARMSTATE_HIGH 2

union utof
{
    float f;
    u32 u;
};

union utod
{
    double d;
    u32 u[2];
};
typedef enum{
    TYPE_INT,
    TYPE_FLOAT,
    TYPE_DOUBLE,
}DATA_TYPE;



extern int cnt_commutoCPU2;
extern int functioncode;
extern int startaddress;
extern int registeramount;
extern int byteamount;
extern uint8_t hold_register_value[];
extern uint8_t input_register_value[];
extern struct InputRegisterData inputValue;


/* 功能码 */
typedef enum {
    FUNCTIONCODE_READ_HOLDREGISTER = 0x03,
    FUNCTIONCODE_READ_INPUTREGISTER = 0x04,
    FUNCTIONCODE_WRITE_COIL = 0x05,
	FUNCTIONCODE_06 = 0x06,
    FUNCTIONCODE_WRITE_MULREGISTER = 0x10,
}FUNCTIONCODE;
/* 输入寄存器地址 */

typedef enum
{
    INPUTREG_PATTERNOFWORK              = 0x0000, // 工作模式
    INPUTREG_SYSTEMSTATE                = 0x0001, // 伺服密度计状态
    INPUTREG_ERRORNUM                   = 0x0002, // 故障代码
    INPUTREG_SP_TEMPERATURE             = 0x0004, // 单点测量温度
    INPUTREG_SP_DENSITY                 = 0x0005, // 单点测量密度
    INPUTREG_SP_TDSTATE                 = 0x0006, // 单点测量密度温度状态
    INPUTREG_SP_POSITION                = 0x0007, // 单点测量密度点位置
    INPUTREG_SP_STANDARDDENSITY         = 0x0009, // 单点测量标准密度
    INPUTREG_SP_VCF20                   = 0x000A, // 单点测量VCF
    INPUTREG_SP_WEIGHTDENSITY           = 0x000C, // 单点测量计重密度

    INPUTREG_SPT_TEMPERATURE            = 0x000D, // 单点检测温度
    INPUTREG_SPT_DENSITY                = 0x000E, // 单点检测密度
    INPUTREG_SPT_TDSTATE                = 0x000F, // 单点检测密度温度状态
    INPUTREG_SPT_POSITION               = 0x0010, // 单点检测密度点位置
    INPUTREG_SPT_STANDARDDENSITY        = 0x0012, // 单点检测标准密度
    INPUTREG_SPT_VCF20                  = 0x0013, // 单点检测VCF
    INPUTREG_SPT_WEIGHTDENSITY          = 0x0015, // 单点检测计重密度

    INPUTREG_SPREAD_AVERAGETEMPERATURE  = 0x0016, // 分布测量平均温度
    INPUTREG_SPREAD_AVERAGEDENSITY      = 0x0017, // 分布测量平均密度
    INPUTREG_SPREAD_AVERAGETDSTATE      = 0x0018, // 分布测量温度密度状态
    INPUTREG_SPREAD_STANDARDDENSITY     = 0x0019, // 分布测量标准密度
    INPUTREG_SPREAD_VCF20               = 0x001A, // 分布测量VCF
    INPUTREG_SPREAD_WEIGHTDENSITY       = 0x001C, // 分布测量计重密度
    INPUTREG_SPREAD_NUMOFDENSITY        = 0x001D, // 分布测量密度点数
    INPUTREG_LIQUIDLEVEL                = 0x001E, // 分布测量液位值
	
    INPUTREG_OILLEVEL                   = 0x0020, // 液位值
    INPUTREG_WATERLEVEL                 = 0x0022, // 水位值
    INPUTREG_BOTTOMHIGH                 = 0x0024, // 罐底值
    INPUTREG_REALTIMEPOSITION           = 0x0026, // 实时位置
    INPUTREG_VALUE_P1                   = 0x0028, // 罐底压力值
    INPUTREG_VALUE_P3                   = 0x002A,// 气相压力值
	INPUTREGISTER_END					= 0x002B

} ModbusInputRegister;

/* 输入寄存器测量值 */
struct InputRegisterData {
    int32_t PatternOfWork;              // 工作模式
    int32_t EquipState;                // 伺服密度计状态
    int32_t ErrorCode;                   // 故障代码

    int32_t SP_Temperature;             // 单点测量温度
    int32_t SP_Density;                 // 单点测量密度
    int32_t SP_TDState;                 // 单点测量密度温度状态
    int32_t SP_Position;                // 单点测量密度点位置
    int32_t SP_StandardDensity;         // 单点测量标准密度
    int32_t SP_VCF20;                   // 单点测量VCF
    int32_t SP_WeightDensity;           // 单点测量计重密度

    int32_t SPM_Temperature;            // 单点监测温度
    int32_t SPM_Density;                // 单点监测密度
    int32_t SPM_TDState;                // 单点监测密度温度状态
    int32_t SPM_Position;               // 单点监测密度点位置
    int32_t SPM_StandardDensity;        // 单点监测标准密度
    int32_t SPM_VCF20;                  // 单点监测VCF
    int32_t SPM_WeightDensity;          // 单点监测计重密度

    int32_t Spread_AvgTemperature;      // 分布测量平均温度
    int32_t Spread_AvgDensity;          // 分布测量平均密度
    int32_t Spread_AvgTDState;          // 分布测量温度密度状态
    int32_t Spread_StandardDensity;     // 分布测量标准密度
    int32_t Spread_VCF20;               // 分布测量VCF
    int32_t Spread_WeightDensity;       // 分布测量计重密度
    int32_t Spread_NumOfDensity;        // 分布测量密度点数
    int32_t LiquidLevel;                // 分布测量液位值

    int32_t OilLevel;                   // 液位值
    int32_t WaterLevel;                 // 水位值
    int32_t BottomHigh;                 // 罐底值
    int32_t RealtimePosition;           // 实时位置
    int32_t Value_P1;                   // 罐底压力值
    int32_t Value_P3;                   // 气相压力值
};
#define SUBSECTION_OIL_AMOUNT 10 //液位分段修正--共分几段
#define SUBSETTION_WATER_AMOUNT 5 //水位分段修正--共分几段
/* 保持寄存器地址 */
typedef enum {
    // 第一段：起始地址 0x0000
    HOLDREGISTER_WORKPATTER_PASSWORD = 0x0000,       // 工作模式密码
    HOLDREGISTER_SP_POSITION = 0x0006,                // 单点测量位置高度
    HOLDREGISTER_SPT_POSITION = 0x0008,               // 单点监测位置高度
    HOLDREGISTER_SRREAD_POSITION = 0x000A,            // 分布测量时液位高度
    HOLDREGISTER_SYNTHETIC_BOTTOM_FREE = 0x000C,      // 无需权限综合指令是否需要测罐底
    HOLDREGISTER_SYNTHETIC_WATER_FREE = 0x000D,       // 无需权限综合指令是否需要测水位
    HOLDREGISTER_SYNTHETIC_SINGLEPOINT_FREE = 0x000E, // 无需权限综合指令是否需要测单点密度
    HOLDREGISTER_SPREAD_STATE_FREE = 0x000F,          // 无需权限分布测量时测量模式
    HOLDREGISTER_SPREAD_NUM_FREE = 0x0010,            // 无需权限分布测量点数
    HOLDREGISTER_SPREAD_DISTANCE_FREE = 0x0011,       // 无需权限分布测量点之间间距
    HOLDREGISTER_SPREAD_TOPLIMIT_FREE = 0x0013,       // 无需权限分布测量最高点距液面间距
    HOLDREGISTER_SPREAD_FLOORLIMIT_FREE = 0x0015,     // 无需权限分布测量最低点距罐底间距
    HOLDREGISTER_SPSYNTHETIC_POSITION = 0x0017,       // 综合指令固定点测量的位置
    HOLDREGISTER_MEASREMENT_METER = 0x0019,           // 每米测量时方向
    HOLDREGISTER_INTERVAL_POINT = 0x001A,             // 区间密度测量点数
    HOLDREGISTER_INTERVAL_DIREDION = 0x001B,          // 区间密度测量方向
    HOLDREGISTER_INTERVAL_OIL_A = 0x001C,             // 区间密度分布测量液位点A
    HOLDREGISTER_INTERVAL_OIL_B = 0x001E,             // 区间密度分布测量液位点B
    HOLDREGISTER_D_CORRECTION_TEM1 = 0x0020,          // 密度分段修正温度阈值1
    HOLDREGISTER_D_CORRECTION_TEM2 = 0x0021,          // 密度分段修正温度阈值2
    HOLDREGISTER_D_CORRECTION_TEM3 = 0x0022,          // 密度分段修正温度阈值3
    HOLDREGISTER_D_CORRECTION_TEM4 = 0x0023,          // 密度分段修正温度阈值4
    HOLDREGISTER_D_CORRECTION_TEM5 = 0x0024,          // 密度分段修正温度阈值5
    HOLDREGISTER_D_CORRECTION_TEM6 = 0x0025,          // 密度分段修正温度阈值6
    HOLDREGISTER_D_CORRECTION_TEM7 = 0x0026,          // 密度分段修正温度阈值7
    HOLDREGISTER_D_CORRECTION_TEM8 = 0x0027,          // 密度分段修正温度阈值8
    HOLDREGISTER_D_CORRECTION_TEM9 = 0x0028,          // 密度分段修正温度阈值9
    HOLDREGISTER_D_CORRECTION_TEM10 = 0x0029,         // 密度分段修正温度阈值10
    HOLDREGISTER_D_CORRECTION_TEM11 = 0x002A,         // 密度分段修正温度阈值11
    HOLDREGISTER_D_CORRECTION_1 = 0x002B,             // 密度修正值1
    HOLDREGISTER_D_CORRECTION_2 = 0x002C,             // 密度修正值2
    HOLDREGISTER_D_CORRECTION_3 = 0x002D,             // 密度修正值3
    HOLDREGISTER_D_CORRECTION_4 = 0x002E,             // 密度修正值4
    HOLDREGISTER_D_CORRECTION_5 = 0x002F,             // 密度修正值5
    HOLDREGISTER_D_CORRECTION_6 = 0x0030,             // 密度修正值6
    HOLDREGISTER_D_CORRECTION_7 = 0x0031,             // 密度修正值7
    HOLDREGISTER_D_CORRECTION_8 = 0x0032,             // 密度修正值8
    HOLDREGISTER_D_CORRECTION_9 = 0x0033,             // 密度修正值9
    HOLDREGISTER_D_CORRECTION_10 = 0x0034,            // 密度修正值10
    HOLDREGISTER_D_CORRECTION_11 = 0x0035,            // 密度修正值11
    HOLDREGISTER_D_CORRECTION_12 = 0x0036,            // 密度修正值12

    // 第二段：起始地址 0x0100
    HOLDREGISTER_TANKHIGHT = 0x0100,                  // 罐高
    HOLDREGISTER_CALIBRATIONLIQUIDLEVEL = 0x0102,     // 标定液位时液位高度
    HOLDREGISTER_SRREAD_MEASURETURN = 0x0104,         // 分布测量时测量顺序
    HOLDREGISTER_SPREAD_STATE = 0x0105,               // 分布测量时测量模式
    HOLDREGISTER_SPREAD_NUM = 0x0106,                 // 分布测量点数
    HOLDREGISTER_SPREAD_DISTANCE = 0x0107,            // 分布测量点之间间距
    HOLDREGISTER_SPREAD_TOPLIMIT = 0x0109,            // 分布测量最高点距液面间距
    HOLDREGISTER_SPREAD_FLOORLIMIT = 0x010B,          // 分布测量最低点距罐底间距
    HOLDREGISTER_SPREAD_1POINTTHRESHOLD = 0x010D,     // 分布测量一个点的高度阈值
    HOLDREGISTER_SPREAD_5POINTTHRESHOLD = 0x010F,     // 分布测量五个点的高度阈值
    HOLDREGISTER_SPREAD_OTHERNUM = 0x0111,            // 分布测量其他情况的测量点数
    HOLDREGISTER_SPREAD_FIXEDDISTANCE = 0x0112,       // 固定间距分布测量的间距
    HOLDREGISTER_SPREAD_FIXEDTOP = 0x0114,            // 固定间距分布测量最高点距液面间距
    HOLDREGISTER_SPREAD_FIXEDBASE = 0x0116,           // 固定间距分布测量最低点距罐底间距
    HOLDREGISTER_THRESHOLD_A = 0x0118,                 // 分层加测阈值点A
    HOLDREGISTER_THRESHOLD_B = 0x0119,                 // 分层加测阈值点B
    HOLDREGISTER_THRESHOLD_STANDARD = 0x011A,          // 国标密度测量点数阈值
    HOLDREGISTER_WATER_IS_REAL_HIGH = 0x011B,          // 水位是否为实高测量
    HOLDREGISTER_REAL_WATER_LEVEL_CORRECT = 0x011C,    // 实高水位修正值
    HOLDREGISTER_IF_REFRESH_TANKHIGH = 0x011D,         // 测量是否更新罐高
    HOLDREGISTER_REAL_TANKHIGH_MAX_DIFF = 0x011E,      // 实高最大偏差阈值
    HOLDREGISTER_LEVEL_MEASURE_METHOD = 0x011F,        // 液位测量方式
    HOLDREGISTER_TEMPERATURE_MODE = 0x0120,            // 单温度模式
    HOLDREGISTER_TEMPERATURE_MODE_D = 0x0121,          // 温度模式密度值
    HOLDREGISTER_WATER_ZERO_MIN_DISTANCE = 0x0122,     // 水位与零点最小距离
    HOLDREGISTER_IF_FINDOIL_POWERON = 0x0123,          // 上电是否自动找液位
    HOLDREGISTER_DENSITY_TIME = 0x0124,                // 密度测量提出油面时间

    // 第三段：起始地址 0x0180
    HOLDREGISTER_RUNTODISTANCE = 0x0180,               // 运行距离
    HOLDREGISTER_ZEROCIRCLE = 0x0182,                  // 零点圈数
    HOLDREGISTER_ZEROANGLE = 0x0183,                    // 零点角度
    HOLDREGISTER_GIRTH_USELESS = 0x0184,                // 导轮周长
    HOLDREGISTER_REDUCTION_RATIO = 0x0185,               // 减速比
    HOLDREGISTER_TYPEOFSENSOR = 0x0186,                 // 传感器类型
    HOLDREGISTER_LENGTHOFSENSOR = 0x0187,               // 传感器长度
    HOLDREGISTER_FADEZERO = 0x0188,                      // 盲区
    HOLDREGISTER_SOFTOFVERSION = 0x018A,                 // 程序版本
    HOLDREGISTER_CORRECTION_OIL = 0x018E,                // 修正的液位值
    HOLDREGISTER_GIRTH_YITI = 0x0190,                    // 一体机转鼓周长
    HOLDREGISTER_LEVELTOWATER_HIGH = 0x0192,             // 水位传感器到液位传感器距离
    HOLDREGISTER_MAXDOWN_DIS = 0x0193,                   // 水位和罐底测量最大下行距离
    HOLDREGISTER_SYNTHETIC_BOTTOM = 0x0194,              // 综合指令是否测罐底
    HOLDREGISTER_SYNTHETIC_WATER = 0x0195,               // 综合指令是否测水位
    HOLDREGISTER_SYNTHETIC_SINGLEPOINT = 0x0196,         // 综合指令是否测单点密度
    HOLDREGISTER_NUMOFDECIMALS = 0x0197,                 // 温度有效位数
    HOLDREGISTER_TEMCORRECTCALUE = 0x0198,               // 温度修正系数
    HOLDREGISTER_DENSITYCORRECTCALUE = 0x0199,           // 密度修正系数
    HOLDREGISTER_DEVICENUM = 0x019A,                      // 传感器编号
    HOLDREGISTER_OIL_MEASUR_POSITION = 0x019C,            // 综合测密度发油位置

    // 第四段：起始地址 0x0200
    HOLDREGISTER_TYPEOFEINDUCTION = 0x0200,               // 霍尔器件类型
    HOLDREGISTER_NUMOFEINDUCTION = 0x0201,                // 霍尔器件数量
    HOLDREGISTER_USEFULNUMOFEINDUCTION = 0x0202,          // 霍尔器件作用数量
    HOLDREGISTER_DIATANCEFROMEINDUCTION = 0x0203,         // 脱离配重步进数
    HOLDREGISTER_CORRECTIONFACTOR = 0x0204,               // 位置修正系数
    HOLDREGISTER_THRESHOLDOFFREQUENCE = 0x0205,           // 频率阈值
    HOLDREGISTER_RANGE_FREQUENCE = 0x0207,                // 精找液位频率阈值范围
    HOLDREGISTER_RANGE_POSITION = 0x0208,                  // 位置阈值范围
    HOLDREGISTER_RANGE_STABLEFREQUENCE = 0x0209,           // 测量时频率稳定范围
    HOLDREGISTER_RANGE_STABLETEMPERATURE = 0x020A,         // 测量时温度稳定范围
    HOLDREGISTER_NUMOFSTABLEHITS = 0x020B,                 // 温度平衡所需点数
    HOLDREGISTER_NUMOFHITS = 0x020C,                       // 温度平衡时间点数

    // 第五段：起始地址 0x0280
    HOLDREGISTER_K0 = 0x0280,      // K0
    HOLDREGISTER_K1 = 0x0284,      // K1
    HOLDREGISTER_K2 = 0x0288,      // K2
    HOLDREGISTER_K3 = 0x028C,      // K3
    HOLDREGISTER_K18 = 0x0290,     // K18
    HOLDREGISTER_K19 = 0x0294,     // K19
    HOLDREGISTER_V1 = 0x0298,      // V1 (K1)
    HOLDREGISTER_V2 = 0x029C,      // V2 (K2)
    HOLDREGISTER_V3 = 0x02A0,      // V3 (K3)

} HoldRegister;

/* 保持寄存器数据结构 */
struct HoldRegisterData {
    uint8_t* name;
    int val;
    const int operanum;
    const uint16_t startadd;
    const uint8_t rgstcnt;
    const bool flag_checkvalue;
    const int valuemin;
    const int valuemax;
    uint8_t* unit;
    const uint8_t point;
    const int offset;
    const bool authority_write;
    int data_type;
    int bits;
    uint8_t *(*pword)();
    uint8_t* name_English;
};

typedef enum{
    LANG_CHINESE = 0,
    LANG_ENGLISH,
    
    
}LANG_NUM;


extern struct HoldRegisterData holdValue[];
/* 线圈指令地址 */
typedef enum{
     STARTADDRESS_COIL = 0x0A,                              //线圈起始地址
     COIL_FINDLEVEL = 0x0A,                                 //跟随液位
     COIL_MEASUREWATER = 0x0B,                              //测量水位
     COIL_MEASUREDENSITY = 0x0C,                            //密度测量
     COIL_CORRECTLEVEL = 0x0D,                              //修正液位
     COIL_MEASUREDENSITYANDTEMPERATURE = 0x0E,              //密度温度测量
     COIL_CALIBRATIONDENSITY = 0x0F,                        //密度校正
     COIL_CALIBRATIONORIGIN = 0x10,                         //标定零点
     COIL_CALIBRATIONLEVEL = 0x11,                          //标定液位
     COIL_BRINGUPDISPLACERTOORIGIN = 0x12,                  //提浮子至零点
     COIL_MEASUREDENSITY_ASSIGNHEIGHT = 0x13,               //指定高度密度测量
     COIL_MEASUREDENSITYANDTEMPERATURE_ASSIGNHEIGHT = 0x14, //指定高度密度温度测量
     COIL_MEASUREBOTTOM = 0x15,                             //罐底测量
     COIL_SYSTEMRESTART = 0x16,                             //系统重启
     COIL_RESERCOIL_INTEGRATEDVE6 = 0x17,                   //综合测量
     COIL_DENSITY_NATION = 0x18,                            //国标测量
     COIL_RESERVE7 = 0x19,
     COIL_RESTOREFACTORYSETTINGS = 0x1A,                    //恢复出厂设置
     COIL_BACKUPCONFIGURATIONFILE = 0x1B,
     COIL_RESTORECONFIGURATIONFILE = 0x1C,                  //恢复配置文件
     COIL_RESETSCOPEOFWEIGHT = 0x1D,                        //重置称重阈值
     COIL_GETVALUEOFWEIGHT = 0x1E,                          //读取称重值
     COIL_GETVALUEOFMOTORSTEPS = 0x1F,                      //读取步进数
     COIL_GETVALUEOFTEMPERATURE = 0x20,                     //读无线温度
     COIL_MOTORRUNUP = 0x21,                                //向上运行
     COIL_MOTORRUNDOWN = 0x22,                              //向下运行
     COIL_INTERVALMEASURE = 0x23,                           //区间测量
     COIL_PERMETERMEASURE = 0x24,                           //每米测量
     COIL_SELFINSPECTION = 0x25,                            //设备自检
     COIL_CONFIG = 0x26,                                    //进入仪表配置模式
     COIL_SETNOLOAD = 0x27,                                 //设置空载阈值
     COIL_SETFULLLOAD = 0x28,                               //设置满载阈值
     COIL_REDAREALTMP = 0x29,                               //读实时温度
     COIL_NOSINEFINDOIL= 0x2A,                              //无拟合找液位
     COIL_NOSINEZERO = 0x2B,                                //无拟合回零点
     COIL_VCHECK_STATIC,                                    //电压检测 - 静止
     COIL_VCHECK_RUN,                                       //电压检测 - 运动
     COIL_WATERFOLLOWING,                                   //水位跟随
     COIL_CORRECTWATERLEVEL,                                //修正水位
     COIL_EQUIPRUNNINGIN,                                   //设备磨合
     COIL_CHECKMODE,                                        //进入检修模式
     
     
     COIL_AMOUNT,//线圈总数
}COIL;
typedef enum
{
    /************ 工作模式线圈区段 (起始地址 0x000A) ************/
    COIL_SET_WORKPATTERN        = 0x000A, // 设置工作模式
    COIL_RETURN_ZERO            = 0x000B, // 返回零点
    COIL_CALIBRATE_ZERO         = 0x000C, // 标定零点
    COIL_MEASURE_SINGLE_POINT   = 0x000D, // 单点测量
    COIL_MONITOR_SINGLE_POINT   = 0x000E, // 单点监测
    COIL_MEASURE_DISTRIBUTED    = 0x000F, // 分布测量（带高度）
    COIL_MEASURE_DISTRIBUTED_AI = 0x0010, // 分布测量（自动）
    COIL_FIND_OIL_LEVEL         = 0x0011, // 寻找液位
    COIL_FIND_WATER_LEVEL       = 0x0012, // 寻找水位
    COIL_FIND_BOTTOM            = 0x0013, // 寻找罐底
    COIL_MEASURE_SYNTHETIC      = 0x0014, // 综合指令测量
    COIL_MEASURE_DENSITY_PER_M  = 0x0015, // 密度每米测量
    COIL_MEASURE_DENSITY_RANGE  = 0x0016, // 区间密度测量

    /************ 调试模式线圈区段 (起始地址 0x0100) ************/
    COIL_CALIBRATE_OIL_LEVEL    = 0x0100, // 液位标定
    COIL_READ_PARAMETERS        = 0x0101, // 读取当前参数
    COIL_MOVE_UP                = 0x0102, // 向上运行
    COIL_MOVE_DOWN              = 0x0103, // 向下运行
    COIL_SET_ZERO_CIRCLE        = 0x0104, // 设置零点圈数
    COIL_SET_ZERO_ANGLE         = 0x0105, // 设置零点角度
    COIL_CORRECT_OIL_LEVEL      = 0x0106, // 修正液位
    COIL_FORCE_RETURN_ZERO      = 0x0107, // 强制零点

    /************ 解锁模式线圈区段 (起始地址 0x0200) ************/
    COIL_RESTORE_FACTORY        = 0x0200, // 恢复出厂设置
    COIL_BACKUP_CONFIG_FILE     = 0x0201, // 备份配置文件
    COIL_RESTORE_CONFIG_FILE    = 0x0202  // 恢复配置文件

} ModbusCoil;


/*****************************状态*****************************************/
typedef enum
{
	STATE_STANDBY = 0x0000,					  // 待机状态
	STATE_INIT = 0x0001,					  // 初始化状态
	STATE_BACKZEROING = 0x0002,				  // 回零点中
	STATE_FINDZEROING = 0x0010,				  // 标定零点中
	STATE_SINGLEPOINTING = 0x0011,			  // 单点测量中
	STATE_RUNTOPOINTING = 0x0012,			  // 运行到测量点中
	STATE_SPREADPOINTING = 0x0013,			  // 分布测量中
	STATE_AI_SPREADPOINTING = 0x0014,		  // 无参分布测量中
	STATE_CALIBRATIONOILING = 0x0015,		  // 标定液位
	STATE_READPARAMETERING = 0x0016,		  // 读取参数中
	STATE_RUNUPING = 0x0017,				  // 向上运行中
	STATE_RUNDOWNING = 0x0018,				  // 向下运行中
	STATE_SETZEROCIRCLING = 0x0019,			  // 设置零点编码值
	STATE_SETZEROANGLING = 0x001A,			  // 设置零点编码值
	STATE_EFACTORYSETTING_RESTORING = 0x001B, // 恢复出厂设置中
	STATE_BACKUPING = 0x001C,				  // 备份配置文件中
	STATE_RESTORYING = 0x001D,				  // 恢复配置文件中
	STATE_FINDOIL = 0x001E,					  // 寻找液位中
	STATE_FINDWATER = 0x001F,				  // 寻找水位中
	STATE_FINDBOTTOM = 0x0020,				  // 寻找罐底中
	STATE_FORCEZERO = 0x0021,				  // 设置电机零点中
	STATE_ONTANKOPRATIONING = 0x0022,		  // 罐上仪表操作中
	STATE_SYNTHETICING = 0x0023,			  // 综合指令
	STATE_METER_DENSITY = 0x0024,			  // 密度每米测量中
	STATE_INTERVAL_DENSITY = 0x0025,		  // 液位区间测量中

	STATE_FINDZEROOVER = 0x8010,			   // 标定零点完成
	STATE_SINGLEPOINTOVER = 0x8011,			   // 单点测量完成
	STATE_SPTESTING = 0x8012,				   // 正在单点检测
	STATE_SPREADPOINTOVER = 0x8013,			   // 分布测量完成
	STATE_AI_SPREADPOINTOVER = 0x8014,		   // 无参分布测量完成
	STATE_FINDOILOVER = 0x8015,				   // 标定液位完成
	STATE_READPARAMETEROVER = 0x8016,		   // 读取参数完成
	STATE_RUNUPOVER = 0x8017,				   // 向上运行完成
	STATE_RUNDOWNOVER = 0x8018,				   // 向下运行完成
	STATE_SETZEROCIRCLOVER = 0x8019,		   // 设置零点编码值完成
	STATE_SETZEROANGLOVER = 0x801A,			   // 设置零点编码值完成
	STATE_EFACTORYSETTING_RESTOROVER = 0x801B, // 恢复出厂设置完成
	STATE_BACKUPOVER = 0x801C,				   // 备份配置文件完成
	STATE_RESTORYOVER = 0x801D,				   // 恢复配置文件完成
	STATE_FLOWOIL = 0x801E,					   // 液位跟随中
	STATE_FINDWATER_OVER = 0x801F,			   // 寻找水位完成
	STATE_FINDBOTTOM_OVER = 0x8020,			   // 寻找罐底完成
	STATE_FORCEZERO_OVER = 0x8021,			   // 设置电机零点完成
	STATE_ONTANKOPRATIONCOMPLATE = 0x8022,	   // 罐上仪表操作完成
	STATE_SYNTHETICING_OVER = 0x8023,		   // 综合指令完成
	STATE_COM_METER_DENSITY_OVER = 0x8024,	   // 密度每米测量完成
	STATE_INTERVAL_DENSITY_OVER = 0x8025,	   // 液位区间测量完成

	STATE_FAIL = 0xFFFF // 故障
} DeviceState;




void setEquipStateError(void);
void AnalysisHoldRegister(void);
void Analysis04Register(void);
int getHoldValueNum(int operanum);
void InputValueInit(void);





#endif









