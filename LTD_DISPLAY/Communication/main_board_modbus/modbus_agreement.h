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

typedef enum
{
    /************ 工作模式线圈区段 (起始地址 0x000A) ************/
    COIL_RETURN_ZERO            = 0x0001, // 返回零点
    COIL_CALIBRATE_ZERO         = 0x0002, // 标定零点
    COIL_MEASURE_SINGLE_POINT   = 0x0003, // 单点测量
    COIL_MONITOR_SINGLE_POINT   = 0x000E, // 单点监测
    COIL_MEASURE_DISTRIBUTED    = 0x000F, // 分布测量（带高度）
    COIL_MEASURE_DISTRIBUTED_AI = 0x0010, // 分布测量（自动）
    COIL_FIND_OIL_LEVEL         = 0x0011, // 寻找液位
    COIL_FIND_WATER_LEVEL       = 0x0012, // 寻找水位
    COIL_FIND_BOTTOM            = 0x0013, // 寻找罐底
    COIL_MEASURE_SYNTHETIC      = 0x0014, // 综合指令测量
    COIL_MEASURE_DENSITY_PER_M  = 0x0015, // 密度每米测量
    COIL_MEASURE_DENSITY_RANGE  = 0x0016, // 区间密度测量
    COIL_RESTORE_FACTORY        = 0x0200, // 恢复出厂设置
    /************ 调试模式线圈区段 (起始地址 0x0100) ************/
    COIL_CALIBRATE_OIL_LEVEL    = 0x0100, // 液位标定
    COIL_READ_PARAMETERS        = 0x0101, // 读取当前参数
    COIL_MOVE_UP                = 0x0102, // 向上运行
    COIL_MOVE_DOWN              = 0x0103, // 向下运行
    COIL_SET_ZERO_CIRCLE        = 0x0104, // 设置零点圈数
    COIL_SET_ZERO_ANGLE         = 0x0105, // 设置零点角度
    COIL_CORRECT_OIL_LEVEL      = 0x0106, // 修正液位
    COIL_FORCE_RETURN_ZERO      = 0x0107, // 强制零点
} ModbusCoil;






void setEquipStateError(void);
void AnalysisHoldRegister(void);
void Analysis04Register(void);
int getHoldValueNum(int operanum);
void InputValueInit(void);





#endif









