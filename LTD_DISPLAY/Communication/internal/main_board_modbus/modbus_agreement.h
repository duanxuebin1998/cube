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









