#ifndef __DISPLAY_TANKOPERA_H
#define __DISPLAY_TANKOPERA_H 
#include "main.h"

#define FIXPASSWORD         1009
extern bool flag_SendCommand;

struct KeyMenu {
    void (*back_opera)();
    void (*up_opera)();
    void (*down_opera)();
    void (*sure_opera)();
    int keyauthority;           //按键权限
    void (*execute_opera)();    //执行本次操作
};
extern struct KeyMenu keymenu[];
/*菜单索引号*/
typedef enum {
    KEYNUM_IF_ENTER_MAINMENU             = 0,  // 是否进入主菜单
    KEYNUM_MAINMENU                      = 1,  // 主菜单
    KEYNUM_IF_EXIT_MAINMENU             = 2,  // 是否退出主菜单
    KEYNUM_MEASURE_MAINMENU             = 3,  // 普通测量指令主菜单
    KEYNUM_MENU_PARACFG_MAIN            = 4,  // 参数配置主菜单
    KEYNUM_MENU_PARACFG_BASE            = 5,  // 菜单 - 基础参数
    KEYNUM_MENU_CMD_MAIN                = 6,  // 菜单 - 维护调试指令
    KEYNUM_IFSENDCMD                    = 7,  // 是否下发指令或参数
    KEYNUM_INPUTCMDPARA                 = 8,  // 请输入参数值(带参指令中的)
    KEYNUM_DISPLAY_PARA                 = 9,  // 参数显示(读写类参数中的)
    KEYNUM_MAGNETIC                     = 10, // 菜单 - 磁通量参数
    KEYNUM_WORDSELECT                   = 11, // 隐藏信息选择
    KEYNUM_SCREEN                       = 12, // 菜单 - 界面显示类参数
    KEYNUM_SOURCE                       = 13, // 菜单 - 数据源
    KEYNUM_MENU_PARACFG_DEBUG_BASIC     = 14, // 维护参数 - 基础参数
    KEYNUM_MENU_PARACFG_DEBUG_SYNTH     = 15, // 维护参数 - 综合测量参数
    KEYNUM_MENU_PARACFG_DEBUG_SPREAD    = 16, // 维护参数 - 分布测量参数
    KEYNUM_MENU_PARACFG_DEBUG_CORRECT   = 17, // 维护参数 - 修正参数
    KEYNUM_MENU_PARACFG_DEBUG_REALHIGH  = 18, // 维护参数 - 实高测量
    KEYNUM_MENU_PARACFG_DEBUG_INFO      = 19, // 维护参数 - 调试信息
	KEYNUM_MENU_LANGUAGE      = 20, 		// 界面显示 - 语言
    KEYNUM_END
} keymenuNumber;
typedef enum{
    COM_NUM_NOOPERA,                //无指令 - 且无特殊跳转
    
    COM_NUM_NOPARACMD_START,        /* 不带参线圈指令类 - 开始*/
	
    COM_NUM_NOPARACMD_NORMAL_START, /* 指令 - 普通 - 开始 */
	COM_NUM_SET_WORKPATTERN,	//	设置工作模式
	COM_NUM_BACK_ZERO,	//	返回零点
	COM_NUM_FIND_ZERO,	//	标定零点
	COM_NUM_SPREADPOINTS_AI,	//	自动分布测量
	COM_NUM_FIND_OIL,	//	寻找液位
	COM_NUM_FIND_WATER,	//	寻找水位
	COM_NUM_FIND_BOTTOM,	//	寻找罐底
	COM_NUM_SYNTHETIC,	//	综合测量
	COM_NUM_METER_DENSITY,	//	每米测量
	COM_NUM_INTERVAL_DENSITY,	//	区间测量
    COM_NUM_NOPARACMD_NORMAL_STOP,  /* 指令 - 普通 - 结束 */

	COM_NUM_DEBUGCMD_START,//调试模式指令
	COM_NUM_READPARAMETER,	//	读部件参数
	COM_NUM_FORCE_ZERO,	//	置零点
	COM_NUM_DEBUGCMD_STOP,//调试模式指令

	COM_NUM_UNLOCK_START,
	COM_NUM_RESTOR_EFACTORYSETTING,   // 恢复出厂设置
    COM_NUM_BACKUP_FILE,          // 备份配置文件
    COM_NUM_RESTORY_FILE,         // 恢复配置文件
	COM_NUM_UNLOCK_STOP,
	
    COM_NUM_NOPARACMD_END,          /* 不带参线圈指令类 - 结束*/
    
	
	
    COM_NUM_ONEPARACMD_START,       /*工作模式带一个参数线圈指令类 - 开始*/
	COM_NUM_SINGLE_POINT,        // 单点测量
    COM_NUM_SP_TEST,           // 单点监测
	COM_NUM_SPREADPOINTS,        // 分布测量
    COM_NUM_ONEPARACMD_END,         /*工作模式带一个参数线圈指令类 - 结束*/
    
    COM_NUM_ONEPARA_DEBUGCMD_START,                   /*调试模式带一个参数线圈指令类 - 开始*/
	COM_NUM_CAL_OIL,	//	液位标定
	COM_NUM_RUNUP,	//	向上运行
	COM_NUM_RUNDOWN,	//	向下运行
	COM_NUM_SET_ZEROCIRCLE,	//	设置零点圈数
	COM_NUM_SET_ZEROANGLE,	//	设置零点角度
	COM_NUM_CORRECTION_OIL,	//	修正液位
    COM_NUM_NOPARA_DEBUGCMD_END,            /*调试模式带一个参数线圈指令类 - 结束*/
    
    
    COM_NUM_PARACONFIG_START,           /* 参数配置类 - 开始 */
    COM_NUM_PARA_LOCAL_START,           /* 本机参数 - 开始 */
	
    COM_NUM_PARA_LOCAL_LEDVERSION,      //屏幕程序版本
	COM_NUM_PARA_LANG,                  //语言
    
    COM_NUM_SOURCE_START,               //数据源相关
    COM_NUM_SCREEN_SOURCE_OIL,
    COM_NUM_SCREEN_SOURCE_WATER,
    COM_NUM_SCREEN_SOURCE_D,
    COM_NUM_SCREEN_SOURCE_T,
    COM_NUM_SCREEN_INPUT_OIL,
    COM_NUM_SCREEN_INPUT_WATER,
    COM_NUM_SCREEN_INPUT_D,
    COM_NUM_SCREEN_INPUT_D_SWITCH,
    COM_NUM_SCREEN_INPUT_T,
    COM_NUM_SOURCE_STOP,
	
	COM_NUM_SCREEN_STR,                 //界面显示类
    COM_NUM_SCREEN_DECIMAL,
    COM_NUM_SCREEN_PASSWARD,
    COM_NUM_SCREEN_OFF,
    COM_NUM_SCREEN_STP,
   
   
    COM_NUM_PARA_LOCAL_STOP,            /* 本机参数 - 结束 */
	
    COM_NUM_PARA_DEBUG_START,           /*基础参数 - 开始 - 从此处往下都是CPU2存储的参数了*/
	COM_NUM_WORKPATTER_PASSWORD,        // 工作模式密码
//	COM_NUM_SP_POSITION,                // 单点测量位置高度
//	COM_NUM_SPT_POSITION,               // 单点监测位置高度
//	COM_NUM_SRREAD_POSITION,           // 分布测量时液位高度
	COM_NUM_SYNTHETIC_BOTTOM_FREE,     // 无权限综合是否测罐底
	COM_NUM_SYNTHETIC_WATER_FREE,      // 无权限综合是否测水位
	COM_NUM_SYNTHETIC_SINGLEPOINT_FREE,// 无权限综合是否测单点密度
	COM_NUM_SPREAD_STATE_FREE,         // 无权限分布测量模式
	COM_NUM_SPREAD_NUM_FREE,           // 无权限分布测量点数
	COM_NUM_SPREAD_DISTANCE_FREE,      // 无权限分布点间距
	COM_NUM_SPREAD_TOPLIMIT_FREE,      // 无权限分布最高点距液位
	COM_NUM_SPREAD_FLOORLIMIT_FREE,    // 无权限分布最低点距罐底
	COM_NUM_SPSYNTHETIC_POSITION,      // 综合测固定点测位置
	COM_NUM_MEASREMENT_METER,          // 每米测方向
	COM_NUM_INTERVAL_POINT,            // 区间密度测量点数
	COM_NUM_INTERVAL_DIREDION,         // 区间密度测量方向
	COM_NUM_INTERVAL_OIL_A,            // 区间测液位A
	COM_NUM_INTERVAL_OIL_B,            // 区间测液位B
	
	
	COM_NUM_TANKHIGHT,                 // 罐高
	//综合测量参数
	COM_NUM_SRREAD_MEASURETURN,        // 综合测量顺序
	COM_NUM_SPREAD_STATE,              // 综合测分布模式
	COM_NUM_SYNTHETIC_BOTTOM,          // 综合测是否测罐底
	COM_NUM_SYNTHETIC_WATER,           // 综合测是否测水位
	COM_NUM_SYNTHETIC_SINGLEPOINT,     // 综合测是否固定点
	COM_NUM_NUMOFDECIMALS,             // 温度有效位数
	COM_NUM_OIL_MEASUR_POSITION,       // 综合测单点测位置
	//分布测量参数
	COM_NUM_SPREAD_NUM,                // 密度分布测量点数
	COM_NUM_SPREAD_DISTANCE,           // 密度点间距
	COM_NUM_SPREAD_TOPLIMIT,           // 顶密距液位
	COM_NUM_SPREAD_FLOORLIMIT,         // 底密距罐底
	COM_NUM_DENSITY_TIME,              // 测密前提出时间
	COM_NUM_THRESHOLD_A,               // 分层加测阈值点A
	COM_NUM_THRESHOLD_B,               // 分层加测阈值点B
	COM_NUM_THRESHOLD_STANDARD,        // 国标测点数阈值
	
		//修正值（磁通量）
	COM_NUM_MAGNETIC_T,           // 温度修正值
	COM_NUM_MAGNETIC_D,      	 // 密度修正值
	
	//实高测量
	COM_NUM_WATER_IS_REAL_HIGH,        // 水位是否实高测
	COM_NUM_REAL_WATER_LEVEL_CORRECT,  // 实高水位修正值
	COM_NUM_IF_REFRESH_TANKHIGH,       // 综合测是否更新罐高
	COM_NUM_REAL_TANKHIGH_MAX_DIFF,    // 实高最大偏差阈值
	
		//调试信息
	COM_NUM_ZEROCIRCLE,//零点编码器圈数
	COM_NUM_ZEROANGLE,//零点编码器编码值
	COM_NUM_GIRTH_USELESS,             // 导轮周长
	COM_NUM_REDUCTION_RATIO,           // 减速比
	COM_NUM_TYPEOFSENSOR,              // 传感器类型
	COM_NUM_LENGTHOFSENSOR,            // 传感器头长度
	COM_NUM_FADEZERO,                  // 盲区
	COM_NUM_SOFTOFVERSION,             // 程序版本
	COM_NUM_GIRTH_YITI,                // 导轮周长（整机）
	COM_NUM_DEVICENUM,                 // 传感器编号
	//其他
	COM_NUM_IF_FINDOIL_POWERON,        // 上电找液位
	COM_NUM_LEVELTOWATER_HIGH,         // 液水传感距离差
	COM_NUM_MAXDOWN_DIS,               // 测罐底最大下行值
	COM_NUM_LEVEL_MEASURE_METHOD,      // 液位测量方式
	COM_NUM_TEMPERATURE_MODE,          // 综合测单温度模式
	COM_NUM_TEMPERATURE_MODE_D,        // 单温度模式密度值
	COM_NUM_WATER_ZERO_MIN_DISTANCE,   // 水位与零点最小距
	
	
    COM_NUM_PARA_DEBUG_END,             /*基础参数 - 结束*/
    
    
    COM_NUM_PARACONFIG_END,                     /* 参数配置类 - 结束 */
    
    COM_NUM_PASSWORD_START,                     /* 密码类 - 开始 */
    COM_NUM_PASSWORD_ENTER_PARA,                //进入参数配置的密码
    COM_NUM_PASSWORD_ENTER_CMD,                 //进入调试指令的密码
    COM_NUM_PASSWORD_END,                       /* 密码类 - 结束 */
	
    COM_NUM_END,
}OperatingNumber;

struct ParaContent{
    int val;
    int points;
    uint8_t *unit;
    uint8_t bits;
};
struct MenuData {
    uint8_t* operaName;
    int operaNum;
    void (*sureopera)();
    int rorw;
    uint8_t* operaName2;
};
/*指令中读写操作*/
typedef enum{
    COMMANE_NORW = 0,
    COMMAND_READ = 1,
    COMMAND_WRITE = 2,
}CommandOpera;

extern struct ParaContent now_Para_CT;






void KeyProcess(uint8_t keypress);
void useKey(void);
void exitTankOpera(void);
uint8_t* ret_arr_word(void);
void ClearPageNum(void);





#endif


