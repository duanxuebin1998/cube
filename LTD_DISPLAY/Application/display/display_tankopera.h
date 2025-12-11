#ifndef __DISPLAY_TANKOPERA_H
#define __DISPLAY_TANKOPERA_H 
#include "main.h"

#define FIXPASSWORD         1009

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
	KEYNUM_IF_ENTER_MAINMENU = 0,  // 是否进入主菜单
	KEYNUM_MAINMENU = 1,  // 主菜单
	KEYNUM_IF_EXIT_MAINMENU = 2,  // 是否退出主菜单
	KEYNUM_MEASURE_MAINMENU = 3,  // 普通测量指令主菜单
	KEYNUM_MENU_PARACFG_MAIN = 4,  // 参数配置主菜单
	KEYNUM_WARTSILA_PARACFG_BASE = 5,  // 菜单 - 基础参数
	KEYNUM_MENU_CMD_MAIN = 6,  // 菜单 - 维护调试指令
	KEYNUM_IFSENDCMD = 7,  // 是否下发指令或参数
	KEYNUM_INPUTCMDPARA = 8,  // 请输入参数值(带参指令中的)
	KEYNUM_DISPLAY_PARA = 9,  // 参数显示(读写类参数中的)
	KEYNUM_MAGNETIC = 10, // 菜单 - 磁通量参数
	KEYNUM_WORDSELECT = 11, // 隐藏信息选择
	KEYNUM_SCREEN = 12, // 菜单 - 界面显示类参数
	KEYNUM_SOURCE = 13, // 菜单 - 数据源
	KEYNUM_MENU_PARACFG_DEBUG_BASIC = 14, // 维护参数 - 基础参数
	KEYNUM_MENU_PARACFG_DEBUG_SYNTH = 15, // 维护参数 - 综合测量参数
	KEYNUM_MENU_PARACFG_DEBUG_SPREAD = 16, // 维护参数 - 分布测量参数
	KEYNUM_MENU_PARACFG_DEBUG_CORRECT = 17, // 维护参数 - 修正参数
	KEYNUM_MENU_PARACFG_DEBUG_REALHIGH = 18, // 维护参数 - 实高测量
	KEYNUM_MENU_PARACFG_DEBUG_LIQUIDLEVEL,
	KEYNUM_MENU_PARACFG_DEBUG_WATERLEVEL,
	KEYNUM_MENU_PARACFG_DEBUG_ALARM_DO,
	KEYNUM_MENU_PARACFG_DEBUG_AO,
	KEYNUM_MENU_LANGUAGE , 		// 界面显示 - 语言
	KEYNUM_END
} keymenuNumber;
typedef enum {
	COM_NUM_NOOPERA,                //无指令 - 且无特殊跳转

	COM_NUM_NOPARACMD_START, /* 不带参线圈指令类 - 开始*/

    /* 普通无参测量指令 */
    COM_NUM_NOPARACMD_NORMAL_START, /* 指令 - 普通 - 开始 */
    COM_NUM_BACK_ZERO,              // 返回零点
    COM_NUM_FIND_ZERO,              // 标定零点
    COM_NUM_SPREADPOINTS_AI,        // 自动分布测量
    COM_NUM_FIND_OIL,               // 寻找液位
    COM_NUM_FIND_WATER,             // 寻找水位
    COM_NUM_FIND_BOTTOM,            // 寻找罐底
    COM_NUM_SYNTHETIC,              // 综合测量
    COM_NUM_METER_DENSITY,          // 每米测量
    COM_NUM_INTERVAL_DENSITY,       // 区间测量
    COM_NUM_WARTSILA_DENSITY,       // 瓦锡兰区间密度（新增）
    COM_NUM_NOPARACMD_NORMAL_STOP,  /* 指令 - 普通 - 结束 */

    /* 调试模式无参指令 */
    COM_NUM_DEBUGCMD_START,         // 调试模式指令开始
    COM_NUM_SET_EMPTY_WEIGHT,       // 设置空载称重
    COM_NUM_SET_FULL_WEIGHT,        // 设置满载称重
    COM_NUM_RESTOR_EFACTORYSETTING, // 恢复出厂设置
    COM_NUM_MAINTENANCE_MODE,       // 维护模式（新增）
    COM_NUM_DEBUGCMD_STOP,          // 调试模式指令结束

    COM_NUM_NOPARACMD_END,          // 不带参线圈指令类 - 结束

    /* 工作模式带一个参数线圈指令 */
    COM_NUM_ONEPARACMD_START,       // 开始
    COM_NUM_SINGLE_POINT,           // 单点测量
    COM_NUM_SP_TEST,                // 单点监测
    COM_NUM_ONEPARACMD_END,         // 结束

    /* 调试模式带一个参数线圈指令 */
    COM_NUM_ONEPARA_DEBUGCMD_START, // 开始
    COM_NUM_CAL_OIL,                // 液位标定
    COM_NUM_RUNUP,                  // 向上运行
    COM_NUM_RUNDOWN,                // 向下运行
    COM_NUM_CORRECTION_OIL,         // 修正液位
    COM_NUM_NOPARA_DEBUGCMD_END,    // 结束

	COM_NUM_PARACONFIG_START, /* 参数配置类 - 开始 */
	COM_NUM_PARA_LOCAL_START, /* 本机参数 - 开始 */

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

	COM_NUM_PARA_LOCAL_STOP, /* 本机参数 - 结束 */

	COM_NUM_PARA_DEBUG_START, /*基础参数 - 开始 - 从此处往下都是CPU2存储的参数了*/

	COM_NUM_DEVICEPARAM_COMMAND,

	COM_NUM_DEVICEPARAM_TANKHEIGHT,              /* 罐高 */
	COM_NUM_DEVICEPARAM_BLINDZONE,               /* 液位盲区 */
	COM_NUM_DEVICEPARAM_WATER_BLINDZONE,         /* 水位盲区 */
	COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, /* 编码轮周长 */
	COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED,         /* 最大电机速度 */
	COM_NUM_DEVICEPARAM_SENSORTYPE,              /* 传感器类型 */
	COM_NUM_DEVICEPARAM_SENSORID,                /* 传感器编号 */
	COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION, /* 传感器软件版本 */
	COM_NUM_DEVICEPARAM_SOFTWAREVERSION,         /* LTD 软件版本 */
	COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,/* 上电默认指令 */
	COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,  /* 找零点下行距离 */
	COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM, /* 尺带首圈周长 */
	COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM,       /* 尺带厚度 */
	COM_NUM_DEVICEPARAM_RESERVED1,               /* 保留 1 */
	COM_NUM_DEVICEPARAM_RESERVED2,               /* 保留 2 */
	COM_NUM_DEVICEPARAM_RESERVED3,               /* 保留 3 */
	/* 称重参数 */
	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT,            /* 空载重量 */
	COM_NUM_DEVICEPARAM_FULL_WEIGHT,             /* 满载重量 */
	COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,/* 上限比例 */
	COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,/* 下限比例 */
	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,/* 空载重量上限 */
	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,/* 空载重量下限 */
	COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT, /* 满载重量上限 */
	COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT, /* 满载重量下限 */
	COM_NUM_DEVICEPARAM_RESERVED4,               /* 保留 4 */
	COM_NUM_DEVICEPARAM_RESERVED5,               /* 保留 5 */

	/* 指令参数 */
	COM_NUM_DEVICEPARAM_CALIBRATE_OIL_LEVEL,     /* 液位标定值 */
	COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL,   /* 水位标定值 */
	COM_NUM_DEVICEPARAM_SP_MEAS_POSITION,        /* 单点测量位置 */
	COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION,     /* 单点监测位置 */
	COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL, /* 密度分布液位 */
	COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,  /* 电机运行距离 */
	COM_NUM_DEVICEPARAM_RESERVED6,               /* 保留 6 */
	COM_NUM_DEVICEPARAM_RESERVED7,               /* 保留 7 */

	/* 密度温度修正 */
	COM_NUM_DEVICEPARAM_DENSITYCORRECTION,       /* 密度修正 */
	COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION,   /* 温度修正 */

	/* 分布测量参数 */
	COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,/* 是否测罐底 */
	COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT, /* 是否测水位 */
	COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,/* 是否测单点密度 */
	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER,  /* 分布测量顺序 */
	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE,   /* 分布测量模式 */
	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT,  /* 分布测量点数 */
	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,/* 点间距 */
	COM_NUM_DEVICEPARAM_SPREADTOPLIMIT,          /* 距液面上限 */
	COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT,       /* 距罐地下限 */
	COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME, /* 首点悬停时间 */
	COM_NUM_DEVICEPARAM_RESERVED8,               /* 保留 8 */
	COM_NUM_DEVICEPARAM_RESERVED9,               /* 保留 9 */

	/* Wartsila 密度区间测量参数 */
	COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,   /* 区间上限密度 */
	COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,   /* 区间下限密度 */
	COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,      /* 密度步进 */
	COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE, /* 最高测点距液面 */

	/* 水位测量参数 */
	COM_NUM_DEVICEPARAM_WATERLEVELCORRECTION,    /* 水位修正 */
	COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE,         /* 最大下行距离 */
	COM_NUM_DEVICEPARAM_RESERVED10,              /* 保留 10 */

	/* 实高测量 */
	COM_NUM_DEVICEPARAM_REFRESHTANKHEIGHTFLAG,   /* 是否更新罐高 */
	COM_NUM_DEVICEPARAM_MAXTANKHEIGHTDEVIATION,  /* 罐高最大变化 */
	COM_NUM_DEVICEPARAM_INITIALTANKHEIGHT,       /* 初始实高 */
	COM_NUM_DEVICEPARAM_CURRENTTANKHEIGHT,       /* 当前实高 */

	/* 液位测量 */
	COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD,       /* 找油阈值 */
	COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD, /* 液位滞后 */
	COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,  /* 液位测量方式 */

	/* 报警（DO） */
	COM_NUM_DEVICEPARAM_ALARM_HIGH_DO,           /* 高液位报警 DO */
	COM_NUM_DEVICEPARAM_ALARM_LOW_DO,            /* 低液位报警 DO */
	COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD,   /* 第三状态阈值 */

	/* 4–20mA 输出与报警（AO） */
	COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA,  /* 电流量程起点 */
	COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA,    /* 电流量程终点 */
	COM_NUM_DEVICEPARAM_ALARM_HIGH_AO,           /* 高位报警 AO */
	COM_NUM_DEVICEPARAM_ALARM_LOW_AO,            /* 低位报警 AO */
	COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA,      /* 初始化电流 */
	COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA,      /* AO 高电流 */
	COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA,       /* AO 低电流 */
	COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA,        /* 故障电流 */
	COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA,        /* 调试电流 */

	/* 版本和校验信息 */
	COM_NUM_DEVICEPARAM_PARAM_VERSION,           /* 参数版本号 */
	COM_NUM_DEVICEPARAM_STRUCT_SIZE,             /* 结构体大小 */
	COM_NUM_DEVICEPARAM_MAGIC,                   /* 魔术字 */
	COM_NUM_DEVICEPARAM_CRC,                     /* CRC 校验 */

	COM_NUM_PARA_DEBUG_END, /*基础参数 - 结束*/

	COM_NUM_PARACONFIG_END, /* 参数配置类 - 结束 */

	COM_NUM_PASSWORD_START, /* 密码类 - 开始 */
	COM_NUM_PASSWORD_ENTER_PARA,                //进入参数配置的密码
	COM_NUM_PASSWORD_ENTER_CMD,                 //进入调试指令的密码
	COM_NUM_PASSWORD_END, /* 密码类 - 结束 */

	COM_NUM_END,
} OperatingNumber;

struct ParaContent {
	int val;
	int points;
	uint8_t *unit;
	uint8_t bits;
};
struct MenuData {
	uint8_t *operaName;
	int operaNum;
	void (*sureopera)();
	int rorw;
	uint8_t *operaName2;
};
/*指令中读写操作*/
typedef enum {
	COMMANE_NORW = 0, COMMAND_READ = 1, COMMAND_WRITE = 2,
} CommandOpera;

extern struct ParaContent now_Para_CT;

void KeyProcess(uint8_t keypress);
void useKey(void);
void exitTankOpera(void);
uint8_t* ret_arr_word(void);
void ClearPageNum(void);

#endif

