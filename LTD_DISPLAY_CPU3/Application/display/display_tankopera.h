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

/* 菜单索引号 */
typedef enum {

    /* ===== 入口/主菜单/测量/维护 ===== */
    KEYNUM_IF_ENTER_MAINMENU = 0,        // 是否进入主菜单
    KEYNUM_MAINMENU,                     // 主菜单
    KEYNUM_IF_EXIT_MAINMENU,             // 是否退出主菜单
    KEYNUM_MEASURE_MAINMENU,             // 普通测量指令主菜单
    KEYNUM_MENU_PARACFG_MAIN,            // 参数配置主菜单（新）
    KEYNUM_MENU_CMD_MAIN,                // 维护/调试指令主菜单
    KEYNUM_IFSENDCMD,                    // 是否下发指令或参数
    KEYNUM_INPUTCMDPARA,                 // 输入参数值(带参指令)
    KEYNUM_DISPLAY_PARA,                 // 参数显示(读写类参数)
    KEYNUM_WORDSELECT,                   // 隐藏信息选择

    /* ===== 界面显示类（如果你还有单独“语言”页） ===== */
    KEYNUM_MENU_LANGUAGE,                // 显示设置 - 语言

    /* ===== 参数配置（新分组页面） ===== */
    KEYNUM_MENU_PARA_DEV_INFO,           // 设备信息参数
    KEYNUM_MENU_PARA_MECH,               // 机械参数
    KEYNUM_MENU_PARA_WEIGHT,             // 称重参数（如果你确实有此页）
    KEYNUM_MENU_PARA_ZERO,               // 零点参数
    KEYNUM_MENU_PARA_LIQUID,             // 液位参数（如果你确实有此页）
    KEYNUM_MENU_PARA_WATER,              // 水位参数（如果你确实有此页）
    KEYNUM_MENU_PARA_BOTTOM_TANKH,       // 罐底/罐高参数（如果你确实有此页）
    KEYNUM_MENU_PARA_CORR,               // 修正参数（磁通量/温度/密度修正）
    KEYNUM_MENU_PARA_POLICY,             // 策略/分布/区间参数
    KEYNUM_MENU_PARA_WARTSILA,           // Wartsila 参数（如果你确实有此页）
    KEYNUM_MENU_PARA_DO,                 // 报警 DO 参数（如果你确实有此页）
    KEYNUM_MENU_PARA_AO,                 // AO 参数（如果你确实有此页）
    KEYNUM_MENU_PARA_CAL_SP,             // 标定/单点参数
    KEYNUM_MENU_PARA_PARAM_CHECK,        // 参数校验信息

    /* ===== CPU3（拆分页面） ===== */
    KEYNUM_MENU_CPU3_BASE,               // CPU3 - 基本参数
    KEYNUM_MENU_CPU3_SOURCE,             // CPU3 - 数据源
    KEYNUM_MENU_CPU3_INPUT,              // CPU3 - 手输值
    KEYNUM_MENU_CPU3_SCREEN,             // CPU3 - 屏幕参数
    KEYNUM_MENU_CPU3_COM1,               // CPU3 - COM1
    KEYNUM_MENU_CPU3_COM2,               // CPU3 - COM2
    KEYNUM_MENU_CPU3_COM3,               // CPU3 - COM3

    KEYNUM_END
} keymenuNumber;


typedef enum {
    MENU_GRP_DEV_INFO = 0,      // 传感器类型/编号/版本/软件版本/上电默认等
    MENU_GRP_MECH,              // 编码轮周长/首圈周长/尺带厚度/电机速度等
    MENU_GRP_WEIGHT,            // 空载/满载/上下限/比例
    MENU_GRP_ZERO,              // 零点阈值/忽略区/最大偏差/找零下行距离
    MENU_GRP_LIQUID,            // 罐高/液位距差/盲区/阈值/滞后/测量方式
    MENU_GRP_WATER,             // 水罐高/水位距差/盲区/电容阈值/滞回/最大下行距离
    MENU_GRP_BOTTOM_TANKH,      // 罐底模式/角度阈值/称重阈值/更新罐高标志/实高偏差/初始/当前
    MENU_GRP_CORR,              // 密度修正/温度修正
    MENU_GRP_POLICY,            // 是否测罐底/是否测水/是否测单点/顺序/模式/点数/间距/悬停/上下限
    MENU_GRP_WARTSILA,          // Wartsila 上下限/步进/最高点距液面
    MENU_GRP_DO_ALARM,          // DO 报警
    MENU_GRP_AO,                // AO 输出/报警/故障电流/调试电流
    MENU_GRP_CAL_SP,            // 标定液位(油/水)/单点位置/监测位置/分布液位/电机运行距离
    MENU_GRP_PARAM_CHECK,       // ParamVer/StructSize/Magic/CRC
    MENU_GRP_CPU3_BASE,         // LedVer/语言
    MENU_GRP_CPU3_SOURCE,       // SrcOil/SrcWater/SrcD/SrcT
    MENU_GRP_CPU3_INPUT,        // InOil/InWater/InD/InDSw/InT
    MENU_GRP_CPU3_SCREEN,       // Decimal/密码/息屏
    MENU_GRP_CPU3_COM1,
    MENU_GRP_CPU3_COM2,
    MENU_GRP_CPU3_COM3,
} MenuGroup;

/**
 * @brief CPU3 显示通信单元 操作码（OperatingNumber）
 *
 * 设计目的：
 *  1) 操作码用于 UI 菜单项、参数表 param_meta[]、以及与 CPU2/CPU3 通讯时的“功能选择”索引。
 *  2) 本枚举分为：指令类（无参/带参）、参数配置类（CPU2 参数 / CPU3 本机参数）、密码类。
 *
 * 约束与约定：
 *  - CPU2 参数区（COM_NUM_PARA_DEBUG_START ~ COM_NUM_PARA_DEBUG_END）顺序需与 CPU2 的
 *    HOLDREGISTER_DEVICEPARAM_*（新 HOLD 表）保持一致，否则会出现“写错寄存器地址/显示错位”。
 *  - 本枚举未显式赋值，默认从 0 递增；若需要兼容历史版本，可在关键项上显式赋值固定编号。
 */

typedef enum
{
    /* =======================================================================
     * 0) 空操作
     * ======================================================================= */
    COM_NUM_NOOPERA = 0,             // 无指令（且无特殊跳转）

    /* =======================================================================
     * 1) 不带参指令（线圈/命令类）- 开始
     * ======================================================================= */
    COM_NUM_NOPARACMD_START,         // 不带参指令类 - 开始

    /* ----------------------- 1.1 普通模式：无参测量指令 ----------------------- */
    COM_NUM_NOPARACMD_NORMAL_START,  // 普通无参测量指令 - 开始

    COM_NUM_BACK_ZERO,               // 回零点（CMD_BACK_ZERO）
    COM_NUM_FIND_OIL,                // 寻找液位（CMD_FIND_OIL）
    COM_NUM_FIND_WATER,              // 寻找水位（CMD_FIND_WATER）
    COM_NUM_FIND_BOTTOM,             // 寻找罐底（CMD_FIND_BOTTOM）
    COM_NUM_SYNTHETIC,               // 综合测量（CMD_SYNTHETIC）
    COM_NUM_FOLLOW_WATER,            // 水位跟随（CMD_FOLLOW_WATER）
    COM_NUM_SPREADPOINTS,         	 // 分布测量（CMD_MEASURE_DISTRIBUTED / 你现有映射）
    COM_NUM_SPREADPOINTS_GB,         // 国标分布测量
    COM_NUM_METER_DENSITY,           // 密度每米测量（CMD_MEASURE_DENSITY_METER）
    COM_NUM_INTERVAL_DENSITY,        // 区间密度测量（CMD_MEASURE_DENSITY_RANGE）
    COM_NUM_WARTSILA_DENSITY,        // Wartsila 密度区间测量（CMD_WARTSILA_DENSITY_RANGE）
	COM_NUM_READ_PART_PARAMS,        // 读取部件参数（CMD_READ_PART_PARAMS / READ_COMPONENT_PARAMS）

    COM_NUM_NOPARACMD_NORMAL_STOP,   // 普通无参测量指令 - 结束

    /* ----------------------- 1.2 调试模式：无参指令 ----------------------- */
    COM_NUM_DEBUGCMD_START,          // 调试模式无参指令 - 开始

    COM_NUM_FIND_ZERO,               // 标定零点（通常对应 CMD_CALIBRATE_ZERO 或旧流程）
    COM_NUM_FORCE_LIFT_ZERO,         // 强制提零点（CMD_FORCE_LIFT_ZERO）
    COM_NUM_SET_EMPTY_WEIGHT,        // 设置空载称重
    COM_NUM_SET_FULL_WEIGHT,         // 设置满载称重
    COM_NUM_RESTOR_EFACTORYSETTING,  // 恢复出厂设置
    COM_NUM_MAINTENANCE_MODE,        // 维护模式

    COM_NUM_DEBUGCMD_STOP,           // 调试模式无参指令 - 结束

    COM_NUM_NOPARACMD_END,           // 不带参指令类 - 结束

    /* =======================================================================
     * 2) 工作模式：带一个参数的指令（例如需要“位置/高度/距离”等）
     * ======================================================================= */
    COM_NUM_ONEPARACMD_START,        // 工作模式带参指令 - 开始

    COM_NUM_SINGLE_POINT,            // 单点测量（CMD_MEASURE_SINGLE，参数：单点测量位置）
    COM_NUM_SP_TEST,                 // 单点监测（CMD_MONITOR_SINGLE，参数：单点监测位置）
    COM_NUM_RUN_TO_POSITION,         // 电机运行到指定位置（CMD_RUN_TO_POSITION）

    COM_NUM_ONEPARACMD_END,          // 工作模式带参指令 - 结束

    /* =======================================================================
     * 3) 调试模式：带一个参数的指令
     * ======================================================================= */
    COM_NUM_ONEPARA_DEBUGCMD_START,  // 调试模式带参指令 - 开始

    COM_NUM_CAL_OIL,                 // 液位标定（CMD_CALIBRATE_OIL，参数：标定值或位置）
    COM_NUM_CORRECTION_OIL,          // 修正液位（CMD_CORRECT_OIL，参数：修正值）
    COM_NUM_CALIBRATE_WATER,         // 水位标定（CMD_CALIBRATE_WATER）
    COM_NUM_RUNUP,                   // 向上运行（CMD_MOVE_UP，参数：距离）
    COM_NUM_RUNDOWN,                 // 向下运行（CMD_MOVE_DOWN，参数：距离）
    COM_NUM_FORCE_RUNUP,             // 电机强制上行（CMD_FORCE_MOVE_UP）
    COM_NUM_FORCE_RUNDOWN,           // 电机强制下行（CMD_FORCE_MOVE_DOWN）
    COM_NUM_NOPARA_DEBUGCMD_END,     // 调试模式带参指令 - 结束

    /* =======================================================================
     * 4) 参数配置类 - 开始
     * ======================================================================= */
    COM_NUM_PARACONFIG_START,        // 参数配置类 - 开始

    /* =======================================================================
     * 4.1 CPU2 基础参数（CPU2 存储参数，顺序必须与新 HOLDREGISTER_DEVICEPARAM_* 一致）
     * ======================================================================= */
    COM_NUM_PARA_DEBUG_START,        // CPU2 基础参数 - 开始

    COM_NUM_DEVICEPARAM_COMMAND,     // 指令寄存器（HOLDREGISTER_DEVICEPARAM_COMMAND）

    /* ---------------- 基础参数 ---------------- */
    COM_NUM_DEVICEPARAM_SENSORTYPE,                     // 传感器类型
    COM_NUM_DEVICEPARAM_SENSORID,                       // 传感器编号
    COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,        // 传感器软件版本
    COM_NUM_DEVICEPARAM_SOFTWAREVERSION,                // 主程序软件版本
    COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,       // 上电默认指令
    COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO,           // 故障自动回零开关/策略
    COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT,         // 故障停止测量开关/策略

    COM_NUM_DEVICEPARAM_RESERVED1,                      // 保留 1
    COM_NUM_DEVICEPARAM_RESERVED2,                      // 保留 2
    COM_NUM_DEVICEPARAM_RESERVED3,                      // 保留 3
    COM_NUM_DEVICEPARAM_RESERVED4,                      // 保留 4
    COM_NUM_DEVICEPARAM_RESERVED5,                      // 保留 5

    /* ---------------- 电机与编码器参数 ---------------- */
    COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, // 编码轮周长
    COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED,                // 最大电机速度
    COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,    // 尺带首圈周长
    COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM,              // 尺带厚度

    COM_NUM_DEVICEPARAM_RESERVED6,                      // 保留 6
    COM_NUM_DEVICEPARAM_RESERVED7,                      // 保留 7

    /* ---------------- 称重参数 ---------------- */
    COM_NUM_DEVICEPARAM_EMPTY_WEIGHT,                   // 空载重量
    COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,       // 空载重量上限
    COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,       // 空载重量下限
    COM_NUM_DEVICEPARAM_FULL_WEIGHT,                    // 满载重量
    COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,        // 满载重量上限
    COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,        // 满载重量下限
    COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,       // 称重上限比例
    COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,       // 称重下限比例

    COM_NUM_DEVICEPARAM_RESERVED8,                      // 保留 8
    COM_NUM_DEVICEPARAM_RESERVED9,                      // 保留 9

    /* ---------------- 零点测量参数 ---------------- */
    COM_NUM_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO,    // 零点称重阈值比例
    COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE,             // 称重忽略区间（抗抖/滤波区）
    COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE,    // 零点最大偏差距离
    COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,         // 找零点下行距离

    COM_NUM_DEVICEPARAM_RESERVED10,                     // 保留 10
    COM_NUM_DEVICEPARAM_RESERVED11,                     // 保留 11

    /* ---------------- 液位测量参数 ---------------- */
    COM_NUM_DEVICEPARAM_TANKHEIGHT,                     // 罐高
    COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF,    // 液位传感器距离差/安装偏差
    COM_NUM_DEVICEPARAM_BLINDZONE,                      // 液位盲区
    COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD,              // 液位阈值
    COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,  // 液位滞回阈值
    COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,   // 液位测量方式
    COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY,             // 液位跟随频率值
    COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY,               // 液位跟随密度值

    /* ---------------- 水位测量参数 ---------------- */
    COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT,              // 水罐高/水参考高度
    COM_NUM_DEVICEPARAM_WATER_LEVEL_SENSOR_DISTANCE_DIFF,// 水位传感器距离差/安装偏差
    COM_NUM_DEVICEPARAM_WATER_BLINDZONE,                // 水位盲区
    COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD,            // 水位电容阈值
    COM_NUM_DEVICEPARAM_WATER_CAP_HYSTERESIS,           // 水位电容滞回
    COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE,                // 最大下行距离
    COM_NUM_DEVICEPARAM_ZERO_CAP,                     // 保留 14

    COM_NUM_DEVICEPARAM_RESERVED15,                     // 保留 15

    /* ---------------- 罐底/罐高测量参数 ---------------- */
    COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE,             // 罐底检测模式
    COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD,         // 罐底角度阈值
    COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,        // 罐底称重阈值
    COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,        // 刷新罐高标志
    COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION,       // 罐高最大偏差
    COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT,             // 初始罐高
    COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT,             // 当前罐高

    COM_NUM_DEVICEPARAM_RESERVED16,                     // 保留 16
    COM_NUM_DEVICEPARAM_RESERVED17,                     // 保留 17

    /* ---------------- 密度与温度修正 ---------------- */
    COM_NUM_DEVICEPARAM_DENSITYCORRECTION,              // 密度修正
    COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION,          // 温度修正

    COM_NUM_DEVICEPARAM_RESERVED18,                     // 保留 18
    COM_NUM_DEVICEPARAM_RESERVED19,                     // 保留 19

    /* ---------------- 分布/区间测量参数 ---------------- */
    COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,       // 是否测罐底
    COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT,        // 是否测水位
    COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,      // 是否测单点密度
    COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER,         // 分布测量顺序
    COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE,          // 分布测量模式
    COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT,         // 分布测量点数
    COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,      // 分布测量间距
    COM_NUM_DEVICEPARAM_SPREADTOPLIMIT,                 // 分布上限（距液面）
    COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT,              // 分布下限（距罐底）
    COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,        // 首点悬停时间
    COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT,              // 区间上限
    COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT,           // 区间下限

    COM_NUM_DEVICEPARAM_RESERVED20,                     // 保留 20
    COM_NUM_DEVICEPARAM_RESERVED21,                     // 保留 21

    /* ---------------- Wartsila 区间密度测量参数 ---------------- */
    COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,   // Wartsila 上限密度
    COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,   // Wartsila 下限密度
    COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,      // Wartsila 密度步进
    COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,// 最高测点距液面

    COM_NUM_DEVICEPARAM_RESERVED22,                     // 保留 22
    COM_NUM_DEVICEPARAM_RESERVED23,                     // 保留 23

    /* ---------------- 继电器报警 DO ---------------- */
    COM_NUM_DEVICEPARAM_ALARM_HIGH_DO,                  // 高报警 DO
    COM_NUM_DEVICEPARAM_ALARM_LOW_DO,                   // 低报警 DO
    COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD,          // 第三状态阈值

    COM_NUM_DEVICEPARAM_RESERVED24,                     // 保留 24
    COM_NUM_DEVICEPARAM_RESERVED25,                     // 保留 25

    /* ---------------- 4-20mA / 报警 AO ---------------- */
    COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA,         // 电流量程起点
    COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA,           // 电流量程终点
    COM_NUM_DEVICEPARAM_ALARM_HIGH_AO,                  // 高报警 AO
    COM_NUM_DEVICEPARAM_ALARM_LOW_AO,                   // 低报警 AO
    COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA,             // 初始化电流
    COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA,             // AO 高报电流
    COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA,              // AO 低报电流
    COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA,               // 故障电流
    COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA,               // 调试电流

    COM_NUM_DEVICEPARAM_RESERVED26,                     // 保留 26
    COM_NUM_DEVICEPARAM_RESERVED27,                     // 保留 27

    /* ---------------- 指令参数（用于带参命令） ---------------- */
    COM_NUM_DEVICEPARAM_CALIBRATE_OIL_LEVEL,            // 液位标定值
    COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL,          // 水位标定值
    COM_NUM_DEVICEPARAM_SP_MEAS_POSITION,               // 单点测量位置
    COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION,            // 单点监测位置
    COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL, // 密度分布测量液位
    COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,         // 电机指令运行距离

    COM_NUM_DEVICEPARAM_RESERVED28,                     // 保留 28
    COM_NUM_DEVICEPARAM_RESERVED29,                     // 保留 29

    /* ---------------- 元信息与校验 ---------------- */
    COM_NUM_DEVICEPARAM_PARAM_VERSION,                  // 参数版本号
    COM_NUM_DEVICEPARAM_STRUCT_SIZE,                    // 结构体大小
    COM_NUM_DEVICEPARAM_MAGIC,                          // 魔术字
    COM_NUM_DEVICEPARAM_CRC,                            // CRC 校验

    COM_NUM_PARA_DEBUG_END,                             // CPU2 基础参数 - 结束

    /* =======================================================================
     * 4.2 CPU3 本机参数（CPU3 存储参数，通常只影响显示/通讯口配置等）
     * ======================================================================= */
    COM_NUM_PARA_LOCAL_START,         // CPU3 本机参数 - 开始

    COM_NUM_PARA_LOCAL_LEDVERSION,    // 屏幕程序版本
    COM_NUM_PARA_LANG,                // 语言

    /* 数据源配置（显示取值来源） */
    COM_NUM_SOURCE_START,
    COM_NUM_SCREEN_SOURCE_OIL,        // 液位来源
    COM_NUM_SCREEN_SOURCE_WATER,      // 水位来源
    COM_NUM_SCREEN_SOURCE_D,          // 密度来源
    COM_NUM_SCREEN_SOURCE_T,          // 温度来源
    COM_NUM_SCREEN_INPUT_OIL,         // 手工输入液位
    COM_NUM_SCREEN_INPUT_WATER,       // 手工输入水位
    COM_NUM_SCREEN_INPUT_D,           // 手工输入密度
    COM_NUM_SCREEN_INPUT_D_SWITCH,    // 密度上传开关
    COM_NUM_SCREEN_INPUT_T,           // 手工输入温度
    COM_NUM_SOURCE_STOP,

    /* 界面显示类参数 */
    COM_NUM_SCREEN_STR,               // 字符串/显示控制（保留用）
    COM_NUM_SCREEN_DECIMAL,           // 小数位设置
    COM_NUM_SCREEN_PASSWARD,          // 密码
    COM_NUM_SCREEN_OFF,               // 熄屏时间
    COM_NUM_SCREEN_STP,               // 其他显示/步进参数（保留）

    /* CPU3 通信口参数（本机参数的一部分） */
    COM_NUM_CPU3_COM1_BAUDRATE,
    COM_NUM_CPU3_COM1_DATABITS,
    COM_NUM_CPU3_COM1_PARITY,
    COM_NUM_CPU3_COM1_STOPBITS,
    COM_NUM_CPU3_COM1_PROTOCOL,

    COM_NUM_CPU3_COM2_BAUDRATE,
    COM_NUM_CPU3_COM2_DATABITS,
    COM_NUM_CPU3_COM2_PARITY,
    COM_NUM_CPU3_COM2_STOPBITS,
    COM_NUM_CPU3_COM2_PROTOCOL,

    COM_NUM_CPU3_COM3_BAUDRATE,
    COM_NUM_CPU3_COM3_DATABITS,
    COM_NUM_CPU3_COM3_PARITY,
    COM_NUM_CPU3_COM3_STOPBITS,
    COM_NUM_CPU3_COM3_PROTOCOL,

    COM_NUM_PARA_LOCAL_STOP,          // CPU3 本机参数 - 结束

    COM_NUM_PARACONFIG_END,           // 参数配置类 - 结束

    /* =======================================================================
     * 5) 密码类
     * ======================================================================= */
    COM_NUM_PASSWORD_START,           // 密码类 - 开始
    COM_NUM_PASSWORD_ENTER_PARA,      // 进入参数配置的密码
    COM_NUM_PASSWORD_ENTER_CMD,       // 进入调试指令的密码
    COM_NUM_PASSWORD_END,             // 密码类 - 结束

    /* =======================================================================
     * 6) 结束
     * ======================================================================= */
    COM_NUM_END                        // 操作码结束标志
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

