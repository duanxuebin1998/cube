/*
 * display_tankopera.c
 *
 * 说明:
 *	1) 罐上操作菜单系统(按键: 上/下/确认/返回)
 *	2) 指令下发与参数读写(与 CPU2 通讯)
 *	3) OLED 菜单显示(中/英切换)
 *
 */

#include "cpu2_communicate.h"
#include "display_tankopera.h"
#include "tim.h"
#include "usart.h"
#include "exit.h"
#include "display.h"
#include "hgs.h"
#include <math.h>
#include <stdio.h>
#include "system_parameter.h"
#include "cpu3_comm_display_params.h"
#include "cpu3_clock.h"
#include "si_modbus_slave.h"
#include "app_version.h"
#include "system_parameter.h"
#include <string.h>    /* for memset, memcpy, strcmp, strlen... */

/* 进入主参数菜单所需的固定密码 1009；该宏只定义本机菜单校验值，不构成安全认证机制。 */
#define PASSWORD_ENTERMAIN		1009
/* 电机启动后 5000 ms 的监测宽限时间；宽限期内避免把电流尚未稳定误判为运行异常。 */
#define MOTOR_RUN_MONITOR_START_GRACE_MS 5000U
/* 电机运行监测页数值区域使用的 OLED 纵向坐标；当前取八行布局的第 4 行。 */
#define MOTOR_RUN_MONITOR_VALUE_LINE OLED_LINE8_4
/* PET 钢带厚度预设值 0.300 mm，存储单位为 0.001 mm。 */
#define TAPE_THICKNESS_PET_001MM 300
/* PEEK 钢带厚度预设值 0.500 mm，存储单位为 0.001 mm。 */
#define TAPE_THICKNESS_PEEK_001MM 500
/* ETFE 钢带厚度预设值 1.100 mm，存储单位为 0.001 mm。 */
#define TAPE_THICKNESS_ETFE_001MM 1100
/* 钢带厚度菜单中的自定义项索引 3；选中后使用用户参数而不是前三个材料预设值。 */
#define TAPE_THICKNESS_CUSTOM_INDEX 3
/* 电机 IRUN 值到 RMS 电流查表索引的偏移量；最小合法 IRUN 映射为表索引 0。 */
#define MOTOR_CURRENT_RMS_TABLE_OFFSET MOTOR_CURRENT_MIN
/* AO 量程 0% 和 100% 两个 UInt32 值合计占用的 16 位寄存器数量 4；用于成对读取和写入，禁止只更新半个 32 位值。 */
#define AO_RANGE_PAIR_REGISTER_COUNT 4U
/* AO 运行来源编码 0：尚未形成有效业务输出的初始状态。 */
#define AO_RUNTIME_SOURCE_INITIAL     0U
/* AO 运行来源编码 1：由有效过程量按量程换算得到。 */
#define AO_RUNTIME_SOURCE_PROCESS     1U
/* AO 运行来源编码 2：由故障输出策略选择。 */
#define AO_RUNTIME_SOURCE_FAULT       2U
/* AO 运行来源编码 3：使用调试模拟电流。 */
#define AO_RUNTIME_SOURCE_SIMULATION  3U
/* AO 运行来源编码 4：使用固定电流配置。 */
#define AO_RUNTIME_SOURCE_FIXED       4U
/* AO 运行来源编码 5：通道禁用并输出禁用电流。 */
#define AO_RUNTIME_SOURCE_DISABLED    5U
/* AO 运行来源编码 6：驱动故障导致当前输出不可用。 */
#define AO_RUNTIME_SOURCE_DRIVER_ERR  6U
/* AO 运行来源编码 7：故障策略保持最近一次有效输出。 */
#define AO_RUNTIME_SOURCE_HOLD_LAST   7U
/* AO 运行来源有效编码数量 8；合法值为 0～7，用于菜单文本表和边界校验。 */
#define AO_RUNTIME_SOURCE_COUNT       8U
/* CPU3 无法识别 AO 运行来源时使用的哨兵值；取有效编码数量 8，明确落在 0～7 合法范围之外。 */
#define AO_RUNTIME_SOURCE_UNAVAILABLE AO_RUNTIME_SOURCE_COUNT
/* 完整 AO 配置块占用的 16 位寄存器数量；由首末保持寄存器地址和统一步长计算，避免字段新增后手工计数失配。 */
#define AO_CONFIG_REGISTER_COUNT ((uint16_t)(HOLDREGISTER_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100 - \
                                             HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE + REG_STRIDE))
extern volatile uint8_t g_cpu3_uart_reinit_pending; /* CPU3 串口重初始化标志 */

typedef void (*pFunc_void)(void);

typedef struct
{
	int menu_num;			/* 菜单第几栏序号(从 0 开始) */
	int menu_cnt;			/* 上下键计数(从 1 开始计数, 便于取模) */
	int menu_page;			/* 当前显示的页数(页起始项) */
} PAGENUM_T;

 PAGENUM_T PageNum[KEYNUM_END]; /* 各菜单层级的光标与分页运行态数组；每项分别保存当前选中序号、上下键循环计数和本页首项索引，ClearPageNum 统一复位。 */
 struct ParaContent now_Para_CT;		/* 当前设置的参数内容 */

static int func_index = 0;					/* 菜单索引 */
static int NowKeyPress = 0;					/* 本次按下的按键 */
static int now_Opera_Num = 0;				/* 当前选择的指令或参数的序号 */
static int timesure = 0;					/* 按下确定键的次数 */
static int timeback = 0;					/* 按下返回键的次数 */
static int debugmode_back = 0;				/* 进入调试模式返回到哪个菜单 */
static uint32_t motor_run_monitor_enter_tick = 0U; /* 电机监控页进入时刻 */
static bool motor_run_monitor_started = false; /* 电机监控页是否已观察到运行态 */
static int debug_weight_wait_opera = COM_NUM_NOOPERA; /* 扭力等待页对应的指令 */
static bool debug_weight_wait_started = false; /* 扭力等待页是否已进入本次等待周期 */
static bool debug_weight_wait_ignore_initial_done = false; /* 扭力等待页是否忽略进入前残留完成态 */
static Cpu3DateTime rtc_menu_dt = {0};
/* RTC 设置页面当前选中的年、月、日、时、分或秒字段索引。 */
static uint8_t rtc_menu_field = 0U;
/* AO 仿真页面当前选择的启停操作项。 */
static uint32_t ao_simulation_selection = 0U;
/* CPU2 通信健康详情当前显示的分页索引。 */
static uint8_t cpu2_comm_health_page = 0U;

/* ==============================
 * 枚举/隐藏含义文字表
 * 注意: 中文使用常用字, 便于 GBK
 * ============================== */
static uint8_t *arr_densitydir[][2] = {
	{ (uint8_t*)"向上", (uint8_t*)"UP" },
	{ (uint8_t*)"向下", (uint8_t*)"DOWN" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_source[][2] = {
	{ (uint8_t*)"设备测量", (uint8_t*)"Measurement" },
	{ (uint8_t*)"手工输入", (uint8_t*)"Input" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_IF[][2] = {
	{ (uint8_t*)"否", (uint8_t*)"NO" },
	{ (uint8_t*)"是", (uint8_t*)"YES" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_position_source_auto_switch[][2] = {
	{ (uint8_t*)"禁用", (uint8_t*)"Disabled" },
	{ (uint8_t*)"启用", (uint8_t*)"Enabled" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_ao_work_mode[][2] = {
	{ (uint8_t*)"禁用", (uint8_t*)"Disabled" },
	{ (uint8_t*)"4-20mA输出", (uint8_t*)"4-20mA Output" },
	{ (uint8_t*)"HART从站+输出", (uint8_t*)"HART+Output" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_ao_current_mode[][2] = {
	{ (uint8_t*)"NE 4-20mA", (uint8_t*)"NE 4-20mA" },
	{ (uint8_t*)"US 4-20mA", (uint8_t*)"US 4-20mA" },
	{ (uint8_t*)"普通4-20mA", (uint8_t*)"Normal 4-20mA" },
	{ (uint8_t*)"固定电流", (uint8_t*)"Fixed Current" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_ao_output_source[][2] = {
	{ (uint8_t*)"储罐液位", (uint8_t*)"Tank Level" },
	{ (uint8_t*)"传感器位置", (uint8_t*)"Sensor Pos" },
	{ (uint8_t*)"水位", (uint8_t*)"Water Level" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_ao_fault_mode[][2] = {
	{ (uint8_t*)"故障电流", (uint8_t*)"Fault Current" },
	{ (uint8_t*)"保持上次", (uint8_t*)"Hold Last" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_ao_runtime_source[][2] = {
	{ (uint8_t*)"AO:初始", (uint8_t*)"AO:Initial" },
	{ (uint8_t*)"AO:过程", (uint8_t*)"AO:Process" },
	{ (uint8_t*)"AO:故障", (uint8_t*)"AO:Fault" },
	{ (uint8_t*)"AO:模拟", (uint8_t*)"AO:Simulation" },
	{ (uint8_t*)"AO:固定", (uint8_t*)"AO:Fixed" },
	{ (uint8_t*)"AO:禁用", (uint8_t*)"AO:Disabled" },
	{ (uint8_t*)"AO:驱动故障", (uint8_t*)"AO:Driver Err" },
	{ (uint8_t*)"AO:保持", (uint8_t*)"AO:Hold Last" },
	{ (uint8_t*)"AO:N/A", (uint8_t*)"AO:Unavailable" },
};

static uint8_t *arr_ao_simulation_enable[][2] = {
	{ (uint8_t*)"关", (uint8_t*)"Off" },
	{ (uint8_t*)"开", (uint8_t*)"On" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_position_count_mode[][2] = {
	{ (uint8_t*)"编码器", (uint8_t*)"Encoder" },
	{ (uint8_t*)"电机", (uint8_t*)"Motor" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_tape_thickness[][2] = {
	{ (uint8_t*)"PET 0.300", (uint8_t*)"PET 0.300" },
	{ (uint8_t*)"PEEK 0.500", (uint8_t*)"PEEK 0.500" },
	{ (uint8_t*)"ETFE 1.100", (uint8_t*)"ETFE 1.100" },
	{ (uint8_t*)"手输", (uint8_t*)"Custom" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_motor_current[][2] = {
	{ (uint8_t*)"01 0.08A", (uint8_t*)"01 0.08A" },
	{ (uint8_t*)"02 0.13A", (uint8_t*)"02 0.13A" },
	{ (uint8_t*)"03 0.17A", (uint8_t*)"03 0.17A" },
	{ (uint8_t*)"04 0.21A", (uint8_t*)"04 0.21A" },
	{ (uint8_t*)"05 0.25A", (uint8_t*)"05 0.25A" },
	{ (uint8_t*)"06 0.30A", (uint8_t*)"06 0.30A" },
	{ (uint8_t*)"07 0.34A", (uint8_t*)"07 0.34A" },
	{ (uint8_t*)"08 0.38A", (uint8_t*)"08 0.38A" },
	{ (uint8_t*)"09 0.42A", (uint8_t*)"09 0.42A" },
	{ (uint8_t*)"10 0.46A", (uint8_t*)"10 0.46A" },
	{ (uint8_t*)"11 0.51A", (uint8_t*)"11 0.51A" },
	{ (uint8_t*)"12 0.55A", (uint8_t*)"12 0.55A" },
	{ (uint8_t*)"13 0.59A", (uint8_t*)"13 0.59A" },
	{ (uint8_t*)"14 0.63A", (uint8_t*)"14 0.63A" },
	{ (uint8_t*)"15 0.68A", (uint8_t*)"15 0.68A" },
	{ (uint8_t*)"16 0.72A", (uint8_t*)"16 0.72A" },
	{ (uint8_t*)"17 0.76A", (uint8_t*)"17 0.76A" },
	{ (uint8_t*)"18 0.80A", (uint8_t*)"18 0.80A" },
	{ (uint8_t*)"19 0.84A", (uint8_t*)"19 0.84A" },
	{ (uint8_t*)"20 0.89A", (uint8_t*)"20 0.89A" },
	{ (uint8_t*)"21 0.93A", (uint8_t*)"21 0.93A" },
	{ (uint8_t*)"22 0.97A", (uint8_t*)"22 0.97A" },
	{ (uint8_t*)"23 1.01A", (uint8_t*)"23 1.01A" },
	{ (uint8_t*)"24 1.06A", (uint8_t*)"24 1.06A" },
	{ (uint8_t*)"25 1.10A", (uint8_t*)"25 1.10A" },
	{ (uint8_t*)"26 1.14A", (uint8_t*)"26 1.14A" },
	{ (uint8_t*)"27 1.18A", (uint8_t*)"27 1.18A" },
	{ (uint8_t*)"28 1.23A", (uint8_t*)"28 1.23A" },
	{ (uint8_t*)"29 1.27A", (uint8_t*)"29 1.27A" },
	{ (uint8_t*)"30 1.31A", (uint8_t*)"30 1.31A" },
	{ (uint8_t*)"31 1.35A", (uint8_t*)"31 1.35A" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static const uint16_t motor_current_rms_ma_table[] = {
	84U, 127U, 169U, 211U, 253U, 296U, 338U, 380U,
	422U, 465U, 507U, 549U, 591U, 634U, 676U, 718U,
	760U, 803U, 845U, 887U, 929U, 972U, 1014U, 1056U,
	1098U, 1141U, 1183U, 1225U, 1267U, 1310U, 1352U,
};

static uint8_t *arr_densitymode[][2] = {
	{ (uint8_t*)"分布测量", (uint8_t*)"Distribution" },
	{ (uint8_t*)"国标测量", (uint8_t*)"National Standard" },
	{ (uint8_t*)"每米测量", (uint8_t*)"Per Meter" },
	{ (uint8_t*)"区间测量", (uint8_t*)"Interval" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_Switch[][2] = {
	{ (uint8_t*)"关闭", (uint8_t*)"CLOSE" },
	{ (uint8_t*)"开启", (uint8_t*)"OPEN" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_language[][2] = {
	{ (uint8_t*)"中文", (uint8_t*)"Chinese" },
	{ (uint8_t*)"英文", (uint8_t*)"English" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_oled_brightness[][2] = {
	{ (uint8_t*)"低", (uint8_t*)"Dark" },
	{ (uint8_t*)"中低", (uint8_t*)"Low" },
	{ (uint8_t*)"中", (uint8_t*)"Standard" },
	{ (uint8_t*)"中高", (uint8_t*)"High" },
	{ (uint8_t*)"高", (uint8_t*)"Bright" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_baudrate[][2] = {
	{ (uint8_t*)"1200", (uint8_t*)"1200" },
	{ (uint8_t*)"2400", (uint8_t*)"2400" },
	{ (uint8_t*)"4800", (uint8_t*)"4800" },
	{ (uint8_t*)"9600", (uint8_t*)"9600" },
	{ (uint8_t*)"19200", (uint8_t*)"19200" },
	{ (uint8_t*)"38400", (uint8_t*)"38400" },
	{ (uint8_t*)"57600", (uint8_t*)"57600" },
	{ (uint8_t*)"115200", (uint8_t*)"115200" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_databits[][2] = {
	{ (uint8_t*)"8位", (uint8_t*)"8 bits" },
	{ (uint8_t*)"9位", (uint8_t*)"9 bits" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_parity[][2] = {
	{ (uint8_t*)"无校验", (uint8_t*)"NO parity" },
	{ (uint8_t*)"偶校验", (uint8_t*)"Even parity" },
	{ (uint8_t*)"奇校验", (uint8_t*)"Odd parity" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_stopbits[][2] = {
	{ (uint8_t*)"1位", (uint8_t*)"1 bits" },
	{ (uint8_t*)"2位", (uint8_t*)"2 bits" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_protocol[][2] = {
	{ (uint8_t*)"计量仪协议", (uint8_t*)"DSM" },
	{ (uint8_t*)"瓦锡兰协议", (uint8_t*)"Wartsila LTD" },
	{ (uint8_t*)"LTD协议", (uint8_t*)"LTD" },
	{ (uint8_t*)"LH协议", (uint8_t*)"LH" },
	{ (uint8_t*)"SI协议", (uint8_t*)"SI" }, /* 显示侧只暴露协议选择，具体串口参数由配置归一化自动处理。 */
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_bottom[][2] = {
	{ (uint8_t*)"扭力", (uint8_t*)"torque" },
	{ (uint8_t*)"角度", (uint8_t*)"angle" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *default_cmmand[][2] = {
	{ (uint8_t*)"无", (uint8_t*)"NONE" },
	{ (uint8_t*)"回零点", (uint8_t*)"BACK ZERO" },
	{ (uint8_t*)"寻找液位", (uint8_t*)"FIND OIL" },
	{ (uint8_t*)"单点监测", (uint8_t*)"MONITOR SINGLE" },
	{ (uint8_t*)"水位跟随", (uint8_t*)"FOLLOW WATER" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *level_mode[][2] = {
    { (uint8_t*)"相对频率",   (uint8_t*)"Rel Frequency" },
    { (uint8_t*)"定频",       (uint8_t*)"Fixed Freq" },
    { (uint8_t*)"密度找液位", (uint8_t*)"Density Level" },
    { (uint8_t*)"超声",       (uint8_t*)"Ultrasonic" },
    { (uint8_t*)"连相对频率", (uint8_t*)"Cont Rel Freq" },
    { (uint8_t*)"连定频",     (uint8_t*)"Cont Fixed" },
    { (uint8_t*)"非法配置",   (uint8_t*)"Illegal CFG" },
};

static uint8_t *water_level_mode[][2] = {
    { (uint8_t*)"低速模式", (uint8_t*)"Slow" },
    { (uint8_t*)"快速模式", (uint8_t*)"Fast" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};


/* 继电器工作模式数值到中英文菜单文本的映射表。 */
static uint8_t *arr_relay_operating[][2] = {
    { (uint8_t*)"禁用", (uint8_t*)"Disabled" },
    { (uint8_t*)"无源输出", (uint8_t*)"Passive Out" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

/* 继电器数字报警组合数值到中英文菜单文本的映射表。 */
static uint8_t *arr_relay_digital[][2] = {
    { (uint8_t*)"无", (uint8_t*)"None" },
    { (uint8_t*)"高报", (uint8_t*)"H" },
    { (uint8_t*)"高高报", (uint8_t*)"HH" },
    { (uint8_t*)"高/高高", (uint8_t*)"H or HH" },
    { (uint8_t*)"低报", (uint8_t*)"L" },
    { (uint8_t*)"低低报", (uint8_t*)"LL" },
    { (uint8_t*)"低/低低", (uint8_t*)"L or LL" },
    { (uint8_t*)"全部报警", (uint8_t*)"Any" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

/* 继电器常开/常闭接点类型到中英文菜单文本的映射表。 */
static uint8_t *arr_relay_contact[][2] = {
    { (uint8_t*)"常开", (uint8_t*)"NO" },
    { (uint8_t*)"常闭", (uint8_t*)"NC" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

/* 继电器报警关闭、开启和锁存模式到中英文菜单文本的映射表。 */
static uint8_t *arr_relay_alarm_mode[][2] = {
    { (uint8_t*)"关闭", (uint8_t*)"Off" },
    { (uint8_t*)"开启", (uint8_t*)"On" },
    { (uint8_t*)"Latch", (uint8_t*)"Latching" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

/* 继电器无效值策略到中英文菜单文本的映射表。 */
static uint8_t *arr_relay_error[][2] = {
    { (uint8_t*)"无报警", (uint8_t*)"No Alarm" },
    { (uint8_t*)"高高/高", (uint8_t*)"HH/H" },
    { (uint8_t*)"高", (uint8_t*)"H" },
    { (uint8_t*)"低", (uint8_t*)"L" },
    { (uint8_t*)"低低/低", (uint8_t*)"LL/L" },
    { (uint8_t*)"全部报警", (uint8_t*)"All" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

/* 继电器报警过程量来源到中英文菜单文本的映射表。 */
static uint8_t *arr_relay_source[][2] = {
    { (uint8_t*)"储罐液位", (uint8_t*)"Tank Level" },
    { (uint8_t*)"液相温度", (uint8_t*)"Liquid Temp" },
    { (uint8_t*)"水位", (uint8_t*)"Water Level" },
    { (uint8_t*)"浮子位置", (uint8_t*)"Displacer" },
    { (uint8_t*)"无", (uint8_t*)"None" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

/* =====================================================================
 * 静态函数声明
 *	按“模块职责”重新分类，便于快速定位
 * ===================================================================== */

/* ---------- 1) 菜单入口 / 页面跳转(顶层流程) ----------
 *	这些函数负责切换页面、组织菜单结构、决定下一步流程
 */
static void mainmenu(void);				/* 主菜单 */
static void measuremenu(void);			/* 测量命令菜单 */
static void menu_measure_water(void);	/* 测量命令 - 水位测量 */
static void menu_measure_density_single(void); /* 测量命令 - 密度单点测量 */
static void menu_measure_density_distribution(void); /* 测量命令 - 密度分布测量 */
/* static void menu_paraconfig(void); / * 参数配置主菜单 * / */
static void menu_cmdconfig_main(void);	/* 调试指令主菜单 */
static void menu_debug_float_motion(void); /* 调试指令 - 浮子运动控制 */
static void menu_debug_calibration(void); /* 调试指令 - 标定修正 */
static void menu_debug_weight(void);	/* 调试指令 - 扭力标定 */
static void menu_debug_wireless(void);	/* 调试指令 - 无线维护 */
static void menu_debug_system(void);	/* 调试指令 - 系统维护 */

/* ---------- 2) 参数分组子菜单(参数页面) ----------
 *	各参数分类页面，仅负责“列出参数项 + 跳转到参数读写流程”
 */
/* static void menu_tankbasicpara(void); / * 基础参数 * / */
/* static void menu_weightpara(void); / * 扭力/载荷相关参数 * / */
/* static void menu_spreadpara(void); / * 分布测量参数 * / */
/* static void menu_correctionpara(void); / * 密度/温度修正参数 * / */
/* static void menu_realhighpara(void); / * 实高测量参数 * / */
/* static void menu_liquidlevelparams(void);/ * 液位测量参数 * / */
/* static void menu_waterlevelparams(void);/ * 水位测量参数 * / */
/* static void menu_aoparams(void); / * 4-20mA/AO 输出参数 * / */
/* static void menu_wartsilapara(void); / * 瓦锡兰参数组 * / */
/* static void menu_screen(void); / * 屏幕/显示相关菜单 * / */
/* static void menu_scr_source(void); / * 数据源菜单 * / */
/* static void menu_cpu3_comm(void); / * CPU3 串口通信配置菜单 * / */
/* static void menu_magnetic(void); / * 磁通量/修正相关菜单(旧菜单或兼容入口) * / */
static int RelayParam_ChannelOf(int operaNum);
static int RelayParam_FieldOf(int operaNum);
static int RelayParam_IsConfig(int operaNum);
static int RelayParam_IsChannelSetting(int operaNum);
static int RelayParam_IsAlarmCondition(int operaNum);
static int RelayParam_IsAlarmValueField(int operaNum);
static pFunc_void RelayParam_BackToConfigMenu(int operaNum);
static MenuGroup ParamGroupOf(int operaNum); /* 根据操作码获取参数分组枚举 */
static void menu_measure_config(void);
static void menu_run_policy(void);
static void menu_dev_info(void);
static void menu_mech(void);
static void menu_weight(void);
static void menu_zero(void);

static void menu_liquid(void);
static void menu_water(void) ;
static void menu_bottom_tankh(void);

static void menu_correct(void);
static void menu_policy(void)  ;
static void menu_wartsila(void)  ;
static void menu_si_config(void);
static void menu_si_profile(void);
static void menu_si_auto_profile(void);
static void menu_si_alarm(void);

static void menu_output_config(void);
static void menu_do_alarm(void)  ;
static void menu_relay1_main(void);
static void menu_relay1_channel(void);
static void menu_relay1_alarm(void);
static void menu_relay1_status(void);
static void menu_relay2_main(void);
static void menu_relay2_channel(void);
static void menu_relay2_alarm(void);
static void menu_relay2_status(void);
static void menu_relay3_main(void);
static void menu_relay3_channel(void);
static void menu_relay3_alarm(void);
static void menu_relay3_status(void);
static void menu_relay4_main(void);
static void menu_relay4_channel(void);
static void menu_relay4_alarm(void);
static void menu_relay4_status(void);
static void menu_ao(void);
static void menu_ao_channel(void);
static void menu_ao_range(void);
static void menu_ao_fault(void);
static void menu_ao_runtime(void);
static void menu_ao_diagnostic(void);
static void ao_simulation_switch_enter(void);
static void ao_simulation_switch_page(void);
static void ao_simulation_switch_back(void);
static void menu_cal_sp(void)     ;

static void menu_param_check(void) ;

/* CPU3：同理，分组页 = 参数列表页 */
static void menu_comm_config(void);
static void menu_display_config(void);
static void menu_display_base(void);
static void menu_display_data(void);
static void menu_display_data_oil(void);
static void menu_display_data_water(void);
static void menu_display_data_density(void);
static void menu_display_data_temp(void);
static void menu_maint_config(void);
static void menu_rtc_datetime(void);
static void menu_cpu2_comm_health(void);
static const char *cpu2_comm_failure_reason_text(Cpu2CommFailureReason reason, bool chinese);
static uint32_t cpu2_comm_display_count(uint64_t count);
static void menu_cpu3_base(void)   ;
static void menu_cpu3_source(void);
static void menu_cpu3_input(void)  ;
static void menu_cpu3_screen(void)  ;
static void menu_cpu3_comm1(void)   ;
static void menu_cpu3_comm2(void)   ;
static void menu_cpu3_comm3(void)  ;

static void menu_paracfg_main(void);
static void motor_run_monitor_page(void); /* 电机运行监控页 */
static void enter_motor_run_monitor_page(void); /* 进入电机运行监控页 */
static void enter_motor_run_monitor_page_waiting_stop(void); /* 停止后回到监控页等待收敛 */
static void motor_run_monitor_back_to_status(void); /* 监控页返回状态页 */
static void motor_run_monitor_request_stop(void); /* 监控页确认键直接停止运动 */
static void display_cpu2_comm_failure(void); /* CPU2 请求失败统一提示 */
static void display_cpu2_modbus_exception(uint8_t exception_code); /* CPU2标准异常提示 */
static void motor_run_monitor_draw_values(void); /* 绘制监控页位置和扭力 */
static bool command_is_motor_monitor_command(uint32_t cmd); /* 纯电机指令范围判断 */
static bool motor_run_monitor_state_is_active(DeviceState state); /* 电机监控运行态判断 */
static bool motor_run_monitor_state_is_done(DeviceState state); /* 电机监控完成态判断 */
static void motor_run_monitor_handle_sent_command(uint32_t cmd); /* 指令下发后页面跳转 */
static void debug_weight_wait_page(void); /* 扭力获取等待页 */
static void enter_debug_weight_wait_page(int operaNum); /* 进入扭力获取等待页 */
static void debug_weight_wait_back_to_menu(void); /* 扭力等待页返回扭力标定菜单 */
static bool debug_weight_wait_state_is_active(int operaNum, DeviceState state); /* 扭力获取中状态判断 */
static bool debug_weight_wait_state_is_done(int operaNum, DeviceState state); /* 扭力获取完成状态判断 */
static bool debug_weight_temperature_x100(int32_t *temperature_x100); /* 解码扭力模块温度原始位 */
static void debug_weight_draw_temperature(void); /* 绘制扭力模块温度或无效占位 */

/* ---------- 3) 通用菜单渲染/选择器 ----------
 *	分页、上下移动、确认/返回等统一菜单交互
 */
static void menuselect(struct MenuData *menu, int menulen);		/* 通用菜单分页选择 */
static void operationselect(uint8_t *(*menu)[2], int menulen, int selected_index);	/* 枚举含义选择列表 */
static void selectparaword(void);								/* 进入“枚举含义选择”页 */

/* ---------- 4) 参数读写流程(读参数 -> 显示 -> 修改 -> 写回) ----------
 *	参数类操作的完整闭环：读、显示、写权限检查、范围检查、写回
 */
static int get_para_data(void);			/* 读取当前参数对应寄存器 */
static void para_mainprocess(void);		/* 参数流程入口：读 -> displaypara */
static void displaypara(void);			/* 显示参数值/含义 */
static void parawritecheck(void);		/* 写权限检查：是否允许修改 */
static void parascopecheck(void);		/* 范围检查：最小/最大等 */
static void cmd_configpara_process(void); /* 组包写参数 -> 回读 -> 刷新显示 */
static bool ao_param_is_config(int operaNum); /* AO持久化配置项判断 */
static bool ao_param_is_editable(int operaNum); /* AO可见持久化参数写权限 */
static int32_t ao_range_max_01mm(void); /* 当前输出源的量程输入上限 */
static bool ao_write_range_pair(void); /* 0%与100%量程成对写入 */
static bool ao_write_output_source(void); /* 输出源确认写入并补读完整AO配置 */
static bool ao_write_simulation_enable(uint32_t enabled); /* 写非持久化仿真开关 */
static bool screen_operation_is_no_para_command(int operaNum); /* 屏幕无参指令分类，含显式扩展操作码 */

/* ---------- 5) 指令下发流程(无参/带参) ----------
 *	把“确定/返回”的动作映射到具体执行：下发指令或写参数
 */
static void ifsendcmd(void);			/* “是否下发/确认返回”页面 */
static void param_protect_confirm(void); /* 保护参数/恢复出厂的额外确认页 */
static pFunc_void dtm_suretofunc(void);	/* 确认键 -> 下一步函数 */
static pFunc_void dtm_backtofunc(void);	/* 返回键 -> 返回上一级函数 */
static void cmd_nopara_process(void);	/* 无参指令：直接下发 */
static void cmd_onepara_process(void);	/* 带参指令：先写参数再下发 */
static bool operation_needs_protect_confirm(int operaNum); /* 是否需要额外保护确认 */
static void protected_operation_process(void); /* 保护确认通过后的实际执行 */
static void tape_thickness_select(void); /* 尺带厚度型号选择 */
static int tape_thickness_to_selection_index(int value); /* 尺带厚度转型号下标 */

/* ---------- 6) 输入与数值编辑(输入框) ----------
 *	数字逐位输入、符号输入、位数/单位/小数点等显示规则
 */
static void inputcmdpara(void);			/* 输入参数页面(数值/符号) */
static bool inputvalue(uint8_t deci, uint8_t row, uint8_t line,
		uint8_t points, uint8_t *unit, int *value);				/* 多位数字输入状态机 */
static int SignInput(uint8_t row, uint8_t line, uint8_t shift); /* 正负号输入 */
static void ResetSignInputState(void); /* 清除符号页跨次编辑状态 */
static void inputcmdpara_back(void); /* 输入页返回，符号阶段只取消本次编辑 */
static bool ParamAllowsSignedInput(int operaNum); /* 参数是否允许选择正负号 */

/* ---------- 7) 名称/单位/枚举含义工具函数 ----------
 *	根据 operaNum 或 param_meta 表，返回名字、单位、小数点位数、显示位数等
 */
static uint8_t *dtm_operaname(int num);	/* 根据操作号返回名称(中/英) */
static uint8_t oled_text_width(const uint8_t *name); /* 按 OLED 绘制列宽估算显示长度 */
static void display_right_aligned_action(uint8_t *chinese, uint8_t *english, uint8_t row, uint8_t shift); /* 底栏右侧操作按实际字宽右对齐 */
static uint8_t *dtm_operaname_short(int num, uint8_t *fallback); /* 菜单列表短名 */
static uint8_t *menu_display_name(const struct MenuData *item); /* 当前语言下的菜单列表显示名 */
static uint8_t *oled_fit_text(uint8_t *name, uint8_t max_width); /* 裁剪到 OLED 单行宽度 */
static void format_version_u32(uint32_t version, char *buf, size_t buf_size); /* 版本编码格式化 */
static int display_formatted_readonly_value(int operaNum, int32_t value, uint8_t line, uint8_t row, uint8_t shift); /* 只读特殊值显示 */
static uint8_t *param_display_unit(int operaNum, const struct ParameterMetadata *meta); /* 参数显示单位 */
static void display_menu_item_with_value(const struct MenuData *item, uint8_t line, uint8_t row, uint8_t shift); /* 菜单列表带值显示 */
static uint8_t display_split_title(uint8_t *name, uint8_t row1, uint8_t row2); /* 长标题拆成最多两行 */
static void display_param_detail_value(const struct ParameterMetadata *meta, uint8_t row); /* 详情页当前值 */
static void display_param_detail_range(const struct ParameterMetadata *meta, uint8_t row); /* 详情页范围 */
static uint32_t motor_current_clamp_irun(uint32_t irun); /* 电机电流档位归一 */
static uint16_t motor_current_rms_ma(uint32_t irun); /* 电机电流档位换算为 RMS mA */
static void format_motor_current_label(uint32_t irun, char *buf, size_t buf_size); /* 电机电流档位显示 */
static void display_motor_current_detail(uint32_t irun, uint8_t row); /* 电机电流详情页显示 */
static uint8_t dtm_points(void);		/* 小数点位数 */
static uint8_t *dtm_unit(void);		/* 单位字符串 */
static uint8_t dtm_bits(void);			/* 显示/输入位数 */
static uint8_t *(*dtm_disarr(int *pindex, int *plen))[2];		/* 获取枚举含义数组 */
static int selection_index_to_value(int operaNum, int selectedIndex); /* 选择项下标转实际写入值 */
static uint8_t *returnWordType(uint8_t *chinese, uint8_t *english); /* 语言选择 */

/* ---------- 8) 密码/权限入口 ----------
 *	进入参数配置/调试指令前的密码流程
 */
static void password_enter_para(void);	/* 进入参数配置前输入密码 */
static void password_enter_cmd(void);	/* 进入调试指令前输入密码 */
static void ifentermainmenu(void);		/* 是否进入罐上操作 */
static void ifexittankopera(void);		/* 是否退出罐上操作 */
static void ifcancelmeasurement(void);  /* 是否取消当前测量 */
static void cancel_confirm_back(void); /* 取消测量确认页返回处理 */
static void confirm_cancel_measurement(void); /* 确认取消当前测量 */

/* ---------- 9) 语言设置 ----------
 *	语言菜单与设置项
 */
static void setlanguage(void);			/* 语言菜单入口 */
static void setchinese(void);			/* 设为中文 */
static void setenglish(void);			/* 设为英文 */

/* ---------- 10) 异常/兜底 ----------
 *	非法操作等统一错误提示
 */
static void errorprocess(void);			/* 非法操作提示并退出/返回 */


/* ==============================
 * 按键菜单映射表（下标必须与 KEYNUM_* 完全一致）
 * ============================== */
struct KeyMenu keymenu[KEYNUM_END] = {

    /* 0 - 是否进入罐上操作 */
    [KEYNUM_IF_ENTER_MAINMENU] =
        { exitTankOpera, NULL, NULL, mainmenu,
          USE_KEY_BACK | USE_KEY_SURE, ifentermainmenu },

    /* 1 - 主菜单 */
    [KEYNUM_MAINMENU] =
        { ifexittankopera, mainmenu, mainmenu, mainmenu,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, mainmenu },

    /* 2 - 是否退出罐上操作 */
    [KEYNUM_IF_EXIT_MAINMENU] =
        { mainmenu, NULL, NULL, exitTankOpera,
          USE_KEY_BACK | USE_KEY_SURE, ifexittankopera },

    /* 3 - 普通测量指令主菜单 */
    [KEYNUM_MEASURE_MAINMENU] =
        { measuremenu, measuremenu, measuremenu, measuremenu,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, measuremenu },

    [KEYNUM_MEASURE_WATER] =
        { menu_measure_water, menu_measure_water, menu_measure_water, menu_measure_water,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_measure_water },

    [KEYNUM_MEASURE_DENSITY_SINGLE] =
        { menu_measure_density_single, menu_measure_density_single, menu_measure_density_single, menu_measure_density_single,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_measure_density_single },

    [KEYNUM_MEASURE_DENSITY_DISTRIBUTION] =
        { menu_measure_density_distribution, menu_measure_density_distribution, menu_measure_density_distribution, menu_measure_density_distribution,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_measure_density_distribution },

    /* 参数配置主菜单（新） */
    [KEYNUM_MENU_PARACFG_MAIN] =
        { menu_paracfg_main, menu_paracfg_main, menu_paracfg_main, menu_paracfg_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_paracfg_main },

    /* 维护/调试指令主菜单 */
    [KEYNUM_MENU_CMD_MAIN] =
        { menu_cmdconfig_main, menu_cmdconfig_main, menu_cmdconfig_main, menu_cmdconfig_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cmdconfig_main },

    [KEYNUM_DEBUG_FLOAT_MOTION] =
        { menu_debug_float_motion, menu_debug_float_motion, menu_debug_float_motion, menu_debug_float_motion,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_debug_float_motion },

    [KEYNUM_DEBUG_CALIBRATION] =
        { menu_debug_calibration, menu_debug_calibration, menu_debug_calibration, menu_debug_calibration,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_debug_calibration },

    [KEYNUM_DEBUG_WEIGHT] =
        { menu_debug_weight, menu_debug_weight, menu_debug_weight, menu_debug_weight,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_debug_weight },

    [KEYNUM_DEBUG_WIRELESS] =
        { menu_debug_wireless, menu_debug_wireless, menu_debug_wireless, menu_debug_wireless,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_debug_wireless },

    [KEYNUM_DEBUG_SYSTEM] =
        { menu_debug_system, menu_debug_system, menu_debug_system, menu_debug_system,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_debug_system },

    /* 是否下发指令或参数 */
    [KEYNUM_IFSENDCMD] =
        { ifsendcmd, NULL, NULL, ifsendcmd,
          USE_KEY_BACK | USE_KEY_SURE, ifsendcmd },

    /* 保护参数额外确认 */
    [KEYNUM_IF_PARAM_PROTECT_CONFIRM] =
        { param_protect_confirm, NULL, NULL, param_protect_confirm,
          USE_KEY_BACK | USE_KEY_SURE, param_protect_confirm },

    /* 7 - 输入参数值(带参指令中的) */
    [KEYNUM_INPUTCMDPARA] =
        { inputcmdpara_back, inputcmdpara, inputcmdpara, inputcmdpara,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, inputcmdpara },

    /* 8 - 参数显示(读写类参数中的) */
    [KEYNUM_DISPLAY_PARA] =
        { displaypara, NULL, NULL, displaypara,
          USE_KEY_BACK | USE_KEY_SURE, displaypara },

    /* 9 - 隐藏信息选择 */
    [KEYNUM_WORDSELECT] =
        { selectparaword, selectparaword, selectparaword, selectparaword,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, selectparaword },

    [KEYNUM_TAPE_THICKNESS_SELECT] =
        { tape_thickness_select, tape_thickness_select, tape_thickness_select, tape_thickness_select,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, tape_thickness_select },

    /* 10 - 显示设置 - 语言 */
    [KEYNUM_MENU_LANGUAGE] =
        { setlanguage, setlanguage, setlanguage, setlanguage,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, setlanguage },

    /* ===== 参数配置（新分组页面） ===== */

    [KEYNUM_MENU_PARA_MEASURE_CONFIG] =
        { menu_measure_config, menu_measure_config, menu_measure_config, menu_measure_config,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_measure_config },

    [KEYNUM_MENU_PARA_RUN_POLICY] =
        { menu_run_policy, menu_run_policy, menu_run_policy, menu_run_policy,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_run_policy },

    /* 11 - 设备信息参数 */
    [KEYNUM_MENU_PARA_DEV_INFO] =
        { menu_dev_info, menu_dev_info, menu_dev_info, menu_dev_info,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_dev_info },

    /* 12 - 机械参数 */
    [KEYNUM_MENU_PARA_MECH] =
        { menu_mech, menu_mech, menu_mech, menu_mech,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_mech },

    /* 13 - 扭力参数（如果暂无此页，用 menu_dev_info 占位也行） */
    [KEYNUM_MENU_PARA_WEIGHT] =
        { menu_weight, menu_weight, menu_weight, menu_weight,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_weight },

    /* 14 - 零点参数 */
    [KEYNUM_MENU_PARA_ZERO] =
        { menu_zero, menu_zero, menu_zero, menu_zero,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_zero },

    /* 15 - 液位参数 */
    [KEYNUM_MENU_PARA_LIQUID] =
        { menu_liquid, menu_liquid, menu_liquid, menu_liquid,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_liquid },

    /* 16 - 水位参数 */
    [KEYNUM_MENU_PARA_WATER] =
        { menu_water, menu_water, menu_water, menu_water,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_water },

    /* 17 - 罐底/罐高参数 */
    [KEYNUM_MENU_PARA_BOTTOM_TANKH] =
        { menu_bottom_tankh, menu_bottom_tankh, menu_bottom_tankh, menu_bottom_tankh,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_bottom_tankh },

    /* 18 - 修正参数 */
    [KEYNUM_MENU_PARA_CORR] =
        { menu_correct, menu_correct, menu_correct, menu_correct,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_correct },

    /* 19 - 策略/分布/区间参数 */
    [KEYNUM_MENU_PARA_POLICY] =
        { menu_policy, menu_policy, menu_policy, menu_policy,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_policy },

    /* 20 - Wartsila 参数 */
    [KEYNUM_MENU_PARA_WARTSILA] =
        { menu_wartsila, menu_wartsila, menu_wartsila, menu_wartsila,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_wartsila },

    /* 21 - 继电器报警输出参数 */
    [KEYNUM_MENU_PARA_DO] =
        { menu_do_alarm, menu_do_alarm, menu_do_alarm, menu_do_alarm,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_do_alarm },

    /* 22 - AO 参数 */
    [KEYNUM_MENU_PARA_AO] =
        { menu_ao, menu_ao, menu_ao, menu_ao,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_ao },

    [KEYNUM_MENU_AO_CHANNEL] =
        { menu_ao_channel, menu_ao_channel, menu_ao_channel, menu_ao_channel,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_ao_channel },

    [KEYNUM_MENU_AO_RANGE] =
        { menu_ao_range, menu_ao_range, menu_ao_range, menu_ao_range,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_ao_range },

    [KEYNUM_MENU_AO_FAULT] =
        { menu_ao_fault, menu_ao_fault, menu_ao_fault, menu_ao_fault,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_ao_fault },

    [KEYNUM_MENU_AO_RUNTIME] =
        { menu_ao_runtime, NULL, NULL, NULL,
          USE_KEY_BACK, menu_ao_runtime },

    [KEYNUM_MENU_AO_DIAGNOSTIC] =
        { menu_ao_diagnostic, menu_ao_diagnostic, menu_ao_diagnostic, menu_ao_diagnostic,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_ao_diagnostic },

    [KEYNUM_AO_SIMULATION_SWITCH] =
        { ao_simulation_switch_back, ao_simulation_switch_page, ao_simulation_switch_page, ao_simulation_switch_page,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, ao_simulation_switch_page },

    /* 23 - 标定/单点参数 */
    [KEYNUM_MENU_PARA_CAL_SP] =
        { menu_cal_sp, menu_cal_sp, menu_cal_sp, menu_cal_sp,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cal_sp },

    /* 24 - 参数校验信息 */
    [KEYNUM_MENU_PARA_PARAM_CHECK] =
        { menu_param_check, menu_param_check, menu_param_check, menu_param_check,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_param_check },

    [KEYNUM_MENU_OUTPUT_CONFIG] =
        { menu_output_config, menu_output_config, menu_output_config, menu_output_config,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_output_config },

    [KEYNUM_MENU_RELAY1_MAIN] =
        { menu_relay1_main, menu_relay1_main, menu_relay1_main, menu_relay1_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay1_main },

    [KEYNUM_MENU_RELAY1_CHANNEL] =
        { menu_relay1_channel, menu_relay1_channel, menu_relay1_channel, menu_relay1_channel,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay1_channel },

    [KEYNUM_MENU_RELAY1_ALARM] =
        { menu_relay1_alarm, menu_relay1_alarm, menu_relay1_alarm, menu_relay1_alarm,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay1_alarm },

    [KEYNUM_MENU_RELAY1_STATUS] =
        { menu_relay1_status, menu_relay1_status, menu_relay1_status, menu_relay1_status,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay1_status },

    [KEYNUM_MENU_RELAY2_MAIN] =
        { menu_relay2_main, menu_relay2_main, menu_relay2_main, menu_relay2_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay2_main },

    [KEYNUM_MENU_RELAY2_CHANNEL] =
        { menu_relay2_channel, menu_relay2_channel, menu_relay2_channel, menu_relay2_channel,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay2_channel },

    [KEYNUM_MENU_RELAY2_ALARM] =
        { menu_relay2_alarm, menu_relay2_alarm, menu_relay2_alarm, menu_relay2_alarm,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay2_alarm },

    [KEYNUM_MENU_RELAY2_STATUS] =
        { menu_relay2_status, menu_relay2_status, menu_relay2_status, menu_relay2_status,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay2_status },

    [KEYNUM_MENU_RELAY3_MAIN] =
        { menu_relay3_main, menu_relay3_main, menu_relay3_main, menu_relay3_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay3_main },

    [KEYNUM_MENU_RELAY3_CHANNEL] =
        { menu_relay3_channel, menu_relay3_channel, menu_relay3_channel, menu_relay3_channel,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay3_channel },

    [KEYNUM_MENU_RELAY3_ALARM] =
        { menu_relay3_alarm, menu_relay3_alarm, menu_relay3_alarm, menu_relay3_alarm,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay3_alarm },

    [KEYNUM_MENU_RELAY3_STATUS] =
        { menu_relay3_status, menu_relay3_status, menu_relay3_status, menu_relay3_status,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay3_status },

    [KEYNUM_MENU_RELAY4_MAIN] =
        { menu_relay4_main, menu_relay4_main, menu_relay4_main, menu_relay4_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay4_main },

    [KEYNUM_MENU_RELAY4_CHANNEL] =
        { menu_relay4_channel, menu_relay4_channel, menu_relay4_channel, menu_relay4_channel,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay4_channel },

    [KEYNUM_MENU_RELAY4_ALARM] =
        { menu_relay4_alarm, menu_relay4_alarm, menu_relay4_alarm, menu_relay4_alarm,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay4_alarm },

    [KEYNUM_MENU_RELAY4_STATUS] =
        { menu_relay4_status, menu_relay4_status, menu_relay4_status, menu_relay4_status,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_relay4_status },

    [KEYNUM_MENU_COMM_CONFIG] =
        { menu_comm_config, menu_comm_config, menu_comm_config, menu_comm_config,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_comm_config },

    [KEYNUM_MENU_DISPLAY_CONFIG] =
        { menu_display_config, menu_display_config, menu_display_config, menu_display_config,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_config },

    [KEYNUM_MENU_DISPLAY_BASE] =
        { menu_display_base, menu_display_base, menu_display_base, menu_display_base,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_base },

    [KEYNUM_MENU_DISPLAY_DATA] =
        { menu_display_data, menu_display_data, menu_display_data, menu_display_data,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_data },

    [KEYNUM_MENU_DISPLAY_DATA_OIL] =
        { menu_display_data_oil, menu_display_data_oil, menu_display_data_oil, menu_display_data_oil,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_data_oil },

    [KEYNUM_MENU_DISPLAY_DATA_WATER] =
        { menu_display_data_water, menu_display_data_water, menu_display_data_water, menu_display_data_water,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_data_water },

    [KEYNUM_MENU_DISPLAY_DATA_DENSITY] =
        { menu_display_data_density, menu_display_data_density, menu_display_data_density, menu_display_data_density,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_data_density },

    [KEYNUM_MENU_DISPLAY_DATA_TEMP] =
        { menu_display_data_temp, menu_display_data_temp, menu_display_data_temp, menu_display_data_temp,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_display_data_temp },

    [KEYNUM_MENU_MAINT_CONFIG] =
        { menu_maint_config, menu_maint_config, menu_maint_config, menu_maint_config,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_maint_config },

    [KEYNUM_MENU_RTC_DATETIME] =
        { menu_rtc_datetime, menu_rtc_datetime, menu_rtc_datetime, menu_rtc_datetime,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_rtc_datetime },

    [KEYNUM_MENU_CPU2_COMM_HEALTH] =
        { menu_maint_config, menu_cpu2_comm_health, menu_cpu2_comm_health, menu_cpu2_comm_health,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu2_comm_health },

    /* ===== CPU3（拆分页面） ===== */

    [KEYNUM_MENU_CPU3_BASE] =
        { menu_cpu3_base, menu_cpu3_base, menu_cpu3_base, menu_cpu3_base,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_base },

    [KEYNUM_MENU_CPU3_SOURCE] =
        { menu_cpu3_source, menu_cpu3_source, menu_cpu3_source, menu_cpu3_source,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_source },

    [KEYNUM_MENU_CPU3_INPUT] =
        { menu_cpu3_input, menu_cpu3_input, menu_cpu3_input, menu_cpu3_input,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_input },

    [KEYNUM_MENU_CPU3_SCREEN] =
        { menu_cpu3_screen, menu_cpu3_screen, menu_cpu3_screen, menu_cpu3_screen,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_screen },

    [KEYNUM_MENU_CPU3_COM1] =
        { menu_cpu3_comm1, menu_cpu3_comm1, menu_cpu3_comm1, menu_cpu3_comm1,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_comm1 },

    [KEYNUM_MENU_CPU3_COM2] =
        { menu_cpu3_comm2, menu_cpu3_comm2, menu_cpu3_comm2, menu_cpu3_comm2,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_comm2 },

    [KEYNUM_MENU_CPU3_COM3] =
        { menu_cpu3_comm3, menu_cpu3_comm3, menu_cpu3_comm3, menu_cpu3_comm3,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_comm3 },

    /* 状态显示界面长按返回后的取消测量确认页 */
    [KEYNUM_IF_CANCEL_MEASUREMENT] =
        { cancel_confirm_back, NULL, NULL, confirm_cancel_measurement,
          USE_KEY_BACK | USE_KEY_SURE, ifcancelmeasurement },

    /* 纯电机指令下发后的运行监控页 */
    [KEYNUM_MOTOR_RUN_MONITOR] =
        { motor_run_monitor_back_to_status, NULL, NULL, motor_run_monitor_request_stop,
          USE_KEY_BACK | USE_KEY_SURE, motor_run_monitor_page },

    /* 获取空载/满载扭力后的等待页 */
    [KEYNUM_DEBUG_WEIGHT_WAIT] =
        { debug_weight_wait_back_to_menu, NULL, NULL, debug_weight_wait_back_to_menu,
          USE_KEY_BACK | USE_KEY_SURE, debug_weight_wait_page },

    /* 故障状态长按返回后的故障原因查看页 */
    [KEYNUM_ERROR_REASON] =
        { exitTankOpera, NULL, NULL, exitTankOpera,
          USE_KEY_BACK | USE_KEY_SURE, Display_ShowErrorReasonPage },

    [KEYNUM_MENU_SI_CONFIG] =
        { menu_si_config, menu_si_config, menu_si_config, menu_si_config,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_si_config },

    [KEYNUM_MENU_SI_PROFILE] =
        { menu_si_profile, menu_si_profile, menu_si_profile, menu_si_profile,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_si_profile },

    [KEYNUM_MENU_SI_AUTO_PROFILE] =
        { menu_si_auto_profile, menu_si_auto_profile, menu_si_auto_profile, menu_si_auto_profile,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_si_auto_profile },

    [KEYNUM_MENU_SI_ALARM] =
        { menu_si_alarm, menu_si_alarm, menu_si_alarm, menu_si_alarm,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_si_alarm },
};


/**
 * @brief 判断菜单当前状态是否允许处理新的按键事件。
 *
 * @param keypress 本次待分发的按键位掩码。
 * @return true 表示菜单索引有效、按键已获授权且存在对应回调；否则返回 false。
 */
bool DisplayTankOpera_CanProcessKey(uint8_t keypress)
{
	if ((func_index < 0) || (func_index >= KEYNUM_END)) {
		return false;
	}

	if ((keypress & keymenu[func_index].keyauthority) == 0) {
		return false;
	}

	if ((keypress == USE_KEY_BACK) && (keymenu[func_index].back_opera != NULL)) {
		return true;
	}
	if ((keypress == USE_KEY_UP) && (keymenu[func_index].up_opera != NULL)) {
		return true;
	}
	if ((keypress == USE_KEY_DOWN) && (keymenu[func_index].down_opera != NULL)) {
		return true;
	}
	if ((keypress == USE_KEY_SURE) && (keymenu[func_index].sure_opera != NULL)) {
		return true;
	}

	return false;
}

/**
 * @brief 按键操作处理。
 *
 * @param keypress 本次待分发的按键位掩码。
 * @return true 表示按键已分发给当前页面回调；无权限、无回调或菜单索引非法时返回 false。
 */
bool KeyProcess(uint8_t keypress)
{
	if ((func_index < 0) || (func_index >= KEYNUM_END)) {
		return false;
	}

	if ((keypress & keymenu[func_index].keyauthority) != 0) {
		NowKeyPress = keypress;
		useKey();

		if (keypress == USE_KEY_BACK && keymenu[func_index].back_opera != NULL) {
			keymenu[func_index].back_opera();
			return true;
		} else if (keypress == USE_KEY_UP && keymenu[func_index].up_opera != NULL) {
			keymenu[func_index].up_opera();
			return true;
		} else if (keypress == USE_KEY_DOWN && keymenu[func_index].down_opera != NULL) {
			keymenu[func_index].down_opera();
			return true;
		} else if (keypress == USE_KEY_SURE && keymenu[func_index].sure_opera != NULL) {
			keymenu[func_index].sure_opera();
			return true;
		} else {
			printf("NULL\r\n");
		}
	}
	return false;
}

/**
 * @brief 按当前菜单层级和选中项重绘页面。
 * @return true 表示当前罐上操作页面已由对应层级的绘制入口重绘；false 表示当前菜单层级、选中项或执行入口无效，无法确定可安全重绘的页面。
 */
bool DisplayTankOpera_RedrawCurrentPage(void)
{
	int saved_key = NowKeyPress;

	if ((func_index < 0) || (func_index >= KEYNUM_END) || (keymenu[func_index].execute_opera == NULL)) {
		return false;
	}

	/* 静电恢复后只重绘当前页，临时清空按键，避免重复执行上一次按键动作。 */
	NowKeyPress = 0;
	keymenu[func_index].execute_opera();
	NowKeyPress = saved_key;
	return true;
}

/**
 * @brief 判断当前前景页是否为纯电机指令运行监控页。
 * @return true 表示当前前景页是电机运行监控页。
 */
bool DisplayTankOpera_IsMotorRunMonitorActive(void)
{
	return (FlagofTankOpera == true) && (func_index == KEYNUM_MOTOR_RUN_MONITOR);
}

/**
 * @brief 判断当前前景页是否为扭力获取等待页。
 * @return true 表示当前前景页是扭力获取等待页。
 */
bool DisplayTankOpera_IsDebugWeightWaitActive(void)
{
	return (FlagofTankOpera == true) && (func_index == KEYNUM_DEBUG_WEIGHT_WAIT);
}

/**
 * @brief 判断当前是否显示 AO 运行状态页。
 * @return true 表示周期刷新应重绘 AO 输入值、输入百分比与输出电流。
 */
bool DisplayTankOpera_IsAoRuntimeActive(void)
{
	return (FlagofTankOpera == true) && (func_index == KEYNUM_MENU_AO_RUNTIME);
}

/**
 * @brief 判断 CPU2 板间通信健康页是否仍处于罐上操作前景。
 *
 * @return true 表示 CPU2 板间通信健康页仍处于罐上操作前景；false 表示 CPU2 板间通信健康页已不再处于罐上操作前景。
 */
bool DisplayTankOpera_IsCpu2CommHealthActive(void)
{
	return (FlagofTankOpera == true) && (func_index == KEYNUM_MENU_CPU2_COMM_HEALTH);
}

/**
 * @brief 判断当前罐上操作页是否允许菜单空闲超时自动退出。
 * @return true 表示可退回状态页，false 表示当前页有业务等待逻辑，不应被空闲超时打断。
 */
bool DisplayTankOpera_CanIdleExit(void)
{
	if (FlagofTankOpera != true) {
		return false;
	}

	if ((func_index < 0) || (func_index >= KEYNUM_END)) {
		return true;
	}

	if ((func_index == KEYNUM_MOTOR_RUN_MONITOR) ||
	    (func_index == KEYNUM_DEBUG_WEIGHT_WAIT)) {
		return false;
	}

	return true;
}

/**
 * @brief 进入电机运行监控页。
 */
static void enter_motor_run_monitor_page(void)
{
	FlagofTankOpera = true;
	func_index = KEYNUM_MOTOR_RUN_MONITOR;
	motor_run_monitor_enter_tick = HAL_GetTick();
	motor_run_monitor_started = false;
	timesure = 0;
	timeback = 0;
	ClearPageNum();
	motor_run_monitor_page();
}

/**
 * @brief 停止命令下发后回到监控页，保持已启动状态以便 CPU2 待机后自动退出。
 */
static void enter_motor_run_monitor_page_waiting_stop(void)
{
	FlagofTankOpera = true;
	func_index = KEYNUM_MOTOR_RUN_MONITOR;
	motor_run_monitor_enter_tick = HAL_GetTick();
	motor_run_monitor_started = true;
	timesure = 0;
	timeback = 0;
	ClearPageNum();
	motor_run_monitor_page();
}

/**
 * @brief 从电机运行监控页返回普通状态页，不取消当前电机运动。
 */
static void motor_run_monitor_back_to_status(void)
{
	FlagofTankOpera = false;
	HAL_TIM_Base_Stop_IT(&htim1);
	oled_clear();
	Display_RequestRefresh();
}

/**
 * @brief 绘制电机监控页的实时位置和扭力，两行使用相同的数值起始列。
 */
static void motor_run_monitor_draw_values(void)
{
	uint8_t value_line = (screen_parameter.language == LANGUAGE_ENGLISH)
	                     ? OLED_LINE8_5
	                     : MOTOR_RUN_MONITOR_VALUE_LINE;

	DisplayLangaugeLineWords((uint8_t*)"位置", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Pos");
	OledValueDisplay((int)g_measurement.debug_data.sensor_position,
	                 value_line,
	                 OLED_ROW4_2,
	                 0,
	                 1,
	                 (uint8_t*)"mm");

	DisplayLangaugeLineWords((uint8_t*)"扭力", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Torque");
	OledValueDisplay((int)g_measurement.debug_data.current_weight,
	                 value_line,
	                 OLED_ROW4_3,
	                 0,
	                 0,
	                 (uint8_t*)" ");
}

/**
 * @brief 显示电机运行监控页。
 */
static void motor_run_monitor_page(void)
{
	DeviceState state = g_measurement.device_status.device_state;
	uint32_t now_tick = HAL_GetTick();
	uint8_t lang = (uint8_t)screen_parameter.language;
	const char *state_text;

	if (lang > LANGUAGE_ENGLISH) {
		lang = LANGUAGE_ENGLISH;
	}

	if (motor_run_monitor_state_is_active(state)) {
		motor_run_monitor_started = true;
	} else if ((motor_run_monitor_started == true) && motor_run_monitor_state_is_done(state)) {
		motor_run_monitor_back_to_status();
		return;
	} else if ((motor_run_monitor_started == false) &&
	           ((now_tick - motor_run_monitor_enter_tick) > MOTOR_RUN_MONITOR_START_GRACE_MS)) {
		motor_run_monitor_back_to_status();
		return;
	}

	oled_clear();
	func_index = KEYNUM_MOTOR_RUN_MONITOR;

	state_text = GetStateString(state, lang);
	OledDisplayLineWords((uint8_t*)state_text, OLED_LINE8_1, OLED_ROW4_1, 0);
	motor_run_monitor_draw_values();

	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"停止运动", (uint8_t*)"Stop", OLED_ROW4_4, 0);
}

/**
 * @brief 从电机监控页直接下发停止当前运动命令。
 */
static void motor_run_monitor_request_stop(void)
{
	if (!DisplayTankOpera_IsMotorRunMonitorActive()) {
		return;
	}

	if (!Display_RequestCancelMeasurement()) {
		display_cpu2_comm_failure();
		motor_run_monitor_page();
		return;
	}
	enter_motor_run_monitor_page_waiting_stop();
}

/**
 * @brief 进入扭力获取等待页。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 */
static void enter_debug_weight_wait_page(int operaNum)
{
	FlagofTankOpera = true;
	func_index = KEYNUM_DEBUG_WEIGHT_WAIT;
	debug_weight_wait_opera = operaNum;
	debug_weight_wait_ignore_initial_done =
	    debug_weight_wait_state_is_done(operaNum, g_measurement.device_status.device_state);
	debug_weight_wait_started = !debug_weight_wait_ignore_initial_done;
	timesure = 0;
	timeback = 0;
	ClearPageNum();
	debug_weight_wait_page();
}

/**
 * @brief 扭力等待页返回扭力标定菜单，不取消 CPU2 正在执行的扭力获取。
 */
static void debug_weight_wait_back_to_menu(void)
{
	debug_weight_wait_opera = COM_NUM_NOOPERA;
	debug_weight_wait_started = false;
	debug_weight_wait_ignore_initial_done = false;
	NowKeyPress = 0;
	timesure = 0;
	timeback = 0;
	ClearPageNum();
	menu_debug_weight();
}

/**
 * @brief 判断 CPU2 当前状态是否属于空载/满载扭力获取中。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @param state CPU2 当前设备状态；结合 operaNum 区分空载或满载扭力获取是否仍在执行。
 * @return true 表示 CPU2 当前状态属于空载/满载扭力获取中；false 表示 CPU2 当前状态不属于空载/满载扭力获取中。
 */
static bool debug_weight_wait_state_is_active(int operaNum, DeviceState state)
{
	if (operaNum == COM_NUM_SET_EMPTY_WEIGHT) {
		return state == STATE_GET_EMPTYWEIGHT;
	}

	if (operaNum == COM_NUM_SET_FULL_WEIGHT) {
		return state == STATE_GET_FULLWEIGHT;
	}

	return false;
}

/**
 * @brief 判断 CPU2 当前状态是否属于空载/满载扭力获取完成。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @param state CPU2 当前设备状态；结合 operaNum 区分空载或满载扭力获取是否已经完成。
 * @return true 表示 CPU2 当前状态属于空载/满载扭力获取完成；false 表示 CPU2 当前状态不属于空载/满载扭力获取完成。
 */
static bool debug_weight_wait_state_is_done(int operaNum, DeviceState state)
{
	if (operaNum == COM_NUM_SET_EMPTY_WEIGHT) {
		return state == STATE_GET_EMPTYWEIGHT_OVER;
	}

	if (operaNum == COM_NUM_SET_FULL_WEIGHT) {
		return state == STATE_GET_FULLWEIGHT_OVER;
	}

	return false;
}

/**
 * @brief 把协议31复用槽中的IEEE754扭力模块温度转换为0.01摄氏度整数。
 *
 * @details 调用场景：扭力标定等待页刷新实时温度时调用。
 * @note 关键约束：拒绝NaN、无穷和超出屏幕表达范围的数据，避免显示无效或溢出值。
 *
 * @param temperature_x100 温度定点值，单位 0.01 ℃。
 * @return true 表示原始 IEEE754 温度为有限值且位于 -999.99～999.99 ℃，并已四舍五入写入 temperature_x100；false 表示输出指针为空、原始位为 NaN/无穷，或温度超出屏幕表达范围。
 */
static bool debug_weight_temperature_x100(int32_t *temperature_x100)
{
	uint32_t temperature_bits = g_measurement.debug_data.torque_temperature_bits;
	float temperature;
	float scaled;

	if ((temperature_x100 == NULL) ||
	    ((temperature_bits & 0x7F800000UL) == 0x7F800000UL)) {
		return false;
	}

	memcpy(&temperature, &temperature_bits, sizeof(temperature));
	if ((temperature < -999.99f) || (temperature > 999.99f)) {
		return false;
	}

	scaled = temperature * 100.0f;
	*temperature_x100 = (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
	return true;
}

/**
 * @brief 在扭力标定等待页第二行显示扭力模块温度。
 *
 * @details 调用场景：等待空载或满载扭力获取期间随页面周期刷新。
 * @note 关键约束：温度无效时显示占位，不把旧占位槽的零值解释为有效温度。
 */
static void debug_weight_draw_temperature(void)
{
	int32_t temperature_x100;

	DisplayLangaugeLineWords((uint8_t*)"温度", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Temp");
	if (debug_weight_temperature_x100(&temperature_x100)) {
		OledValueDisplay((int)temperature_x100,
		                 OLED_LINE8_5,
		                 OLED_ROW4_2,
		                 0,
		                 2,
		                 (uint8_t*)"C");
	} else {
		OledDisplayLineWords((uint8_t*)"--.--", OLED_LINE8_5, OLED_ROW4_2, 0);
	}
}

/**
 * @brief 显示扭力获取等待页，完成后直接回到调试指令的扭力标定菜单。
 */
static void debug_weight_wait_page(void)
{
	DeviceState state = g_measurement.device_status.device_state;
	int operaNum = debug_weight_wait_opera;
	uint8_t *title_cn;
	uint8_t *title_en;

	if ((operaNum != COM_NUM_SET_EMPTY_WEIGHT) && (operaNum != COM_NUM_SET_FULL_WEIGHT)) {
		operaNum = now_Opera_Num;
	}

	if (!debug_weight_wait_state_is_done(operaNum, state)) {
		debug_weight_wait_started = true;
		debug_weight_wait_ignore_initial_done = false;
	}

	if (debug_weight_wait_state_is_active(operaNum, state)) {
		debug_weight_wait_started = true;
		debug_weight_wait_ignore_initial_done = false;
	}

	if (debug_weight_wait_state_is_done(operaNum, state) &&
	    (debug_weight_wait_ignore_initial_done == false) &&
	    (debug_weight_wait_started == true)) {
		debug_weight_wait_back_to_menu();
		return;
	}

	if (state == STATE_ERROR) {
		FlagofTankOpera = false;
		HAL_TIM_Base_Stop_IT(&htim1);
		debug_weight_wait_opera = COM_NUM_NOOPERA;
		debug_weight_wait_started = false;
		debug_weight_wait_ignore_initial_done = false;
		oled_clear();
		Display_RequestRefresh();
		return;
	}

	oled_clear();
	func_index = KEYNUM_DEBUG_WEIGHT_WAIT;

	if (operaNum == COM_NUM_SET_FULL_WEIGHT) {
		title_cn = (uint8_t*)"获取满载扭力中";
		title_en = (uint8_t*)"Getting Full";
	} else {
		title_cn = (uint8_t*)"获取空载扭力中";
		title_en = (uint8_t*)"Getting Empty";
	}

	DisplayLangaugeLineWords(title_cn, OLED_LINE8_1, OLED_ROW4_1, 0, title_en);
	debug_weight_draw_temperature();
	DisplayLangaugeLineWords((uint8_t*)"扭力", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Torque");
	OledValueDisplay((int)g_measurement.debug_data.current_weight,
	                 OLED_LINE8_5,
	                 OLED_ROW4_3,
	                 0,
	                 0,
	                 (uint8_t*)" ");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
}

/**
 * @brief 记录一次有效按键活动并重启菜单空闲计时器。
 */
void useKey(void)
{
	static int keytimeout = 300;		/* 按键超时时间 */
	(void)keytimeout;
	/* Timer1Start(keytimeout); */
}

/* ==============================
 * 输入参数页面
 * ============================== */
/**
 * @brief 判断当前参数输入是否允许选择正负号。
 *
 * @details 调用场景：数字输入完成后决定是否进入符号选择页。
 * @note 关键约束：继电器仅 HH、H、L、LL 阈值允许负号，报警滞回保持非负。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return true 表示当前参数输入允许选择正负号；false 表示当前参数输入不允许选择正负号。
 */
static bool ParamAllowsSignedInput(int operaNum)
{
	int relay_field = RelayParam_FieldOf(operaNum);

	if ((relay_field >= 6) && (relay_field <= 9)) {
		return true;
	}

	switch (operaNum) {
	case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT:
	case COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF:
	case COM_NUM_DEVICEPARAM_DENSITYCORRECTION:
	case COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION:
	case COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000:
	case COM_NUM_CPU3_SI_LOW_TEMPERATURE_SETPOINT:
	case COM_NUM_CPU3_SI_HIGH_TEMPERATURE_SETPOINT:
	case COM_NUM_SCREEN_INPUT_T:
		return true;
	default:
		return false;
	}
}

/* 有符号参数编辑是否正在等待符号确认的状态。 */
static bool s_param_sign_input_active = false;
/* 当前有符号参数编辑采用的符号，取 1 或 -1。 */
static int s_param_sign_value = 1;
/* 符号确认页面的选择索引；-1 表示尚未选择。 */
static int s_param_sign_confirm_count = -1;

/**
 * @brief 清除带符号数值输入的编辑状态。
 */
static void ResetSignInputState(void)
{
	s_param_sign_input_active = false;
	s_param_sign_value = 1;
	s_param_sign_confirm_count = -1;
}

/**
 * @brief 符号选择阶段按返回只取消本次编辑，恢复参数详情页且不下发。
 *
 * 数字输入阶段仍沿用原 inputvalue() 的逐位返回行为。
 */
static void inputcmdpara_back(void)
{
	if (s_param_sign_input_active) {
		ResetSignInputState();
		NowKeyPress = 0;
		timeback = 0;
		timesure = 1;
		displaypara();
		return;
	}
	inputcmdpara();
}

/**
 * @brief 显示当前参数输入页；数字输入完成后，对允许负值的参数追加符号选择，再统一下发。
 */
static void inputcmdpara(void)
{
	uint8_t line, row = OLED_ROW4_1;
	int value;
	uint8_t *name = NULL;

	if (func_index != KEYNUM_INPUTCMDPARA) {
		/* 从其它页面重新进入输入流程时，不继承上一次未完成的符号选择。 */
		ResetSignInputState();
	}
	oled_clear();
	func_index = KEYNUM_INPUTCMDPARA;

	DisplayLangaugeLineWords((uint8_t*)"请输入", OLED_LINE8_1, OLED_ROW4_1, 0, (u8*)"Please enter ");
	name = dtm_operaname(now_Opera_Num);
	row = display_split_title(name, OLED_ROW4_2, OLED_ROW4_3);

	now_Para_CT.points = dtm_points();
	now_Para_CT.unit = dtm_unit();
	now_Para_CT.bits = dtm_bits();

	/* 位数少且无单位时, 显示位置偏右 */
	if (now_Para_CT.bits <= 3 && now_Para_CT.unit == NULL) {
		line = OLED_LINE8_5;
	} else {
		line = OLED_LINE8_4;
	}

	/* 先输入数值, 特定参数再输入符号 */
	if (s_param_sign_input_active == false) {
		if (inputvalue(now_Para_CT.bits, row, line, now_Para_CT.points, now_Para_CT.unit, &value)) {
			now_Para_CT.val = value;

			if (ParamAllowsSignedInput(now_Opera_Num)) {
				ResetSignInputState();
				s_param_sign_input_active = true;
				SignInput(row, line - 4, 1);
				return;
			}

			ifsendcmd();
			return;
		}
	} else {
		int sgn;

		sgn = SignInput(row, line - 4, 1);
		if (sgn != 0) {
			now_Para_CT.val *= sgn;
			ResetSignInputState();
			ifsendcmd();
			return;
		}
		return;
	}
}

/**
 * @brief 确定小数点位数。
 *
 * @return 返回当前参数输入和显示使用的小数位数。
 */
static uint8_t dtm_points(void)
{
	uint8_t p = 0;

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		p = param_meta[index].point;
	} else {
		switch (now_Opera_Num) {
		default:
			break;
		}
	}

	return p;
}
/* 菜单操作码与中英文完整名称的只读映射项；用于参数页标题和操作名称查表。 */
typedef struct {
    /* 菜单操作码与中英文完整名称的一一映射。 */
    int opera; /* 菜单操作码，用于把当前表项与菜单元数据及命令映射关联。 */
    uint8_t *name_cn; /* 该表项对应的中文显示名称。 */
    uint8_t *name_en; /* 该表项对应的英文显示名称。 */
} OperaNameMap_t;

/**
 * @brief 返回当前操作编号对应的中英文菜单名称。
 *
 * @param num 待查询的菜单操作号或序号。
 * @return 返回当前操作编号对应的中英文菜单名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static uint8_t *dtm_operaname(int num)
{
    switch (num) {
    case COM_NUM_AO_SIMULATION_ENABLE:
        return returnWordType((uint8_t*)"输出模拟", (uint8_t*)"Simulation");
    case COM_NUM_AO_RUNTIME_PROCESS_VALUE:
        return returnWordType((uint8_t*)"输入值", (uint8_t*)"Input Value");
    case COM_NUM_AO_RUNTIME_PERCENT:
        return returnWordType((uint8_t*)"输入比例", (uint8_t*)"Input Percent");
    case COM_NUM_AO_RUNTIME_OUTPUT_CURRENT:
        return returnWordType((uint8_t*)"输出电流", (uint8_t*)"Output Current");
    default:
        break;
    }

    /* 1) 普通无参测量指令（显式映射，避免依赖枚举连续性） */
    static const OperaNameMap_t normal_cmd_map[] = {
        { COM_NUM_BACK_ZERO,           (uint8_t*)"提零点",         (uint8_t*)"Return to Zero" },
        { COM_NUM_FIND_OIL,            (uint8_t*)"液位测量",       (uint8_t*)"Find Oil Level" },
        { COM_NUM_FIND_WATER,          (uint8_t*)"水位单次测量",   (uint8_t*)"Find Water Level" },
        { COM_NUM_FIND_BOTTOM,         (uint8_t*)"罐高测量",       (uint8_t*)"Find Tank Bottom" },
        { COM_NUM_SYNTHETIC,           (uint8_t*)"综合测量",       (uint8_t*)"Comprehensive-M" },

        { COM_NUM_FOLLOW_WATER,        (uint8_t*)"水位跟随",       (uint8_t*)"Water Follow" },
        { COM_NUM_SPREADPOINTS,        (uint8_t*)"分布测量",       (uint8_t*)"Spread-M" },
        { COM_NUM_SPREADPOINTS_GB,     (uint8_t*)"国标分布测量",   (uint8_t*)"GB Spread-M" },

        { COM_NUM_METER_DENSITY,       (uint8_t*)"密度每米测量",   (uint8_t*)"DT-PerMeter-M" },
        { COM_NUM_INTERVAL_DENSITY,    (uint8_t*)"区间密度测量",   (uint8_t*)"Interval-M" },
        { COM_NUM_WARTSILA_DENSITY,    (uint8_t*)"瓦锡兰区间密度", (uint8_t*)"Wartsila Interval-M" },
        { COM_NUM_SI_PROFILE,          (uint8_t*)"SI Profile",     (uint8_t*)"SI Profile" },

        { COM_NUM_READ_PART_PARAMS,    (uint8_t*)"读取部件参数",   (uint8_t*)"Read Component Params" },
    };

    /* 2) 无参调试指令（显式映射） */
    static const OperaNameMap_t debug_cmd_map[] = {
        { COM_NUM_FIND_ZERO,           (uint8_t*)"标定零点",       (uint8_t*)"Zero Calibration" },

        { COM_NUM_SET_EMPTY_WEIGHT,    (uint8_t*)"获取空载扭力",   (uint8_t*)"Set Empty Torque" },
        { COM_NUM_SET_FULL_WEIGHT,     (uint8_t*)"获取满载扭力",   (uint8_t*)"Set Full Torque" },
        { COM_NUM_RESTOR_EFACTORYSETTING,(uint8_t*)"恢复出厂设置", (uint8_t*)"Factory Reset" },
        { COM_NUM_MAINTENANCE_MODE,    (uint8_t*)"进入维护模式",   (uint8_t*)"Maintenance Mode" },
        { COM_NUM_MAINTENANCE_EXIT,    (uint8_t*)"退出维护模式",   (uint8_t*)"Exit Maintenance" },
        { COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS, (uint8_t*)"清全部锁存", (uint8_t*)"Clear All Latched" },
        { COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING, (uint8_t*)"匹配无线滑环", (uint8_t*)"Pair Wireless" },
    };

    static const OperaNameMap_t one_para_cmd_map[] = {
        { COM_NUM_SINGLE_POINT,         (uint8_t*)"密度单点测量",   (uint8_t*)"SingleMeasure" },
        { COM_NUM_SP_TEST,              (uint8_t*)"密度单点监测",   (uint8_t*)"SingleMonitor" },
        { COM_NUM_RUN_TO_POSITION,      (uint8_t*)"浮子运行到高度", (uint8_t*)"RunToPos" },
    };

    static const OperaNameMap_t debug_one_para_cmd_map[] = {
        { COM_NUM_CAL_OIL,              (uint8_t*)"标定液位",       (uint8_t*)"CalOil" },
        { COM_NUM_CORRECTION_OIL,       (uint8_t*)"修正液位",       (uint8_t*)"CorrectOil" },
        { COM_NUM_CALIBRATE_WATER,      (uint8_t*)"标定水位",       (uint8_t*)"CalWater" },
        { COM_NUM_CALIBRATE_TANKHEIGHT, (uint8_t*)"标定罐高",       (uint8_t*)"CalTankH" },
        { COM_NUM_RUNUP,                (uint8_t*)"上行",           (uint8_t*)"MoveUp" },
        { COM_NUM_RUNDOWN,              (uint8_t*)"下行",           (uint8_t*)"MoveDown" },
        { COM_NUM_FORCE_RUNUP,          (uint8_t*)"强制上行",       (uint8_t*)"ForceMoveUp" },
        { COM_NUM_FORCE_RUNDOWN,        (uint8_t*)"强制下行",       (uint8_t*)"ForceMoveDown" },
    };

    /* 3) CPU3 本机“固定项”名称（如果你仍然需要这种非 param_meta 的本机项） */
    static const OperaNameMap_t local_fixed_map[] = {
        /* 你原来写了“设备地址/屏幕程序版本”，但新枚举里未看到“设备地址”对应项
           如果你有对应的 COM_NUM_xxx，就填进去；没有就先注释掉。
         */
        /* { COM_NUM_PARA_LOCAL_DEVICEADDR, (uint8_t*)"设备地址", (uint8_t*)"Device Address" }, */
        { COM_NUM_PARA_LOCAL_LEDVERSION,  (uint8_t*)"CPU3程序版本", (uint8_t*)"CPU3 FW Ver" },
        { COM_NUM_PARA_LANG,              (uint8_t*)"语言",         (uint8_t*)"Language" },
    };

    /* ---------- A) 普通无参测量指令 ---------- */
    if (num > COM_NUM_NOPARACMD_NORMAL_START && num < COM_NUM_NOPARACMD_NORMAL_STOP) {
        for (int i = 0; i < (int)(sizeof(normal_cmd_map)/sizeof(normal_cmd_map[0])); i++) {
            if (num == normal_cmd_map[i].opera) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? normal_cmd_map[i].name_cn
                        : normal_cmd_map[i].name_en;
            }
        }
        return returnWordType((uint8_t*)"未知指令", (uint8_t*)"Unknown Command");
    }

    /* ---------- B) 无参调试指令 ---------- */
    if (screen_operation_is_no_para_command(num)) {
        for (int i = 0; i < (int)(sizeof(debug_cmd_map)/sizeof(debug_cmd_map[0])); i++) {
            if (num == debug_cmd_map[i].opera) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? debug_cmd_map[i].name_cn
                        : debug_cmd_map[i].name_en;
            }
        }
        return returnWordType((uint8_t*)"未知调试指令", (uint8_t*)"Unknown Debug Cmd");
    }

    if (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_ONEPARACMD_END) {
        for (int i = 0; i < (int)(sizeof(one_para_cmd_map)/sizeof(one_para_cmd_map[0])); i++) {
            if (num == one_para_cmd_map[i].opera) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? one_para_cmd_map[i].name_cn
                        : one_para_cmd_map[i].name_en;
            }
        }
        return returnWordType((uint8_t*)"未知带参指令", (uint8_t*)"Unknown Param Cmd");
    }

    if (num > COM_NUM_ONEPARA_DEBUGCMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END) {
        for (int i = 0; i < (int)(sizeof(debug_one_para_cmd_map)/sizeof(debug_one_para_cmd_map[0])); i++) {
            if (num == debug_one_para_cmd_map[i].opera) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? debug_one_para_cmd_map[i].name_cn
                        : debug_one_para_cmd_map[i].name_en;
            }
        }
        return returnWordType((uint8_t*)"未知调试指令", (uint8_t*)"Unknown Debug Cmd");
    }

    /* ---------- C) 参数类：统一走 param_meta ---------- */
    /* 注意：你原逻辑里的区间判断有两个隐患：
       1) (num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_LOCAL_STOP) 已经覆盖了 CPU2+CPU3 参数
       2) (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END) 这个 stop 名字本身不一致，建议你修成 ONEPARA_DEBUGCMD_END/STOP
       这里我保留你的区间语义，但把条件拆清晰一点。
     */
    if (num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_LOCAL_STOP)
    {
        int index = getHoldValueNum(num);
        if (index >= 0) {
            if (screen_parameter.language == LANGUAGE_CHINESE) {
                return param_meta[index].name;
            } else {
                return param_meta[index].name_English;
            }
        }
        return returnWordType((uint8_t*)"参数未定义", (uint8_t*)"Param Undefined");
    }

    /* ---------- D) 密码类 ---------- */
    if (num > COM_NUM_PASSWORD_START && num < COM_NUM_PASSWORD_END) {
        return returnWordType((uint8_t*)"密码", (uint8_t*)"Password");
    }

    /* ---------- E) CPU3 本机参数固定项（兜底） ---------- */
    if (num > COM_NUM_PARA_LOCAL_START && num < COM_NUM_PARA_LOCAL_STOP) {
        for (int i = 0; i < (int)(sizeof(local_fixed_map)/sizeof(local_fixed_map[0])); i++) {
            if (num == local_fixed_map[i].opera) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? local_fixed_map[i].name_cn
                        : local_fixed_map[i].name_en;
            }
        }

        /* 如果你 CPU3 本机参数也已经纳入 param_meta，那么这里也可以再尝试一次 param_meta */
        {
            int index = getHoldValueNum(num);
            if (index >= 0) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? param_meta[index].name
                        : param_meta[index].name_English;
            }
        }

        return returnWordType((uint8_t*)"本机参数未知", (uint8_t*)"Unknown Local Param");
    }

    return returnWordType((uint8_t*)"非法操作", (uint8_t*)"Invalid Operation");
}

/* / * 返回操作名称 * / */
/* static uint8_t *dtm_operaname(int num) */
/* { */
/* / * 1) 普通无参测量指令 * / */
/* static uint8_t *OperaNameArr_normal_cmd[][2] = { */
/* { (uint8_t*)"回零点", (uint8_t*)"Return to Zero" }, */
/* { (uint8_t*)"标定零点", (uint8_t*)"Zero Calibration" }, */
/* { (uint8_t*)"分布测量", (uint8_t*)"Spread-M" }, */
/* { (uint8_t*)"寻找液位", (uint8_t*)"Find Oil Level" }, */
/* { (uint8_t*)"寻找水位", (uint8_t*)"Find Water Level" }, */
/* { (uint8_t*)"寻找罐底", (uint8_t*)"Find Tank Bottom" }, */
/* { (uint8_t*)"综合测量", (uint8_t*)"Comprehensive-M" }, */
/* { (uint8_t*)"每米测量", (uint8_t*)"DT-PerMeter-M" }, */
/* { (uint8_t*)"区间测量", (uint8_t*)"Interval-M" }, */
/* { (uint8_t*)"瓦锡兰区间密度", (uint8_t*)"Wartsila Interval-M" }, */
/* }; */
/* */
/* / * 2) 无参调试指令 * / */
/* static uint8_t *OperaNameArr_debug_cmd[][2] = { */
/* { (uint8_t*)"设置空载扭力", (uint8_t*)"Set Empty Torque" }, */
/* { (uint8_t*)"设置满载扭力", (uint8_t*)"Set Full Torque" }, */
/* { (uint8_t*)"恢复出厂设置", (uint8_t*)"Factory Reset" }, */
/* { (uint8_t*)"维护模式", (uint8_t*)"Maintenance Mode" }, */
/* }; */
/* */
/* / * 3) 本机参数名称 * / */
/* static uint8_t *OperaNameArr_local[][2] = { */
/* { (uint8_t*)"设备地址", (uint8_t*)"DeviceAddress" }, */
/* { (uint8_t*)"屏幕程序版本", (uint8_t*)"Screen FW Ver" }, */
/* }; */
/* */
/* int idx; */
/* */
/* / * A) 普通不带参指令 * / */
/* if (num > COM_NUM_NOPARACMD_NORMAL_START && num < COM_NUM_NOPARACMD_NORMAL_STOP) { */
/* idx = num - COM_NUM_NOPARACMD_NORMAL_START - 1; */
/* if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_normal_cmd) / sizeof(OperaNameArr_normal_cmd[0]))) { */
/* return OperaNameArr_normal_cmd[idx][screen_parameter.language]; */
/* } */
/* } */
/* / * B) 无参调试指令 * / */
/* else if (num > COM_NUM_DEBUGCMD_START && num < COM_NUM_DEBUGCMD_STOP) { */
/* idx = num - COM_NUM_DEBUGCMD_START - 1; */
/* if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_debug_cmd) / sizeof(OperaNameArr_debug_cmd[0]))) { */
/* return OperaNameArr_debug_cmd[idx][screen_parameter.language]; */
/* } */
/* } */
/* / * C) 参数类 & 带参指令: 使用 param_meta 表 * / */
/* else if ((num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_LOCAL_STOP) */
/* || (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END)) { */
/* int index = getHoldValueNum(num); */
/* if (index >= 0) { */
/* if (screen_parameter.language == LANGUAGE_CHINESE) { */
/* return param_meta[index].name; */
/* } else if (screen_parameter.language == LANGUAGE_ENGLISH) { */
/* return param_meta[index].name_English; */
/* } */
/* } */
/* } */
/* / * D) 密码类 * / */
/* else if (num > COM_NUM_PASSWORD_START && num < COM_NUM_PASSWORD_END) { */
/* return returnWordType((uint8_t*)"密码", (uint8_t*)"Password"); */
/* } */
/* / * E) 本机参数类 * / */
/* else if (num > COM_NUM_PARA_LOCAL_START && num < COM_NUM_PARA_LOCAL_STOP) { */
/* idx = num - COM_NUM_PARA_LOCAL_START - 1; */
/* if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_local) / sizeof(OperaNameArr_local[0]))) { */
/* return OperaNameArr_local[idx][screen_parameter.language]; */
/* } */
/* } */
/* */
/* return returnWordType((uint8_t*)"非法操作", (uint8_t*)"Invalid Operation"); */
/* } */

/**
 * @brief 按当前语言返回参数输入类型文本；非法语言值会递归调用本函数，调用方必须保证语言枚举有效。
 *
 * @param chinese 中文显示文字指针。
 * @param english 英文显示文字指针。
 * @return 返回选中的按当前语言返回参数输入类型文本；非法语言值会递归调用本函数，调用方必须保证语言枚举有效首地址；结果可能直接别名引用调用方输入，调用方继续持有其存储并负责保证生命周期。
 */
static uint8_t *returnWordType(uint8_t *chinese, uint8_t *english)
{
	if (screen_parameter.language == LANGUAGE_CHINESE) {
		return chinese;
	} else if (screen_parameter.language == LANGUAGE_ENGLISH) {
		return english;
	} else {
		return returnWordType((uint8_t*)"语言错误", (uint8_t*)"LANGUAGE ERROR");
	}
}

/**
 * @brief 计算中英文混排文本的 OLED 像素宽度。
 *
 * @param name 待测量、裁剪、分行或匹配的 OLED 菜单文字字节串；中文按双字节字库字符处理，ASCII 按单字节处理。
 * @return 返回文字占用的 OLED 像素宽度。
 */
static uint8_t oled_text_width(const uint8_t *name)
{
	uint8_t width = 0;

	if (name == NULL) {
		return 0;
	}

	while (*name != 0U) {
		if (*name < 128U) {
			width += 4U;
			name++;
		} else {
			width += 7U;
			name += 3U;
		}
	}

	return width;
}

/**
 * @brief 按当前语言和实际字符宽度把底栏右侧操作贴齐屏幕右边界。
 *
 * @details 调用场景：确认、保存、修改、只读和停止运动等底栏右侧文字绘制。
 * @note 关键约束：文字超过单行宽度时从左边界绘制，并沿用底层现有裁剪行为。
 *
 * @param chinese 中文显示文字指针。
 * @param english 英文显示文字指针。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 */
static void display_right_aligned_action(uint8_t *chinese, uint8_t *english, uint8_t row, uint8_t shift)
{
	uint8_t *text;
	uint8_t width;
	uint8_t line;

	text = (screen_parameter.language == LANGUAGE_ENGLISH && english != NULL) ? english : chinese;
	if (text == NULL) {
		return;
	}

	width = oled_text_width(text);
	line = (width < OLED_LINE8_END) ? (uint8_t)(OLED_LINE8_END - width) : OLED_LINE8_1;
	OledDisplayLineWords(text, line, row, shift);
}

/* 菜单操作码与中英文短名称的只读映射项；用于显示宽度受限的状态栏或紧凑页面。 */
typedef struct {
	/* 菜单操作码与中英文短名称的一一映射。 */
	int opera; /* 菜单操作码，用于把当前表项与菜单元数据及命令映射关联。 */
	uint8_t *name_cn; /* 该表项对应的中文显示名称。 */
	uint8_t *name_en; /* 该表项对应的英文显示名称。 */
} OperaShortNameMap_t;

/**
 * @brief 返回参数或命令的短标签；未命中显式映射时使用传入回退名称。
 *
 * @param num 待查询的菜单操作号或序号。
 * @param fallback 未找到短名称映射时返回的兜底显示文字。
 * @return 返回选中的参数或命令的短标签；未命中显式映射时使用传入回退名称首地址；结果可能直接别名引用调用方输入，调用方继续持有其存储并负责保证生命周期。
 */
static uint8_t *dtm_operaname_short(int num, uint8_t *fallback)
{
	static const OperaShortNameMap_t short_map[] = {
		{ COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION, (uint8_t*)"传感器版本", (uint8_t*)"SenVer" },
		{ COM_NUM_DEVICEPARAM_SOFTWAREVERSION, (uint8_t*)"C2版本", (uint8_t*)"C2Ver" },
		{ COM_NUM_DEVICEPARAM_MAGIC, (uint8_t*)"魔术字", (uint8_t*)"M" },
		{ COM_NUM_DEVICEPARAM_CRC, (uint8_t*)"CRC", (uint8_t*)"CRC" },
		{ COM_NUM_PARA_LOCAL_LEDVERSION, (uint8_t*)"C3版本", (uint8_t*)"C3Ver" },
		{ COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND, (uint8_t*)"上电指令", (uint8_t*)"PwrOnCmd" },
		{ COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO, (uint8_t*)"故障回零", (uint8_t*)"ErrZero" },
		{ COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT, (uint8_t*)"故障停测", (uint8_t*)"ErrStop" },
		{ COM_NUM_DEVICEPARAM_RESERVED2, (uint8_t*)"重跑次数", (uint8_t*)"AutoRecover" },
		{ COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH, (uint8_t*)"位置源切换", (uint8_t*)"PosSwitch" },
		{ COM_NUM_DEVICEPARAM_MOTOR_CURRENT, (uint8_t*)"电机电流", (uint8_t*)"MotorCur" },
		{ COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, (uint8_t*)"编码轮周长", (uint8_t*)"EncCirc" },
		{ COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED, (uint8_t*)"电机限速", (uint8_t*)"MaxSpeed" },
		{ COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM, (uint8_t*)"首圈周长", (uint8_t*)"FirstLoop" },
		{ COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM, (uint8_t*)"尺带厚度", (uint8_t*)"TapeThick" },
		{ COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC, (uint8_t*)"局部周长", (uint8_t*)"MotorCirc" },
		{ COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT, (uint8_t*)"空载上限", (uint8_t*)"EmptyHi" },
		{ COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT, (uint8_t*)"空载下限", (uint8_t*)"EmptyLo" },
		{ COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT, (uint8_t*)"满载上限", (uint8_t*)"FullHi" },
		{ COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT, (uint8_t*)"满载下限", (uint8_t*)"FullLo" },
		{ COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO, (uint8_t*)"碰撞上限", (uint8_t*)"UpperRatio" },
		{ COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO, (uint8_t*)"碰撞下限", (uint8_t*)"LowerRatio" },
		{ COM_NUM_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO, (uint8_t*)"零点阈值", (uint8_t*)"ZeroTh" },
		{ COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE, (uint8_t*)"扭力忽略区", (uint8_t*)"Ignore" },
		{ COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE, (uint8_t*)"零点偏差", (uint8_t*)"ZeroDev" },
		{ COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE, (uint8_t*)"找零距离", (uint8_t*)"ZeroDown" },
		{ COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF, (uint8_t*)"探头距差", (uint8_t*)"SenDiff" },
		{ COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD, (uint8_t*)"找液阈值", (uint8_t*)"FindLvlTh" },
		{ COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, (uint8_t*)"液位方式", (uint8_t*)"LvlMode" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY, (uint8_t*)"跟随频率", (uint8_t*)"FollowHz" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY, (uint8_t*)"跟随密度", (uint8_t*)"FollowD" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME, (uint8_t*)"滞后时间", (uint8_t*)"HysTime" },
		{ COM_NUM_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME, (uint8_t*)"滞后时间", (uint8_t*)"WaterHys" },
		{ COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE, (uint8_t*)"水位方式", (uint8_t*)"WaterMode" },
		{ COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD, (uint8_t*)"跟随阈值", (uint8_t*)"CapTh" },
		{ COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD, (uint8_t*)"寻找阈值", (uint8_t*)"FindCap" },
		{ COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE, (uint8_t*)"下行距离", (uint8_t*)"MaxDown" },
		{ COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD, (uint8_t*)"稳定距离", (uint8_t*)"WaterStb" },
		{ COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD, (uint8_t*)"滞后阈值", (uint8_t*)"LagCap" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE, (uint8_t*)"罐底模式", (uint8_t*)"BotMode" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD, (uint8_t*)"角度阈值", (uint8_t*)"AngleTh" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD, (uint8_t*)"扭力阈值", (uint8_t*)"TorqueTh" },
		{ COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG, (uint8_t*)"更新罐高", (uint8_t*)"UpdTank" },
		{ COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION, (uint8_t*)"罐高偏差", (uint8_t*)"TankDev" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE, (uint8_t*)"探底后修正", (uint8_t*)"BotFix" },
		{ COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, (uint8_t*)"测罐底", (uint8_t*)"NeedBottom" },
		{ COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT, (uint8_t*)"测水位", (uint8_t*)"NeedWater" },
		{ COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, (uint8_t*)"测单点密度", (uint8_t*)"SingleD" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER, (uint8_t*)"分布顺序", (uint8_t*)"SpreadOrd" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE, (uint8_t*)"分布模式", (uint8_t*)"SpreadMode" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT, (uint8_t*)"分布点数", (uint8_t*)"SpreadCnt" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, (uint8_t*)"分布间距", (uint8_t*)"SpreadDist" },
		{ COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME, (uint8_t*)"悬停时间", (uint8_t*)"Hover" },
		{ COM_NUM_DEVICEPARAM_SPREADTOPLIMIT, (uint8_t*)"最高距液面", (uint8_t*)"SpreadTop" },
		{ COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT, (uint8_t*)"最低距罐底", (uint8_t*)"SpreadBot" },
		{ COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT, (uint8_t*)"区间上限", (uint8_t*)"IntTop" },
		{ COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT, (uint8_t*)"区间下限", (uint8_t*)"IntBottom" },
		{ COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE, (uint8_t*)"最高距液面", (uint8_t*)"MaxHeight" },
		{ COM_NUM_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL, (uint8_t*)"探底间隔", (uint8_t*)"BotInterval" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT, (uint8_t*)"修正罐高", (uint8_t*)"FixTankH" },
		{ COM_NUM_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL, (uint8_t*)"SI探底", (uint8_t*)"SIBtmInt" },
		{ COM_NUM_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL, (uint8_t*)"修正液位", (uint8_t*)"LastOil" },
		{ COM_NUM_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE, (uint8_t*)"气相温度", (uint8_t*)"GasTemp" },
		{ COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT, (uint8_t*)"尺带伸缩率", (uint8_t*)"TapeExp" },
		{ COM_NUM_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE, (uint8_t*)"尺带温度", (uint8_t*)"TapeTemp" },
		{ COM_NUM_DEVICEPARAM_RELAY1_OPERATING_MODE, (uint8_t*)"工作模式", (uint8_t*)"K1Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY1_DIGITAL_SOURCE, (uint8_t*)"报警组合", (uint8_t*)"K1AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY1_CONTACT_TYPE, (uint8_t*)"接点类型", (uint8_t*)"K1Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ALARM_MODE, (uint8_t*)"报警模式", (uint8_t*)"K1AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ERROR_VALUE, (uint8_t*)"无效报警", (uint8_t*)"K1Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ALARM_SOURCE, (uint8_t*)"报警源", (uint8_t*)"K1Src" },
		{ COM_NUM_DEVICEPARAM_RELAY1_HH_ALARM_VALUE, (uint8_t*)"HH阈值", (uint8_t*)"K1HH" },
		{ COM_NUM_DEVICEPARAM_RELAY1_H_ALARM_VALUE, (uint8_t*)"H阈值", (uint8_t*)"K1H" },
		{ COM_NUM_DEVICEPARAM_RELAY1_L_ALARM_VALUE, (uint8_t*)"L阈值", (uint8_t*)"K1L" },
		{ COM_NUM_DEVICEPARAM_RELAY1_LL_ALARM_VALUE, (uint8_t*)"LL阈值", (uint8_t*)"K1LL" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ALARM_HYSTERESIS, (uint8_t*)"报警滞回", (uint8_t*)"K1Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY1_DAMPING_FACTOR, (uint8_t*)"阻尼系数", (uint8_t*)"K1Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY1_CLEAR_ALARM, (uint8_t*)"清除锁存", (uint8_t*)"K1Clear" },
		{ COM_NUM_DEVICEPARAM_RELAY2_OPERATING_MODE, (uint8_t*)"工作模式", (uint8_t*)"K2Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY2_DIGITAL_SOURCE, (uint8_t*)"报警组合", (uint8_t*)"K2AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY2_CONTACT_TYPE, (uint8_t*)"接点类型", (uint8_t*)"K2Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ALARM_MODE, (uint8_t*)"报警模式", (uint8_t*)"K2AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ERROR_VALUE, (uint8_t*)"无效报警", (uint8_t*)"K2Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ALARM_SOURCE, (uint8_t*)"报警源", (uint8_t*)"K2Src" },
		{ COM_NUM_DEVICEPARAM_RELAY2_HH_ALARM_VALUE, (uint8_t*)"HH阈值", (uint8_t*)"K2HH" },
		{ COM_NUM_DEVICEPARAM_RELAY2_H_ALARM_VALUE, (uint8_t*)"H阈值", (uint8_t*)"K2H" },
		{ COM_NUM_DEVICEPARAM_RELAY2_L_ALARM_VALUE, (uint8_t*)"L阈值", (uint8_t*)"K2L" },
		{ COM_NUM_DEVICEPARAM_RELAY2_LL_ALARM_VALUE, (uint8_t*)"LL阈值", (uint8_t*)"K2LL" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ALARM_HYSTERESIS, (uint8_t*)"报警滞回", (uint8_t*)"K2Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY2_DAMPING_FACTOR, (uint8_t*)"阻尼系数", (uint8_t*)"K2Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY2_CLEAR_ALARM, (uint8_t*)"清除锁存", (uint8_t*)"K2Clear" },
		{ COM_NUM_DEVICEPARAM_RELAY3_OPERATING_MODE, (uint8_t*)"工作模式", (uint8_t*)"K3Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY3_DIGITAL_SOURCE, (uint8_t*)"报警组合", (uint8_t*)"K3AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY3_CONTACT_TYPE, (uint8_t*)"接点类型", (uint8_t*)"K3Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ALARM_MODE, (uint8_t*)"报警模式", (uint8_t*)"K3AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ERROR_VALUE, (uint8_t*)"无效报警", (uint8_t*)"K3Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ALARM_SOURCE, (uint8_t*)"报警源", (uint8_t*)"K3Src" },
		{ COM_NUM_DEVICEPARAM_RELAY3_HH_ALARM_VALUE, (uint8_t*)"HH阈值", (uint8_t*)"K3HH" },
		{ COM_NUM_DEVICEPARAM_RELAY3_H_ALARM_VALUE, (uint8_t*)"H阈值", (uint8_t*)"K3H" },
		{ COM_NUM_DEVICEPARAM_RELAY3_L_ALARM_VALUE, (uint8_t*)"L阈值", (uint8_t*)"K3L" },
		{ COM_NUM_DEVICEPARAM_RELAY3_LL_ALARM_VALUE, (uint8_t*)"LL阈值", (uint8_t*)"K3LL" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ALARM_HYSTERESIS, (uint8_t*)"报警滞回", (uint8_t*)"K3Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY3_DAMPING_FACTOR, (uint8_t*)"阻尼系数", (uint8_t*)"K3Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY3_CLEAR_ALARM, (uint8_t*)"清除锁存", (uint8_t*)"K3Clear" },
		{ COM_NUM_DEVICEPARAM_RELAY4_OPERATING_MODE, (uint8_t*)"工作模式", (uint8_t*)"K4Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY4_DIGITAL_SOURCE, (uint8_t*)"报警组合", (uint8_t*)"K4AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY4_CONTACT_TYPE, (uint8_t*)"接点类型", (uint8_t*)"K4Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ALARM_MODE, (uint8_t*)"报警模式", (uint8_t*)"K4AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ERROR_VALUE, (uint8_t*)"无效报警", (uint8_t*)"K4Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ALARM_SOURCE, (uint8_t*)"报警源", (uint8_t*)"K4Src" },
		{ COM_NUM_DEVICEPARAM_RELAY4_HH_ALARM_VALUE, (uint8_t*)"HH阈值", (uint8_t*)"K4HH" },
		{ COM_NUM_DEVICEPARAM_RELAY4_H_ALARM_VALUE, (uint8_t*)"H阈值", (uint8_t*)"K4H" },
		{ COM_NUM_DEVICEPARAM_RELAY4_L_ALARM_VALUE, (uint8_t*)"L阈值", (uint8_t*)"K4L" },
		{ COM_NUM_DEVICEPARAM_RELAY4_LL_ALARM_VALUE, (uint8_t*)"LL阈值", (uint8_t*)"K4LL" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ALARM_HYSTERESIS, (uint8_t*)"报警滞回", (uint8_t*)"K4Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY4_DAMPING_FACTOR, (uint8_t*)"阻尼系数", (uint8_t*)"K4Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY4_CLEAR_ALARM, (uint8_t*)"清除锁存", (uint8_t*)"K4Clear" },
		{ COM_NUM_DEVICEPARAM_AO_WORK_MODE, (uint8_t*)"工作模式", (uint8_t*)"Work Mode" },
		{ COM_NUM_DEVICEPARAM_AO_CURRENT_MODE, (uint8_t*)"电流模式", (uint8_t*)"Current Mode" },
		{ COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE, (uint8_t*)"输出源", (uint8_t*)"Source" },
		{ COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000, (uint8_t*)"电流修正", (uint8_t*)"AO Trim" },
		{ COM_NUM_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100, (uint8_t*)"固定电流", (uint8_t*)"Fixed Current" },
		{ COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM, (uint8_t*)"0%对应值", (uint8_t*)"0% Value" },
		{ COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM, (uint8_t*)"100%值", (uint8_t*)"100% Value" },
		{ COM_NUM_DEVICEPARAM_AO_DAMPING_X10_S, (uint8_t*)"阻尼系数", (uint8_t*)"Damping" },
		{ COM_NUM_DEVICEPARAM_AO_FAULT_MODE, (uint8_t*)"故障动作", (uint8_t*)"Fault Action" },
		{ COM_NUM_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100, (uint8_t*)"故障电流", (uint8_t*)"Fault Current" },
		{ COM_NUM_DEVICEPARAM_AO_ERROR_LEVEL, (uint8_t*)"隐藏预留", (uint8_t*)"Reserved" },
		{ COM_NUM_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100, (uint8_t*)"初始电流", (uint8_t*)"Initial Current" },
		{ COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100, (uint8_t*)"模拟电流", (uint8_t*)"Sim Current" },
		{ COM_NUM_AO_SIMULATION_ENABLE, (uint8_t*)"输出模拟", (uint8_t*)"Simulation" },
		{ COM_NUM_AO_RUNTIME_PROCESS_VALUE, (uint8_t*)"输入值", (uint8_t*)"Input Value" },
		{ COM_NUM_AO_RUNTIME_PERCENT, (uint8_t*)"输入比例", (uint8_t*)"Input Percent" },
		{ COM_NUM_AO_RUNTIME_OUTPUT_CURRENT, (uint8_t*)"输出电流", (uint8_t*)"Output Current" },
		{ COM_NUM_MAINTENANCE_EXIT, (uint8_t*)"退出维护模式", (uint8_t*)"Exit MNT" },
		{ COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS, (uint8_t*)"清全部锁存", (uint8_t*)"Clear All" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD, (uint8_t*)"滞后阈值", (uint8_t*)"HysTh" },
		{ COM_NUM_DEVICEPARAM_SP_MEAS_POSITION, (uint8_t*)"测量位置", (uint8_t*)"SP_MeasPos" },
		{ COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION, (uint8_t*)"监测位置", (uint8_t*)"SP_MonPos" },
		{ COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL, (uint8_t*)"运行位置", (uint8_t*)"DistOilLvl" },
		{ COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE, (uint8_t*)"指令距离", (uint8_t*)"MotorDist" },
		{ COM_NUM_SCREEN_INPUT_D_SWITCH, (uint8_t*)"上传密度", (uint8_t*)"InDSw" },
		{ COM_NUM_SCREEN_DECIMAL, (uint8_t*)"小数点位", (uint8_t*)"Decimal" },
		{ COM_NUM_CPU3_COM1_BAUDRATE, (uint8_t*)"C1波特率", (uint8_t*)"C1Baud" },
		{ COM_NUM_CPU3_COM1_DATABITS, (uint8_t*)"C1数据位", (uint8_t*)"C1Data" },
		{ COM_NUM_CPU3_COM1_STOPBITS, (uint8_t*)"C1停止位", (uint8_t*)"C1Stop" },
		{ COM_NUM_CPU3_COM2_BAUDRATE, (uint8_t*)"C2波特率", (uint8_t*)"C2Baud" },
		{ COM_NUM_CPU3_COM2_DATABITS, (uint8_t*)"C2数据位", (uint8_t*)"C2Data" },
		{ COM_NUM_CPU3_COM2_STOPBITS, (uint8_t*)"C2停止位", (uint8_t*)"C2Stop" },
		{ COM_NUM_CPU3_COM3_BAUDRATE, (uint8_t*)"C3波特率", (uint8_t*)"C3Baud" },
		{ COM_NUM_CPU3_COM3_DATABITS, (uint8_t*)"C3数据位", (uint8_t*)"C3Data" },
		{ COM_NUM_CPU3_COM3_STOPBITS, (uint8_t*)"C3停止位", (uint8_t*)"C3Stop" },
	};

	if (screen_operation_is_no_para_command(num) ||
	    ((num > COM_NUM_ONEPARACMD_START) && (num < COM_NUM_ONEPARACMD_END)) ||
	    ((num > COM_NUM_ONEPARA_DEBUGCMD_START) && (num < COM_NUM_NOPARA_DEBUGCMD_END))) {
		return fallback;
	}

	for (int i = 0; i < (int)(sizeof(short_map) / sizeof(short_map[0])); i++) {
		if (num == short_map[i].opera) {
			return (screen_parameter.language == LANGUAGE_CHINESE) ?
				short_map[i].name_cn : short_map[i].name_en;
		}
	}

	return fallback;
}

/**
 * @brief 返回菜单项在当前语言下使用的显示名称。
 *
 * @param item 待显示或判断的菜单项元数据。该参数包含菜单操作号、中英文名称、回调及显示属性，用于选择文字和当前参数值。
 * @return 返回菜单项在当前语言下使用的显示名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static uint8_t *menu_display_name(const struct MenuData *item)
{
	uint8_t *name;

	if (item == NULL) {
		return (uint8_t*)"";
	}

	name = (screen_parameter.language == LANGUAGE_CHINESE || item->operaName2 == NULL) ?
		item->operaName : item->operaName2;

	return dtm_operaname_short(item->operaNum, name);
}

/**
 * @brief 按 OLED 实际列宽裁剪字符串，避免长英文或长中文名称越过单行右边界。
 *
 * @param name 待测量、裁剪、分行或匹配的 OLED 菜单文字字节串；中文按双字节字库字符处理，ASCII 按单字节处理。
 * @param max_width 允许文字或百分比占用的最大 OLED 像素宽度。
 * @return 返回存放按 OLED 实际列宽裁剪字符串，避免长英文或长中文名称越过单行右边界的模块静态缓冲区首地址；后续调用可能覆盖其内容，调用方不得释放。
 */
static uint8_t *oled_fit_text(uint8_t *name, uint8_t max_width)
{
	static uint8_t fit[64];
	uint8_t width = 0;
	uint8_t len = 0;
	uint8_t *p = name;

	if (name == NULL) {
		fit[0] = 0U;
		return fit;
	}

	memset(fit, 0, sizeof(fit));
	while (*p != 0U) {
		uint8_t step = (*p < 128U) ? 1U : 3U;
		uint8_t char_width = (*p < 128U) ? 4U : 7U;

		if ((uint8_t)(width + char_width) > max_width || (uint8_t)(len + step) >= sizeof(fit)) {
			break;
		}

		memcpy(&fit[len], p, step);
		len += step;
		width += char_width;
		p += step;
	}

	return fit;
}

/**
 * @brief 判断参数操作号是否对应软件或协议版本字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号对应 CPU2/CPU3 软件版本或协议版本字段，0 表示不是版本字段。
 */
static int is_version_value_opera(int operaNum)
{
	switch (operaNum) {
	case COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION:
	case COM_NUM_DEVICEPARAM_SOFTWAREVERSION:
	case COM_NUM_PARA_LOCAL_LEDVERSION:
		return 1;
	default:
		return 0;
	}
}

/**
 * @brief 判断参数操作号是否对应应以十六进制显示的 32 位字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号对应需要按 8 位十六进制显示的 32 位字段，0 表示使用普通数值格式。
 */
static int is_hex_u32_value_opera(int operaNum)
{
	switch (operaNum) {
	case COM_NUM_DEVICEPARAM_MAGIC:
	case COM_NUM_DEVICEPARAM_CRC:
		return 1;
	default:
		return 0;
	}
}

/**
 * @brief 显示 32 位只读值：版本号按 Vx.x.x.x，魔术字/CRC 按 0xXXXXXXXX。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @param value 待格式化并绘制到 OLED 的数值。
 * @param line OLED 绘制使用的横向列位置。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 * @return 1 表示 operaNum 属于版本号或 32 位十六进制只读项，数值已按对应格式绘制；0 表示该操作号不属于这两类，调用方应继续使用普通数值格式。
 */
static int display_formatted_readonly_value(int operaNum, int32_t value, uint8_t line, uint8_t row, uint8_t shift)
{
	char text[24];
	uint32_t raw = (uint32_t)value;

	if (is_version_value_opera(operaNum) != 0) {
		format_version_u32(raw, text, sizeof(text));
		OledDisplayLineWords((uint8_t*)text, line, row, shift);
		return 1;
	}

	if (is_hex_u32_value_opera(operaNum) != 0) {
		snprintf(text, sizeof(text), "0x%08lX", (unsigned long)raw);
		OledDisplayLineWords((uint8_t*)text, line, row, shift);
		return 1;
	}

	return 0;
}

/**
 * @brief 返回继电器报警源对应的工程单位文字。
 *
 * @param source 待判断或转换的数据来源枚举值。该 RelayAlarmSource 编码决定详情页显示 mm、℃ 或无单位，非法来源返回空单位。
 * @return 成功时返回指向继电器报警源对应的工程单位文字的指针；输入非法或未找到匹配项时返回 NULL。
 */
static uint8_t *relay_alarm_source_unit(uint32_t source)
{
	switch ((RelayAlarmSource)source) {
	case RELAY_ALARM_SOURCE_TANK_LEVEL:
	case RELAY_ALARM_SOURCE_WATER_LEVEL:
	case RELAY_ALARM_SOURCE_DISPLACER_POS:
		return (uint8_t*)"mm";
	case RELAY_ALARM_SOURCE_LIQUID_TEMP:
		return (uint8_t*)"℃";
	default:
		return NULL;
	}
}

/**
 * @brief 根据参数操作号和元数据返回详情页使用的工程单位。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @param meta 目标参数的元数据描述。
 * @return 成功时返回指向根据参数操作号和元数据返回详情页使用的工程单位的指针；输入非法或未找到匹配项时返回 NULL。
 */
static uint8_t *param_display_unit(int operaNum, const struct ParameterMetadata *meta)
{
	int channel;

	if ((meta != NULL) && (meta->unit != NULL)) {
		return meta->unit;
	}

	/* 继电器阈值和滞回的单位取决于同一路“报警取值源”，不能写死在 param_meta[]。 */
	if (RelayParam_IsAlarmValueField(operaNum) == 0) {
		return (meta != NULL) ? meta->unit : NULL;
	}

	channel = RelayParam_ChannelOf(operaNum);
	if ((channel < 0) || (channel >= (int)RELAY_ALARM_CHANNEL_COUNT)) {
		return NULL;
	}

	return relay_alarm_source_unit(g_deviceParams.relayAlarm[channel].alarm_source);
}

/**
 * @brief 把电机电流参数归一到 TMC5130 IRUN 的合法档位。
 *
 * @details 调用场景：电机电流详情页、菜单列表和选择列表显示前调用。
 * @note 关键约束：只修正显示侧口径，实际写入范围仍由参数范围检查和 CPU2 保护。
 *
 * @param irun TMC5130 运行电流档位 IRUN。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint32_t motor_current_clamp_irun(uint32_t irun)
{
	if ((irun < MOTOR_CURRENT_MIN) || (irun > MOTOR_CURRENT_MAX)) {
		return MOTOR_CURRENT_DEFAULT;
	}

	return irun;
}

/**
 * @brief 按当前硬件 RSENSE=0.15Ω 和 TMC5130 vsense=0 的口径换算 RMS 电流。
 *
 * @details 调用场景：屏幕显示电机电流档位对应的相电流参考值。
 * @note 关键约束：返回值只用于显示，不改变 motor_current 保存和通信语义。
 *
 * @param irun TMC5130 运行电流档位 IRUN。
 * @return 返回 IRUN 档位按当前采样电阻和 vsense 配置换算的 RMS 电流，单位 mA。
 */
static uint16_t motor_current_rms_ma(uint32_t irun)
{
	uint32_t normalized = motor_current_clamp_irun(irun);
	uint32_t index = normalized - MOTOR_CURRENT_RMS_TABLE_OFFSET;

	if (index >= (sizeof(motor_current_rms_ma_table) / sizeof(motor_current_rms_ma_table[0]))) {
		return motor_current_rms_ma_table[MOTOR_CURRENT_DEFAULT - MOTOR_CURRENT_RMS_TABLE_OFFSET];
	}

	return motor_current_rms_ma_table[index];
}

/**
 * @brief 生成“IRUN档位 + RMS电流”的短显示文本。
 *
 * @details 调用场景：机械参数列表、参数详情页和选择确认后的显示刷新。
 * @note 关键约束：电流值按 0.01A 四舍五入，保持与选择列表一致。
 *
 * @param irun TMC5130 运行电流档位 IRUN。
 * @param buf 用于接收 IRUN 档位和 RMS 电流格式化文字的目标缓冲区。
 * @param buf_size 缓冲区容量，单位字节。
 */
static void format_motor_current_label(uint32_t irun, char *buf, size_t buf_size)
{
	uint32_t normalized = motor_current_clamp_irun(irun);
	uint16_t rms_ma = motor_current_rms_ma(normalized);
	uint16_t rms_centiamps = (uint16_t)((rms_ma + 5U) / 10U);

	if ((buf == NULL) || (buf_size == 0U)) {
		return;
	}

	snprintf(buf,
	         buf_size,
	         "%02lu %lu.%02luA",
	         (unsigned long)normalized,
	         (unsigned long)(rms_centiamps / 100U),
	         (unsigned long)(rms_centiamps % 100U));
}

/**
 * @brief 在参数详情页显示电机电流档位和对应 RMS 电流。
 *
 * @details 调用场景：查看“电机运行电流”参数时调用。
 * @note 关键约束：第三行显示的是 IRUN 档位范围，不改变 Modbus 参数范围。
 *
 * @param irun TMC5130 运行电流档位 IRUN。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 */
static void display_motor_current_detail(uint32_t irun, uint8_t row)
{
	char label[16];
	uint8_t line;

	line = DisplayLangaugeLineWords((uint8_t*)"当前值:", OLED_LINE8_1, row, 0, (uint8_t*)"Value:");
	format_motor_current_label(irun, label, sizeof(label));
	OledDisplayLineWords((uint8_t*)label, line, row, 0);
	line = DisplayLangaugeLineWords((uint8_t*)"RMS范围:", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"RMS Range:");
	OledDisplayLineWords((uint8_t*)"01-31", line, OLED_ROW4_3, 0);
}

/**
 * @brief 显示菜单项名称；参数项同时读取并显示当前值或枚举文本。
 *
 * @param item 待显示或判断的菜单项元数据。该参数包含菜单操作号、中英文名称、回调及显示属性，用于选择文字和当前参数值。
 * @param line OLED 绘制使用的横向列位置。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 */
static void display_menu_item_with_value(const struct MenuData *item, uint8_t line, uint8_t row, uint8_t shift)
{
	int index;
	int old_opera;
	int opera;
	bool is_param_item;
	uint8_t *name;

	if (item == NULL) {
		return;
	}

	opera = item->operaNum;
	is_param_item = (((opera > COM_NUM_PARA_DEBUG_START) && (opera < COM_NUM_PARA_DEBUG_END)) ||
	                 ((opera > COM_NUM_PARA_LOCAL_START) && (opera < COM_NUM_PARA_LOCAL_STOP)));
	name = oled_fit_text(menu_display_name(item), is_param_item ? OLED_LINE8_6 : OLED_LINE8_END);
	line = OledDisplayLineWords(name, line, row, shift);
	if ((item->sureopera == setchinese) || (item->sureopera == setenglish)) {
		return;
	}

	/* 带参指令只显示指令名，参数值在输入页处理，不在菜单项后追加。 */
	if (!is_param_item) {
		if (opera == COM_NUM_AO_SIMULATION_ENABLE) {
			uint32_t enabled = (g_measurement.ao_output_runtime.simulation_enabled == 0U) ? 0U : 1U;
			line = OledDisplayLineWords((uint8_t*)":", line, row, shift);
			OledDisplayLineWords(arr_ao_simulation_enable[enabled][screen_parameter.language], line, row, shift);
		}
		return;
	}

	index = getHoldValueNum(opera);
	if (index < 0) {
		return;
	}

	if (Cpu3Local_IsParam((OperatingNumber)opera)) {
		param_meta[index].val = Cpu3Local_ReadValue((OperatingNumber)opera);
	}

	line = OledDisplayLineWords((uint8_t*)":", line, row, shift);
	if (display_formatted_readonly_value(opera, param_meta[index].val, line, row, shift) != 0) {
		return;
	}

	if (opera == COM_NUM_DEVICEPARAM_MOTOR_CURRENT) {
		char label[16];
		char *unit;

		format_motor_current_label((uint32_t)param_meta[index].val, label, sizeof(label));
		unit = strchr(label, 'A');
		if (unit != NULL) {
			*unit = '\0';
		}
		OledDisplayLineWords((uint8_t*)label, line, row, shift);
		return;
	}

	if (param_meta[index].pword == NULL) {
		OledValueDisplay(param_meta[index].val,
		                 line,
		                 row,
		                 shift,
		                 param_meta[index].point,
		                 param_display_unit(opera, &param_meta[index]));
	} else {
		old_opera = now_Opera_Num;
		now_Opera_Num = opera;
		OledDisplayLineWords(oled_fit_text(param_meta[index].pword(), OLED_LINE8_END - line), line, row, shift);
		now_Opera_Num = old_opera;
	}
}

/**
 * @brief 按 OLED 宽度把标题拆成最多两行，并返回下一可用行。
 *
 * @param name 待测量、裁剪、分行或匹配的 OLED 菜单文字字节串；中文按双字节字库字符处理，ASCII 按单字节处理。
 * @param row1 用于返回标题第一显示行文字的定长缓存。
 * @param row2 用于返回标题第二显示行文字的定长缓存。
 * @return 返回标题绘制完成后的下一可用 OLED 行坐标；单行和双行标题分别按实际占用行数推进。
 */
static uint8_t display_split_title(uint8_t *name, uint8_t row1, uint8_t row2)
{
	static uint8_t part1[64];
	static uint8_t part2[64];
	uint8_t width = 0;
	uint8_t len1 = 0;
	uint8_t len2 = 0;
	uint8_t *p = name;

	if (name == NULL) {
		return (uint8_t)(row1 + OLED_ROW4_2);
	}

	if (oled_text_width(name) <= OLED_LINE8_END) {
		OledDisplayLineWords(name, OLED_LINE8_1, row1, 0);
		return (uint8_t)(row1 + OLED_ROW4_2);
	}

	memset(part1, 0, sizeof(part1));
	memset(part2, 0, sizeof(part2));

	while (*p != 0U) {
		uint8_t step = (*p < 128U) ? 1U : 3U;
		uint8_t w = (*p < 128U) ? 4U : 7U;

		if ((width + w) > OLED_LINE8_END || (len1 + step) >= sizeof(part1)) {
			break;
		}
		memcpy(&part1[len1], p, step);
		len1 += step;
		width += w;
		p += step;
	}

	width = 0;
	while (*p != 0U) {
		uint8_t step = (*p < 128U) ? 1U : 3U;
		uint8_t w = (*p < 128U) ? 4U : 7U;

		if ((width + w) > OLED_LINE8_END || (len2 + step) >= sizeof(part2)) {
			break;
		}
		memcpy(&part2[len2], p, step);
		len2 += step;
		width += w;
		p += step;
	}

	OledDisplayLineWords(part1, OLED_LINE8_1, row1, 0);
	if (part2[0] != 0U) {
		OledDisplayLineWords(part2, OLED_LINE8_1, row2, 0);
		return (uint8_t)(row2 + OLED_ROW4_2);
	}

	return (uint8_t)(row1 + OLED_ROW4_2);
}

/**
 * @brief 显示当前参数值及单位，并返回下一可用行。
 *
 * @param meta 目标参数的元数据描述。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 */
static void display_param_detail_value(const struct ParameterMetadata *meta, uint8_t row)
{
	uint8_t line;

	line = DisplayLangaugeLineWords((uint8_t*)"当前值:", OLED_LINE8_1, row, 0, (uint8_t*)"Value:");
	if (meta == NULL) {
		DisplayLangaugeLineWords((uint8_t*)"--", line, row, 0, (uint8_t*)"--");
		return;
	}

	if (meta->pword == NULL) {
		OledValueDisplay(meta->val, line, row, 0, meta->point, param_display_unit(meta->operanum, meta));
	} else {
		OledDisplayLineWords(meta->pword(), line, row, 0);
	}
}

/**
 * @brief 按参数类型显示可写范围或枚举范围。
 *
 * @param meta 目标参数的元数据描述。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 */
static void display_param_detail_range(const struct ParameterMetadata *meta, uint8_t row)
{
	uint8_t line;

	line = DisplayLangaugeLineWords((uint8_t*)"范围:", OLED_LINE8_1, row, 0, (uint8_t*)"Range:");
	if (meta == NULL) {
		DisplayLangaugeLineWords((uint8_t*)"--", line, row, 0, (uint8_t*)"--");
		return;
	}

	if (meta->flag_checkvalue) {
		line = OledValueDisplay(meta->valuemin, line, row, 0, meta->point, NULL);
		line = OledDisplayOneNmb(11, row, line, 0);
		if ((meta->operanum == COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM) ||
		    (meta->operanum == COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM)) {
			OledValueDisplay(ao_range_max_01mm(), line, row, 0, meta->point, param_display_unit(meta->operanum, meta));
		} else {
			OledValueDisplay(meta->valuemax, line, row, 0, meta->point, param_display_unit(meta->operanum, meta));
		}
	} else {
		DisplayLangaugeLineWords((uint8_t*)"--", line, row, 0, (uint8_t*)"--");
	}
}

/**
 * @brief 按当前参数操作号返回数值输入页和详情页使用的工程单位文字。
 *
 * @return 当前操作号具有参数元数据时返回对应工程单位文字首地址；无单位或非参数操作时返回 NULL。
 */
static uint8_t *dtm_unit(void)
{
	uint8_t *u = NULL;

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		u = param_display_unit(now_Opera_Num, &param_meta[index]);
	} else {
		switch (now_Opera_Num) {
		default:
			break;
		}
	}

	return u;
}

/**
 * @brief 确定显示位数。
 *
 * @return 返回当前参数输入框允许显示的十进制总位数。
 */
static uint8_t dtm_bits(void)
{
	uint8_t b = 6;

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		b = param_meta[index].bits;
	} else {
		switch (now_Opera_Num) {
		case COM_NUM_PASSWORD_ENTER_PARA:
		case COM_NUM_PASSWORD_ENTER_CMD:
			b = 4;
			break;
		default:
			break;
		}
	}

	return b;
}

/**
 * @brief 输入数据(数值输入状态机)。
 *
 * @param deci 当前数值整数部分的十进制位数。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param line OLED 绘制使用的横向列位置。
 * @param points 当前参数允许输入的小数位数；用于限定小数点位置和逐位编辑范围。
 * @param unit 当前参数的工程单位文字，用于输入页提示。
 * @param value 数值输入缓存；函数按键逐位修改并在确认时输出最终整数。
 * @return 1 表示数值输入已确认并写回，0 表示仍在编辑、取消或输入尚未完成。
 */
static bool inputvalue(uint8_t deci, uint8_t row, uint8_t line, uint8_t points, uint8_t *unit, int *value)
{
	static int nowbit = 8;
	static int sgl_val = 0;
	static uint8_t bit_0 = 0;
	static uint8_t bit_1 = 0;
	static uint8_t bit_2 = 0;
	static uint8_t bit_3 = 0;
	static uint8_t bit_4 = 0;
	static uint8_t bit_5 = 0;
	static uint8_t bit_6 = 0;

	if (nowbit >= 0) {
		if (nowbit > deci) {
			nowbit = deci;
		}

		if (NowKeyPress == USE_KEY_UP) {
			sgl_val++;
			if (sgl_val > 9) {
				sgl_val %= 10;
			}
		} else if (NowKeyPress == USE_KEY_DOWN) {
			sgl_val--;
			if (sgl_val < 0) {
				sgl_val += 10;
			}
		} else if (NowKeyPress == USE_KEY_SURE) {
			nowbit--;
			sgl_val = 0;

			if (nowbit < 0) {
				if (NowKeyPress == USE_KEY_SURE) {
					*value = bit_0 + bit_1 * 10 + bit_2 * 100 + bit_3 * 1000 + bit_4 * 10000 + bit_5 * 100000 + bit_6 * 1000000;
					nowbit = 8;
					sgl_val = 0;
					bit_0 = bit_1 = bit_2 = bit_3 = bit_4 = bit_5 = bit_6 = 0;
					return true;
				}
			}
		} else if (NowKeyPress == USE_KEY_BACK) {
			if ((nowbit == (deci - 1)) && (sgl_val == 0)) {
				nowbit = 8;
				if (timeback != 0) {
					timeback = 0;
					timesure = 1;
					dtm_backtofunc()();
					return false;
				}
				timeback++;
				timesure--;
			}

			nowbit = deci - 1;
			sgl_val = 0;
			bit_0 = bit_1 = bit_2 = bit_3 = bit_4 = bit_5 = bit_6 = 0;
		}

		switch (nowbit) {
		case 0:
			bit_0 = (uint8_t)sgl_val;
			break;
		case 1:
			bit_1 = (uint8_t)sgl_val;
			break;
		case 2:
			bit_2 = (uint8_t)sgl_val;
			break;
		case 3:
			bit_3 = (uint8_t)sgl_val;
			break;
		case 4:
			bit_4 = (uint8_t)sgl_val;
			break;
		case 5:
			bit_5 = (uint8_t)sgl_val;
			break;
		case 6:
			bit_6 = (uint8_t)sgl_val;
			break;
		default:
			break;
		}
	} else {
		if (NowKeyPress == USE_KEY_SURE) {
			*value = bit_0 + bit_1 * 10 + bit_2 * 100 + bit_3 * 1000 + bit_4 * 10000 + bit_5 * 100000 + bit_6 * 1000000;
			nowbit = 8;
			sgl_val = 0;
			bit_0 = bit_1 = bit_2 = bit_3 = bit_4 = bit_5 = bit_6 = 0;
			return true;
		}
	}

	/* 显示 */
	switch (deci) {
	case 7: {
		if (points == 6) {
			line = OledDisplayOneNmb(10, row, line, 0) - 2;
		}
		line = OledDisplayOneNmb(bit_6, row, line, (nowbit & 1) == 0 && (nowbit | 1) == 7);
	}
	case 6: {
		if (points == 5) {
			line = OledDisplayOneNmb(10, row, line, 0) - 2;
		}
		line = OledDisplayOneNmb(bit_5, row, line, (nowbit & 2) == 0 && (nowbit | 2) == 7);
	}
	case 5: {
		line = OledDisplayOneNmb(bit_4, row, line, (nowbit & 3) == 0 && (nowbit | 3) == 7);
	}
	case 4: {
		if (points == 4) {
			line = OledDisplayOneNmb(10, row, line, 0) - 2;
		}
		line = OledDisplayOneNmb(bit_3, row, line, (nowbit & 4) == 0 && (nowbit | 4) == 7);
	}
	case 3: {
		if (points == 3) {
			line = OledDisplayOneNmb(10, row, line, 0) - 2;
		}
		line = OledDisplayOneNmb(bit_2, row, line, (nowbit & 5) == 0 && (nowbit | 5) == 7);
	}
	case 2: {
		if (points == 2) {
			line = OledDisplayOneNmb(10, row, line, 0) - 2;
		}
		line = OledDisplayOneNmb(bit_1, row, line, (nowbit & 6) == 0 && (nowbit | 6) == 7);
	}
	case 1: {
		if (points == 1) {
			line = OledDisplayOneNmb(10, row, line, 0) - 2;
		}
		line = OledDisplayOneNmb(bit_0, row, line, !(nowbit & 7));
	}
	default:
		break;
	}

	if (unit != NULL) {
		OledDisplayLineWords(unit, line, row, 0);
	}

	display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, !(nowbit & 7));

	if ((nowbit == (deci - 1)) && (sgl_val == 0)) {
		DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	} else {
		DisplayLangaugeLineWords((uint8_t*)"清零", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Zero out");
	}

	return false;
}

/**
 * @brief 绘制当前命令或参数的确认页，并按确认或返回按键跳转到对应处理流程。
 *
 * 函数依据 now_Opera_Num 区分无参命令、带参命令、普通参数和密码入口，分别绘制中英文确认标题、操作短名称以及待写参数值。
 * 密码入口在本页直接校验屏幕密码或固定维护密码，成功后进入参数或命令菜单，失败时提示并返回主菜单，不再进入通用确认分支。
 * 确认和返回状态由 timesure、timeback 维护；达到确认条件后调用 dtm_suretofunc 取得执行入口，返回条件成立时调用 dtm_backtofunc 恢复上一页。
 */
static void ifsendcmd(void)
{
	oled_clear();
	func_index = KEYNUM_IFSENDCMD;

	if (screen_operation_is_no_para_command(now_Opera_Num)) {
		if (now_Opera_Num == COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS) {
			DisplayLangaugeLineWords((uint8_t*)"清除全部继电器", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Clear All Relay");
			DisplayLangaugeLineWords((uint8_t*)"锁存报警?", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Latched Alarms?");
		} else if (now_Opera_Num == COM_NUM_MAINTENANCE_MODE) {
			DisplayLangaugeLineWords((uint8_t*)"是否进入", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Enter");
			DisplayLangaugeLineWords((uint8_t*)"维护模式?", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Maintenance?");
		} else if (now_Opera_Num == COM_NUM_MAINTENANCE_EXIT) {
			DisplayLangaugeLineWords((uint8_t*)"是否退出", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Exit");
			DisplayLangaugeLineWords((uint8_t*)"维护模式?", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Maintenance?");
		} else {
			DisplayLangaugeLineWords((uint8_t*)"是否下发指令:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue instruct:");
			display_split_title(dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num)), OLED_ROW4_2, OLED_ROW4_3);
		}
	} else if (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发带参指令:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue IWP:");
		display_split_title(dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num)), OLED_ROW4_2, OLED_ROW4_3);
	} else if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发参数:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue Para:");
		OledDisplayLineWords(oled_fit_text(dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num)), OLED_LINE8_END),
		                     OLED_LINE8_1,
		                     OLED_ROW4_2,
		                     0);
		if (now_Para_CT.bits <= 3) {
			OledValueDisplay(now_Para_CT.val, OLED_LINE8_4, OLED_ROW4_3, 0, now_Para_CT.points, now_Para_CT.unit);
		} else {
			OledValueDisplay(now_Para_CT.val, OLED_LINE8_3, OLED_ROW4_3, 0, now_Para_CT.points, now_Para_CT.unit);
		}
	} else if (now_Opera_Num > COM_NUM_PASSWORD_START && now_Opera_Num < COM_NUM_PASSWORD_END) {
		if (now_Para_CT.val == g_cpu3_comm_display_params.screen_password || now_Para_CT.val == FIXPASSWORD) {
			DisplayLangaugeLineWords((uint8_t*)"密码正确!", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Password Correct");
			HAL_Delay(100);
			if (now_Opera_Num == COM_NUM_PASSWORD_ENTER_PARA) {
				menu_paracfg_main();
			} else if (now_Opera_Num == COM_NUM_PASSWORD_ENTER_CMD) {
				menu_cmdconfig_main();
			}
		} else {
			DisplayLangaugeLineWords((uint8_t*)"密码错误!", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Password Error");
			HAL_Delay(500);
			mainmenu();
		}
		return;
	}

	if (NowKeyPress == USE_KEY_SURE) {
		if (timesure > 1) {
			timesure = 0;
			timeback = 0;
			dtm_suretofunc()();
			return;
		}
		timesure++;
		if (timeback > 0) {
			timeback = 0;
		}
	} else if (NowKeyPress == USE_KEY_BACK) {
		if (timeback != 0) {
			timeback = 0;
			timesure = 1;
			dtm_backtofunc()();
			return;
		}
		timeback++;
		timesure--;
	}

	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, timeback, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, timesure);
}

/**
 * @brief 判断当前操作是否需要二次保护确认。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return true 表示当前操作需要二次保护确认；false 表示当前操作不需要二次保护确认。
 */
static bool operation_needs_protect_confirm(int operaNum)
{
	switch (operaNum) {
	case COM_NUM_RESTOR_EFACTORYSETTING:

	/* 运行策略 */
	case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:
	case COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO:
	case COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT:
	case COM_NUM_DEVICEPARAM_RESERVED2:
	case COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH:

	/* 测量位置换算和电机动作 */
	case COM_NUM_DEVICEPARAM_MOTOR_CURRENT:
	case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
	case COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED:
	case COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM:
	case COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM:
	case COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE:
	case COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC:

	/* 罐高、罐底和关键测量边界 */
	case COM_NUM_DEVICEPARAM_TANKHEIGHT:
	case COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF:
	case COM_NUM_DEVICEPARAM_BLINDZONE:
	case COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD:
	case COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD:
	case COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD:
	case COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT:
	case COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE:
	case COM_NUM_DEVICEPARAM_WATER_BLINDZONE:
	case COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD:
	case COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD:
	case COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE:
	case COM_NUM_DEVICEPARAM_ZERO_CAP:
	case COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD:
	case COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD:
	case COM_NUM_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME:
	case COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION:
	case COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE:
	case COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD:
	case COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD:
	case COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG:
	case COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION:
	case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE:
	case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT:
	case COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE:

	/* 通信协议和串口物理参数 */
	case COM_NUM_CPU3_COM1_BAUDRATE:
	case COM_NUM_CPU3_COM1_DATABITS:
	case COM_NUM_CPU3_COM1_PARITY:
	case COM_NUM_CPU3_COM1_STOPBITS:
	case COM_NUM_CPU3_COM1_PROTOCOL:
	case COM_NUM_CPU3_COM2_BAUDRATE:
	case COM_NUM_CPU3_COM2_DATABITS:
	case COM_NUM_CPU3_COM2_PARITY:
	case COM_NUM_CPU3_COM2_STOPBITS:
	case COM_NUM_CPU3_COM2_PROTOCOL:
	case COM_NUM_CPU3_COM3_BAUDRATE:
	case COM_NUM_CPU3_COM3_DATABITS:
	case COM_NUM_CPU3_COM3_PARITY:
	case COM_NUM_CPU3_COM3_STOPBITS:
	case COM_NUM_CPU3_COM3_PROTOCOL:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 执行受保护操作的确认、取消和命令下发流程。
 */
static void protected_operation_process(void)
{
	if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
		cmd_nopara_process();
		return;
	}

	if (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE) {
		(void)ao_write_output_source();
		return;
	}

	if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP) {
		cmd_configpara_process();
		return;
	}

	errorprocess();
}

/**
 * @brief 保护参数不增加维护权限，只在真正写入前追加确认，降低误保存风险。
 */
static void param_protect_confirm(void)
{
	uint8_t *name;

	oled_clear();
	func_index = KEYNUM_IF_PARAM_PROTECT_CONFIRM;

	if (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE) {
		DisplayLangaugeLineWords((uint8_t*)"切换输出源", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Change Source");
	} else {
		DisplayLangaugeLineWords((uint8_t*)"参数保护", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Protected");
	}
	name = dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num));
	OledDisplayLineWords(oled_fit_text(name, OLED_LINE8_END), OLED_LINE8_1, OLED_ROW4_2, 0);
	if (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE) {
		DisplayLangaugeLineWords((uint8_t*)"量程重新加载", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Reload Range");
	} else {
		DisplayLangaugeLineWords((uint8_t*)"请确认", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Confirm");
	}

	if (NowKeyPress == USE_KEY_SURE) {
		if (timesure > 1) {
			timesure = 0;
			timeback = 0;
			protected_operation_process();
			return;
		}
		timesure++;
		if (timeback > 0) {
			timeback = 0;
		}
	} else if (NowKeyPress == USE_KEY_BACK) {
		if (timeback != 0) {
			timeback = 0;
			timesure = 1;
			if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
				menu_debug_system();
			} else {
				displaypara();
			}
			return;
		}
		timeback++;
		if (timesure > 0) {
			timesure--;
		}
	}

	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, timeback, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"确认保存", (uint8_t*)"Save", OLED_ROW4_4, timesure);
}



/**
 * @brief 返回按返回键后要跳转的函数指针。
 *
 * @return 返回当前操作按返回键时应调用的页面函数；未命中特殊映射时返回参数元数据配置的返回函数。
 */
static pFunc_void dtm_backtofunc(void)
{
    pFunc_void p = errorprocess;

    /* ---------- 0) 如果当前就在“语言菜单页”，返回主菜单 ---------- */
    /* 说明：setlanguage() 那一页属于 KEYNUM_MENU_LANGUAGE，它不是参数分组页 */
    if (func_index == KEYNUM_MENU_LANGUAGE) {
        return mainmenu;
    }

    /* ---------- 1) 纯指令类：回到指令入口 ---------- */
    switch (now_Opera_Num) {

    /* 普通测量主菜单直属指令 */
    case COM_NUM_BACK_ZERO:
    case COM_NUM_FIND_OIL:
    case COM_NUM_FIND_BOTTOM:
    case COM_NUM_SYNTHETIC:
    case COM_NUM_READ_PART_PARAMS:
    case COM_NUM_RUN_TO_POSITION:
        return measuremenu;

    /* 水位测量子菜单 */
    case COM_NUM_FIND_WATER:
    case COM_NUM_FOLLOW_WATER:
        return menu_measure_water;

    /* 密度分布测量子菜单 */
    case COM_NUM_SPREADPOINTS:
    case COM_NUM_SPREADPOINTS_GB:
    case COM_NUM_METER_DENSITY:
    case COM_NUM_INTERVAL_DENSITY:
    case COM_NUM_WARTSILA_DENSITY:
    case COM_NUM_SI_PROFILE:
        return menu_measure_density_distribution;

    /* 密度单点测量子菜单 */
    case COM_NUM_SINGLE_POINT:
    case COM_NUM_SP_TEST:
        return menu_measure_density_single;

    /* 浮子运动控制子菜单 */
    case COM_NUM_RUNUP:
    case COM_NUM_RUNDOWN:
    case COM_NUM_FORCE_RUNUP:
    case COM_NUM_FORCE_RUNDOWN:
        return menu_debug_float_motion;

    /* 标定修正子菜单 */
    case COM_NUM_FIND_ZERO:
    case COM_NUM_CORRECTION_OIL:
    case COM_NUM_CAL_OIL:
    case COM_NUM_CALIBRATE_WATER:
    case COM_NUM_CALIBRATE_TANKHEIGHT:
        return menu_debug_calibration;

    /* 扭力标定子菜单 */
    case COM_NUM_SET_EMPTY_WEIGHT:
    case COM_NUM_SET_FULL_WEIGHT:
        return menu_debug_weight;

    /* 无线维护子菜单 */
    case COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING:
        return menu_debug_wireless;

    /* 系统维护子菜单 */
    case COM_NUM_RESTOR_EFACTORYSETTING:
    case COM_NUM_MAINTENANCE_MODE:
    case COM_NUM_MAINTENANCE_EXIT:
    case COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS:
        return menu_debug_system;

    /* 密码入口：回主菜单 */
    case COM_NUM_PASSWORD_ENTER_PARA:
    case COM_NUM_PASSWORD_ENTER_CMD:
        return mainmenu;

    default:
        break;
    }

    /* ---------- 2) 参数类：按分组映射回到对应分组菜单 ---------- */
    if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END) ||
        (now_Opera_Num > COM_NUM_PARA_LOCAL_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP))
    {
        switch (ParamGroupOf(now_Opera_Num)) {

        /* CPU2 分组 */
        case MENU_GRP_DEV_INFO:      p = menu_dev_info;     break;
        case MENU_GRP_RUN_POLICY:    p = menu_run_policy;   break;
        case MENU_GRP_MECH:          p = menu_mech;         break;
        case MENU_GRP_WEIGHT:        p = menu_weight;       break;
        case MENU_GRP_ZERO:          p = menu_zero;         break;
        case MENU_GRP_LIQUID:        p = menu_liquid;       break;
        case MENU_GRP_WATER:         p = menu_water;        break;
        case MENU_GRP_BOTTOM_TANKH:  p = menu_bottom_tankh; break;
        case MENU_GRP_CORR:          p = menu_correct;      break;
        case MENU_GRP_POLICY:        p = menu_policy;       break;
        case MENU_GRP_WARTSILA:      p = menu_wartsila;     break;
        case MENU_GRP_SI_PROFILE:
        case MENU_GRP_CPU3_SI_AUTO:
        case MENU_GRP_CPU3_SI_ALARM:
            p = menu_si_config;
            break;
        case MENU_GRP_DO_ALARM:      p = RelayParam_BackToConfigMenu(now_Opera_Num); break;
        case MENU_GRP_AO_CHANNEL:    p = menu_ao_channel;    break;
        case MENU_GRP_AO_RANGE:      p = menu_ao_range;      break;
        case MENU_GRP_AO_FAULT:      p = menu_ao_fault;      break;
        case MENU_GRP_AO_RUNTIME:    p = menu_ao_runtime;    break;
        case MENU_GRP_AO_DIAGNOSTIC: p = menu_ao_diagnostic; break;
        case MENU_GRP_AO_RESERVED:   p = menu_ao;            break;
        case MENU_GRP_CAL_SP:        p = menu_cal_sp;       break;
        case MENU_GRP_PARAM_CHECK:   p = menu_param_check;  break;

        /* CPU3 分组 */
        case MENU_GRP_CPU3_BASE:
        case MENU_GRP_CPU3_SCREEN:
            if ((now_Opera_Num == COM_NUM_PARA_LANG) || (now_Opera_Num == COM_NUM_SCREEN_PASSWARD)) {
                p = menu_display_config;
            } else {
                p = menu_display_base;
            }
            break;
        case MENU_GRP_CPU3_SOURCE:
        case MENU_GRP_CPU3_INPUT:
            switch (now_Opera_Num) {
            case COM_NUM_SCREEN_SOURCE_OIL:
            case COM_NUM_SCREEN_INPUT_OIL:
                p = menu_display_data_oil;
                break;
            case COM_NUM_SCREEN_SOURCE_WATER:
            case COM_NUM_SCREEN_INPUT_WATER:
                p = menu_display_data_water;
                break;
            case COM_NUM_SCREEN_SOURCE_D:
            case COM_NUM_SCREEN_INPUT_D:
            case COM_NUM_SCREEN_INPUT_D_SWITCH:
                p = menu_display_data_density;
                break;
            case COM_NUM_SCREEN_SOURCE_T:
            case COM_NUM_SCREEN_INPUT_T:
                p = menu_display_data_temp;
                break;
            default:
                p = menu_display_data;
                break;
            }
            break;
        case MENU_GRP_CPU3_COM1:     p = menu_cpu3_comm1;   break;
        case MENU_GRP_CPU3_COM2:     p = menu_cpu3_comm2;   break;
        case MENU_GRP_CPU3_COM3:     p = menu_cpu3_comm3;   break;

        default:
            p = menu_paracfg_main;
            break;
        }
        return p;
    }

    /* ---------- 3) 兜底：按大区间返回 ---------- */
    if (now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END) {
        return measuremenu;
    }
    if (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_ONEPARACMD_END) {
        return measuremenu;
    }
    if (now_Opera_Num > COM_NUM_ONEPARA_DEBUGCMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END) {
        return menu_cmdconfig_main;
    }
    if (now_Opera_Num > COM_NUM_PASSWORD_START && now_Opera_Num < COM_NUM_PASSWORD_END) {
        return mainmenu;
    }

    return p;
}


/**
 * @brief 返回按确认键后要跳转的函数指针。
 *
 * @return 返回当前操作按确认键时应调用的处理函数；受保护参数、无参数命令和单参数命令分别映射到对应入口。
 */
static pFunc_void dtm_suretofunc(void)
{
	if (screen_operation_is_no_para_command(now_Opera_Num)) {
		if (operation_needs_protect_confirm(now_Opera_Num)) {
			return param_protect_confirm;
		}
		return cmd_nopara_process;
	} else if (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END) {
		return cmd_onepara_process;
	} else if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP) {
		return parascopecheck;
	} else {
		printf("非法指令\r\n");
		return errorprocess;
	}
}
/* 无附加参数的菜单操作码到 CPU2 命令码映射项；用于只需下发命令本身的菜单动作。 */
typedef struct {
    /* 无参数菜单动作到 CPU2 命令码的一一映射。 */
    uint32_t opera; /* 菜单操作码，用于把当前表项与菜单元数据及命令映射关联。 */
    uint32_t cmd; /* 向 CPU2 下发的设备命令码。 */
} NoParaCmdMap_t;

/**
 * @brief 统一判断屏幕操作码是否属于无参 CPU2 命令。
 *
 * @details 调用场景：名称分类、确认页、确认键分发共用，避免显式操作码在多个区间判断中漏配。
 * @note 关键约束：1000~1006 中只有列出的命令操作码可进入下发流程，AO 只读/仿真操作不得误分类。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return true 表示操作号属于无需先写参数即可直接下发的 CPU2 命令白名单；false 表示它是参数项、带参命令、仅本机操作或未知操作号。
 */
static bool screen_operation_is_no_para_command(int operaNum)
{
	if ((operaNum > COM_NUM_NOPARACMD_START) && (operaNum < COM_NUM_NOPARACMD_END)) {
		return true;
	}

	switch (operaNum) {
	case COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING:
	case COM_NUM_MAINTENANCE_EXIT:
	case COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 向 CPU2 下发当前菜单命令，并统一处理确认、异常响应和通信失败。
 *
 * @param cmd 命令值。该 32 位 CommandType 编码将写入 CPU2 共享命令寄存器，并等待板间参数同步结果确认。
 * @return true 表示 CPU2 返回合法写响应，false 表示本次请求失败。
 */
static bool send_cpu2_command(uint32_t cmd)
{
    uint16_t command_regs[REG_STRIDE];

    if (!CPU2_CommCanSendCommand((CommandType)cmd)) {
        return false;
    }
    command_regs[0] = (uint16_t)(cmd >> 16);
    command_regs[1] = (uint16_t)(cmd & 0xFFFFU);
    return CPU2_CommWriteHoldingRegistersEx(
               HOLDREGISTER_DEVICEPARAM_COMMAND,
               REG_STRIDE,
               command_regs) == CPU2_MODBUS_RESULT_OK;
}

/**
 * @brief 统一显示 CPU2 请求未获得合法响应的菜单提示。
 *
 * @details 调用场景：CPU2 参数读取、写入或命令下发失败后调用。
 * @note 关键约束：不得继续进入成功页、运动监控页或参数确认页。
 */
static void display_cpu2_comm_failure(void)
{
    uint8_t exception_code = CPU2_CommGetLastModbusException();

    if ((exception_code != CPU2_MODBUS_RESULT_OK) && CPU2_CommIsAvailable()) {
        display_cpu2_modbus_exception(exception_code);
        return;
    }
    if (CPU2_CommIsProtocolMismatch()) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"协议版本不匹配", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Proto Mismatch");
        HAL_Delay(800);
        return;
    }
    if (CPU2_CommHasRuntimeSnapshot() && !CPU2_CommIsAvailable()) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在读取参数", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Syncing Para");
        HAL_Delay(800);
        return;
    }
    oled_clear();
    DisplayLangaugeLineWords((uint8_t*)"与CPU2通讯故障!", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Cpu2 CF!");
    HAL_Delay(800);
}

/**
 * @brief 标准Modbus异常属于CPU2业务拒绝，不计作板间物理通信故障。
 *
 * @param exception_code 异常码。
 */
static void display_cpu2_modbus_exception(uint8_t exception_code)
{
    oled_clear();
    switch (exception_code) {
    case CPU2_MODBUS_EX_ILLEGAL_FUNCTION:
        DisplayLangaugeLineWords((uint8_t*)"操作不支持!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Unsupported");
        break;
    case CPU2_MODBUS_EX_ILLEGAL_ADDRESS:
        DisplayLangaugeLineWords((uint8_t*)"非法参数!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
        break;
    case CPU2_MODBUS_EX_ILLEGAL_VALUE:
        DisplayLangaugeLineWords((uint8_t*)"数值超范围!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Value out of Range");
        break;
    case CPU2_MODBUS_EX_SLAVE_DEVICE_BUSY:
        DisplayLangaugeLineWords((uint8_t*)"CPU2设备忙!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"CPU2 Busy");
        break;
    case CPU2_MODBUS_EX_SLAVE_DEVICE_FAILURE:
    default:
        DisplayLangaugeLineWords((uint8_t*)"CPU2操作失败!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"CPU2 Failed");
        break;
    }
    HAL_Delay(800);
}

/**
 * @brief 判断命令是否属于本需求定义的纯电机运行监控范围。
 *
 * 只有用户直接下发的纯电机指令进入监控页，普通测量流程即使内部驱动电机也不进入。
 *
 * @param cmd 待分类的 32 位 CommandType 编码；函数只判断该命令是否需要进入电机运行监控页。
 * @return true 表示命令属于本需求定义的纯电机运行监控范围；false 表示命令不属于本需求定义的纯电机运行监控范围。
 */
static bool command_is_motor_monitor_command(uint32_t cmd)
{
	switch (cmd) {
	case CMD_RUN_TO_POSITION:
	case CMD_MOVE_UP:
	case CMD_MOVE_DOWN:
	case CMD_FORCE_MOVE_UP:
	case CMD_FORCE_MOVE_DOWN:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 判断 CPU2 当前状态是否仍属于电机监控运行态。
 *
 * @param state CPU2 当前设备状态；纯电机命令对应的上行、下行或停止过渡态均视为监控仍活动。
 * @return true 表示 CPU2 当前状态仍属于电机监控运行态；false 表示 CPU2 当前状态已不再属于电机监控运行态。
 */
static bool motor_run_monitor_state_is_active(DeviceState state)
{
	switch (state) {
	case STATE_RUNUPING:
	case STATE_RUNDOWNING:
	case STATE_RUN_TO_POSITIONING:
	case STATE_FORCE_RUNUPING:
	case STATE_FORCE_RUNDOWNING:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 判断 CPU2 当前状态是否已从纯电机指令收敛到完成态。
 *
 * @param state CPU2 当前设备状态；用于判断纯电机命令是否已收敛到可退出监控页的完成态。
 * @return true 表示 CPU2 当前状态已从纯电机指令收敛到完成态；false 表示 CPU2 当前状态尚未从纯电机指令收敛到完成态。
 */
static bool motor_run_monitor_state_is_done(DeviceState state)
{
	switch (state) {
	case STATE_RUNUPOVER:
	case STATE_RUNDOWNOVER:
	case STATE_RUN_TO_POSITION_OVER:
	case STATE_FORCE_RUNUP_OVER:
	case STATE_FORCE_RUNDOWN_OVER:
	case STATE_STANDBY:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 纯电机指令下发后进入运行监控页，其它业务命令保持原有退出策略。
 *
 * @param cmd 刚刚获得 CPU2 发送确认的 32 位 CommandType 编码；用于决定监控页的目标命令和初始状态。
 */
static void motor_run_monitor_handle_sent_command(uint32_t cmd)
{
	if (command_is_motor_monitor_command(cmd)) {
		enter_motor_run_monitor_page();
	} else {
		exitTankOpera();
	}
}

/**
 * @brief 判断指定设备状态是否属于允许用户取消的测量过程。
 *
 * @param state 准备由用户取消的 CPU2 测量状态；仅正在执行且具备安全取消出口的状态被放行。
 * @return 1 表示指定设备状态属于允许用户取消的测量过程；0 表示指定设备状态不属于允许用户取消的测量过程。
 */
static uint8_t display_state_can_cancel_measurement(DeviceState state)
{
    if ((state == STATE_STANDBY) ||
        (state == STATE_ERROR) ||
        (state == STATE_MAINTENANCEMODE)) {
        return 0;
    }

    if ((g_measurement.device_status.current_command != CMD_NONE) ||
        ((state >= STATE_BACKZEROING) && (state <= STATE_CALIBRATE_TANKHEIGHTING)) ||
        (state == STATE_FLOWOIL) ||
        (state == STATE_FOLLOW_WATERING)) {
        return 1;
    }

    return 0;
}

/**
 * @brief 判断当前页面和设备状态是否允许进入取消测量确认页。
 * @return true 表示当前页面和设备状态允许进入取消测量确认页；false 表示当前页面和设备状态不允许进入取消测量确认页。
 */
bool Display_CanEnterCancelMeasurementConfirm(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    /* 正式错误态即使已经没有可取消的测量流程，也允许进入确认入口，以便用户查看故障原因。 */
    if ((state == STATE_ERROR) && (g_measurement.device_status.error_code != NO_ERROR)) {
        return true;
    }

    return display_state_can_cancel_measurement(state) != 0U;
}

/**
 * @brief 向 CPU2 提交取消当前测量的命令。
 * @return true 表示无需取消或取消命令获得合法响应，false 表示通信失败。
 */
bool Display_RequestCancelMeasurement(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    /* 确认期间流程可能已自然结束，此时按幂等成功处理，避免误报通信故障。 */
    if (!display_state_can_cancel_measurement(state)) {
        return true;
    }

    if (!CPU2_CommCanSendCommand(CMD_CANCEL_MEASUREMENT)) {
        return false;
    }
    return send_cpu2_command(CMD_CANCEL_MEASUREMENT);
}

/**
 * @brief 进入取消测量确认页并保存返回位置。
 * @return true 表示已进入取消测量确认页，或当前故障已转入故障原因页；false 表示当前设备状态不属于可取消的测量流程，页面未进入确认态。
 */
bool Display_EnterCancelMeasurementConfirm(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    /* 错误态下取消入口改为直接打开故障原因页，不再向 CPU2 重复发送取消测量命令。 */
    if ((state == STATE_ERROR) && (g_measurement.device_status.error_code != NO_ERROR)) {
        FlagofTankOpera = true;
        useKey();
        keymenu[KEYNUM_ERROR_REASON].execute_opera();
        return true;
    }

    if (!display_state_can_cancel_measurement(state)) {
        FlagofTankOpera = false;
        return false;
    }

    FlagofTankOpera = true;
    useKey();
    keymenu[KEYNUM_IF_CANCEL_MEASUREMENT].execute_opera();
    return true;
}

/**
 * @brief 将当前无参菜单操作映射为 CPU2 命令，校验通信状态后下发并更新界面。
 *
 * 静态映射表统一覆盖普通测量、密度 Profile、部件读取、标定、扭力采集、恢复出厂、维护、清锁存和无线配对等不携带独立参数的操作号。
 * 未找到操作号映射，或 CPU2 当前协议、通信状态不允许发送该命令时，函数提示失败并退出罐上操作，禁止下发 CMD_UNKNOWN。
 * SI Profile 通过 si_profile_request_start 建立专用生命周期，其他命令通过统一 CPU2
 * 命令接口发送；请求成功后按命令类型进入运行监视、调试扭力等待页或专用完成提示。
 * 无线配对命令发送后退出普通罐上操作并停止页面定时器，使后续状态页能够展示配对过程。
 */
static void cmd_nopara_process(void)
{
    static const NoParaCmdMap_t map[] = {
        /* -------- 普通模式：无参测量类 -------- */
        { COM_NUM_BACK_ZERO,          CMD_BACK_ZERO },
        { COM_NUM_FIND_OIL,           CMD_FIND_OIL },
        { COM_NUM_FIND_WATER,         CMD_FIND_WATER },
        { COM_NUM_FIND_BOTTOM,        CMD_FIND_BOTTOM },
        { COM_NUM_SYNTHETIC,          CMD_SYNTHETIC },

        { COM_NUM_FOLLOW_WATER,       CMD_FOLLOW_WATER },
        { COM_NUM_SPREADPOINTS,       CMD_MEASURE_DISTRIBUTED },
        { COM_NUM_SPREADPOINTS_GB,    CMD_GB_MEASURE_DISTRIBUTED },

        { COM_NUM_METER_DENSITY,      CMD_MEASURE_DENSITY_METER },
        { COM_NUM_INTERVAL_DENSITY,   CMD_MEASURE_DENSITY_RANGE },
        { COM_NUM_WARTSILA_DENSITY,   CMD_WARTSILA_DENSITY_RANGE },
        { COM_NUM_SI_PROFILE,         CMD_SI_PROFILE },

        { COM_NUM_READ_PART_PARAMS,   CMD_READ_PART_PARAMS },

        /* -------- 调试模式：无参指令 -------- */
        { COM_NUM_FIND_ZERO,          CMD_CALIBRATE_ZERO },

        { COM_NUM_SET_EMPTY_WEIGHT,   CMD_SET_EMPTY_WEIGHT },
        { COM_NUM_SET_FULL_WEIGHT,    CMD_SET_FULL_WEIGHT },
        { COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },
        { COM_NUM_MAINTENANCE_MODE,   CMD_MAINTENANCE_MODE },
        { COM_NUM_MAINTENANCE_EXIT,   CMD_MAINTENANCE_EXIT },
        { COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS, CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS },
        { COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING, CMD_PAIR_NEAREST_WIRELESS_SLIPRING },
    };

    uint32_t cmd = CMD_UNKNOWN;
    int found = 0;

    for (int i = 0; i < (int)(sizeof(map) / sizeof(map[0])); i++) {
        if (now_Opera_Num == (int)map[i].opera) {
            cmd = map[i].cmd;
            found = 1;
            break;
        }
    }

    /* 未找到映射：直接退出，避免下发错误命令 */
    if (!found) {
        exitTankOpera();
        return;
    }

    if (!CPU2_CommCanSendCommand((CommandType)cmd)) {
        display_cpu2_comm_failure();
        exitTankOpera();
        return;
    }

    /* 下发命令 */
    bool request_ok;
    if (now_Opera_Num == COM_NUM_SI_PROFILE) {
        request_ok = si_profile_request_start();
    } else {
        request_ok = send_cpu2_command(cmd);
    }
    if (!request_ok) {
        display_cpu2_comm_failure();
        exitTankOpera();
        return;
    }

    /* ---------- UI 反馈与退出策略（保留你现有行为） ---------- */
    if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在恢复出厂设置", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Factory Settings");
        HAL_Delay(800);
        exitTankOpera();
    } else if (now_Opera_Num == COM_NUM_MAINTENANCE_MODE) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"进入指令已发送", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Enter MNT Sent");
        HAL_Delay(800);
        exitTankOpera();
    } else if (now_Opera_Num == COM_NUM_MAINTENANCE_EXIT) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"退出指令已发送", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Exit MNT Sent");
        HAL_Delay(800);
        exitTankOpera();
    } else if (now_Opera_Num == COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"清锁存请求已发送", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Clear Request Sent");
        HAL_Delay(800);
        exitTankOpera();
    } else if (now_Opera_Num == COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING) {
        FlagofTankOpera = false;
        HAL_TIM_Base_Stop_IT(&htim1);
        ClearPageNum();
    } else if ((now_Opera_Num == COM_NUM_SET_EMPTY_WEIGHT) ||
               (now_Opera_Num == COM_NUM_SET_FULL_WEIGHT)) {
        enter_debug_weight_wait_page(now_Opera_Num);
    } else {
        motor_run_monitor_handle_sent_command(cmd);
    }
}

/**
 * @brief 先写入当前带参命令的 32 位参数，再映射并下发对应 CPU2 命令。
 *
 * 静态映射表覆盖单点测量、单点监测、运行到位置、液位或水位或罐高标定、手动运动和强制运动等携带一个 32 位参数的操作。
 * 函数先用当前操作号定位参数元数据，要求对应字段恰好占两个保持寄存器，再把 now_Para_CT.val 的有符号 32 位原始值按高字在前拆分并写入 CPU2。
 * 参数写入成功后才查找并下发对应 CommandType；任一步发生元数据、寄存器长度、板间写入、命令映射或命令发送错误时，都显示明确提示并退出当前操作。
 * 命令发送成功后进入统一电机运行监视；当前实现先写参数再确认命令映射，因此新增带参命令时必须同步维护参数元数据和 onepara_cmd_map。
 */
static void cmd_onepara_process(void)
{
    int index;
    int i;

    /* 带一个数值参数的菜单操作码到 CPU2 命令码映射项；发送前还需由调用方写入对应命令参数槽。 */
    typedef struct {
        /* 单参数菜单动作到 CPU2 命令码的一一映射。 */
        int opera; /* 菜单操作码，用于把当前表项与菜单元数据及命令映射关联。 */
        uint32_t cmd; /* 向 CPU2 下发的设备命令码。 */
    } OneParaCmdMap_t;

    /* 新版：带参指令映射 */
    static const OneParaCmdMap_t onepara_cmd_map[] = {
        /* 工作模式：带参 */
        { COM_NUM_SINGLE_POINT,      CMD_MEASURE_SINGLE },
        { COM_NUM_SP_TEST,           CMD_MONITOR_SINGLE },
        { COM_NUM_RUN_TO_POSITION,   CMD_RUN_TO_POSITION },   /* 新增：运行到指定位置 */

        /* 调试模式：带参 */
        { COM_NUM_CAL_OIL,           CMD_CALIBRATE_OIL },
        { COM_NUM_CORRECTION_OIL,    CMD_CORRECT_OIL },

        { COM_NUM_CALIBRATE_WATER,   CMD_CALIBRATE_WATER },   /* 新增：水位标定 */
        { COM_NUM_CALIBRATE_TANKHEIGHT, CMD_CALIBRATE_TANKHEIGHT }, /* 新增：罐高标定 */

        { COM_NUM_RUNUP,             CMD_MOVE_UP },
        { COM_NUM_RUNDOWN,           CMD_MOVE_DOWN },

        { COM_NUM_FORCE_RUNUP,       CMD_FORCE_MOVE_UP },     /* 新增：强制上行 */
        { COM_NUM_FORCE_RUNDOWN,     CMD_FORCE_MOVE_DOWN },   /* 新增：强制下行 */
    };

    const int mapamount = (int)(sizeof(onepara_cmd_map) / sizeof(onepara_cmd_map[0]));

    /* 1) 找到该操作对应的“参数寄存器元数据” */
    index = getHoldValueNum(now_Opera_Num);
    if (index < 0) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"非法参数!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
        HAL_Delay(800);
        exitTankOpera();
        return;
    }

    /* 2) 七个带参命令字段都是一个 32 位值，按共享 Modbus 高字在前组织。 */
    {
        uint32_t raw_value;
        uint16_t parameter_regs[REG_STRIDE];
        uint8_t write_result;

        if (param_meta[index].rgstcnt != REG_STRIDE) {
            oled_clear();
            DisplayLangaugeLineWords((uint8_t*)"参数长度异常!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Bad Para Len");
            HAL_Delay(800);
            exitTankOpera();
            return;
        }

        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在下发参数", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Send Para");

        raw_value = (uint32_t)((int32_t)now_Para_CT.val);
        parameter_regs[0] = (uint16_t)(raw_value >> 16);
        parameter_regs[1] = (uint16_t)(raw_value & 0xFFFFU);
        write_result = CPU2_CommWriteHoldingRegistersEx(
            param_meta[index].startadd,
            param_meta[index].rgstcnt,
            parameter_regs);
        if (write_result != CPU2_MODBUS_RESULT_OK) {
            display_cpu2_comm_failure();
            exitTankOpera();
            return;
        }
    }

    /* 3) 下发命令 */
    oled_clear();
    DisplayLangaugeLineWords((uint8_t*)"正在下发指令", OLED_LINE8_2, OLED_ROW3_3, 0, (uint8_t*)"Send Command");

    {
        int found = 0;
        uint32_t cmd = CMD_NONE;

        for (i = 0; i < mapamount; i++) {
            if (now_Opera_Num == onepara_cmd_map[i].opera) {
                cmd = onepara_cmd_map[i].cmd;
                found = 1;
                break;
            }
        }

        if (!found) {
            /* 参数写了，但没有对应命令：给出明确提示 */
            oled_clear();
            DisplayLangaugeLineWords((uint8_t*)"指令未定义!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Cmd Undefined");
            HAL_Delay(800);
            exitTankOpera();
            return;
        }

        if (!send_cpu2_command(cmd)) {
            display_cpu2_comm_failure();
            exitTankOpera();
            return;
        }
        motor_run_monitor_handle_sent_command(cmd);
        return;
    }
}
/* */
/* / * 不带参线圈指令处理过程 * / */
/* static void cmd_nopara_process(void) */
/* { */
/* static const uint32_t nopara_cmd_map[][2] = { */
/* { COM_NUM_BACK_ZERO, CMD_BACK_ZERO }, */
/* { COM_NUM_FIND_ZERO, CMD_CALIBRATE_ZERO }, */
/* { COM_NUM_SPREADPOINTS, CMD_MEASURE_DISTRIBUTED }, */
/* { COM_NUM_FIND_OIL, CMD_FIND_OIL }, */
/* { COM_NUM_FIND_WATER, CMD_FIND_WATER }, */
/* { COM_NUM_FIND_BOTTOM, CMD_FIND_BOTTOM }, */
/* { COM_NUM_SYNTHETIC, CMD_SYNTHETIC }, */
/* { COM_NUM_METER_DENSITY, CMD_MEASURE_DENSITY_METER }, */
/* { COM_NUM_INTERVAL_DENSITY, CMD_MEASURE_DENSITY_RANGE }, */
/* { COM_NUM_WARTSILA_DENSITY, CMD_WARTSILA_DENSITY_RANGE }, */
/* */
/* { COM_NUM_SET_EMPTY_WEIGHT, CMD_SET_EMPTY_WEIGHT }, */
/* { COM_NUM_SET_FULL_WEIGHT, CMD_SET_FULL_WEIGHT }, */
/* { COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY }, */
/* { COM_NUM_MAINTENANCE_MODE, CMD_MAINTENANCE_MODE }, */
/* }; */
/* */
/* int mapamount = (int)(sizeof(nopara_cmd_map) / sizeof(nopara_cmd_map[0])); */
/* int i; */
/* */
/* for (i = 0; i < mapamount; i++) { */
/* if (now_Opera_Num == (int)nopara_cmd_map[i][0]) { */
/* CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER, */
/* HOLDREGISTER_DEVICEPARAM_COMMAND, 2, */
/* (uint32_t*)&nopara_cmd_map[i][1]); */
/* break; */
/* } */
/* } */
/* */
/* if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) { */
/* oled_clear(); */
/* if ((g_measurement.device_status.device_state != STATE_STANDBY) */
/* && (g_measurement.device_status.device_state != STATE_ERROR) */
/* && (g_measurement.device_status.device_state != STATE_MAINTENANCEMODE)) { */
/* DisplayLangaugeLineWords((uint8_t*)"失败", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Failed to set"); */
/* DisplayLangaugeLineWords((uint8_t*)"请先进入调试模式", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Enter debug mode"); */
/* } else { */
/* DisplayLangaugeLineWords((uint8_t*)"正在恢复出厂设置", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Factory Settings"); */
/* HAL_Delay(800); */
/* exitTankOpera(); */
/* } */
/* } else if (now_Opera_Num == COM_NUM_MAINTENANCE_MODE) { */
/* oled_clear(); */
/* DisplayLangaugeLineWords((uint8_t*)"已进入维护模式", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Maintenance Mode"); */
/* } else { */
/* exitTankOpera(); */
/* } */
/* } */

/**
 * @brief 显示非法操作提示，保持 1 秒后退出罐上屏幕操作。
 */
static void errorprocess(void)
{
	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"非法操作!", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Illegal operation");
	DisplayLangaugeLineWords((uint8_t*)"1s后退出屏幕操作", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Exit after 1 second");
	HAL_Delay(1000);
	exitTankOpera();
}

/**
 * @brief 将0xMMmmppbb版本编码显示为V主.次.修订.构建。
 *
 * @param version 版本。
 * @param buf 用于接收 V主.次.修订.构建 格式版本文字的目标缓冲区。
 * @param buf_size 缓冲区容量，单位字节。
 */
static void format_version_u32(uint32_t version, char *buf, size_t buf_size)
{
	snprintf(buf, buf_size, "V%lu.%lu.%lu.%lu",
			(unsigned long)((version >> 24) & 0xFFU),
			(unsigned long)((version >> 16) & 0xFFU),
			(unsigned long)((version >> 8) & 0xFFU),
			(unsigned long)(version & 0xFFU));
}

/**
 * @brief 绘制当前操作参数的名称、现值、允许范围和修改入口，并处理确认或返回按键。
 *
 * CPU3 本机版本、版本号和十六进制参数采用只读格式；电机电流和普通数值分别使用专用详情或元数据范围显示。
 * 确认键连续触发后进入写权限检查，返回键按当前菜单回退函数恢复上一页；模拟量输出只读项只提供返回入口。
 */
static void displaypara(void)
{
	int index;

	oled_clear();
	func_index = KEYNUM_DISPLAY_PARA;

	OledDisplayLineWords(oled_fit_text(dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num)), OLED_LINE8_END),
	                     OLED_LINE8_1,
	                     OLED_ROW4_1,
	                     0);

	/* 本机参数 */
	if (now_Opera_Num == COM_NUM_PARA_LOCAL_LEDVERSION) {
		DisplayLangaugeLineWords((uint8_t*)"当前值:", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Value:");
		display_formatted_readonly_value(now_Opera_Num,
		                                 Cpu3Local_ReadValue((OperatingNumber)now_Opera_Num),
		                                 OLED_LINE8_4,
		                                 OLED_ROW4_2,
		                                 0);
		DisplayLangaugeLineWords((uint8_t*)"范围:--", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Range:--");
	} else {
		index = getHoldValueNum(now_Opera_Num);
		if (index == -1) {
			DisplayLangaugeLineWords((uint8_t*)"当前值:非法", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Value:Illegal");
			DisplayLangaugeLineWords((uint8_t*)"范围:--", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Range:--");
		} else if ((is_version_value_opera(now_Opera_Num) != 0) || (is_hex_u32_value_opera(now_Opera_Num) != 0)) {
			DisplayLangaugeLineWords((uint8_t*)"当前值:", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Value:");
			display_formatted_readonly_value(now_Opera_Num,
			                                 param_meta[index].val,
			                                 OLED_LINE8_4,
			                                 OLED_ROW4_2,
			                                 0);
			DisplayLangaugeLineWords((uint8_t*)"范围:--", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Range:--");
		} else if (now_Opera_Num == COM_NUM_DEVICEPARAM_MOTOR_CURRENT) {
			display_motor_current_detail((uint32_t)param_meta[index].val, OLED_ROW4_2);
		} else {
			display_param_detail_value(&param_meta[index], OLED_ROW4_2);
			display_param_detail_range(&param_meta[index], OLED_ROW4_3);
		}
	}

	if (NowKeyPress == USE_KEY_SURE) {
		if (timesure > 1) {
			timesure = 0;
			timeback = 0;
			parawritecheck();
			return;
		}
		timesure++;
		if (timeback > 0) {
			timeback = 0;
		}
	} else if (NowKeyPress == USE_KEY_BACK) {
		if (timeback != 0) {
			timeback = 0;
			timesure = 1;
			dtm_backtofunc()();
			return;
		}
		timeback++;
		timesure--;
	}

	if (ao_param_is_config(now_Opera_Num) && !ao_param_is_editable(now_Opera_Num)) {
		DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 1, (uint8_t*)"Back");
		display_right_aligned_action((uint8_t*)"只读", (uint8_t*)"Readonly", OLED_ROW4_4, 1);
	} else {
		DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 1, (uint8_t*)"Back");
		display_right_aligned_action((uint8_t*)"修改", (uint8_t*)"Alter", OLED_ROW4_4, 1);
	}
}

/**
 * @brief 判断当前设备状态是否允许修改CPU2持久参数。
 *
 * @param state CPU2 当前设备状态；只有共享持久参数写白名单内的稳定状态允许继续编辑并下发。
 * @return true 表示当前设备状态允许修改CPU2持久参数；false 表示当前设备状态不允许修改CPU2持久参数。
 */
static bool state_allows_param_write(DeviceState state)
{
	return DeviceState_AllowsPersistentParamWrite(state);
}

/**
 * @brief AO仅在普通电流输出或HART从站加输出模式下具备输出能力。
 *
 * @return true 表示 AO 工作模式为普通电流输出或 HART 从站加电流输出；false 表示当前模式不驱动过程量电流输出。
 */
static bool ao_work_mode_is_output(void)
{
	return (g_deviceParams.ao_output.work_mode == AO_WORK_MODE_CURRENT_OUTPUT) ||
	       (g_deviceParams.ao_output.work_mode == AO_WORK_MODE_HART_SLAVE_OUTPUT);
}

/**
 * @brief 判断操作码是否属于协议26沿用原地址的13项AO持久化配置。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return true 表示操作码属于协议26沿用原地址的13项AO持久化配置；false 表示操作码不属于协议26沿用原地址的13项AO持久化配置。
 */
static bool ao_param_is_config(int operaNum)
{
	switch (operaNum) {
	case COM_NUM_DEVICEPARAM_AO_WORK_MODE:
	case COM_NUM_DEVICEPARAM_AO_CURRENT_MODE:
	case COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE:
	case COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000:
	case COM_NUM_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100:
	case COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM:
	case COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM:
	case COM_NUM_DEVICEPARAM_AO_DAMPING_X10_S:
	case COM_NUM_DEVICEPARAM_AO_FAULT_MODE:
	case COM_NUM_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100:
	case COM_NUM_DEVICEPARAM_AO_ERROR_LEVEL:
	case COM_NUM_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100:
	case COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 持久化AO参数允许预配置，当前模式不使用时仅暂不参与输出计算。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return true 表示操作号是 12 个允许预配置的 AO 持久化参数之一；false 表示该操作号不是 AO 配置项，不能走 AO 参数编辑入口。
 */
static bool ao_param_is_editable(int operaNum)
{
	switch (operaNum) {
	case COM_NUM_DEVICEPARAM_AO_WORK_MODE:
	case COM_NUM_DEVICEPARAM_AO_CURRENT_MODE:
	case COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE:
	case COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000:
	case COM_NUM_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100:
	case COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM:
	case COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM:
	case COM_NUM_DEVICEPARAM_AO_DAMPING_X10_S:
	case COM_NUM_DEVICEPARAM_AO_FAULT_MODE:
	case COM_NUM_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100:
	case COM_NUM_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100:
	case COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 量程上限跟随真实输出源；水位罐高未配置时回退液位罐高。
 *
 * @return 返回 AO 当前过程量来源对应的量程上限，单位 0.1 mm；水位罐高未配置时回退液位罐高。
 */
static int32_t ao_range_max_01mm(void)
{
	uint32_t maximum;

	if (g_deviceParams.ao_output.output_source == AO_PROCESS_SOURCE_WATER_LEVEL) {
		maximum = g_deviceParams.water_tank_height;
		if (maximum == 0U) {
			maximum = g_deviceParams.tankHeight;
		}
	} else {
		maximum = g_deviceParams.tankHeight;
	}

	if (maximum == 0U) {
		maximum = 1U;
	}
	if (maximum > 2147483647U) {
		maximum = 2147483647U;
	}
	return (int32_t)maximum;
}

/**
 * @brief 0%和100%值必须在同一次FC10事务内写入，避免CPU2观察到半更新配置。
 *
 * @return true 表示 0%/100% 量程已成对写入 CPU2、补读确认且通信仍可用；false 表示参数元数据缺失、CPU2 不可用、FC10 写入或补读失败，函数已恢复本地旧量程并请求后续刷新。
 */
static bool ao_write_range_pair(void)
{
	int index_0 = getHoldValueNum(COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM);
	int index_100 = getHoldValueNum(COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM);
	int32_t old_0;
	int32_t old_100;
	int32_t candidate_0;
	int32_t candidate_100;
	uint32_t pair[2];
	bool write_ok;
	bool success;

	if ((index_0 < 0) || (index_100 < 0)) {
		return false;
	}

	old_0 = (int32_t)param_meta[index_0].val;
	old_100 = (int32_t)param_meta[index_100].val;
	candidate_0 = old_0;
	candidate_100 = old_100;
	if (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM) {
		candidate_0 = (int32_t)now_Para_CT.val;
	} else {
		candidate_100 = (int32_t)now_Para_CT.val;
	}

	pair[0] = (uint32_t)candidate_0;
	pair[1] = (uint32_t)candidate_100;
	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"正在修改量程", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Modify Range");

	write_ok = CPU2_CommIsAvailable() &&
	           CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
	                                       HOLDREGISTER_DEVICEPARAM_AO_RANGE_0_01MM,
	                                       AO_RANGE_PAIR_REGISTER_COUNT,
	                                       pair);
	success = write_ok &&
	          CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
	                                      HOLDREGISTER_DEVICEPARAM_AO_RANGE_0_01MM,
	                                      AO_RANGE_PAIR_REGISTER_COUNT,
	                                      NULL) &&
	          CPU2_CommIsAvailable();
	if (!success) {
		param_meta[index_0].val = old_0;
		param_meta[index_100].val = old_100;
		g_deviceParams.ao_output.range_0_01mm = old_0;
		g_deviceParams.ao_output.range_100_01mm = old_100;
		if (write_ok || (CPU2_CommGetLastModbusException() == CPU2_MODBUS_RESULT_OK)) {
			CPU2_CommRequestParameterRefresh();
		}
		display_cpu2_comm_failure();
		mainmenu();
		return false;
	}

	HAL_Delay(300);
	displaypara();
	return true;
}

/**
 * @brief 输出源写入后补读13项AO配置，接收CPU2按新源生成的默认量程。
 *
 * @return true 表示输出源已写入，并已补读完整 AO 配置以接收 CPU2 生成的新默认量程；false 表示元数据缺失、CPU2 不可用、写入或补读失败，函数已恢复旧输出源和旧量程。
 */
static bool ao_write_output_source(void)
{
	int index_source = getHoldValueNum(COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE);
	int index_0 = getHoldValueNum(COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM);
	int index_100 = getHoldValueNum(COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM);
	uint32_t old_source;
	int32_t old_0;
	int32_t old_100;
	uint32_t source_value;
	bool write_ok;
	bool success;

	if ((index_source < 0) || (index_0 < 0) || (index_100 < 0)) {
		return false;
	}

	old_source = g_deviceParams.ao_output.output_source;
	old_0 = g_deviceParams.ao_output.range_0_01mm;
	old_100 = g_deviceParams.ao_output.range_100_01mm;
	source_value = (uint32_t)now_Para_CT.val;
	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"正在切换输出源", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Change Source");

	write_ok = CPU2_CommIsAvailable() &&
	           CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
	                                       HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_SOURCE,
	                                       REG_STRIDE,
	                                       &source_value);
	success = write_ok &&
	          CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
	                                      HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE,
	                                      AO_CONFIG_REGISTER_COUNT,
	                                      NULL) &&
	          CPU2_CommIsAvailable();
	if (!success) {
		param_meta[index_source].val = (int)old_source;
		param_meta[index_0].val = old_0;
		param_meta[index_100].val = old_100;
		g_deviceParams.ao_output.output_source = old_source;
		g_deviceParams.ao_output.range_0_01mm = old_0;
		g_deviceParams.ao_output.range_100_01mm = old_100;
		if (write_ok || (CPU2_CommGetLastModbusException() == CPU2_MODBUS_RESULT_OK)) {
			CPU2_CommRequestParameterRefresh();
		}
		display_cpu2_comm_failure();
		mainmenu();
		return false;
	}

	HAL_Delay(300);
	displaypara();
	return true;
}

/**
 * @brief 仿真开关使用独立保持寄存器，不写入DeviceParameters或CPU3 FRAM。
 *
 * @param enabled 目标使能状态，非零表示启用，零表示禁用。
 * @return true 表示归一化后的仿真开关已写入并从 CPU2 补读确认；false 表示CPU2 不可用、写入失败或补读失败，函数已恢复旧运行态并按需请求刷新。
 */
static bool ao_write_simulation_enable(uint32_t enabled)
{
	uint32_t old_enabled = (g_measurement.ao_output_runtime.simulation_enabled == 0U) ? 0U : 1U;
	uint32_t normalized = (enabled == 0U) ? 0U : 1U;
	bool write_ok;
	bool success;

	write_ok = CPU2_CommIsAvailable() &&
	           CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
	                                       HOLDREGISTER_AO_SIMULATION_ENABLE,
	                                       REG_STRIDE,
	                                       &normalized);
	success = write_ok &&
	          CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
	                                      HOLDREGISTER_AO_SIMULATION_ENABLE,
	                                      REG_STRIDE,
	                                      NULL) &&
	          CPU2_CommIsAvailable();
	if (!success) {
		g_measurement.ao_output_runtime.simulation_enabled = old_enabled;
		if (write_ok || (CPU2_CommGetLastModbusException() == CPU2_MODBUS_RESULT_OK)) {
			CPU2_CommRequestParameterRefresh();
		}
		return false;
	}
	return true;
}

/**
 * @brief 判断当前屏幕参数项是否属于七个命令前置参数。
 *
 * @param index 零基数组或菜单索引。
 * @return true 表示当前屏幕参数项属于七个命令前置参数；false 表示当前屏幕参数项不属于七个命令前置参数。
 */
static bool screen_param_is_command_argument(int index)
{
	if (index < 0) {
		return false;
	}
	return LtdModbus_HoldingWriteIsCommandArgumentOnly(
		param_meta[index].startadd,
		param_meta[index].rgstcnt);
}

/**
 * @brief 校验当前参数的写权限与设备状态，并进入数值输入、枚举选择或专用设置页。
 *
 * @note CPU3 本机参数和仅供命令使用的参数不受 CPU2 运行状态限制；普通 CPU2 持久参数只有在当前设备状态允许时才能修改。
 */
static void parawritecheck(void)
{
	int index;
	bool allow_write = false;

	index = getHoldValueNum(now_Opera_Num);
	if (index != -1 && param_meta[index].authority_write &&
	    (!ao_param_is_config(now_Opera_Num) || ao_param_is_editable(now_Opera_Num))) {
		/* CPU3 本机参数只写本地 FRAM，不受 CPU2 测量状态限制。 */
		if (Cpu3Local_IsParam((OperatingNumber)now_Opera_Num)) {
			allow_write = true;
		} else if (screen_param_is_command_argument(index)) {
			allow_write = true;
		} else {
			allow_write = state_allows_param_write(g_measurement.device_status.device_state);
		}
	}

	if (allow_write) {

		if (now_Opera_Num == COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM) {
			tape_thickness_select();
			return;
		}

		if (now_Opera_Num == COM_NUM_DEVICEPARAM_MOTOR_CURRENT) {
			selectparaword();
			return;
		}

		if (param_meta[index].pword == NULL) {
			inputcmdpara();
		} else {
			selectparaword();
		}
	} else {
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"无修改权限!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"No permission");
		HAL_Delay(800);
		displaypara();
	}
}

/**
 * @brief 绘制进入罐上操作的中英文确认页，并登记对应按键处理状态。
 */
static void ifentermainmenu(void)
{
	oled_clear();
	func_index = KEYNUM_IF_ENTER_MAINMENU;
	DisplayLangaugeLineWords((uint8_t*)"是否进入罐上操作?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Enter operation?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, 0);
}

/**
 * @brief 绘制退出罐上操作的中英文确认页，并登记对应按键处理状态。
 */
static void ifexittankopera(void)
{
	oled_clear();
	func_index = KEYNUM_IF_EXIT_MAINMENU;
	DisplayLangaugeLineWords((uint8_t*)"是否退出罐上操作?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Exit operation?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, 0);
}

/**
 * @brief 绘制停止当前测量的中英文确认页，并登记对应按键处理状态。
 */
static void ifcancelmeasurement(void)
{
	oled_clear();
	func_index = KEYNUM_IF_CANCEL_MEASUREMENT;
	DisplayLangaugeLineWords((uint8_t*)"是否停止测量?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Cancel measure?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, 0);
}

/**
 * @brief 取消测量确认页返回处理。
 */
static void cancel_confirm_back(void)
{
	exitTankOpera();
}

/**
 * @brief 确认取消测量：确认键触发后直接下发CPU2取消测量命令。
 */
static void confirm_cancel_measurement(void)
{
	if (!Display_RequestCancelMeasurement()) {
		display_cpu2_comm_failure();
		ifcancelmeasurement();
		return;
	}
	exitTankOpera();
}

/**
 * @brief 进入参数配置前的密码输入操作页。
 */
static void password_enter_para(void)
{
	now_Opera_Num = COM_NUM_PASSWORD_ENTER_PARA;
	inputcmdpara();
}

/**
 * @brief 进入调试指令前的密码输入操作页。
 */
static void password_enter_cmd(void)
{
	now_Opera_Num = COM_NUM_PASSWORD_ENTER_CMD;
	inputcmdpara();
}

/**
 * @brief 发送指令获取对应参数的数据。
 *
 * @return 0 表示参数读取命令已成功下发并取得有效数据；-1 表示通信、响应或参数映射失败。
 */
static int get_para_data(void)
{
	int index;
	index = getHoldValueNum(now_Opera_Num);
	  /* CPU3 本机参数：不走 CPU2 通讯，直接刷新 val */
	if (Cpu3Local_IsParam((OperatingNumber)now_Opera_Num)) {
		param_meta[index].val = Cpu3Local_ReadValue((OperatingNumber)now_Opera_Num);
		return 0;
	}
	else  /* CPU2 参数 */
	{
		/* 首次状态快照前或本机通信故障尚未由状态帧恢复时，禁止读取 CPU2 参数。 */
		if (!CPU2_CommIsAvailable()) {
			display_cpu2_comm_failure();
			return -1;
		}
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"正在读取参数", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Reading Para");

		if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
				param_meta[index].startadd,
				param_meta[index].rgstcnt,
				NULL) || !CPU2_CommIsAvailable()) {
			display_cpu2_comm_failure();
			return -1;
		}
		HAL_Delay(150);

		return 0;
	}
}

/**
 * @brief 读取当前操作号对应的参数元数据；有效时进入参数详情页，无效时返回主菜单。
 */
static void para_mainprocess(void)
{
	if (get_para_data() == 0) {
		displaypara();
	} else {
		mainmenu();
	}
}

/**
 * @brief 校验待写参数的通用范围和模拟量输出双端点约束，再转入保护确认或实际写入流程。
 *
 * 模拟量输出起止点必须处于当前量程上限内且彼此不同；普通参数在启用范围检查时必须位于元数据最小值和最大值之间。
 * 校验通过后，受保护参数先进入二次确认页，其余参数直接进入配置写入流程；失败时显示原因并返回参数详情页。
 */
static void parascopecheck(void)
{
	int index;
	int other_index;
	int32_t maximum;

	index = getHoldValueNum(now_Opera_Num);
	if (index == -1) {
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"非法参数!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
		HAL_Delay(800);
		displaypara();
	} else if ((now_Opera_Num == COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM) ||
	           (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM)) {
		maximum = ao_range_max_01mm();
		other_index = getHoldValueNum((now_Opera_Num == COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM) ?
		                              COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM :
		                              COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM);
		if ((now_Para_CT.val < 0) || (now_Para_CT.val > maximum)) {
			oled_clear();
			DisplayLangaugeLineWords((uint8_t*)"数值超范围!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Value out of Range");
			HAL_Delay(800);
			displaypara();
		} else if ((other_index < 0) || (now_Para_CT.val == param_meta[other_index].val)) {
			oled_clear();
			DisplayLangaugeLineWords((uint8_t*)"起止值不能一样", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Endpoints Differ");
			HAL_Delay(800);
			displaypara();
		} else {
			(void)ao_write_range_pair();
		}
	} else if (param_meta[index].flag_checkvalue) {
		if (now_Para_CT.val < param_meta[index].valuemin || now_Para_CT.val > param_meta[index].valuemax) {
			oled_clear();
			DisplayLangaugeLineWords((uint8_t*)"数值超范围!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Value out of Range");
			HAL_Delay(800);
			displaypara();
		} else {
			if (operation_needs_protect_confirm(now_Opera_Num)) {
				param_protect_confirm();
			} else {
				cmd_configpara_process();
			}
		}
	} else {
		if (operation_needs_protect_confirm(now_Opera_Num)) {
			param_protect_confirm();
		} else {
			cmd_configpara_process();
		}
	}
}

/**
 * @brief 按参数归属将待写值保存到 CPU3 本地 FRAM，或通过板间 Modbus 写入 CPU2 并回读确认。
 *
 * 模拟量输出量程和输出源使用各自的成对写入流程；CPU3 本机参数按数据类型去除显示偏移后写入本地参数区。
 * CPU2 参数按整数、float 或 double 的原始位模式组织 32 位字；命令参数直接写保持寄存器，持久参数通过统一组包接口写入并立即回读。
 *
 * @note 外部串口参数写入后只设置重配置挂起标志，由主循环安全点执行实际重配置；板间写入或回读失败时请求参数刷新并返回菜单，避免继续显示未经确认的值。
 */
static void cmd_configpara_process(void)
{
	int index;

	index = getHoldValueNum(now_Opera_Num);
	if ((now_Opera_Num == COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM) ||
	    (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM)) {
		(void)ao_write_range_pair();
		return;
	}
	if (now_Opera_Num == COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE) {
		(void)ao_write_output_source();
		return;
	}

	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"正在修改参数", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Modify Para");

	/* 如果是本机参数 */
	if(now_Opera_Num > COM_NUM_PARA_LOCAL_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
	{
		int local_value = now_Para_CT.val;
		if ((param_meta[index].data_type != TYPE_FLOAT) && (param_meta[index].data_type != TYPE_DOUBLE)) {
			local_value -= param_meta[index].offset;
		}
	    /* CPU3 本机参数：本地写 + 保存FRAM */
	    Cpu3Local_WriteValue((OperatingNumber)now_Opera_Num, local_value);

	    if (Cpu3Local_IsUartParam((OperatingNumber)now_Opera_Num)) {
	        /* 不在 UI 线程里直接重配，避免与通信收发并发；交给主循环安全点处理 */
	        g_cpu3_uart_reinit_pending = 1;
	    }

	    /* 刷新元数据值供显示 */
	    param_meta[index].val = Cpu3Local_ReadValue((OperatingNumber)now_Opera_Num);
	}
	else /* 下发给CPU2 */
	{
	    uint32_t hold32[16]; /* rgstcnt 最大一般不会很大；16 个 u32 = 64 字节 */
	    int raw_value;
	    bool command_argument_only;


	    if (!CPU2_CommIsAvailable()) {
	        display_cpu2_comm_failure();
	        mainmenu();
	        return;
	    }

	    memset(hold32, 0, sizeof(hold32));

	    /* 直接组织 32 位原始值，字序统一交给 CPU2_CombinatePackage_Send 处理。 */
	    if (param_meta[index].data_type == TYPE_FLOAT) {
	        union utof tmp_f;
	        tmp_f.f = now_Para_CT.val * pow(0.1, (double)param_meta[index].point);
	        hold32[0] = tmp_f.u;
	    } else if (param_meta[index].data_type == TYPE_DOUBLE) {
	        union utod tmp_d;
	        tmp_d.d = now_Para_CT.val * pow(0.1, (double)param_meta[index].point);
	        hold32[0] = tmp_d.u[1];
	        hold32[1] = tmp_d.u[0];
	    } else {
	        raw_value = now_Para_CT.val - param_meta[index].offset;
	        hold32[0] = (uint32_t)((int32_t)raw_value);
	    }

	    command_argument_only = screen_param_is_command_argument(index);
	    if (command_argument_only) {
	        uint16_t parameter_regs[REG_STRIDE];
	        uint8_t write_result;

	        if (param_meta[index].rgstcnt != REG_STRIDE) {
	            display_cpu2_modbus_exception(CPU2_MODBUS_EX_ILLEGAL_VALUE);
	            mainmenu();
	            return;
	        }
	        parameter_regs[0] = (uint16_t)(hold32[0] >> 16);
	        parameter_regs[1] = (uint16_t)(hold32[0] & 0xFFFFU);
	        write_result = CPU2_CommWriteHoldingRegistersEx(
	            param_meta[index].startadd,
	            param_meta[index].rgstcnt,
	            parameter_regs);
	        if (write_result != CPU2_MODBUS_RESULT_OK) {
	            display_cpu2_comm_failure();
	            mainmenu();
	            return;
	        }
	    } else {
	        if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
	                                        param_meta[index].startadd,
	                                        param_meta[index].rgstcnt,
	                                        hold32)) {
	            if (CPU2_CommGetLastModbusException() != CPU2_MODBUS_RESULT_OK) {
	                display_cpu2_modbus_exception(CPU2_CommGetLastModbusException());
	                NowKeyPress = 0;
	                displaypara();
	                return;
	            }
	            CPU2_CommRequestParameterRefresh();
	            display_cpu2_comm_failure();
	            mainmenu();
	            return;
	        }
	        if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
	                                        param_meta[index].startadd,
	                                        param_meta[index].rgstcnt,
	                                        NULL) ||
	            !CPU2_CommIsAvailable()) {
	            CPU2_CommRequestParameterRefresh();
	            display_cpu2_comm_failure();
	            mainmenu();
	            return;
	        }
	    }
	}
	HAL_Delay(800);
	displaypara();
}

/**
 * @brief 菜单选项栏(分页显示)。
 *
 * @param menu 当前页面使用的菜单项数组。
 * @param menulen 当前菜单文字数组的有效条目数量。
 */
static void menuselect(struct MenuData *menu, int menulen)
{
	const static int RowsPerPage = 4;
	static int shift = 0;
	int i = 0;
	int row = 0;
	int line = 0;
	int now_menu_num = 0;

	if (NowKeyPress == USE_KEY_UP) {
		PageNum[func_index].menu_cnt--;
		if (PageNum[func_index].menu_cnt <= 0) {
			PageNum[func_index].menu_cnt += menulen;
		}
	} else if (NowKeyPress == USE_KEY_DOWN) {
		PageNum[func_index].menu_cnt++;
	} else if (NowKeyPress == USE_KEY_SURE) {
		if (timesure != 0) {
			timesure = 0;
			now_menu_num = PageNum[func_index].menu_num;
			now_Opera_Num = menu[now_menu_num].operaNum;
			menu[now_menu_num].sureopera();
			return;
		}
		timesure++;
	} else if (NowKeyPress == USE_KEY_BACK) {
		if (timeback != 0) {
			timeback = 0;
			timesure = 1;
			menu[menulen - 1].sureopera();
			return;
		}
		timeback++;
	}

	PageNum[func_index].menu_num = (PageNum[func_index].menu_cnt - 1) % menulen;
	PageNum[func_index].menu_page = (PageNum[func_index].menu_num / RowsPerPage) * RowsPerPage;

	for (i = PageNum[func_index].menu_page;
		(i < (PageNum[func_index].menu_page + RowsPerPage)) && (i < menulen);
		i++) {

		if (i == PageNum[func_index].menu_num) {
			shift = 1;
		} else {
			shift = 0;
		}

		display_menu_item_with_value(&menu[i], line, row, shift);
		row += OLED_ROW4_2;
	}
}

/**
 * @brief 正负号输入。
 *
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param line OLED 绘制使用的横向列位置。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 * @return 返回正负号输入流程结果；确认完成时返回 1，取消或尚未完成时返回 0。
 */
static int SignInput(uint8_t row, uint8_t line, uint8_t shift)
{
	int ret = 0;

	if (NowKeyPress == USE_KEY_SURE) {
		s_param_sign_confirm_count++;
	} else if (NowKeyPress == USE_KEY_UP) {
		s_param_sign_value *= -1;
		s_param_sign_confirm_count = 0;
	} else if (NowKeyPress == USE_KEY_DOWN) {
		s_param_sign_value *= -1;
		s_param_sign_confirm_count = 0;
	} else {
		s_param_sign_value = 1;
		s_param_sign_confirm_count = 0;
	}

	if (s_param_sign_value == -1) {
		line = OledDisplayLineWords((u8*)"-", line, row, shift);
	} else {
		line = OledDisplayLineWords((u8*)"+", line, row, shift);
	}

	OledValueDisplay(now_Para_CT.val, line, row, 0, now_Para_CT.points, now_Para_CT.unit);

	if (s_param_sign_confirm_count > 1) {
		ret = s_param_sign_value;
	}

	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, s_param_sign_confirm_count > 0);

	return ret;
}

/**
 * @brief 判断参数操作号是否对应外部 COM 口的协议选择项。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return true 表示 operaNum 为 CPU3 COM1、COM2 或 COM3 的协议选择操作号；false 表示其它参数或命令操作号。
 */
static bool opera_is_com_protocol(int operaNum)
{
	return (operaNum == COM_NUM_CPU3_COM1_PROTOCOL)
		|| (operaNum == COM_NUM_CPU3_COM2_PROTOCOL)
		|| (operaNum == COM_NUM_CPU3_COM3_PROTOCOL);
}

/**
 * @brief 把串口协议枚举值转换为菜单选择下标。
 *
 * @param value 当前串口协议枚举值。
 * @return 返回 DSM、瓦锡兰、LTD、LH、SI 对应的菜单下标 0 至 4；未知协议回退到 DSM 下标 0。
 */
static int protocol_value_to_selection_index(int value)
{
	switch (value) {
	case COM_PROTO_DSM:
		return 0;
	case COM_PROTO_WARTSILA:
		return 1;
	case COM_PROTO_LTD:
		return 2;
	case COM_PROTO_LH:
		return 3;
	case COM_PROTO_SI:
		return 4;
	default:
		return 0;
	}
}

/**
 * @brief 将菜单下标转换为实际参数编码；串口协议和电机电流使用非零起点/稀疏枚举，越界时回退默认值。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @param selectedIndex 索引值。
 * @return 返回菜单下标对应的实际协议、电流或普通参数编码；下标越界时返回该参数类型的默认值。
 */
static int selection_index_to_value(int operaNum, int selectedIndex)
{
	static const int protocol_values[] = {
		COM_PROTO_DSM,
		COM_PROTO_WARTSILA,
		COM_PROTO_LTD,
		COM_PROTO_LH,
		COM_PROTO_SI,
	};

	if (opera_is_com_protocol(operaNum)) {
		if (selectedIndex < 0 || selectedIndex >= (int)(sizeof(protocol_values) / sizeof(protocol_values[0]))) {
			return COM_PROTO_DSM;
		}
		return protocol_values[selectedIndex];
	}

	if (operaNum == COM_NUM_DEVICEPARAM_MOTOR_CURRENT) {
		if (selectedIndex < 0) {
			return (int)MOTOR_CURRENT_DEFAULT;
		}
		if (selectedIndex >= (int)(MOTOR_CURRENT_MAX - MOTOR_CURRENT_MIN + 1U)) {
			return (int)MOTOR_CURRENT_DEFAULT;
		}
		return selectedIndex + (int)MOTOR_CURRENT_MIN;
	}

	return selectedIndex;
}

/**
 * @brief 将尺带厚度参数值映射为材料型号选择索引。
 *
 * @param value 当前尺带厚度参数值。
 * @return 返回 0.15、0.20、0.25 mm 对应的下标 0、1、2；其他厚度返回自定义厚度下标。
 */
static int tape_thickness_to_selection_index(int value)
{
	switch (value) {
	case TAPE_THICKNESS_PET_001MM:
		return 0;
	case TAPE_THICKNESS_PEEK_001MM:
		return 1;
	case TAPE_THICKNESS_ETFE_001MM:
		return 2;
	default:
		return TAPE_THICKNESS_CUSTOM_INDEX;
	}
}

/**
 * @brief 为尺带厚度提供材料型号快速选择。
 *
 * @details 调用场景：参数详情页选择修改“尺带厚度”时调用。
 * @note 关键约束：前三项仍写入现有厚度参数，手输保留原数字输入入口。
 */
static void tape_thickness_select(void)
{
	const int menulen = (int)((sizeof(arr_tape_thickness) / sizeof(arr_tape_thickness[0])) - 1U);
	static int menu_cnt = 1;
	static int timesure_local = 0;
	static int timeback_local = 0;
	static int last_value = -1;
	int selected_index;
	int i;
	int row = 0;
	int line = 0;
	int shift = 0;
	int current_value;

	current_value = param_meta[getHoldValueNum(COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM)].val;
	if (last_value != current_value) {
		menu_cnt = tape_thickness_to_selection_index(current_value) + 1;
		timesure_local = 0;
		timeback_local = 0;
		last_value = current_value;
	}

	oled_clear();
	func_index = KEYNUM_TAPE_THICKNESS_SELECT;

	if (NowKeyPress == USE_KEY_UP) {
		menu_cnt--;
		if (menu_cnt <= 0) {
			menu_cnt += menulen;
		}
	} else if (NowKeyPress == USE_KEY_DOWN) {
		menu_cnt++;
	} else if (NowKeyPress == USE_KEY_SURE) {
		if (timesure_local != 0) {
			selected_index = (menu_cnt - 1) % menulen;
			timesure_local = 0;
			timeback_local = 0;
			last_value = -1;

			if (selected_index == TAPE_THICKNESS_CUSTOM_INDEX) {
				menu_cnt = 1;
				inputcmdpara();
				return;
			}

			if (selected_index == 0) {
				now_Para_CT.val = TAPE_THICKNESS_PET_001MM;
			} else if (selected_index == 1) {
				now_Para_CT.val = TAPE_THICKNESS_PEEK_001MM;
			} else {
				now_Para_CT.val = TAPE_THICKNESS_ETFE_001MM;
			}
			now_Para_CT.points = 3;
			now_Para_CT.unit = (uint8_t*)"mm";
			now_Para_CT.bits = 6;
			parascopecheck();
			return;
		}
		timesure_local++;
		if (timeback_local != 0) {
			timeback_local = 0;
		}
	} else if (NowKeyPress == USE_KEY_BACK) {
		if (timeback_local != 0) {
			menu_cnt = 1;
			timeback_local = 0;
			timesure_local = 0;
			last_value = -1;
			displaypara();
			return;
		}
		timeback_local++;
		if (timesure_local != 0) {
			timesure_local = 0;
		}
	}

	selected_index = (menu_cnt - 1) % menulen;
	for (i = 0; i < menulen; i++) {
		shift = (i == selected_index) ? 1 : 0;
		OledDisplayLineWords(oled_fit_text(arr_tape_thickness[i][screen_parameter.language], OLED_LINE8_END), line, row, shift);
		row += OLED_ROW4_2;
	}
}

/**
 * @brief 返回通讯方式文字信息。
 *
 * @return 返回通讯方式文字信息对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
uint8_t *ret_arr_word(void)
{
	int index, len;
	/* 当前参数对应的中英文枚举文字表指针；由 dtm_disarr 返回静态二维数组，同时通过输出参数给出当前索引和条目数。 */
	uint8_t *(*p)[2];

	p = dtm_disarr(&index, &len);
	if (p != NULL && index >= 0 && index < len) {
		return p[index][screen_parameter.language];
	} else {
		return (uint8_t*)"非法配置";
	}
}

/**
 * @brief 根据当前操作参数选择对应的中英文枚举文字表。
 *
 * 函数以 now_Opera_Num 和参数元数据现值为输入，返回枚举选择页应使用的静态双语文字表，同时通过输出参数给出当前选中索引和表项数量。
 * 继电器配置先按每路字段号选择工作模式、数字量、触点、报警模式、故障源、测量源或启用开关表；普通参数再按操作号选择密度方向、开关、模拟量输出、显示、串口和测量模式等文字表。
 * 电机电流越界时回退到默认电流对应索引，串口协议通过协议值映射为菜单索引，水位模式把 CPU2 的所有非零值归一为快速模式；不支持的操作号或空表返回 NULL。
 *
 * @param pindex 用于返回当前菜单文字数组的默认选中项，索引从 0 开始。
 * @param plen 用于返回当前菜单文字数组的条目数量。
 * @return 返回当前操作对应的中英文文字二维数组；无法映射时返回 NULL。
 * @note pindex 和 plen 必须指向可写整数；返回值指向静态文字表，调用方不得修改或释放。
 */
uint8_t *(*dtm_disarr(int *pindex, int *plen))[2]
{
	int index = 0, len = -1;
	/* 当前参数对应的中英文枚举文字表指针；由 dtm_disarr 返回静态二维数组，同时通过输出参数给出当前索引和条目数。 */
	uint8_t *(*p)[2] = NULL;

	index = getHoldValueNum(now_Opera_Num);

    if (RelayParam_IsConfig(now_Opera_Num)) {
        int field_index = RelayParam_FieldOf(now_Opera_Num);
        index = param_meta[index].val;
        switch (field_index) {
        case 0:
            len = (int)(sizeof(arr_relay_operating) / sizeof(arr_relay_operating[0]));
            p = arr_relay_operating;
            break;
        case 1:
            len = (int)(sizeof(arr_relay_digital) / sizeof(arr_relay_digital[0]));
            p = arr_relay_digital;
            break;
        case 2:
            len = (int)(sizeof(arr_relay_contact) / sizeof(arr_relay_contact[0]));
            p = arr_relay_contact;
            break;
        case 3:
            len = (int)(sizeof(arr_relay_alarm_mode) / sizeof(arr_relay_alarm_mode[0]));
            p = arr_relay_alarm_mode;
            break;
        case 4:
            len = (int)(sizeof(arr_relay_error) / sizeof(arr_relay_error[0]));
            p = arr_relay_error;
            break;
        case 5:
            len = (int)(sizeof(arr_relay_source) / sizeof(arr_relay_source[0]));
            p = arr_relay_source;
            break;
        case 12:
            len = (int)(sizeof(arr_IF) / sizeof(arr_IF[0]));
            p = arr_IF;
            break;
        default:
            return NULL;
        }
        *pindex = index;
        *plen = len;
        return p;
    }

	switch (now_Opera_Num) {
	case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_densitydir) / sizeof(arr_densitydir[0]));
		p = arr_densitydir;
		break;
	}
	case COM_NUM_SCREEN_SOURCE_OIL:
	case COM_NUM_SCREEN_SOURCE_WATER:
	case COM_NUM_SCREEN_SOURCE_D:
	case COM_NUM_SCREEN_SOURCE_T: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_source) / sizeof(arr_source[0]));
		p = arr_source;
		break;
	}
	case COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT:
	case COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT:
	case COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY:
	case COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO:
	case COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT:
	case COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG:
	case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE:
	case COM_NUM_CPU3_SI_AUTO_PROFILE_ENABLE:
	case COM_NUM_SCREEN_OFF: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_IF) / sizeof(arr_IF[0]));
		p = arr_IF;
		break;
	}
	case COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_position_source_auto_switch) / sizeof(arr_position_source_auto_switch[0]));
		p = arr_position_source_auto_switch;
		break;
	}
	case COM_NUM_DEVICEPARAM_AO_WORK_MODE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_ao_work_mode) / sizeof(arr_ao_work_mode[0]));
		p = arr_ao_work_mode;
		break;
	}
	case COM_NUM_DEVICEPARAM_AO_CURRENT_MODE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_ao_current_mode) / sizeof(arr_ao_current_mode[0]));
		p = arr_ao_current_mode;
		break;
	}
	case COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_ao_output_source) / sizeof(arr_ao_output_source[0]));
		p = arr_ao_output_source;
		break;
	}
	case COM_NUM_DEVICEPARAM_AO_FAULT_MODE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_ao_fault_mode) / sizeof(arr_ao_fault_mode[0]));
		p = arr_ao_fault_mode;
		break;
	}
	case COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_position_count_mode) / sizeof(arr_position_count_mode[0]));
		p = arr_position_count_mode;
		break;
	}
	case COM_NUM_DEVICEPARAM_MOTOR_CURRENT: {
		int value = param_meta[index].val;

		if ((value < (int)MOTOR_CURRENT_MIN) || (value > (int)MOTOR_CURRENT_MAX)) {
			value = (int)MOTOR_CURRENT_DEFAULT;
		}
		index = value - (int)MOTOR_CURRENT_MIN;
		len = (int)(sizeof(arr_motor_current) / sizeof(arr_motor_current[0]));
		p = arr_motor_current;
		break;
	}
	case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_densitymode) / sizeof(arr_densitymode[0]));
		p = arr_densitymode;
		break;
	}
	case COM_NUM_SCREEN_INPUT_D_SWITCH: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_Switch) / sizeof(arr_Switch[0]));
		p = arr_Switch;
		break;
	}
	case COM_NUM_PARA_LANG: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_language) / sizeof(arr_language[0]));
		p = arr_language;
		break;
	}
	case COM_NUM_SCREEN_BRIGHTNESS: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_oled_brightness) / sizeof(arr_oled_brightness[0]));
		p = arr_oled_brightness;
		break;
	}
	case COM_NUM_CPU3_COM1_BAUDRATE:
	case COM_NUM_CPU3_COM2_BAUDRATE:
	case COM_NUM_CPU3_COM3_BAUDRATE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_baudrate) / sizeof(arr_baudrate[0]));
		p = arr_baudrate;
		break;
	}
	case COM_NUM_CPU3_COM1_DATABITS:
	case COM_NUM_CPU3_COM2_DATABITS:
	case COM_NUM_CPU3_COM3_DATABITS: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_databits) / sizeof(arr_databits[0]));
		p = arr_databits;
		break;
	}
	case COM_NUM_CPU3_COM1_PARITY:
	case COM_NUM_CPU3_COM2_PARITY:
	case COM_NUM_CPU3_COM3_PARITY:{
		index = param_meta[index].val;
		len = (int)(sizeof(arr_parity) / sizeof(arr_parity[0]));
		p = arr_parity;
		break;
	}
	case COM_NUM_CPU3_COM1_STOPBITS:
	case COM_NUM_CPU3_COM2_STOPBITS:
	case COM_NUM_CPU3_COM3_STOPBITS:{
		index = param_meta[index].val;
		len = (int)(sizeof(arr_stopbits) / sizeof(arr_stopbits[0]));
		p = arr_stopbits;
		break;
	}
	case COM_NUM_CPU3_COM1_PROTOCOL:
	case COM_NUM_CPU3_COM2_PROTOCOL:
	case COM_NUM_CPU3_COM3_PROTOCOL:{
		index = protocol_value_to_selection_index(param_meta[index].val);
		len = (int)(sizeof(arr_protocol) / sizeof(arr_protocol[0]));
		p = arr_protocol;
		break;
	}
	case COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE:{
		index = param_meta[index].val;
		len = (int)(sizeof(arr_bottom) / sizeof(arr_bottom[0]));
		p = arr_bottom;
		break;
	}
	case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:{
		index = param_meta[index].val;
		len = (int)(sizeof(default_cmmand) / sizeof(default_cmmand[0]));
		p = default_cmmand;
		break;
	}
	case COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD:{
		index = param_meta[index].val;
		len = (int)(sizeof(level_mode) / sizeof(level_mode[0]));
		p = level_mode;
		break;
	}
	case COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE:{
		/* CPU2 按 0/非0 判断水位流程，显示侧把所有非0值归一为快速模式。 */
		index = (param_meta[index].val == 0) ? 0 : 1;
		len = (int)(sizeof(water_level_mode) / sizeof(water_level_mode[0]));
		p = water_level_mode;
		break;
	}
	default:
		return NULL;
	}

	if ((p == NULL) || (len <= 0)) {
		return NULL;
	}

	*pindex = index;
	*plen = len;
	return p;
}


/**
 * @brief 按当前参数元数据取得枚举选项并定位现值，随后进入分页选择页。
 */
static void selectparaword(void)
{
	/* 当前参数选择页使用的中英文枚举文字表指针；指向 dtm_disarr 返回的静态数组，只读使用且不得释放。 */
	uint8_t *(*parr)[2];
	int len, index;
	int selected_index;

	oled_clear();
	func_index = KEYNUM_WORDSELECT;

	parr = dtm_disarr(&index, &len);
	if (parr == NULL || len <= 1) {
		DisplayLangaugeLineWords((uint8_t*)"非法参数!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
		HAL_Delay(800);
		displaypara();
		return;
	}

	selected_index = index;
	if (selected_index < 0) {
		selected_index = 0;
	} else if (selected_index >= (len - 1)) {
		selected_index = len - 2;
	}
	operationselect(parr, len - 1, selected_index);
}

/**
 * @brief 隐藏信息含义选择栏。
 *
 * @param menu 当前页面使用的菜单项数组。
 * @param menulen 当前菜单文字数组的有效条目数量。
 * @param selected_index 已选索引。
 */
static void operationselect(uint8_t *(*menu)[2], int menulen, int selected_index)
{
	const static int RowsPerPage = 4;
	static int shift = 0;
	int i = 0;
	int row = 0;
	int line = 0;
	static int menu_cnt_hide = 1;
	static int timesure_hide = 0;
	static int timeback_hide = 0;
	static int menu_page_hide = 0;
	static int menu_num_hide = -1;
	static int last_opera_num = -1;

	if (menu == NULL || menulen <= 0) {
		DisplayLangaugeLineWords((uint8_t*)"非法参数!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
		HAL_Delay(800);
		displaypara();
		return;
	}

	if (last_opera_num != now_Opera_Num) {
		if (selected_index < 0) {
			selected_index = 0;
		} else if (selected_index >= menulen) {
			selected_index = menulen - 1;
		}
		menu_cnt_hide = selected_index + 1;
		menu_num_hide = selected_index;
		menu_page_hide = (menu_num_hide / RowsPerPage) * RowsPerPage;
		timesure_hide = 0;
		timeback_hide = 0;
		last_opera_num = now_Opera_Num;
	}

	if (NowKeyPress == USE_KEY_UP) {
		menu_cnt_hide--;
		if (menu_cnt_hide <= 0) {
			menu_cnt_hide += menulen;
		}
	} else if (NowKeyPress == USE_KEY_DOWN) {
		menu_cnt_hide++;
	} else if (NowKeyPress == USE_KEY_SURE) {
		if (timesure_hide != 0) {
			timesure_hide = 0;
			now_Para_CT.val = selection_index_to_value(now_Opera_Num, menu_num_hide);
			last_opera_num = -1;
			parascopecheck();
			return;
		}
		timesure_hide++;
		if (timeback_hide != 0) {
			timeback_hide = 0;
		}
	} else if (NowKeyPress == USE_KEY_BACK) {
		if (timeback_hide != 0) {
			menu_num_hide = -1;
			menu_cnt_hide = 1;
			menu_page_hide = 0;
			timeback_hide = 0;
			timesure_hide = 0;
			last_opera_num = -1;
			displaypara();
			return;
		}
		timeback_hide++;
		if (timesure_hide != 0) {
			timesure_hide = 0;
		}
	}

	menu_num_hide = (menu_cnt_hide - 1) % menulen;
	menu_page_hide = (menu_num_hide / RowsPerPage) * RowsPerPage;

	for (i = menu_page_hide; (i < (menu_page_hide + RowsPerPage)) && (i < menulen); i++) {
		if (i == menu_num_hide) {
			shift = 1;
		} else {
			shift = 0;
		}
		OledDisplayLineWords(oled_fit_text(menu[i][screen_parameter.language], OLED_LINE8_END), line, row, shift);
		row += OLED_ROW4_2;
	}
}

/**
 * @brief 清零菜单分页、光标和选项缓存，准备构建新页面。
 */
void ClearPageNum(void)
{
	int i;
	for (i = 0; i < KEYNUM_END; i++) {
		PageNum[i].menu_cnt = 1;
		PageNum[i].menu_num = -1;
		PageNum[i].menu_page = 0;
	}
	timesure = 0;
	timeback = 0;
}

/**
 * @brief 构建并显示罐上操作主菜单，提供测量、参数、调试、语言和退出入口。
 */
static void mainmenu(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"测量命令", COM_NUM_NOOPERA, measuremenu, COMMANE_NORW, (uint8_t*)"MeasureCommend" },
		{ (uint8_t*)"参数配置", COM_NUM_NOOPERA, password_enter_para, COMMANE_NORW, (uint8_t*)"Para-config" },
		{ (uint8_t*)"调试指令", COM_NUM_NOOPERA, password_enter_cmd, COMMANE_NORW, (uint8_t*)"Debug Commend" },
		{ (uint8_t*)"语言", COM_NUM_PARA_LANG, setlanguage, COMMANE_NORW, (uint8_t*)"Language" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, ifexittankopera, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	oled_clear();
	func_index = KEYNUM_MAINMENU;
	menuselect(menu, menulen);
}
/**
 * @brief 构建罐上操作测量菜单，分派零点、液位、水位、罐高、综合、部件参数和密度测量入口。
 */
static void measuremenu(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"提零点", COM_NUM_BACK_ZERO, ifsendcmd, COMMANE_NORW, (uint8_t*)"BackZero" },
        { (uint8_t*)"液位测量", COM_NUM_FIND_OIL, ifsendcmd, COMMANE_NORW, (uint8_t*)"FindOil" },
        { (uint8_t*)"水位测量", COM_NUM_NOOPERA, menu_measure_water, COMMANE_NORW, (uint8_t*)"WaterMeasure" },
        { (uint8_t*)"罐高测量", COM_NUM_FIND_BOTTOM, ifsendcmd, COMMANE_NORW, (uint8_t*)"FindBottom" },
        { (uint8_t*)"综合测量", COM_NUM_SYNTHETIC, ifsendcmd, COMMANE_NORW, (uint8_t*)"Synthetic" },
        { (uint8_t*)"读取部件参数", COM_NUM_READ_PART_PARAMS, ifsendcmd, COMMANE_NORW, (uint8_t*)"ReadPartParams" },
        { (uint8_t*)"密度单点测量", COM_NUM_NOOPERA, menu_measure_density_single, COMMANE_NORW, (uint8_t*)"SingleDensity" },
        { (uint8_t*)"密度分布测量", COM_NUM_NOOPERA, menu_measure_density_distribution, COMMANE_NORW, (uint8_t*)"DistDensity" },
        { (uint8_t*)"浮子运行到高度", COM_NUM_RUN_TO_POSITION, inputcmdpara, COMMANE_NORW, (uint8_t*)"RunToPos" },
        { (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
    };

    int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

    oled_clear();
    func_index = KEYNUM_MEASURE_MAINMENU;
    menuselect(menu, menulen);
}

/**
 * @brief 构建水位测量子菜单并进入选择界面。
 */
static void menu_measure_water(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"水位单次测量", COM_NUM_FIND_WATER, ifsendcmd, COMMANE_NORW, (uint8_t*)"FindWater" },
        { (uint8_t*)"水位跟随", COM_NUM_FOLLOW_WATER, ifsendcmd, COMMANE_NORW, (uint8_t*)"FollowWater" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, measuremenu, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_MEASURE_WATER;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建单点密度测量与监测子菜单并进入选择界面。
 */
static void menu_measure_density_single(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"密度单点测量", COM_NUM_SINGLE_POINT, inputcmdpara, COMMANE_NORW, (uint8_t*)"SingleMeasure" },
        { (uint8_t*)"密度单点监测", COM_NUM_SP_TEST, inputcmdpara, COMMANE_NORW, (uint8_t*)"SingleMonitor" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, measuremenu, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_MEASURE_DENSITY_SINGLE;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建分布类密度测量子菜单并进入选择界面。
 */
static void menu_measure_density_distribution(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"分布测量", COM_NUM_SPREADPOINTS, ifsendcmd, COMMANE_NORW, (uint8_t*)"DistMeasure" },
        { (uint8_t*)"国标分布测量", COM_NUM_SPREADPOINTS_GB, ifsendcmd, COMMANE_NORW, (uint8_t*)"GB_DistMeasure" },
        { (uint8_t*)"密度每米测量", COM_NUM_METER_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"MeterDensity" },
        { (uint8_t*)"区间密度测量", COM_NUM_INTERVAL_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"RangeDensity" },
        { (uint8_t*)"瓦锡兰区间密度", COM_NUM_WARTSILA_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"WartsilaRange" },
        { (uint8_t*)"SI Profile", COM_NUM_SI_PROFILE, ifsendcmd, COMMANE_NORW, (uint8_t*)"SIProfile" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, measuremenu, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_MEASURE_DENSITY_DISTRIBUTION;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}


/**
 * @brief 菜单 - 调试指令。
 */
static void menu_cmdconfig_main(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"浮子运动控制", COM_NUM_NOOPERA, menu_debug_float_motion, COMMANE_NORW, (uint8_t*)"FloatMotion" },
        { (uint8_t*)"标定修正", COM_NUM_NOOPERA, menu_debug_calibration, COMMANE_NORW, (uint8_t*)"Calibration" },
        { (uint8_t*)"扭力标定", COM_NUM_NOOPERA, menu_debug_weight, COMMANE_NORW, (uint8_t*)"TorqueCal" },
        { (uint8_t*)"无线维护", COM_NUM_NOOPERA, menu_debug_wireless, COMMANE_NORW, (uint8_t*)"WirelessMaint" },
        { (uint8_t*)"系统维护", COM_NUM_NOOPERA, menu_debug_system, COMMANE_NORW, (uint8_t*)"SystemMaint" },
        { (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
    };

    int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

    oled_clear();
    func_index = KEYNUM_MENU_CMD_MAIN;
    debugmode_back = 1;
    menuselect(menu, menulen);
}

/**
 * @brief 构建浮子运动控制子菜单并进入选择界面。
 */
static void menu_debug_float_motion(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"上行", COM_NUM_RUNUP, inputcmdpara, COMMANE_NORW, (uint8_t*)"MoveUp" },
        { (uint8_t*)"下行", COM_NUM_RUNDOWN, inputcmdpara, COMMANE_NORW, (uint8_t*)"MoveDown" },
        { (uint8_t*)"强制上行", COM_NUM_FORCE_RUNUP, inputcmdpara, COMMANE_NORW, (uint8_t*)"ForceMoveUp" },
        { (uint8_t*)"强制下行", COM_NUM_FORCE_RUNDOWN, inputcmdpara, COMMANE_NORW, (uint8_t*)"ForceMoveDown" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, menu_cmdconfig_main, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_DEBUG_FLOAT_MOTION;
    debugmode_back = 1;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建标定修正子菜单并进入选择界面。
 */
static void menu_debug_calibration(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"标定零点", COM_NUM_FIND_ZERO, ifsendcmd, COMMANE_NORW, (uint8_t*)"CalZero" },
        { (uint8_t*)"标定液位", COM_NUM_CAL_OIL, inputcmdpara, COMMANE_NORW, (uint8_t*)"CalOil" },
        { (uint8_t*)"修正液位", COM_NUM_CORRECTION_OIL, inputcmdpara, COMMANE_NORW, (uint8_t*)"CorrectOil" },
        { (uint8_t*)"标定水位", COM_NUM_CALIBRATE_WATER, inputcmdpara, COMMANE_NORW, (uint8_t*)"CalWater" },
        { (uint8_t*)"标定罐高", COM_NUM_CALIBRATE_TANKHEIGHT, inputcmdpara, COMMANE_NORW, (uint8_t*)"CalTankH" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, menu_cmdconfig_main, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_DEBUG_CALIBRATION;
    debugmode_back = 1;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建扭力标定子菜单并进入选择界面。
 */
static void menu_debug_weight(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"获取空载扭力", COM_NUM_SET_EMPTY_WEIGHT, ifsendcmd, COMMANE_NORW, (uint8_t*)"SetEmptyTorque" },
        { (uint8_t*)"获取满载扭力", COM_NUM_SET_FULL_WEIGHT, ifsendcmd, COMMANE_NORW, (uint8_t*)"SetFullTorque" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, menu_cmdconfig_main, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_DEBUG_WEIGHT;
    debugmode_back = 1;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建无线维护子菜单并进入选择界面。
 */
static void menu_debug_wireless(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"匹配无线滑环", COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING, ifsendcmd, COMMANE_NORW, (uint8_t*)"PairWireless" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, menu_cmdconfig_main, COMMANE_NORW, (uint8_t*)"Back" },
    };

    oled_clear();
    func_index = KEYNUM_DEBUG_WIRELESS;
    debugmode_back = 1;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建系统维护子菜单并进入选择界面。
 */
static void menu_debug_system(void)
{
    static struct MenuData menu[] = {
        { (uint8_t*)"进入维护模式", COM_NUM_MAINTENANCE_MODE, ifsendcmd, COMMANE_NORW, (uint8_t*)"Maintenance" },
        { (uint8_t*)"清全部锁存", COM_NUM_CLEAR_ALL_RELAY_LATCHED_ALARMS, ifsendcmd, COMMANE_NORW, (uint8_t*)"ClearAllLatched" },
        { (uint8_t*)"恢复出厂设置", COM_NUM_RESTOR_EFACTORYSETTING, ifsendcmd, COMMANE_NORW, (uint8_t*)"RestoreFactory" },
        { (uint8_t*)"返回", COM_NUM_NOOPERA, menu_cmdconfig_main, COMMANE_NORW, (uint8_t*)"Back" },
    };
	bool snapshot_valid = CPU2_CommHasRuntimeSnapshot();

	/* 快照无效时第一项只显示未知状态，不允许把未知误判为可进入维护。 */
	if (!snapshot_valid) {
		menu[0].operaName = (uint8_t*)"维护状态未知";
		menu[0].operaName2 = (uint8_t*)"MNT Unavailable";
		menu[0].operaNum = COM_NUM_NOOPERA;
		menu[0].sureopera = menu_debug_system;
	} else if (g_measurement.device_status.maintenance_mode_active != 0U) {
		menu[0].operaName = (uint8_t*)"退出维护模式";
		menu[0].operaName2 = (uint8_t*)"Exit Maintenance";
		menu[0].operaNum = COM_NUM_MAINTENANCE_EXIT;
		menu[0].sureopera = ifsendcmd;
	} else {
		menu[0].operaName = (uint8_t*)"进入维护模式";
		menu[0].operaName2 = (uint8_t*)"Enter Maintenance";
		menu[0].operaNum = COM_NUM_MAINTENANCE_MODE;
		menu[0].sureopera = ifsendcmd;
	}

    oled_clear();
    func_index = KEYNUM_DEBUG_SYSTEM;
    debugmode_back = 1;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}


/**
 * @brief 构建并显示中文、英文及退出三项语言选择菜单。
 */
static void setlanguage(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"中文", COM_NUM_PARA_LANG, setchinese, COMMANE_NORW, (uint8_t*)"CHINESE" },
		{ (uint8_t*)"英文", COM_NUM_PARA_LANG, setenglish, COMMANE_NORW, (uint8_t*)"ENGLISH" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	oled_clear();
	func_index = KEYNUM_MENU_LANGUAGE;
	menuselect(menu, menulen);
}

/**
 * @brief 把 CPU3 菜单语言切换为中文并刷新当前页面。
 */
static void setchinese(void)
{
	screen_parameter.language = LANGUAGE_CHINESE;
	mainmenu();
}

/**
 * @brief 把 CPU3 菜单语言切换为英文并刷新当前页面。
 */
static void setenglish(void)
{
	screen_parameter.language = LANGUAGE_ENGLISH;
	mainmenu();
}



/**
 * @brief 停止罐上操作定时器并清理显示分页状态，显示退出提示后交还普通状态页。
 */
void exitTankOpera(void)
{
	FlagofTankOpera = false;
	HAL_TIM_Base_Stop_IT(&htim1);

	oled_clear();
	ClearPageNum();

	DisplayLangaugeLineWords((uint8_t*)"正在退出罐上操作",
			OLED_LINE8_1, OLED_ROW3_2, 0,
			(uint8_t*)"Exit operation");
}



/**
 * @brief 判断菜单名称是否以中文“保留”前缀开头。
 *
 * @param name 待测量、裁剪、分行或匹配的 OLED 菜单文字字节串；中文按双字节字库字符处理，ASCII 按单字节处理。
 * @return 1 表示名称以中文“保留”前缀开头；空指针或普通名称返回 0。
 * @note 仅用于隐藏自动菜单中的保留项，不改变参数元数据或寄存器映射。
 * @note 当前源码按 UTF-8 维护，必须比较完整“保留”前缀，不能只比较半个汉字的字节。
 */
static int is_reserved_cn(const uint8_t *name)
{
    static const char reserved_prefix[] = "保留";

    if (name == NULL) {
        return 0;
    }

    return (strncmp((const char*)name, reserved_prefix, sizeof(reserved_prefix) - 1U) == 0);
}

/**
 * @brief 把继电器参数操作号转换为零基通道号。
 *
 * 这里集中识别通道和字段，避免菜单和枚举文字显示各自写裸范围判断。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 返回零基继电器通道号 0 至 3；操作号不属于四路继电器字段时返回 -1。
 * @note 每路继电器报警输出固定占用 13 个参数字段；通道和字段识别集中在本函数维护。
 */
static int RelayParam_ChannelOf(int operaNum)
{
    if ((operaNum >= COM_NUM_DEVICEPARAM_RELAY1_OPERATING_MODE) &&
        (operaNum <= COM_NUM_DEVICEPARAM_RELAY1_CLEAR_ALARM)) {
        return 0;
    }

    if ((operaNum >= COM_NUM_DEVICEPARAM_RELAY2_OPERATING_MODE) &&
        (operaNum <= COM_NUM_DEVICEPARAM_RELAY2_CLEAR_ALARM)) {
        return 1;
    }

    if ((operaNum >= COM_NUM_DEVICEPARAM_RELAY3_OPERATING_MODE) &&
        (operaNum <= COM_NUM_DEVICEPARAM_RELAY3_CLEAR_ALARM)) {
        return 2;
    }

    if ((operaNum >= COM_NUM_DEVICEPARAM_RELAY4_OPERATING_MODE) &&
        (operaNum <= COM_NUM_DEVICEPARAM_RELAY4_CLEAR_ALARM)) {
        return 3;
    }

    return -1;
}

/**
 * @brief 把继电器参数操作号转换为通道内字段序号。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 返回通道内字段序号 0 至 12；操作号不属于四路继电器字段时返回 -1。
 */
static int RelayParam_FieldOf(int operaNum)
{
    switch (RelayParam_ChannelOf(operaNum)) {
    case 0:
        return operaNum - COM_NUM_DEVICEPARAM_RELAY1_OPERATING_MODE;
    case 1:
        return operaNum - COM_NUM_DEVICEPARAM_RELAY2_OPERATING_MODE;
    case 2:
        return operaNum - COM_NUM_DEVICEPARAM_RELAY3_OPERATING_MODE;
    case 3:
        return operaNum - COM_NUM_DEVICEPARAM_RELAY4_OPERATING_MODE;
    default:
        return -1;
    }
}

/**
 * @brief 判断操作号是否属于四路继电器的持久化配置字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int RelayParam_IsConfig(int operaNum)
{
    int field = RelayParam_FieldOf(operaNum);

    return (field >= 0) && (field < (int)RELAY_ALARM_FIELD_COUNT);
}

/**
 * @brief 判断继电器参数是否属于通道类型、触点类型或报警模式设置。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示继电器参数属于通道类型、触点类型或报警模式设置；0 表示继电器参数不属于通道类型、触点类型或报警模式设置。
 */
static int RelayParam_IsChannelSetting(int operaNum)
{
    switch (RelayParam_FieldOf(operaNum)) {
    case 0:  /* 工作模式 */
    case 1:  /* 输出报警位 */
    case 2:  /* 触点类型 */
    case 12: /* 清除锁存报警 */
        return 1;
    default:
        return 0;
    }
}

/**
 * @brief 判断继电器参数是否属于报警阈值或滞回条件。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int RelayParam_IsAlarmCondition(int operaNum)
{
    int field = RelayParam_FieldOf(operaNum);

    return (field >= 3) && (field <= 11);
}

/**
 * @brief 判断继电器字段是否为可带符号的报警阈值。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int RelayParam_IsAlarmValueField(int operaNum)
{
    int field = RelayParam_FieldOf(operaNum);

    return (field >= 6) && (field <= 10);
}

/**
 * @brief 返回继电器参数详情页的上一级配置菜单。
 *
 * @details 调用场景：参数详情页按返回键时，由 dtm_backtofunc() 根据当前参数调用。
 * @note 关键约束：通道设置和报警配置是四级菜单，不能统一返回继电器总列表。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 返回当前继电器详情字段对应的上一级菜单回调函数指针。
 */
static pFunc_void RelayParam_BackToConfigMenu(int operaNum)
{
    int channel = RelayParam_ChannelOf(operaNum);

    if (RelayParam_IsChannelSetting(operaNum) != 0) {
        switch (channel) {
        case 0:
            return menu_relay1_channel;
        case 1:
            return menu_relay2_channel;
        case 2:
            return menu_relay3_channel;
        case 3:
            return menu_relay4_channel;
        default:
            return menu_do_alarm;
        }
    }

    if (RelayParam_IsAlarmCondition(operaNum) != 0) {
        switch (channel) {
        case 0:
            return menu_relay1_alarm;
        case 1:
            return menu_relay2_alarm;
        case 2:
            return menu_relay3_alarm;
        case 3:
            return menu_relay4_alarm;
        default:
            return menu_do_alarm;
        }
    }

    return menu_do_alarm;
}

/* 继电器运行状态页的字段标识；枚举顺序同时作为双语字段名表的索引，新增或调整时必须同步 relay_status_field_name。 */
typedef enum {
    /* 继电器状态页字段顺序；该顺序与双语字段名数组及页面选择索引一一对应。 */
    RELAY_STATUS_FIELD_ALARM_VALUE = 0, /* 显示当前报警源过程量或比较值。 */
    RELAY_STATUS_FIELD_HH, /* 显示高高报警判定状态。 */
    RELAY_STATUS_FIELD_H, /* 显示高报警判定状态。 */
    RELAY_STATUS_FIELD_HH_H, /* 显示高高或高报警的合并状态。 */
    RELAY_STATUS_FIELD_L, /* 显示低报警判定状态。 */
    RELAY_STATUS_FIELD_LL, /* 显示低低报警判定状态。 */
    RELAY_STATUS_FIELD_LL_L, /* 显示低低或低报警的合并状态。 */
    RELAY_STATUS_FIELD_ANY, /* 显示任一配置报警条件是否成立。 */
    RELAY_STATUS_FIELD_CLEAR_LATCHED, /* 显示或执行清除锁存报警字段。 */
    RELAY_STATUS_FIELD_ACTION_INHIBITED, /* 显示当前继电器逻辑动作是否被抑制。 */
    RELAY_STATUS_FIELD_FINAL_ACTION, /* 显示抑制和锁存处理后的最终逻辑动作。 */
    RELAY_STATUS_FIELD_COUNT /* 继电器状态字段总数，仅用于数组容量和边界检查。 */
} RelayStatusField;

typedef struct {
    /* 继电器运行状态字段的中英文显示名称。 */
    uint8_t *name_cn; /* 该表项对应的中文显示名称。 */
    uint8_t *name_en; /* 该表项对应的英文显示名称。 */
} RelayStatusFieldName;

/* 继电器运行状态字段的中英文名称表，索引必须与 RelayStatusField 枚举一致。 */
static const RelayStatusFieldName relay_status_field_name[] = {
    { (uint8_t*)"报警值", (uint8_t*)"Value" },
    { (uint8_t*)"HH",     (uint8_t*)"HH" },
    { (uint8_t*)"H",      (uint8_t*)"H" },
    { (uint8_t*)"HH/H",   (uint8_t*)"HH/H" },
    { (uint8_t*)"L",      (uint8_t*)"L" },
    { (uint8_t*)"LL",     (uint8_t*)"LL" },
    { (uint8_t*)"LL/L",   (uint8_t*)"LL/L" },
    { (uint8_t*)"Any",    (uint8_t*)"Any" },
    { (uint8_t*)"清锁存", (uint8_t*)"Clear" },
    { (uint8_t*)"动作禁用", (uint8_t*)"Block" },
    { (uint8_t*)"最终动作", (uint8_t*)"Action" },
};

/**
 * @brief 把继电器报警激活状态转换为当前语言的显示文字。
 *
 * @param state 继电器报警运行状态，期望为 RELAY_ALARM_STATE_ACTIVE 或 RELAY_ALARM_STATE_INACTIVE；其他值显示为非法。
 * @return 返回当前语言的显示文字对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static uint8_t *relay_alarm_state_word(uint32_t state)
{
    if (state == RELAY_ALARM_STATE_ACTIVE) {
        return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"报警" : (uint8_t*)"ALM";
    }

    if (state == RELAY_ALARM_STATE_INACTIVE) {
        return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"正常" : (uint8_t*)"OK";
    }

    return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"非法" : (uint8_t*)"Invalid";
}

/**
 * @brief 把继电器锁存清除状态转换为当前语言的显示文字。
 *
 * @param state 继电器锁存清除状态，期望为 RELAY_ALARM_CLEAR_YES 或 RELAY_ALARM_CLEAR_NO；其他值显示为非法。
 * @return 返回当前语言的显示文字对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static uint8_t *relay_clear_state_word(uint32_t state)
{
    if (state == RELAY_ALARM_CLEAR_YES) {
        return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"是" : (uint8_t*)"YES";
    }

    if (state == RELAY_ALARM_CLEAR_NO) {
        return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"否" : (uint8_t*)"NO";
    }

    return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"非法" : (uint8_t*)"Invalid";
}

/**
 * @brief 按 HH、H、组合高限、L、LL、组合低限或任一报警字段读取继电器运行快照；未知字段按未激活处理。
 *
 * @param state 待读取的单路继电器运行快照；传入 NULL 时按未激活返回。
 * @param field 待查询或显示的字段枚举值。该枚举指定继电器运行快照中的报警值、分级状态、锁存清除或最终动作字段。
 * @return 返回指定报警字段的 RELAY_ALARM_STATE_ACTIVE 或 RELAY_ALARM_STATE_INACTIVE；字段非法时返回非活动态。
 */
static uint32_t relay_status_state_of(const volatile RelayAlarmRuntimeState *state, RelayStatusField field)
{
    if (state == NULL) {
        return RELAY_ALARM_STATE_INACTIVE;
    }

    switch (field) {
    case RELAY_STATUS_FIELD_HH:
        return state->HH_alarm;
    case RELAY_STATUS_FIELD_H:
        return state->H_alarm;
    case RELAY_STATUS_FIELD_HH_H:
        return state->HH_H_alarm;
    case RELAY_STATUS_FIELD_L:
        return state->L_alarm;
    case RELAY_STATUS_FIELD_LL:
        return state->LL_alarm;
    case RELAY_STATUS_FIELD_LL_L:
        return state->LL_L_alarm;
    case RELAY_STATUS_FIELD_ANY:
        return state->any_error;
    default:
        return RELAY_ALARM_STATE_INACTIVE;
    }
}

/**
 * @brief 最终动作表示维护/人工禁用处理后的逻辑动作，不代表 NO/NC 反相后的物理触点反馈。
 *
 * @param active 目标活动状态，true 表示活动。
 * @return 返回最终动作表示维护/人工禁用处理后的逻辑动作，不代表 NO/NC 反相后的物理触点反馈对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static uint8_t *relay_final_action_word(bool active)
{
    if (active) {
        return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"动作" : (uint8_t*)"ON";
    }

    return (screen_parameter.language == LANGUAGE_CHINESE) ? (uint8_t*)"不动作" : (uint8_t*)"OFF";
}

/**
 * @brief 把继电器实时报警值换算为一位小数显示使用的整数。
 *
 * @param value CPU2 运行态快照中的继电器报警工程值。
 * @return 返回按一位小数放大并进行正负对称四舍五入后的显示整数。
 */
static int relay_status_alarm_value_x10(float value)
{
    float scaled = value * 10.0f;

    return (scaled >= 0.0f) ? (int)(scaled + 0.5f) : (int)(scaled - 0.5f);
}

/**
 * @brief 绘制继电器报警值、各级报警状态、锁存清除、动作禁用或最终动作；快照无效时显示 N/A，且不触发参数写入。
 *
 * @param state 当前通道的只读继电器运行快照，包含报警值、分级报警、锁存清除和最终动作状态。
 * @param channel 零基通道号。合法范围为 0～3，用于把继电器菜单字段映射到第 1～4 路显示和配置项。
 * @param field 待查询或显示的字段枚举值。该枚举指定继电器运行快照中的报警值、分级状态、锁存清除或最终动作字段。
 * @param snapshot_valid true 表示继电器运行快照有效，false 表示必须显示不可用状态。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 */
static void display_relay_status_row(const volatile RelayAlarmRuntimeState *state,
                                     uint32_t channel,
                                     RelayStatusField field,
                                     bool snapshot_valid,
                                     uint8_t row,
                                     uint8_t shift)
{
    uint8_t line;
    uint8_t *name;

    if ((field < 0) || (field >= (int)(sizeof(relay_status_field_name) / sizeof(relay_status_field_name[0])))) {
        return;
    }

    name = (screen_parameter.language == LANGUAGE_CHINESE) ?
        relay_status_field_name[field].name_cn : relay_status_field_name[field].name_en;
    line = OledDisplayLineWords(name, OLED_LINE8_1, row, shift);
    line = OledDisplayLineWords((uint8_t*)":", line, row, shift);

    if (!snapshot_valid || (state == NULL)) {
        OledDisplayLineWords((uint8_t*)"N/A", line, row, shift);
    } else if (field == RELAY_STATUS_FIELD_ALARM_VALUE) {
        OledValueDisplay(relay_status_alarm_value_x10(state->alarm_value),
                         line,
                         row,
                         shift,
                         1,
                         NULL);
    } else if (field == RELAY_STATUS_FIELD_CLEAR_LATCHED) {
        OledDisplayLineWords(relay_clear_state_word(state->clear_alarm),
                             line,
                             row,
                             shift);
    } else if (field == RELAY_STATUS_FIELD_ACTION_INHIBITED) {
        OledDisplayLineWords((g_measurement.device_status.relay_alarm_inhibit_effective != 0U) ?
                             returnWordType((uint8_t*)"是", (uint8_t*)"YES") :
                             returnWordType((uint8_t*)"否", (uint8_t*)"NO"),
                             line,
                             row,
                             shift);
    } else if (field == RELAY_STATUS_FIELD_FINAL_ACTION) {
        OledDisplayLineWords(relay_final_action_word((g_measurement.device_status.relay_alarm_action_mask &
                                                      (1UL << channel)) != 0U),
                             line,
                             row,
                             shift);
    } else {
        OledDisplayLineWords(relay_alarm_state_word(relay_status_state_of(state, field)), line, row, shift);
    }
}

/**
 * @brief 显示单路继电器报警运行态，只读消费 CPU2 输入寄存器快照，不触发参数下发。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于把继电器菜单字段映射到第 1～4 路显示和配置项。
 * @param keynum 当前页面收到的按键编码。
 * @param backfunc 退出当前继电器状态页时调用的返回页回调函数。
 */
static void menu_relay_status(uint32_t channel, keymenuNumber keynum, pFunc_void backfunc)
{
    /* 继电器运行状态页的局部布局常量。 */
    enum { RELAY_STATUS_ROWS = 2 /* 继电器运行状态页固定显示的字段行数。 */ };
    volatile RelayAlarmRuntimeState *state;
    bool snapshot_valid;
    int selected;
    int first;
    int i;
    char title[16];

    if ((NowKeyPress == USE_KEY_BACK) && (backfunc != NULL)) {
        timeback = 0;
        timesure = 1;
        backfunc();
        return;
    }

    oled_clear();
    func_index = keynum;

    if (PageNum[keynum].menu_cnt <= 0) {
        PageNum[keynum].menu_cnt = 1;
    }

    if (NowKeyPress == USE_KEY_UP) {
        PageNum[keynum].menu_cnt--;
        if (PageNum[keynum].menu_cnt <= 0) {
            PageNum[keynum].menu_cnt += RELAY_STATUS_FIELD_COUNT;
        }
    } else if (NowKeyPress == USE_KEY_DOWN) {
        PageNum[keynum].menu_cnt++;
    }

    PageNum[keynum].menu_num = (PageNum[keynum].menu_cnt - 1) % RELAY_STATUS_FIELD_COUNT;
    first = (PageNum[keynum].menu_num / RELAY_STATUS_ROWS) * RELAY_STATUS_ROWS;

    snprintf(title, sizeof(title), "K%lu%s",
             (unsigned long)(channel + 1U),
             (screen_parameter.language == LANGUAGE_CHINESE) ? "报警状态" : " Alarm");
    OledDisplayLineWords((uint8_t*)title, OLED_LINE8_1, OLED_ROW4_1, 0);

    snapshot_valid = CPU2_CommHasRuntimeSnapshot();
    state = (snapshot_valid && (channel < RELAY_ALARM_CHANNEL_COUNT)) ?
            &g_measurement.relay_alarm_runtime[channel] : NULL;
    for (i = 0; (i < RELAY_STATUS_ROWS) && ((first + i) < RELAY_STATUS_FIELD_COUNT); i++) {
        selected = first + i;
        display_relay_status_row(state,
                                 channel,
                                 (RelayStatusField)selected,
                                 snapshot_valid,
                                 (uint8_t)(OLED_ROW4_2 + (i * OLED_ROW4_2)),
                                 (selected == PageNum[keynum].menu_num) ? 1U : 0U);
    }

    DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
}

/**
 * @brief 将参数操作号映射到自动菜单分组。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 返回参数操作号所属的 MenuGroup 枚举，用于自动构造对应菜单分组。
 * @note 返回分组只决定 CPU3 菜单入口，不改变参数读写、同步或持久化路径。
 */
static MenuGroup ParamGroupOf(int operaNum)
{
    if (RelayParam_IsConfig(operaNum)) {
        return MENU_GRP_DO_ALARM;
    }

    switch (operaNum) {
    /* 设备信息 */
    case COM_NUM_DEVICEPARAM_SENSORTYPE:
    case COM_NUM_DEVICEPARAM_SENSORID:
    case COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION:
    case COM_NUM_DEVICEPARAM_SOFTWAREVERSION:
    case COM_NUM_PARA_LOCAL_LEDVERSION:
    case COM_NUM_DEVICEPARAM_PROTOCOL_VERSION:
        return MENU_GRP_DEV_INFO;

    /* 运行策略 */
    case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:
    case COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO:
    case COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT:
    case COM_NUM_DEVICEPARAM_RESERVED2:
        return MENU_GRP_RUN_POLICY;

    /* 机械/电机/编码器 */
    case COM_NUM_DEVICEPARAM_MOTOR_CURRENT:
    case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
    case COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED:
    case COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM:
    case COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM:
    case COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH:
    case COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE:
    case COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC:
        return MENU_GRP_MECH;

    /* 扭力 */
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT:
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT:
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT:
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT:
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT:
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT:
    case COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO:
    case COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO:
        return MENU_GRP_WEIGHT;

    /* 零点 */
    case COM_NUM_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO:
    case COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE:
    case COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE:
    case COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE:
        return MENU_GRP_ZERO;

    /* 液位 */
    case COM_NUM_DEVICEPARAM_TANKHEIGHT:
    case COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF:
    case COM_NUM_DEVICEPARAM_BLINDZONE:
    case COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD:
    case COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD:
    case COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD:
    case COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY:
    case COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY:
    case COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME:
        return MENU_GRP_LIQUID;

    /* 水位 */
    case COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT:
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE:
    case COM_NUM_DEVICEPARAM_WATER_BLINDZONE:
    case COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD:
    case COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD:
    case COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE:
    case COM_NUM_DEVICEPARAM_ZERO_CAP:
    case COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD:
    case COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD:
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME:
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION:
        return MENU_GRP_WATER;

    /* 罐底/罐高 */
    case COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE:
    case COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD:
    case COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD:
    case COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG:
    case COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION:
    case COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT:
    case COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT:
    case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE:
        return MENU_GRP_BOTTOM_TANKH;

    /* 修正 */
    case COM_NUM_DEVICEPARAM_DENSITYCORRECTION:
    case COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION:
    case COM_NUM_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL:
    case COM_NUM_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE:
    case COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT:
    case COM_NUM_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE:
        return MENU_GRP_CORR;

    /* 策略/分布/区间 */
    case COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT:
    case COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT:
    case COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY:
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER:
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE:
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT:
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE:
    case COM_NUM_DEVICEPARAM_SPREADTOPLIMIT:
    case COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT:
    case COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME:
    case COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT:
    case COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT:
        return MENU_GRP_POLICY;

    /* Wartsila */
    case COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT:
    case COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT:
    case COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL:
    case COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE:
    case COM_NUM_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL:
    case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT:
        return MENU_GRP_WARTSILA;

    case COM_NUM_DEVICEPARAM_SI_PROFILE_FIRST_POINT:
    case COM_NUM_DEVICEPARAM_SI_PROFILE_INCREMENT:
    case COM_NUM_DEVICEPARAM_SI_PROFILE_DWELL_TIME:
    case COM_NUM_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL:
        return MENU_GRP_SI_PROFILE;

    /* AO通道设置 */
    case COM_NUM_DEVICEPARAM_AO_WORK_MODE:
    case COM_NUM_DEVICEPARAM_AO_CURRENT_MODE:
    case COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE:
        return MENU_GRP_AO_CHANNEL;

    /* AO量程设置 */
    case COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000:
    case COM_NUM_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100:
    case COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM:
    case COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM:
    case COM_NUM_DEVICEPARAM_AO_DAMPING_X10_S:
        return MENU_GRP_AO_RANGE;

    /* AO故障设置 */
    case COM_NUM_DEVICEPARAM_AO_FAULT_MODE:
    case COM_NUM_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100:
    case COM_NUM_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100:
        return MENU_GRP_AO_FAULT;

    case COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100:
    case COM_NUM_AO_SIMULATION_ENABLE:
        return MENU_GRP_AO_DIAGNOSTIC;

    case COM_NUM_AO_RUNTIME_PROCESS_VALUE:
    case COM_NUM_AO_RUNTIME_PERCENT:
    case COM_NUM_AO_RUNTIME_OUTPUT_CURRENT:
        return MENU_GRP_AO_RUNTIME;

    case COM_NUM_DEVICEPARAM_AO_ERROR_LEVEL:
        return MENU_GRP_AO_RESERVED;

    /* 标定/单点/位置 */
    case COM_NUM_DEVICEPARAM_CALIBRATE_OIL_LEVEL:
    case COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL:
    case COM_NUM_DEVICEPARAM_CALIBRATE_TANK_HEIGHT:
    case COM_NUM_DEVICEPARAM_SP_MEAS_POSITION:
    case COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION:
    case COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL:
    case COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE:
    case COM_NUM_CAL_OIL:
    case COM_NUM_CORRECTION_OIL:
    case COM_NUM_CALIBRATE_WATER:
    case COM_NUM_CALIBRATE_TANKHEIGHT:
    case COM_NUM_SINGLE_POINT:
    case COM_NUM_SP_TEST:
    case COM_NUM_RUN_TO_POSITION:
    case COM_NUM_RUNUP:
    case COM_NUM_RUNDOWN:
    case COM_NUM_FORCE_RUNUP:
    case COM_NUM_FORCE_RUNDOWN:
        return MENU_GRP_CAL_SP;

    /* 校验信息 */
    case COM_NUM_DEVICEPARAM_PARAM_VERSION:
    case COM_NUM_DEVICEPARAM_STRUCT_SIZE:
    case COM_NUM_DEVICEPARAM_MAGIC:
    case COM_NUM_DEVICEPARAM_CRC:
        return MENU_GRP_PARAM_CHECK;

    /* CPU3 本机参数 */
    case COM_NUM_PARA_LANG:
        return MENU_GRP_CPU3_BASE;

    case COM_NUM_SCREEN_SOURCE_OIL:
    case COM_NUM_SCREEN_SOURCE_WATER:
    case COM_NUM_SCREEN_SOURCE_D:
    case COM_NUM_SCREEN_SOURCE_T:
        return MENU_GRP_CPU3_SOURCE;

    case COM_NUM_SCREEN_INPUT_OIL:
    case COM_NUM_SCREEN_INPUT_WATER:
    case COM_NUM_SCREEN_INPUT_D:
    case COM_NUM_SCREEN_INPUT_D_SWITCH:
    case COM_NUM_SCREEN_INPUT_T:
        return MENU_GRP_CPU3_INPUT;

    case COM_NUM_SCREEN_DECIMAL:
    case COM_NUM_SCREEN_PASSWARD:
    case COM_NUM_SCREEN_OFF:
    case COM_NUM_SCREEN_BRIGHTNESS:
        return MENU_GRP_CPU3_SCREEN;

    case COM_NUM_CPU3_COM1_BAUDRATE:
    case COM_NUM_CPU3_COM1_DATABITS:
    case COM_NUM_CPU3_COM1_PARITY:
    case COM_NUM_CPU3_COM1_STOPBITS:
    case COM_NUM_CPU3_COM1_PROTOCOL:
        return MENU_GRP_CPU3_COM1;

    case COM_NUM_CPU3_COM2_BAUDRATE:
    case COM_NUM_CPU3_COM2_DATABITS:
    case COM_NUM_CPU3_COM2_PARITY:
    case COM_NUM_CPU3_COM2_STOPBITS:
    case COM_NUM_CPU3_COM2_PROTOCOL:
        return MENU_GRP_CPU3_COM2;

    case COM_NUM_CPU3_COM3_BAUDRATE:
    case COM_NUM_CPU3_COM3_DATABITS:
    case COM_NUM_CPU3_COM3_PARITY:
    case COM_NUM_CPU3_COM3_STOPBITS:
    case COM_NUM_CPU3_COM3_PROTOCOL:
        return MENU_GRP_CPU3_COM3;

    case COM_NUM_CPU3_SI_AUTO_PROFILE_INTERVAL:
    case COM_NUM_CPU3_SI_AUTO_PROFILE_ENABLE:
    case COM_NUM_CPU3_SI_AUTO_PROFILE_HOUR:
    case COM_NUM_CPU3_SI_AUTO_PROFILE_MINUTE:
        return MENU_GRP_CPU3_SI_AUTO;

    case COM_NUM_CPU3_SI_LOW_DENSITY_SETPOINT:
    case COM_NUM_CPU3_SI_HIGH_DENSITY_SETPOINT:
    case COM_NUM_CPU3_SI_LOW_TEMPERATURE_SETPOINT:
    case COM_NUM_CPU3_SI_HIGH_TEMPERATURE_SETPOINT:
    case COM_NUM_CPU3_SI_LL_LEVEL_SETPOINT:
    case COM_NUM_CPU3_SI_HH_LEVEL_SETPOINT:
    case COM_NUM_CPU3_SI_LOW_LEVEL_SETPOINT:
    case COM_NUM_CPU3_SI_HIGH_LEVEL_SETPOINT:
    case COM_NUM_CPU3_SI_TEMP_DEVIATION_SETPOINT:
    case COM_NUM_CPU3_SI_DENSITY_DEVIATION_SETPOINT:
        return MENU_GRP_CPU3_SI_ALARM;

    default:
        /* 未分类项：避免丢失，统一放到“校验信息”或“基础信息”都可以 */
        return MENU_GRP_DEV_INFO;
    }
}

/**
 * @brief 判断参数是否应显示在指定菜单分组。
 *
 * @details 调用场景：自动构造参数菜单时使用；允许少数跨业务参数在多个入口出现。
 * @note 关键约束：只影响屏幕菜单入口，不复制参数元数据，也不改变寄存器和写回路径。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @param grp 菜单参数分组标识，用于筛选当前分组可见项。
 * @return true 表示参数原生分组等于 grp，或参数为底部编码器修正罐高或使能项，且 grp 为底部罐高、Wärtsilä 或 SI Profile 分组；false 表示既不属于目标原生分组，也不满足这两个跨分组参数的例外规则。
 */
static bool ParamVisibleInGroup(int operaNum, MenuGroup grp)
{
    if (ParamGroupOf(operaNum) == grp) {
        return true;
    }

    if (operaNum == COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT) {
        return (grp == MENU_GRP_BOTTOM_TANKH) ||
               (grp == MENU_GRP_WARTSILA) ||
               (grp == MENU_GRP_SI_PROFILE);
    }

    if (operaNum == COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE) {
        return (grp == MENU_GRP_BOTTOM_TANKH) ||
               (grp == MENU_GRP_WARTSILA) ||
               (grp == MENU_GRP_SI_PROFILE);
    }

    return false;
}

/* -------------------- 自动生成菜单列表 -------------------- */
#define AUTO_MENU_MAX_ITEMS  90

/**
 * @brief 按菜单分组动态构造参数菜单列表并进入选择界面。
 * @param grp 菜单分组。
 * @param key_index 当前菜单页索引。
 * @param backFunc 返回上一级菜单的回调函数。
 */
static void menu_build_by_group(MenuGroup grp, int key_index, void (*backFunc)(void))
{
    static struct MenuData menu[AUTO_MENU_MAX_ITEMS + 1];
    int menulen = 0;

    for (int i = 0; i < (int)param_metaAmount; i++) {

        const struct ParameterMetadata *m = &param_meta[i];

        /* 分组过滤 */
        if (!ParamVisibleInGroup(m->operanum, grp)) continue;

        /* 不展示“设备指令寄存器” */
        if (m->operanum == COM_NUM_DEVICEPARAM_COMMAND) continue;

        /* 过滤保留项 */
        if (is_reserved_cn(m->name)) continue;

        if (menulen >= AUTO_MENU_MAX_ITEMS) break;

        menu[menulen].operaName  = m->name;
        menu[menulen].operaNum   = m->operanum;
        menu[menulen].sureopera  = para_mainprocess;

        /* 关键：用你的 authority_write 来决定读/写 */
        menu[menulen].rorw = (m->authority_write &&
                              (!ao_param_is_config(m->operanum) || ao_param_is_editable(m->operanum))) ?
                             COMMAND_WRITE : COMMAND_READ;

        /* 英文名称 */
        menu[menulen].operaName2 = m->name_English;
        menulen++;
    }

    /* 返回项 */
    menu[menulen].operaName  = (uint8_t*)"返回";
    menu[menulen].operaNum   = COM_NUM_NOOPERA;
    menu[menulen].sureopera  = backFunc;
    menu[menulen].rorw       = COMMANE_NORW;
    menu[menulen].operaName2 = (uint8_t*)"Back";
    menulen++;

    oled_clear();
    func_index = key_index;
    menuselect(menu, menulen);
}

/**
 * @brief 按显式参数顺序构造包含共享快捷入口的菜单。
 * @param operas 参数操作号列表。
 * @param count 参数数量。
 * @param key_index 当前菜单页索引。
 * @param backFunc 返回上一级菜单的回调函数。
 * @note 同一参数可出现在多个菜单中，但仍共用唯一元数据、寄存器和写回路径。
 */
static void menu_build_by_operas(const int *operas, int count, int key_index, void (*backFunc)(void))
{
    static struct MenuData menu[AUTO_MENU_MAX_ITEMS + 1];
    int menulen = 0;

    for (int i = 0; (i < count) && (menulen < AUTO_MENU_MAX_ITEMS); i++) {
        int index = getHoldValueNum(operas[i]);
        const struct ParameterMetadata *m;

        if (index < 0) {
            continue;
        }
        m = &param_meta[index];
        menu[menulen].operaName = m->name;
        menu[menulen].operaNum = m->operanum;
        menu[menulen].sureopera = para_mainprocess;
        menu[menulen].rorw = m->authority_write ? COMMAND_WRITE : COMMAND_READ;
        menu[menulen].operaName2 = m->name_English;
        menulen++;
    }

    menu[menulen].operaName = (uint8_t*)"返回";
    menu[menulen].operaNum = COM_NUM_NOOPERA;
    menu[menulen].sureopera = backFunc;
    menu[menulen].rorw = COMMANE_NORW;
    menu[menulen].operaName2 = (uint8_t*)"Back";
    menulen++;

    oled_clear();
    func_index = key_index;
    menuselect(menu, menulen);
}

/**
 * @brief 按过滤条件从 param_meta 收集可见项，追加返回项并构建参数菜单。
 *
 * @param filter 参数菜单项过滤回调；返回非零时保留该项。
 * @param key_index 索引值。
 * @param backFunc 按返回键时调用的页面回调函数。
 */
static void menu_build_by_filter(int (*filter)(int), int key_index, void (*backFunc)(void))
{
    static struct MenuData menu[AUTO_MENU_MAX_ITEMS + 1];
    int menulen = 0;

    for (int i = 0; i < (int)param_metaAmount; i++) {
        const struct ParameterMetadata *m = &param_meta[i];

        if ((filter == NULL) || (filter(m->operanum) == 0)) continue;
        if (is_reserved_cn(m->name)) continue;
        if (menulen >= AUTO_MENU_MAX_ITEMS) break;

        menu[menulen].operaName  = m->name;
        menu[menulen].operaNum   = m->operanum;
        menu[menulen].sureopera  = para_mainprocess;
        menu[menulen].rorw       = (m->authority_write) ? COMMAND_WRITE : COMMAND_READ;
        menu[menulen].operaName2 = m->name_English;
        menulen++;
    }

    menu[menulen].operaName  = (uint8_t*)"返回";
    menu[menulen].operaNum   = COM_NUM_NOOPERA;
    menu[menulen].sureopera  = backFunc;
    menu[menulen].rorw       = COMMANE_NORW;
    menu[menulen].operaName2 = (uint8_t*)"Back";
    menulen++;

    oled_clear();
    func_index = key_index;
    menuselect(menu, menulen);
}

/**
 * @brief 判断参数操作号是否属于继电器 1 的通道配置字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay1_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 0) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 1 的报警条件字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay1_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 0) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 2 的通道配置字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay2_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 1) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 2 的报警条件字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay2_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 1) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 3 的通道配置字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay3_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 2) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 3 的报警条件字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay3_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 2) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 4 的通道配置字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay4_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 3) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 判断参数操作号是否属于继电器 4 的报警条件字段。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_relay4_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 3) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 判断参数操作号是否属于CPU3 基础显示参数。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示参数操作号属于CPU3 基础显示参数；0 表示参数操作号不属于CPU3 基础显示参数。
 */
static int menu_filter_display_base(int operaNum)
{
    switch (operaNum) {
    case COM_NUM_SCREEN_DECIMAL:
    case COM_NUM_SCREEN_OFF:
    case COM_NUM_SCREEN_BRIGHTNESS:
        return 1;
    default:
        return 0;
    }
}

/**
 * @brief 判断参数操作号是否属于油位状态页数据项。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_display_data_oil(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_OIL) || (operaNum == COM_NUM_SCREEN_INPUT_OIL);
}

/**
 * @brief 判断参数操作号是否属于水位状态页数据项。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_display_data_water(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_WATER) || (operaNum == COM_NUM_SCREEN_INPUT_WATER);
}

/**
 * @brief 判断参数操作号是否属于密度状态页数据项。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_display_data_density(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_D)
        || (operaNum == COM_NUM_SCREEN_INPUT_D)
        || (operaNum == COM_NUM_SCREEN_INPUT_D_SWITCH);
}

/**
 * @brief 判断参数操作号是否属于温度状态页数据项。
 *
 * @param operaNum 菜单操作号，对应参数元数据或命令操作码。
 * @return 1 表示操作号属于目标字段集合，0 表示不属于。
 */
static int menu_filter_display_data_temp(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_T) || (operaNum == COM_NUM_SCREEN_INPUT_T);
}

/**
 * @brief 构建测量参数配置菜单并进入参数选择页。
 */
static void menu_measure_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"运行设置",      0, menu_run_policy,   COMMANE_NORW, (uint8_t*)"Run Policy"},
        {(uint8_t*)"电机与编码参数",0, menu_mech,         COMMANE_NORW, (uint8_t*)"Mechanism"},
        {(uint8_t*)"扭力参数",      0, menu_weight,       COMMANE_NORW, (uint8_t*)"Torque"},
        {(uint8_t*)"零点参数",      0, menu_zero,         COMMANE_NORW, (uint8_t*)"Zero"},
        {(uint8_t*)"液位参数",      0, menu_liquid,       COMMANE_NORW, (uint8_t*)"Level"},
        {(uint8_t*)"水位参数",      0, menu_water,        COMMANE_NORW, (uint8_t*)"Water"},
        {(uint8_t*)"罐底与罐高",    0, menu_bottom_tankh, COMMANE_NORW, (uint8_t*)"Bottom/TankH"},
        {(uint8_t*)"密度测量参数",  0, menu_policy,       COMMANE_NORW, (uint8_t*)"Density"},
        {(uint8_t*)"Wartsila参数",  0, menu_wartsila,     COMMANE_NORW, (uint8_t*)"Wartsila"},
        {(uint8_t*)"SI参数",        0, menu_si_config,COMMANE_NORW, (uint8_t*)"SI"},
        {(uint8_t*)"修正参数",      0, menu_correct,      COMMANE_NORW, (uint8_t*)"Correction"},
        {(uint8_t*)"返回",          0, menu_paracfg_main, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_PARA_MEASURE_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建外部通信参数配置菜单并进入端口选择页。
 */
static void menu_comm_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"COM1配置", 0, menu_cpu3_comm1,   COMMANE_NORW, (uint8_t*)"COM1"},
        {(uint8_t*)"COM2配置", 0, menu_cpu3_comm2,   COMMANE_NORW, (uint8_t*)"COM2"},
        {(uint8_t*)"COM3配置", 0, menu_cpu3_comm3,   COMMANE_NORW, (uint8_t*)"COM3"},
        {(uint8_t*)"返回",     0, menu_paracfg_main, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_COMM_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建 CPU3 显示设置菜单并进入参数选择页。
 */
static void menu_display_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"显示基础",      0, menu_display_base,  COMMANE_NORW,   (uint8_t*)"Display"},
        {(uint8_t*)"语言",          COM_NUM_PARA_LANG, para_mainprocess, COMMAND_WRITE, (uint8_t*)"Lang"},
        {(uint8_t*)"屏幕密码",      COM_NUM_SCREEN_PASSWARD, para_mainprocess, COMMAND_WRITE, (uint8_t*)"ScrPwd"},
        {(uint8_t*)"数据源与手输值",0, menu_display_data,  COMMANE_NORW,   (uint8_t*)"Data Source"},
        {(uint8_t*)"返回",          0, menu_paracfg_main, COMMANE_NORW,   (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_DISPLAY_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建状态页数据项配置菜单并进入分类选择页。
 */
static void menu_display_data(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"液位", 0, menu_display_data_oil,     COMMANE_NORW, (uint8_t*)"Oil"},
        {(uint8_t*)"水位", 0, menu_display_data_water,   COMMANE_NORW, (uint8_t*)"Water"},
        {(uint8_t*)"密度", 0, menu_display_data_density, COMMANE_NORW, (uint8_t*)"Density"},
        {(uint8_t*)"温度", 0, menu_display_data_temp,    COMMANE_NORW, (uint8_t*)"Temp"},
        {(uint8_t*)"返回", 0, menu_display_config,       COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_DISPLAY_DATA;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建维护参数配置菜单并进入参数选择页。
 */
static void menu_maint_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"设备信息", 0, menu_dev_info,     COMMANE_NORW, (uint8_t*)"Info"},
        {(uint8_t*)"参数校验", 0, menu_param_check,  COMMANE_NORW, (uint8_t*)"Check"},
        {(uint8_t*)"CPU2通讯", 0, menu_cpu2_comm_health, COMMANE_NORW, (uint8_t*)"CPU2 Comm"},
        {(uint8_t*)"RTC设置",  0, menu_rtc_datetime, COMMANE_NORW, (uint8_t*)"RTC Set"},
        {(uint8_t*)"返回",     0, menu_paracfg_main, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_MAINT_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 返回通讯健康页使用的最近失败原因文字。
 *
 * @details 调用场景：中文或英文故障状态页刷新时调用。
 * @note 关键约束：中文文字必须全部来自现有OLED字库，技术缩写保持ASCII。
 *
 * @param reason CPU2 板间通信失败原因枚举；用于区分启动失败、响应超时、帧错误、Modbus 异常和重试耗尽等失败阶段。
 * @param chinese 中文显示文字指针。
 * @return 返回通讯健康页使用的最近失败原因文字对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *cpu2_comm_failure_reason_text(Cpu2CommFailureReason reason, bool chinese)
{
	switch (reason) {
	case CPU2_COMM_FAIL_TIMEOUT:
		return chinese ? "响应超时" : "TIMEOUT";
	case CPU2_COMM_FAIL_CRC:
		return chinese ? "CRC校验" : "CRC";
	case CPU2_COMM_FAIL_ADDRESS:
		return chinese ? "地址错误" : "ADDRESS";
	case CPU2_COMM_FAIL_FUNCTION:
		return chinese ? "功能码错误" : "FUNCTION";
	case CPU2_COMM_FAIL_LENGTH:
		return chinese ? "长度错误" : "LENGTH";
	case CPU2_COMM_FAIL_UART:
		return chinese ? "UART错误" : "UART";
	case CPU2_COMM_FAIL_TX_DMA:
		return chinese ? "发送失败" : "TXDMA";
	case CPU2_COMM_FAIL_NONE:
	default:
		return chinese ? "无" : "NONE";
	}
}

/**
 * @brief 把健康计数限制到OLED四位显示范围。
 *
 * @details 调用场景：通讯健康页格式化累计计数前调用。
 * @note 关键约束：只限制显示值，不修改RAM中的原始累计值。
 *
 * @param count 参与本次处理的数据项数量。
 * @return 返回可供 OLED 四位数显示的计数值；原始计数超过 9999 时钳位为 9999。
 */
static uint32_t cpu2_comm_display_count(uint64_t count)
{
	return (count > 9999U) ? 9999U : (uint32_t)count;
}

/**
 * @brief 显示CPU3本机累计的CPU2通讯健康计数。
 *
 * @details 调用场景：维护设置进入页面、上下翻页或前景页周期刷新。
 * @note 关键约束：只读取RAM快照，不清零计数、不修改通信状态和共享协议。
 */
static void menu_cpu2_comm_health(void)
{
	Cpu2CommHealthSnapshot snapshot;
	uint64_t total_count;
	uint64_t response_failure_count;
	uint32_t failure_rate_thousandths = 0U;
	bool chinese = (screen_parameter.language == LANGUAGE_CHINESE);
	char line1[32];
	char line2[32];
	char line3[32];
	char line4[32];

	if (func_index != KEYNUM_MENU_CPU2_COMM_HEALTH) {
		cpu2_comm_health_page = 0U;
	} else if (NowKeyPress == USE_KEY_UP) {
		cpu2_comm_health_page = (cpu2_comm_health_page == 0U) ? 2U : (uint8_t)(cpu2_comm_health_page - 1U);
	} else if ((NowKeyPress == USE_KEY_DOWN) || (NowKeyPress == USE_KEY_SURE)) {
		cpu2_comm_health_page = (uint8_t)((cpu2_comm_health_page + 1U) % 3U);
	}

	CPU2_CommGetHealthSnapshot(&snapshot);
	total_count = (uint64_t)snapshot.success_count + (uint64_t)snapshot.total_failure_count;
	response_failure_count = (uint64_t)snapshot.timeout_count +
	                         (uint64_t)snapshot.crc_count +
	                         (uint64_t)snapshot.address_count +
	                         (uint64_t)snapshot.function_count +
	                         (uint64_t)snapshot.length_count;
	if (total_count != 0U) {
		/* 以0.001%为一单位四舍五入，100%对应100000，使用64位避免乘法溢出。 */
		failure_rate_thousandths = (uint32_t)((((uint64_t)snapshot.total_failure_count * 100000U) +
		                                          (total_count / 2U)) /
		                                         total_count);
	}
	oled_clear();
	func_index = KEYNUM_MENU_CPU2_COMM_HEALTH;
	if (cpu2_comm_health_page == 0U) {
		(void)snprintf(line1, sizeof(line1), chinese ? "CPU2通讯 1/3" : "CPU2 COMM 1/3");
		(void)snprintf(line2, sizeof(line2), chinese ? "总次数:%lu" : "TOTAL:%lu",
		               (unsigned long)cpu2_comm_display_count(total_count));
		(void)snprintf(line3, sizeof(line3), chinese ? "失败次数:%lu" : "FAIL:%lu",
		               (unsigned long)cpu2_comm_display_count(snapshot.total_failure_count));
		if (total_count == 0U) {
			(void)snprintf(line4, sizeof(line4), chinese ? "失败率:--.---%%" : "F-RATE:--.---%%");
		} else {
			(void)snprintf(line4, sizeof(line4), chinese ? "失败率:%lu.%03lu%%" : "F-RATE:%lu.%03lu%%",
			               (unsigned long)(failure_rate_thousandths / 1000U),
			               (unsigned long)(failure_rate_thousandths % 1000U));
		}
	} else if (cpu2_comm_health_page == 1U) {
		(void)snprintf(line1, sizeof(line1), chinese ? "错误分类 2/3" : "ERROR TYPE 2/3");
		(void)snprintf(line2, sizeof(line2), chinese ? "响应错误:%lu" : "RESP:%lu",
		               (unsigned long)cpu2_comm_display_count(response_failure_count));
		(void)snprintf(line3, sizeof(line3), chinese ? "UART错误:%lu" : "UART:%lu",
		               (unsigned long)cpu2_comm_display_count(snapshot.uart_failure_count));
		(void)snprintf(line4, sizeof(line4), chinese ? "发送失败:%lu" : "TX:%lu",
		               (unsigned long)cpu2_comm_display_count(snapshot.tx_dma_start_fail_count));
	} else {
		(void)snprintf(line1, sizeof(line1), chinese ? "故障状态 3/3" : "FAIL STATE 3/3");
		(void)snprintf(line2, sizeof(line2), chinese ? "上次:%s" : "LAST:%s",
		               cpu2_comm_failure_reason_text(snapshot.last_failure_reason, chinese));
		(void)snprintf(line3, sizeof(line3), chinese ? "当前连败:%lu" : "SEQ:%lu",
		               (unsigned long)cpu2_comm_display_count(snapshot.consecutive_failure_count));
		(void)snprintf(line4, sizeof(line4), chinese ? "最大连败:%lu" : "MAX:%lu",
		               (unsigned long)cpu2_comm_display_count(snapshot.max_consecutive_failure_count));
	}

	OledDisplayLineWords((uint8_t *)line1, OLED_LINE8_1, OLED_ROW4_1, 0);
	OledDisplayLineWords((uint8_t *)line2, OLED_LINE8_1, OLED_ROW4_2, 0);
	OledDisplayLineWords((uint8_t *)line3, OLED_LINE8_1, OLED_ROW4_3, 0);
	OledDisplayLineWords((uint8_t *)line4, OLED_LINE8_1, OLED_ROW4_4, 0);
}

/**
 * @brief 返回指定年月的实际天数。
 *
 * @param year 完整年份数值。
 * @param month 月份，合法范围为 1～12。
 * @return 返回指定月份的天数；月份非法时返回 0。
 */
static uint8_t rtc_menu_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[] = {
        31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
    };

    if ((month < 1U) || (month > 12U)) {
        return 31U;
    }

    if ((month == 2U) &&
        (((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U)))) {
        return 29U;
    }

    return days[month - 1U];
}

/**
 * @brief 在年月变化后将日期限制到当月有效范围。
 */
static void rtc_menu_normalize_day(void)
{
    uint8_t max_day = rtc_menu_days_in_month(rtc_menu_dt.year, rtc_menu_dt.month);

    if (rtc_menu_dt.day > max_day) {
        rtc_menu_dt.day = max_day;
    }
}

/**
 * @brief 从 RTC 读取当前时间并初始化菜单编辑缓存。
 */
static void rtc_menu_load_current_time(void)
{
    if (Cpu3Clock_GetDateTime(&rtc_menu_dt) == 0U) {
        rtc_menu_dt.year = 2026U;
        rtc_menu_dt.month = 1U;
        rtc_menu_dt.day = 1U;
        rtc_menu_dt.hour = 0U;
        rtc_menu_dt.minute = 0U;
        rtc_menu_dt.second = 0U;
        rtc_menu_dt.valid = 1U;
    }

    rtc_menu_normalize_day();
}

/**
 * @brief 按键调整 RTC 菜单当前选中的日期或时间字段。
 *
 * @param delta 增量。
 */
static void rtc_menu_change_field(int delta)
{
    uint8_t max_day;

    switch (rtc_menu_field) {
    case 0U:
        if ((delta > 0) && (rtc_menu_dt.year < 2099U)) {
            rtc_menu_dt.year++;
        } else if ((delta < 0) && (rtc_menu_dt.year > 2000U)) {
            rtc_menu_dt.year--;
        }
        rtc_menu_normalize_day();
        break;
    case 1U:
        if (delta > 0) {
            rtc_menu_dt.month = (rtc_menu_dt.month >= 12U) ? 1U : (uint8_t)(rtc_menu_dt.month + 1U);
        } else {
            rtc_menu_dt.month = (rtc_menu_dt.month <= 1U) ? 12U : (uint8_t)(rtc_menu_dt.month - 1U);
        }
        rtc_menu_normalize_day();
        break;
    case 2U:
        max_day = rtc_menu_days_in_month(rtc_menu_dt.year, rtc_menu_dt.month);
        if (delta > 0) {
            rtc_menu_dt.day = (rtc_menu_dt.day >= max_day) ? 1U : (uint8_t)(rtc_menu_dt.day + 1U);
        } else {
            rtc_menu_dt.day = (rtc_menu_dt.day <= 1U) ? max_day : (uint8_t)(rtc_menu_dt.day - 1U);
        }
        break;
    case 3U:
        if (delta > 0) {
            rtc_menu_dt.hour = (uint8_t)((rtc_menu_dt.hour + 1U) % 24U);
        } else {
            rtc_menu_dt.hour = (rtc_menu_dt.hour == 0U) ? 23U : (uint8_t)(rtc_menu_dt.hour - 1U);
        }
        break;
    case 4U:
        if (delta > 0) {
            rtc_menu_dt.minute = (uint8_t)((rtc_menu_dt.minute + 1U) % 60U);
        } else {
            rtc_menu_dt.minute = (rtc_menu_dt.minute == 0U) ? 59U : (uint8_t)(rtc_menu_dt.minute - 1U);
        }
        break;
    default:
        if (delta > 0) {
            rtc_menu_dt.second = (uint8_t)((rtc_menu_dt.second + 1U) % 60U);
        } else {
            rtc_menu_dt.second = (rtc_menu_dt.second == 0U) ? 59U : (uint8_t)(rtc_menu_dt.second - 1U);
        }
        break;
    }
}

/**
 * @brief 返回 RTC 设置页当前时钟状态和时钟源对应的显示文字。
 *
 * @return 返回模块静态缓冲区首地址，内容为 RTC ERR、RTC LSI、RTC LSE、RTC NONE 或 RTC UNSET；后续调用会覆盖该缓冲区。
 */
static uint8_t *rtc_menu_status_text(void)
{
    static uint8_t text[16];
    Cpu3ClockState state = Cpu3Clock_GetState();
    Cpu3ClockSource source = Cpu3Clock_GetSource();

    if (state == CPU3_CLOCK_STATE_ERROR) {
        snprintf((char *)text, sizeof(text), "RTC ERR");
    } else if (source == CPU3_CLOCK_SOURCE_LSI) {
        snprintf((char *)text, sizeof(text), "RTC LSI");
    } else if (source == CPU3_CLOCK_SOURCE_LSE) {
        snprintf((char *)text, sizeof(text), "RTC LSE");
    } else {
        snprintf((char *)text, sizeof(text), "RTC NONE");
    }

    if (state == CPU3_CLOCK_STATE_UNSET) {
        snprintf((char *)text, sizeof(text), "RTC UNSET");
    }

    return text;
}

/**
 * @brief 绘制 RTC 日期时间设置页。
 */
static void rtc_menu_draw(void)
{
    static const uint8_t *field_name[] = {
        (uint8_t *)"Y", (uint8_t *)"M", (uint8_t *)"D", (uint8_t *)"h", (uint8_t *)"m", (uint8_t *)"s"
    };
    char line[24];
    uint8_t status_line;

    oled_clear();
    status_line = OledDisplayLineWords(rtc_menu_status_text(), OLED_LINE8_1, OLED_ROW4_1, 0);
    OledDisplayLineWords((uint8_t *)field_name[rtc_menu_field], status_line, OLED_ROW4_1, 1);

    snprintf(line,
             sizeof(line),
             "%04u-%02u-%02u",
             (unsigned int)rtc_menu_dt.year,
             (unsigned int)rtc_menu_dt.month,
             (unsigned int)rtc_menu_dt.day);
    OledDisplayLineWords((uint8_t *)line, OLED_LINE8_1, OLED_ROW4_2, (rtc_menu_field <= 2U) ? 1U : 0U);

    snprintf(line,
             sizeof(line),
             "%02u:%02u:%02u",
             (unsigned int)rtc_menu_dt.hour,
             (unsigned int)rtc_menu_dt.minute,
             (unsigned int)rtc_menu_dt.second);
    OledDisplayLineWords((uint8_t *)line, OLED_LINE8_1, OLED_ROW4_3, (rtc_menu_field >= 3U) ? 1U : 0U);

    DisplayLangaugeLineWords((uint8_t *)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t *)"Back");
    if (rtc_menu_field >= 5U) {
        display_right_aligned_action((uint8_t *)"确认保存", (uint8_t *)"Save", OLED_ROW4_4, 1);
    } else {
        display_right_aligned_action((uint8_t *)"确认", (uint8_t *)"Ok", OLED_ROW4_4, 0);
    }
}

/**
 * @brief 显示 RTC 日期时间编辑页并处理字段切换、保存和返回。
 */
static void menu_rtc_datetime(void)
{
    uint8_t save_ok;

    if (func_index != KEYNUM_MENU_RTC_DATETIME) {
        func_index = KEYNUM_MENU_RTC_DATETIME;
        rtc_menu_field = 0U;
        rtc_menu_load_current_time();
        rtc_menu_draw();
        return;
    }

    if (NowKeyPress == USE_KEY_UP) {
        rtc_menu_change_field(1);
    } else if (NowKeyPress == USE_KEY_DOWN) {
        rtc_menu_change_field(-1);
    } else if (NowKeyPress == USE_KEY_SURE) {
        if (rtc_menu_field >= 5U) {
            save_ok = Cpu3Clock_SetDateTime(&rtc_menu_dt);
            oled_clear();
            if (save_ok != 0U) {
                DisplayLangaugeLineWords((uint8_t *)"已保存", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t *)"Saved");
            } else {
                DisplayLangaugeLineWords((uint8_t *)"保存失败", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t *)"Save Failed");
            }
            HAL_Delay(600);
            NowKeyPress = 0;
            menu_maint_config();
            return;
        }
        rtc_menu_field++;
    } else if (NowKeyPress == USE_KEY_BACK) {
        if (rtc_menu_field == 0U) {
            NowKeyPress = 0;
            menu_maint_config();
            return;
        }
        rtc_menu_field--;
    }

    rtc_menu_draw();
}

/**
 * @brief CPU2：分组页 = 参数列表页（取消 DEBUG 容器页）。
 */
static void menu_run_policy(void)   { menu_build_by_group(MENU_GRP_RUN_POLICY,   KEYNUM_MENU_PARA_RUN_POLICY,   menu_measure_config); }
/**
 * @brief 进入设备信息参数分组菜单。
 */
static void menu_dev_info(void)     { menu_build_by_group(MENU_GRP_DEV_INFO,     KEYNUM_MENU_PARA_DEV_INFO,     menu_maint_config); }
/**
 * @brief 进入机械参数分组菜单。
 */
static void menu_mech(void)         { menu_build_by_group(MENU_GRP_MECH,         KEYNUM_MENU_PARA_MECH,         menu_measure_config); }
/**
 * @brief 进入扭力参数分组菜单。
 */
static void menu_weight(void)       { menu_build_by_group(MENU_GRP_WEIGHT,       KEYNUM_MENU_PARA_WEIGHT,       menu_measure_config); }
/**
 * @brief 构建零点相关参数菜单。
 */
static void menu_zero(void)         { menu_build_by_group(MENU_GRP_ZERO,         KEYNUM_MENU_PARA_ZERO,         menu_measure_config); }

/**
 * @brief 构建油位测量相关参数菜单。
 */
static void menu_liquid(void)       { menu_build_by_group(MENU_GRP_LIQUID,       KEYNUM_MENU_PARA_LIQUID,       menu_measure_config); }
/**
 * @brief 构建水位测量相关参数菜单。
 */
static void menu_water(void)
{
    static const int operas[] = {
        COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE,
        COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT,
        COM_NUM_DEVICEPARAM_WATER_BLINDZONE,
        COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD,
        COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD,
        COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD,
        COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD,
        COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION,
        COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE,
        COM_NUM_DEVICEPARAM_ZERO_CAP,
        COM_NUM_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME,
    };

    menu_build_by_operas(operas,
                         (int)(sizeof(operas) / sizeof(operas[0])),
                         KEYNUM_MENU_PARA_WATER,
                         menu_measure_config);
}
/**
 * @brief 构建探底和罐高标定参数菜单。
 */
static void menu_bottom_tankh(void)
{
    static const int operas[] = {
        COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE,
        COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD,
        COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,
        COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,
        COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION,
        COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT,
        COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT,
        COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE,
        COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT,
    };

    menu_build_by_operas(operas,
                         (int)(sizeof(operas) / sizeof(operas[0])),
                         KEYNUM_MENU_PARA_BOTTOM_TANKH,
                         menu_measure_config);
}

/**
 * @brief 构建测量修正参数菜单。
 */
static void menu_correct(void)      { menu_build_by_group(MENU_GRP_CORR,         KEYNUM_MENU_PARA_CORR,         menu_measure_config); }
/**
 * @brief 构建故障处理与运行策略参数菜单。
 */
static void menu_policy(void)       { menu_build_by_group(MENU_GRP_POLICY,       KEYNUM_MENU_PARA_POLICY,       menu_measure_config); }
/**
 * @brief 构建瓦锡兰密度分布参数菜单。
 */
static void menu_wartsila(void)
{
    static const int operas[] = {
        COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,
        COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,
        COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,
        COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,
        COM_NUM_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL,
        COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT,
        COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE,
    };

    menu_build_by_operas(operas,
                         (int)(sizeof(operas) / sizeof(operas[0])),
                         KEYNUM_MENU_PARA_WARTSILA,
                         menu_measure_config);
}

/**
 * @brief 构建 SI 配置入口菜单。
 */
static void menu_si_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"Profile参数", 0, menu_si_profile,       COMMANE_NORW, (uint8_t*)"Profile"},
        {(uint8_t*)"自动Profile", 0, menu_si_auto_profile,  COMMANE_NORW, (uint8_t*)"Auto Profile"},
        {(uint8_t*)"报警限值",    0, menu_si_alarm,         COMMANE_NORW, (uint8_t*)"Alarm Limit"},
        {(uint8_t*)"返回",        0, menu_measure_config,   COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_SI_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建 SI Profile 参数菜单。
 */
static void menu_si_profile(void)
{
    static const int operas[] = {
        COM_NUM_DEVICEPARAM_SI_PROFILE_FIRST_POINT,
        COM_NUM_DEVICEPARAM_SI_PROFILE_INCREMENT,
        COM_NUM_DEVICEPARAM_SI_PROFILE_DWELL_TIME,
        COM_NUM_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL,
        COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT,
        COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE,
    };

    menu_build_by_operas(operas,
                         (int)(sizeof(operas) / sizeof(operas[0])),
                         KEYNUM_MENU_SI_PROFILE,
                         menu_si_config);
}

/**
 * @brief 构建 SI 自动剖面参数配置页。
 */
static void menu_si_auto_profile(void)
{
    menu_build_by_group(MENU_GRP_CPU3_SI_AUTO, KEYNUM_MENU_SI_AUTO_PROFILE, menu_si_config);
}

/**
 * @brief 构建 SI 报警限值菜单。
 */
static void menu_si_alarm(void)
{
    menu_build_by_group(MENU_GRP_CPU3_SI_ALARM, KEYNUM_MENU_SI_ALARM, menu_si_config);
}

/**
 * @brief 构建模拟量和继电器输出配置菜单。
 */
static void menu_output_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"继电器报警输出", 0, menu_do_alarm,    COMMANE_NORW, (uint8_t*)"Relay Out"},
        {(uint8_t*)"AO输出",     0, menu_ao,          COMMANE_NORW, (uint8_t*)"AO"},
        {(uint8_t*)"返回",       0, menu_paracfg_main,COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_OUTPUT_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建四路继电器报警输出入口菜单。
 */
static void menu_do_alarm(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"K1继电器", 0, menu_relay1_main,  COMMANE_NORW, (uint8_t*)"Relay1"},
        {(uint8_t*)"K2继电器", 0, menu_relay2_main,  COMMANE_NORW, (uint8_t*)"Relay2"},
        {(uint8_t*)"K3继电器", 0, menu_relay3_main,  COMMANE_NORW, (uint8_t*)"Relay3"},
        {(uint8_t*)"K4继电器", 0, menu_relay4_main,  COMMANE_NORW, (uint8_t*)"Relay4"},
        {(uint8_t*)"返回",     0, menu_output_config,COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_PARA_DO;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建继电器 1 的主菜单。
 */
static void menu_relay1_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"通道设置", 0, menu_relay1_channel, COMMANE_NORW, (uint8_t*)"Channel"},
        {(uint8_t*)"报警配置", 0, menu_relay1_alarm,   COMMANE_NORW, (uint8_t*)"Alarm"},
        {(uint8_t*)"报警状态", 0, menu_relay1_status,  COMMANE_NORW, (uint8_t*)"Status"},
        {(uint8_t*)"返回",     0, menu_do_alarm,       COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_RELAY1_MAIN;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建继电器 1 的通道配置菜单。
 */
static void menu_relay1_channel(void)
{
    menu_build_by_filter(menu_filter_relay1_channel, KEYNUM_MENU_RELAY1_CHANNEL, menu_relay1_main);
}

/**
 * @brief 构建继电器 1 的报警条件菜单。
 */
static void menu_relay1_alarm(void)
{
    menu_build_by_filter(menu_filter_relay1_alarm, KEYNUM_MENU_RELAY1_ALARM, menu_relay1_main);
}

/**
 * @brief 显示继电器 1 的实时报警状态页。
 */
static void menu_relay1_status(void)
{
    menu_relay_status(0U, KEYNUM_MENU_RELAY1_STATUS, menu_relay1_main);
}

/**
 * @brief 构建继电器 2 的主菜单。
 */
static void menu_relay2_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"通道设置", 0, menu_relay2_channel, COMMANE_NORW, (uint8_t*)"Channel"},
        {(uint8_t*)"报警配置", 0, menu_relay2_alarm,   COMMANE_NORW, (uint8_t*)"Alarm"},
        {(uint8_t*)"报警状态", 0, menu_relay2_status,  COMMANE_NORW, (uint8_t*)"Status"},
        {(uint8_t*)"返回",     0, menu_do_alarm,       COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_RELAY2_MAIN;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建继电器 2 的通道配置菜单。
 */
static void menu_relay2_channel(void)
{
    menu_build_by_filter(menu_filter_relay2_channel, KEYNUM_MENU_RELAY2_CHANNEL, menu_relay2_main);
}

/**
 * @brief 构建继电器 2 的报警条件菜单。
 */
static void menu_relay2_alarm(void)
{
    menu_build_by_filter(menu_filter_relay2_alarm, KEYNUM_MENU_RELAY2_ALARM, menu_relay2_main);
}

/**
 * @brief 显示继电器 2 的实时报警状态页。
 */
static void menu_relay2_status(void)
{
    menu_relay_status(1U, KEYNUM_MENU_RELAY2_STATUS, menu_relay2_main);
}

/**
 * @brief 构建继电器 3 的主菜单。
 */
static void menu_relay3_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"通道设置", 0, menu_relay3_channel, COMMANE_NORW, (uint8_t*)"Channel"},
        {(uint8_t*)"报警配置", 0, menu_relay3_alarm,   COMMANE_NORW, (uint8_t*)"Alarm"},
        {(uint8_t*)"报警状态", 0, menu_relay3_status,  COMMANE_NORW, (uint8_t*)"Status"},
        {(uint8_t*)"返回",     0, menu_do_alarm,       COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_RELAY3_MAIN;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建继电器 3 的通道配置菜单。
 */
static void menu_relay3_channel(void)
{
    menu_build_by_filter(menu_filter_relay3_channel, KEYNUM_MENU_RELAY3_CHANNEL, menu_relay3_main);
}

/**
 * @brief 构建继电器 3 的报警条件菜单。
 */
static void menu_relay3_alarm(void)
{
    menu_build_by_filter(menu_filter_relay3_alarm, KEYNUM_MENU_RELAY3_ALARM, menu_relay3_main);
}

/**
 * @brief 显示继电器 3 的实时报警状态页。
 */
static void menu_relay3_status(void)
{
    menu_relay_status(2U, KEYNUM_MENU_RELAY3_STATUS, menu_relay3_main);
}

/**
 * @brief 构建继电器 4 的主菜单。
 */
static void menu_relay4_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"通道设置", 0, menu_relay4_channel, COMMANE_NORW, (uint8_t*)"Channel"},
        {(uint8_t*)"报警配置", 0, menu_relay4_alarm,   COMMANE_NORW, (uint8_t*)"Alarm"},
        {(uint8_t*)"报警状态", 0, menu_relay4_status,  COMMANE_NORW, (uint8_t*)"Status"},
        {(uint8_t*)"返回",     0, menu_do_alarm,       COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_RELAY4_MAIN;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 构建继电器 4 的通道配置菜单。
 */
static void menu_relay4_channel(void)
{
    menu_build_by_filter(menu_filter_relay4_channel, KEYNUM_MENU_RELAY4_CHANNEL, menu_relay4_main);
}

/**
 * @brief 构建继电器 4 的报警条件菜单。
 */
static void menu_relay4_alarm(void)
{
    menu_build_by_filter(menu_filter_relay4_alarm, KEYNUM_MENU_RELAY4_ALARM, menu_relay4_main);
}

/**
 * @brief 显示继电器 4 的实时报警状态页。
 */
static void menu_relay4_status(void)
{
    menu_relay_status(3U, KEYNUM_MENU_RELAY4_STATUS, menu_relay4_main);
}

/**
 * @brief AO根菜单固定为五组，电流修正归入量程设置，错误等级与DAC回读保持隐藏。
 */
static void menu_ao(void)
{
	static struct MenuData menu[] = {
		{(uint8_t*)"基本设置", 0, menu_ao_channel,    COMMANE_NORW, (uint8_t*)"Basic"},
		{(uint8_t*)"量程设置", 0, menu_ao_range,      COMMANE_NORW, (uint8_t*)"Range"},
		{(uint8_t*)"故障设置", 0, menu_ao_fault,      COMMANE_NORW, (uint8_t*)"Fault"},
		{(uint8_t*)"运行状态", 0, menu_ao_runtime,    COMMANE_NORW, (uint8_t*)"Runtime"},
		{(uint8_t*)"模拟设置", 0, menu_ao_diagnostic, COMMANE_NORW, (uint8_t*)"Simulation"},
		{(uint8_t*)"返回",     0, menu_output_config, COMMANE_NORW, (uint8_t*)"Back"},
	};

	oled_clear();
	func_index = KEYNUM_MENU_PARA_AO;
	menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief AO基本设置：工作模式、电流模式、输出源。
 */
static void menu_ao_channel(void)
{
	menu_build_by_group(MENU_GRP_AO_CHANNEL, KEYNUM_MENU_AO_CHANNEL, menu_ao);
}

/**
 * @brief AO量程设置：电流修正、固定电流、0%、100%、阻尼。
 */
static void menu_ao_range(void)
{
	menu_build_by_group(MENU_GRP_AO_RANGE, KEYNUM_MENU_AO_RANGE, menu_ao);
}

/**
 * @brief AO故障设置：故障动作、故障电流、初始电流。
 */
static void menu_ao_fault(void)
{
	menu_build_by_group(MENU_GRP_AO_FAULT, KEYNUM_MENU_AO_FAULT, menu_ao);
}

/**
 * @brief 安全格式化AO有符号百分比，避免对INT32_MIN直接取绝对值。
 *
 * @details 调用场景：AO运行状态页显示CPU2快照中的0.01%定点数。
 * @note 关键约束：优先保留有效小数，空间不足时逐级降精度，仍放不下则显示OVER。
 *
 * @param percent_x100 百分比定点值，单位 0.01%。
 * @param max_width 允许文字或百分比占用的最大 OLED 像素宽度。
 * @param text 输入文字或格式化结果缓冲区。该可写输出区容量为 text_size，函数把 0.01% 定点值格式化为 NUL 结尾百分比文字。
 * @param text_size 文字。
 */
static void ao_format_percent_x100(int32_t percent_x100,
								   uint8_t max_width,
								   char *text,
								   size_t text_size)
{
	uint32_t magnitude;
	uint32_t fraction;
	uint8_t points;
	const char *sign;
	int written;

	if ((text == NULL) || (text_size == 0U)) {
		return;
	}

	magnitude = (percent_x100 < 0) ?
	            (uint32_t)(-(int64_t)percent_x100) : (uint32_t)percent_x100;
	fraction = magnitude % 100U;
	sign = (percent_x100 < 0) ? "-" : "";
	points = (fraction == 0U) ? 0U : (((fraction % 10U) == 0U) ? 1U : 2U);

	for (;;) {
		if (points >= 2U) {
			written = snprintf(text,
			                   text_size,
			                   "%s%lu.%02lu%%",
			                   sign,
			                   (unsigned long)(magnitude / 100U),
			                   (unsigned long)fraction);
		} else if (points == 1U) {
			written = snprintf(text,
			                   text_size,
			                   "%s%lu.%01lu%%",
			                   sign,
			                   (unsigned long)(magnitude / 100U),
			                   (unsigned long)(fraction / 10U));
		} else {
			written = snprintf(text,
			                   text_size,
			                   "%s%lu%%",
			                   sign,
			                   (unsigned long)(magnitude / 100U));
		}

		if ((written >= 0) &&
		    ((size_t)written < text_size) &&
		    (oled_text_width((const uint8_t *)text) <= max_width)) {
			return;
		}
		if (points == 0U) {
			break;
		}
		points--;
	}

	(void)snprintf(text, text_size, "OVER");
}

/**
 * @brief AO运行状态从同一份CPU2快照显示过程输入、输入比例和最近成功下发电流。
 *
 * 返回键直接回到 AO 菜单；普通刷新时一次性复制 CPU2 AO 运行态快照，避免同一页面的输出源、过程值、比例和电流来自不同通信时刻。
 * 只有 CPU2 通信可用、AO 处于输出工作模式、来源属于过程量或保持或故障输出且过程值有效时，才显示输入值和比例；否则显示 N/A。
 * 最近成功下发电流只有在输出模式启用、来源不是禁用且缓存值非零时显示，并按 0.001 mA 格式化；输出来源索引非法时统一显示不可用项。
 */
static void menu_ao_runtime(void)
{
	AoOutputRuntime snapshot;
	bool runtime_valid;
	bool process_valid;
	bool current_valid;
	uint32_t source_index;
	char percent_text[16];
	char current_text[16];
	uint8_t line;

	if (NowKeyPress == USE_KEY_BACK) {
		NowKeyPress = 0;
		timeback = 0;
		timesure = 1;
		menu_ao();
		return;
	}

	memcpy(&snapshot, (const void *)&g_measurement.ao_output_runtime, sizeof(snapshot));
	runtime_valid = CPU2_CommIsAvailable();
	source_index = (runtime_valid && (snapshot.source < AO_RUNTIME_SOURCE_COUNT)) ?
	               snapshot.source : AO_RUNTIME_SOURCE_UNAVAILABLE;
	process_valid = runtime_valid &&
	                ao_work_mode_is_output() &&
	                ((snapshot.source == AO_RUNTIME_SOURCE_PROCESS) ||
	                 (snapshot.source == AO_RUNTIME_SOURCE_HOLD_LAST) ||
	                 (snapshot.source == AO_RUNTIME_SOURCE_FAULT)) &&
	                (snapshot.process_valid != 0U);
	current_valid = runtime_valid &&
	                ao_work_mode_is_output() &&
	                (snapshot.source != AO_RUNTIME_SOURCE_DISABLED) &&
	                (snapshot.last_sent_mA_x1000 != 0U);

	oled_clear();
	func_index = KEYNUM_MENU_AO_RUNTIME;
	OledDisplayLineWords(arr_ao_runtime_source[source_index][screen_parameter.language],
	                     OLED_LINE8_1,
	                     OLED_ROW4_1,
	                     0);
	line = DisplayLangaugeLineWords((uint8_t*)"输入值:", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Input:");
	if (process_valid) {
		OledValueDisplay((int)snapshot.process_value_01mm, line, OLED_ROW4_2, 0, 1, (uint8_t*)"mm");
	} else {
		OledDisplayLineWords((uint8_t*)"N/A", line, OLED_ROW4_2, 0);
	}

	line = DisplayLangaugeLineWords((uint8_t*)"输入比例:", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Percent:");
	if (process_valid) {
		ao_format_percent_x100(snapshot.percent_x100,
		                       (uint8_t)((OLED_LINE8_END + 1U) - line),
		                       percent_text,
		                       sizeof(percent_text));
		OledDisplayLineWords((uint8_t*)percent_text, line, OLED_ROW4_3, 0);
	} else {
		OledDisplayLineWords((uint8_t*)"N/A", line, OLED_ROW4_3, 0);
	}

	/* 第4行用于显示输出电流，物理返回键仍由运行状态页按键表处理。 */
	line = DisplayLangaugeLineWords((uint8_t*)"输出电流:", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Output:");
	if (current_valid) {
		(void)snprintf(current_text,
		               sizeof(current_text),
		               "%lu.%03lumA",
		               (unsigned long)(snapshot.last_sent_mA_x1000 / 1000U),
		               (unsigned long)(snapshot.last_sent_mA_x1000 % 1000U));
		OledDisplayLineWords((uint8_t*)current_text, line, OLED_ROW4_4, 0);
	} else {
		OledDisplayLineWords((uint8_t*)"N/A", line, OLED_ROW4_4, 0);
	}
}

/**
 * @brief AO诊断仿真仅显示非持久化仿真开关和已持久化的仿真电流。
 */
static void menu_ao_diagnostic(void)
{
	static struct MenuData menu[] = {
		{(uint8_t*)"输出模拟", COM_NUM_AO_SIMULATION_ENABLE, ao_simulation_switch_enter, COMMANE_NORW, (uint8_t*)"Simulation"},
		{(uint8_t*)"模拟电流", COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100, para_mainprocess, COMMAND_WRITE, (uint8_t*)"Sim Current"},
		{(uint8_t*)"返回", COM_NUM_NOOPERA, menu_ao, COMMANE_NORW, (uint8_t*)"Back"},
	};

	menu[0].rorw = ao_work_mode_is_output() ? COMMAND_WRITE : COMMAND_READ;
	menu[1].rorw = ao_param_is_editable(COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100) ?
	               COMMAND_WRITE : COMMAND_READ;
	oled_clear();
	func_index = KEYNUM_MENU_AO_DIAGNOSTIC;
	menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 进入仿真开关页时以CPU2运行态为唯一当前值。
 */
static void ao_simulation_switch_enter(void)
{
	ao_simulation_selection = (g_measurement.ao_output_runtime.simulation_enabled == 0U) ? 0U : 1U;
	timesure = 0;
	timeback = 0;
	NowKeyPress = 0;
	ao_simulation_switch_page();
}

/**
 * @brief 仿真开关不写持久参数；确认后只写独立保持寄存器。
 */
static void ao_simulation_switch_page(void)
{
	bool editable = ao_work_mode_is_output();
	uint32_t current = (g_measurement.ao_output_runtime.simulation_enabled == 0U) ? 0U : 1U;
	uint8_t line;

	oled_clear();
	func_index = KEYNUM_AO_SIMULATION_SWITCH;
	if (editable && ((NowKeyPress == USE_KEY_UP) || (NowKeyPress == USE_KEY_DOWN))) {
		ao_simulation_selection = (ao_simulation_selection == 0U) ? 1U : 0U;
		timesure = 0;
	} else if (NowKeyPress == USE_KEY_SURE) {
		if (!editable) {
			DisplayLangaugeLineWords((uint8_t*)"当前模式只读", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Readonly Mode");
			HAL_Delay(800);
			ao_simulation_selection = current;
			NowKeyPress = 0;
		} else if (timesure != 0) {
			timesure = 0;
			DisplayLangaugeLineWords((uint8_t*)"正在修改模拟", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Modify Sim");
			if (!ao_write_simulation_enable(ao_simulation_selection)) {
				display_cpu2_comm_failure();
				ao_simulation_selection = current;
				NowKeyPress = 0;
				mainmenu();
				return;
			}
			NowKeyPress = 0;
			menu_ao_diagnostic();
			return;
		} else {
			timesure++;
		}
	}

	DisplayLangaugeLineWords((uint8_t*)"输出模拟", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Simulation");
	line = DisplayLangaugeLineWords((uint8_t*)"当前值:", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Value:");
	OledDisplayLineWords(arr_ao_simulation_enable[current][screen_parameter.language], line, OLED_ROW4_2, 0);
	line = DisplayLangaugeLineWords((uint8_t*)"设置:", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Select:");
	OledDisplayLineWords(arr_ao_simulation_enable[ao_simulation_selection][screen_parameter.language], line, OLED_ROW4_3, editable ? 1U : 0U);
	if (editable) {
		DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
		display_right_aligned_action((uint8_t*)"确认", (uint8_t*)"Ok", OLED_ROW4_4, (timesure != 0));
	} else {
		DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
		display_right_aligned_action((uint8_t*)"只读", (uint8_t*)"Readonly", OLED_ROW4_4, 0);
	}
}

/**
 * @brief 取消 AO 仿真开关编辑并返回 AO 诊断页。
 */
static void ao_simulation_switch_back(void)
{
	NowKeyPress = 0;
	timesure = 0;
	timeback = 0;
	menu_ao_diagnostic();
}
/**
 * @brief 进入标定、单点测量和运动距离参数的兼容菜单。
 * @note 标定、单点和运动距离参数随测量/调试指令输入，不挂入参数配置主菜单；该页仅保留为旧返回映射兜底。
 */
static void menu_cal_sp(void)       { menu_build_by_group(MENU_GRP_CAL_SP,       KEYNUM_MENU_PARA_CAL_SP,       menu_paracfg_main); }

/**
 * @brief 构建参数版本、结构长度、魔术字和 CRC 检查菜单。
 */
static void menu_param_check(void)  { menu_build_by_group(MENU_GRP_PARAM_CHECK, KEYNUM_MENU_PARA_PARAM_CHECK,  menu_maint_config); }

/**
 * @brief 构建 CPU3 本机参数兼容分组页，供旧返回路径兜底。
 *
 * @note 该旧分组页仅用于兼容历史返回路径；现场入口使用显示设置、通信设置的静态菜单和过滤页。
 */
static void menu_cpu3_base(void)    { menu_build_by_group(MENU_GRP_CPU3_BASE,   KEYNUM_MENU_CPU3_BASE,   menu_paracfg_main); }
/**
 * @brief 进入 CPU3 来源配置菜单。
 */
static void menu_cpu3_source(void)  { menu_build_by_group(MENU_GRP_CPU3_SOURCE, KEYNUM_MENU_CPU3_SOURCE, menu_paracfg_main); }
/**
 * @brief 进入 CPU3 手输值配置菜单。
 */
static void menu_cpu3_input(void)   { menu_build_by_group(MENU_GRP_CPU3_INPUT,  KEYNUM_MENU_CPU3_INPUT,  menu_paracfg_main); }
/**
 * @brief 进入 CPU3 屏幕配置菜单。
 */
static void menu_cpu3_screen(void)  { menu_build_by_group(MENU_GRP_CPU3_SCREEN, KEYNUM_MENU_CPU3_SCREEN, menu_paracfg_main); }
/**
 * @brief 构建 CPU3 基础显示参数菜单。
 */
static void menu_display_base(void) { menu_build_by_filter(menu_filter_display_base, KEYNUM_MENU_DISPLAY_BASE, menu_display_config); }
/**
 * @brief 构建油位状态页数据项配置菜单。
 */
static void menu_display_data_oil(void)     { menu_build_by_filter(menu_filter_display_data_oil,     KEYNUM_MENU_DISPLAY_DATA_OIL,     menu_display_data); }
/**
 * @brief 构建水位状态页数据项配置菜单。
 */
static void menu_display_data_water(void)   { menu_build_by_filter(menu_filter_display_data_water,   KEYNUM_MENU_DISPLAY_DATA_WATER,   menu_display_data); }
/**
 * @brief 构建密度状态页数据项配置菜单。
 */
static void menu_display_data_density(void) { menu_build_by_filter(menu_filter_display_data_density, KEYNUM_MENU_DISPLAY_DATA_DENSITY, menu_display_data); }
/**
 * @brief 构建温度状态页数据项配置菜单。
 */
static void menu_display_data_temp(void)    { menu_build_by_filter(menu_filter_display_data_temp,    KEYNUM_MENU_DISPLAY_DATA_TEMP,    menu_display_data); }
/**
 * @brief 构建 CPU3 COM1 协议和串口参数菜单。
 */
static void menu_cpu3_comm1(void)   { menu_build_by_group(MENU_GRP_CPU3_COM1,   KEYNUM_MENU_CPU3_COM1,   menu_comm_config); }
/**
 * @brief 构建 CPU3 COM2 协议和串口参数菜单。
 */
static void menu_cpu3_comm2(void)   { menu_build_by_group(MENU_GRP_CPU3_COM2,   KEYNUM_MENU_CPU3_COM2,   menu_comm_config); }
/**
 * @brief 构建 CPU3 COM3 协议和串口参数菜单。
 */
static void menu_cpu3_comm3(void)   { menu_build_by_group(MENU_GRP_CPU3_COM3,   KEYNUM_MENU_CPU3_COM3,   menu_comm_config); }

/**
 * @brief 构建参数配置主菜单并进入分类选择页。
 */
static void menu_paracfg_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"测量参数",   0, menu_measure_config, COMMANE_NORW, (uint8_t*)"Measure Para"},
        {(uint8_t*)"输出配置",   0, menu_output_config,  COMMANE_NORW, (uint8_t*)"Output"},
        {(uint8_t*)"通信设置",   0, menu_comm_config,    COMMANE_NORW, (uint8_t*)"Comm"},
        {(uint8_t*)"显示设置",   0, menu_display_config, COMMANE_NORW, (uint8_t*)"Display"},
        {(uint8_t*)"维护设置",   0, menu_maint_config,   COMMANE_NORW, (uint8_t*)"Maintain"},
        {(uint8_t*)"返回主菜单", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_PARACFG_MAIN;
    menuselect(menu, (int)(sizeof(menu)/sizeof(menu[0])));
}
