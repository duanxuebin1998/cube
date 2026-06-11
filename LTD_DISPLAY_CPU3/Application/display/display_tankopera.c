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
#include "app_version.h"
#include "system_parameter.h"
#include <string.h>    /* for memset, memcpy, strcmp, strlen... */

#define PASSWORD_ENTERMAIN		1009
extern volatile uint8_t g_cpu3_uart_reinit_pending; /* CPU3 串口重初始化标志 */

typedef void (*pFunc_void)(void);

typedef struct
{
	int menu_num;			/* 菜单第几栏序号(从 0 开始) */
	int menu_cnt;			/* 上下键计数(从 1 开始计数, 便于取模) */
	int menu_page;			/* 当前显示的页数(页起始项) */
} PAGENUM_T;

 PAGENUM_T PageNum[KEYNUM_END]; /* 屏幕菜单操作计数值，用于节拍、统计或协议数量控制。 */
 struct ParaContent now_Para_CT;		/* 当前设置的参数内容 */

static int func_index = 0;					/* 菜单索引 */
static int NowKeyPress = 0;					/* 本次按下的按键 */
static int now_Opera_Num = 0;				/* 当前选择的指令或参数的序号 */
static int timesure = 0;					/* 按下确定键的次数 */
static int timeback = 0;					/* 按下返回键的次数 */
static int debugmode_back = 0;				/* 进入调试模式返回到哪个菜单 */

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

static uint8_t *arr_position_count_mode[][2] = {
	{ (uint8_t*)"编码器", (uint8_t*)"Encoder" },
	{ (uint8_t*)"电机", (uint8_t*)"Motor" },
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
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
	{ (uint8_t*)"SI7000协议", (uint8_t*)"SI7000" }, /* 显示侧只暴露协议选择，具体串口参数由配置归一化自动处理。 */
	{ (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};
static uint8_t *arr_bottom[][2] = {
	{ (uint8_t*)"称重", (uint8_t*)"weight" },
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
    { (uint8_t*)"非法配置",   (uint8_t*)"Illegal CFG" },
};

static uint8_t *water_level_mode[][2] = {
    { (uint8_t*)"低速模式", (uint8_t*)"Slow" },
    { (uint8_t*)"快速模式", (uint8_t*)"Fast" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};


static uint8_t *arr_relay_operating[][2] = {
    { (uint8_t*)"禁用", (uint8_t*)"Disabled" },
    { (uint8_t*)"无源输出", (uint8_t*)"Passive Out" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

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

static uint8_t *arr_relay_contact[][2] = {
    { (uint8_t*)"常开", (uint8_t*)"NO" },
    { (uint8_t*)"常闭", (uint8_t*)"NC" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_relay_alarm_mode[][2] = {
    { (uint8_t*)"关闭", (uint8_t*)"Off" },
    { (uint8_t*)"开启", (uint8_t*)"On" },
    { (uint8_t*)"Latch", (uint8_t*)"Latching" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

static uint8_t *arr_relay_error[][2] = {
    { (uint8_t*)"无报警", (uint8_t*)"No Alarm" },
    { (uint8_t*)"高高/高", (uint8_t*)"HH/H" },
    { (uint8_t*)"高", (uint8_t*)"H" },
    { (uint8_t*)"低", (uint8_t*)"L" },
    { (uint8_t*)"低低/低", (uint8_t*)"LL/L" },
    { (uint8_t*)"全部报警", (uint8_t*)"All" },
    { (uint8_t*)"非法配置", (uint8_t*)"Illegal CFG" },
};

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
/* static void menu_paraconfig(void); / * 参数配置主菜单 * / */
static void menu_cmdconfig_main(void);	/* 调试指令主菜单 */

/* ---------- 2) 参数分组子菜单(参数页面) ----------
 *	各参数分类页面，仅负责“列出参数项 + 跳转到参数读写流程”
 */
/* static void menu_tankbasicpara(void); / * 基础参数 * / */
/* static void menu_weightpara(void); / * 称重/载荷相关参数 * / */
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
static void menu_ao(void)      ;
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
static void menu_cpu3_base(void)   ;
static void menu_cpu3_source(void);
static void menu_cpu3_input(void)  ;
static void menu_cpu3_screen(void)  ;
static void menu_cpu3_comm1(void)   ;
static void menu_cpu3_comm2(void)   ;
static void menu_cpu3_comm3(void)  ;

static void menu_paracfg_main(void);

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

/* ---------- 6) 输入与数值编辑(输入框) ----------
 *	数字逐位输入、符号输入、位数/单位/小数点等显示规则
 */
static void inputcmdpara(void);			/* 输入参数页面(数值/符号) */
static bool inputvalue(uint8_t deci, uint8_t row, uint8_t line,
		uint8_t points, uint8_t *unit, int *value);				/* 多位数字输入状态机 */
static int SignInput(uint8_t row, uint8_t line, uint8_t shift); /* 正负号输入 */

/* ---------- 7) 名称/单位/枚举含义工具函数 ----------
 *	根据 operaNum 或 param_meta 表，返回名字、单位、小数点位数、显示位数等
 */
static uint8_t *dtm_operaname(int num);	/* 根据操作号返回名称(中/英) */
static uint8_t oled_text_width(const uint8_t *name); /* 按 OLED 绘制列宽估算显示长度 */
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

    /* 4 - 参数配置主菜单（新） */
    [KEYNUM_MENU_PARACFG_MAIN] =
        { menu_paracfg_main, menu_paracfg_main, menu_paracfg_main, menu_paracfg_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_paracfg_main },

    /* 5 - 维护/调试指令主菜单 */
    [KEYNUM_MENU_CMD_MAIN] =
        { menu_cmdconfig_main, menu_cmdconfig_main, menu_cmdconfig_main, menu_cmdconfig_main,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cmdconfig_main },

    /* 6 - 是否下发指令或参数 */
    [KEYNUM_IFSENDCMD] =
        { ifsendcmd, NULL, NULL, ifsendcmd,
          USE_KEY_BACK | USE_KEY_SURE, ifsendcmd },

    /* 保护参数额外确认 */
    [KEYNUM_IF_PARAM_PROTECT_CONFIRM] =
        { param_protect_confirm, NULL, NULL, param_protect_confirm,
          USE_KEY_BACK | USE_KEY_SURE, param_protect_confirm },

    /* 7 - 输入参数值(带参指令中的) */
    [KEYNUM_INPUTCMDPARA] =
        { inputcmdpara, inputcmdpara, inputcmdpara, inputcmdpara,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, inputcmdpara },

    /* 8 - 参数显示(读写类参数中的) */
    [KEYNUM_DISPLAY_PARA] =
        { displaypara, NULL, NULL, displaypara,
          USE_KEY_BACK | USE_KEY_SURE, displaypara },

    /* 9 - 隐藏信息选择 */
    [KEYNUM_WORDSELECT] =
        { selectparaword, selectparaword, selectparaword, selectparaword,
          USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, selectparaword },

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

    /* 13 - 称重参数（如果暂无此页，用 menu_dev_info 占位也行） */
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
        { exitTankOpera, NULL, NULL, confirm_cancel_measurement,
          USE_KEY_BACK | USE_KEY_SURE, ifcancelmeasurement },

    /* 故障状态长按返回后的故障原因查看页 */
    [KEYNUM_ERROR_REASON] =
        { exitTankOpera, NULL, NULL, exitTankOpera,
          USE_KEY_BACK | USE_KEY_SURE, Display_ShowErrorReasonPage },
};


/**
 * @brief 处理屏幕菜单操作中的 DisplayTankOpera_CanProcessKey 逻辑。
 *
 * @param keypress 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
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

/* ==============================
 * 按键操作处理
 * ============================== */
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
 * @brief 显示或打印屏幕菜单操作中的 DisplayTankOpera_RedrawCurrentPage 逻辑。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
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

/* 使用了按键 - 更新按键检测定时器(你原来有 Timer1Start, 保留结构) */
void useKey(void)
{
	static int keytimeout = 300;		/* 按键超时时间 */
	(void)keytimeout;
	/* Timer1Start(keytimeout); */
}

/* ==============================
 * 输入参数页面
 * ============================== */
static void inputcmdpara(void)
{
	uint8_t line, row = OLED_ROW4_1;
	int value;
	uint8_t *name = NULL;
	static bool flag_inputsign = false;

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
	if (flag_inputsign == false) {
		if (inputvalue(now_Para_CT.bits, row, line, now_Para_CT.points, now_Para_CT.unit, &value)) {
			now_Para_CT.val = value;

			if (now_Opera_Num == COM_NUM_DEVICEPARAM_DENSITYCORRECTION
				|| now_Opera_Num == COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION) {
				flag_inputsign = true;
				SignInput(row, line - 4, 1);
				return;
			}

			ifsendcmd();
			return;
		}
	} else {
		static int sgn = 0;

		sgn = SignInput(row, line - 4, 1);
		if (sgn != 0) {
			now_Para_CT.val *= sgn;
			sgn = 0;
			flag_inputsign = false;
			ifsendcmd();
			return;
		}
		return;
	}
}

/* 确定小数点位数 */
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
typedef struct {
    int opera;
    uint8_t *name_cn;
    uint8_t *name_en;
} OperaNameMap_t;

/**
 * @brief 执行屏幕菜单操作中的 dtm_operaname 逻辑。
 *
 * @param num 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t *dtm_operaname(int num)
{
    /* 1) 普通无参测量指令（显式映射，避免依赖枚举连续性） */
    static const OperaNameMap_t normal_cmd_map[] = {
        { COM_NUM_BACK_ZERO,           (uint8_t*)"回零点",         (uint8_t*)"Return to Zero" },
        { COM_NUM_FIND_OIL,            (uint8_t*)"寻找液位",       (uint8_t*)"Find Oil Level" },
        { COM_NUM_FIND_WATER,          (uint8_t*)"寻找水位",       (uint8_t*)"Find Water Level" },
        { COM_NUM_FIND_BOTTOM,         (uint8_t*)"寻找罐底",       (uint8_t*)"Find Tank Bottom" },
        { COM_NUM_SYNTHETIC,           (uint8_t*)"综合测量",       (uint8_t*)"Comprehensive-M" },

        { COM_NUM_FOLLOW_WATER,        (uint8_t*)"水位跟随",       (uint8_t*)"Water Follow" },
        { COM_NUM_SPREADPOINTS,        (uint8_t*)"分布测量",       (uint8_t*)"Spread-M" },
        { COM_NUM_SPREADPOINTS_GB,     (uint8_t*)"国标分布测量",   (uint8_t*)"GB Spread-M" },

        { COM_NUM_METER_DENSITY,       (uint8_t*)"每米测量",       (uint8_t*)"DT-PerMeter-M" },
        { COM_NUM_INTERVAL_DENSITY,    (uint8_t*)"区间测量",       (uint8_t*)"Interval-M" },
        { COM_NUM_WARTSILA_DENSITY,    (uint8_t*)"瓦锡兰区间密度", (uint8_t*)"Wartsila Interval-M" },

        { COM_NUM_READ_PART_PARAMS,    (uint8_t*)"读取部件参数",   (uint8_t*)"Read Component Params" },
    };

    /* 2) 无参调试指令（显式映射） */
    static const OperaNameMap_t debug_cmd_map[] = {
        { COM_NUM_FIND_ZERO,           (uint8_t*)"标定零点",       (uint8_t*)"Zero Calibration" },
        { COM_NUM_FORCE_LIFT_ZERO,     (uint8_t*)"强制提零点",     (uint8_t*)"Force Lift Zero" },

        { COM_NUM_SET_EMPTY_WEIGHT,    (uint8_t*)"设置空载称重",   (uint8_t*)"Set Empty Weight" },
        { COM_NUM_SET_FULL_WEIGHT,     (uint8_t*)"设置满载称重",   (uint8_t*)"Set Full Weight" },
        { COM_NUM_RESTOR_EFACTORYSETTING,(uint8_t*)"恢复出厂设置", (uint8_t*)"Factory Reset" },
        { COM_NUM_MAINTENANCE_MODE,    (uint8_t*)"维护模式",       (uint8_t*)"Maintenance Mode" },
        { COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING, (uint8_t*)"匹配无线滑环", (uint8_t*)"Pair Wireless" },
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
    if ((num > COM_NUM_DEBUGCMD_START && num < COM_NUM_DEBUGCMD_STOP) ||
        (num == COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING)) {
        for (int i = 0; i < (int)(sizeof(debug_cmd_map)/sizeof(debug_cmd_map[0])); i++) {
            if (num == debug_cmd_map[i].opera) {
                return (screen_parameter.language == LANGUAGE_CHINESE)
                        ? debug_cmd_map[i].name_cn
                        : debug_cmd_map[i].name_en;
            }
        }
        return returnWordType((uint8_t*)"未知调试指令", (uint8_t*)"Unknown Debug Cmd");
    }

    /* ---------- C) 参数类 & 带参指令：统一走 param_meta ---------- */
    /* 注意：你原逻辑里的区间判断有两个隐患：
       1) (num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_LOCAL_STOP) 已经覆盖了 CPU2+CPU3 参数
       2) (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END) 这个 stop 名字本身不一致，建议你修成 ONEPARA_DEBUGCMD_END/STOP
       这里我保留你的区间语义，但把条件拆清晰一点。
     */
    if ( (num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_LOCAL_STOP) ||
         (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_ONEPARACMD_END) ||
         (num > COM_NUM_ONEPARA_DEBUGCMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END) )
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
/* { (uint8_t*)"设置空载称重", (uint8_t*)"Set Empty Weight" }, */
/* { (uint8_t*)"设置满载称重", (uint8_t*)"Set Full Weight" }, */
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
 * @brief 执行屏幕菜单操作中的 oled_text_width 逻辑。
 *
 * @param name 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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

typedef struct {
	int opera;
	uint8_t *name_cn;
	uint8_t *name_en;
} OperaShortNameMap_t;

/**
 * @brief 执行屏幕菜单操作中的 dtm_operaname_short 逻辑。
 *
 * @param num 业务参数。
 * @param fallback 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t *dtm_operaname_short(int num, uint8_t *fallback)
{
	static const OperaShortNameMap_t short_map[] = {
		{ COM_NUM_FIND_WATER, (uint8_t*)"水位测量", (uint8_t*)"FindWater" },
		{ COM_NUM_SINGLE_POINT, (uint8_t*)"单点测量", (uint8_t*)"SingleMeas" },
		{ COM_NUM_SP_TEST, (uint8_t*)"单点监测", (uint8_t*)"SingleMon" },
		{ COM_NUM_RUN_TO_POSITION, (uint8_t*)"运行到高", (uint8_t*)"RunToPos" },
		{ COM_NUM_SPREADPOINTS_GB, (uint8_t*)"国标分布", (uint8_t*)"GBSpread" },
		{ COM_NUM_METER_DENSITY, (uint8_t*)"每米密度", (uint8_t*)"MeterDens" },
		{ COM_NUM_INTERVAL_DENSITY, (uint8_t*)"区间密度", (uint8_t*)"RangeDens" },
		{ COM_NUM_WARTSILA_DENSITY, (uint8_t*)"瓦锡兰区间", (uint8_t*)"Wartsila" },
		{ COM_NUM_READ_PART_PARAMS, (uint8_t*)"读部件参数", (uint8_t*)"ReadPart" },
		{ COM_NUM_FORCE_LIFT_ZERO, (uint8_t*)"强制提零", (uint8_t*)"ForceZero" },
		{ COM_NUM_SET_EMPTY_WEIGHT, (uint8_t*)"空载称重", (uint8_t*)"EmptyWt" },
		{ COM_NUM_SET_FULL_WEIGHT, (uint8_t*)"满载称重", (uint8_t*)"FullWt" },
		{ COM_NUM_RESTOR_EFACTORYSETTING, (uint8_t*)"恢复出厂", (uint8_t*)"Factory" },
		{ COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING, (uint8_t*)"匹配滑环", (uint8_t*)"PairRing" },
		{ COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION, (uint8_t*)"传感器", (uint8_t*)"SenVer" },
		{ COM_NUM_DEVICEPARAM_SOFTWAREVERSION, (uint8_t*)"C2版本", (uint8_t*)"C2Ver" },
		{ COM_NUM_DEVICEPARAM_MAGIC, (uint8_t*)"魔术", (uint8_t*)"M" },
		{ COM_NUM_DEVICEPARAM_CRC, (uint8_t*)"CRC", (uint8_t*)"CRC" },
		{ COM_NUM_PARA_LOCAL_LEDVERSION, (uint8_t*)"C3版本", (uint8_t*)"C3Ver" },
		{ COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND, (uint8_t*)"上电指令", (uint8_t*)"PwrOnCmd" },
		{ COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO, (uint8_t*)"故障回零", (uint8_t*)"ErrZero" },
		{ COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT, (uint8_t*)"故障停测", (uint8_t*)"ErrStop" },
		{ COM_NUM_DEVICEPARAM_RESERVED2, (uint8_t*)"恢复重跑次数", (uint8_t*)"AutoRecover" },
		{ COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH, (uint8_t*)"位置源切换", (uint8_t*)"PosSwitch" },
		{ COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, (uint8_t*)"编码轮周", (uint8_t*)"EncCirc" },
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
		{ COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE, (uint8_t*)"称重忽略", (uint8_t*)"Ignore" },
		{ COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE, (uint8_t*)"零点偏差", (uint8_t*)"ZeroDev" },
		{ COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE, (uint8_t*)"找零下行", (uint8_t*)"ZeroDown" },
		{ COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF, (uint8_t*)"探头距差", (uint8_t*)"SenDiff" },
		{ COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD, (uint8_t*)"找液阈值", (uint8_t*)"FindLvlTh" },
		{ COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, (uint8_t*)"液位方式", (uint8_t*)"LvlMode" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY, (uint8_t*)"跟随频率", (uint8_t*)"FollowHz" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY, (uint8_t*)"跟随密度", (uint8_t*)"FollowD" },
		{ COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME, (uint8_t*)"滞后时间", (uint8_t*)"HysTime" },
		{ COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE, (uint8_t*)"水位方式", (uint8_t*)"WaterMode" },
		{ COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD, (uint8_t*)"水位跟随阈值", (uint8_t*)"CapTh" },
		{ COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD, (uint8_t*)"水位寻找阈值", (uint8_t*)"FindCap" },
		{ COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE, (uint8_t*)"水位最大下行", (uint8_t*)"MaxDown" },
		{ COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD, (uint8_t*)"水位稳定", (uint8_t*)"WaterStb" },
		{ COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD, (uint8_t*)"滞后电容", (uint8_t*)"LagCap" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE, (uint8_t*)"罐底模式", (uint8_t*)"BotMode" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD, (uint8_t*)"探底角度", (uint8_t*)"AngleTh" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD, (uint8_t*)"探底称重", (uint8_t*)"WeightTh" },
		{ COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG, (uint8_t*)"更新罐高", (uint8_t*)"UpdTank" },
		{ COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION, (uint8_t*)"罐高最大偏差", (uint8_t*)"TankDev" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE, (uint8_t*)"探底后修正", (uint8_t*)"BotFix" },
		{ COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, (uint8_t*)"测罐底", (uint8_t*)"NeedBottom" },
		{ COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT, (uint8_t*)"测水位", (uint8_t*)"NeedWater" },
		{ COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, (uint8_t*)"测单点密度", (uint8_t*)"SingleD" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER, (uint8_t*)"分布顺序", (uint8_t*)"SpreadOrd" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE, (uint8_t*)"分布模式", (uint8_t*)"SpreadMode" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT, (uint8_t*)"分布点数", (uint8_t*)"SpreadCnt" },
		{ COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, (uint8_t*)"分布间距", (uint8_t*)"SpreadDist" },
		{ COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME, (uint8_t*)"点悬停", (uint8_t*)"Hover" },
		{ COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT, (uint8_t*)"区间上限", (uint8_t*)"IntTop" },
		{ COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT, (uint8_t*)"区间下限", (uint8_t*)"IntBottom" },
		{ COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE, (uint8_t*)"最高点距液", (uint8_t*)"MaxHeight" },
		{ COM_NUM_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL, (uint8_t*)"探底间隔", (uint8_t*)"BotInterval" },
		{ COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT, (uint8_t*)"修正罐高", (uint8_t*)"FixTankH" },
		{ COM_NUM_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL, (uint8_t*)"上次修正液位", (uint8_t*)"LastOil" },
		{ COM_NUM_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE, (uint8_t*)"气相温度", (uint8_t*)"GasTemp" },
		{ COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT, (uint8_t*)"尺带伸缩", (uint8_t*)"TapeExp" },
		{ COM_NUM_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE, (uint8_t*)"尺带温度", (uint8_t*)"TapeTemp" },
		{ COM_NUM_DEVICEPARAM_RELAY1_OPERATING_MODE, (uint8_t*)"R1工作模式", (uint8_t*)"R1Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY1_DIGITAL_SOURCE, (uint8_t*)"R1报警组合", (uint8_t*)"R1AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY1_CONTACT_TYPE, (uint8_t*)"R1接点", (uint8_t*)"R1Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ALARM_MODE, (uint8_t*)"R1报警模式", (uint8_t*)"R1AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ERROR_VALUE, (uint8_t*)"R1无效报警", (uint8_t*)"R1Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ALARM_SOURCE, (uint8_t*)"R1报警源", (uint8_t*)"R1Src" },
		{ COM_NUM_DEVICEPARAM_RELAY1_HH_ALARM_VALUE, (uint8_t*)"R1HH值", (uint8_t*)"R1HH" },
		{ COM_NUM_DEVICEPARAM_RELAY1_H_ALARM_VALUE, (uint8_t*)"R1H值", (uint8_t*)"R1H" },
		{ COM_NUM_DEVICEPARAM_RELAY1_L_ALARM_VALUE, (uint8_t*)"R1L值", (uint8_t*)"R1L" },
		{ COM_NUM_DEVICEPARAM_RELAY1_LL_ALARM_VALUE, (uint8_t*)"R1LL值", (uint8_t*)"R1LL" },
		{ COM_NUM_DEVICEPARAM_RELAY1_ALARM_HYSTERESIS, (uint8_t*)"R1滞回", (uint8_t*)"R1Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY1_DAMPING_FACTOR, (uint8_t*)"R1阻尼", (uint8_t*)"R1Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY1_CLEAR_ALARM, (uint8_t*)"R1清锁存", (uint8_t*)"R1Clear" },
		{ COM_NUM_DEVICEPARAM_RELAY2_OPERATING_MODE, (uint8_t*)"R2工作模式", (uint8_t*)"R2Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY2_DIGITAL_SOURCE, (uint8_t*)"R2报警组合", (uint8_t*)"R2AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY2_CONTACT_TYPE, (uint8_t*)"R2接点", (uint8_t*)"R2Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ALARM_MODE, (uint8_t*)"R2报警模式", (uint8_t*)"R2AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ERROR_VALUE, (uint8_t*)"R2无效报警", (uint8_t*)"R2Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ALARM_SOURCE, (uint8_t*)"R2报警源", (uint8_t*)"R2Src" },
		{ COM_NUM_DEVICEPARAM_RELAY2_HH_ALARM_VALUE, (uint8_t*)"R2HH值", (uint8_t*)"R2HH" },
		{ COM_NUM_DEVICEPARAM_RELAY2_H_ALARM_VALUE, (uint8_t*)"R2H值", (uint8_t*)"R2H" },
		{ COM_NUM_DEVICEPARAM_RELAY2_L_ALARM_VALUE, (uint8_t*)"R2L值", (uint8_t*)"R2L" },
		{ COM_NUM_DEVICEPARAM_RELAY2_LL_ALARM_VALUE, (uint8_t*)"R2LL值", (uint8_t*)"R2LL" },
		{ COM_NUM_DEVICEPARAM_RELAY2_ALARM_HYSTERESIS, (uint8_t*)"R2滞回", (uint8_t*)"R2Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY2_DAMPING_FACTOR, (uint8_t*)"R2阻尼", (uint8_t*)"R2Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY2_CLEAR_ALARM, (uint8_t*)"R2清锁存", (uint8_t*)"R2Clear" },
		{ COM_NUM_DEVICEPARAM_RELAY3_OPERATING_MODE, (uint8_t*)"R3工作模式", (uint8_t*)"R3Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY3_DIGITAL_SOURCE, (uint8_t*)"R3报警组合", (uint8_t*)"R3AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY3_CONTACT_TYPE, (uint8_t*)"R3接点", (uint8_t*)"R3Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ALARM_MODE, (uint8_t*)"R3报警模式", (uint8_t*)"R3AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ERROR_VALUE, (uint8_t*)"R3无效报警", (uint8_t*)"R3Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ALARM_SOURCE, (uint8_t*)"R3报警源", (uint8_t*)"R3Src" },
		{ COM_NUM_DEVICEPARAM_RELAY3_HH_ALARM_VALUE, (uint8_t*)"R3HH值", (uint8_t*)"R3HH" },
		{ COM_NUM_DEVICEPARAM_RELAY3_H_ALARM_VALUE, (uint8_t*)"R3H值", (uint8_t*)"R3H" },
		{ COM_NUM_DEVICEPARAM_RELAY3_L_ALARM_VALUE, (uint8_t*)"R3L值", (uint8_t*)"R3L" },
		{ COM_NUM_DEVICEPARAM_RELAY3_LL_ALARM_VALUE, (uint8_t*)"R3LL值", (uint8_t*)"R3LL" },
		{ COM_NUM_DEVICEPARAM_RELAY3_ALARM_HYSTERESIS, (uint8_t*)"R3滞回", (uint8_t*)"R3Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY3_DAMPING_FACTOR, (uint8_t*)"R3阻尼", (uint8_t*)"R3Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY3_CLEAR_ALARM, (uint8_t*)"R3清锁存", (uint8_t*)"R3Clear" },
		{ COM_NUM_DEVICEPARAM_RELAY4_OPERATING_MODE, (uint8_t*)"R4工作模式", (uint8_t*)"R4Mode" },
		{ COM_NUM_DEVICEPARAM_RELAY4_DIGITAL_SOURCE, (uint8_t*)"R4报警组合", (uint8_t*)"R4AlarmSet" },
		{ COM_NUM_DEVICEPARAM_RELAY4_CONTACT_TYPE, (uint8_t*)"R4接点", (uint8_t*)"R4Contact" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ALARM_MODE, (uint8_t*)"R4报警模式", (uint8_t*)"R4AlarmMode" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ERROR_VALUE, (uint8_t*)"R4无效报警", (uint8_t*)"R4Invalid" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ALARM_SOURCE, (uint8_t*)"R4报警源", (uint8_t*)"R4Src" },
		{ COM_NUM_DEVICEPARAM_RELAY4_HH_ALARM_VALUE, (uint8_t*)"R4HH值", (uint8_t*)"R4HH" },
		{ COM_NUM_DEVICEPARAM_RELAY4_H_ALARM_VALUE, (uint8_t*)"R4H值", (uint8_t*)"R4H" },
		{ COM_NUM_DEVICEPARAM_RELAY4_L_ALARM_VALUE, (uint8_t*)"R4L值", (uint8_t*)"R4L" },
		{ COM_NUM_DEVICEPARAM_RELAY4_LL_ALARM_VALUE, (uint8_t*)"R4LL值", (uint8_t*)"R4LL" },
		{ COM_NUM_DEVICEPARAM_RELAY4_ALARM_HYSTERESIS, (uint8_t*)"R4滞回", (uint8_t*)"R4Hys" },
		{ COM_NUM_DEVICEPARAM_RELAY4_DAMPING_FACTOR, (uint8_t*)"R4阻尼", (uint8_t*)"R4Damp" },
		{ COM_NUM_DEVICEPARAM_RELAY4_CLEAR_ALARM, (uint8_t*)"R4清锁存", (uint8_t*)"R4Clear" },
		{ COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA, (uint8_t*)"AO起点电流", (uint8_t*)"AOStart" },
		{ COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA, (uint8_t*)"AO终点电流", (uint8_t*)"AOEnd" },
		{ COM_NUM_DEVICEPARAM_ALARM_HIGH_AO, (uint8_t*)"AO高限报警", (uint8_t*)"AOHighAlarm" },
		{ COM_NUM_DEVICEPARAM_ALARM_LOW_AO, (uint8_t*)"AO低限报警", (uint8_t*)"AOLowAlarm" },
		{ COM_NUM_SCREEN_INPUT_D_SWITCH, (uint8_t*)"上传手输密度", (uint8_t*)"InDSw" },
		{ COM_NUM_SCREEN_DECIMAL, (uint8_t*)"小数点位", (uint8_t*)"Decimal" },
	};

	for (int i = 0; i < (int)(sizeof(short_map) / sizeof(short_map[0])); i++) {
		if (num == short_map[i].opera) {
			return (screen_parameter.language == LANGUAGE_CHINESE) ?
				short_map[i].name_cn : short_map[i].name_en;
		}
	}

	return fallback;
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_name 逻辑。
 *
 * @param item 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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

/* 按 OLED 实际列宽裁剪字符串，避免长英文或长中文名称越过单行右边界。 */
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
 * @brief 检查屏幕菜单操作中的 is_version_value_opera 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 检查屏幕菜单操作中的 is_hex_u32_value_opera 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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

/* 显示 32 位只读值：版本号按 Vx.x.x.x，魔术字/CRC 按 0xXXXXXXXX。 */
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
 * @brief 执行屏幕菜单操作中的 relay_alarm_source_unit 逻辑。
 *
 * @param source 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 显示或打印屏幕菜单操作中的 param_display_unit 逻辑。
 *
 * @param operaNum 业务参数。
 * @param meta 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 显示或打印屏幕菜单操作中的 display_menu_item_with_value 逻辑。
 *
 * @param item 业务参数。
 * @param line 业务参数。
 * @param row 业务参数。
 * @param shift 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 显示或打印屏幕菜单操作中的 display_split_title 逻辑。
 *
 * @param name 业务参数。
 * @param row1 业务参数。
 * @param row2 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 显示或打印屏幕菜单操作中的 display_param_detail_value 逻辑。
 *
 * @param meta 业务参数。
 * @param row 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 显示或打印屏幕菜单操作中的 display_param_detail_range 逻辑。
 *
 * @param meta 业务参数。
 * @param row 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
		line = OledDisplayLineWords((uint8_t*)"~", line, row, 0);
		OledValueDisplay(meta->valuemax, line, row, 0, meta->point, param_display_unit(meta->operanum, meta));
	} else {
		DisplayLangaugeLineWords((uint8_t*)"--", line, row, 0, (uint8_t*)"--");
	}
}

/* 确定单位 */
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

/* 确定显示位数 */
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

/* 输入数据(数值输入状态机) */
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
		line = OledDisplayOneNmb(bit_6, row, line, (nowbit & 1) == 0 && (nowbit | 1) == 7);
	}
	case 6: {
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

	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, !(nowbit & 7), (uint8_t*)"Ok");

	if ((nowbit == (deci - 1)) && (sgl_val == 0)) {
		DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	} else {
		DisplayLangaugeLineWords((uint8_t*)"清零", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Zero out");
	}

	return false;
}

/* 是否下发指令或参数判断页 */
static void ifsendcmd(void)
{
	oled_clear();
	func_index = KEYNUM_IFSENDCMD;

	if ((now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END) ||
        (now_Opera_Num == COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING)) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发指令:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue instruct:");
		display_split_title(dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num)), OLED_ROW4_2, OLED_ROW4_3);
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
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, timesure, (uint8_t*)"Ok");
}

/**
 * @brief 执行屏幕菜单操作中的 operation_needs_protect_confirm 逻辑。
 *
 * @param operaNum 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
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
	case COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION:
	case COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE:
	case COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD:
	case COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD:
	case COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG:
	case COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION:
	case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE:
	case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT:

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
 * @brief 处理屏幕菜单操作中的 protected_operation_process 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void protected_operation_process(void)
{
	if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
		cmd_nopara_process();
		return;
	}

	if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP) {
		cmd_configpara_process();
		return;
	}

	errorprocess();
}

/* 保护参数不增加维护权限，只在真正写入前追加确认，降低误保存风险。 */
static void param_protect_confirm(void)
{
	uint8_t *name;

	oled_clear();
	func_index = KEYNUM_IF_PARAM_PROTECT_CONFIRM;

	DisplayLangaugeLineWords((uint8_t*)"参数保护", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Protected");
	name = dtm_operaname_short(now_Opera_Num, dtm_operaname(now_Opera_Num));
	OledDisplayLineWords(oled_fit_text(name, OLED_LINE8_END), OLED_LINE8_1, OLED_ROW4_2, 0);
	DisplayLangaugeLineWords((uint8_t*)"请确认", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Confirm");

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
				menu_cmdconfig_main();
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
	DisplayLangaugeLineWords((uint8_t*)"确认保存", OLED_LINE8_6, OLED_ROW4_4, timesure, (uint8_t*)"Save");
}

/* 返回按返回键后要跳转的函数指针 */
/* 返回按返回键后要跳转的函数指针 */
/* 返回按返回键后要跳转的函数指针 */
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

    /* 普通无参测量指令 */
    case COM_NUM_BACK_ZERO:
    case COM_NUM_FIND_OIL:
    case COM_NUM_SPREADPOINTS:
    case COM_NUM_FIND_WATER:
    case COM_NUM_FIND_BOTTOM:
    case COM_NUM_SYNTHETIC:
    case COM_NUM_METER_DENSITY:
    case COM_NUM_INTERVAL_DENSITY:
    case COM_NUM_WARTSILA_DENSITY:
    case COM_NUM_FOLLOW_WATER:
    case COM_NUM_SPREADPOINTS_GB:
    case COM_NUM_READ_PART_PARAMS:
        return measuremenu;

    /* 普通带参测量指令 */
    case COM_NUM_SINGLE_POINT:
    case COM_NUM_SP_TEST:
    case COM_NUM_RUN_TO_POSITION:
        return measuremenu;

    /* 调试无参/带参指令入口 */
    case COM_NUM_FIND_ZERO:
    case COM_NUM_FORCE_LIFT_ZERO:
    case COM_NUM_RUNUP:
    case COM_NUM_RUNDOWN:
    case COM_NUM_FORCE_RUNUP:
    case COM_NUM_FORCE_RUNDOWN:
    case COM_NUM_CORRECTION_OIL:
    case COM_NUM_CAL_OIL:
    case COM_NUM_CALIBRATE_WATER:
    case COM_NUM_RESTOR_EFACTORYSETTING:
    case COM_NUM_MAINTENANCE_MODE:
    case COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING:
        return menu_cmdconfig_main;

    /* 获取空载/满载称重：它们属于调试菜单项，返回也应回调试菜单 */
    case COM_NUM_SET_EMPTY_WEIGHT:
    case COM_NUM_SET_FULL_WEIGHT:
        return menu_cmdconfig_main;

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
        case MENU_GRP_DO_ALARM:      p = menu_do_alarm;     break;
        case MENU_GRP_AO:            p = menu_ao;           break;
        case MENU_GRP_CAL_SP:        p = menu_cal_sp;       break;
        case MENU_GRP_PARAM_CHECK:   p = menu_param_check;  break;

        /* CPU3 分组 */
        case MENU_GRP_CPU3_BASE:
        case MENU_GRP_CPU3_SCREEN:
            p = menu_display_base;
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


/* 返回按确认键后要跳转的函数指针 */
static pFunc_void dtm_suretofunc(void)
{
	if ((now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END) ||
        (now_Opera_Num == COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING)) {
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
typedef struct {
    uint32_t opera;
    uint32_t cmd;
} NoParaCmdMap_t;

/**
 * @brief 执行屏幕菜单操作中的 __attribute__ 逻辑。
 *
 * @param opera 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t __attribute__((unused)) is_debug_cmd(uint32_t opera)
{
    return ((opera > COM_NUM_DEBUGCMD_START) && (opera < COM_NUM_DEBUGCMD_STOP)) ||
           (opera == COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING);
}

/* 统一的“调试指令允许条件”判定（按你现有逻辑扩展） */
static uint8_t __attribute__((unused)) debug_cmd_is_allowed(void)
{
    uint32_t st = g_measurement.device_status.device_state;

    /* 你当前对恢复出厂的限制：允许 STANDBY / ERROR / MAINTENANCEMODE
       这里建议把调试类都统一到同一套口径，避免口径不一致 */
    /* 先处理异常边界，避免屏幕菜单操作状态机带故障继续运行。 */
    if ((st == STATE_STANDBY) || (st == STATE_ERROR) || (st == STATE_MAINTENANCEMODE)) {
        return 1;
    }
    return 0;
}

/**
 * @brief 发送屏幕菜单操作中的 send_cpu2_command 逻辑。
 *
 * @param cmd 命令值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void send_cpu2_command(uint32_t cmd)
{
    /* 写 2 个寄存器：如果协议定义为 command 占 32bit，这里保持 2 不动。 */
    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                              HOLDREGISTER_DEVICEPARAM_COMMAND,
                              2,
                              &cmd);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 display_state_can_cancel_measurement 逻辑。
 *
 * @param state 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 显示或打印屏幕菜单操作中的 Display_CanEnterCancelMeasurementConfirm 逻辑。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool Display_CanEnterCancelMeasurementConfirm(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    /* 先处理异常边界，避免屏幕菜单操作状态机带故障继续运行。 */
    if ((state == STATE_ERROR) && (g_measurement.device_status.error_code != NO_ERROR)) {
        return true;
    }

    return display_state_can_cancel_measurement(state) != 0U;
}

/**
 * @brief 显示或打印屏幕菜单操作中的 Display_RequestCancelMeasurement 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Display_RequestCancelMeasurement(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    /* 只在确实有测量/运动过程时下发取消命令，避免待机页误触发。 */
    if (!display_state_can_cancel_measurement(state)) {
        return;
    }

    send_cpu2_command(CMD_CANCEL_MEASUREMENT);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 Display_EnterCancelMeasurementConfirm 逻辑。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool Display_EnterCancelMeasurementConfirm(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    /* 先处理异常边界，避免屏幕菜单操作状态机带故障继续运行。 */
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

/* 不带参线圈指令处理过程 */
static void cmd_nopara_process(void)
{
    static const NoParaCmdMap_t map[] = {
        /* -------- 普通模式：无参测量类 -------- */
        { COM_NUM_BACK_ZERO,          CMD_BACK_ZERO },
        { COM_NUM_FIND_OIL,           CMD_FIND_OIL },
        { COM_NUM_FIND_WATER,         CMD_FIND_WATER },
        { COM_NUM_FIND_BOTTOM,        CMD_FIND_BOTTOM },
        { COM_NUM_SYNTHETIC,          CMD_SYNTHETIC },

        { COM_NUM_FOLLOW_WATER,       CMD_FOLLOW_WATER },              /* 新增 */
        { COM_NUM_SPREADPOINTS,       CMD_MEASURE_DISTRIBUTED },
        { COM_NUM_SPREADPOINTS_GB,    CMD_GB_MEASURE_DISTRIBUTED },     /* 新增 */

        { COM_NUM_METER_DENSITY,      CMD_MEASURE_DENSITY_METER },
        { COM_NUM_INTERVAL_DENSITY,   CMD_MEASURE_DENSITY_RANGE },
        { COM_NUM_WARTSILA_DENSITY,   CMD_WARTSILA_DENSITY_RANGE },

        { COM_NUM_READ_PART_PARAMS,   CMD_READ_PART_PARAMS },          /* 新增 */

        /* -------- 调试模式：无参指令 -------- */
        { COM_NUM_FIND_ZERO,          CMD_CALIBRATE_ZERO },
        { COM_NUM_FORCE_LIFT_ZERO,    CMD_FORCE_LIFT_ZERO },           /* 新增 */

        { COM_NUM_SET_EMPTY_WEIGHT,   CMD_SET_EMPTY_WEIGHT },
        { COM_NUM_SET_FULL_WEIGHT,    CMD_SET_FULL_WEIGHT },
        { COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },
        { COM_NUM_MAINTENANCE_MODE,   CMD_MAINTENANCE_MODE },
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

    /* 调试类命令：统一加权限/状态限制（你也可以只限制“危险指令”子集） */
/* if (is_debug_cmd((uint32_t)now_Opera_Num)) { */
/* if (!debug_cmd_is_allowed()) { */
/* oled_clear(); */
/* DisplayLangaugeLineWords((uint8_t*)"失败", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Failed"); */
/* DisplayLangaugeLineWords((uint8_t*)"请先进入调试模式", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Enter debug mode"); */
/* return; */
/* } */
/* } */

    /* 下发命令 */
    send_cpu2_command(cmd);

    /* ---------- UI 反馈与退出策略（保留你现有行为） ---------- */
    if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在恢复出厂设置", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Factory Settings");
        HAL_Delay(800);
        exitTankOpera();
    } else if (now_Opera_Num == COM_NUM_MAINTENANCE_MODE) {
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"已进入维护模式", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Maintenance Mode");
    } else if (now_Opera_Num == COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING) {
        FlagofTankOpera = false;
        HAL_TIM_Base_Stop_IT(&htim1);
        ClearPageNum();
    } else {
        exitTankOpera();
    }
}

/* 带一参线圈指令处理过程（适配新指令） */
static void cmd_onepara_process(void)
{
    int index;
    int i;

    typedef struct {
        int opera;
        uint32_t cmd;
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

    /* 2) 动态组织要下发的参数字节：rgstcnt 个寄存器 = rgstcnt*2 字节 */
    {
        int bytes = (int)param_meta[index].rgstcnt * 2;
        if (bytes <= 0 || bytes > 64) {
            oled_clear();
            DisplayLangaugeLineWords((uint8_t*)"参数长度异常!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Bad Para Len");
            HAL_Delay(800);
            exitTankOpera();
            return;
        }

        uint8_t paraarr[64];
        memset(paraarr, 0, sizeof(paraarr));

        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在下发参数", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Send Para");

        for (i = 0; i < bytes; i++) {
            paraarr[i] = (uint8_t)((now_Para_CT.val >> (8 * i)) & 0xFF);  /* 你原先的小端拆字节 */
        }

        /* 把 byte 流打包成 uint32_t 数组：每 4 字节一个 uint32_t（小端） */
        uint32_t hold32[16];                 /* 64B => 16 个 uint32_t */
        memset(hold32, 0, sizeof(hold32));

        for (i = 0; i < bytes; i++) {
            hold32[i >> 2] |= ((uint32_t)paraarr[i]) << (8u * (uint32_t)(i & 3));
        }

        CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                                  param_meta[index].startadd,
                                  param_meta[index].rgstcnt,
                                  hold32);
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

        /* 2 个寄存器写入：把 cmd 作为 32bit 写入 command 寄存器 */
        CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                                  HOLDREGISTER_DEVICEPARAM_COMMAND,
                                  2,
                                  (uint32_t *)&cmd);
    }

    exitTankOpera();
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

/* 非法操作处理 */
static void errorprocess(void)
{
	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"非法操作!", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Illegal operation");
	DisplayLangaugeLineWords((uint8_t*)"1s后退出屏幕操作", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Exit after 1 second");
	HAL_Delay(1000);
	exitTankOpera();
}

/* 将0xMMmmppbb版本编码显示为V主.次.修订.构建。 */
static void format_version_u32(uint32_t version, char *buf, size_t buf_size)
{
	snprintf(buf, buf_size, "V%lu.%lu.%lu.%lu",
			(unsigned long)((version >> 24) & 0xFFU),
			(unsigned long)((version >> 16) & 0xFFU),
			(unsigned long)((version >> 8) & 0xFFU),
			(unsigned long)(version & 0xFFU));
}

/* 显示参数内容 */
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

	if (screen_parameter.language == LANGUAGE_CHINESE) {
		OledDisplayLineWords((uint8_t*)"返回  修改        ", OLED_LINE8_1, OLED_ROW4_4, 1);
	} else {
		OledDisplayLineWords((uint8_t*)"Back Alter      ", OLED_LINE8_1, OLED_ROW4_4, 1);
	}
}

/* 判断当前设备状态是否允许修改参数 */
static bool state_allows_param_write(DeviceState state)
{
	/* 这些状态虽然使用 0x80xx 编码，但属于长期运行态，不应按“完成态”放行修改参数。 */
	if (state == STATE_FLOWOIL
		|| state == STATE_FOLLOW_WATERING
		|| state == STATE_SPTESTING) {
		return false;
	}

	return (state == STATE_STANDBY) || ((state & 0x8000U) != 0U);
}

/* 写权限范围检查 */
static void parawritecheck(void)
{
	int index;

	index = getHoldValueNum(now_Opera_Num);
	if (index != -1
		&& param_meta[index].authority_write
		&& state_allows_param_write(g_measurement.device_status.device_state)) {

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

/* 是否进入罐上操作 */
static void ifentermainmenu(void)
{
	oled_clear();
	func_index = KEYNUM_IF_ENTER_MAINMENU;
	DisplayLangaugeLineWords((uint8_t*)"是否进入罐上操作?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Enter operation?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, 0, (uint8_t*)"Ok");
}

/* 是否退出罐上操作 */
static void ifexittankopera(void)
{
	oled_clear();
	func_index = KEYNUM_IF_EXIT_MAINMENU;
	DisplayLangaugeLineWords((uint8_t*)"是否退出罐上操作?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Exit operation?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, 0, (uint8_t*)"Ok");
}

/* 是否取消当前测量 */
static void ifcancelmeasurement(void)
{
	oled_clear();
	func_index = KEYNUM_IF_CANCEL_MEASUREMENT;
	DisplayLangaugeLineWords((uint8_t*)"是否停止测量?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Cancel measure?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, 0, (uint8_t*)"Ok");
}

/* 确认取消测量：确认键触发后直接下发CPU2取消测量命令。 */
static void confirm_cancel_measurement(void)
{
	Display_RequestCancelMeasurement();
	exitTankOpera();
}

/* 进入参数配置前的密码输入操作页 */
static void password_enter_para(void)
{
	now_Opera_Num = COM_NUM_PASSWORD_ENTER_PARA;
	inputcmdpara();
}

/* 进入调试指令前的密码输入操作页 */
static void password_enter_cmd(void)
{
	now_Opera_Num = COM_NUM_PASSWORD_ENTER_CMD;
	inputcmdpara();
}

/* 发送指令获取对应参数的数据 */
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
		/* 先处理异常边界，避免屏幕菜单操作状态机带故障继续运行。 */
		if (cnt_commutoCPU2 >= COMMU_ERROR_MAX) {
			oled_clear();
			DisplayLangaugeLineWords((uint8_t*)"与CPU2通讯故障!", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Cpu2 CF!");
			HAL_Delay(800);
			return -1;
		}
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"正在读取参数", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Reading Para");

		CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
				param_meta[index].startadd,
				param_meta[index].rgstcnt,
				NULL);
		HAL_Delay(150);

		return 0;
	}
}

/* 参数类处理入口 */
static void para_mainprocess(void)
{
	if (get_para_data() == 0) {
		displaypara();
	} else {
		mainmenu();
	}
}

/* 参数范围检查 */
static void parascopecheck(void)
{
	int index;

	index = getHoldValueNum(now_Opera_Num);
	if (index == -1) {
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"非法参数!", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
		HAL_Delay(800);
		displaypara();
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

/* 配置参数过程 */
static void cmd_configpara_process(void)
{
	int i;
	int arrlen = 8;
	static uint8_t paraarr[8];
	int index;

	index = getHoldValueNum(now_Opera_Num);

	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"正在修改参数", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Modify Para");

	if (param_meta[index].data_type == TYPE_FLOAT) {
		union utof tmp_f;
		tmp_f.f = now_Para_CT.val * pow(0.1, (double)param_meta[index].point);
		for (i = 0; i < 4; i++) {
			paraarr[i] = (uint8_t)(tmp_f.u >> (8 * (3 - i)));
		}
	} else if (param_meta[index].data_type == TYPE_DOUBLE) {
		union utod tmp_d;
		tmp_d.d = now_Para_CT.val * pow(0.1, (double)param_meta[index].point);
		for (i = 0; i < 4; i++) {
			paraarr[i] = (uint8_t)(tmp_d.u[1] >> (8 * (3 - i)));
		}
		for (i = 4; i < 8; i++) {
			paraarr[i] = (uint8_t)(tmp_d.u[0] >> (8 * (7 - i)));
		}
	} else {
		now_Para_CT.val -= param_meta[index].offset;
		for (i = 0; i < param_meta[index].rgstcnt * 2 && i < arrlen; i++) {
			paraarr[i] = (uint8_t)((now_Para_CT.val >> (8 * i)) & 0xFF);
		}
	}

	/* 如果是本机参数 */
	if(now_Opera_Num > COM_NUM_PARA_LOCAL_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
	{
	    /* CPU3 本机参数：本地写 + 保存FRAM */
	    Cpu3Local_WriteValue((OperatingNumber)now_Opera_Num, now_Para_CT.val);

	    if (Cpu3Local_IsUartParam((OperatingNumber)now_Opera_Num)) {
	        /* 不在 UI 线程里直接重配，避免与通信收发并发；交给主循环安全点处理 */
	        g_cpu3_uart_reinit_pending = 1;
	    }

	    /* 刷新元数据值供显示 */
	    param_meta[index].val = Cpu3Local_ReadValue((OperatingNumber)now_Opera_Num);
	}
	else /* 下发给CPU2 */
	{
	    uint16_t regs16[32]; /* rgstcnt 最大一般不会很大；32=最多64字节 */
	    int rc = (int)param_meta[index].rgstcnt;


	    memset(regs16, 0, sizeof(regs16));

	    /* paraarr[] 当前的组织方式：
	       - float/double 分支：你是按“高字节在前”的大端字节序写入 paraarr
	       - int 分支：你是按小端（低字节在前）写入 paraarr
	       为了不改变你现有逻辑，这里统一按 paraarr 的“字节顺序”去组 16-bit 寄存器：
	       每个寄存器 = paraarr[2*i] 作为高字节，paraarr[2*i+1] 作为低字节（即网络序/寄存器序）
	    */
	    for (i = 0; i < rc; i++) {
	        uint8_t hi = 0, lo = 0;
	        int p = 2 * i;
	        if (p < arrlen)     hi = paraarr[p + 1];
	        if (p + 1 < arrlen) lo = paraarr[p];
	        regs16[i] = ((uint16_t)hi << 8) | (uint16_t)lo;
	    }

	    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
	                              param_meta[index].startadd,
	                              param_meta[index].rgstcnt,
	                              (uint32_t *)regs16);

	    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
	                              param_meta[index].startadd,
	                              param_meta[index].rgstcnt,
	                              NULL);
	}
	HAL_Delay(800);
	displaypara();
}

/* 菜单选项栏(分页显示) */
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

/* 正负号输入 */
static int SignInput(uint8_t row, uint8_t line, uint8_t shift)
{
	static int sgn = 1;
	static int tosure = -1;
	int ret = 0;

	if (NowKeyPress == USE_KEY_SURE) {
		tosure++;
	} else if (NowKeyPress == USE_KEY_UP) {
		sgn *= -1;
		tosure = 0;
	} else if (NowKeyPress == USE_KEY_DOWN) {
		sgn *= -1;
		tosure = 0;
	} else {
		sgn = 1;
		tosure = 0;
	}

	if (sgn == -1) {
		line = OledDisplayLineWords((u8*)"-", line, row, shift);
	} else {
		line = OledDisplayLineWords((u8*)"+", line, row, shift);
	}

	OledValueDisplay(now_Para_CT.val, line, row, 0, now_Para_CT.points, now_Para_CT.unit);

	if (tosure > 1) {
		ret = sgn;
	}

	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, tosure > 0, (uint8_t*)"Ok");

	return ret;
}

/**
 * @brief 执行屏幕菜单操作中的 opera_is_com_protocol 逻辑。
 *
 * @param operaNum 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
static bool opera_is_com_protocol(int operaNum)
{
	return (operaNum == COM_NUM_CPU3_COM1_PROTOCOL)
		|| (operaNum == COM_NUM_CPU3_COM2_PROTOCOL)
		|| (operaNum == COM_NUM_CPU3_COM3_PROTOCOL);
}

/**
 * @brief 执行屏幕菜单操作中的 protocol_value_to_selection_index 逻辑。
 *
 * @param value 待处理数值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
	case COM_PROTO_SI7000:
		return 3;
	default:
		return 0;
	}
}

/**
 * @brief 执行屏幕菜单操作中的 selection_index_to_value 逻辑。
 *
 * @param operaNum 业务参数。
 * @param selectedIndex 索引值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int selection_index_to_value(int operaNum, int selectedIndex)
{
	static const int protocol_values[] = {
		COM_PROTO_DSM,
		COM_PROTO_WARTSILA,
		COM_PROTO_LTD,
		COM_PROTO_SI7000,
	};

	if (opera_is_com_protocol(operaNum)) {
		if (selectedIndex < 0 || selectedIndex >= (int)(sizeof(protocol_values) / sizeof(protocol_values[0]))) {
			return COM_PROTO_DSM;
		}
		return protocol_values[selectedIndex];
	}

	return selectedIndex;
}

/* 返回通讯方式文字信息 */
uint8_t *ret_arr_word(void)
{
	int index, len;
	uint8_t *(*p)[2];

	p = dtm_disarr(&index, &len);
	if (p != NULL && index >= 0 && index < len) {
		return p[index][screen_parameter.language];
	} else {
		return (uint8_t*)"非法配置";
	}
}

/* 决定显示哪个数组里的文字 */
uint8_t *(*dtm_disarr(int *pindex, int *plen))[2]
{
	int index = 0, len = -1;
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
	case COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_position_count_mode) / sizeof(arr_position_count_mode[0]));
		p = arr_position_count_mode;
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


/* 显示要选择的信息 */
static void selectparaword(void)
{
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

/* 隐藏信息含义选择栏 */
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
 * @brief 清除或复位屏幕菜单操作中的 ClearPageNum 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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

/* 主菜单 */
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
/* 普通测量指令菜单 */
static void measuremenu(void)
{
    static struct MenuData menu[] = {

        /* ===== 基础动作 ===== */
        { (uint8_t*)"提零点",     COM_NUM_BACK_ZERO,     ifsendcmd,   COMMANE_NORW, (uint8_t*)"BackZero"   },
        { (uint8_t*)"液位测量",   COM_NUM_FIND_OIL,      ifsendcmd,   COMMANE_NORW, (uint8_t*)"FindOil"    },
        { (uint8_t*)"水位单次测量",   COM_NUM_FIND_WATER,    ifsendcmd,   COMMANE_NORW, (uint8_t*)"FindWater"  },
        { (uint8_t*)"罐高测量",   COM_NUM_FIND_BOTTOM,   ifsendcmd,   COMMANE_NORW, (uint8_t*)"FindBottom" },

        /* ===== 单点/监测/综合 ===== */
        { (uint8_t*)"密度单点测量",   COM_NUM_SINGLE_POINT,  inputcmdpara,COMMANE_NORW, (uint8_t*)"SingleMeasure" },
        { (uint8_t*)"密度单点监测",   COM_NUM_SP_TEST,       inputcmdpara,COMMANE_NORW, (uint8_t*)"SingleMonitor" },
        { (uint8_t*)"综合测量",   COM_NUM_SYNTHETIC,     ifsendcmd,   COMMANE_NORW, (uint8_t*)"Synthetic"     },

        /* ===== 跟随/运动控制（新增）===== */
        { (uint8_t*)"水位跟随",   COM_NUM_FOLLOW_WATER,  ifsendcmd,   COMMANE_NORW, (uint8_t*)"FollowWater"   },
        { (uint8_t*)"浮子运行到高度", COM_NUM_RUN_TO_POSITION,inputcmdpara,COMMANE_NORW, (uint8_t*)"RunToPos"      },

        /* ===== 分布/密度系列 ===== */
        { (uint8_t*)"分布测量",     COM_NUM_SPREADPOINTS,     ifsendcmd, COMMANE_NORW, (uint8_t*)"DistMeasure"     },
        { (uint8_t*)"国标分布测量", COM_NUM_SPREADPOINTS_GB,  ifsendcmd, COMMANE_NORW, (uint8_t*)"GB_DistMeasure"  },

        { (uint8_t*)"密度每米测量", COM_NUM_METER_DENSITY,    ifsendcmd, COMMANE_NORW, (uint8_t*)"MeterDensity"    },
        { (uint8_t*)"区间密度测量", COM_NUM_INTERVAL_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"RangeDensity"    },
        { (uint8_t*)"瓦锡兰区间密度", COM_NUM_WARTSILA_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"WartsilaRange"  },

        /* ===== 读取类（新增）===== */
        { (uint8_t*)"读取部件参数", COM_NUM_READ_PART_PARAMS, ifsendcmd, COMMANE_NORW, (uint8_t*)"ReadPartParams" },

        /* ===== 退出 ===== */
        { (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
    };

    int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

    oled_clear();
    func_index = KEYNUM_MEASURE_MAINMENU;
    menuselect(menu, menulen);
}


/* 菜单 - 调试指令 */
static void menu_cmdconfig_main(void)
{
    static struct MenuData menu[] = {

        /* ===== 运动控制 ===== */
        { (uint8_t*)"上行",       COM_NUM_RUNUP,        inputcmdpara, COMMANE_NORW, (uint8_t*)"MoveUp"       },
        { (uint8_t*)"下行",       COM_NUM_RUNDOWN,      inputcmdpara, COMMANE_NORW, (uint8_t*)"MoveDown"     },
        { (uint8_t*)"强制上行",   COM_NUM_FORCE_RUNUP,  inputcmdpara, COMMANE_NORW, (uint8_t*)"ForceMoveUp"  },
        { (uint8_t*)"强制下行",   COM_NUM_FORCE_RUNDOWN,inputcmdpara, COMMANE_NORW, (uint8_t*)"ForceMoveDown"},
        { (uint8_t*)"强制提零点", COM_NUM_FORCE_LIFT_ZERO, ifsendcmd, COMMANE_NORW, (uint8_t*)"ForceLiftZero"},

        /* ===== 标定/修正 ===== */
        { (uint8_t*)"标定零点",   COM_NUM_FIND_ZERO,    ifsendcmd,    COMMANE_NORW, (uint8_t*)"CalZero"      },
        { (uint8_t*)"标定液位", COM_NUM_CAL_OIL,     inputcmdpara, COMMANE_NORW, (uint8_t*)"CalOil"       },
        { (uint8_t*)"修正液位",   COM_NUM_CORRECTION_OIL,inputcmdpara, COMMANE_NORW, (uint8_t*)"CorrectOil"   },
        { (uint8_t*)"标定水位",   COM_NUM_CALIBRATE_WATER, inputcmdpara, COMMANE_NORW, (uint8_t*)"CalWater"    },
        { (uint8_t*)"标定罐高",   COM_NUM_CALIBRATE_TANKHEIGHT, inputcmdpara, COMMANE_NORW, (uint8_t*)"CalTankH" },

        /* ===== 称重相关 ===== */
        { (uint8_t*)"获取空载称重", COM_NUM_SET_EMPTY_WEIGHT, ifsendcmd, COMMANE_NORW, (uint8_t*)"SetEmptyWeight" },
        { (uint8_t*)"获取满载称重", COM_NUM_SET_FULL_WEIGHT,  ifsendcmd, COMMANE_NORW, (uint8_t*)"SetFullWeight"  },

        /* ===== 系统/维护 ===== */
        { (uint8_t*)"恢复出厂设置", COM_NUM_RESTOR_EFACTORYSETTING, ifsendcmd, COMMANE_NORW, (uint8_t*)"RestoreFactory" },
        { (uint8_t*)"进入维护模式",     COM_NUM_MAINTENANCE_MODE,       ifsendcmd, COMMANE_NORW, (uint8_t*)"Maintenance"    },
        { (uint8_t*)"匹配无线滑环", COM_NUM_PAIR_NEAREST_WIRELESS_SLIPRING, ifsendcmd, COMMANE_NORW, (uint8_t*)"PairWireless" },

        /* ===== 退出 ===== */
        { (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
    };

    int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

    oled_clear();
    func_index = KEYNUM_MENU_CMD_MAIN;
    debugmode_back = 1;
    menuselect(menu, menulen);
}


/* 设置语言 */
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
 * @brief 写入或设置屏幕菜单操作中的 setchinese 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void setchinese(void)
{
	screen_parameter.language = LANGUAGE_CHINESE;
	mainmenu();
}

/**
 * @brief 写入或设置屏幕菜单操作中的 setenglish 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void setenglish(void)
{
	screen_parameter.language = LANGUAGE_ENGLISH;
	mainmenu();
}



/* 退出罐上操作 */
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



/* -------------------- 可选：过滤“保留项” --------------------
 * 说明：
 * - 如果你的字符串是 GBK： "保留" 通常 2 个汉字 4 字节；UTF-8 是 6 字节
 * - 为避免编码差异，这里用 strncmp("保留",2) 做弱判断；若你发现无效，按编码改成 memcmp。
 */
static int is_reserved_cn(const uint8_t *name)
{
    if (name == NULL) return 0;
    return (strncmp((const char*)name, "保留", 2) == 0);
}

/* 每路继电器报警输出参数固定为 13 个字段。
 * 这里集中识别通道和字段，避免菜单和枚举文字显示各自写裸范围判断。 */
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
 * @brief 执行屏幕菜单操作中的 RelayParam_FieldOf 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 执行屏幕菜单操作中的 RelayParam_IsConfig 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int RelayParam_IsConfig(int operaNum)
{
    int field = RelayParam_FieldOf(operaNum);

    return (field >= 0) && (field < (int)RELAY_ALARM_FIELD_COUNT);
}

/**
 * @brief 执行屏幕菜单操作中的 RelayParam_IsChannelSetting 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 执行屏幕菜单操作中的 RelayParam_IsAlarmCondition 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int RelayParam_IsAlarmCondition(int operaNum)
{
    int field = RelayParam_FieldOf(operaNum);

    return (field >= 3) && (field <= 11);
}

/**
 * @brief 执行屏幕菜单操作中的 RelayParam_IsAlarmValueField 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int RelayParam_IsAlarmValueField(int operaNum)
{
    int field = RelayParam_FieldOf(operaNum);

    return (field >= 6) && (field <= 10);
}

typedef struct {
    uint8_t *name_cn;
    uint8_t *name_en;
} RelayStatusFieldName;

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
};

/**
 * @brief 执行屏幕菜单操作中的 relay_alarm_state_word 逻辑。
 *
 * @param state 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 清除或复位屏幕菜单操作中的 relay_clear_state_word 逻辑。
 *
 * @param state 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 执行屏幕菜单操作中的 relay_status_state_of 逻辑。
 *
 * @param state 状态值。
 * @param field 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t relay_status_state_of(const volatile RelayAlarmRuntimeState *state, int field)
{
    if (state == NULL) {
        return RELAY_ALARM_STATE_INACTIVE;
    }

    switch (field) {
    case 1:
        return state->HH_alarm;
    case 2:
        return state->H_alarm;
    case 3:
        return state->HH_H_alarm;
    case 4:
        return state->L_alarm;
    case 5:
        return state->LL_alarm;
    case 6:
        return state->LL_L_alarm;
    case 7:
        return state->any_error;
    default:
        return RELAY_ALARM_STATE_INACTIVE;
    }
}

/**
 * @brief 执行屏幕菜单操作中的 relay_status_alarm_value_x10 逻辑。
 *
 * @param value 待处理数值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int relay_status_alarm_value_x10(float value)
{
    float scaled = value * 10.0f;

    return (scaled >= 0.0f) ? (int)(scaled + 0.5f) : (int)(scaled - 0.5f);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 display_relay_status_row 逻辑。
 *
 * @param state 状态值。
 * @param field 业务参数。
 * @param row 业务参数。
 * @param shift 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void display_relay_status_row(const volatile RelayAlarmRuntimeState *state, int field, uint8_t row, uint8_t shift)
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

    if (field == 0) {
        OledValueDisplay(relay_status_alarm_value_x10((state != NULL) ? state->alarm_value : 0.0f),
                         line,
                         row,
                         shift,
                         1,
                         NULL);
    } else if (field == 8) {
        OledDisplayLineWords(relay_clear_state_word((state != NULL) ? state->clear_alarm : RELAY_ALARM_CLEAR_NO),
                             line,
                             row,
                             shift);
    } else {
        OledDisplayLineWords(relay_alarm_state_word(relay_status_state_of(state, field)), line, row, shift);
    }
}

/* 显示单路继电器报警运行态，只读消费 CPU2 输入寄存器快照，不触发参数下发。 */
static void menu_relay_status(uint32_t channel, keymenuNumber keynum, pFunc_void backfunc)
{
    enum { RELAY_STATUS_FIELD_COUNT = 9, RELAY_STATUS_ROWS = 2 };
    volatile RelayAlarmRuntimeState *state;
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

    snprintf(title, sizeof(title), "R%lu%s",
             (unsigned long)(channel + 1U),
             (screen_parameter.language == LANGUAGE_CHINESE) ? "报警状态" : " Alarm");
    OledDisplayLineWords((uint8_t*)title, OLED_LINE8_1, OLED_ROW4_1, 0);

    state = (channel < RELAY_ALARM_CHANNEL_COUNT) ? &g_measurement.relay_alarm_runtime[channel] : NULL;
    for (i = 0; (i < RELAY_STATUS_ROWS) && ((first + i) < RELAY_STATUS_FIELD_COUNT); i++) {
        selected = first + i;
        display_relay_status_row(state,
                                 selected,
                                 (uint8_t)(OLED_ROW4_2 + (i * OLED_ROW4_2)),
                                 (selected == PageNum[keynum].menu_num) ? 1U : 0U);
    }

    DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
}

/* -------------------- 分组映射（只维护这个即可） -------------------- */
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
    case COM_NUM_DEVICEPARAM_PROTOCOL_VERSION:
        return MENU_GRP_DEV_INFO;

    /* 运行策略 */
    case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:
    case COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO:
    case COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT:
    case COM_NUM_DEVICEPARAM_RESERVED2:
    case COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH:
        return MENU_GRP_RUN_POLICY;

    /* 机械/电机/编码器 */
    case COM_NUM_DEVICEPARAM_MOTOR_CURRENT:
    case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
    case COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED:
    case COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM:
    case COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM:
    case COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE:
    case COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC:
        return MENU_GRP_MECH;

    /* 称重 */
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

    /* AO */
    case COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA:
    case COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA:
    case COM_NUM_DEVICEPARAM_ALARM_HIGH_AO:
    case COM_NUM_DEVICEPARAM_ALARM_LOW_AO:
    case COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA:
    case COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA:
    case COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA:
    case COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA:
    case COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA:
        return MENU_GRP_AO;

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
    case COM_NUM_PARA_LOCAL_LEDVERSION:
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

    default:
        /* 未分类项：避免丢失，统一放到“校验信息”或“基础信息”都可以 */
        return MENU_GRP_DEV_INFO;
    }
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
        if (ParamGroupOf(m->operanum) != grp) continue;

        /* 不展示“设备指令寄存器” */
        if (m->operanum == COM_NUM_DEVICEPARAM_COMMAND) continue;

        /* 过滤保留项 */
        if (is_reserved_cn(m->name)) continue;

        if (menulen >= AUTO_MENU_MAX_ITEMS) break;

        menu[menulen].operaName  = m->name;
        menu[menulen].operaNum   = m->operanum;
        menu[menulen].sureopera  = para_mainprocess;

        /* 关键：用你的 authority_write 来决定读/写 */
        menu[menulen].rorw = (m->authority_write) ? COMMAND_WRITE : COMMAND_READ;

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
 * @brief 执行屏幕菜单操作中的 menu_build_by_filter 逻辑。
 *
 * @param filter 业务参数。
 * @param key_index 索引值。
 * @param backFunc 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_filter_relay1_channel 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay1_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 0) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay1_alarm 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay1_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 0) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay2_channel 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay2_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 1) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay2_alarm 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay2_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 1) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay3_channel 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay3_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 2) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay3_alarm 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay3_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 2) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay4_channel 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay4_channel(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 3) && RelayParam_IsChannelSetting(operaNum);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_filter_relay4_alarm 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_relay4_alarm(int operaNum)
{
    return (RelayParam_ChannelOf(operaNum) == 3) && RelayParam_IsAlarmCondition(operaNum);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_filter_display_base 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_display_base(int operaNum)
{
    switch (operaNum) {
    case COM_NUM_PARA_LOCAL_LEDVERSION:
    case COM_NUM_PARA_LANG:
    case COM_NUM_SCREEN_DECIMAL:
    case COM_NUM_SCREEN_PASSWARD:
    case COM_NUM_SCREEN_OFF:
    case COM_NUM_SCREEN_BRIGHTNESS:
        return 1;
    default:
        return 0;
    }
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_filter_display_data_oil 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_display_data_oil(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_OIL) || (operaNum == COM_NUM_SCREEN_INPUT_OIL);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_filter_display_data_water 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_display_data_water(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_WATER) || (operaNum == COM_NUM_SCREEN_INPUT_WATER);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_filter_display_data_density 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_display_data_density(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_D)
        || (operaNum == COM_NUM_SCREEN_INPUT_D)
        || (operaNum == COM_NUM_SCREEN_INPUT_D_SWITCH);
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_filter_display_data_temp 逻辑。
 *
 * @param operaNum 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int menu_filter_display_data_temp(int operaNum)
{
    return (operaNum == COM_NUM_SCREEN_SOURCE_T) || (operaNum == COM_NUM_SCREEN_INPUT_T);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_measure_config 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_measure_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"运行设置",      0, menu_run_policy,   COMMANE_NORW, (uint8_t*)"Run Policy"},
        {(uint8_t*)"电机与编码参数",0, menu_mech,         COMMANE_NORW, (uint8_t*)"Mechanism"},
        {(uint8_t*)"称重参数",      0, menu_weight,       COMMANE_NORW, (uint8_t*)"Weight"},
        {(uint8_t*)"零点参数",      0, menu_zero,         COMMANE_NORW, (uint8_t*)"Zero"},
        {(uint8_t*)"液位参数",      0, menu_liquid,       COMMANE_NORW, (uint8_t*)"Level"},
        {(uint8_t*)"水位参数",      0, menu_water,        COMMANE_NORW, (uint8_t*)"Water"},
        {(uint8_t*)"罐底与罐高",    0, menu_bottom_tankh, COMMANE_NORW, (uint8_t*)"Bottom/TankH"},
        {(uint8_t*)"密度测量参数",  0, menu_policy,       COMMANE_NORW, (uint8_t*)"Density"},
        {(uint8_t*)"Wartsila参数",  0, menu_wartsila,     COMMANE_NORW, (uint8_t*)"Wartsila"},
        {(uint8_t*)"修正参数",      0, menu_correct,      COMMANE_NORW, (uint8_t*)"Correction"},
        {(uint8_t*)"返回",          0, menu_paracfg_main, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_PARA_MEASURE_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 执行屏幕菜单操作中的 menu_comm_config 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 显示或打印屏幕菜单操作中的 menu_display_config 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_display_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"显示基础",      0, menu_display_base,  COMMANE_NORW, (uint8_t*)"Display"},
        {(uint8_t*)"数据源与手输值",0, menu_display_data,  COMMANE_NORW, (uint8_t*)"Data Source"},
        {(uint8_t*)"返回",          0, menu_paracfg_main, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_DISPLAY_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_data 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_maint_config 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_maint_config(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"设备信息", 0, menu_dev_info,     COMMANE_NORW, (uint8_t*)"Info"},
        {(uint8_t*)"参数校验", 0, menu_param_check,  COMMANE_NORW, (uint8_t*)"Check"},
        {(uint8_t*)"返回",     0, menu_paracfg_main, COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_MAINT_CONFIG;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/* CPU2：分组页 = 参数列表页（取消 DEBUG 容器页） */
static void menu_run_policy(void)   { menu_build_by_group(MENU_GRP_RUN_POLICY,   KEYNUM_MENU_PARA_RUN_POLICY,   menu_measure_config); }
/* * @brief 进入设备信息参数分组菜单。 */
static void menu_dev_info(void)     { menu_build_by_group(MENU_GRP_DEV_INFO,     KEYNUM_MENU_PARA_DEV_INFO,     menu_maint_config); }
/* * @brief 进入机械参数分组菜单。 */
static void menu_mech(void)         { menu_build_by_group(MENU_GRP_MECH,         KEYNUM_MENU_PARA_MECH,         menu_measure_config); }
/* * @brief 进入称重参数分组菜单。 */
static void menu_weight(void)       { menu_build_by_group(MENU_GRP_WEIGHT,       KEYNUM_MENU_PARA_WEIGHT,       menu_measure_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_zero 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_zero(void)         { menu_build_by_group(MENU_GRP_ZERO,         KEYNUM_MENU_PARA_ZERO,         menu_measure_config); }

/**
 * @brief 执行屏幕菜单操作中的 menu_liquid 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_liquid(void)       { menu_build_by_group(MENU_GRP_LIQUID,       KEYNUM_MENU_PARA_LIQUID,       menu_measure_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_water 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_water(void)        { menu_build_by_group(MENU_GRP_WATER,        KEYNUM_MENU_PARA_WATER,        menu_measure_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_bottom_tankh 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_bottom_tankh(void) { menu_build_by_group(MENU_GRP_BOTTOM_TANKH, KEYNUM_MENU_PARA_BOTTOM_TANKH, menu_measure_config); }

/**
 * @brief 执行屏幕菜单操作中的 menu_correct 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_correct(void)      { menu_build_by_group(MENU_GRP_CORR,         KEYNUM_MENU_PARA_CORR,         menu_measure_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_policy 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_policy(void)       { menu_build_by_group(MENU_GRP_POLICY,       KEYNUM_MENU_PARA_POLICY,       menu_measure_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_wartsila 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_wartsila(void)     { menu_build_by_group(MENU_GRP_WARTSILA,     KEYNUM_MENU_PARA_WARTSILA,     menu_measure_config); }

/**
 * @brief 执行屏幕菜单操作中的 menu_output_config 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_do_alarm 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_do_alarm(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"R1继电器", 0, menu_relay1_main,  COMMANE_NORW, (uint8_t*)"Relay1"},
        {(uint8_t*)"R2继电器", 0, menu_relay2_main,  COMMANE_NORW, (uint8_t*)"Relay2"},
        {(uint8_t*)"R3继电器", 0, menu_relay3_main,  COMMANE_NORW, (uint8_t*)"Relay3"},
        {(uint8_t*)"R4继电器", 0, menu_relay4_main,  COMMANE_NORW, (uint8_t*)"Relay4"},
        {(uint8_t*)"返回",     0, menu_output_config,COMMANE_NORW, (uint8_t*)"Back"},
    };

    oled_clear();
    func_index = KEYNUM_MENU_PARA_DO;
    menuselect(menu, (int)(sizeof(menu) / sizeof(menu[0])));
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay1_main 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_relay1_channel 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay1_channel(void)
{
    menu_build_by_filter(menu_filter_relay1_channel, KEYNUM_MENU_RELAY1_CHANNEL, menu_relay1_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay1_alarm 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay1_alarm(void)
{
    menu_build_by_filter(menu_filter_relay1_alarm, KEYNUM_MENU_RELAY1_ALARM, menu_relay1_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay1_status 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay1_status(void)
{
    menu_relay_status(0U, KEYNUM_MENU_RELAY1_STATUS, menu_relay1_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay2_main 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_relay2_channel 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay2_channel(void)
{
    menu_build_by_filter(menu_filter_relay2_channel, KEYNUM_MENU_RELAY2_CHANNEL, menu_relay2_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay2_alarm 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay2_alarm(void)
{
    menu_build_by_filter(menu_filter_relay2_alarm, KEYNUM_MENU_RELAY2_ALARM, menu_relay2_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay2_status 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay2_status(void)
{
    menu_relay_status(1U, KEYNUM_MENU_RELAY2_STATUS, menu_relay2_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay3_main 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_relay3_channel 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay3_channel(void)
{
    menu_build_by_filter(menu_filter_relay3_channel, KEYNUM_MENU_RELAY3_CHANNEL, menu_relay3_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay3_alarm 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay3_alarm(void)
{
    menu_build_by_filter(menu_filter_relay3_alarm, KEYNUM_MENU_RELAY3_ALARM, menu_relay3_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay3_status 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay3_status(void)
{
    menu_relay_status(2U, KEYNUM_MENU_RELAY3_STATUS, menu_relay3_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay4_main 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行屏幕菜单操作中的 menu_relay4_channel 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay4_channel(void)
{
    menu_build_by_filter(menu_filter_relay4_channel, KEYNUM_MENU_RELAY4_CHANNEL, menu_relay4_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay4_alarm 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay4_alarm(void)
{
    menu_build_by_filter(menu_filter_relay4_alarm, KEYNUM_MENU_RELAY4_ALARM, menu_relay4_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_relay4_status 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_relay4_status(void)
{
    menu_relay_status(3U, KEYNUM_MENU_RELAY4_STATUS, menu_relay4_main);
}

/**
 * @brief 执行屏幕菜单操作中的 menu_ao 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_ao(void)           { menu_build_by_group(MENU_GRP_AO,           KEYNUM_MENU_PARA_AO,           menu_output_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_cal_sp 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_cal_sp(void)       { menu_build_by_group(MENU_GRP_CAL_SP,       KEYNUM_MENU_PARA_CAL_SP,       menu_paracfg_main); }

/**
 * @brief 检查屏幕菜单操作中的 menu_param_check 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_param_check(void)  { menu_build_by_group(MENU_GRP_PARAM_CHECK, KEYNUM_MENU_PARA_PARAM_CHECK,  menu_maint_config); }

/* CPU3：同理，分组页 = 参数列表页 */
static void menu_cpu3_base(void)    { menu_build_by_group(MENU_GRP_CPU3_BASE,   KEYNUM_MENU_CPU3_BASE,   menu_paracfg_main); }
/* * @brief 进入 CPU3 来源配置菜单。 */
static void menu_cpu3_source(void)  { menu_build_by_group(MENU_GRP_CPU3_SOURCE, KEYNUM_MENU_CPU3_SOURCE, menu_paracfg_main); }
/* * @brief 进入 CPU3 手输值配置菜单。 */
static void menu_cpu3_input(void)   { menu_build_by_group(MENU_GRP_CPU3_INPUT,  KEYNUM_MENU_CPU3_INPUT,  menu_paracfg_main); }
/* * @brief 进入 CPU3 屏幕配置菜单。 */
static void menu_cpu3_screen(void)  { menu_build_by_group(MENU_GRP_CPU3_SCREEN, KEYNUM_MENU_CPU3_SCREEN, menu_paracfg_main); }
/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_base 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_display_base(void) { menu_build_by_filter(menu_filter_display_base, KEYNUM_MENU_DISPLAY_BASE, menu_display_config); }
/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_data_oil 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_display_data_oil(void)     { menu_build_by_filter(menu_filter_display_data_oil,     KEYNUM_MENU_DISPLAY_DATA_OIL,     menu_display_data); }
/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_data_water 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_display_data_water(void)   { menu_build_by_filter(menu_filter_display_data_water,   KEYNUM_MENU_DISPLAY_DATA_WATER,   menu_display_data); }
/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_data_density 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_display_data_density(void) { menu_build_by_filter(menu_filter_display_data_density, KEYNUM_MENU_DISPLAY_DATA_DENSITY, menu_display_data); }
/**
 * @brief 显示或打印屏幕菜单操作中的 menu_display_data_temp 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_display_data_temp(void)    { menu_build_by_filter(menu_filter_display_data_temp,    KEYNUM_MENU_DISPLAY_DATA_TEMP,    menu_display_data); }
/**
 * @brief 执行屏幕菜单操作中的 menu_cpu3_comm1 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_cpu3_comm1(void)   { menu_build_by_group(MENU_GRP_CPU3_COM1,   KEYNUM_MENU_CPU3_COM1,   menu_comm_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_cpu3_comm2 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_cpu3_comm2(void)   { menu_build_by_group(MENU_GRP_CPU3_COM2,   KEYNUM_MENU_CPU3_COM2,   menu_comm_config); }
/**
 * @brief 执行屏幕菜单操作中的 menu_cpu3_comm3 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void menu_cpu3_comm3(void)   { menu_build_by_group(MENU_GRP_CPU3_COM3,   KEYNUM_MENU_CPU3_COM3,   menu_comm_config); }

/**
 * @brief 执行屏幕菜单操作中的 menu_paracfg_main 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
