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
#include "Display.h"
#include "hgs.h"
#include <math.h>
#include "system_parameter.h"
#include "cpu3_comm_display_params.h"
#include "system_parameter.h"
#include <string.h>    // for memset, memcpy, strcmp, strlen...

#define PASSWORD_ENTERMAIN		1009
extern volatile uint8_t g_cpu3_uart_reinit_pending;// CPU3 串口重初始化标志

typedef void (*pFunc_void)(void);

typedef struct
{
	int menu_num;			/* 菜单第几栏序号(从 0 开始) */
	int menu_cnt;			/* 上下键计数(从 1 开始计数, 便于取模) */
	int menu_page;			/* 当前显示的页数(页起始项) */
} PAGENUM_T;

 PAGENUM_T PageNum[KEYNUM_END];
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
	{ (uint8_t*)"预留1", (uint8_t*)"No Protocol" },
	{ (uint8_t*)"预留2", (uint8_t*)"No Protocol" },
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

/* =====================================================================
 * 静态函数声明
 *	按“模块职责”重新分类，便于快速定位
 * ===================================================================== */

/* ---------- 1) 菜单入口 / 页面跳转(顶层流程) ----------
 *	这些函数负责切换页面、组织菜单结构、决定下一步流程
 */
static void mainmenu(void);				/* 主菜单 */
static void measuremenu(void);			/* 测量命令菜单 */
//static void menu_paraconfig(void);		/* 参数配置主菜单 */
static void menu_cmdconfig_main(void);	/* 调试指令主菜单 */

/* ---------- 2) 参数分组子菜单(参数页面) ----------
 *	各参数分类页面，仅负责“列出参数项 + 跳转到参数读写流程”
 */
//static void menu_tankbasicpara(void);	/* 基础参数 */
//static void menu_weightpara(void);		/* 称重/载荷相关参数 */
//static void menu_spreadpara(void);		/* 分布测量参数 */
//static void menu_correctionpara(void);	/* 密度/温度修正参数 */
//static void menu_realhighpara(void);	/* 实高测量参数 */
//static void menu_liquidlevelparams(void);/* 液位测量参数 */
//static void menu_waterlevelparams(void);/* 水位测量参数 */
//static void menu_alarmdoparams(void);	/* 继电器/DO 报警参数 */
//static void menu_aoparams(void);		/* 4-20mA/AO 输出参数 */
//static void menu_wartsilapara(void);	/* 瓦锡兰参数组 */
//static void menu_screen(void);			/* 屏幕/显示相关菜单 */
//static void menu_scr_source(void);		/* 数据源菜单 */
//static void menu_cpu3_comm(void);		/* CPU3 串口通信配置菜单 */
//static void menu_magnetic(void);		/* 磁通量/修正相关菜单(旧菜单或兼容入口) */
static MenuGroup ParamGroupOf(int operaNum);/* 根据操作码获取参数分组枚举 */
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

static void menu_do_alarm(void)  ;
static void menu_ao(void)      ;
static void menu_cal_sp(void)     ;

static void menu_param_check(void) ;

/* CPU3：同理，分组页 = 参数列表页 */
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
static void operationselect(uint8_t *(*menu)[2], int menulen);	/* 枚举含义选择列表 */
static void selectparaword(void);								/* 进入“枚举含义选择”页 */

/* ---------- 4) 参数读写流程(读参数 -> 显示 -> 修改 -> 写回) ----------
 *	参数类操作的完整闭环：读、显示、写权限检查、范围检查、写回
 */
static int get_para_data(void);			/* 读取当前参数对应寄存器 */
static void para_mainprocess(void);		/* 参数流程入口：读 -> displaypara */
static void displaypara(void);			/* 显示参数值/含义 */
static void parawritecheck(void);		/* 写权限检查：是否允许修改 */
static void parascopecheck(void);		/* 范围检查：最小/最大等 */
static void cmd_configpara_process(void);/* 组包写参数 -> 回读 -> 刷新显示 */

/* ---------- 5) 指令下发流程(无参/带参) ----------
 *	把“确定/返回”的动作映射到具体执行：下发指令或写参数
 */
static void ifsendcmd(void);			/* “是否下发/确认返回”页面 */
static pFunc_void dtm_suretofunc(void);	/* 确认键 -> 下一步函数 */
static pFunc_void dtm_backtofunc(void);	/* 返回键 -> 返回上一级函数 */
static void cmd_nopara_process(void);	/* 无参指令：直接下发 */
static void cmd_onepara_process(void);	/* 带参指令：先写参数再下发 */

/* ---------- 6) 输入与数值编辑(输入框) ----------
 *	数字逐位输入、符号输入、位数/单位/小数点等显示规则
 */
static void inputcmdpara(void);			/* 输入参数页面(数值/符号) */
static bool inputvalue(uint8_t deci, uint8_t row, uint8_t line,
		uint8_t points, uint8_t *unit, int *value);				/* 多位数字输入状态机 */
static int SignInput(uint8_t row, uint8_t line, uint8_t shift);/* 正负号输入 */

/* ---------- 7) 名称/单位/枚举含义工具函数 ----------
 *	根据 operaNum 或 param_meta 表，返回名字、单位、小数点位数、显示位数等
 */
static uint8_t *dtm_operaname(int num);	/* 根据操作号返回名称(中/英) */
static uint8_t dtm_namelength(uint8_t *name);/* 计算字符串长度 */
static uint8_t dtm_points(void);		/* 小数点位数 */
static uint8_t *dtm_unit(void);		/* 单位字符串 */
static uint8_t dtm_bits(void);			/* 显示/输入位数 */
static uint8_t *(*dtm_disarr(int *pindex, int *plen))[2];		/* 获取枚举含义数组 */
static uint8_t *returnWordType(uint8_t *chinese, uint8_t *english);/* 语言选择 */

/* ---------- 8) 密码/权限入口 ----------
 *	进入参数配置/调试指令前的密码流程
 */
static void password_enter_para(void);	/* 进入参数配置前输入密码 */
static void password_enter_cmd(void);	/* 进入调试指令前输入密码 */
static void ifentermainmenu(void);		/* 是否进入罐上操作 */
static void ifexittankopera(void);		/* 是否退出罐上操作 */

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

    /* 21 - 报警 DO 参数 */
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
};


/* ==============================
 * 按键操作处理
 * ============================== */
void KeyProcess(uint8_t keypress)
{
	if ((keypress & keymenu[func_index].keyauthority) != 0) {
		NowKeyPress = keypress;
		useKey();

		if (keypress == USE_KEY_BACK && keymenu[func_index].back_opera != NULL) {
			keymenu[func_index].back_opera();
		} else if (keypress == USE_KEY_UP && keymenu[func_index].up_opera != NULL) {
			keymenu[func_index].up_opera();
		} else if (keypress == USE_KEY_DOWN && keymenu[func_index].down_opera != NULL) {
			keymenu[func_index].down_opera();
		} else if (keypress == USE_KEY_SURE && keymenu[func_index].sure_opera != NULL) {
			keymenu[func_index].sure_opera();
		} else {
			printf("NULL\r\n");
		}
	}
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
	uint8_t namelen = 0;
	uint8_t *name = NULL;
	static bool flag_inputsign = false;

	oled_clear();
	func_index = KEYNUM_INPUTCMDPARA;

	line = DisplayLangaugeLineWords((uint8_t*)"请输入", OLED_LINE8_1, OLED_ROW4_1, 0, (u8*)"Please enter ");
	name = dtm_operaname(now_Opera_Num);
	namelen = dtm_namelength(name);

	/* 名字太长或英文时, 标题显示换行 */
	if (namelen > 10 || screen_parameter.language != LANGUAGE_CHINESE) {
		row = OLED_ROW4_2;
		line = OLED_LINE8_1;
	}

	OledDisplayLineWords(name, line, row, 0);

	if (namelen > 10 || screen_parameter.language != LANGUAGE_CHINESE) {
		row = OLED_ROW4_3;
	} else {
		row = OLED_ROW3_2;
	}

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
    };

    /* 3) CPU3 本机“固定项”名称（如果你仍然需要这种非 param_meta 的本机项） */
    static const OperaNameMap_t local_fixed_map[] = {
        /* 你原来写了“设备地址/屏幕程序版本”，但新枚举里未看到“设备地址”对应项
           如果你有对应的 COM_NUM_xxx，就填进去；没有就先注释掉。
         */
        /* { COM_NUM_PARA_LOCAL_DEVICEADDR, (uint8_t*)"设备地址", (uint8_t*)"Device Address" }, */
        { COM_NUM_PARA_LOCAL_LEDVERSION,  (uint8_t*)"屏幕程序版本", (uint8_t*)"Screen FW Ver" },
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
    if (num > COM_NUM_DEBUGCMD_START && num < COM_NUM_DEBUGCMD_STOP) {
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

///* 返回操作名称 */
//static uint8_t *dtm_operaname(int num)
//{
//	/* 1) 普通无参测量指令 */
//	static uint8_t *OperaNameArr_normal_cmd[][2] = {
//		{ (uint8_t*)"回零点", (uint8_t*)"Return to Zero" },
//		{ (uint8_t*)"标定零点", (uint8_t*)"Zero Calibration" },
//		{ (uint8_t*)"分布测量", (uint8_t*)"Spread-M" },
//		{ (uint8_t*)"寻找液位", (uint8_t*)"Find Oil Level" },
//		{ (uint8_t*)"寻找水位", (uint8_t*)"Find Water Level" },
//		{ (uint8_t*)"寻找罐底", (uint8_t*)"Find Tank Bottom" },
//		{ (uint8_t*)"综合测量", (uint8_t*)"Comprehensive-M" },
//		{ (uint8_t*)"每米测量", (uint8_t*)"DT-PerMeter-M" },
//		{ (uint8_t*)"区间测量", (uint8_t*)"Interval-M" },
//		{ (uint8_t*)"瓦锡兰区间密度", (uint8_t*)"Wartsila Interval-M" },
//	};
//
//	/* 2) 无参调试指令 */
//	static uint8_t *OperaNameArr_debug_cmd[][2] = {
//		{ (uint8_t*)"设置空载称重", (uint8_t*)"Set Empty Weight" },
//		{ (uint8_t*)"设置满载称重", (uint8_t*)"Set Full Weight" },
//		{ (uint8_t*)"恢复出厂设置", (uint8_t*)"Factory Reset" },
//		{ (uint8_t*)"维护模式", (uint8_t*)"Maintenance Mode" },
//	};
//
//	/* 3) 本机参数名称 */
//	static uint8_t *OperaNameArr_local[][2] = {
//		{ (uint8_t*)"设备地址", (uint8_t*)"DeviceAddress" },
//		{ (uint8_t*)"屏幕程序版本", (uint8_t*)"Screen FW Ver" },
//	};
//
//	int idx;
//
//	/* A) 普通不带参指令 */
//	if (num > COM_NUM_NOPARACMD_NORMAL_START && num < COM_NUM_NOPARACMD_NORMAL_STOP) {
//		idx = num - COM_NUM_NOPARACMD_NORMAL_START - 1;
//		if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_normal_cmd) / sizeof(OperaNameArr_normal_cmd[0]))) {
//			return OperaNameArr_normal_cmd[idx][screen_parameter.language];
//		}
//	}
//	/* B) 无参调试指令 */
//	else if (num > COM_NUM_DEBUGCMD_START && num < COM_NUM_DEBUGCMD_STOP) {
//		idx = num - COM_NUM_DEBUGCMD_START - 1;
//		if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_debug_cmd) / sizeof(OperaNameArr_debug_cmd[0]))) {
//			return OperaNameArr_debug_cmd[idx][screen_parameter.language];
//		}
//	}
//	/* C) 参数类 & 带参指令: 使用 param_meta 表 */
//	else if ((num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_LOCAL_STOP)
//		|| (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END)) {
//		int index = getHoldValueNum(num);
//		if (index >= 0) {
//			if (screen_parameter.language == LANGUAGE_CHINESE) {
//				return param_meta[index].name;
//			} else if (screen_parameter.language == LANGUAGE_ENGLISH) {
//				return param_meta[index].name_English;
//			}
//		}
//	}
//	/* D) 密码类 */
//	else if (num > COM_NUM_PASSWORD_START && num < COM_NUM_PASSWORD_END) {
//		return returnWordType((uint8_t*)"密码", (uint8_t*)"Password");
//	}
//	/* E) 本机参数类 */
//	else if (num > COM_NUM_PARA_LOCAL_START && num < COM_NUM_PARA_LOCAL_STOP) {
//		idx = num - COM_NUM_PARA_LOCAL_START - 1;
//		if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_local) / sizeof(OperaNameArr_local[0]))) {
//			return OperaNameArr_local[idx][screen_parameter.language];
//		}
//	}
//
//	return returnWordType((uint8_t*)"非法操作", (uint8_t*)"Invalid Operation");
//}

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

static uint8_t dtm_namelength(uint8_t *name)
{
	int len = 0;
	while (*(name + len) != 0) {
		len++;
	}
	return (uint8_t)len;
}

/* 确定单位 */
static uint8_t *dtm_unit(void)
{
	uint8_t *u = NULL;

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		u = param_meta[index].unit;
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
	all_screen(0x00);
	func_index = KEYNUM_IFSENDCMD;

	if (now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发指令:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue instruct:");
		OledDisplayLineWords(dtm_operaname(now_Opera_Num), OLED_LINE8_1, OLED_ROW3_2, 0);
	} else if (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发带参指令:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue IWP:");
		OledDisplayLineWords(dtm_operaname(now_Opera_Num), OLED_LINE8_1, OLED_ROW4_2, 0);
		OledValueDisplay(now_Para_CT.val, OLED_LINE8_3, OLED_ROW4_3, 0, now_Para_CT.points, now_Para_CT.unit);
	} else if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发参数:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue Para:");
		OledDisplayLineWords(dtm_operaname(now_Opera_Num), OLED_LINE8_1, OLED_ROW4_2, 0);
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
        case MENU_GRP_CPU3_BASE:     p = menu_cpu3_base;    break;
        case MENU_GRP_CPU3_SOURCE:   p = menu_cpu3_source;  break;
        case MENU_GRP_CPU3_INPUT:    p = menu_cpu3_input;   break;
        case MENU_GRP_CPU3_SCREEN:   p = menu_cpu3_screen;  break;
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
	if (now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END) {
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

static uint8_t is_debug_cmd(uint32_t opera)
{
    return (opera > COM_NUM_DEBUGCMD_START) && (opera < COM_NUM_DEBUGCMD_STOP);
}

/* 统一的“调试指令允许条件”判定（按你现有逻辑扩展） */
static uint8_t debug_cmd_is_allowed(void)
{
    uint32_t st = g_measurement.device_status.device_state;

    /* 你当前对恢复出厂的限制：允许 STANDBY / ERROR / MAINTENANCEMODE
       这里建议把调试类都统一到同一套口径，避免口径不一致 */
    if ((st == STATE_STANDBY) || (st == STATE_ERROR) || (st == STATE_MAINTENANCEMODE)) {
        return 1;
    }
    return 0;
}

static void send_cpu2_command(uint32_t cmd)
{
    /* 写 2 个寄存器：如果你的协议定义就是“command 占 32bit”，这里保持 2 不动 */
    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                              HOLDREGISTER_DEVICEPARAM_COMMAND,
                              2,
                              &cmd);
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
//    if (is_debug_cmd((uint32_t)now_Opera_Num)) {
//        if (!debug_cmd_is_allowed()) {
//            oled_clear();
//            DisplayLangaugeLineWords((uint8_t*)"失败", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Failed");
//            DisplayLangaugeLineWords((uint8_t*)"请先进入调试模式", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Enter debug mode");
//            return;
//        }
//    }

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

        { COM_NUM_RUNUP,             CMD_MOVE_UP },
        { COM_NUM_RUNDOWN,           CMD_MOVE_DOWN },

        { COM_NUM_FORCE_RUNUP,       CMD_FORCE_MOVE_UP },     /* 新增：强制上行 */
        { COM_NUM_FORCE_RUNDOWN,     CMD_FORCE_MOVE_DOWN },   /* 新增：强制下行 */
    };

    const int mapamount = (int)(sizeof(onepara_cmd_map) / sizeof(onepara_cmd_map[0]));

    /* 1) 找到该操作对应的“参数寄存器元数据” */
    index = getHoldValueNum(now_Opera_Num);
    if (index < 0) {
        all_screen(0x00);
        DisplayLangaugeLineWords((uint8_t*)"非法参数！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
        HAL_Delay(800);
        exitTankOpera();
        return;
    }

    /* 2) 动态组织要下发的参数字节：rgstcnt 个寄存器 = rgstcnt*2 字节 */
    {
        int bytes = (int)param_meta[index].rgstcnt * 2;
        if (bytes <= 0 || bytes > 64) {
            all_screen(0x00);
            DisplayLangaugeLineWords((uint8_t*)"参数长度异常！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Bad Para Len");
            HAL_Delay(800);
            exitTankOpera();
            return;
        }

        uint8_t paraarr[64];
        memset(paraarr, 0, sizeof(paraarr));

        all_screen(0x00);
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
    all_screen(0x00);
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
            all_screen(0x00);
            DisplayLangaugeLineWords((uint8_t*)"指令未定义！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Cmd Undefined");
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
//
///* 不带参线圈指令处理过程 */
//static void cmd_nopara_process(void)
//{
//	static const uint32_t nopara_cmd_map[][2] = {
//		{ COM_NUM_BACK_ZERO, CMD_BACK_ZERO },
//		{ COM_NUM_FIND_ZERO, CMD_CALIBRATE_ZERO },
//		{ COM_NUM_SPREADPOINTS, CMD_MEASURE_DISTRIBUTED },
//		{ COM_NUM_FIND_OIL, CMD_FIND_OIL },
//		{ COM_NUM_FIND_WATER, CMD_FIND_WATER },
//		{ COM_NUM_FIND_BOTTOM, CMD_FIND_BOTTOM },
//		{ COM_NUM_SYNTHETIC, CMD_SYNTHETIC },
//		{ COM_NUM_METER_DENSITY, CMD_MEASURE_DENSITY_METER },
//		{ COM_NUM_INTERVAL_DENSITY, CMD_MEASURE_DENSITY_RANGE },
//		{ COM_NUM_WARTSILA_DENSITY, CMD_WARTSILA_DENSITY_RANGE },
//
//		{ COM_NUM_SET_EMPTY_WEIGHT, CMD_SET_EMPTY_WEIGHT },
//		{ COM_NUM_SET_FULL_WEIGHT, CMD_SET_FULL_WEIGHT },
//		{ COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },
//		{ COM_NUM_MAINTENANCE_MODE, CMD_MAINTENANCE_MODE },
//	};
//
//	int mapamount = (int)(sizeof(nopara_cmd_map) / sizeof(nopara_cmd_map[0]));
//	int i;
//
//	for (i = 0; i < mapamount; i++) {
//		if (now_Opera_Num == (int)nopara_cmd_map[i][0]) {
//			CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
//					HOLDREGISTER_DEVICEPARAM_COMMAND, 2,
//					(uint32_t*)&nopara_cmd_map[i][1]);
//			break;
//		}
//	}
//
//	if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
//		oled_clear();
//		if ((g_measurement.device_status.device_state != STATE_STANDBY)
//			&& (g_measurement.device_status.device_state != STATE_ERROR)
//			&& (g_measurement.device_status.device_state != STATE_MAINTENANCEMODE)) {
//			DisplayLangaugeLineWords((uint8_t*)"失败", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Failed to set");
//			DisplayLangaugeLineWords((uint8_t*)"请先进入调试模式", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Enter debug mode");
//		} else {
//			DisplayLangaugeLineWords((uint8_t*)"正在恢复出厂设置", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Factory Settings");
//			HAL_Delay(800);
//			exitTankOpera();
//		}
//	} else if (now_Opera_Num == COM_NUM_MAINTENANCE_MODE) {
//		oled_clear();
//		DisplayLangaugeLineWords((uint8_t*)"已进入维护模式", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Maintenance Mode");
//	} else {
//		exitTankOpera();
//	}
//}

/* 非法操作处理 */
static void errorprocess(void)
{
	all_screen(0x00);
	DisplayLangaugeLineWords((uint8_t*)"非法操作！", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Illegal operation");
	DisplayLangaugeLineWords((uint8_t*)"1s后退出屏幕操作", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Exit after 1 second");
	HAL_Delay(1000);
	exitTankOpera();
}

/* 显示参数内容 */
static void displaypara(void)
{
	int bits, line;
	int index;

	all_screen(0x00);
	func_index = KEYNUM_DISPLAY_PARA;

	OledDisplayLineWords(dtm_operaname(now_Opera_Num), OLED_LINE8_1, OLED_ROW4_1, 0);

	/* 本机参数 */
	if (now_Opera_Num == COM_NUM_PARA_LOCAL_LEDVERSION) {
		OledValueDisplay(CPU3VERSION, OLED_LINE8_4, OLED_ROW3_2, 0, 0, NULL);
	} else {
		index = getHoldValueNum(now_Opera_Num);
		if (index == -1) {
			DisplayLangaugeLineWords((uint8_t*)"非法操作", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Illegal operation");
		} else {
			if (param_meta[index].pword == NULL) {
				bits = dtm_bits();
				if (bits <= 3 && param_meta[index].unit == NULL) {
					line = OLED_LINE8_5;
				} else {
					line = OLED_LINE8_4;
				}
				OledValueDisplay(param_meta[index].val, line, OLED_ROW3_2, 0, param_meta[index].point, param_meta[index].unit);
			} else {
				if (dtm_namelength(param_meta[index].pword()) >= 12) {
					line = OLED_LINE8_2;
				} else if (dtm_namelength(param_meta[index].pword()) > 6) {
					line = OLED_LINE8_3;
				} else {
					line = OLED_LINE8_4;
				}
				OledDisplayLineWords(param_meta[index].pword(), line, OLED_ROW3_2, 0);
			}
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

	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, timeback, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"修改", OLED_LINE8_7, OLED_ROW4_4, timesure, (uint8_t*)"Alter");
}

/* 写权限范围检查 */
static void parawritecheck(void)
{
	int index;

	index = getHoldValueNum(now_Opera_Num);
	if (index != -1
		&& param_meta[index].authority_write
		&& (g_measurement.device_status.device_state == STATE_STANDBY
			|| g_measurement.device_status.device_state|0x8000 != 0)) {

		if (param_meta[index].pword == NULL) {
			inputcmdpara();
		} else {
			selectparaword();
		}
	} else {
		all_screen(0x00);
		DisplayLangaugeLineWords((uint8_t*)"无修改权限！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"No permission");
		HAL_Delay(800);
		displaypara();
	}
}

/* 是否进入罐上操作 */
static void ifentermainmenu(void)
{
	all_screen(0x00);
	func_index = KEYNUM_IF_ENTER_MAINMENU;
	DisplayLangaugeLineWords((uint8_t*)"是否进入罐上操作?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Enter operation?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, 0, (uint8_t*)"Ok");
}

/* 是否退出罐上操作 */
static void ifexittankopera(void)
{
	all_screen(0x00);
	func_index = KEYNUM_IF_EXIT_MAINMENU;
	DisplayLangaugeLineWords((uint8_t*)"是否退出罐上操作?", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Exit operation?");
	DisplayLangaugeLineWords((uint8_t*)"返回", OLED_LINE8_1, OLED_ROW4_4, 0, (uint8_t*)"Back");
	DisplayLangaugeLineWords((uint8_t*)"确认", OLED_LINE8_8, OLED_ROW4_4, 0, (uint8_t*)"Ok");
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
	  // CPU3 本机参数：不走 CPU2 通讯，直接刷新 val
	if (Cpu3Local_IsParam((OperatingNumber)now_Opera_Num)) {
		param_meta[index].val = Cpu3Local_ReadValue((OperatingNumber)now_Opera_Num);
		return 0;
	}
	else  // CPU2 参数
	{
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
		all_screen(0x00);
		DisplayLangaugeLineWords((uint8_t*)"非法参数！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
		HAL_Delay(800);
		displaypara();
	} else if (param_meta[index].flag_checkvalue) {
		if (now_Para_CT.val < param_meta[index].valuemin || now_Para_CT.val > param_meta[index].valuemax) {
			all_screen(0x00);
			DisplayLangaugeLineWords((uint8_t*)"数值超范围！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Value out of Range");
			HAL_Delay(800);
			displaypara();
		} else {
			cmd_configpara_process();
		}
	} else {
		cmd_configpara_process();
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

	all_screen(0x00);
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

	//如果是本机参数
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
	else //下发给CPU2
	{
	    uint16_t regs16[32]; /* rgstcnt 最大一般不会很大；32=最多64字节 */
	    int rc = (int)param_meta[index].rgstcnt;
	    int bytes = rc * 2;

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
	        if (p < arrlen)     hi = paraarr[p];
	        if (p + 1 < arrlen) lo = paraarr[p + 1];
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

		DisplayLangaugeLineWords(menu[i].operaName, line, row, shift, menu[i].operaName2);
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

/* 返回通讯方式文字信息 */
uint8_t *ret_arr_word(void)
{
	int index, len;
	uint8_t *(*p)[2];

	p = dtm_disarr(&index, &len);
	if (index < len && p != NULL) {
		return p[index][screen_parameter.language];
	} else {
		return (uint8_t*)"非法配置";
	}
}

/* 决定显示哪个数组里的文字 */
uint8_t *(*dtm_disarr(int *pindex, int *plen))[2]
{
	int index = 0, len = -1;
	uint8_t *(*p)[2];

	index = getHoldValueNum(now_Opera_Num);

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
	case COM_NUM_SCREEN_OFF: {
		index = param_meta[index].val;
		len = (int)(sizeof(arr_IF) / sizeof(arr_IF[0]));
		p = arr_IF;
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
		index = param_meta[index].val;
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
	default:
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

	oled_clear();
	func_index = KEYNUM_WORDSELECT;

	parr = dtm_disarr(&index, &len);
	operationselect(parr, len-1);
}

/* 隐藏信息含义选择栏 */
static void operationselect(uint8_t *(*menu)[2], int menulen)
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
			now_Para_CT.val = menu_num_hide;
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
		OledDisplayLineWords(menu[i][screen_parameter.language], line, row, shift);
		row += OLED_ROW4_2;
	}
}

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

	all_screen(0x00);
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

    all_screen(0x00);
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

        /* ===== 称重相关 ===== */
        { (uint8_t*)"获取空载称重", COM_NUM_SET_EMPTY_WEIGHT, ifsendcmd, COMMANE_NORW, (uint8_t*)"SetEmptyWeight" },
        { (uint8_t*)"获取满载称重", COM_NUM_SET_FULL_WEIGHT,  ifsendcmd, COMMANE_NORW, (uint8_t*)"SetFullWeight"  },

        /* ===== 系统/维护 ===== */
        { (uint8_t*)"恢复出厂设置", COM_NUM_RESTOR_EFACTORYSETTING, ifsendcmd, COMMANE_NORW, (uint8_t*)"RestoreFactory" },
        { (uint8_t*)"进入维护模式",     COM_NUM_MAINTENANCE_MODE,       ifsendcmd, COMMANE_NORW, (uint8_t*)"Maintenance"    },

        /* ===== 退出 ===== */
        { (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
    };

    int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

    all_screen(0x00);
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

	all_screen(0x00);
	func_index = KEYNUM_MENU_LANGUAGE;
	menuselect(menu, menulen);
}

static void setchinese(void)
{
	screen_parameter.language = LANGUAGE_CHINESE;
	mainmenu();
}

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

/* -------------------- 分组映射（只维护这个即可） -------------------- */
static MenuGroup ParamGroupOf(int operaNum)
{
    switch (operaNum) {
    /* 设备信息 */
    case COM_NUM_DEVICEPARAM_SENSORTYPE:
    case COM_NUM_DEVICEPARAM_SENSORID:
    case COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION:
    case COM_NUM_DEVICEPARAM_SOFTWAREVERSION:
    case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:
    case COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO:
    case COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT:
        return MENU_GRP_DEV_INFO;

    /* 机械/电机/编码器 */
    case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
    case COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED:
    case COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM:
    case COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM:
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
        return MENU_GRP_LIQUID;

    /* 水位 */
    case COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT:
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE:
    case COM_NUM_DEVICEPARAM_WATER_BLINDZONE:
    case COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD:
    case COM_NUM_DEVICEPARAM_WATER_CAP_HYSTERESIS:
    case COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE:
    case COM_NUM_DEVICEPARAM_ZERO_CAP:
    case COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD:
        return MENU_GRP_WATER;

    /* 罐底/罐高 */
    case COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE:
    case COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD:
    case COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD:
    case COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG:
    case COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION:
    case COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT:
    case COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT:
        return MENU_GRP_BOTTOM_TANKH;

    /* 修正 */
    case COM_NUM_DEVICEPARAM_DENSITYCORRECTION:
    case COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION:
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
        return MENU_GRP_WARTSILA;

    /* DO */
    case COM_NUM_DEVICEPARAM_ALARM_HIGH_DO:
    case COM_NUM_DEVICEPARAM_ALARM_LOW_DO:
    case COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD:
        return MENU_GRP_DO_ALARM;

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
    case COM_NUM_CAL_OIL:
    case COM_NUM_CALIBRATE_WATER:
    case COM_NUM_SINGLE_POINT:
    case COM_NUM_SP_TEST:
    case COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL:
    case COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE:
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

    all_screen(0x00);
    func_index = key_index;
    menuselect(menu, menulen);
}

/* CPU2：分组页 = 参数列表页（取消 DEBUG 容器页） */
static void menu_dev_info(void)     { menu_build_by_group(MENU_GRP_DEV_INFO,     KEYNUM_MENU_PARA_DEV_INFO,     menu_paracfg_main); }
static void menu_mech(void)         { menu_build_by_group(MENU_GRP_MECH,         KEYNUM_MENU_PARA_MECH,         menu_paracfg_main); }
static void menu_weight(void)       { menu_build_by_group(MENU_GRP_WEIGHT,       KEYNUM_MENU_PARA_WEIGHT,       menu_paracfg_main); }
static void menu_zero(void)         { menu_build_by_group(MENU_GRP_ZERO,         KEYNUM_MENU_PARA_ZERO,         menu_paracfg_main); }

static void menu_liquid(void)       { menu_build_by_group(MENU_GRP_LIQUID,       KEYNUM_MENU_PARA_LIQUID,       menu_paracfg_main); }
static void menu_water(void)        { menu_build_by_group(MENU_GRP_WATER,        KEYNUM_MENU_PARA_WATER,        menu_paracfg_main); }
static void menu_bottom_tankh(void) { menu_build_by_group(MENU_GRP_BOTTOM_TANKH, KEYNUM_MENU_PARA_BOTTOM_TANKH, menu_paracfg_main); }

static void menu_correct(void)      { menu_build_by_group(MENU_GRP_CORR,         KEYNUM_MENU_PARA_CORR,         menu_paracfg_main); }
static void menu_policy(void)       { menu_build_by_group(MENU_GRP_POLICY,       KEYNUM_MENU_PARA_POLICY,       menu_paracfg_main); }
static void menu_wartsila(void)     { menu_build_by_group(MENU_GRP_WARTSILA,     KEYNUM_MENU_PARA_WARTSILA,     menu_paracfg_main); }

static void menu_do_alarm(void)     { menu_build_by_group(MENU_GRP_DO_ALARM,     KEYNUM_MENU_PARA_DO,           menu_paracfg_main); }
static void menu_ao(void)           { menu_build_by_group(MENU_GRP_AO,           KEYNUM_MENU_PARA_AO,           menu_paracfg_main); }
static void menu_cal_sp(void)       { menu_build_by_group(MENU_GRP_CAL_SP,       KEYNUM_MENU_PARA_CAL_SP,       menu_paracfg_main); }

static void menu_param_check(void)  { menu_build_by_group(MENU_GRP_PARAM_CHECK, KEYNUM_MENU_PARA_PARAM_CHECK,  menu_paracfg_main); }

/* CPU3：同理，分组页 = 参数列表页 */
static void menu_cpu3_base(void)    { menu_build_by_group(MENU_GRP_CPU3_BASE,   KEYNUM_MENU_CPU3_BASE,   menu_paracfg_main); }
static void menu_cpu3_source(void)  { menu_build_by_group(MENU_GRP_CPU3_SOURCE, KEYNUM_MENU_CPU3_SOURCE, menu_paracfg_main); }
static void menu_cpu3_input(void)   { menu_build_by_group(MENU_GRP_CPU3_INPUT,  KEYNUM_MENU_CPU3_INPUT,  menu_paracfg_main); }
static void menu_cpu3_screen(void)  { menu_build_by_group(MENU_GRP_CPU3_SCREEN, KEYNUM_MENU_CPU3_SCREEN, menu_paracfg_main); }
static void menu_cpu3_comm1(void)   { menu_build_by_group(MENU_GRP_CPU3_COM1,   KEYNUM_MENU_CPU3_COM1,   menu_paracfg_main); }
static void menu_cpu3_comm2(void)   { menu_build_by_group(MENU_GRP_CPU3_COM2,   KEYNUM_MENU_CPU3_COM2,   menu_paracfg_main); }
static void menu_cpu3_comm3(void)   { menu_build_by_group(MENU_GRP_CPU3_COM3,   KEYNUM_MENU_CPU3_COM3,   menu_paracfg_main); }

static void menu_paracfg_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"基础信息",      0, menu_dev_info,     COMMANE_NORW, (uint8_t*)"Info"},
        {(uint8_t*)"电机与编码参数",      0, menu_mech,         COMMANE_NORW, (uint8_t*)"Mechanism"},
        {(uint8_t*)"称重参数",      0, menu_weight,       COMMANE_NORW, (uint8_t*)"Weight"},
        {(uint8_t*)"零点参数",      0, menu_zero,         COMMANE_NORW, (uint8_t*)"Zero"},

        {(uint8_t*)"液位参数",      0, menu_liquid,       COMMANE_NORW, (uint8_t*)"Level"},
        {(uint8_t*)"水位参数",      0, menu_water,        COMMANE_NORW, (uint8_t*)"Water"},
        {(uint8_t*)"罐高参数",     0, menu_bottom_tankh, COMMANE_NORW, (uint8_t*)"Bottom/TankH"},

        {(uint8_t*)"磁通量参数",      0, menu_correct,      COMMANE_NORW, (uint8_t*)"Correction"},
        {(uint8_t*)"密度测量参数",0, menu_policy,       COMMANE_NORW, (uint8_t*)"Policy"},
        {(uint8_t*)"Wartsila参数",  0, menu_wartsila,     COMMANE_NORW, (uint8_t*)"Wartsila"},

        {(uint8_t*)"DO报警",        0, menu_do_alarm,     COMMANE_NORW, (uint8_t*)"DO Alarm"},
        {(uint8_t*)"AO输出",        0, menu_ao,           COMMANE_NORW, (uint8_t*)"AO"},
//        {(uint8_t*)"标定/单点",     0, menu_cal_sp,       COMMANE_NORW, (uint8_t*)"Cal/SP"},
        {(uint8_t*)"校验信息",      0, menu_param_check,  COMMANE_NORW, (uint8_t*)"Check"},

        {(uint8_t*)"CPU3版本信息",     0, menu_cpu3_base,    COMMANE_NORW, (uint8_t*)"CPU3 Base"},
        {(uint8_t*)"数据源设置",   0, menu_cpu3_source,  COMMANE_NORW, (uint8_t*)"CPU3 Source"},
        {(uint8_t*)"数据源手输值",   0, menu_cpu3_input,   COMMANE_NORW, (uint8_t*)"CPU3 Input"},
        {(uint8_t*)"显示设置",     0, menu_cpu3_screen,  COMMANE_NORW, (uint8_t*)"CPU3 Screen"},
        {(uint8_t*)"COM1配置",     0, menu_cpu3_comm1,   COMMANE_NORW, (uint8_t*)"CPU3 COM1"},
        {(uint8_t*)"COM2配置",     0, menu_cpu3_comm2,   COMMANE_NORW, (uint8_t*)"CPU3 COM2"},
        {(uint8_t*)"COM3配置",     0, menu_cpu3_comm3,   COMMANE_NORW, (uint8_t*)"CPU3 COM3"},

        {(uint8_t*)"返回主菜单",    COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Back"},
    };

    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_MAIN;
    menuselect(menu, (int)(sizeof(menu)/sizeof(menu[0])));
}
