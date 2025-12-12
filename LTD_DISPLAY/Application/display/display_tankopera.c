/*
 * display_tankopera.c
 *
 * 说明:
 *	1) 罐上操作菜单系统(按键: 上/下/确认/返回)
 *	2) 指令下发与参数读写(与 CPU2 通讯)
 *	3) OLED 菜单显示(中/英切换)
 *
 * 编码要求:
 *	- 中文字符串/注释使用简体中文常用字, 建议保存为 GBK
 *	- 避免使用特殊符号(如长破折号等)
 */

#include "display_tankopera.h"
#include "tim.h"
#include "usart.h"
#include "exit.h"
#include "Display.h"
#include "hgs.h"
#include <math.h>
#include "communicate.h"
#include "system_parameter.h"

#define PASSWORD_ENTERMAIN		1009

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
	{ (uint8_t*)"固定点测量", (uint8_t*)"Fixed Point" },
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

/* =====================================================================
 * 静态函数声明
 *	按“模块职责”重新分类，便于快速定位
 * ===================================================================== */

/* ---------- 1) 菜单入口 / 页面跳转(顶层流程) ----------
 *	这些函数负责切换页面、组织菜单结构、决定下一步流程
 */
static void mainmenu(void);				/* 主菜单 */
static void measuremenu(void);			/* 测量命令菜单 */
static void menu_paraconfig(void);		/* 参数配置主菜单 */
static void menu_cmdconfig_main(void);	/* 调试指令主菜单 */

/* ---------- 2) 参数分组子菜单(参数页面) ----------
 *	各参数分类页面，仅负责“列出参数项 + 跳转到参数读写流程”
 */
static void menu_tankbasicpara(void);	/* 基础参数 */
static void menu_weightpara(void);		/* 称重/载荷相关参数 */
static void menu_spreadpara(void);		/* 分布测量参数 */
static void menu_correctionpara(void);	/* 密度/温度修正参数 */
static void menu_realhighpara(void);	/* 实高测量参数 */
static void menu_liquidlevelparams(void);/* 液位测量参数 */
static void menu_waterlevelparams(void);/* 水位测量参数 */
static void menu_alarmdoparams(void);	/* 继电器/DO 报警参数 */
static void menu_aoparams(void);		/* 4-20mA/AO 输出参数 */
static void menu_wartsilapara(void);	/* 瓦锡兰参数组 */
static void menu_screen(void);			/* 屏幕/显示相关菜单 */
static void menu_scr_source(void);		/* 数据源菜单 */
static void menu_cpu3_comm(void);		/* CPU3 串口通信配置菜单 */
static void menu_magnetic(void);		/* 磁通量/修正相关菜单(旧菜单或兼容入口) */
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
 *	根据 operaNum 或 holdValue 表，返回名字、单位、小数点位数、显示位数等
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
 * 按键菜单映射表
 * ============================== */
struct KeyMenu keymenu[128] = {
	/* 0 - 是否进入罐上操作 */
	{ exitTankOpera, NULL, NULL, mainmenu, USE_KEY_BACK | USE_KEY_SURE, ifentermainmenu },

	/* 1 - 主菜单 */
	{ ifexittankopera, mainmenu, mainmenu, mainmenu, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, mainmenu },

	/* 2 - 是否退出罐上操作 */
	{ mainmenu, NULL, NULL, exitTankOpera, USE_KEY_BACK | USE_KEY_SURE, ifexittankopera },

	/* 3 - 普通测量指令主菜单 */
	{ measuremenu, measuremenu, measuremenu, measuremenu, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, measuremenu },

	/* 4 - 参数配置主菜单 */
	{ menu_paraconfig, menu_paraconfig, menu_paraconfig, menu_paraconfig, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_paraconfig },

	/* 5 - 菜单 - 基础参数(这里你原来写的是 wartsila 菜单, 保持不改功能) */
	{ menu_wartsilapara, menu_wartsilapara, menu_wartsilapara, menu_wartsilapara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_wartsilapara },

	/* 6 - 菜单 - 维护调试指令 */
	{ menu_cmdconfig_main, menu_cmdconfig_main, menu_cmdconfig_main, menu_cmdconfig_main, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cmdconfig_main },

	/* 7 - 是否下发指令或参数 */
	{ ifsendcmd, NULL, NULL, ifsendcmd, USE_KEY_BACK | USE_KEY_SURE, ifsendcmd },

	/* 8 - 请输入参数值(带参指令中的) */
	{ inputcmdpara, inputcmdpara, inputcmdpara, inputcmdpara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, inputcmdpara },

	/* 9 - 参数显示(读写类参数中的) */
	{ displaypara, NULL, NULL, displaypara, USE_KEY_BACK | USE_KEY_SURE, displaypara },

	/* 10 - 菜单 - 磁通量参数 */
	{ menu_magnetic, menu_magnetic, menu_magnetic, menu_magnetic, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_magnetic },

	/* 11 - 隐藏信息选择 */
	{ selectparaword, selectparaword, selectparaword, selectparaword, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, selectparaword },

	/* 12 - 菜单 - 界面显示类参数 */
	{ menu_screen, menu_screen, menu_screen, menu_screen, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_screen },

	/* 13 - 菜单 - 数据源 */
	{ menu_scr_source, menu_scr_source, menu_scr_source, menu_scr_source, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_scr_source },

	/* 14 - 维护参数 - 基础参数 */
	{ menu_tankbasicpara, menu_tankbasicpara, menu_tankbasicpara, menu_tankbasicpara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_tankbasicpara },

	/* 15 - 维护参数 - 综合测量参数(称重) */
	{ menu_weightpara, menu_weightpara, menu_weightpara, menu_weightpara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_weightpara },

	/* 16 - 维护参数 - 分布测量参数 */
	{ menu_spreadpara, menu_spreadpara, menu_spreadpara, menu_spreadpara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_spreadpara },

	/* 17 - 维护参数 - 修正参数 */
	{ menu_correctionpara, menu_correctionpara, menu_correctionpara, menu_correctionpara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_correctionpara },

	/* 18 - 维护参数 - 实高测量 */
	{ menu_realhighpara, menu_realhighpara, menu_realhighpara, menu_realhighpara, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_realhighpara },

	/* 19 - 维护参数 - 液位测量参数 */
	{ menu_liquidlevelparams, menu_liquidlevelparams, menu_liquidlevelparams, menu_liquidlevelparams, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_liquidlevelparams },

	/* 20 - 维护参数 - 水位测量参数 */
	{ menu_waterlevelparams, menu_waterlevelparams, menu_waterlevelparams, menu_waterlevelparams, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_waterlevelparams },

	/* 21 - 维护参数 - 报警(DO) */
	{ menu_alarmdoparams, menu_alarmdoparams, menu_alarmdoparams, menu_alarmdoparams, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_alarmdoparams },

	/* 22 - 维护参数 - 4-20mA(AO) */
	{ menu_aoparams, menu_aoparams, menu_aoparams, menu_aoparams, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_aoparams },

	/* 23 - 显示设置 - 语言 */
	{ setlanguage, setlanguage, setlanguage, setlanguage, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, setlanguage },

	/* 24 - CPU3 通信参数菜单 */
	{ menu_cpu3_comm, menu_cpu3_comm, menu_cpu3_comm, menu_cpu3_comm, USE_KEY_BACK | USE_KEY_UP | USE_KEY_DOWN | USE_KEY_SURE, menu_cpu3_comm },
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

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		p = holdValue[index].point;
	} else {
		switch (now_Opera_Num) {
		default:
			break;
		}
	}

	return p;
}

/* 返回操作名称 */
static uint8_t *dtm_operaname(int num)
{
	/* 1) 普通无参测量指令 */
	static uint8_t *OperaNameArr_normal_cmd[][2] = {
		{ (uint8_t*)"回零点", (uint8_t*)"Return to Zero" },
		{ (uint8_t*)"标定零点", (uint8_t*)"Zero Calibration" },
		{ (uint8_t*)"分布测量", (uint8_t*)"Spread-M" },
		{ (uint8_t*)"寻找液位", (uint8_t*)"Find Oil Level" },
		{ (uint8_t*)"寻找水位", (uint8_t*)"Find Water Level" },
		{ (uint8_t*)"寻找罐底", (uint8_t*)"Find Tank Bottom" },
		{ (uint8_t*)"综合测量", (uint8_t*)"Comprehensive-M" },
		{ (uint8_t*)"每米测量", (uint8_t*)"DT-PerMeter-M" },
		{ (uint8_t*)"区间测量", (uint8_t*)"Interval-M" },
		{ (uint8_t*)"瓦锡兰区间密度", (uint8_t*)"Wartsila Interval-M" },
	};

	/* 2) 无参调试指令 */
	static uint8_t *OperaNameArr_debug_cmd[][2] = {
		{ (uint8_t*)"设置空载称重", (uint8_t*)"Set Empty Weight" },
		{ (uint8_t*)"设置满载称重", (uint8_t*)"Set Full Weight" },
		{ (uint8_t*)"恢复出厂设置", (uint8_t*)"Factory Reset" },
		{ (uint8_t*)"维护模式", (uint8_t*)"Maintenance Mode" },
	};

	/* 3) 本机参数名称 */
	static uint8_t *OperaNameArr_local[][2] = {
		{ (uint8_t*)"设备地址", (uint8_t*)"DeviceAddress" },
		{ (uint8_t*)"屏幕程序版本", (uint8_t*)"Screen FW Ver" },
	};

	int idx;

	/* A) 普通不带参指令 */
	if (num > COM_NUM_NOPARACMD_NORMAL_START && num < COM_NUM_NOPARACMD_NORMAL_STOP) {
		idx = num - COM_NUM_NOPARACMD_NORMAL_START - 1;
		if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_normal_cmd) / sizeof(OperaNameArr_normal_cmd[0]))) {
			return OperaNameArr_normal_cmd[idx][screen_parameter.language];
		}
	}
	/* B) 无参调试指令 */
	else if (num > COM_NUM_DEBUGCMD_START && num < COM_NUM_DEBUGCMD_STOP) {
		idx = num - COM_NUM_DEBUGCMD_START - 1;
		if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_debug_cmd) / sizeof(OperaNameArr_debug_cmd[0]))) {
			return OperaNameArr_debug_cmd[idx][screen_parameter.language];
		}
	}
	/* C) 参数类 & 带参指令: 使用 holdValue 表 */
	else if ((num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARA_DEBUG_END)
		|| (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(num);
		if (index >= 0) {
			if (screen_parameter.language == LANGUAGE_CHINESE) {
				return holdValue[index].name;
			} else if (screen_parameter.language == LANGUAGE_ENGLISH) {
				return holdValue[index].name_English;
			}
		}
	}
	/* D) 密码类 */
	else if (num > COM_NUM_PASSWORD_START && num < COM_NUM_PASSWORD_END) {
		return returnWordType((uint8_t*)"密码", (uint8_t*)"Password");
	}
	/* E) 本机参数类 */
	else if (num > COM_NUM_PARA_LOCAL_START && num < COM_NUM_PARA_LOCAL_STOP) {
		idx = num - COM_NUM_PARA_LOCAL_START - 1;
		if (idx >= 0 && idx < (int)(sizeof(OperaNameArr_local) / sizeof(OperaNameArr_local[0]))) {
			return OperaNameArr_local[idx][screen_parameter.language];
		}
	}

	return returnWordType((uint8_t*)"非法操作", (uint8_t*)"Invalid Operation");
}

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

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		u = holdValue[index].unit;
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

	if ((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END)
		|| (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)) {
		int index = getHoldValueNum(now_Opera_Num);
		b = holdValue[index].bits;
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
	} else if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END) {
		DisplayLangaugeLineWords((uint8_t*)"是否下发参数:", OLED_LINE8_1, OLED_ROW4_1, 0, (uint8_t*)"Issue Para:");
		OledDisplayLineWords(dtm_operaname(now_Opera_Num), OLED_LINE8_1, OLED_ROW4_2, 0);
		if (now_Para_CT.bits <= 3) {
			OledValueDisplay(now_Para_CT.val, OLED_LINE8_4, OLED_ROW4_3, 0, now_Para_CT.points, now_Para_CT.unit);
		} else {
			OledValueDisplay(now_Para_CT.val, OLED_LINE8_3, OLED_ROW4_3, 0, now_Para_CT.points, now_Para_CT.unit);
		}
	} else if (now_Opera_Num > COM_NUM_PASSWORD_START && now_Opera_Num < COM_NUM_PASSWORD_END) {
		if (now_Para_CT.val == screen_parameter.passward || now_Para_CT.val == FIXPASSWORD) {
			DisplayLangaugeLineWords((uint8_t*)"密码正确!", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Password Correct");
			HAL_Delay(100);
			if (now_Opera_Num == COM_NUM_PASSWORD_ENTER_PARA) {
				menu_paraconfig();
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
static pFunc_void dtm_backtofunc(void)
{
	pFunc_void p = errorprocess;

	switch (now_Opera_Num) {
	/* 普通测量命令 */
	case COM_NUM_BACK_ZERO:
	case COM_NUM_FIND_OIL:
	case COM_NUM_SPREADPOINTS_AI:
	case COM_NUM_FIND_WATER:
	case COM_NUM_FIND_BOTTOM:
	case COM_NUM_SYNTHETIC:
	case COM_NUM_METER_DENSITY:
	case COM_NUM_INTERVAL_DENSITY:
	case COM_NUM_SINGLE_POINT:
	case COM_NUM_SP_TEST:
		p = measuremenu;
		break;

	/* 调试指令 */
	case COM_NUM_FIND_ZERO:
	case COM_NUM_RUNUP:
	case COM_NUM_RUNDOWN:
	case COM_NUM_CORRECTION_OIL:
	case COM_NUM_CAL_OIL:
	case COM_NUM_RESTOR_EFACTORYSETTING:
		p = menu_cmdconfig_main;
		break;

	case COM_NUM_SET_EMPTY_WEIGHT:
	case COM_NUM_SET_FULL_WEIGHT:
		p = menu_weightpara;
		break;

	/* 本机参数 / 语言 / 密码 */
	case COM_NUM_PARA_LOCAL_LEDVERSION:
	case COM_NUM_PARA_LANG:
	case COM_NUM_PASSWORD_ENTER_PARA:
	case COM_NUM_PASSWORD_ENTER_CMD:
		p = mainmenu;
		break;

	/* 界面显示类 */
	case COM_NUM_SCREEN_DECIMAL:
	case COM_NUM_SCREEN_PASSWARD:
	case COM_NUM_SCREEN_OFF:
		p = menu_screen;
		break;

	/* 数据源类 */
	case COM_NUM_SCREEN_SOURCE_OIL:
	case COM_NUM_SCREEN_SOURCE_WATER:
	case COM_NUM_SCREEN_SOURCE_D:
	case COM_NUM_SCREEN_SOURCE_T:
	case COM_NUM_SCREEN_INPUT_OIL:
	case COM_NUM_SCREEN_INPUT_WATER:
	case COM_NUM_SCREEN_INPUT_D:
	case COM_NUM_SCREEN_INPUT_D_SWITCH:
	case COM_NUM_SCREEN_INPUT_T:
		p = menu_scr_source;
		break;

	/* CPU3 通信配置参数 */
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
		p = menu_cpu3_comm;
		break;

	/* CPU2 参数分组: 基础参数 */
	case COM_NUM_DEVICEPARAM_TANKHEIGHT:
	case COM_NUM_DEVICEPARAM_BLINDZONE:
	case COM_NUM_DEVICEPARAM_WATER_BLINDZONE:
	case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
	case COM_NUM_DEVICEPARAM_SENSORTYPE:
	case COM_NUM_DEVICEPARAM_SENSORID:
	case COM_NUM_DEVICEPARAM_SOFTWAREVERSION:
	case COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE:
		p = menu_tankbasicpara;
		break;

	/* 称重测量参数 */
	case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT:
	case COM_NUM_DEVICEPARAM_FULL_WEIGHT:
	case COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO:
	case COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO:
	case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT:
	case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT:
	case COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT:
	case COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT:
		p = menu_weightpara;
		break;

	/* 分布测量参数 */
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
		p = menu_spreadpara;
		break;

	/* 密度/温度修正 */
	case COM_NUM_DEVICEPARAM_DENSITYCORRECTION:
	case COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION:
		p = menu_correctionpara;
		break;

	/* 实高测量参数 */
	case COM_NUM_DEVICEPARAM_REFRESHTANKHEIGHTFLAG:
	case COM_NUM_DEVICEPARAM_MAXTANKHEIGHTDEVIATION:
	case COM_NUM_DEVICEPARAM_INITIALTANKHEIGHT:
	case COM_NUM_DEVICEPARAM_CURRENTTANKHEIGHT:
		p = menu_realhighpara;
		break;

	/* 水位测量参数 */
	case COM_NUM_DEVICEPARAM_WATERLEVELCORRECTION:
	case COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE:
		p = menu_waterlevelparams;
		break;

	/* 液位测量参数 */
	case COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD:
	case COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD:
		p = menu_liquidlevelparams;
		break;

	/* 报警 DO 参数 */
	case COM_NUM_DEVICEPARAM_ALARM_HIGH_DO:
	case COM_NUM_DEVICEPARAM_ALARM_LOW_DO:
	case COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD:
		p = menu_alarmdoparams;
		break;

	/* AO 参数 */
	case COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA:
	case COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA:
	case COM_NUM_DEVICEPARAM_ALARM_HIGH_AO:
	case COM_NUM_DEVICEPARAM_ALARM_LOW_AO:
	case COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA:
	case COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA:
	case COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA:
	case COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA:
	case COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA:
		p = menu_aoparams;
		break;

	default:
		if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END) {
			p = menu_paraconfig;
		} else if (now_Opera_Num > COM_NUM_PARA_LOCAL_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP) {
			p = mainmenu;
		} else if (now_Opera_Num > COM_NUM_PASSWORD_START && now_Opera_Num < COM_NUM_PASSWORD_END) {
			p = mainmenu;
		} else if (now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END) {
			p = measuremenu;
		}
		break;
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
	} else if (now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END) {
		return parascopecheck;
	} else {
		printf("非法指令\r\n");
		return errorprocess;
	}
}

/* 不带参线圈指令处理过程 */
static void cmd_nopara_process(void)
{
	static const uint32_t nopara_cmd_map[][2] = {
		{ COM_NUM_BACK_ZERO, CMD_BACK_ZERO },
		{ COM_NUM_FIND_ZERO, CMD_CALIBRATE_ZERO },
		{ COM_NUM_SPREADPOINTS_AI, CMD_MEASURE_DISTRIBUTED },
		{ COM_NUM_FIND_OIL, CMD_FIND_OIL },
		{ COM_NUM_FIND_WATER, CMD_FIND_WATER },
		{ COM_NUM_FIND_BOTTOM, CMD_FIND_BOTTOM },
		{ COM_NUM_SYNTHETIC, CMD_SYNTHETIC },
		{ COM_NUM_METER_DENSITY, CMD_MEASURE_DENSITY_METER },
		{ COM_NUM_INTERVAL_DENSITY, CMD_MEASURE_DENSITY_RANGE },
		{ COM_NUM_WARTSILA_DENSITY, CMD_WARTSILA_DENSITY_RANGE },

		{ COM_NUM_SET_EMPTY_WEIGHT, CMD_SET_EMPTY_WEIGHT },
		{ COM_NUM_SET_FULL_WEIGHT, CMD_SET_FULL_WEIGHT },
		{ COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },
		{ COM_NUM_MAINTENANCE_MODE, CMD_MAINTENANCE_MODE },
	};

	int mapamount = (int)(sizeof(nopara_cmd_map) / sizeof(nopara_cmd_map[0]));
	int i;

	for (i = 0; i < mapamount; i++) {
		if (now_Opera_Num == (int)nopara_cmd_map[i][0]) {
			CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
					HOLDREGISTER_DEVICEPARAM_COMMAND, 2,
					(uint32_t*)&nopara_cmd_map[i][1]);
			break;
		}
	}

	if (now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING) {
		oled_clear();
		if ((g_measurement.device_status.device_state != STATE_STANDBY)
			&& (g_measurement.device_status.device_state != STATE_ERROR)
			&& (g_measurement.device_status.device_state != STATE_MAINTENANCEMODE)) {
			DisplayLangaugeLineWords((uint8_t*)"失败", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Failed to set");
			DisplayLangaugeLineWords((uint8_t*)"请先进入调试模式", OLED_LINE8_1, OLED_ROW4_3, 0, (uint8_t*)"Enter debug mode");
		} else {
			DisplayLangaugeLineWords((uint8_t*)"正在恢复出厂设置", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Factory Settings");
			HAL_Delay(800);
			exitTankOpera();
		}
	} else if (now_Opera_Num == COM_NUM_MAINTENANCE_MODE) {
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"已进入维护模式", OLED_LINE8_1, OLED_ROW4_2, 0, (uint8_t*)"Maintenance Mode");
	} else {
		exitTankOpera();
	}
}

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
			if (holdValue[index].pword == NULL) {
				bits = dtm_bits();
				if (bits <= 3 && holdValue[index].unit == NULL) {
					line = OLED_LINE8_5;
				} else {
					line = OLED_LINE8_4;
				}
				OledValueDisplay(holdValue[index].val, line, OLED_ROW3_2, 0, holdValue[index].point, holdValue[index].unit);
			} else {
				if (dtm_namelength(holdValue[index].pword()) >= 12) {
					line = OLED_LINE8_2;
				} else if (dtm_namelength(holdValue[index].pword()) > 6) {
					line = OLED_LINE8_3;
				} else {
					line = OLED_LINE8_4;
				}
				OledDisplayLineWords(holdValue[index].pword(), line, OLED_ROW3_2, 0);
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
		&& holdValue[index].authority_write
		&& (g_measurement.device_status.device_state == STATE_STANDBY
			|| g_measurement.device_status.device_state == STATE_ERROR
			|| g_measurement.device_status.device_state == STATE_MAINTENANCEMODE)) {

		if (holdValue[index].pword == NULL) {
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

	if (cnt_commutoCPU2 >= COMMU_ERROR_MAX) {
		oled_clear();
		DisplayLangaugeLineWords((uint8_t*)"与CPU2通讯故障!", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Cpu2 CF!");
		HAL_Delay(800);
		return -1;
	}

	oled_clear();
	DisplayLangaugeLineWords((uint8_t*)"正在读取参数", OLED_LINE8_1, OLED_ROW3_2, 0, (uint8_t*)"Reading Para");

	index = getHoldValueNum(now_Opera_Num);
	CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
			holdValue[index].startadd,
			holdValue[index].rgstcnt,
			NULL);
	HAL_Delay(150);

	return 0;
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
	} else if (holdValue[index].flag_checkvalue) {
		if (now_Para_CT.val < holdValue[index].valuemin || now_Para_CT.val > holdValue[index].valuemax) {
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

	if (holdValue[index].data_type == TYPE_FLOAT) {
		union utof tmp_f;
		tmp_f.f = now_Para_CT.val * pow(0.1, (double)holdValue[index].point);
		for (i = 0; i < 4; i++) {
			paraarr[i] = (uint8_t)(tmp_f.u >> (8 * (3 - i)));
		}
	} else if (holdValue[index].data_type == TYPE_DOUBLE) {
		union utod tmp_d;
		tmp_d.d = now_Para_CT.val * pow(0.1, (double)holdValue[index].point);
		for (i = 0; i < 4; i++) {
			paraarr[i] = (uint8_t)(tmp_d.u[1] >> (8 * (3 - i)));
		}
		for (i = 4; i < 8; i++) {
			paraarr[i] = (uint8_t)(tmp_d.u[0] >> (8 * (7 - i)));
		}
	} else {
		now_Para_CT.val -= holdValue[index].offset;
		for (i = 0; i < holdValue[index].rgstcnt * 2 && i < arrlen; i++) {
			paraarr[i] = (uint8_t)((now_Para_CT.val >> (8 * i)) & 0xFF);
		}
	}

	CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
			holdValue[index].startadd,
			holdValue[index].rgstcnt,
			paraarr);

	CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
			holdValue[index].startadd,
			holdValue[index].rgstcnt,
			NULL);

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
		index = holdValue[index].val;
		len = (int)(sizeof(arr_densitydir) / sizeof(arr_densitydir[0]));
		p = arr_densitydir;
		break;
	}
	case COM_NUM_SCREEN_SOURCE_OIL:
	case COM_NUM_SCREEN_SOURCE_WATER:
	case COM_NUM_SCREEN_SOURCE_D:
	case COM_NUM_SCREEN_SOURCE_T: {
		index = holdValue[index].val;
		len = (int)(sizeof(arr_source) / sizeof(arr_source[0]));
		p = arr_source;
		break;
	}
	case COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT:
	case COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT:
	case COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY:
	case COM_NUM_DEVICEPARAM_REFRESHTANKHEIGHTFLAG:
	case COM_NUM_SCREEN_OFF: {
		index = holdValue[index].val;
		len = (int)(sizeof(arr_IF) / sizeof(arr_IF[0]));
		p = arr_IF;
		break;
	}
	case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE: {
		index = holdValue[index].val;
		len = (int)(sizeof(arr_densitymode) / sizeof(arr_densitymode[0]));
		p = arr_densitymode;
		break;
	}
	case COM_NUM_SCREEN_INPUT_D_SWITCH: {
		index = holdValue[index].val;
		len = (int)(sizeof(arr_Switch) / sizeof(arr_Switch[0]));
		p = arr_Switch;
		break;
	}
	case COM_NUM_PARA_LANG: {
		index = holdValue[index].val;
		len = (int)(sizeof(arr_language) / sizeof(arr_language[0]));
		p = arr_language;
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
	operationselect(parr, len - 1);
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
		{ (uint8_t*)"回零点", COM_NUM_BACK_ZERO, ifsendcmd, COMMANE_NORW, (uint8_t*)"BackZero" },
		{ (uint8_t*)"寻找液位", COM_NUM_FIND_OIL, ifsendcmd, COMMANE_NORW, (uint8_t*)"FindOil" },
		{ (uint8_t*)"寻找水位", COM_NUM_FIND_WATER, ifsendcmd, COMMANE_NORW, (uint8_t*)"FindWater" },
		{ (uint8_t*)"寻找罐底", COM_NUM_FIND_BOTTOM, ifsendcmd, COMMANE_NORW, (uint8_t*)"FindBottom" },

		{ (uint8_t*)"单点测量", COM_NUM_SINGLE_POINT, inputcmdpara, COMMANE_NORW, (uint8_t*)"SingleMeasure" },
		{ (uint8_t*)"单点监测", COM_NUM_SP_TEST, inputcmdpara, COMMANE_NORW, (uint8_t*)"SingleMonitor" },
		{ (uint8_t*)"综合测量", COM_NUM_SYNTHETIC, ifsendcmd, COMMANE_NORW, (uint8_t*)"Synthetic" },

		{ (uint8_t*)"分布测量", COM_NUM_SPREADPOINTS_AI, ifsendcmd, COMMANE_NORW, (uint8_t*)"DistMeasure" },
		{ (uint8_t*)"国标分布测量", COM_NUM_SPREADPOINTS_AI, ifsendcmd, COMMANE_NORW, (uint8_t*)"GB_DistMeasure" },

		{ (uint8_t*)"密度每米测量", COM_NUM_METER_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"MeterDensity" },
		{ (uint8_t*)"区间密度测量", COM_NUM_INTERVAL_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"RangeDensity" },
		{ (uint8_t*)"瓦锡兰区间密度", COM_NUM_WARTSILA_DENSITY, ifsendcmd, COMMANE_NORW, (uint8_t*)"WartsilaRange" },

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
		{ (uint8_t*)"上行", COM_NUM_RUNUP, inputcmdpara, COMMANE_NORW, (uint8_t*)"MoveUp" },
		{ (uint8_t*)"下行", COM_NUM_RUNDOWN, inputcmdpara, COMMANE_NORW, (uint8_t*)"MoveDown" },

		{ (uint8_t*)"标定零点", COM_NUM_FIND_ZERO, ifsendcmd, COMMANE_NORW, (uint8_t*)"CalZero" },
		{ (uint8_t*)"标定液位", COM_NUM_CAL_OIL, inputcmdpara, COMMANE_NORW, (uint8_t*)"CalOil" },
		{ (uint8_t*)"修正液位", COM_NUM_CORRECTION_OIL, inputcmdpara, COMMANE_NORW, (uint8_t*)"CorrectOil" },

		{ (uint8_t*)"设置空载称重", COM_NUM_SET_EMPTY_WEIGHT, inputcmdpara, COMMANE_NORW, (uint8_t*)"SetEmptyWt" },
		{ (uint8_t*)"设置满载称重", COM_NUM_SET_FULL_WEIGHT, inputcmdpara, COMMANE_NORW, (uint8_t*)"SetFullWt" },

		{ (uint8_t*)"恢复出厂设置", COM_NUM_RESTOR_EFACTORYSETTING, ifsendcmd, COMMANE_NORW, (uint8_t*)"RestoreFactory" },
		{ (uint8_t*)"维护模式", COM_NUM_MAINTENANCE_MODE, ifsendcmd, COMMANE_NORW, (uint8_t*)"Maintenance" },

		{ (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_CMD_MAIN;
	debugmode_back = 1;
	menuselect(menu, menulen);
}

/* 参数配置菜单 */
static void menu_paraconfig(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"基础参数", COM_NUM_NOOPERA, menu_tankbasicpara, COMMANE_NORW, (uint8_t*)"BasicPara" },
		{ (uint8_t*)"称重测量参数", COM_NUM_NOOPERA, menu_weightpara, COMMANE_NORW, (uint8_t*)"WeightPara" },
		{ (uint8_t*)"分布测量参数", COM_NUM_NOOPERA, menu_spreadpara, COMMANE_NORW, (uint8_t*)"SpreadPara" },
		{ (uint8_t*)"瓦锡兰分布测参数", COM_NUM_NOOPERA, menu_wartsilapara, COMMANE_NORW, (uint8_t*)"WartsilaPara" },
		{ (uint8_t*)"磁通量", COM_NUM_NOOPERA, menu_correctionpara, COMMANE_NORW, (uint8_t*)"MAGNETIC" },
		{ (uint8_t*)"实高测量参数", COM_NUM_NOOPERA, menu_realhighpara, COMMANE_NORW, (uint8_t*)"RealHighPara" },
		{ (uint8_t*)"水位测量参数", COM_NUM_NOOPERA, menu_waterlevelparams, COMMANE_NORW, (uint8_t*)"DebugInfo" },
		{ (uint8_t*)"液位测量参数", COM_NUM_NOOPERA, menu_liquidlevelparams, COMMANE_NORW, (uint8_t*)"DebugInfo" },
		{ (uint8_t*)"继电器输出配置", COM_NUM_NOOPERA, menu_alarmdoparams, COMMANE_NORW, (uint8_t*)"DebugInfo" },
		{ (uint8_t*)"电流输出配置", COM_NUM_NOOPERA, menu_aoparams, COMMANE_NORW, (uint8_t*)"DebugInfo" },
		{ (uint8_t*)"界面显示参数", COM_NUM_NOOPERA, menu_screen, COMMANE_NORW, (uint8_t*)"ScreenPara" },
		{ (uint8_t*)"CPU3通信配置", COM_NUM_NOOPERA, menu_cpu3_comm, COMMANE_NORW, (uint8_t*)"CPU3Comm" },

		{ (uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_MAIN;
	debugmode_back = 0;
	menuselect(menu, menulen);
}

/* 基础参数菜单 */
static void menu_tankbasicpara(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"罐高", COM_NUM_DEVICEPARAM_TANKHEIGHT, para_mainprocess, COMMAND_READ, (uint8_t*)"TankHeight" },
		{ (uint8_t*)"液位盲区", COM_NUM_DEVICEPARAM_BLINDZONE, para_mainprocess, COMMAND_READ, (uint8_t*)"BlindZone" },
		{ (uint8_t*)"水位盲区", COM_NUM_DEVICEPARAM_WATER_BLINDZONE, para_mainprocess, COMMAND_READ, (uint8_t*)"WaterBlind" },

		{ (uint8_t*)"编码轮周长", COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, para_mainprocess, COMMAND_READ, (uint8_t*)"EncWheelCirc" },
		{ (uint8_t*)"首圈周长", COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM, para_mainprocess, COMMAND_READ, (uint8_t*)"FirstLoopCirc" },
		{ (uint8_t*)"尺带厚度", COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM, para_mainprocess, COMMAND_READ, (uint8_t*)"TapeThick" },
		{ (uint8_t*)"电机最大速度", COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED, para_mainprocess, COMMAND_READ, (uint8_t*)"MaxMotorSpd" },

		{ (uint8_t*)"传感器类型", COM_NUM_DEVICEPARAM_SENSORTYPE, para_mainprocess, COMMANE_NORW, (uint8_t*)"SensorType" },
		{ (uint8_t*)"传感器编号", COM_NUM_DEVICEPARAM_SENSORID, para_mainprocess, COMMANE_NORW, (uint8_t*)"SensorID" },
		{ (uint8_t*)"传感器软件版本", COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION, para_mainprocess, COMMANE_NORW, (uint8_t*)"SenSWVer" },
		{ (uint8_t*)"LTD软件版本", COM_NUM_DEVICEPARAM_SOFTWAREVERSION, para_mainprocess, COMMANE_NORW, (uint8_t*)"FWVersion" },

		{ (uint8_t*)"上电默认指令", COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND, para_mainprocess, COMMAND_READ, (uint8_t*)"PwrOnCmd" },
		{ (uint8_t*)"找零下行距离", COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE, para_mainprocess, COMMAND_READ, (uint8_t*)"FindZeroDown" },

		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_BASIC;
	menuselect(menu, menulen);
}

/* 称重参数菜单 */
static void menu_weightpara(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"空载重量", COM_NUM_DEVICEPARAM_EMPTY_WEIGHT, para_mainprocess, COMMAND_READ, (uint8_t*)"EmptyWeight" },
		{ (uint8_t*)"满载称重", COM_NUM_DEVICEPARAM_FULL_WEIGHT, para_mainprocess, COMMAND_READ, (uint8_t*)"FullWeight" },
		{ (uint8_t*)"获取空载阈值", COM_NUM_SET_EMPTY_WEIGHT, ifsendcmd, COMMANE_NORW, (uint8_t*)"Set empty Weight" },
		{ (uint8_t*)"获取满载阈值", COM_NUM_SET_FULL_WEIGHT, ifsendcmd, COMMANE_NORW, (uint8_t*)"Set full Weight" },
		{ (uint8_t*)"称重上限比例(%)", COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO, para_mainprocess, COMMAND_READ, (uint8_t*)"WeightUpperRatio" },
		{ (uint8_t*)"称重下限比例(%)", COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO, para_mainprocess, COMMAND_READ, (uint8_t*)"WeightLowerRatio" },
		{ (uint8_t*)"空载称重上限", COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"EmptyWtHi" },
		{ (uint8_t*)"空载称重下限", COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"EmptyWtLo" },
		{ (uint8_t*)"满载称重上限", COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"FullWtHi" },
		{ (uint8_t*)"满载称重下限", COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"FullWtLo" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_SYNTH;
	menuselect(menu, menulen);
}

/* 修正参数菜单 */
static void menu_correctionpara(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"磁通量T", COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION, para_mainprocess, COMMAND_READ, (uint8_t*)"MAGNETIC_T" },
		{ (uint8_t*)"磁通量D", COM_NUM_DEVICEPARAM_DENSITYCORRECTION, para_mainprocess, COMMAND_READ, (uint8_t*)"MAGNETIC_D" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_CORRECT;
	menuselect(menu, menulen);
}

/* 分布测量参数菜单 */
static void menu_spreadpara(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"测量罐底(0否1是)", COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, para_mainprocess, COMMAND_READ, (uint8_t*)"ReqBottomMeas" },
		{ (uint8_t*)"测量水位(0否1是)", COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT, para_mainprocess, COMMAND_READ, (uint8_t*)"ReqWaterMeas" },
		{ (uint8_t*)"测单点密度(0否1是)", COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, para_mainprocess, COMMAND_READ, (uint8_t*)"ReqSingleDensity" },
		{ (uint8_t*)"测量顺序", COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER, para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadOrder" },
		{ (uint8_t*)"测量模式", COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE, para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadMode" },
		{ (uint8_t*)"测量数量", COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT, para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadCount" },
		{ (uint8_t*)"测量间距", COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadDistance" },
		{ (uint8_t*)"上限距离(距液面)", COM_NUM_DEVICEPARAM_SPREADTOPLIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadTopLimit" },
		{ (uint8_t*)"下限距离(距罐底)", COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadBottomLimit" },
		{ (uint8_t*)"测量前悬停时间", COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME, para_mainprocess, COMMAND_READ, (uint8_t*)"HoverTime" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_SPREAD;
	menuselect(menu, menulen);
}

/* 瓦锡兰参数菜单 */
static void menu_wartsilapara(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"密度上限", COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"WUpperDen" },
		{ (uint8_t*)"密度下限", COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT, para_mainprocess, COMMAND_READ, (uint8_t*)"WLowerDen" },
		{ (uint8_t*)"密度步进", COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL, para_mainprocess, COMMAND_READ, (uint8_t*)"WDenStep" },
		{ (uint8_t*)"最高测点距液面", COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE, para_mainprocess, COMMAND_READ, (uint8_t*)"WMaxHgt" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_WARTSILA_PARACFG_BASE;
	menuselect(menu, menulen);
}

/* 水位测量参数 */
static void menu_waterlevelparams(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"水位修正值", COM_NUM_DEVICEPARAM_WATERLEVELCORRECTION, para_mainprocess, COMMAND_READ, (uint8_t*)"WaterLevelCorr" },
		{ (uint8_t*)"最大下行距离", COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE, para_mainprocess, COMMAND_READ, (uint8_t*)"MaxDownDist" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_WATERLEVEL;
	menuselect(menu, menulen);
}

/* 磁通量菜单(旧, 保留) */
static void menu_magnetic(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"磁通量D", COM_NUM_DEVICEPARAM_DENSITYCORRECTION, para_mainprocess, COMMAND_READ, (uint8_t*)"Flux D" },
		{ (uint8_t*)"磁通量T", COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION, para_mainprocess, COMMAND_READ, (uint8_t*)"Flux T" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MAGNETIC;
	menuselect(menu, menulen);
}

/* 实高测量参数 */
static void menu_realhighpara(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"更新罐高(0否1是)", COM_NUM_DEVICEPARAM_REFRESHTANKHEIGHTFLAG, para_mainprocess, COMMAND_READ, (uint8_t*)"RefreshTankH" },
		{ (uint8_t*)"罐高最大偏差", COM_NUM_DEVICEPARAM_MAXTANKHEIGHTDEVIATION, para_mainprocess, COMMAND_READ, (uint8_t*)"MaxTankDev" },
		{ (uint8_t*)"初始实高", COM_NUM_DEVICEPARAM_INITIALTANKHEIGHT, para_mainprocess, COMMAND_READ, (uint8_t*)"InitTankH" },
		{ (uint8_t*)"当前实高", COM_NUM_DEVICEPARAM_CURRENTTANKHEIGHT, para_mainprocess, COMMAND_READ, (uint8_t*)"CurrTankH" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_REALHIGH;
	menuselect(menu, menulen);
}

/* 液位测量参数 */
static void menu_liquidlevelparams(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"找油阈值", COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD, para_mainprocess, COMMAND_READ, (uint8_t*)"OilLvlTh" },
		{ (uint8_t*)"滞后阈值", COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD, para_mainprocess, COMMAND_READ, (uint8_t*)"HysTh" },
		{ (uint8_t*)"液位测量方式", COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, para_mainprocess, COMMAND_READ, (uint8_t*)"LvlMeasMode" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_LIQUIDLEVEL;
	menuselect(menu, menulen);
}

/* 报警 DO 参数 */
static void menu_alarmdoparams(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"高液位报警(DO)", COM_NUM_DEVICEPARAM_ALARM_HIGH_DO, para_mainprocess, COMMAND_READ, (uint8_t*)"AlarmHiDO" },
		{ (uint8_t*)"低液位报警(DO)", COM_NUM_DEVICEPARAM_ALARM_LOW_DO, para_mainprocess, COMMAND_READ, (uint8_t*)"AlarmLoDO" },
		{ (uint8_t*)"第三状态阈值", COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD, para_mainprocess, COMMAND_READ, (uint8_t*)"ThirdStateTh" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_ALARM_DO;
	menuselect(menu, menulen);
}

/* 4-20mA 输出/AO 参数 */
static void menu_aoparams(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"输出范围起点(mA)", COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"RangeStart" },
		{ (uint8_t*)"输出范围终点(mA)", COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"RangeEnd" },
		{ (uint8_t*)"高限报警(AO)", COM_NUM_DEVICEPARAM_ALARM_HIGH_AO, para_mainprocess, COMMAND_READ, (uint8_t*)"AlarmHiAO" },
		{ (uint8_t*)"低限报警(AO)", COM_NUM_DEVICEPARAM_ALARM_LOW_AO, para_mainprocess, COMMAND_READ, (uint8_t*)"AlarmLoAO" },
		{ (uint8_t*)"初始电流(mA)", COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"InitCurrent" },
		{ (uint8_t*)"高位电流(mA)", COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"HighCurrent" },
		{ (uint8_t*)"低位电流(mA)", COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"LowCurrent" },
		{ (uint8_t*)"故障电流(mA)", COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"FaultCurrent" },
		{ (uint8_t*)"调试电流(mA)", COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA, para_mainprocess, COMMAND_READ, (uint8_t*)"DebugCurrent" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_PARACFG_DEBUG_AO;
	menuselect(menu, menulen);
}

/* 界面显示类参数 */
static void menu_screen(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"小数点位数", COM_NUM_SCREEN_DECIMAL, para_mainprocess, COMMAND_READ, (uint8_t*)"DecimalPlaces" },
		{ (uint8_t*)"屏幕密码", COM_NUM_SCREEN_PASSWARD, para_mainprocess, COMMAND_READ, (uint8_t*)"Password" },
		{ (uint8_t*)"是否息屏", COM_NUM_SCREEN_OFF, para_mainprocess, COMMAND_READ, (uint8_t*)"Screen Off" },
		{ (uint8_t*)"数据源", COM_NUM_NOOPERA, menu_scr_source, COMMAND_READ, (uint8_t*)"Data Source" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_SCREEN;
	menuselect(menu, menulen);
}

/* 数据源菜单 */
static void menu_scr_source(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"液位数据源", COM_NUM_SCREEN_SOURCE_OIL, para_mainprocess, COMMAND_READ, (uint8_t*)"Level Source" },
		{ (uint8_t*)"液位手工输入值", COM_NUM_SCREEN_INPUT_OIL, para_mainprocess, COMMAND_READ, (uint8_t*)"LevelInputVal" },
		{ (uint8_t*)"水位数据源", COM_NUM_SCREEN_SOURCE_WATER, para_mainprocess, COMMAND_READ, (uint8_t*)"Water Source" },
		{ (uint8_t*)"水位手工输入值", COM_NUM_SCREEN_INPUT_WATER, para_mainprocess, COMMAND_READ, (uint8_t*)"WaterInputVal" },
		{ (uint8_t*)"密度数据源", COM_NUM_SCREEN_SOURCE_D, para_mainprocess, COMMAND_READ, (uint8_t*)"Densi Source" },
		{ (uint8_t*)"密度手工输入值", COM_NUM_SCREEN_INPUT_D, para_mainprocess, COMMAND_READ, (uint8_t*)"DensiInputVal" },
		{ (uint8_t*)"密度手输值上传开关", COM_NUM_SCREEN_INPUT_D_SWITCH, para_mainprocess, COMMAND_READ, (uint8_t*)"DensiSwitch" },
		{ (uint8_t*)"温度数据源", COM_NUM_SCREEN_SOURCE_T, para_mainprocess, COMMAND_READ, (uint8_t*)"Temp Source" },
		{ (uint8_t*)"温度手工输入值", COM_NUM_SCREEN_INPUT_T, para_mainprocess, COMMAND_READ, (uint8_t*)"TempInputVal" },
		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_screen, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_SOURCE;
	menuselect(menu, menulen);
}

/* CPU3 通信配置菜单 */
static void menu_cpu3_comm(void)
{
	static struct MenuData menu[] = {
		{ (uint8_t*)"COM1波特率", COM_NUM_CPU3_COM1_BAUDRATE, para_mainprocess, COMMAND_READ, (uint8_t*)"COM1 Baud" },
		{ (uint8_t*)"COM1数据位", COM_NUM_CPU3_COM1_DATABITS, para_mainprocess, COMMAND_READ, (uint8_t*)"COM1 Databits" },
		{ (uint8_t*)"COM1校验", COM_NUM_CPU3_COM1_PARITY, para_mainprocess, COMMAND_READ, (uint8_t*)"COM1 Parity" },
		{ (uint8_t*)"COM1停止位", COM_NUM_CPU3_COM1_STOPBITS, para_mainprocess, COMMAND_READ, (uint8_t*)"COM1 StopBits" },
		{ (uint8_t*)"COM1协议", COM_NUM_CPU3_COM1_PROTOCOL, para_mainprocess, COMMAND_READ, (uint8_t*)"COM1 Proto" },

		{ (uint8_t*)"COM2波特率", COM_NUM_CPU3_COM2_BAUDRATE, para_mainprocess, COMMAND_READ, (uint8_t*)"COM2 Baud" },
		{ (uint8_t*)"COM2数据位", COM_NUM_CPU3_COM2_DATABITS, para_mainprocess, COMMAND_READ, (uint8_t*)"COM2 Databits" },
		{ (uint8_t*)"COM2校验", COM_NUM_CPU3_COM2_PARITY, para_mainprocess, COMMAND_READ, (uint8_t*)"COM2 Parity" },
		{ (uint8_t*)"COM2停止位", COM_NUM_CPU3_COM2_STOPBITS, para_mainprocess, COMMAND_READ, (uint8_t*)"COM2 StopBits" },
		{ (uint8_t*)"COM2协议", COM_NUM_CPU3_COM2_PROTOCOL, para_mainprocess, COMMAND_READ, (uint8_t*)"COM2 Proto" },

		{ (uint8_t*)"COM3波特率", COM_NUM_CPU3_COM3_BAUDRATE, para_mainprocess, COMMAND_READ, (uint8_t*)"COM3 Baud" },
		{ (uint8_t*)"COM3数据位", COM_NUM_CPU3_COM3_DATABITS, para_mainprocess, COMMAND_READ, (uint8_t*)"COM3 Databits" },
		{ (uint8_t*)"COM3校验", COM_NUM_CPU3_COM3_PARITY, para_mainprocess, COMMAND_READ, (uint8_t*)"COM3 Parity" },
		{ (uint8_t*)"COM3停止位", COM_NUM_CPU3_COM3_STOPBITS, para_mainprocess, COMMAND_READ, (uint8_t*)"COM3 StopBits" },
		{ (uint8_t*)"COM3协议", COM_NUM_CPU3_COM3_PROTOCOL, para_mainprocess, COMMAND_READ, (uint8_t*)"COM3 Proto" },

		{ (uint8_t*)"退出", COM_NUM_NOOPERA, menu_paraconfig, COMMANE_NORW, (uint8_t*)"Exit" },
	};

	int menulen = (int)(sizeof(menu) / sizeof(menu[0]));

	all_screen(0x00);
	func_index = KEYNUM_MENU_CPU3_COMM;
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

/* 带一参线圈指令处理过程 */
static void cmd_onepara_process(void)
{
	int index;
	int i;
	int arrlen = 8;
	static uint8_t paraarr[8];

	static uint32_t onepara_cmd_map[][2] = {
		{ COM_NUM_SINGLE_POINT, CMD_MEASURE_SINGLE },
		{ COM_NUM_SP_TEST, CMD_MONITOR_SINGLE },
		{ COM_NUM_CAL_OIL, CMD_CALIBRATE_OIL },
		{ COM_NUM_RUNUP, CMD_MOVE_UP },
		{ COM_NUM_RUNDOWN, CMD_MOVE_DOWN },
		{ COM_NUM_CORRECTION_OIL, CMD_CORRECT_OIL },
	};

	int mapamount = (int)(sizeof(onepara_cmd_map) / sizeof(onepara_cmd_map[0]));

	index = getHoldValueNum(now_Opera_Num);
	if (index == -1) {
		all_screen(0x00);
		DisplayLangaugeLineWords((uint8_t*)"非法参数！", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Invalid Para");
		HAL_Delay(800);
	} else {
		all_screen(0x00);
		DisplayLangaugeLineWords((uint8_t*)"正在下发参数", OLED_LINE8_2, OLED_ROW3_2, 0, (uint8_t*)"Send Para");

		for (i = 0; i < holdValue[index].rgstcnt * 2 && i < arrlen; i++) {
			paraarr[i] = (uint8_t)((now_Para_CT.val >> (8 * i)) & 0xFF);
		}

		CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
				holdValue[index].startadd,
				holdValue[index].rgstcnt,
				paraarr);

		DisplayLangaugeLineWords((uint8_t*)"正在下发指令", OLED_LINE8_2, OLED_ROW3_3, 0, (uint8_t*)"Send Command");

		for (i = 0; i < mapamount; i++) {
			if (now_Opera_Num == (int)onepara_cmd_map[i][0]) {
				CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
						HOLDREGISTER_DEVICEPARAM_COMMAND, 2,
						&onepara_cmd_map[i][1]);
				break;
			}
		}
	}

	exitTankOpera();
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
