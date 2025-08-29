#include "display_tankopera.h"
#include "tim.h"
#include "usart.h"
#include "exit.h"
#include "Display.h"
#include "hgs.h"
#include "modbus_agreement.h"
#include <math.h>
#include "communicate.h"
#include "system_parameter.h"

#define PASSWORD_ENTERMAIN 1009

typedef void (*pFunc_void)(void);
bool flag_SendCommand = false;
struct PAGENUM {
    int menu_num;//菜单第几栏序号
    int menu_cnt;//上下键计数
    int menu_page;//当前显示的页数
};
struct PAGENUM PageNum[KEYNUM_END];
struct ParaContent now_Para_CT;//当前设置的参数内容

static int func_index = 0;//菜单索引
static int NowKeyPress = 0;//本次按下的按键
static int now_Opera_Num = 0;//当前选择的指令或参数的序号
static int timesure = 0;//按下确定键的次数
static int timeback = 0;//按下返回键的次数
static int debugmode_back = 0;//进入调试模式返回到哪个菜单

static uint8_t* arr_powerontodo[][2] = {
    {(uint8_t*)"上电后液位跟随",(uint8_t*)"Level Following"},
    {(uint8_t*)"上电后待机",(uint8_t*)"Standby"},
};
static uint8_t* arr_densitydir[][2] = {
    {(uint8_t*)"向上",(uint8_t*)"UP"},
    {(uint8_t*)"向下",(uint8_t*)"DOWN"},
    {(uint8_t*)"非法配置",(uint8_t*)"Illegal CFG"},
};
static uint8_t* arr_source[][2] = {
    {(uint8_t*)"设备测量",(uint8_t*)"Measurement"},
    {(uint8_t*)"手工输入",(uint8_t*)"Input"},
    {(uint8_t*)"非法配置",(uint8_t*)"Illegal CFG"},
};
static uint8_t* arr_IF[][2] = {
    {(uint8_t*)"否",(uint8_t*)"NO"},
    {(uint8_t*)"是",(uint8_t*)"YES"},
    {(uint8_t*)"非法配置",(uint8_t*)"Illegal CFG"},
};
static uint8_t* arr_densitymode[][2] = {
    {(uint8_t*)"分布测量",(uint8_t*)"Distribution"},
    {(uint8_t*)"国标测量",(uint8_t*)"National Standard"},
    {(uint8_t*)"每米测量",(uint8_t*)"Per Meter"},
    {(uint8_t*)"固定点测量",(uint8_t*)"Fixed Point"},
    {(uint8_t*)"区间测量",(uint8_t*)"Interval"},
    {(uint8_t*)"非法配置",(uint8_t*)"Illegal CFG"},
};
static uint8_t* arr_Switch[][2] = {
    {(uint8_t*)"关闭",(uint8_t*)"CLOSE"},
    {(uint8_t*)"开启",(uint8_t*)"OPEN"},
    {(uint8_t*)"非法配置",(uint8_t*)"Illegal CFG"},
};
static uint8_t* arr_language[][2] = {
    {(uint8_t*)"中文",(uint8_t*)"Chinese"},
    {(uint8_t*)"英文",(uint8_t*)"English"},
    {(uint8_t*)"非法配置",(uint8_t*)"Illegal CFG"},
};


static void inputcmdpara(void);
static uint8_t* dtm_operaname(int num);
static uint8_t dtm_namelength(uint8_t* name);
static uint8_t dtm_points(void);
static uint8_t* dtm_unit(void);
static uint8_t dtm_bits(void);
static bool inputvalue(uint8_t deci,uint8_t row,uint8_t line,uint8_t points,uint8_t* unit,int *value);
static void ifsendcmd(void);
static void mainmenu(void);
static void ifentermainmenu(void);
static void menuselect(struct MenuData* menu,int menulen);
static void ifexittankopera(void);
static void measuremenu(void);
static void displaypara(void);
static void errorprocess(void);
static void parawritecheck(void);
static pFunc_void dtm_suretofunc(void);
static void cmd_nopara_process(void);
static pFunc_void dtm_backtofunc(void);
static void password_enter_para(void);
static void menu_paraconfig(void);
static void menu_basepara(void);
static int get_para_data(void);
static void parascopecheck(void);
static void cmd_configpara_process(void);
static void para_mainprocess(void);
static void password_enter_cmd(void);
static void menu_cmdconfig_main(void);
static void cmd_onepara_process(void);
static void menu_magnetic(void);
static int SignInput(uint8_t row,uint8_t line,uint8_t shift);
uint8_t *(*dtm_disarr(int* pindex, int* plen))[2];
static void selectparaword(void);
static void operationselect(uint8_t* (*menu)[2],int menulen);
static void menu_screen(void);
static void menu_scr_source(void);
static void menu_paraconfig(void);//菜单 - 维护模式参数
static uint8_t* returnWordType(uint8_t* chinese,uint8_t* english);

static void menu_tankbasicpara(void);
static void menu_syntheticpara(void);
static void menu_spreadpara(void);
static void menu_correctionpara(void);
static void menu_realhighpara(void);
static void menu_debuginfo(void);
static void setlanguage(void);
static void setchinese(void);
static void setenglish(void);
struct KeyMenu keymenu[128] = {
    // 0 - 是否进入罐上操作
    {exitTankOpera,NULL,NULL,mainmenu,USE_KEY_BACK|USE_KEY_SURE,ifentermainmenu},

    // 1 - 主菜单
    {ifexittankopera,mainmenu,mainmenu,mainmenu,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,mainmenu},

    // 2 - 是否退出罐上操作
    {mainmenu,NULL,NULL,exitTankOpera,USE_KEY_BACK|USE_KEY_SURE,ifexittankopera},

    // 3 - 普通测量指令主菜单
    {measuremenu,measuremenu,measuremenu,measuremenu,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,measuremenu},

    // 4 - 参数配置主菜单
    {menu_paraconfig,menu_paraconfig,menu_paraconfig,menu_paraconfig,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_paraconfig},

    // 5 - 菜单 - 基础参数
    {menu_basepara,menu_basepara,menu_basepara,menu_basepara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_basepara},

    // 6 - 菜单 - 维护调试指令
    {menu_cmdconfig_main,menu_cmdconfig_main,menu_cmdconfig_main,menu_cmdconfig_main,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_cmdconfig_main},

    // 7 - 是否下发指令或参数
    {ifsendcmd,NULL,NULL,ifsendcmd,USE_KEY_BACK|USE_KEY_SURE,ifsendcmd},

    // 8 - 请输入参数值(带参指令中的)
    {inputcmdpara,inputcmdpara,inputcmdpara,inputcmdpara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,inputcmdpara},

    // 9 - 参数显示(读写类参数中的)
    {displaypara,NULL,NULL,displaypara,USE_KEY_BACK|USE_KEY_SURE,displaypara},

    // 10 - 菜单 - 磁通量参数
    {menu_magnetic,menu_magnetic,menu_magnetic,menu_magnetic,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_magnetic},

    // 11 - 隐藏信息选择
    {selectparaword,selectparaword,selectparaword,selectparaword,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,selectparaword},

    // 12 - 菜单 - 界面显示类参数
    {menu_screen,menu_screen,menu_screen,menu_screen,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_screen},

    // 13 - 菜单 - 数据源
    {menu_scr_source,menu_scr_source,menu_scr_source,menu_scr_source,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_scr_source},

    // 14 - 维护参数 - 基础参数
    {menu_tankbasicpara,menu_tankbasicpara,menu_tankbasicpara,menu_tankbasicpara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_tankbasicpara},

    // 15 - 维护参数 - 综合测量参数
    {menu_syntheticpara,menu_syntheticpara,menu_syntheticpara,menu_syntheticpara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_syntheticpara},

    // 16 - 维护参数 - 分布测量参数
    {menu_spreadpara,menu_spreadpara,menu_spreadpara,menu_spreadpara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_spreadpara},

    // 17 - 维护参数 - 修正参数
    {menu_correctionpara,menu_correctionpara,menu_correctionpara,menu_correctionpara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_correctionpara},

    // 18 - 维护参数 - 实高测量
    {menu_realhighpara,menu_realhighpara,menu_realhighpara,menu_realhighpara,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_realhighpara},

    // 19 - 维护参数 - 调试信息
    {menu_debuginfo,menu_debuginfo,menu_debuginfo,menu_debuginfo,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,menu_debuginfo},
	
	    // 20 - 显示设置 - 语言
    {setlanguage,setlanguage,setlanguage,setlanguage,USE_KEY_BACK|USE_KEY_UP|USE_KEY_DOWN|USE_KEY_SURE,setlanguage},
};





/*按键操作处理*/
void KeyProcess(uint8_t keypress)
{
    if((keypress & keymenu[func_index].keyauthority) != 0)
    {
        NowKeyPress = keypress;
        useKey();
        if(keypress == USE_KEY_BACK && keymenu[func_index].back_opera != NULL)
            keymenu[func_index].back_opera();
        else if(keypress == USE_KEY_UP && keymenu[func_index].up_opera != NULL)
            keymenu[func_index].up_opera();
        else if(keypress == USE_KEY_DOWN && keymenu[func_index].down_opera != NULL)
            keymenu[func_index].down_opera();
        else if(keypress == USE_KEY_SURE && keymenu[func_index].sure_opera != NULL)
            keymenu[func_index].sure_opera();
        else
            printf("NULL\r\n");
    }
}
/*使用了按键-更新一下按键检测定时器*/
void useKey(void)
{
    static int keytimeout = 300;//按键超时时间
//    Timer1Start(keytimeout);
}
/*退出罐上操作*/
void exitTankOpera(void)
{
    FlagofTankOpera = false;
    HAL_TIM_Base_Stop_IT(&htim1);
//    Timer1Stop();
    oled_clear();
    ClearPageNum();
    DisplayLangaugeLineWords((uint8_t*)"正在退出罐上操作",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Exit operation");
//	 CPU2_CombinatePackage_Send(FUNCTIONCODE_06,0x000E,0xFF00,NULL);
}
/* 输入参数的数值 */
static void inputcmdpara(void)
{
    uint8_t line,row = OLED_ROW4_1;
    int value;
    uint8_t namelen = 0;
    uint8_t* name = NULL;
    static bool flag_inputsign = false;
    
    oled_clear();
    func_index = KEYNUM_INPUTCMDPARA;
    line = DisplayLangaugeLineWords((uint8_t*)"请输入",OLED_LINE8_1,OLED_ROW4_1,0,(u8*)"Please enter ");
    name = dtm_operaname(now_Opera_Num); /* 名称 */
    namelen = dtm_namelength(name); /* 名称长度 */
    if(namelen > 10 || screen_parameter.language != LANGUAGE_CHINESE)
    {
        row = OLED_ROW4_2;
        line = OLED_LINE8_1;
    }
    OledDisplayLineWords(name,line,row,0); /* 确定参数名称 */
    if(namelen > 10 || screen_parameter.language != LANGUAGE_CHINESE)
        row = OLED_ROW4_3;
    else 
        row = OLED_ROW3_2;
    now_Para_CT.points = dtm_points(); /* 确定小数点位数 */
    now_Para_CT.unit = dtm_unit(); /* 确定单位 */
    now_Para_CT.bits = dtm_bits(); /* 确定显示几位 */
    if(now_Para_CT.bits <= 3 && now_Para_CT.unit == NULL) line = OLED_LINE8_5;
    else line = OLED_LINE8_4;

    if(flag_inputsign == false)
    {
        if(inputvalue(now_Para_CT.bits,row,line,now_Para_CT.points,now_Para_CT.unit,&value))
        {
            now_Para_CT.val = value;
            if(now_Opera_Num == COM_NUM_MAGNETIC_D || now_Opera_Num == COM_NUM_MAGNETIC_T)
            {
                flag_inputsign = true;
                SignInput(row,line - 4,1);
                return;
            }
            ifsendcmd();
            return;
        }
    }
    else
    {
        static int sgn = 0;
        sgn = SignInput(row,line - 4,1);
        if(sgn != 0)
        {
            now_Para_CT.val *= sgn;
            sgn = 0;
            flag_inputsign = false;
            ifsendcmd();
            return;
        }
        return;
    }
}
/* 返回操作名称的字符串地址 */
static uint8_t* dtm_operaname(int num)
{
    //指令名称
	static uint8_t* OperaNameArr_cmd[][2] = {
		{(uint8_t*)"进入调试模式",   (uint8_t*)"Set Debug Mode"},
		{(uint8_t*)"回零点",         (uint8_t*)"Return to Zero"},
		{(uint8_t*)"标定零点",       (uint8_t*)"Zero Calibration"},
		{(uint8_t*)"自动分布测量",   (uint8_t*)"Auto Spread-M"},
		{(uint8_t*)"寻找液位",       (uint8_t*)"Find Oil Level"},
		{(uint8_t*)"寻找水位",       (uint8_t*)"Find Water Level"},
		{(uint8_t*)"寻找罐底",       (uint8_t*)"Find Tank Bottom"},
		{(uint8_t*)"综合测量",       (uint8_t*)"Comprehensive-M"},
		{(uint8_t*)"每米测量",       (uint8_t*)"DT-PerMeter-M"},
		{(uint8_t*)"区间测量",       (uint8_t*)"Interval-M"},
	};
    static uint8_t* OperaNameArr_local[][2] = {
        {(uint8_t*)"设备地址",(uint8_t*)"DeviceAddress"},
        {(uint8_t*)"屏幕程序版本",(uint8_t*)"Screen FW Ver"},
    };
    static uint8_t* OperaNameArr_nopara_cmd[][2] = {
		{(uint8_t*)"读部件参数",     (uint8_t*)"Read Parameters"},
		{(uint8_t*)"置零点",         (uint8_t*)"Force Set Zero"},
    };
    
    if(num > COM_NUM_NOPARACMD_NORMAL_START && num < COM_NUM_NOPARACMD_NORMAL_STOP)
    {//无参 - 普通 - 指令
        return OperaNameArr_cmd[num - COM_NUM_NOPARACMD_NORMAL_START - 1][screen_parameter.language];
    }
    else if((num > COM_NUM_PARA_DEBUG_START && num < COM_NUM_PARACONFIG_END)
        || (num > COM_NUM_ONEPARACMD_START && num < COM_NUM_NOPARA_DEBUGCMD_END))
    {//参数类、带一参\两参指令中的参数
        int index;
        index = getHoldValueNum(now_Opera_Num);
        if(screen_parameter.language == LANG_CHINESE)
            return holdValue[index].name;
        else if(screen_parameter.language == LANG_ENGLISH)
            return holdValue[index].name_English;
    }
    else if(num > COM_NUM_PASSWORD_START && num < COM_NUM_PASSWORD_END)
        return returnWordType((uint8_t*)"密码",(uint8_t*)"Password");
    else if(num == COM_NUM_SET_WORKPATTERN)
        return returnWordType((uint8_t*)"进入调试模式",(uint8_t*)"Go into debug");
    else if(num > COM_NUM_PARA_LOCAL_START && num < COM_NUM_PARA_LOCAL_STOP)
    {//本机参数类
        return OperaNameArr_local[num - COM_NUM_PARA_LOCAL_START - 1][screen_parameter.language];
    }
    else if(num > COM_NUM_DEBUGCMD_START && num < COM_NUM_DEBUGCMD_STOP)
        return OperaNameArr_nopara_cmd[num - COM_NUM_DEBUGCMD_START - 1][screen_parameter.language];
  

    
    return returnWordType((uint8_t*)"非法操作",(uint8_t*)"Invalid Operation");
}static uint8_t* returnWordType(uint8_t* chinese,uint8_t* english)
{
    if(screen_parameter.language == LANGUAGE_CHINESE)
        return chinese;
    else if(screen_parameter.language == LANGUAGE_ENGLISH)
        return english;
    else 
        return returnWordType((uint8_t*)"语言错误",(uint8_t*)"LANGUAGE ERROR");
}

/* 求名称长度 */
static uint8_t dtm_namelength(uint8_t* name)
{
    int len = 0;
    while(*(name + len) != 0)
        len++;
    return len;
}
/* 确定小数点位数 */
static uint8_t dtm_points(void)
{
    uint8_t p = 0;
    
    if((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARACONFIG_END)
        || (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END))
    {
        int index = getHoldValueNum(now_Opera_Num);
        p = holdValue[index].point;
    }
    else
    {
        switch(now_Opera_Num)
        {
            
            default: break;
        }
    }
    return p;
}
/* 确定单位 */
static uint8_t* dtm_unit(void)
{
    uint8_t* u = NULL;
    if((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARACONFIG_END)
        || (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END))
    {
        int index = getHoldValueNum(now_Opera_Num);
        u = holdValue[index].unit;
    }
    else
    {
        switch(now_Opera_Num)
        {
            
            default: break;
        }
    }
    return u;
}
/* 确定要显示的位数 */
static uint8_t dtm_bits(void)
{
    uint8_t b = 6;
    if((now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARACONFIG_END)
        || (now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END))
    {
        int index = getHoldValueNum(now_Opera_Num);
        b = holdValue[index].bits;
    }
    else
    {
        switch(now_Opera_Num)
        {
            case COM_NUM_PASSWORD_ENTER_PARA:
            case COM_NUM_PASSWORD_ENTER_CMD:
                b = 4;break;
            default: break;
        }
    }
    return b;
}
/* 输入数据 */
static bool inputvalue(uint8_t deci,uint8_t row,uint8_t line,uint8_t points,uint8_t* unit,int *value)
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
    
    if(nowbit >= 0)
    {
        if(nowbit > deci)
            nowbit = deci;
        if(NowKeyPress == USE_KEY_UP)
        {
            sgl_val++;
            if(sgl_val > 9)
                sgl_val %= 10;
        }
        else if(NowKeyPress == USE_KEY_DOWN)
        {
            sgl_val--;
            if(sgl_val < 0)
                sgl_val += 10;
        }
        else if(NowKeyPress == USE_KEY_SURE)
        {
            nowbit--;
            sgl_val = 0;
            if(nowbit < 0)
                if(NowKeyPress == USE_KEY_SURE)
                {
                    *value = bit_0 + bit_1 * 10 + bit_2 * 100 + bit_3 * 1000 + bit_4 * 10000 + bit_5 * 100000 + bit_6 * 1000000;
                    nowbit = 8;
                    sgl_val = 0;
                    bit_0 = bit_1 = bit_2 = bit_3 = bit_4 = bit_5 = bit_6 = 0;
                    return true;
                }
        }
        else if(NowKeyPress == USE_KEY_BACK)
        {
            nowbit = deci - 1;
            sgl_val = 0;
            bit_0 = bit_1 = bit_2 = bit_3 = bit_4 = bit_5 = bit_6 = 0;
        }
        switch(nowbit)
        {
            case 0: bit_0 = sgl_val; break;
            case 1: bit_1 = sgl_val; break;
            case 2: bit_2 = sgl_val; break;
            case 3: bit_3 = sgl_val; break;
            case 4: bit_4 = sgl_val; break;
            case 5: bit_5 = sgl_val; break;
            case 6: bit_6 = sgl_val; break;
            default: break;
        }
    }
    else
    {
        if(NowKeyPress == USE_KEY_SURE)
        {
            *value = bit_0 + bit_1 * 10 + bit_2 * 100 + bit_3 * 1000 + bit_4 * 10000 + bit_5 * 100000+ bit_6 * 1000000;
            nowbit = 8;
            sgl_val = 0;
            bit_0 = bit_1 = bit_2 = bit_3 = bit_4 = bit_5 = bit_6 = 0;
            return true;
        }
    }
    //显示
    switch(deci)
    {
        case 7:
        {
            line = OledDisplayOneNmb(bit_6,row,line,(nowbit & 1) == 0 && (nowbit | 1) == 7);
        }
        case 6: 
        {
            line = OledDisplayOneNmb(bit_5,row,line,(nowbit & 2) == 0 && (nowbit | 2) == 7);
        }
        case 5: 
        {
            line = OledDisplayOneNmb(bit_4,row,line,(nowbit & 3) == 0 && (nowbit | 3) == 7);
        }
        case 4: 
        {
            if(points == 4)
                line = OledDisplayOneNmb(10,row,line,0) - 2;
            line = OledDisplayOneNmb(bit_3,row,line,(nowbit & 4) == 0 && (nowbit | 4) == 7);
        }
        case 3: 
        {
            if(points == 3)
                line = OledDisplayOneNmb(10,row,line,0) - 2;
            line = OledDisplayOneNmb(bit_2,row,line,(nowbit & 5) == 0 && (nowbit | 5) == 7);
        }
        case 2: 
        {
            if(points == 2)
                line = OledDisplayOneNmb(10,row,line,0) - 2;
            line = OledDisplayOneNmb(bit_1,row,line,(nowbit & 6) == 0 && (nowbit | 6) == 7);
        }
        case 1: 
        {
            if(points == 1)
                line = OledDisplayOneNmb(10,row,line,0) - 2;
            line = OledDisplayOneNmb(bit_0,row,line,!(nowbit & 7));
        }
    }
    if(unit != NULL)
        OledDisplayLineWords(unit,line,row,0);
    DisplayLangaugeLineWords((uint8_t*)"确认",OLED_LINE8_8,OLED_ROW4_4,!(nowbit & 7),(uint8_t*)"Ok");
    DisplayLangaugeLineWords((uint8_t*)"清零",OLED_LINE8_1,OLED_ROW4_4,0,(uint8_t*)"Zero out");
    return false;
}
/* 是否下发指令或参数判断页 */
static void ifsendcmd(void)
{
    all_screen(0x00);
    func_index = KEYNUM_IFSENDCMD;
    if(now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END)
    {//无参指令
        DisplayLangaugeLineWords((uint8_t*)"是否下发指令:",OLED_LINE8_1,OLED_ROW4_1,0,(uint8_t*)"Issue instruct:");
        OledDisplayLineWords(dtm_operaname(now_Opera_Num),OLED_LINE8_1,OLED_ROW3_2,0);
    }
    else if(now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)
    {
        DisplayLangaugeLineWords((uint8_t*)"是否下发带参指令:",OLED_LINE8_1,OLED_ROW4_1,0,(uint8_t*)"Issue IWP:");
        OledDisplayLineWords(dtm_operaname(now_Opera_Num),OLED_LINE8_1,OLED_ROW4_2,0);
        OledValueDisplay(now_Para_CT.val,OLED_LINE8_3,OLED_ROW4_3,0,now_Para_CT.points,now_Para_CT.unit);
    }
    else if(now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARACONFIG_END)
    {//CPU2参数
        DisplayLangaugeLineWords((uint8_t*)"是否下发参数:",OLED_LINE8_1,OLED_ROW4_1,0,(uint8_t*)"Issue Para:");
        OledDisplayLineWords(dtm_operaname(now_Opera_Num),OLED_LINE8_1,OLED_ROW4_2,0);
        if(now_Para_CT.bits <= 3)
            OledValueDisplay(now_Para_CT.val,OLED_LINE8_4,OLED_ROW4_3,0,now_Para_CT.points,now_Para_CT.unit);
        else
            OledValueDisplay(now_Para_CT.val,OLED_LINE8_3,OLED_ROW4_3,0,now_Para_CT.points,now_Para_CT.unit);
    }
    else if(now_Opera_Num > COM_NUM_PASSWORD_START && now_Opera_Num < COM_NUM_PASSWORD_END)
    {
        if(now_Para_CT.val == screen_parameter.passward || now_Para_CT.val == FIXPASSWORD)
        {//密码正确
            DisplayLangaugeLineWords((uint8_t*)"密码正确!",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Password Correct");
            HAL_Delay(100);
            if(now_Opera_Num == COM_NUM_PASSWORD_ENTER_PARA)//进入参数配置
                menu_paraconfig();
            else if(now_Opera_Num == COM_NUM_PASSWORD_ENTER_CMD)//进入调试指令
                menu_cmdconfig_main();
        }
        else
        {//密码错误
            DisplayLangaugeLineWords((uint8_t*)"密码错误!",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Password Error");
            HAL_Delay(500);
            mainmenu();
        }
        return;
    }
    if(NowKeyPress == USE_KEY_SURE)
    {
        if(timesure > 1)
        {
            timesure = 0;
            timeback = 0;
            dtm_suretofunc()();
            return;
        }
        timesure++;
        if(timeback > 0)
            timeback = 0;
    }
    else if(NowKeyPress == USE_KEY_BACK)
    {
        if(timeback != 0)
        {
            timeback = 0;
            timesure = 1;
            dtm_backtofunc()();
            return;
        }
        timeback++;
        timesure--;
    }
    DisplayLangaugeLineWords((uint8_t*)"返回",OLED_LINE8_1,OLED_ROW4_4,timeback,(uint8_t*)"Back");
    DisplayLangaugeLineWords((uint8_t*)"确认",OLED_LINE8_8,OLED_ROW4_4,timesure,(uint8_t*)"Ok");
}
/* 返回按返回键后要跳转的函数指针 */
static pFunc_void dtm_backtofunc(void)
{
    pFunc_void p = errorprocess;

    if(now_Opera_Num > COM_NUM_NOPARACMD_NORMAL_START && now_Opera_Num < COM_NUM_NOPARACMD_NORMAL_STOP)
        p = measuremenu;
    else if(now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END)
        p = menu_paraconfig;
    else if(now_Opera_Num == COM_NUM_SET_WORKPATTERN)
        p = menu_paraconfig;
    else if(now_Opera_Num > COM_NUM_DEBUGCMD_START && now_Opera_Num < COM_NUM_DEBUGCMD_STOP)
        p = menu_cmdconfig_main;
    else if((now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_ONEPARACMD_END)
        || (now_Opera_Num > COM_NUM_ONEPARA_DEBUGCMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END))
        p = menu_cmdconfig_main;
    else if(now_Opera_Num == COM_NUM_MAGNETIC_D || now_Opera_Num == COM_NUM_MAGNETIC_T)
        p = menu_magnetic;
    else if(now_Opera_Num > COM_NUM_SCREEN_STR && now_Opera_Num < COM_NUM_SCREEN_STP)
        p = menu_screen;
    else if(now_Opera_Num > COM_NUM_SOURCE_START && now_Opera_Num < COM_NUM_SOURCE_STOP)
        p = menu_scr_source;
    else if(now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARA_DEBUG_END)
        p = menu_paraconfig;
    else if(now_Opera_Num == COM_NUM_PARA_LANG)
        p = mainmenu;
    
    return p;
}
/* 返回按确认键后要跳转的函数指针 */
static pFunc_void dtm_suretofunc(void)
{
    /*不带参线圈指令*/
    if(now_Opera_Num > COM_NUM_NOPARACMD_START && now_Opera_Num < COM_NUM_NOPARACMD_END)
        return cmd_nopara_process;
    /*带一参线圈指令*/
    else if(now_Opera_Num > COM_NUM_ONEPARACMD_START && now_Opera_Num < COM_NUM_NOPARA_DEBUGCMD_END)
        return cmd_onepara_process;
    /* CPU2参数类 */
    else if(now_Opera_Num > COM_NUM_PARA_DEBUG_START && now_Opera_Num < COM_NUM_PARACONFIG_END)
        return parascopecheck;
    else
    {
        printf("非法指令\r\n");
        return errorprocess;
    }
}
/* 不带参线圈指令处理过程 */
static void cmd_nopara_process(void)
{
static const uint32_t nopara_cmd_map[][2] = {
    { COM_NUM_BACK_ZERO,           CMD_BACK_ZERO },           // 返回零点
    { COM_NUM_FIND_ZERO,           CMD_CALIBRATE_ZERO },        // 标定零点
    { COM_NUM_SPREADPOINTS_AI,     CMD_MEASURE_DISTRIBUTED },// 自动分布测量
    { COM_NUM_FIND_OIL,            CMD_FIND_OIL },        // 寻找液位
    { COM_NUM_FIND_WATER,          CMD_FIND_WATER },      // 寻找水位
    { COM_NUM_FIND_BOTTOM,         CMD_FIND_BOTTOM },           // 寻找罐底
    { COM_NUM_SYNTHETIC,           CMD_SYNTHETIC },     // 综合测量
    { COM_NUM_METER_DENSITY,       CMD_MEASURE_DENSITY_METER }, // 每米测量
    { COM_NUM_INTERVAL_DENSITY,    CMD_MEASURE_DENSITY_RANGE }, // 区间测量
    { COM_NUM_READPARAMETER,       CMD_READ_PARAM },       // 读取参数
    { COM_NUM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },    // 恢复出厂设置
};

    int mapamount = sizeof(nopara_cmd_map) / sizeof(nopara_cmd_map[0]);
    int i;
    //发送指令

    for(i = 0;i < mapamount;i++)
    {
        if(now_Opera_Num == nopara_cmd_map[i][0])
        {
           CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,HOLDREGISTER_DEVICEPARAM_COMMAND,2,&nopara_cmd_map[i][1]);
            flag_SendCommand = true;
            break;
        }
    }
    if(now_Opera_Num == COM_NUM_SET_WORKPATTERN)//进入调试模式后返回的地方
    {
        //读状态，看是否成功进入调试模式
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在进入调试模式",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Entering debug mode");
        CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,REG_DEVICE_STATUS_WORK_MODE,1,NULL);
        HAL_Delay(150);
        oled_clear();
        if(g_measurement.device_status.work_mode == 1)//调试模式重新处理
        {
            DisplayLangaugeLineWords((uint8_t*)"成功!",OLED_LINE8_4,OLED_ROW3_2,0,(uint8_t*)"succeed!");
            HAL_Delay(500);
			if(debugmode_back == 0)
			{
				menu_paraconfig();
			}
			else
			{
				menu_cmdconfig_main();
			}
        }
        else
        {
            DisplayLangaugeLineWords((uint8_t*)"失败!",OLED_LINE8_1,OLED_ROW4_2,0,(uint8_t*)"fail!");
            DisplayLangaugeLineWords((uint8_t*)"请重新尝试!",OLED_LINE8_1,OLED_ROW4_3,0,(uint8_t*)"Please try again!");
            HAL_Delay(800);
			if(debugmode_back == 0)
			{
				menu_paraconfig();
			}
			else
			{
				menu_cmdconfig_main();
			}
        }
    }
    else if(now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING || now_Opera_Num == COM_NUM_RESTORY_FILE)
    {//恢复出厂设置、恢复备份文件
        oled_clear();
        if((g_measurement.device_status.device_state != STATE_STANDBY) 
            && (g_measurement.device_status.device_state != STATE_ERROR))
        {
            DisplayLangaugeLineWords((uint8_t*)"失败",OLED_LINE8_1,OLED_ROW4_2,0,(uint8_t*)"Failed to set");
            DisplayLangaugeLineWords((uint8_t*)"请先进入调试模式",OLED_LINE8_1,OLED_ROW4_3,0,(uint8_t*)"Enter debug mode");
        }
        else
        {
            if(now_Opera_Num == COM_NUM_RESTOR_EFACTORYSETTING)
                DisplayLangaugeLineWords((uint8_t*)"正在恢复出厂设置",OLED_LINE8_1,OLED_ROW4_2,0,(uint8_t*)"Factory Settings");
            else
                DisplayLangaugeLineWords((uint8_t*)"正在恢复备份文件",OLED_LINE8_1,OLED_ROW4_2,0,(uint8_t*)"Backup files");
//            HAL_Delay(800);
//            menu_reset();
        }
    }
    else
        exitTankOpera();//退出罐上操作
}

/* 非法操作处理 */
static void errorprocess(void)
{
    all_screen(0x00);
    DisplayLangaugeLineWords((uint8_t*)"非法操作！",OLED_LINE8_1,OLED_ROW4_2,0,(uint8_t*)"Illegal operation");
    DisplayLangaugeLineWords((uint8_t*)"1s后退出屏幕操作",OLED_LINE8_1,OLED_ROW4_3,0,(uint8_t*)"Exit after 1 second");
    HAL_Delay(1000);
    exitTankOpera();
}
/* 显示参数内容 */
static void displaypara(void)
{
    int bits,line;
    int index;
    all_screen(0x00);
    func_index = KEYNUM_DISPLAY_PARA;
    OledDisplayLineWords(dtm_operaname(now_Opera_Num),OLED_LINE8_1,OLED_ROW4_1,0); /* 确定参数名称 */
    //显示数据
    //判断是不是本机参数
    if(now_Opera_Num == COM_NUM_PARA_LOCAL_LEDVERSION)
        OledValueDisplay(CPU3VERSION,OLED_LINE8_4,OLED_ROW3_2,0,0,NULL);
    else
    {
        index = getHoldValueNum(now_Opera_Num);
        if(index == -1)
            DisplayLangaugeLineWords((uint8_t*)"非法操作",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Illegal operation");
        else
        {
            if(holdValue[index].pword == NULL)
            {//显示数据
                bits = dtm_bits(); /* 确定显示几位 */
                if(bits <= 3 && index != -1 && holdValue[index].unit == NULL) 
                    line = OLED_LINE8_5;
                else 
                    line = OLED_LINE8_4;
                OledValueDisplay(holdValue[index].val,line,OLED_ROW3_2,0,holdValue[index].point,holdValue[index].unit);
            }
            else
            {//显示数字代表的隐藏文字含义
                if(dtm_namelength(holdValue[index].pword()) >= 12)
                    line = OLED_LINE8_2;
                else if(dtm_namelength(holdValue[index].pword()) > 6)
                    line = OLED_LINE8_3;
                else
                    line = OLED_LINE8_4;
                OledDisplayLineWords(holdValue[index].pword(),line,OLED_ROW3_2,0);
            }
        }
    }
    if(NowKeyPress == USE_KEY_SURE)
    {
        if(timesure > 1)
        {
            timesure = 0;
            timeback = 0;
            parawritecheck();
            return;
        }
        timesure++;
        if(timeback > 0)
            timeback = 0;
    }
    else if(NowKeyPress == USE_KEY_BACK)
    {
        if(timeback != 0)
        {
            timeback = 0;
            timesure = 1;
            dtm_backtofunc()();
            return;
        }
        timeback++;
        timesure--;
    }
    DisplayLangaugeLineWords((uint8_t*)"返回",OLED_LINE8_1,OLED_ROW4_4,timeback,(uint8_t*)"Back");
    DisplayLangaugeLineWords((uint8_t*)"修改",OLED_LINE8_7,OLED_ROW4_4,timesure,(uint8_t*)"Alter");
}
/* 写权限范围检查 */
static void parawritecheck(void)
{
    int index;
    index = getHoldValueNum(now_Opera_Num);
    if(index != -1 && holdValue[index].authority_write 
        && (g_measurement.device_status.device_state == STATE_STANDBY
            || g_measurement.device_status.device_state == STATE_ERROR))
    {
        if(holdValue[index].pword == NULL)//数字输入类
            inputcmdpara();
        else//文字选择类
            selectparaword();
    }
    else
    {
        all_screen(0x00);
        DisplayLangaugeLineWords((uint8_t*)"无修改权限！",OLED_LINE8_2,OLED_ROW3_2,0,(uint8_t*)"No permission");
        HAL_Delay(800);
        displaypara();
    }
}
/* 是否进入罐上操作 */
static void ifentermainmenu(void)
{
    all_screen(0x00);
    func_index = KEYNUM_IF_ENTER_MAINMENU;
    DisplayLangaugeLineWords((uint8_t*)"是否进入罐上操作?",OLED_LINE8_1,OLED_ROW4_1,0,(uint8_t*)"Enter operation?");
    DisplayLangaugeLineWords((uint8_t*)"返回",OLED_LINE8_1,OLED_ROW4_4,0,(uint8_t*)"Back");
    DisplayLangaugeLineWords((uint8_t*)"确认",OLED_LINE8_8,OLED_ROW4_4,0,(uint8_t*)"Ok");
}
/* 是否退出罐上操作 */
static void ifexittankopera(void)
{
    all_screen(0x00);
    func_index = KEYNUM_IF_EXIT_MAINMENU;
    DisplayLangaugeLineWords((uint8_t*)"是否退出罐上操作?",OLED_LINE8_1,OLED_ROW4_1,0,(uint8_t*)"Exit operation?");
    DisplayLangaugeLineWords((uint8_t*)"返回",OLED_LINE8_1,OLED_ROW4_4,0,(uint8_t*)"Back");
    DisplayLangaugeLineWords((uint8_t*)"确认",OLED_LINE8_8,OLED_ROW4_4,0,(uint8_t*)"Ok");
}
/* 进入参数配置前的密码输入操作页 */
static void password_enter_para(void)
{
    now_Opera_Num = COM_NUM_PASSWORD_ENTER_PARA;
    inputcmdpara();
}
/* 主菜单 */
static void mainmenu(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"测量命令",COM_NUM_NOOPERA,measuremenu,COMMANE_NORW,(uint8_t*)"MeasureCommend"},
        {(uint8_t*)"参数配置",COM_NUM_NOOPERA,password_enter_para,COMMANE_NORW,(uint8_t*)"Para-config"},
        {(uint8_t*)"调试指令",COM_NUM_NOOPERA,password_enter_cmd,COMMANE_NORW,(uint8_t*)"Debug Commend"},
        {(uint8_t*)"语言",COM_NUM_PARA_LANG,setlanguage,COMMANE_NORW,(uint8_t*)"Language"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,ifexittankopera,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    flag_SendCommand = false;
    all_screen(0x00);
    func_index = KEYNUM_MAINMENU;
    menuselect(menu,menulen);
}
/* 普通测量指令菜单 */
static void measuremenu(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"回零点",COM_NUM_BACK_ZERO,ifsendcmd,COMMANE_NORW,(uint8_t*)"Return to Zero"},
        {(uint8_t*)"液位跟随",COM_NUM_FIND_OIL,ifsendcmd,COMMANE_NORW,(uint8_t*)"Level Follow"},
        {(uint8_t*)"水位测量",COM_NUM_FIND_WATER,ifsendcmd,COMMANE_NORW,(uint8_t*)"Water level-M"},
        {(uint8_t*)"罐底测量",COM_NUM_FIND_BOTTOM,ifsendcmd,COMMANE_NORW,(uint8_t*)"Tank Bottom-M"},
        {(uint8_t*)"密度分布测量",COM_NUM_SPREADPOINTS,ifsendcmd,COMMANE_NORW,(uint8_t*)"DT-M"},
        {(uint8_t*)"固定点测量",COM_NUM_SINGLE_POINT,inputcmdpara,COMMANE_NORW,(uint8_t*)"DT-SinglePoint-M"},
        {(uint8_t*)"固定点监测",COM_NUM_SP_TEST,ifsendcmd,COMMANE_NORW,(uint8_t*)"DT-National-M"},
		{(uint8_t*)"测量参数",COM_NUM_NOOPERA,menu_basepara,COMMANE_NORW,(uint8_t*)"M-Parameters"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,mainmenu,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_MEASURE_MAINMENU;
    menuselect(menu,menulen);
}
/* 菜单 - 调试指令 */
static void menu_cmdconfig_main(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"进入调试模式",COM_NUM_SET_WORKPATTERN,ifsendcmd,COMMANE_NORW,(uint8_t*)"Go Into Debug"},
        {(uint8_t*)"上行",COM_NUM_RUNUP,inputcmdpara,COMMANE_NORW,(uint8_t*)"Move Up"},
        {(uint8_t*)"下行",COM_NUM_RUNDOWN,inputcmdpara,COMMANE_NORW,(uint8_t*)"Move Down"},
        {(uint8_t*)"置零点",COM_NUM_FORCE_ZERO,ifsendcmd,COMMANE_NORW,(uint8_t*)"MeasLevel(NoFit)"},
		{(uint8_t*)"液位修正",COM_NUM_CORRECTION_OIL,inputcmdpara,COMMANE_NORW,(uint8_t*)"LevelCorrect"},
        {(uint8_t*)"标定液位",COM_NUM_CAL_OIL,inputcmdpara,COMMANE_NORW,(uint8_t*)"LevelCalibrat"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,mainmenu,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_MENU_CMD_MAIN;
	debugmode_back = 1;
    menuselect(menu,menulen);
}
/* 参数配置菜单 */
static void menu_paraconfig(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"进入调试模式",COM_NUM_SET_WORKPATTERN,ifsendcmd,COMMANE_NORW,(uint8_t*)"Go Into Debug"},
        {(uint8_t*)"基础参数",       COM_NUM_NOOPERA, menu_tankbasicpara,  COMMANE_NORW, (uint8_t*)"BasicPara"},
        {(uint8_t*)"综合测量参数",   COM_NUM_NOOPERA, menu_syntheticpara,  COMMANE_NORW, (uint8_t*)"SynthPara"},
        {(uint8_t*)"分布测量参数",   COM_NUM_NOOPERA, menu_spreadpara,     COMMANE_NORW, (uint8_t*)"SpreadPara"},
        {(uint8_t*)"磁通量",       COM_NUM_NOOPERA, menu_correctionpara, COMMANE_NORW, (uint8_t*)"MAGNETIC"},
        {(uint8_t*)"实高测量参数",   COM_NUM_NOOPERA, menu_realhighpara,   COMMANE_NORW, (uint8_t*)"RealHighPara"},
        {(uint8_t*)"调试信息",       COM_NUM_NOOPERA, menu_debuginfo,      COMMANE_NORW, (uint8_t*)"DebugInfo"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,mainmenu,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_MAIN;
	debugmode_back = 0;
    menuselect(menu,menulen);
}
/* 菜单 - 基础参数 */
static void menu_basepara(void)
{
    static struct MenuData menu[] = {
	{(uint8_t*)"分布测模式",	COM_NUM_SPREAD_STATE_FREE,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"DistMeasMode"},
	{(uint8_t*)"分布测测量点数",	COM_NUM_SPREAD_NUM_FREE,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"DistMeasPts"},
	{(uint8_t*)"分布测量密度点间距",	COM_NUM_SPREAD_DISTANCE_FREE,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"DistDensPtDist"},
	{(uint8_t*)"顶密距液位",	COM_NUM_SPREAD_TOPLIMIT_FREE,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"TopDensToLvl"},
	{(uint8_t*)"底密距罐底",	COM_NUM_SPREAD_FLOORLIMIT_FREE,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"BotDensToBot"},
	{(uint8_t*)"综合测单点位置",	COM_NUM_SPSYNTHETIC_POSITION,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"IntgSinglePos"},
	{(uint8_t*)"每米测方向",	COM_NUM_MEASREMENT_METER,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"MeterMeasDir"},
	{(uint8_t*)"区间密度测量点数",	COM_NUM_INTERVAL_POINT,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"SectDensPts"},
	{(uint8_t*)"区间密度测量方向",	COM_NUM_INTERVAL_DIREDION,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"SectDensDir"},
	{(uint8_t*)"区间测量液位A",	COM_NUM_INTERVAL_OIL_A,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"SectMeasLvlA"},
	{(uint8_t*)"区间测量液位B",	COM_NUM_INTERVAL_OIL_B,	para_mainprocess,	COMMAND_READ,	(uint8_t*)"SectMeasLvlB"},
	{(uint8_t*)"退出",COM_NUM_NOOPERA,menu_paraconfig,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_BASE;
    menuselect(menu,menulen);
}
/* 发送指令获取对应参数的数据 */
static int get_para_data(void)
{
    int index;
    //判断是本机参数还是CPU2存储的参数
    if(now_Opera_Num > COM_NUM_PARA_LOCAL_START && now_Opera_Num < COM_NUM_PARA_LOCAL_STOP)
    {//本机参数
        return 0;
    }
    else
    {//CPU2存储的参数，需要通讯
        if(cnt_commutoCPU2 >= COMMU_ERROR_MAX)
        {
            oled_clear();
            DisplayLangaugeLineWords((uint8_t*)"与CPU2通讯故障!",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Cpu2 CF!");
            HAL_Delay(800);
            return -1;
        }
        oled_clear();
        DisplayLangaugeLineWords((uint8_t*)"正在读取参数",OLED_LINE8_1,OLED_ROW3_2,0,(uint8_t*)"Reading Para");
        index = getHoldValueNum(now_Opera_Num);
        CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,holdValue[index].startadd,holdValue[index].rgstcnt,NULL);
        HAL_Delay(150);
        return 0;
    }
}
/* 参数范围检查 */
static void parascopecheck(void)
{
    int index;
    index = getHoldValueNum(now_Opera_Num);
    if(index == -1)
    {
        all_screen(0x00);
        DisplayLangaugeLineWords((uint8_t*)"非法参数！",OLED_LINE8_2,OLED_ROW3_2,0,(uint8_t*)"Invalid Para");
        HAL_Delay(800);
        displaypara();
    }
    else if(holdValue[index].flag_checkvalue)
    {
        if(now_Para_CT.val < holdValue[index].valuemin || now_Para_CT.val > holdValue[index].valuemax)
        {
            all_screen(0x00);
            DisplayLangaugeLineWords((uint8_t*)"数值超范围！",OLED_LINE8_2,OLED_ROW3_2,0,(uint8_t*)"Value out of Range");
            HAL_Delay(800);
            displaypara();
        }
        else
            cmd_configpara_process();
    }
    else
        cmd_configpara_process();
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
    DisplayLangaugeLineWords((uint8_t*)"正在修改参数",OLED_LINE8_2,OLED_ROW3_2,0,(uint8_t*)"Modify Para");
    //下发指令
    if(holdValue[index].data_type == TYPE_FLOAT)
    {
        union utof tmp_f;
        tmp_f.f = now_Para_CT.val * pow(0.1,(double)holdValue[index].point);
        for(i = 0;i < 4;i++)
            paraarr[i] = tmp_f.u >> (8 * (3 - i));
    }
    else if(holdValue[index].data_type == TYPE_DOUBLE)
    {
        union utod tmp_d;
        tmp_d.d = now_Para_CT.val;
        tmp_d.d = now_Para_CT.val * pow(0.1,(double)holdValue[index].point);
        for(i = 0;i < 4;i++)
            paraarr[i] = tmp_d.u[1] >> (8 * (3 - i));
        for(i = 4;i < 8;i++)
            paraarr[i] = tmp_d.u[0] >> (8 * (7 - i));
    }
    else
    {
        now_Para_CT.val -= holdValue[index].offset;
        for(i = 0;i < holdValue[index].rgstcnt * 2 && i < arrlen;i++)
            paraarr[i] = now_Para_CT.val >> (8 * ((holdValue[index].rgstcnt * 2 - 1) - i));
    }
    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,holdValue[index].startadd,holdValue[index].rgstcnt,paraarr);
    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,holdValue[index].startadd,holdValue[index].rgstcnt,NULL);
    HAL_Delay(800);
    //返回参数显示页
//    if(now_Opera_Num == COM_NUM_CORRECTDENSITY_POSI)
//    {
//        now_Opera_Num = COM_NUM_CORRECTDENSITY_DENS;
//        inputcmdpara();
//    }
//    else
        displaypara();
}
/* 菜单选项栏 */
static void menuselect(struct MenuData* menu,int menulen)
{
    const static int RowsPerPage = 4;//菜单栏每页显示的行数
    static int shift = 0;//阴阳码
    int i = 0;
    int row = 0;
    int line = 0;
    int now_menu_num = 0;

    if(NowKeyPress == USE_KEY_UP)
    {
        PageNum[func_index].menu_cnt--;
        if(PageNum[func_index].menu_cnt <= 0)
            PageNum[func_index].menu_cnt += menulen;
    }
    else if(NowKeyPress == USE_KEY_DOWN)
        PageNum[func_index].menu_cnt++;
    else if(NowKeyPress == USE_KEY_SURE)
    {
        if(timesure != 0)
        {
            timesure = 0;
            now_menu_num = PageNum[func_index].menu_num;
            now_Opera_Num = menu[now_menu_num].operaNum;
            //跳转
            menu[now_menu_num].sureopera();
            return;
        }
        timesure++;
    }
    else if(NowKeyPress == USE_KEY_BACK)
    {
        if(timeback != 0)
        {
            timeback = 0;
            timesure = 1;
            menu[menulen - 1].sureopera();
            return;
        }
        timeback++;
    }
    PageNum[func_index].menu_num = (PageNum[func_index].menu_cnt - 1) % menulen;
    PageNum[func_index].menu_page = (PageNum[func_index].menu_num / RowsPerPage) * RowsPerPage;
    for(i = PageNum[func_index].menu_page;(i < (PageNum[func_index].menu_page + RowsPerPage)) && (i < menulen);i++)
    {
        if(i == PageNum[func_index].menu_num)
            shift = 1;
        else
            shift = 0;
        DisplayLangaugeLineWords(menu[i].operaName,line,row,shift,menu[i].operaName2);
        row += OLED_ROW4_2;
    }
}
/* 参数类处理 */
static void para_mainprocess(void)
{
    if(get_para_data() == 0)
            displaypara();
        else
            mainmenu();
}

/* 进入调试指令前的密码输入操作页 */
static void password_enter_cmd(void)
{
    now_Opera_Num = COM_NUM_PASSWORD_ENTER_CMD;
    inputcmdpara();
}

/* 带一参线圈指令处理过程 */
static void cmd_onepara_process(void)
{
    int index;
    int i;
    int arrlen = 8; //发送数组长度
    static uint8_t paraarr[8];
    static const uint16_t onepara_cmd_map[][2] = {/* 操作码到指令地址映射*/
    { COM_NUM_SINGLE_POINT,        COIL_MEASURE_SINGLE_POINT },  // 单点测量
    { COM_NUM_SP_TEST,             COIL_MONITOR_SINGLE_POINT },  // 单点监测
    { COM_NUM_SPREADPOINTS,        COIL_MEASURE_DISTRIBUTED },   // 分布测量

    { COM_NUM_CAL_OIL,             COIL_CALIBRATE_OIL_LEVEL },   // 液位标定
    { COM_NUM_RUNUP,               COIL_MOVE_UP },               // 向上运行
    { COM_NUM_RUNDOWN,             COIL_MOVE_DOWN },             // 向下运行
    { COM_NUM_SET_ZEROCIRCLE,      COIL_SET_ZERO_CIRCLE },       // 设置零点圈数
    { COM_NUM_SET_ZEROANGLE,       COIL_SET_ZERO_ANGLE },        // 设置零点角度
    { COM_NUM_CORRECTION_OIL,      COIL_CORRECT_OIL_LEVEL },     // 修正液位

    };
    int mapamount = sizeof(onepara_cmd_map) / sizeof(onepara_cmd_map[0]);
    
    index = getHoldValueNum(now_Opera_Num);
    if(index == -1)
    {
        all_screen(0x00);
        DisplayLangaugeLineWords((uint8_t*)"非法参数！",OLED_LINE8_2,OLED_ROW3_2,0,(uint8_t*)"Invalid Para");
        HAL_Delay(800);
    }
    else
    {
        //发送参数
        all_screen(0x00);
        DisplayLangaugeLineWords((uint8_t*)"正在下发参数",OLED_LINE8_2,OLED_ROW3_2,0,(uint8_t*)"Send Para");
        //下发指令
        for(i = 0;i < holdValue[index].rgstcnt * 2 && i < arrlen;i++)
            paraarr[i] = now_Para_CT.val >> (8 * ((holdValue[index].rgstcnt * 2 - 1) - i));
        CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,holdValue[index].startadd,holdValue[index].rgstcnt,paraarr);
        //发送指令
        DisplayLangaugeLineWords((uint8_t*)"正在下发指令",OLED_LINE8_2,OLED_ROW3_3,0,(uint8_t*)"Send Command");
        for(i = 0;i < mapamount;i++)
        {
            if(now_Opera_Num == onepara_cmd_map[i][0])
            {
//                CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_COIL,onepara_cmd_map[i][1],0xFF00,NULL);
                flag_SendCommand = true;
                break;
            }
        }
    }
    //退出罐上操作
    exitTankOpera();
}
/* 菜单 - 磁通量参数 */
static void menu_magnetic(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"磁通量D",COM_NUM_MAGNETIC_D,para_mainprocess,COMMAND_READ,(uint8_t*)"Flux D"},
        {(uint8_t*)"磁通量T",COM_NUM_MAGNETIC_T,para_mainprocess,COMMAND_READ,(uint8_t*)"Flux T"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,menu_basepara,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_MAGNETIC;
    menuselect(menu,menulen);
}
/* 正负号输入 */
static int SignInput(uint8_t row,uint8_t line,uint8_t shift)
{
    static int sgn = 1;
    static int tosure = -1;
    int ret = 0;
    //决定显示正还是负
    
    if(NowKeyPress == USE_KEY_SURE)
    {
        tosure++;
    }
    else if(NowKeyPress == USE_KEY_UP)
    {
        sgn *= -1;
        tosure = 0;
    }
    else if(NowKeyPress == USE_KEY_DOWN)
    {
        sgn *= -1;
        tosure = 0;
    }
    else
    {
        sgn = 1;
        tosure = 0;
    }

    if(sgn == -1)
        line = OledDisplayLineWords((u8*)"-",line,row,shift);
    else
        line = OledDisplayLineWords((u8*)"+",line,row,shift);

    OledValueDisplay(now_Para_CT.val,line,row,0,now_Para_CT.points,now_Para_CT.unit);
    //是否确定
    if(tosure > 1)
    {
        ret = sgn;
    }
    DisplayLangaugeLineWords((uint8_t*)"返回",OLED_LINE8_1,OLED_ROW4_4,0,(uint8_t*)"Back");
    DisplayLangaugeLineWords((uint8_t*)"确认",OLED_LINE8_8,OLED_ROW4_4,tosure > 0,(uint8_t*)"Ok");
    return ret;
}



/* 返回通讯方式文字信息 */
uint8_t* ret_arr_word(void)
{
    int index,len;
    uint8_t* (*p)[2]; 
    
    p = dtm_disarr(&index,&len);
    if(index < len && p != NULL)
        return p[index][screen_parameter.language];
    else
        return (uint8_t*)"非法配置";
}
/* 决定显示哪个数组里的文字 */
uint8_t *(*dtm_disarr(int* pindex, int* plen))[2]
{
    int index = 0,len = -1;
    uint8_t* (*p)[2]; 
    
    index = getHoldValueNum(now_Opera_Num);
    switch(now_Opera_Num)
    {
        case COM_NUM_IF_FINDOIL_POWERON:
        {
            index = holdValue[index].val;
            len = sizeof(arr_powerontodo) / sizeof(arr_powerontodo[0]);
            p = arr_powerontodo;
            break;
        }
        case COM_NUM_SRREAD_MEASURETURN:
        case COM_NUM_INTERVAL_DIREDION://分布测量方向
        {
            index = holdValue[index].val;
            len = sizeof(arr_densitydir) / sizeof(arr_densitydir[0]);
            p = arr_densitydir;
            break;
        }
        case COM_NUM_SCREEN_SOURCE_OIL://数据源
        case COM_NUM_SCREEN_SOURCE_WATER:
        case COM_NUM_SCREEN_SOURCE_D:
        case COM_NUM_SCREEN_SOURCE_T:
        {
            index = holdValue[index].val;
            len = sizeof(arr_source) / sizeof(arr_source[0]);
            p = arr_source;
            break;
        }
        case COM_NUM_SYNTHETIC_BOTTOM:
        case COM_NUM_SYNTHETIC_WATER:
        case COM_NUM_SYNTHETIC_SINGLEPOINT:
        case COM_NUM_IF_REFRESH_TANKHIGH:
        case COM_NUM_SCREEN_OFF://是否
        {
            index = holdValue[index].val;
            len = sizeof(arr_IF) / sizeof(arr_IF[0]);
            p = arr_IF;
            break;
        }
        case COM_NUM_SPREAD_STATE://分布测量模式
        {
            index = holdValue[index].val;
            len = sizeof(arr_densitymode) / sizeof(arr_densitymode[0]);
            p = arr_densitymode;
            break;
        }
        case COM_NUM_SCREEN_INPUT_D_SWITCH://开启关闭
        {
            index = holdValue[index].val;
            len = sizeof(arr_Switch) / sizeof(arr_Switch[0]);
            p = arr_Switch;
            break;
        }
        case COM_NUM_PARA_LANG:
        {
            index = holdValue[index].val;
            len = sizeof(arr_language) / sizeof(arr_language[0]);
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
    uint8_t* (*parr)[2]; 
    int len,index;
    
    oled_clear();
    func_index = KEYNUM_WORDSELECT;
    parr = dtm_disarr(&index,&len);
    operationselect(parr,len - 1);
}
/* 隐藏信息含义选择栏 */
static void operationselect(uint8_t* (*menu)[2],int menulen)
{
    const static int RowsPerPage = 4;//菜单栏每页显示的行数
    static int shift = 0;//阴阳码
    int i = 0;
    int row = 0;
    int line = 0;
    static int menu_cnt_hide = 1;
    static int timesure_hide = 0;
    static int timeback_hide = 0;
    static int menu_page_hide = 0;
    static int menu_num_hide = -1;

    if(NowKeyPress == USE_KEY_UP)
    {
        menu_cnt_hide--;
        if(menu_cnt_hide <= 0)
            menu_cnt_hide += menulen;
    }
    else if(NowKeyPress == USE_KEY_DOWN)
        menu_cnt_hide++;
    else if(NowKeyPress == USE_KEY_SURE)
    {
        if(timesure_hide != 0)
        {
            timesure_hide = 0;
            now_Para_CT.val = menu_num_hide;
            parascopecheck();
            return;
        }
        timesure_hide++;
        if(timeback_hide != 0)
            timeback_hide = 0;
    }
    else if(NowKeyPress == USE_KEY_BACK)
    {
        if(timeback_hide != 0)
        {
            menu_num_hide = -1;
            menu_cnt_hide = 1;
            menu_page_hide = 0;
            timeback_hide = 0;
            timesure_hide = 0;
            displaypara();
            return;
        }
        timeback_hide++;
        if(timesure_hide != 0)
            timesure_hide = 0;
    }
    menu_num_hide = (menu_cnt_hide - 1) % menulen;
    menu_page_hide = (menu_num_hide / RowsPerPage) * RowsPerPage;
    for(i = menu_page_hide;(i < (menu_page_hide + RowsPerPage)) && (i < menulen);i++)
    {
        if(i == menu_num_hide)
            shift = 1;
        else
            shift = 0;
        OledDisplayLineWords(menu[i][screen_parameter.language],line,row,shift);
        row += OLED_ROW4_2;
    }
}


/* 菜单 - 界面显示类 */
static void menu_screen(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"小数点位数",COM_NUM_SCREEN_DECIMAL,para_mainprocess,COMMAND_READ,(uint8_t*)"DecimalPlaces"},
        {(uint8_t*)"屏幕密码",COM_NUM_SCREEN_PASSWARD,para_mainprocess,COMMAND_READ,(uint8_t*)"Password"},
        {(uint8_t*)"是否息屏",COM_NUM_SCREEN_OFF,para_mainprocess,COMMAND_READ,(uint8_t*)"Screen Off"},
        {(uint8_t*)"数据源",COM_NUM_NOOPERA,menu_scr_source,COMMAND_READ,(uint8_t*)"Data Source"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,menu_paraconfig,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_SCREEN;
    menuselect(menu,menulen);
}
/* 菜单 - 数据源 */
static void menu_scr_source(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"液位数据源",COM_NUM_SCREEN_SOURCE_OIL,para_mainprocess,COMMAND_READ,(uint8_t*)"Level Source"},
        {(uint8_t*)"液位手工输入值",COM_NUM_SCREEN_INPUT_OIL,para_mainprocess,COMMAND_READ,(uint8_t*)"LevelInputVal"},
        {(uint8_t*)"水位数据源",COM_NUM_SCREEN_SOURCE_WATER,para_mainprocess,COMMAND_READ,(uint8_t*)"Water Source"},
        {(uint8_t*)"水位手工输入值",COM_NUM_SCREEN_INPUT_WATER,para_mainprocess,COMMAND_READ,(uint8_t*)"WaterInputVal"},
        {(uint8_t*)"密度数据源",COM_NUM_SCREEN_SOURCE_D,para_mainprocess,COMMAND_READ,(uint8_t*)"Densi Source"},
        {(uint8_t*)"密度手工输入值",COM_NUM_SCREEN_INPUT_D,para_mainprocess,COMMAND_READ,(uint8_t*)"DensiInputVal"},
        {(uint8_t*)"密度手输值上传开关",COM_NUM_SCREEN_INPUT_D_SWITCH,para_mainprocess,COMMAND_READ,(uint8_t*)"DensiSwitch"},
        {(uint8_t*)"温度数据源",COM_NUM_SCREEN_SOURCE_T,para_mainprocess,COMMAND_READ,(uint8_t*)"Temp Source"},
        {(uint8_t*)"温度手工输入值",COM_NUM_SCREEN_INPUT_T,para_mainprocess,COMMAND_READ,(uint8_t*)"TempInputVal"},
        {(uint8_t*)"退出",COM_NUM_NOOPERA,menu_screen,COMMANE_NORW,(uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_SOURCE;
    menuselect(menu,menulen);
}


void ClearPageNum(void)
{
    int i;
    for(i = 0;i < KEYNUM_END;i++)
    {
        PageNum[i].menu_cnt = 1;
        PageNum[i].menu_num = -1;
        PageNum[i].menu_page = 0;
    }
    timesure = 0;
    timeback = 0;
}


static void menu_tankbasicpara(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"罐高",                     COM_NUM_TANKHIGHT,               para_mainprocess, COMMAND_READ, (uint8_t*)"TankHeight"},
        {(uint8_t*)"上电找液位",               COM_NUM_IF_FINDOIL_POWERON,     para_mainprocess, COMMAND_READ, (uint8_t*)"PwrOnFindLvl"},
        {(uint8_t*)"液水传感距离差",           COM_NUM_LEVELTOWATER_HIGH,      para_mainprocess, COMMAND_READ, (uint8_t*)"LvlToWaterDiff"},
        {(uint8_t*)"测罐底最大下行值",         COM_NUM_MAXDOWN_DIS,            para_mainprocess, COMMAND_READ, (uint8_t*)"MaxDownTank"},
        {(uint8_t*)"液位测量方式",             COM_NUM_LEVEL_MEASURE_METHOD,   para_mainprocess, COMMAND_READ, (uint8_t*)"LevelMeasMode"},
        {(uint8_t*)"单温度模式",               COM_NUM_TEMPERATURE_MODE,       para_mainprocess, COMMAND_READ, (uint8_t*)"SingleTempMode"},
        {(uint8_t*)"温度模式密度值",           COM_NUM_TEMPERATURE_MODE_D,     para_mainprocess, COMMAND_READ, (uint8_t*)"TempModeDens"},
        {(uint8_t*)"水位与零点最小距",         COM_NUM_WATER_ZERO_MIN_DISTANCE,para_mainprocess, COMMAND_READ, (uint8_t*)"MinLvlZeroDist"},
        {(uint8_t*)"退出",                     COM_NUM_NOOPERA,                menu_paraconfig,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);
    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_DEBUG_BASIC;
    menuselect(menu, menulen);
}

static void menu_syntheticpara(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"综合测量顺序",             COM_NUM_SRREAD_MEASURETURN,     para_mainprocess, COMMAND_READ, (uint8_t*)"MeasureOrder"},
        {(uint8_t*)"分布测模式",               COM_NUM_SPREAD_STATE,           para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadMode"},
        {(uint8_t*)"综合测是否测罐底",         COM_NUM_SYNTHETIC_BOTTOM,       para_mainprocess, COMMAND_READ, (uint8_t*)"IntgMeasTankBot"},
        {(uint8_t*)"综合测是否测水位",         COM_NUM_SYNTHETIC_WATER,        para_mainprocess, COMMAND_READ, (uint8_t*)"IntgMeasWater"},
        {(uint8_t*)"综合测是否单点",           COM_NUM_SYNTHETIC_SINGLEPOINT,  para_mainprocess, COMMAND_READ, (uint8_t*)"IntgSinglePt"},
        {(uint8_t*)"温度有效位数",             COM_NUM_NUMOFDECIMALS,          para_mainprocess, COMMAND_READ, (uint8_t*)"TempDecimals"},
        {(uint8_t*)"综合测单点位置",           COM_NUM_OIL_MEASUR_POSITION,    para_mainprocess, COMMAND_READ, (uint8_t*)"IntgPos"},
        {(uint8_t*)"退出",                     COM_NUM_NOOPERA,                menu_paraconfig,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);
    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_DEBUG_SYNTH;
    menuselect(menu, menulen);
}

static void menu_spreadpara(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"分布测点数",               COM_NUM_SPREAD_NUM,             para_mainprocess, COMMAND_READ, (uint8_t*)"SpreadPts"},
        {(uint8_t*)"分布点间距",               COM_NUM_SPREAD_DISTANCE,        para_mainprocess, COMMAND_READ, (uint8_t*)"PtSpacing"},
        {(uint8_t*)"顶密距液位",               COM_NUM_SPREAD_TOPLIMIT,        para_mainprocess, COMMAND_READ, (uint8_t*)"TopToLevel"},
        {(uint8_t*)"底密距罐底",               COM_NUM_SPREAD_FLOORLIMIT,      para_mainprocess, COMMAND_READ, (uint8_t*)"BottomToBot"},
        {(uint8_t*)"测密前等待时间",           COM_NUM_DENSITY_TIME,           para_mainprocess, COMMAND_READ, (uint8_t*)"PreDensWait"},
        {(uint8_t*)"分层阈值A",                COM_NUM_THRESHOLD_A,            para_mainprocess, COMMAND_READ, (uint8_t*)"LayerThresA"},
        {(uint8_t*)"分层阈值B",                COM_NUM_THRESHOLD_B,            para_mainprocess, COMMAND_READ, (uint8_t*)"LayerThresB"},
        {(uint8_t*)"国标测点数阈值",           COM_NUM_THRESHOLD_STANDARD,     para_mainprocess, COMMAND_READ, (uint8_t*)"NatThresPts"},
        {(uint8_t*)"退出",                     COM_NUM_NOOPERA,                menu_paraconfig,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);
    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_DEBUG_SPREAD;
    menuselect(menu, menulen);
}

static void menu_correctionpara(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"磁通量T",               COM_NUM_MAGNETIC_T,             para_mainprocess, COMMAND_READ, (uint8_t*)"MAGNETIC_T"},
        {(uint8_t*)"磁通量D",               COM_NUM_MAGNETIC_D,             para_mainprocess, COMMAND_READ, (uint8_t*)"MAGNETIC_D"},
        {(uint8_t*)"退出",                     COM_NUM_NOOPERA,                menu_paraconfig,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);
    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_DEBUG_CORRECT;
    menuselect(menu, menulen);
}

static void menu_realhighpara(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"水位是否实高测",           COM_NUM_WATER_IS_REAL_HIGH,     para_mainprocess, COMMAND_READ, (uint8_t*)"RealHighWLevel"},
        {(uint8_t*)"实高水位修正",             COM_NUM_REAL_WATER_LEVEL_CORRECT,para_mainprocess, COMMAND_READ, (uint8_t*)"RealWaterCorr"},
        {(uint8_t*)"是否更新罐高",             COM_NUM_IF_REFRESH_TANKHIGH,    para_mainprocess, COMMAND_READ, (uint8_t*)"RefreshTankH"},
        {(uint8_t*)"实高最大偏差",             COM_NUM_REAL_TANKHIGH_MAX_DIFF, para_mainprocess, COMMAND_READ, (uint8_t*)"MaxTankHDiff"},
        {(uint8_t*)"退出",                     COM_NUM_NOOPERA,                menu_paraconfig,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);
    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_DEBUG_REALHIGH;
    menuselect(menu, menulen);
}

static void menu_debuginfo(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"零点圈数",                 COM_NUM_ZEROCIRCLE,             para_mainprocess, COMMAND_READ, (uint8_t*)"ZeroCircle"},
        {(uint8_t*)"零点角度",                 COM_NUM_ZEROANGLE,              para_mainprocess, COMMAND_READ, (uint8_t*)"ZeroAngle"},
        {(uint8_t*)"导轮周长",                 COM_NUM_GIRTH_USELESS,          para_mainprocess, COMMAND_READ, (uint8_t*)"GirthUseless"},
        {(uint8_t*)"减速比",                   COM_NUM_REDUCTION_RATIO,        para_mainprocess, COMMAND_READ, (uint8_t*)"ReductRatio"},
        {(uint8_t*)"传感器类型",               COM_NUM_TYPEOFSENSOR,           para_mainprocess, COMMAND_READ, (uint8_t*)"SensorType"},
        {(uint8_t*)"传感器头长度",             COM_NUM_LENGTHOFSENSOR,         para_mainprocess, COMMAND_READ, (uint8_t*)"SensorLength"},
        {(uint8_t*)"盲区",                     COM_NUM_FADEZERO,               para_mainprocess, COMMAND_READ, (uint8_t*)"DeadZone"},
        {(uint8_t*)"程序版本",                 COM_NUM_SOFTOFVERSION,          para_mainprocess, COMMAND_READ, (uint8_t*)"SoftVersion"},
        {(uint8_t*)"导轮周长(整机)",           COM_NUM_GIRTH_YITI,             para_mainprocess, COMMAND_READ, (uint8_t*)"GirthYiti"},
        {(uint8_t*)"传感器编号",               COM_NUM_DEVICENUM,              para_mainprocess, COMMAND_READ, (uint8_t*)"SensorNumber"},
        {(uint8_t*)"退出",                     COM_NUM_NOOPERA,                menu_paraconfig,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);
    all_screen(0x00);
    func_index = KEYNUM_MENU_PARACFG_DEBUG_INFO;
    menuselect(menu, menulen);
}
/* 设置语言 */
static void setlanguage(void)
{
    static struct MenuData menu[] = {
        {(uint8_t*)"中文",COM_NUM_PARA_LANG,setchinese,COMMANE_NORW,(uint8_t*)"CHINESE"},
        {(uint8_t*)"英文",COM_NUM_PARA_LANG,setenglish,COMMANE_NORW,(uint8_t*)"ENGLISH"},
		{(uint8_t*)"退出", COM_NUM_NOOPERA, mainmenu,  COMMANE_NORW, (uint8_t*)"Exit"},
    };
    int menulen = sizeof(menu) / sizeof(menu[0]);

    all_screen(0x00);
    func_index = KEYNUM_MENU_LANGUAGE;
    menuselect(menu,menulen);
}

static void setchinese(void)
{
	screen_parameter.language = LANGUAGE_CHINESE;
//	SaveParamsToFlash(&screen_parameter);
	mainmenu();
}
static void setenglish(void)
{
	screen_parameter.language = LANGUAGE_ENGLISH;
//	SaveParamsToFlash(&screen_parameter);
	mainmenu();
}

