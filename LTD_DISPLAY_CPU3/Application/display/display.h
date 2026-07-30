#ifndef __DISPLAY_H
/* __DISPLAY_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __DISPLAY_H
#include "main.h"
#include "app_version.h"

/* CPU3 显示界面使用的软件版本数值别名；直接引用统一版本源 CPU3_APP_VERSION_DISPLAY_VALUE，禁止在显示模块单独维护版本号。 */
#define CPU3VERSION CPU3_APP_VERSION_DISPLAY_VALUE

#define CONVER_QUALITY_P 3 /* 质量显示几位小数 */
#define CONVER_VOLUME_P 3 /* 体积显示几位小数 */

/* OLED 亮度档位编码 0：全暗；该编码用于参数存储和菜单选择，不是直接写入控制器的 PWM 数值。 */
#define OLED_BRIGHTNESS_LEVEL_DARK      0U
/* OLED 亮度档位编码 1：低亮；该编码用于参数存储和菜单选择，不是直接写入控制器的 PWM 数值。 */
#define OLED_BRIGHTNESS_LEVEL_LOW       1U
/* OLED 亮度档位编码 2：标准亮度；该编码用于参数存储和菜单选择，不是直接写入控制器的 PWM 数值。 */
#define OLED_BRIGHTNESS_LEVEL_STANDARD  2U
/* OLED 亮度档位编码 3：高亮；该编码用于参数存储和菜单选择，不是直接写入控制器的 PWM 数值。 */
#define OLED_BRIGHTNESS_LEVEL_HIGH      3U
/* OLED 亮度档位编码 4：最高亮度；该编码用于参数存储和菜单选择，不是直接写入控制器的 PWM 数值。 */
#define OLED_BRIGHTNESS_LEVEL_MAX       4U
/* OLED 可选亮度档位总数 5；合法档位索引范围为 0～4，菜单循环和参数校验必须使用该计数。 */
#define OLED_BRIGHTNESS_LEVEL_COUNT     5U

/* 行 - 共4行 */
/* OLED 4 行布局中第 1 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW4_1     0
/* OLED 4 行布局中第 2 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW4_2     16
/* OLED 4 行布局中第 3 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW4_3     32
/* OLED 4 行布局中第 4 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW4_4     48
/* 行 - 共3行 */
/* OLED 3 行布局中第 1 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW3_1     0
/* OLED 3 行布局中第 2 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW3_2     22
/* OLED 3 行布局中第 3 行的 Y 像素起始坐标；文本基线和行高由对应字体布局统一确定。 */
#define OLED_ROW3_3     45
/* 列 - 共8列 */
/* OLED 八等分横向布局中第 1 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_1    0
/* OLED 八等分横向布局中第 2 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_2    7
/* OLED 八等分横向布局中第 3 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_3    14
/* OLED 八等分横向布局中第 4 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_4    21
/* OLED 八等分横向布局中第 5 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_5    28
/* OLED 八等分横向布局中第 6 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_6    35
/* OLED 八等分横向布局中第 7 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_7    42
/* OLED 八等分横向布局中第 8 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_8    49
/* OLED 八等分横向布局中第 9 个分隔坐标；用于菜单字段和监测值定位，单位为显示像素列。 */
#define OLED_LINE8_9    56
/* OLED 八等分横向布局的最右有效像素列 63；右对齐文本应以此边界计算宽度，不能依赖尾随空格。 */
#define OLED_LINE8_END  63

typedef struct {
	/* 波特率 */
	/* 数据位 */
	/* 校验 */
	/* 停止位 */
	/* 通信协议 */
} COMM; /* 通信参数结构体 */
struct ScreenPARA{
	uint32_t flag;          /* 标志字段，必须放第一位或者单独定义 */
    int decimalplaces; /* 小数点 */
    int passward; /* 密码 */
    int input_val_oil;
    /* OLED 首页可配置的水位、密度、温度显示开关和语言选择。 */
    int input_val_water; /* 首页水位字段的显示使能值，由参数菜单配置。 */
    int input_val_d; /* 首页密度字段的显示使能值，由参数菜单配置。 */
    int input_val_t; /* 首页温度字段的显示使能值，由参数菜单配置。 */
    int language; /* 语言 */
    int screenoff; /* 息屏 */
    int brightness; /* 屏幕亮度挡位 */
};
/* OLED 界面语言选择；该枚举只影响显示文本，不改变协议数值、参数单位或业务状态。 */
typedef enum{
    /* OLED 显示语言选项。 */
    LANGUAGE_CHINESE, /* 显示中文界面。 */
    LANGUAGE_ENGLISH, /* 显示英文界面。 */
}LANGUAGE_TYPE;
typedef struct {
	/* 设备状态码到中英文状态文本的只读映射。 */
	uint16_t state; /* CPU2 设备主状态码，作为中英文状态文本查表键。 */
	const char *disp_cn; /* 该设备状态对应的中文只读显示文本。 */
	const char *disp_en; /* 该设备状态对应的英文只读显示文本。 */
} EquipStateDisplay; /* 设备状态显示 */
extern struct ScreenPARA screen_parameter;
/**
 * @brief 把设备运行状态转换为当前语言的状态文字。
 *
 * @param state CPU2 共享的 16 位设备状态码；函数按当前语言查找对应状态文字。
 * @param lang 状态文字使用的语言枚举值。
 * @return 返回当前语言的状态文字对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char* GetStateString(uint16_t state, uint8_t lang);
/**
 * @brief 设备刚上电还未与CPU2通讯时显示初始化中。
 */
void EquipFirstPower(void);
/**
 * @brief 初始化 OLED、显示恢复节拍、按键计时、刷新定时器以及菜单输入状态。
 */
void DisplayInit(void);
/**
  * @brief 处理显示刷新、按键事件和状态页增量更新。
 */
void Display_Task(void);
/**
 * @brief 请求主循环执行一次屏幕刷新，不在中断中直接绘制 OLED。
 */
void Display_RequestRefresh(void);
/**
 * @brief 显示多个汉字或字符。
 *
 * @param data 待绘制的 NUL 结尾 OLED 文字字节串；ASCII 和中文字节按当前字库规则依次推进横向列坐标。
 * @param x 算法、坐标或比较使用的 X 值。
 * @param y 算法、坐标或比较使用的 Y 值。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 * @return 返回字符串绘制完成后的下一 OLED 横坐标；data 为空时保持传入横坐标 y。
 */
uint8_t OledDisplayLineWords(uint8_t* data,uint8_t x,uint8_t y,uint8_t shift);
/**
 * @brief 显示多个汉字或字符 - 带中英文选择。
 *
 * @param name1 中文模式使用的显示文字。
 * @param line OLED 绘制使用的横向列位置。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 * @param name2 英文模式使用的显示文字；为 NULL 时沿用中文文字。
 * @return 返回当前语言文本绘制完成后的下一 OLED 横坐标；英文文本为空时使用中文文本。
 */
uint8_t DisplayLangaugeLineWords(uint8_t* name1,uint8_t line,uint8_t row,uint8_t shift,uint8_t* name2);
/**
 * @brief 根据当前前景页、恢复标志和状态快照选择局部或全量方式刷新 OLED。
 *
 * @note 罐上操作页只在监控页、恢复请求或显式重绘时刷新；状态页按布局变化选择全量绘制或差量绘制，并在每帧结束时核对 OLED SPI 错误计数。
 */
void RefreshScreen(void);
/**
 * @brief 绘制当前故障的详细原因查看页。
 */
void Display_ShowErrorReasonPage(void);
/**
 * @brief 显示一个数字。
 *
 * @param c 待绘制的十进制数字，合法范围为 0 至 9。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param line OLED 绘制使用的横向列位置。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 * @return 返回单个数字字模绘制完成后的下一 OLED 横坐标，即传入 line 加 4。
 */
uint8_t OledDisplayOneNmb(int c,uint8_t row,uint8_t line,uint8_t shift);
/**
 * @brief 裁剪定点数尾随零后，在 OLED 指定位置绘制带符号数值及可选工程单位。
 *
 * @param value 待格式化并绘制到 OLED 的数值。
 * @param line OLED 绘制使用的横向列位置。
 * @param row OLED 绘制使用的行号。取值使用 OLED_ROW4_x 等页面行坐标常量，决定文字或数字写入的纵向基线。
 * @param shift OLED 字模阴码/阳码或显示偏移选项。
 * @param points value 当前采用的小数位数；函数先裁掉尾随零，再按剩余位数插入小数点。
 * @param unit 数值对应的工程单位文字。
 * @return 返回数值及可选单位绘制完成后的下一 OLED 横坐标。
 */
uint8_t OledValueDisplay(int value,uint8_t line,uint8_t row,uint8_t shift,uint8_t points,uint8_t* unit);
/**
 * @brief OLED 初始化完成后立即显示启动页。
 */
void DisplayAubonLogo(void);
/**
 * @brief 写入或设置屏幕显示中的 SetScreenBright 逻辑。
 */
void SetScreenBright( void );




#endif

