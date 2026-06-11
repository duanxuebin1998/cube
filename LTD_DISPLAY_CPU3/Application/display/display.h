#ifndef __DISPLAY_H
#define __DISPLAY_H
#include "main.h"
#include "app_version.h"

#define CPU3VERSION CPU3_APP_VERSION_DISPLAY_VALUE

#define CONVER_QUALITY_P 3 /* 质量显示几位小数 */
#define CONVER_VOLUME_P 3 /* 体积显示几位小数 */

#define OLED_BRIGHTNESS_LEVEL_DARK      0U
#define OLED_BRIGHTNESS_LEVEL_LOW       1U
#define OLED_BRIGHTNESS_LEVEL_STANDARD  2U
#define OLED_BRIGHTNESS_LEVEL_HIGH      3U
#define OLED_BRIGHTNESS_LEVEL_MAX       4U
#define OLED_BRIGHTNESS_LEVEL_COUNT     5U

/* 行 - 共4行 */
#define OLED_ROW4_1     0
#define OLED_ROW4_2     16
#define OLED_ROW4_3     32
#define OLED_ROW4_4     48
/* 行 - 共3行 */
#define OLED_ROW3_1     0
#define OLED_ROW3_2     22
#define OLED_ROW3_3     45
/* 列 - 共8列 */
#define OLED_LINE8_1    0
#define OLED_LINE8_2    7
#define OLED_LINE8_3    14
#define OLED_LINE8_4    21
#define OLED_LINE8_5    28
#define OLED_LINE8_6    35
#define OLED_LINE8_7    42
#define OLED_LINE8_8    49
#define OLED_LINE8_9    56
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
    int input_val_water;
    int input_val_d;
    int input_val_t;
    int language; /* 语言 */
    int screenoff; /* 息屏 */
    int brightness; /* 屏幕亮度挡位 */
};
typedef enum{
    LANGUAGE_CHINESE,
    LANGUAGE_ENGLISH,  
}LANGUAGE_TYPE;
typedef struct {
	uint16_t state;
	const char *disp_cn;
	const char *disp_en;
} EquipStateDisplay; /* 设备状态显示 */
extern struct ScreenPARA screen_parameter;
/**
 * @brief 执行屏幕显示中的 EquipFirstPower 逻辑。
 */
void EquipFirstPower(void);
/**
 * @brief 显示或打印屏幕显示中的 DisplayInit 逻辑。
 */
void DisplayInit(void);
/**
 * @brief 显示或打印屏幕显示中的 Display_Task 逻辑。
 */
void Display_Task(void);
/**
 * @brief 更新屏幕显示中的 Display_RequestRefresh 逻辑。
 */
void Display_RequestRefresh(void);
/**
 * @brief 显示或打印屏幕显示中的 OledDisplayLineWords 逻辑。
 *
 * @param data 数据缓冲区。
 * @param x 业务参数。
 * @param y 业务参数。
 * @param shift 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t OledDisplayLineWords(uint8_t* data,uint8_t x,uint8_t y,uint8_t shift);
/**
 * @brief 显示或打印屏幕显示中的 DisplayLangaugeLineWords 逻辑。
 *
 * @param name1 业务参数。
 * @param line 业务参数。
 * @param row 业务参数。
 * @param shift 业务参数。
 * @param name2 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t DisplayLangaugeLineWords(uint8_t* name1,uint8_t line,uint8_t row,uint8_t shift,uint8_t* name2);
/**
 * @brief 更新屏幕显示中的 RefreshScreen 逻辑。
 */
void RefreshScreen(void);
/**
 * @brief 显示或打印屏幕显示中的 Display_ShowErrorReasonPage 逻辑。
 */
void Display_ShowErrorReasonPage(void);
/**
 * @brief 显示或打印屏幕显示中的 OledDisplayOneNmb 逻辑。
 *
 * @param c 业务参数。
 * @param row 业务参数。
 * @param line 业务参数。
 * @param shift 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t OledDisplayOneNmb(int c,uint8_t row,uint8_t line,uint8_t shift);
/**
 * @brief 显示或打印屏幕显示中的 OledValueDisplay 逻辑。
 *
 * @param value 待处理数值。
 * @param line 业务参数。
 * @param row 业务参数。
 * @param shift 业务参数。
 * @param points 输入/输出指针。
 * @param unit 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t OledValueDisplay(int value,uint8_t line,uint8_t row,uint8_t shift,uint8_t points,uint8_t* unit);
/**
 * @brief 显示或打印屏幕显示中的 DisplayAubonLogo 逻辑。
 */
void DisplayAubonLogo(void);
/**
 * @brief 写入或设置屏幕显示中的 SetScreenBright 逻辑。
 */
void SetScreenBright( void );




#endif

