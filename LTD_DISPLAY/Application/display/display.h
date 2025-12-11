#ifndef __DISPLAY_H
#define __DISPLAY_H
#include "main.h"

#define CPU3VERSION 1002

#define CONVER_QUALITY_P 3//质量显示几位小数
#define CONVER_VOLUME_P 3//体积显示几位小数

/*行 - 共4行*/
#define OLED_ROW4_1     0
#define OLED_ROW4_2     16
#define OLED_ROW4_3     32
#define OLED_ROW4_4     48
/*行 - 共3行*/
#define OLED_ROW3_1     0
#define OLED_ROW3_2     22
#define OLED_ROW3_3     45
/*列 - 共8列*/
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
	//波特率
	//数据位
	//校验
	//停止位
	//通信协议
} COMM;//通信参数结构体
struct ScreenPARA{
	uint32_t flag;          // 标志字段，必须放第一位或者单独定义
    int decimalplaces; //小数点
    int passward;//密码
    int input_val_oil;
    int input_val_water;
    int input_val_d;
    int input_val_t;
    int language;//语言
    int screenoff;//息屏
};
typedef enum{
    LANGUAGE_CHINESE,
    LANGUAGE_ENGLISH,  
}LANGUAGE_TYPE;
typedef struct {
	uint16_t state;
	const char *disp_cn;
	const char *disp_en;
} EquipStateDisplay;//设备状态显示
extern struct ScreenPARA screen_parameter;
void EquipFirstPower(void);
void DisplayInit(void);
uint8_t OledDisplayLineWords(uint8_t* data,uint8_t x,uint8_t y,uint8_t shift);
uint8_t DisplayLangaugeLineWords(uint8_t* name1,uint8_t line,uint8_t row,uint8_t shift,uint8_t* name2);
void RefreshScreen(void);
uint8_t OledDisplayOneNmb(int c,uint8_t row,uint8_t line,uint8_t shift);
uint8_t OledValueDisplay(int value,uint8_t line,uint8_t row,uint8_t shift,uint8_t points,uint8_t* unit);
void DisplayAubonLogo(void);
void SetScreenBright( void );




#endif

