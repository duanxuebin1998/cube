#ifndef __HGS_H
#define __HGS_H 
#include "main.h"

#define oled_clear() OLED_Clear()
#define DIS_MAXROWBYTE 18//屏幕一行最多能显示的汉字为9个,共占18个字节


void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width_cols, uint8_t height_rows);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_SetContrast(uint8_t contrast);
void OLED_SetBrightnessLevel(uint8_t level);
void OLED_RecoverAndClear(void);
void OLED_MarkFrameComplete(void);
uint32_t OLED_GetSpiErrorCount(void);
uint32_t OLED_GetShadowCrc(void);
uint32_t OLED_GetRefreshSeq(void);
void all_screen(uint8_t m);
void write_hanzi16(uint8_t x,uint8_t y,uint8_t *buf,uint8_t m,uint8_t endm,uint8_t select);
void write_816(uint8_t x,uint8_t y, uint8_t *buf,uint8_t coder,uint8_t en);




#endif



