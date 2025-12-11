#ifndef __HGS_H
#define __HGS_H 
#include "main.h"

#define oled_clear() all_screen(0x00)
#define DIS_MAXROWBYTE 18//屏幕一行最多能显示的汉字为9个,共占18个字节


void OLED_Init(void);
void all_screen(uint8_t m);
void CLS_scope(uint8_t row_start,uint8_t row_end,uint8_t column_start,uint8_t column_end);
void write_hanzi16(uint8_t x,uint8_t y,uint8_t *buf,uint8_t m,uint8_t endm,uint8_t select);
void write_816(uint8_t x,uint8_t y, uint8_t *buf,uint8_t coder,uint8_t en);
void oled_map_128_64(uint8_t * map);




#endif



