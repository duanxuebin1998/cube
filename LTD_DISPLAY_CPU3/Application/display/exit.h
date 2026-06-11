#ifndef __EXTI_H
#define __EXTI_H 
#include "main.h"



extern bool FlagofTankOpera;

#define USE_KEY_BACK    0x01
#define USE_KEY_UP      0x02
#define USE_KEY_DOWN    0x04
#define USE_KEY_SURE    0x08


extern volatile uint8_t button_press_counter;
extern volatile uint8_t button_long_press_key;

#define LONG_PRESS_KEY_NONE 0u
#define LONG_PRESS_KEY_SURE 1u
#define LONG_PRESS_KEY_BACK 2u
#define REQUIRED_PRESS_COUNT 30  // 需要连续按下3次才触发


void Display_RequestKey(uint8_t keypress);
void Display_RequestLongPressAction(uint8_t long_press_key);
uint8_t Display_TakePendingKey(void);
uint8_t Display_TakePendingLongPressAction(void);






#endif


