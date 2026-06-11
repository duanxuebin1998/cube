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
#define REQUIRED_PRESS_COUNT 15  // TIM1 约 100ms 一次，15 次约 1.5s
#define DISPLAY_KEY_DEBOUNCE_MS 50U
#define DISPLAY_LONG_PRESS_RELEASE_GUARD_MS 50U


void Display_RequestKey(uint8_t keypress);
void Display_RequestLongPressAction(uint8_t long_press_key);
uint8_t Display_TakePendingKey(void);
uint8_t Display_TakePendingLongPressAction(void);
void Display_ClearPendingKeys(void);
void Display_ArmLongPressReleaseGuard(uint8_t long_press_key);
void Display_UpdateLongPressReleaseGuard(void);






#endif


