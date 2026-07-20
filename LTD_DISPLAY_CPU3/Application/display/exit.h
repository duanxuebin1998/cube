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
#define REQUIRED_PRESS_COUNT 30  /* TIM1 约 100ms 一次，30 次约 3s */
#define DISPLAY_KEY_DEBOUNCE_MS 50U
#define DISPLAY_LONG_PRESS_RELEASE_GUARD_MS 50U


/**
 * @brief 显示或打印屏幕显示中的 Display_RequestKey 逻辑。
 *
 * @param keypress 业务参数。
 */
void Display_RequestKey(uint8_t keypress);
/**
 * @brief 显示或打印屏幕显示中的 Display_RequestLongPressAction 逻辑。
 *
 * @param long_press_key 业务参数。
 */
void Display_RequestLongPressAction(uint8_t long_press_key);
/**
 * @brief 显示或打印屏幕显示中的 Display_TakePendingKey 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t Display_TakePendingKey(void);
/**
 * @brief 显示或打印屏幕显示中的 Display_TakePendingLongPressAction 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t Display_TakePendingLongPressAction(void);
/**
 * @brief 清除或复位屏幕显示中的 Display_ClearPendingKeys 逻辑。
 */
void Display_ClearPendingKeys(void);
/**
 * @brief 显示或打印屏幕显示中的 Display_ArmLongPressReleaseGuard 逻辑。
 *
 * @param long_press_key 业务参数。
 */
void Display_ArmLongPressReleaseGuard(uint8_t long_press_key);
/**
 * @brief 更新屏幕显示中的 Display_UpdateLongPressReleaseGuard 逻辑。
 */
void Display_UpdateLongPressReleaseGuard(void);






#endif


