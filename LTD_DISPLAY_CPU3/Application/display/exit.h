#ifndef __EXTI_H
/* __EXTI_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __EXTI_H 
#include "main.h"



extern bool FlagofTankOpera;

/* 按键位图中返回键对应的独立位；多个按键状态可以按位组合，判断时必须使用掩码而不是相等比较。 */
#define USE_KEY_BACK    0x01
/* 按键位图中上移键对应的独立位；多个按键状态可以按位组合，判断时必须使用掩码而不是相等比较。 */
#define USE_KEY_UP      0x02
/* 按键位图中下移键对应的独立位；多个按键状态可以按位组合，判断时必须使用掩码而不是相等比较。 */
#define USE_KEY_DOWN    0x04
/* 按键位图中确认键对应的独立位；多个按键状态可以按位组合，判断时必须使用掩码而不是相等比较。 */
#define USE_KEY_SURE    0x08


extern volatile uint8_t button_press_counter;
extern volatile uint8_t button_long_press_key;

/* 长按识别结果编码：无长按事件；该值描述一次已确认事件，不等同于实时按键位图。 */
#define LONG_PRESS_KEY_NONE 0u
/* 长按识别结果编码：确认键长按事件；该值描述一次已确认事件，不等同于实时按键位图。 */
#define LONG_PRESS_KEY_SURE 1u
/* 长按识别结果编码：返回键长按事件；该值描述一次已确认事件，不等同于实时按键位图。 */
#define LONG_PRESS_KEY_BACK 2u
#define REQUIRED_PRESS_COUNT 30  /* TIM1 约 100ms 一次，30 次约 3s */
/* 按键电平变化后的软件消抖时间 50 ms；在该窗口内不生成新的稳定按键事件。 */
#define DISPLAY_KEY_DEBOUNCE_MS 50U
/* 长按事件触发后等待按键释放的保护时间 50 ms；用于防止同一次长按紧接着被识别为普通短按。 */
#define DISPLAY_LONG_PRESS_RELEASE_GUARD_MS 50U


/**
 * @brief 将有效按键加入环形待处理队列；队列满时丢弃新按键并累计溢出次数。
 *
 * @param keypress 本次待分发的按键位掩码。
 */
void Display_RequestKey(uint8_t keypress);
/**
 * @brief 为指定按键登记一次长按动作请求。
 *
 * @param long_press_key 长按按键。
 */
void Display_RequestLongPressAction(uint8_t long_press_key);
/**
 * @brief 从按键环形队列取出下一条待处理按键。
 * @return 返回下一条待处理按键编码；队列为空时返回 0。
 */
uint8_t Display_TakePendingKey(void);
/**
 * @brief 原子取出并清除待处理长按动作。
 * @return 返回并清除一条待处理长按按键编码；没有待处理动作时返回 0。
 */
uint8_t Display_TakePendingLongPressAction(void);
/**
 * @brief 清空待处理按键队列和长按状态。
 */
void Display_ClearPendingKeys(void);
/**
 * @brief 启用长按释放保护，避免释放沿被误识别为短按。
 *
 * @param long_press_key 长按按键。
 */
void Display_ArmLongPressReleaseGuard(uint8_t long_press_key);
/**
  * @brief 按按键释放状态维护长按后的防重复触发门限。
 */
void Display_UpdateLongPressReleaseGuard(void);






#endif


