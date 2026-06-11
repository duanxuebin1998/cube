#include "exit.h"
#include "display_tankopera.h"
#include "display.h"
#include "tim.h"

#define DISPLAY_KEY_QUEUE_SIZE 8U

volatile uint8_t button_press_counter = 0;
volatile uint8_t button_long_press_key = LONG_PRESS_KEY_NONE;
bool FlagofTankOpera = false; //罐上操作标志位
static volatile uint8_t pending_key_queue[DISPLAY_KEY_QUEUE_SIZE];
static volatile uint8_t pending_key_head = 0U;
static volatile uint8_t pending_key_tail = 0U;
static volatile uint8_t pending_key_overflow_count = 0U;
static volatile uint8_t pending_long_press_mask = 0U;

static uint32_t Display_EnterCritical(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

static void Display_ExitCritical(uint32_t primask)
{
	if (primask == 0U) {
		__enable_irq();
	}
}

static uint8_t Display_NextKeyQueueIndex(uint8_t index)
{
	index++;
	if (index >= DISPLAY_KEY_QUEUE_SIZE) {
		index = 0U;
	}
	return index;
}

void Display_RequestKey(uint8_t keypress)
{
	uint8_t next_head = Display_NextKeyQueueIndex(pending_key_head);

	if (next_head == pending_key_tail) {
		/* 队列满时丢弃最新按键，避免中断覆盖尚未处理的历史按键。 */
		pending_key_overflow_count++;
		return;
	}

	pending_key_queue[pending_key_head] = keypress;
	pending_key_head = next_head;
}

void Display_RequestLongPressAction(uint8_t long_press_key)
{
	if (long_press_key == LONG_PRESS_KEY_SURE) {
		pending_long_press_mask |= LONG_PRESS_KEY_SURE;
	} else if (long_press_key == LONG_PRESS_KEY_BACK) {
		pending_long_press_mask |= LONG_PRESS_KEY_BACK;
	}
}

uint8_t Display_TakePendingKey(void)
{
	uint8_t keypress = 0U;
	uint32_t primask = Display_EnterCritical();

	if (pending_key_tail != pending_key_head) {
		keypress = pending_key_queue[pending_key_tail];
		pending_key_tail = Display_NextKeyQueueIndex(pending_key_tail);
	}

	Display_ExitCritical(primask);
	return keypress;
}

uint8_t Display_TakePendingLongPressAction(void)
{
	uint8_t long_press_key = LONG_PRESS_KEY_NONE;
	uint32_t primask = Display_EnterCritical();

	if ((pending_long_press_mask & LONG_PRESS_KEY_SURE) != 0U) {
		long_press_key = LONG_PRESS_KEY_SURE;
	} else if ((pending_long_press_mask & LONG_PRESS_KEY_BACK) != 0U) {
		long_press_key = LONG_PRESS_KEY_BACK;
	}

	if (long_press_key != LONG_PRESS_KEY_NONE) {
		pending_long_press_mask &= (uint8_t)(~long_press_key);
	}

	Display_ExitCritical(primask);
	return long_press_key;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	SetScreenBright();
	//延时消抖
//    HAL_Delay(5);
	switch (GPIO_Pin) {
	case KEY_BACK_Pin: {
		if (HAL_GPIO_ReadPin(KEY_BACK_GPIO_Port, KEY_BACK_Pin) == GPIO_PIN_RESET) {
			if (FlagofTankOpera == true) {
				Display_RequestKey(USE_KEY_BACK);
			} else {
				/* 状态显示界面长按返回键用于请求取消当前测量。 */
				button_press_counter = 0;
				button_long_press_key = LONG_PRESS_KEY_BACK;
				HAL_TIM_Base_Start_IT(&htim1);
			}
		}
		break;
	}
	case KEY_UP_Pin: {
		if (HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET) {
			if (FlagofTankOpera == true)
				Display_RequestKey(USE_KEY_UP);
		}
		break;
	}
	case KEY_DOWN_Pin: {
		if (HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET) {
			if (FlagofTankOpera == true)
				Display_RequestKey(USE_KEY_DOWN);
		}
		break;
	}
	case KEY_SURE_Pin: {
		if (HAL_GPIO_ReadPin(KEY_SURE_GPIO_Port, KEY_SURE_Pin) == GPIO_PIN_RESET) {
			if (FlagofTankOpera == true)
				Display_RequestKey(USE_KEY_SURE);
			else {
//				__HAL_TIM_SET_COUNTER(&htim1, 0);
				button_press_counter = 0;
				button_long_press_key = LONG_PRESS_KEY_SURE;
				HAL_TIM_Base_Start_IT(&htim1); //解锁
			}
		}
		break;
	}
	default:
		break;
	}
}

