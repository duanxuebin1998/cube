#include "exit.h"
#include "display_tankopera.h"
#include "display.h"
#include "tim.h"

#define DISPLAY_KEY_QUEUE_SIZE 8U
#define DISPLAY_KEY_INDEX_COUNT 4U

volatile uint8_t button_press_counter = 0; /* 屏幕显示计数值，用于节拍、统计或协议数量控制。 */
volatile uint8_t button_long_press_key = LONG_PRESS_KEY_NONE; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
bool FlagofTankOpera = false; /* 罐上操作标志位 */
static volatile uint8_t pending_key_queue[DISPLAY_KEY_QUEUE_SIZE]; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static volatile uint8_t pending_key_head = 0U; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static volatile uint8_t pending_key_tail = 0U; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static volatile uint8_t pending_key_overflow_count = 0U; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static volatile uint8_t pending_long_press_mask = 0U; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static volatile uint32_t key_last_falling_tick[DISPLAY_KEY_INDEX_COUNT]; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static volatile uint8_t key_last_falling_valid[DISPLAY_KEY_INDEX_COUNT]; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static volatile uint8_t long_press_release_guard_key = LONG_PRESS_KEY_NONE; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static volatile uint8_t long_press_release_guard_released = 0U; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
static volatile uint32_t long_press_release_guard_tick = 0U; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */

/**
 * @brief 显示或打印屏幕显示中的 Display_EnterCritical 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Display_EnterCritical(void)
{
	uint32_t primask = __get_PRIMASK();
	if (__get_IPSR() == 0U) {
		/* 进入临界区，保护屏幕显示共享状态，避免中断同时修改。 */
		__disable_irq();
	}
	return primask;
}

/**
 * @brief 显示或打印屏幕显示中的 Display_ExitCritical 逻辑。
 *
 * @param primask 进入临界区前保存的中断屏蔽状态。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void Display_ExitCritical(uint32_t primask)
{
	if ((__get_IPSR() == 0U) && (primask == 0U)) {
		__enable_irq();
	}
}

/**
 * @brief 显示或打印屏幕显示中的 Display_NextKeyQueueIndex 逻辑。
 *
 * @param index 索引值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t Display_NextKeyQueueIndex(uint8_t index)
{
	index++;
	if (index >= DISPLAY_KEY_QUEUE_SIZE) {
		index = 0U;
	}
	return index;
}

/**
 * @brief 显示或打印屏幕显示中的 Display_KeyIndexFromPin 逻辑。
 *
 * @param GPIO_Pin 业务参数。
 * @param index 索引值。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
static bool Display_KeyIndexFromPin(uint16_t GPIO_Pin, uint8_t *index)
{
	if (GPIO_Pin == KEY_BACK_Pin) {
		*index = 0U;
	} else if (GPIO_Pin == KEY_UP_Pin) {
		*index = 1U;
	} else if (GPIO_Pin == KEY_DOWN_Pin) {
		*index = 2U;
	} else if (GPIO_Pin == KEY_SURE_Pin) {
		*index = 3U;
	} else {
		return false;
	}

	return true;
}

/**
 * @brief 显示或打印屏幕显示中的 Display_AcceptKeyFalling 逻辑。
 *
 * @param GPIO_Pin 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
static bool Display_AcceptKeyFalling(uint16_t GPIO_Pin)
{
	uint8_t index = 0U;
	uint32_t now = HAL_GetTick();
	uint32_t primask;
	bool accepted = true;

	if (!Display_KeyIndexFromPin(GPIO_Pin, &index)) {
		return true;
	}

	primask = Display_EnterCritical();
	if ((key_last_falling_valid[index] != 0U) &&
	    ((uint32_t)(now - key_last_falling_tick[index]) < DISPLAY_KEY_DEBOUNCE_MS)) {
		accepted = false;
	} else {
		key_last_falling_tick[index] = now;
		key_last_falling_valid[index] = 1U;
	}
	Display_ExitCritical(primask);

	return accepted;
}

/**
 * @brief 读取屏幕显示中的 Display_ReadLongPressKey 逻辑。
 *
 * @param long_press_key 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static GPIO_PinState Display_ReadLongPressKey(uint8_t long_press_key)
{
	if (long_press_key == LONG_PRESS_KEY_SURE) {
		return HAL_GPIO_ReadPin(KEY_SURE_GPIO_Port, KEY_SURE_Pin);
	}
	if (long_press_key == LONG_PRESS_KEY_BACK) {
		return HAL_GPIO_ReadPin(KEY_BACK_GPIO_Port, KEY_BACK_Pin);
	}

	return GPIO_PIN_SET;
}

/**
 * @brief 显示或打印屏幕显示中的 Display_StartLongPress 逻辑。
 *
 * @param long_press_key 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void Display_StartLongPress(uint8_t long_press_key)
{
	bool should_start = false;
	bool locked_key_released = false;
	uint32_t primask = Display_EnterCritical();

	if ((button_long_press_key != LONG_PRESS_KEY_NONE) &&
	    (Display_ReadLongPressKey(button_long_press_key) == GPIO_PIN_SET)) {
		button_press_counter = 0;
		button_long_press_key = LONG_PRESS_KEY_NONE;
		locked_key_released = true;
	}

	if ((button_long_press_key == LONG_PRESS_KEY_NONE) &&
	    (long_press_release_guard_key == LONG_PRESS_KEY_NONE)) {
		button_press_counter = 0;
		button_long_press_key = long_press_key;
		should_start = true;
	}

	Display_ExitCritical(primask);

	if (locked_key_released) {
		HAL_TIM_Base_Stop_IT(&htim1);
	}
	if (should_start) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
		HAL_TIM_Base_Start_IT(&htim1);
	}
}

/**
 * @brief 显示或打印屏幕显示中的 Display_RequestKey 逻辑。
 *
 * @param keypress 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Display_RequestKey(uint8_t keypress)
{
	uint8_t next_head = Display_NextKeyQueueIndex(pending_key_head);

	if (long_press_release_guard_key != LONG_PRESS_KEY_NONE) {
		return;
	}

	if (next_head == pending_key_tail) {
		/* 队列满时丢弃最新按键，避免中断覆盖尚未处理的历史按键。 */
		pending_key_overflow_count++;
		return;
	}

	pending_key_queue[pending_key_head] = keypress;
	pending_key_head = next_head;
}

/**
 * @brief 清除或复位屏幕显示中的 Display_ClearPendingKeys 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Display_ClearPendingKeys(void)
{
	uint32_t primask = Display_EnterCritical();
	pending_key_tail = pending_key_head;
	Display_ExitCritical(primask);
}

/**
 * @brief 显示或打印屏幕显示中的 Display_RequestLongPressAction 逻辑。
 *
 * @param long_press_key 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Display_RequestLongPressAction(uint8_t long_press_key)
{
	if (long_press_key == LONG_PRESS_KEY_SURE) {
		pending_long_press_mask |= LONG_PRESS_KEY_SURE;
	} else if (long_press_key == LONG_PRESS_KEY_BACK) {
		pending_long_press_mask |= LONG_PRESS_KEY_BACK;
	}
}

/**
 * @brief 显示或打印屏幕显示中的 Display_TakePendingKey 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
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

/**
 * @brief 显示或打印屏幕显示中的 Display_TakePendingLongPressAction 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
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

/**
 * @brief 显示或打印屏幕显示中的 Display_ArmLongPressReleaseGuard 逻辑。
 *
 * @param long_press_key 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Display_ArmLongPressReleaseGuard(uint8_t long_press_key)
{
	uint32_t primask = Display_EnterCritical();

	pending_key_tail = pending_key_head;
	long_press_release_guard_key = long_press_key;
	long_press_release_guard_released = 0U;
	long_press_release_guard_tick = HAL_GetTick();

	Display_ExitCritical(primask);
}

/**
 * @brief 更新屏幕显示中的 Display_UpdateLongPressReleaseGuard 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Display_UpdateLongPressReleaseGuard(void)
{
	uint8_t guard_key = long_press_release_guard_key;
	uint32_t now;

	if (guard_key == LONG_PRESS_KEY_NONE) {
		return;
	}

	now = HAL_GetTick();
	if (long_press_release_guard_released == 0U) {
		if (Display_ReadLongPressKey(guard_key) == GPIO_PIN_SET) {
			long_press_release_guard_released = 1U;
			long_press_release_guard_tick = now;
		}
		return;
	}

	if ((uint32_t)(now - long_press_release_guard_tick) >= DISPLAY_LONG_PRESS_RELEASE_GUARD_MS) {
		long_press_release_guard_key = LONG_PRESS_KEY_NONE;
		long_press_release_guard_released = 0U;
	}
}

/**
 * @brief 执行屏幕显示中的 HAL_GPIO_EXTI_Callback 逻辑。
 *
 * @param GPIO_Pin 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	SetScreenBright();
	/* 延时消抖 */
/* HAL_Delay(5); */
	switch (GPIO_Pin) {
	case KEY_BACK_Pin: {
		if ((HAL_GPIO_ReadPin(KEY_BACK_GPIO_Port, KEY_BACK_Pin) == GPIO_PIN_RESET) &&
		    Display_AcceptKeyFalling(GPIO_Pin)) {
			if (FlagofTankOpera == true) {
				Display_RequestKey(USE_KEY_BACK);
			} else {
				/* 状态显示界面长按返回键用于请求取消当前测量。 */
				Display_StartLongPress(LONG_PRESS_KEY_BACK);
			}
		}
		break;
	}
	case KEY_UP_Pin: {
		if ((HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET) &&
		    Display_AcceptKeyFalling(GPIO_Pin)) {
			if (FlagofTankOpera == true)
				Display_RequestKey(USE_KEY_UP);
		}
		break;
	}
	case KEY_DOWN_Pin: {
		if ((HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET) &&
		    Display_AcceptKeyFalling(GPIO_Pin)) {
			if (FlagofTankOpera == true)
				Display_RequestKey(USE_KEY_DOWN);
		}
		break;
	}
	case KEY_SURE_Pin: {
		if ((HAL_GPIO_ReadPin(KEY_SURE_GPIO_Port, KEY_SURE_Pin) == GPIO_PIN_RESET) &&
		    Display_AcceptKeyFalling(GPIO_Pin)) {
			if (FlagofTankOpera == true)
				Display_RequestKey(USE_KEY_SURE);
			else {
				Display_StartLongPress(LONG_PRESS_KEY_SURE); /* 解锁 */
			}
		}
		break;
	}
	default:
		break;
	}
}

