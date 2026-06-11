#include "exit.h"
#include "display_tankopera.h"
#include "display.h"
#include "tim.h"

#define DISPLAY_KEY_QUEUE_SIZE 8U
#define DISPLAY_KEY_INDEX_COUNT 4U

volatile uint8_t button_press_counter = 0;
volatile uint8_t button_long_press_key = LONG_PRESS_KEY_NONE;
bool FlagofTankOpera = false; //罐上操作标志位
static volatile uint8_t pending_key_queue[DISPLAY_KEY_QUEUE_SIZE];
static volatile uint8_t pending_key_head = 0U;
static volatile uint8_t pending_key_tail = 0U;
static volatile uint8_t pending_key_overflow_count = 0U;
static volatile uint8_t pending_long_press_mask = 0U;
static volatile uint32_t key_last_falling_tick[DISPLAY_KEY_INDEX_COUNT];
static volatile uint8_t key_last_falling_valid[DISPLAY_KEY_INDEX_COUNT];
static volatile uint8_t long_press_release_guard_key = LONG_PRESS_KEY_NONE;
static volatile uint8_t long_press_release_guard_released = 0U;
static volatile uint32_t long_press_release_guard_tick = 0U;

static uint32_t Display_EnterCritical(void)
{
	uint32_t primask = __get_PRIMASK();
	if (__get_IPSR() == 0U) {
		__disable_irq();
	}
	return primask;
}

static void Display_ExitCritical(uint32_t primask)
{
	if ((__get_IPSR() == 0U) && (primask == 0U)) {
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

void Display_ClearPendingKeys(void)
{
	uint32_t primask = Display_EnterCritical();
	pending_key_tail = pending_key_head;
	Display_ExitCritical(primask);
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

void Display_ArmLongPressReleaseGuard(uint8_t long_press_key)
{
	uint32_t primask = Display_EnterCritical();

	pending_key_tail = pending_key_head;
	long_press_release_guard_key = long_press_key;
	long_press_release_guard_released = 0U;
	long_press_release_guard_tick = HAL_GetTick();

	Display_ExitCritical(primask);
}

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	SetScreenBright();
	//延时消抖
//    HAL_Delay(5);
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
				Display_StartLongPress(LONG_PRESS_KEY_SURE); //解锁
			}
		}
		break;
	}
	default:
		break;
	}
}

