#include "exit.h"
#include "display_tankopera.h"
#include "display.h"
#include "tim.h"

/* 显示按键事件队列容量 8；队列只保存已消抖事件，满队列时必须按既定策略处理，不能覆盖尚未消费事件。 */
#define DISPLAY_KEY_QUEUE_SIZE 8U
/* 参与按键扫描和状态数组索引的物理按键数量 4；与返回、上、下、确认四个键一一对应。 */
#define DISPLAY_KEY_INDEX_COUNT 4U

volatile uint8_t button_press_counter = 0; /* TIM1 周期中断累计的长按持续计数；达到 REQUIRED_PRESS_COUNT 时触发一次长按动作，按键释放、切换或重新开始计时时清零。 */
volatile uint8_t button_long_press_key = LONG_PRESS_KEY_NONE; /* 当前由 TIM1 计时识别的长按按键编码；外部中断锁存按键，定时器和前台动作流程消费，LONG_PRESS_KEY_NONE 表示未跟踪长按。 */
bool FlagofTankOpera = false; /* 罐上操作标志位 */
static volatile uint8_t pending_key_queue[DISPLAY_KEY_QUEUE_SIZE]; /* 外部中断生产、显示前台消费的已消抖短按环形队列；每个元素保存一条 USE_KEY_* 按键位掩码。 */
static volatile uint8_t pending_key_head = 0U; /* 短按环形队列的下一写入索引；外部中断入队成功后推进，追上 tail 表示队列已满。 */
static volatile uint8_t pending_key_tail = 0U; /* 短按环形队列的下一读取索引；前台取走事件或显式清队列时推进到相应位置。 */
static volatile uint8_t pending_key_overflow_count = 0U; /* 短按环形队列已满而丢弃新事件的累计次数；只做按键路径诊断，不改变仍在队列中的历史事件。 */
static volatile uint8_t pending_long_press_mask = 0U; /* 确认键和返回键待处理长按动作的位掩码；定时器路径置位，前台按固定优先级原子取走并清除相应位。 */
static volatile uint32_t key_last_falling_tick[DISPLAY_KEY_INDEX_COUNT]; /* 返回、上、下、确认四个物理按键最近一次已接受下降沿的 HAL 毫秒节拍；按键索引与 GPIO 映射保持一致。 */
static volatile uint8_t key_last_falling_valid[DISPLAY_KEY_INDEX_COUNT]; /* 四个按键的下降沿消抖节拍是否已经建立；首次按下不与零时刻比较，建立后才应用 DISPLAY_KEY_DEBOUNCE_MS。 */
static volatile uint8_t long_press_release_guard_key = LONG_PRESS_KEY_NONE; /* 长按动作完成后仍受释放保护的按键编码；保护期间丢弃新的短按和长按起始，避免释放沿重复触发页面动作。 */
static volatile uint8_t long_press_release_guard_released = 0U; /* 受保护长按键已经检测到物理释放的标志；释放后重新计时，达到保护间隔才允许清除 long_press_release_guard_key。 */
static volatile uint32_t long_press_release_guard_tick = 0U; /* 长按释放保护的 HAL 毫秒节拍；启用保护时记录初值，检测到释放时重置，并据此等待释放后的防抖间隔。 */

/**
 * @brief 进入显示按键队列临界区并返回原中断状态。
 * @return 返回进入临界区前的 PRIMASK 中断屏蔽状态，退出临界区时必须原样恢复。
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
 * @brief 恢复进入显示按键队列临界区前的中断状态。
 *
 * @param primask 进入临界区前保存的中断屏蔽状态。
 */
static void Display_ExitCritical(uint32_t primask)
{
	if ((__get_IPSR() == 0U) && (primask == 0U)) {
		__enable_irq();
	}
}

/**
 * @brief 计算按键环形队列的下一个索引。
 *
 * @param index 索引值。
 * @return 返回按键环形队列的后继下标；末尾下标自动回绕到 0。
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
 * @brief 把 GPIO 按键引脚转换为内部按键索引。
 *
 * @param GPIO_Pin 待映射的按键引脚；KEY_BACK、KEY_UP、KEY_DOWN、KEY_SURE 依次映射为索引 0、1、2、3。
 * @param index 按键索引输出地址；映射成功时写入 0～3，输入引脚不属于四个界面按键时不写入。
 * @return true 表示 GPIO_Pin 是返回、上、下或确认键，并已写入索引 0～3；false 表示引脚不属于四个界面按键，index 保持未写入。
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
 * @brief 对按键下降沿进行消抖并登记短按事件。
 *
 * @param GPIO_Pin 本次下降沿对应的按键引脚；四个界面按键分别映射到独立消抖时间槽，其他 EXTI 引脚按无需处理返回。
 * @return true 表示非界面引脚无需拦截，或界面按键下降沿已通过消抖并登记；false 表示同一按键在 DISPLAY_KEY_DEBOUNCE_MS 内重复下降，事件被抑制。
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
 * @brief 读取当前达到长按门限的按键并消费对应事件。
 *
 * @param long_press_key 长按按键。
 *
 * @return 返回已达到长按门限按键的当前 GPIO 电平并消费事件；没有待处理长按时返回 GPIO_PIN_SET。
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
 * @brief 记录长按起始按键和计时基准。
 *
 * @param long_press_key 长按按键。
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
 * @brief 将有效按键加入环形待处理队列；队列满时丢弃新按键并累计溢出次数。
 *
 * @param keypress 本次待分发的按键位掩码。
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
 * @brief 清空待处理按键队列和长按状态。
 */
void Display_ClearPendingKeys(void)
{
	uint32_t primask = Display_EnterCritical();
	pending_key_tail = pending_key_head;
	Display_ExitCritical(primask);
}

/**
 * @brief 为指定按键登记一次长按动作请求。
 *
 * @param long_press_key 长按按键。
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
 * @brief 从按键环形队列取出下一条待处理按键。
 * @return 返回下一条待处理按键编码；队列为空时返回 0。
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
 * @brief 原子取出并清除待处理长按动作。
 * @return 返回并清除一条待处理长按按键编码；没有待处理动作时返回 0。
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
 * @brief 启用长按释放保护，避免释放沿被误识别为短按。
 *
 * @param long_press_key 长按按键。
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
 * @brief 按按键释放状态维护长按后的防重复触发门限。
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
 * @brief 在按键外部中断中完成去抖、短按排队和长按计时启动，不直接执行页面逻辑。
 *
 * @param GPIO_Pin HAL EXTI 回调上报的引脚掩码；仅 KEY_BACK_Pin、KEY_UP_Pin、KEY_DOWN_Pin 和 KEY_SURE_Pin 进入按键处理，其余引脚直接忽略。
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

