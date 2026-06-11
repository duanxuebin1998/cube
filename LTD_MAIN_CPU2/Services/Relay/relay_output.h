#ifndef RELAY_OUTPUT_H_
#define RELAY_OUTPUT_H_

#include <stdint.h>
#include "system_parameter.h"

typedef enum {
    RELAY_OUTPUT_1 = 0U,
    RELAY_OUTPUT_2,
    RELAY_OUTPUT_3,
    RELAY_OUTPUT_4,
    RELAY_OUTPUT_COUNT = RELAY_ALARM_CHANNEL_COUNT
} RelayOutputChannel;

/**
 * @brief 初始化继电器输出中的 RelayOutput_Init 逻辑。
 */
void RelayOutput_Init(void);

/* IRQ-safe request hook. It only sets a flag and does not read parameters or GPIO. */
void RelayOutput_RequestUpdate(void);

/* Run pending relay calculation in the main context. */
void RelayOutput_ProcessPending(void);

/* Full relay calculation. Keep this in the main context unless a caller owns the timing risk. */
void RelayOutput_Update(void);

/**
 * @brief 直接设置指定继电器输出通道的物理输出状态。
 * @param channel 继电器通道。
 * @param active 非 0 表示吸合，0 表示释放。
 */
void RelayOutput_SetChannel(RelayOutputChannel channel, uint8_t active);

/**
 * @brief 获取四路继电器当前输出状态位图。
 * @return bit0~bit3 分别表示 1~4 路继电器吸合状态。
 */
uint8_t RelayOutput_GetStateMask(void);

/**
 * @brief 执行继电器输出中的 RelayOutput_GetRuntimeState 逻辑。
 *
 * @param channel 业务参数。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
const volatile RelayAlarmRuntimeState *RelayOutput_GetRuntimeState(uint32_t channel);

#endif /* RELAY_OUTPUT_H_ */
