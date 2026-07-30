#ifndef RELAY_OUTPUT_H_
/* RELAY_OUTPUT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define RELAY_OUTPUT_H_

#include <stdint.h>
#include "system_parameter.h"

/* 继电器物理输出通道索引；零基序号同时用于 GPIO 映射表和四路运行态数组。 */
typedef enum {
    /* 继电器物理输出通道的零基索引。 */
    RELAY_OUTPUT_1 = 0U, /* 第 1 路继电器输出。 */
    RELAY_OUTPUT_2, /* 第 2 路继电器输出。 */
    RELAY_OUTPUT_3, /* 第 3 路继电器输出。 */
    RELAY_OUTPUT_4, /* 第 4 路继电器输出。 */
    RELAY_OUTPUT_COUNT = RELAY_ALARM_CHANNEL_COUNT /* 继电器输出通道总数，必须等于报警通道总数。 */
} RelayOutputChannel;

/**
 * @brief 初始化继电器输出中的 RelayOutput_Init 逻辑。
 */
void RelayOutput_Init(void);

/**
 * @brief 请求清除四路继电器锁存报警。
 * @note 请求由继电器更新周期消费，不写入参数存储。
 */
void RelayOutput_RequestClearAllLatchedAlarms(void);

/**
 * @brief 根据最新测量值、故障和报警配置刷新四路继电器输出。
 *
 * 完整计算继电器输出状态；当前由 TIM4 中断直接调用。
 */
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
 * @brief 复制指定通道的继电器报警运行态快照。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @return 成功时返回指向复制指定通道的继电器报警运行态快照的指针；输入非法或未找到匹配项时返回 NULL。
 */
const volatile RelayAlarmRuntimeState *RelayOutput_GetRuntimeState(uint32_t channel);

#endif /* RELAY_OUTPUT_H_ */
