#ifndef __TIMER_H
#define __TIMER_H

#include "main.h"


extern TIM_HandleTypeDef htim1;
/**
 * @brief 执行定时器辅助中的 auto_change_out_frequence 逻辑。
 */
void auto_change_out_frequence(void);
/**
 * @brief 执行定时器辅助中的 change_out_frequence 逻辑。
 *
 * @param frequence 业务参数。
 */
void change_out_frequence(uint32_t frequence);
/**
 * @brief 写入或设置定时器辅助中的 set_pwm_param 逻辑。
 *
 * @param htim 业务参数。
 * @param Channel 业务参数。
 * @param freq 业务参数。
 * @param duty 业务参数。
 */
void set_pwm_param(TIM_HandleTypeDef htim, uint32_t Channel, uint32_t freq, uint16_t duty);
#endif /* __TIMER_H */
