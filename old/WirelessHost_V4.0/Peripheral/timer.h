#ifndef __TIMER_H
#define __TIMER_H

#include "main.h"

extern TIM_HandleTypeDef htim1;

void change_out_frequence(uint32_t frequence);
void set_pwm_param(TIM_HandleTypeDef htim, uint32_t Channel, uint32_t freq, uint16_t duty);
#endif /* __TIMER_H */
