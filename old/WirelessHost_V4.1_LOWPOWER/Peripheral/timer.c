#include "timer.h"

uint32_t current_frequency = 160000;
void auto_change_out_frequence(void)
{
	uint8_t fre;
	if(current_frequency<4000000)
	{
		current_frequency=current_frequency+1000;
	}
	else
	{
		current_frequency = 10000;
	}
	change_out_frequence(current_frequency);
	fre =  (uint8_t)(current_frequency/1000);
	HAL_UART_Transmit_DMA(&huart2, &fre, 1) ;
}

void change_out_frequence(uint32_t frequence)
{
	set_pwm_param(htim1, TIM_CHANNEL_1, frequence, 50);
	set_pwm_param(htim1, TIM_CHANNEL_4, frequence, 50);
}

//通用接口，主频72MHz，预分频值为1-1，设置PWM的脉冲频率freq(0.16-10kHz)、占空比参数 pulse (0-100)
void set_pwm_param(TIM_HandleTypeDef htim, uint32_t Channel, uint32_t freq, uint16_t duty)
{
    uint16_t prescaler = 1-1;
    uint64_t tim_clk_freq = 16000000;
    //计算PWM频率，所对应的自动重装载值   ---> ARR = 主频 / (预分频+1) / 预期PWM频率(Hz) - 1
    float pwm_freq_arr  = (tim_clk_freq * 1.0) / (prescaler+1) / freq * 1.0 - 1;
    //计算PWM占空比，所对应比较寄存器的值 ---> CCR = 预期占空比 * (自动重装载值+1)
    //占空比则由捕获/比较寄存器（TIMx_CRx）寄存器决定。占空比:duty = Pluse / (ARR+1)
    float pwm_duty_pulse = duty * 1.0 / 100 * (pwm_freq_arr + 1);

    //配置PSC预分频值
    __HAL_TIM_SET_PRESCALER(&htim, prescaler);
    //配置PWM频率 ARR
    __HAL_TIM_SetAutoreload(&htim, (uint16_t)pwm_freq_arr);
    //配置PWM占空比
    __HAL_TIM_SetCompare(&htim, Channel, (uint16_t)pwm_duty_pulse);
//    printf("pwm_freq_arr:%.2f\r\n", pwm_freq_arr);
//    printf("pwm_duty_pulse:%.2f\r\n", pwm_duty_pulse);
}
