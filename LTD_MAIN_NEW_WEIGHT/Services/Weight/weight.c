/*
 * weight.c
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#include "main.h"
#include "weight.h"
#include "stdio.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "AS5145.h"

int16_t weight; //称重值
Weight_ParamentTypeDef weight_parament;

void get_stable_weight(void);
Weight_StateTypeDef determine_weight_status(void);

void get_stable_weight(void)
{
	HAL_Delay(5000);
	weight_parament.stable_weight = abs(weight);
	printf("稳定的称重\t%d\r\n", weight_parament.stable_weight);
}

Weight_StateTypeDef determine_weight_status(void)
{
	weight_parament.current_weight = weight;
	printf("当前称重值\t%d\t", weight);
	if (weight_parament.current_weight > weight_parament.stable_weight*(100+ZERO_WEIGHT_THRESHOLD)/100)
	{
		printf("称重值过大\r\n");
		return ZERO;
	}
	else if (weight_parament.current_weight < weight_parament.stable_weight*(100-BOTTOM_WEIGHT_THRESHOLD)/100)
	{
		printf("称重值过小\r\n");
		return BOTTOM;
	}
//	else if(fabs(weight_parament.current_weight-weight_parament.stable_weight)>IMPACT_WEIGHT_THRESHOLD)
//	{
//		printf("稳定的称重\t%d\r\n",weight_parament.stable_weight);
//		printf("称重值波动超限\r\n");
//		return IMPACT;
//	}
	else
	{
		printf("称重值正常\r\n");
		return NORMAL;
	}
}

