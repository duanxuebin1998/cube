/*
 * @FilePath     : \undefinedd:\CUBE\LTD_MAIN_CPU2\Application\Inc\measure_water_level.h
 * @Description  : 
 * @Author       : Aubon
 * @Date         : 2025-12-19 15:55:33
 * @LastEditors  : Duan
 * @LastEditTime : 2025-12-26 09:16:22
 * Copyright 2025 Aubon, All Rights Reserved. 
 * 2025-12-19 15:55:33
 */
/*
 * measure_water_level.h
 *
 *  Created on: 2025年12月19日
 *      Author: admin
 */

#ifndef INC_MEASURE_WATER_LEVEL_H_
#define INC_MEASURE_WATER_LEVEL_H_

#include <stdint.h>
#include "main.h"

#define NORMAL 0
#define WATER  1

uint32_t read_zero_capacitance(void);//获取零点电容值
uint32_t read_oil_capacitance(void);//获取油区电容值
uint32_t check_water_status(uint8_t *water_state);
uint32_t SearchWaterLevel(void);
uint32_t FollowWaterLevel(void);

#endif /* INC_MEASURE_WATER_LEVEL_H_ */
