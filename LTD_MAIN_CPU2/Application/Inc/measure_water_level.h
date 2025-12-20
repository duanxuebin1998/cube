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

#ifndef MEASUREMENT_WATER_DOWN_FAIL
#define MEASUREMENT_WATER_DOWN_FAIL  (0xE201) // 示例：请替换为你的工程规范
#endif

#endif /* INC_MEASURE_WATER_LEVEL_H_ */
