/*
 * TMC5130_Pins.h
 *
 *  Created on: Apr 29, 2021
 *      Author: Kristjan
 */

#ifndef TMC5130_TMC5130_PINS_H_
#define TMC5130_TMC5130_PINS_H_

#include "main.h"

/* Name macros as per your CubeMX labels */
#define CHIP_SELECT_1_PIN TMC1_CS_Pin /* TMC5130 第一路片选引脚别名。 */
#define CHIP_SELECT_1_PORT TMC1_CS_GPIO_Port /* TMC5130 第一路片选端口别名。 */

#define TMC1_CS_Pin TMC1_CS_Pin /* TMC5130 第一路片选引脚。 */
#define TMC1_CS_GPIO_Port TMC1_CS_GPIO_Port /* TMC5130 第一路片选 GPIO 端口。 */
#endif /* TMC5130_TMC5130_PINS_H_ */
