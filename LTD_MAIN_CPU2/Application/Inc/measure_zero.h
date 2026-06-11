/*
 * measure_zero.h
 *
 *  Created on: Mar 5, 2025
 *      Author: Duan Xuebin
 */
#ifndef INC_MEASURE_ZERO_H_
#define INC_MEASURE_ZERO_H_
extern int32_t zero_position ;
/**
 * @brief 执行回零测量中的 SearchZero 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int SearchZero(void);

#endif /* INC_MEASURE_ZERO_H_ */
