/*
 * measure_zero.h
 *
 *  Created on: Mar 5, 2025
 *      Author: Duan Xuebin
 */
#ifndef INC_MEASURE_ZERO_H_
/* INC_MEASURE_ZERO_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MEASURE_ZERO_H_
extern int32_t zero_position ;
/**
 * @brief 在有限重试次数内依次执行零点粗找和精找，成功后重建编码器及卷筒位置基准。
 *
 * 初始扭力过大时先下行脱离零点；粗找和精找分别最多尝试 ZERO_SEARCH_RETRY_MAX 次，并记录失败重试、恢复成功及驱动恢复错误。
 * 最终校验零点偏差后，保存编码器零点、重置电机卷筒参考，按配置切回编码轮位置源，再下行脱离并在传感器支持时保存水位零电容和陀螺仪零基准。
 *
 * @return NO_ERROR 表示粗找、精找、零点保存及后续脱离动作全部完成；STATE_SWITCH
 *         表示被新命令正常打断；其他值为运动、驱动、丢步、零点范围、位置保存或可选传感器基准读取错误码。
 */
int SearchZero(void);

#endif /* INC_MEASURE_ZERO_H_ */
