/*
 * sensor.h
 *
 *  Created on: Mar 19, 2025
 *      Author: 1
 */

#ifndef SENSOR_SENSOR_H_
#define SENSOR_SENSOR_H_

#include <stdint.h>
#include"dsm_sensor_communication.h"
#include"ltd_sensor_communication.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stdio.h"
#define DEBUG_DSM
#define DEBUG_UART6 0
// 模式枚举
typedef enum {
	DSM_SENSOR = 12,   // 一体机传感器
	LTD_SENSOR = 13,   // LTD传感器
} SENSOR_TYPE;

#define DSM_MAX_RETRY 1 //最大重试次数
#define DSM_BCC_DELAY 300 //校验错误延时
#define DSM_PRE_SEND_DELAY 5//重试延时
#define DSM_MIN_RESP_LEN 3//接收数据最小长度
#define DSM_CMD_TIMEOUT 1000  //接收字节间超时时间
#define RX_BUF_LEN 128

uint32_t EnableDensityMode(void);
uint32_t EnableLevelMode(void);
uint32_t DSM_Get_LevelMode_Frequence(uint32_t *frequency_out);
uint32_t DSM_Get_LevelMode_Frequence_Avg(uint32_t *frequency_out);
#endif /* SENSOR_SENSOR_H_ */
