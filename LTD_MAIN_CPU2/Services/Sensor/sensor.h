/*
 * sensor.h
 *
 *  Created on: Mar 19, 2025
 *      Author: Duan Xuebin
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

#define DSM_MAX_RETRY 1 //最大重试次数
#define DSM_BCC_DELAY 300 //校验错误延时
#define DSM_PRE_SEND_DELAY 5//重试延时
#define DSM_MIN_RESP_LEN 3//接收数据最小长度
#define DSM_CMD_TIMEOUT 1000  //接收字节间超时时间
#define RX_BUF_LEN 128

uint32_t DetectSensorType(void);
uint32_t EnableDensityMode(void);
uint32_t EnableLevelMode(void);
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency_out);
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency_out);
uint32_t Read_Density(float *frequency, float *density, float *temp);

uint32_t WIRELESS_PrintInfo(uint8_t addr);
#endif /* SENSOR_SENSOR_H_ */
