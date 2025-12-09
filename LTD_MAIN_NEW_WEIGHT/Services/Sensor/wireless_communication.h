/*
 * wireless_communication.h
 *
 *  Created on: 2025年12月8日
 *      Author: admin
 */

#ifndef SENSOR_WIRELESS_COMMUNICATION_H_
#define SENSOR_WIRELESS_COMMUNICATION_H_


/*
 * dsm_v2.h
 *	与LTD传感器通信
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */
#ifndef DSM_V2_H
#define DSM_V2_H

#include <stdint.h>
#include "main.h"
#include "sensor.h"
/*****************无线透传抓包协议说明***********
一、协议包说明：
通讯协议包长为8个字节，
1.首字节为地址,
2.第二字节为功能码，
	W为写入参数
	R为读取参数
3.第3-6个字节为数据位，
	1）主机到从机：读取时无效，写入参数时为参数值
	2）从机到主机：读取时返回数据，写入时返回写入值
4.第七个字节为参数码，
	1）主机到从机：读取和写入参数时，需要操作的参数
	2）从机到主机：用于指示是否成功
	成功时，返回参数码；失败时返回FF
5.第8个字节为校验码，为以上7个字节的和

二、具体参数
功能码	参数码	参数		读写类型	数据类型

	 --------------------------基本信息-------------------------------------
		R		00			软件版本		R				单浮点
		R		01			电压		R				单浮点
		....
*/
// 模式枚举
typedef enum {
    DSM_V2_MODE_LEVEL   = 'R',   // 液位模式
    DSM_V2_MODE_DENSITY = 'W',   // 密度模式
} dsm_v2_mode_t;

// === 对外 API ===

// 通用读取
uint32_t WIRELESS_Read_FloatParam(uint8_t addr,uint8_t param, float *out_value);
uint32_t WIRELESS_Read_IntParam  (uint8_t addr,uint8_t param, int32_t *out_value);

uint32_t WIRELESS_Read_SoftwareVersion(uint8_t addr,float *v) ;// 软件版本
uint32_t WIRELESS_Read_Voltage(uint8_t addr,float *v) ;// 电压
#endif /* SENSOR_WIRELESS_COMMUNICATION_H_ */
