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

#define SENSOR_COMM_MAX_RETRY 10             /* 协议层默认通信重试次数 */
#define UART6_COMM_MAX_RETRY 3U             /* UART6 传感器/无线链路统一通信重试次数 */
#define SENSOR_COMM_RETRY_DELAY_MS 300       /* 两次通信尝试之间的延时 */
#define SENSOR_COMM_ERROR_RETRY_DELAY_MS 300 /* 校验/设备错误后的退避延时 */
#define SENSOR_LEVEL_MODE_SETTLE_MS 10000     /* 切换液位模式后的稳定等待时间 */

#define DSM_MAX_RETRY UART6_COMM_MAX_RETRY
#define DSM_BCC_DELAY SENSOR_COMM_ERROR_RETRY_DELAY_MS
#define DSM_PRE_SEND_DELAY SENSOR_COMM_RETRY_DELAY_MS
#define DSM_MIN_RESP_LEN 3 /* 接收数据最小长度 */
#define DSM_CMD_TIMEOUT 1000  /* 接收字节间超时时间 */
#define RX_BUF_LEN 128

/**
 * @brief 执行传感器数据中的 DetectSensorType 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t DetectSensorType(void);
/**
 * @brief 执行传感器数据中的 EnableDensityMode 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t EnableDensityMode(void);
/**
 * @brief 执行传感器数据中的 EnableLevelMode 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t EnableLevelMode(void);
/**
 * @brief 执行传感器数据中的 DSM_Get_LevelMode_Frequence 逻辑。
 *
 * @param frequency_out 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency_out);
/**
 * @brief 执行传感器数据中的 DSM_Get_LevelMode_Frequence_Avg 逻辑。
 *
 * @param frequency_out 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency_out);
/**
 * @brief 读取传感器数据中的 Read_Density 逻辑。
 *
 * @param frequency 业务参数。
 * @param density 业务参数。
 * @param temp 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Read_Density(float *frequency, float *density, float *temp);
/**
 * @brief 读取传感器数据中的 Sensor_ReadWaterCapacitance 逻辑。
 *
 * @param cap_out 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Sensor_ReadWaterCapacitance(float *cap_out);
/**
 * @brief 读取传感器数据中的 Sensor_ReadGyroAngle 逻辑。
 *
 * @param angle_x_deg 业务参数。
 * @param angle_y_deg 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Sensor_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg);
/**
 * @brief 检查传感器数据中的 Sensor_CheckAllPartParams 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Sensor_CheckAllPartParams(void);

/**
 * @brief 显示或打印传感器数据中的 WIRELESS_PrintInfo 逻辑。
 *
 * @param addr 地址参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t WIRELESS_PrintInfo(uint8_t addr);
/* * 轻量无线链路探测：只做一次最小读请求，命令切换时返回 STATE_SWITCH，不打印节点信息。 */
uint32_t WIRELESS_ProbeNode(uint8_t addr);
#endif /* SENSOR_SENSOR_H_ */
