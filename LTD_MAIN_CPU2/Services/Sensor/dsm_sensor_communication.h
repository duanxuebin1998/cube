/*
 * dsm_sensor_communication.h
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */

#ifndef SENSOR_DSM_SENSOR_COMMUNICATION_H_
/* SENSOR_DSM_SENSOR_COMMUNICATION_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_DSM_SENSOR_COMMUNICATION_H_

#include <stddef.h>
#include "sensor.h"

#define DSM_WATER_CAP "Cl"		 /* 读取测水电容数据 */

#define DSM_SENSORSTART "CD"		 /* 切换密度模式 */
#define DSM_SENSORGET "Cd"			 /* 读取传感器数据 */

#define DSM_VERSIONSTART "CV"		 /* 读取传感器版本 */
#define DSM_DENSITYPARAMETER "CM"	 /* 读取密度分析参数 */
#define DSM_NUMBER "CN"				 /* 读取震动管编号 */
#define DSM_POWER "CK"				 /* 读取供电电压 */

#define DSM_GET_FREQUENCE_START "CB" /* 切换液位模式 */
#define DSM_GET_FREQUENCE "Cb"		 /* 读取零点空气密度计频率 */

#define DSM_CMDREPLY "%\r\n"		 /* 开启命令返回 */

#define RCVBUFFLEN 32 /* 传感器通信参数：接收缓冲长度。 */


typedef struct
{
	float Sensor_Frequency; /* 频率 */
	float Sensor_Temperature; /* 温度 */
	float Density; /* 密度 */
	float Dynamic_Viscosity; /* 动力粘度 */
	float MeanSquareOf225DegreeSweepPeriod; /* 225度扫频均方根 */
	float SquareMeanOf45degreeSweepPeriod; /* 45度扫频均方根 */
	float Power_Voltage; /* 电压 */
	float level_frequency; /* 液位频率 */
	float capacitance; /* 电容 */
}DSMSENSOR_DATA;
extern DSMSENSOR_DATA dsmsensor_data;
/* int DSMSendcommand3times(uint8_t *pCommand, uint16_t commandLen); */

/**
 * @brief 开启液位模式。
 *
 * @return NO_ERROR 表示开启液位模式已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
int DSM_EnableLevelMode(void);
/**
 * @brief 将 DSM 传感器切换到密度测量模式。
 * @return 0 表示成功，非 0 表示通信或模式切换失败。
 */
int DSM_EnableDensityMode(void);
/**
 * @brief 读取 DSM 传感器供电电压。
 * @param voltage_out 电压输出指针。
 * @return NO_ERROR 表示读取成功，其他值表示通信异常。
 */
uint32_t Read_Sensor_Voltage(float *voltage_out);
/**
 * @brief 读取液位跟随频率（单次）。
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t Read_Level_Frequency(uint32_t *frequency_out);
/**
 * @brief 开启测水探针 (CL 命令)。
 *
 * @return NO_ERROR 表示开启测水探针 (CL 命令)已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
int Probe_EnableWaterSensor(void);
/**
 * @brief 发送 DSM Cd 命令并解析同一应答中的频率、密度和温度。
 *
 * @param frequency DSM 传感器振动频率输出指针，成功时写入协议定义的 Hz 浮点值。
 * @param density DSM 传感器密度输出指针，成功时写入 kg/m3 浮点值。
 * @param temp 用于返回传感器温度的输出参数，单位 ℃。
 * @return NO_ERROR 表示三项数据均已解析；SYSTEM_CALL_CONDITION_ERROR 表示输出指针无效；SENSOR_RESP_FORMAT_ERROR
 *         表示应答字段不完整；其他值为 UART6 发送或接收错误码。
 */
int DSM_Read_Frequency_Density_Temp(float *frequency,float *density, float *temp);
/**
 * @brief 读取振动管编号（CN 指令）。
 *
 * @param id_out 用于返回 CN 应答中的振动管编号。
 * @param id_out_size 编号。该值实际表示振动管 ID 输出缓冲区容量，单位字节；写入时为末尾 NUL 预留一个字节。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t Read_VibrationTube_ID(char *id_out, size_t id_out_size);
/**
 * @brief 读取电容值（Cl 指令）
 * @param[out] cap_out  输出电容值（单位与 WaterSendPack 一致，通常是 pF 或等效单位）
 * @return uint32_t 错误码（NO_ERROR 成功）
 *
 * 期望响应帧(11字节):
 *   [0]  'D' 或 'E'
 *   [1..7] 7位数字/小数点格式（sprintf: "%07.1f" 生成，实际包含小数点）
 *   [8]  BCC（对 [0..7] 计算）
 *   [9]  '\r'
 *   [10] '\n'
 */
uint32_t Read_Water_Capacitance(float *cap_out);
/**
 * @brief 读取陀螺仪角度（Ch 指令）
 * @param[out] angle_x_deg  X轴角度（A）
 * @param[out] angle_y_deg  Y轴角度（B）
 * @return uint32_t 错误码（NO_ERROR 成功）
 *
 * 期望响应示例：
 *   A+0040.1B+0097.8+
 * （实际帧尾通常还带 BCC + \r\n，你的 UART6_SendWithRetry 已做 BCC 校验）
 */
uint32_t Read_Gyro_Angle(float *angle_x_deg, float *angle_y_deg);
#endif /* SENSOR_DSM_SENSOR_COMMUNICATION_H_ */
