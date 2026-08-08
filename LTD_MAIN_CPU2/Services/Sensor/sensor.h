/*
 * sensor.h
 *
 *  Created on: Mar 19, 2025
 *      Author: Duan Xuebin
 */

#ifndef SENSOR_SENSOR_H_
/* SENSOR_SENSOR_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SENSOR_H_

#include <stdint.h>
#include"dsm_sensor_communication.h"
#include"multiparam_v3_communication.h"
#include "multiparam_v4_communication.h"
#include "multiparam_v4_measurement.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stdio.h"
#define DEBUG_DSM /* 调试开关宏：调试 传感器。 */
#define DEBUG_UART6 0 /* 调试开关宏：调试 UART6。 */

#define SENSOR_COMM_MAX_RETRY 10             /* 协议层默认通信重试次数 */
#define UART6_COMM_MAX_RETRY 3U             /* UART6 传感器/无线链路统一通信重试次数 */
#define SENSOR_COMM_RETRY_DELAY_MS 300       /* 两次通信尝试之间的延时 */
#define SENSOR_COMM_ERROR_RETRY_DELAY_MS 300 /* 校验/设备错误后的退避延时 */
#define SENSOR_LEVEL_MODE_SETTLE_MS 10000     /* 切换液位模式后的稳定等待时间 */

#define DSM_MAX_RETRY UART6_COMM_MAX_RETRY /* 传感器通信参数：传感器 最大值 重试。 */
#define DSM_BCC_DELAY SENSOR_COMM_ERROR_RETRY_DELAY_MS /* 传感器通信参数：传感器 BCC 校验 延时。 */
#define DSM_PRE_SEND_DELAY SENSOR_COMM_RETRY_DELAY_MS /* 传感器通信参数：传感器 前置 发送 延时。 */
#define DSM_MIN_RESP_LEN 3 /* 接收数据最小长度 */
#define DSM_CMD_TIMEOUT 1000  /* 接收字节间超时时间 */
#define RX_BUF_LEN 128 /* 传感器串口接收缓冲区长度。 */

/**
 * @brief 自动识别传感器类型（多参数V4主动/交互、安全协议、多参数V3.0、DSM一代）
 *
 * @return uint32_t 错误码或 NO_ERROR
 */
uint32_t DetectSensorType(void);
/**
 * @brief 判断本次上电或最近一次重新探测是否已经确认传感器身份。
 * @return 1 表示当前运行态身份有效，0 表示探测尚未完成、失败或被命令切换打断。
 */
uint8_t Sensor_IsDetectionValid(void);
/**
 * @brief 获取最近一次传感器自动识别结果。
 * @return NO_ERROR 表示身份有效；其他值为最近一次探测的通信、协议或命令切换结果。
 */
uint32_t Sensor_GetDetectionResult(void);
/**
 * @brief 切换传感器到密度测量模式并等待模式生效。
 * @return 返回整机错误码；NO_ERROR 表示传感器已进入密度模式并完成稳定等待，其他值由模式切换或链路诊断返回。
 */
uint32_t EnableDensityMode(void);
/**
 * @brief 按当前传感器类型切换液位模式，并执行对应的稳定等待。
 * @return 返回整机错误码；NO_ERROR 表示当前传感器已进入液位模式并完成稳定等待，其他值透传模式切换失败。
 */
uint32_t EnableLevelMode(void);
/**
 * @brief 判断当前传感器是否提供水位电容通道。
 * @return 1 表示支持，0 表示不支持。
 */
int Sensor_SupportsWaterCapChannel(void);
/**
 * @brief 判断当前传感器是否提供姿态角通道。
 * @return 1 表示支持，0 表示不支持。
 */
int Sensor_SupportsGyroChannel(void);
/**
 * @brief 按当前传感器协议读取一次液位模式频率，不执行模式恢复。
 * @param frequency_out 用于返回整数Hz频率。
 * @return 返回整机错误码。
 */
uint32_t Sensor_ReadLevelFrequency(uint32_t *frequency_out);
/**
 * @brief 把多参数V4测水功能调整到明确目标状态。
 * @param enabled 1表示使能，0表示关闭。
 * @return 返回整机错误码；非V4传感器返回能力不支持。
 */
uint32_t Sensor_SetWaterEnabled(uint8_t enabled);
/**
 * @brief 把多参数V4磁零点功能调整到明确目标状态。
 * @param enabled 1表示使能，0表示关闭。
 * @return 返回整机错误码；非V4传感器返回能力不支持。
 */
uint32_t Sensor_SetMagneticZeroEnabled(uint8_t enabled);
/**
 * @brief 按协议层重试策略读取整数 Hz 液位频率；连续三次为 0 或超过 6500 Hz 时执行受电机状态约束的模式恢复，多轮恢复仍无效则返回 SONIC_FREQ_ABNORMAL。
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency_out);
/**
 * @brief 获取液位跟随频率的平均值
 *        （10 次采样，2s 间隔，去 2 大 2 小，取中间 6 次均值）
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency_out);
/**
 * @brief 按传感器类型读取频率、密度和温度，应用固定修正并更新调试快照。
 *
 * @param frequency 实时传感器频率输出指针，成功时写入 Hz 浮点值。
 * @param density 实时传感器密度输出指针，成功时写入经过固定修正的 kg/m3 浮点值。
 * @param temp 用于返回传感器温度的输出参数，单位 ℃。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行。
 */
uint32_t Read_Density(float *frequency, float *density, float *temp);
/**
 * @brief 读取水位传感器电容值。
 *
 * @param cap_out 用于返回水位通道电容值，单位 pF。
 * @return 返回整机错误码；NO_ERROR 表示电容值有效，PARAM_FEATURE_UNSUPPORTED 表示当前传感器不支持该通道，其他值表示通信或响应异常。
 */
uint32_t Sensor_ReadWaterCapacitance(float *cap_out);
/**
 * @brief 读取传感器陀螺仪姿态角。
 *
 * @param angle_x_deg 用于返回陀螺仪 X 轴角度的输出参数，单位度。
 * @param angle_y_deg 用于返回陀螺仪 Y 轴角度的输出参数，单位度。
 * @return 当前传感器不支持姿态通道时返回 PARAM_FEATURE_UNSUPPORTED；否则 NO_ERROR 表示双轴角度有效，通信超时会经蓝牙链路诊断映射，其他底层错误或
 *         STATE_SWITCH 原样返回。
 */
uint32_t Sensor_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg);
/**
 * @brief 逐项读取并核对传感器部件参数完整性。
 * @return 返回整机错误码；NO_ERROR 表示全部传感器部件参数均已读取并通过核对，其他值定位首个失败项。
 */
uint32_t Sensor_CheckAllPartParams(void);

#endif /* SENSOR_SENSOR_H_ */
