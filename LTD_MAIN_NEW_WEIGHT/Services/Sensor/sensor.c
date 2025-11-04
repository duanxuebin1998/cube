/*
 * @FilePath     : \KEILe:\03CodeRepository\DSM_MCB\HARDWARE\SENSOR\sensor.c
 * @Description  : 传感器通信相关函数
 * @Author       : Aubon
 * @Date         : 2024-02-23 10:20:27
 * @LastEditors  : Duan
 * @LastEditTime : 2024-11-15 15:36:58
 * Copyright 2024 Aubon, All Rights Reserved.
 * 2024-02-23 10:20:27
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "sensor.h"
#include "system_parameter.h"
#include "main.h"
#include "stdio.h"
#define DEBUG_UART6 0

#define DSM_CMD_TIMEOUT 1000  //接收字节间超时时间
#define RX_BUF_LEN 128
//uint8_t rx_buf[RX_BUF_LEN];
DSMSENSOR_DATA dsmsensor_data;

#define DSM_MAX_RETRY 1 //最大重试次数
#define DSM_BCC_DELAY 300 //校验错误延时
#define DSM_PRE_SEND_DELAY 5//重试延时
#define DSM_MIN_RESP_LEN 3//接收数据最小长度
#define DEBUG_DSM
char DSMCommand[RCVBUFFLEN];
char DSMRcvBuffer[RCVBUFFLEN];
int DSMRcvLen;

static char CalculationBCC_DSM(char command[], int count);

// 错误码关键字表
const char *error_codes[] = { "A+111.11B+111.11", // 超声无谐振
		"A+222.22B+222.22", // 电源电压异常
		"A+333.33B+333.33", // 陀螺仪IIC通讯超时
		"A+444.44B+444.44", // 陀螺仪角度异常
		"A+555.55B+555.55", // CPU1自检错误
		"A+888.88B+888.88", // 与CPU0通讯超时
		"A+999.99B+999.99"  // 与CPU0通讯校验错误
		};
#define ERROR_CODES_COUNT (sizeof(error_codes)/sizeof(error_codes[0]))

// 检查返回是否为错误码
int IsErrorResponse(const char *resp) {
	for (int i = 0; i < ERROR_CODES_COUNT; i++) {
		if (strstr(resp, error_codes[i]) != NULL) {
			return 1; // 是错误码
		}
	}
	return 0; // 正常
}

// 串口发送并接收（带调试打印）
int UART6_SendCommand(const char *cmd, char *response, uint16_t maxLen, uint32_t timeout) {
	char bcc;
//	memset(rx_buf, 0, RX_BUF_LEN);
	memset(response, 0, maxLen);
	uint16_t recvLen = 0;
#ifdef DEBUG_UART6
	printf("[UART6] Send: %s\n", cmd);
#endif

	// 发送
	if (HAL_UART_Transmit(&huart6, (uint8_t*) cmd, strlen(cmd), 100) != HAL_OK) {
#ifdef DEBUG_UART6
		printf("[UART6] Transmit failed!\n");
#endif
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	}

	/* 接收阶段（带超时机制） */
	uint32_t startTick = HAL_GetTick();
	while (HAL_GetTick() - startTick < DSM_CMD_TIMEOUT) {
		uint8_t byte;

		if (HAL_UART_Receive(&huart6, &byte, 1, 1) == HAL_OK) {
			if (recvLen < RX_BUF_LEN - 1) {
				response[recvLen++] = byte;
				// 根据协议判断帧结束符（示例为 0x0D 换行）
				if (byte == 0x0A)
					break;
			}
		}
	}
	/* 响应校验 */
	if (recvLen == 0) {
		printf("[UART6] 通信失败：无数据\r\n");
		return SENSOR_COMM_TIMEOUT;
	}
#ifdef DEBUG_UART6
	printf("[UART6] 接收成功，共 %d 字节\r\n", recvLen);
	printf("[UART6] 响应字符串: %s\r\n", response);
	printf("[UART6] 响应HEX: ");
	for (int i = 0; i < recvLen; i++) {
		printf("%02X ", (uint8_t) response[i]);
	}
	printf("\r\n");
#endif
	//校验BCC
	bcc = CalculationBCC_DSM(response, (recvLen - 3));
	if (bcc == response[recvLen - 3]) {
#ifdef DEBUG_UART6
		printf("DSM:**rcv:BCC校验通过!\r\n");
#endif
		return NO_ERROR;
	} else {
#ifdef DEBUG_UART6
		printf("DSM:**rcv:BCC校验失败!\r\n");
#endif
		return SENSOR_BCC_ERROR; //校验失败
	}
	return NO_ERROR;
}
// 发送指令，带3次重试机制
int UART6_SendWithRetry(const char *cmd, char *response, uint16_t maxLen, uint32_t timeout) {
	uint32_t ret;
	for (int i = 0; i < 3; i++) {
		ret = UART6_SendCommand(cmd, response, maxLen, timeout);
		if (ret == 0) {
			if (!IsErrorResponse(response)) {
				return NO_ERROR; // 成功且不是错误码
			} else {
				printf("[UART6] 接收到错误码，重试 %d/3\r\n", i + 1);
			}
		} else {
			printf("[UART6] 通信失败，重试 %d/3\r\n", i + 1);
		}
	}
	CHECK_ERROR(ret); // 重试3次失败
	return ret;
}

// 读取传感器电压
float Read_Sensor_Voltage(void) {
	uint32_t ret;
	char resp[RX_BUF_LEN];
	ret = UART6_SendWithRetry("CK\r\n", resp, RX_BUF_LEN, 500);
	if (ret == NO_ERROR) {
		// 响应格式: E06.6379V
		printf("[UART6] 接收成功: %s\r\n", resp);
		if ((resp[0] == 'E') || (resp[0] == 'e')) {
//			printf("电压数据: %s\r\n", atof(resp + 1));
			return atof(resp + 1);
		} else {
			printf("无效电压响应: %x\r\n", resp[0]);
		}
	}
	return -1.0f; // 错误
}
// 开启测水探针 (CL 命令)
int Probe_EnableWaterSensor(void) {
	char resp[RX_BUF_LEN];

	// 发送命令 "CL\r\n" 并带 3 次重试
	if (UART6_SendWithRetry("CL\r\n", resp, RX_BUF_LEN, 500) == 0) {
		printf("[Probe] 开启测水探针响应: %s\r\n", resp);

		// 协议约定：如果返回包含 "OK" 或其他成功标识，就认为成功
		if (strstr(resp, "%") != NULL || strstr(resp, "%") != NULL) {
			printf("[Probe] 测水探针开启成功！\r\n");
			return 0;
		} else {
			printf("[Probe] 无效响应: %s\r\n", resp);
			return -2; // 响应格式不对
		}
	} else {
		printf("[Probe] 测水探针开启失败！\r\n");
		return -1; // 通信失败或错误码
	}
}
// 开启测水探针 (CL 命令)
int EnableLevelMode(void) {
	char resp[RX_BUF_LEN];

	// 发送命令 "CL\r\n" 并带 3 次重试
	if (UART6_SendWithRetry("CB\r\n", resp, RX_BUF_LEN, 500) == 0) {
		printf("[液位模式] 开启液位模式响应: %s\r\n", resp);

		// 协议约定：如果返回包含 "OK" 或其他成功标识，就认为成功
		if (strstr(resp, "%") != NULL || strstr(resp, "%") != NULL) {
			printf("[液位模式] 开启成功！\r\n");
			return NO_ERROR;
		} else {
			printf("[液位模式] 无效响应: %s\r\n", resp);
			return NO_ERROR; // 响应格式不对
		}
	} else {
		printf("[液位模式] 开启失败！\r\n");
		return -1; // 通信失败或错误码
	}
}

// 读取液位跟随频率
float Read_Level_Frequency(void) {
	char resp[RX_BUF_LEN];
	if (UART6_SendWithRetry("Cb\r\n", resp, RX_BUF_LEN, 500) == 0) {
		// 响应格式: E06.6379V
		printf("[UART6] 接收成功: %s\r\n", resp);
		if (1) {
//			printf("电压数据: %s\r\n", atof(resp + 1));
			return atof(resp + 1);
		} else {
			printf("无效电压响应: %x\r\n", resp[0]);
		}
	}
	return -1.0f; // 错误
}
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency) {
	if (*frequency < 0) {
		return SENSOR_COMM_TIMEOUT;
	}
	*frequency = (uint32_t) Read_Level_Frequency();
	printf("液位频率: %ld Hz\r\n", *frequency);
	return NO_ERROR;
}
/**
 * @brief 获取液位跟随频率的平均值
 *        共采样10次，每次间隔2秒，去掉两个最大值和两个最小值，
 *        对中间6次结果求平均。
 * @param frequency  输出平均频率值 (Hz)
 * @return uint32_t  错误码，NO_ERROR 表示成功
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency) {
	float values[10];
	float sum = 0.0f;

	// 采集10次数据
	for (int i = 0; i < 10; i++) {
		values[i] = Read_Level_Frequency();
		printf("第%d次液位频率: %.2f Hz\r\n", i + 1, values[i]);
		HAL_Delay(2000); // 2秒间隔
	}

	// 排序 (简单冒泡排序)
	for (int i = 0; i < 10 - 1; i++) {
		for (int j = 0; j < 10 - i - 1; j++) {
			if (values[j] > values[j + 1]) {
				float tmp = values[j];
				values[j] = values[j + 1];
				values[j + 1] = tmp;
			}
		}
	}

	// 去掉两个最大和两个最小值，取中间6个
	for (int i = 2; i < 8; i++) {
		sum += values[i];
	}

	float avg = sum / 6.0f;
	*frequency = (uint32_t) avg;

	printf("液位频率平均值(去掉极值): %ld Hz\r\n", *frequency);

	return NO_ERROR;
}

// 读取密度、温度
int Read_Density_Temp(float *density, float *viscosity, float *temp) {
	char resp[RX_BUF_LEN];
	if (UART6_SendWithRetry("Cd\r\n", resp, RX_BUF_LEN, 500) == 0) {
		// 格式: F+0000.0V+000.00T+19.570P
		if ((resp[0] == 'E') || (resp[0] == 'F')) {
			char *pT = strchr(resp, 'T');
			char *pP = strchr(resp, 'V');

			if (pT && pP) {
				*density = atof(resp + 1);
				*viscosity = atof(pP + 1);
				*temp = atof(pT + 1);
				return 0;
			}
		}
	}
	return -1;
}
// 读取超声小管的密度、温度
int Read_Density_SIL(float *density, float *viscosity, float *temp) {
	char resp[RX_BUF_LEN];
	if (UART6_SendWithRetry("Cf\r\n", resp, RX_BUF_LEN, 500) == 0) {
		// 格式: F+0000.0V+000.00T+19.570P
		if ((resp[0] == 'E') || (resp[0] == 'F')) {
			char *pT = strchr(resp, 'T');
			char *pP = strchr(resp, 'D');

			if (pT && pP) {
				*density = atof(pP + 1);
				*viscosity = atof(resp + 1);
				*temp = atof(pT + 1);
				return 0;
			}
		}
	}
	return -1;
}
void Sensor_Test(void) {
	float voltage;
	float frequency;
	float density, viscosity, temp;

	printf("=== 开始传感器测试 ===\r\n");

//	// 读取电压
//	voltage = Read_Sensor_Voltage();
//	if (voltage > 0) {
//		printf("传感器电压: %.4f V\r\n", voltage);
//	} else {
//		printf("读取电压失败！\r\n");
//	}
//	// 读取电压
//	frequency = Read_Level_Frequency();
//	if (Read_Level_Frequency > 0) {
//		printf("液位跟随频率: %.1f V\r\n", frequency);
//	} else {
//		printf("读取电压失败！\r\n");
//	}
	// 读取密度、温度
	if (Read_Density_Temp(&density, &viscosity, &temp) == 0) {
		printf("密度: %.3f  粘度: %.3f  温度: %.3f ℃\r\n", density, viscosity, temp);
	} else {
		printf("读取密度/温度失败！\r\n");
	}

	printf("=== 测试结束 ===\r\n");
}
/**
 * @func: CalculationBCC_Am4096（u8 *command,int count）
 * @description: BCC校验
 * @param {char} command	待校验数组：编码值缓存区
 * @param {int} count		数组长度：编码值长度：5 ————两位圈数，两位角度，一位校验
 * @return {char}bcc		返回BCC校验码
 */
static char CalculationBCC_DSM(char command[], int count) {
	char i, bcc;
	bcc = command[0];
	for (i = 1; i < count - 1; i++) {
		bcc = bcc ^ command[i + 1];
	}
	return bcc;
}

/**
 * @func: int DSMSendcommand3times(char *command, unsigned int commandlen)
 * @description: 向传感器连续发送三次指令
 * @param {char} *command
 * @param {unsigned int} commandlen
 * @return { }
 */
int DSMSendcommand3times(uint8_t *pCommand, uint16_t commandLen) {
	int retFlag = NO_ERROR;
	HAL_StatusTypeDef halStatus;
	uint8_t bcc = 0;

	for (int attempt = 0; attempt < DSM_MAX_RETRY; attempt++) {
		// 错误退避策略
		if (retFlag == 0) {
			HAL_Delay(DSM_BCC_DELAY); //3-1
		}

		// 预发送延时
		HAL_Delay(DSM_PRE_SEND_DELAY);

		// 清空接收缓冲区（DMA版本需用HAL_UART_DMAStop）
		memset(DSMRcvBuffer, 0, sizeof(DSMRcvBuffer));
		DSMRcvLen = 0;

		/* 发送阶段 */
		halStatus = HAL_UART_Transmit(&huart6, pCommand, commandLen,
		DSM_CMD_TIMEOUT);
		if (halStatus != HAL_OK) {
#ifdef DEBUG_DSM
			printf("DSM TX Error: %d\r\n", halStatus);
#endif
			retFlag = 0;
			continue;
		}

#ifdef DEBUG_DSM
		printf("DSM:**snd:");
		printf("%s ", pCommand);
		printf("\r\n");
#endif
		/* 接收阶段（带超时机制） */
		uint32_t startTick = HAL_GetTick();
		while (HAL_GetTick() - startTick < DSM_CMD_TIMEOUT) {
			uint8_t byte;
			halStatus = HAL_UART_Receive(&huart6, &byte, 1, 1); // 1ms单字节接收

			if (halStatus == HAL_OK) {
				if (DSMRcvLen < sizeof(DSMRcvBuffer) - 1) {
					DSMRcvBuffer[DSMRcvLen++] = byte;
					// 根据协议判断帧结束符（示例为0x0D）
					if (byte == 0x0A)
						break;
				}
			}
		}
#ifdef DEBUG_DSM
		printf("DSM:**rcv:");
		printf("%s ", DSMRcvBuffer);
		printf("\r\n");
#endif

		/* 响应校验 */
		if (DSMRcvLen == 0) {
			printf("communication error\r\n");
			retFlag = SENSOR_COMM_TIMEOUT; //通信超时
			continue;
		}
		printf("communication success:%s\r\n", DSMRcvBuffer + 1);
		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
		//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
		// 改进校验：长度+头部+BCC校验
		if ((DSMRcvLen >= DSM_MIN_RESP_LEN) && (bcc == DSMRcvBuffer[DSMRcvLen - 3])) {
			return NO_ERROR; // 全校验通过
		} else {
			retFlag = SENSOR_BCC_ERROR; //校验失败
		}
	}

	return retFlag;
}
//int DSMSendcommand3times(char *command, unsigned int commandlen)
//{
//	int i;
//	int j;
//	int retflag = 0;
//	for (i = 0; i < 3; ++i)
//	{
//		if (ERROE_DSM_BCC_3_1 == retflag)
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(5); // 每次发送之前延时5ms,否则尝试出错可能成为连续的错误
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(command, commandlen);
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%s", command);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%s", DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包\r\n");
//			retflag = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (strncmp(DSMRcvBuffer, DSM_CMDREPLY, strlen(DSM_CMDREPLY)) == 0)
//		{
//#ifdef DEBUG_DSM
//			printf("DSM:**rcv:over!\r\n");
//#endif
//			// 判断一下数据
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误L:%d串:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			retflag = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return retflag;
//}
///**********************************************************************************************
//**函数名称：	water_version(void)
//**函数功能：	发送读取传感器编号功能，根据编号判断测水方式
//**参数:			command：发送的命令 commandlen：命令长度
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//
//**********************************************************************************************/
//int water_version(char *command, unsigned int commandlen)
//{
//	int i;
//	int j;
//	int retflag = 0;
//	u8 error_flag = 0;
//	for (i = 0; i < 3; ++i)
//	{
//		if (ERROE_DSM_BCC_3_1 == retflag)
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(5); // 每次发送之前延时10ms,否则尝试出错可能成为连续的错误
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(command, commandlen);
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%s", command);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//		printf("DSM:**rcv:%s\r\n", DSMRcvBuffer);
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包\r\n");
//			retflag = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (((DSMRcvBuffer[0] == 'B') && (DSMRcvBuffer[1] == 'C') && (DSMRcvBuffer[2] == 'B')) || ((DSMRcvBuffer[0] == '0') && DSMRcvBuffer[1] == '2'))
//		{
//			printf("LEMES测水方式\r\n");
//			Water_flag = 0;
//			error_flag = 1;
//			return NO_ERROR;
//		}
//		else if (((DSMRcvBuffer[0] == '0') && (DSMRcvBuffer[1] == '3')) || ((DSMRcvBuffer[0] == '0') && (DSMRcvBuffer[1] == '5'))) // 兼容陀螺仪驱动 老版本03  新版本05
//		{
//			printf("电阻+运放式测水方式\r\n");
//			Water_flag = 1;
//			error_flag = 1;
//			return NO_ERROR;
//		}
//		else if ((DSMRcvBuffer[0] == '0') && (DSMRcvBuffer[1] == '4'))
//		{
//			printf("超声波+电阻式测水\r\n");
//			Water_flag = 1;
//			error_flag = 1;
//			Sonic_flag = 1;
//			return NO_ERROR;
//		}
//		else if ((DSMRcvBuffer[0] == '0') && (DSMRcvBuffer[1] == '6')) /*2022.08.11段 新版本06*/
//		{
//			printf("大管SIL超声波+电阻式测水\r\n");
//			Water_flag = 1;
//			error_flag = 1;
//			Sonic_flag = 2;
//			return NO_ERROR;
//		}
//		else if (((DSMRcvBuffer[0] == '0') || (DSMRcvBuffer[0] == 'E')) && (DSMRcvBuffer[1] == '7')) /*2022.11.17段 新版本07*/
//		{
//			printf("小管SIL超声波+电阻式测水\r\n");
//			Water_flag = 1;
//			error_flag = 1;
//			Sonic_flag = 3;
//			return NO_ERROR;
//		}
//		else if (((DSMRcvBuffer[0] == '0') || (DSMRcvBuffer[0] == 'E')) && (DSMRcvBuffer[1] == '8'))
//		{
//			printf("小管SIL超声波+电容式测水\r\n");
//			Water_flag = 2;
//			error_flag = 1;
//			Sonic_flag = 3;
//			return NO_ERROR;
//		}
//		if (error_flag == 0)
//		{
//			printf("数据包校验错误\r\n");
//			retflag = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return retflag;
//}
///**********************************************************************************************
//**函数名称：	DSM_AngleStart(void)()
//**函数功能：	开始读取角度命令
//**参数:			无
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//
//**********************************************************************************************/
//int DSM_AngleStart(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_ANGLESTART, strlen(DSM_ANGLESTART));
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_SonicStart(void)
//**函数功能：	开始读取超声波命令
//**参数:			无
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//
//**********************************************************************************************/
//int DSM_SonicStart(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_SONICSTART, strlen(DSM_SONICSTART));
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_AngleGet(float *gyroscopevalue)
//**函数功能：	读取角度数值
//**参数:			gyroscopevalue：结果返回值
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//
//**********************************************************************************************/
//int DSM_AngleGet(float *gyroscopevalue_x, float *gyroscopevalue_y)
//{
//	int j;
//	char bcc;
//	float gyroscopevalue;
//	static float gyroscopevalue_last = 0.0; // V1.106，避免角度传感器初始值为0
//	int ret;
//	char no_answer_com = 0, no_answer_gyro = 0, bcc_err_gyro = 0, unvalue_err = 0, bcc_err_com = 0, nan_answer_com = 0;
//	while (1)
//	{
//		if ((no_answer_gyro != 0) || (bcc_err_gyro != 0) || (nan_answer_com != 0) || (unvalue_err != 0)) // V1.115 2020.3.24
//		// 传感器STM32和陀螺仪通讯不上，通讯校验错误。nan值，0值时等待2s再重试发送。
//		{
//			HAL_Delay(1000);
//			HAL_Delay(1000);
//		}
//		HAL_Delay(30);			   // 每次发送之前延时10ms,否则尝试出错可能成为连续的错误
//		Uart2_Interrupt_Disable(); // 关闭中断发送后再打开，避免这段时间内有数据干扰
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_ANGLEGET, strlen(DSM_ANGLEGET));
//
//		Uart2_Interrupt_Enable(); // 打开中断
//
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			no_answer_com++;
//			if (no_answer_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			if ((strncmp(DSMRcvBuffer, DSM_ANGLE_ERROR_COMTIMEOUT, strlen(DSM_ANGLE_ERROR_COMTIMEOUT)) == 0) || (strncmp(DSMRcvBuffer, DSM_SONIC_GYO_ERROR_COMTIMEOUT, strlen(DSM_SONIC_GYO_ERROR_COMTIMEOUT)) == 0))
//			{
//				printf("传感器通讯故障3-8:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_GYROMODULO_TIMEOUT_3_8;
//				no_answer_gyro++;
//				if (no_answer_gyro > 3)
//				{
//					break;
//				}
//				continue;
//			}
//			if (strncmp(DSMRcvBuffer, DSM_ANGLE_ERROR_COMBCC, strlen(DSM_ANGLE_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-9:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_GYROMODULOBCC_ERROR_3_9;
//				bcc_err_gyro++;
//				if (bcc_err_gyro > 3)
//				{
//					break;
//				}
//				continue;
//			}
//			gyroscopevalue = atof(DSMRcvBuffer + 1);
//			*gyroscopevalue_y = gyroscopevalue;
//			gyroscopevalue = atof(DSMRcvBuffer + 9);
//			if ((gyroscopevalue < 0) && (gyroscopevalue < -90))
//			{
//				*gyroscopevalue_x = gyroscopevalue + 360.0;
//			}
//			else
//			{
//				*gyroscopevalue_x = gyroscopevalue;
//			}
//#ifdef DEBUG_DSM
//			printf("DSM:**rcv:over!X=%f,Y=%f\r\n", *gyroscopevalue_x, *gyroscopevalue_y);
//#endif
//			if (Sonic_flag == 0)
//			{
//				if ((*gyroscopevalue_y == 0.0) && (labs(gyroscopevalue_last - *gyroscopevalue_y) > 3.0)) // 数据为零且与上次数据差值大于三，避免真零值
//				{
//					printf("传感器通讯故障读到0\t%s\r\n", DSMRcvBuffer);
//					ret = DSM_ANGLE_3TIMES_ZERO;
//					unvalue_err++;
//					if (unvalue_err > 3)
//					{
//						break;
//					}
//					continue;
//				}
//				else
//				{
//					unvalue_err = 0;
//					bcc_err_com = 0;
//					no_answer_gyro = 0;
//					no_answer_com = 0;
//					bcc_err_com = 0;
//					ret = NO_ERROR;
//					gyroscopevalue_last = *gyroscopevalue_y;
//					break;
//				}
//			}
//			else
//			{
//				unvalue_err = 0;
//				no_answer_gyro = 0;
//				no_answer_com = 0;
//				bcc_err_com = 0;
//				ret = NO_ERROR;
//				gyroscopevalue_last = *gyroscopevalue_y;
//				break;
//			}
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			bcc_err_com++;
//			if (bcc_err_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//	}
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_ResistanceStart(void)()
//**函数功能：	开始读取电阻命令
//**参数:			无
//**返回值:
//						NO_ERROR
//						DSM_GYROMODULOBCC_ERROR
//						DSM_GYROMODULO_TIMEOUT
//**********************************************************************************************/
//int DSM_ResistanceStart(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_RESISTANCESTART, strlen(DSM_RESISTANCESTART));
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_ResistanceGet(float *resistancevalue)
//**函数功能：	读取电阻数值
//**参数:			resistancevalue：结果返回值
//**返回值:		NO_ERROR
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//**********************************************************************************************/
//int DSM_ResistanceGet(float *resistancevalue)
//{
//	char i;
//	char j;
//	char bcc;
//	int ret = 0;
//	for (i = 0; i < 3; ++i)
//	{
//		if ((ret == DSM_ADWATERVALUE_ERROR) || (ret == ERROE_DSM_BCC_3_1)) // 值偏离范围时延时300ms重试
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10); // 重试时没有,最好还是在内部延时，这样外部就不用处理了;
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%s!\r\n", DSM_RESISTANCEGET);
//#endif
//		uart2_send(DSM_RESISTANCEGET, strlen(DSM_RESISTANCEGET));
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s!\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			*resistancevalue = atof((DSMRcvBuffer + 1));
//#ifdef DEBUG_DSM
//			printf("DSM:**rcv:bcc ok!%f\r\n", *resistancevalue);
//#endif
//			if ((*resistancevalue < 0.0) || (*resistancevalue > 3.3))
//			{
//#ifdef DEBUG_DSM
//				printf("传感器通讯故障水位值偏离:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//				ret = DSM_ADWATERVALUE_ERROR;
//				continue;
//			}
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
//
///**********************************************************************************************
//**函数名称：	DSM_ZeroGet(void)()
//**函数功能：	开始读取电阻命令
//**参数:			无
//**返回值:
//						NO_ERROR
//						DSM_GYROMODULOBCC_ERROR
//						DSM_GYROMODULO_TIMEOUT
//**********************************************************************************************/
//int DSM_ZeroStart(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_ZEROSTART, strlen(DSM_ZEROSTART));
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_ZeroGet(float *resistancevalue)
//**函数功能：	读取电阻数值
//**参数:			resistancevalue：结果返回值
//**返回值:		NO_ERROR
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//**********************************************************************************************/
//int DSM_ZeroGet(float *resistancevalue)
//{
//	char i;
//	char j;
//	char bcc;
//	int ret;
//
//	for (i = 0; i < 3; ++i)
//	{
//		if ((ret == DSM_ADZEROVALUE_ERROR) || (ret == ERROE_DSM_BCC_3_1)) // 值偏离范围时延时300ms重试
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10); // 重试时没有,最好还是在内部延时，这样外部就不用处理了;
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%s!\r\n", DSM_ZEROGET);
//#endif
//		uart2_send(DSM_ZEROGET, strlen(DSM_ZEROGET));
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s!\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			*resistancevalue = atof((DSMRcvBuffer + 1));
//#ifdef DEBUG_DSM
//			printf("DSM:**rcv:bcc ok!%f\r\n", *resistancevalue);
//#endif
//			if ((*resistancevalue < 0.0) || (*resistancevalue > 3.3))
//			{
//#ifdef DEBUG_DSM
//				printf("传感器通讯故障零点值偏离:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//				ret = DSM_ADZEROVALUE_ERROR;
//				continue;
//			}
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
///**********************************************************************************************
//int DSM_SonicGet(float *Sonic_A,float *Sonic_B,float *Sonic_C)
//**函数功能：	读取频率值
//**参数:			float *Sonic_A,float *Sonic_B,float *Sonic_C
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//
//**********************************************************************************************/
//int DSM_SonicGet(float *Sonic_A, float *Sonic_B, float *Sonic_C) // V1.118
//{
//	int j;
//	char bcc;
//	float sonicvalue;
//	int ret;
//	char no_answer_com = 0, no_answer_nic = 0, bcc_err_gyro = 0, unvalue_err = 0, bcc_err_com = 0, nan_answer_com = 0;
//	while (1)
//	{
//		if ((no_answer_nic != 0) || (nan_answer_com != 0) || (unvalue_err != 0) || (bcc_err_gyro != 0)) // V1.115 2020.3.24
//		// 传感器STM32和陀螺仪通讯不上，通讯校验错误。nan值，0值时等待300ms再重试发送。
//		{
//			HAL_Delay(1000); // V1.123
//			HAL_Delay(1000);
//			HAL_Delay(1000);
//		}
//		HAL_Delay(40);			   // 每次发送之前延时10ms,否则尝试出错可能成为连续的错误
//		Uart2_Interrupt_Disable(); // 关闭中断发送后再打开，避免这段时间内有数据干扰
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_SONIC, strlen(DSM_SONIC));
//
//		Uart2_Interrupt_Enable(); // 打开中断
//
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			no_answer_com++;
//			if (no_answer_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			if (strncmp(DSMRcvBuffer, DSM_SONIC_ERROR_COMTIMEOUT, strlen(DSM_SONIC_ERROR_COMTIMEOUT)) == 0)
//			{
//				printf("传感器通讯故障3-8:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_SONIC_TIMEOUT_3_18;
//				no_answer_nic++;
//				if (no_answer_nic > 3)
//				{
//					break;
//				}
//				continue;
//			}
//			if (strncmp(DSMRcvBuffer, DSM_SONIC_ERROR_COMBCC, strlen(DSM_SONIC_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-19:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_SONIC_BCC_3_19;
//				bcc_err_gyro++;
//				if (bcc_err_gyro > 3)
//				{
//					break;
//				}
//				continue;
//			}
//			sonicvalue = atof(DSMRcvBuffer + 2);
//			if (sonicvalue >= 100)
//			{
//				*Sonic_A = (sonicvalue / 100);
//			}
//			else if (sonicvalue >= 10)
//			{
//				*Sonic_A = (sonicvalue / 10);
//			}
//			else
//			{
//				*Sonic_A = sonicvalue;
//			}
//
//			sonicvalue = atof(DSMRcvBuffer + 9);
//			if (sonicvalue >= 100)
//			{
//				*Sonic_B = (sonicvalue / 100);
//			}
//			else if (sonicvalue >= 10)
//			{
//				*Sonic_B = (sonicvalue / 10);
//			}
//			else
//			{
//				*Sonic_B = sonicvalue;
//			}
//			sonicvalue = atof(DSMRcvBuffer + 17);
//			if (sonicvalue >= 100)
//			{
//				*Sonic_C = (sonicvalue / 100);
//			}
//			else if (sonicvalue >= 10)
//			{
//				*Sonic_C = (sonicvalue / 10);
//			}
//			else
//			{
//				*Sonic_C = sonicvalue;
//			}
//#ifdef DEBUG_DSM
//			printf("DSM:**nic:over!A=%f,B=%f,C=%f\r\n", *Sonic_A, *Sonic_B, *Sonic_C);
//#endif
//			if ((*Sonic_A == 0.0) || (*Sonic_B == 0.0) || (*Sonic_C == 0.0) || (*Sonic_B > 3.3) || (*Sonic_C > 3.3) || (*Sonic_A > 3.3))
//			{
//				printf("超声波数据故障\t%s\r\n", DSMRcvBuffer);
//				ret = DSM_SONIC_VALUEERROR;
//				unvalue_err++;
//				if (unvalue_err > 3)
//				{
//					break;
//				}
//				continue;
//			}
//			else
//			{
//				ret = NO_ERROR;
//				break;
//			}
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			bcc_err_com++;
//			if (bcc_err_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//	}
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_GetZeroeAverage(float *gyro_angley_average,int count)
//**函数功能：	获取零点平均值
//**参数:
//**返回值:
//						NO_ERROR
//						DSM_GYROMODULOBCC_ERROR
//						DSM_GYROMODULO_TIMEOUT
//**********************************************************************************************/
//int DSM_GetZeroeAverage(float *resistance_average, int count)
//{
//	int ret;
//	float buff, average, resistance;
//	u8 n, m;
//	u8 ii;
//	u8 ave_begin, ave_end, ave_count;
//	ret = DSM_ZeroStart();
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	for (ii = 0; ii < count; ii++)
//	{
//		ret = DSM_ZeroGet(&resistance);
//#ifdef DEBUG_DSM
//		printf("MMWaterMeasure:resistance=%f\r\n", resistance);
//#endif
//		if (ret != NO_ERROR)
//		{
//			// 电阻值读取故障
//			return ret;
//		}
//		Gyro_Angle_Array[ii] = resistance;
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:读取值%f\r\n", resistance);
//#endif
//	}
//	for (n = 0; n < (count - 1); n++)
//	{
//		for (m = 0; m < (count - 1 - n); m++)
//		{
//			if (Gyro_Angle_Array[m] > Gyro_Angle_Array[m + 1])
//			{
//				buff = Gyro_Angle_Array[m];
//				Gyro_Angle_Array[m] = Gyro_Angle_Array[m + 1];
//				Gyro_Angle_Array[m + 1] = buff;
//			}
//		}
//	}
//	ave_begin = count * 0.3;
//	ave_end = count * 0.7;
//	average = 0;
//	ave_count = 0;
//	for (ii = ave_begin; ii < ave_end; ii++)
//	{
//		average = Gyro_Angle_Array[ii] + average;
//		ave_count++;
//	}
//	average = average / ave_count;
//	*resistance_average = average;
//	return NO_ERROR;
//}
//
///**********************************************************************************************
//**函数名称：	DSM_WaterStart(void)() //dq
//**函数功能：	开始读取传感器版本信息判断水位测试方式
//**参数:			无
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//
//**********************************************************************************************/
//int DSM_WaterStart(void)
//{
//	int ret;
//	ret = water_version(DSM_VERSIONSTART, strlen(DSM_VERSIONSTART));
//	return ret;
//}
//
///**********************************************************************************************
//**函数名称：	DSM_SensorStart(void)()
//**函数功能：	开始读取传感器数值（频率/幅值/温度）命令
//**参数:			无
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//
//**********************************************************************************************/
//int DSM_SensorStart(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_SENSORSTART, strlen(DSM_SENSORSTART));
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	int  DSM_SensorGet(struct AM4096_DATA *sensorvalue)
//**函数功能：	读取温度/幅值/频率
//**参数:			sensorvalue：	 结果返回值
//**返回值:		NO_ERROR
//						ERROE_DSM_TIMEOUT_3_3
//						ERROE_DSM_BCC_3_1
//						DSM_GETAD7915_ERROR
//						DSM_DENSITY_BCCERR
//**********************************************************************************************/
//int DSM_SensorGet(struct DSMSENSOR_DATA *sensorvalue)
//{
//	char i;
//	char j, x;
//	int ret = 0;
//	char freq_temp[8];
//	char temperature_temp[8];
//	char voltage_temp[8];
//	double detT = coefficient_detT.coefficient;
//	for (x = 0; x < 3; ++x)
//	{
//		if (ERROE_DSM_BCC_3_1 == ret)
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10);
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_SENSORGET, strlen(DSM_SENSORGET));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_RESISTANCEGET), DSM_RESISTANCEGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		// if(DSMRcvLen != 27)
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			// 7915timeout
//			if (strncmp(DSMRcvBuffer, DSM_SENSOR_ERROR_7915TIMEOUT, strlen(DSM_SENSOR_ERROR_7915TIMEOUT)) == 0)
//			{
//				printf("传感器通讯故障3-5%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				return DSM_GETAD7915_ERROR;
//			}
//			if (strncmp(DSMRcvBuffer, DSM_DENSITY_ERROR_COMBCC, strlen(DSM_DENSITY_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-16%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				return DSM_DENSITY_BCCERR;
//			}
//			/*frequency*/
//			for (i = 0; i < 7; i++)
//			{
//				freq_temp[i] = DSMRcvBuffer[i + 1];
//			}
//			sensorvalue->Sensor_Frequency = atof(freq_temp);
//			/*voltage*/
//			for (i = 0; i < 7; i++)
//			{
//				voltage_temp[i] = DSMRcvBuffer[i + 9];
//			}
//			sensorvalue->Sensor_Voltage = atof(voltage_temp);
//			/*temperature*/
//			for (i = 0; i < 7; i++)
//			{
//				temperature_temp[i] = DSMRcvBuffer[i + 17];
//			}
//			sensorvalue->Sensor_Temperature = atof(temperature_temp) + detT;
//			if ((sensorvalue->Sensor_Frequency > 5000.0) || (sensorvalue->Sensor_Frequency < 4000.0) || (sensorvalue->Sensor_Voltage < 0.1) || (sensorvalue->Sensor_Voltage > 10.0) || (sensorvalue->Sensor_Temperature > 100.0) || (sensorvalue->Sensor_Temperature < -100.0))
//			{
//				printf("传感器通讯读取密度参数不能用作温度判断的原始数据包\t长度：%d字符串：%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			}
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
///**********************************************************************************************
//**函数名称：	DSM_SensorStart(void)()
//**函数功能：	启动快速读取传感器数值（频率/幅值/温度）的命令
//**参数:			无
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//**********************************************************************************************/
//int DSM_Sensorfast_Start(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_SENSORFASTSTART, strlen(DSM_SENSORFASTSTART));
//	return ret;
//}
//
///**********************************************************************************************
//**函数名称：	int  DSM_Sensorfast_Get(struct AM4096_DATA *sensorvalue)
//**函数功能：读取反应快速的频率值
//**参数:			sensorvalue：	 结果返回值
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//						DSM_GETAD7915_ERROR
//
//**********************************************************************************************/
//int DSM_Sensorfast_Get(struct DSMSENSOR_DATA *sensorvalue)
//{
//	char i, x;
//	char j;
//	int ret = 0;
//	char freq_temp[8];
//	char temperature_temp[8];
//	char voltage_temp[8];
//	double detT = coefficient_detT.coefficient;
//	for (x = 0; x < 3; ++x)
//	{
//		if (ERROE_DSM_BCC_3_1 == ret)
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10);
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_SENSORFASTGET, strlen(DSM_SENSORFASTGET));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_SENSORFASTGET), DSM_SENSORFASTGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			if (strncmp(DSMRcvBuffer, DSM_SENSOR_ERROR_7915TIMEOUT, strlen(DSM_SENSOR_ERROR_7915TIMEOUT)) == 0)
//			{
//				printf("传感器通讯故障3-5%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				return DSM_GETAD7915_ERROR;
//			}
//			if (strncmp(DSMRcvBuffer, DSM_DENSITY_ERROR_COMBCC, strlen(DSM_DENSITY_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-16%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				return DSM_DENSITY_BCCERR;
//			}
//			/*frequency*/
//			for (i = 0; i < 7; i++)
//			{
//				freq_temp[i] = DSMRcvBuffer[i + 1];
//			}
//			sensorvalue->Sensor_Frequency = atof(freq_temp);
//			/*voltage*/
//			for (i = 0; i < 7; i++)
//			{
//				voltage_temp[i] = DSMRcvBuffer[i + 9];
//			}
//			sensorvalue->Sensor_Voltage = atof(voltage_temp) + 0.05;
//			/*temperature*/
//			for (i = 0; i < 7; i++)
//			{
//				temperature_temp[i] = DSMRcvBuffer[i + 17];
//			}
//			sensorvalue->Sensor_Temperature = atof(temperature_temp) + detT;
//			if ((sensorvalue->Sensor_Frequency > 250.0) || (sensorvalue->Sensor_Frequency < 0.0))
//			{
//				printf("传感器通讯读取频率参数故障\t长度：%d字符串：%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			}
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
//
///**********************************************************************************************
//**函数名称：	int DSM_GetFreverage(float *fre,int count)
//**函数功能：读取反应快速的频率值
//**参数:			sensorvalue：	 结果返回值
//**返回值:		NO_ERROR
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//						DSM_GETAD7915_ERROR
//**********************************************************************************************/
//
//int DSM_GetFreverage(float *fre, int count)
//{
//	struct DSMSENSOR_DATA dsmsensor_data;
//	int ret;
//	float buff, average;
//	int i, n, m;
//
//	int ave_begin;
//	int ave_end;
//	int ave_count;
//	ret = DSM_Sensorfast_Start();
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	if (count >= 100)
//	{
//		count = 100;
//	}
//	for (i = 0; i < count; i++)
//	{
//		ret = DSM_Sensorfast_Get(&dsmsensor_data);
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//		Gyro_Angle_Array[i] = dsmsensor_data.Sensor_Frequency;
//	}
//	for (n = 0; n < (count - 1); n++)
//	{
//		for (m = 0; m < (count - 1 - n); m++)
//		{
//			if (Gyro_Angle_Array[m] > Gyro_Angle_Array[m + 1])
//			{
//				buff = Gyro_Angle_Array[m];
//				Gyro_Angle_Array[m] = Gyro_Angle_Array[m + 1];
//				Gyro_Angle_Array[m + 1] = buff;
//			}
//		}
//	}
//	ave_begin = count * 0.3;
//	ave_end = count * 0.6;
//	average = 0;
//	ave_count = 0;
//	for (i = ave_begin; i < ave_end; i++)
//	{
//		average = Gyro_Angle_Array[i] + average;
//		ave_count++;
//	}
//	average = average / ave_count;
//	*fre = average;
//	return NO_ERROR;
//}
//
///**********************************************************************************************
//**函数名称：	DSM_GetAngleXverage(float *gyro_anglex_average,int count)
//**函数功能：	获取角度X的平均值
//**参数:
//**返回值:		NO_ERROR 	0 		 通信成功
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//
//**********************************************************************************************/
//int DSM_GetAngleXverage(float *gyro_anglex_average, int count)
//{
//	int ret;
//	float buff, average;
//	int i, n, m;
//	float gyroscopevalue_x;
//	float gyroscopevalue_y;
//
//	int ave_begin;
//	int ave_end;
//	int ave_count;
//
//	ret = DSM_AngleStart();
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//
//	for (i = 0; i < count; i++)
//	{
//		// HAL_Delay(5);
//		ret = DSM_AngleGet(&gyroscopevalue_x, &gyroscopevalue_y);
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//		Gyro_Angle_Array[i] = gyroscopevalue_x;
//	}
//
//	for (n = 0; n < (count - 1); n++)
//	{
//		for (m = 0; m < (count - 1 - n); m++)
//		{
//			if (Gyro_Angle_Array[m] > Gyro_Angle_Array[m + 1])
//			{
//				buff = Gyro_Angle_Array[m];
//				Gyro_Angle_Array[m] = Gyro_Angle_Array[m + 1];
//				Gyro_Angle_Array[m + 1] = buff;
//			}
//		}
//	}
//
//	ave_begin = count * 0.3;
//	ave_end = count * 0.6;
//	average = 0;
//	ave_count = 0;
//	for (i = ave_begin; i < ave_end; i++)
//	{
//		average = Gyro_Angle_Array[i] + average;
//		ave_count++;
//	}
//	average = average / ave_count;
//	*gyro_anglex_average = average;
//
//	return NO_ERROR;
//}
//
///**********************************************************************************************
//**函数名称：	DSM_GetAngleYverage(float *gyro_angley_average,int count)
//**函数功能：	获取角度Y的平均值
//**参数:
//**返回值:		NO_ERROR 	0 		 通信成功
//						DSM_ERROR_COMTIMEOUT
//						DSM_ERROR_COMBCC
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//
//**********************************************************************************************/
//int DSM_GetAngleYverage(float *gyro_angley_average, int count)
//{
//	int ret;
//	float buff, average;
//	int i, n, m;
//	float gyroscopevalue_x;
//	float gyroscopevalue_y;
//	//	float gyroscopevalue_z;
//	int ave_begin;
//	int ave_end;
//	int ave_count;
//
//	ret = DSM_AngleStart();
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	for (i = 0; i < count; i++)
//	{
//		ret = DSM_AngleGet(&gyroscopevalue_x, &gyroscopevalue_y);
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//		Gyro_Angle_Array[i] = gyroscopevalue_y;
//	}
//	for (n = 0; n < (count - 1); n++)
//	{
//		for (m = 0; m < (count - 1 - n); m++)
//		{
//			if (Gyro_Angle_Array[m] > Gyro_Angle_Array[m + 1])
//			{
//				buff = Gyro_Angle_Array[m];
//				Gyro_Angle_Array[m] = Gyro_Angle_Array[m + 1];
//				Gyro_Angle_Array[m + 1] = buff;
//			}
//		}
//	}
//	ave_begin = count * 0.3;
//	ave_end = count * 0.7;
//	average = 0;
//	ave_count = 0;
//	for (i = ave_begin; i < ave_end; i++)
//	{
//		average = Gyro_Angle_Array[i] + average;
//		ave_count++;
//	}
//	average = average / ave_count;
//	*gyro_angley_average = average;
//	return NO_ERROR;
//}
//
///**********************************************************************************************
//**函数名称：	DSM_GetAngleYverage(float *gyro_angley_average,int count)
//**函数功能：	获取角度Y的平均值
//**参数:
//**返回值:
//						NO_ERROR
//						DSM_GYROMODULOBCC_ERROR
//						DSM_GYROMODULO_TIMEOUT
//**********************************************************************************************/
//int DSM_GetResistanceAverage(float *resistance_average, int count)
//{
//	int ret;
//	float buff, average, resistance;
//	u8 n, m;
//	u8 ii;
//	u8 ave_begin, ave_end, ave_count;
//
//	ret = DSM_ResistanceStart();
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	HAL_Delay(1000);
//	for (ii = 0; ii < count; ii++)
//	{
//		ret = DSM_ResistanceGet(&resistance);
//#ifdef DEBUG_DSM
//		printf("MMWaterMeasure:resistance=%f\r\n", resistance);
//#endif
//		if (ret != NO_ERROR)
//		{
//			// 电阻值读取故障
//			return ret;
//		}
//		Gyro_Angle_Array[ii] = resistance;
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:读取值%f\r\n", resistance);
//#endif
//	}
//	for (n = 0; n < (count - 1); n++)
//	{
//		for (m = 0; m < (count - 1 - n); m++)
//		{
//			if (Gyro_Angle_Array[m] > Gyro_Angle_Array[m + 1])
//			{
//				buff = Gyro_Angle_Array[m];
//				Gyro_Angle_Array[m] = Gyro_Angle_Array[m + 1];
//				Gyro_Angle_Array[m + 1] = buff;
//			}
//		}
//	}
//	ave_begin = count * 0.3;
//	ave_end = count * 0.7;
//	average = 0;
//	ave_count = 0;
//	for (ii = ave_begin; ii < ave_end; ii++)
//	{
//		average = Gyro_Angle_Array[ii] + average;
//		ave_count++;
//	}
//	average = average / ave_count;
//	*resistance_average = average;
//	return NO_ERROR;
//}
//
///**
// * @func: int DSM_GetSonicverage(float *sonic_a_average, float *sonic_b_average, float *sonic_c_average, int count)
// * @description: 获取超声波探头的平均值
// * @param {float} *sonic_a_average
// * @param {float} *sonic_b_average
// * @param {float} *sonic_c_average
// * @param {int} count
// * @return { }
// */
//int DSM_GetSonicverage(float *sonic_a_average, float *sonic_b_average, float *sonic_c_average, int count)
//{
//	int ret;
//	float buff, average, average1, average2;
//	int i, n, m;
//	float sonic_a;
//	float sonic_b;
//	float sonic_c;
//
//	int ave_begin;
//	int ave_end;
//	int ave_count;
//
//	ret = DSM_SonicStart();
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	for (i = 0; i < count; i++)
//	{
//		// HAL_Delay(5);
//		ret = DSM_SonicGet(&sonic_a, &sonic_b, &sonic_c);
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//		Sonic_Array_A[i] = sonic_a;
//		Sonic_Array_B[i] = sonic_b;
//		Sonic_Array_C[i] = sonic_c;
//	}
//	for (n = 0; n < (count - 1); n++)
//	{
//		for (m = 0; m < (count - 1 - n); m++)
//		{
//			if (Sonic_Array_A[m] > Sonic_Array_A[m + 1])
//			{
//				buff = Sonic_Array_A[m];
//				Sonic_Array_A[m] = Sonic_Array_A[m + 1];
//				Sonic_Array_A[m + 1] = buff;
//			}
//			if (Sonic_Array_B[m] > Sonic_Array_B[m + 1])
//			{
//				buff = Sonic_Array_B[m];
//				Sonic_Array_B[m] = Sonic_Array_B[m + 1];
//				Sonic_Array_B[m + 1] = buff;
//			}
//			if (Sonic_Array_C[m] > Sonic_Array_C[m + 1])
//			{
//				buff = Sonic_Array_C[m];
//				Sonic_Array_C[m] = Sonic_Array_C[m + 1];
//				Sonic_Array_C[m + 1] = buff;
//			}
//		}
//	}
//	ave_begin = count * 0.3;
//	ave_end = count * 0.6;
//	average = 0;
//	average1 = 0;
//	average2 = 0;
//	ave_count = 0;
//	for (i = ave_begin; i < ave_end; i++)
//	{
//		average = Sonic_Array_A[i] + average;
//		average1 = Sonic_Array_B[i] + average1;
//		average2 = Sonic_Array_C[i] + average2;
//		ave_count++;
//	}
//	average = average / ave_count;
//	average1 = average1 / ave_count;
//	average2 = average2 / ave_count;
//	*sonic_a_average = average;
//	*sonic_b_average = average1;
//	*sonic_c_average = average2;
//	return NO_ERROR;
//}
//
///**
// * @func: int DSM_SonicFrequency_Get(float *Frequency1, float *Frequency2)
// * @description: 获取超声传感器频率
// * @param {float} *Frequency1
// * @param {float} *Frequency2
// * @return NO_ERROR
// */
//int DSM_SonicFrequency_Get(float *Frequency1, float *Frequency2)
//{
//	int ret;
//	float sonic_a, sonic_b, sonic_c;
//	if (Sonic_flag == 1)
//	{
//		ret = DSM_SonicGet(&sonic_a, &sonic_b, &sonic_c);
//		*Frequency1 = sonic_c * 6800.0;
//		*Frequency2 = sonic_a * 6800.0;
//		printf("扫频完成\tA路频率值：%f\t", *Frequency1);
//		printf("B路频率值：%f\r\n", *Frequency2);
//	}
//	else
//	{
//		ret = DSM_Sweep(Frequency1, Frequency2);
//	}
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	return NO_ERROR;
//}
//
///**
// * @func: static int DSM_Sweep(float *sonicFrequency1, float *sonicFrequency2)
// * @description: 传感器扫频
// * @param {float} *sonicFrequency1
// * @param {float} *sonicFrequency2
// * @return NO_ERROR
// */
//static int DSM_Sweep(float *sonicFrequency1, float *sonicFrequency2)
//{
//	int j;
//	char bcc;
//	int ret;
//	float a, b, c;
//	char no_answer_com = 0, nan_answer_com = 0, bcc_err_com = 0, sweep_err = 0;
//	/*		传感器无数据包	传感器接受包异常	传感器校验错误		传感器扫频错误					*/
//	while (1)
//	{
//		if ((nan_answer_com != 0) || (sweep_err != 0)) // V1.115 2020.3.24
//		// 传感器STM32和陀螺仪通讯不上，通讯校验错误。nan值时等待3000ms再重试发送。
//		{
//			HAL_Delay(300); // V1.123
//		}
//		HAL_Delay(40);			   // 每次发送之前延时10ms,否则尝试出错可能成为连续的错误
//		Uart2_Interrupt_Disable(); // 关闭中断发送后再打开，避免这段时间内有数据干扰
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_SWEEP_AB, strlen(DSM_SWEEP_AB));
//
//		Uart2_Interrupt_Enable(); // 打开中断
//
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			no_answer_com++;
//			if (no_answer_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			if (strncmp(DSMRcvBuffer, DSM_SONIC_AD_ERROR, strlen(DSM_SONIC_AD_ERROR)) == 0)
//			{
//				printf("扫频幅值异常3-21:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_SONIC_FREQUENCY_ERROR;
//				sweep_err++;
//				if (sweep_err > 3)
//				{
//					break;
//				}
//				continue;
//			}
//			else
//			{
//				*sonicFrequency1 = atof(DSMRcvBuffer + 2);
//				printf("扫频完成\tA路频率值：%f\t", *sonicFrequency1);
//				*sonicFrequency2 = atof(DSMRcvBuffer + 9);
//				printf("B路频率值：%f\r\n", *sonicFrequency2);
//				if (DSMRcvBuffer[0] == 'a')
//					printf("故障 A探头频率不稳定\r\n");
//				if (DSMRcvBuffer[8] == 'b')
//					printf("故障 B探头频率不稳定\r\n");
//				if ((*sonicFrequency1 > 18000) || (*sonicFrequency2 > 18000))
//				{
//					printf("故障 探头频率过高\r\n");
//				}
//				if ((*sonicFrequency1 > 19000) || (*sonicFrequency2 > 19000) || (*sonicFrequency1 < 10010))
//				{
//					printf("故障 探头频率异常\r\n");
//					DSM_SonicAD_Get_SIL(&a, &b, &c);
//					DSM_SenserVoltageGet(); /* 打印电压值 */
//					if (DSMRcvBuffer[0] == 'E')
//						printf("故障 传感器电压异常\r\n");
//					if (*sonicFrequency1 > 19000)
//					{
//						ret = DSM_SONIC_A_OVER;
//					}
//					else if (*sonicFrequency2 > 19000)
//					{
//						ret = DSM_SONIC_B_OVER;
//					}
//					else
//					{
//						ret = DSM_SONIC_A_OLW;
//					}
//					sweep_err++;
//					if (sweep_err > 3)
//					{
//						break;
//					}
//					continue;
//				}
//				return NO_ERROR;
//			}
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			bcc_err_com++;
//			if (bcc_err_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//	}
//	return ret;
//}
//
///**
// * @func: int DSM_SonicAD_Get_SIL(float *Sonic_A, float *Sonic_B, float *Sonic_C)
// * @description: 读取超声AD值
// * @param {float} *Sonic_A	A探头幅值
// * @param {float} *Sonic_B	B探头幅值
// * @param {float} *Sonic_C	C探头幅值
// * @return NO_ERROR
// */
//int DSM_SonicAD_Get_SIL(float *Sonic_A, float *Sonic_B, float *Sonic_C)
//{
//	int j;
//	char bcc;
//	//	float 	sonicvalue;
//	int ret;
//	char no_answer_nic = 0, bcc_err_nic = 0, no_answer_com = 0, nan_answer_com = 0, bcc_err_com = 0;
//	/*		超声板无数据包		超声校验错误包	传感器无数据包		传感器接受包异常	传感器校验错误	*/
//	while (1)
//	{
//		if ((no_answer_nic != 0) || (nan_answer_com != 0) || (bcc_err_nic != 0)) // V1.115 2020.3.24
//		// 传感器STM32和陀螺仪通讯不上，通讯校验错误。nan值时等待3000ms再重试发送。
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(40);			   // 每次发送之前延时40ms,否则尝试出错可能成为连续的错误
//		Uart2_Interrupt_Disable(); // 关闭中断发送后再打开，避免这段时间内有数据干扰
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_SONIC, strlen(DSM_SONIC)); /*  发送读取超声指令   */
//
//		Uart2_Interrupt_Enable(); // 打开中断
//
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			no_answer_com++;
//			if (no_answer_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//
//			*Sonic_A = atof(DSMRcvBuffer + 2);
//			*Sonic_B = atof(DSMRcvBuffer + 9);
//			printf("AD采集完成 A=%f,B=%f\r\n", *Sonic_A, *Sonic_B);
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			bcc_err_com++;
//			if (bcc_err_com > 3)
//			{
//				break;
//			}
//			continue;
//		}
//	}
//	return ret;
//}
//
///**
// * @func: int DSM_SensorGet_SIL(struct DSMSENSOR_DATA *sensorvalue)
// * @description: 读取温度/密度/频率
// * @param {DSMSENSOR_DATA} *sensorvalue	传感器数据
// * @return NO_ERROR
// */
//int DSM_SensorGet_SIL(struct DSMSENSOR_DATA *sensorvalue)
//{
//	char i;
//	char j, x;
//	int ret = 0;
//	char density_temp[8];
//	char temperature_temp[8];
//	char dynamic_temp[8];
//	double detT = coefficient_detT.coefficient;
//	for (x = 0; x < 3; ++x)
//	{
//		if ((ERROE_DSM_BCC_3_1 == ret) || (DSM_GETAD7915_ERROR == ret))
//		{
//			HAL_Delay(300);
//		}
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_SENSORGET, strlen(DSM_SENSORGET));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_RESISTANCEGET), DSM_RESISTANCEGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			// 7915timeout
//			if (strncmp(DSMRcvBuffer, DSM_CPU0_ERROR_COMTIMEOUT, strlen(DSM_CPU0_ERROR_COMTIMEOUT)) == 0)
//			{
//				printf("传感器通讯故障3-5 %d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_GETAD7915_ERROR;
//			}
//			else if (strncmp(DSMRcvBuffer, DSM_CPU0_ERROR_COMBCC, strlen(DSM_CPU0_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-16 %d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_DENSITY_BCCERR;
//			}
//			else
//			{
//				/*frequency*/
//				for (i = 0; i < 7; i++)
//				{
//					density_temp[i] = DSMRcvBuffer[i + 1];
//				}
//				sensorvalue->Sensor_Frequency = atof(density_temp);
//				/*voltage*/
//				for (i = 0; i < 7; i++)
//				{
//					dynamic_temp[i] = DSMRcvBuffer[i + 9];
//				}
//				sensorvalue->Density = atof(dynamic_temp);
//				/*temperature*/
//				for (i = 0; i < 7; i++)
//				{
//					temperature_temp[i] = DSMRcvBuffer[i + 17];
//				}
//				sensorvalue->Sensor_Temperature = atof(temperature_temp) + detT;
//				if (DSMRcvBuffer[0] == 'E')
//					printf("故障 传感器电压异常");
//				return NO_ERROR;
//			}
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
//
///**
// * @func: int DSM_DensityParameterGet_SIL(struct DSMSENSOR_DATA *sensorvalue)
// * @description: 读取密度分析参数
// * @param {DSMSENSOR_DATA} *sensorvalue 传感器数据
// * @return NO_ERROR
// */
//int DSM_DensityParameterGet_SIL(struct DSMSENSOR_DATA *sensorvalue)
//{
//	char i;
//	char j, x;
//	int ret = 0;
//	char temp[8];
//
//	for (x = 0; x < 3; ++x)
//	{
//		if ((ERROE_DSM_BCC_3_1 == ret) || (DSM_GETAD7915_ERROR == ret))
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10);
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_DENSITYPARAMETER, strlen(DSM_DENSITYPARAMETER));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_RESISTANCEGET), DSM_RESISTANCEGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		// if(DSMRcvLen != 27)
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			// 7915timeout
//			if (strncmp(DSMRcvBuffer, DSM_CPU0_ERROR_COMTIMEOUT, strlen(DSM_CPU0_ERROR_COMTIMEOUT)) == 0)
//			{
//				printf("传感器通讯故障3-5 %d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_GETAD7915_ERROR;
//			}
//			else if (strncmp(DSMRcvBuffer, DSM_CPU0_ERROR_COMBCC, strlen(DSM_CPU0_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-16 %d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_DENSITY_BCCERR;
//			}
//			else
//			{
//				for (i = 0; i < 7; i++)
//				{
//					temp[i] = DSMRcvBuffer[i + 1];
//				}
//				sensorvalue->MeanSquareOf225DegreeSweepPeriod = atof(temp);
//				for (i = 0; i < 7; i++)
//				{
//					temp[i] = DSMRcvBuffer[i + 9];
//				}
//				sensorvalue->SquareMeanOf45degreeSweepPeriod = atof(temp);
//				for (i = 0; i < 7; i++)
//				{
//					temp[i] = DSMRcvBuffer[i + 17];
//				}
//				sensorvalue->Dynamic_Viscosity = atof(temp);
//				return NO_ERROR;
//			}
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
//
///**
// * @func: int DSM_SenserNumGet(void)
// * @description: 读取传感器编号
// * @return NO_ERROR
// */
//int DSM_SenserNumGet(void)
//{
//	char i;
//	char j, x;
//	int ret = 0;
//	char SenserNum_temp[8];
//
//	for (x = 0; x < 3; ++x)
//	{
//		if ((ERROE_DSM_BCC_3_1 == ret) || (DSM_GETAD7915_ERROR == ret))
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10);
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_NUMBER, strlen(DSM_NUMBER));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_RESISTANCEGET), DSM_RESISTANCEGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		// if(DSMRcvLen != 27)
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			// 7915timeout
//			if (strncmp(DSMRcvBuffer, DSM_CPU0_ERROR_COMTIMEOUT, strlen(DSM_CPU0_ERROR_COMTIMEOUT)) == 0)
//			{
//				printf("传感器通讯故障3-5 %d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_GETAD7915_ERROR;
//			}
//			else if (strncmp(DSMRcvBuffer, DSM_CPU0_ERROR_COMBCC, strlen(DSM_CPU0_ERROR_COMBCC)) == 0)
//			{
//				printf("传感器通讯故障3-16 %d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//				ret = DSM_DENSITY_BCCERR;
//			}
//			else
//			{
//				for (i = 0; i < 7; i++)
//				{
//					SenserNum_temp[i] = DSMRcvBuffer[i + 1];
//				}
//				systemunion.systemparameter.DeviceNum = (int)atof(SenserNum_temp);
//				SystemParameterWrite(); /* 写入铁电 */
//				return NO_ERROR;
//			}
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
//
///**
// * @func: int DSM_SenserVoltageGet(void)
// * @description: 读取传感器电压
// * @return NO_ERROR
// */
//int DSM_SenserVoltageGet(void)
//{
//	char i;
//	char j, x;
//	int ret = 0;
//	char SenserVoltage_temp[8];
//	float SenserVoltage;
//
//	for (x = 0; x < 3; ++x)
//	{
//		if (ERROE_DSM_BCC_3_1 == ret)
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10);
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_POWER, strlen(DSM_POWER));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_RESISTANCEGET), DSM_RESISTANCEGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		// if(DSMRcvLen != 27)
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			for (i = 0; i < 7; i++)
//			{
//				SenserVoltage_temp[i] = DSMRcvBuffer[i + 1];
//			}
//			SenserVoltage = atof(SenserVoltage_temp);
//			printf("传感器供电电压:%fV\r\n", SenserVoltage);
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
///**
// * @func: int DSM_Switch_LevelMode(void)
// * @description: 传感器切换液位模式
// * @return { }
// */
//int DSM_Switch_LevelMode(void)
//{
//	int ret;
//	ret = DSMSendcommand3times(DSM_GET_FREQUENCE_START, strlen(DSM_GET_FREQUENCE_START));
//	HAL_Delay(1000);
//	HAL_Delay(1000);
//	HAL_Delay(1000); /*切换模式后需要等待3S读取数据*/
//	return ret;
//}
///**
// * @func: DSM_Get_LevelMode_Frequence(float* Frequency)
// * @description: 获取传感器震动管频率，用于寻找液位
// * @param {float*} AirFrequency
// * @return { }
// */
//int DSM_Get_LevelMode_Frequence(float *Frequency)
//{
//	char i;
//	char retry_times;
//	int ret = 0;
//	char temp[8];
//
//	for (retry_times = 0; retry_times < 3; ++retry_times)
//	{
//		if (ERROE_DSM_BCC_3_1 == ret)
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10);
//		for (i = 0; i < RCVBUFFLEN; ++i)
//		{
//			DSMRcvBuffer[i] = 0;
//		}
//		DSMRcvLen = 0;
//		uart2_send(DSM_GET_FREQUENCE, strlen(DSM_GET_FREQUENCE));
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%d:%s\r\n", strlen(DSM_RESISTANCEGET), DSM_RESISTANCEGET);
//#endif
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		if (CalculationBCC_DSM(DSMRcvBuffer, DSMRcvLen - 3) == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			for (i = 0; i < 7; i++)
//			{
//				temp[i] = DSMRcvBuffer[i + 1];
//			}
//			*Frequency = atof(temp);
//			printf("密度计频率\t%f\r\n", *Frequency);
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误%d,%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
//
///**
// * @func:uint32_t isSensorTiltAngleAbnormal(float gyro_anglex, float gyro_angley)
// * @description: 判断传感器倾角姿态是否正常
// * @param {float} gyro_anglex	x轴倾角
// * @param {float} gyro_angley	y轴倾角
// * @return {0：正常	1：异常}
// */
//uint32_t isSensorTiltAngleAbnormal(float gyro_anglex, float gyro_angley)
//{
//	if ((fabs(gyro_anglex - X_Zero) > XY_START_JUDGE) && (Sonic_flag != 3)) /*非小管传感器传感器*/
//	{
//		printf("读取X轴角度值异常\t当前角度%f\t零点角度%f\r\n", gyro_anglex, X_Zero);
//		return 1;
//	}
//	if ((fabs(gyro_angley - Y_Zero) > XY_START_JUDGE) && (Sonic_flag != 0)) /*超声传感器*/
//	{
//		printf("读取Y轴角度值异常\t当前角度%f\t零点角度%f\r\n", gyro_angley, Y_Zero);
//		return 1;
//	}
//	return 0;
//}
///**********************************************************************************************
//**函数名称：	DSM_CapacitanceGet(float *resistancevalue)
//**函数功能：	读取电阻数值
//**参数:			resistancevalue：结果返回值
//**返回值:		NO_ERROR
//						DSM_GYROMODULO_TIMEOUT
//						DSM_GYROMODULOBCC_ERROR
//**********************************************************************************************/
//int DSM_CapacitanceGet(float *capacitance)
//{
//	char i;
//	char j;
//	char bcc;
//	int ret = 0;
//	for (i = 0; i < 3; ++i)
//	{
//		if ((ret == DSM_ADWATERVALUE_ERROR) || (ret == ERROE_DSM_BCC_3_1)) // 值偏离范围时延时300ms重试
//		{
//			HAL_Delay(300);
//		}
//		HAL_Delay(10); // 重试时没有,最好还是在内部延时，这样外部就不用处理了;
//		for (j = 0; j < RCVBUFFLEN; ++j)
//		{
//			DSMRcvBuffer[j] = 0;
//		}
//		DSMRcvLen = 0;
//#ifdef DEBUG_DSM
//		printf("DSM:**snd:%s!\r\n", DSM_RESISTANCEGET);
//#endif
//		uart2_send(DSM_RESISTANCEGET, strlen(DSM_RESISTANCEGET));
//		DSMRcvLen = uart2_receive(DSMRcvBuffer);
//#ifdef DEBUG_DSM
//		printf("DSM:**rcv:%d:%s!\r\n", DSMRcvLen, DSMRcvBuffer);
//#endif
//		if (DSMRcvLen == 0)
//		{
//			printf("传感器通讯故障无数据包3-3\r\n");
//			ret = ERROE_DSM_TIMEOUT_3_3;
//			continue;
//		}
//		bcc = CalculationBCC_DSM(DSMRcvBuffer, (DSMRcvLen - 3));
//		if (bcc == DSMRcvBuffer[DSMRcvLen - 3])
//		{
//			*capacitance = atof((DSMRcvBuffer + 1));
//			if (*capacitance < 40)
//			{
//				*capacitance = 9999;
//			}
//			return NO_ERROR;
//		}
//		else
//		{
//			printf("传感器通讯故障校验错误:%d:%s\r\n", DSMRcvLen, DSMRcvBuffer);
//			ret = ERROE_DSM_BCC_3_1;
//			continue;
//		}
//	}
//	return ret;
//}
