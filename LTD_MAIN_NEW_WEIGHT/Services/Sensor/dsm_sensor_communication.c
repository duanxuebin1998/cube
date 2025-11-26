/*
 * dsm_sensor_communication.c
 *
 *  Created on: Nov 10, 2025
 *      Author: admin
 */
#include "dsm_sensor_communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <inttypes.h>

DSMSENSOR_DATA dsmsensor_data;

char DSMCommand[RCVBUFFLEN];
char DSMRcvBuffer[RCVBUFFLEN];
int DSMRcvLen;

static char CalculationBCC_DSM(char command[], int count);

// 错误码关键字表
const char *error_codes[] = {
    "A+111.11B+111.11", // 超声无谐振
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
    memset(response, 0, maxLen);
    uint16_t recvLen = 0;
#ifdef DEBUG_UART6
    printf("[UART6] Send: %s\n", cmd);
#endif

    // 发送
    if (HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), 100) != HAL_OK) {
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
                // 根据协议判断帧结束符（示例为 0x0A 换行）
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
        printf("%02X ", (uint8_t)response[i]);
    }
    printf("\r\n");
#endif

    // 校验 BCC
    bcc = CalculationBCC_DSM(response, (recvLen - 3));
    if (bcc == response[recvLen - 3]) {
#ifdef DEBUG_UART6
        printf("DSM: rcv BCC校验通过!\r\n");
#endif
        return NO_ERROR;
    } else {
#ifdef DEBUG_UART6
        printf("DSM: rcv BCC校验失败!\r\n");
#endif
        return SENSOR_BCC_ERROR; // 校验失败
    }
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
                ret = SENSOR_BCC_ERROR; // 错误码视为通信失败
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
            return (float)atof(resp + 1);
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

        // 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功
        if (strstr(resp, "%") != NULL) {
            printf("[Probe] 测水探针开启成功！\r\n");
            return 0;
        } else {
            printf("[Probe] 无效响应: %s\r\n", resp);
            return SENSOR_COMM_TIMEOUT; // 响应格式不对
        }
    } else {
        printf("[Probe] 测水探针开启失败！\r\n");
        return SENSOR_COMM_TIMEOUT; // 通信失败或错误码
    }
}

// 开启液位模式
int DSM_EnableLevelMode(void) {
    char resp[RX_BUF_LEN];

    // 发送命令 "CB\r\n" 并带 3 次重试
    if (UART6_SendWithRetry("CB\r\n", resp, RX_BUF_LEN, 500) == 0) {
        printf("[液位模式] 开启液位模式响应: %s\r\n", resp);

        // 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功
        if (strstr(resp, "%") != NULL) {
            printf("[液位模式] 开启成功！\r\n");
            return NO_ERROR;
        } else {
            printf("[液位模式] 无效响应: %s\r\n", resp);
            return NO_ERROR; // 响应格式不对
        }
    } else {
        printf("[液位模式] 开启失败！\r\n");
        return SENSOR_COMM_TIMEOUT; // 通信失败或错误码
    }
}

// 开启密度模式
int DSM_EnableDensityMode(void) {
    char resp[RX_BUF_LEN];

    // 发送命令 "CD\r\n" 并带 3 次重试
    if (UART6_SendWithRetry("CD\r\n", resp, RX_BUF_LEN, 500) == 0) {
        printf("[密度模式] 开启密度模式响应: %s\r\n", resp);

        // 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功
        if (strstr(resp, "%") != NULL) {
            printf("[密度模式] 开启成功！\r\n");
            return NO_ERROR;
        } else {
            printf("[密度模式] 无效响应: %s\r\n", resp);
            return NO_ERROR; // 响应格式不对
        }
    } else {
        printf("[密度模式] 开启失败！\r\n");
        return SENSOR_COMM_TIMEOUT; // 通信失败或错误码
    }
}

// 工具函数: 解析 "E06.6379V\r\n" 这类响应为浮点数
static int parse_freq_response(const char *resp, float *out_hz)
{
    if (!resp || !out_hz) return -1;

    // 1) 跳过起始标志（例如 'E'）和前导空白
    const char *p = resp;
    while (*p && !isdigit((unsigned char)*p) && *p != '-' && *p != '+') {
        ++p;
    }
    if (!*p) return -2;

    // 2) 使用 strtod 解析到非数字处（会自动停在 'V' 或回车）
    char *endp = NULL;
    double v = strtod(p, &endp);
    if (endp == p) return -3;   // 没解析到数字
    if (!isfinite(v)) return -4;

    *out_hz = (float)v;
    return 0;
}

// 读取液位跟随频率（单次）
uint32_t Read_Level_Frequency(uint32_t *frequency_out)
{
    if (!frequency_out) return SENSOR_COMM_TIMEOUT;

    char resp[RX_BUF_LEN] = {0};
    uint32_t ret = UART6_SendWithRetry("Cb\r\n", resp, RX_BUF_LEN, 500);
    if (ret != 0) {
        return SENSOR_COMM_TIMEOUT;
    }

    printf("[UART6] 接收成功: %s\r\n", resp);

    float hz = 0.0f;
    int perr = parse_freq_response(resp, &hz);
    if ((perr != 0) || (hz == 0.0f)) {
        printf("无效频率响应，解析失败: err=%d, 原始: %s\r\n", perr, resp);
        return SENSOR_COMM_TIMEOUT;
    }

    *frequency_out = (uint32_t)hz;   // Hz
    return NO_ERROR;
}

// 读取密度、温度
int DSM_Read_Frequency_Density_Temp(float *frequency, float *density, float *temp) {
    int ret = NO_ERROR;
    char resp[RX_BUF_LEN];

    ret = UART6_SendWithRetry("Cd\r\n", resp, RX_BUF_LEN, 500);
    if (ret == 0) {
        // 格式: F+0000.0D+000.00T+19.570P
        if ((resp[0] == 'E') || (resp[0] == 'F')) {
            char *pD = strchr(resp, 'D');
            char *pT = strchr(resp, 'T');

            if (pD || pT) {
                *frequency = (float)atof(resp + 1);
                *density   = (float)atof(pD + 1);
                *temp      = (float)atof(pT + 1);
                return NO_ERROR;
            }
        }
    }
    return ret;
}

void Sensor_Test(void) {
    float density, viscosity, temp;

    // 读取密度、温度
    if (Read_Density(&density, &viscosity, &temp) == 0) {
        printf("密度: %.3f  粘度: %.3f  温度: %.3f ℃\r\n", density, viscosity, temp);
        g_measurement.single_point_monitoring.density = density / 10.0f;
        g_measurement.single_point_monitoring.temperature = temp;
        g_measurement.single_point_monitoring.temperature_position = g_measurement.debug_data.sensor_position;
        g_measurement.single_point_measurement.density = density / 10.0f;
        g_measurement.single_point_measurement.temperature = temp;
        g_measurement.single_point_measurement.temperature_position = g_measurement.debug_data.sensor_position;
    } else {
        printf("读取密度/温度失败！\r\n");
    }
}

/**
 * @func: CalculationBCC_DSM
 * @description: BCC校验
 * @param command 待校验数组：编码值缓存区
 * @param count   数组长度
 * @return 返回BCC校验码
 */
static char CalculationBCC_DSM(char command[], int count) {
    char i, bcc;
    bcc = command[0];
    for (i = 1; i < count - 1; i++) {
        bcc = (char)(bcc ^ command[i + 1]);
    }
    return bcc;
}

// 读取振动管编号（CN 指令）
uint32_t Read_VibrationTube_ID(char *id_out, size_t id_out_size)
{
    if ((id_out == NULL) || (id_out_size == 0)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    char resp[RX_BUF_LEN] = {0};

    // 发送 CN 指令
    uint32_t ret = UART6_SendWithRetry("CN\r\n", resp, RX_BUF_LEN, 500);
    if (ret != 0) {
        printf("[UART6] 发送 CN 指令或接收超时\r\n");
        return SENSOR_COMM_TIMEOUT;
    }

    printf("[UART6] CN 响应: %s\r\n", resp);

    // 解析响应: 去掉前导空白
    char *p = resp;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        p++;
    }

    // 按协议，一般以 'N' 开头，例如 N2009924H
    if (*p != 'N') {
        printf("振动管ID响应格式错误，未以 'N' 开头，原始: %s\r\n", resp);
        return SENSOR_COMM_TIMEOUT;
    }

    // 找到行尾 / 结束符（遇到 CR/LF/* 就停）
    char *end = p;
    while (*end != '\0' && *end != '\r' && *end != '\n' && *end != '*') {
        end++;
    }

    size_t id_len = (size_t)(end - p);
    if (id_len == 0) {
        printf("振动管ID长度为 0，原始: %s\r\n", resp);
        return SENSOR_COMM_TIMEOUT;
    }

    // 拷贝到输出缓冲区，确保以 '\0' 结尾
    if (id_len >= id_out_size) {
        id_len = id_out_size - 1;   // 截断，避免越界
    }
    memcpy(id_out, p, id_len);
    id_out[id_len] = '\0';

    printf("振动管ID: %s\r\n", id_out);

    return NO_ERROR;
}
