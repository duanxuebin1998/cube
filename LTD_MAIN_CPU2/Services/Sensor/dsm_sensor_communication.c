/*
 * dsm_sensor_communication.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
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
#if DEBUG_UART6
    printf("[UART6] Send: %s\n", cmd);
#endif

    // 发送
    if (HAL_UART_Transmit(&huart6, (uint8_t*)cmd, strlen(cmd), 100) != HAL_OK) {
#if DEBUG_UART6
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
#if DEBUG_UART6
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
#if DEBUG_UART6
        printf("DSM: rcv BCC校验通过!\r\n");
#endif
        return NO_ERROR;
    } else {
#if DEBUG_UART6
        printf("DSM: rcv BCC校验失败!\r\n");
#endif
        return SENSOR_BCC_ERROR; // 校验失败
    }
}

// 发送指令，带3次重试机制
int UART6_SendWithRetry(const char *cmd, char *response, uint16_t maxLen, uint32_t timeout) {
    uint32_t ret;
    for (int i = 0; i < 10; i++) {
//    	//延时3ms
//    	HAL_Delay(3);
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
    ret = UART6_SendWithRetry("CK", resp, RX_BUF_LEN, 500);
    if (ret == NO_ERROR) {
        // 响应格式: E06.6379V
        printf("[UART6] 接收成功: %s\r\n", resp);
        if ((resp[0] == 'E') || (resp[0] == 'e')) {
            return (float)atof(resp + 1);
        } else {
            printf("无效电压响应: %x\r\n", resp[0]);
        }
    }
    return SENSOR_COMM_TIMEOUT; // 错误
}

// 开启测水探针 (CL 命令)
int Probe_EnableWaterSensor(void) {
    char resp[RX_BUF_LEN];

    // 发送命令 "CL\r\n" 并带 3 次重试
    if (UART6_SendWithRetry("CL", resp, RX_BUF_LEN, 500) == 0) {
        printf("[Probe] 开启测水探针响应: %s\r\n", resp);

        // 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功
        if (strstr(resp, "%") != NULL) {
            printf("[Probe] 测水探针开启成功！\r\n");
            return NO_ERROR;
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
    if (UART6_SendWithRetry("CB", resp, RX_BUF_LEN, 500) == 0) {
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
    if (UART6_SendWithRetry("CD", resp, RX_BUF_LEN, 500) == 0) {
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
    if (!resp || !out_hz) return PARAM_ERROR;

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
    uint32_t ret = UART6_SendWithRetry("Cb", resp, RX_BUF_LEN, 500);
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

    ret = UART6_SendWithRetry("Cd", resp, RX_BUF_LEN, 500);
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
    uint32_t ret = UART6_SendWithRetry("CN", resp, RX_BUF_LEN, 500);
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
uint32_t Read_Water_Capacitance(float *cap_out)
{
    if (cap_out == NULL) {
        return PARAM_ERROR;   // 你工程里若叫 PARAM_ADDRESS_OVERFLOW/PARAM_ERROR 请替换
    }

    char resp[RX_BUF_LEN] = {0};
    uint32_t ret = UART6_SendWithRetry("Cl", resp, RX_BUF_LEN, 500);
    if (ret != NO_ERROR) {
        return ret;
    }

//    /* 基本长度检查：WaterSendPack 固定 11 字节 */
//    size_t len = strlen(resp);
//    if (len < 11) {
//        /* 有些情况下 resp 里可能包含 '\0'（若你按字节收），建议你用 recvLen 更准；
//           在你现有 UART6_SendCommand 里目前没把 recvLen 返回出来，所以先用 strlen 做最低保障。 */
//        printf("[UART6] 电容响应长度异常: len=%u, resp=%s\r\n", (unsigned)len, resp);
//        return SENSOR_COMM_TIMEOUT; // 或 SENSOR_RESP_FORMAT_ERROR（更合理）
//    }

    /* 格式检查：起始必须是 D 或 E，且以 \r\n 结束 */
    if (!((resp[0] == 'D') || (resp[0] == 'E'))) {
        printf("[UART6] 电容响应头错误: 0x%02X, resp=%s\r\n", (unsigned char)resp[0], resp);
        return SENSOR_BCC_ERROR;
    }
//    if (!(resp[9] == '\r' && resp[10] == '\n')) {
//        printf("[UART6] 电容响应结尾错误: [%02X %02X]\r\n", (unsigned char)resp[9], (unsigned char)resp[10]);
//        return SENSOR_BCC_ERROR;
//    }

//    /* BCC 校验：按你的 WaterSendPack，BCC 在 resp[8]，覆盖 resp[0..7] */
//    char bcc = CalculationBCC_DSM(resp, 8);
//    if (bcc != resp[8]) {
//        printf("[UART6] 电容 BCC 校验失败: cal=%02X rcv=%02X\r\n", (unsigned char)bcc, (unsigned char)resp[8]);
//        return SENSOR_BCC_ERROR;
//    }

    /* 电压异常标志：resp[0]=='E' */
    if (resp[0] == 'E') {
        printf("[UART6] 电容响应提示：电压异常\r\n");
//        return SENSOR_POWER_ABNORMAL;   // 建议新增错误码；若你已有对应错误码则替换
    }

    /* 解析数值：resp[1..7] 是数字/小数点字符串。直接 atof(resp+1) 即可 */
    *cap_out = (float)atof(resp + 1);
	if (*cap_out < 30.0f) {
		*cap_out = 99999.9;
	}
    return NO_ERROR;
}
/* 解析形如 "+0040.1" 或 "-12.3" 的浮点数，p 会自动跳过到数字起始 */
static int dsm_parse_float_after_tag(const char *tag_pos, float *out_val)
{
    if (!tag_pos || !out_val) return -1;

    const char *p = tag_pos;

    /* 跳过标签字符本身（例如 'A' 或 'B'） */
    p++;

    /* 跳过非数字/符号字符 */
    while (*p && !isdigit((unsigned char)*p) && *p != '-' && *p != '+') {
        p++;
    }
    if (!*p) return -2;

    char *endp = NULL;
    double v = strtod(p, &endp);
    if (endp == p) return -3;

    *out_val = (float)v;
    return 0;
}
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
uint32_t Read_Gyro_Angle(float *angle_x_deg, float *angle_y_deg)
{
    if (!angle_x_deg || !angle_y_deg) {
        return PARAM_ERROR;
    }

    char resp[RX_BUF_LEN] = {0};
    uint32_t ret = UART6_SendWithRetry("Ch", resp, RX_BUF_LEN, 500);
    if (ret != NO_ERROR) {
        return ret;
    }

    /* 查找 A/B 标签 */
    char *pA = strchr(resp, 'A');
    if (!pA) {
        pA = strchr(resp, 'E');   /* ★ 兼容 E 作为 X 轴标签 */
    }
    char *pB = strchr(resp, 'B');

    if (!pA || !pB) {
        printf("[UART6] 陀螺仪响应格式错误: %s\r\n", resp);
        return SENSOR_COMM_TIMEOUT; /* 或建议新增 SENSOR_RESP_FORMAT_ERROR */
    }

    float ax = 0.0f, ay = 0.0f;
    int ea = dsm_parse_float_after_tag(pA, &ax);
    int eb = dsm_parse_float_after_tag(pB, &ay);

    if (ea != 0 || eb != 0) {
        printf("[UART6] 陀螺仪角度解析失败: ea=%d eb=%d, resp=%s\r\n", ea, eb, resp);
        return SENSOR_COMM_TIMEOUT; /* 或 SENSOR_RESP_FORMAT_ERROR */
    }

    *angle_x_deg = ax;
    *angle_y_deg = ay;
    g_measurement.debug_data.angle_x = ax*100;
    g_measurement.debug_data.angle_y = ay*100;
    return NO_ERROR;
}

