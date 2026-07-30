/*
 * dsm_sensor_communication.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */
#include "dsm_sensor_communication.h"
#include "error_log.h"
#include "system_parameter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <inttypes.h>

DSMSENSOR_DATA dsmsensor_data; /* 最近一次 DSM 传感器有效响应解码后的共享测量数据。 */

char DSMCommand[RCVBUFFLEN]; /* 兼容旧 DSM 文本收发接口保留的全局命令缓冲区；当前本文件收发实现未读写该数组，不能据此判断正在发送的命令。 */
char DSMRcvBuffer[RCVBUFFLEN]; /* 兼容旧 DSM 文本收发接口保留的全局接收缓冲区；当前 DMA 接收使用 s_uart6_dma_rx_buf，不能把本数组当作最新应答。 */
int DSMRcvLen; /* 兼容旧 DSM 文本收发接口保留的全局接收长度；当前本文件未更新该值，实际长度由局部输出参数返回。 */

static char CalculationBCC_DSM(char command[], int count);

/**
 * @brief 在限定时间内排空 UART6 残留数据，连续空闲后结束。
 *
 * @param idle_ms 进入 AT 操作前要求 UART6 连续无数据的空闲时间，单位 ms。
 */
static void UART6_DrainRX_UntilIdle(uint32_t idle_ms)
{
    uint8_t dump;
    uint32_t last = HAL_GetTick();

    for (;;) {
        if (HAL_UART_Receive(&huart6, &dump, 1, 1) == HAL_OK) {
            last = HAL_GetTick();
        } else if ((HAL_GetTick() - last) >= idle_ms) {
            break;
        }
    }

    __HAL_UART_CLEAR_OREFLAG(&huart6);
}

/* DSM 文本错误响应到统一故障码的映射项；按明确响应关键字保留远端错误原因。 */
typedef struct {
    /* DSM 文本错误关键字到统一故障码的映射。 */
    const char *response_text; /* DSM 远端错误响应的匹配关键字。 */
    uint32_t error_code; /* 匹配该 DSM 响应关键字时上报的完整统一故障码。 */
} DsmResponseErrorMap;

/* DSM 特殊返回值沿用一代子码语义，二代只把类别 3 平移为 13。 */
static const DsmResponseErrorMap s_dsm_response_errors[] = {
    {"A+111.11B+111.11", SENSOR_NO_RESONANCE},
    {"A+222.22B+222.22", SENSOR_POWER_SUPPLY_ERROR},
    {"A+333.33B+333.33", SENSOR_GYRO_COMM_TIMEOUT},
    {"A+444.44B+444.44", SENSOR_GYRO_ANGLE_ERROR},
    {"A+555.55B+555.55", SENSOR_SELF_TEST_FAILED},
    {"A+888.88B+888.88", SENSOR_INTERNAL_CPU_COMM_TIMEOUT},
    {"A+999.99B+999.99", SENSOR_INTERNAL_COMM_CHECK_ERROR}
};
/* DSM 响应错误映射表条目数；由 s_dsm_response_errors 数组长度自动推导。 */
#define DSM_RESPONSE_ERROR_COUNT (sizeof(s_dsm_response_errors) / sizeof(s_dsm_response_errors[0]))

/**
 * @brief 识别并记录 DSM 低电压告警文本帧。
 *
 * @param resp 已经接收完成、等待识别或解析的 DSM 响应文本。
 */
static void DSM_LogLowVoltageFrame(const char *resp)
{
    if ((resp != NULL) && ((resp[0] == 'E') || (resp[0] == 'e'))) {
        printf("DSM传感器电压过低\r\n");
    }
}

/**
 * @brief 把 DSM 特殊响应转换为可直接上报的二代故障码。
 *
 * @param resp 已经接收完成、等待识别或解析的 DSM 响应文本。
 * @return NO_ERROR 表示响应不是已知 DSM 错误文本；命中错误表时返回该文本对应的二代传感器错误码。
 */
static uint32_t DSM_MapErrorResponse(const char *resp)
{
    if ((resp == NULL) || (resp[0] == 'E') || (resp[0] == 'e')) {
        return NO_ERROR;
    }
    for (uint32_t i = 0U; i < DSM_RESPONSE_ERROR_COUNT; i++) {
        if (strstr(resp, s_dsm_response_errors[i].response_text) != NULL) {
            return s_dsm_response_errors[i].error_code;
        }
    }
    return NO_ERROR;
}

#define DSM_UART_MAX_RETRY UART6_COMM_MAX_RETRY /* 传感器通信参数：传感器 UART 最大值 重试。 */
static uint32_t s_uart6_last_error = HAL_UART_ERROR_NONE; /* 最近一次 DSM UART6 HAL 硬件错误位快照；在 DMA 等待或应答收尾阶段锁存，供链路诊断输出 ORE、FE、NE、PE 等根因。 */
static uint8_t s_uart6_dma_rx_buf[RX_BUF_LEN]; /* DSM 文本命令单次 UART6 DMA 接收缓冲区；启动前清零，DMA 计数器换算实际长度后再复制到调用方响应缓存。 */

/**
 * @brief 停止 UART6 单次 DMA 接收并清理硬件错误状态。
 *
 * DSM 文本协议每次命令只等待一帧应答，退出等待前必须停止 DMA，避免下一次命令接收沿用旧 DMA 状态。
 */
static void UART6_StopDmaReceive(void)
{
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
 * @brief 读取当前 UART6 DMA 已接收字节数。
 *
 * DMA 计数器表示剩余空间，这里转换为已收长度；若句柄异常则返回 0，交给上层按超时处理。
 *
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @return 返回当前 UART6 DMA 已接收字节数的有效长度，单位字节；0 表示没有可供消费的数据。
 */
static uint16_t UART6_GetDmaReceivedLength(uint16_t rx_len)
{
    uint16_t remain;

    if (huart6.hdmarx == NULL) {
        return 0U;
    }
    remain = (uint16_t)__HAL_DMA_GET_COUNTER(huart6.hdmarx);
    if (remain > rx_len) {
        return 0U;
    }
    return (uint16_t)(rx_len - remain);
}

/**
 * @brief 等待 UART6 DMA 发送完成。
 *
 * 接收 DMA 已提前启动，发送阶段只等待 TX 状态回到 READY；若命令切换或超时，停止 DMA 并向上返回对应状态。
 *
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示 DMA 发送已完成；等待期间收到新命令返回 STATE_SWITCH，HAL 错误或 timeout 到期返回 COMM_UART_TRANSFER_ERROR。
 */
static uint32_t UART6_WaitTransmitDmaDone(uint32_t timeout)
{
    uint32_t startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeout) {
        if (HasEffectiveCommandSwitchRequest()) {
            UART6_StopDmaReceive();
            return STATE_SWITCH;
        }
        if (huart6.gState == HAL_UART_STATE_READY) {
            return NO_ERROR;
        }
        /* 等待发送 DMA 完成期间发现 UART6 硬件错误时先锁存 HAL 错误位并停止 DMA，避免下一事务继承故障状态。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            s_uart6_last_error = huart6.ErrorCode;
            UART6_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }
        /* 发送 DMA 等待循环每 1 ms 让出一次 CPU，避免在超时窗口内持续忙等。 */
        HAL_Delay(1);
    }

    UART6_StopDmaReceive();
    return COMM_UART_TRANSFER_ERROR;
}

/**
 * @brief 启动 DSM 文本应答的 UART6 DMA 接收。
 *
 * 该函数只启动接收 DMA，不等待帧完成；调用方会先发送命令，再进入终止符等待阶段。
 *
 * @param maxLen 调用方接收缓冲区的最大容量，单位字节。
 * @param dma_len_out 用于返回本次实际启动的 UART6 DMA 接收长度，单位字节。
 * @return NO_ERROR 表示已按缓冲区容量启动 UART6 DMA 接收并写回 dma_len_out；容量或输出指针无效返回 SENSOR_RESP_FORMAT_ERROR，HAL 启动失败返回 COMM_UART_TRANSFER_ERROR。
 */
static uint32_t UART6_StartTextReceiveDma(uint16_t maxLen, uint16_t *dma_len_out)
{
    uint16_t dma_len;

    if ((dma_len_out == NULL) || (maxLen < 2U)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    dma_len = (uint16_t)(maxLen - 1U);
    if (dma_len > (uint16_t)sizeof(s_uart6_dma_rx_buf)) {
        dma_len = (uint16_t)sizeof(s_uart6_dma_rx_buf);
    }
    memset(s_uart6_dma_rx_buf, 0, sizeof(s_uart6_dma_rx_buf));
    *dma_len_out = dma_len;

    UART6_StopDmaReceive();
    if (HAL_UART_Receive_DMA(&huart6, s_uart6_dma_rx_buf, dma_len) != HAL_OK) {
        UART6_StopDmaReceive();
        return COMM_UART_TRANSFER_ERROR;
    }
    return NO_ERROR;
}

/**
 * @brief 等待 DSM 文本应答帧接收完成。
 *
 * 接收 DMA 在发送前已经启动；这里持续扫描 DMA 缓冲区，遇到换行终止符后复制完整帧给调用方。
 *
 * @param response UART6 响应文字缓冲区；接收函数按 response_size 限制写入并保证 NUL 结尾，格式化函数只读有效内容。
 * @param dma_len 用于返回 UART6 DMA 实际接收字节数；仅成功或已取得部分数据的路径有效。
 * @param recv_len_out 用于返回本次收到的有效 DSM 文本应答长度，单位字节。
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示已收到非空、未溢出的 NUL 结尾文本；参数或长度不合法返回 SENSOR_RESP_FORMAT_ERROR，命令切换返回 STATE_SWITCH，HAL 错误返回 COMM_UART_TRANSFER_ERROR，无数据超时返回 SENSOR_DEVICE_COMM_TIMEOUT，仅有不完整数据时返回 SENSOR_RESP_FORMAT_ERROR。
 */
static uint32_t UART6_WaitTextReceiveDma(char *response,
                                         uint16_t dma_len,
                                         uint16_t *recv_len_out,
                                         uint32_t timeout)
{
    uint32_t startTick;

    if ((response == NULL) || (dma_len == 0U)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < timeout) {
        uint16_t recvLen = UART6_GetDmaReceivedLength(dma_len);

        if (HasEffectiveCommandSwitchRequest()) {
            UART6_StopDmaReceive();
            return STATE_SWITCH;
        }

        /* 接收文本应答期间发现 UART6 硬件错误时立即停止 DMA；已有部分字节也不得继续按完整协议帧解析。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            s_uart6_last_error = huart6.ErrorCode;
            UART6_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }

        for (uint16_t i = 0U; i < recvLen; i++) {
            if (s_uart6_dma_rx_buf[i] == 0x0AU) {
                uint16_t frameLen = (uint16_t)(i + 1U);
                memcpy(response, s_uart6_dma_rx_buf, frameLen);
                response[frameLen] = '\0';
                if (recv_len_out != NULL) {
                    *recv_len_out = frameLen;
                }
                UART6_StopDmaReceive();
                return NO_ERROR;
            }
        }

        if (recvLen >= dma_len) {
            memcpy(response, s_uart6_dma_rx_buf, recvLen);
            response[recvLen] = '\0';
            if (recv_len_out != NULL) {
                *recv_len_out = recvLen;
            }
            UART6_StopDmaReceive();
            return SENSOR_RESP_FORMAT_ERROR;
        }
        /* 文本应答轮询每 1 ms 检查一次 DMA 进度、命令切换和 UART 错误，避免空转占满前台。 */
        HAL_Delay(1);
    }

    {
        uint16_t recvLen = UART6_GetDmaReceivedLength(dma_len);
        if (recvLen > 0U) {
            memcpy(response, s_uart6_dma_rx_buf, recvLen);
            response[recvLen] = '\0';
        }
        if (recv_len_out != NULL) {
            *recv_len_out = recvLen;
        }
        UART6_StopDmaReceive();
        return (recvLen == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
    }
}
/**
 * @brief 取走并清理 UART6 硬件错误标志。
 *
 * HAL_UART_Receive 超时本身不算故障；只有 ErrorCode 非空时才表示 ORE/FE/NE 等硬件异常。
 * 这里记录最近一次硬件错误，随后清 ORE 和 ErrorCode，避免错误标志挂住后续收包。
 *
 * @return 返回并清除此前由 UART6 错误中断锁存的 HAL 错误位图；没有待处理错误时返回 0。
 */
static uint32_t UART6_TakeHardwareError(void)
{
    uint32_t error = huart6.ErrorCode;

    /* 读取到 UART6 硬件错误后清除 ORE、保存原始 HAL 错误位并复位句柄，让调用方取得一次性故障快照。 */
    if (error != HAL_UART_ERROR_NONE) {
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        s_uart6_last_error = error;
        huart6.ErrorCode = HAL_UART_ERROR_NONE;
    }
    return error;
}

/**
 * @brief 生成 UART6 异常重试日志详情。
 *
 * 详情中保留命令、实际接收长度、UART 硬件错误标志和原始 HEX，便于现场区分超时、短帧、BCC 错误和硬件溢出。
 *
 * @param prefix 附加在十六进制字节串前的日志字段名称，例如接收或残留数据。
 * @param cmd 准备通过 UART6 发送、重试或写入诊断详情的 NUL 结尾传感器命令文字。
 * @param response UART6 响应文字缓冲区；接收函数按 response_size 限制写入并保证 NUL 结尾，格式化函数只读有效内容。
 * @param recv_len 待格式化接收数据的有效长度，单位字节。
 * @param uart_error UART故障。
 * @param detail 错误或诊断记录使用的详细信息。
 * @param detail_size 详情。
 */
static void UART6_FormatHexDetail(const char *prefix,
                                  const char *cmd,
                                  const char *response,
                                  uint16_t recv_len,
                                  uint32_t uart_error,
                                  char *detail,
                                  size_t detail_size)
{
    int used;
    uint16_t i;

    if ((detail == NULL) || (detail_size == 0U)) {
        return;
    }

    used = snprintf(detail, detail_size,
                    "%s,cmd=%s,len=%u,uart=0x%08lX,hex=",
                    (prefix != NULL) ? prefix : "UART6",
                    (cmd != NULL) ? cmd : "",
                    (unsigned int)recv_len,
                    (unsigned long)uart_error);
    if (used < 0) {
        detail[0] = '\0';
        return;
    }
    if ((size_t)used >= detail_size) {
        detail[detail_size - 1U] = '\0';
        return;
    }

    for (i = 0; (i < recv_len) && ((size_t)used + 4U < detail_size); i++) {
        used += snprintf(&detail[used],
                         detail_size - (size_t)used,
                         "%02X ",
                         (unsigned int)(uint8_t)response[i]);
    }
}

/**
 * @brief 检查 WaterSendPack 中 7 字节数值字段是否只包含数字和最多一个小数点。
 *
 * 该检查用于阻止短帧或错位帧绕过 BCC 后被 atof() 当成有效电容值。
 *
 * @param value 用于返回通过七位数字格式校验的整数值。
 * @return true 表示 value 非空，前 7 字节全部为十进制数字且最多含一个小数点；false 表示指针为空、出现其他字符，或小数点超过一个。
 */
static bool DSM_IsSevenDigitValue(const char *value)
{
    uint8_t dot_count = 0U;

    if (value == NULL) {
        return false;
    }
    for (uint8_t i = 0U; i < 7U; i++) {
        if (value[i] == '.') {
            dot_count++;
        } else if (!isdigit((unsigned char)value[i])) {
            return false;
        }
    }
    return dot_count <= 1U;
}

/**
 * @brief 串口发送并接收（带调试打印）。
 *
 * @param cmd 准备通过 UART6 发送、重试或写入诊断详情的 NUL 结尾传感器命令文字。
 * @param response UART6 响应文字缓冲区；接收函数按 response_size 限制写入并保证 NUL 结尾，格式化函数只读有效内容。
 * @param maxLen 调用方接收缓冲区的最大容量，单位字节。
 * @param recv_len_out 用于返回本次收到的有效 DSM 文本应答长度，单位字节。
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示命令发送、文本接收和 BCC 校验全部成功；其他值为接收启动或等待错误、UART6 发送错误、无响应超时、响应格式错误或 SENSOR_BCC_ERROR。
 */
static int UART6_SendCommand(const char *cmd,
                             char *response,
                             uint16_t maxLen,
                             uint16_t *recv_len_out,
                             uint32_t timeout) {
    char bcc;
    uint32_t uart_error;
    memset(response, 0, maxLen);
    uint16_t recvLen = 0;
#if DEBUG_UART6
    printf("[UART6] 发送: %s\n", cmd);
#endif

    s_uart6_last_error = HAL_UART_ERROR_NONE;
    UART6_DrainRX_UntilIdle(5);

    /* 先启动接收 DMA，再启动发送 DMA，避免从机快速回包时丢掉应答开头。 */
    uint16_t dma_len = 0U;
    uint32_t rx_ret = UART6_StartTextReceiveDma(maxLen, &dma_len);
    if (rx_ret != NO_ERROR) {
        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        return (int)rx_ret;
    }

    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t*)cmd, (uint16_t)strlen(cmd)) != HAL_OK) {
#if DEBUG_UART6
        printf("[UART6] 发送失败！\n");
#endif
        UART6_StopDmaReceive();
        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        return COMM_UART_TRANSFER_ERROR;
    }

    rx_ret = UART6_WaitTransmitDmaDone(100);
    if (rx_ret != NO_ERROR) {
        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        return (int)rx_ret;
    }

    /* 接收 DMA 已在发送前启动，这里只等待换行终止符确认帧边界。 */
    rx_ret = UART6_WaitTextReceiveDma(response, dma_len, &recvLen, timeout);
    if (rx_ret != NO_ERROR) {
        if (recv_len_out != NULL) {
            *recv_len_out = recvLen;
        }
        return (int)rx_ret;
    }
    if (recv_len_out != NULL) {
        *recv_len_out = recvLen;
    }

    /* 响应校验前再取一次硬件错误，避免最后一字节后留下 ORE/FE/NE 却继续解析。 */
    uart_error = UART6_TakeHardwareError();
    /* 应答校验前最后一次硬件检查若发现 ORE、FE、NE 或 PE，即使缓冲区已有字节也按传输失败返回。 */
    if (uart_error != HAL_UART_ERROR_NONE) {
        return COMM_UART_TRANSFER_ERROR;
    }
    if (recvLen == 0) {
        return SENSOR_DEVICE_COMM_TIMEOUT;
    }
    if (recvLen < 3) {
        return SENSOR_RESP_FORMAT_ERROR;
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

    /* 校验 BCC */
    bcc = CalculationBCC_DSM(response, (recvLen - 3));
    if (bcc == response[recvLen - 3]) {
#if DEBUG_UART6
        printf("DSM: 接收BCC校验通过！\r\n");
#endif
        return NO_ERROR;
    } else {
        return SENSOR_BCC_ERROR; /* 校验失败 */
    }
}

/**
 * @brief 发送指令，带重试机制。
 *
 * @param cmd 准备通过 UART6 发送、重试或写入诊断详情的 NUL 结尾传感器命令文字。
 * @param response UART6 响应文字缓冲区；接收函数按 response_size 限制写入并保证 NUL 结尾，格式化函数只读有效内容。
 * @param maxLen 调用方接收缓冲区的最大容量，单位字节。
 * @param recv_len_out 用于返回本次收到的有效 DSM 文本应答长度，单位字节。
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示某次尝试完成有效收发；命令切换立即返回 STATE_SWITCH，全部重试失败时返回最后一次 UART6、超时、格式或 BCC 错误。
 */
static int UART6_SendWithRetry(const char *cmd,
                               char *response,
                               uint16_t maxLen,
                               uint16_t *recv_len_out,
                               uint32_t timeout) {
    uint32_t ret = SENSOR_DEVICE_COMM_TIMEOUT;
    uint16_t recvLen = 0;
    char detail[160];
    for (int i = 0; i < DSM_UART_MAX_RETRY; i++) {
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        if (i > 0) {
            /* 重试发送前等待 DSM_PRE_SEND_DELAY，让上一事务的尾字节和传感器处理状态结束后再发新命令。 */
            HAL_Delay(DSM_PRE_SEND_DELAY);
        }
        ret = UART6_SendCommand(cmd, response, maxLen, &recvLen, timeout);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不进入 DSM 通信重试日志。 */
            return STATE_SWITCH;
        }
        if (ret == 0) {
            uint32_t response_error;

            DSM_LogLowVoltageFrame(response);
            response_error = DSM_MapErrorResponse(response);
            if (response_error == NO_ERROR) {
                if (recv_len_out != NULL) {
                    *recv_len_out = recvLen;
                }
                if (i > 0) {
                    ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                     ERROR_LOG_OP_READ_LEVEL,
                                     ERROR_LOG_REASON_COMM_FAIL,
                                     (uint32_t)(i + 1),
                                     DSM_UART_MAX_RETRY);
                }
                return NO_ERROR;
            }

            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_READ_LEVEL,
                           ErrorLog_GetReasonByCode(response_error),
                           (uint32_t)(i + 1),
                           DSM_UART_MAX_RETRY,
                           response_error);
            ret = response_error;
        } else {
            /* 重试日志带上原始帧和 UART 错误标志，现场可直接判断失败类型。 */
            UART6_FormatHexDetail(ErrorLog_GetReasonByCode(ret),
                                  cmd,
                                  response,
                                  recvLen,
                                  s_uart6_last_error,
                                  detail,
                                  sizeof(detail));
            ErrorLog_RetryDetail(ERROR_LOG_MODULE_SENSOR,
                                 ERROR_LOG_OP_READ_LEVEL,
                                 ErrorLog_GetReasonByCode(ret),
                                 (uint32_t)(i + 1),
                                 DSM_UART_MAX_RETRY,
                                 ret,
                                 detail);
        }
    }
    return ret;
}

/**
 * @brief 读取传感器电压。
 *
 * @param voltage_out 用于返回传感器供电电压。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t Read_Sensor_Voltage(float *voltage_out) {
    uint32_t ret;
    char resp[RX_BUF_LEN];

    if (voltage_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = UART6_SendWithRetry("CK", resp, RX_BUF_LEN, NULL, 500);
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("[UART6] 接收成功: %s\r\n", resp);
    if ((resp[0] == 'E') || (resp[0] == 'e')) {
        *voltage_out = (float)atof(resp + 1);
        return NO_ERROR;
    }

    printf("无效电压响应: %x\r\n", resp[0]);
    return SENSOR_RESP_FORMAT_ERROR;
}

/**
 * @brief 开启测水探针 (CL 命令)。
 *
 * @return NO_ERROR 表示 CL 应答已收到并通过内容校验；应答内容不匹配返回 SENSOR_RESP_FORMAT_ERROR，收发失败返回 UART6_SendWithRetry 的具体错误。
 */
int Probe_EnableWaterSensor(void) {
    char resp[RX_BUF_LEN];
    uint32_t ret;

    /* 发送命令 "CL\r\n" 并带 3 次重试 */
    ret = UART6_SendWithRetry("CL", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        printf("[探针] 开启测水探针响应: %s\r\n", resp);

        /* 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功 */
        if (strstr(resp, "%") != NULL) {
            printf("[探针] 测水探针开启成功！\r\n");
            return NO_ERROR;
        } else {
            printf("[探针] 无效响应: %s\r\n", resp);
            return SENSOR_RESP_FORMAT_ERROR;
        }
    } else {
        return ret;
    }
}

/**
 * @brief 开启液位模式。
 *
 * @return NO_ERROR 表示液位模式命令应答已确认；回显内容不匹配返回 SENSOR_RESP_FORMAT_ERROR，收发失败返回最后一次具体通信错误。
 */
int DSM_EnableLevelMode(void) {
    char resp[RX_BUF_LEN];
    uint32_t ret;

    /* 发送命令 "CB\r\n" 并带 3 次重试 */
    ret = UART6_SendWithRetry("CB", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        printf("[液位模式] 开启液位模式响应: %s\r\n", resp);

        /* 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功 */
        if (strstr(resp, "%") != NULL) {
            printf("[液位模式] 开启成功！\r\n");
            return NO_ERROR;
        } else {
            printf("[液位模式] 无效响应: %s\r\n", resp);
            return SENSOR_RESP_FORMAT_ERROR;
        }
    } else {
        return ret;
    }
}

/**
 * @brief 开启密度模式。
 *
 * @return NO_ERROR 表示密度模式命令应答已确认；回显内容不匹配返回 SENSOR_RESP_FORMAT_ERROR，收发失败返回最后一次具体通信错误。
 */
int DSM_EnableDensityMode(void) {
    char resp[RX_BUF_LEN];
    uint32_t ret;

    /* 发送命令 "CD\r\n" 并带 3 次重试 */
    ret = UART6_SendWithRetry("CD", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        printf("[密度模式] 开启密度模式响应: %s\r\n", resp);

        /* 协议约定：如果返回包含 "%" 或其他成功标识，就认为成功 */
        if (strstr(resp, "%") != NULL) {
            printf("[密度模式] 开启成功！\r\n");
            return NO_ERROR;
        } else {
            printf("[密度模式] 无效响应: %s\r\n", resp);
            return SENSOR_RESP_FORMAT_ERROR;
        }
    } else {
        return ret;
    }
}

/**
 * @brief 工具函数: 解析 "E06.6379V\r\n" 这类响应为浮点数。
 *
 * @param resp 已经接收完成、等待识别或解析的 DSM 响应文本。
 * @param out_hz 用于返回解析后频率的输出参数，单位 Hz。
 * @return 0 表示已跳过前导非数字字符并解析出有限频率；参数为空返回 SYSTEM_CALL_CONDITION_ERROR，-2 表示整段文本没有数字或正负号起点，-3 表示 strtod 未能解析任何数字，-4 表示结果为 NaN 或无穷。
 */
static int parse_freq_response(const char *resp, float *out_hz)
{
    if (!resp || !out_hz) return SYSTEM_CALL_CONDITION_ERROR;

    /* 1) 跳过起始标志（例如 'E'）和前导空白 */
    const char *p = resp;
    while (*p && !isdigit((unsigned char)*p) && *p != '-' && *p != '+') {
        ++p;
    }
    if (!*p) return -2;

    /* 2) 使用 strtod 解析到非数字处（会自动停在 'V' 或回车） */
    char *endp = NULL;
    double v = strtod(p, &endp);
    if (endp == p) return -3;   /* 没解析到数字 */
    if (!isfinite(v)) return -4;

    *out_hz = (float)v;
    return 0;
}

/**
 * @brief 读取液位跟随频率（单次）。
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t Read_Level_Frequency(uint32_t *frequency_out)
{
    if (!frequency_out) return SYSTEM_CALL_CONDITION_ERROR;

    char resp[RX_BUF_LEN] = {0};
    uint32_t ret = UART6_SendWithRetry("Cb", resp, RX_BUF_LEN, NULL, 500);
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("[UART6] 接收成功: %s\r\n", resp);

    float hz = 0.0f;
    int perr = parse_freq_response(resp, &hz);
    if (perr != 0) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *frequency_out = (uint32_t)hz;   /* Hz */
    return NO_ERROR;
}

/**
 * @brief 发送 DSM Cd 命令并解析同一应答中的频率、密度和温度。
 *
 * @param frequency DSM 传感器振动频率输出指针，成功时写入协议定义的 Hz 浮点值。
 * @param density DSM 传感器密度输出指针，成功时写入 kg/m3 浮点值。
 * @param temp 用于返回传感器温度的输出参数，单位 ℃。
 * @return NO_ERROR 表示三项数据均已解析；SYSTEM_CALL_CONDITION_ERROR 表示输出指针无效；SENSOR_RESP_FORMAT_ERROR
 *         表示应答字段不完整；其他值为 UART6 发送或接收错误码。
 */
int DSM_Read_Frequency_Density_Temp(float *frequency, float *density, float *temp) {
    if (!frequency || !density || !temp) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    int ret = NO_ERROR;
    char resp[RX_BUF_LEN];

    ret = UART6_SendWithRetry("Cd", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        /* 格式: F+0000.0D+000.00T+19.570P */
        if ((resp[0] == 'E') || (resp[0] == 'F')) {
            char *pD = strchr(resp, 'D');
            char *pT = strchr(resp, 'T');

            if (pD && pT) {
                *frequency = (float)atof(resp + 1);
                *density   = (float)atof(pD + 1);
                *temp      = (float)atof(pT + 1);
                return NO_ERROR;
            }
        }
        return SENSOR_RESP_FORMAT_ERROR;
    }
    return ret;
}


/**
 * @brief 按现有 DSM 帧约定计算 BCC 异或校验码。
 *
 * 算法以 command[0] 为初值，并依次异或 command[2] 至 command[count-1]；command[1] 不参与当前协议校验。
 *
 * @param command 包含待校验 DSM 帧数据的只读字节数组。
 * @param count command 中按当前 BCC 约定参与索引计算的字节数量。
 * @return 返回按 DSM 当前异或范围计算得到的 1 字节 BCC 校验码。
 * @note 调用方必须保证 command 至少包含 count 个可访问字节；本函数不执行空指针和长度检查。
 */
static char CalculationBCC_DSM(char command[], int count) {
    char i, bcc;
    bcc = command[0];
    for (i = 1; i < count - 1; i++) {
        bcc = (char)(bcc ^ command[i + 1]);
    }
    return bcc;
}

/**
 * @brief 读取振动管编号（CN 指令）。
 *
 * @param id_out 用于返回 CN 应答中的振动管编号。
 * @param id_out_size 编号。该值实际表示振动管 ID 输出缓冲区容量，单位字节；写入时为末尾 NUL 预留一个字节。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t Read_VibrationTube_ID(char *id_out, size_t id_out_size)
{
    if ((id_out == NULL) || (id_out_size == 0)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    char resp[RX_BUF_LEN] = {0};

    /* 发送 CN 指令 */
    uint32_t ret = UART6_SendWithRetry("CN", resp, RX_BUF_LEN, NULL, 500);
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("[UART6] CN 响应: %s\r\n", resp);

    /* 解析响应: 去掉前导空白 */
    char *p = resp;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        p++;
    }

    /* CN 低电压响应可能以 E/e 开头，但仍携带编号数字，继续解析。 */
    if ((*p != 'N') && (*p != 'E') && (*p != 'e')) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    /* 找到行尾 / 结束符（遇到 CR/LF/ * 就停） */
    char *end = p;
    while (*end != '\0' && *end != '\r' && *end != '\n' && *end != '*') {
        end++;
    }

    size_t id_len = (size_t)(end - p);
    if (id_len == 0) {
        printf("振动管ID长度为 0，原始: %s\r\n", resp);
        return SENSOR_RESP_FORMAT_ERROR;
    }

    /* 拷贝到输出缓冲区，确保以 '\0' 结尾 */
    if (id_len >= id_out_size) {
        id_len = id_out_size - 1;   /* 截断，避免越界 */
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
        return SYSTEM_CALL_CONDITION_ERROR;   /* 你工程里若叫 SYSTEM_CALL_CONDITION_ERROR/PARAM_RANGE_ERROR 请替换 */
    }

    char resp[RX_BUF_LEN] = {0};
    uint16_t recv_len = 0;
    uint32_t ret = UART6_SendWithRetry("Cl", resp, RX_BUF_LEN, &recv_len, 500);
    if (ret != NO_ERROR) {
        return ret;
    }

    if (recv_len != 11U) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    /* 格式检查：起始必须是 D 或 E，且以 \r\n 结束 */
    if (!((resp[0] == 'D') || (resp[0] == 'E'))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (!(resp[9] == '\r' && resp[10] == '\n')) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (!DSM_IsSevenDigitValue(&resp[1])) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    /* BCC 校验：WaterSendPack 的 BCC 在 resp[8]，覆盖 resp[0..7]。 */
    char bcc = CalculationBCC_DSM(resp, 8);
    if (bcc != resp[8]) {
        return SENSOR_BCC_ERROR;
    }
    /* resp[0]=='E' 只提示传感器电压过低，不影响水位电容值解析。 */

    /* 解析数值：resp[1..7] 是数字/小数点字符串。直接 atof(resp+1) 即可 */
    *cap_out = (float)atof(resp + 1);
	if (*cap_out < 30.0f) {
		*cap_out = 99999.9;
	}
    return NO_ERROR;
}
/**
 * @brief 解析形如 "+0040.1" 或 "-12.3" 的浮点数，p 会自动跳过到数字起始。
 *
 * @param tag_pos 指向 DSM 响应字段标签的只读文本位置。
 * @param out_val 用于返回从标签后解析出的浮点值。
 * @return 0 表示浮点值解析成功；-1 表示输入或输出指针为空，-2 表示标签后没有数值，-3 表示数值转换失败。
 */
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
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    char resp[RX_BUF_LEN] = {0};
    uint32_t ret = UART6_SendWithRetry("Ch", resp, RX_BUF_LEN, NULL, 500);
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
        return SENSOR_RESP_FORMAT_ERROR;
    }

    float ax = 0.0f, ay = 0.0f;
    int ea = dsm_parse_float_after_tag(pA, &ax);
    int eb = dsm_parse_float_after_tag(pB, &ay);

    if (ea != 0 || eb != 0) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *angle_x_deg = ax;
    *angle_y_deg = ay;
    g_measurement.debug_data.angle_x = ax*100;
    g_measurement.debug_data.angle_y = ay*100;
    return NO_ERROR;
}

