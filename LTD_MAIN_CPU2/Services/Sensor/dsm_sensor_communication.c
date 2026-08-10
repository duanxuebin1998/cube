/*
 * dsm_sensor_communication.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */
#include "dsm_sensor_communication.h"
#include "ch9141_at.h"
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
static DsmSessionContext s_dsm_session_context; /* 当前 DSM UART6 链路的 RAM-only 版本上下文。 */

char DSMCommand[RCVBUFFLEN]; /* 兼容旧 DSM 文本收发接口保留的全局命令缓冲区；当前本文件收发实现未读写该数组，不能据此判断正在发送的命令。 */
char DSMRcvBuffer[RCVBUFFLEN]; /* 兼容旧 DSM 文本收发接口保留的全局接收缓冲区；当前 DMA 接收使用 s_uart6_dma_rx_buf，不能把本数组当作最新应答。 */
int DSMRcvLen; /* 兼容旧 DSM 文本收发接口保留的全局接收长度；当前本文件未更新该值，实际长度由局部输出参数返回。 */

static char CalculationBCC_DSM(char command[], int count);
static bool DSM_IsSevenDigitInteger(const char *value);
static bool DSM_IsSevenDigitValue(const char *value);
static bool DSM_IsSignedSevenDigitValue(const char *value, uint8_t require_decimal);
static bool DSM_IsSignedSevenDigitInteger(const char *value);
static bool DSM_ParseSevenByteFloat(const char *value, float *result_out);
static uint32_t s_uart6_last_error = HAL_UART_ERROR_NONE; /* 最近一次 DSM UART6 事务锁存的 HAL 硬件错误位。 */

#define DSM_UART6_DRAIN_TOTAL_TIMEOUT_MS 100U /* DSM UART6 排空允许占用的总时限。 */
#define DSM_RESPONSE_ERROR_PAYLOAD_LEN 16U /* DSM 统一错误载荷长度，不含 BCC 和 CRLF。 */

typedef enum {
    DSM_FRAME_ACK = 0,
    DSM_FRAME_COMBINED_VERSION,
    DSM_FRAME_CPU1_VERSION,
    DSM_FRAME_SENSOR_NUMBER,
    DSM_FRAME_SUPPLY_VOLTAGE,
    DSM_FRAME_LEVEL_FREQUENCY,
    DSM_FRAME_DENSITY,
    DSM_FRAME_DENSITY_ANALYSIS,
    DSM_FRAME_GYRO_ANGLE,
    DSM_FRAME_WATER_CAPACITANCE
} DsmFrameType;

typedef struct {
    const char *command;
    uint16_t normal_len;
    uint16_t error_len;
    DsmFrameType frame_type;
    const char *operation_name;
} DsmCommandFrameSpec;

/* 生产命令和版本命令共用这一张响应契约表；未登记命令不得退回按 LF 截帧。 */
static const DsmCommandFrameSpec s_dsm_frame_specs[] = {
    {"CL", 3U, 0U, DSM_FRAME_ACK, "启用测水探针"},
    {"CB", 3U, 0U, DSM_FRAME_ACK, "启用液位模式"},
    {"CD", 3U, 0U, DSM_FRAME_ACK, "启用密度模式"},
    {"CV", 6U, 0U, DSM_FRAME_COMBINED_VERSION, "读取组合版本"},
    {"Cv", 6U, 0U, DSM_FRAME_CPU1_VERSION, "读取传感器版本"},
    {"CN", 11U, 19U, DSM_FRAME_SENSOR_NUMBER, "读取传感器编号"},
    {"CK", 11U, 0U, DSM_FRAME_SUPPLY_VOLTAGE, "读取传感器电压"},
    {"Cb", 11U, 19U, DSM_FRAME_LEVEL_FREQUENCY, "读取液位频率"},
    {"Cd", 27U, 19U, DSM_FRAME_DENSITY, "读取频率密度温度"},
    {"CM", 27U, 19U, DSM_FRAME_DENSITY_ANALYSIS, "读取密度分析参数"},
    {"Ch", 19U, 19U, DSM_FRAME_GYRO_ANGLE, "读取传感器倾角"},
    {"Cl", 11U, 19U, DSM_FRAME_WATER_CAPACITANCE, "读取测水电容"}
};

#define DSM_FRAME_SPEC_COUNT (sizeof(s_dsm_frame_specs) / sizeof(s_dsm_frame_specs[0]))

/**
 * @brief 在限定时间内排空 UART6 残留数据，连续空闲后结束。
 *
 * @param idle_ms 发送 DSM 命令前要求 UART6 连续无数据的空闲时间，单位 ms。
 * @return NO_ERROR 表示 UART6 已连续空闲；命令切换返回 STATE_SWITCH，硬件错误返回 COMM_UART_TRANSFER_ERROR，持续有数据达到总时限返回 SENSOR_DEVICE_COMM_TIMEOUT。
 */
static uint32_t UART6_DrainRX_UntilIdle(uint32_t idle_ms)
{
    uint8_t dump;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_rx_tick = start_tick;

    for (;;) {
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, &dump, 1U, 1U);
        uint32_t now_tick = HAL_GetTick();

        if (status == HAL_OK) {
            last_rx_tick = now_tick;
        } else if (status == HAL_ERROR) {
            s_uart6_last_error = huart6.ErrorCode;
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            huart6.ErrorCode = HAL_UART_ERROR_NONE;
            return COMM_UART_TRANSFER_ERROR;
        } else if ((now_tick - last_rx_tick) >= idle_ms) {
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            return NO_ERROR;
        }

        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        if ((now_tick - start_tick) >= DSM_UART6_DRAIN_TOTAL_TIMEOUT_MS) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
    }
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
    {"A+666.66B+666.66", SENSOR_REMOTE_INTERNAL_ERROR},
    {"A+777.88B+777.88", SENSOR_REMOTE_INTERNAL_ERROR},
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
    if ((resp != NULL) && (resp[0] == 'E')) {
        printf("DSM传感器电压过低\r\n");
    }
}

/**
 * @brief 把 DSM 特殊响应转换为可直接上报的二代故障码。
 *
 * @param resp 已经接收完成、等待识别或解析的 DSM 响应文本。
 * @return NO_ERROR 表示响应不是已知 DSM 错误文本；命中错误表时返回该文本对应的二代传感器错误码。
 */
static uint32_t DSM_MapErrorResponse(const char *resp, uint16_t recv_len)
{
    if ((resp == NULL) || (recv_len != 19U) ||
        (resp[17] != '\r') || (resp[18] != '\n')) {
        return NO_ERROR;
    }
    for (uint32_t i = 0U; i < DSM_RESPONSE_ERROR_COUNT; i++) {
        if (memcmp(resp,
                   s_dsm_response_errors[i].response_text,
                   DSM_RESPONSE_ERROR_PAYLOAD_LEN) == 0) {
            return s_dsm_response_errors[i].error_code;
        }
    }
    return NO_ERROR;
}

/**
 * @brief 按命令查找 DSM 定长响应规格。
 *
 * 调用场景：每次 DSM 事务在启动 DMA、判断帧边界、校验和写日志时共用同一规格。
 * 关键约束：未登记命令返回 NULL，禁止回退到首个 LF 结束的历史路径。
 *
 * @param cmd 两字节 DSM 命令文字。
 * @return 命中时返回只读规格指针，未命中或参数为空返回 NULL。
 */
static const DsmCommandFrameSpec *DSM_FindFrameSpec(const char *cmd)
{
    if (cmd == NULL) {
        return NULL;
    }
    for (uint32_t i = 0U; i < DSM_FRAME_SPEC_COUNT; i++) {
        if (strcmp(cmd, s_dsm_frame_specs[i].command) == 0) {
            return &s_dsm_frame_specs[i];
        }
    }
    return NULL;
}

/**
 * @brief 根据命令规格和响应首字节确定本次候选帧的精确长度。
 *
 * 多数字段命令以 A 开头时使用 19 字节统一错误帧长度；其它首字节使用正常响应长度。
 * CM 正常帧也以 A 开头，只有第 18、19 字节已经收到 CRLF 时才按 19 字节错误帧结束。
 * Ch 的正常帧和错误帧同为 19 字节，不需要额外分支。
 *
 * @param spec 当前命令规格。
 * @param rx DMA 接收缓冲区。
 * @param recv_len 当前已接收字节数。
 * @return 本候选帧必须接收的精确总长度；规格无效时返回 0。
 */
static uint16_t DSM_GetExpectedFrameLength(const DsmCommandFrameSpec *spec,
                                           const uint8_t *rx,
                                           uint16_t recv_len)
{
    if (spec == NULL) {
        return 0U;
    }
    if ((spec->frame_type == DSM_FRAME_DENSITY_ANALYSIS) &&
        (spec->error_len != 0U) &&
        (rx != NULL) &&
        (recv_len >= spec->error_len) &&
        (rx[spec->error_len - 2U] == (uint8_t)'\r') &&
        (rx[spec->error_len - 1U] == (uint8_t)'\n')) {
        return spec->error_len;
    }
    if (spec->frame_type == DSM_FRAME_DENSITY_ANALYSIS) {
        return spec->normal_len;
    }
    if ((spec->error_len != 0U) &&
        (spec->error_len != spec->normal_len) &&
        (rx != NULL) && (recv_len > 0U) &&
        (rx[0] == (uint8_t)'A')) {
        return spec->error_len;
    }
    return spec->normal_len;
}

/**
 * @brief 判断当前 DMA 字节数是否达到命令规定的精确候选长度。
 *
 * @param spec 当前命令规格。
 * @param rx DMA 接收缓冲区。
 * @param recv_len 当前已接收字节数。
 * @return 1 表示已经达到或超过候选长度，0 表示仍需等待。
 */
static uint8_t DSM_IsFixedFrameCandidateComplete(const DsmCommandFrameSpec *spec,
                                                 const uint8_t *rx,
                                                 uint16_t recv_len)
{
    uint16_t expected_len = DSM_GetExpectedFrameLength(spec, rx, recv_len);

    return (uint8_t)(((expected_len != 0U) && (recv_len >= expected_len)) ? 1U : 0U);
}

#define DSM_UART_MAX_RETRY UART6_COMM_MAX_RETRY /* 传感器通信参数：传感器 UART 最大值 重试。 */
static uint8_t s_uart6_dma_rx_buf[RX_BUF_LEN]; /* DSM 文本命令单次 UART6 DMA 接收缓冲区；启动前清零，DMA 计数器换算实际长度后再复制到调用方响应缓存。 */

/**
 * @brief 停止 UART6 单次 DMA 接收并清理硬件错误状态。
 *
 * DSM 文本协议每次命令只等待一帧应答，退出等待前必须停止 DMA，避免下一次命令接收沿用旧 DMA 状态。
 */
static void UART6_StopDmaReceive(void)
{
    /* 停止 DMA 前先锁存最后字节附近的 FE/NE/ORE/PE，避免 HAL 清场后丢失真实根因。 */
    s_uart6_last_error |= huart6.ErrorCode;
    (void)HAL_UART_DMAStop(&huart6);
    /* HAL 停止过程中若又发布错误位，清除前再次合并到本事务快照。 */
    s_uart6_last_error |= huart6.ErrorCode;
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
static uint32_t UART6_StartTextReceiveDma(uint16_t maxLen,
                                         const DsmCommandFrameSpec *spec,
                                         uint16_t *dma_len_out)
{
    uint16_t max_frame_len;
    uint16_t dma_len;

    if ((dma_len_out == NULL) || (spec == NULL) || (maxLen < 2U)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    max_frame_len = (spec->normal_len > spec->error_len)
                    ? spec->normal_len
                    : spec->error_len;
    /* 多接收 1 字节用于发现已经进入 DMA 缓冲区的超长帧；精确长度到达后不依赖 CR/LF。 */
    dma_len = (uint16_t)(max_frame_len + 1U);
    if ((dma_len >= maxLen) || (dma_len > (uint16_t)sizeof(s_uart6_dma_rx_buf))) {
        return SENSOR_RESP_FORMAT_ERROR;
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
 * @param spec 当前命令的固定候选长度配置；NULL 表示沿用历史换行结束规则。
 * @param recv_len_out 用于返回本次收到的有效 DSM 文本应答长度，单位字节。
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示已收到非空、未溢出的 NUL 结尾文本；参数或长度不合法返回 SENSOR_RESP_FORMAT_ERROR，命令切换返回 STATE_SWITCH，HAL 错误返回 COMM_UART_TRANSFER_ERROR，无数据超时返回 SENSOR_DEVICE_COMM_TIMEOUT，仅有不完整数据时返回 SENSOR_RESP_FORMAT_ERROR。
 */
static uint32_t UART6_WaitTextReceiveDma(char *response,
                                         uint16_t dma_len,
                                         const DsmCommandFrameSpec *spec,
                                         uint16_t *recv_len_out,
                                         uint32_t timeout)
{
    uint32_t startTick;

    if ((response == NULL) || (dma_len == 0U) || (spec == NULL)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < timeout) {
        uint16_t recvLen = UART6_GetDmaReceivedLength(dma_len);
        uint16_t expectedLen = DSM_GetExpectedFrameLength(spec, s_uart6_dma_rx_buf, recvLen);

        if (HasEffectiveCommandSwitchRequest()) {
            UART6_StopDmaReceive();
            return STATE_SWITCH;
        }

        /* 接收期间发现硬件错误时立即锁存并停止 DMA，部分字节不得继续进入协议解析。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            s_uart6_last_error |= huart6.ErrorCode;
            UART6_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }

        if ((expectedLen == 0U) || (recvLen > expectedLen)) {
            memcpy(response, s_uart6_dma_rx_buf, recvLen);
            response[recvLen] = '\0';
            if (recv_len_out != NULL) {
                *recv_len_out = recvLen;
            }
            UART6_StopDmaReceive();
            return SENSOR_RESP_FORMAT_ERROR;
        }

        if (DSM_IsFixedFrameCandidateComplete(spec, s_uart6_dma_rx_buf, recvLen) != 0U) {
            memcpy(response, s_uart6_dma_rx_buf, recvLen);
            response[recvLen] = '\0';
            if (recv_len_out != NULL) {
                *recv_len_out = recvLen;
            }
            UART6_StopDmaReceive();
            return NO_ERROR;
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
    /* 合并停止 DMA 前后的错误位；保留快照供本次重试日志输出，下一事务开始时统一清零。 */
    s_uart6_last_error |= huart6.ErrorCode;
    if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        huart6.ErrorCode = HAL_UART_ERROR_NONE;
    }
    return s_uart6_last_error;
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
 * @brief 检查 7 字节无符号整数字段。
 *
 * @param value 指向协议字段首地址。
 * @return true 表示 7 字节全部为十进制数字，false 表示格式异常。
 */
static bool DSM_IsSevenDigitInteger(const char *value)
{
    if (value == NULL) {
        return false;
    }
    for (uint8_t i = 0U; i < 7U; i++) {
        if (!isdigit((unsigned char)value[i])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查 7 字节无符号小数字段。
 *
 * @param value 指向协议字段首地址。
 * @return true 表示字段只含数字且恰好一个小数点，false 表示格式异常。
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
    return dot_count == 1U;
}

/**
 * @brief 检查 DSM 的 7 字节有符号数值字段。
 *
 * @param value 指向 7 字节协议数值字段首地址。
 * @param require_decimal 非零表示必须包含一个小数点，0 表示允许六位整数。
 * @return true 表示字段符合有符号十进制格式，false 表示格式异常。
 */
static bool DSM_IsSignedSevenDigitValue(const char *value, uint8_t require_decimal)
{
    uint8_t dot_count = 0U;

    if ((value == NULL) || ((value[0] != '+') && (value[0] != '-'))) {
        return false;
    }
    for (uint8_t i = 1U; i < 7U; i++) {
        if (value[i] == '.') {
            dot_count++;
        } else if (!isdigit((unsigned char)value[i])) {
            return false;
        }
    }
    return (dot_count <= 1U) && ((require_decimal == 0U) || (dot_count == 1U));
}

/**
 * @brief 检查符号加六位数字组成的 DSM 7 字节整数字段。
 * @param value 指向字段首地址。
 * @return true 表示首字节为正负号且其余六字节全为数字。
 */
static bool DSM_IsSignedSevenDigitInteger(const char *value)
{
    if ((value == NULL) || ((value[0] != '+') && (value[0] != '-'))) {
        return false;
    }
    for (uint8_t i = 1U; i < 7U; i++) {
        if (!isdigit((unsigned char)value[i])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 只解析协议规定的 7 字节数值字段，禁止把后续 BCC 当成数字继续消费。
 *
 * @param value 指向 7 字节协议字段首地址。
 * @param result_out 用于返回有限浮点值。
 * @return true 表示全部 7 字节被成功解析为有限数值，false 表示转换异常。
 */
static bool DSM_ParseSevenByteFloat(const char *value, float *result_out)
{
    char field[8];
    char *end = NULL;
    double parsed;

    if ((value == NULL) || (result_out == NULL)) {
        return false;
    }
    memcpy(field, value, 7U);
    field[7] = '\0';
    parsed = strtod(field, &end);
    if ((end != &field[7]) || !isfinite(parsed)) {
        return false;
    }
    *result_out = (float)parsed;
    return true;
}

/**
 * @brief 校验命令规格对应的精确帧长度、固定字段、CRLF 和 BCC。
 *
 * @param spec 当前命令规格。
 * @param response 已接收帧缓冲区。
 * @param recv_len 已接收有效字节数。
 * @return NO_ERROR 表示帧结构和 BCC 完整；其他值区分格式异常和 BCC 错误。
 */
static uint32_t DSM_ValidateFixedFrame(const DsmCommandFrameSpec *spec,
                                      const char *response,
                                      uint16_t recv_len)
{
    uint32_t mapped_error;
    uint8_t normal_shape = 0U;

    if ((spec == NULL) || (response == NULL)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if ((recv_len != spec->normal_len) &&
        ((spec->error_len == 0U) || (recv_len != spec->error_len))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    if (spec->frame_type == DSM_FRAME_ACK) {
        static const uint8_t exact_ack[3] = {0x25U, 0x0DU, 0x0AU};
        return ((recv_len == 3U) && (memcmp(response, exact_ack, sizeof(exact_ack)) == 0))
               ? NO_ERROR
               : SENSOR_RESP_FORMAT_ERROR;
    }

    if ((spec->frame_type == DSM_FRAME_COMBINED_VERSION) ||
        (spec->frame_type == DSM_FRAME_CPU1_VERSION)) {
        if (recv_len != 6U) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        if (spec->frame_type == DSM_FRAME_COMBINED_VERSION) {
            normal_shape = (uint8_t)(isxdigit((unsigned char)response[0]) &&
                                     isxdigit((unsigned char)response[1]) &&
                                     isdigit((unsigned char)response[2]) &&
                                     isdigit((unsigned char)response[3]) &&
                                     isdigit((unsigned char)response[4]));
        } else {
            normal_shape = (uint8_t)(((isdigit((unsigned char)response[0])) ||
                                      (response[0] == 'E') || (response[0] == 'e')) &&
                                     isdigit((unsigned char)response[1]) &&
                                     (response[2] == '.') &&
                                     isdigit((unsigned char)response[3]) &&
                                     isdigit((unsigned char)response[4]));
        }
        if (normal_shape == 0U) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        return (CalculationBCC_DSM((char *)response, 5) == response[5])
               ? NO_ERROR
               : SENSOR_BCC_ERROR;
    }

    if ((recv_len < 3U) ||
        (response[recv_len - 2U] != '\r') ||
        (response[recv_len - 1U] != '\n')) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    mapped_error = DSM_MapErrorResponse(response, recv_len);
    if (mapped_error != NO_ERROR) {
        normal_shape = 1U;
    } else {
        switch (spec->frame_type) {
        case DSM_FRAME_SENSOR_NUMBER:
            normal_shape = (uint8_t)((recv_len == 11U) &&
                                     ((response[0] == 'N') || (response[0] == 'E') || (response[0] == 'e')) &&
                                     DSM_IsSevenDigitInteger(&response[1]));
            break;
        case DSM_FRAME_SUPPLY_VOLTAGE:
            normal_shape = (uint8_t)((recv_len == 11U) &&
                                     ((response[0] == 'e') || (response[0] == 'E')) &&
                                     DSM_IsSevenDigitValue(&response[1]));
            break;
        case DSM_FRAME_LEVEL_FREQUENCY:
            normal_shape = (uint8_t)((recv_len == 11U) &&
                                     ((response[0] == 'F') || (response[0] == 'E')) &&
                                     DSM_IsSevenDigitInteger(&response[1]));
            break;
        case DSM_FRAME_DENSITY:
            normal_shape = (uint8_t)((recv_len == 27U) &&
                                     ((response[0] == 'F') || (response[0] == 'E')) &&
                                     (response[8] == 'D') &&
                                     (response[16] == 'T') &&
                                     DSM_IsSignedSevenDigitValue(&response[1], 0U) &&
                                     DSM_IsSignedSevenDigitValue(&response[9], 1U) &&
                                     DSM_IsSignedSevenDigitValue(&response[17], 1U));
            break;
        case DSM_FRAME_DENSITY_ANALYSIS:
            normal_shape = (uint8_t)((recv_len == 27U) &&
                                     ((response[0] == 'A') || (response[0] == 'E')) &&
                                     (response[8] == 'B') &&
                                     (response[16] == 'C') &&
                                     DSM_IsSignedSevenDigitInteger(&response[1]) &&
                                     DSM_IsSignedSevenDigitInteger(&response[9]) &&
                                     DSM_IsSignedSevenDigitValue(&response[17], 1U));
            break;
        case DSM_FRAME_GYRO_ANGLE:
            normal_shape = (uint8_t)((recv_len == 19U) &&
                                     ((response[0] == 'A') || (response[0] == 'E')) &&
                                     (response[8] == 'B') &&
                                     DSM_IsSignedSevenDigitValue(&response[1], 1U) &&
                                     DSM_IsSignedSevenDigitValue(&response[9], 1U));
            break;
        case DSM_FRAME_WATER_CAPACITANCE:
            normal_shape = (uint8_t)((recv_len == 11U) &&
                                     ((response[0] == 'D') || (response[0] == 'E')) &&
                                     DSM_IsSevenDigitValue(&response[1]));
            break;
        default:
            normal_shape = 0U;
            break;
        }
    }

    if (normal_shape == 0U) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (CalculationBCC_DSM((char *)response, (int)(recv_len - 3U)) != response[recv_len - 3U]) {
        return SENSOR_BCC_ERROR;
    }
    return NO_ERROR;
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
static int UART6_SendCommand(const DsmCommandFrameSpec *frame_spec,
                             char *response,
                             uint16_t maxLen,
                             uint16_t *recv_len_out,
                             uint32_t timeout) {
    uint32_t drain_ret;
    uint32_t uart_error;
    const char *cmd;

    if ((frame_spec == NULL) || (response == NULL) || (maxLen < 2U)) {
        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        return SENSOR_RESP_FORMAT_ERROR;
    }
    cmd = frame_spec->command;

    if (CH9141_AT_GetUart6LinkState() != CH9141_UART6_TRANSPARENT_READY) {
        CH9141Uart6LinkState link_state = CH9141_AT_GetUart6LinkState();

        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        printf("DSM通信\t拒绝发送\tcmd=%s\tUART6状态=%u\r\n",
               cmd,
               (unsigned)link_state);
        return SENSOR_MODE_NOT_READY;
    }

    memset(response, 0, maxLen);
    uint16_t recvLen = 0;
#if DEBUG_UART6
    printf("[UART6] 发送: %s\n", cmd);
#endif

    s_uart6_last_error = HAL_UART_ERROR_NONE;
    drain_ret = UART6_DrainRX_UntilIdle(5U);
    if (drain_ret != NO_ERROR) {
        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        return (int)drain_ret;
    }

    /* 先启动精确长度接收 DMA，再启动发送 DMA，避免快速应答丢头。 */
    uint16_t dma_len = 0U;
    uint32_t rx_ret = UART6_StartTextReceiveDma(maxLen, frame_spec, &dma_len);
    if (rx_ret != NO_ERROR) {
        if (recv_len_out != NULL) {
            *recv_len_out = 0U;
        }
        return (int)rx_ret;
    }

    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t*)cmd, (uint16_t)strlen(cmd)) != HAL_OK) {
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

    rx_ret = UART6_WaitTextReceiveDma(response, dma_len, frame_spec, &recvLen, timeout);
    if (rx_ret != NO_ERROR) {
        uart_error = UART6_TakeHardwareError();
        if (recv_len_out != NULL) {
            *recv_len_out = recvLen;
        }
        return (uart_error != HAL_UART_ERROR_NONE)
               ? COMM_UART_TRANSFER_ERROR
               : (int)rx_ret;
    }
    if (recv_len_out != NULL) {
        *recv_len_out = recvLen;
    }

    /* 停止 DMA 时已经锁存错误；协议解析前必须消费该快照，不能只读已清零的 HAL 字段。 */
    uart_error = UART6_TakeHardwareError();
    if (uart_error != HAL_UART_ERROR_NONE) {
        return COMM_UART_TRANSFER_ERROR;
    }
    if (recvLen == 0U) {
        return SENSOR_DEVICE_COMM_TIMEOUT;
    }

#if DEBUG_UART6
    printf("[UART6] 接收成功，共 %d 字节\r\n", recvLen);
    printf("[UART6] 响应HEX: ");
    for (int i = 0; i < recvLen; i++) {
        printf("%02X ", (uint8_t)response[i]);
    }
    printf("\r\n");
#endif

    return (int)DSM_ValidateFixedFrame(frame_spec, response, recvLen);
}

/**
 * @brief 按指定次数和日志策略发送 DSM 命令。
 *
 * @param cmd 准备通过 UART6 发送、可读且以 NUL 结尾的两字节命令文本。
 * @param response UART6 响应可写缓冲区；接收前清零 response_size 字节并保证 NUL 结尾，格式错误时只包含实际收到的字节。
 * @param maxLen 调用方接收缓冲区容量，单位字节。
 * @param recv_len_out 用于返回本轮收到的有效 DSM 文本响应长度，单位字节。
 * @param timeout 本次操作使用的超时门限。
 * @param max_attempts 本轮允许的发送次数，不得超过 DSM 统一重试次数。
 * @param log_retry 非零时记录错误重试和恢复日志；零用于兼容性静默探测。
 * @return NO_ERROR 表示某次尝试完成有效收发；命令切换立即返回 STATE_SWITCH，全部尝试失败时返回最后一次错误。
 */
static int UART6_SendWithPolicy(const char *cmd,
                                char *response,
                                uint16_t maxLen,
                                uint16_t *recv_len_out,
                                uint32_t timeout,
                                uint32_t max_attempts,
                                uint8_t log_retry) {
    uint32_t ret = SENSOR_DEVICE_COMM_TIMEOUT;
    uint16_t recvLen = 0;
    char detail[160];
    CH9141Uart6LinkState link_state;
    const DsmCommandFrameSpec *frame_spec = DSM_FindFrameSpec(cmd);
    const char *operation;

    if ((frame_spec == NULL) ||
        (max_attempts == 0U) ||
        (max_attempts > DSM_UART_MAX_RETRY)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    operation = frame_spec->operation_name;

    if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    link_state = CH9141_AT_GetUart6LinkState();
    if (link_state == CH9141_UART6_NOT_READY) {
        /* NOT_READY 表示上一轮 AT 收尾已经结束但恢复失败；发 DSM 前只执行一次有界恢复。 */
        printf("DSM通信\tUART6透明链路恢复\tcmd=%s\t原状态=%u\r\n",
               cmd,
               (unsigned)link_state);
        ret = CH9141_AT_RecoverRssiQuery();
        link_state = CH9141_AT_GetUart6LinkState();
        if ((ret != NO_ERROR) || (link_state != CH9141_UART6_TRANSPARENT_READY)) {
            printf("DSM通信\tUART6透明链路恢复失败\tcmd=%s\t结果=0x%08lX\t状态=%u\r\n",
                   cmd,
                   (unsigned long)ret,
                   (unsigned)link_state);
            return (ret != NO_ERROR) ? (int)ret : SENSOR_MODE_NOT_READY;
        }
        printf("DSM通信\tUART6透明链路恢复成功\tcmd=%s\r\n", cmd);
    } else if (link_state != CH9141_UART6_TRANSPARENT_READY) {
        /* ENTERING_AT、AT_ACTIVE 或 RECOVERING 都属于正在执行的事务，DSM 不得抢占。 */
        printf("DSM通信\tUART6透明链路忙\tcmd=%s\t状态=%u\r\n",
               cmd,
               (unsigned)link_state);
        return SENSOR_MODE_NOT_READY;
    }

    for (uint32_t i = 0U; i < max_attempts; i++) {
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        if (i > 0U) {
            HAL_Delay(DSM_PRE_SEND_DELAY);
        }
        ret = UART6_SendCommand(frame_spec, response, maxLen, &recvLen, timeout);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (ret == SENSOR_MODE_NOT_READY) {
            return SENSOR_MODE_NOT_READY;
        }
        if (ret == NO_ERROR) {
            uint32_t response_error;

            DSM_LogLowVoltageFrame(response);
            response_error = DSM_MapErrorResponse(response, recvLen);
            if (response_error == NO_ERROR) {
                if (recv_len_out != NULL) {
                    *recv_len_out = recvLen;
                }
                if ((i > 0U) && (log_retry != 0U)) {
                    /* 实际打印“错误，阶段：重试成功”，描述重试成功的当前 DSM 操作。 */
                    ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                     operation,
                                     ERROR_LOG_REASON_COMM_FAIL,
                                     i + 1U,
                                     max_attempts);
                }
                return NO_ERROR;
            }

            if (log_retry != 0U) {
                /* 实际打印“错误，阶段：错误重试”，描述远端错误原因和当前 DSM 操作。 */
                ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                               operation,
                               ErrorLog_GetReasonByCode(response_error),
                               i + 1U,
                               max_attempts,
                               response_error);
            }
            ret = response_error;
        } else if (log_retry != 0U) {
            UART6_FormatHexDetail(ErrorLog_GetReasonByCode(ret),
                                  cmd,
                                  response,
                                  recvLen,
                                  s_uart6_last_error,
                                  detail,
                                  sizeof(detail));
            /* 实际打印“错误，阶段：错误重试”，诊断详情保留命令、长度、UART 位图和 HEX。 */
            ErrorLog_RetryDetail(ERROR_LOG_MODULE_SENSOR,
                                 operation,
                                 ErrorLog_GetReasonByCode(ret),
                                 i + 1U,
                                 max_attempts,
                                 ret,
                                 detail);
        }
    }
    return ret;
}

/**
 * @brief 使用统一三次重试和错误日志策略发送 DSM 命令。
 * @return 透传策略发送函数的通信、格式、远端或命令切换结果。
 */
static int UART6_SendWithRetry(const char *cmd,
                               char *response,
                               uint16_t maxLen,
                               uint16_t *recv_len_out,
                               uint32_t timeout) {
    return UART6_SendWithPolicy(cmd,
                                response,
                                maxLen,
                                recv_len_out,
                                timeout,
                                DSM_UART_MAX_RETRY,
                                1U);
}

/**
 * @brief 把五字节 CPU1 版本文本转换为百分之一版本整数。
 * @param version 指向形如 05.40 的五字节版本文本。
 * @return 版本乘以 100 后的整数，例如 05.40 返回 540。
 */
static uint16_t DSM_ParseCpu1VersionX100(const char *version)
{
    uint16_t major = (uint16_t)(((uint16_t)(version[0] - '0') * 10U) +
                                (uint16_t)(version[1] - '0'));
    uint16_t minor = (uint16_t)(((uint16_t)(version[3] - '0') * 10U) +
                                (uint16_t)(version[4] - '0'));
    return (uint16_t)((major * 100U) + minor);
}

/**
 * @brief 根据 CPU1 版本值分类 DSM 维护线。
 * @param version_x100 版本乘以 100 后的整数。
 * @return 版本维护线；05.00 单独标记为无法区分标准版和 Lemis。
 */
static DsmVersionProfile DSM_ClassifyVersion(uint16_t version_x100)
{
    if ((version_x100 >= 300U) && (version_x100 < 400U)) {
        return DSM_VERSION_PROFILE_V3;
    }
    if ((version_x100 >= 400U) && (version_x100 < 500U)) {
        return DSM_VERSION_PROFILE_V4;
    }
    if (version_x100 == 500U) {
        return DSM_VERSION_PROFILE_V5_0_AMBIGUOUS;
    }
    if ((version_x100 > 500U) && (version_x100 < 600U)) {
        return DSM_VERSION_PROFILE_V5;
    }
    if ((version_x100 >= 600U) && (version_x100 < 700U)) {
        return DSM_VERSION_PROFILE_V6;
    }
    return DSM_VERSION_PROFILE_UNKNOWN;
}

/**
 * @brief 读取 DSM CPU1/CPU0 版本并刷新 RAM 会话上下文。
 * @return NO_ERROR 表示版本上下文可用；前缀 07 允许仅 CV 有效，其他失败返回原错误码。
 */
uint32_t DSM_ReadVersionContext(void)
{
    char resp[RX_BUF_LEN] = {0};
    uint32_t cpu1_ret;
    uint32_t ret;

    memset(&s_dsm_session_context, 0, sizeof(s_dsm_session_context));
    s_dsm_session_context.profile = DSM_VERSION_PROFILE_UNREAD;

    /* V3.02 以前不支持 Cv；只静默探测一次，避免旧设备产生三轮超时和错误日志。 */
    cpu1_ret = UART6_SendWithPolicy("Cv", resp, RX_BUF_LEN, NULL, 500U, 1U, 0U);
    if (cpu1_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    if (cpu1_ret == NO_ERROR) {
        memcpy(s_dsm_session_context.cpu1_version, resp, 5U);
        s_dsm_session_context.cpu1_version[5] = '\0';
        if ((resp[0] == 'E') || (resp[0] == 'e')) {
            s_dsm_session_context.cpu1_version[0] = '0';
            s_dsm_session_context.low_voltage = 1U;
        }
        s_dsm_session_context.cpu1_version_x100 =
            DSM_ParseCpu1VersionX100(s_dsm_session_context.cpu1_version);
        s_dsm_session_context.profile =
            DSM_ClassifyVersion(s_dsm_session_context.cpu1_version_x100);
        s_dsm_session_context.cpu1_valid = 1U;
    }

    /* Cv 失败时仍读取所有历史版本都支持的 CV，保留可用的 CPU0 组合版本。 */
    memset(resp, 0, sizeof(resp));
    ret = UART6_SendWithRetry("CV", resp, RX_BUF_LEN, NULL, 500U);
    if (ret != NO_ERROR) {
        if (cpu1_ret != NO_ERROR) {
            s_dsm_session_context.profile = DSM_VERSION_PROFILE_UNKNOWN;
        }
        return ret;
    }

    s_dsm_session_context.combined_prefix[0] =
        ((resp[0] == 'E') || (resp[0] == 'e')) ? '0' : resp[0];
    s_dsm_session_context.combined_prefix[1] = resp[1];
    s_dsm_session_context.combined_prefix[2] = '\0';
    s_dsm_session_context.cpu0_version_x100 =
        (uint16_t)(((uint16_t)(resp[2] - '0') * 100U) +
                   ((uint16_t)(resp[3] - '0') * 10U) +
                   (uint16_t)(resp[4] - '0'));
    s_dsm_session_context.cpu0_valid = 1U;
    if ((resp[0] == 'E') || (resp[0] == 'e')) {
        s_dsm_session_context.low_voltage = 1U;
    }

    if (cpu1_ret != NO_ERROR) {
        if ((cpu1_ret == SENSOR_DEVICE_COMM_TIMEOUT) &&
            (strcmp(s_dsm_session_context.combined_prefix, "07") == 0)) {
            /* 前缀 07 覆盖 V1/V2、历史过渡线和 V3；Cv 未响应时保留可用的 CV 上下文。 */
            s_dsm_session_context.profile = DSM_VERSION_PROFILE_CV07_AMBIGUOUS;
            return NO_ERROR;
        }
        s_dsm_session_context.profile = DSM_VERSION_PROFILE_UNKNOWN;
        return cpu1_ret;
    }
    return NO_ERROR;
}

/**
 * @brief 获取当前 DSM RAM 会话上下文。
 * @return 只读上下文指针。
 */
const DsmSessionContext *DSM_GetSessionContext(void)
{
    return &s_dsm_session_context;
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
    float voltage;

    if (voltage_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    dsmsensor_data.Power_Voltage_Valid = 0U;
    ret = UART6_SendWithRetry("CK", resp, RX_BUF_LEN, NULL, 500);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!DSM_ParseSevenByteFloat(&resp[1], &voltage)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    dsmsensor_data.Power_Voltage = voltage;
    dsmsensor_data.Power_Voltage_Valid = 1U;
    *voltage_out = voltage;
    return NO_ERROR;
}

/**
 * @brief 读取 DSM 22.5 度、45 度扫频周期平方和动态黏度。
 * @param period_225_out 22.5 度扫频周期平方输出指针。
 * @param period_45_out 45 度扫频周期平方输出指针。
 * @param dynamic_viscosity_out 动态黏度输出指针。
 * @return NO_ERROR 表示 CM 帧及三个字段均有效；其他值为调用条件、通信或响应格式错误。
 */
uint32_t DSM_ReadDensityAnalysis(float *period_225_out,
                                 float *period_45_out,
                                 float *dynamic_viscosity_out)
{
    char resp[RX_BUF_LEN] = {0};
    uint32_t ret;
    float parsed_period_225;
    float parsed_period_45;
    float parsed_dynamic_viscosity;

    if ((period_225_out == NULL) ||
        (period_45_out == NULL) ||
        (dynamic_viscosity_out == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    dsmsensor_data.Density_Analysis_Valid = 0U;
    ret = UART6_SendWithRetry("CM", resp, RX_BUF_LEN, NULL, 500U);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!DSM_ParseSevenByteFloat(&resp[1], &parsed_period_225) ||
        !DSM_ParseSevenByteFloat(&resp[9], &parsed_period_45) ||
        !DSM_ParseSevenByteFloat(&resp[17], &parsed_dynamic_viscosity)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    /* 三个字段全部解析成功后一次性发布，避免调用方观察到混合新旧值。 */
    dsmsensor_data.MeanSquareOf225DegreeSweepPeriod = parsed_period_225;
    dsmsensor_data.SquareMeanOf45degreeSweepPeriod = parsed_period_45;
    dsmsensor_data.Dynamic_Viscosity = parsed_dynamic_viscosity;
    dsmsensor_data.Density_Analysis_Valid = 1U;
    *period_225_out = parsed_period_225;
    *period_45_out = parsed_period_45;
    *dynamic_viscosity_out = parsed_dynamic_viscosity;
    return NO_ERROR;
}

/**
 * @brief 开启测水探针 (CL 命令)。
 *
 * @return NO_ERROR 表示 CL 应答已收到并通过内容校验；应答内容不匹配返回 SENSOR_RESP_FORMAT_ERROR，收发失败返回 UART6_SendWithRetry 的具体错误。
 */
int Probe_EnableWaterSensor(void) {
    char resp[RX_BUF_LEN];
    uint32_t ret;

    ret = UART6_SendWithRetry("CL", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        printf("[探针] 开启测水探针响应: %%\r\n");
        printf("[探针] 测水探针开启成功！\r\n");
    }
    return (int)ret;
}

/**
 * @brief 开启液位模式。
 *
 * @return NO_ERROR 表示液位模式命令应答已确认；回显内容不匹配返回 SENSOR_RESP_FORMAT_ERROR，收发失败返回最后一次具体通信错误。
 */
int DSM_EnableLevelMode(void) {
    char resp[RX_BUF_LEN];
    uint32_t ret;

    ret = UART6_SendWithRetry("CB", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        printf("[液位模式] 开启液位模式响应: %%\r\n");
        printf("[液位模式] 开启成功！\r\n");
    }
    return (int)ret;
}

/**
 * @brief 开启密度模式。
 *
 * @return NO_ERROR 表示密度模式命令应答已确认；回显内容不匹配返回 SENSOR_RESP_FORMAT_ERROR，收发失败返回最后一次具体通信错误。
 */
int DSM_EnableDensityMode(void) {
    char resp[RX_BUF_LEN];
    uint32_t ret;

    ret = UART6_SendWithRetry("CD", resp, RX_BUF_LEN, NULL, 500);
    if (ret == NO_ERROR) {
        printf("[密度模式] 开启密度模式响应: %%\r\n");
        printf("[密度模式] 开启成功！\r\n");
    }
    return (int)ret;
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
    if ((resp == NULL) || (out_hz == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    return DSM_ParseSevenByteFloat(&resp[1], out_hz) ? 0 : SENSOR_RESP_FORMAT_ERROR;
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
    int ret;
    char resp[RX_BUF_LEN];
    float parsed_frequency;
    float parsed_density;
    float parsed_temp;

    if ((frequency == NULL) || (density == NULL) || (temp == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = UART6_SendWithRetry("Cd", resp, RX_BUF_LEN, NULL, 500);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!DSM_ParseSevenByteFloat(&resp[1], &parsed_frequency) ||
        !DSM_ParseSevenByteFloat(&resp[9], &parsed_density) ||
        !DSM_ParseSevenByteFloat(&resp[17], &parsed_temp)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *frequency = parsed_frequency;
    *density = parsed_density;
    *temp = parsed_temp;
    return NO_ERROR;
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

    /* 只解析固定 7 字节字段，避免原始 BCC 恰好为数字时被 atof 继续消费。 */
    float parsed_cap;
    if (!DSM_ParseSevenByteFloat(&resp[1], &parsed_cap)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *cap_out = parsed_cap;
	if (*cap_out < 30.0f) {
		*cap_out = 99999.9;
	}
    return NO_ERROR;
}
/**
 * @brief 读取陀螺仪角度（Ch 指令）。
 *
 * @param angle_x_deg 用于返回 X 轴角度。
 * @param angle_y_deg 用于返回 Y 轴角度。
 * @return NO_ERROR 表示双轴角度均由严格定长帧解析成功；其他值为通信、远端或格式错误。
 */
uint32_t Read_Gyro_Angle(float *angle_x_deg, float *angle_y_deg)
{
    char resp[RX_BUF_LEN] = {0};
    uint32_t ret;
    float ax;
    float ay;

    if ((angle_x_deg == NULL) || (angle_y_deg == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = UART6_SendWithRetry("Ch", resp, RX_BUF_LEN, NULL, 500);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!DSM_ParseSevenByteFloat(&resp[1], &ax) ||
        !DSM_ParseSevenByteFloat(&resp[9], &ay)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *angle_x_deg = ax;
    *angle_y_deg = ay;
    g_measurement.debug_data.angle_x = ax * 100.0f;
    g_measurement.debug_data.angle_y = ay * 100.0f;
    return NO_ERROR;
}

