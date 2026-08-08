/*
 * multiparam_v3_communication.c
 * 本文件实现多参数传感器通信协议 V3.0，与 DSM 传感器协议命名隔离。
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */

#include <multiparam_v3_communication.h>
#include "system_parameter.h"
#include "sensor.h"
#include "error_log.h"
#include <math.h>
#include <stdio.h>

#ifndef MULTIPARAM_V3_MAX_RETRY
#define MULTIPARAM_V3_MAX_RETRY   UART6_COMM_MAX_RETRY /* 传感器通信参数：多参数协议 V3.0 最大值 重试。 */
#endif
#ifndef MULTIPARAM_V3_RX_TIMEOUT
#define MULTIPARAM_V3_RX_TIMEOUT  DSM_CMD_TIMEOUT /* 传感器通信参数：多参数协议 V3.0 RX 超时。 */
#endif

/* 主机方向功能码 */
typedef enum {
	/* 多参数传感器通信协议 V3.0的单字符功能码。 */
	MULTIPARAM_V3_FUNC_R = 'R', /* 多参数传感器通信协议 V3.0 读取寄存器或参数功能。 */ MULTIPARAM_V3_FUNC_W = 'W', /* 多参数传感器通信协议 V3.0 写入寄存器或参数功能。 */ MULTIPARAM_V3_FUNC_L = 'T', /* 多参数传感器通信协议 V3.0 液位/跟踪相关功能，线字符为 T。 */ MULTIPARAM_V3_FUNC_D = 'D', /* 多参数传感器通信协议 V3.0 密度测量相关功能。 */ MULTIPARAM_V3_FUNC_B = 'B', /* 多参数传感器通信协议 V3.0 罐底或回零相关功能。 */
} multiparam_v3_func_t;

/**
 * @brief 计算 多参数传感器通信协议 V3.0 固定帧前 7 字节的低 8 位累加校验值。
 *
 * @param f 多参数传感器通信协议 V3.0 固定 8 字节帧；校验只累计前 7 字节。
 * @return 返回 多参数传感器通信协议 V3.0 固定帧前 7 字节累加和的低 8 位校验值。
 */
static inline uint8_t MULTIPARAM_V3_CalcSum(const uint8_t f[8]) {
	uint32_t s = 0;
	for (int i = 0; i < 7; ++i)
		s += f[i];
	return (uint8_t) (s & 0xFF);
}

/**
 * @brief 按 多参数传感器通信协议 V3.0 固定 8 字节格式组装请求帧并写入累加校验。
 *
 * @param out 固定 8 字节 多参数传感器通信协议 V3.0 请求帧输出数组，依次写入地址、功能、数据、参数码和累加校验。
 * @param func 多参数传感器通信协议 V3.0 请求功能字节，例如模式字符或读写功能码，写入固定帧第 2 字节。
 * @param data_be 待放入 多参数传感器通信协议 V3.0 请求帧的数据字段，调用方按大端数值传入。
 * @param param 多参数传感器通信协议 V3.0 参数码，决定本次读写的传感器寄存器。
 * @note 帧地址固定为 0x00；data 按大端顺序写入，数值为 0 时端序不影响结果。
 */
static inline void MULTIPARAM_V3_MakeFrame(uint8_t out[8], uint8_t func, uint32_t data_be, uint8_t param) {
	out[0] = 0x00;
	out[1] = func;
	out[2] = (uint8_t) ((data_be >> 24) & 0xFF);
	out[3] = (uint8_t) ((data_be >> 16) & 0xFF);
	out[4] = (uint8_t) ((data_be >> 8) & 0xFF);
	out[5] = (uint8_t) (data_be & 0xFF);
	out[6] = param;
	out[7] = MULTIPARAM_V3_CalcSum(out);
}

/* === 内部：传输 8→8 === */

/**
 * @brief === 1) 发前清空可能的残留：非正式“flush” ===。
 *
 * @param idle_ms 进入 AT 操作前要求 UART6 连续无数据的空闲时间，单位 ms。
 */
static void UART6_DrainRX_UntilIdle(uint32_t idle_ms) {
    uint8_t dump;
    uint32_t last = HAL_GetTick();
    for (;;) {
        /* 1字节超时设为1ms：有字节就读走并更新时间；没字节就看是否空闲超时 */
        if (HAL_UART_Receive(&huart6, &dump, 1, 1) == HAL_OK) {
            last = HAL_GetTick(); /* 读到了字节，刷新“最近一次活动时间” */
        } else {
            if ((HAL_GetTick() - last) >= idle_ms) break; /* 空闲>=idle_ms 认为干净 */
        }
    }

    /* 清理溢出等异常标志（可选，避免ORE悬挂） */
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    /* 读SR/DR清RXNE的老派做法（F4上清ORE通常需要读SR后读DR，HAL宏已封装） */
}

static uint8_t s_multiparam_v3_dma_rx_buf[8]; /* 多参数传感器通信协议 V3.0 固定 8 字节响应的 UART6 DMA 接收缓冲区；事务结束前按实际接收长度复制到调用方缓冲并记录诊断。 */
static const char *s_multiparam_v3_last_stage = "未开始";
/* 最近一次 多参数传感器通信协议 V3.0 事务记录的 HAL UART 错误位。 */
static uint32_t s_multiparam_v3_last_uart_error = HAL_UART_ERROR_NONE;
/* 最近一次 多参数传感器通信协议 V3.0 接收的实际字节数。 */
static uint16_t s_multiparam_v3_last_received_length = 0U;

/**
 * @brief 保存 多参数传感器通信协议 V3.0 最近一次失败阶段，供重试日志定位。
 *
 * @param stage 用于诊断日志或参数保存记录的 NUL 结尾阶段名称；标识本次输出对应的加载、比较、写入、回读或协议处理阶段。
 * @param received_length 本次 多参数传感器通信协议 V3.0 响应实际接收长度，单位字节。
 */
static void MULTIPARAM_V3_RecordDiagnostic(const char *stage, uint16_t received_length)
{
    s_multiparam_v3_last_stage = (stage != NULL) ? stage : "未知阶段";
    s_multiparam_v3_last_uart_error = huart6.ErrorCode;
    s_multiparam_v3_last_received_length = received_length;
}

/**
 * @brief 在统一重试日志中附带完整的 8 字节请求和当前应答。
 *
 * @param operation 正在重试的 多参数传感器通信协议 V3.0 操作名称，用于区分切换、读取和诊断日志。
 * @param error_code 待记录、转换或判断的错误码。该值是 多参数传感器通信协议 V3.0 本次通信尝试的失败原因，用于逐次重试日志。
 * @param attempt 当前重试序号。
 * @param tx 触发本次重试的只读 8 字节 多参数传感器通信协议 V3.0 请求帧，用于在错误日志中保留原始证据。
 * @param rx 接收到的数据缓冲区。该参数指向固定 8 字节请求帧，函数按协议字段位置只读地址、功能、数据和校验字节。
 */
static void MULTIPARAM_V3_LogRetry(const char *operation,
                            uint32_t error_code,
                            uint32_t attempt,
                            const uint8_t tx[8],
                            const uint8_t rx[8])
{
    char detail[256];

    (void)snprintf(detail,
                   sizeof(detail),
                   "阶段=%s,UART错误=0x%08lX,接收长度=%u,发送=%02X %02X %02X %02X %02X %02X %02X %02X,接收=%02X %02X %02X %02X %02X %02X %02X %02X",
                   s_multiparam_v3_last_stage,
                   (unsigned long)s_multiparam_v3_last_uart_error,
                   (unsigned int)s_multiparam_v3_last_received_length,
                   tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7],
                   rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
    ErrorLog_RetryDetail(ERROR_LOG_MODULE_SENSOR,
                         operation,
                         ErrorLog_GetReasonByCode(error_code),
                         attempt,
                         MULTIPARAM_V3_MAX_RETRY,
                         error_code,
                         detail);
}

/**
 * @brief 停止 UART6 DMA 接收并清理固定 8 字节协议的硬件错误状态。
 *
 * 当前LTD传感器采用多参数传感器通信协议 V3.0的一问一答固定帧，任何提前返回都必须停止 DMA，避免下次收发继续占用 UART6。
 */
static void MULTIPARAM_V3_StopDmaReceive(void)
{
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
 * @brief 获取 UART6 DMA 当前已收到的字节数。
 *
 * 固定 8 字节协议只关心是否收满 8 字节；句柄异常时返回 0，由上层按通信超时处理。
 *
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @return 返回 UART6 DMA 当前已收到的字节数的有效长度，单位字节；0 表示没有可供消费的数据。
 */
static uint16_t MULTIPARAM_V3_GetDmaReceivedLength(uint16_t rx_len)
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
 * @brief 等待 多参数传感器通信协议 V3.0 的 UART6 DMA 发送完成。
 *
 * 接收 DMA 已提前启动，发送阶段只等待 TX 状态回到 READY；异常退出时停止 DMA，避免占用后续收发。
 *
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示 多参数传感器通信协议 V3.0 请求帧 DMA 发送完成；命令切换返回 STATE_SWITCH，HAL 错误或等待超时返回 COMM_UART_TRANSFER_ERROR。
 */
static uint32_t MULTIPARAM_V3_WaitTransmitDmaDone(uint32_t timeout)
{
    uint32_t startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeout) {
        if (HasEffectiveCommandSwitchRequest()) {
            MULTIPARAM_V3_StopDmaReceive();
            return STATE_SWITCH;
        }
        if (huart6.gState == HAL_UART_STATE_READY) {
            return NO_ERROR;
        }
        /* 等待 LTD 发送 DMA 完成时发现 UART6 硬件错误，先记录发送阶段诊断并停止 DMA，再返回传输失败。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            MULTIPARAM_V3_RecordDiagnostic("发送DMA硬件错误", 0U);
            MULTIPARAM_V3_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }
        /* LTD 发送 DMA 等待循环每 1 ms 让出 CPU，并在下一轮继续检查完成、超时和 UART 错误。 */
        HAL_Delay(1);
    }

    MULTIPARAM_V3_RecordDiagnostic("发送DMA等待超时", 0U);
    MULTIPARAM_V3_StopDmaReceive();
    return COMM_UART_TRANSFER_ERROR;
}

/**
 * @brief 启动 多参数传感器通信协议 V3.0 固定 8 字节应答的 UART6 DMA 接收。
 *
 * 该函数只负责提前打开接收窗口，避免发送完成后再启动 DMA 导致快速回包丢头。
 *
 * @param rx 接收到的数据缓冲区。该参数指向固定 8 字节帧；函数按职责解析字段，必要时在局部调用链内复用可写缓冲区。
 * @return NO_ERROR 表示固定 8 字节接收 DMA 已启动；rx 为空返回 SENSOR_RESP_FORMAT_ERROR，HAL 启动失败返回 COMM_UART_TRANSFER_ERROR。
 */
static uint32_t MULTIPARAM_V3_StartFixedReceiveDma(uint8_t rx[8])
{
    const uint16_t expect_len = 8U;

    if (rx == NULL) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    for (uint16_t i = 0U; i < expect_len; i++) {
        s_multiparam_v3_dma_rx_buf[i] = 0U;
        rx[i] = 0U;
    }

    MULTIPARAM_V3_StopDmaReceive();
    if (HAL_UART_Receive_DMA(&huart6, s_multiparam_v3_dma_rx_buf, expect_len) != HAL_OK) {
        MULTIPARAM_V3_RecordDiagnostic("接收DMA启动失败", 0U);
        MULTIPARAM_V3_StopDmaReceive();
        return COMM_UART_TRANSFER_ERROR;
    }
    return NO_ERROR;
}

/**
 * @brief 等待 多参数传感器通信协议 V3.0 固定 8 字节应答接收完成。
 *
 * 该函数只判断是否收满一帧和是否出现 UART 硬件错误，求和校验与功能码校验仍由调用方完成。
 *
 * @param rx 接收到的数据缓冲区。该参数指向固定 8 字节帧；函数按职责解析字段，必要时在局部调用链内复用可写缓冲区。
 * @param timeout 本次操作使用的超时门限。
 * @return NO_ERROR 表示固定 8 字节应答已完整收到；rx 为空返回 SENSOR_RESP_FORMAT_ERROR，命令切换返回 STATE_SWITCH，HAL 错误返回 COMM_UART_TRANSFER_ERROR，无字节超时返回 SENSOR_DEVICE_COMM_TIMEOUT，不足 8 字节返回 SENSOR_RESP_FORMAT_ERROR。
 */
static uint32_t MULTIPARAM_V3_WaitFixedReceiveDma(uint8_t rx[8], uint32_t timeout)
{
    const uint16_t expect_len = 8U;
    uint32_t startTick;

    if (rx == NULL) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < timeout) {
        uint16_t got = MULTIPARAM_V3_GetDmaReceivedLength(expect_len);

        if (HasEffectiveCommandSwitchRequest()) {
            MULTIPARAM_V3_StopDmaReceive();
            return STATE_SWITCH;
        }
        /* 固定长度应答尚未收满时若 UART6 报错，记录当时已收字节数并停止 DMA，禁止把短帧继续交给校验。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            MULTIPARAM_V3_RecordDiagnostic("接收DMA硬件错误", got);
            MULTIPARAM_V3_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }
        if (got >= expect_len) {
            for (uint16_t i = 0U; i < expect_len; i++) {
                rx[i] = s_multiparam_v3_dma_rx_buf[i];
            }
            MULTIPARAM_V3_StopDmaReceive();
            return NO_ERROR;
        }
        /* 固定长度应答轮询每 1 ms 检查一次 DMA 已收长度，避免在接收门限内持续忙等。 */
        HAL_Delay(1);
    }

    {
        uint16_t got = MULTIPARAM_V3_GetDmaReceivedLength(expect_len);
        for (uint16_t i = 0U; (i < got) && (i < expect_len); i++) {
            rx[i] = s_multiparam_v3_dma_rx_buf[i];
        }
        MULTIPARAM_V3_RecordDiagnostic((got == 0U) ? "等待应答超时" : "应答长度不足", got);
        MULTIPARAM_V3_StopDmaReceive();
        return (got == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
    }
}
/**
 * @brief 按 多参数传感器通信协议 V3.0 固定 8 字节帧完成一次 UART6 DMA 收发；帧由地址、ASCII 功能码、4 字节数据、参数码和前 7 字节累加和组成。
 *
 * @param tx 准备经 UART6 DMA 发送的只读 8 字节 多参数传感器通信协议 V3.0 请求帧。
 * @param rx 接收到的数据缓冲区。该参数指向固定 8 字节帧；函数按职责解析字段，必要时在局部调用链内复用可写缓冲区。
 * @return NO_ERROR 表示固定帧发送、接收和前 7 字节累加和校验均成功；其他值为接收启动或等待错误、UART6 发送错误或 SENSOR_BCC_ERROR。
 */
static int MULTIPARAM_V3_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_DSM
	printf("[多参数协议V3.0] 发送: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", tx[i]);
	printf("\r\n");
#endif
	UART6_DrainRX_UntilIdle(5); /* 发前清空残留数据 */
	uint32_t rx_ret = MULTIPARAM_V3_StartFixedReceiveDma(rx);
	if (rx_ret != NO_ERROR) {
		return (int)rx_ret;
	}

	if (HAL_UART_Transmit_DMA(&huart6, (uint8_t*) tx, 8) != HAL_OK) {
#ifdef DEBUG_DSM
#endif
		MULTIPARAM_V3_RecordDiagnostic("发送DMA启动失败", 0U);
		MULTIPARAM_V3_StopDmaReceive();
		return COMM_UART_TRANSFER_ERROR;
	}

	rx_ret = MULTIPARAM_V3_WaitTransmitDmaDone(DSM_CMD_TIMEOUT);
	if (rx_ret != NO_ERROR) {
		return (int)rx_ret;
	}

	rx_ret = MULTIPARAM_V3_WaitFixedReceiveDma(rx, MULTIPARAM_V3_RX_TIMEOUT);
	if (rx_ret != NO_ERROR) {
		return (int)rx_ret;
	}
#ifdef DEBUG_DSM
	printf("[多参数协议V3.0] 接收: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", rx[i]);
	printf("\r\n");
#endif

	if (MULTIPARAM_V3_CalcSum(rx) != rx[7]) {
#ifdef DEBUG_DSM
#endif
		MULTIPARAM_V3_RecordDiagnostic("应答求和校验错误", 8U);
		return SENSOR_BCC_ERROR;
	}
	return NO_ERROR;
}

/**
 * @brief 核对 多参数传感器通信协议 V3.0 应答的功能码、参数码和从机错误标志。
 *
 * @param tx 本次只读 8 字节请求帧，用于核对应答中的功能码和参数码是否与请求一致。
 * @param rx 接收到的数据缓冲区。该参数指向固定 8 字节请求帧，函数按协议字段位置只读地址、功能、数据和校验字节。
 * @return NO_ERROR 表示功能码、参数码和从机状态均与请求一致；帧字段不一致返回 SENSOR_RESP_FORMAT_ERROR，从机参数字节为 0xFF 返回 SENSOR_REMOTE_INTERNAL_ERROR。
 */
static int MULTIPARAM_V3_CheckReply(const uint8_t tx[8], const uint8_t rx[8]) {
	uint8_t expect_func = tx[1] | 0x80; /* 从机高位置1 */
	uint8_t expect_param = ((tx[1] == (uint8_t)MULTIPARAM_V3_MODE_LEVEL) ||
	                        (tx[1] == (uint8_t)MULTIPARAM_V3_MODE_DENSITY)) ? 0x00U : tx[6];

	if (rx[1] != expect_func) {
#ifdef DEBUG_DSM
		printf("[多参数协议V3.0] 接收功能码不匹配: 期望：%02X, 实际=%02X\r\n", expect_func, rx[1]);
#endif
		MULTIPARAM_V3_RecordDiagnostic("应答功能码不匹配", 8U);
		return SENSOR_RESP_FORMAT_ERROR;
	}
	if (rx[6] == 0xFFU) {
		/* 远端仅给出粗粒度失败，不解析数据或猜测更具体原因。 */
		MULTIPARAM_V3_RecordDiagnostic("应答远端粗粒度失败", 8U);
		return SENSOR_REMOTE_INTERNAL_ERROR;
	}
	if (rx[6] != expect_param) {
#ifdef DEBUG_DSM
		printf("[多参数协议V3.0] 接收参数不匹配: 期望：%02X, 实际=%02X\r\n", expect_param, rx[6]);
#endif
		MULTIPARAM_V3_RecordDiagnostic("应答参数码不匹配", 8U);
		return SENSOR_RESP_FORMAT_ERROR;
	}
	return NO_ERROR;
}

/**
 * @brief 从 多参数传感器通信协议 V3.0 应答数据区按低字节在前还原 32 位有符号整数。
 *
 * @param d 包含 多参数传感器通信协议 V3.0 小端 32 位数据的 4 字节只读缓冲区。
 * @return 返回从连续 4 字节低字节在前数据还原的 32 位有符号整数。
 */
static inline int32_t MULTIPARAM_V3_ParseInt32_LE(const uint8_t *d) {
	return (int32_t) ((uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0]);
}
/**
 * @brief 按小端字节序解析 float，用于 多参数传感器通信协议 V3.0 协议浮点参数。
 *
 * @param d 包含 多参数传感器通信协议 V3.0 小端 32 位数据的 4 字节只读缓冲区。
 * @return 返回从 4 字节小端 IEEE 754 位模式还原的单精度浮点值。
 */
static inline float MULTIPARAM_V3_ParseFloat_LE(const uint8_t *d) {
	union {
		/* 无名称联合体的无符号整数与单精度浮点数位模式视图。 */
		uint32_t u; /* 按无符号整数字项解释的同一位模式视图。 */
		float f; /* 按 IEEE 754 单精度浮点数解释的位模式视图。 */
	} cvt;
	cvt.u = (uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0];
	return cvt.f;
}

/**
 * @brief 向 多参数传感器通信协议 V3.0 传感器下发模式切换请求并核对模式回显。
 *
 * @param mode 准备写入 多参数传感器通信协议 V3.0 模式切换帧的 multiparam_v3_mode_t 模式字符。
 * @return NO_ERROR 表示在重试次数内完成模式切换并确认回显；命令切换立即返回 STATE_SWITCH，全部尝试失败时返回最后一次收发、校验、格式或远端错误。
 */
int MULTIPARAM_V3_SwitchMode(multiparam_v3_mode_t mode) {
	uint8_t tx[8], rx[8];
	int last_err = SENSOR_DEVICE_COMM_TIMEOUT;

	MULTIPARAM_V3_MakeFrame(tx, (uint8_t) mode, 0x00000000u, 0x00U); /* 模式切换帧参数码固定为 0x00 */

	for (int attempt = 0; attempt < MULTIPARAM_V3_MAX_RETRY; ++attempt) {
		if (HasEffectiveCommandSwitchRequest()) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 每次模式切换命令前等待 DSM_PRE_SEND_DELAY，为上一帧收尾和 UART6 事务切换留出间隔。 */
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = MULTIPARAM_V3_Transceive(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		if (ret != NO_ERROR) {
			last_err = ret;
			MULTIPARAM_V3_LogRetry(ERROR_LOG_OP_SWITCH_MODE,
			                (uint32_t)ret,
			                (uint32_t)(attempt + 1),
			                tx,
			                rx);
			continue;
		}

		ret = MULTIPARAM_V3_CheckReply(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		if (ret == NO_ERROR) {
			if (attempt > 0) {
				ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
				                 ERROR_LOG_OP_SWITCH_MODE,
				                 ErrorLog_GetReasonByCode((uint32_t)last_err),
				                 (uint32_t)(attempt + 1),
				                 MULTIPARAM_V3_MAX_RETRY);
			}
#ifdef DEBUG_DSM
			printf("[多参数协议V3.0] 切换模式'%c'成功\r\n", (char) mode);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		MULTIPARAM_V3_LogRetry(ERROR_LOG_OP_SWITCH_MODE,
		                (uint32_t)ret,
		                (uint32_t)(attempt + 1),
		                tx,
		                rx);
		/* 本次模式切换校验未通过时等待 DSM_BCC_DELAY 后再重试，避免上一应答尾部污染下一帧。 */
		HAL_Delay(DSM_BCC_DELAY);
	}
#ifdef DEBUG_DSM
	printf("[多参数协议V3.0] 切换模式'%c'失败, 错误码：%d\r\n", (char) mode, last_err);
#endif
	return last_err;
}
/**
 * @brief 通过 多参数传感器通信协议 V3.0 命令切换传感器到液位模式。
 * @return 返回液位模式切换结果码；NO_ERROR 表示传感器已确认，其他值为条件、通信或应答校验错误。
 */
int MULTIPARAM_V3_SwitchToLevelMode(void) {
	return MULTIPARAM_V3_SwitchMode(MULTIPARAM_V3_MODE_LEVEL);
}
/**
 * @brief 通过 多参数传感器通信协议 V3.0 命令切换传感器到密度模式。
 * @return NO_ERROR 表示传感器已确认切换到密度模式；STATE_SWITCH 表示被新命令正常打断，其他值为重试结束后的最后一次传输或应答校验错误。
 */
int MULTIPARAM_V3_SwitchToDensityMode(void) {
	return MULTIPARAM_V3_SwitchMode(MULTIPARAM_V3_MODE_DENSITY);
}

/**
 * @brief 读取指定 多参数传感器通信协议 V3.0 浮点参数，并统一执行重试、应答校验和错误日志。
 *
 * @param param 多参数传感器通信协议 V3.0 参数码，决定本次读写的传感器寄存器。
 * @param out_value 用于返回读取或解析得到的参数值。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
int MULTIPARAM_V3_Read_FloatParam(uint8_t param, float *out_value) {
	if (!out_value)
		return SYSTEM_CALL_CONDITION_ERROR;

	uint8_t tx[8], rx[8];
	int last_err = SENSOR_DEVICE_COMM_TIMEOUT;

	MULTIPARAM_V3_MakeFrame(tx, (uint8_t) MULTIPARAM_V3_FUNC_R, 0x00000000u, param);

	for (int attempt = 0; attempt < MULTIPARAM_V3_MAX_RETRY; ++attempt) {
		if (HasEffectiveCommandSwitchRequest()) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 读取浮点参数前等待 DSM_PRE_SEND_DELAY，确保上一条 LTD 事务已经完全结束。 */
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = MULTIPARAM_V3_Transceive(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		if (ret != NO_ERROR) {
			last_err = ret;
			MULTIPARAM_V3_LogRetry(ERROR_LOG_OP_READ_FLOAT_PARAM,
			                (uint32_t)ret,
			                (uint32_t)(attempt + 1),
			                tx,
			                rx);
			continue;
		}

		ret = MULTIPARAM_V3_CheckReply(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		if (ret == NO_ERROR) {
			float v = MULTIPARAM_V3_ParseFloat_LE(rx + 2);
			*out_value = v;
            if (attempt > 0) {
                ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                 ERROR_LOG_OP_READ_FLOAT_PARAM,
                                 ErrorLog_GetReasonByCode((uint32_t)last_err),
                                 (uint32_t)(attempt + 1),
                                 MULTIPARAM_V3_MAX_RETRY);
            }
#ifdef DEBUG_DSM
			printf("[多参数协议V3.0] 读取浮点寄存器 R%u: %f\r\n", (unsigned) param, (double) v);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		MULTIPARAM_V3_LogRetry(ERROR_LOG_OP_READ_FLOAT_PARAM,
		                (uint32_t)ret,
		                (uint32_t)(attempt + 1),
		                tx,
		                rx);
		/* 浮点参数读取失败后等待 DSM_BCC_DELAY 再重试，避免连续请求压缩传感器应答间隔。 */
		HAL_Delay(DSM_BCC_DELAY);
	}
	return last_err;
}


/**
 * @brief 读取 多参数传感器通信协议 V3.0 整型参数，支持识别探测阶段抑制重试日志。
 *
 * 正式读取保持错误重试日志，自动识别候选协议未命中时只返回错误码。
 *
 * @param param 多参数传感器通信协议 V3.0 参数码，决定本次读写的传感器寄存器。
 * @param out_value 用于返回读取或解析得到的参数值。
 * @param log_retry 日志。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 * @note 不改变通信重试次数和返回码，只控制是否打印统一错误日志。
 */
static int MULTIPARAM_V3_Read_IntParamInternal(uint8_t param, int32_t *out_value, uint8_t log_retry) {
	if (!out_value)
		return SYSTEM_CALL_CONDITION_ERROR;

	uint8_t tx[8], rx[8];
	int last_err = SENSOR_DEVICE_COMM_TIMEOUT;

	MULTIPARAM_V3_MakeFrame(tx, (uint8_t) MULTIPARAM_V3_FUNC_R, 0x00000000u, param);

	for (int attempt = 0; attempt < MULTIPARAM_V3_MAX_RETRY; ++attempt) {
		if (HasEffectiveCommandSwitchRequest()) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 读取整数参数前等待 DSM_PRE_SEND_DELAY，确保 UART6 和传感器均已退出上一事务。 */
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = MULTIPARAM_V3_Transceive(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		if (ret != NO_ERROR) {
			last_err = ret;
			if (log_retry != 0U) {
				MULTIPARAM_V3_LogRetry(ERROR_LOG_OP_READ_INT_PARAM,
				                (uint32_t)ret,
				                (uint32_t)(attempt + 1),
				                tx,
				                rx);
			}
			continue;
		}

		ret = MULTIPARAM_V3_CheckReply(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		if (ret == NO_ERROR) {
			int32_t v = MULTIPARAM_V3_ParseInt32_LE(rx + 2);
			*out_value = v;
			if ((attempt > 0) && (log_retry != 0U)) {
				ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
				                 ERROR_LOG_OP_READ_INT_PARAM,
				                 ErrorLog_GetReasonByCode((uint32_t)last_err),
				                 (uint32_t)(attempt + 1),
				                 MULTIPARAM_V3_MAX_RETRY);
			}
#ifdef DEBUG_DSM
			printf("[多参数协议V3.0] 读取整数寄存器 R%u: %ld (0x%08lX)\r\n", (unsigned) param, (long) v, (unsigned long) v);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		if (log_retry != 0U) {
			MULTIPARAM_V3_LogRetry(ERROR_LOG_OP_READ_INT_PARAM,
			                (uint32_t)ret,
			                (uint32_t)(attempt + 1),
			                tx,
			                rx);
		}
		/* 整数参数读取失败后等待 DSM_BCC_DELAY 再重试，防止上一帧残留字节参与下一次 BCC 校验。 */
		HAL_Delay(DSM_BCC_DELAY);
	}
	return last_err;
}

/**
 * @brief 正式读取 多参数传感器通信协议 V3.0 整数寄存器，并使用统一重试和日志策略。
 *
 * @param param 多参数传感器通信协议 V3.0 参数码，决定本次读写的传感器寄存器。
 * @param out_value 用于返回读取或解析得到的参数值。
 * @return 返回整机错误码；NO_ERROR 表示整数参数已写入 out_value，其他值表示参数非法、命令切换、通信或应答校验失败。
 */
int MULTIPARAM_V3_Read_IntParam(uint8_t param, int32_t *out_value) {
	return MULTIPARAM_V3_Read_IntParamInternal(param, out_value, 1U);
}

/**
 * @brief 读取 多参数传感器通信协议 V3.0 传感器软件版本参数。
 *
 * @param v 用于返回 多参数传感器通信协议 V3.0 应答中的软件版本浮点值。
 * @return 返回软件版本参数读取结果码；NO_ERROR 表示 v 已更新，其他值为条件、通信或应答校验错误。
 */
int MULTIPARAM_V3_Read_SoftwareVersion(float *v) {
	return MULTIPARAM_V3_Read_FloatParam(0x00, v);
}
/**
 * @brief 读取 多参数传感器通信协议 V3.0 传感器温度参数。
 * @param t 温度输出指针。
 * @return 0 表示读取成功，非 0 表示通信失败。
 */
int MULTIPARAM_V3_Read_Temperature(float *t) {
	return MULTIPARAM_V3_Read_FloatParam(0x06, t);
}
/**
 * @brief 读取 LTD 传感器实时密度。
 *
 * @param rho 用于返回传感器应答中的实时密度值，工程单位按多参数传感器通信协议 V3.0定义。
 * @return NO_ERROR 表示 多参数传感器通信协议 V3.0 R07 浮点密度已写入 rho；输出指针非法、命令切换、传输失败或应答校验失败时返回对应错误码。
 */
int MULTIPARAM_V3_Read_Density(float *rho) {
	return MULTIPARAM_V3_Read_FloatParam(0x07, rho);
}
/**
 * @brief 读取 LTD 传感器动力黏度。
 *
 * @param mu 用于返回传感器应答中的动力黏度值，工程单位按多参数传感器通信协议 V3.0定义。
 * @return NO_ERROR 表示 多参数传感器通信协议 V3.0 R08 动力黏度浮点值已写入 mu；输出指针非法、命令切换、传输失败或应答校验失败时返回对应错误码。
 */
int MULTIPARAM_V3_Read_DynamicViscosity(float *mu) {
	return MULTIPARAM_V3_Read_FloatParam(0x08, mu);
}
/**
 * @brief 读取 LTD 传感器运动黏度。
 *
 * @param nu 用于返回传感器应答中的运动黏度值，工程单位按多参数传感器通信协议 V3.0定义。
 * @return NO_ERROR 表示 多参数传感器通信协议 V3.0 R09 运动黏度浮点值已写入 nu；输出指针非法、命令切换、传输失败或应答校验失败时返回对应错误码。
 */
int MULTIPARAM_V3_Read_KinematicViscosity(float *nu) {
	return MULTIPARAM_V3_Read_FloatParam(0x09, nu);
}
/**
 * @brief 读取 LTD 传感器 45° 均方值。
 *
 * @param msq45 用于返回传感器 45° 振动通道的均方值。
 * @return 返回 45° 均方值读取结果码；NO_ERROR 表示 msq45 已更新，其他值为通信或应答校验错误。
 */
int MULTIPARAM_V3_Read_MeanSquare45(float *msq45) {
	return MULTIPARAM_V3_Read_FloatParam(0x11, msq45);
} /* 17 */
/**
 * @brief 读取 LTD 传感器 22.5° 均方值。
 *
 * @param msq22p5 用于返回传感器 22.5° 振动通道的均方值。
 * @return 返回 22.5° 均方值读取结果码；NO_ERROR 表示 msq22p5 已更新，其他值为通信或应答校验错误。
 */
int MULTIPARAM_V3_Read_MeanSquare22p5(float *msq22p5) {
	return MULTIPARAM_V3_Read_FloatParam(0x12, msq22p5);
} /* 18 */
/**
 * @brief 读取 LTD 传感器液位通道频率。
 *
 * @param freq_hz 用于返回传感器频率的输出参数，单位 Hz。
 * @return NO_ERROR 表示 R04 液位频率已解码并写入 freq_hz；输出指针为空返回 SYSTEM_CALL_CONDITION_ERROR，其他值为请求收发或应答检查的具体错误。
 * @note 该接口读取参数码 R04 的液位频率整数值。
 */
int MULTIPARAM_V3_Read_LevelFrequency(uint32_t *freq_hz) {
	if (!freq_hz)
		return SYSTEM_CALL_CONDITION_ERROR;
	int32_t v = 0;
	int ret = MULTIPARAM_V3_Read_IntParam(0x04, &v);   /* 参数码 0x04 = R04 */
	if (ret == NO_ERROR) {
		if (v < 0) {
			v = -v;
		}
		if (v > 10000) {
			v = v - 10000;
		}
		*freq_hz = (uint32_t) v;
	}
	return ret;
}
/**
 * @brief 读取 LTD 传感器密度通道频率。
 *
 * @param freq_hz 用于返回传感器频率的输出参数，单位 Hz。
 * @param freq_45 用于返回 45° 振动通道频率值。
 * @param freq_225 用于返回 22.5° 振动通道频率值。
 * @return NO_ERROR 表示 R16 的主频率、45° 和 22.5° 三个值均已解码并写入输出；任一输出指针为空返回 SYSTEM_CALL_CONDITION_ERROR，其他值为请求收发或应答检查的具体错误。
 * @note 该接口读取参数码 R16 的密度通道频率整数值；旧注释中的“液位频率”属于复制错误。
 */
int MULTIPARAM_V3_Read_DensityFrequency(float *freq_hz,float *freq_45,float *freq_225) {
	if (!freq_hz || !freq_45 || !freq_225)
		return SYSTEM_CALL_CONDITION_ERROR;
	float v = 0;
	int ret = MULTIPARAM_V3_Read_FloatParam(0x11, &v);   /* 参数码 0x11 = 45度扫频平方均值 */
	if (ret == NO_ERROR) {
		*freq_45 = v;
		if (v > 0.0f) {
			*freq_hz = (float)sqrt(1000000000000.0/(double) v);
		} else {
			*freq_hz = 0.0f;
			printf("LTD频率周期平方均值为0，等待稳定/未测到\r\n");
		}
	}
	ret = MULTIPARAM_V3_Read_FloatParam(0x12, &v);   /* 参数码 0x12 = 22.5度扫频平方均值 */
	if (ret == NO_ERROR) {
		*freq_225 =  v;
		if (v <= 0.0f) {
			printf("LTD 22.5度扫频周期平方均值为0，等待稳定/未测到\r\n");
		}
	}
	return ret;
}
/**
 * @brief 读取 LTD 传感器设备编号。
 *
 * @param sensor_id 用于返回探测到的传感器编号。
 * @return NO_ERROR 表示设备编号应答已通过校验并写入 sensor_id；输出指针为空返回 SYSTEM_CALL_CONDITION_ERROR，其他值为请求收发、格式、校验或远端错误。
 */
int MULTIPARAM_V3_Read_SensorID(uint32_t *sensor_id) {
	if (!sensor_id)
		return SYSTEM_CALL_CONDITION_ERROR;
	int32_t v = 0;
	int ret = MULTIPARAM_V3_Read_IntParam(0x16, &v); /* 22 */
	if (ret == NO_ERROR)
		*sensor_id = (uint32_t) v;
	return ret;
}

/**
 * @brief 在传感器自动识别阶段静默读取 多参数传感器通信协议 V3.0 的 R22 设备编号。
 *
 * 函数以参数码 R22 调用无日志版本的整数读取接口，只有完整通信和应答校验成功时才写入 sensor_id。
 *
 * @param sensor_id 用于返回探测到的传感器编号。
 * @return NO_ERROR 表示 R22 设备编号已写入 sensor_id；SYSTEM_CALL_CONDITION_ERROR 表示输出指针为空；其他值为 UART6
 *         传输、累加和、应答格式或远端错误码。
 * @note 探测失败仅作为候选未命中返回；底层读取关闭重试错误日志，避免自动识别阶段反复刷屏。
 */
int MULTIPARAM_V3_Probe_SensorID(uint32_t *sensor_id) {
	if (!sensor_id)
		return SYSTEM_CALL_CONDITION_ERROR;
	int32_t v = 0;
	int ret = MULTIPARAM_V3_Read_IntParamInternal(0x16, &v, 0U); /* 22 */
	/* 识别阶段的协议探测失败属于候选未命中，不打印错误重试。 */
	if (ret == NO_ERROR)
		*sensor_id = (uint32_t) v;
	return ret;
}

