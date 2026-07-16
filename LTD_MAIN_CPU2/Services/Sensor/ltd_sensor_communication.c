/*
 * dsm_v2.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Duan Xuebin
 */

#include <ltd_sensor_communication.h>
#include "system_parameter.h"
#include "sensor.h"
#include "error_log.h"
#include <math.h>
#include <stdio.h>

#ifndef DSM_V2_MAX_RETRY
#define DSM_V2_MAX_RETRY   UART6_COMM_MAX_RETRY /* 传感器通信参数：传感器 V2 最大值 重试。 */
#endif
#ifndef DSM_V2_RX_TIMEOUT
#define DSM_V2_RX_TIMEOUT  DSM_CMD_TIMEOUT /* 传感器通信参数：传感器 V2 RX 超时。 */
#endif

/* 主机方向功能码 */
typedef enum {
	DSM_V2_FUNC_R = 'R', DSM_V2_FUNC_W = 'W', DSM_V2_FUNC_L = 'T', DSM_V2_FUNC_D = 'D', DSM_V2_FUNC_B = 'B',
} dsm_v2_func_t;

/* === 内部：求和校验（前7字节） === */
static inline uint8_t DSM_V2_CalcSum(const uint8_t f[8]) {
	uint32_t s = 0;
	for (int i = 0; i < 7; ++i)
		s += f[i];
	return (uint8_t) (s & 0xFF);
}

/* === 内部：打帧（addr=0x00，data 大端入参；若给 0 则无所谓端序） === */
static inline void DSM_V2_MakeFrame(uint8_t out[8], uint8_t func, uint32_t data_be, uint8_t param) {
	out[0] = 0x00;
	out[1] = func;
	out[2] = (uint8_t) ((data_be >> 24) & 0xFF);
	out[3] = (uint8_t) ((data_be >> 16) & 0xFF);
	out[4] = (uint8_t) ((data_be >> 8) & 0xFF);
	out[5] = (uint8_t) (data_be & 0xFF);
	out[6] = param;
	out[7] = DSM_V2_CalcSum(out);
}

/* === 内部：传输 8→8 === */

/* === 1) 发前清空可能的残留：非正式“flush” === */
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

static uint8_t s_dsm_v2_dma_rx_buf[8]; /* LTD 传感器通信数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */
static const char *s_dsm_v2_last_stage = "未开始";
static uint32_t s_dsm_v2_last_uart_error = HAL_UART_ERROR_NONE;
static uint16_t s_dsm_v2_last_received_length = 0U;

/* 保存 LTD/V2 最近一次失败阶段，供重试日志定位。 */
static void DSM_V2_RecordDiagnostic(const char *stage, uint16_t received_length)
{
    s_dsm_v2_last_stage = (stage != NULL) ? stage : "未知阶段";
    s_dsm_v2_last_uart_error = huart6.ErrorCode;
    s_dsm_v2_last_received_length = received_length;
}

/* 在统一重试日志中附带完整的 8 字节请求和当前应答。 */
static void DSM_V2_LogRetry(const char *operation,
                            uint32_t error_code,
                            uint32_t attempt,
                            const uint8_t tx[8],
                            const uint8_t rx[8])
{
    char detail[256];

    (void)snprintf(detail,
                   sizeof(detail),
                   "阶段=%s,UART错误=0x%08lX,接收长度=%u,发送=%02X %02X %02X %02X %02X %02X %02X %02X,接收=%02X %02X %02X %02X %02X %02X %02X %02X",
                   s_dsm_v2_last_stage,
                   (unsigned long)s_dsm_v2_last_uart_error,
                   (unsigned int)s_dsm_v2_last_received_length,
                   tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7],
                   rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
    ErrorLog_RetryDetail(ERROR_LOG_MODULE_SENSOR,
                         operation,
                         ErrorLog_GetReasonByCode(error_code),
                         attempt,
                         DSM_V2_MAX_RETRY,
                         error_code,
                         detail);
}

/**
 * @brief 停止 UART6 DMA 接收并清理固定 8 字节协议的硬件错误状态。
 *
 * LTD/V2 传感器是一问一答固定帧，任何提前返回都必须停止 DMA，避免下次收发继续占用 UART6。
 */
static void DSM_V2_StopDmaReceive(void)
{
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
 * @brief 获取 UART6 DMA 当前已收到的字节数。
 *
 * 固定 8 字节协议只关心是否收满 8 字节；句柄异常时返回 0，由上层按通信超时处理。
 */
static uint16_t DSM_V2_GetDmaReceivedLength(uint16_t rx_len)
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
 * @brief 等待 LTD/V2 的 UART6 DMA 发送完成。
 *
 * 接收 DMA 已提前启动，发送阶段只等待 TX 状态回到 READY；异常退出时停止 DMA，避免占用后续收发。
 */
static uint32_t DSM_V2_WaitTransmitDmaDone(uint32_t timeout)
{
    uint32_t startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeout) {
        if (HasEffectiveCommandSwitchRequest()) {
            DSM_V2_StopDmaReceive();
            return STATE_SWITCH;
        }
        if (huart6.gState == HAL_UART_STATE_READY) {
            return NO_ERROR;
        }
        /* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            DSM_V2_RecordDiagnostic("发送DMA硬件错误", 0U);
            DSM_V2_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }
        /* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(1);
    }

    DSM_V2_RecordDiagnostic("发送DMA等待超时", 0U);
    DSM_V2_StopDmaReceive();
    return COMM_UART_TRANSFER_ERROR;
}

/**
 * @brief 启动 LTD/V2 固定 8 字节应答的 UART6 DMA 接收。
 *
 * 该函数只负责提前打开接收窗口，避免发送完成后再启动 DMA 导致快速回包丢头。
 */
static uint32_t DSM_V2_StartFixedReceiveDma(uint8_t rx[8])
{
    const uint16_t expect_len = 8U;

    if (rx == NULL) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    for (uint16_t i = 0U; i < expect_len; i++) {
        s_dsm_v2_dma_rx_buf[i] = 0U;
        rx[i] = 0U;
    }

    DSM_V2_StopDmaReceive();
    if (HAL_UART_Receive_DMA(&huart6, s_dsm_v2_dma_rx_buf, expect_len) != HAL_OK) {
        DSM_V2_RecordDiagnostic("接收DMA启动失败", 0U);
        DSM_V2_StopDmaReceive();
        return COMM_UART_TRANSFER_ERROR;
    }
    return NO_ERROR;
}

/**
 * @brief 等待 LTD/V2 固定 8 字节应答接收完成。
 *
 * 该函数只判断是否收满一帧和是否出现 UART 硬件错误，求和校验与功能码校验仍由调用方完成。
 */
static uint32_t DSM_V2_WaitFixedReceiveDma(uint8_t rx[8], uint32_t timeout)
{
    const uint16_t expect_len = 8U;
    uint32_t startTick;

    if (rx == NULL) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < timeout) {
        uint16_t got = DSM_V2_GetDmaReceivedLength(expect_len);

        if (HasEffectiveCommandSwitchRequest()) {
            DSM_V2_StopDmaReceive();
            return STATE_SWITCH;
        }
        /* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            DSM_V2_RecordDiagnostic("接收DMA硬件错误", got);
            DSM_V2_StopDmaReceive();
            return COMM_UART_TRANSFER_ERROR;
        }
        if (got >= expect_len) {
            for (uint16_t i = 0U; i < expect_len; i++) {
                rx[i] = s_dsm_v2_dma_rx_buf[i];
            }
            DSM_V2_StopDmaReceive();
            return NO_ERROR;
        }
        /* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(1);
    }

    {
        uint16_t got = DSM_V2_GetDmaReceivedLength(expect_len);
        for (uint16_t i = 0U; (i < got) && (i < expect_len); i++) {
            rx[i] = s_dsm_v2_dma_rx_buf[i];
        }
        DSM_V2_RecordDiagnostic((got == 0U) ? "等待应答超时" : "应答长度不足", got);
        DSM_V2_StopDmaReceive();
        return (got == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
    }
}
/**
 * @brief 执行LTD 传感器通信中的 DSM_V2_Transceive 逻辑。
 *
 * @param tx 业务参数。
 * @param rx 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int DSM_V2_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_DSM
	printf("V2发送: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", tx[i]);
	printf("\r\n");
#endif
	UART6_DrainRX_UntilIdle(5); /* 发前清空残留数据 */
	uint32_t rx_ret = DSM_V2_StartFixedReceiveDma(rx);
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
	if (rx_ret != NO_ERROR) {
		return (int)rx_ret;
	}

	if (HAL_UART_Transmit_DMA(&huart6, (uint8_t*) tx, 8) != HAL_OK) {
#ifdef DEBUG_DSM
#endif
		DSM_V2_RecordDiagnostic("发送DMA启动失败", 0U);
		DSM_V2_StopDmaReceive();
		return COMM_UART_TRANSFER_ERROR;
	}

	rx_ret = DSM_V2_WaitTransmitDmaDone(DSM_CMD_TIMEOUT);
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
	if (rx_ret != NO_ERROR) {
		return (int)rx_ret;
	}

	rx_ret = DSM_V2_WaitFixedReceiveDma(rx, DSM_V2_RX_TIMEOUT);
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
	if (rx_ret != NO_ERROR) {
		return (int)rx_ret;
	}
#ifdef DEBUG_DSM
	printf("V2接收: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", rx[i]);
	printf("\r\n");
#endif

	if (DSM_V2_CalcSum(rx) != rx[7]) {
#ifdef DEBUG_DSM
#endif
		DSM_V2_RecordDiagnostic("应答求和校验错误", 8U);
		return SENSOR_BCC_ERROR;
	}
	return NO_ERROR;
}

/* === 内部：校验功能码/参数码 === */
static int DSM_V2_CheckReply(const uint8_t tx[8], const uint8_t rx[8]) {
	uint8_t expect_func = tx[1] | 0x80; /* 从机高位置1 */
	uint8_t expect_param = ((tx[1] == (uint8_t)DSM_V2_MODE_LEVEL) ||
	                        (tx[1] == (uint8_t)DSM_V2_MODE_DENSITY)) ? 0x00U : tx[6];

	if (rx[1] != expect_func) {
#ifdef DEBUG_DSM
		printf("V2接收功能码不匹配: 期望：%02X, 实际=%02X\r\n", expect_func, rx[1]);
#endif
		DSM_V2_RecordDiagnostic("应答功能码不匹配", 8U);
		return SENSOR_RESP_FORMAT_ERROR;
	}
	if (rx[6] == 0xFFU) {
		printf("V2应答参数为0xFF，仅记录原始帧，不置错误：%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
		       rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
		return NO_ERROR;
	}
	if (rx[6] != expect_param) {
#ifdef DEBUG_DSM
		printf("V2接收参数不匹配: 期望：%02X, 实际=%02X\r\n", expect_param, rx[6]);
#endif
		DSM_V2_RecordDiagnostic("应答参数码不匹配", 8U);
		return SENSOR_RESP_FORMAT_ERROR;
	}
	return NO_ERROR;
}

/* === 内部：端序解析 === */
static inline int32_t DSM_V2_ParseInt32_LE(const uint8_t *d) {
	return (int32_t) ((uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0]);
}
/* 按小端字节序解析 float，用于 LTD/DSM V2 协议浮点参数。 */
static inline float DSM_V2_ParseFloat_LE(const uint8_t *d) {
	union {
		uint32_t u;
		float f;
	} cvt;
	cvt.u = (uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0];
	return cvt.f;
}

/* === 对外：切换模式（param=0x00） === */
int DSM_V2_SwitchMode(dsm_v2_mode_t mode) {
	uint8_t tx[8], rx[8];
	int last_err = SENSOR_DEVICE_COMM_TIMEOUT;

	DSM_V2_MakeFrame(tx, (uint8_t) mode, 0x00000000u, 0x00U); /* 模式切换帧参数码固定为 0x00 */

	for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
		if (HasEffectiveCommandSwitchRequest()) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = DSM_V2_Transceive(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
		if (ret != NO_ERROR) {
			last_err = ret;
			DSM_V2_LogRetry(ERROR_LOG_OP_SWITCH_MODE,
			                (uint32_t)ret,
			                (uint32_t)(attempt + 1),
			                tx,
			                rx);
			continue;
		}

		ret = DSM_V2_CheckReply(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
		if (ret == NO_ERROR) {
			if (attempt > 0) {
				/* 错误 阶段：重试成功 模块：传感器 操作：切换模式 原因：通信失败 尝试：(attempt + 1)/DSM_V2_MAX_RETRY */
				ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
				                 ERROR_LOG_OP_SWITCH_MODE,
				                 ERROR_LOG_REASON_COMM_FAIL,
				                 (uint32_t)(attempt + 1),
				                 DSM_V2_MAX_RETRY);
			}
#ifdef DEBUG_DSM
			printf("[V2] 切换模式'%c'成功\r\n", (char) mode);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		DSM_V2_LogRetry(ERROR_LOG_OP_SWITCH_MODE,
		                (uint32_t)ret,
		                (uint32_t)(attempt + 1),
		                tx,
		                rx);
		/* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
		HAL_Delay(DSM_BCC_DELAY);
	}
#ifdef DEBUG_DSM
	printf("[V2] 切换模式'%c'失败, 错误码：%d\r\n", (char) mode, last_err);
#endif
	return last_err;
}
/**
 * @brief 执行LTD 传感器通信中的 DSM_V2_SwitchToLevelMode 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_SwitchToLevelMode(void) {
	return DSM_V2_SwitchMode(DSM_V2_MODE_LEVEL);
}
/**
 * @brief 执行LTD 传感器通信中的 DSM_V2_SwitchToDensityMode 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_SwitchToDensityMode(void) {
	return DSM_V2_SwitchMode(DSM_V2_MODE_DENSITY);
}

/* === 对外：通用读取 === */
int DSM_V2_Read_FloatParam(uint8_t param, float *out_value) {
	if (!out_value)
		return SYSTEM_CALL_CONDITION_ERROR;

	uint8_t tx[8], rx[8];
	int last_err = SENSOR_DEVICE_COMM_TIMEOUT;

	DSM_V2_MakeFrame(tx, (uint8_t) DSM_V2_FUNC_R, 0x00000000u, param);

	for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
		if (HasEffectiveCommandSwitchRequest()) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = DSM_V2_Transceive(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
		if (ret != NO_ERROR) {
			last_err = ret;
			DSM_V2_LogRetry(ERROR_LOG_OP_READ_FLOAT_PARAM,
			                (uint32_t)ret,
			                (uint32_t)(attempt + 1),
			                tx,
			                rx);
			continue;
		}

		ret = DSM_V2_CheckReply(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
		if (ret == NO_ERROR) {
			float v = DSM_V2_ParseFloat_LE(rx + 2);
			*out_value = v;
            if (attempt > 0) {
                /* 错误 阶段：重试成功 模块：传感器 操作：读取浮点参数 原因：通信失败 尝试：(attempt + 1)/DSM_V2_MAX_RETRY */
                ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                 ERROR_LOG_OP_READ_FLOAT_PARAM,
                                 ERROR_LOG_REASON_COMM_FAIL,
                                 (uint32_t)(attempt + 1),
                                 DSM_V2_MAX_RETRY);
            }
#ifdef DEBUG_DSM
			printf("[V2] 读取浮点寄存器 R%u: %f\r\n", (unsigned) param, (double) v);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		DSM_V2_LogRetry(ERROR_LOG_OP_READ_FLOAT_PARAM,
		                (uint32_t)ret,
		                (uint32_t)(attempt + 1),
		                tx,
		                rx);
		/* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
		HAL_Delay(DSM_BCC_DELAY);
	}
	return last_err;
}

/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_IntParam 逻辑。
 *
 * @param param 输入/输出指针。
 * @param out_value 待处理数值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
/*
 * 函数用途：读取 LTD/V2 整型参数，支持识别探测阶段抑制重试日志。
 * 调用场景：正式读取保持错误重试日志，自动识别候选协议未命中时只返回错误码。
 * 关键约束：不改变通信重试次数和返回码，只控制是否打印统一错误日志。
 */
static int DSM_V2_Read_IntParamInternal(uint8_t param, int32_t *out_value, uint8_t log_retry) {
	if (!out_value)
		return SYSTEM_CALL_CONDITION_ERROR;

	uint8_t tx[8], rx[8];
	int last_err = SENSOR_DEVICE_COMM_TIMEOUT;

	DSM_V2_MakeFrame(tx, (uint8_t) DSM_V2_FUNC_R, 0x00000000u, param);

	for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
		if (HasEffectiveCommandSwitchRequest()) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = DSM_V2_Transceive(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
		if (ret != NO_ERROR) {
			last_err = ret;
			if (log_retry != 0U) {
				DSM_V2_LogRetry(ERROR_LOG_OP_READ_INT_PARAM,
				                (uint32_t)ret,
				                (uint32_t)(attempt + 1),
				                tx,
				                rx);
			}
			continue;
		}

		ret = DSM_V2_CheckReply(tx, rx);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}
		/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
		if (ret == NO_ERROR) {
			int32_t v = DSM_V2_ParseInt32_LE(rx + 2);
			*out_value = v;
			if ((attempt > 0) && (log_retry != 0U)) {
				/* 错误 阶段：重试成功 模块：传感器 操作：读取整数参数 原因：通信失败 尝试：(attempt + 1)/DSM_V2_MAX_RETRY */
				ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
				                 ERROR_LOG_OP_READ_INT_PARAM,
				                 ERROR_LOG_REASON_COMM_FAIL,
				                 (uint32_t)(attempt + 1),
				                 DSM_V2_MAX_RETRY);
			}
#ifdef DEBUG_DSM
			printf("[V2] 读取整数寄存器 R%u: %ld (0x%08lX)\r\n", (unsigned) param, (long) v, (unsigned long) v);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		if (log_retry != 0U) {
			DSM_V2_LogRetry(ERROR_LOG_OP_READ_INT_PARAM,
			                (uint32_t)ret,
			                (uint32_t)(attempt + 1),
			                tx,
			                rx);
		}
		/* LTD 传感器通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
		HAL_Delay(DSM_BCC_DELAY);
	}
	return last_err;
}

int DSM_V2_Read_IntParam(uint8_t param, int32_t *out_value) {
	return DSM_V2_Read_IntParamInternal(param, out_value, 1U);
}

/* === 便捷读取 === */
int DSM_V2_Read_SoftwareVersion(float *v) {
	return DSM_V2_Read_FloatParam(0x00, v);
}
/**
 * @brief 读取 LTD/DSM V2 传感器温度参数。
 * @param t 温度输出指针。
 * @return 0 表示读取成功，非 0 表示通信失败。
 */
int DSM_V2_Read_Temperature(float *t) {
	return DSM_V2_Read_FloatParam(0x06, t);
}
/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_Density 逻辑。
 *
 * @param rho 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_Read_Density(float *rho) {
	return DSM_V2_Read_FloatParam(0x07, rho);
}
/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_DynamicViscosity 逻辑。
 *
 * @param mu 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_Read_DynamicViscosity(float *mu) {
	return DSM_V2_Read_FloatParam(0x08, mu);
}
/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_KinematicViscosity 逻辑。
 *
 * @param nu 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_Read_KinematicViscosity(float *nu) {
	return DSM_V2_Read_FloatParam(0x09, nu);
}
/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_MeanSquare45 逻辑。
 *
 * @param msq45 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_Read_MeanSquare45(float *msq45) {
	return DSM_V2_Read_FloatParam(0x11, msq45);
} /* 17 */
/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_MeanSquare22p5 逻辑。
 *
 * @param msq22p5 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_Read_MeanSquare22p5(float *msq22p5) {
	return DSM_V2_Read_FloatParam(0x12, msq22p5);
} /* 18 */
/* R04 液位频率（整型） */
int DSM_V2_Read_LevelFrequency(uint32_t *freq_hz) {
	if (!freq_hz)
		return SYSTEM_CALL_CONDITION_ERROR;
	int32_t v = 0;
	int ret = DSM_V2_Read_IntParam(0x04, &v);   /* 参数码 0x04 = R04 */
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
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
/* R16 液位频率（整型） */
int DSM_V2_Read_DensityFrequency(float *freq_hz,float *freq_45,float *freq_225) {
	if (!freq_hz || !freq_45 || !freq_225)
		return SYSTEM_CALL_CONDITION_ERROR;
	float v = 0;
	int ret = DSM_V2_Read_FloatParam(0x11, &v);   /* 参数码 0x11 = 45度扫频平方均值 */
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
	if (ret == NO_ERROR) {
		*freq_45 = v;
		if (v > 0.0f) {
			*freq_hz = (float)sqrt(1000000000000.0/(double) v);
		} else {
			*freq_hz = 0.0f;
			printf("LTD频率周期平方均值为0，等待稳定/未测到\r\n");
		}
	}
	ret = DSM_V2_Read_FloatParam(0x12, &v);   /* 参数码 0x12 = 22.5度扫频平方均值 */
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
	if (ret == NO_ERROR) {
		*freq_225 =  v;
		if (v <= 0.0f) {
			printf("LTD 22.5度扫频周期平方均值为0，等待稳定/未测到\r\n");
		}
	}
	return ret;
}
/**
 * @brief 读取LTD 传感器通信中的 DSM_V2_Read_SensorID 逻辑。
 *
 * @param sensor_id 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_V2_Read_SensorID(uint32_t *sensor_id) {
	if (!sensor_id)
		return SYSTEM_CALL_CONDITION_ERROR;
	int32_t v = 0;
	int ret = DSM_V2_Read_IntParam(0x16, &v); /* 22 */
	/* 先处理异常边界，避免LTD 传感器通信状态机带故障继续运行。 */
	if (ret == NO_ERROR)
		*sensor_id = (uint32_t) v;
	return ret;
}

int DSM_V2_Probe_SensorID(uint32_t *sensor_id) {
	if (!sensor_id)
		return SYSTEM_CALL_CONDITION_ERROR;
	int32_t v = 0;
	int ret = DSM_V2_Read_IntParamInternal(0x16, &v, 0U); /* 22 */
	/* 识别阶段的协议探测失败属于候选未命中，不打印错误重试。 */
	if (ret == NO_ERROR)
		*sensor_id = (uint32_t) v;
	return ret;
}

