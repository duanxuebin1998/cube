#include "sensor_safe_transport_uart6.h"

#include <string.h>

#include "sensor_safe_frame.h"
#include "usart.h"

#define SENSOR_SAFE_UART6_DEFAULT_TX_TIMEOUT_MS  200U
#define SENSOR_SAFE_UART6_DEFAULT_DRAIN_IDLE_MS  5U
#define SENSOR_SAFE_UART6_DEFAULT_DRAIN_TOTAL_MS 30U

static SensorSafeTransportUart6Config s_config;
static SensorSafeTransportDiagnostics s_diagnostics;
static uint8_t s_rx_buffer[SENSOR_SAFE_UART6_RX_BUFFER_SIZE];
static uint8_t s_busy;

/* 传输诊断计数饱和保持，避免长稳运行后回绕造成故障次数倒退。 */
static void SensorSafeTransport_IncrementCounter(uint32_t *counter)
{
    if ((counter != NULL) && (*counter != UINT32_MAX)) {
        (*counter)++;
    }
}

/* 把 HAL 位图拆成独立计数，长稳记录可保留各硬件错误分子。 */
static void SensorSafeTransport_RecordHalError(void)
{
    uint32_t error = huart6.ErrorCode;
    uint8_t break_detected =
        (uint8_t)((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_LBD) != RESET) ? 1U : 0U);

    if ((error == HAL_UART_ERROR_NONE) && (break_detected == 0U)) {
        return;
    }
    s_diagnostics.last_hal_error = error;
    SensorSafeTransport_IncrementCounter(&s_diagnostics.hardware_error_count);
    if ((error & HAL_UART_ERROR_PE) != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.parity_error_count);
    }
    if ((error & HAL_UART_ERROR_NE) != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.noise_error_count);
    }
    if ((error & HAL_UART_ERROR_FE) != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.framing_error_count);
    }
    if ((error & HAL_UART_ERROR_ORE) != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.overrun_error_count);
    }
    if ((error & HAL_UART_ERROR_DMA) != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.dma_error_count);
    }
    if (break_detected != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.break_detect_count);
        __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_LBD);
    }
}

/* 查询上层命令切换请求；未配置回调时保持当前传输继续执行。 */
static uint8_t SensorSafeTransport_ShouldAbort(void)
{
    if (s_config.should_abort == NULL) {
        return 0U;
    }
    return s_config.should_abort();
}

/* 恢复外部收发器为接收方向；具体 GPIO 控制由可选平台回调承担。 */
static void SensorSafeTransport_SetReceiveMode(void)
{
    if (s_config.set_receive_mode != NULL) {
        s_config.set_receive_mode();
    }
}

/*
 * 函数用途：停止 UART6 DMA、记录硬件错误并恢复可再次接收的外设状态。
 * 调用场景：正常收帧结束以及超时、溢出、硬件错误、主动中止的统一出口。
 * 关键约束：必须清除 ORE 和 HAL 错误锁存，避免错误污染下一协议探测。
 */
static void SensorSafeTransport_StopDma(void)
{
    SensorSafeTransport_RecordHalError();
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    SensorSafeTransport_SetReceiveMode();
}

/* 根据 DMA 剩余计数计算已接收字节数，异常计数固定返回 0。 */
static uint16_t SensorSafeTransport_GetReceivedLength(void)
{
    uint16_t remaining;

    if (huart6.hdmarx == NULL) {
        return 0U;
    }
    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart6.hdmarx);
    if (remaining > SENSOR_SAFE_UART6_RX_BUFFER_SIZE) {
        return 0U;
    }
    return (uint16_t)(SENSOR_SAFE_UART6_RX_BUFFER_SIZE - remaining);
}

/*
 * 函数用途：清理旧协议、AT 模式或上一次失败事务遗留的 UART6 字节。
 * 调用场景：请求响应事务启动前；周期接收入口不调用，避免丢弃合法快报。
 * 关键约束：连续空闲达到 idle_ms 即结束，并受 total_ms 总上限约束。
 */
static SensorSafeTransportResult SensorSafeTransport_Drain(uint32_t idle_ms, uint32_t total_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t last_byte = start;
    uint8_t value;

    SensorSafeTransport_StopDma();
    while ((HAL_GetTick() - start) < total_ms) {
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, &value, 1U, 1U);

        if (SensorSafeTransport_ShouldAbort() != 0U) {
            return SENSOR_SAFE_TRANSPORT_ABORTED;
        }
        if (status == HAL_OK) {
            last_byte = HAL_GetTick();
        } else if ((status == HAL_TIMEOUT) && ((HAL_GetTick() - last_byte) >= idle_ms)) {
            return SENSOR_SAFE_TRANSPORT_OK;
        } else if ((status != HAL_TIMEOUT) && (huart6.ErrorCode != HAL_UART_ERROR_NONE)) {
            SensorSafeTransport_StopDma();
            return SENSOR_SAFE_TRANSPORT_HARDWARE_ERROR;
        }
    }
    return SENSOR_SAFE_TRANSPORT_OK;
}

/* 启动覆盖整个本地缓冲区的 UART6 DMA 接收，失败时统一清理 DMA 状态。 */
static SensorSafeTransportResult SensorSafeTransport_StartReceive(void)
{
    (void)memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    SensorSafeTransport_StopDma();
    if (HAL_UART_Receive_DMA(&huart6,
                             s_rx_buffer,
                             (uint16_t)SENSOR_SAFE_UART6_RX_BUFFER_SIZE) != HAL_OK) {
        SensorSafeTransport_StopDma();
        return SENSOR_SAFE_TRANSPORT_RX_FAILED;
    }
    return SENSOR_SAFE_TRANSPORT_OK;
}

/*
 * 函数用途：等待 DMA 发送完成并在可打断、超时和硬件错误之间分类返回。
 * 调用场景：请求响应事务发送控制帧后调用。
 * 关键约束：任务上下文按 1 ms 轮询，完成后立即切回接收方向，不可在 ISR 调用。
 */
static SensorSafeTransportResult SensorSafeTransport_WaitTransmit(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (SensorSafeTransport_ShouldAbort() != 0U) {
            return SENSOR_SAFE_TRANSPORT_ABORTED;
        }
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            return SENSOR_SAFE_TRANSPORT_HARDWARE_ERROR;
        }
        if (huart6.gState == HAL_UART_STATE_READY) {
            SensorSafeTransport_SetReceiveMode();
            return SENSOR_SAFE_TRANSPORT_OK;
        }
        HAL_Delay(1U);
    }
    return SENSOR_SAFE_TRANSPORT_TX_FAILED;
}

/* 只用 CRC 完整的后续候选打破伪长度前缀阻塞，语义仍由上层会话校验。 */
/*
 * 函数用途：在当前 DMA 数据中查找 CRC 已通过的后续完整帧候选。
 * 调用场景：前导 SOF 声明的伪长度阻塞当前扫描时用于重新同步。
 * 关键约束：候选必须完成 frame 层 CRC 校验，不能仅凭 SOF 和长度抢占当前帧。
 */
static uint8_t SensorSafeTransport_FindCompleteCandidate(uint16_t received,
                                                         uint16_t start_offset,
                                                         uint16_t *candidate_offset)
{
    uint16_t offset;

    if (candidate_offset == NULL) {
        return 0U;
    }
    for (offset = start_offset; (uint16_t)(received - offset) >= 2U; offset++) {
        uint16_t candidate_len = 0U;
        uint8_t is_fast = 0U;
        SensorSafeResult result;

        if ((s_rx_buffer[offset] != SENSOR_SAFE_SOF0) ||
            (s_rx_buffer[offset + 1U] != SENSOR_SAFE_SOF1)) {
            continue;
        }
        result = SensorSafeFrame_PeekLength(&s_rx_buffer[offset],
                                            (size_t)(received - offset),
                                            &candidate_len,
                                            &is_fast);
        if ((result != SENSOR_SAFE_OK) ||
            ((uint16_t)(received - offset) < candidate_len)) {
            continue;
        }
        if (is_fast != 0U) {
            SensorSafeFastReport report;

            result = SensorSafeFrame_DecodeFastReport(&s_rx_buffer[offset],
                                                      candidate_len,
                                                      &report);
        } else {
            SensorSafeControlFrame frame;

            result = SensorSafeFrame_DecodeControl(&s_rx_buffer[offset],
                                                   candidate_len,
                                                   &frame);
        }
        if (result == SENSOR_SAFE_OK) {
            *candidate_offset = offset;
            return 1U;
        }
    }
    return 0U;
}

/*
 * 函数用途：在 DMA 已接收字节中同步 SOF 并提取一帧。
 * 调用场景：请求响应和周期上报共用等待路径。
 * 关键约束：长度字段只决定候选帧边界，CRC 和语义由 frame/session 层继续校验。
 */
static SensorSafeTransportResult SensorSafeTransport_WaitFrame(uint8_t *frame,
                                                               size_t frame_capacity,
                                                               uint16_t *frame_len,
                                                               uint32_t *rx_timestamp_ms,
                                                               uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint16_t scan_offset = 0U;

    s_diagnostics.last_received_bytes = 0U;

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint16_t received = SensorSafeTransport_GetReceivedLength();

        s_diagnostics.last_received_bytes = received;
        if (SensorSafeTransport_ShouldAbort() != 0U) {
            SensorSafeTransport_StopDma();
            return SENSOR_SAFE_TRANSPORT_ABORTED;
        }
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            SensorSafeTransport_StopDma();
            return SENSOR_SAFE_TRANSPORT_HARDWARE_ERROR;
        }

        while ((uint16_t)(received - scan_offset) >= 2U) {
            uint16_t candidate_len = 0U;
            uint8_t is_fast = 0U;
            SensorSafeResult peek_result;

            if ((s_rx_buffer[scan_offset] != SENSOR_SAFE_SOF0) ||
                (s_rx_buffer[scan_offset + 1U] != SENSOR_SAFE_SOF1)) {
                scan_offset++;
                continue;
            }
            peek_result = SensorSafeFrame_PeekLength(&s_rx_buffer[scan_offset],
                                                     (size_t)(received - scan_offset),
                                                     &candidate_len,
                                                     &is_fast);
            if (peek_result == SENSOR_SAFE_BUFFER_TOO_SMALL) {
                break;
            }
            if (peek_result != SENSOR_SAFE_OK) {
                SensorSafeTransport_IncrementCounter(&s_diagnostics.malformed_prefix_count);
                scan_offset++;
                continue;
            }
            if ((uint16_t)(received - scan_offset) < candidate_len) {
                uint16_t later_offset = 0U;

                if (SensorSafeTransport_FindCompleteCandidate(received,
                                                              (uint16_t)(scan_offset + 1U),
                                                              &later_offset) != 0U) {
                    SensorSafeTransport_IncrementCounter(&s_diagnostics.malformed_prefix_count);
                    SensorSafeTransport_IncrementCounter(&s_diagnostics.resync_count);
                    scan_offset = later_offset;
                    continue;
                }
                break;
            }
            if (frame_capacity < (size_t)candidate_len) {
                SensorSafeTransport_IncrementCounter(&s_diagnostics.overflow_count);
                SensorSafeTransport_StopDma();
                return SENSOR_SAFE_TRANSPORT_OVERFLOW;
            }

            (void)is_fast;
            (void)memcpy(frame, &s_rx_buffer[scan_offset], candidate_len);
            *frame_len = candidate_len;
            *rx_timestamp_ms = HAL_GetTick();
            SensorSafeTransport_StopDma();
            return SENSOR_SAFE_TRANSPORT_OK;
        }

        if (received >= SENSOR_SAFE_UART6_RX_BUFFER_SIZE) {
            SensorSafeTransport_IncrementCounter(&s_diagnostics.overflow_count);
            SensorSafeTransport_StopDma();
            return SENSOR_SAFE_TRANSPORT_OVERFLOW;
        }
        HAL_Delay(1U);
    }

    SensorSafeTransport_IncrementCounter(&s_diagnostics.timeout_count);
    if (s_diagnostics.last_received_bytes != 0U) {
        SensorSafeTransport_IncrementCounter(&s_diagnostics.short_frame_count);
    }
    SensorSafeTransport_StopDma();
    return SENSOR_SAFE_TRANSPORT_TIMEOUT;
}

/*
 * 函数用途：初始化 UART6 安全传输配置、诊断计数和方向状态。
 * 调用场景：设备适配层首次初始化安全服务时调用。
 * 关键约束：零超时配置替换为受控默认值；本函数不启动 DMA 或发送数据。
 */
void SensorSafeTransportUart6_Init(const SensorSafeTransportUart6Config *config)
{
    (void)memset(&s_config, 0, sizeof(s_config));
    (void)memset(&s_diagnostics, 0, sizeof(s_diagnostics));
    s_busy = 0U;
    if (config != NULL) {
        s_config = *config;
    }
    if (s_config.transmit_timeout_ms == 0U) {
        s_config.transmit_timeout_ms = SENSOR_SAFE_UART6_DEFAULT_TX_TIMEOUT_MS;
    }
    if (s_config.drain_idle_ms == 0U) {
        s_config.drain_idle_ms = SENSOR_SAFE_UART6_DEFAULT_DRAIN_IDLE_MS;
    }
    if (s_config.drain_total_ms == 0U) {
        s_config.drain_total_ms = SENSOR_SAFE_UART6_DEFAULT_DRAIN_TOTAL_MS;
    }
    SensorSafeTransport_SetReceiveMode();
}

/*
 * 函数用途：串行完成清残留、先开接收、DMA 发送和等待单帧响应。
 * 调用场景：client 的全部控制请求响应事务使用。
 * 关键约束：s_busy 保证不可重入；所有失败出口停止 DMA 并释放占用标志。
 */
SensorSafeTransportResult SensorSafeTransportUart6_Exchange(const uint8_t *request,
                                                            uint16_t request_len,
                                                            uint8_t *response,
                                                            size_t response_capacity,
                                                            uint16_t *response_len,
                                                            uint32_t *rx_timestamp_ms,
                                                            uint32_t response_timeout_ms)
{
    SensorSafeTransportResult result;

    if ((request == NULL) || (request_len == 0U) ||
        (response == NULL) || (response_len == NULL) ||
        (rx_timestamp_ms == NULL) || (response_timeout_ms == 0U)) {
        return SENSOR_SAFE_TRANSPORT_INVALID_ARGUMENT;
    }
    if (s_busy != 0U) {
        return SENSOR_SAFE_TRANSPORT_BUSY;
    }

    s_busy = 1U;
    *response_len = 0U;
    SensorSafeTransport_IncrementCounter(&s_diagnostics.exchange_count);
    /* 先清旧协议残留，再先开 RX 后发 TX，降低半双工切向期间丢首字节风险。 */
    result = SensorSafeTransport_Drain(s_config.drain_idle_ms, s_config.drain_total_ms);
    if (result == SENSOR_SAFE_TRANSPORT_OK) {
        result = SensorSafeTransport_StartReceive();
    }
    if (result == SENSOR_SAFE_TRANSPORT_OK) {
        if (s_config.set_transmit_mode != NULL) {
            s_config.set_transmit_mode();
        }
        if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)request, request_len) != HAL_OK) {
            result = SENSOR_SAFE_TRANSPORT_TX_FAILED;
        }
    }
    if (result == SENSOR_SAFE_TRANSPORT_OK) {
        result = SensorSafeTransport_WaitTransmit(s_config.transmit_timeout_ms);
    }
    if (result == SENSOR_SAFE_TRANSPORT_OK) {
        result = SensorSafeTransport_WaitFrame(response,
                                               response_capacity,
                                               response_len,
                                               rx_timestamp_ms,
                                               response_timeout_ms);
    } else {
        SensorSafeTransport_StopDma();
    }
    s_busy = 0U;
    return result;
}

/*
 * 函数用途：在周期模式下只启动接收并等待一帧，不发送控制请求。
 * 调用场景：client 轮询传感器主动上报的固定长度快报。
 * 关键约束：不执行 drain，避免把已到达的合法快报当作残留字节丢弃。
 */
SensorSafeTransportResult SensorSafeTransportUart6_ReceiveFrame(uint8_t *frame,
                                                                size_t frame_capacity,
                                                                uint16_t *frame_len,
                                                                uint32_t *rx_timestamp_ms,
                                                                uint32_t timeout_ms)
{
    SensorSafeTransportResult result;

    if ((frame == NULL) || (frame_len == NULL) ||
        (rx_timestamp_ms == NULL) || (timeout_ms == 0U)) {
        return SENSOR_SAFE_TRANSPORT_INVALID_ARGUMENT;
    }
    if (s_busy != 0U) {
        return SENSOR_SAFE_TRANSPORT_BUSY;
    }

    s_busy = 1U;
    *frame_len = 0U;
    SensorSafeTransport_IncrementCounter(&s_diagnostics.receive_count);
    SensorSafeTransport_SetReceiveMode();
    result = SensorSafeTransport_StartReceive();
    if (result == SENSOR_SAFE_TRANSPORT_OK) {
        result = SensorSafeTransport_WaitFrame(frame,
                                               frame_capacity,
                                               frame_len,
                                               rx_timestamp_ms,
                                               timeout_ms);
    }
    s_busy = 0U;
    return result;
}

/* 主动停止当前 UART6 DMA 并释放不可重入标志，供命令切换和协议回退调用。 */
void SensorSafeTransportUart6_Abort(void)
{
    SensorSafeTransport_StopDma();
    s_busy = 0U;
}

/* 返回安全传输是否正在占用 UART6；只用于状态观察，不替代互斥保护。 */
uint8_t SensorSafeTransportUart6_IsBusy(void)
{
    return s_busy;
}

/* 复制累计传输诊断快照；空指针时不写输出。 */
void SensorSafeTransportUart6_GetDiagnostics(SensorSafeTransportDiagnostics *diagnostics)
{
    if (diagnostics != NULL) {
        *diagnostics = s_diagnostics;
    }
}
