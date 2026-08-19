/*
 * 模块职责：维护DM4-V4主动上发的Receive-to-IDLE DMA、环形流缓冲、帧重同步和快照发布。
 * 上下文边界：UART ISR只复制字节并挂起PendSV；有界解析在PendSV执行；打印和故障锁存
 * 留在线程态Service中，禁止在中断路径执行阻塞操作。
 * 所有权约束：主动DMA启动前必须持有UART6主动所有权，DMA完全停止后才能释放。
 */
#include "multiparam_v4_internal.h"

#include "error_log.h"
#include "main.h"
#include "my_crc.h"
#include "sensor_transport_config.h"
#include "sensor_uart6_owner.h"
#include "system_parameter.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
/*
 * 函数用途：把字节写入ISR与线程态共享的流缓冲。
 * 调用场景：Receive-to-IDLE回调和主机测试注入。
 * 关键约束：缓冲满时丢弃最旧字节，保留最新线上数据以便重新找帧头。
 */
static void MULTIPARAM_V4_StreamPushByte(uint8_t value)
{
    uint16_t next = (uint16_t)((s_stream_head + 1U) % MULTIPARAM_V4_STREAM_BUFFER_SIZE);

    if (next == s_stream_tail) {
        s_stream_tail = (uint16_t)((s_stream_tail + 1U) % MULTIPARAM_V4_STREAM_BUFFER_SIZE);
        s_diagnostics.stream_overflows++;
    }
    s_stream_buffer[s_stream_head] = value;
    s_stream_head = next;
}

/*
 * 函数用途：启动一次Receive-to-IDLE DMA接收。
 * 调用场景：主动接收启动、完整事件后的缓冲切换和线程态错误恢复。
 * 关键约束：不打印、不解析；调用方负责确保UART6所有权属于V4模块。
 */
uint32_t MULTIPARAM_V4_StartReceiveDmaInternal(void)
{
    HAL_StatusTypeDef status;

    memset(s_active_dma_buffer, 0, sizeof(s_active_dma_buffer));
    s_active_dma_consumed = 0U;
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                         s_active_dma_buffer,
                                         (uint16_t)sizeof(s_active_dma_buffer));
    if (status != HAL_OK) {
        s_active_receive_running = 0U;
        return COMM_UART_TRANSFER_ERROR;
    }
    if (huart6.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
    s_active_receive_running = 1U;
    return NO_ERROR;
}

/*
 * 函数用途：原子取得一次UART6接收恢复执行权。
 * 调用场景：PendSV快速恢复和线程态完整DMA恢复共用。
 * 关键约束：同一时刻只允许一个上下文操作UART6接收状态。
 */
static uint8_t MULTIPARAM_V4_TryClaimReceiveRestart(uint8_t require_ready)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t claimed = 0U;

    __disable_irq();
    if ((s_active_receive_requested != 0U) &&
        (s_receive_restart_pending != 0U) &&
        (s_receive_restart_in_progress == 0U) &&
        ((require_ready == 0U) || (huart6.RxState == HAL_UART_STATE_READY))) {
        s_receive_restart_pending = 0U;
        s_receive_restart_in_progress = 1U;
        s_receive_restart_wait_started = 0U;
        s_receive_abort_requested = 0U;
        claimed = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return claimed;
}

/*
 * 函数用途：结束一次UART6接收恢复并在失败时重新排队。
 * 调用场景：PendSV快速恢复或线程态完整恢复完成后。
 * 关键约束：状态更新在短临界区完成；只重新排队，不在本函数内递归启动DMA。
 */
static void MULTIPARAM_V4_CompleteReceiveRestart(uint32_t result)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_receive_restart_in_progress = 0U;
    s_receive_restart_wait_started = 0U;
    s_receive_abort_requested = 0U;
    s_receive_abort_in_progress = 0U;
    if ((result != NO_ERROR) && (s_active_receive_requested != 0U)) {
        s_receive_restart_pending = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：从解析缓冲头部移除指定字节数。
 * 调用场景：消费完整帧或校验失败后逐字节重同步。
 * 关键约束：移除长度覆盖全部数据时直接清零；不修改流缓冲和诊断计数。
 */
static void MULTIPARAM_V4_DiscardParseBytes(uint16_t count)
{
    if (count >= s_parse_length) {
        s_parse_length = 0U;
        return;
    }
    memmove(s_parse_buffer, s_parse_buffer + count, (size_t)(s_parse_length - count));
    s_parse_length = (uint16_t)(s_parse_length - count);
}

/*
 * 函数用途：发布通过固定字段、地址、CRC和序号检查的主动快照。
 * 调用场景：PendSV有界流解析器完成一帧解析后。
 * 关键约束：重复帧和乱序帧只记诊断，不刷新新鲜度或业务代次。
 */
static uint8_t MULTIPARAM_V4_PublishSnapshot(
    multiparam_v4_snapshot_t *candidate,
    const uint8_t frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE])
{
    if (s_sequence_valid != 0U) {
        uint16_t delta = (uint16_t)(candidate->sequence - s_last_sequence);

        if (delta == 0U) {
            s_diagnostics.duplicate_frames++;
            MULTIPARAM_V4_RecordQualityAnomaly(
                SENSOR_REPLAY_DETECTED, 1U, frame, MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
            return 0U;
        }
        if (delta >= 0x8000U) {
            if ((s_latest_snapshot.valid != 0U) &&
                ((HAL_GetTick() - s_latest_snapshot.received_tick) <=
                 MULTIPARAM_V4_ACTIVE_TIMEOUT_MS)) {
                s_diagnostics.out_of_order_frames++;
                MULTIPARAM_V4_RecordQualityAnomaly(
                    SENSOR_SEQUENCE_ERROR, 1U, frame, MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
                return 0U;
            }
            s_diagnostics.sequence_resets++;
            MULTIPARAM_V4_RecordQualityAnomaly(
                SENSOR_SEQUENCE_ERROR, 1U, frame, MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
        }
        else if (delta > 1U) {
            uint32_t lost = (uint32_t)(delta - 1U);

            s_diagnostics.lost_frames =
                MULTIPARAM_V4_SaturatingAdd(s_diagnostics.lost_frames, lost);
            MULTIPARAM_V4_RecordQualityAnomaly(
                SENSOR_SEQUENCE_ERROR, lost, frame, MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
        }
    }

    if (s_expected_address == MULTIPARAM_V4_ANY_ADDRESS) {
        /* 首个完整主动帧锁定真实从机地址，后续交互不得发送ANY地址。 */
        s_expected_address = candidate->address;
    }
    s_sequence_valid = 1U;
    s_last_sequence = candidate->sequence;
    candidate->generation = s_latest_snapshot.generation + 1U;
    candidate->received_tick = HAL_GetTick();
    candidate->valid = 1U;
    memcpy(&s_latest_snapshot, candidate, sizeof(s_latest_snapshot));
    s_diagnostics.valid_frames =
        MULTIPARAM_V4_SaturatingAdd(s_diagnostics.valid_frames, 1U);
    s_timeout_latched = 0U;
    MULTIPARAM_V4_RecordQualitySuccess();
    return 1U;
}

/*
 * 函数用途：记录完整主动帧接收结束时刻。
 * 调用场景：主动流解析或停流事务识别出完整主动帧后。
 * 关键约束：时间戳取在完整帧解析之后，15ms从不早于总线最后一个主动帧字节开始计算。
 */
void MULTIPARAM_V4_MarkActiveFrameEnd(void)
{
    s_last_active_frame_end_tick = HAL_GetTick();
    s_last_active_frame_end_valid = 1U;
}

/*
 * 函数用途：阻塞等待完整主动帧后的半双工发送保护期结束。
 * 调用场景：下发切换交互通信指令或失败恢复读回之前。
 * 关键约束：仅在任务上下文调用，等待期间保持中断开启。
 */
void MULTIPARAM_V4_WaitActiveFrameGuard(void)
{
    while ((s_last_active_frame_end_valid != 0U) &&
           ((HAL_GetTick() - s_last_active_frame_end_tick) <
            MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS)) {
        HAL_Delay(1U);
    }
}

/*
 * 函数用途：在解析缓冲中有界查找并消费V4主动帧。
 * 调用场景：PendSV每轮最多检查指定数量的候选帧。
 * 关键约束：找不到帧头时只保留末尾单个0x55，供下一片段续接。
 */
static uint8_t MULTIPARAM_V4_ProcessParseBuffer(uint8_t frame_budget)
{
    uint8_t candidates_checked = 0U;
    uint8_t published = 0U;

    while (candidates_checked < frame_budget) {
        multiparam_v4_snapshot_t candidate;
        uint8_t canonical_frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE];
        uint16_t header_offset = 0U;
        uint16_t consumed_length = 0U;
        uint8_t wireless_padding_used = 0U;
        uint32_t parse_result = SENSOR_RESP_FORMAT_ERROR;
        multiparam_v4_active_candidate_state_t candidate_state;

        while ((header_offset + 1U) < s_parse_length) {
            if ((s_parse_buffer[header_offset] == MULTIPARAM_V4_ACTIVE_HEADER0) &&
                (s_parse_buffer[header_offset + 1U] == MULTIPARAM_V4_ACTIVE_HEADER1)) {
                break;
            }
            header_offset++;
        }
        if ((header_offset + 1U) >= s_parse_length) {
            uint16_t keep = (uint16_t)(((s_parse_length > 0U) &&
                                       (s_parse_buffer[s_parse_length - 1U] ==
                                        MULTIPARAM_V4_ACTIVE_HEADER0)) ? 1U : 0U);
            if (s_parse_length > keep) {
                if (keep != 0U) {
                    s_parse_buffer[0] = MULTIPARAM_V4_ACTIVE_HEADER0;
                }
                s_parse_length = keep;
                s_diagnostics.resync_events++;
            }
            return published;
        }
        if (header_offset != 0U) {
            MULTIPARAM_V4_DiscardParseBytes(header_offset);
            s_diagnostics.resync_events++;
        }

        candidate_state = MULTIPARAM_V4_DecodeActiveCandidate(
            s_parse_buffer, s_parse_length, &candidate, canonical_frame,
            &consumed_length, &wireless_padding_used, &parse_result);
        if (candidate_state == MULTIPARAM_V4_ACTIVE_CANDIDATE_NEED_MORE) {
            return published;
        }

        /* 完整主动帧即使重复或校验失败也占用了半双工总线，必须刷新发送保护期。 */
        MULTIPARAM_V4_MarkActiveFrameEnd();
        candidates_checked++;
        s_diagnostics.frames_seen++;
        if (candidate_state == MULTIPARAM_V4_ACTIVE_CANDIDATE_VALID) {
            uint8_t accepted;

            if (wireless_padding_used != 0U) {
                s_diagnostics.wireless_zero_padding_frames++;
            }
            accepted = MULTIPARAM_V4_PublishSnapshot(&candidate, canonical_frame);
            if (accepted != 0U) {
                memcpy(s_latest_active_frame,
                       canonical_frame,
                       MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
                s_latest_active_frame_valid = 1U;
            }
            MULTIPARAM_V4_DiscardParseBytes(consumed_length);
            published = (uint8_t)(published | accepted);
            continue;
        }

        if (parse_result == SENSOR_BCC_ERROR) {
            s_diagnostics.crc_errors++;
        } else if (parse_result == SENSOR_ADDRESS_MISMATCH) {
            s_diagnostics.address_errors++;
        } else if (parse_result == SENSOR_PROTOCOL_VERSION_INCOMPATIBLE) {
            s_diagnostics.value_errors++;
        } else {
            s_diagnostics.fixed_field_errors++;
        }
        {
            uint16_t abnormal_length = MULTIPARAM_V4_ACTIVE_FRAME_SIZE;

            if ((s_parse_length >= MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE) &&
                (MULTIPARAM_V4_IsZeroRange(
                     s_parse_buffer + MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET,
                     MULTIPARAM_V4_WIRELESS_ZERO_PADDING_SIZE) != 0U)) {
                abnormal_length = MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE;
            }
            MULTIPARAM_V4_RecordQualityAnomaly(
                parse_result, 1U, s_parse_buffer, abnormal_length);
        }
        MULTIPARAM_V4_DiscardParseBytes(1U);
        s_diagnostics.resync_events++;
    }
    return published;
}

/*
 * 函数用途：取得UART6主动所有权并建立V4 Receive-to-IDLE常驻接收。
 * 调用场景：识别到主动模式、写R65恢复主动模式或协议栈重新启动时。
 * 关键约束：启动前清空旧流状态和快照有效位；DMA启动失败必须释放本次取得的所有权。
 */
uint32_t MULTIPARAM_V4_StartActiveReceive(void)
{
    uint32_t result;
    uint32_t primask;
    uint8_t acquired = 0U;

    /* 先取得UART6所有权再触碰DMA，避免与交互、Safe或CH9141 AT事务交叉。 */
    if (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) == 0U) {
        if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) == 0U) {
            return SENSOR_MODE_NOT_READY;
        }
        acquired = 1U;
    }

    (void)HAL_UART_DMAStop(&huart6);
    primask = __get_PRIMASK();
    __disable_irq();
    s_stream_head = 0U;
    s_stream_tail = 0U;
    s_parse_length = 0U;
    s_deferred_parse_requested = 0U;
    s_sequence_valid = 0U;
    s_timeout_latched = 0U;
    s_quality_alarm_latched = 0U;
    s_diagnostics.consecutive_abnormal_frames = 0U;
    s_active_receive_start_tick = 0U;
    s_latest_snapshot.valid = 0U;
    s_last_active_frame_end_tick = 0U;
    s_last_active_frame_end_valid = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    s_active_receive_requested = 1U;
    s_receive_abort_in_progress = 0U;
    result = MULTIPARAM_V4_StartReceiveDmaInternal();
    if (result == NO_ERROR) {
        s_receive_restart_pending = 0U;
        s_receive_restart_wait_started = 0U;
        s_receive_abort_requested = 0U;
        s_active_receive_start_tick = HAL_GetTick();
        s_communication_mode = MULTIPARAM_V4_COMMUNICATION_ACTIVE;
    } else {
        s_diagnostics.receive_restart_errors++;
        s_active_receive_requested = 0U;
        if ((acquired != 0U) ||
            (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) != 0U)) {
            (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_DM4_V4_ACTIVE);
        }
    }
    return result;
}

/*
 * 函数用途：停止V4主动DMA、清理解析队列并释放UART6主动所有权。
 * 调用场景：进入交互窗口、协议栈反初始化或其它协议需要接管UART6时。
 * 关键约束：先禁止恢复请求并停止DMA，最后才释放所有权；累计诊断和最后异常包保留。
 */
void MULTIPARAM_V4_StopActiveReceive(void)
{
    uint32_t primask;

    s_active_receive_requested = 0U;
    s_active_receive_running = 0U;
    s_receive_restart_pending = 0U;
    s_receive_restart_in_progress = 0U;
    s_receive_restart_wait_started = 0U;
    s_receive_restart_wait_tick = 0U;
    s_receive_abort_requested = 0U;
    s_receive_abort_in_progress = 0U;
    s_active_dma_consumed = 0U;
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;

    primask = __get_PRIMASK();
    __disable_irq();
    s_stream_head = 0U;
    s_stream_tail = 0U;
    s_parse_length = 0U;
    s_deferred_parse_requested = 0U;
    s_active_receive_start_tick = 0U;
    s_diagnostics.consecutive_abnormal_frames = 0U;
    s_quality_alarm_latched = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    /* DMA和共享解析状态均已停止/清空后，最后释放UART6给下一协议使用。 */
    if (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) != 0U) {
        (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_DM4_V4_ACTIVE);
    }
}

/*
 * 函数用途：挂起多参数V4的PendSV延后解析。
 * 调用场景：UART6接收/错误中断和主机测试字节注入完成后。
 * 关键约束：只置位请求，不解析、不打印、不等待。
 */
void MULTIPARAM_V4_RequestDeferredFromISR(void)
{
    s_deferred_parse_requested = 1U;
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

/*
 * 函数用途：把UART6 Receive-to-IDLE事件中新到字节搬入V4流缓冲并续接DMA。
 * 调用场景：HAL UART6接收事件回调确认当前所有者为V4主动流后。
 * 关键约束：中断中只复制、计数、置位和快速重启DMA；不解析浮点、不打印、不等待。
 */
void MULTIPARAM_V4_OnUartRxEventISR(uint16_t received_length)
{
    uint16_t start;
    uint16_t index;
    HAL_UART_RxEventTypeTypeDef event_type;

    if (s_active_receive_requested == 0U) {
        return;
    }
    if (received_length > MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE) {
        received_length = MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE;
    }
    start = s_active_dma_consumed;
    if (start > received_length) {
        start = 0U;
    }
    for (index = start; index < received_length; index++) {
        MULTIPARAM_V4_StreamPushByte(s_active_dma_buffer[index]);
    }
    s_diagnostics.received_bytes += (uint32_t)(received_length - start);
    s_diagnostics.receive_events++;
    MULTIPARAM_V4_RequestDeferredFromISR();

    event_type = HAL_UARTEx_GetRxEventType(&huart6);
    if (event_type == HAL_UART_RXEVENT_HT) {
        s_active_dma_consumed = received_length;
        return;
    }
    if (received_length < MULTIPARAM_V4_ACTIVE_FRAME_SIZE) {
        s_diagnostics.short_receive_events++;
    }
    s_active_receive_running = 0U;
    if (MULTIPARAM_V4_StartReceiveDmaInternal() != NO_ERROR) {
        s_diagnostics.receive_restart_errors++;
        s_receive_restart_pending = 1U;
        s_receive_restart_wait_started = 0U;
    } else {
        s_receive_restart_pending = 0U;
        s_receive_restart_wait_started = 0U;
        s_receive_abort_requested = 0U;
    }
}

/*
 * 函数用途：记录UART6主动接收硬件错误并安排延后恢复。
 * 调用场景：HAL UART6错误回调确认当前所有者为V4主动流后。
 * 关键约束：中断中不停止DMA、不打印、不阻塞；实际恢复由PendSV或线程态Service完成。
 */
void MULTIPARAM_V4_OnUartErrorISR(uint32_t uart_error)
{
    if (s_active_receive_requested == 0U) {
        return;
    }
    if (uart_error != HAL_UART_ERROR_NONE) {
        s_diagnostics.uart_errors++;
    }
    s_active_receive_running = 0U;
    s_receive_restart_pending = 1U;
    s_receive_restart_wait_started = 0U;
    s_receive_abort_requested = 0U;
    MULTIPARAM_V4_RequestDeferredFromISR();
}

/*
 * 函数用途：确认V4主动流是否已经达到通信超时，并生成一次质量错误。
 * 调用场景：PendSV延后处理和兼容的线程态Service入口。
 * 关键约束：调用方不得在高优先级ISR中执行；本函数只更新内存并挂起故障消费，不打印。
 */
static void MULTIPARAM_V4_CheckActiveTimeout(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((s_active_receive_requested != 0U) &&
        ((now - ((s_latest_snapshot.valid != 0U)
                     ? s_latest_snapshot.received_tick
                     : s_active_receive_start_tick)) >= MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) &&
        (s_timeout_latched == 0U)) {
        s_timeout_latched = 1U;
        s_diagnostics.timeout_events =
            MULTIPARAM_V4_SaturatingAdd(s_diagnostics.timeout_events, 1U);
        MULTIPARAM_V4_RecordQualityAnomaly(
            SENSOR_DEVICE_COMM_TIMEOUT,
            MULTIPARAM_V4_CONSECUTIVE_ERROR_LIMIT,
            NULL,
            0U);
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：用1ms系统节拍检查主动流超时和UART6接收恢复到期条件。
 * 调用场景：SysTick在HAL_IncTick之后调用，确保阻塞主循环不影响通信维护时序。
 * 关键约束：ISR只读状态、置恢复请求并挂起PendSV；不停止DMA、不解析、不打印。
 */
void MULTIPARAM_V4_TickFromISR(void)
{
    uint32_t now;
    uint32_t reference_tick;

    if (s_active_receive_requested == 0U) {
        return;
    }

    now = HAL_GetTick();
    reference_tick = (s_latest_snapshot.valid != 0U)
                         ? s_latest_snapshot.received_tick
                         : s_active_receive_start_tick;
    if ((s_timeout_latched == 0U) &&
        ((now - reference_tick) >= MULTIPARAM_V4_ACTIVE_TIMEOUT_MS)) {
        /* ISR只负责让PendSV重新核对，避免在系统节拍中复制异常包或发布故障。 */
        MULTIPARAM_V4_RequestDeferredFromISR();
    }

    if ((s_receive_restart_pending == 0U) ||
        (s_receive_restart_in_progress != 0U)) {
        return;
    }
    if (huart6.RxState == HAL_UART_STATE_READY) {
        MULTIPARAM_V4_RequestDeferredFromISR();
        return;
    }

    if (s_receive_restart_wait_started == 0U) {
        s_receive_restart_wait_tick = now;
        s_receive_restart_wait_started = 1U;
        return;
    }
    if ((now - s_receive_restart_wait_tick) >= MULTIPARAM_V4_RX_FORCE_ABORT_DELAY_MS) {
        s_receive_abort_requested = 1U;
        MULTIPARAM_V4_RequestDeferredFromISR();
    }
}

/*
 * 函数用途：接收异步终止完成后重新挂起V4 DMA恢复。
 * 调用场景：HAL UART6 AbortReceive完成回调。
 * 关键约束：中断中只更新状态并挂起PendSV，不直接重启或打印。
 */
void MULTIPARAM_V4_OnUartAbortReceiveCompleteISR(void)
{
    if (s_receive_abort_in_progress == 0U) {
        return;
    }
    s_receive_abort_in_progress = 0U;
    s_receive_restart_in_progress = 0U;
    s_receive_restart_wait_started = 0U;
    s_receive_abort_requested = 0U;
    s_active_receive_running = 0U;
    if (s_active_receive_requested != 0U) {
        s_receive_restart_pending = 1U;
        MULTIPARAM_V4_RequestDeferredFromISR();
    } else {
        s_receive_restart_pending = 0U;
    }
}

/*
 * 函数用途：查询V4主动DMA当前是否处于运行状态。
 * 调用场景：串口调试、协议切换和UART回调过滤。
 * 关键约束：只返回软件快照，不等同于通信方式、UART所有权或快照新鲜度。
 */
uint8_t MULTIPARAM_V4_IsActiveReceiverRunning(void)
{
    return s_active_receive_running;
}

/*
 * 函数用途：向V4主动流缓冲注入一段测试或已接收字节并请求延后解析。
 * 调用场景：主机协议测试和不经过HAL回调的受控字节输入。
 * 关键约束：短临界区内只复制字节；不直接解析、不打印，空输入直接忽略。
 */
void MULTIPARAM_V4_FeedBytes(const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint16_t index;

    if ((data == NULL) || (length == 0U)) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < length; index++) {
        MULTIPARAM_V4_StreamPushByte(data[index]);
    }
    s_diagnostics.received_bytes += length;
    if (primask == 0U) {
        __enable_irq();
    }
    MULTIPARAM_V4_RequestDeferredFromISR();
}

/*
 * 函数用途：在PendSV中有界恢复接收、搬运流字节、重同步并发布主动快照。
 * 调用场景：统一PendSV_Handler收到V4延后处理请求时。
 * 关键约束：每轮字节数和候选帧数有上限；不打印、不执行HAL阻塞停止，积压时重新挂起。
 */
uint8_t MULTIPARAM_V4_ProcessDeferredPendSV(void)
{
    uint16_t local_length = 0U;
    uint8_t published;
    uint8_t work_requested;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    work_requested = (uint8_t)(((s_deferred_parse_requested != 0U) ||
                                (s_receive_restart_pending != 0U) ||
                                (s_receive_abort_requested != 0U) ||
                                (s_stream_tail != s_stream_head) ||
                                (s_parse_length >= MULTIPARAM_V4_ACTIVE_FRAME_SIZE)) ? 1U : 0U);
    if (work_requested != 0U) {
        s_deferred_parse_requested = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    if (work_requested == 0U) {
        return 0U;
    }

    /* 超时由系统节拍触发、PendSV二次确认，不能依赖可能阻塞的主循环。 */
    MULTIPARAM_V4_CheckActiveTimeout(HAL_GetTick());

    /* UART错误回调已把HAL状态收尾时，PendSV只做一次无等待快速重启。 */
    if (MULTIPARAM_V4_TryClaimReceiveRestart(1U) != 0U) {
        uint32_t restart_result = MULTIPARAM_V4_StartReceiveDmaInternal();
        if (restart_result != NO_ERROR) {
            s_diagnostics.receive_restart_errors++;
        }
        MULTIPARAM_V4_CompleteReceiveRestart(restart_result);
    }

    /* HAL状态长时间未收尾时只发起异步终止，完成回调再重启DMA。 */
    if ((s_receive_abort_requested != 0U) &&
        (MULTIPARAM_V4_TryClaimReceiveRestart(0U) != 0U)) {
        s_receive_abort_in_progress = 1U;
        if (HAL_UART_AbortReceive_IT(&huart6) != HAL_OK) {
            s_receive_abort_in_progress = 0U;
            s_diagnostics.receive_restart_errors++;
            MULTIPARAM_V4_CompleteReceiveRestart(COMM_UART_TRANSFER_ERROR);
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    while ((s_stream_tail != s_stream_head) &&
           (local_length < MULTIPARAM_V4_DEFERRED_BYTE_BUDGET)) {
        s_deferred_bytes[local_length++] = s_stream_buffer[s_stream_tail];
        s_stream_tail = (uint16_t)((s_stream_tail + 1U) % MULTIPARAM_V4_STREAM_BUFFER_SIZE);
    }
    if (primask == 0U) {
        __enable_irq();
    }

    for (uint16_t index = 0U; index < local_length; index++) {
        if (s_parse_length >= MULTIPARAM_V4_PARSE_BUFFER_SIZE) {
            MULTIPARAM_V4_DiscardParseBytes(1U);
            s_diagnostics.resync_events++;
        }
        s_parse_buffer[s_parse_length++] = s_deferred_bytes[index];
    }
    published = MULTIPARAM_V4_ProcessParseBuffer(MULTIPARAM_V4_DEFERRED_FRAME_BUDGET);

    if ((s_stream_tail != s_stream_head) ||
        (s_parse_length >= MULTIPARAM_V4_ACTIVE_FRAME_SIZE)) {
        MULTIPARAM_V4_RequestDeferredFromISR();
    }
    return published;
}

/*
 * 函数用途：为既有同步等待流程保留一次主动流超时复核。
 * 调用场景：模式切换等待和主动快照消费循环中的兼容调用。
 * 关键约束：实时超时和接收恢复已由SysTick/PendSV负责，本入口不是时序依赖点。
 */
void MULTIPARAM_V4_Service(void)
{
    MULTIPARAM_V4_CheckActiveTimeout(HAL_GetTick());
}

/*
 * 函数用途：原子复制最近一次通过完整校验的V4主动快照。
 * 调用场景：测量适配、功能状态确认和串口调试读取主动参数。
 * 关键约束：只在短临界区复制；无有效快照返回通信超时，不返回半写入数据。
 */
uint32_t MULTIPARAM_V4_CopyLatestSnapshot(multiparam_v4_snapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_latest_snapshot.valid == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return SENSOR_DEVICE_COMM_TIMEOUT;
    }
    memcpy(snapshot, &s_latest_snapshot, sizeof(*snapshot));
    if (primask == 0U) {
        __enable_irq();
    }
    return NO_ERROR;
}

/*
 * 函数用途：判断最近主动快照年龄是否在调用方允许范围内。
 * 调用场景：模式确认、功能状态确认和业务读取门禁。
 * 关键约束：没有有效快照时返回否；本函数不触发接收或等待新帧。
 */
uint8_t MULTIPARAM_V4_IsSnapshotFresh(uint32_t maximum_age_ms)
{
    multiparam_v4_snapshot_t snapshot;

    if (MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) != NO_ERROR) {
        return 0U;
    }
    return (uint8_t)(((HAL_GetTick() - snapshot.received_tick) <= maximum_age_ms) ? 1U : 0U);
}

/*
 * 函数用途：返回最近主动快照距当前时刻的毫秒数。
 * 调用场景：串口V4状态和通信质量诊断。
 * 关键约束：没有有效快照时返回UINT32_MAX，调用方必须按无数据解释。
 */
uint32_t MULTIPARAM_V4_GetSnapshotAgeMs(void)
{
    multiparam_v4_snapshot_t snapshot;

    if (MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) != NO_ERROR) {
        return UINT32_MAX;
    }
    return HAL_GetTick() - snapshot.received_tick;
}

/*
 * 函数用途：在不清除快照和序号的前提下恢复主动接收。
 * 调用场景：参数65停流事务被一帧主动上报抢占后。
 * 关键约束：保留已收到帧并继续使用原地址，启动失败时交给延后恢复。
 */
uint32_t MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot(void)
{
    uint32_t result;
    uint8_t owner_changed = 0U;

    if (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE) != 0U) {
        if (SensorUart6Owner_Transition(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE,
                                        SENSOR_UART6_OWNER_DM4_V4_ACTIVE) == 0U) {
            return SENSOR_MODE_NOT_READY;
        }
        owner_changed = 1U;
    } else if (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) == 0U) {
        if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) == 0U) {
            return SENSOR_MODE_NOT_READY;
        }
        owner_changed = 1U;
    }

    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    s_active_receive_requested = 1U;
    s_receive_restart_pending = 0U;
    s_receive_restart_in_progress = 0U;
    s_receive_restart_wait_started = 0U;
    s_receive_abort_requested = 0U;
    s_receive_abort_in_progress = 0U;
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_ACTIVE;
    result = MULTIPARAM_V4_StartReceiveDmaInternal();
    if (result != NO_ERROR) {
        s_diagnostics.receive_restart_errors++;
        s_receive_restart_pending = 1U;
        MULTIPARAM_V4_RequestDeferredFromISR();
        if ((owner_changed != 0U) &&
            (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_ACTIVE) != 0U)) {
            (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_DM4_V4_ACTIVE);
        }
    }
    return result;
}
