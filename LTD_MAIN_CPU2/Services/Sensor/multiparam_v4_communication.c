/*
 * multiparam_v4_communication.c
 * 多参数传感器通信协议 V4.0 的主动帧接收、快照发布和8字节交互事务。
 */

#include "multiparam_v4_communication.h"

#include "main.h"
#include "my_crc.h"
#include "sensor.h"
#include "system_parameter.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#define MULTIPARAM_V4_ACTIVE_HEADER0                 0x55U
#define MULTIPARAM_V4_ACTIVE_HEADER1                 0xAAU
#define MULTIPARAM_V4_ACTIVE_FORMAT_VERSION          0x01U
#define MULTIPARAM_V4_ACTIVE_FRAME_TYPE              0xA1U
#define MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE         128U
#define MULTIPARAM_V4_STREAM_BUFFER_SIZE             256U
#define MULTIPARAM_V4_PARSE_BUFFER_SIZE              256U
#define MULTIPARAM_V4_DEFERRED_BYTE_BUDGET           128U
#define MULTIPARAM_V4_DEFERRED_FRAME_BUDGET          2U
#define MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS         DSM_CMD_TIMEOUT
#define MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS     300U
#define MULTIPARAM_V4_ACTIVE_WINDOW_LATEST_MS        100U
#define MULTIPARAM_V4_STOP_ACTIVE_TOTAL_TIMEOUT_MS    3000U
#define MULTIPARAM_V4_RX_FORCE_ABORT_DELAY_MS         100U
#define MULTIPARAM_V4_MAX_RETRY                       UART6_COMM_MAX_RETRY
#define MULTIPARAM_V4_PARAM_PROTOCOL_VERSION          0x01U
#define MULTIPARAM_V4_PARAM_STATUS                    0x02U
#define MULTIPARAM_V4_PARAM_COMMUNICATION_MODE        0x41U
#define MULTIPARAM_V4_PARAM_SENSOR_ID                 0x43U
#define MULTIPARAM_V4_PARAM_MAX                       0x9FU
#define MULTIPARAM_V4_FUNCTION_READ                   ((uint8_t)'R')
#define MULTIPARAM_V4_FUNCTION_WRITE                  ((uint8_t)'W')
#define MULTIPARAM_V4_FUNCTION_DENSITY                ((uint8_t)'D')
#define MULTIPARAM_V4_FUNCTION_WATER                  ((uint8_t)'L')
#define MULTIPARAM_V4_FUNCTION_MAGNETIC_ZERO          ((uint8_t)'M')
#define MULTIPARAM_V4_FUNCTION_LEVEL                  ((uint8_t)'S')

static uint8_t s_expected_address = MULTIPARAM_V4_ANY_ADDRESS;
static uint8_t s_active_dma_buffer[MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE];
static volatile uint16_t s_active_dma_consumed = 0U;
static volatile uint8_t s_active_receive_requested = 0U;
static volatile uint8_t s_active_receive_running = 0U;
static volatile uint8_t s_receive_restart_pending = 0U;
static volatile uint8_t s_receive_restart_in_progress = 0U;
static volatile uint8_t s_receive_restart_wait_started = 0U;
static volatile uint32_t s_receive_restart_wait_tick = 0U;
static volatile uint8_t s_receive_abort_requested = 0U;
static volatile uint8_t s_receive_abort_in_progress = 0U;

static uint8_t s_stream_buffer[MULTIPARAM_V4_STREAM_BUFFER_SIZE];
static volatile uint16_t s_stream_head = 0U;
static volatile uint16_t s_stream_tail = 0U;
static uint8_t s_deferred_bytes[MULTIPARAM_V4_DEFERRED_BYTE_BUDGET];
static volatile uint8_t s_deferred_parse_requested = 0U;
static uint8_t s_parse_buffer[MULTIPARAM_V4_PARSE_BUFFER_SIZE];
static uint16_t s_parse_length = 0U;

static multiparam_v4_snapshot_t s_latest_snapshot;
static multiparam_v4_diagnostics_t s_diagnostics;
static uint8_t s_sequence_valid = 0U;
static uint16_t s_last_sequence = 0U;
static uint8_t s_timeout_latched = 0U;
static multiparam_v4_communication_mode_t s_communication_mode = MULTIPARAM_V4_COMMUNICATION_UNKNOWN;

/*
 * 函数用途：返回可用于交互请求的确定地址。
 * 调用场景：主动帧尚未锁定地址时的R01探测，以及地址锁定后的普通事务。
 * 关键约束：ANY只用于接收过滤，绝不能作为0xFF请求地址发到总线。
 */
static uint8_t MULTIPARAM_V4_GetRequestAddress(void)
{
    return (s_expected_address == MULTIPARAM_V4_ANY_ADDRESS) ? 0U : s_expected_address;
}

/*
 * 函数用途：按小端字节序还原32位无符号原始值。
 * 调用场景：主动帧参数区和8字节交互应答的数据区解析。
 */
static uint32_t MULTIPARAM_V4_DecodeU32Le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/*
 * 函数用途：把32位原始值按小端字节序写入线协议数据区。
 * 调用场景：生成8字节读写、模式和功能控制请求。
 */
static void MULTIPARAM_V4_EncodeU32Le(uint32_t value, uint8_t *data)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/*
 * 函数用途：把IEEE 754单精度原始位转换为浮点值。
 * 调用场景：解析V4.0浮点参数；调用方还需检查有限性和业务范围。
 */
static float MULTIPARAM_V4_RawToFloat(uint32_t raw)
{
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/*
 * 函数用途：判断参数码是否属于V4.0可读区域。
 * 调用场景：交互读取入口的本地边界检查。
 */
static uint8_t MULTIPARAM_V4_IsReadableParameter(uint8_t parameter)
{
    return (uint8_t)(((parameter <= 25U) ||
                      ((parameter >= 65U) && (parameter <= MULTIPARAM_V4_PARAM_MAX))) ? 1U : 0U);
}

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
static uint32_t MULTIPARAM_V4_StartReceiveDmaInternal(void)
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
 * 函数用途：解析状态字低四位为测量模式和功能状态。
 * 调用场景：主动快照发布以及L/M目标状态核对。
 */
static void MULTIPARAM_V4_DecodeOperatingState(uint32_t status_word,
                                               multiparam_v4_measurement_mode_t *mode,
                                               multiparam_v4_feature_state_t *feature)
{
    uint32_t state = status_word & 0x0FU;

    *mode = MULTIPARAM_V4_MEASUREMENT_INVALID;
    *feature = MULTIPARAM_V4_FEATURE_INVALID;
    if (state == 0U) {
        *mode = MULTIPARAM_V4_MEASUREMENT_DENSITY;
        *feature = MULTIPARAM_V4_FEATURE_NONE;
    } else if (state == 1U) {
        *mode = MULTIPARAM_V4_MEASUREMENT_LEVEL;
        *feature = MULTIPARAM_V4_FEATURE_NONE;
    } else if (state == 2U) {
        *mode = MULTIPARAM_V4_MEASUREMENT_DENSITY;
        *feature = MULTIPARAM_V4_FEATURE_WATER;
    } else if (state == 3U) {
        *mode = MULTIPARAM_V4_MEASUREMENT_DENSITY;
        *feature = MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO;
    }
}

/*
 * 函数用途：发布通过固定字段、地址、CRC和序号检查的主动快照。
 * 调用场景：PendSV有界流解析器完成一帧解析后。
 * 关键约束：重复帧和乱序帧只记诊断，不刷新新鲜度或业务代次。
 */
static uint8_t MULTIPARAM_V4_PublishSnapshot(multiparam_v4_snapshot_t *candidate)
{
    if (s_sequence_valid != 0U) {
        uint16_t delta = (uint16_t)(candidate->sequence - s_last_sequence);

        if (delta == 0U) {
            s_diagnostics.duplicate_frames++;
            return 0U;
        }
        if (delta >= 0x8000U) {
            if ((s_latest_snapshot.valid != 0U) &&
                ((HAL_GetTick() - s_latest_snapshot.received_tick) <=
                 MULTIPARAM_V4_ACTIVE_TIMEOUT_MS)) {
                s_diagnostics.out_of_order_frames++;
                return 0U;
            }
            s_diagnostics.sequence_resets++;
        }
        else if (delta > 1U) {
            s_diagnostics.lost_frames += (uint32_t)(delta - 1U);
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
    s_diagnostics.valid_frames++;
    s_timeout_latched = 0U;
    return 1U;
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
        uint16_t header_offset = 0U;
        uint32_t parse_result;

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
        if (s_parse_length < MULTIPARAM_V4_ACTIVE_FRAME_SIZE) {
            return published;
        }

        candidates_checked++;
        s_diagnostics.frames_seen++;
        parse_result = MULTIPARAM_V4_ParseActiveFrame(s_parse_buffer, &candidate);
        if (parse_result == NO_ERROR) {
            MULTIPARAM_V4_DiscardParseBytes(MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
            published = (uint8_t)(published |
                                 MULTIPARAM_V4_PublishSnapshot(&candidate));
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
        MULTIPARAM_V4_DiscardParseBytes(1U);
        s_diagnostics.resync_events++;
    }
    return published;
}

void MULTIPARAM_V4_Init(uint8_t expected_address)
{
    MULTIPARAM_V4_StopActiveReceive();
    s_expected_address = expected_address;
    s_stream_head = 0U;
    s_stream_tail = 0U;
    s_parse_length = 0U;
    s_sequence_valid = 0U;
    s_last_sequence = 0U;
    s_timeout_latched = 0U;
    s_deferred_parse_requested = 0U;
    s_receive_restart_in_progress = 0U;
    s_receive_restart_wait_started = 0U;
    s_receive_restart_wait_tick = 0U;
    s_receive_abort_requested = 0U;
    s_receive_abort_in_progress = 0U;
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_UNKNOWN;
    memset(&s_latest_snapshot, 0, sizeof(s_latest_snapshot));
    memset(&s_diagnostics, 0, sizeof(s_diagnostics));
}

void MULTIPARAM_V4_Deinit(void)
{
    MULTIPARAM_V4_StopActiveReceive();
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_UNKNOWN;
}

uint32_t MULTIPARAM_V4_StartActiveReceive(void)
{
    uint32_t result;
    uint32_t primask;

    (void)HAL_UART_DMAStop(&huart6);
    primask = __get_PRIMASK();
    __disable_irq();
    s_stream_head = 0U;
    s_stream_tail = 0U;
    s_parse_length = 0U;
    s_deferred_parse_requested = 0U;
    s_sequence_valid = 0U;
    s_timeout_latched = 0U;
    s_latest_snapshot.valid = 0U;
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
        s_communication_mode = MULTIPARAM_V4_COMMUNICATION_ACTIVE;
    } else {
        s_diagnostics.receive_restart_errors++;
    }
    return result;
}

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
    if (primask == 0U) {
        __enable_irq();
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
 * 函数用途：由周期定时中断检查V4接收恢复是否需要继续推进。
 * 调用场景：CPU2主循环可能长期阻塞时，由TIM4业务节拍调用。
 * 关键约束：只检查状态、记录异步终止请求并挂起PendSV，不停止DMA、不等待。
 */
void MULTIPARAM_V4_PollRecoveryFromTimerISR(void)
{
    uint32_t now;

    if ((s_active_receive_requested == 0U) ||
        (s_receive_restart_pending == 0U) ||
        (s_receive_restart_in_progress != 0U)) {
        return;
    }
    if (huart6.RxState == HAL_UART_STATE_READY) {
        MULTIPARAM_V4_RequestDeferredFromISR();
        return;
    }

    now = HAL_GetTick();
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

uint8_t MULTIPARAM_V4_IsActiveReceiverRunning(void)
{
    return s_active_receive_running;
}

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

uint32_t MULTIPARAM_V4_ParseActiveFrame(const uint8_t frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE],
                                        multiparam_v4_snapshot_t *snapshot)
{
    uint32_t index;

    if ((frame == NULL) || (snapshot == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if ((frame[0] != MULTIPARAM_V4_ACTIVE_HEADER0) ||
        (frame[1] != MULTIPARAM_V4_ACTIVE_HEADER1) ||
        (frame[2] != MULTIPARAM_V4_ACTIVE_FORMAT_VERSION) ||
        (frame[3] != MULTIPARAM_V4_ACTIVE_FRAME_TYPE) ||
        (frame[7] != MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT) ||
        (frame[8] != (uint8_t)(MULTIPARAM_V4_ACTIVE_DATA_SIZE & 0xFFU)) ||
        (frame[9] != (uint8_t)(MULTIPARAM_V4_ACTIVE_DATA_SIZE >> 8U))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if ((s_expected_address != MULTIPARAM_V4_ANY_ADDRESS) &&
        (frame[4] != s_expected_address)) {
        return SENSOR_ADDRESS_MISMATCH;
    }
    if (!SlaveCheckCRC(frame, (int)MULTIPARAM_V4_ACTIVE_FRAME_SIZE)) {
        return SENSOR_BCC_ERROR;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->address = frame[4];
    snapshot->sequence = (uint16_t)(((uint16_t)frame[6] << 8U) | frame[5]);
    for (index = 0U; index < MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT; index++) {
        snapshot->raw_parameter[index] = MULTIPARAM_V4_DecodeU32Le(frame + 10U + index * 4U);
    }
    snapshot->software_version = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[0]);
    snapshot->protocol_version = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[1]);
    snapshot->status_word = snapshot->raw_parameter[2];
    snapshot->magnetic_zero_voltage = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[3]);
    snapshot->measurement_frequency_hz = (int32_t)snapshot->raw_parameter[4];
    snapshot->water_capacitance_pf = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[5]);
    snapshot->temperature_c = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[6]);
    snapshot->density_kg_m3 = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[7]);
    snapshot->dynamic_viscosity_cp = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[8]);
    snapshot->kinematic_viscosity_cst = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[9]);
    snapshot->supply_voltage_v = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[10]);
    snapshot->angle_x_deg = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[11]);
    snapshot->angle_y_deg = MULTIPARAM_V4_RawToFloat(snapshot->raw_parameter[12]);
    MULTIPARAM_V4_DecodeOperatingState(snapshot->status_word,
                                       &snapshot->measurement_mode,
                                       &snapshot->feature_state);

    if ((!isfinite(snapshot->software_version)) ||
        (!isfinite(snapshot->protocol_version)) ||
        (!isfinite(snapshot->magnetic_zero_voltage)) ||
        (!isfinite(snapshot->water_capacitance_pf)) ||
        (!isfinite(snapshot->temperature_c)) ||
        (!isfinite(snapshot->density_kg_m3)) ||
        (!isfinite(snapshot->dynamic_viscosity_cp)) ||
        (!isfinite(snapshot->kinematic_viscosity_cst)) ||
        (!isfinite(snapshot->supply_voltage_v)) ||
        (!isfinite(snapshot->angle_x_deg)) ||
        (!isfinite(snapshot->angle_y_deg)) ||
        (snapshot->measurement_mode == MULTIPARAM_V4_MEASUREMENT_INVALID)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (fabsf(snapshot->protocol_version - MULTIPARAM_V4_PROTOCOL_VERSION_VALUE) > 0.01f) {
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    }
    return NO_ERROR;
}

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

void MULTIPARAM_V4_Service(void)
{
    uint32_t primask;
    uint32_t now = HAL_GetTick();

    /* 需要强制终止DMA的恢复留在线程态，避免PendSV进入HAL超时等待。 */
    if (MULTIPARAM_V4_TryClaimReceiveRestart(0U) != 0U) {
        uint32_t restart_result;

        (void)HAL_UART_DMAStop(&huart6);
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        restart_result = MULTIPARAM_V4_StartReceiveDmaInternal();
        if (restart_result != NO_ERROR) {
            s_diagnostics.receive_restart_errors++;
        }
        MULTIPARAM_V4_CompleteReceiveRestart(restart_result);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_active_receive_requested != 0U) &&
        (s_latest_snapshot.valid != 0U) &&
        ((now - s_latest_snapshot.received_tick) >= MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) &&
        (s_timeout_latched == 0U)) {
        s_timeout_latched = 1U;
        s_diagnostics.timeout_events++;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

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

void MULTIPARAM_V4_GetDiagnostics(multiparam_v4_diagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(diagnostics, &s_diagnostics, sizeof(*diagnostics));
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t MULTIPARAM_V4_IsSnapshotFresh(uint32_t maximum_age_ms)
{
    multiparam_v4_snapshot_t snapshot;

    if (MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) != NO_ERROR) {
        return 0U;
    }
    return (uint8_t)(((HAL_GetTick() - snapshot.received_tick) <= maximum_age_ms) ? 1U : 0U);
}

uint32_t MULTIPARAM_V4_GetSnapshotAgeMs(void)
{
    multiparam_v4_snapshot_t snapshot;

    if (MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) != NO_ERROR) {
        return UINT32_MAX;
    }
    return HAL_GetTick() - snapshot.received_tick;
}

uint8_t MULTIPARAM_V4_CalculateChecksum(const uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE])
{
    uint32_t sum = 0U;

    if (frame == NULL) {
        return 0U;
    }
    for (uint32_t index = 0U; index < 7U; index++) {
        sum += frame[index];
    }
    return (uint8_t)(sum & 0xFFU);
}

void MULTIPARAM_V4_BuildRequestFrame(uint8_t address,
                                     uint8_t function,
                                     uint32_t raw_value,
                                     uint8_t parameter,
                                     uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE])
{
    if (frame == NULL) {
        return;
    }
    frame[0] = address;
    frame[1] = function;
    MULTIPARAM_V4_EncodeU32Le(raw_value, frame + 2U);
    frame[6] = parameter;
    frame[7] = MULTIPARAM_V4_CalculateChecksum(frame);
}

uint32_t MULTIPARAM_V4_ValidateReply(const uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
                                    const uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE])
{
    if ((request == NULL) || (reply == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_CalculateChecksum(reply) != reply[7]) {
        return SENSOR_BCC_ERROR;
    }
    if ((reply[0] != request[0]) ||
        (reply[1] != (uint8_t)(request[1] | 0x80U))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if (reply[6] == 0xFFU) {
        return SENSOR_REMOTE_INTERNAL_ERROR;
    }
    if (reply[6] != request[6]) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    if ((request[1] == MULTIPARAM_V4_FUNCTION_WRITE) &&
        (memcmp(request + 2U, reply + 2U, 4U) != 0)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    return NO_ERROR;
}

/*
 * 函数用途：在不清除快照和序号的前提下恢复主动接收。
 * 调用场景：参数65停流事务被一帧主动上报抢占后。
 * 关键约束：保留已收到帧并继续使用原地址，启动失败时交给延后恢复。
 */
static uint32_t MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot(void)
{
    uint32_t result;

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
    }
    return result;
}

/*
 * 函数用途：在停止主动上报的写事务中区分8字节应答和64字节优先主动帧。
 * 调用场景：主动模式写参数65为1的唯一事务。
 * 关键约束：主动帧必须收满、保留并恢复常驻接收；调用方随后等待下一安全窗口重试。
 */
static uint32_t MULTIPARAM_V4_TransceiveStopActiveOnce(const uint8_t request[8],
                                                        uint8_t reply[8],
                                                        uint32_t timeout_ms,
                                                        uint8_t *active_preempted)
{
    uint8_t receive_buffer[MULTIPARAM_V4_ACTIVE_FRAME_SIZE];
    multiparam_v4_snapshot_t candidate;
    uint32_t start_tick;
    uint32_t result;
    uint16_t received_length;

    if ((request == NULL) || (reply == NULL) || (active_preempted == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    memset(receive_buffer, 0, sizeof(receive_buffer));
    memset(reply, 0, 8U);
    *active_preempted = 0U;
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;

    if (HAL_UART_Receive_DMA(&huart6, receive_buffer,
                             MULTIPARAM_V4_ACTIVE_FRAME_SIZE) != HAL_OK) {
        return COMM_UART_TRANSFER_ERROR;
    }
    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)request, 8U) != HAL_OK) {
        (void)HAL_UART_DMAStop(&huart6);
        return COMM_UART_TRANSFER_ERROR;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (HasEffectiveCommandSwitchRequest()) {
            (void)HAL_UART_DMAStop(&huart6);
            return STATE_SWITCH;
        }
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            (void)HAL_UART_DMAStop(&huart6);
            return COMM_UART_TRANSFER_ERROR;
        }
        received_length = (huart6.hdmarx == NULL)
                              ? 0U
                              : (uint16_t)(MULTIPARAM_V4_ACTIVE_FRAME_SIZE -
                                           __HAL_DMA_GET_COUNTER(huart6.hdmarx));
        if ((received_length >= 2U) &&
            (receive_buffer[0] == MULTIPARAM_V4_ACTIVE_HEADER0) &&
            (receive_buffer[1] == MULTIPARAM_V4_ACTIVE_HEADER1)) {
            if (received_length < MULTIPARAM_V4_ACTIVE_FRAME_SIZE) {
                HAL_Delay(1U);
                continue;
            }
            (void)HAL_UART_DMAStop(&huart6);
            result = MULTIPARAM_V4_ParseActiveFrame(receive_buffer, &candidate);
            if (result != NO_ERROR) {
                s_diagnostics.interactive_transactions++;
                s_diagnostics.interactive_errors++;
                return result;
            }
            result = MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot();
            MULTIPARAM_V4_FeedBytes(receive_buffer, MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
            *active_preempted = 1U;
            s_diagnostics.interactive_transactions++;
            return result;
        }
        if ((received_length >= 8U) && (huart6.gState == HAL_UART_STATE_READY)) {
            (void)HAL_UART_DMAStop(&huart6);
            memcpy(reply, receive_buffer, 8U);
            s_diagnostics.interactive_transactions++;
            return MULTIPARAM_V4_ValidateReply(request, reply);
        }
        HAL_Delay(1U);
    }

    received_length = (huart6.hdmarx == NULL)
                          ? 0U
                          : (uint16_t)(MULTIPARAM_V4_ACTIVE_FRAME_SIZE -
                                       __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    (void)HAL_UART_DMAStop(&huart6);
    s_diagnostics.interactive_transactions++;
    s_diagnostics.interactive_errors++;
    return (received_length == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
}

/*
 * 函数用途：停止其它V4 DMA后完成一次8字节请求和8字节应答事务。
 * 调用场景：交互读取、明确值写入、模式选择和L/M单次翻转。
 * 关键约束：函数不自动重试；幂等性策略由上层按命令类别决定。
 */
static uint32_t MULTIPARAM_V4_TransceiveOnceInternal(const uint8_t request[8],
                                                     uint8_t reply[8],
                                                     uint32_t timeout_ms,
                                                     uint8_t check_command_switch)
{
    uint32_t start_tick;
    uint16_t received_length;

    if ((request == NULL) || (reply == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    memset(reply, 0, 8U);
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;

    if (HAL_UART_Receive_DMA(&huart6, reply, 8U) != HAL_OK) {
        return COMM_UART_TRANSFER_ERROR;
    }
    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)request, 8U) != HAL_OK) {
        (void)HAL_UART_DMAStop(&huart6);
        return COMM_UART_TRANSFER_ERROR;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if ((check_command_switch != 0U) && HasEffectiveCommandSwitchRequest()) {
            (void)HAL_UART_DMAStop(&huart6);
            return STATE_SWITCH;
        }
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            (void)HAL_UART_DMAStop(&huart6);
            return COMM_UART_TRANSFER_ERROR;
        }
        received_length = (huart6.hdmarx == NULL)
                              ? 0U
                              : (uint16_t)(8U - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
        if ((received_length >= 8U) && (huart6.gState == HAL_UART_STATE_READY)) {
            (void)HAL_UART_DMAStop(&huart6);
            s_diagnostics.interactive_transactions++;
            return MULTIPARAM_V4_ValidateReply(request, reply);
        }
        HAL_Delay(1U);
    }

    received_length = (huart6.hdmarx == NULL)
                          ? 0U
                          : (uint16_t)(8U - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    (void)HAL_UART_DMAStop(&huart6);
    s_diagnostics.interactive_transactions++;
    s_diagnostics.interactive_errors++;
    return (received_length == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
}

/*
 * 函数用途：执行允许重试的幂等8字节事务。
 * 调用场景：R读取和D/S模式选择；L/M翻转不得使用本函数。
 */
/* 普通交互事务允许新命令打断；参数65恢复事务使用内部入口完成必要收尾。 */
static uint32_t MULTIPARAM_V4_TransceiveOnce(const uint8_t request[8],
                                             uint8_t reply[8],
                                             uint32_t timeout_ms)
{
    return MULTIPARAM_V4_TransceiveOnceInternal(request, reply, timeout_ms, 1U);
}

static uint32_t MULTIPARAM_V4_TransceiveIdempotent(const uint8_t request[8], uint8_t reply[8])
{
    uint32_t last_error = SENSOR_DEVICE_COMM_TIMEOUT;

    for (uint32_t attempt = 0U; attempt < MULTIPARAM_V4_MAX_RETRY; attempt++) {
        last_error = MULTIPARAM_V4_TransceiveOnce(request,
                                                  reply,
                                                  MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
        if ((last_error == NO_ERROR) || (last_error == STATE_SWITCH)) {
            return last_error;
        }
        HAL_Delay(DSM_BCC_DELAY);
    }
    return last_error;
}

/*
 * 函数用途：不经过通信方式门禁读取一个V4参数原始值。
 * 调用场景：普通交互读取和参数65写应答丢失后的事实恢复。
 */
static uint32_t MULTIPARAM_V4_ReadParamRawInternal(uint8_t parameter,
                                                   uint32_t *raw_value,
                                                   uint8_t retry_allowed,
                                                   uint32_t timeout_ms)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t result;

    if ((raw_value == NULL) || (MULTIPARAM_V4_IsReadableParameter(parameter) == 0U)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                    MULTIPARAM_V4_FUNCTION_READ,
                                    0U,
                                    parameter,
                                    request);
    result = (retry_allowed != 0U)
                 ? MULTIPARAM_V4_TransceiveIdempotent(request, reply)
                 : MULTIPARAM_V4_TransceiveOnce(request, reply, timeout_ms);
    if (result == NO_ERROR) {
        *raw_value = MULTIPARAM_V4_DecodeU32Le(reply + 2U);
    }
    return result;
}

uint32_t MULTIPARAM_V4_ReadParamRaw(uint8_t parameter, uint32_t *raw_value)
{
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    return MULTIPARAM_V4_ReadParamRawInternal(parameter,
                                              raw_value,
                                              1U,
                                              MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
}

uint32_t MULTIPARAM_V4_ReadIntParam(uint8_t parameter, int32_t *value)
{
    uint32_t raw;
    uint32_t result;

    if (value == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRaw(parameter, &raw);
    if (result == NO_ERROR) {
        *value = (int32_t)raw;
    }
    return result;
}

/*
 * 函数用途：读取多参数传感器通信协议 V4.0 的 R67 传感器号。
 * 调用场景：CPU2 已确认 V4 协议并处于交互通信后读取物理传感器编号。
 * 关键约束：V4 的 R22 是密度扫频原始量，不能沿用 V3.0 的 R22 编号语义；编号必须为正整数。
 */
uint32_t MULTIPARAM_V4_ReadSensorID(uint32_t *sensor_id)
{
    int32_t value;
    uint32_t result;

    if (sensor_id == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadIntParam(MULTIPARAM_V4_PARAM_SENSOR_ID, &value);
    if (result != NO_ERROR) {
        return result;
    }
    if (value <= 0) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *sensor_id = (uint32_t)value;
    return NO_ERROR;
}

uint32_t MULTIPARAM_V4_ReadFloatParam(uint8_t parameter, float *value)
{
    uint32_t raw;
    uint32_t result;

    if (value == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRaw(parameter, &raw);
    if (result == NO_ERROR) {
        float parsed = MULTIPARAM_V4_RawToFloat(raw);
        if (!isfinite(parsed)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        *value = parsed;
    }
    return result;
}

/*
 * 函数用途：以单次短事务读取一个有限浮点参数。
 * 调用场景：交互核心测量完成后的非关键调试量刷新。
 * 关键约束：不重试，避免附加量缺失时把业务流程阻塞数秒。
 */
uint32_t MULTIPARAM_V4_ReadFloatParamOnce(uint8_t parameter, float *value)
{
    uint32_t raw;
    uint32_t result;

    if (value == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRawInternal(parameter,
                                                &raw,
                                                0U,
                                                MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS);
    if (result == NO_ERROR) {
        float parsed = MULTIPARAM_V4_RawToFloat(raw);
        if (!isfinite(parsed)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        *value = parsed;
    }
    return result;
}

/*
 * 函数用途：读取R02并统一解析测量模式、功能状态和异常位原值。
 * 调用场景：交互测量读取R04、R05、R11和R12之前。
 * 关键约束：无法识别的低四位状态视为响应格式错误。
 */
uint32_t MULTIPARAM_V4_ReadOperatingState(uint32_t *status_word,
                                          multiparam_v4_measurement_mode_t *mode,
                                          multiparam_v4_feature_state_t *feature)
{
    uint32_t raw_status;
    uint32_t result;

    if ((status_word == NULL) || (mode == NULL) || (feature == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRaw(MULTIPARAM_V4_PARAM_STATUS, &raw_status);
    if (result != NO_ERROR) {
        return result;
    }
    MULTIPARAM_V4_DecodeOperatingState(raw_status, mode, feature);
    if ((*mode == MULTIPARAM_V4_MEASUREMENT_INVALID) ||
        (*feature == MULTIPARAM_V4_FEATURE_INVALID)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *status_word = raw_status;
    return NO_ERROR;
}

uint32_t MULTIPARAM_V4_WriteParamRaw(uint8_t parameter, uint32_t raw_value)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t readback = 0U;
    uint32_t result;

    if ((parameter < 65U) || (parameter > MULTIPARAM_V4_PARAM_MAX) ||
        (parameter == MULTIPARAM_V4_PARAM_COMMUNICATION_MODE) ||
        (s_active_receive_requested != 0U)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                    MULTIPARAM_V4_FUNCTION_WRITE,
                                    raw_value,
                                    parameter,
                                    request);
    result = MULTIPARAM_V4_TransceiveOnce(request, reply, MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
    if (result == NO_ERROR) {
        return NO_ERROR;
    }

    if (MULTIPARAM_V4_ReadParamRawInternal(parameter,
                                           &readback,
                                           1U,
                                           MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS) == NO_ERROR) {
        if (readback == raw_value) {
            return NO_ERROR;
        }
    }
    return result;
}

uint32_t MULTIPARAM_V4_ProbeProtocolVersion(float *protocol_version)
{
    uint32_t raw;
    uint32_t result;
    float value;

    if (protocol_version == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRawInternal(MULTIPARAM_V4_PARAM_PROTOCOL_VERSION,
                                                &raw,
                                                0U,
                                                MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS);
    if (result != NO_ERROR) {
        return result;
    }
    value = MULTIPARAM_V4_RawToFloat(raw);
    if (!isfinite(value)) {
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    }
    *protocol_version = value;
    if (fabsf(value - MULTIPARAM_V4_PROTOCOL_VERSION_VALUE) > 0.01f) {
        /* 向自动识别层保留实际R01值，由其判断是否继续按V3.0读取R22。 */
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    }
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_INTERACTIVE;
    return NO_ERROR;
}

/*
 * 函数用途：发送可重试的D/S模式选择并用状态字确认最终模式。
 * 调用场景：交互通信模式下切换密度或液位测量过程。
 */
static uint32_t MULTIPARAM_V4_SelectMeasurementMode(uint8_t function,
                                                    multiparam_v4_measurement_mode_t expected_mode)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t status_word;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t result;

    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(), function, 0U, 0U, request);
    result = MULTIPARAM_V4_TransceiveIdempotent(request, reply);
    if (result != NO_ERROR) {
        return result;
    }
    result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
    if (result != NO_ERROR) {
        return result;
    }
    return ((mode == expected_mode) && (feature == MULTIPARAM_V4_FEATURE_NONE))
               ? NO_ERROR
               : SENSOR_STREAM_STATE_ERROR;
}

uint32_t MULTIPARAM_V4_SelectDensityMode(void)
{
    return MULTIPARAM_V4_SelectMeasurementMode(MULTIPARAM_V4_FUNCTION_DENSITY,
                                               MULTIPARAM_V4_MEASUREMENT_DENSITY);
}

uint32_t MULTIPARAM_V4_SelectLevelMode(void)
{
    return MULTIPARAM_V4_SelectMeasurementMode(MULTIPARAM_V4_FUNCTION_LEVEL,
                                               MULTIPARAM_V4_MEASUREMENT_LEVEL);
}

/*
 * 函数用途：发送一次非幂等L/M翻转命令并读取状态字恢复事实。
 * 调用场景：目标状态与当前状态不同，且UART6处于交互通信模式时。
 * 关键约束：应答丢失后禁止直接重发；无论应答结果都先读状态字。
 */
static uint32_t MULTIPARAM_V4_ToggleFeatureOnce(uint8_t function,
                                                multiparam_v4_feature_state_t expected_feature)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t command_result;
    int32_t status_word;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t read_result;

    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(), function, 0U, 0U, request);
    command_result = MULTIPARAM_V4_TransceiveOnce(request,
                                                  reply,
                                                  MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
    read_result = MULTIPARAM_V4_ReadIntParam(MULTIPARAM_V4_PARAM_STATUS, &status_word);
    if (read_result != NO_ERROR) {
        return (command_result == NO_ERROR) ? read_result : command_result;
    }
    MULTIPARAM_V4_DecodeOperatingState((uint32_t)status_word, &mode, &feature);
    if ((mode == MULTIPARAM_V4_MEASUREMENT_DENSITY) && (feature == expected_feature)) {
        return NO_ERROR;
    }
    return (command_result == NO_ERROR) ? SENSOR_STREAM_STATE_ERROR : command_result;
}

/*
 * 函数用途：把测水或磁零点功能调整到明确目标状态。
 * 调用场景：业务层请求使能或关闭功能，不允许业务层直接盲发L/M翻转。
 */
static uint32_t MULTIPARAM_V4_EnsureFeature(uint8_t enabled,
                                           multiparam_v4_feature_state_t target_feature,
                                           uint8_t target_function,
                                           multiparam_v4_feature_state_t opposite_feature,
                                           uint8_t opposite_function)
{
    int32_t status_word;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t result;

    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    result = MULTIPARAM_V4_ReadIntParam(MULTIPARAM_V4_PARAM_STATUS, &status_word);
    if (result != NO_ERROR) {
        return result;
    }
    MULTIPARAM_V4_DecodeOperatingState((uint32_t)status_word, &mode, &feature);
    if (mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    if (enabled == 0U) {
        if (feature != target_feature) {
            return NO_ERROR;
        }
        return MULTIPARAM_V4_ToggleFeatureOnce(target_function, MULTIPARAM_V4_FEATURE_NONE);
    }
    if (feature == target_feature) {
        return NO_ERROR;
    }
    if (feature == opposite_feature) {
        result = MULTIPARAM_V4_ToggleFeatureOnce(opposite_function, MULTIPARAM_V4_FEATURE_NONE);
        if (result != NO_ERROR) {
            return result;
        }
    } else if (feature != MULTIPARAM_V4_FEATURE_NONE) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    return MULTIPARAM_V4_ToggleFeatureOnce(target_function, target_feature);
}

uint32_t MULTIPARAM_V4_EnsureWaterEnabled(uint8_t enabled)
{
    return MULTIPARAM_V4_EnsureFeature(enabled,
                                       MULTIPARAM_V4_FEATURE_WATER,
                                       MULTIPARAM_V4_FUNCTION_WATER,
                                       MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO,
                                       MULTIPARAM_V4_FUNCTION_MAGNETIC_ZERO);
}

uint32_t MULTIPARAM_V4_EnsureMagneticZeroEnabled(uint8_t enabled)
{
    return MULTIPARAM_V4_EnsureFeature(enabled,
                                       MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO,
                                       MULTIPARAM_V4_FUNCTION_MAGNETIC_ZERO,
                                       MULTIPARAM_V4_FEATURE_WATER,
                                       MULTIPARAM_V4_FUNCTION_WATER);
}

uint32_t MULTIPARAM_V4_EnterInteractive(void)
{
    uint8_t request[8];
    uint8_t reply[8];
    multiparam_v4_snapshot_t snapshot;
    uint32_t result;
    uint32_t readback = 0U;
    uint32_t operation_start;
    uint32_t restore_result;
    uint8_t active_preempted;

    if ((s_communication_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) &&
        (s_active_receive_requested == 0U)) {
        return NO_ERROR;
    }
    if (s_active_receive_requested == 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    operation_start = HAL_GetTick();
    for (;;) {
        uint32_t event_age;

        MULTIPARAM_V4_Service();
        if (MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) == NO_ERROR) {
            event_age = HAL_GetTick() - snapshot.received_tick;
            if ((event_age >= MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS) &&
                (event_age <= MULTIPARAM_V4_ACTIVE_WINDOW_LATEST_MS)) {
                MULTIPARAM_V4_StopActiveReceive();
                MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                                MULTIPARAM_V4_FUNCTION_WRITE,
                                                1U,
                                                MULTIPARAM_V4_PARAM_COMMUNICATION_MODE,
                                                request);
                result = MULTIPARAM_V4_TransceiveStopActiveOnce(
                    request, reply, MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS,
                    &active_preempted);
                if (active_preempted != 0U) {
                    if (result != NO_ERROR) {
                        return result;
                    }
                    /* 主动帧优先，保留该帧后等待它的20ms保护期再重试停流。 */
                    continue;
                }
                break;
            }
        }
        if ((HAL_GetTick() - operation_start) >=
            MULTIPARAM_V4_STOP_ACTIVE_TOTAL_TIMEOUT_MS) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
        HAL_Delay(1U);
    }

    if (result == NO_ERROR) {
        s_communication_mode = MULTIPARAM_V4_COMMUNICATION_INTERACTIVE;
        return NO_ERROR;
    }
    if (result == STATE_SWITCH) {
        /* 新命令优先：只恢复原主动接收，不再追加读回事务。 */
        restore_result = MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot();
        return (restore_result == NO_ERROR) ? STATE_SWITCH : restore_result;
    }

    if ((MULTIPARAM_V4_ReadParamRawInternal(MULTIPARAM_V4_PARAM_COMMUNICATION_MODE,
                                            &readback,
                                            0U,
                                            MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS) == NO_ERROR) &&
        (readback == 1U)) {
        s_communication_mode = MULTIPARAM_V4_COMMUNICATION_INTERACTIVE;
        return NO_ERROR;
    }

    if (MULTIPARAM_V4_StartActiveReceive() != NO_ERROR) {
        return COMM_UART_TRANSFER_ERROR;
    }
    return result;
}

uint32_t MULTIPARAM_V4_EnterActive(void)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t result;
    uint32_t start_result;
    uint32_t start_tick;
    uint32_t previous_generation = s_latest_snapshot.generation;

    if ((s_communication_mode == MULTIPARAM_V4_COMMUNICATION_ACTIVE) &&
        (s_active_receive_requested != 0U)) {
        return NO_ERROR;
    }
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                    MULTIPARAM_V4_FUNCTION_WRITE,
                                    0U,
                                    MULTIPARAM_V4_PARAM_COMMUNICATION_MODE,
                                    request);
    /* 恢复原主动通信属于必要收尾，不能被已经到达的新命令在发送前打断。 */
    result = MULTIPARAM_V4_TransceiveOnceInternal(
        request, reply, MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS, 0U);
    start_result = MULTIPARAM_V4_StartActiveReceive();
    if (start_result != NO_ERROR) {
        return start_result;
    }
    /* 写入已确认时，主动接收启动后即可让位给新命令；帧接收继续由中断链推进。 */
    if ((result == NO_ERROR) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }
    /* 应答丢失时仍等待首帧恢复事实，避免把未确认的通信方式当作主动模式。 */
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) {
        if ((result == NO_ERROR) && HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        MULTIPARAM_V4_Service();
        if (s_latest_snapshot.generation != previous_generation) {
            return NO_ERROR;
        }
        HAL_Delay(1U);
    }
    return (result == NO_ERROR) ? SENSOR_DEVICE_COMM_TIMEOUT : result;
}

multiparam_v4_communication_mode_t MULTIPARAM_V4_GetCommunicationMode(void)
{
    return s_communication_mode;
}
