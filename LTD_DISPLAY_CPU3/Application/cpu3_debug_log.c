#include "cpu3_debug_log.h"

#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CPU3_LOG_TX_QUEUE_BYTES 4096U
#define CPU3_LOG_LINE_MAX_BYTES 384U
#define CPU3_LOG_HIGH_LEVEL_RESERVED_BYTES 512U
#define CPU3_LOG_DROP_NOTICE_MIN_FREE_BYTES 256U
#define CPU3_LOG_DMA_RETRY_INTERVAL_MS 10U
#define CPU3_LOG_NOTICE_INTERVAL_MS 1000U

static uint8_t s_log_tx_queue[CPU3_LOG_TX_QUEUE_BYTES];
static volatile uint16_t s_log_tx_head = 0U;
static volatile uint16_t s_log_tx_tail = 0U;
static volatile uint16_t s_log_tx_length = 0U;
static volatile uint8_t s_log_tx_busy = 0U;
static volatile uint8_t s_log_async_ready = 0U;
static volatile uint32_t s_log_transport_failure_count = 0U;
static volatile uint32_t s_log_next_retry_tick = 0U;
static uint32_t s_log_dropped_record_count = 0U;
static uint32_t s_log_dropped_byte_count = 0U;
static uint32_t s_log_last_notice_tick = 0U;

/* 返回日志级别的现场可读名称。 */
static const char *Cpu3Log_LevelText(Cpu3LogLevel level)
{
    switch (level) {
    case CPU3_LOG_LEVEL_CRITICAL:
        return "严重";
    case CPU3_LOG_LEVEL_WARNING:
        return "警告";
    case CPU3_LOG_LEVEL_INFO:
        return "信息";
    case CPU3_LOG_LEVEL_DEBUG:
        return "调试";
    case CPU3_LOG_LEVEL_OFF:
    default:
        return "关闭";
    }
}

/* 返回当前队列可写字节数；尾指针只会在发送完成中断中前进，旧快照只会低估空间。 */
static uint16_t Cpu3Log_FreeBytes(void)
{
    uint16_t head = s_log_tx_head;
    uint16_t tail = s_log_tx_tail;
    uint16_t used;

    if (head >= tail) {
        used = (uint16_t)(head - tail);
    } else {
        used = (uint16_t)(CPU3_LOG_TX_QUEUE_BYTES - tail + head);
    }

    return (uint16_t)(CPU3_LOG_TX_QUEUE_BYTES - used - 1U);
}

/*
 * 函数用途：把一段完整日志写入单生产者、单消费者环形队列。
 * 调用场景：主循环中的统一日志和普通printf重定向。
 * 关键约束：队列不足时只累计丢弃量，绝不等待；INFO/DEBUG为WARN/CRITICAL保留空间。
 */
static bool Cpu3Log_QueueBytes(Cpu3LogLevel level,
                               const uint8_t *data,
                               uint16_t length,
                               bool count_as_record)
{
    uint16_t head;
    uint16_t first_length;
    uint16_t new_head;
    uint16_t reserve = 0U;

    if ((data == NULL) || (length == 0U) || (s_log_async_ready == 0U)) {
        return false;
    }

    if (level >= CPU3_LOG_LEVEL_INFO) {
        reserve = CPU3_LOG_HIGH_LEVEL_RESERVED_BYTES;
    }
    if (Cpu3Log_FreeBytes() < (uint16_t)(length + reserve)) {
        if (count_as_record) {
            s_log_dropped_record_count++;
        }
        s_log_dropped_byte_count += length;
        return false;
    }

    head = s_log_tx_head;
    first_length = (uint16_t)(CPU3_LOG_TX_QUEUE_BYTES - head);
    if (first_length > length) {
        first_length = length;
    }
    memcpy(&s_log_tx_queue[head], data, first_length);
    if (first_length < length) {
        memcpy(s_log_tx_queue, &data[first_length], (uint16_t)(length - first_length));
    }

    new_head = (uint16_t)(head + length);
    if (new_head >= CPU3_LOG_TX_QUEUE_BYTES) {
        new_head = (uint16_t)(new_head - CPU3_LOG_TX_QUEUE_BYTES);
    }
    __DMB();
    s_log_tx_head = new_head;
    return true;
}

/* 在当前上下文启动下一段连续DMA发送；调用方必须保证不会与发送完成回调并发。 */
static void Cpu3Log_StartNextTransfer(void)
{
    uint32_t now;
    uint16_t head;
    uint16_t tail;
    uint16_t length;

    if ((s_log_async_ready == 0U) || (s_log_tx_busy != 0U)) {
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - s_log_next_retry_tick) < 0) {
        return;
    }

    head = s_log_tx_head;
    tail = s_log_tx_tail;
    if (head == tail) {
        return;
    }

    length = (head > tail) ?
             (uint16_t)(head - tail) :
             (uint16_t)(CPU3_LOG_TX_QUEUE_BYTES - tail);
    s_log_tx_length = length;
    s_log_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart1, &s_log_tx_queue[tail], length) != HAL_OK) {
        s_log_tx_busy = 0U;
        s_log_tx_length = 0U;
        s_log_transport_failure_count++;
        s_log_next_retry_tick = now + CPU3_LOG_DMA_RETRY_INTERVAL_MS;
    }
}

const char *Cpu3Log_ActiveLevelText(void)
{
    return Cpu3Log_LevelText((Cpu3LogLevel)CPU3_LOG_ACTIVE_LEVEL);
}

void Cpu3Log_Init(void)
{
    uint32_t interrupt_mask;

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_log_tx_head = 0U;
    s_log_tx_tail = 0U;
    s_log_tx_length = 0U;
    s_log_tx_busy = 0U;
    s_log_transport_failure_count = 0U;
    s_log_next_retry_tick = 0U;
    s_log_dropped_record_count = 0U;
    s_log_dropped_byte_count = 0U;
    s_log_last_notice_tick = HAL_GetTick();
    s_log_async_ready = 1U;
    if (interrupt_mask == 0U) {
        __enable_irq();
    }
}

bool Cpu3Log_IsAsyncReady(void)
{
    return s_log_async_ready != 0U;
}

void Cpu3Log_WriteRaw(const uint8_t *data, uint16_t length)
{
    (void)Cpu3Log_QueueBytes(CPU3_LOG_LEVEL_INFO, data, length, false);
}

void Cpu3Log_Print(Cpu3LogLevel level, const char *module, const char *format, ...)
{
    char line[CPU3_LOG_LINE_MAX_BYTES];
    va_list args;
    int written;
    uint16_t used;
    uint16_t available;

    if ((level == CPU3_LOG_LEVEL_OFF) ||
        (level > (Cpu3LogLevel)CPU3_LOG_ACTIVE_LEVEL) ||
        (format == NULL)) {
        return;
    }

    if (module == NULL) {
        module = "未分类";
    }

    written = snprintf(line,
                       sizeof(line) - 2U,
                       "[%010lu][%s][%s] ",
                       (unsigned long)HAL_GetTick(),
                       Cpu3Log_LevelText(level),
                       module);
    if (written < 0) {
        return;
    }
    used = (written < (int)(sizeof(line) - 2U)) ?
           (uint16_t)written : (uint16_t)(sizeof(line) - 3U);
    available = (uint16_t)(sizeof(line) - used - 2U);

    va_start(args, format);
    written = vsnprintf(&line[used], available, format, args);
    va_end(args);
    if (written < 0) {
        return;
    }
    if (written >= (int)available) {
        used = (uint16_t)(sizeof(line) - 3U);
    } else {
        used = (uint16_t)(used + (uint16_t)written);
    }

    line[used++] = '\r';
    line[used++] = '\n';
    (void)Cpu3Log_QueueBytes(level, (const uint8_t *)line, used, true);
}

void Cpu3Log_Frame(Cpu3LogLevel level,
                   const char *module,
                   const char *direction,
                   const uint8_t *data,
                   uint16_t length)
{
    static const char hex_text[] = "0123456789ABCDEF";
    char line[CPU3_LOG_LINE_MAX_BYTES];
    uint16_t print_length;
    uint16_t used;
    uint16_t i;
    int written;

    if ((level == CPU3_LOG_LEVEL_OFF) ||
        (level > (Cpu3LogLevel)CPU3_LOG_ACTIVE_LEVEL)) {
        return;
    }

    if (module == NULL) {
        module = "未分类";
    }
    if (direction == NULL) {
        direction = "未知";
    }

    print_length = length;
    if (print_length > CPU3_LOG_FRAME_MAX_BYTES) {
        print_length = CPU3_LOG_FRAME_MAX_BYTES;
    }

    written = snprintf(line,
                       sizeof(line) - 2U,
                       "[%010lu][%s][%s] 方向=%s 长度=%u 数据=",
                       (unsigned long)HAL_GetTick(),
                       Cpu3Log_LevelText(level),
                       module,
                       direction,
                       (unsigned int)length);
    if (written < 0) {
        return;
    }
    used = (written < (int)(sizeof(line) - 2U)) ?
           (uint16_t)written : (uint16_t)(sizeof(line) - 3U);

    if ((data == NULL) && (length > 0U)) {
        written = snprintf(&line[used], sizeof(line) - used - 2U, "<空指针>");
        if (written >= (int)(sizeof(line) - used - 2U)) {
            used = (uint16_t)(sizeof(line) - 3U);
        } else if (written > 0) {
            used = (uint16_t)(used + (uint16_t)written);
        }
    } else if (length == 0U) {
        written = snprintf(&line[used], sizeof(line) - used - 2U, "<空帧>");
        if (written >= (int)(sizeof(line) - used - 2U)) {
            used = (uint16_t)(sizeof(line) - 3U);
        } else if (written > 0) {
            used = (uint16_t)(used + (uint16_t)written);
        }
    } else {
        for (i = 0U; i < print_length; i++) {
            if ((uint16_t)(used + 4U) >= sizeof(line)) {
                break;
            }
            line[used++] = hex_text[(data[i] >> 4) & 0x0FU];
            line[used++] = hex_text[data[i] & 0x0FU];
            if ((uint16_t)(i + 1U) < print_length) {
                line[used++] = ' ';
            }
        }
        if ((print_length < length) && ((uint16_t)(used + 20U) < sizeof(line))) {
            written = snprintf(&line[used],
                               sizeof(line) - used - 2U,
                               " ...(+%u字节)",
                               (unsigned int)(length - print_length));
            if (written >= (int)(sizeof(line) - used - 2U)) {
                used = (uint16_t)(sizeof(line) - 3U);
            } else if (written > 0) {
                used = (uint16_t)(used + (uint16_t)written);
            }
        }
    }

    if (used > (uint16_t)(sizeof(line) - 3U)) {
        used = (uint16_t)(sizeof(line) - 3U);
    }
    line[used++] = '\r';
    line[used++] = '\n';
    (void)Cpu3Log_QueueBytes(level, (const uint8_t *)line, used, true);
}

void Cpu3Log_Service(void)
{
    uint32_t now;
    uint32_t interrupt_mask;
    uint32_t dropped_records;
    uint32_t dropped_bytes;
    uint32_t transport_failures;

    if ((s_log_async_ready == 0U) || (__get_IPSR() != 0U)) {
        return;
    }

    dropped_records = 0U;
    dropped_bytes = 0U;
    transport_failures = 0U;
    now = HAL_GetTick();
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    if (((s_log_dropped_byte_count > 0U) ||
         (s_log_transport_failure_count > 0U)) &&
        ((uint32_t)(now - s_log_last_notice_tick) >= CPU3_LOG_NOTICE_INTERVAL_MS) &&
        (Cpu3Log_FreeBytes() >= CPU3_LOG_DROP_NOTICE_MIN_FREE_BYTES)) {
        dropped_records = s_log_dropped_record_count;
        dropped_bytes = s_log_dropped_byte_count;
        transport_failures = s_log_transport_failure_count;
        s_log_dropped_record_count = 0U;
        s_log_dropped_byte_count = 0U;
        s_log_transport_failure_count = 0U;
        s_log_last_notice_tick = now;
    }
    if (interrupt_mask == 0U) {
        __enable_irq();
    }

    if ((dropped_bytes > 0U) || (transport_failures > 0U)) {
        CPU3_LOG_WARNING("日志",
                         "非阻塞输出发生丢弃 记录=%lu 字节=%lu DMA启动失败=%lu",
                         (unsigned long)dropped_records,
                         (unsigned long)dropped_bytes,
                         (unsigned long)transport_failures);
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    Cpu3Log_StartNextTransfer();
    if (interrupt_mask == 0U) {
        __enable_irq();
    }
}

void Cpu3Log_TxCompleteFromISR(void)
{
    uint16_t new_tail;

    if (s_log_tx_busy == 0U) {
        return;
    }

    new_tail = (uint16_t)(s_log_tx_tail + s_log_tx_length);
    if (new_tail >= CPU3_LOG_TX_QUEUE_BYTES) {
        new_tail = (uint16_t)(new_tail - CPU3_LOG_TX_QUEUE_BYTES);
    }
    s_log_tx_tail = new_tail;
    s_log_tx_length = 0U;
    s_log_tx_busy = 0U;
    Cpu3Log_StartNextTransfer();
}

void Cpu3Log_TxErrorFromISR(void)
{
    s_log_tx_length = 0U;
    s_log_tx_busy = 0U;
    s_log_transport_failure_count++;
    s_log_next_retry_tick = HAL_GetTick() + CPU3_LOG_DMA_RETRY_INTERVAL_MS;
}
