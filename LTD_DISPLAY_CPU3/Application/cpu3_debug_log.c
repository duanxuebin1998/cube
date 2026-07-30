#include "cpu3_debug_log.h"

#include "main.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* CPU3 异步日志发送环形队列容量 4096 字节；用于吸收短时日志突发，队列满时按等级丢弃而不阻塞业务。 */
#define CPU3_LOG_TX_QUEUE_BYTES 4096U
/* 单条格式化日志允许的最大缓冲长度 384 字节；超长内容必须截断并保持字符串终止。 */
#define CPU3_LOG_LINE_MAX_BYTES 384U
/* 为 CRITICAL/WARNING 高等级日志预留的队列空间 512 字节；低等级日志不得占用该保留区。 */
#define CPU3_LOG_HIGH_LEVEL_RESERVED_BYTES 512U
/* 输出日志丢弃汇总通知前要求的最小空闲队列空间 256 字节；避免通知本身加剧拥塞。 */
#define CPU3_LOG_DROP_NOTICE_MIN_FREE_BYTES 256U
/* 日志 DMA 启动失败或忙时的最小重试间隔 10 ms。 */
#define CPU3_LOG_DMA_RETRY_INTERVAL_MS 10U
/* 重复日志丢弃/拥塞通知的最小间隔 1000 ms。 */
#define CPU3_LOG_NOTICE_INTERVAL_MS 1000U

/* CPU3 异步日志字节环形队列，生产者只入队，UART4 DMA 完成回调推进发送。 */
static uint8_t s_log_tx_queue[CPU3_LOG_TX_QUEUE_BYTES];
/* 日志环形队列写入索引；入队临界区内更新。 */
static volatile uint16_t s_log_tx_head = 0U;
/* 日志环形队列发送索引；DMA 完成后推进。 */
static volatile uint16_t s_log_tx_tail = 0U;
/* 日志环形队列当前待发送字节数。 */
static volatile uint16_t s_log_tx_length = 0U;
/* UART4 日志 DMA 正在发送的标志，防止重复启动同一外设。 */
static volatile uint8_t s_log_tx_busy = 0U;
/* 异步日志传输已经完成初始化的标志。 */
static volatile uint8_t s_log_async_ready = 0U;
/* UART4 日志发送启动或完成异常的累计次数。 */
static volatile uint32_t s_log_transport_failure_count = 0U;
/* 日志发送失败后允许再次启动 DMA 的最早 HAL 毫秒节拍。 */
static volatile uint32_t s_log_next_retry_tick = 0U;
/* 因环形队列空间不足而整条丢弃的日志记录累计数。 */
static uint32_t s_log_dropped_record_count = 0U;
/* 因队列空间不足而丢弃的日志字节累计数。 */
static uint32_t s_log_dropped_byte_count = 0U;
/* 最近一次输出日志丢弃汇总提示的 HAL 毫秒节拍。 */
static uint32_t s_log_last_notice_tick = 0U;

/**
 * @brief 返回日志级别的现场可读名称。
 *
 * @param level 待判断或显示的级别值。该 CPU3 日志级别决定消息是否被编译期或运行期门限过滤，并映射为固定级别文字。
 * @return 返回日志级别的现场可读名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
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

/**
 * @brief 返回当前队列可写字节数；尾指针只会在发送完成中断中前进，旧快照只会低估空间。
 *
 * @return 返回当前队列可写字节数；尾指针只会在发送完成中断中前进，旧快照只会低估空间的有效长度，单位字节；0 表示没有可供消费的数据。
 */
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

/**
 * @brief 把一段完整日志写入单生产者、单消费者环形队列。
 *
 * @details 调用场景：主循环中的统一日志和普通printf重定向。
 * @note 关键约束：队列不足时只累计丢弃量，绝不等待；INFO/DEBUG为WARN/CRITICAL保留空间。
 *
 * @param level 待判断或显示的级别值。该 CPU3 日志级别决定消息是否被编译期或运行期门限过滤，并映射为固定级别文字。
 * @param data 待写入 CPU3 调试日志队列的原始字节序列；有效范围为 data[0..length-1]，日志函数不修改调用方数据。
 * @param length 输入数据的有效长度，单位字节。
 * @param count_as_record true 表示本段字节同时计入一条完整日志记录，false 表示只追加字节。
 * @return true 表示整段日志已原子写入环形队列；false 表示 data 为空、长度为 0、长度超过队列容量，或当前剩余空间不足以容纳整段日志。
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

/**
 * @brief 在当前上下文启动下一段连续DMA发送；调用方必须保证不会与发送完成回调并发。
 */
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

/**
 * @brief 返回当前生效日志级别的现场可读名称。
 *
 * @return 返回当前生效日志级别的现场可读名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *Cpu3Log_ActiveLevelText(void)
{
    return Cpu3Log_LevelText((Cpu3LogLevel)CPU3_LOG_ACTIVE_LEVEL);
}

/**
 * @brief 清空日志环形队列和统计，并在退出临界区后启用异步输出。
 */
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

/**
 * @brief 判断异步日志发送所需的串口和 DMA 是否就绪。
 *
 * @return true 表示异步日志串口和 DMA 已完成初始化，可接收发送请求；false 表示初始化尚未完成，异步日志发送仍需等待或降级处理。
 */
bool Cpu3Log_IsAsyncReady(void)
{
    return s_log_async_ready != 0U;
}

/**
 * @brief 将原始字节序列直接写入 CPU3 异步日志队列。
 *
 * @param data 待写入 CPU3 调试日志队列的原始字节序列；有效范围为 data[0..length-1]，日志函数不修改调用方数据。
 * @param length 输入数据的有效长度，单位字节。
 */
void Cpu3Log_WriteRaw(const uint8_t *data, uint16_t length)
{
    (void)Cpu3Log_QueueBytes(CPU3_LOG_LEVEL_INFO, data, length, false);
}

/**
 * @brief 按时间戳、级别和模块格式化单行日志，超长截断后非阻塞入队。
 *
 * @param level 待判断或显示的级别值。该 CPU3 日志级别决定消息是否被编译期或运行期门限过滤，并映射为固定级别文字。
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签写入 CPU3 日志前缀，空指针时使用未分类占位。
 * @param format 日志正文的 printf 风格格式字符串。
 */
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

/**
 * @brief 将收发帧限长格式化为十六进制日志，并区分空帧和空指针。
 *
 * @param level 待判断或显示的级别值。该 CPU3 日志级别决定消息是否被编译期或运行期门限过滤，并映射为固定级别文字。
 * @param module 用于日志分类的只读模块名称。该 NUL 结尾标签写入 CPU3 日志前缀，空指针时使用未分类占位。
 * @param direction 通信帧方向的 NUL 结尾只读标签，例如 RX 或 TX；仅用于日志前缀，不参与协议帧解析。
 * @param data 待写入 CPU3 调试日志队列的原始字节序列；有效范围为 data[0..length-1]，日志函数不修改调用方数据。
 * @param length 输入数据的有效长度，单位字节。
 */
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

/**
 * @brief 在主循环限流汇总日志丢弃和 DMA 失败，并启动下一段 UART1 DMA。
 */
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

/**
 * @brief 在串口发送完成中断中结束当前日志帧并启动后续发送。
 *
 * @note 关键约束：在中断或回调上下文中只更新必要状态，避免阻塞和高耗时操作。
 */
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

/**
 * @brief 在串口发送错误中断中终止当前日志帧并记录失败。
 *
 * @note 关键约束：在中断或回调上下文中只更新必要状态，避免阻塞和高耗时操作。
 */
void Cpu3Log_TxErrorFromISR(void)
{
    s_log_tx_length = 0U;
    s_log_tx_busy = 0U;
    s_log_transport_failure_count++;
    s_log_next_retry_tick = HAL_GetTick() + CPU3_LOG_DMA_RETRY_INTERVAL_MS;
}
