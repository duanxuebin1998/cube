#ifndef CPU3_DEBUG_LOG_H_
#define CPU3_DEBUG_LOG_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CPU3_LOG_LEVEL_OFF = 0,
    CPU3_LOG_LEVEL_CRITICAL = 1,
    CPU3_LOG_LEVEL_WARNING = 2,
    CPU3_LOG_LEVEL_INFO = 3,
    CPU3_LOG_LEVEL_DEBUG = 4
} Cpu3LogLevel;

/*
 * 默认只输出启动、状态变化、异常和周期摘要。
 * 现场需要查看完整收发帧时临时改为 CPU3_LOG_LEVEL_DEBUG，联调结束后恢复 INFO。
 * 统一日志通过USART1 TX DMA异步输出，队列压力不会阻塞CPU2或外部COM通信。
 */
#ifndef CPU3_LOG_ACTIVE_LEVEL
#define CPU3_LOG_ACTIVE_LEVEL CPU3_LOG_LEVEL_INFO
#endif

/* 单帧最多打印的字节数，超出部分只报告剩余长度。 */
#ifndef CPU3_LOG_FRAME_MAX_BYTES
#define CPU3_LOG_FRAME_MAX_BYTES 64U
#endif

/*
 * 函数用途：初始化CPU3非阻塞日志队列。
 * 调用场景：USART1初始化完成后、首次统一日志之前调用一次。
 * 关键约束：只初始化日志传输状态，不修改任何业务串口或协议状态。
 */
void Cpu3Log_Init(void);

/*
 * 函数用途：推进USART1日志DMA并延后报告队列丢弃情况。
 * 调用场景：CPU3主循环完成业务通信调度后每轮调用。
 * 关键约束：不得在ISR中调用；队列或DMA忙时立即返回，不执行阻塞等待。
 */
void Cpu3Log_Service(void);

/*
 * 函数用途：按统一格式把一条CPU3日志放入非阻塞队列。
 * 调用场景：初始化、通信状态变化、周期摘要和业务异常。
 * 关键约束：禁止在ISR中调用；队列不足时累计丢弃量，不影响业务返回值。
 */
void Cpu3Log_Print(Cpu3LogLevel level, const char *module, const char *format, ...);

/*
 * 函数用途：把一帧十六进制通信数据放入非阻塞日志队列。
 * 调用场景：DEBUG级短时联调，或WARN级输出失败帧证据。
 * 关键约束：禁止在ISR中调用；长帧按CPU3_LOG_FRAME_MAX_BYTES截断。
 */
void Cpu3Log_Frame(Cpu3LogLevel level,
                   const char *module,
                   const char *direction,
                   const uint8_t *data,
                   uint16_t length);

/* 返回当前编译启用的日志级别名称。 */
const char *Cpu3Log_ActiveLevelText(void);

/* 普通printf重定向使用的非阻塞原始字节入口。 */
void Cpu3Log_WriteRaw(const uint8_t *data, uint16_t length);

/* 判断统一日志异步后端是否已经初始化。 */
bool Cpu3Log_IsAsyncReady(void);

/* USART1 DMA发送完成中断只推进队列，不格式化或打印。 */
void Cpu3Log_TxCompleteFromISR(void);

/* USART1异常中断只释放日志发送状态，主循环稍后重试并报告。 */
void Cpu3Log_TxErrorFromISR(void);

#define CPU3_LOG_CRITICAL(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_CRITICAL, module, __VA_ARGS__)
#define CPU3_LOG_WARNING(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_WARNING, module, __VA_ARGS__)
#define CPU3_LOG_INFO(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_INFO, module, __VA_ARGS__)
#define CPU3_LOG_DEBUG(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_DEBUG, module, __VA_ARGS__)

#endif /* CPU3_DEBUG_LOG_H_ */
