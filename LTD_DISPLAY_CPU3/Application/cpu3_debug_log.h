#ifndef CPU3_DEBUG_LOG_H_
/* CPU3_DEBUG_LOG_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define CPU3_DEBUG_LOG_H_

#include <stdbool.h>
#include <stdint.h>

/* CPU3 异步日志严重度等级；运行时阈值按数值从“关闭”到“调试”逐级放宽，用于控制哪些日志可以进入非阻塞发送队列。 */
typedef enum {
    /* CPU3 日志过滤等级，数值越大允许输出的日志越详细。 */
    CPU3_LOG_LEVEL_OFF = 0, /* 关闭 CPU3 运行日志输出。 */
    CPU3_LOG_LEVEL_CRITICAL = 1, /* 仅输出会影响核心功能或恢复能力的严重错误。 */
    CPU3_LOG_LEVEL_WARNING = 2, /* 输出严重错误及可恢复的异常告警。 */
    CPU3_LOG_LEVEL_INFO = 3, /* 额外输出关键状态变化和正常业务事件。 */
    CPU3_LOG_LEVEL_DEBUG = 4 /* 输出最详细的调试信息，仅用于问题定位。 */
} Cpu3LogLevel;

/*
 * 默认只输出启动、状态变化、异常和周期摘要。
 * 现场需要查看完整收发帧时临时改为 CPU3_LOG_LEVEL_DEBUG，联调结束后恢复 INFO。
 * 统一日志通过USART1 TX DMA异步输出，队列压力不会阻塞CPU2或外部COM通信。
 */
#ifndef CPU3_LOG_ACTIVE_LEVEL
/* CPU3 编译期启用的最低日志等级，当前为 INFO；DEBUG 调用在更高阈值构建中不产生发送负载。 */
#define CPU3_LOG_ACTIVE_LEVEL CPU3_LOG_LEVEL_INFO
#endif

/* 单帧最多打印的字节数，超出部分只报告剩余长度。 */
#ifndef CPU3_LOG_FRAME_MAX_BYTES
/* 日志 DMA 单次发送分片的最大字节数 64；长日志由队列服务分帧发送，不能假设一次 DMA 完成整行。 */
#define CPU3_LOG_FRAME_MAX_BYTES 64U
#endif

/**
 * @brief 初始化CPU3非阻塞日志队列。
 *
 * @details 调用场景：USART1初始化完成后、首次统一日志之前调用一次。
 * @note 关键约束：只初始化日志传输状态，不修改任何业务串口或协议状态。
 */
void Cpu3Log_Init(void);

/**
 * @brief 推进USART1日志DMA并延后报告队列丢弃情况。
 *
 * @details 调用场景：CPU3主循环完成业务通信调度后每轮调用。
 * @note 关键约束：不得在ISR中调用；队列或DMA忙时立即返回，不执行阻塞等待。
 */
void Cpu3Log_Service(void);

/**
 * @brief 按时间戳、级别和模块格式化单行日志，超长截断后写入非阻塞队列。
 *
 * 初始化、通信状态变化、周期摘要和业务异常均通过该入口统一输出；队列不足时只累计丢弃量，不改变调用方业务返回值。
 *
 * @param level CPU3 日志级别；用于执行编译期和运行期门限过滤，并映射固定级别文字。
 * @param module NUL 结尾的只读模块名称；用于生成日志前缀，空指针时使用未分类占位。
 * @param format printf 风格的日志正文格式字符串；其后的可变参数必须与格式占位符保持一致。
 * @note 禁止在 ISR 中调用；中断只记录轻量事件，再由主循环调用本接口格式化输出。
 */
void Cpu3Log_Print(Cpu3LogLevel level, const char *module, const char *format, ...);

/**
 * @brief 把一帧十六进制通信数据放入非阻塞日志队列。
 *
 * @details 调用场景：DEBUG级短时联调，或WARN级输出失败帧证据。
 * @note 关键约束：禁止在ISR中调用；长帧按CPU3_LOG_FRAME_MAX_BYTES截断。
 */
void Cpu3Log_Frame(Cpu3LogLevel level,
                   const char *module,
                   const char *direction,
                   const uint8_t *data,
                   uint16_t length);

/**
 * @brief 返回当前生效日志级别的现场可读名称。
 *
 * 返回当前编译启用的日志级别名称。
 *
 * @return 返回当前生效日志级别的现场可读名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *Cpu3Log_ActiveLevelText(void);

/**
 * @brief 将原始字节序列直接写入 CPU3 异步日志队列。
 *
 * 普通printf重定向使用的非阻塞原始字节入口。
 *
 * @param data 待写入 CPU3 调试日志队列的原始字节序列；有效范围为 data[0..length-1]，日志函数不修改调用方数据。
 * @param length 输入数据的有效长度，单位字节。
 */
void Cpu3Log_WriteRaw(const uint8_t *data, uint16_t length);

/**
 * @brief 判断异步日志发送所需的串口和 DMA 是否就绪。
 *
 * 判断统一日志异步后端是否已经初始化。
 *
 * @return true 表示异步日志串口和 DMA 已完成初始化，可接收发送请求；false 表示初始化尚未完成，异步日志发送仍需等待或降级处理。
 */
bool Cpu3Log_IsAsyncReady(void);

/**
 * @brief 在串口发送完成中断中结束当前日志帧并启动后续发送。
 *
 * USART1 DMA发送完成中断只推进队列，不格式化或打印。
 *
 * @note 关键约束：在中断或回调上下文中只更新必要状态，避免阻塞和高耗时操作。
 */
void Cpu3Log_TxCompleteFromISR(void);

/**
 * @brief 在串口发送错误中断中终止当前日志帧并记录失败。
 *
 * USART1异常中断只释放日志发送状态，主循环稍后重试并报告。
 *
 * @note 关键约束：在中断或回调上下文中只更新必要状态，避免阻塞和高耗时操作。
 */
void Cpu3Log_TxErrorFromISR(void);

/* CRITICAL 级日志入口宏；自动携带模块名并转发可变参数，用于必须优先保留的致命或不可继续故障。 */
#define CPU3_LOG_CRITICAL(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_CRITICAL, module, __VA_ARGS__)
/* WARNING 级日志入口宏；用于可恢复异常或需要现场关注的降级状态。 */
#define CPU3_LOG_WARNING(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_WARNING, module, __VA_ARGS__)
/* INFO 级日志入口宏；用于关键状态迁移和正常业务里程碑。 */
#define CPU3_LOG_INFO(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_INFO, module, __VA_ARGS__)
/* DEBUG 级日志入口宏；仅在活动等级允许时输出详细诊断，量产默认可被编译期过滤。 */
#define CPU3_LOG_DEBUG(module, ...) \
    Cpu3Log_Print(CPU3_LOG_LEVEL_DEBUG, module, __VA_ARGS__)

#endif /* CPU3_DEBUG_LOG_H_ */
