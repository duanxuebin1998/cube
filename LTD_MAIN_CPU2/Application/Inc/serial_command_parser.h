#ifndef INC_SERIAL_COMMAND_PARSER_H_
/* INC_SERIAL_COMMAND_PARSER_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_SERIAL_COMMAND_PARSER_H_

#include <stddef.h>
#include <stdint.h>

/* 串口文本命令接收缓冲容量 64 字节；解析器必须为终止符预留空间并拒绝超长命令。 */
#define SERIAL_COMMAND_RX_CAPACITY 64U

/* 调试串口命令解析分类；先区分格式错误、正式命令和各类文本查询，再由上层执行对应入口。 */
typedef enum {
    /* 调试串口完整命令行的解析分类。 */
    SERIAL_COMMAND_KIND_UNSUPPORTED = 0, /* 命令语法可识别，但当前固件不支持该命令。 */
    SERIAL_COMMAND_KIND_INVALID, /* 命令为空、格式错误或参数非法。 */
    SERIAL_COMMAND_KIND_FORMAL, /* 命令是正式设备命令码形式。 */
    SERIAL_COMMAND_KIND_TEST, /* 命令进入测试和调试命令处理器。 */
    SERIAL_COMMAND_KIND_STOP, /* 命令要求立即停止当前可中止流程。 */
    SERIAL_COMMAND_KIND_HELP, /* 命令要求打印帮助和可用命令列表。 */
    SERIAL_COMMAND_KIND_VERSION, /* 命令要求打印固件和协议版本。 */
    SERIAL_COMMAND_KIND_STATUS, /* 命令要求打印当前设备运行状态。 */
    SERIAL_COMMAND_KIND_ERROR, /* 命令要求打印当前故障和诊断状态。 */
    SERIAL_COMMAND_KIND_POWER_STATUS,   /* PWR?：只读24V监控RAM快照。 */
    SERIAL_COMMAND_KIND_ENCODER_STATUS, /* ENC?：只读编码器持久化RAM快照。 */
    SERIAL_COMMAND_KIND_POWER_TEST      /* PWRTEST：投递软件紧急保存，不伪造低压。 */
} SerialCommandKind;

typedef struct {
    /* 调试串口命令分类结果及正式二进制命令码。 */
    SerialCommandKind kind; /* 命令行解析分类，决定上层是否读取 formal_command 或执行文本查询。 */
    uint8_t formal_command; /* 正式设备命令的二进制命令码；仅在 kind 为 FORMAL 时有效。 */
} SerialCommandParseResult;

typedef enum {
    /* 调试串口逐字节接收器向上层报告的事件。 */
    SERIAL_COMMAND_RX_NONE = 0, /* 本次输入字节尚未形成完整命令事件。 */
    SERIAL_COMMAND_RX_READY, /* 已接收到行结束符，缓冲区内有一条完整命令。 */
    SERIAL_COMMAND_RX_TOO_LONG /* 命令超过缓冲容量，已进入丢弃直到行结束的状态。 */
} SerialCommandRxEvent;

typedef struct {
    /* 调试串口逐字节组帧缓冲区、当前长度和超长丢弃状态。 */
    uint8_t buffer[SERIAL_COMMAND_RX_CAPACITY]; /* 调试串口命令行缓冲区，写入长度不得超过 SERIAL_COMMAND_RX_CAPACITY。 */
    size_t length; /* 当前命令行缓冲区内的有效字节数。 */
    uint8_t discarding; /* 接收器正丢弃超长命令剩余字节的标志；遇到行结束符后清除。 */
} SerialCommandRxState;

/**
 * @brief 严格解析一条完整的 CPU2 调试串口命令。
 *
 * @details 调用场景：主循环收到完整文本帧后，在执行任何电机或测量动作前调用。
 * @note 关键约束：命令区分大小写，不忽略首尾空白，不接受数值后的尾随字符。
 */
SerialCommandParseResult SerialCommandParser_Parse(const uint8_t *command);

/**
 * @brief 初始化调试串口逐字节收帧状态。
 *
 * @details 调用场景：首次接收或测试用例复位接收器时调用。
 * @note 关键约束：只清理内存状态，不访问硬件、不打印、不阻塞。
 */
void SerialCommandRx_Init(SerialCommandRxState *state);

/**
 * @brief 向调试串口收帧器压入一个字节并报告完整帧或超长帧事件。
 *
 * @details 调用场景：USART1 中断逐字节转交 DMA 数据时调用。
 * @note 关键约束：中断安全，只做定长内存操作；超长后丢弃整帧直到换行。
 */
SerialCommandRxEvent SerialCommandRx_PushByte(SerialCommandRxState *state, uint8_t byte);

#endif /* INC_SERIAL_COMMAND_PARSER_H_ */
