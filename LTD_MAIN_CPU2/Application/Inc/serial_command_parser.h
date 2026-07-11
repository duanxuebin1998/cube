#ifndef INC_SERIAL_COMMAND_PARSER_H_
#define INC_SERIAL_COMMAND_PARSER_H_

#include <stddef.h>
#include <stdint.h>

#define SERIAL_COMMAND_RX_CAPACITY 64U

typedef enum {
    SERIAL_COMMAND_KIND_UNSUPPORTED = 0,
    SERIAL_COMMAND_KIND_INVALID,
    SERIAL_COMMAND_KIND_FORMAL,
    SERIAL_COMMAND_KIND_TEST,
    SERIAL_COMMAND_KIND_STOP,
    SERIAL_COMMAND_KIND_HELP,
    SERIAL_COMMAND_KIND_VERSION,
    SERIAL_COMMAND_KIND_STATUS,
    SERIAL_COMMAND_KIND_ERROR
} SerialCommandKind;

typedef struct {
    SerialCommandKind kind;
    uint8_t formal_command;
} SerialCommandParseResult;

typedef enum {
    SERIAL_COMMAND_RX_NONE = 0,
    SERIAL_COMMAND_RX_READY,
    SERIAL_COMMAND_RX_TOO_LONG
} SerialCommandRxEvent;

typedef struct {
    uint8_t buffer[SERIAL_COMMAND_RX_CAPACITY];
    size_t length;
    uint8_t discarding;
} SerialCommandRxState;

/*
 * 函数用途：严格解析一条完整的 CPU2 调试串口命令。
 * 调用场景：主循环收到完整文本帧后，在执行任何电机或测量动作前调用。
 * 关键约束：命令区分大小写，不忽略首尾空白，不接受数值后的尾随字符。
 */
SerialCommandParseResult SerialCommandParser_Parse(const uint8_t *command);

/*
 * 函数用途：初始化调试串口逐字节收帧状态。
 * 调用场景：首次接收或测试用例复位接收器时调用。
 * 关键约束：只清理内存状态，不访问硬件、不打印、不阻塞。
 */
void SerialCommandRx_Init(SerialCommandRxState *state);

/*
 * 函数用途：向调试串口收帧器压入一个字节并报告完整帧或超长帧事件。
 * 调用场景：USART1 中断逐字节转交 DMA 数据时调用。
 * 关键约束：中断安全，只做定长内存操作；超长后丢弃整帧直到换行。
 */
SerialCommandRxEvent SerialCommandRx_PushByte(SerialCommandRxState *state, uint8_t byte);

#endif /* INC_SERIAL_COMMAND_PARSER_H_ */
