/*
 * serial_command.h
 *
 * 文件职责：声明串口命令接收、线程态处理和延后报告接口。
 */

#ifndef INC_SERIAL_COMMAND_H_
/* INC_SERIAL_COMMAND_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_SERIAL_COMMAND_H_

#include <stdint.h>

#ifndef CPU2_USART1_COMMAND_RX_ENABLED
#define CPU2_USART1_COMMAND_RX_ENABLED 1U
#endif

/**
 * @brief 在 USART1 中断中记录一个调试串口接收字节。
 *
 * @details 调用场景：USART1 IDLE 中断遍历 DMA 接收块时调用。
 * @note 关键约束：中断内不打印、不阻塞、不执行命令，只记录完整帧状态。
 */
void SerialCommand_RxByteFromIsr(uint8_t byte);

/**
 * @brief 复制并处理一条已就绪的串口帧。
 *
 * @details 调用场景：主循环确认 new_command_ready 后调用。
 * @note 关键约束：先复制完整帧再开放下一帧接收，避免执行期间缓冲区被覆盖。
 */
void SerialCommand_ProcessReady(void);

/**
 * @brief 严格解析并分发 CPU2 调试串口命令。
 *
 * @details 调用场景：主循环取得完整且未超长的命令后调用。
 * @note 关键约束：查询命令只读；正式业务命令只挂到主循环待执行命令。
 */
void SerialCommand_Process(const uint8_t *command);

/**
 * @brief 在线程态输出中断或PendSV延后提交的调试快照。
 *
 * @details 调用场景：App_MainLoop每轮后台服务入口。
 * @note 关键约束：只消费已完成快照；不得从ISR或PendSV调用。
 */
void SerialCommand_ProcessDeferredReports(void);

/**
 * @brief 在主循环报告一条超长串口命令。
 *
 * @details 调用场景：SerialCommand_ProcessReady 检测到超长帧后在主循环调用。
 * @note 关键约束：不得在 USART1 中断内调用。
 */
void SerialCommand_ReportTooLong(void);

#endif /* INC_SERIAL_COMMAND_H_ */
