#ifndef CPU3_UART_IDLE_FRAME_GATE_H_
#define CPU3_UART_IDLE_FRAME_GATE_H_

#include <stdint.h>

typedef enum
{
    CPU3_UART_RX_IRQ_NONE = 0U,
    CPU3_UART_RX_IRQ_IDLE_FRAME,
    CPU3_UART_RX_IRQ_ERROR
} Cpu3UartRxIrqAction;

/*
 * 函数用途：按同一次 UART 状态快照决定错误恢复或 IDLE 成帧。
 * 调用场景：UART DMA 接收中断在读取数据长度和清除 IDLE 之前调用。
 * 关键约束：任一接收错误必须高于 IDLE，组合事件不得进入协议解析。
 */
static inline Cpu3UartRxIrqAction CPU3_UartClassifyRxIrq(uint32_t status,
                                                         uint32_t idle_mask,
                                                         uint32_t error_mask)
{
    if ((status & error_mask) != 0U)
    {
        return CPU3_UART_RX_IRQ_ERROR;
    }

    if ((status & idle_mask) != 0U)
    {
        return CPU3_UART_RX_IRQ_IDLE_FRAME;
    }

    return CPU3_UART_RX_IRQ_NONE;
}

#endif /* CPU3_UART_IDLE_FRAME_GATE_H_ */
