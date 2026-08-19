/*
 * 模块职责：为UART6的V4主动接收、V4交互事务、Safe事务和CH9141 AT操作提供互斥所有权。
 * 并发约束：接口只修改短小的内存状态，临界区内不得启动DMA、等待外设或打印日志。
 * 嵌套约束：同一所有者允许计数式重入，不同所有者必须等待现有所有者完整释放。
 */
#include "sensor_uart6_owner.h"

#include "main.h"

/* 当前UART6软件所有者；只在短临界区内读取或更新。 */
static volatile SensorUart6Owner s_uart6_owner = SENSOR_UART6_OWNER_NONE;
/* 同一所有者的嵌套申请深度；归零时UART6才真正释放。 */
static volatile uint16_t s_uart6_owner_depth = 0U;

/*
 * 函数用途：以不可抢占方式申请一次UART6所有权。
 * 调用场景：V4主动流、V4交互、Safe、DSM、V3或CH9141开始操作UART6之前。
 * 关键约束：临界区内只改内存；同一所有者可嵌套，不同所有者冲突立即失败且不阻塞。
 */
uint8_t SensorUart6Owner_Acquire(SensorUart6Owner owner)
{
    uint32_t primask;
    uint8_t acquired = 0U;

    if (owner == SENSOR_UART6_OWNER_NONE) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_uart6_owner == SENSOR_UART6_OWNER_NONE) {
        s_uart6_owner = owner;
        s_uart6_owner_depth = 1U;
        acquired = 1U;
    } else if ((s_uart6_owner == owner) && (s_uart6_owner_depth < UINT16_MAX)) {
        /* 同一协议栈内部的嵌套调用共享所有权，但每次Acquire都必须对应一次Release。 */
        s_uart6_owner_depth++;
        acquired = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return acquired;
}

/*
 * 函数用途：释放一次与申请配对的UART6所有权。
 * 调用场景：对应协议完成DMA停止、事务收尾或AT模式退出之后。
 * 关键约束：所有者不匹配时不改变状态；嵌套深度归零后才真正释放。
 */
uint8_t SensorUart6Owner_Release(SensorUart6Owner owner)
{
    uint32_t primask;
    uint8_t released = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if ((owner != SENSOR_UART6_OWNER_NONE) &&
        (s_uart6_owner == owner) &&
        (s_uart6_owner_depth > 0U)) {
        s_uart6_owner_depth--;
        if (s_uart6_owner_depth == 0U) {
            /* 先清深度再释放所有者，使观察者不会看到“无所有者但仍有嵌套层”的状态。 */
            s_uart6_owner = SENSOR_UART6_OWNER_NONE;
        }
        released = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return released;
}

/*
 * 函数用途：把UART6从一个协议所有者原子交接给另一个所有者。
 * 调用场景：V4主动接收与交互窗口需要无空档切换时。
 * 关键约束：仅允许最外层深度为1时切换；函数不停止DMA、不等待硬件、不打印。
 */
uint8_t SensorUart6Owner_Transition(SensorUart6Owner current_owner,
                                    SensorUart6Owner next_owner)
{
    uint32_t primask;
    uint8_t transitioned = 0U;

    if ((current_owner == SENSOR_UART6_OWNER_NONE) ||
        (next_owner == SENSOR_UART6_OWNER_NONE)) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    /* 仅最外层持有者可以原子交接，存在嵌套调用时切换会破坏配对释放。 */
    if ((s_uart6_owner == current_owner) && (s_uart6_owner_depth == 1U)) {
        s_uart6_owner = next_owner;
        transitioned = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return transitioned;
}

/*
 * 函数用途：读取UART6当前所有者。
 * 调用场景：协议状态检查和诊断输出。
 * 关键约束：返回瞬时快照，调用方不得据此绕过Acquire直接操作UART6。
 */
SensorUart6Owner SensorUart6Owner_Get(void)
{
    return s_uart6_owner;
}

/*
 * 函数用途：判断UART6当前是否由指定协议持有。
 * 调用场景：DMA回调过滤、停止收尾和协议模式检查。
 * 关键约束：只比较所有者，不验证DMA状态或嵌套深度。
 */
uint8_t SensorUart6Owner_Is(SensorUart6Owner owner)
{
    return (s_uart6_owner == owner) ? 1U : 0U;
}
