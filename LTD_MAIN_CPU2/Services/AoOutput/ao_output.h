#ifndef AO_OUTPUT_H_
#define AO_OUTPUT_H_

#include <stdint.h>

typedef enum {
    AO_OUTPUT_SOURCE_INIT = 0U,
    AO_OUTPUT_SOURCE_LEVEL,
    AO_OUTPUT_SOURCE_ALARM_HIGH,
    AO_OUTPUT_SOURCE_ALARM_LOW,
    AO_OUTPUT_SOURCE_FAULT,
    AO_OUTPUT_SOURCE_DEBUG,
    AO_OUTPUT_SOURCE_DISABLED,
    AO_OUTPUT_SOURCE_DRIVER_ERROR
} AoOutputSource;

typedef struct {
    uint32_t target_mA_x100;
    uint32_t last_sent_mA_x100;
    uint32_t source;
    uint32_t driver_fault_flags;
    uint32_t driver_fault_register;
    uint32_t last_error_code;
    uint32_t update_counter;
    uint32_t last_update_tick;
    uint32_t last_sent_tick;
} AoOutputRuntime;

/*
 * 函数用途：初始化 AO 输出服务并发布初始运行态。
 * 调用场景：系统参数加载后由主流程调用。
 * 关键约束：未使能时不访问 AD5421；使能时会访问 SPI 和诊断寄存器。
 */
uint32_t AoOutput_Init(void);

/*
 * 函数用途：刷新 AO 目标电流、驱动诊断和输出写入。
 * 调用场景：主循环或测量流程周期调用。
 * 关键约束：会访问 SPI/GPIO，不应在中断中调用。
 */
uint32_t AoOutput_Update(void);

/*
 * 函数用途：由 TIM4 中断请求一次 AO 延后刷新。
 * 调用场景：TIM4_IRQHandler 在继电器刷新后调用。
 * 关键约束：只置位请求并挂起 PendSV，不访问 AD5421、不打印、不阻塞。
 */
void AoOutput_RequestTimerRefreshFromTim4Isr(void);

/*
 * 函数用途：处理 TIM4 请求的 AO 延后刷新。
 * 调用场景：PendSV_Handler 最低优先级调用。
 * 关键约束：会访问 AD5421 SPI；驱动内部打印被抑制，错误延后由系统统一兜底。
 */
uint32_t AoOutput_ProcessPendingTimerRefresh(void);

/*
 * 函数用途：暂停或恢复定时触发的 AO 自动刷新。
 * 调用场景：串口 AO 测试直接访问 AD5421 期间使用。
 * 关键约束：只影响 TIM4 请求和 PendSV 延后刷新，不影响继电器和前台 AO 调用。
 */
void AoOutput_SuspendTimerRefresh(void);
void AoOutput_ResumeTimerRefresh(void);
/*
 * 函数用途：返回 AO 运行态只读指针。
 * 调用场景：Modbus 输入寄存器打包和调试查看。
 * 关键约束：调用方不得修改返回的运行态数据。
 */
const AoOutputRuntime *AoOutput_GetRuntime(void);

/*
 * 函数用途：返回当前 AO 目标电流 mA 值。
 * 调用场景：HART 电流响应或调试查看。
 * 关键约束：只读运行态，不刷新硬件输出。
 */
float AoOutput_GetCurrent_mA(void);

/*
 * 函数用途：返回当前 AO 目标电流在 4-20mA 量程内的比例。
 * 调用场景：HART Command 2/3 百分比响应。
 * 关键约束：只读运行态，返回值已钳位到 0..1。
 */
float AoOutput_GetPercentOfRange(void);

/*
 * 函数用途：兼容旧 AD5421 电流决策入口。
 * 调用场景：旧调用链仍调用该函数时转入 AO 更新流程。
 * 关键约束：内部会进入 AO 更新流程，不应在中断中调用。
 */
void CurrentStateJudgeAndSend(void);

#endif /* AO_OUTPUT_H_ */
