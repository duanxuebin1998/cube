#ifndef AO_OUTPUT_H_
#define AO_OUTPUT_H_

#include <stdint.h>
#include "system_parameter.h"

typedef enum {
    AO_OUTPUT_SOURCE_INITIAL = 0U,
    AO_OUTPUT_SOURCE_PROCESS = 1U,
    AO_OUTPUT_SOURCE_FAULT = 2U,
    AO_OUTPUT_SOURCE_SIMULATION = 3U,
    AO_OUTPUT_SOURCE_FIXED = 4U,
    AO_OUTPUT_SOURCE_DISABLED = 5U,
    AO_OUTPUT_SOURCE_DRIVER_ERROR = 6U,
    AO_OUTPUT_SOURCE_HOLD_LAST = 7U
} AoOutputSource;

typedef struct {
    int32_t value_01mm;
    uint32_t update_tick;
    uint32_t update_counter;
    uint8_t valid;
} AoProcessSample;

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
    int32_t process_value_01mm;
    int32_t percent_x100;
    uint32_t process_valid;
    uint32_t simulation_enabled;
    uint32_t dac_readback_mA_x100;
    uint32_t dac_readback_valid;
} AoOutputRuntime;

/* 初始化AO服务并写入禁用、固定或初始电流。 */
uint32_t AoOutput_Init(void);

/* 在任务态刷新AO状态机和AD5421输出。 */
uint32_t AoOutput_Update(void);

/* 发布或失效指定过程量；函数只更新内存快照，不访问外设。 */
void AoOutput_PublishProcessSample(AoProcessSource source, int32_t value_01mm, uint8_t valid);
void AoOutput_InvalidateProcessSample(AoProcessSource source);
uint32_t AoOutput_ReadProcessSample(AoProcessSource source, AoProcessSample *sample);
uint32_t AoOutput_GetSelectedProcessSample(AoProcessSample *sample);

/* 设置或读取AO仿真运行态；仿真开关不持久化。 */
void AoOutput_SetSimulationEnabled(uint32_t enabled);
uint32_t AoOutput_IsSimulationEnabled(void);

/* 复制一致的AO运行态快照，供Modbus和HART跨上下文读取。 */
void AoOutput_GetRuntimeSnapshot(AoOutputRuntime *runtime);
const AoOutputRuntime *AoOutput_GetRuntime(void);

/* HART使用最后成功下发电流和真实过程百分数。 */
float AoOutput_GetCurrent_mA(void);
float AoOutput_GetPercentOfRange(void);

/* TIM4中断只请求刷新，PendSV执行实际SPI访问。 */
void AoOutput_RequestTimerRefreshFromTim4Isr(void);
uint32_t AoOutput_ProcessPendingTimerRefresh(void);
void AoOutput_ProcessDeferredDiagnostics(void);
void AoOutput_SuspendTimerRefresh(void);
void AoOutput_ResumeTimerRefresh(void);

#endif /* AO_OUTPUT_H_ */
