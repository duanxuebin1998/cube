#ifndef AO_OUTPUT_H_
/* AO_OUTPUT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define AO_OUTPUT_H_

#include <stdint.h>
#include "system_parameter.h"

/* AO 本次目标电流的来源分类；运行态通过该值区分过程量、故障、仿真、固定值和驱动异常等路径。 */
typedef enum {
    /* AO 当前目标电流的实际来源。 */
    AO_OUTPUT_SOURCE_INITIAL = 0U, /* 当前目标来自上电初始电流。 */
    AO_OUTPUT_SOURCE_PROCESS = 1U, /* 当前目标来自过程量换算电流。 */
    AO_OUTPUT_SOURCE_FAULT = 2U, /* 当前目标来自故障策略电流。 */
    AO_OUTPUT_SOURCE_SIMULATION = 3U, /* 当前目标来自调试仿真电流。 */
    AO_OUTPUT_SOURCE_FIXED = 4U, /* 当前目标来自固定电流模式。 */
    AO_OUTPUT_SOURCE_DISABLED = 5U, /* 当前目标来自输出关闭。 */
    AO_OUTPUT_SOURCE_DRIVER_ERROR = 6U, /* 当前目标来自驱动异常回退值。 */
    AO_OUTPUT_SOURCE_HOLD_LAST = 7U /* 当前目标来自最近一次有效电流。 */
} AoOutputSource;

typedef struct {
    /* 单个 AO 过程量来源的最新值、更新时间、代际和有效性。 */
    int32_t value_01mm; /* 0.1 mm 定点值，单位为 0.1 mm；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t update_tick; /* 该 AO 过程量样本最近一次发布的 HAL 毫秒节拍。 */
    uint32_t update_counter; /* 内容成功发布的更新代际；消费者可据此判断是否出现新样本。 */
    uint8_t valid; /* 该过程量来源已经发布过有效样本的标志。 */
} AoProcessSample;

typedef struct {
    /* AO 对外运行态快照；同时保存目标/实发电流、输入过程量、百分比、驱动诊断和更新时间。 */
    uint32_t target_mA_x1000; /* 目标输出电流千倍定点值，单位为 0.001 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t last_sent_mA_x1000; /* 最近实发电流千倍定点值，单位为 0.001 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t source; /* 当前目标电流的实际来源，取值遵循 AoOutputSource。 */
    uint32_t driver_fault_flags; /* AO 驱动最近一次诊断得到的归一化故障位集合。 */
    uint32_t driver_fault_register; /* AO 驱动最近一次故障寄存器原始值。 */
    uint32_t last_error_code; /* AO 更新流程最近一次统一故障码；成功后按实现约定清除。 */
    uint32_t update_counter; /* 内容成功发布的更新代际；消费者可据此判断是否出现新样本。 */
    uint32_t last_update_tick; /* AO 运行态最近一次完成整体更新的 HAL 毫秒节拍。 */
    uint32_t last_sent_tick; /* 最近一次成功向 AD5421 写入电流的 HAL 毫秒节拍。 */
    int32_t process_value_01mm; /* 过程量 0.1 mm 定点值，单位为 0.1 mm；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    int32_t percent_x100; /* 过程量在配置量程中的百分比百倍定点值，10000 表示 100.00%。 */
    uint32_t process_valid; /* 当前过程量已通过来源、量程和新鲜度校验的标志。 */
    uint32_t simulation_enabled; /* AO 仿真输出当前是否生效的运行态标志。 */
    uint32_t dac_readback_mA_x100; /* DAC 回读电流百倍定点值，单位为 0.01 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t dac_readback_valid; /* DAC 回读电流已由有效驱动诊断换算得到的标志。 */
} AoOutputRuntime;

/**
 * @brief 初始化AO并按当前配置写入禁用、固定或初始电流。
 *
 * @return NO_ERROR 表示初始化AO并按当前配置写入禁用、固定或初始电流已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t AoOutput_Init(void);

/**
 * @brief 带重入保护地执行一次任务态AO刷新。
 *
 * 在任务态刷新AO状态机和AD5421输出。
 *
 * @return 返回 AO 刷新结果码；可延后恢复的运行期驱动错误被记录后返回 NO_ERROR，其他失败原样传播。
 */
uint32_t AoOutput_Update(void);

/**
 * @brief 发布液位或水位过程量；传感器位置由AO读取权威运行值。
 *
 * 发布或失效指定过程量；函数只更新内存快照，不访问外设。
 *
 * @param source AO 过程量来源枚举；区分储罐液位、传感器位置和水位，用于选择并更新各自独立的过程样本槽。
 * @param value_01mm 位置或距离值，单位 0.1 mm。
 * @param valid 有效。
 */
void AoOutput_PublishProcessSample(AoProcessSource source, int32_t value_01mm, uint8_t valid);
/**
 * @brief 显式失效一个过程量样本。
 *
 * @param source AO 过程量来源枚举；区分储罐液位、传感器位置和水位，用于选择并更新各自独立的过程样本槽。
 */
void AoOutput_InvalidateProcessSample(AoProcessSource source);
/**
 * @brief 复制指定过程量快照，传感器位置直接读取当前权威位置。
 *
 * @details 调用场景：AO目标选择和运行态查询。
 * @note 关键约束：位置0.0mm允许有效；有效性由当前记步源是否就绪判定。
 *
 * @param source AO 过程量来源枚举；区分储罐液位、传感器位置和水位，用于选择并更新各自独立的过程样本槽。
 * @param sample 待处理的单次测量样本。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t AoOutput_ReadProcessSample(AoProcessSource source, AoProcessSample *sample);
/**
 * @brief 复制当前配置选中的过程量快照。
 *
 * @param sample 待处理的单次测量样本。
 * @return 返回整机错误码；NO_ERROR 表示已复制当前输出源的有效过程量快照，其他值表示来源或样本无效。
 */
uint32_t AoOutput_GetSelectedProcessSample(AoProcessSample *sample);

/**
 * @brief 设置仿真运行态；上电初始化会强制清零。
 *
 * 设置或读取AO仿真运行态；仿真开关不持久化。
 *
 * @param enabled 目标使能状态，非零表示启用，零表示禁用。
 */
void AoOutput_SetSimulationEnabled(uint32_t enabled);
/**
 * @brief 返回仿真运行态开关。
 *
 * @return 返回 AO 仿真运行态开关；1 表示使用仿真输入，0 表示使用真实过程量。
 */
uint32_t AoOutput_IsSimulationEnabled(void);

/**
 * @brief 复制一致的AO运行态快照。
 *
 * @param runtime 用于接收当前模拟输出目标、实际值、诊断和状态标志的一致快照。
 */
void AoOutput_GetRuntimeSnapshot(AoOutputRuntime *runtime);
/**
 * @brief 返回兼容旧调用点的只读运行态指针。
 *
 * @return 返回 ao_output_runtime 只读运行态地址；对象由模块静态持有，调用方不得修改或释放。
 */
const AoOutputRuntime *AoOutput_GetRuntime(void);

/**
 * @brief HART读取最后一次成功下发的电流，而不是尚未写入的目标值。
 *
 * HART使用最后成功下发电流和真实过程百分数。
 *
 * @return 返回运行快照中最后一次成功下发电流 last_sent_mA_x1000 除以 1000 后的值，单位 mA；未确认的目标电流不参与返回。
 */
float AoOutput_GetCurrent_mA(void);
/**
 * @brief 返回真实百分数，例如50.00表示50%，不再返回0.5。
 *
 * @return 返回运行快照中 percent_x100 除以 100 后的真实百分数，例如内部 5000 返回 50.00%。
 */
float AoOutput_GetPercentOfRange(void);

/**
 * @brief 由 TIM4 中断请求一次 AO 延后刷新。
 *
 * @details 调用场景：TIM4_IRQHandler 在继电器刷新后调用。
 *
 * TIM4中断只请求刷新，PendSV执行实际SPI访问。
 *
 * @note 关键约束：只置位请求并挂起 PendSV，不访问 AD5421、不打印、不阻塞。
 */
void AoOutput_RequestTimerRefreshFromTim4Isr(void);
/**
 * @brief 处理 TIM4 请求的 AO 延后刷新。
 *
 * @details 调用场景：PendSV_Handler 最低优先级调用，补偿主循环阻塞时 AO 长时间不刷新。
 * @note 关键约束：会访问 AD5421 SPI；驱动内部打印被抑制，错误只记录到 AO 运行态和全局错误码。
 *
 * @return 返回本次延后刷新结果码；无待处理请求时返回上次运行态错误码，执行刷新时返回实际写入结果。
 */
uint32_t AoOutput_ProcessPendingTimerRefresh(void);
/**
 * @brief 在主循环任务态统一输出 AD5421 故障和恢复日志。
 *
 * @details 调用场景：App_MainLoop 每轮后台轻量检查阶段调用。
 * @note 关键约束：不得在 ISR 或 PendSV 中调用；持续相同故障不会重复刷屏。
 */
void AoOutput_ProcessDeferredDiagnostics(void);
/**
 * @brief 暂停定时触发的 AO 自动刷新。
 *
 * @details 调用场景：串口 AO 正式测试直接访问 AD5421 前调用。
 * @note 关键约束：只影响 TIM4 请求和 PendSV 延后刷新，不影响继电器和前台 AO 调用。
 */
void AoOutput_SuspendTimerRefresh(void);
/**
 * @brief 恢复定时触发的 AO 自动刷新。
 *
 * @details 调用场景：串口 AO 正式测试退出前调用。
 * @note 关键约束：按计数恢复；如果暂停期间已有请求，则重新挂起 PendSV。
 */
void AoOutput_ResumeTimerRefresh(void);

#endif /* AO_OUTPUT_H_ */
