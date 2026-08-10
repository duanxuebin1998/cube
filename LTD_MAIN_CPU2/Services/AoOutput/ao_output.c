#include "ao_output.h"

#include "ad5421.h"
#include "encoder.h"
#include "error_log.h"
#include "main.h"
#include "motor_ctrl.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* AD5421 硬件输出保护下限 3.20 mA，线值单位为 0.001 mA；所有业务目标在下发前都必须钳位到硬件范围。 */
#define AO_OUTPUT_HARDWARE_MIN_MA_X1000 3200U
/* AD5421 硬件输出保护上限 24.00 mA，线值单位为 0.001 mA。 */
#define AO_OUTPUT_HARDWARE_MAX_MA_X1000 24000U
/* AO 目标值周期刷新间隔 1000 ms。 */
#define AO_OUTPUT_REFRESH_INTERVAL_MS  1000U
/* AD5421 状态与回读诊断的周期 1000 ms；诊断失败由独立恢复节拍处理，不在主循环内忙等。 */
#define AO_OUTPUT_DIAG_INTERVAL_MS     1000U
/* AO 驱动故障后的最小恢复尝试间隔 1000 ms。 */
#define AO_OUTPUT_RECOVER_INTERVAL_MS  1000U
/* AO 可选择的过程量来源数量 3；用于来源枚举边界和运行值映射表长度校验。 */
#define AO_PROCESS_SOURCE_COUNT        3U

/* 不同 AO 电流制式的过程输出和故障输出边界，所有值均以 0.01 mA 定点数保存。 */
typedef struct {
    /* AO 电流制式对应的过程输出及故障输出上下限，单位均为 0.01 mA。 */
    uint32_t process_min_mA_x100; /* 过程输出下限，单位为 0.01 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t process_max_mA_x100; /* 过程输出上限，单位为 0.01 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t fault_min_mA_x100; /* 故障输出下限，单位为 0.01 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t fault_max_mA_x100; /* 故障输出上限，单位为 0.01 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
} AoCurrentModeLimits;

/* AO 当前运行态快照，由更新流程统一刷新并供协议与诊断读取。 */
static AoOutputRuntime ao_output_runtime = {
    4000U, 4000U, AO_OUTPUT_SOURCE_INITIAL, 0U, 0U, NO_ERROR,
    0U, 0U, 0U, 0, 0, 0U, 0U, 0U, 0U
};
/* 各 AO 过程量来源的最近有效样本和更新代际。 */
static AoProcessSample ao_process_samples[AO_PROCESS_SOURCE_COUNT] = {0};
/* AO 服务已经完成首次配置和驱动初始化的标志。 */
static uint8_t ao_output_initialized = 0U;
/* AO 更新流程正在执行的互斥标志，防止定时器与前台重入。 */
static volatile uint8_t ao_output_update_busy = 0U;
/* 定时器要求前台补做一次 AO 刷新的挂起标志。 */
static volatile uint8_t ao_output_timer_refresh_pending = 0U;
/* AO 定时刷新被临界流程暂停的嵌套计数。 */
static volatile uint32_t ao_output_timer_suspend_count = 0U;
/* AO 仿真输出当前是否生效的运行态开关。 */
static volatile uint32_t ao_simulation_enabled = 0U;
/* AD5421 驱动已经初始化并可接受输出请求的标志。 */
static uint8_t ao_output_driver_ready = 0U;
/* 驱动重试节拍基线已经建立的标志。 */
static uint8_t ao_output_driver_retry_valid = 0U;
/* 最近一次 AD5421 诊断结果可用的标志。 */
static uint8_t ao_output_diag_valid = 0U;
/* 最近一次电流写入尝试记录有效的标志。 */
static uint8_t ao_output_write_attempt_valid = 0U;
/* 最近一次 AD5421 初始化重试的 HAL 毫秒节拍。 */
static uint32_t ao_output_last_driver_retry_tick = 0U;
/* 最近一次读取 AD5421 诊断寄存器的 HAL 毫秒节拍。 */
static uint32_t ao_output_last_diag_tick = 0U;
/* 最近一次 AD5421 诊断返回的统一故障码。 */
static uint32_t ao_output_last_diag_error = NO_ERROR;
/* AO 驱动恢复节拍基线已经建立的标志。 */
static uint8_t ao_output_recover_valid = 0U;
/* 最近一次执行 AO 驱动恢复的 HAL 毫秒节拍。 */
static uint32_t ao_output_last_recover_tick = 0U;
/* 最近一次尝试写入 AO 电流的 HAL 毫秒节拍。 */
static uint32_t ao_output_last_write_attempt_tick = 0U;
/* 最近一次尝试写入的 AO 电流，单位为 0.001 mA。 */
static uint32_t ao_output_last_write_attempt_mA_x1000 = 0U;
/* 最近一次驱动恢复后重写的目标电流，单位为 0.001 mA。 */
static uint32_t ao_output_last_recover_target_mA_x1000 = 0U;
/* 最近一次过程量到电流换算结果有效的标志。 */
static uint8_t ao_last_process_valid = 0U;
/* 最近一次阻尼前的过程量基准电流，单位为 0.01 mA。 */
static uint32_t ao_last_process_base_current_mA_x100 = 0U;
/* 最近一次参与 AO 换算的过程量，单位为 0.1 mm。 */
static int32_t ao_last_process_value_01mm = 0;
/* 最近一次过程量占量程百分比的百倍定点值。 */
static int32_t ao_last_process_percent_x100 = 0;
/* AO 配置变化检测基线已经建立的标志。 */
static uint8_t ao_config_snapshot_valid = 0U;
/* 最近一次已经应用的 AO 配置快照，用于识别运行时参数变化。 */
static AoOutputConfig ao_last_config_snapshot = {0};
/* 位置源换算最近使用的罐高，单位为 0.1 mm。 */
static uint32_t ao_last_position_tank_height_01mm = 0U;
/* 位置源换算最近使用的编码器/电机记步模式。 */
static uint32_t ao_last_position_count_mode = POSITION_COUNT_MODE_ENCODER;
/* AO 阻尼滤波器已经用首个有效输入初始化的标志。 */
static uint8_t ao_filter_initialized = 0U;
/* 当前阻尼滤波状态对应的 AO 过程量来源。 */
static uint32_t ao_filter_source = AO_PROCESS_SOURCE_TANK_LEVEL;
/* AO 阻尼滤波器最近更新时间的 HAL 毫秒节拍。 */
static uint32_t ao_filter_tick = 0U;
/* AO 阻尼滤波内部电流状态的千倍定点值，用于降低整数截断误差。 */
static int64_t ao_filter_value_x1000 = 0;
/* AD5421 新故障快照等待前台上报的标志。 */
static volatile uint8_t ao_output_diag_error_pending = 0U;
/* AD5421 故障恢复快照等待前台清除故障的标志。 */
static volatile uint8_t ao_output_diag_recover_pending = 0U;
/* AD5421 诊断故障当前处于活动状态的标志。 */
static volatile uint8_t ao_output_diag_fault_active = 0U;
/* 当前活动的 AD5421 统一故障码。 */
static uint32_t ao_output_diag_active_error_code = NO_ERROR;
/* 等待前台上报的 AD5421 故障码。 */
static uint32_t ao_output_diag_pending_error_code = NO_ERROR;
/* 等待前台恢复处理的 AD5421 故障码。 */
static uint32_t ao_output_diag_pending_recover_code = NO_ERROR;
/* 当前活动 AD5421 故障对应的驱动诊断快照。 */
static AD5421DiagnosticSnapshot ao_output_diag_active_snapshot = {0U};
/* 等待前台上报的 AD5421 故障诊断快照。 */
static AD5421DiagnosticSnapshot ao_output_diag_pending_error_snapshot = {0U};
/* 等待前台恢复处理的 AD5421 诊断快照。 */
static AD5421DiagnosticSnapshot ao_output_diag_pending_recover_snapshot = {0U};

/**
 * @brief 执行AO固定优先级状态机并刷新AD5421。
 *
 * 函数在 AO 子系统尚未初始化时无操作返回；初始化后先取得一致的配置快照、处理配置切换，并按禁用或固定或上电初始来源计算经电流修正后的 AD5421 初始化目标。
 * 驱动未就绪时根据 allow_driver_init 决定是否执行初始化；随后限频读取诊断。非过温故障可按恢复间隔对最近成功目标执行一次 AD5421
 * 恢复，过温告警或关断只等待温度恢复，不周期复位器件。
 * 本轮基础目标严格按禁用、整机故障、仿真、固定电流、有效过程量的优先级选择；过程样本无效时优先保持最近一次已经成功写入的过程目标，没有历史值才回退到上电电流。
 * 目标选择阶段只产生未经修正的基础电流，随后统一执行一次零点或增益修正并钳位到硬件范围；只有变化量或刷新周期要求写入时才访问 AD5421。
 * 过程目标必须在 AD5421 写入成功后才进入保持缓存；写入或诊断失败仍发布本轮输入、比例、故障寄存器和最近成功电流到运行快照，但不会把失败目标伪装成已下发值。
 *
 * @param allow_driver_init 非 0 允许在 AD5421 尚未就绪时执行初始化并下发初始电流；0 禁止初始化，驱动未就绪时返回 AD5421_INIT_ERROR。
 * @return NO_ERROR 表示本轮无需动作或目标已保持或成功刷新；其他值为 AD5421 初始化、诊断恢复、目标选择或电流写入错误，并已同步记录到 AO 运行快照。
 * @note 本函数不自行获取 AO 重入锁；普通任务调用前通过 AoOutput_TryEnterUpdate，PendSV 延后刷新则在关中断临界区设置
 *       ao_output_update_busy，二者都必须先建立独占更新上下文。
 */
static uint32_t AoOutput_UpdateInternal(uint8_t allow_driver_init);

/**
 * @brief 判断两次 AD5421 故障是否属于同一故障现场。
 *
 * @details 调用场景：AO 诊断日志入队前抑制持续故障的重复打印。
 * @note 关键约束：忽略快照序号，只比较实际定位信息。
 *
 * @param left 区间左端值或左侧比较对象。
 * @param right 区间右端值或右侧比较对象。
 * @param left_error 故障。
 * @param right_error 故障。
 * @return 1 表示两次 AD5421 故障属于同一故障现场；0 表示两次 AD5421 故障不属于同一故障现场。
 */
static uint8_t AoOutput_IsSameDiagnostic(const AD5421DiagnosticSnapshot *left,
                                         const AD5421DiagnosticSnapshot *right,
                                         uint32_t left_error,
                                         uint32_t right_error)
{
    if ((left == NULL) || (right == NULL) || (left_error != right_error)) {
        return 0U;
    }

    if ((left->root_error_code == right->root_error_code) &&
        (left->fault_flags == right->fault_flags) &&
        (left->fault_register == right->fault_register) &&
        (left->hal_status == right->hal_status) &&
        (left->expected_value == right->expected_value) &&
        (left->actual_value == right->actual_value) &&
        (left->stage == right->stage) &&
        (left->direction == right->direction) &&
        (left->reg == right->reg)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 把 AD5421 故障快照放入主循环延后日志槽。
 *
 * @details 调用场景：AO 初始化、诊断、恢复或写电流失败后调用。
 * @note 关键约束：只复制数值并置位，不打印；相同持续故障只保留一次。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是 AD5421 驱动或 AO 运行错误，函数按错误类别决定入队、恢复或诊断升级。
 */
static void AoOutput_QueueDriverError(uint32_t error_code)
{
    AD5421DiagnosticSnapshot snapshot;
    uint32_t primask;
    uint8_t same_fault;

    AD5421_GetDiagnosticSnapshot(&snapshot);
    if (snapshot.sequence == 0U) {
        snapshot.error_code = error_code;
        snapshot.root_error_code = error_code;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    same_fault = 0U;
    if (ao_output_diag_fault_active != 0U) {
        same_fault = AoOutput_IsSameDiagnostic(&snapshot,
                                               &ao_output_diag_active_snapshot,
                                               error_code,
                                               ao_output_diag_active_error_code);
    }
    if (same_fault == 0U) {
        if (ao_output_diag_fault_active == 0U) {
            ao_output_diag_recover_pending = 0U;
        }
        ao_output_diag_active_snapshot = snapshot;
        ao_output_diag_active_error_code = error_code;
        ao_output_diag_pending_error_snapshot = snapshot;
        ao_output_diag_pending_error_code = error_code;
        ao_output_diag_error_pending = 1U;
        ao_output_diag_fault_active = 1U;
    }
    __set_PRIMASK(primask);
}

/**
 * @brief 记录 AD5421 故障已恢复，交由主循环统一打印。
 *
 * @details 调用场景：驱动重新初始化、恢复序列或正常电流写入成功后调用。
 * @note 关键约束：只有之前记录过有效故障时才生成恢复日志。
 */
static void AoOutput_QueueDriverRecovery(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if (ao_output_diag_fault_active != 0U) {
        ao_output_diag_pending_recover_snapshot = ao_output_diag_active_snapshot;
        ao_output_diag_pending_recover_code = ao_output_diag_active_error_code;
        ao_output_diag_recover_pending = 1U;
        ao_output_diag_fault_active = 0U;
        ao_output_diag_active_error_code = NO_ERROR;
    }
    __set_PRIMASK(primask);
}
/**
 * @brief 挂起最低优先级 AO 延后刷新。
 *
 * @details 调用场景：TIM4 请求 AO 刷新或测试流程恢复自动刷新时调用。
 * @note 关键约束：只设置 PendSV 挂起位，不访问 SPI、不打印、不阻塞。
 */
static void AoOutput_PendDeferredRefresh(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}

/**
 * @brief 尝试占用 AO 更新窗口。
 *
 * @details 调用场景：前台刷新和 PendSV 延后刷新进入 AO 服务前调用。
 * @note 关键约束：只用短临界区保护标志位，不在临界区内访问 SPI。
 *
 * @return 1 表示当前调用已取得 AO 更新窗口；另一任务或中断路径正在更新时返回 0。
 */
static uint8_t AoOutput_TryEnterUpdate(void)
{
    uint32_t primask;
    uint8_t entered = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (ao_output_update_busy == 0U) {
        ao_output_update_busy = 1U;
        entered = 1U;
    }
    __set_PRIMASK(primask);

    return entered;
}

/**
 * @brief 释放 AO 更新窗口。
 *
 * @details 调用场景：前台刷新和 PendSV 延后刷新退出 AO 服务时调用。
 * @note 关键约束：只清保护标志，真实错误状态已在调用链里保存。
 */
static void AoOutput_LeaveUpdate(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ao_output_update_busy = 0U;
    __set_PRIMASK(primask);
}
/**
 * @brief 原子发布AO运行态，避免Modbus与HART读到跨轮次字段。
 *
 * @param runtime 已经完成边界计算和驱动写入的 AO 运行态候选快照，提交后对外可见。
 */
static void AoOutput_CommitRuntime(const AoOutputRuntime *runtime)
{
    uint32_t primask;

    if (runtime == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    ao_output_runtime = *runtime;
    __set_PRIMASK(primask);
}

/**
 * @brief 复制一致的AO运行态快照。
 *
 * @param runtime 用于接收当前模拟输出目标、实际值、诊断和状态标志的一致快照。
 */
void AoOutput_GetRuntimeSnapshot(AoOutputRuntime *runtime)
{
    uint32_t primask;

    if (runtime == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *runtime = ao_output_runtime;
    __set_PRIMASK(primask);
}

/**
 * @brief 在短临界区内更新过程量样本。
 *
 * @param source AO 过程量来源枚举；区分储罐液位、传感器位置和水位，用于选择并更新各自独立的过程样本槽。
 * @param value_01mm 位置或距离值，单位 0.1 mm。
 * @param valid 有效。
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 */
static void AoOutput_WriteProcessSample(AoProcessSource source,
                                        int32_t value_01mm,
                                        uint8_t valid,
                                        uint32_t now)
{
    AoProcessSample *sample;

    if ((uint32_t)source >= AO_PROCESS_SOURCE_COUNT) {
        return;
    }
    sample = &ao_process_samples[(uint32_t)source];
    sample->value_01mm = value_01mm;
    sample->valid = (valid == 0U) ? 0U : 1U;
    sample->update_tick = now;
    sample->update_counter++;
}

/**
 * @brief 发布液位或水位过程量；传感器位置由AO读取权威运行值。
 *
 * @param source AO 过程量来源枚举；区分储罐液位、传感器位置和水位，用于选择并更新各自独立的过程样本槽。
 * @param value_01mm 位置或距离值，单位 0.1 mm。
 * @param valid 有效。
 */
void AoOutput_PublishProcessSample(AoProcessSource source, int32_t value_01mm, uint8_t valid)
{
    uint32_t primask;
    uint32_t now = HAL_GetTick();

    primask = __get_PRIMASK();
    __disable_irq();
    AoOutput_WriteProcessSample(source, value_01mm, valid, now);
    __set_PRIMASK(primask);
}

/**
 * @brief 显式失效一个过程量样本。
 *
 * @param source AO 过程量来源枚举；区分储罐液位、传感器位置和水位，用于选择并更新各自独立的过程样本槽。
 */
void AoOutput_InvalidateProcessSample(AoProcessSource source)
{
    AoOutput_PublishProcessSample(source, 0, 0U);
}

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
uint32_t AoOutput_ReadProcessSample(AoProcessSource source, AoProcessSample *sample)
{
    uint32_t position_count_mode;
    uint32_t primask;
    uint8_t position_ready;

    if ((sample == NULL) || ((uint32_t)source >= AO_PROCESS_SOURCE_COUNT)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    if (source == AO_PROCESS_SOURCE_SENSOR_POSITION) {
        primask = __get_PRIMASK();
        __disable_irq();
        position_count_mode = g_deviceParams.position_count_mode;
        position_ready = (position_count_mode == POSITION_COUNT_MODE_MOTOR) ?
                         (uint8_t)MotorCtrl_IsDriverInitValid() :
                         (uint8_t)Encoder_IsReady();
        sample->value_01mm = g_measurement.debug_data.sensor_position;
        sample->update_counter = ao_process_samples[(uint32_t)source].update_counter + 1U;
        sample->update_tick = HAL_GetTick();
        sample->valid = (position_ready == 0U) ? 0U : 1U;
        ao_process_samples[(uint32_t)source] = *sample;
        __set_PRIMASK(primask);
        return NO_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *sample = ao_process_samples[(uint32_t)source];
    __set_PRIMASK(primask);
    return NO_ERROR;
}

/**
 * @brief 在短临界区复制完整AO配置，避免FC10中断替换参数时单轮混用。
 *
 * @param config 可写 AO 输出配置对象；函数按职责填充默认值或当前快照，字段覆盖模式、来源、量程、电流修正、阻尼、故障动作、上电电流和仿真电流。
 */
static void AoOutput_GetConfigSnapshot(AoOutputConfig *config)
{
    uint32_t primask;

    if (config == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *config = g_deviceParams.ao_output;
    __set_PRIMASK(primask);
}

/**
 * @brief 复制当前配置选中的过程量快照。
 *
 * @param sample 待处理的单次测量样本。
 * @return 返回整机错误码；NO_ERROR 表示已复制当前输出源的有效过程量快照，其他值表示来源或样本无效。
 */
uint32_t AoOutput_GetSelectedProcessSample(AoProcessSample *sample)
{
    AoOutputConfig config;

    AoOutput_GetConfigSnapshot(&config);
    return AoOutput_ReadProcessSample((AoProcessSource)config.output_source, sample);
}

/**
 * @brief 设置仿真运行态；上电初始化会强制清零。
 *
 * @param enabled 目标使能状态，非零表示启用，零表示禁用。
 */
void AoOutput_SetSimulationEnabled(uint32_t enabled)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ao_simulation_enabled = (enabled == 0U) ? 0U : 1U;
    ao_output_runtime.simulation_enabled = ao_simulation_enabled;
    ao_filter_initialized = 0U;
    __set_PRIMASK(primask);
}

/**
 * @brief 返回仿真运行态开关。
 *
 * @return 返回 AO 仿真运行态开关；1 表示使用仿真输入，0 表示使用真实过程量。
 */
uint32_t AoOutput_IsSimulationEnabled(void)
{
    return ao_simulation_enabled;
}

/**
 * @brief 将电流限制在AD5421物理允许范围内。
 *
 * @param current_mA_x1000 本次设置、限制或测试使用的模拟输出电流，单位 0.001 mA。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint32_t AoOutput_ClampHardwareCurrent(uint32_t current_mA_x1000)
{
    if (current_mA_x1000 < AO_OUTPUT_HARDWARE_MIN_MA_X1000) {
        return AO_OUTPUT_HARDWARE_MIN_MA_X1000;
    }
    if (current_mA_x1000 > AO_OUTPUT_HARDWARE_MAX_MA_X1000) {
        return AO_OUTPUT_HARDWARE_MAX_MA_X1000;
    }
    return current_mA_x1000;
}

/**
 * @brief AO启用时在基础目标上统一叠加一次修正；禁用态3.40mA保持原值。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @param source 已经解析的 AO 输出来源枚举；区分过程量、固定电流、仿真、故障电流和保持上次有效值等目标来源。
 * @param base_target_mA_x100 尚未叠加电流修正量的模拟输出目标，单位 0.01 mA。
 * @return 返回叠加一次 AO 电流修正并钳位后的目标，单位 0.001 mA；禁用态 3.400 mA 不参与修正。
 */
static uint32_t AoOutput_ApplyCurrentCorrection(const AoOutputConfig *config,
                                                AoOutputSource source,
                                                uint32_t base_target_mA_x100)
{
    int64_t corrected_target;

    if ((config == NULL) || (source == AO_OUTPUT_SOURCE_DISABLED)) {
        return base_target_mA_x100 * 10U;
    }

    corrected_target = ((int64_t)base_target_mA_x100 * 10) +
                       (int64_t)config->current_correction_mA_x1000;
    if (corrected_target < (int64_t)AO_OUTPUT_HARDWARE_MIN_MA_X1000) {
        return AO_OUTPUT_HARDWARE_MIN_MA_X1000;
    }
    if (corrected_target > (int64_t)AO_OUTPUT_HARDWARE_MAX_MA_X1000) {
        return AO_OUTPUT_HARDWARE_MAX_MA_X1000;
    }
    return (uint32_t)corrected_target;
}

/**
 * @brief 返回NMS81普通、NE、US及固定模式的过程与故障边界。
 *
 * @param current_mode 当前 AO 电流模式枚举，用于选择过程量程和故障电流的最小、最大边界。
 * @return 返回当前电流模式的过程最小值、过程最大值、故障最小值和故障最大值边界结构。
 */
static AoCurrentModeLimits AoOutput_GetModeLimits(uint32_t current_mode)
{
    AoCurrentModeLimits limits;

    limits.process_min_mA_x100 = 380U;
    limits.process_max_mA_x100 = 2050U;
    limits.fault_min_mA_x100 = 350U;
    limits.fault_max_mA_x100 = 2260U;
    if (current_mode == AO_CURRENT_MODE_FIXED) {
        limits.process_min_mA_x100 = 400U;
        limits.process_max_mA_x100 = 2250U;
        limits.fault_min_mA_x100 = 400U;
        limits.fault_max_mA_x100 = 2250U;
    } else if (current_mode == AO_CURRENT_MODE_US) {
        limits.process_min_mA_x100 = 390U;
        limits.process_max_mA_x100 = 2080U;
        limits.fault_max_mA_x100 = 2200U;
    } else if (current_mode == AO_CURRENT_MODE_NORMAL) {
        limits.process_min_mA_x100 = 400U;
        limits.process_max_mA_x100 = 2050U;
    }
    return limits;
}

/**
 * @brief 由过程值计算有符号百分比和模式限幅后的电流。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @param value_01mm 位置或距离值，单位 0.1 mm。
 * @param percent_x100 百分比定点值，单位 0.01%。
 * @return 返回过程量按量程、输出模式和百分比限制换算后的目标电流，单位 0.001 mA。
 */
static uint32_t AoOutput_MapProcessToCurrent(const AoOutputConfig *config,
                                             int32_t value_01mm,
                                             int32_t *percent_x100)
{
    int64_t range_0 = (int64_t)config->range_0_01mm;
    int64_t range_100 = (int64_t)config->range_100_01mm;
    int64_t range_span = range_100 - range_0;
    int64_t percent;
    int64_t current;
    AoCurrentModeLimits limits = AoOutput_GetModeLimits(config->current_mode);

    if (range_span == 0) {
        percent = 0;
    } else {
        percent = (((int64_t)value_01mm - range_0) * 10000LL) / range_span;
    }
    if (percent > 0x7FFFFFFFLL) {
        percent = 0x7FFFFFFFLL;
    } else if (percent < -2147483648LL) {
        percent = -2147483648LL;
    }
    if (percent_x100 != NULL) {
        *percent_x100 = (int32_t)percent;
    }

    current = 400LL + ((percent * 1600LL) / 10000LL);
    if (current < (int64_t)limits.process_min_mA_x100) {
        current = limits.process_min_mA_x100;
    } else if (current > (int64_t)limits.process_max_mA_x100) {
        current = limits.process_max_mA_x100;
    }
    return (uint32_t)current;
}

/**
 * @brief 对正常过程输入应用一阶阻尼，特殊输出状态不调用本函数。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @param raw_01mm 模拟输出换算前的原始过程量，单位 0.1 mm。
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @return 返回一阶阻尼后的过程量，单位 0.1 mm；滤波尚未建立时直接返回当前原始值。
 */
static int32_t AoOutput_FilterProcess(const AoOutputConfig *config,
                                      int32_t raw_01mm,
                                      uint32_t now)
{
    uint32_t damping_x10_s = config->damping_x10_s;
    uint32_t source = config->output_source;
    uint32_t dt_ms;
    uint64_t tau_ms;
    int64_t delta;

    if ((damping_x10_s == 0U) ||
        (ao_filter_initialized == 0U) ||
        (ao_filter_source != source)) {
        ao_filter_initialized = 1U;
        ao_filter_source = source;
        ao_filter_tick = now;
        ao_filter_value_x1000 = (int64_t)raw_01mm * 1000LL;
        return raw_01mm;
    }

    dt_ms = now - ao_filter_tick;
    if (dt_ms == 0U) {
        return (int32_t)(ao_filter_value_x1000 / 1000LL);
    }
    tau_ms = (uint64_t)damping_x10_s * 100ULL;
    delta = ((int64_t)raw_01mm * 1000LL) - ao_filter_value_x1000;
    ao_filter_value_x1000 += (delta * (int64_t)dt_ms) / (int64_t)(tau_ms + dt_ms);
    ao_filter_tick = now;
    if (ao_filter_value_x1000 >= 0) {
        return (int32_t)((ao_filter_value_x1000 + 500LL) / 1000LL);
    }
    return (int32_t)((ao_filter_value_x1000 - 500LL) / 1000LL);
}

/**
 * @brief 判断液位源是否处于允许AO跟随的命令。
 *
 * @param command 当前 CommandType 命令；找油、油位跟随及其兼容测量流程用于判定液位 AO 是否跟随。
 * @return 1 表示 command 为 CMD_FIND_OIL、CMD_CALIBRATE_OIL 或 CMD_CORRECT_OIL，液位源允许进入 AO 连续跟随；0 表示其它命令，不允许按液位跟随。
 */
static uint8_t AoOutput_IsLiquidFollowCommand(CommandType command)
{
    return ((command == CMD_FIND_OIL) ||
            (command == CMD_CALIBRATE_OIL) ||
            (command == CMD_CORRECT_OIL)) ? 1U : 0U;
}

/**
 * @brief 判断水位源是否处于允许AO跟随的命令。
 *
 * @param command 当前 CommandType 命令；找水、水位跟随及其兼容测量流程用于判定水位 AO 是否跟随。
 * @return 1 表示 command 为 CMD_FOLLOW_WATER 或 CMD_CALIBRATE_WATER，水位源允许进入 AO 连续跟随；0 表示其它命令，不允许按水位跟随。
 */
static uint8_t AoOutput_IsWaterFollowCommand(CommandType command)
{
    return ((command == CMD_FOLLOW_WATER) ||
            (command == CMD_CALIBRATE_WATER)) ? 1U : 0U;
}

/**
 * @brief 判断所选过程源是否正处于与其匹配的连续跟随状态。
 *
 * @details 调用场景：AO每轮选择目标电流前只读整机状态、当前命令和待切换命令。
 * @note 关键约束：液位和水位存在待处理命令时退出跟随；传感器位置不受命令门禁限制。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @return 1 表示输出源为传感器位置，或在没有 pending 命令时，罐液位或水位源分别处于匹配的跟随状态和命令；0 表示配置为空、存在待执行命令、输出源不支持连续跟随，或当前状态和命令与所选过程源不匹配。
 */
static uint8_t AoOutput_IsSelectedProcessFollowing(const AoOutputConfig *config)
{
    DeviceState state;
    CommandType current_command;
    CommandType pending_command;
    uint32_t primask;

    if (config == NULL) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    state = g_measurement.device_status.device_state;
    current_command = g_measurement.device_status.current_command;
    pending_command = g_deviceParams.command;
    __set_PRIMASK(primask);

    if (config->output_source == AO_PROCESS_SOURCE_SENSOR_POSITION) {
        return 1U;
    }
    if (pending_command != CMD_NONE) {
        return 0U;
    }
    if (config->output_source == AO_PROCESS_SOURCE_TANK_LEVEL) {
        return ((state == STATE_FLOWOIL) &&
                (AoOutput_IsLiquidFollowCommand(current_command) != 0U)) ? 1U : 0U;
    }
    if (config->output_source == AO_PROCESS_SOURCE_WATER_LEVEL) {
        return ((state == STATE_FOLLOW_WATERING) &&
                (AoOutput_IsWaterFollowCommand(current_command) != 0U)) ? 1U : 0U;
    }
    return 0U;
}

/**
 * @brief 处理会改变过程电流解释的AO配置切换。
 *
 * @details 调用场景：AO初始化及每轮取得一致配置快照后调用。
 * @note 关键约束：禁用、输出源、电流模式或量程变化会清除上次有效过程电流；阻尼变化只重置滤波。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 */
static void AoOutput_HandleConfigTransition(const AoOutputConfig *config)
{
    uint32_t position_count_mode;
    uint32_t position_tank_height_01mm;
    uint32_t primask;
    uint8_t clear_last_valid = 0U;
    uint8_t reset_filter = 0U;

    if (config == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    position_tank_height_01mm = g_deviceParams.tankHeight;
    position_count_mode = g_deviceParams.position_count_mode;
    __set_PRIMASK(primask);

    if (ao_config_snapshot_valid != 0U) {
        if ((config->output_source != ao_last_config_snapshot.output_source) ||
            (config->current_mode != ao_last_config_snapshot.current_mode) ||
            (config->range_0_01mm != ao_last_config_snapshot.range_0_01mm) ||
            (config->range_100_01mm != ao_last_config_snapshot.range_100_01mm)) {
            clear_last_valid = 1U;
            reset_filter = 1U;
        } else if (config->damping_x10_s != ao_last_config_snapshot.damping_x10_s) {
            reset_filter = 1U;
        }
        if ((config->output_source == AO_PROCESS_SOURCE_SENSOR_POSITION) &&
            ((position_tank_height_01mm != ao_last_position_tank_height_01mm) ||
             (position_count_mode != ao_last_position_count_mode))) {
            clear_last_valid = 1U;
            reset_filter = 1U;
        }
    }
    if (config->work_mode == AO_WORK_MODE_DISABLED) {
        clear_last_valid = 1U;
        reset_filter = 1U;
    }

    if (clear_last_valid != 0U) {
        ao_last_process_valid = 0U;
        ao_last_process_base_current_mA_x100 = 0U;
        ao_last_process_value_01mm = 0;
        ao_last_process_percent_x100 = 0;
    }
    if (reset_filter != 0U) {
        ao_filter_initialized = 0U;
    }
    ao_last_config_snapshot = *config;
    ao_last_position_tank_height_01mm = position_tank_height_01mm;
    ao_last_position_count_mode = position_count_mode;
    ao_config_snapshot_valid = 1U;
}

/**
 * @brief 仅非维护状态下整机明确进入错误态且带真实错误码时执行AO故障电流。
 *
 * @return 1 表示非维护状态下整机处于真实错误态，应切换故障电流；其他状态返回 0。
 */
static uint8_t AoOutput_ShouldUseFaultCurrent(void)
{
    DeviceState state;
    uint32_t error_code;
    uint32_t maintenance_mode_active;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    state = g_measurement.device_status.device_state;
    error_code = g_measurement.device_status.error_code;
    maintenance_mode_active = g_measurement.device_status.maintenance_mode_active;
    __set_PRIMASK(primask);

    /* 维护模式只旁路设备故障电流，后续固定、仿真和过程输出选择保持不变。 */
    return ((maintenance_mode_active == 0U) &&
            (state == STATE_ERROR) &&
            (error_code != NO_ERROR) &&
            (error_code != STATE_SWITCH)) ? 1U : 0U;
}

/**
 * @brief 把上次成功过程目标对应的输入和值域比例写入本轮运行态。
 *
 * @param process_value_01mm 模拟输出使用的有效过程量，单位 0.1 mm。
 * @param percent_x100 百分比定点值，单位 0.01%。
 * @param process_valid 用于返回当前过程值是否可用于 AO 正常输出。
 */
static void AoOutput_UseLastProcess(int32_t *process_value_01mm,
                                    int32_t *percent_x100,
                                    uint32_t *process_valid)
{
    *process_value_01mm = ao_last_process_value_01mm;
    *percent_x100 = ao_last_process_percent_x100;
    *process_valid = 1U;
}

/**
 * @brief 保持状态冻结阻尼时间，不让旁路持续时间形成恢复瞬间的大步进。
 *
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 */
static void AoOutput_FreezeFilter(uint32_t now)
{
    if (ao_filter_initialized != 0U) {
        ao_filter_tick = now;
    }
}

/**
 * @brief 按故障动作选择旁路阻尼的电流；保持动作无历史时回退初始电流。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @param process_value_01mm 模拟输出使用的有效过程量，单位 0.1 mm。
 * @param percent_x100 百分比定点值，单位 0.01%。
 * @param process_valid 用于返回当前过程值是否可用于 AO 正常输出。
 * @return 返回故障动作选择的目标电流，单位 0.01 mA；保持动作且无历史值时回退到初始基础电流。
 */
static uint32_t AoOutput_SelectFaultCurrent(const AoOutputConfig *config,
                                            int32_t *process_value_01mm,
                                            int32_t *percent_x100,
                                            uint32_t *process_valid)
{
    if ((config->fault_mode == AO_FAULT_ACTION_HOLD_LAST_VALID) &&
        (ao_last_process_valid != 0U)) {
        AoOutput_UseLastProcess(process_value_01mm, percent_x100, process_valid);
        return ao_last_process_base_current_mA_x100;
    }
    if (config->fault_mode == AO_FAULT_ACTION_HOLD_LAST_VALID) {
        return config->power_on_current_mA_x100;
    }
    return config->fault_current_mA_x100;
}

/**
 * @brief 按禁用、设备故障、仿真、固定和过程输出的固定优先级选择本轮目标。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @param source AO 输出来源枚举输出指针；目标选择成功后写入本轮实际采用的过程量、固定、仿真、故障或保持来源。
 * @param base_target_mA_x100 尚未叠加电流修正量的模拟输出目标，单位 0.01 mA。
 * @param process_value_01mm 模拟输出使用的有效过程量，单位 0.1 mm。
 * @param percent_x100 百分比定点值，单位 0.01%。
 * @param process_valid 用于返回当前过程值是否可用于 AO 正常输出。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t AoOutput_SelectTarget(const AoOutputConfig *config,
                                      uint32_t now,
                                      AoOutputSource *source,
                                      uint32_t *base_target_mA_x100,
                                      int32_t *process_value_01mm,
                                      int32_t *percent_x100,
                                      uint32_t *process_valid)
{
    AoProcessSample sample = {0};
    int32_t filtered_01mm;

    if ((config == NULL) || (source == NULL) || (base_target_mA_x100 == NULL) ||
        (process_value_01mm == NULL) || (percent_x100 == NULL) ||
        (process_valid == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    *process_value_01mm = 0;
    *process_valid = 0U;
    *percent_x100 = 0;

    if (config->work_mode == AO_WORK_MODE_DISABLED) {
        ao_filter_initialized = 0U;
        *source = AO_OUTPUT_SOURCE_DISABLED;
        *base_target_mA_x100 = AO_DISABLED_CURRENT_MA_X100;
        return NO_ERROR;
    }
    if (AoOutput_ShouldUseFaultCurrent() != 0U) {
        *source = AO_OUTPUT_SOURCE_FAULT;
        *base_target_mA_x100 = AoOutput_SelectFaultCurrent(config,
                                                       process_value_01mm,
                                                       percent_x100,
                                                       process_valid);
        if (*process_valid != 0U) {
            AoOutput_FreezeFilter(now);
        } else {
            ao_filter_initialized = 0U;
        }
        return NO_ERROR;
    }
    if (ao_simulation_enabled != 0U) {
        ao_filter_initialized = 0U;
        *source = AO_OUTPUT_SOURCE_SIMULATION;
        *base_target_mA_x100 = config->simulation_current_mA_x100;
        return NO_ERROR;
    }
    if (config->current_mode == AO_CURRENT_MODE_FIXED) {
        ao_filter_initialized = 0U;
        *source = AO_OUTPUT_SOURCE_FIXED;
        *base_target_mA_x100 = config->fixed_current_mA_x100;
        return NO_ERROR;
    }

    (void)AoOutput_ReadProcessSample((AoProcessSource)config->output_source, &sample);
    if ((AoOutput_IsSelectedProcessFollowing(config) == 0U) ||
        (sample.valid == 0U)) {
        if (ao_last_process_valid != 0U) {
            AoOutput_FreezeFilter(now);
            AoOutput_UseLastProcess(process_value_01mm, percent_x100, process_valid);
            *source = AO_OUTPUT_SOURCE_HOLD_LAST;
            *base_target_mA_x100 = ao_last_process_base_current_mA_x100;
        } else {
            ao_filter_initialized = 0U;
            *source = AO_OUTPUT_SOURCE_INITIAL;
            *base_target_mA_x100 = config->power_on_current_mA_x100;
        }
        return NO_ERROR;
    }

    filtered_01mm = AoOutput_FilterProcess(config, sample.value_01mm, now);
    /* 运行态输入值与换算比例必须使用同一阻尼后过程量。 */
    *process_value_01mm = filtered_01mm;
    *process_valid = 1U;
    *source = AO_OUTPUT_SOURCE_PROCESS;
    *base_target_mA_x100 = AoOutput_MapProcessToCurrent(config, filtered_01mm, percent_x100);
    return NO_ERROR;
}

/**
 * @brief 清除AD5421诊断恢复节流状态。
 */
static void AoOutput_ResetRecoverState(void)
{
    ao_output_recover_valid = 0U;
    ao_output_last_recover_tick = 0U;
    ao_output_last_recover_target_mA_x1000 = 0U;
}

/**
 * @brief 判断当前是否允许执行一次AD5421恢复序列。
 *
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @return 1 表示当前允许执行一次AD5421恢复序列；0 表示当前不允许执行一次AD5421恢复序列。
 */
static uint8_t AoOutput_ShouldRecover(uint32_t now)
{
    if (ao_output_recover_valid == 0U) {
        return 1U;
    }
    return ((now - ao_output_last_recover_tick) >= AO_OUTPUT_RECOVER_INTERVAL_MS) ? 1U : 0U;
}

/**
 * @brief 判断诊断错误是否属于需要原地等待清除的过温状态。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是 AD5421 驱动或 AO 运行错误，函数按错误类别决定入队、恢复或诊断升级。
 * @return 1 表示诊断错误属于需要原地等待清除的过温状态；0 表示诊断错误不属于需要原地等待清除的过温状态。
 */
static uint8_t AoOutput_IsOvertemperatureError(uint32_t error_code)
{
    return ((error_code == AD5421_OVERTEMP_SHUTDOWN) ||
            (error_code == AD5421_OVERTEMP_WARNING)) ? 1U : 0U;
}

/**
 * @brief 过温故障必须等待降温，不通过周期复位强行恢复。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是 AD5421 驱动或 AO 运行错误，函数按错误类别决定入队、恢复或诊断升级。
 * @return 1 表示当前诊断错误允许周期复位恢复；过温错误必须等待降温并返回 0。
 */
static uint8_t AoOutput_IsDiagnosticRecoveryAllowed(uint32_t error_code)
{
    return (AoOutput_IsOvertemperatureError(error_code) != 0U) ? 0U : 1U;
}

/**
 * @brief 判断错误是否属于AO运行期驱动故障。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是 AD5421 驱动或 AO 运行错误，函数按错误类别决定入队、恢复或诊断升级。
 * @return 1 表示错误属于AO运行期驱动故障；0 表示错误不属于AO运行期驱动故障。
 */
static uint8_t AoOutput_IsRuntimeDriverError(uint32_t error_code)
{
    if ((error_code == AD5421_INIT_ERROR) ||
        (error_code == AD5421_READBACK_ERROR) ||
        (error_code == AD5421_INTERNAL_COMM_ERROR) ||
        (error_code == AD5421_LOOP_CURRENT_HIGH) ||
        (error_code == AD5421_LOOP_CURRENT_LOW) ||
        (error_code == AD5421_LOOP_VOLTAGE_LOW) ||
        (error_code == AD5421_SPI_TRANSFER_ERROR) ||
        (error_code == AD5421_ACCESS_BUSY) ||
        (error_code == AD5421_OVERTEMP_SHUTDOWN) ||
        (error_code == AD5421_OVERTEMP_WARNING)) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 记录AO运行期硬件故障，不把后台故障升级为整机最终错误。
 *
 * @details 调用场景：初始化或诊断失败传NULL；写电流失败传本轮待发布快照。
 * @note 关键约束：保留最近成功下发电流，写失败时仍发布本轮过程输入和比例。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @param runtime_base 驱动访问失败前的 AO 运行态基线，用于保留目标值并叠加故障状态。
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @param error_code 待记录、转换或判断的错误码。该值是 AD5421 驱动或 AO 运行错误，函数按错误类别决定入队、恢复或诊断升级。
 * @param preserve_driver_ready true 表示记录错误后保留 AO 驱动就绪状态，false 表示同时将驱动标记为不可用。
 * @return 固定返回 NO_ERROR，使可延后恢复的 AD5421 运行期故障不升级为整机最终错误；真实驱动错误保存在 AO 运行态和诊断快照中。
 */
static uint32_t AoOutput_RecordRuntimeDriverError(const AoOutputConfig *config,
                                                   const AoOutputRuntime *runtime_base,
                                                   uint32_t now,
                                                   uint32_t error_code,
                                                   uint8_t preserve_driver_ready)
{
    AoOutputRuntime next;
    AoCurrentModeLimits limits = AoOutput_GetModeLimits(config->current_mode);

    if (runtime_base != NULL) {
        next = *runtime_base;
    } else {
        AoOutput_GetRuntimeSnapshot(&next);
    }
    if (preserve_driver_ready == 0U) {
        ao_output_driver_ready = 0U;
    }
    next.target_mA_x1000 = limits.fault_max_mA_x100 * 10U;
    next.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
    next.driver_fault_flags = AD5421_GetFaultFlags();
    next.driver_fault_register = AD5421_GetFaultRegister();
    next.last_update_tick = now;
    next.last_error_code = error_code;
    next.simulation_enabled = ao_simulation_enabled;
    next.dac_readback_mA_x100 = 0U;
    next.dac_readback_valid = 0U;
    next.update_counter++;
    AoOutput_CommitRuntime(&next);
    AoOutput_QueueDriverError(error_code);
    return NO_ERROR;
}

/**
 * @brief 按当前工作模式选择芯片初始化时应保持的禁用、固定或初始基础电流。
 *
 * @param config 只读 AO 输出配置；包含工作与电流模式、过程量来源、量程、修正、阻尼、故障动作、上电电流、定点输出和仿真电流等持久参数。
 * @return 返回 AD5421 初始化阶段使用的基础电流，单位 0.01 mA。
 */
static uint32_t AoOutput_GetInitialBaseCurrent(const AoOutputConfig *config)
{
    if (config->work_mode == AO_WORK_MODE_DISABLED) {
        return AO_DISABLED_CURRENT_MA_X100;
    }
    if (config->current_mode == AO_CURRENT_MODE_FIXED) {
        return config->fixed_current_mA_x100;
    }
    return config->power_on_current_mA_x100;
}

/**
 * @brief 按需初始化AD5421并缓存驱动可用状态。
 *
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @param allow_init 允许初始化。
 * @param initial_mA_x1000 AD5421 初始化完成后准备输出的起始电流，单位 0.001 mA。
 * @return NO_ERROR 表示 AD5421 已处于可写状态；初始化被禁用或无法完成时返回 AD5421_INIT_ERROR，其他值为复位、SPI、回读或器件诊断错误。
 */
static uint32_t AoOutput_EnsureDriverReady(uint32_t now,
                                           uint8_t allow_init,
                                           uint32_t initial_mA_x1000)
{
    uint32_t ret;

    if (ao_output_driver_ready != 0U) {
        return NO_ERROR;
    }
    if (allow_init == 0U) {
        return AD5421_INIT_ERROR;
    }
    if ((ao_output_driver_retry_valid != 0U) &&
        ((now - ao_output_last_driver_retry_tick) < AO_OUTPUT_DIAG_INTERVAL_MS)) {
        return AD5421_INIT_ERROR;
    }

    ao_output_driver_retry_valid = 1U;
    ao_output_last_driver_retry_tick = now;
    ret = AD5421_InitCurrentX1000(AoOutput_ClampHardwareCurrent(initial_mA_x1000));
    if (ret == NO_ERROR) {
        ao_output_driver_ready = 1U;
        ao_output_driver_retry_valid = 0U;
        ao_output_diag_valid = 0U;
        ao_output_last_diag_tick = 0U;
        ao_output_last_diag_error = NO_ERROR;
        AoOutput_ResetRecoverState();
        ao_output_last_recover_target_mA_x1000 = initial_mA_x1000;
        AoOutput_QueueDriverRecovery();
    } else if (AoOutput_IsOvertemperatureError(ret) != 0U) {
        /* 初始化已完成到末尾诊断，保留SPI诊断能力并等待温度故障清除。 */
        ao_output_driver_ready = 1U;
        ao_output_driver_retry_valid = 0U;
    }
    return ret;
}

/**
 * @brief 按节流周期轮询AD5421诊断。
 *
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @return 返回最近一次 AD5421 诊断对应的整机错误码；未到轮询周期时沿用上次结果。
 */
static uint32_t AoOutput_PollDiagnosticsThrottled(uint32_t now)
{
    if ((ao_output_diag_valid != 0U) &&
        ((now - ao_output_last_diag_tick) < AO_OUTPUT_DIAG_INTERVAL_MS)) {
        return ao_output_last_diag_error;
    }
    ao_output_diag_valid = 1U;
    ao_output_last_diag_tick = now;
    ao_output_last_diag_error = AD5421_PollDiagnostics();
    return ao_output_last_diag_error;
}

/**
 * @brief 目标变化立即写，目标不变每秒刷新，失败目标按周期重试。
 *
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @param target_mA_x1000 目标电流定点值，单位 0.001 mA。
 * @return 1 表示目标变化、刷新周期到期或失败目标需要重试；本周期无需写入时返回 0。
 */
static uint8_t AoOutput_ShouldWriteCurrent(uint32_t now, uint32_t target_mA_x1000)
{
    if (target_mA_x1000 != ao_output_runtime.last_sent_mA_x1000) {
        if ((ao_output_write_attempt_valid == 0U) ||
            (target_mA_x1000 != ao_output_last_write_attempt_mA_x1000)) {
            return 1U;
        }
        return ((now - ao_output_last_write_attempt_tick) >= AO_OUTPUT_REFRESH_INTERVAL_MS) ? 1U : 0U;
    }
    return ((now - ao_output_runtime.last_sent_tick) >= AO_OUTPUT_REFRESH_INTERVAL_MS) ? 1U : 0U;
}

/**
 * @brief 初始化AO并按当前配置写入禁用、固定或初始电流。
 *
 * @return NO_ERROR 表示 AO 已按禁用、固定或上电电流配置完成初始化；配置校验失败返回缓存的参数错误，AD5421 启动、写电流或诊断失败返回对应驱动错误。
 */
uint32_t AoOutput_Init(void)
{
    AoOutputConfig config;
    AoOutputRuntime next = {0};
    uint32_t now;
    uint32_t initial_base_mA_x100;
    uint32_t initial_target_mA_x1000;
    uint32_t ret;
    AoOutputSource initial_source;

    if (AoOutput_TryEnterUpdate() == 0U) {
        return ao_output_runtime.last_error_code;
    }
    now = HAL_GetTick();
    ao_output_initialized = 1U;
    ao_output_driver_ready = 0U;
    ao_output_driver_retry_valid = 0U;
    ao_output_diag_valid = 0U;
    ao_output_write_attempt_valid = 0U;
    ao_last_process_valid = 0U;
    ao_last_process_base_current_mA_x100 = 0U;
    ao_last_process_value_01mm = 0;
    ao_last_process_percent_x100 = 0;
    ao_config_snapshot_valid = 0U;
    ao_filter_initialized = 0U;
    memset(ao_process_samples, 0, sizeof(ao_process_samples));
    ao_simulation_enabled = 0U;

    AoOutput_GetConfigSnapshot(&config);
    AoOutput_HandleConfigTransition(&config);
    initial_base_mA_x100 = AoOutput_GetInitialBaseCurrent(&config);
    initial_source = (config.work_mode == AO_WORK_MODE_DISABLED) ?
                     AO_OUTPUT_SOURCE_DISABLED :
                     ((config.current_mode == AO_CURRENT_MODE_FIXED) ?
                      AO_OUTPUT_SOURCE_FIXED : AO_OUTPUT_SOURCE_INITIAL);
    initial_target_mA_x1000 = AoOutput_ApplyCurrentCorrection(&config,
                                                             initial_source,
                                                             initial_base_mA_x100);
    next.target_mA_x1000 = initial_target_mA_x1000;
    next.last_sent_mA_x1000 = 0U;
    next.source = (uint32_t)initial_source;
    next.last_update_tick = now;
    next.last_error_code = NO_ERROR;
    next.simulation_enabled = 0U;
    next.dac_readback_valid = 0U;

    ret = AoOutput_EnsureDriverReady(now, 1U, initial_target_mA_x1000);
    next.driver_fault_flags = AD5421_GetFaultFlags();
    next.driver_fault_register = AD5421_GetFaultRegister();
    next.last_error_code = ret;
    if (ret == NO_ERROR) {
        next.last_sent_mA_x1000 = initial_target_mA_x1000;
        next.last_sent_tick = now;
        next.update_counter = 1U;
        AoOutput_CommitRuntime(&next);
        AoOutput_LeaveUpdate();
        return NO_ERROR;
    }

    next.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
    next.update_counter = 1U;
    AoOutput_CommitRuntime(&next);
    AoOutput_QueueDriverError(ret);
    AoOutput_LeaveUpdate();
    return ret;
}

/**
 * @brief 执行AO固定优先级状态机并刷新AD5421。
 *
 * 函数在 AO 子系统尚未初始化时无操作返回；初始化后先取得一致的配置快照、处理配置切换，并按禁用或固定或上电初始来源计算经电流修正后的 AD5421 初始化目标。
 * 驱动未就绪时根据 allow_driver_init 决定是否执行初始化；随后限频读取诊断。非过温故障可按恢复间隔对最近成功目标执行一次 AD5421
 * 恢复，过温告警或关断只等待温度恢复，不周期复位器件。
 * 本轮基础目标严格按禁用、整机故障、仿真、固定电流、有效过程量的优先级选择；过程样本无效时优先保持最近一次已经成功写入的过程目标，没有历史值才回退到上电电流。
 * 目标选择阶段只产生未经修正的基础电流，随后统一执行一次零点或增益修正并钳位到硬件范围；只有变化量或刷新周期要求写入时才访问 AD5421。
 * 过程目标必须在 AD5421 写入成功后才进入保持缓存；写入或诊断失败仍发布本轮输入、比例、故障寄存器和最近成功电流到运行快照，但不会把失败目标伪装成已下发值。
 *
 * @param allow_driver_init 非 0 允许在 AD5421 尚未就绪时执行初始化并下发初始电流；0 禁止初始化，驱动未就绪时返回 AD5421_INIT_ERROR。
 * @return NO_ERROR 表示本轮无需动作或目标已保持或成功刷新；其他值为 AD5421 初始化、诊断恢复、目标选择或电流写入错误，并已同步记录到 AO 运行快照。
 * @note 本函数不自行获取 AO 重入锁；普通任务调用前通过 AoOutput_TryEnterUpdate，PendSV 延后刷新则在关中断临界区设置
 *       ao_output_update_busy，二者都必须先建立独占更新上下文。
 */
static uint32_t AoOutput_UpdateInternal(uint8_t allow_driver_init)
{
    AoOutputConfig config;
    AoOutputRuntime next;
    AoOutputSource source = AO_OUTPUT_SOURCE_INITIAL;
    uint32_t now;
    uint32_t initial_base_mA_x100;
    uint32_t initial_target_mA_x1000;
    uint32_t base_target_mA_x100 = 400U;
    uint32_t target_mA_x1000 = 4000U;
    uint32_t process_valid = 0U;
    uint32_t ret;
    uint32_t diag_ret;
    int32_t process_value_01mm = 0;
    int32_t percent_x100 = 0;
    uint8_t should_send;
    uint8_t process_write_succeeded = 0U;

    if (ao_output_initialized == 0U) {
        return NO_ERROR;
    }
    AoOutput_GetConfigSnapshot(&config);
    AoOutput_HandleConfigTransition(&config);
    now = HAL_GetTick();
    initial_base_mA_x100 = AoOutput_GetInitialBaseCurrent(&config);
    initial_target_mA_x1000 = AoOutput_ApplyCurrentCorrection(
            &config,
            (config.work_mode == AO_WORK_MODE_DISABLED) ?
            AO_OUTPUT_SOURCE_DISABLED :
            ((config.current_mode == AO_CURRENT_MODE_FIXED) ?
             AO_OUTPUT_SOURCE_FIXED : AO_OUTPUT_SOURCE_INITIAL),
            initial_base_mA_x100);
    ret = AoOutput_EnsureDriverReady(now, allow_driver_init, initial_target_mA_x1000);
    if (ret != NO_ERROR) {
        return AoOutput_RecordRuntimeDriverError(&config,
                                                  NULL,
                                                  now,
                                                  ret,
                                                  AoOutput_IsOvertemperatureError(ret));
    }

    diag_ret = AoOutput_PollDiagnosticsThrottled(now);
    if (diag_ret != NO_ERROR) {
        uint32_t recover_target_mA_x1000 = ao_output_runtime.last_sent_mA_x1000;
        uint32_t recover_ret = diag_ret;

        if (recover_target_mA_x1000 == 0U) {
            recover_target_mA_x1000 = initial_target_mA_x1000;
        }
        if ((AoOutput_IsDiagnosticRecoveryAllowed(diag_ret) != 0U) &&
            (AoOutput_ShouldRecover(now) != 0U)) {
            ao_output_recover_valid = 1U;
            ao_output_last_recover_tick = now;
            recover_ret = AD5421_RecoverCurrentX1000(recover_target_mA_x1000);
            if (recover_ret == NO_ERROR) {
                ao_output_driver_ready = 1U;
                ao_output_diag_valid = 0U;
                ao_output_last_diag_error = NO_ERROR;
                ao_output_write_attempt_valid = 0U;
                ao_output_last_recover_target_mA_x1000 = recover_target_mA_x1000;
                AoOutput_QueueDriverRecovery();
            }
        }
        if (recover_ret != NO_ERROR) {
            uint8_t preserve_driver_ready =
                    (AoOutput_IsOvertemperatureError(diag_ret) != 0U) ? 1U : 0U;

            return AoOutput_RecordRuntimeDriverError(&config,
                                                      NULL,
                                                      now,
                                                      recover_ret,
                                                      preserve_driver_ready);
        }
    }

    ret = AoOutput_SelectTarget(&config,
                                now,
                                &source,
                                &base_target_mA_x100,
                                &process_value_01mm,
                                &percent_x100,
                                &process_valid);
    if (ret != NO_ERROR) {
        return ret;
    }
    /* 选择阶段只产生基础目标；修正阶段统一执行一次，保持缓存也只保存基础值。 */
    target_mA_x1000 = AoOutput_ApplyCurrentCorrection(&config,
                                                      source,
                                                      base_target_mA_x100);
    target_mA_x1000 = AoOutput_ClampHardwareCurrent(target_mA_x1000);
    should_send = AoOutput_ShouldWriteCurrent(now, target_mA_x1000);
    if ((source == AO_OUTPUT_SOURCE_PROCESS) &&
        (ao_last_process_valid == 0U)) {
        /* 首个有效过程目标即使等于初始电流，也要明确写入成功后才能建立保持缓存。 */
        should_send = 1U;
    }

    AoOutput_GetRuntimeSnapshot(&next);
    next.target_mA_x1000 = target_mA_x1000;
    next.source = (uint32_t)source;
    next.driver_fault_flags = AD5421_GetFaultFlags();
    next.driver_fault_register = AD5421_GetFaultRegister();
    next.last_error_code = NO_ERROR;
    next.last_update_tick = now;
    next.process_value_01mm = process_value_01mm;
    next.percent_x100 = percent_x100;
    next.process_valid = process_valid;
    next.simulation_enabled = ao_simulation_enabled;
    next.dac_readback_mA_x100 = 0U;
    next.dac_readback_valid = 0U;

    if (should_send != 0U) {
        ret = AD5421_SetCurrentX1000(target_mA_x1000);
        ao_output_write_attempt_valid = 1U;
        ao_output_last_write_attempt_tick = now;
        ao_output_last_write_attempt_mA_x1000 = target_mA_x1000;
        next.driver_fault_flags = AD5421_GetFaultFlags();
        next.driver_fault_register = AD5421_GetFaultRegister();
        if (ret == NO_ERROR) {
            next.last_sent_mA_x1000 = target_mA_x1000;
            next.last_sent_tick = now;
            if (source == AO_OUTPUT_SOURCE_PROCESS) {
                process_write_succeeded = 1U;
            }
            ao_output_last_recover_target_mA_x1000 = target_mA_x1000;
            AoOutput_QueueDriverRecovery();
        } else {
            return AoOutput_RecordRuntimeDriverError(&config,
                                                      &next,
                                                      now,
                                                      ret,
                                                      AoOutput_IsOvertemperatureError(ret));
        }
    }

    if (process_write_succeeded != 0U) {
        /* 只缓存AD5421已确认成功接受的过程目标及其同轮输入快照。 */
        ao_last_process_base_current_mA_x100 = base_target_mA_x100;
        ao_last_process_value_01mm = process_value_01mm;
        ao_last_process_percent_x100 = percent_x100;
        ao_last_process_valid = 1U;
    }
    next.update_counter++;
    AoOutput_CommitRuntime(&next);
    return NO_ERROR;
}

/**
 * @brief 带重入保护地执行一次任务态AO刷新。
 *
 * @return 返回 AO 刷新结果码；可延后恢复的运行期驱动错误被记录后返回 NO_ERROR，其他失败原样传播。
 */
uint32_t AoOutput_Update(void)
{
    uint32_t ret;

    if (AoOutput_TryEnterUpdate() == 0U) {
        ret = ao_output_runtime.last_error_code;
        return (AoOutput_IsRuntimeDriverError(ret) != 0U) ? NO_ERROR : ret;
    }
    ret = AoOutput_UpdateInternal(1U);
    AoOutput_LeaveUpdate();
    return ret;
}

/**
 * @brief 暂停定时触发的 AO 自动刷新。
 *
 * @details 调用场景：串口 AO 正式测试直接访问 AD5421 前调用。
 * @note 关键约束：只影响 TIM4 请求和 PendSV 延后刷新，不影响继电器和前台 AO 调用。
 */
void AoOutput_SuspendTimerRefresh(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ao_output_timer_suspend_count++;
    __set_PRIMASK(primask);
}

/**
 * @brief 恢复定时触发的 AO 自动刷新。
 *
 * @details 调用场景：串口 AO 正式测试退出前调用。
 * @note 关键约束：按计数恢复；如果暂停期间已有请求，则重新挂起 PendSV。
 */
void AoOutput_ResumeTimerRefresh(void)
{
    uint32_t primask;
    uint8_t should_pend = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (ao_output_timer_suspend_count > 0U) {
        ao_output_timer_suspend_count--;
    }
    if ((ao_output_timer_suspend_count == 0U) &&
        (ao_output_timer_refresh_pending != 0U)) {
        should_pend = 1U;
    }
    __set_PRIMASK(primask);

    if (should_pend != 0U) {
        AoOutput_PendDeferredRefresh();
    }
}

/**
 * @brief 由 TIM4 中断请求一次 AO 延后刷新。
 *
 * @details 调用场景：TIM4_IRQHandler 在继电器刷新后调用。
 * @note 关键约束：只置位请求并挂起 PendSV，不访问 AD5421、不打印、不阻塞。
 */
void AoOutput_RequestTimerRefreshFromTim4Isr(void)
{
    if (ao_output_timer_suspend_count != 0U) {
        return;
    }

    ao_output_timer_refresh_pending = 1U;
    AoOutput_PendDeferredRefresh();
}

/**
 * @brief 处理 TIM4 请求的 AO 延后刷新。
 *
 * @details 调用场景：PendSV_Handler 最低优先级调用，补偿主循环阻塞时 AO 长时间不刷新。
 * @note 关键约束：会访问 AD5421 SPI；驱动内部打印被抑制，错误只记录到 AO 运行态和全局错误码。
 *
 * @return 返回本次延后刷新结果码；无待处理请求时返回上次运行态错误码，执行刷新时返回实际写入结果。
 */
uint32_t AoOutput_ProcessPendingTimerRefresh(void)
{
    uint32_t primask;
    uint32_t ret;
    uint8_t trace_suppressed;

    if (ao_output_timer_suspend_count != 0U) {
        return ao_output_runtime.last_error_code;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (ao_output_timer_refresh_pending == 0U) {
        __set_PRIMASK(primask);
        return ao_output_runtime.last_error_code;
    }
    if ((ao_output_update_busy != 0U) ||
        (AD5421_IsAccessBusy() != 0U)) {
        __set_PRIMASK(primask);
        return ao_output_runtime.last_error_code;
    }
    ao_output_timer_refresh_pending = 0U;
    ao_output_update_busy = 1U;
    __set_PRIMASK(primask);

    trace_suppressed = AD5421_SetTraceSuppressed(1U);
    ret = AoOutput_UpdateInternal(1U);
    (void)AD5421_SetTraceSuppressed(trace_suppressed);
    AoOutput_LeaveUpdate();

    if ((ret != NO_ERROR) &&
        (ret != STATE_SWITCH) &&
        (g_measurement.device_status.error_code == NO_ERROR)) {
        g_measurement.device_status.error_code = ret;
    }

    return ret;
}

/**
 * @brief 返回 AD5421 故障快照中的阶段中文名称。
 *
 * @param stage 外设故障现场的诊断阶段编号；用于区分参数检查、总线访问、寄存器读写、回读核对和设备状态检查等失败位置。
 * @return 返回 AD5421 故障快照中的阶段中文名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *AoOutput_GetDiagnosticStageText(uint8_t stage)
{
    switch (stage) {
    case AD5421_DIAG_STAGE_ACCESS_BUSY:
        return "访问冲突";
    case AD5421_DIAG_STAGE_SPI_WRITE:
        return "SPI写入";
    case AD5421_DIAG_STAGE_SPI_READ_COMMAND:
        return "SPI读命令";
    case AD5421_DIAG_STAGE_SPI_READ_DATA:
        return "SPI读数据";
    case AD5421_DIAG_STAGE_CONTROL_READBACK:
        return "控制寄存器核对";
    case AD5421_DIAG_STAGE_FAULT_STATUS:
        return "芯片故障状态";
    default:
        return "未记录";
    }
}

/**
 * @brief 返回 AD5421 故障快照中的访问方向中文名称。
 *
 * @param direction 外设故障现场中的访问方向枚举；写方向、读方向和未指定方向分别使用对应模块的诊断常量编码。
 * @return 返回 AD5421 故障快照中的访问方向中文名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *AoOutput_GetDiagnosticDirectionText(uint8_t direction)
{
    if (direction == AD5421_DIAG_DIRECTION_WRITE) {
        return "写";
    }
    if (direction == AD5421_DIAG_DIRECTION_READ) {
        return "读";
    }
    return "无";
}

/**
 * @brief 返回 HAL 外设访问状态的中文名称。
 *
 * @param hal_status 状态。
 * @return 返回 HAL 外设访问状态的中文名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *AoOutput_GetHalStatusText(uint32_t hal_status)
{
    switch (hal_status) {
    case (uint32_t)HAL_OK:
        return "正常";
    case (uint32_t)HAL_ERROR:
        return "访问错误";
    case (uint32_t)HAL_BUSY:
        return "总线忙";
    case (uint32_t)HAL_TIMEOUT:
        return "访问超时";
    default:
        return "未知状态";
    }
}

/**
 * @brief 在主循环任务态统一输出 AD5421 故障和恢复日志。
 *
 * @details 调用场景：App_MainLoop 每轮后台轻量检查阶段调用。
 * @note 关键约束：不得在 ISR 或 PendSV 中调用；持续相同故障不会重复刷屏。
 */
void AoOutput_ProcessDeferredDiagnostics(void)
{
    AD5421DiagnosticSnapshot error_snapshot = {0U};
    AD5421DiagnosticSnapshot recover_snapshot = {0U};
    uint32_t error_code = NO_ERROR;
    uint32_t recover_code = NO_ERROR;
    uint32_t primask;
    uint8_t error_pending;
    uint8_t recover_pending;
    char detail[256];

    primask = __get_PRIMASK();
    __disable_irq();
    error_pending = (uint8_t)ao_output_diag_error_pending;
    recover_pending = (uint8_t)ao_output_diag_recover_pending;
    if (error_pending != 0U) {
        error_snapshot = ao_output_diag_pending_error_snapshot;
        error_code = ao_output_diag_pending_error_code;
        ao_output_diag_error_pending = 0U;
    }
    if (recover_pending != 0U) {
        recover_snapshot = ao_output_diag_pending_recover_snapshot;
        recover_code = ao_output_diag_pending_recover_code;
        ao_output_diag_recover_pending = 0U;
    }
    __set_PRIMASK(primask);

    if (error_pending != 0U) {
        (void)snprintf(detail,
                       sizeof(detail),
                       "阶段=%s，方向=%s，寄存器=0x%02X，底层状态=%s(%lu)，根因码=%lu，故障寄存器=0x%04lX，故障标志=0x%08lX，期望值=0x%04lX，实际值=0x%04lX",
                       AoOutput_GetDiagnosticStageText(error_snapshot.stage),
                       AoOutput_GetDiagnosticDirectionText(error_snapshot.direction),
                       (unsigned int)error_snapshot.reg,
                       AoOutput_GetHalStatusText(error_snapshot.hal_status),
                       (unsigned long)error_snapshot.hal_status,
                       (unsigned long)error_snapshot.root_error_code,
                       (unsigned long)error_snapshot.fault_register,
                       (unsigned long)error_snapshot.fault_flags,
                       (unsigned long)error_snapshot.expected_value,
                       (unsigned long)error_snapshot.actual_value);
        ErrorLog_WarnDetail("模拟量输出",
                            "AD5421故障定位",
                            ErrorLog_GetReasonByCode(error_code),
                            "保持主流程并后台恢复",
                            detail);
    }

    if (recover_pending != 0U) {
        (void)snprintf(detail,
                       sizeof(detail),
                       "原故障=%s，原阶段=%s，寄存器=0x%02X，故障寄存器=0x%04lX",
                       ErrorLog_GetCodeName(recover_code),
                       AoOutput_GetDiagnosticStageText(recover_snapshot.stage),
                       (unsigned int)recover_snapshot.reg,
                       (unsigned long)recover_snapshot.fault_register);
        ErrorLog_RecoverDetail("模拟量输出",
                               "AD5421自动恢复",
                               "通信及芯片诊断恢复",
                               1U,
                               1U,
                               detail);
    }
}
/**
 * @brief 返回兼容旧调用点的只读运行态指针。
 *
 * @return 返回 ao_output_runtime 只读运行态地址；对象由模块静态持有，调用方不得修改或释放。
 */
const AoOutputRuntime *AoOutput_GetRuntime(void)
{
    return &ao_output_runtime;
}

/**
 * @brief HART读取最后一次成功下发的电流，而不是尚未写入的目标值。
 *
 * @return 返回运行快照中最后一次成功下发电流 last_sent_mA_x1000 除以 1000 后的值，单位 mA；未确认的目标电流不参与返回。
 */
float AoOutput_GetCurrent_mA(void)
{
    AoOutputRuntime snapshot;

    AoOutput_GetRuntimeSnapshot(&snapshot);
    return ((float)snapshot.last_sent_mA_x1000) / 1000.0f;
}

/**
 * @brief 返回真实百分数，例如50.00表示50%，不再返回0.5。
 *
 * @return 返回运行快照中 percent_x100 除以 100 后的真实百分数，例如内部 5000 返回 50.00%。
 */
float AoOutput_GetPercentOfRange(void)
{
    AoOutputRuntime snapshot;

    AoOutput_GetRuntimeSnapshot(&snapshot);
    return ((float)snapshot.percent_x100) / 100.0f;
}
