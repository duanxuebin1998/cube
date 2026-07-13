#include "ao_output.h"

#include "ad5421.h"
#include "error_log.h"
#include "main.h"
#include "system_parameter.h"
#include <stddef.h>
#include <stdio.h>

#define AO_OUTPUT_MIN_MA_X100          320U /* 4-20mA 模拟量输出参数：最小值 MA 放大 100 倍。 */
#define AO_OUTPUT_MAX_MA_X100          2400U /* 4-20mA 模拟量输出参数：最大值 MA 放大 100 倍。 */
#define AO_OUTPUT_NORMAL_MIN_MA_X100   400U /* 4-20mA 模拟量输出参数：正常 最小值 MA 放大 100 倍。 */
#define AO_OUTPUT_NORMAL_MAX_MA_X100   2000U /* 4-20mA 模拟量输出参数：正常 最大值 MA 放大 100 倍。 */
#define AO_OUTPUT_REFRESH_INTERVAL_MS  1000U /* 4-20mA 模拟量输出参数：刷新 间隔 毫秒。 */
#define AO_OUTPUT_DIAG_INTERVAL_MS     1000U /* 4-20mA 模拟量输出芯片诊断检查间隔，单位毫秒。 */
#define AO_OUTPUT_RECOVER_INTERVAL_MS  1000U /* AD5421 诊断异常后的最小恢复重试间隔，单位毫秒。 */

static AoOutputRuntime ao_output_runtime = {
    AO_OUTPUT_NORMAL_MIN_MA_X100,
    AO_OUTPUT_NORMAL_MIN_MA_X100,
    AO_OUTPUT_SOURCE_INIT,
    0U,
    0U,
    NO_ERROR,
    0U,
    0U,
    0U
};

static uint8_t ao_output_initialized = 0U;
static volatile uint8_t ao_output_update_busy = 0U;
static volatile uint8_t ao_output_timer_refresh_pending = 0U;
static volatile uint32_t ao_output_timer_suspend_count = 0U;
static uint8_t ao_output_driver_ready = 0U;
static uint8_t ao_output_driver_retry_valid = 0U;
static uint8_t ao_output_diag_valid = 0U;
static uint8_t ao_output_write_attempt_valid = 0U;
static uint32_t ao_output_last_driver_retry_tick = 0U;
static uint32_t ao_output_last_diag_tick = 0U;
static uint32_t ao_output_last_diag_error = NO_ERROR;
static uint8_t ao_output_recover_valid = 0U;
static uint32_t ao_output_last_recover_tick = 0U;
static uint32_t ao_output_last_write_attempt_tick = 0U;
static uint32_t ao_output_last_write_attempt_mA_x100 = 0U;
static uint32_t ao_output_last_recover_target_mA_x100 = 0U;
static volatile uint8_t ao_output_diag_error_pending = 0U;
static volatile uint8_t ao_output_diag_recover_pending = 0U;
static volatile uint8_t ao_output_diag_fault_active = 0U;
static uint32_t ao_output_diag_active_error_code = NO_ERROR;
static uint32_t ao_output_diag_pending_error_code = NO_ERROR;
static uint32_t ao_output_diag_pending_recover_code = NO_ERROR;
static AD5421DiagnosticSnapshot ao_output_diag_active_snapshot = {0U};
static AD5421DiagnosticSnapshot ao_output_diag_pending_error_snapshot = {0U};
static AD5421DiagnosticSnapshot ao_output_diag_pending_recover_snapshot = {0U};
static uint32_t AoOutput_UpdateInternal(uint8_t allow_driver_init);
static uint32_t AoOutput_NormalizeHardwareCurrent(uint32_t current_mA_x100);

/*
 * 函数用途：判断两次 AD5421 故障是否属于同一故障现场。
 * 调用场景：AO 诊断日志入队前抑制持续故障的重复打印。
 * 关键约束：忽略快照序号，只比较实际定位信息。
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

/*
 * 函数用途：把 AD5421 故障快照放入主循环延后日志槽。
 * 调用场景：AO 初始化、诊断、恢复或写电流失败后调用。
 * 关键约束：只复制数值并置位，不打印；相同持续故障只保留一次。
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

/*
 * 函数用途：记录 AD5421 故障已恢复，交由主循环统一打印。
 * 调用场景：驱动重新初始化、恢复序列或正常电流写入成功后调用。
 * 关键约束：只有之前记录过有效故障时才生成恢复日志。
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
/*
 * 函数用途：挂起最低优先级 AO 延后刷新。
 * 调用场景：TIM4 请求 AO 刷新或测试流程恢复自动刷新时调用。
 * 关键约束：只设置 PendSV 挂起位，不访问 SPI、不打印、不阻塞。
 */
static void AoOutput_PendDeferredRefresh(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}

/*
 * 函数用途：尝试占用 AO 更新窗口。
 * 调用场景：前台刷新和 PendSV 延后刷新进入 AO 服务前调用。
 * 关键约束：只用短临界区保护标志位，不在临界区内访问 SPI。
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

/*
 * 函数用途：释放 AO 更新窗口。
 * 调用场景：前台刷新和 PendSV 延后刷新退出 AO 服务时调用。
 * 关键约束：只清保护标志，真实错误状态已在调用链里保存。
 */
static void AoOutput_LeaveUpdate(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ao_output_update_busy = 0U;
    __set_PRIMASK(primask);
}
/*
 * 函数用途：判断 AO 输出功能是否被参数使能。
 * 调用场景：AO 初始化和周期更新入口。
 * 关键约束：只读取参数，不访问 AD5421，未接电流环时可安全调用。
 */
static uint8_t AoOutput_IsEnabled(void)
{
    return (g_deviceParams.AoOutputEnable == 0U) ? 0U : 1U;
}

/*
 * 函数用途：清除 AD5421 诊断恢复节流状态。
 * 调用场景：AO 初始化、禁用或驱动状态重建时调用。
 * 关键约束：只更新本模块恢复状态，不访问 AD5421。
 */
static void AoOutput_ResetRecoverState(void)
{
    ao_output_recover_valid = 0U;
    ao_output_last_recover_tick = 0U;
    ao_output_last_recover_target_mA_x100 = 0U;
}

/*
 * 函数用途：判断当前是否允许执行一次 AD5421 恢复序列。
 * 调用场景：诊断发现异常后，在写入故障电流前尝试恢复。
 * 关键约束：使用 HAL tick 差值判断节流窗口，允许计数回绕。
 */
static uint8_t AoOutput_ShouldRecover(uint32_t now)
{
    if (ao_output_recover_valid == 0U) {
        return 1U;
    }

    return ((now - ao_output_last_recover_tick) >= AO_OUTPUT_RECOVER_INTERVAL_MS) ? 1U : 0U;
}
/*
 * 函数用途：判断 AO 运行期驱动错误是否只记录到 AO 运行态。
 * 调用场景：自动刷新或测量刷新遇到 AD5421 诊断、恢复、写入异常后调用。
 * 关键约束：启动初始化失败仍由 AoOutput_Init() 原样上报，运行期错误不拉起整机最终错误。
 */
static uint8_t AoOutput_IsRuntimeDriverError(uint32_t error_code)
{
    if ((error_code == AD5421_INIT_ERROR) ||
        (error_code == AD5421_WRITE_CURRENT_ERROR) ||
        (error_code == AD5421_FAULT_PIN_ERROR) ||
        (error_code == AD5421_READFAULT_ERROR) ||
        (error_code == AD5421_FAULT_STATUS_ERROR) ||
        (error_code == AD5421_READBACK_ERROR)) {
        return 1U;
    }

    return 0U;
}

/*
 * 函数用途：记录 AO 运行期 AD5421 驱动错误并保持服务返回成功。
 * 调用场景：AO 已初始化后，后台或测量刷新发现 AD5421 暂时不可用。
 * 关键约束：错误保存在 AO 运行态和驱动故障标志中，不写全局错误码。
 */
static uint32_t AoOutput_RecordRuntimeDriverError(uint32_t now, uint32_t error_code)
{
    ao_output_driver_ready = 0U;
    ao_output_runtime.target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
    ao_output_runtime.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
    ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
    ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();
    ao_output_runtime.last_update_tick = now;
    ao_output_runtime.last_error_code = error_code;
    ao_output_runtime.update_counter++;
    AoOutput_QueueDriverError(error_code);

    return NO_ERROR;
}

/*
 * 函数用途：把 AO 运行态更新为软件关闭状态。
 * 调用场景：AO 输出未使能或运行中被关闭时调用。
 * 关键约束：不访问 AD5421，不做阻塞操作，避免未接电流环触发诊断故障。
 */
static void AoOutput_SetDisabledRuntime(uint32_t now)
{
    ao_output_runtime.target_mA_x100 = 0U;
    ao_output_runtime.last_sent_mA_x100 = 0U;
    ao_output_runtime.source = AO_OUTPUT_SOURCE_DISABLED;
    ao_output_runtime.driver_fault_flags = 0U;
    ao_output_runtime.driver_fault_register = 0U;
    ao_output_runtime.last_error_code = NO_ERROR;
    ao_output_runtime.last_update_tick = now;
    ao_output_runtime.last_sent_tick = 0U;
    ao_output_runtime.update_counter++;
}

/*
 * 函数用途：按需初始化 AD5421，并缓存驱动可用状态。
 * 调用场景：AO 输出已使能时，由初始化和周期更新路径调用。
 * 关键约束：会访问 SPI 和 AD5421 诊断寄存器，不应在中断中调用。
 */
static uint32_t AoOutput_EnsureDriverReady(uint32_t now, uint8_t allow_init)
{
    uint32_t ret;

    if (ao_output_driver_ready != 0U) {
        return NO_ERROR;
    }

    if (allow_init == 0U) {
        ret = ao_output_runtime.last_error_code;
        if (ret == NO_ERROR) {
            ret = AD5421_INIT_ERROR;
        }
        ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
        ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();
        ao_output_runtime.last_update_tick = now;
        ao_output_runtime.last_error_code = ret;
        return ret;
    }

    if ((ao_output_driver_retry_valid != 0U) &&
        ((now - ao_output_last_driver_retry_tick) < AO_OUTPUT_DIAG_INTERVAL_MS)) {
        return ao_output_runtime.last_error_code;
    }

    ao_output_driver_retry_valid = 1U;
    ao_output_last_driver_retry_tick = now;
    ret = Ad5421Init();
    ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
    ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();
    ao_output_runtime.last_update_tick = now;
    ao_output_runtime.last_error_code = ret;
    if (ret == NO_ERROR) {
        ao_output_driver_ready = 1U;
        ao_output_driver_retry_valid = 0U;
        ao_output_diag_valid = 0U;
        ao_output_last_diag_tick = 0U;
        ao_output_last_diag_error = NO_ERROR;
        AoOutput_ResetRecoverState();
        ao_output_last_recover_target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.InitialCurrent_mA);
        AoOutput_QueueDriverRecovery();
    }

    return ret;
}

/*
 * 函数用途：按节流周期轮询 AD5421 诊断。
 * 调用场景：AO 已使能且驱动已初始化后的周期刷新。
 * 关键约束：诊断访问 SPI，故障持续时不在每轮主循环阻塞访问硬件。
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

/*
 * 函数用途：判断本轮是否需要写入 AD5421 电流。
 * 调用场景：AO 目标电流计算完成后，进入硬件写入前调用。
 * 关键约束：目标变化立即写；目标不变且上次写失败时按周期重试，避免主循环持续阻塞。
 */
static uint8_t AoOutput_ShouldWriteCurrent(uint32_t now, uint32_t target_mA_x100)
{
    if (target_mA_x100 != ao_output_runtime.last_sent_mA_x100) {
        if ((ao_output_write_attempt_valid == 0U) ||
            (target_mA_x100 != ao_output_last_write_attempt_mA_x100)) {
            return 1U;
        }
        return ((now - ao_output_last_write_attempt_tick) >= AO_OUTPUT_REFRESH_INTERVAL_MS) ? 1U : 0U;
    }

    return ((now - ao_output_runtime.last_sent_tick) >= AO_OUTPUT_REFRESH_INTERVAL_MS) ? 1U : 0U;
}

/*
 * 函数用途：将无符号数限制在指定上下限内。
 * 调用场景：AO 电流参数归一化和液位映射计算。
 * 关键约束：纯计算函数，不访问外设和全局运行态。
 */
static uint32_t AoOutput_ClampU32(uint32_t value, uint32_t min, uint32_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

/*
 * 函数用途：按 AD5421 硬件允许范围归一化电流。
 * 调用场景：写入 AD5421 前统一限制输出电流。
 * 关键约束：单位为 0.01mA，只做纯计算。
 */
static uint32_t AoOutput_NormalizeHardwareCurrent(uint32_t current_mA_x100)
{
    return AoOutput_ClampU32(current_mA_x100, AO_OUTPUT_MIN_MA_X100, AO_OUTPUT_MAX_MA_X100);
}

/*
 * 函数用途：按正常 4-20mA 业务量程归一化电流参数。
 * 调用场景：液位线性映射使用起点/终点电流前。
 * 关键约束：单位为 0.01mA，只做纯计算。
 */
static uint32_t AoOutput_NormalizeNormalCurrent(uint32_t current_mA_x100)
{
    return AoOutput_ClampU32(current_mA_x100,
                             AO_OUTPUT_NORMAL_MIN_MA_X100,
                             AO_OUTPUT_NORMAL_MAX_MA_X100);
}

/*
 * 函数用途：判断当前液位是否可用于 AO 线性输出。
 * 调用场景：选择 AO 输出来源时过滤无效液位。
 * 关键约束：只读取测量结果，不改变测量状态。
 */
static uint8_t AoOutput_LevelIsValid(uint32_t level_01mm)
{
    if (level_01mm == UNVALID_LEVEL) {
        return 0U;
    }
    if ((g_measurement.oil_measurement.probe_at_liquid_level == 0U) &&
        (g_measurement.oil_measurement.liquid_stable == 0U)) {
        return 0U;
    }
    return 1U;
}

/*
 * 函数用途：判断当前设备是否处于调试模式。
 * 调用场景：AO 输出来源选择，调试模式优先输出调试电流。
 * 关键约束：只读取设备状态，不切换状态机。
 */
static uint8_t AoOutput_IsDebugState(void)
{
    return (g_measurement.device_status.device_state == STATE_DEBUG_MODE) ? 1U : 0U;
}
/*
 * 函数用途：选择 AO 正常液位电流换算使用的液位量程。
 * 调用场景：液位有效并准备计算 4-20mA 目标电流前调用。
 * 关键约束：AOStartLevel_01mm/AOEndLevel_01mm 复用为 起点/终点液位，不改变参数存储结构大小。
 */
static uint32_t AoOutput_GetLevelRange(uint32_t *level_min_01mm, uint32_t *level_max_01mm)
{
    if ((level_min_01mm == NULL) || (level_max_01mm == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    *level_min_01mm = g_deviceParams.AOStartLevel_01mm;
    *level_max_01mm = g_deviceParams.AOEndLevel_01mm;
    if (*level_max_01mm <= *level_min_01mm) {
        *level_min_01mm = 0U;
        *level_max_01mm = g_deviceParams.tankHeight;
        if (*level_max_01mm == 0U) {
            *level_max_01mm = 1U;
        }
    }

    return NO_ERROR;
}

/*
 * 函数用途：根据液位和量程参数计算 AO 目标电流。
 * 调用场景：液位有效且无故障/调试优先级时调用。
 * 关键约束：只做线性换算，不写 AD5421；参数异常时返回错误供上层转故障电流。
 */
static uint32_t AoOutput_CalculateLevelCurrent(uint32_t level_01mm, uint32_t *target_mA_x100)
{
    uint32_t start_mA_x100 = g_deviceParams.CurrentRangeStart_mA;
    uint32_t end_mA_x100 = g_deviceParams.CurrentRangeEnd_mA;
    uint32_t level_min_01mm;
    uint32_t level_max_01mm;
    uint32_t level_span_01mm;
    int32_t current_delta_mA_x100;
    uint64_t offset_01mm;
    int64_t scaled;
    int64_t result_mA_x100;
    uint32_t ret;

    if (target_mA_x100 == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    start_mA_x100 = AoOutput_NormalizeNormalCurrent(start_mA_x100);
    end_mA_x100 = AoOutput_NormalizeNormalCurrent(end_mA_x100);
    *target_mA_x100 = start_mA_x100;

    ret = AoOutput_GetLevelRange(&level_min_01mm, &level_max_01mm);
    if (ret != NO_ERROR) {
        return ret;
    }

    if (end_mA_x100 == start_mA_x100) {
        start_mA_x100 = AO_OUTPUT_NORMAL_MIN_MA_X100;
        end_mA_x100 = AO_OUTPUT_NORMAL_MAX_MA_X100;
        *target_mA_x100 = start_mA_x100;
    }

    if (level_01mm <= level_min_01mm) {
        *target_mA_x100 = start_mA_x100;
        return NO_ERROR;
    }
    if (level_01mm >= level_max_01mm) {
        *target_mA_x100 = end_mA_x100;
        return NO_ERROR;
    }

    level_span_01mm = level_max_01mm - level_min_01mm;
    current_delta_mA_x100 = (int32_t)end_mA_x100 - (int32_t)start_mA_x100;
    offset_01mm = (uint64_t)(level_01mm - level_min_01mm);
    scaled = (int64_t)offset_01mm * (int64_t)current_delta_mA_x100;
    if (scaled >= 0) {
        scaled += (int64_t)(level_span_01mm / 2U);
    } else {
        scaled -= (int64_t)(level_span_01mm / 2U);
    }
    scaled /= (int64_t)level_span_01mm;
    result_mA_x100 = (int64_t)start_mA_x100 + scaled;

    if (result_mA_x100 < (int64_t)AO_OUTPUT_MIN_MA_X100) {
        *target_mA_x100 = AO_OUTPUT_MIN_MA_X100;
    } else if (result_mA_x100 > (int64_t)AO_OUTPUT_MAX_MA_X100) {
        *target_mA_x100 = AO_OUTPUT_MAX_MA_X100;
    } else {
        *target_mA_x100 = (uint32_t)result_mA_x100;
    }

    return NO_ERROR;
}

/*
 * 函数用途：根据独立 AO 高低报警液位覆盖目标电流。
 * 调用场景：正常液位电流计算完成后，输出前处理 AO 报警优先级。
 * 关键约束：不读取继电器报警状态，AO 报警阈值与继电器阈值相互独立。
 */
static void AoOutput_ApplyAlarm(uint32_t level_01mm, uint32_t *target_mA_x100, AoOutputSource *source)
{
    uint32_t high_alarm_01mm = g_deviceParams.AlarmHighAO;
    uint32_t low_alarm_01mm = g_deviceParams.AlarmLowAO;
    uint32_t tank_height_01mm = g_deviceParams.tankHeight;

    if ((target_mA_x100 == NULL) || (source == NULL)) {
        return;
    }

    if (tank_height_01mm != 0U) {
        if (high_alarm_01mm > tank_height_01mm) {
            high_alarm_01mm = 0U;
        }
        if (low_alarm_01mm > tank_height_01mm) {
            low_alarm_01mm = 0U;
        }
    }

    if ((high_alarm_01mm != 0U) &&
        (low_alarm_01mm != 0U) &&
        (low_alarm_01mm >= high_alarm_01mm)) {
        return;
    }

    if ((low_alarm_01mm != 0U) && (level_01mm <= low_alarm_01mm)) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.AOLowCurrent_mA);
        *source = AO_OUTPUT_SOURCE_ALARM_LOW;
    } else if ((high_alarm_01mm != 0U) && (level_01mm >= high_alarm_01mm)) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.AOHighCurrent_mA);
        *source = AO_OUTPUT_SOURCE_ALARM_HIGH;
    }
}

/*
 * 函数用途：按故障、调试、液位和报警优先级选择 AO 目标电流。
 * 调用场景：AO 周期更新中，驱动诊断正常后调用。
 * 关键约束：只计算目标值和来源，不直接写 AD5421。
 */
static uint32_t AoOutput_SelectTarget(AoOutputSource *source, uint32_t *target_mA_x100)
{
    uint32_t level_01mm;
    uint32_t ret;

    if ((source == NULL) || (target_mA_x100 == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.InitialCurrent_mA);

    if ((g_measurement.device_status.error_code != NO_ERROR) &&
        (g_measurement.device_status.error_code != STATE_SWITCH)) {
        *source = AO_OUTPUT_SOURCE_FAULT;
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
        return NO_ERROR;
    }

    if (AoOutput_IsDebugState() != 0U) {
        *source = AO_OUTPUT_SOURCE_DEBUG;
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.DebugCurrent_mA);
        return NO_ERROR;
    }

    level_01mm = g_measurement.oil_measurement.oil_level;
    if (AoOutput_LevelIsValid(level_01mm) != 0U) {
        *source = AO_OUTPUT_SOURCE_LEVEL;
        ret = AoOutput_CalculateLevelCurrent(level_01mm, target_mA_x100);
        if (ret != NO_ERROR) {
            *source = AO_OUTPUT_SOURCE_FAULT;
            *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
            return ret;
        }
        AoOutput_ApplyAlarm(level_01mm, target_mA_x100, source);
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(*target_mA_x100);
        return NO_ERROR;
    }

    *source = AO_OUTPUT_SOURCE_INIT;
    *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.InitialCurrent_mA);
    return NO_ERROR;
}

/*
 * 函数用途：选择 AD5421 恢复后应写回的目标电流。
 * 调用场景：诊断异常触发恢复序列前调用。
 * 关键约束：优先使用当前有效目标，其次使用上次成功输出，最后回退到初始电流。
 */
static void AoOutput_GetRecoverTarget(AoOutputSource *source, uint32_t *target_mA_x100)
{
    uint32_t select_ret;

    if ((source == NULL) || (target_mA_x100 == NULL)) {
        return;
    }

    select_ret = AoOutput_SelectTarget(source, target_mA_x100);
    if (select_ret == NO_ERROR) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(*target_mA_x100);
        return;
    }

    *source = (AoOutputSource)ao_output_runtime.source;
    if ((*source == AO_OUTPUT_SOURCE_DISABLED) ||
        (*source == AO_OUTPUT_SOURCE_DRIVER_ERROR)) {
        *source = AO_OUTPUT_SOURCE_INIT;
    }

    if (ao_output_last_recover_target_mA_x100 != 0U) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(ao_output_last_recover_target_mA_x100);
    } else if (ao_output_runtime.last_sent_mA_x100 != 0U) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(ao_output_runtime.last_sent_mA_x100);
    } else {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.InitialCurrent_mA);
    }
}
/*
 * 函数用途：初始化 AO 输出服务并发布初始运行态。
 * 调用场景：系统参数加载后由主流程调用。
 * 关键约束：AO 未使能时不访问 AD5421；AO 使能时会走 SPI 初始化和诊断，不应在中断中调用。
 */
uint32_t AoOutput_Init(void)
{
    uint32_t ret;
    uint32_t now;

    if (AoOutput_TryEnterUpdate() == 0U) {
        return ao_output_runtime.last_error_code;
    }

    ao_output_initialized = 1U;
    ao_output_driver_ready = 0U;
    now = HAL_GetTick();
    if (AoOutput_IsEnabled() == 0U) {
        AoOutput_SetDisabledRuntime(now);
        AoOutput_LeaveUpdate();
        return NO_ERROR;
    }

    ret = AoOutput_EnsureDriverReady(now, 1U);
    ao_output_runtime.target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.InitialCurrent_mA);
    ao_output_runtime.last_sent_mA_x100 = 0U;
    ao_output_runtime.source = AO_OUTPUT_SOURCE_INIT;
    ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
    ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();
    ao_output_runtime.last_error_code = ret;
    ao_output_runtime.update_counter = 0U;
    ao_output_runtime.last_update_tick = now;
    ao_output_runtime.last_sent_tick = 0U;

    if (ret != NO_ERROR) {
        ao_output_runtime.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
        ao_output_runtime.target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
        AoOutput_QueueDriverError(ret);
        AoOutput_LeaveUpdate();
        return ret;
    }

    ret = AoOutput_UpdateInternal(1U);
    AoOutput_LeaveUpdate();
    return ret;
}

/*
 * 函数用途：刷新 AO 目标电流并写入 AD5421。
 * 调用场景：AO 初始化、前台刷新和 PendSV 延后刷新共用内部流程。
 * 关键约束：调用方必须先持有 AO 更新窗口；由 PendSV 调用时会抑制驱动内部打印。
 */
static uint32_t AoOutput_UpdateInternal(uint8_t allow_driver_init)
{
    AoOutputSource source = AO_OUTPUT_SOURCE_INIT;
    uint32_t target_mA_x100;
    uint32_t now;
    uint32_t ret = NO_ERROR;
    uint32_t select_ret = NO_ERROR;
    uint32_t diag_ret;
    uint8_t should_send = 0U;

    if (ao_output_initialized == 0U) {
        return NO_ERROR;
    }

    now = HAL_GetTick();
    if (AoOutput_IsEnabled() == 0U) {
        ao_output_driver_ready = 0U;
        ao_output_driver_retry_valid = 0U;
        ao_output_diag_valid = 0U;
        ao_output_write_attempt_valid = 0U;
        ao_output_last_driver_retry_tick = 0U;
        ao_output_last_diag_tick = 0U;
        ao_output_last_diag_error = NO_ERROR;
        ao_output_last_write_attempt_tick = 0U;
        ao_output_last_write_attempt_mA_x100 = 0U;
        AoOutput_ResetRecoverState();
        AoOutput_SetDisabledRuntime(now);
        return NO_ERROR;
    }

    ret = AoOutput_EnsureDriverReady(now, allow_driver_init);
    if (ret != NO_ERROR) {
        return AoOutput_RecordRuntimeDriverError(now, ret);
    }

    diag_ret = AoOutput_PollDiagnosticsThrottled(now);
    if (diag_ret != NO_ERROR) {
        uint32_t recover_ret = diag_ret;

        if (AoOutput_ShouldRecover(now) != 0U) {
            AoOutputSource recover_source = source;
            uint32_t recover_target_mA_x100;

            AoOutput_GetRecoverTarget(&recover_source, &recover_target_mA_x100);
            ao_output_recover_valid = 1U;
            ao_output_last_recover_tick = now;
            recover_ret = AD5421_RecoverCurrentX100(recover_target_mA_x100);
            ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
            ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();

            if (recover_ret == NO_ERROR) {
                ao_output_diag_valid = 0U;
                ao_output_last_diag_error = NO_ERROR;
                ao_output_write_attempt_valid = 0U;
                ao_output_last_recover_target_mA_x100 = recover_target_mA_x100;
                ao_output_runtime.target_mA_x100 = recover_target_mA_x100;
                ao_output_runtime.last_sent_mA_x100 = recover_target_mA_x100;
                ao_output_runtime.source = (uint32_t)recover_source;
                ao_output_runtime.last_update_tick = now;
                ao_output_runtime.last_sent_tick = now;
                ao_output_runtime.last_error_code = NO_ERROR;
                ao_output_runtime.update_counter++;
                AoOutput_QueueDriverRecovery();
                return NO_ERROR;
            }

            ao_output_diag_valid = 1U;
            ao_output_last_diag_tick = now;
            ao_output_last_diag_error = recover_ret;
            if (recover_ret != AD5421_FAULT_STATUS_ERROR) {
                return AoOutput_RecordRuntimeDriverError(now, recover_ret);
            }
        }

        source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
        target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
        ret = recover_ret;
    } else {
        select_ret = AoOutput_SelectTarget(&source, &target_mA_x100);
        if (select_ret != NO_ERROR) {
            ret = select_ret;
        }
    }

    should_send = AoOutput_ShouldWriteCurrent(now, target_mA_x100);

    ao_output_runtime.target_mA_x100 = target_mA_x100;
    ao_output_runtime.source = (uint32_t)source;
    ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
    ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();
    ao_output_runtime.last_update_tick = now;
    ao_output_runtime.update_counter++;
    ao_output_runtime.last_error_code = ret;

    if (should_send != 0U) {
        uint32_t write_ret = AD5421_SetCurrentX100(target_mA_x100);
        ao_output_write_attempt_valid = 1U;
        ao_output_last_write_attempt_tick = now;
        ao_output_last_write_attempt_mA_x100 = target_mA_x100;
        ao_output_runtime.driver_fault_flags = AD5421_GetFaultFlags();
        ao_output_runtime.driver_fault_register = AD5421_GetFaultRegister();
        if (write_ret == NO_ERROR) {
            ao_output_runtime.last_sent_mA_x100 = target_mA_x100;
            ao_output_runtime.last_sent_tick = now;
            if (source != AO_OUTPUT_SOURCE_DRIVER_ERROR) {
                ao_output_last_recover_target_mA_x100 = target_mA_x100;
                AoOutput_QueueDriverRecovery();
            }
        } else {
            ret = write_ret;
            target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
            ao_output_runtime.target_mA_x100 = target_mA_x100;
            ao_output_runtime.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
        }
        ao_output_runtime.last_error_code = ret;
    }
    if (AoOutput_IsRuntimeDriverError(ret) != 0U) {
        AoOutput_QueueDriverError(ret);
        return NO_ERROR;
    }

    return ret;
}

/*
 * 函数用途：刷新 AO 目标电流并写入 AD5421，带重入保护。
 * 调用场景：前台流程或测量流程需要主动刷新 AO 时调用。
 * 关键约束：内部会访问 SPI/GPIO；如果 PendSV 正在刷新，则返回上一次 AO 状态。
 */
uint32_t AoOutput_Update(void)
{
    uint32_t ret;

    if (AoOutput_TryEnterUpdate() == 0U) {
        ret = ao_output_runtime.last_error_code;
        if (AoOutput_IsRuntimeDriverError(ret) != 0U) {
            return NO_ERROR;
        }
        return ret;
    }

    ret = AoOutput_UpdateInternal(1U);
    AoOutput_LeaveUpdate();

    return ret;
}

/*
 * 函数用途：暂停定时触发的 AO 自动刷新。
 * 调用场景：串口 AO 正式测试直接访问 AD5421 前调用。
 * 关键约束：只影响 TIM4 请求和 PendSV 延后刷新，不影响继电器和前台 AO 调用。
 */
void AoOutput_SuspendTimerRefresh(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ao_output_timer_suspend_count++;
    __set_PRIMASK(primask);
}

/*
 * 函数用途：恢复定时触发的 AO 自动刷新。
 * 调用场景：串口 AO 正式测试退出前调用。
 * 关键约束：按计数恢复；如果暂停期间已有请求，则重新挂起 PendSV。
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

/*
 * 函数用途：由 TIM4 中断请求一次 AO 延后刷新。
 * 调用场景：TIM4_IRQHandler 在继电器刷新后调用。
 * 关键约束：只置位请求并挂起 PendSV，不访问 AD5421、不打印、不阻塞。
 */
void AoOutput_RequestTimerRefreshFromTim4Isr(void)
{
    if (ao_output_timer_suspend_count != 0U) {
        return;
    }

    ao_output_timer_refresh_pending = 1U;
    AoOutput_PendDeferredRefresh();
}

/*
 * 函数用途：处理 TIM4 请求的 AO 延后刷新。
 * 调用场景：PendSV_Handler 最低优先级调用，补偿主循环阻塞时 AO 长时间不刷新。
 * 关键约束：会访问 AD5421 SPI；驱动内部打印被抑制，错误只记录到 AO 运行态和全局错误码。
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

/* 返回 AD5421 故障快照中的阶段中文名称。 */
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

/* 返回 AD5421 故障快照中的访问方向中文名称。 */
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

/* 返回 HAL 外设访问状态的中文名称。 */
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

/*
 * 函数用途：在主循环任务态统一输出 AD5421 故障和恢复日志。
 * 调用场景：App_MainLoop 每轮后台轻量检查阶段调用。
 * 关键约束：不得在 ISR 或 PendSV 中调用；持续相同故障不会重复刷屏。
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
/*
 * 函数用途：返回 AO 运行态只读指针。
 * 调用场景：Modbus 输入寄存器、HART 和调试查看。
 * 关键约束：调用方不得通过返回指针修改运行态数据。
 */
const AoOutputRuntime *AoOutput_GetRuntime(void)
{
    return &ao_output_runtime;
}

/*
 * 函数用途：返回当前 AO 目标电流的人读 mA 值。
 * 调用场景：HART 电流响应或调试显示。
 * 关键约束：只读取运行态，不触发输出刷新。
 */
float AoOutput_GetCurrent_mA(void)
{
    return ((float)ao_output_runtime.target_mA_x100) / 100.0f;
}

/*
 * 函数用途：按 4-20mA 标准量程计算当前 AO 百分比。
 * 调用场景：HART Command 2/3 响应电流百分比。
 * 关键约束：结果钳位在 0..1，不访问 AD5421。
 */
float AoOutput_GetPercentOfRange(void)
{
    float percent;
    float current_mA;

    current_mA = AoOutput_GetCurrent_mA();
    percent = (current_mA - 4.0f) / 16.0f;
    if (percent < 0.0f) {
        percent = 0.0f;
    }
    if (percent > 1.0f) {
        percent = 1.0f;
    }

    return percent;
}
