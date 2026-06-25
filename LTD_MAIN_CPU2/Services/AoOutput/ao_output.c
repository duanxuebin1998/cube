#include "ao_output.h"

#include "ad5421.h"
#include "main.h"
#include "../Relay/relay_output.h"
#include "system_parameter.h"
#include <stddef.h>

#define AO_OUTPUT_MIN_MA_X100          320U /* 4-20mA 模拟量输出参数：最小值 MA 放大 100 倍。 */
#define AO_OUTPUT_MAX_MA_X100          2400U /* 4-20mA 模拟量输出参数：最大值 MA 放大 100 倍。 */
#define AO_OUTPUT_NORMAL_MIN_MA_X100   400U /* 4-20mA 模拟量输出参数：正常 最小值 MA 放大 100 倍。 */
#define AO_OUTPUT_NORMAL_MAX_MA_X100   2000U /* 4-20mA 模拟量输出参数：正常 最大值 MA 放大 100 倍。 */
#define AO_OUTPUT_REFRESH_INTERVAL_MS  1000U /* 4-20mA 模拟量输出参数：刷新 间隔 毫秒。 */
#define AO_OUTPUT_DIAG_INTERVAL_MS     1000U /* 4-20mA 模拟量输出参数：诊断 间隔 毫秒。 */

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
static uint32_t ao_output_last_write_attempt_tick = 0U;
static uint32_t ao_output_last_write_attempt_mA_x100 = 0U;
static uint32_t AoOutput_UpdateInternal(uint8_t allow_driver_init);
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
 * 函数用途：根据液位和量程参数计算 AO 目标电流。
 * 调用场景：液位有效且无故障/调试优先级时调用。
 * 关键约束：只做线性换算，不写 AD5421；参数异常时返回错误供上层转故障电流。
 */
static uint32_t AoOutput_CalculateLevelCurrent(uint32_t level_01mm, uint32_t *target_mA_x100)
{
    uint32_t start_mA_x100 = g_deviceParams.CurrentRangeStart_mA;
    uint32_t end_mA_x100 = g_deviceParams.CurrentRangeEnd_mA;
    uint32_t level_min_01mm = g_deviceParams.blindZone;
    uint32_t level_max_01mm = g_deviceParams.tankHeight;
    uint32_t level_span_01mm;
    int32_t current_delta_mA_x100;
    uint64_t offset_01mm;
    int64_t scaled;
    int64_t result_mA_x100;

    if (target_mA_x100 == NULL) {
        return PARAM_ERROR;
    }

    start_mA_x100 = AoOutput_NormalizeNormalCurrent(start_mA_x100);
    end_mA_x100 = AoOutput_NormalizeNormalCurrent(end_mA_x100);
    *target_mA_x100 = start_mA_x100;

    if ((level_max_01mm <= level_min_01mm) || (end_mA_x100 <= start_mA_x100)) {
        return PARAM_RANGE_ERROR;
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
 * 函数用途：根据继电器 HH/H 或 LL/L 报警覆盖 AO 目标电流。
 * 调用场景：液位电流计算完成后，输出前统一处理报警优先级。
 * 关键约束：只读取继电器运行态，不清报警、不修改继电器配置。
 */
static void AoOutput_ApplyAlarm(uint32_t *target_mA_x100, AoOutputSource *source)
{
    uint8_t high_active = 0U;
    uint8_t low_active = 0U;

    if ((target_mA_x100 == NULL) || (source == NULL)) {
        return;
    }

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        const volatile RelayAlarmRuntimeState *state = RelayOutput_GetRuntimeState(channel);
        if (state == NULL) {
            continue;
        }
        if (state->HH_H_alarm == RELAY_ALARM_STATE_ACTIVE) {
            high_active = 1U;
        }
        if (state->LL_L_alarm == RELAY_ALARM_STATE_ACTIVE) {
            low_active = 1U;
        }
    }

    if (high_active != 0U) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.AOHighCurrent_mA);
        *source = AO_OUTPUT_SOURCE_ALARM_HIGH;
    } else if (low_active != 0U) {
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.AOLowCurrent_mA);
        *source = AO_OUTPUT_SOURCE_ALARM_LOW;
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
        return PARAM_ERROR;
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
        AoOutput_ApplyAlarm(target_mA_x100, source);
        *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(*target_mA_x100);
        return NO_ERROR;
    }

    *source = AO_OUTPUT_SOURCE_INIT;
    *target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.InitialCurrent_mA);
    return NO_ERROR;
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
        AoOutput_SetDisabledRuntime(now);
        return NO_ERROR;
    }

    ret = AoOutput_EnsureDriverReady(now, allow_driver_init);
    if (ret != NO_ERROR) {
        ao_output_runtime.target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
        ao_output_runtime.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
        ao_output_runtime.last_update_tick = now;
        ao_output_runtime.last_error_code = ret;
        ao_output_runtime.update_counter++;
        return ret;
    }

    diag_ret = AoOutput_PollDiagnosticsThrottled(now);
    if (diag_ret != NO_ERROR) {
        source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
        target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
        ret = diag_ret;
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
        } else {
            ret = write_ret;
            target_mA_x100 = AoOutput_NormalizeHardwareCurrent(g_deviceParams.FaultCurrent_mA);
            ao_output_runtime.target_mA_x100 = target_mA_x100;
            ao_output_runtime.source = AO_OUTPUT_SOURCE_DRIVER_ERROR;
        }
        ao_output_runtime.last_error_code = ret;
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
        return ao_output_runtime.last_error_code;
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

/*
 * 函数用途：兼容旧 AD5421 电流决策入口，转调 AO 服务更新。
 * 调用场景：保留给旧调用链或历史接口。
 * 关键约束：内部会进入 AO 更新流程，不应在中断中调用。
 */
void CurrentStateJudgeAndSend(void)
{
    (void)AoOutput_Update();
}
