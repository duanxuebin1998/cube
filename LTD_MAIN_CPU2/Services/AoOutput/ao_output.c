#include "ao_output.h"

#include "ad5421.h"
#include "error_log.h"
#include "main.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define AO_OUTPUT_HARDWARE_MIN_MA_X100 320U
#define AO_OUTPUT_HARDWARE_MAX_MA_X100 2400U
#define AO_OUTPUT_REFRESH_INTERVAL_MS  1000U
#define AO_OUTPUT_DIAG_INTERVAL_MS     1000U
#define AO_OUTPUT_RECOVER_INTERVAL_MS  1000U
#define AO_PROCESS_SOURCE_COUNT        3U

typedef struct {
    uint32_t process_min_mA_x100;
    uint32_t process_max_mA_x100;
    uint32_t fault_min_mA_x100;
    uint32_t fault_max_mA_x100;
} AoCurrentModeLimits;

static AoOutputRuntime ao_output_runtime = {
    400U, 400U, AO_OUTPUT_SOURCE_NON_FOLLOW, 0U, 0U, NO_ERROR,
    0U, 0U, 0U, 0, 0, 0U, 0U, 0U, 0U
};
static AoProcessSample ao_process_samples[AO_PROCESS_SOURCE_COUNT] = {0};
static uint8_t ao_output_initialized = 0U;
static volatile uint8_t ao_output_update_busy = 0U;
static volatile uint8_t ao_output_timer_refresh_pending = 0U;
static volatile uint32_t ao_output_timer_suspend_count = 0U;
static volatile uint32_t ao_simulation_enabled = 0U;
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
static uint8_t ao_last_normal_current_valid = 0U;
static uint32_t ao_last_normal_current_mA_x100 = 0U;
static uint8_t ao_config_snapshot_valid = 0U;
static AoOutputConfig ao_last_config_snapshot = {0};
static uint8_t ao_filter_initialized = 0U;
static uint32_t ao_filter_source = AO_PROCESS_SOURCE_TANK_LEVEL;
static uint32_t ao_filter_tick = 0U;
static int64_t ao_filter_value_x1000 = 0;
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
/* 原子发布AO运行态，避免Modbus与HART读到跨轮次字段。 */
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

/* 复制一致的AO运行态快照。 */
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

/* 在短临界区内更新过程量样本。 */
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

/* 发布过程量；储罐液位同时派生空高有效性。 */
void AoOutput_PublishProcessSample(AoProcessSource source, int32_t value_01mm, uint8_t valid)
{
    uint32_t primask;
    uint32_t now = HAL_GetTick();

    primask = __get_PRIMASK();
    __disable_irq();
    AoOutput_WriteProcessSample(source, value_01mm, valid, now);
    if (source == AO_PROCESS_SOURCE_TANK_LEVEL) {
        if ((valid != 0U) &&
            (value_01mm >= 0) &&
            ((uint32_t)value_01mm <= g_deviceParams.tankHeight) &&
            (g_deviceParams.tankHeight <= 0x7FFFFFFFUL)) {
            AoOutput_WriteProcessSample(AO_PROCESS_SOURCE_ULLAGE,
                                        (int32_t)g_deviceParams.tankHeight - value_01mm,
                                        1U,
                                        now);
        } else {
            AoOutput_WriteProcessSample(AO_PROCESS_SOURCE_ULLAGE, 0, 0U, now);
        }
    }
    __set_PRIMASK(primask);
}

/* 显式失效一个过程量样本。 */
void AoOutput_InvalidateProcessSample(AoProcessSource source)
{
    AoOutput_PublishProcessSample(source, 0, 0U);
}

/* 复制指定过程量快照。 */
uint32_t AoOutput_ReadProcessSample(AoProcessSource source, AoProcessSample *sample)
{
    uint32_t primask;

    if ((sample == NULL) || ((uint32_t)source >= AO_PROCESS_SOURCE_COUNT)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *sample = ao_process_samples[(uint32_t)source];
    __set_PRIMASK(primask);
    return NO_ERROR;
}

/* 在短临界区复制完整AO配置，避免FC10中断替换参数时单轮混用。 */
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

/* 复制当前配置选中的过程量快照。 */
uint32_t AoOutput_GetSelectedProcessSample(AoProcessSample *sample)
{
    AoOutputConfig config;

    AoOutput_GetConfigSnapshot(&config);
    return AoOutput_ReadProcessSample((AoProcessSource)config.output_source, sample);
}

/* 设置仿真运行态；上电初始化会强制清零。 */
void AoOutput_SetSimulationEnabled(uint32_t enabled)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ao_simulation_enabled = (enabled == 0U) ? 0U : 1U;
    ao_output_runtime.simulation_enabled = ao_simulation_enabled;
    ao_filter_initialized = 0U;
    __set_PRIMASK(primask);
}

/* 返回仿真运行态开关。 */
uint32_t AoOutput_IsSimulationEnabled(void)
{
    return ao_simulation_enabled;
}

/* 将电流限制在AD5421物理允许范围内。 */
static uint32_t AoOutput_ClampHardwareCurrent(uint32_t current_mA_x100)
{
    if (current_mA_x100 < AO_OUTPUT_HARDWARE_MIN_MA_X100) {
        return AO_OUTPUT_HARDWARE_MIN_MA_X100;
    }
    if (current_mA_x100 > AO_OUTPUT_HARDWARE_MAX_MA_X100) {
        return AO_OUTPUT_HARDWARE_MAX_MA_X100;
    }
    return current_mA_x100;
}

/* 返回NMS81普通、NE、US及固定模式的过程与故障边界。 */
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

/* 由过程值计算有符号百分比和模式限幅后的电流。 */
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

/* 对正常过程输入应用一阶阻尼，特殊输出状态不调用本函数。 */
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
        (ao_filter_source != source) ||
        (ao_output_runtime.source != AO_OUTPUT_SOURCE_PROCESS)) {
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

/* 判断液位或空高源是否处于允许AO跟随的命令。 */
static uint8_t AoOutput_IsLiquidFollowCommand(CommandType command)
{
    return ((command == CMD_FIND_OIL) ||
            (command == CMD_CALIBRATE_OIL) ||
            (command == CMD_CORRECT_OIL)) ? 1U : 0U;
}

/* 判断水位源是否处于允许AO跟随的命令。 */
static uint8_t AoOutput_IsWaterFollowCommand(CommandType command)
{
    return ((command == CMD_FOLLOW_WATER) ||
            (command == CMD_CALIBRATE_WATER)) ? 1U : 0U;
}

/*
 * 函数用途：判断所选过程源是否正处于与其匹配的连续跟随状态。
 * 调用场景：AO每轮选择目标电流前只读整机状态、当前命令和待切换命令。
 * 关键约束：存在待处理命令时立即退出跟随，函数不调用任何会修改状态的接口。
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

    if (pending_command != CMD_NONE) {
        return 0U;
    }
    if ((config->output_source == AO_PROCESS_SOURCE_TANK_LEVEL) ||
        (config->output_source == AO_PROCESS_SOURCE_ULLAGE)) {
        return ((state == STATE_FLOWOIL) &&
                (AoOutput_IsLiquidFollowCommand(current_command) != 0U)) ? 1U : 0U;
    }
    if (config->output_source == AO_PROCESS_SOURCE_WATER_LEVEL) {
        return ((state == STATE_FOLLOW_WATERING) &&
                (AoOutput_IsWaterFollowCommand(current_command) != 0U)) ? 1U : 0U;
    }
    return 0U;
}

/*
 * 函数用途：处理会改变过程电流解释的AO配置切换。
 * 调用场景：AO初始化及每轮取得一致配置快照后调用。
 * 关键约束：禁用、输出源、电流模式或量程变化会清除上次有效过程电流；阻尼变化只重置滤波。
 */
static void AoOutput_HandleConfigTransition(const AoOutputConfig *config)
{
    uint8_t clear_last_valid = 0U;
    uint8_t reset_filter = 0U;

    if (config == NULL) {
        return;
    }

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
    }
    if (config->work_mode == AO_WORK_MODE_DISABLED) {
        clear_last_valid = 1U;
        reset_filter = 1U;
    }

    if (clear_last_valid != 0U) {
        ao_last_normal_current_valid = 0U;
        ao_last_normal_current_mA_x100 = 0U;
    }
    if (reset_filter != 0U) {
        ao_filter_initialized = 0U;
    }
    ao_last_config_snapshot = *config;
    ao_config_snapshot_valid = 1U;
}

/* 仅整机明确进入错误态且带真实错误码时执行AO故障动作。 */
static uint8_t AoOutput_IsDeviceFault(void)
{
    DeviceState state;
    uint32_t error_code;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    state = g_measurement.device_status.device_state;
    error_code = g_measurement.device_status.error_code;
    __set_PRIMASK(primask);

    return ((state == STATE_ERROR) &&
            (error_code != NO_ERROR) &&
            (error_code != STATE_SWITCH)) ? 1U : 0U;
}

/* 按故障动作选择旁路阻尼的电流；保持动作无历史时回退非跟随电流。 */
static uint32_t AoOutput_SelectFaultCurrent(const AoOutputConfig *config)
{
    if ((config->fault_mode == AO_FAULT_ACTION_HOLD_LAST_VALID) &&
        (ao_last_normal_current_valid != 0U)) {
        return ao_last_normal_current_mA_x100;
    }
    if (config->fault_mode == AO_FAULT_ACTION_HOLD_LAST_VALID) {
        return config->power_on_current_mA_x100;
    }
    return config->fault_current_mA_x100;
}

/* 按固定优先级选择本轮目标，只有正常过程输出进入阻尼。 */
static uint32_t AoOutput_SelectTarget(const AoOutputConfig *config,
                                      uint32_t now,
                                      AoOutputSource *source,
                                      uint32_t *target_mA_x100,
                                      int32_t *process_value_01mm,
                                      int32_t *percent_x100,
                                      uint32_t *process_valid)
{
    AoProcessSample sample = {0};
    int32_t filtered_01mm;

    if ((config == NULL) || (source == NULL) || (target_mA_x100 == NULL) ||
        (process_value_01mm == NULL) || (percent_x100 == NULL) ||
        (process_valid == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    (void)AoOutput_ReadProcessSample((AoProcessSource)config->output_source, &sample);
    *process_value_01mm = 0;
    *process_valid = 0U;
    *percent_x100 = 0;

    if (config->work_mode == AO_WORK_MODE_DISABLED) {
        *source = AO_OUTPUT_SOURCE_DISABLED;
        *target_mA_x100 = AO_DISABLED_CURRENT_MA_X100;
        return NO_ERROR;
    }
    if (ao_simulation_enabled != 0U) {
        *source = AO_OUTPUT_SOURCE_SIMULATION;
        *target_mA_x100 = config->simulation_current_mA_x100;
        return NO_ERROR;
    }
    if (AoOutput_IsDeviceFault() != 0U) {
        *source = AO_OUTPUT_SOURCE_FAULT;
        *target_mA_x100 = AoOutput_SelectFaultCurrent(config);
        return NO_ERROR;
    }
    if (config->current_mode == AO_CURRENT_MODE_FIXED) {
        *source = AO_OUTPUT_SOURCE_FIXED;
        *target_mA_x100 = config->fixed_current_mA_x100;
        return NO_ERROR;
    }
    if ((AoOutput_IsSelectedProcessFollowing(config) == 0U) ||
        (sample.valid == 0U)) {
        ao_filter_initialized = 0U;
        *source = AO_OUTPUT_SOURCE_NON_FOLLOW;
        *target_mA_x100 = config->power_on_current_mA_x100;
        return NO_ERROR;
    }

    filtered_01mm = AoOutput_FilterProcess(config, sample.value_01mm, now);
    /* 运行态输入值与换算比例必须使用同一阻尼后过程量。 */
    *process_value_01mm = filtered_01mm;
    *process_valid = 1U;
    *source = AO_OUTPUT_SOURCE_PROCESS;
    *target_mA_x100 = AoOutput_MapProcessToCurrent(config, filtered_01mm, percent_x100);
    return NO_ERROR;
}

/* 清除AD5421诊断恢复节流状态。 */
static void AoOutput_ResetRecoverState(void)
{
    ao_output_recover_valid = 0U;
    ao_output_last_recover_tick = 0U;
    ao_output_last_recover_target_mA_x100 = 0U;
}

/* 判断当前是否允许执行一次AD5421恢复序列。 */
static uint8_t AoOutput_ShouldRecover(uint32_t now)
{
    if (ao_output_recover_valid == 0U) {
        return 1U;
    }
    return ((now - ao_output_last_recover_tick) >= AO_OUTPUT_RECOVER_INTERVAL_MS) ? 1U : 0U;
}

/* 判断诊断错误是否属于需要原地等待清除的过温状态。 */
static uint8_t AoOutput_IsOvertemperatureError(uint32_t error_code)
{
    return ((error_code == AD5421_OVERTEMP_SHUTDOWN) ||
            (error_code == AD5421_OVERTEMP_WARNING)) ? 1U : 0U;
}

/* 过温故障必须等待降温，不通过周期复位强行恢复。 */
static uint8_t AoOutput_IsDiagnosticRecoveryAllowed(uint32_t error_code)
{
    return (AoOutput_IsOvertemperatureError(error_code) != 0U) ? 0U : 1U;
}

/* 判断错误是否属于AO运行期驱动故障。 */
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

/*
 * 函数用途：记录AO运行期硬件故障，不把后台故障升级为整机最终错误。
 * 调用场景：初始化或诊断失败传NULL；写电流失败传本轮待发布快照。
 * 关键约束：保留最近成功下发电流，写失败时仍发布本轮过程输入和比例。
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
    next.target_mA_x100 = limits.fault_max_mA_x100;
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

/* 按当前工作模式选择芯片初始化时应保持的禁用、固定或非跟随电流。 */
static uint32_t AoOutput_GetInitialCurrent(const AoOutputConfig *config)
{
    if (config->work_mode == AO_WORK_MODE_DISABLED) {
        return AO_DISABLED_CURRENT_MA_X100;
    }
    if (config->current_mode == AO_CURRENT_MODE_FIXED) {
        return config->fixed_current_mA_x100;
    }
    return config->power_on_current_mA_x100;
}

/* 按需初始化AD5421并缓存驱动可用状态。 */
static uint32_t AoOutput_EnsureDriverReady(uint32_t now,
                                           uint8_t allow_init,
                                           uint32_t initial_mA_x100)
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
    ret = AD5421_InitCurrentX100(AoOutput_ClampHardwareCurrent(initial_mA_x100));
    if (ret == NO_ERROR) {
        ao_output_driver_ready = 1U;
        ao_output_driver_retry_valid = 0U;
        ao_output_diag_valid = 0U;
        ao_output_last_diag_tick = 0U;
        ao_output_last_diag_error = NO_ERROR;
        AoOutput_ResetRecoverState();
        ao_output_last_recover_target_mA_x100 = initial_mA_x100;
        AoOutput_QueueDriverRecovery();
    } else if (AoOutput_IsOvertemperatureError(ret) != 0U) {
        /* 初始化已完成到末尾诊断，保留SPI诊断能力并等待温度故障清除。 */
        ao_output_driver_ready = 1U;
        ao_output_driver_retry_valid = 0U;
    }
    return ret;
}

/* 按节流周期轮询AD5421诊断。 */
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

/* 目标变化立即写，目标不变每秒刷新，失败目标按周期重试。 */
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

/* 初始化AO并按当前配置写入禁用、固定或非跟随电流。 */
uint32_t AoOutput_Init(void)
{
    AoOutputConfig config;
    AoOutputRuntime next = {0};
    uint32_t now;
    uint32_t initial_mA_x100;
    uint32_t ret;

    if (AoOutput_TryEnterUpdate() == 0U) {
        return ao_output_runtime.last_error_code;
    }
    now = HAL_GetTick();
    ao_output_initialized = 1U;
    ao_output_driver_ready = 0U;
    ao_output_driver_retry_valid = 0U;
    ao_output_diag_valid = 0U;
    ao_output_write_attempt_valid = 0U;
    ao_last_normal_current_valid = 0U;
    ao_last_normal_current_mA_x100 = 0U;
    ao_config_snapshot_valid = 0U;
    ao_filter_initialized = 0U;
    memset(ao_process_samples, 0, sizeof(ao_process_samples));
    ao_simulation_enabled = 0U;

    AoOutput_GetConfigSnapshot(&config);
    AoOutput_HandleConfigTransition(&config);
    initial_mA_x100 = AoOutput_GetInitialCurrent(&config);
    next.target_mA_x100 = initial_mA_x100;
    next.last_sent_mA_x100 = 0U;
    next.source = (config.work_mode == AO_WORK_MODE_DISABLED) ?
                  AO_OUTPUT_SOURCE_DISABLED :
                  ((config.current_mode == AO_CURRENT_MODE_FIXED) ?
                   AO_OUTPUT_SOURCE_FIXED : AO_OUTPUT_SOURCE_NON_FOLLOW);
    next.last_update_tick = now;
    next.last_error_code = NO_ERROR;
    next.simulation_enabled = 0U;
    next.dac_readback_valid = 0U;

    ret = AoOutput_EnsureDriverReady(now, 1U, initial_mA_x100);
    next.driver_fault_flags = AD5421_GetFaultFlags();
    next.driver_fault_register = AD5421_GetFaultRegister();
    next.last_error_code = ret;
    if (ret == NO_ERROR) {
        next.last_sent_mA_x100 = initial_mA_x100;
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

/* 执行AO固定优先级状态机并刷新AD5421。 */
static uint32_t AoOutput_UpdateInternal(uint8_t allow_driver_init)
{
    AoOutputConfig config;
    AoOutputRuntime next;
    AoOutputSource source = AO_OUTPUT_SOURCE_NON_FOLLOW;
    uint32_t now;
    uint32_t initial_mA_x100;
    uint32_t target_mA_x100 = 400U;
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
    initial_mA_x100 = AoOutput_GetInitialCurrent(&config);
    ret = AoOutput_EnsureDriverReady(now, allow_driver_init, initial_mA_x100);
    if (ret != NO_ERROR) {
        return AoOutput_RecordRuntimeDriverError(&config,
                                                  NULL,
                                                  now,
                                                  ret,
                                                  AoOutput_IsOvertemperatureError(ret));
    }

    diag_ret = AoOutput_PollDiagnosticsThrottled(now);
    if (diag_ret != NO_ERROR) {
        uint32_t recover_target = ao_output_runtime.last_sent_mA_x100;
        uint32_t recover_ret = diag_ret;

        if (recover_target == 0U) {
            recover_target = initial_mA_x100;
        }
        if ((AoOutput_IsDiagnosticRecoveryAllowed(diag_ret) != 0U) &&
            (AoOutput_ShouldRecover(now) != 0U)) {
            ao_output_recover_valid = 1U;
            ao_output_last_recover_tick = now;
            recover_ret = AD5421_RecoverCurrentX100(recover_target);
            if (recover_ret == NO_ERROR) {
                ao_output_driver_ready = 1U;
                ao_output_diag_valid = 0U;
                ao_output_last_diag_error = NO_ERROR;
                ao_output_write_attempt_valid = 0U;
                ao_output_last_recover_target_mA_x100 = recover_target;
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
                                &target_mA_x100,
                                &process_value_01mm,
                                &percent_x100,
                                &process_valid);
    if (ret != NO_ERROR) {
        return ret;
    }
    target_mA_x100 = AoOutput_ClampHardwareCurrent(target_mA_x100);
    should_send = AoOutput_ShouldWriteCurrent(now, target_mA_x100);

    AoOutput_GetRuntimeSnapshot(&next);
    next.target_mA_x100 = target_mA_x100;
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
        ret = AD5421_SetCurrentX100(target_mA_x100);
        ao_output_write_attempt_valid = 1U;
        ao_output_last_write_attempt_tick = now;
        ao_output_last_write_attempt_mA_x100 = target_mA_x100;
        next.driver_fault_flags = AD5421_GetFaultFlags();
        next.driver_fault_register = AD5421_GetFaultRegister();
        if (ret == NO_ERROR) {
            next.last_sent_mA_x100 = target_mA_x100;
            next.last_sent_tick = now;
            if (source == AO_OUTPUT_SOURCE_PROCESS) {
                process_write_succeeded = 1U;
            }
            ao_output_last_recover_target_mA_x100 = target_mA_x100;
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
        ao_last_normal_current_mA_x100 = target_mA_x100;
        ao_last_normal_current_valid = 1U;
    }
    next.update_counter++;
    AoOutput_CommitRuntime(&next);
    return NO_ERROR;
}

/* 带重入保护地执行一次任务态AO刷新。 */
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
        /* 错误 阶段：错误报警 模块：模拟量输出 操作：AD5421故障定位 原因：当前错误码映射 处理：保持主流程并后台恢复 */
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
        /* 错误 阶段：重试成功 模块：模拟量输出 操作：AD5421自动恢复 原因：通信及芯片诊断恢复 尝试：1/1 */
        ErrorLog_RecoverDetail("模拟量输出",
                               "AD5421自动恢复",
                               "通信及芯片诊断恢复",
                               1U,
                               1U,
                               detail);
    }
}
/* 返回兼容旧调用点的只读运行态指针。 */
const AoOutputRuntime *AoOutput_GetRuntime(void)
{
    return &ao_output_runtime;
}

/* HART读取最后一次成功下发的电流，而不是尚未写入的目标值。 */
float AoOutput_GetCurrent_mA(void)
{
    AoOutputRuntime snapshot;

    AoOutput_GetRuntimeSnapshot(&snapshot);
    return ((float)snapshot.last_sent_mA_x100) / 100.0f;
}

/* 返回真实百分数，例如50.00表示50%，不再返回0.5。 */
float AoOutput_GetPercentOfRange(void)
{
    AoOutputRuntime snapshot;

    AoOutput_GetRuntimeSnapshot(&snapshot);
    return ((float)snapshot.percent_x100) / 100.0f;
}
