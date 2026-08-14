#include "fault_recovery.h"

#include <stdio.h>
#include "main.h"
#include "motor_ctrl.h"
#include "sensor.h"
#include "fault_manager.h"
#include "error_log.h"

#define FAULT_RECOVERY_INTERVAL_MS 1000U /* 故障自动恢复参数：故障 恢复 间隔 毫秒。 */

typedef struct {
    uint8_t active;               /* 是否存在待恢复命令；为 0 时主循环不进入恢复轮询。 */
    CommandType command;          /* 失败前正在执行的命令，恢复成功后由主循环重跑。 */
    uint32_t error_code;          /* 最近一次真实故障码，用于保持现场错误状态和恢复前附加动作判断。 */
    DeviceState device_state;     /* 进入恢复时的设备状态，等待期反复恢复，避免空闲兜底清掉错误态。 */
    uint32_t zero_point_status;   /* 进入恢复时的零点状态，等待期保持给 CPU3/显示侧读取。 */
    uint32_t last_check_tick;     /* 上次恢复检查时刻，用于 1 秒节流，避免连续刷通信和日志。 */
    uint32_t command_retry_count; /* 已经由自动恢复触发的业务命令重跑次数。 */
    uint8_t awaiting_retry_result; /* 已触发重跑后置 1，等待命令结果决定清理或继续恢复。 */
} FaultRecoveryContext;

/* 故障自动恢复状态机上下文，集中保存目标命令、故障快照、节拍和重试次数。 */
static FaultRecoveryContext s_fault_recovery = {
    .active = 0U,
    .command = CMD_NONE,
    .error_code = NO_ERROR,
    .device_state = STATE_ERROR,
    .zero_point_status = 0U,
    .last_check_tick = 0U,
    .command_retry_count = 0U,
    .awaiting_retry_result = 0U,
};

/**
 * @brief 判断错误码是否需要在自动恢复检查前重新初始化电机驱动。
 *
 * 自动恢复是否继续只按原始命令判断；这里仅决定恢复检查前是否需要重建 TMC5130 配置。
 * @param error_code 当前恢复上下文中的错误码。
 * @return 1 表示需要先执行 MotorCtrl_Init()；0 表示直接做部件参数检查。
 */
static uint8_t FaultRecovery_IsMotorDriverError(uint32_t error_code)
{
    switch (error_code) {
    /* 这些错误都可能通过重新下发 TMC5130 配置恢复，所以恢复检查前先 MotorCtrl_Init()。 */
    case MOTOR_TMC_COMM_ERROR:
    case MOTOR_TMC_CONFIG_LOST:
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
    case MOTOR_DISABLED:
    case MOTOR_DRIVER_NOT_INITIALIZED:
    case MOTOR_RUN_TIMEOUT:
    case MOTOR_STOP_WAIT_TIMEOUT:
    case MOTOR_ARRIVAL_WAIT_TIMEOUT:
        return 1U;
    default:
        return 0U;
    }
}

/**
 * @brief 判断错误码是否属于确定性配置/边界错误。
 *
 * 这类错误不会通过重新初始化或重读部件参数恢复，自动重跑只会重复失败。
 * @param error_code 当前错误码。
 * @return 1 表示不进入自动恢复；0 表示继续按命令白名单判断。
 */
static uint8_t FaultRecovery_IsNonRecoverableError(uint32_t error_code)
{
    switch (error_code) {
    /* 驱动保护、相线和堵转故障需要先排查硬件或机构，禁止自动重跑测量。 */
    case MOTOR_UNKNOWN_FEEDBACK:
    case MOTOR_OVERTEMPERATURE:
    case MOTOR_STALL_ERROR:
    case MOTOR_PHASE_SHORT_ERROR:
    case MOTOR_PHASE_OPEN_ERROR:
    case MOTOR_DRIVER_OVERTEMP_WARNING:
    case ENCODER_POWERON_CHANGE:
    case ENCODER_LINEARITY_WARNING:
    case ENCODER_POSITION_JUMP:
    case ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR:
    case SENSOR_IDENTITY_MISMATCH:
    case SENSOR_PROTOCOL_VERSION_INCOMPATIBLE:
    case SENSOR_CAPABILITY_UNSUPPORTED:
    case SENSOR_COMMAND_UNSUPPORTED:
    case SENSOR_ARGUMENT_REJECTED:
    case SENSOR_MODE_MISMATCH:
    case SENSOR_MODE_NOT_ALLOWED:
    case PARAM_RANGE_ERROR:
    case PARAM_CONFIG_MISSING:
    case PARAM_COMBINATION_CONFLICT:
    case PARAM_FEATURE_UNSUPPORTED:
    case PARAM_STORAGE_SIZE_MISMATCH:
    case PARAM_STORAGE_VERSION_MISMATCH:
    case PARAM_STORAGE_WRITE_VERIFY_FAILED:
    /*
     * 23类电源故障拥有独立恢复状态机和新正式流程解锁边界，
     * 通用故障恢复器不得并行清码或重复启动硬件。
     */
    case POWER_SUPPLY_24V_UNDERVOLTAGE:
    case POWER_MONITOR_ADC_OVERRUN:
    case POWER_MONITOR_DMA_STOPPED:
    case POWER_MONITOR_INIT_FAILED:
    case POWER_MONITOR_RECOVERY_FAILED:
    case POWER_LOSS_POSITION_SAVE_FAILED:
    case MEASUREMENT_TANK_HEIGHT_NOT_CONFIGURED:
    case MEASUREMENT_WATER_CALIBRATION_NOT_CONFIGURED:
    case MEASUREMENT_TANK_HEIGHT_RESULT_INVALID:
    case MEASUREMENT_WATER_CALC_OUT_OF_RANGE:
    case MEASUREMENT_DENSITY_PLAN_INVALID:
    case SYSTEM_BUFFER_CAPACITY_ERROR:
    case SYSTEM_CALL_CONDITION_ERROR:
    case SYSTEM_CALCULATION_ERROR:
        return 1U;
    default:
        return 0U;
    }
}

/**
 * @brief 获取故障自动恢复允许的业务重跑次数。
 *
 * 参数为0时关闭自动恢复；1~10为允许重跑次数，非法值按默认3次处理。
 * @return 允许的自动重跑次数。
 */
static uint32_t FaultRecovery_GetRetryLimit(void)
{
    uint32_t retry_limit = g_deviceParams.fault_auto_recovery_retry_limit;

    /* 持久化重试次数超过固件上限时回退到安全默认值，避免损坏参数导致恢复循环失控。 */
    if (retry_limit > FAULT_AUTO_RECOVERY_RETRY_MAX) {
        retry_limit = FAULT_AUTO_RECOVERY_RETRY_DEFAULT;
    }

    return retry_limit;
}

/**
 * @brief 判断命令失败后是否允许进入自动恢复流程。
 *
 * 当前策略先排除确定性配置/边界错误，再按命令白名单恢复，避免无效命令自动重跑。
 * @param command 失败的测量命令。
 * @return 1 表示允许自动恢复；0 表示交给普通错误态处理。
 */
static uint8_t FaultRecovery_IsRecoverableCommand(CommandType command)
{
    switch (command) {
    /* 白名单只描述“这个命令是否允许自动重跑”，不描述错误类型。 */
    case CMD_BACK_ZERO:
    case CMD_FIND_OIL:
    case CMD_FIND_WATER:
    case CMD_FIND_BOTTOM:
    case CMD_MEASURE_SINGLE:
    case CMD_MONITOR_SINGLE:
    case CMD_SYNTHETIC:
    case CMD_FOLLOW_WATER:
    case CMD_RUN_TO_POSITION:
    case CMD_MEASURE_DISTRIBUTED:
    case CMD_GB_MEASURE_DISTRIBUTED:
    case CMD_MEASURE_DENSITY_METER:
    case CMD_MEASURE_DENSITY_RANGE:
    case CMD_SI_PROFILE:
    case CMD_WARTSILA_DENSITY_RANGE:
    case CMD_CALIBRATE_ZERO:
    case CMD_CALIBRATE_OIL:
    case CMD_CORRECT_OIL:
    case CMD_CALIBRATE_WATER:
    case CMD_CALIBRATE_TANKHEIGHT:
        return 1U;
    default:
        return 0U;
    }
}

/**
 * @brief 清空自动恢复上下文。
 *
 * 清空后主循环不再维持旧错误状态，也不会继续周期性检查恢复条件。
 */
static void FaultRecovery_ClearContext(void)
{
    s_fault_recovery.active = 0U;                  /* 清除恢复任务标志。 */
    s_fault_recovery.command = CMD_NONE;           /* 清除待重跑命令，避免下次误恢复旧命令。 */
    s_fault_recovery.error_code = NO_ERROR;        /* 清除恢复上下文错误码；全局错误状态由调用方决定。 */
    s_fault_recovery.device_state = STATE_ERROR;   /* 保持默认错误态，下一次启动恢复会重新覆盖。 */
    s_fault_recovery.zero_point_status = 0U;       /* 清除恢复期保持给显示侧的零点状态。 */
    s_fault_recovery.last_check_tick = 0U;         /* 清除节流时间，下一次启动时重新计时。 */
    s_fault_recovery.command_retry_count = 0U;     /* 清除自动重跑计数，新命令重新计算。 */
    s_fault_recovery.awaiting_retry_result = 0U;   /* 清除等待重跑结果标志。 */
}

/**
 * @brief 查询自动故障恢复是否正在占用设备恢复流程。
 *
 * @details 调用场景：CPU2持久参数最终写门禁判断错误态是否真正空闲。
 * @note 关键约束：只读返回恢复上下文，不清故障、不取消恢复、不触发重试。
 *
 * @return true 表示自动故障恢复状态机已激活，正在占用恢复流程；false 表示当前没有自动恢复事务。
 */
bool FaultRecovery_IsActive(void)
{
    return s_fault_recovery.active != 0U;
}

/**
 * @brief 自动恢复等待期间恢复原错误状态。
 *
 * 空闲错误兜底会根据全局状态做处理；恢复等待期必须保持原错误态，避免恢复上下文被提前清掉。
 */
static void FaultRecovery_RestoreErrorStatus(void)
{
    /* 没有活动恢复上下文时不得改写全局状态；否则可能把正常业务状态恢复成过期故障。 */
    if (!s_fault_recovery.active) {
        return;
    }

    g_measurement.device_status.device_state = s_fault_recovery.device_state;         /* 等待恢复时继续对外呈现原错误态。 */
    g_measurement.device_status.error_code = s_fault_recovery.error_code;             /* 保持最近真实故障码，方便现场和 CPU3 读取。 */
    g_measurement.device_status.zero_point_status = s_fault_recovery.zero_point_status; /* 保持零点错误标志，不让空闲逻辑提前清理。 */
    g_measurement.device_status.current_command = CMD_NONE;                           /* 恢复等待期没有业务命令正在执行。 */
}

/**
 * @brief 记录本轮自动恢复检查失败后的真实错误状态。
 *
 * 恢复是否继续仍按原始命令判断；这里更新错误码是为了让后续日志显示当前真实故障。
 * @param error_code 本轮恢复检查返回的错误码。
 */
static void FaultRecovery_RecordFailure(uint32_t error_code)
{
    /* 恢复检查返回正常或命令切换时不记录失败，命令切换由取消路径单独处理。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    g_measurement.device_status.device_state = STATE_ERROR; /* 恢复失败后仍处于错误态。 */
    g_measurement.device_status.error_code = error_code;   /* 记录本轮检查得到的真实错误，而不是旧错误。 */
    g_measurement.device_status.zero_point_status = 1U;    /* 维持错误零点状态，阻止业务继续认为设备正常。 */
    g_measurement.device_status.current_command = CMD_NONE; /* 恢复检查不是测量命令，不能占用 current_command。 */

    s_fault_recovery.error_code = error_code;        /* 后续恢复检查按最新错误决定是否先初始化电机。 */
    s_fault_recovery.device_state = STATE_ERROR;     /* 上下文也同步为错误态，等待期可反复恢复。 */
    s_fault_recovery.zero_point_status = 1U;         /* 上下文同步零点错误状态，避免下一轮被覆盖。 */
}

/**
 * @brief 自动恢复重跑测量次数达到上限后停止继续重跑。
 *
 * 读取部件参数可以持续用于恢复确认，但真正业务测量不能无限次重入；
 * 达到上限后保留最后一次错误状态，交给空闲错误兜底和显示侧处理。
 * @param error_code 最后一次业务测量失败的错误码。
 */
static void FaultRecovery_StopAfterMaxRetry(uint32_t error_code)
{
    uint32_t retry_limit = FaultRecovery_GetRetryLimit();

    if (retry_limit == 0U) {
        retry_limit = FAULT_AUTO_RECOVERY_RETRY_DEFAULT;
    }

    /* 达到重试上限时若本轮没有新的真实故障，继续使用恢复上下文中最初锁存的错误码作为最终原因。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        error_code = s_fault_recovery.error_code;
    }

    g_measurement.device_status.device_state = STATE_ERROR;
    g_measurement.device_status.error_code = error_code;
    g_measurement.device_status.zero_point_status = 1U;
    g_measurement.device_status.current_command = CMD_NONE;

    printf("自动恢复\t测量重跑次数达到上限%lu次，停止自动重跑，错误码=0x%08lX\r\n",
           (unsigned long)retry_limit,
           (unsigned long)error_code);

    ErrorLog_Warn(ERROR_LOG_MODULE_SYSTEM,
                  ERROR_LOG_OP_AUTO_RECOVER,
                  ERROR_LOG_REASON_AUTO_RECOVER_FAIL,
                  ERROR_LOG_ACTION_STOP_MEASURE);
    FaultRecovery_ClearContext();
}

/**
 * @brief 测量命令失败后启动自动恢复等待。
 *
 * 这里只保存失败命令和错误上下文，不立即执行恢复动作，避免在命令收尾过程中嵌套重跑业务。
 * @param command 失败的测量命令。
 * @param error_code 命令失败后的全局错误码。
 */
static void FaultRecovery_Start(CommandType command, uint32_t error_code)
{
    uint32_t retry_count = 0U;

    /* 正常结果和用户命令切换不启动自动恢复，避免把主动中断误判成故障。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    /* 不可恢复故障保持原错误态并交给人工处理，禁止自动执行可能扩大风险的电机或通信动作。 */
    if (FaultRecovery_IsNonRecoverableError(error_code)) {
        return;
    }

    /* 重试次数配置为 0 表示关闭自动恢复，当前故障只按普通错误流程处理。 */
    if (FaultRecovery_GetRetryLimit() == 0U) {
        return;
    }

    /* 只有明确列入白名单的测量命令才允许恢复后自动重跑，即时控制和维护命令不会被后台重复执行。 */
    if (!FaultRecovery_IsRecoverableCommand(command)) {
        return;
    }

    /* 同一命令再次进入恢复时继承已经完成的业务重跑次数，防止重新建上下文绕过总重试上限。 */
    if (s_fault_recovery.active && (s_fault_recovery.command == command)) {
        retry_count = s_fault_recovery.command_retry_count;
    }

    /* 首次进入恢复且设备尚未发布错误态时先执行统一停机，随后固定错误状态等待部件检查。 */
    if (g_measurement.device_status.device_state != STATE_ERROR) {
        HandleError();
        /* 进入自动恢复前先固定为错误态，后续由恢复检查决定是否清除。 */
        g_measurement.device_status.device_state = STATE_ERROR;
        g_measurement.device_status.zero_point_status = 1U;
    }

    s_fault_recovery.active = 1U;                                             /* 标记主循环后续由恢复模块接管。 */
    s_fault_recovery.command = command;                                       /* 保存原命令，恢复成功后由主循环重跑。 */
    s_fault_recovery.error_code = error_code;                                 /* 保存失败原因，恢复前判断是否需要先初始化电机。 */
    s_fault_recovery.device_state = g_measurement.device_status.device_state; /* 保存当前错误态，等待期反复恢复。 */
    s_fault_recovery.zero_point_status = g_measurement.device_status.zero_point_status; /* 保存显示侧需要看到的零点状态。 */
    s_fault_recovery.last_check_tick = HAL_GetTick();                         /* 从启动恢复时开始计 1 秒检查间隔。 */
    s_fault_recovery.command_retry_count = retry_count;                       /* 同一命令多轮恢复时保留已重跑次数。 */
    s_fault_recovery.awaiting_retry_result = 0U;                              /* 重新进入恢复等待，还没有触发下一次重跑。 */

    ErrorLog_Warn(ERROR_LOG_MODULE_SYSTEM,
                  ERROR_LOG_OP_AUTO_RECOVER,
                  ERROR_LOG_REASON_AUTO_RECOVER_START,
                  ERROR_LOG_ACTION_CONTINUE);
    FaultRecovery_RestoreErrorStatus();
}

/**
 * @brief 在命令执行后更新故障自动恢复计数和结果。
 *
 * @param command 命令值。该 CommandType 是刚执行完成的正式测量命令，用于更新恢复状态机的成功、失败和重试归属。
 */
void FaultRecovery_UpdateAfterCommand(CommandType command)
{
    uint32_t error_code = g_measurement.device_status.error_code;
    uint32_t retry_limit = FaultRecovery_GetRetryLimit();

    if (retry_limit == 0U) {
        /* 现场关闭自动恢复时，命令结果交给普通错误兜底处理。 */
        if (s_fault_recovery.active) {
            FaultRecovery_ClearContext();
        }
        return;
    }

    /* 只有与当前恢复命令匹配、且确实正在等待重跑结果时，本次命令返回值才可推进恢复状态机。 */
    if (s_fault_recovery.active &&
        s_fault_recovery.awaiting_retry_result &&
        (s_fault_recovery.command == command)) {
        s_fault_recovery.awaiting_retry_result = 0U;

        /* 只有业务命令重跑返回成功，才记录完整恢复成功。 */
        if (error_code == NO_ERROR) {
            ErrorLog_Recover(ERROR_LOG_MODULE_SYSTEM,
                             ERROR_LOG_OP_AUTO_RECOVER,
                             ERROR_LOG_REASON_RECOVER_OK,
                             s_fault_recovery.command_retry_count,
                             retry_limit);
            FaultRecovery_ClearContext();
            return;
        }

        /* 命令切换是非故障退出，只清理恢复上下文，不记录恢复成功。 */
        if (error_code == STATE_SWITCH) {
            FaultRecovery_ClearContext();
            return;
        }

        /* 业务重跑仍失败且累计次数达到上限时终止恢复，不再重新建立下一轮等待。 */
        if (s_fault_recovery.command_retry_count >= retry_limit) {
            FaultRecovery_StopAfterMaxRetry(error_code);
            return;
        }

        FaultRecovery_Start(command, error_code);
        return;
    }

    /* 命令结束后仍有真实错误码，才进入恢复等待；命令切换不算故障。 */
    if ((error_code != NO_ERROR) && (error_code != STATE_SWITCH)) {
        FaultRecovery_Start(command, error_code);
        return;
    }

    /* 原命令重新执行成功后清掉恢复上下文，避免后续主循环再次重跑。 */
    if (s_fault_recovery.active && (s_fault_recovery.command == command)) {
        FaultRecovery_ClearContext();
    }
}

/**
 * @brief 取消当前故障自动恢复流程并清理恢复状态。
 *
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 */
void FaultRecovery_Cancel(const char *reason)
{
    (void)reason;

    /* 没有活动恢复上下文时取消操作是幂等空操作，不打印误导性的取消日志。 */
    if (!s_fault_recovery.active) {
        return;
    }

    /* 命令切换是用户主动中断，不进入错误日志链路，只打印普通提示方便现场确认。 */
    printf("自动恢复\t命令切换，取消恢复\r\n");
    FaultRecovery_ClearContext();
}

/**
 * @brief 轮询故障自动恢复状态机并在等待期结束后触发重试。
 *
 * 未激活恢复上下文时返回全零结果；恢复激活后立即标记本轮已经处理，并重新发布原错误状态，避免等待期间被空闲逻辑误清除。
 * 配置的重试次数为 0 时关闭恢复；其他情况按 FAULT_RECOVERY_INTERVAL_MS 节流检查，防止每轮主循环重复初始化电机、访问传感器和刷写日志。
 * 原错误属于电机驱动类或驱动初始化状态已经失效时，先重新执行 MotorCtrl_Init；随后统一读取全部部件参数，确认命令切换、传感器通信和电机位置刷新链路可用。
 * 初始化或部件检查失败时记录本次恢复失败并保留上下文；达到业务命令重试上限时停止恢复。检查成功且仍有额度时只在结果中返回原命令，由主循环负责真正重跑。
 * 部件检查成功只代表具备重跑条件，恢复上下文保持到重跑命令结束，再由 FaultRecovery_OnCommandFinished 根据最终结果清除或继续恢复。
 *
 * @return 恢复处理结果，包含是否已处理、是否需要重跑命令以及重跑命令号。
 */
FaultRecoveryResult FaultRecovery_Poll(void)
{
    FaultRecoveryResult result = {
        .handled = 0U,
        .should_retry_command = 0U,
        .retry_command = CMD_NONE,
    };
    uint32_t now;
    uint32_t check_ret;
    uint32_t retry_limit;
    uint8_t need_motor_init;

    /* 主循环每轮都会调用轮询；无活动上下文时立即返回“未处理”，让正常命令和空闲逻辑继续运行。 */
    if (!s_fault_recovery.active) {
        return result;
    }

    result.handled = 1U;                /* 告诉主循环本轮已由恢复逻辑处理，不再进入空闲兜底。 */
    FaultRecovery_RestoreErrorStatus(); /* 每轮都恢复原错误态，避免等待期间状态被其他空闲逻辑清掉。 */

    retry_limit = FaultRecovery_GetRetryLimit();
    if (retry_limit == 0U) {
        printf("自动恢复\t配置关闭，停止自动恢复\r\n");
        FaultRecovery_ClearContext();
        return result;
    }

    now = HAL_GetTick();
    /* 自动恢复按 1 秒节流，减少传感器/电机反复通信和重复日志。 */
    if ((uint32_t)(now - s_fault_recovery.last_check_tick) < FAULT_RECOVERY_INTERVAL_MS) {
        return result;
    }
    s_fault_recovery.last_check_tick = now; /* 记录本轮检查时刻，下一轮继续节流。 */

    need_motor_init = FaultRecovery_IsMotorDriverError(s_fault_recovery.error_code) ? 1U : 0U;
    if (!MotorCtrl_IsDriverInitValid()) {
        need_motor_init = 1U;
    }

    if (need_motor_init != 0U) {
        /* 电机驱动类故障或驱动状态已失效时，先重新下发 TMC5130 配置，再读取部件参数确认整机是否恢复。 */
        check_ret = MotorCtrl_Init();
        if ((check_ret == STATE_SWITCH) || HasEffectiveCommandSwitchRequest()) {
            FaultRecovery_Cancel("command switch");
            return result;
        }
        /* 需要重建电机驱动的恢复检查失败时保留本轮真实错误，本轮不允许重跑原测量命令。 */
        if (check_ret != NO_ERROR) {
            ErrorLog_Retry(ERROR_LOG_MODULE_SYSTEM,
                           ERROR_LOG_OP_AUTO_RECOVER,
                           ERROR_LOG_REASON_AUTO_RECOVER_FAIL,
                           1U,
                           1U,
                           check_ret);
            FaultRecovery_RecordFailure(check_ret);
            return result;
        }
    }

    /* 部件参数读取作为统一恢复确认点：命令可继续、传感器可通信、电机位置可被刷新。 */
    check_ret = Sensor_CheckAllPartParams();
    if ((check_ret == STATE_SWITCH) || HasEffectiveCommandSwitchRequest()) {
        FaultRecovery_Cancel("command switch");
        return result;
    }
    /* 无需重建驱动的健康检查失败同样锁存真实错误，禁止仅因检查路径较短就继续业务重试。 */
    if (check_ret != NO_ERROR) {
        ErrorLog_Retry(ERROR_LOG_MODULE_SYSTEM,
                       ERROR_LOG_OP_AUTO_RECOVER,
                       ERROR_LOG_REASON_AUTO_RECOVER_FAIL,
                       1U,
                       1U,
                       check_ret);
        FaultRecovery_RecordFailure(check_ret);
        return result;
    }

    /* 部件检查成功后仍需先核对业务重跑上限；只有尚有额度时才向主循环返回原命令。 */
    if (s_fault_recovery.command_retry_count >= retry_limit) {
        FaultRecovery_StopAfterMaxRetry(s_fault_recovery.error_code);
        return result;
    }

    s_fault_recovery.command_retry_count++;    /* 只统计真正进入业务测量的自动重跑次数，不限制部件参数读取。 */
    s_fault_recovery.awaiting_retry_result = 1U; /* 保持上下文，等重跑命令结束后判断成功或继续恢复。 */
    result.should_retry_command = 1U;          /* 恢复确认成功，通知主循环重跑原命令。 */
    result.retry_command = s_fault_recovery.command; /* 恢复模块不直接执行业务，只返回需要重跑的命令。 */

    /* 部件检查只证明具备重跑条件；完整恢复要等业务命令真正成功。 */
    return result;
}
