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
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
    case MOTOR_DISABLED:
    case MOTOR_RUN_TIMEOUT:
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
    case PARAM_RANGE_ERROR:
    case PARAM_ADDRESS_OVERFLOW:
    case PARAM_ERROR:
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

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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
 * @brief 自动恢复等待期间恢复原错误状态。
 *
 * 空闲错误兜底会根据全局状态做处理；恢复等待期必须保持原错误态，避免恢复上下文被提前清掉。
 */
static void FaultRecovery_RestoreErrorStatus(void)
{
    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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
    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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

    /* 错误 阶段：错误报警 模块：系统 操作：自动恢复 原因：自动恢复失败 处理：停止测量 */
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

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (FaultRecovery_IsNonRecoverableError(error_code)) {
        return;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (FaultRecovery_GetRetryLimit() == 0U) {
        return;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (!FaultRecovery_IsRecoverableCommand(command)) {
        return;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (s_fault_recovery.active && (s_fault_recovery.command == command)) {
        retry_count = s_fault_recovery.command_retry_count;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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

    /* 错误 阶段：错误报警 模块：系统 操作：自动恢复 原因：进入自动恢复 处理：继续尝试 */
    ErrorLog_Warn(ERROR_LOG_MODULE_SYSTEM,
                  ERROR_LOG_OP_AUTO_RECOVER,
                  ERROR_LOG_REASON_AUTO_RECOVER_START,
                  ERROR_LOG_ACTION_CONTINUE);
    FaultRecovery_RestoreErrorStatus();
}

/**
 * @brief 更新故障处理中的 FaultRecovery_UpdateAfterCommand 逻辑。
 *
 * @param command 命令值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (s_fault_recovery.active &&
        s_fault_recovery.awaiting_retry_result &&
        (s_fault_recovery.command == command)) {
        s_fault_recovery.awaiting_retry_result = 0U;

        /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
        if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
            FaultRecovery_ClearContext();
            return;
        }

        /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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
 * @brief 执行故障处理中的 FaultRecovery_Cancel 逻辑。
 *
 * @param reason 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void FaultRecovery_Cancel(const char *reason)
{
    (void)reason;

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (!s_fault_recovery.active) {
        return;
    }

    /* 命令切换是用户主动中断，不进入错误日志链路，只打印普通提示方便现场确认。 */
    printf("自动恢复\t命令切换，取消恢复\r\n");
    FaultRecovery_ClearContext();
}

/**
 * @brief 轮询故障自动恢复状态机并在等待期结束后触发重试。
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

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
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
        /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
        if (check_ret != NO_ERROR) {
            /* 错误 阶段：错误重试 模块：系统 操作：自动恢复 原因：自动恢复失败 尝试：1U/1U 错误码：check_ret 错误名：ErrorLog_GetCodeName(check_ret) */
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
    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (check_ret != NO_ERROR) {
        /* 错误 阶段：错误重试 模块：系统 操作：自动恢复 原因：自动恢复失败 尝试：1U/1U 错误码：check_ret 错误名：ErrorLog_GetCodeName(check_ret) */
        ErrorLog_Retry(ERROR_LOG_MODULE_SYSTEM,
                       ERROR_LOG_OP_AUTO_RECOVER,
                       ERROR_LOG_REASON_AUTO_RECOVER_FAIL,
                       1U,
                       1U,
                       check_ret);
        FaultRecovery_RecordFailure(check_ret);
        return result;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (s_fault_recovery.command_retry_count >= retry_limit) {
        FaultRecovery_StopAfterMaxRetry(s_fault_recovery.error_code);
        return result;
    }

    s_fault_recovery.command_retry_count++;    /* 只统计真正进入业务测量的自动重跑次数，不限制部件参数读取。 */
    s_fault_recovery.awaiting_retry_result = 1U; /* 保持上下文，等重跑命令结束后判断成功或继续恢复。 */
    result.should_retry_command = 1U;          /* 恢复确认成功，通知主循环重跑原命令。 */
    result.retry_command = s_fault_recovery.command; /* 恢复模块不直接执行业务，只返回需要重跑的命令。 */

    /* 错误 阶段：重试成功 模块：系统 操作：自动恢复 原因：恢复成功 尝试：s_fault_recovery.command_retry_count/retry_limit */
    ErrorLog_Recover(ERROR_LOG_MODULE_SYSTEM,
                     ERROR_LOG_OP_AUTO_RECOVER,
                     ERROR_LOG_REASON_RECOVER_OK,
                     s_fault_recovery.command_retry_count,
                     retry_limit);
    g_measurement.device_status.error_code = NO_ERROR; /* 恢复成功后清除全局错误码。 */
    return result;
}
