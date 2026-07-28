/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Duan Xuebin
 *
 * 说明:
 *  - DeviceParameters 中 command 仅用于command，不参与掉电参数校验
 *  - 从 sensorType 开始到 crc 之前的区域为持久化参数区
 */

#include "system_parameter.h"
#include "error_log.h"
#include "app_version.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include "Relay/relay_alarm_config.h"
#include "main.h"
#include "stateformodbus.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h> /* offsetof */

volatile MeasurementResult g_measurement = {0};   /* 测量结果 */
volatile DeviceParameters  g_deviceParams = {0};  /* 设备参数 */
static volatile uint8_t g_device_params_save_pending = 0; /* Deferred save request flag */
static volatile uint8_t g_device_params_save_retry_count = 0U; /* Deferred save retry count */
static volatile uint32_t g_device_params_save_request_tick = 0; /* Last deferred save request tick */
static volatile uint8_t g_device_params_write_snapshot_valid = 0U; /* 写参前快照是否有效。 */
static volatile uint8_t g_device_params_factory_restore_in_progress = 0U; /* 恢复出厂覆盖整套参数期间关闭并发写入口。 */
static DeviceParameters g_device_params_write_snapshot; /* 写参前快照，供主循环延后打印差异。 */
static const char *g_device_params_last_load_source = "UNKNOWN";
#ifndef DEVICE_PARAMS_BOOT_FULL_PRINT_ENABLE
#define DEVICE_PARAMS_BOOT_FULL_PRINT_ENABLE 1U
#endif
static void print_device_params_event(DeviceParamPrintEvent event, const DeviceParameters *params, const char *reason, const char *source);
#define AO_NORMAL_CURRENT_MIN_MA_X100 400U /* AO正常输出电流最小值，单位0.01mA。 */
#define AO_NORMAL_CURRENT_MAX_MA_X100 2000U /* AO正常输出电流最大值，单位0.01mA。 */
#define AO_OUTPUT_CURRENT_MIN_MA_X100 320U /* AO特殊电流最小值，单位0.01mA。 */
#define AO_OUTPUT_CURRENT_MAX_MA_X100 2400U /* AO特殊电流最大值，单位0.01mA。 */

typedef struct {
    CommandType command;
    uint32_t values[DEVICE_COMMAND_ARG_COUNT];
    uint32_t generations[DEVICE_COMMAND_ARG_COUNT];
    uint8_t valid;
} DeviceCommandArgumentSnapshot;

static volatile DeviceCommandArgumentSnapshot g_pending_command_arguments;
static DeviceCommandArgumentSnapshot g_active_command_arguments;
static volatile uint32_t g_command_argument_generations[DEVICE_COMMAND_ARG_COUNT];

/*
 * 函数用途：报告CPU2是否正在用出厂值整体覆盖运行参数。
 * 调用场景：UART5 FC10在解析候选参数前判断七个命令参数能否写入。
 * 关键约束：只读取易失标志，不阻塞、不打印，可在UART5中断上下文调用。
 */
bool DeviceParams_IsFactoryRestoreInProgress(void)
{
    return g_device_params_factory_restore_in_progress != 0U;
}

/* 返回指定命令前置参数对应的共享保持寄存器起始地址。 */
static uint16_t DeviceCommandArguments_RegisterAddress(DeviceCommandArgumentField field)
{
    static const uint16_t addresses[DEVICE_COMMAND_ARG_COUNT] = {
        HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,
        HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,
        HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT,
        HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,
        HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,
        HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,
        HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE
    };

    return (field < DEVICE_COMMAND_ARG_COUNT) ? addresses[field] : HOLDREGISTER_AMOUNT;
}

/* 从运行参数读取指定命令前置参数。 */
static uint32_t DeviceCommandArguments_ReadGlobal(DeviceCommandArgumentField field)
{
    switch (field) {
    case DEVICE_COMMAND_ARG_CALIBRATE_OIL_LEVEL:
        return g_deviceParams.calibrateOilLevel;
    case DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL:
        return g_deviceParams.calibrateWaterLevel;
    case DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT:
        return g_deviceParams.calibrateTankHeight;
    case DEVICE_COMMAND_ARG_SINGLE_POINT_MEASUREMENT_POSITION:
        return g_deviceParams.singlePointMeasurementPosition;
    case DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION:
        return g_deviceParams.singlePointMonitoringPosition;
    case DEVICE_COMMAND_ARG_DENSITY_DISTRIBUTION_OIL_LEVEL:
        return g_deviceParams.densityDistributionOilLevel;
    case DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE:
        return g_deviceParams.motorCommandDistance;
    default:
        return 0U;
    }
}

/* 把指定值写回运行参数；调用方必须持有短临界区。 */
static void DeviceCommandArguments_WriteGlobal(DeviceCommandArgumentField field, uint32_t value)
{
    switch (field) {
    case DEVICE_COMMAND_ARG_CALIBRATE_OIL_LEVEL:
        g_deviceParams.calibrateOilLevel = value;
        break;
    case DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL:
        g_deviceParams.calibrateWaterLevel = value;
        break;
    case DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT:
        g_deviceParams.calibrateTankHeight = value;
        break;
    case DEVICE_COMMAND_ARG_SINGLE_POINT_MEASUREMENT_POSITION:
        g_deviceParams.singlePointMeasurementPosition = value;
        break;
    case DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION:
        g_deviceParams.singlePointMonitoringPosition = value;
        break;
    case DEVICE_COMMAND_ARG_DENSITY_DISTRIBUTION_OIL_LEVEL:
        g_deviceParams.densityDistributionOilLevel = value;
        break;
    case DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE:
        g_deviceParams.motorCommandDistance = value;
        break;
    default:
        break;
    }
}

/* 在已关闭中断的短临界区内，把当前七字段绑定到待执行命令。 */
static void DeviceCommandArguments_CapturePendingLocked(CommandType command)
{
    uint32_t index;

    g_pending_command_arguments.command = command;
    g_pending_command_arguments.valid = (command != CMD_NONE) ? 1U : 0U;
    for (index = 0U; index < DEVICE_COMMAND_ARG_COUNT; index++) {
        g_pending_command_arguments.values[index] =
            DeviceCommandArguments_ReadGlobal((DeviceCommandArgumentField)index);
        g_pending_command_arguments.generations[index] =
            g_command_argument_generations[index];
    }
}

/*
 * 函数用途：恢复出厂完成后重新发布期间已经ACK的最新待执行命令。
 * 调用场景：整套默认参数保存完成、重新开放七字段写入口之前。
 * 关键约束：按最终出厂值重新绑定参数；重复恢复命令仍按不可自中断规则丢弃。
 */
static void DeviceCommandArguments_RebindAfterFactoryRestore(void)
{
    CommandType pending_command;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (g_pending_command_arguments.valid != 0U) {
        pending_command = g_pending_command_arguments.command;
        if (pending_command == CMD_RESTORE_FACTORY) {
            g_pending_command_arguments.valid = 0U;
            if (g_deviceParams.command == CMD_RESTORE_FACTORY) {
                g_deviceParams.command = CMD_NONE;
            }
        } else {
            g_deviceParams.command = pending_command;
            DeviceCommandArguments_CapturePendingLocked(pending_command);
        }
    }
    __set_PRIMASK(primask);
}

/*
 * 函数用途：为本次成功写入涉及的命令前置参数递增逐字段代次。
 * 调用场景：CPU2 FC10 候选参数全部校验通过并提交到 g_deviceParams 后。
 * 关键约束：同值写入也必须递增，防止当前命令收尾清零覆盖下一次同值请求。
 */
void DeviceCommandArguments_RecordWrite(uint16_t start, uint16_t count)
{
    uint32_t index;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (index = 0U; index < DEVICE_COMMAND_ARG_COUNT; index++) {
        uint16_t address =
            DeviceCommandArguments_RegisterAddress((DeviceCommandArgumentField)index);
        if (LtdModbus_RangeContains(start, count, address, REG_STRIDE)) {
            g_command_argument_generations[index]++;
        }
    }
    __set_PRIMASK(primask);
}

/*
 * 函数用途：把当前七字段与已经接受的待执行命令原子绑定。
 * 调用场景：Modbus 命令寄存器写入成功后。
 * 关键约束：只更新 pending 快照，不得覆盖正在执行命令的 active 快照。
 */
void DeviceCommandArguments_CapturePending(CommandType command)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    DeviceCommandArguments_CapturePendingLocked(command);
    __set_PRIMASK(primask);
}

/*
 * 函数用途：由 CPU2 内部路径入队正式命令并同步绑定当前七字段。
 * 调用场景：上电默认命令、串口命令和业务内部续接命令。
 * 关键约束：命令值和参数快照在同一短临界区发布。
 */
void DeviceCommand_Queue(CommandType command)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_deviceParams.command = command;
    DeviceCommandArguments_CapturePendingLocked(command);
    __set_PRIMASK(primask);
}

/*
 * 函数用途：原子判断是否存在能够切换当前流程的新命令。
 * 调用场景：阻塞测量、传感器通信和电机等待循环的既有退出检查。
 * 关键约束：保持原自中断白名单和重复命令规则，只防止条件清零覆盖并发到达的新命令。
 */
bool HasEffectiveCommandSwitchRequest(void)
{
    CommandType pending_command;
    CommandType current_command;
    uint32_t primask = __get_PRIMASK();
    bool switch_requested = false;

    __disable_irq();
    pending_command = g_deviceParams.command;
    current_command = g_measurement.device_status.current_command;
    if (pending_command != CMD_NONE) {
        if ((current_command != CMD_NONE) &&
            (pending_command == current_command) &&
            !IsSelfInterruptibleCommand(current_command)) {
            g_deviceParams.command = CMD_NONE;
            if ((g_pending_command_arguments.valid != 0U) &&
                (g_pending_command_arguments.command == pending_command)) {
                g_pending_command_arguments.valid = 0U;
            }
        } else {
            switch_requested = true;
        }
    } else if (new_command_ready != 0U) {
        switch_requested = true;
    }
    __set_PRIMASK(primask);
    return switch_requested;
}

/* 在已关闭中断的短临界区内，把匹配的 pending 参数提升为 active 参数。 */
static void DeviceCommandArguments_ActivatePendingLocked(CommandType command)
{
    uint32_t index;
    uint8_t pending_matches =
        ((g_pending_command_arguments.valid != 0U) &&
         (g_pending_command_arguments.command == command)) ? 1U : 0U;

    g_active_command_arguments.command = command;
    g_active_command_arguments.valid = 1U;
    for (index = 0U; index < DEVICE_COMMAND_ARG_COUNT; index++) {
        if (pending_matches != 0U) {
            g_active_command_arguments.values[index] =
                g_pending_command_arguments.values[index];
            g_active_command_arguments.generations[index] =
                g_pending_command_arguments.generations[index];
        } else {
            g_active_command_arguments.values[index] =
                DeviceCommandArguments_ReadGlobal((DeviceCommandArgumentField)index);
            g_active_command_arguments.generations[index] =
                g_command_argument_generations[index];
        }
    }
    if (pending_matches != 0U) {
        g_pending_command_arguments.valid = 0U;
    }
}

/*
 * 函数用途：原子取走一个待执行正式命令，并同步把其 pending 参数提升为 active 参数。
 * 调用场景：主循环在原始串口命令分支之后、自动恢复分支之前调用。
 * 关键约束：命令读取、条件清零和参数提升位于同一短临界区；不消费其它命令的 pending 快照。
 */
bool DeviceCommand_TakePending(CommandType *command)
{
    CommandType captured_command;
    uint32_t primask;

    if (command == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    captured_command = g_deviceParams.command;
    if (captured_command == CMD_NONE) {
        __set_PRIMASK(primask);
        return false;
    }

    g_deviceParams.command = CMD_NONE;
    DeviceCommandArguments_ActivatePendingLocked(captured_command);
    g_measurement.device_status.current_command = captured_command;
    *command = captured_command;
    __set_PRIMASK(primask);
    return true;
}

/*
 * 函数用途：在自动恢复重试与最后时刻到达的正式命令之间做一次原子仲裁。
 * 调用场景：FaultRecovery_Poll 决定重试后、主循环真正执行命令前。
 * 关键约束：有 pending 时保持正式命令优先；否则复用原 active 参数，并在同一临界区发布 current_command。
 * 返回值：true 表示选中了新的 pending 命令，false 表示继续自动重试。
 */
bool DeviceCommand_PrepareRecoveryExecution(CommandType retry_command,
                                            CommandType *selected_command)
{
    CommandType pending_command;
    uint32_t primask;
    bool pending_selected = false;

    if (selected_command == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    pending_command = g_deviceParams.command;
    if (pending_command != CMD_NONE) {
        g_deviceParams.command = CMD_NONE;
        DeviceCommandArguments_ActivatePendingLocked(pending_command);
        *selected_command = pending_command;
        pending_selected = true;
    } else {
        if ((g_active_command_arguments.valid == 0U) ||
            (g_active_command_arguments.command != retry_command)) {
            DeviceCommandArguments_ActivatePendingLocked(retry_command);
        }
        *selected_command = retry_command;
    }
    g_measurement.device_status.current_command = *selected_command;
    __set_PRIMASK(primask);
    return pending_selected;
}

/*
 * 函数用途：返回当前命令绑定的参数值；非命令上下文返回最新全局值。
 * 调用场景：测量、标定和电机业务消费七个前置参数时。
 * 关键约束：只有 active 命令与 current_command 一致时才读取快照。
 */
uint32_t DeviceCommandArguments_Get(DeviceCommandArgumentField field)
{
    if (field >= DEVICE_COMMAND_ARG_COUNT) {
        return 0U;
    }
    if ((g_active_command_arguments.valid != 0U) &&
        (g_active_command_arguments.command ==
         g_measurement.device_status.current_command)) {
        return g_active_command_arguments.values[field];
    }
    return DeviceCommandArguments_ReadGlobal(field);
}

/*
 * 函数用途：消费一次性参数后仅在该字段未被后续写入时清零全局值。
 * 调用场景：油位或水位标定完成后的原有清零位置。
 * 关键约束：后续同值写入也由逐字段代次识别，不能被当前命令收尾误清。
 */
void DeviceCommandArguments_ClearIfUnchanged(DeviceCommandArgumentField field)
{
    uint32_t primask;
    uint8_t active_matches;

    if (field >= DEVICE_COMMAND_ARG_COUNT) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    active_matches =
        ((g_active_command_arguments.valid != 0U) &&
         (g_active_command_arguments.command ==
          g_measurement.device_status.current_command)) ? 1U : 0U;
    if ((active_matches == 0U) ||
        ((g_command_argument_generations[field] ==
          g_active_command_arguments.generations[field]) &&
         (DeviceCommandArguments_ReadGlobal(field) ==
          g_active_command_arguments.values[field]))) {
        DeviceCommandArguments_WriteGlobal(field, 0U);
        g_command_argument_generations[field]++;
    }
    __set_PRIMASK(primask);
}
/* 将继电器报警输出枚举值转换成中文打印文本，便于现场调试查看。 */
static const char * relay_operating_mode_str(uint32_t value)
{
    switch ((RelayAlarmOperatingMode)value) {
    case RELAY_ALARM_OPERATING_DISABLED:
        return "禁用";
    case RELAY_ALARM_OPERATING_OUTPUT_PASSIVE:
        return "无源输出";
    default:
        return "未定义";
    }
}

/**
 * @brief 执行系统参数中的 relay_digital_source_str 逻辑。
 *
 * @param value 待处理数值。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
static const char * relay_digital_source_str(uint32_t value)
{
    switch ((RelayAlarmDigitalSource)value) {
    case RELAY_ALARM_DIGITAL_NONE:
        return "无";
    case RELAY_ALARM_DIGITAL_H:
        return "高报";
    case RELAY_ALARM_DIGITAL_HH:
        return "高高报";
    case RELAY_ALARM_DIGITAL_H_OR_HH:
        return "高报或高高报";
    case RELAY_ALARM_DIGITAL_L:
        return "低报";
    case RELAY_ALARM_DIGITAL_LL:
        return "低低报";
    case RELAY_ALARM_DIGITAL_L_OR_LL:
        return "低报或低低报";
    case RELAY_ALARM_DIGITAL_ANY:
        return "任意报警";
    default:
        return "未定义";
    }
}

/**
 * @brief 执行系统参数中的 relay_contact_type_str 逻辑。
 *
 * @param value 待处理数值。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
static const char * relay_contact_type_str(uint32_t value)
{
    switch ((RelayAlarmContactType)value) {
    case RELAY_ALARM_CONTACT_NORMALLY_OPEN:
        return "常开";
    case RELAY_ALARM_CONTACT_NORMALLY_CLOSED:
        return "常闭";
    default:
        return "未定义";
    }
}

/**
 * @brief 执行系统参数中的 relay_alarm_mode_str 逻辑。
 *
 * @param value 待处理数值。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
static const char * relay_alarm_mode_str(uint32_t value)
{
    switch ((RelayAlarmMode)value) {
    case RELAY_ALARM_MODE_OFF:
        return "关";
    case RELAY_ALARM_MODE_ON:
        return "开";
    case RELAY_ALARM_MODE_LATCHING:
        return "锁存";
    default:
        return "未定义";
    }
}

/**
 * @brief 执行系统参数中的 relay_alarm_source_str 逻辑。
 *
 * @param value 待处理数值。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
static const char * relay_alarm_source_str(uint32_t value)
{
    switch ((RelayAlarmSource)value) {
    case RELAY_ALARM_SOURCE_TANK_LEVEL:
        return "储罐液位";
    case RELAY_ALARM_SOURCE_LIQUID_TEMP:
        return "液相温度";
    case RELAY_ALARM_SOURCE_WATER_LEVEL:
        return "水位";
    case RELAY_ALARM_SOURCE_DISPLACER_POS:
        return "浮子位置";
    case RELAY_ALARM_SOURCE_NONE:
        return "无";
    default:
        return "未定义";
    }
}

static const char * relay_error_value_str(uint32_t value)
{
    switch ((RelayAlarmErrorValue)value) {
    case RELAY_ALARM_ERROR_NO_ALARM:
        return "无报警";
    case RELAY_ALARM_ERROR_HH_H:
        return "高高/高";
    case RELAY_ALARM_ERROR_H:
        return "高";
    case RELAY_ALARM_ERROR_L:
        return "低";
    case RELAY_ALARM_ERROR_LL_L:
        return "低低/低";
    case RELAY_ALARM_ERROR_ALL_ALARMS:
        return "全部报警";
    default:
        return "未定义";
    }
}

/* 将继电器清锁存命令转换成中文打印文本。 */
static const char * relay_clear_alarm_str(uint32_t value)
{
    switch (value) {
    case RELAY_ALARM_CLEAR_NO:
        return "否";
    case RELAY_ALARM_CLEAR_YES:
        return "是";
    default:
        return "未定义";
    }
}
static float relay_alarm_raw_to_float(uint32_t raw)
{
    float value;
    /* 按原始位转换 IEEE754 float，避免字段打印改变协议解释口径。 */
    memcpy(&value, &raw, sizeof(value));
    return value;
}

#ifndef DEVICE_PARAMS_SAVE_DEBOUNCE_MS
#define DEVICE_PARAMS_SAVE_DEBOUNCE_MS 100u /* 参数存储配置：设备 PARAMS 保存 DEBOUNCE 毫秒。 */
#endif
/* 主循环延后保存连续失败上限；到达3次后清待处理标志并保留存储故障码。 */
#define DEVICE_PARAMS_SAVE_DEFERRED_RETRY_LIMIT 3U

/* param_version和 magic 常量 */
#define DEVICE_PARAM_VERSION   (3u) /* 参数存储配置：设备 参数 版本。 */
#define DEVICE_PARAM_MAGIC     (0x4C54444Du)  /* 'LTDM' */
#define DENSITY_X100_PROTOCOL_VERSION (13u) /* 密度参数从协议13开始按 x100 存储。 */

/*
 * 持久化区域说明:
 *  - command 不参与掉电参数校验
 *  - 从 sensorType 起到 crc 前为持久化参数区
 */
#define DEVICE_PARAM_PERSIST_OFFSET   (offsetof(DeviceParameters, sensorType)) /* 参数结构中参与持久化校验的起始偏移。 */
#define DEVICE_PARAM_PERSIST_LEN      (offsetof(DeviceParameters, crc) - DEVICE_PARAM_PERSIST_OFFSET) /* 参数结构中参与持久化校验的字节长度。 */
#define DEVICE_PARAM_PERSIST_START(p) ((uint8_t *)(p) + DEVICE_PARAM_PERSIST_OFFSET) /* 参数存储配置：设备 参数 持久化 启动。 */

/**
 * @brief 执行系统参数中的 relay_alarm_float_to_raw 逻辑。
 *
 * @param value 待处理数值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t relay_alarm_float_to_raw(float value)
{
    uint32_t raw;
    /* 按结构或原始字节复制，保持系统参数协议/存储布局不被字段解释改变。 */
    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

/* ========================= 参数存储逻辑 ========================= */

/* 根据持久化区计算crc（不含command和crc字段） */
static uint32_t device_param_crc(const DeviceParameters *params)
{
    const uint8_t *crc_base = DEVICE_PARAM_PERSIST_START(params);
    const uint32_t crc_size = (uint32_t)DEVICE_PARAM_PERSIST_LEN;
    return CRC32_HAL((const uint8_t *)crc_base, crc_size);
}

/**
 * @brief 清除或复位系统参数中的 clear_relay_alarm_runtime_commands 逻辑。
 *
 * @param params 输入/输出指针。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void clear_relay_alarm_runtime_commands(DeviceParameters *params)
{
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        params->relayAlarm[channel].clear_alarm = RELAY_ALARM_CLEAR_NO;
    }
}
#define MOTOR_LOCAL_CIRC_MIN_001MM  (50000u)    /* 50.000mm，和motor_count_first_loop_circumference_mm模型下限保持一致 */
#define MOTOR_LOCAL_CIRC_MAX_001MM  (5000000u)  /* 5000.000mm，防止旧 reserved 脏值被当成有效周长 */
#define MOTOR_LOCAL_CIRC_FALLBACK_001MM (600000u) /* 默认 600.000mm，用于first_loop_circumference_mm本身也异常时兜底 */

static uint32_t device_params_default_motor_local_circ_001mm(void)
{
    uint32_t value = g_deviceParams.first_loop_circumference_mm * 100U;

    if ((value < MOTOR_LOCAL_CIRC_MIN_001MM) ||
        (value > MOTOR_LOCAL_CIRC_MAX_001MM)) {
        value = MOTOR_LOCAL_CIRC_FALLBACK_001MM;
    }
    return value;
}

/* 软件版本跟随当前固件，避免被 FRAM 里的旧参数覆盖。 */
static uint32_t density_param_scale_x10_to_x100(uint32_t raw)
{
    if (raw == 0U) {
        return 0U;
    }
    if (raw > (0xFFFFFFFFUL / DENSITY_PARAM_MIGRATE_FACTOR)) {
        return 0xFFFFFFFFUL;
    }
    return raw * DENSITY_PARAM_MIGRATE_FACTOR;
}

static uint32_t density_correction_scale_x10_to_x100(uint32_t raw)
{
    int64_t delta = (int64_t)raw - (int64_t)DENSITY_CORRECTION_OLD_BASE_RAW;
    int64_t value = (int64_t)DENSITY_CORRECTION_BASE_RAW +
                    (delta * (int64_t)DENSITY_PARAM_MIGRATE_FACTOR);

    if (value < 0) {
        return 0U;
    }
    if (value > 0xFFFFFFFFLL) {
        return 0xFFFFFFFFUL;
    }
    return (uint32_t)value;
}

static int migrate_density_params_runtime(void)
{
    if (g_deviceParams.protocolVersion >= DENSITY_X100_PROTOCOL_VERSION) {
        return 0;
    }

    g_deviceParams.oilLevelDensity = density_param_scale_x10_to_x100(g_deviceParams.oilLevelDensity);
    g_deviceParams.oilLevelThreshold = density_param_scale_x10_to_x100(g_deviceParams.oilLevelThreshold);
    g_deviceParams.oilLevelHysteresisThreshold = density_param_scale_x10_to_x100(g_deviceParams.oilLevelHysteresisThreshold);
    g_deviceParams.densityCorrection = density_correction_scale_x10_to_x100(g_deviceParams.densityCorrection);
    return 1;
}
static int apply_firmware_version_runtime(void)
{
    if (g_deviceParams.softwareVersion == CPU2_APP_VERSION_U32) {
        return 0;
    }

    g_deviceParams.softwareVersion = CPU2_APP_VERSION_U32;
    return 1;
}
/* 协议版本固定由当前固件维护。
 * 旧程序没有该语义，原reserved1位置默认为0；新程序统一写入当前协议，供CPU3判断共享数据能力。 */
static int apply_protocol_version_runtime(void)
{
    uint32_t old_protocol = g_deviceParams.protocolVersion;

    if (old_protocol == DEVICE_PROTOCOL_VERSION) {
        return 0;
    }

    if ((old_protocol < 10U) || (old_protocol > DEVICE_PROTOCOL_VERSION)) {
        g_deviceParams.bottom_encoder_correction_tank_height = 0U;
        g_deviceParams.fault_auto_recovery_retry_limit = FAULT_AUTO_RECOVERY_RETRY_DEFAULT;
    }

    if ((old_protocol < 14U) || (old_protocol > DEVICE_PROTOCOL_VERSION)) {
        g_deviceParams.si_profile_first_point = 5000U;
        g_deviceParams.si_profile_increment = 10000U;
        g_deviceParams.si_profile_dwell_time = 10U;
        g_deviceParams.si_profile_bottom_detect_interval = 1U;
    }

    if ((old_protocol < 25U) || (old_protocol > DEVICE_PROTOCOL_VERSION)) {
        /* 协议25首次赋予reserved3水位滞后时间语义，旧槽位不得直接作为有效秒数。 */
        g_deviceParams.water_level_hysteresis_time_s = 0U;
    }
    g_deviceParams.protocolVersion = DEVICE_PROTOCOL_VERSION;
    return 1;
}

/* 协议19及更早版本的AO原始布局，仅用于一次性FRAM迁移。 */
typedef struct {
    uint32_t range_start_01mm;
    uint32_t range_end_01mm;
    uint32_t normal_current_start_mA_x100;
    uint32_t normal_current_end_mA_x100;
    uint32_t alarm_high_01mm;
    uint32_t alarm_low_01mm;
    uint32_t initial_current_mA_x100;
    uint32_t high_current_mA_x100;
    uint32_t low_current_mA_x100;
    uint32_t fault_current_mA_x100;
    uint32_t debug_current_mA_x100;
    uint32_t output_enable;
    uint32_t reserved27;
} AoOutputLegacyV19;

/* 返回输出源允许的默认量程上限，单位0.1mm。 */
static int32_t ao_source_range_max_01mm(const DeviceParameters *params, uint32_t source)
{
    uint32_t max_01mm;

    if (params == NULL) {
        return 1;
    }

    if (source == AO_PROCESS_SOURCE_WATER_LEVEL) {
        max_01mm = params->water_tank_height;
        if (max_01mm == 0U) {
            max_01mm = params->tankHeight;
        }
    } else {
        max_01mm = params->tankHeight;
    }

    if (max_01mm == 0U) {
        max_01mm = 1U;
    }
    if (max_01mm > 0x7FFFFFFFUL) {
        max_01mm = 0x7FFFFFFFUL;
    }
    return (int32_t)max_01mm;
}

/* 为指定输出源装载0%和100%的默认量程。 */
static void ao_load_default_range(const DeviceParameters *params, AoOutputConfig *config)
{
    if (config == NULL) {
        return;
    }
    config->range_0_01mm = 0;
    config->range_100_01mm = ao_source_range_max_01mm(params, config->output_source);
}

/* 建立当前AO出厂配置，供恢复默认和旧协议迁移共用。 */
static void ao_load_default_config(const DeviceParameters *params, AoOutputConfig *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->work_mode = AO_WORK_MODE_DISABLED;
    config->current_mode = AO_CURRENT_MODE_NE;
    config->output_source = AO_PROCESS_SOURCE_TANK_LEVEL;
    config->current_correction_mA_x100 = 0;
    config->fixed_current_mA_x100 = 400U;
    ao_load_default_range(params, config);
    config->damping_x10_s = 0U;
    config->fault_mode = AO_FAULT_ACTION_OUTPUT_CURRENT;
    config->fault_current_mA_x100 = 2200U;
    config->error_level = 0U;
    config->power_on_current_mA_x100 = 400U;
    config->simulation_current_mA_x100 = 1200U;
}

/* 检查当前AO配置是否满足全部枚举、范围和量程约束。 */
static int ao_config_is_valid(const DeviceParameters *params, const AoOutputConfig *config)
{
    int32_t range_max_01mm;

    if ((params == NULL) || (config == NULL)) {
        return 0;
    }
    if ((config->work_mode > AO_WORK_MODE_HART_SLAVE_OUTPUT) ||
        (config->current_mode > AO_CURRENT_MODE_FIXED) ||
        (config->output_source > AO_PROCESS_SOURCE_WATER_LEVEL) ||
        (config->current_correction_mA_x100 < AO_CURRENT_CORRECTION_MIN_MA_X100) ||
        (config->current_correction_mA_x100 > AO_CURRENT_CORRECTION_MAX_MA_X100) ||
        (config->fault_mode > AO_FAULT_ACTION_HOLD_LAST_VALID) ||
        (config->error_level != 0U)) {
        return 0;
    }
    if ((config->fixed_current_mA_x100 < AO_FIXED_CURRENT_MIN_MA_X100) ||
        (config->fixed_current_mA_x100 > AO_FIXED_CURRENT_MAX_MA_X100) ||
        (config->fault_current_mA_x100 < AO_FAULT_CURRENT_MIN_MA_X100) ||
        (config->fault_current_mA_x100 > AO_FAULT_CURRENT_MAX_MA_X100) ||
        (config->power_on_current_mA_x100 < AO_INITIAL_CURRENT_MIN_MA_X100) ||
        (config->power_on_current_mA_x100 > AO_INITIAL_CURRENT_MAX_MA_X100) ||
        (config->simulation_current_mA_x100 < AO_SIMULATION_CURRENT_MIN_MA_X100) ||
        (config->simulation_current_mA_x100 > AO_SIMULATION_CURRENT_MAX_MA_X100) ||
        (config->damping_x10_s > AO_DAMPING_MAX_X10_S)) {
        return 0;
    }

    range_max_01mm = ao_source_range_max_01mm(params, config->output_source);
    if ((config->range_0_01mm < 0) ||
        (config->range_100_01mm < 0) ||
        (config->range_0_01mm > range_max_01mm) ||
        (config->range_100_01mm > range_max_01mm) ||
        (config->range_0_01mm == config->range_100_01mm)) {
        return 0;
    }
    return 1;
}

/* 将协议19及更早版本的AO字段迁移为当前配置，不清除其它现场参数。 */
static int migrate_ao_params_runtime(void)
{
    AoOutputLegacyV19 legacy;
    AoOutputConfig migrated;
    uint32_t old_protocol = g_deviceParams.protocolVersion;
    int32_t range_max_01mm;

    if (old_protocol >= 20U) {
        return 0;
    }

    memcpy(&legacy, (const void *)&g_deviceParams.ao_output, sizeof(legacy));
    ao_load_default_config((const DeviceParameters *)&g_deviceParams, &migrated);
    if ((old_protocol >= 10U) && (legacy.output_enable != 0U)) {
        migrated.work_mode = AO_WORK_MODE_CURRENT_OUTPUT;
    }

    range_max_01mm = ao_source_range_max_01mm((const DeviceParameters *)&g_deviceParams,
                                              AO_PROCESS_SOURCE_TANK_LEVEL);
    if ((old_protocol >= 11U) &&
        (legacy.range_start_01mm <= (uint32_t)range_max_01mm) &&
        (legacy.range_end_01mm <= (uint32_t)range_max_01mm) &&
        (legacy.range_start_01mm != legacy.range_end_01mm)) {
        migrated.range_0_01mm = (int32_t)legacy.range_start_01mm;
        migrated.range_100_01mm = (int32_t)legacy.range_end_01mm;
    }
    if ((legacy.initial_current_mA_x100 >= AO_INITIAL_CURRENT_MIN_MA_X100) &&
        (legacy.initial_current_mA_x100 <= AO_INITIAL_CURRENT_MAX_MA_X100)) {
        migrated.power_on_current_mA_x100 = legacy.initial_current_mA_x100;
    }
    if ((legacy.fault_current_mA_x100 >= AO_FAULT_CURRENT_MIN_MA_X100) &&
        (legacy.fault_current_mA_x100 <= AO_FAULT_CURRENT_MAX_MA_X100)) {
        migrated.fault_current_mA_x100 = legacy.fault_current_mA_x100;
    }
    if ((legacy.debug_current_mA_x100 >= AO_SIMULATION_CURRENT_MIN_MA_X100) &&
        (legacy.debug_current_mA_x100 <= AO_SIMULATION_CURRENT_MAX_MA_X100)) {
        migrated.simulation_current_mA_x100 = legacy.debug_current_mA_x100;
    }

    g_deviceParams.ao_output = migrated;
    return 1;
}

typedef enum {
    AO_LEGACY_FAULT_MINIMUM = 0U,
    AO_LEGACY_FAULT_MAXIMUM = 1U,
    AO_LEGACY_FAULT_LAST_VALID = 2U,
    AO_LEGACY_FAULT_ACTUAL_VALUE = 3U,
    AO_LEGACY_FAULT_SET_VALUE = 4U
} AoLegacyFaultMode;

/* 返回协议22及更早故障最小值/最大值对应的实际电流。 */
static uint32_t ao_legacy_fault_boundary_mA_x100(uint32_t current_mode, uint8_t maximum)
{
    uint32_t minimum = 350U;
    uint32_t maximum_value = 2260U;

    if (current_mode == AO_CURRENT_MODE_FIXED) {
        minimum = 400U;
        maximum_value = 2250U;
    } else if (current_mode == AO_CURRENT_MODE_US) {
        maximum_value = 2200U;
    }
    return (maximum != 0U) ? maximum_value : minimum;
}

/*
 * 函数用途：把协议20至22的五种故障模式迁移成协议23的两种故障动作。
 * 调用场景：FRAM旧布局迁移完成后、AO严格归一化之前调用。
 * 关键约束：不改变13槽位结构；旧最小/最大值固化到故障电流，旧最近有效值迁移为保持动作。
 */
static int migrate_ao_fault_action_runtime(void)
{
    AoOutputConfig *config = (AoOutputConfig *)&g_deviceParams.ao_output;
    uint32_t old_protocol = g_deviceParams.protocolVersion;
    uint32_t old_action = config->fault_mode;
    uint32_t old_fault_current = config->fault_current_mA_x100;
    uint32_t old_reserved = config->error_level;

    if (old_protocol == DEVICE_PROTOCOL_VERSION) {
        return 0;
    }

    if (old_protocol < 20U) {
        /* 前一步旧布局迁移已按协议23默认语义重建AO配置。 */
        config->error_level = 0U;
    } else if (old_protocol <= 22U) {
        switch ((AoLegacyFaultMode)old_action) {
        case AO_LEGACY_FAULT_MINIMUM:
            config->fault_mode = AO_FAULT_ACTION_OUTPUT_CURRENT;
            config->fault_current_mA_x100 =
                    ao_legacy_fault_boundary_mA_x100(config->current_mode, 0U);
            break;
        case AO_LEGACY_FAULT_MAXIMUM:
            config->fault_mode = AO_FAULT_ACTION_OUTPUT_CURRENT;
            config->fault_current_mA_x100 =
                    ao_legacy_fault_boundary_mA_x100(config->current_mode, 1U);
            break;
        case AO_LEGACY_FAULT_LAST_VALID:
            config->fault_mode = AO_FAULT_ACTION_HOLD_LAST_VALID;
            break;
        case AO_LEGACY_FAULT_ACTUAL_VALUE:
        case AO_LEGACY_FAULT_SET_VALUE:
        default:
            config->fault_mode = AO_FAULT_ACTION_OUTPUT_CURRENT;
            break;
        }
        config->error_level = 0U;
    } else if (old_protocol <= DEVICE_PROTOCOL_VERSION) {
        /* 协议23及以后已使用当前两种故障动作，升级时只清理旧预留槽。 */
        config->error_level = 0U;
    } else {
        /* 新于当前固件的未知语义按安全默认动作收敛，剩余范围交给归一化处理。 */
        config->fault_mode = AO_FAULT_ACTION_OUTPUT_CURRENT;
        config->error_level = 0U;
    }

    return ((config->fault_mode != old_action) ||
            (config->fault_current_mA_x100 != old_fault_current) ||
            (config->error_level != old_reserved)) ? 1 : 0;
}

/* 启动加载时归一化AO配置；用户写参走严格拒绝路径，不调用本函数兜底。 */
static int normalize_ao_params_runtime(void)
{
    AoOutputConfig normalized = g_deviceParams.ao_output;
    AoOutputConfig defaults;
    int changed = 0;

    ao_load_default_config((const DeviceParameters *)&g_deviceParams, &defaults);
    if (normalized.work_mode > AO_WORK_MODE_HART_SLAVE_OUTPUT) {
        normalized.work_mode = defaults.work_mode;
    }
    if (normalized.current_mode > AO_CURRENT_MODE_FIXED) {
        normalized.current_mode = defaults.current_mode;
    }
    if (normalized.output_source > AO_PROCESS_SOURCE_WATER_LEVEL) {
        normalized.output_source = defaults.output_source;
    }
    if ((g_deviceParams.protocolVersion < 26U) ||
        (g_deviceParams.protocolVersion > DEVICE_PROTOCOL_VERSION) ||
        (normalized.current_correction_mA_x100 < AO_CURRENT_CORRECTION_MIN_MA_X100) ||
        (normalized.current_correction_mA_x100 > AO_CURRENT_CORRECTION_MAX_MA_X100)) {
        /* 协议25及更早版本中该槽位为隐藏预留，升级时必须从零修正开始。 */
        normalized.current_correction_mA_x100 = defaults.current_correction_mA_x100;
    }
    if ((normalized.fixed_current_mA_x100 < AO_FIXED_CURRENT_MIN_MA_X100) ||
        (normalized.fixed_current_mA_x100 > AO_FIXED_CURRENT_MAX_MA_X100)) {
        normalized.fixed_current_mA_x100 = defaults.fixed_current_mA_x100;
    }
    if (normalized.damping_x10_s > AO_DAMPING_MAX_X10_S) {
        normalized.damping_x10_s = defaults.damping_x10_s;
    }
    if (normalized.fault_mode > AO_FAULT_ACTION_HOLD_LAST_VALID) {
        normalized.fault_mode = defaults.fault_mode;
    }
    if ((normalized.fault_current_mA_x100 < AO_FAULT_CURRENT_MIN_MA_X100) ||
        (normalized.fault_current_mA_x100 > AO_FAULT_CURRENT_MAX_MA_X100)) {
        normalized.fault_current_mA_x100 = defaults.fault_current_mA_x100;
    }
    if (normalized.error_level != 0U) {
        normalized.error_level = 0U;
    }
    if ((normalized.power_on_current_mA_x100 < AO_INITIAL_CURRENT_MIN_MA_X100) ||
        (normalized.power_on_current_mA_x100 > AO_INITIAL_CURRENT_MAX_MA_X100)) {
        normalized.power_on_current_mA_x100 = defaults.power_on_current_mA_x100;
    }
    if ((normalized.simulation_current_mA_x100 < AO_SIMULATION_CURRENT_MIN_MA_X100) ||
        (normalized.simulation_current_mA_x100 > AO_SIMULATION_CURRENT_MAX_MA_X100)) {
        normalized.simulation_current_mA_x100 = defaults.simulation_current_mA_x100;
    }
    if (ao_config_is_valid((const DeviceParameters *)&g_deviceParams, &normalized) == 0) {
        ao_load_default_range((const DeviceParameters *)&g_deviceParams, &normalized);
    }

    if (memcmp(&normalized, (const void *)&g_deviceParams.ao_output, sizeof(normalized)) != 0) {
        g_deviceParams.ao_output = normalized;
        changed = 1;
    }
    return changed;
}

/* 对FC10候选配置执行源切换、非法旧量程回退和严格校验。 */
int prepare_ao_params_for_write(const DeviceParameters *current, DeviceParameters *candidate)
{
    int32_t current_range_max_01mm;
    int32_t candidate_range_max_01mm;

    if ((current == NULL) || (candidate == NULL)) {
        return -1;
    }
    if (candidate->ao_output.output_source != current->ao_output.output_source) {
        ao_load_default_range(candidate, &candidate->ao_output);
    } else {
        current_range_max_01mm = ao_source_range_max_01mm(current,
                                                          current->ao_output.output_source);
        candidate_range_max_01mm = ao_source_range_max_01mm(candidate,
                                                            candidate->ao_output.output_source);
        if ((candidate_range_max_01mm != current_range_max_01mm) &&
            (candidate->ao_output.range_0_01mm == current->ao_output.range_0_01mm) &&
            (candidate->ao_output.range_100_01mm == current->ao_output.range_100_01mm) &&
            ((candidate->ao_output.range_0_01mm < 0) ||
             (candidate->ao_output.range_100_01mm < 0) ||
             (candidate->ao_output.range_0_01mm > candidate_range_max_01mm) ||
             (candidate->ao_output.range_100_01mm > candidate_range_max_01mm) ||
             (candidate->ao_output.range_0_01mm == candidate->ao_output.range_100_01mm))) {
            /* 单独改罐高且旧量程已非法时才成对回默认，保留仍有效的自定义量程。 */
            ao_load_default_range(candidate, &candidate->ao_output);
        }
    }
    return (ao_config_is_valid(candidate, &candidate->ao_output) != 0) ? 0 : -1;
}

/* 修正新增参数的非法值。
 * reserved6/reserved7 复用为position_count_mode和motor_count_first_loop_circumference_mm后，旧 FRAM 里可能残留任意非 0 值。
 * 这里既修正 RAM，又把是否修正返回给 load_device_params()，由加载流程决定是否写回 FRAM。 */
static int normalize_device_params_runtime(void)
{
    int changed = 0;
    uint32_t relay_changed_mask;
    uint32_t relay_invalid_mask;

    if ((g_deviceParams.position_count_mode != POSITION_COUNT_MODE_ENCODER) &&
        (g_deviceParams.position_count_mode != POSITION_COUNT_MODE_MOTOR)) {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
        changed = 1;
    }

    if ((g_deviceParams.position_source_auto_switch != POSITION_SOURCE_AUTO_SWITCH_DISABLE) &&
        (g_deviceParams.position_source_auto_switch != POSITION_SOURCE_AUTO_SWITCH_ENABLE)) {
        g_deviceParams.position_source_auto_switch = POSITION_SOURCE_AUTO_SWITCH_DISABLE;
        changed = 1;
    }

    if ((g_deviceParams.bottom_encoder_correction_enable != BOTTOM_ENCODER_CORRECTION_DISABLE) &&
        (g_deviceParams.bottom_encoder_correction_enable != BOTTOM_ENCODER_CORRECTION_ENABLE)) {
        g_deviceParams.bottom_encoder_correction_enable = BOTTOM_ENCODER_CORRECTION_DISABLE;
        changed = 1;
    }

    if (g_deviceParams.bottom_detect_mode > 1U) {
        g_deviceParams.bottom_detect_mode = 0U;
        changed = 1;
    }

    if ((g_deviceParams.motor_current < MOTOR_CURRENT_MIN) ||
        (g_deviceParams.motor_current > MOTOR_CURRENT_MAX)) {
        g_deviceParams.motor_current = MOTOR_CURRENT_DEFAULT;
        changed = 1;
    }

    if (normalize_ao_params_runtime() != 0) {
        changed = 1;
    }

    /* 先处理异常边界，避免系统参数状态机带故障继续运行。 */
    if (g_deviceParams.fault_auto_recovery_retry_limit > FAULT_AUTO_RECOVERY_RETRY_MAX) {
        g_deviceParams.fault_auto_recovery_retry_limit = FAULT_AUTO_RECOVERY_RETRY_DEFAULT;
        changed = 1;
    }

    if ((g_deviceParams.motor_count_first_loop_circumference_mm < MOTOR_LOCAL_CIRC_MIN_001MM) ||
        (g_deviceParams.motor_count_first_loop_circumference_mm > MOTOR_LOCAL_CIRC_MAX_001MM)) {
        g_deviceParams.motor_count_first_loop_circumference_mm =
            device_params_default_motor_local_circ_001mm();
        changed = 1;
    }

    /* 旧版本该位置为 reserved17，可能读到 0；这里补默认值，避免监测阈值过小。 */
    if (g_deviceParams.water_lag_cap_threshold == 0U) {
        g_deviceParams.water_lag_cap_threshold = 15000U;
        changed = 1;
    }

    /* Out-of-range old reserved22 values fall back to the new default: no bottom detect. */
    if (g_deviceParams.wartsila_bottom_detect_interval > 100U) {
        g_deviceParams.wartsila_bottom_detect_interval = 0U;
        changed = 1;
    }


    if (g_deviceParams.si_profile_first_point == 0U) {
        g_deviceParams.si_profile_first_point = 5000U;
        changed = 1;
    }
    if (g_deviceParams.water_level_hysteresis_time_s > 3600U) {
        g_deviceParams.water_level_hysteresis_time_s = 0U;
        changed = 1;
    }

    if (g_deviceParams.si_profile_increment == 0U) {
        g_deviceParams.si_profile_increment = 10000U;
        changed = 1;
    }
    if ((g_deviceParams.si_profile_dwell_time == 0U) ||
        (g_deviceParams.si_profile_dwell_time > 3600U)) {
        g_deviceParams.si_profile_dwell_time = 10U;
        changed = 1;
    }
    if ((g_deviceParams.si_profile_bottom_detect_interval == 0U) ||
        (g_deviceParams.si_profile_bottom_detect_interval > 1000U)) {
        g_deviceParams.si_profile_bottom_detect_interval = 1U;
        changed = 1;
    }
    relay_changed_mask = RelayAlarmConfig_Normalize(
        (DeviceParameters *)&g_deviceParams,
        &relay_invalid_mask);
    if (relay_changed_mask != 0U) {
        changed = 1;
    }
    if (relay_invalid_mask != 0U) {
        printf("[参数][上电][修复] 继电器非法配置已归一化并禁用通道 | 通道掩码=0x%02lX\r\n",
               (unsigned long)relay_invalid_mask);
    }

    return changed;
}

/*
 * 函数用途：Modbus 写参后归一化 AO 相关运行参数。
 * 调用场景：0x10 写保持寄存器后，保存 FRAM 前调用。
 * 关键约束：不处理继电器清报警等一次性命令；调用方负责同步保持寄存器和触发延后保存。
 */
int normalize_ao_params_after_write(void)
{
    return normalize_ao_params_runtime();
}

typedef enum {
    DEVICE_PARAM_SLOT_VALID = 0,
    DEVICE_PARAM_SLOT_UNINITIALIZED,
    DEVICE_PARAM_SLOT_SIZE_MISMATCH,
    DEVICE_PARAM_SLOT_VERSION_MISMATCH,
    DEVICE_PARAM_SLOT_CRC_ERROR,
    DEVICE_PARAM_SLOT_IO_ERROR
} DeviceParamSlotLoadResult;

/* 内部通用读取接口：
 * 1. 统一对 FRAM 槽位做 magic/version/CRC 校验；
 * 2. verbose=0 时用于静默判重，避免因为每次保存前判重而打大量日志；
 * 3. verbose=1 时用于正常加载诊断，保留详细失败原因。 */
static DeviceParamSlotLoadResult load_device_params_from_slot_impl(uint32_t base_addr,
                                                                   DeviceParameters *out,
                                                                   const char *slot_name,
                                                                   int verbose)
{
    DeviceParameters temp;
    char detail[128];

    if (FRAM_Read((uint8_t *)&temp,
                  base_addr,
                  sizeof(DeviceParameters)) != FRAM_STATUS_OK) {
        if (verbose) {
            printf("参数[%s]FRAM读取失败\r\n", slot_name);
        }
        return DEVICE_PARAM_SLOT_IO_ERROR;
    }

    if (temp.magic != DEVICE_PARAM_MAGIC)
    {
        if (verbose) {
            printf("参数[%s]魔术字不匹配: 0x%08lX\r\n", slot_name, (unsigned long)temp.magic);
        }
        if (verbose) {
            snprintf(detail, sizeof(detail),
                     "分区：%s,魔术字：0x%08lX,期望：0x%08lX",
                     slot_name,
                     (unsigned long)temp.magic,
                     (unsigned long)DEVICE_PARAM_MAGIC);
            /* 错误 阶段：错误报警 模块：参数 操作：参数校验 原因：魔术字不匹配 处理：继续尝试 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                ERROR_LOG_OP_PARAM_VALIDATE,
                                ERROR_LOG_REASON_PARAM_MAGIC,
                                ERROR_LOG_ACTION_CONTINUE,
                                detail);
        }
        return DEVICE_PARAM_SLOT_UNINITIALIZED;
    }

    if (temp.struct_size != sizeof(DeviceParameters))
    {
        if (verbose) {
            printf("参数[%s]结构体大小不匹配: FRAM=%lu, 当前：%lu\r\n",
                   slot_name,
                   (unsigned long)temp.struct_size,
                   (unsigned long)sizeof(DeviceParameters));
        }
        if (verbose) {
            snprintf(detail, sizeof(detail),
                     "分区：%s,FRAM大小：%lu,当前大小：%lu",
                     slot_name,
                     (unsigned long)temp.struct_size,
                     (unsigned long)sizeof(DeviceParameters));
            /* 错误 阶段：错误报警 模块：参数 操作：参数校验 原因：结构体大小不匹配 处理：继续尝试 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                ERROR_LOG_OP_PARAM_VALIDATE,
                                ERROR_LOG_REASON_PARAM_SIZE,
                                ERROR_LOG_ACTION_CONTINUE,
                                detail);
        }
        return DEVICE_PARAM_SLOT_SIZE_MISMATCH;
    }

    if (temp.param_version != DEVICE_PARAM_VERSION)
    {
        if (verbose) {
            printf("参数[%s]版本不匹配: FRAM=%lu, 当前：%lu\r\n",
                   slot_name,
                   (unsigned long)temp.param_version,
                   (unsigned long)DEVICE_PARAM_VERSION);
        }
        if (verbose) {
            snprintf(detail, sizeof(detail),
                     "分区：%s,FRAM版本：%lu,当前版本：%lu",
                     slot_name,
                     (unsigned long)temp.param_version,
                     (unsigned long)DEVICE_PARAM_VERSION);
            /* 错误 阶段：错误报警 模块：参数 操作：参数校验 原因：版本不匹配 处理：继续尝试 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                ERROR_LOG_OP_PARAM_VALIDATE,
                                ERROR_LOG_REASON_PARAM_VERSION,
                                ERROR_LOG_ACTION_CONTINUE,
                                detail);
        }
        return DEVICE_PARAM_SLOT_VERSION_MISMATCH;
    }

    {
        const uint32_t calc_crc = device_param_crc(&temp);
        if (calc_crc != temp.crc)
        {
            if (verbose) {
                printf("参数[%s] CRC不匹配: 计算=0x%08lX, FRAM=0x%08lX\r\n",
                       slot_name,
                       (unsigned long)calc_crc,
                       (unsigned long)temp.crc);
            }
            if (verbose) {
                snprintf(detail, sizeof(detail),
                         "分区：%s,计算CRC：0x%08lX,FRAMCRC：0x%08lX",
                         slot_name,
                         (unsigned long)calc_crc,
                         (unsigned long)temp.crc);
                /* 错误 阶段：错误报警 模块：参数 操作：参数校验 原因：CRC不匹配 处理：继续尝试 详情：detail */
                ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                    ERROR_LOG_OP_PARAM_VALIDATE,
                                    ERROR_LOG_REASON_PARAM_CRC,
                                    ERROR_LOG_ACTION_CONTINUE,
                                    detail);
            }
            return DEVICE_PARAM_SLOT_CRC_ERROR;
        }
    }

    *out = temp;
    return DEVICE_PARAM_SLOT_VALID;
}

/* 两个参数分区均不可用时，优先返回能够直接定位的校验原因。 */
static uint32_t device_param_error_from_slot_results(DeviceParamSlotLoadResult slot_a_result,
                                                     DeviceParamSlotLoadResult slot_b_result)
{
    if ((slot_a_result == DEVICE_PARAM_SLOT_CRC_ERROR) ||
        (slot_b_result == DEVICE_PARAM_SLOT_CRC_ERROR)) {
        return PARAM_CRC_ERROR;
    }

    if ((slot_a_result == DEVICE_PARAM_SLOT_SIZE_MISMATCH) ||
        (slot_b_result == DEVICE_PARAM_SLOT_SIZE_MISMATCH)) {
        return PARAM_STORAGE_SIZE_MISMATCH;
    }

    if ((slot_a_result == DEVICE_PARAM_SLOT_VERSION_MISMATCH) ||
        (slot_b_result == DEVICE_PARAM_SLOT_VERSION_MISMATCH)) {
        return PARAM_STORAGE_VERSION_MISMATCH;
    }

    if ((slot_a_result == DEVICE_PARAM_SLOT_UNINITIALIZED) &&
        (slot_b_result == DEVICE_PARAM_SLOT_UNINITIALIZED)) {
        return PARAM_UNINITIALIZED;
    }

    return PARAM_EEPROM_FAIL;
}

/* 把当前内存里的 g_deviceParams 整理成“准备写入 FRAM 的完整镜像”。
 * 这一步会顺手补齐 version/size/magic/crc，
 * 保证后面的“判断是否需要保存”与“真正写入的内容”完全一致。 */
static void build_saved_device_params(DeviceParameters *out)
{
    apply_firmware_version_runtime();
    apply_protocol_version_runtime();
    *out = g_deviceParams;
    clear_relay_alarm_runtime_commands(out);
    out->param_version = DEVICE_PARAM_VERSION;
    out->struct_size   = (uint32_t)sizeof(DeviceParameters);
    out->magic         = DEVICE_PARAM_MAGIC;
    out->crc           = device_param_crc(out);
}

/* 只比较持久化参数区（sensorType ~ crc 之前）。
 * command 属于运行态指令，不应该因为它的变化就触发整个参数区重写。 */
static int device_param_persist_equal(const DeviceParameters *lhs, const DeviceParameters *rhs)
{
    return memcmp(DEVICE_PARAM_PERSIST_START(lhs),
                  DEVICE_PARAM_PERSIST_START(rhs),
                  DEVICE_PARAM_PERSIST_LEN) == 0;
}

/* 清除写参前快照，避免一次写参的快照串到后续保存场景。 */
static void clear_device_params_write_snapshot(void)
{
    g_device_params_write_snapshot_valid = 0U;
}

/* 打印本次保存的参数差异：
 * Modbus 批量写参优先使用写入前快照，其它业务保存使用 FRAM 有效槽作为旧值。
 * 该函数只在真正准备写 FRAM 前调用，保存跳过或自修复场景不会输出误导性差异。 */
static void print_device_params_save_diff(const DeviceParameters *new_params,
                                          const DeviceParameters *slot_a,
                                          int slot_a_valid,
                                          const DeviceParameters *slot_b,
                                          int slot_b_valid,
                                          int mark_updated)
{
    const DeviceParameters *old_params = NULL;

    if (new_params == NULL) {
        clear_device_params_write_snapshot();
        return;
    }

    if (mark_updated != 0) {
        if (g_device_params_write_snapshot_valid != 0U) {
            old_params = &g_device_params_write_snapshot;
        } else if ((slot_a_valid != 0) && (slot_a != NULL)) {
            old_params = slot_a;
        } else if ((slot_b_valid != 0) && (slot_b != NULL)) {
            old_params = slot_b;
        }
    }

    if (old_params != NULL) {
        DeviceParams_PrintDiff(old_params, new_params);
    }

    clear_device_params_write_snapshot();
}

/* 每个A/B参数槽在一次保存轮次内允许的“写入+完整读回”尝试次数。 */
#define DEVICE_PARAM_SAVE_RETRY_LIMIT 3U

/*
 * 函数用途：对单个参数槽执行写入、读回和内容校验。
 * 调用场景：参数保存和单槽冗余修复。
 * 关键约束：每次最多尝试 3 次，只有完整镜像一致才返回成功。
 */
static int write_and_verify_device_param_slot(uint32_t address,
                                              const DeviceParameters *params,
                                              DeviceParameters *verify,
                                              DeviceParamSlotLoadResult *verify_result)
{
    uint32_t attempt;

    *verify_result = DEVICE_PARAM_SLOT_IO_ERROR;
    for (attempt = 0U; attempt < DEVICE_PARAM_SAVE_RETRY_LIMIT; attempt++) {
        if (FRAM_Write((const uint8_t *)params,
                       address,
                       sizeof(DeviceParameters)) != FRAM_STATUS_OK) {
            continue;
        }
        *verify_result = load_device_params_from_slot_impl(address, verify, "", 0);
        if ((*verify_result == DEVICE_PARAM_SLOT_VALID) &&
            (memcmp(verify, params, sizeof(DeviceParameters)) == 0)) {
            return 1;
        }
    }
    return 0;
}

/* 统一的参数保存入口：
 * mark_updated=1：说明这是一次“真正的参数变更”，写入并回读校验成功后递增
 *                 parameter_update_flag，让 CPU3 检测到持久化完成并补读保持寄存器。
 * force_write=1：忽略判重，强制回写 FRAM，主要用于 A/B 分区自修复这种场景。 */
static bool save_device_params_internal(int mark_updated, int force_write)
{
    DeviceParameters params;
    DeviceParameters slot_a;
    DeviceParameters slot_b;
    DeviceParameters verify_a;
    DeviceParameters verify_b;
    DeviceParamSlotLoadResult slot_a_result;
    DeviceParamSlotLoadResult slot_b_result;
    DeviceParamSlotLoadResult verify_a_result;
    DeviceParamSlotLoadResult verify_b_result;
    int slot_a_valid;
    int slot_b_valid;
    int verify_a_valid;
    int verify_b_valid;
    char detail[128];

    if (sizeof(DeviceParameters) > FRAM_PARAM_SLOT_SIZE)
    {
        printf("参数大小超出分区容量: 大小=%lu, 分区：%lu\r\n",
               (unsigned long)sizeof(DeviceParameters),
               (unsigned long)FRAM_PARAM_SLOT_SIZE);
        g_measurement.device_status.error_code = SYSTEM_BUFFER_CAPACITY_ERROR;
        clear_device_params_write_snapshot();
        return false;
    }

    build_saved_device_params(&params);
    /* 保存镜像里的元信息也回填到全局参数，避免后续参数打印显示旧 CRC。 */
    g_deviceParams.param_version = params.param_version;
    g_deviceParams.struct_size = params.struct_size;
    g_deviceParams.magic = params.magic;
    g_deviceParams.crc = params.crc;

    slot_a_result = load_device_params_from_slot_impl(FRAM_PARAM_A_ADDRESS, &slot_a, "A", 0);
    slot_b_result = load_device_params_from_slot_impl(FRAM_PARAM_B_ADDRESS, &slot_b, "B", 0);
    slot_a_valid = (slot_a_result == DEVICE_PARAM_SLOT_VALID) ? 1 : 0;
    slot_b_valid = (slot_b_result == DEVICE_PARAM_SLOT_VALID) ? 1 : 0;

    /* 仅当 A/B 两份 FRAM 都有效，且持久化区和本次待保存内容完全一致时，
     * 才真正跳过写入。
     * 只要 A/B 有一份异常或内容不一致，就仍然要重写，避免错过自修复机会。 */
    if (!force_write
        && slot_a_valid
        && slot_b_valid
        && device_param_persist_equal(&slot_a, &params)
        && device_param_persist_equal(&slot_b, &params))
    {
        /* A/B已与请求值一致也属于持久化确认完成，保证同值写入仍有可等待的完成标志。 */
        if (mark_updated) {
            g_measurement.device_status.parameter_update_flag++;
        }
        clear_device_params_write_snapshot();
        print_device_params_event(PARAM_PRINT_SAVE_SKIP, &params, NULL, NULL);
        return true;
    }

    verify_a_valid = write_and_verify_device_param_slot(FRAM_PARAM_A_ADDRESS,
                                                        &params,
                                                        &verify_a,
                                                        &verify_a_result);
    verify_b_valid = write_and_verify_device_param_slot(FRAM_PARAM_B_ADDRESS,
                                                        &params,
                                                        &verify_b,
                                                        &verify_b_result);

    /*
     * 一份新镜像已验证有效时允许降级继续，同一保存轮次已对另一槽完成有限修复尝试。
     * 只有两槽经过写入、读回和修复尝试后仍全部失败，才锁存 0x0011000C。
     */

    if ((!verify_a_valid) && (!verify_b_valid)) {
        snprintf(detail, sizeof(detail),
                 "A读取结果：%u,B读取结果：%u,每槽尝试：%u次",
                 (unsigned int)verify_a_result,
                 (unsigned int)verify_b_result,
                 (unsigned int)DEVICE_PARAM_SAVE_RETRY_LIMIT);
        /* 错误 阶段：错误报警 模块：参数 操作：参数校验 原因：ErrorLog_GetReasonByCode(PARAM_STORAGE_WRITE_VERIFY_FAILED) 处理：停止测量 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                            ERROR_LOG_OP_PARAM_VALIDATE,
                            ErrorLog_GetReasonByCode(PARAM_STORAGE_WRITE_VERIFY_FAILED),
                            ERROR_LOG_ACTION_STOP_MEASURE,
                            detail);
        if (g_measurement.device_status.error_code == NO_ERROR) {
            g_measurement.device_status.error_code = PARAM_STORAGE_WRITE_VERIFY_FAILED;
        }
        clear_device_params_write_snapshot();
        return false;
    }

    if ((!verify_a_valid) || (!verify_b_valid)) {
        snprintf(detail, sizeof(detail),
                 "A有效：%u,B有效：%u,已保留单槽新镜像并完成修复尝试",
                 (unsigned int)verify_a_valid,
                 (unsigned int)verify_b_valid);
        /* 错误 阶段：错误报警 模块：参数 操作：参数校验 原因：参数冗余槽降级 处理：继续运行 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                            ERROR_LOG_OP_PARAM_VALIDATE,
                            "参数冗余槽降级",
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
    }

    /* 仅清除上一轮写后校验故障，不覆盖测量、电机或传感器等其它故障。 */
    if (g_measurement.device_status.error_code == PARAM_STORAGE_WRITE_VERIFY_FAILED) {
        g_measurement.device_status.error_code = NO_ERROR;
    }

    /* 至少一份本次新镜像写后回读一致才发布完成；A/B 都失败时 CPU3 不能向外应答成功。 */
    if (mark_updated) {
        g_measurement.device_status.parameter_update_flag++;
    }

    print_device_params_save_diff(&params, &slot_a, slot_a_valid, &slot_b, slot_b_valid, mark_updated);
    print_device_params_event(PARAM_PRINT_SAVE_META,
                              &params,
                              NULL,
                              (verify_a_valid && verify_b_valid) ? "FRAM A/B" : "FRAM 单槽降级");
    return true;
}

/* 从指定分区读取并校验设备参数：返回1成功，0失败 */
static DeviceParamSlotLoadResult load_device_params_from_slot(uint32_t base_addr,
                                                              DeviceParameters *out,
                                                              const char *slot_name)
{
    return load_device_params_from_slot_impl(base_addr, out, slot_name, 1);
}

/* 对外的默认保存入口：
 * 表示“用户或业务逻辑确实修改了系统参数”，
 * 因此需要同时进行判重 + 必要时写 FRAM + 递增参数更新标志。 */
void save_device_params(void)
{
    (void)save_device_params_internal(1, 0);
}

/* Called from the Modbus write path after a 0x10 parameter update.
 * This function only sets a pending flag. The actual FRAM write is moved
 * to the main loop so the UART interrupt path stays short. */
void request_device_params_save(void)
{
    g_device_params_save_pending = 1;
    g_device_params_save_retry_count = 0U;
    g_device_params_save_request_tick = HAL_GetTick();
}

/* Handle deferred tasks in the main loop.
 * For now this only flushes pending parameter saves, but more deferred
 * work can be merged here later if needed. */
void process_device_params_deferred_tasks(void)
{
    uint32_t now;

    if (!g_device_params_save_pending)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - g_device_params_save_request_tick) < DEVICE_PARAMS_SAVE_DEBOUNCE_MS)
    {
        return;
    }

    if (FRAM_IsEmergencyReserved()) {
        /* 掉电紧急保存优先；等待期间不累计参数保存失败次数。 */
        return;
    }

    if (save_device_params_internal(1, 0)) {
        g_device_params_save_pending = 0U;
        g_device_params_save_retry_count = 0U;
        return;
    }

    if (g_device_params_save_retry_count < UINT8_MAX) {
        g_device_params_save_retry_count++;
    }
    if (g_device_params_save_retry_count >=
        DEVICE_PARAMS_SAVE_DEFERRED_RETRY_LIMIT) {
        g_device_params_save_pending = 0U;
    } else {
        g_device_params_save_request_tick = now;
    }
}
/**
 * @brief 加载或恢复系统参数中的 load_device_params 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int load_device_params(void)
{
    DeviceParameters temp;
    DeviceParamSlotLoadResult slot_a_result;
    DeviceParamSlotLoadResult slot_b_result;
    int loaded_from_a = 0;
    int params_normalized = 0;

    if (sizeof(DeviceParameters) > FRAM_PARAM_SLOT_SIZE)
    {
        printf("设备参数超出单分区容量: 大小=%lu, 分区：%lu\r\n", (unsigned long)sizeof(DeviceParameters), (unsigned long)FRAM_PARAM_SLOT_SIZE);
        g_measurement.device_status.error_code = SYSTEM_BUFFER_CAPACITY_ERROR;
        return 0;
    }

    slot_a_result = load_device_params_from_slot(FRAM_PARAM_A_ADDRESS, &temp, "A");
    if (slot_a_result == DEVICE_PARAM_SLOT_VALID)
    {
        loaded_from_a = 1;
    }
    else
    {
        slot_b_result = load_device_params_from_slot(FRAM_PARAM_B_ADDRESS, &temp, "B");
        if (slot_b_result == DEVICE_PARAM_SLOT_VALID) {
            /* 错误 阶段：重试成功 模块：参数 操作：FRAM参数分区回退 原因：A分区异常，使用B分区 尝试：1U/1U */
            ErrorLog_Recover(ERROR_LOG_MODULE_PARAM,
                             ERROR_LOG_OP_FRAM_FALLBACK,
                             ERROR_LOG_REASON_FRAM_FALLBACK,
                             1U,
                             1U);
        } else {
            g_measurement.device_status.error_code =
                device_param_error_from_slot_results(slot_a_result, slot_b_result);
            return 0;
        }
    }

    /* 按结构或原始字节复制，保持系统参数协议/存储布局不被字段解释改变。 */
    memcpy((void * volatile)&g_deviceParams, &temp, sizeof(DeviceParameters));

    g_deviceParams.command = g_deviceParams.powerOnDefaultCommand;
    params_normalized = migrate_density_params_runtime();
    params_normalized |= migrate_ao_params_runtime();
    params_normalized |= migrate_ao_fault_action_runtime();
    params_normalized |= normalize_device_params_runtime();
    params_normalized |= apply_firmware_version_runtime();
    params_normalized |= apply_protocol_version_runtime();

    /* 上电时如果 A 分区损坏、但 B 分区有效，
     * 这里只做“存储介质自修复”，不视为用户修改参数，
     * 所以不递增 parameter_update_flag，避免 CPU3 在上电后被平白触发一次“参数变更”。 */
    /* repair A from B without bumping update flag */
    if ((!loaded_from_a) || params_normalized) {
        (void)save_device_params_internal(0, 1);
    }

    g_device_params_last_load_source = loaded_from_a ? "FRAM A" : "FRAM B";
    printf("[参数][上电][加载] 设备参数加载成功 | 来源=%s\r\n", g_device_params_last_load_source);
    return 1;
}

/* ========================= 参数初始化 ========================= */

/* 初始化设备参数模块：连续 3 次读取失败才恢复出厂 */
void init_device_params(void)
{
    const int MAX_RETRY = 3;
    uint32_t load_error_code = PARAM_EEPROM_FAIL;
    int ok = 0;

    for (int attempt = 1; attempt <= MAX_RETRY; attempt++)
    {
        if (load_device_params())
        {
            ok = 1;
            /* 上电成功读取参数后固定打印一次完整参数表，
             * 不再依赖保存、A/B 修复或恢复出厂等附带路径。 */
            print_device_params_event(PARAM_PRINT_BOOT_FULL, NULL, "上电参数加载完成", g_device_params_last_load_source);
            break;
        }
        load_error_code = g_measurement.device_status.error_code;
        /* 错误 阶段：错误重试 模块：参数 操作：FRAM参数分区回退 原因：ErrorLog_GetReasonByCode(load_error_code) 尝试：attempt/MAX_RETRY 错误码：load_error_code 错误名：ErrorLog_GetCodeName(load_error_code) */
        ErrorLog_Retry(ERROR_LOG_MODULE_PARAM,
                       ERROR_LOG_OP_FRAM_FALLBACK,
                       ErrorLog_GetReasonByCode(load_error_code),
                       (uint32_t)attempt,
                       (uint32_t)MAX_RETRY,
                       load_error_code);
        HAL_Delay(100);
    }

    if (!ok)
    {
        /* 连续失败 3 次 -> 恢复出厂参数并保存 */
        RestoreFactoryParamsConfig(); /* 内部会调用 save_device_params() */

        g_measurement.device_status.error_code = load_error_code;
        /* 错误 阶段：错误报警 模块：参数 操作：FRAM参数分区回退 原因：ErrorLog_GetReasonByCode(load_error_code) 处理：使用默认参数 */
        ErrorLog_Warn(ERROR_LOG_MODULE_PARAM,
                      ERROR_LOG_OP_FRAM_FALLBACK,
                      ErrorLog_GetReasonByCode(load_error_code),
                      ERROR_LOG_ACTION_USE_DEFAULT_PARAM);
    }
}
/* ========================= 恢复出厂参数 ========================= */

/*
 * 恢复出厂参数配置
 * 注意: 这里设置的是默认值, 可根据实际项目需要调整
 */
void RestoreFactoryParamsConfig(void)
{
    /* 先关闭七个命令参数写入口，避免已ACK的新值随后被整体默认值覆盖。 */
    g_device_params_factory_restore_in_progress = 1U;
    /* 整体清零, 保证保留字段等为 0 */
    memset((void * volatile)&g_deviceParams, 0, sizeof(DeviceParameters));

    /* ---------------- 指令类字段 ---------------- */
    g_deviceParams.command               = CMD_NONE;
    g_deviceParams.powerOnDefaultCommand = CMD_NONE;

    /* ---------------- 基础参数 ---------------- */
    g_deviceParams.sensorType            = DSM_SENSOR;
    g_deviceParams.sensorID              = 1234567;
    g_deviceParams.sensorSoftwareVersion = 0x00010001;
    g_deviceParams.softwareVersion       = CPU2_APP_VERSION_U32;
    g_deviceParams.protocolVersion      = DEVICE_PROTOCOL_VERSION;
    g_deviceParams.error_auto_back_zero  = 0;   /* 默认关闭 */
    g_deviceParams.error_stop_measurement= 1;   /* 默认: 报错停止测量 */
    g_deviceParams.fault_auto_recovery_retry_limit = FAULT_AUTO_RECOVERY_RETRY_DEFAULT; /* 默认: 故障自动恢复最多重跑3次 */
    g_deviceParams.water_level_hysteresis_time_s = 0U; /* 预留参数默认0s，当前不参与水位流程 */
    g_deviceParams.position_source_auto_switch = POSITION_SOURCE_AUTO_SWITCH_DISABLE; /* 默认关闭 */

    /* ---------------- 电机与编码器参数 ---------------- */
    g_deviceParams.encoder_wheel_circumference_mm = 95000;  /* 0.001mm */
    g_deviceParams.max_motor_speed                = 200;    /* 0.01m/min */
    g_deviceParams.first_loop_circumference_mm    = 6000; /* 0.1mm */
    g_deviceParams.tape_thickness_mm              = 200;    /* 0.001mm */
    g_deviceParams.motor_current                 = MOTOR_CURRENT_DEFAULT; /* TMC5130 IRUN */
    g_deviceParams.position_count_mode                        = POSITION_COUNT_MODE_ENCODER; /* 默认编码轮记步 */
    g_deviceParams.motor_count_first_loop_circumference_mm =
        g_deviceParams.first_loop_circumference_mm * 100U; /* 0.001mm */

    /* ---------------- 扭力参数 ---------------- */
    g_deviceParams.empty_weight             = 0;
    g_deviceParams.empty_weight_upper_limit = 5000;
    g_deviceParams.empty_weight_lower_limit = 0;

    g_deviceParams.full_weight              = 5000;
    g_deviceParams.full_weight_upper_limit  = 30000;
    g_deviceParams.full_weight_lower_limit  = 1000;

    g_deviceParams.weight_upper_limit_ratio = 80;
    g_deviceParams.weight_lower_limit_ratio = 80;

    /* ---------------- 零点测量 ---------------- */
    g_deviceParams.zero_weight_threshold_ratio                 = 50;   /* 按算法需要调整 */
    g_deviceParams.weight_ignore_zone          = 1000; /* 0.1mm => 100mm */
    g_deviceParams.max_zero_deviation_distance = 1000;  /* 零点区域最大偏差值 0.1mm => 20mm */
    g_deviceParams.findZeroDownDistance        = 1000; /* 0.1mm => 100mm */

    /* ---------------- 液位测量 ---------------- */
    g_deviceParams.tankHeight                  = 200000; /* 0.1mm => 20000mm */
    g_deviceParams.liquid_sensor_distance_diff = 1500; /* 0.1mm => 150mm */
    g_deviceParams.blindZone                   = 3000; /* 0.1mm => 300mm */

    g_deviceParams.oilLevelThreshold                     = 150;     /* 项目自定义倍率/单位 */
    g_deviceParams.oilLevelHysteresisThreshold = 200;     /* 项目自定义倍率/单位 */
    g_deviceParams.liquidLevelMeasurementMethod= 0;		/* 0 空气+液体频率/2 1：按设置频率步进跟随 2 密度连续跟随 3.根据振动管跟随 4 连续相对频率 5 连续定频 */
    g_deviceParams.oilLevelFrequency                = 5200;      /* oilLevelFrequency */
    g_deviceParams.oilLevelDensity                = 0;      /* oilLevelDensity */

    /* ---------------- 水位测量参数 ---------------- */
    g_deviceParams.water_tank_height                = 200000; /* 0.1mm */
    g_deviceParams.water_level_mode                      = 0;      /* 0:慢速 */
    g_deviceParams.waterBlindZone                   = 300;    /* 0.1mm => 30mm */
    g_deviceParams.water_cap_threshold                      = 20000;      /* x1000 => 20.000pF */
    g_deviceParams.water_find_cap_threshold                      = 10000;      /* x1000 => 10.000pF */
    g_deviceParams.maxDownDistance                  = 3000;   /* 0.1mm => 300mm */
    g_deviceParams.zero_cap                         = 0;      /* 0.1pf */
    g_deviceParams.water_stable_threshold                      = 500;      /* 0.1mm */
    g_deviceParams.water_lag_cap_threshold  = 15000;  /* x1000 => 15.000pF */

    /* ---------------- 罐高/罐底测量 ---------------- */
    g_deviceParams.bottom_detect_mode      = 0;    /* 0=按项目定义 */
    g_deviceParams.bottom_angle_threshold  = 12;    /* 单位(度）/倍率*1 */
    g_deviceParams.bottom_weight_threshold = 500;

    g_deviceParams.refreshTankHeightFlag   = 0;  /* 不自动刷新 */
    g_deviceParams.maxTankHeightDeviation  = 1000; /* 0.1mm => 100mm */
    g_deviceParams.initialTankHeight       = 0;
    g_deviceParams.currentTankHeight       = 0;
    g_deviceParams.bottom_encoder_correction_enable = BOTTOM_ENCODER_CORRECTION_DISABLE; /* 默认: 罐底测量后不修正编码器 */

    /* ---------------- 密度/温度修正 ---------------- */
    g_deviceParams.densityCorrection       = DENSITY_CORRECTION_BASE_RAW;
    g_deviceParams.temperatureCorrection   = 1000;

    /* ---------------- 分布/区间测量参数 ---------------- */
    g_deviceParams.requireBottomMeasurement                   = 0;
    g_deviceParams.requireWaterMeasurement                   = 0;
    g_deviceParams.requireSinglePointDensity               = 0;

    g_deviceParams.spreadMeasurementOrder                   = 0;
    g_deviceParams.spreadMeasurementMode                   = 0;
    g_deviceParams.spreadMeasurementCount                   = 5;
    g_deviceParams.spreadMeasurementDistance                   = 10000; /* 0.1mm */
    g_deviceParams.spreadTopLimit              = 1000;  /* 0.1mm => 100mm */
    g_deviceParams.spreadBottomLimit           = 3000;  /* 0.1mm => 300mm */
    g_deviceParams.spreadPointHoverTime               = 10;

    g_deviceParams.intervalMeasurementTopLimit    = 0; /* 0.1mm */
    g_deviceParams.intervalMeasurementBottomLimit = 0; /* 0.1mm */

    /* ---------------- Wartsila 密度区间 ---------------- */
    g_deviceParams.wartsila_upper_density_limit      = 38000;
    g_deviceParams.wartsila_lower_density_limit      = 500;
    g_deviceParams.wartsila_density_interval         = 1000;
    g_deviceParams.wartsila_max_height_above_surface = 200; /* 0.1mm 或按定义 */
    g_deviceParams.wartsila_bottom_detect_interval  = 0; /* 0: no bottom detect, N: every N measurements */
    g_deviceParams.bottom_encoder_correction_tank_height = 0; /* 0: 编码器修正沿用液位罐高 */


    /* ---------------- SI Profile参数 ---------------- */
    g_deviceParams.si_profile_first_point = 5000U;
    g_deviceParams.si_profile_increment = 10000U;
    g_deviceParams.si_profile_dwell_time = 10U;
    g_deviceParams.si_profile_bottom_detect_interval = 1U;
    /* ---------------- 继电器报警输出（旧阈值兼容字段） ---------------- */

    /* ---------------- 继电器报警输出配置（三路） ---------------- */
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        volatile RelayAlarmConfig *cfg = &g_deviceParams.relayAlarm[channel];
        cfg->operating_mode = RELAY_ALARM_OPERATING_DISABLED;
        cfg->digital_source = RELAY_ALARM_DIGITAL_NONE;
        cfg->contact_type = RELAY_ALARM_CONTACT_NORMALLY_OPEN;
        cfg->alarm_mode = RELAY_ALARM_MODE_OFF;
        cfg->error_value = RELAY_ALARM_ERROR_NO_ALARM;
        cfg->alarm_source = RELAY_ALARM_SOURCE_TANK_LEVEL;
        cfg->HH_alarm_value = relay_alarm_float_to_raw(0.0f);
        cfg->H_alarm_value = relay_alarm_float_to_raw(0.0f);
        cfg->L_alarm_value = relay_alarm_float_to_raw(0.0f);
        cfg->LL_alarm_value = relay_alarm_float_to_raw(0.0f);
        cfg->alarm_hysteresis = relay_alarm_float_to_raw(0.0f);
        cfg->damping_factor = 0U;
        cfg->clear_alarm = RELAY_ALARM_CLEAR_NO;
    }

    /* ---------------- 4~20mA 输出 ---------------- */
    ao_load_default_config((const DeviceParameters *)&g_deviceParams, (AoOutputConfig *)&g_deviceParams.ao_output);

    /* ---------------- 指令参数 ---------------- */
    g_deviceParams.calibrateOilLevel                      = 0;
    g_deviceParams.calibrateWaterLevel                      = 0;
    g_deviceParams.calibrateTankHeight           = 0;
    g_deviceParams.singlePointMeasurementPosition = 0;
    g_deviceParams.singlePointMonitoringPosition  = 0;
    g_deviceParams.densityDistributionOilLevel                = 0;
    g_deviceParams.motorCommandDistance                    = 0;

    g_deviceParams.oilLevelHysteresisTime                    = 0;
    g_deviceParams.waterLevelCorrection                      = 0;
    g_deviceParams.lastOilCorrectionLevel                = 0;
    g_deviceParams.tankGasPhaseTemperature                        = 0;
    g_deviceParams.tapeExpansionCoefficient                      = 0;
    g_deviceParams.tapeCalibrationTemperature                    = 0;

    /* ---------------- 元信息与校验字段 ---------------- */
    g_deviceParams.param_version = DEVICE_PARAM_VERSION;
    g_deviceParams.struct_size   = (uint32_t)sizeof(DeviceParameters);
    g_deviceParams.magic         = DEVICE_PARAM_MAGIC;
    g_deviceParams.crc           = 0; /* save_device_params 内更新 */

    save_device_params();
    DeviceCommandArguments_RebindAfterFactoryRestore();
    /* 默认参数及FRAM保存均已完成，重新开放七个命令参数写入口。 */
    g_device_params_factory_restore_in_progress = 0U;
    print_device_params_event(PARAM_PRINT_FACTORY_RESET_FULL, NULL, "恢复出厂默认参数", NULL);
}

/* ========================= 参数打印 ========================= */

typedef enum {
    PARAM_PRINT_TYPE_U32 = 0,
    PARAM_PRINT_TYPE_I32,
    PARAM_PRINT_TYPE_U32_UNIT,
    PARAM_PRINT_TYPE_I32_UNIT,
    PARAM_PRINT_TYPE_HEX32,
    PARAM_PRINT_TYPE_VERSION_TEXT,
    PARAM_PRINT_TYPE_U32_01MM,
    PARAM_PRINT_TYPE_I32_01MM,
    PARAM_PRINT_TYPE_U32_001MM,
    PARAM_PRINT_TYPE_U32_01M_PER_MIN,
    PARAM_PRINT_TYPE_U32_01MA,
    PARAM_PRINT_TYPE_U32_001PF,
    PARAM_PRINT_TYPE_U32_01PF,
    PARAM_PRINT_TYPE_U32_DENSITY,
    PARAM_PRINT_TYPE_U32_OIL_LEVEL_THRESHOLD,
    PARAM_PRINT_TYPE_U32_DENSITY_OFFSET,
    PARAM_PRINT_TYPE_U32_TEMP_OFFSET,
    PARAM_PRINT_TYPE_U32_01C,
    PARAM_PRINT_TYPE_U32_000001_RATIO,
    PARAM_PRINT_TYPE_RELAY_BLOCK
} ParamPrintType;

typedef struct {
    const char *group;
    const char *name;
    uint16_t offset;
    ParamPrintType type;
    const char *unit; /* 直接工程单位或原始存储倍率，仅用于统一打印格式。 */
} ParamPrintItem;

typedef enum {
    RELAY_PARAM_PRINT_U32 = 0,
    RELAY_PARAM_PRINT_FLOAT_RAW
} RelayParamPrintType;

typedef struct {
    const char *name;
    uint16_t offset;
    RelayParamPrintType type;
} RelayParamPrintItem;

#define DEVICE_PARAM_ITEM(group_name, item_name, field_name, item_type, item_unit) \
    { (group_name), (item_name), (uint16_t)offsetof(DeviceParameters, field_name), (item_type), (item_unit) }
#define DEVICE_PARAM_RELAY_ITEM(group_name) \
    { (group_name), "继电器报警", 0U, PARAM_PRINT_TYPE_RELAY_BLOCK, NULL }
#define RELAY_PARAM_ITEM(item_name, field_name, item_type) \
    { (item_name), (uint16_t)offsetof(RelayAlarmConfig, field_name), (item_type) }

static const ParamPrintItem g_device_param_print_table[] = {
    DEVICE_PARAM_ITEM("指令", "当前指令", command, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("指令", "上电默认指令", powerOnDefaultCommand, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "传感器类型", sensorType, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "传感器编号", sensorID, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "传感器软件版本", sensorSoftwareVersion, PARAM_PRINT_TYPE_HEX32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "软件版本", softwareVersion, PARAM_PRINT_TYPE_HEX32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "软件版本文本", softwareVersion, PARAM_PRINT_TYPE_VERSION_TEXT, NULL),
    DEVICE_PARAM_ITEM("基础参数", "协议版本", protocolVersion, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "故障自动回零", error_auto_back_zero, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "故障停止测量", error_stop_measurement, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("基础参数", "故障自动恢复重跑次数", fault_auto_recovery_retry_limit, PARAM_PRINT_TYPE_U32_UNIT, "次"),
    DEVICE_PARAM_ITEM("基础参数", "位置源自动切换", position_source_auto_switch, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("电机与编码器", "编码轮周长", encoder_wheel_circumference_mm, PARAM_PRINT_TYPE_U32_001MM, "0.001mm"),
    DEVICE_PARAM_ITEM("电机与编码器", "电机限速", max_motor_speed, PARAM_PRINT_TYPE_U32_01M_PER_MIN, "0.01m/min"),
    DEVICE_PARAM_ITEM("电机与编码器", "电机运行电流", motor_current, PARAM_PRINT_TYPE_U32_UNIT, "IRUN"),
    DEVICE_PARAM_ITEM("电机与编码器", "首圈周长", first_loop_circumference_mm, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("电机与编码器", "尺带厚度", tape_thickness_mm, PARAM_PRINT_TYPE_U32_001MM, "0.001mm"),
    DEVICE_PARAM_ITEM("电机与编码器", "记步模式", position_count_mode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("电机与编码器", "电机局部周长", motor_count_first_loop_circumference_mm, PARAM_PRINT_TYPE_U32_001MM, "0.001mm"),
    DEVICE_PARAM_ITEM("扭力参数", "空载扭力", empty_weight, PARAM_PRINT_TYPE_I32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("扭力参数", "空载扭力上限", empty_weight_upper_limit, PARAM_PRINT_TYPE_U32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("扭力参数", "空载扭力下限", empty_weight_lower_limit, PARAM_PRINT_TYPE_U32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("扭力参数", "满载扭力", full_weight, PARAM_PRINT_TYPE_U32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("扭力参数", "满载扭力上限", full_weight_upper_limit, PARAM_PRINT_TYPE_U32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("扭力参数", "满载扭力下限", full_weight_lower_limit, PARAM_PRINT_TYPE_U32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("扭力参数", "碰撞上限比率", weight_upper_limit_ratio, PARAM_PRINT_TYPE_U32_UNIT, "%"),
    DEVICE_PARAM_ITEM("扭力参数", "碰撞下限比率", weight_lower_limit_ratio, PARAM_PRINT_TYPE_U32_UNIT, "%"),
    DEVICE_PARAM_ITEM("零点参数", "零点阈值比例", zero_weight_threshold_ratio, PARAM_PRINT_TYPE_U32_UNIT, "%"),
    DEVICE_PARAM_ITEM("零点参数", "扭力忽略区", weight_ignore_zone, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("零点参数", "零点最大偏差", max_zero_deviation_distance, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("零点参数", "找零下行距离", findZeroDownDistance, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("液位参数", "液位罐高", tankHeight, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("液位参数", "液位探头距差", liquid_sensor_distance_diff, PARAM_PRINT_TYPE_I32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("液位参数", "液位盲区", blindZone, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("液位参数", "液位找液阈值", oilLevelThreshold, PARAM_PRINT_TYPE_U32_OIL_LEVEL_THRESHOLD, NULL),
    DEVICE_PARAM_ITEM("液位参数", "液位滞后阈值", oilLevelHysteresisThreshold, PARAM_PRINT_TYPE_U32_OIL_LEVEL_THRESHOLD, NULL),
    DEVICE_PARAM_ITEM("液位参数", "液位测量方式", liquidLevelMeasurementMethod, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("液位参数", "液位跟随频率", oilLevelFrequency, PARAM_PRINT_TYPE_U32_UNIT, "Hz"),
    DEVICE_PARAM_ITEM("液位参数", "液位跟随密度", oilLevelDensity, PARAM_PRINT_TYPE_U32_DENSITY, "0.01kg/m3"),
    DEVICE_PARAM_ITEM("液位参数", "液位滞后时间", oilLevelHysteresisTime, PARAM_PRINT_TYPE_U32_UNIT, "s"),
    DEVICE_PARAM_ITEM("水位参数", "水位罐高", water_tank_height, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("水位参数", "水位测量方式", water_level_mode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("水位参数", "水位盲区", waterBlindZone, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("水位参数", "水位电容阈值", water_cap_threshold, PARAM_PRINT_TYPE_U32_001PF, "0.001pF"),
    DEVICE_PARAM_ITEM("水位参数", "水位寻找电容阈值", water_find_cap_threshold, PARAM_PRINT_TYPE_U32_001PF, "0.001pF"),
    DEVICE_PARAM_ITEM("水位参数", "水位最大下行距离", maxDownDistance, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("水位参数", "水位零点电容", zero_cap, PARAM_PRINT_TYPE_U32_01PF, "0.1pF"),
    DEVICE_PARAM_ITEM("水位参数", "水位稳定阈值", water_stable_threshold, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("水位参数", "水位滞后电容阈值", water_lag_cap_threshold, PARAM_PRINT_TYPE_U32_001PF, "0.001pF"),
    DEVICE_PARAM_ITEM("水位参数", "水位滞后时间", water_level_hysteresis_time_s, PARAM_PRINT_TYPE_U32_UNIT, "s"),
    DEVICE_PARAM_ITEM("水位参数", "水位修正值", waterLevelCorrection, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "罐底检测模式", bottom_detect_mode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "探底角度阈值", bottom_angle_threshold, PARAM_PRINT_TYPE_U32_UNIT, "deg"),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "探底扭力阈值", bottom_weight_threshold, PARAM_PRINT_TYPE_U32_UNIT, "计数"),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "更新罐高标志", refreshTankHeightFlag, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "实测罐高最大偏差", maxTankHeightDeviation, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "初始罐高", initialTankHeight, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "当前罐高", currentTankHeight, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("罐底/罐高参数", "罐底后编码器修正", bottom_encoder_correction_enable, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("修正参数", "密度修正值", densityCorrection, PARAM_PRINT_TYPE_U32_DENSITY_OFFSET, "0.01kg/m3"),
    DEVICE_PARAM_ITEM("修正参数", "温度修正值", temperatureCorrection, PARAM_PRINT_TYPE_U32_TEMP_OFFSET, "0.1C"),
    DEVICE_PARAM_ITEM("分布/区间参数", "是否测罐底", requireBottomMeasurement, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("分布/区间参数", "是否测水位", requireWaterMeasurement, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("分布/区间参数", "是否测单点密度", requireSinglePointDensity, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("分布/区间参数", "分布测顺序", spreadMeasurementOrder, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("分布/区间参数", "分布测模式", spreadMeasurementMode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("分布/区间参数", "分布测点数", spreadMeasurementCount, PARAM_PRINT_TYPE_U32_UNIT, "点"),
    DEVICE_PARAM_ITEM("分布/区间参数", "分布点间距", spreadMeasurementDistance, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("分布/区间参数", "最高点距液面", spreadTopLimit, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("分布/区间参数", "最低点距罐底", spreadBottomLimit, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("分布/区间参数", "分布点悬停时间", spreadPointHoverTime, PARAM_PRINT_TYPE_U32_UNIT, "s"),
    DEVICE_PARAM_ITEM("分布/区间参数", "区间测量上限", intervalMeasurementTopLimit, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("分布/区间参数", "区间测量下限", intervalMeasurementBottomLimit, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("瓦锡兰参数", "最高密度点", wartsila_upper_density_limit, PARAM_PRINT_TYPE_U32_UNIT, "mm"),
    DEVICE_PARAM_ITEM("瓦锡兰参数", "最低密度点", wartsila_lower_density_limit, PARAM_PRINT_TYPE_U32_UNIT, "mm"),
    DEVICE_PARAM_ITEM("瓦锡兰参数", "密度点间距", wartsila_density_interval, PARAM_PRINT_TYPE_U32_UNIT, "mm"),
    DEVICE_PARAM_ITEM("瓦锡兰参数", "最高点液面距", wartsila_max_height_above_surface, PARAM_PRINT_TYPE_U32_UNIT, "mm"),
    DEVICE_PARAM_ITEM("瓦锡兰参数", "瓦锡兰探底间隔", wartsila_bottom_detect_interval, PARAM_PRINT_TYPE_U32_UNIT, "次"),
    DEVICE_PARAM_ITEM("瓦锡兰参数", "探底修正罐高", bottom_encoder_correction_tank_height, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_RELAY_ITEM("继电器报警输出参数"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "工作模式", ao_output.work_mode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "电流模式", ao_output.current_mode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "输出源", ao_output.output_source, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "电流修正", ao_output.current_correction_mA_x100, PARAM_PRINT_TYPE_I32_UNIT, "0.01mA"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "固定电流", ao_output.fixed_current_mA_x100, PARAM_PRINT_TYPE_U32_01MA, "0.01mA"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "0%量程", ao_output.range_0_01mm, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "100%量程", ao_output.range_100_01mm, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "阻尼", ao_output.damping_x10_s, PARAM_PRINT_TYPE_U32_01C, "0.1s"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "故障动作", ao_output.fault_mode, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "故障电流", ao_output.fault_current_mA_x100, PARAM_PRINT_TYPE_U32_01MA, "0.01mA"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "隐藏预留", ao_output.error_level, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "初始电流", ao_output.power_on_current_mA_x100, PARAM_PRINT_TYPE_U32_01MA, "0.01mA"),
    DEVICE_PARAM_ITEM("4-20mA/AO参数", "仿真电流", ao_output.simulation_current_mA_x100, PARAM_PRINT_TYPE_U32_01MA, "0.01mA"),
    DEVICE_PARAM_ITEM("指令参数", "标定液位值", calibrateOilLevel, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("指令参数", "标定水位值", calibrateWaterLevel, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("指令参数", "标定罐高值", calibrateTankHeight, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("指令参数", "单点测量位置", singlePointMeasurementPosition, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("指令参数", "单点监测位置", singlePointMonitoringPosition, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("指令参数", "密度分布测量液位", densityDistributionOilLevel, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("指令参数", "电机指令距离", motorCommandDistance, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("尺带补偿参数", "上次液位修正液位", lastOilCorrectionLevel, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("尺带补偿参数", "气相温度", tankGasPhaseTemperature, PARAM_PRINT_TYPE_U32_01C, "0.1C"),
    DEVICE_PARAM_ITEM("尺带补偿参数", "尺带伸缩率", tapeExpansionCoefficient, PARAM_PRINT_TYPE_U32_000001_RATIO, "0.000001"),
    DEVICE_PARAM_ITEM("尺带补偿参数", "尺带标定温度", tapeCalibrationTemperature, PARAM_PRINT_TYPE_U32_01C, "0.1C"),
    DEVICE_PARAM_ITEM("SI参数", "Profile首点", si_profile_first_point, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("SI参数", "Profile步距", si_profile_increment, PARAM_PRINT_TYPE_U32_01MM, "0.1mm"),
    DEVICE_PARAM_ITEM("SI参数", "Profile停留", si_profile_dwell_time, PARAM_PRINT_TYPE_U32_UNIT, "s"),
    DEVICE_PARAM_ITEM("SI参数", "Profile探底频次", si_profile_bottom_detect_interval, PARAM_PRINT_TYPE_U32_UNIT, "次"),
    DEVICE_PARAM_ITEM("元信息/CRC", "参数版本号", param_version, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("元信息/CRC", "结构体大小", struct_size, PARAM_PRINT_TYPE_U32, NULL),
    DEVICE_PARAM_ITEM("元信息/CRC", "魔术字", magic, PARAM_PRINT_TYPE_HEX32, NULL),
    DEVICE_PARAM_ITEM("元信息/CRC", "参数CRC32", crc, PARAM_PRINT_TYPE_HEX32, NULL)
};

static const RelayParamPrintItem g_relay_param_print_table[] = {
    RELAY_PARAM_ITEM("模式", operating_mode, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("报警位", digital_source, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("接点", contact_type, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("报警模式", alarm_mode, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("无效值", error_value, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("报警源", alarm_source, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("HH", HH_alarm_value, RELAY_PARAM_PRINT_FLOAT_RAW),
    RELAY_PARAM_ITEM("H", H_alarm_value, RELAY_PARAM_PRINT_FLOAT_RAW),
    RELAY_PARAM_ITEM("L", L_alarm_value, RELAY_PARAM_PRINT_FLOAT_RAW),
    RELAY_PARAM_ITEM("LL", LL_alarm_value, RELAY_PARAM_PRINT_FLOAT_RAW),
    RELAY_PARAM_ITEM("滞回", alarm_hysteresis, RELAY_PARAM_PRINT_FLOAT_RAW),
    RELAY_PARAM_ITEM("阻尼", damping_factor, RELAY_PARAM_PRINT_U32),
    RELAY_PARAM_ITEM("清锁存", clear_alarm, RELAY_PARAM_PRINT_U32)
};

static uint32_t device_param_item_read_u32(const DeviceParameters *params, const ParamPrintItem *item)
{
    uint32_t value;
    const uint8_t *base;

    if ((params == NULL) || (item == NULL)) {
        return 0U;
    }

    if (item->offset == (uint16_t)offsetof(DeviceParameters, command)) {
        return (uint32_t)params->command;
    }
    if (item->offset == (uint16_t)offsetof(DeviceParameters, powerOnDefaultCommand)) {
        return (uint32_t)params->powerOnDefaultCommand;
    }

    base = (const uint8_t *)params;
    memcpy(&value, base + item->offset, sizeof(value));
    return value;
}

static int32_t device_param_item_read_i32(const DeviceParameters *params, const ParamPrintItem *item)
{
    int32_t value;
    const uint8_t *base = (const uint8_t *)params;
    memcpy(&value, base + item->offset, sizeof(value));
    return value;
}

static uint32_t relay_param_item_read_u32(const RelayAlarmConfig *cfg, const RelayParamPrintItem *item)
{
    uint32_t value;
    const uint8_t *base = (const uint8_t *)cfg;
    memcpy(&value, base + item->offset, sizeof(value));
    return value;
}

/*
 * 函数用途：返回继电器配置字段的枚举含义。
 * 调用场景：继电器参数 diff 打印，和全量打印保持同一套中文解释。
 * 关键约束：只解释枚举字段，阈值和阻尼等数值字段返回 NULL。
 */
static const char *relay_param_value_desc(const RelayParamPrintItem *item, uint32_t value)
{
    if (item == NULL) {
        return NULL;
    }

    switch (item->offset) {
    case (uint16_t)offsetof(RelayAlarmConfig, operating_mode):
        return relay_operating_mode_str(value);
    case (uint16_t)offsetof(RelayAlarmConfig, digital_source):
        return relay_digital_source_str(value);
    case (uint16_t)offsetof(RelayAlarmConfig, contact_type):
        return relay_contact_type_str(value);
    case (uint16_t)offsetof(RelayAlarmConfig, alarm_mode):
        return relay_alarm_mode_str(value);
    case (uint16_t)offsetof(RelayAlarmConfig, error_value):
        return relay_error_value_str(value);
    case (uint16_t)offsetof(RelayAlarmConfig, alarm_source):
        return relay_alarm_source_str(value);
    case (uint16_t)offsetof(RelayAlarmConfig, clear_alarm):
        return relay_clear_alarm_str(value);
    default:
        return NULL;
    }
}
static int device_param_item_can_diff(const ParamPrintItem *item)
{
    if (item == NULL) {
        return 0;
    }
    if ((item->type == PARAM_PRINT_TYPE_RELAY_BLOCK) ||
        (item->type == PARAM_PRINT_TYPE_VERSION_TEXT)) {
        return 0;
    }
    if ((item->offset == (uint16_t)offsetof(DeviceParameters, command)) ||
        (item->offset == (uint16_t)offsetof(DeviceParameters, param_version)) ||
        (item->offset == (uint16_t)offsetof(DeviceParameters, struct_size)) ||
        (item->offset == (uint16_t)offsetof(DeviceParameters, magic)) ||
        (item->offset == (uint16_t)offsetof(DeviceParameters, crc))) {
        return 0;
    }
    return 1;
}

static void print_relay_alarm_params(const DeviceParameters *params)
{
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        const RelayAlarmConfig *cfg = &params->relayAlarm[channel];
        printf("  继电器%lu报警\r\n", (unsigned long)(channel + 1U));
        printf("    输出配置 : 模式=%lu(%s) 接点=%lu(%s)\r\n",
               (unsigned long)cfg->operating_mode,
               relay_operating_mode_str(cfg->operating_mode),
               (unsigned long)cfg->contact_type,
               relay_contact_type_str(cfg->contact_type));
        printf("    报警配置 : 报警源=%lu(%s) 报警位=%lu(%s) 报警模式=%lu(%s) 无效值=%lu(%s)\r\n",
               (unsigned long)cfg->alarm_source,
               relay_alarm_source_str(cfg->alarm_source),
               (unsigned long)cfg->digital_source,
               relay_digital_source_str(cfg->digital_source),
               (unsigned long)cfg->alarm_mode,
               relay_alarm_mode_str(cfg->alarm_mode),
               (unsigned long)cfg->error_value,
               relay_error_value_str(cfg->error_value));
        printf("    阈值配置 : HH=%.1f H=%.1f L=%.1f LL=%.1f 滞回=%.1f 阻尼=%lu\r\n",
               relay_alarm_raw_to_float(cfg->HH_alarm_value),
               relay_alarm_raw_to_float(cfg->H_alarm_value),
               relay_alarm_raw_to_float(cfg->L_alarm_value),
               relay_alarm_raw_to_float(cfg->LL_alarm_value),
               relay_alarm_raw_to_float(cfg->alarm_hysteresis),
               (unsigned long)cfg->damping_factor);
        printf("    运行命令 : 清锁存=%lu(%s)\r\n",
               (unsigned long)cfg->clear_alarm,
               relay_clear_alarm_str(cfg->clear_alarm));
    }
}

static uint32_t device_param_frequency_compat_threshold_hz(uint32_t raw_threshold)
{
    if (raw_threshold == 0U) {
        return 0U;
    }
    return (raw_threshold + (DENSITY_PARAM_MIGRATE_FACTOR / 2U)) / DENSITY_PARAM_MIGRATE_FACTOR;
}

static int device_param_is_density_level_method(const DeviceParameters *params)
{
    return ((params != NULL) && (params->liquidLevelMeasurementMethod == 2U)) ? 1 : 0;
}

/*
 * 函数用途：返回和 CPU3 参数菜单一致的枚举含义。
 * 调用场景：设备参数全量打印和写参差异打印。
 * 关键约束：只解释数值，不修改参数，不参与参数范围归一化。
 */
static const char *device_param_value_desc(const ParamPrintItem *item, uint32_t value)
{
    if (item == NULL) {
        return NULL;
    }

    switch (item->offset) {
    case (uint16_t)offsetof(DeviceParameters, sensorType):
        if (value == (uint32_t)DSM_SENSOR) {
            return "一体机传感器";
        }
        if (value == (uint32_t)LTD_SENSOR) {
            return "LTD传感器";
        }
        if (value == (uint32_t)SAFE_SENSOR) {
            return "安全协议传感器";
        }
        return "非法配置";
    case (uint16_t)offsetof(DeviceParameters, command):
        switch (value) {
        case CMD_NONE:
            return "无命令";
        case CMD_BACK_ZERO:
            return "回零点";
        case CMD_FIND_OIL:
            return "寻找液位";
        case CMD_FIND_WATER:
            return "寻找水位";
        case CMD_FIND_BOTTOM:
            return "寻找罐底";
        case CMD_MEASURE_SINGLE:
            return "单点测量";
        case CMD_MONITOR_SINGLE:
            return "单点监测";
        case CMD_SYNTHETIC:
            return "综合测量";
        case CMD_FOLLOW_WATER:
            return "水位跟随";
        case CMD_RUN_TO_POSITION:
            return "运行到指定位置";
        case CMD_MEASURE_DISTRIBUTED:
            return "普通分布测量";
        case CMD_GB_MEASURE_DISTRIBUTED:
            return "国标分布测量";
        case CMD_MEASURE_DENSITY_METER:
            return "密度每米测量";
        case CMD_MEASURE_DENSITY_RANGE:
            return "区间密度测量";
        case CMD_WARTSILA_DENSITY_RANGE:
            return "瓦锡兰区间密度测量";
        case CMD_READ_PART_PARAMS:
            return "读取部件参数";
        case CMD_CANCEL_MEASUREMENT:
            return "取消当前测量";
        case CMD_DEBUG_MODE:
            return "调试模式";
        case CMD_CALIBRATE_ZERO:
            return "标定零点";
        case CMD_CALIBRATE_OIL:
            return "标定液位";
        case CMD_CORRECT_OIL:
            return "修正液位";
        case CMD_MOVE_UP:
            return "上行";
        case CMD_MOVE_DOWN:
            return "下行";
        case CMD_SET_EMPTY_WEIGHT:
            return "设置空载扭力";
        case CMD_SET_FULL_WEIGHT:
            return "设置满载扭力";
        case CMD_RESTORE_FACTORY:
            return "恢复出厂设置";
        case CMD_MAINTENANCE_MODE:
            return "开启维护模式";
        case CMD_CALIBRATE_TANKHEIGHT:
            return "标定罐高";
        case CMD_FORCE_MOVE_UP:
            return "强制上行";
        case CMD_FORCE_MOVE_DOWN:
            return "强制下行";
        case CMD_CALIBRATE_WATER:
            return "标定水位";
        case CMD_PAIR_NEAREST_WIRELESS_SLIPRING:
            return "匹配最近无线滑环";
        case CMD_MAINTENANCE_EXIT:
            return "退出维护模式";
        case CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS:
            return "清除全部继电器锁存报警";
        case CMD_UNKNOWN:
            return "未知命令";
        default:
            return "非法配置";
        }
    case (uint16_t)offsetof(DeviceParameters, powerOnDefaultCommand):
        switch (value) {
        case CMD_NONE_DEF:
            return "无";
        case CMD_BACK_ZERO_DEF:
            return "回零点";
        case CMD_FIND_OIL_DEF:
            return "寻找液位";
        case CMD_MONITOR_SINGLE_DEF:
            return "单点监测";
        case CMD_FOLLOW_WATER_DEF:
            return "水位跟随";
        default:
            return "非法配置";
        }
    case (uint16_t)offsetof(DeviceParameters, error_auto_back_zero):
    case (uint16_t)offsetof(DeviceParameters, error_stop_measurement):
    case (uint16_t)offsetof(DeviceParameters, refreshTankHeightFlag):
    case (uint16_t)offsetof(DeviceParameters, bottom_encoder_correction_enable):
    case (uint16_t)offsetof(DeviceParameters, requireBottomMeasurement):
    case (uint16_t)offsetof(DeviceParameters, requireWaterMeasurement):
    case (uint16_t)offsetof(DeviceParameters, requireSinglePointDensity):
        if (value == 0U) {
            return "否";
        }
        if (value == 1U) {
            return "是";
        }
        return "非法配置";
    case (uint16_t)offsetof(DeviceParameters, position_source_auto_switch):
        if (value == 0U) {
            return "禁用";
        }
        if (value == 1U) {
            return "启用";
        }
        return "非法配置";
    case (uint16_t)offsetof(DeviceParameters, position_count_mode):
        if (value == 0U) {
            return "编码器";
        }
        if (value == 1U) {
            return "电机";
        }
        return "非法配置";
    case (uint16_t)offsetof(DeviceParameters, liquidLevelMeasurementMethod):
        switch (value) {
        case 0U:
            return "相对频率";
        case 1U:
            return "定频";
        case 2U:
            return "密度找液位";
        case 3U:
            return "超声预留";
        case 4U:
            return "连续相对频率";
        case 5U:
            return "连续定频";
        default:
            return "非法配置";
        }
    case (uint16_t)offsetof(DeviceParameters, water_level_mode):
        return (value == 0U) ? "低速模式" : "快速模式";
    case (uint16_t)offsetof(DeviceParameters, bottom_detect_mode):
        if (value == 0U) {
            return "扭力";
        }
        if (value == 1U) {
            return "角度";
        }
        return "非法配置";
    case (uint16_t)offsetof(DeviceParameters, spreadMeasurementOrder):
        if (value == 0U) {
            return "向上";
        }
        if (value == 1U) {
            return "向下";
        }
        return "非法配置";
    case (uint16_t)offsetof(DeviceParameters, spreadMeasurementMode):
        switch (value) {
        case 0U:
            return "分布测量";
        case 1U:
            return "国标测量";
        case 2U:
            return "每米测量";
        case 3U:
            return "区间测量";
        default:
            return "非法配置";
        }
    case (uint16_t)offsetof(DeviceParameters, ao_output.work_mode):
        if (value == 0U) {
            return "禁用";
        }
        if (value == 1U) {
            return "4-20mA输出";
        }
        if (value == 2U) {
            return "HART从站+输出";
        }
        return "非法配置";
    default:
        return NULL;
    }
}

/*
 * 函数用途：把带枚举含义的参数格式化为“值(含义)”。
 * 调用场景：复用在全量打印和 diff 打印中，保证同一字段显示一致。
 * 关键约束：缓冲区不足时由 snprintf 截断，不影响主流程执行。
 */
static void format_device_param_u32_value(const ParamPrintItem *item, uint32_t value, char *buffer, size_t buffer_size)
{
    const char *desc;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }

    desc = device_param_value_desc(item, value);
    if (desc != NULL) {
        (void)snprintf(buffer, buffer_size, "%lu(%s)", (unsigned long)value, desc);
    } else {
        (void)snprintf(buffer, buffer_size, "%lu", (unsigned long)value);
    }
}

/*
 * 函数用途：把带单位的参数格式化为工程值。
 * 调用场景：普通 U32_UNIT 参数打印，补充电机电流和重跑次数的现场含义。
 * 关键约束：只做显示换算，不改变实际 IRUN 档位或重跑次数。
 */
static void format_device_param_u32_unit_value(const ParamPrintItem *item, uint32_t value, char *buffer, size_t buffer_size)
{
    static const uint16_t motor_current_rms_ma_table[] = {
        84U, 127U, 169U, 211U, 253U, 296U, 338U, 380U,
        422U, 465U, 507U, 549U, 591U, 634U, 676U, 718U,
        760U, 803U, 845U, 887U, 929U, 972U, 1014U, 1056U,
        1098U, 1141U, 1183U, 1225U, 1267U, 1310U, 1352U,
    };
    uint32_t rms_ma;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return;
    }
    if (item == NULL) {
        (void)snprintf(buffer, buffer_size, "%lu", (unsigned long)value);
        return;
    }

    if (item->offset == (uint16_t)offsetof(DeviceParameters, motor_current)) {
        if ((value >= MOTOR_CURRENT_MIN) && (value <= MOTOR_CURRENT_MAX)) {
            rms_ma = motor_current_rms_ma_table[value - MOTOR_CURRENT_MIN];
            (void)snprintf(buffer,
                           buffer_size,
                           "%lu IRUN(%.2f A)",
                           (unsigned long)value,
                           ((double)rms_ma) / 1000.0);
        } else {
            (void)snprintf(buffer, buffer_size, "%lu IRUN(非法配置)", (unsigned long)value);
        }
        return;
    }

    if (item->offset == (uint16_t)offsetof(DeviceParameters, fault_auto_recovery_retry_limit)) {
        if (value == 0U) {
            (void)snprintf(buffer, buffer_size, "0 次(关闭自动重跑)");
        } else {
            (void)snprintf(buffer, buffer_size, "%lu 次", (unsigned long)value);
        }
        return;
    }

    if (item->unit != NULL) {
        (void)snprintf(buffer, buffer_size, "%lu %s", (unsigned long)value, item->unit);
    } else {
        format_device_param_u32_value(item, value, buffer, buffer_size);
    }
}

static void build_device_param_print_label(const ParamPrintItem *item, const char *fallback_unit, char *label, size_t label_size)
{
    (void)fallback_unit;

    if ((item == NULL) || (label == NULL) || (label_size == 0U)) {
        return;
    }

    (void)snprintf(label, label_size, "%s", item->name);
}

static void print_device_param_item(const DeviceParameters *params, const ParamPrintItem *item)
{
    uint32_t raw;
    int32_t signed_raw;
    char label[80];
    char value_text[96];

    if ((params == NULL) || (item == NULL)) {
        return;
    }

    if (item->type == PARAM_PRINT_TYPE_RELAY_BLOCK) {
        print_relay_alarm_params(params);
        return;
    }

    build_device_param_print_label(item, NULL, label, sizeof(label));

    switch (item->type) {
    case PARAM_PRINT_TYPE_U32:
        raw = device_param_item_read_u32(params, item);
        format_device_param_u32_value(item, raw, value_text, sizeof(value_text));
        printf("  %-32s : %s\r\n", label, value_text);
        break;
    case PARAM_PRINT_TYPE_I32:
        signed_raw = device_param_item_read_i32(params, item);
        printf("  %-32s : %ld\r\n", label, (long)signed_raw);
        break;
    case PARAM_PRINT_TYPE_U32_UNIT:
        raw = device_param_item_read_u32(params, item);
        format_device_param_u32_unit_value(item, raw, value_text, sizeof(value_text));
        printf("  %-32s : %s\r\n", label, value_text);
        break;
    case PARAM_PRINT_TYPE_I32_UNIT:
        signed_raw = device_param_item_read_i32(params, item);
        printf("  %-32s : %ld %s\r\n", label, (long)signed_raw, item->unit);
        break;
    case PARAM_PRINT_TYPE_HEX32:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : 0x%08lX\r\n", label, (unsigned long)raw);
        break;
    case PARAM_PRINT_TYPE_VERSION_TEXT:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : V%lu.%lu.%lu.%lu\r\n",
               label,
               (unsigned long)((raw >> 24) & 0xFFU),
               (unsigned long)((raw >> 16) & 0xFFU),
               (unsigned long)((raw >> 8) & 0xFFU),
               (unsigned long)(raw & 0xFFU));
        break;
    case PARAM_PRINT_TYPE_U32_01MM:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.1f mm\r\n", label, ((double)raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_I32_01MM:
        signed_raw = device_param_item_read_i32(params, item);
        printf("  %-32s : %.1f mm\r\n", label, ((double)signed_raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_001MM:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.3f mm\r\n", label, ((double)raw) / 1000.0);
        break;
    case PARAM_PRINT_TYPE_U32_01M_PER_MIN:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.2f m/min\r\n", label, ((double)raw) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_01MA:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.2f mA\r\n", label, ((double)raw) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_001PF:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.3f pF\r\n", label, ((double)raw) / 1000.0);
        break;
    case PARAM_PRINT_TYPE_U32_01PF:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.1f pF\r\n", label, ((double)raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_DENSITY:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.2f kg/m3\r\n", label, ((double)raw) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_OIL_LEVEL_THRESHOLD:
        raw = device_param_item_read_u32(params, item);
        if (device_param_is_density_level_method(params) != 0) {
            printf("  %-32s : %.2f kg/m3\r\n", label, ((double)raw) / 100.0);
        } else {
            printf("  %-32s : %lu Hz\r\n", label, (unsigned long)device_param_frequency_compat_threshold_hz(raw));
        }
        break;
    case PARAM_PRINT_TYPE_U32_DENSITY_OFFSET:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.2f kg/m3\r\n",
               label,
               (((double)raw) - (double)DENSITY_CORRECTION_BASE_RAW) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_TEMP_OFFSET:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.1f C\r\n", label, (((double)raw) - 1000.0) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_01C:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.1f C\r\n", label, ((double)raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_000001_RATIO:
        raw = device_param_item_read_u32(params, item);
        printf("  %-32s : %.6f\r\n", label, ((double)raw) / 1000000.0);
        break;
    default:
        break;
    }
}
/*
 * 函数用途：判断全量参数表是否打印该字段。
 * 调用场景：上电、恢复出厂和人工全量打印。
 * 关键约束：全量打印不再屏蔽运行态命令，继电器等特殊块也必须保留。
 */
static int device_param_item_can_full_print(const ParamPrintItem *item)
{
    return (item != NULL) ? 1 : 0;
}
static void print_device_params_items(const DeviceParameters *params)
{
    const char *current_group = NULL;
    uint32_t item_count = (uint32_t)(sizeof(g_device_param_print_table) / sizeof(g_device_param_print_table[0]));

    for (uint32_t index = 0U; index < item_count; index++) {
        const ParamPrintItem *item = &g_device_param_print_table[index];
        if (device_param_item_can_full_print(item) == 0) {
            continue;
        }
        if ((current_group == NULL) || (strcmp(current_group, item->group) != 0)) {
            current_group = item->group;
            printf("\r\n-- %s --\r\n", current_group);
        }
        print_device_param_item(params, item);
    }
}

static uint32_t count_relay_alarm_diff(const DeviceParameters *old_params, const DeviceParameters *new_params)
{
    uint32_t count = 0U;
    uint32_t item_count = (uint32_t)(sizeof(g_relay_param_print_table) / sizeof(g_relay_param_print_table[0]));

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        const RelayAlarmConfig *old_cfg = &old_params->relayAlarm[channel];
        const RelayAlarmConfig *new_cfg = &new_params->relayAlarm[channel];
        for (uint32_t index = 0U; index < item_count; index++) {
            const RelayParamPrintItem *item = &g_relay_param_print_table[index];
            if (relay_param_item_read_u32(old_cfg, item) != relay_param_item_read_u32(new_cfg, item)) {
                count++;
            }
        }
    }
    return count;
}

static uint32_t count_device_params_diff(const DeviceParameters *old_params, const DeviceParameters *new_params)
{
    uint32_t count = 0U;
    uint32_t item_count = (uint32_t)(sizeof(g_device_param_print_table) / sizeof(g_device_param_print_table[0]));

    for (uint32_t index = 0U; index < item_count; index++) {
        const ParamPrintItem *item = &g_device_param_print_table[index];
        if (item->type == PARAM_PRINT_TYPE_RELAY_BLOCK) {
            count += count_relay_alarm_diff(old_params, new_params);
        } else if (device_param_item_can_diff(item) != 0) {
            if ((item->type == PARAM_PRINT_TYPE_I32) ||
                (item->type == PARAM_PRINT_TYPE_I32_UNIT) ||
                (item->type == PARAM_PRINT_TYPE_I32_01MM)) {
                if (device_param_item_read_i32(old_params, item) != device_param_item_read_i32(new_params, item)) {
                    count++;
                }
            } else if (device_param_item_read_u32(old_params, item) != device_param_item_read_u32(new_params, item)) {
                count++;
            }
        }
    }
    return count;
}

static void print_device_param_diff_item(const DeviceParameters *old_params, const DeviceParameters *new_params, const ParamPrintItem *item)
{
    uint32_t old_raw;
    uint32_t new_raw;
    int32_t old_signed;
    int32_t new_signed;
    char label[80];
    char old_text[96];
    char new_text[96];

    build_device_param_print_label(item, NULL, label, sizeof(label));

    if ((item->type == PARAM_PRINT_TYPE_I32) ||
        (item->type == PARAM_PRINT_TYPE_I32_UNIT) ||
        (item->type == PARAM_PRINT_TYPE_I32_01MM)) {
        old_signed = device_param_item_read_i32(old_params, item);
        new_signed = device_param_item_read_i32(new_params, item);
        if (item->type == PARAM_PRINT_TYPE_I32_01MM) {
            printf("  %-32s : %.1f mm -> %.1f mm\r\n",
                   label,
                   ((double)old_signed) / 10.0,
                   ((double)new_signed) / 10.0);
        } else if (item->type == PARAM_PRINT_TYPE_I32_UNIT) {
            printf("  %-32s : %ld %s -> %ld %s\r\n",
                   label,
                   (long)old_signed,
                   item->unit,
                   (long)new_signed,
                   item->unit);
        } else {
            printf("  %-32s : %ld -> %ld\r\n", label, (long)old_signed, (long)new_signed);
        }
        return;
    }

    old_raw = device_param_item_read_u32(old_params, item);
    new_raw = device_param_item_read_u32(new_params, item);

    switch (item->type) {
    case PARAM_PRINT_TYPE_U32:
        format_device_param_u32_value(item, old_raw, old_text, sizeof(old_text));
        format_device_param_u32_value(item, new_raw, new_text, sizeof(new_text));
        printf("  %-32s : %s -> %s\r\n", label, old_text, new_text);
        break;
    case PARAM_PRINT_TYPE_U32_UNIT:
        format_device_param_u32_unit_value(item, old_raw, old_text, sizeof(old_text));
        format_device_param_u32_unit_value(item, new_raw, new_text, sizeof(new_text));
        printf("  %-32s : %s -> %s\r\n", label, old_text, new_text);
        break;
    case PARAM_PRINT_TYPE_HEX32:
        printf("  %-32s : 0x%08lX -> 0x%08lX\r\n", label, (unsigned long)old_raw, (unsigned long)new_raw);
        break;
    case PARAM_PRINT_TYPE_U32_01MM:
        printf("  %-32s : %.1f mm -> %.1f mm\r\n",
               label,
               ((double)old_raw) / 10.0,
               ((double)new_raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_001MM:
        printf("  %-32s : %.3f mm -> %.3f mm\r\n",
               label,
               ((double)old_raw) / 1000.0,
               ((double)new_raw) / 1000.0);
        break;
    case PARAM_PRINT_TYPE_U32_01M_PER_MIN:
        printf("  %-32s : %.2f m/min -> %.2f m/min\r\n",
               label,
               ((double)old_raw) / 100.0,
               ((double)new_raw) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_01MA:
        printf("  %-32s : %.2f mA -> %.2f mA\r\n",
               label,
               ((double)old_raw) / 100.0,
               ((double)new_raw) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_001PF:
        printf("  %-32s : %.3f pF -> %.3f pF\r\n",
               label,
               ((double)old_raw) / 1000.0,
               ((double)new_raw) / 1000.0);
        break;
    case PARAM_PRINT_TYPE_U32_01PF:
        printf("  %-32s : %.1f pF -> %.1f pF\r\n",
               label,
               ((double)old_raw) / 10.0,
               ((double)new_raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_DENSITY:
        printf("  %-32s : %.2f kg/m3 -> %.2f kg/m3\r\n",
               label,
               ((double)old_raw) / 100.0,
               ((double)new_raw) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_OIL_LEVEL_THRESHOLD:
        if (device_param_is_density_level_method(new_params) != 0) {
            printf("  %-32s : %.2f kg/m3 -> %.2f kg/m3\r\n",
                   label,
                   ((double)old_raw) / 100.0,
                   ((double)new_raw) / 100.0);
        } else {
            printf("  %-32s : %lu Hz -> %lu Hz\r\n",
                   label,
                   (unsigned long)device_param_frequency_compat_threshold_hz(old_raw),
                   (unsigned long)device_param_frequency_compat_threshold_hz(new_raw));
        }
        break;
    case PARAM_PRINT_TYPE_U32_DENSITY_OFFSET:
        printf("  %-32s : %.2f kg/m3 -> %.2f kg/m3\r\n",
               label,
               (((double)old_raw) - (double)DENSITY_CORRECTION_BASE_RAW) / 100.0,
               (((double)new_raw) - (double)DENSITY_CORRECTION_BASE_RAW) / 100.0);
        break;
    case PARAM_PRINT_TYPE_U32_TEMP_OFFSET:
        printf("  %-32s : %.1f C -> %.1f C\r\n",
               label,
               (((double)old_raw) - 1000.0) / 10.0,
               (((double)new_raw) - 1000.0) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_01C:
        printf("  %-32s : %.1f C -> %.1f C\r\n",
               label,
               ((double)old_raw) / 10.0,
               ((double)new_raw) / 10.0);
        break;
    case PARAM_PRINT_TYPE_U32_000001_RATIO:
        printf("  %-32s : %.6f -> %.6f\r\n",
               label,
               ((double)old_raw) / 1000000.0,
               ((double)new_raw) / 1000000.0);
        break;
    default:
        format_device_param_u32_value(item, old_raw, old_text, sizeof(old_text));
        format_device_param_u32_value(item, new_raw, new_text, sizeof(new_text));
        printf("  %-32s : %s -> %s\r\n", label, old_text, new_text);
        break;
    }
}
static void print_relay_alarm_diff(const DeviceParameters *old_params, const DeviceParameters *new_params)
{
    uint32_t item_count = (uint32_t)(sizeof(g_relay_param_print_table) / sizeof(g_relay_param_print_table[0]));

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        const RelayAlarmConfig *old_cfg = &old_params->relayAlarm[channel];
        const RelayAlarmConfig *new_cfg = &new_params->relayAlarm[channel];
        for (uint32_t index = 0U; index < item_count; index++) {
            const RelayParamPrintItem *item = &g_relay_param_print_table[index];
            uint32_t old_raw = relay_param_item_read_u32(old_cfg, item);
            uint32_t new_raw = relay_param_item_read_u32(new_cfg, item);
            if (old_raw == new_raw) {
                continue;
            }
            if (item->type == RELAY_PARAM_PRINT_FLOAT_RAW) {
                printf("  继电器%lu.%-18s : %.1f -> %.1f\r\n",
                       (unsigned long)(channel + 1U),
                       item->name,
                       relay_alarm_raw_to_float(old_raw),
                       relay_alarm_raw_to_float(new_raw));
            } else {
                const char *old_desc = relay_param_value_desc(item, old_raw);
                const char *new_desc = relay_param_value_desc(item, new_raw);
                if ((old_desc != NULL) || (new_desc != NULL)) {
                    printf("  继电器%lu.%-18s : %lu(%s) -> %lu(%s)\r\n",
                           (unsigned long)(channel + 1U),
                           item->name,
                           (unsigned long)old_raw,
                           (old_desc != NULL) ? old_desc : "未定义",
                           (unsigned long)new_raw,
                           (new_desc != NULL) ? new_desc : "未定义");
                } else {
                    printf("  继电器%lu.%-18s : %lu -> %lu\r\n",
                           (unsigned long)(channel + 1U),
                           item->name,
                           (unsigned long)old_raw,
                           (unsigned long)new_raw);
                }
            }
        }
    }
}

/*
 * 函数用途：把参数打印场景转换成中文标签。
 * 调用场景：上电、恢复出厂和人工全量打印的标题/结束行。
 * 关键约束：只影响串口打印文本，不改变参数内容和保存流程。
 */
static const char *device_param_print_stage_name(const char *stage)
{
    if (stage == NULL) {
        return "人工";
    }
    if (strcmp(stage, "BOOT") == 0) {
        return "上电";
    }
    if (strcmp(stage, "FACTORY_RESET") == 0) {
        return "恢复出厂";
    }
    if (strcmp(stage, "MANUAL") == 0) {
        return "人工";
    }
    return stage;
}
static void print_device_params_full(const DeviceParameters *snapshot,
                                     const char *stage,
                                     const char *reason,
                                     const char *source)
{
    DeviceParameters params;
    if (snapshot != NULL) {
        memcpy(&params, snapshot, sizeof(DeviceParameters));
    } else {
        memcpy(&params, (void *)&g_deviceParams, sizeof(DeviceParameters));
    }

    printf("\r\n========================================\r\n");
    printf("[参数][%s][全量] 设备参数全量打印\r\n", device_param_print_stage_name(stage));
    if (reason != NULL) {
        printf("原因       : %s\r\n", reason);
    }
    if (source != NULL) {
        printf("参数来源   : %s\r\n", source);
    }
    printf("              设备参数\r\n");
    printf("========================================\r\n");
    print_device_params_items(&params);
    printf("========================================\r\n");
    printf("[参数][%s][全量] 设备参数打印结束\r\n", device_param_print_stage_name(stage));
    printf("========================================\r\n");
}

static void print_device_params_save_meta(const DeviceParameters *params, const char *source)
{
    DeviceParameters snapshot;
    const DeviceParameters *print_params = params;

    if (print_params == NULL) {
        memcpy(&snapshot, (void *)&g_deviceParams, sizeof(DeviceParameters));
        print_params = &snapshot;
    }

    printf("[参数][保存][成功] 保存结果=成功 | 位置=%s | 版本=%lu | 大小=%lu | CRC32=0x%08lX\r\n",
           (source != NULL) ? source : "未知",
           (unsigned long)print_params->param_version,
           (unsigned long)print_params->struct_size,
           (unsigned long)print_params->crc);
}

static void print_device_params_event(DeviceParamPrintEvent event, const DeviceParameters *params, const char *reason, const char *source)
{
    switch (event) {
    case PARAM_PRINT_BOOT_FULL:
#if DEVICE_PARAMS_BOOT_FULL_PRINT_ENABLE
        print_device_params_full(params, "BOOT", reason, source);
#endif
        break;
    case PARAM_PRINT_FACTORY_RESET_FULL:
        print_device_params_full(params, "FACTORY_RESET", reason, source);
        break;
    case PARAM_PRINT_MANUAL_FULL:
        print_device_params_full(params, "MANUAL", reason, source);
        break;
    case PARAM_PRINT_SAVE_META:
        print_device_params_save_meta(params, source);
        break;
    case PARAM_PRINT_SAVE_SKIP:
        printf("[参数][保存][跳过] 设备参数未变化，跳过保存\r\n");
        break;
    default:
        break;
    }
}

/*
 * 函数用途：按统一场景入口打印设备参数。
 * 调用场景：上电、恢复出厂、保存摘要和人工调试入口。
 * 关键约束：该函数会直接 printf，不应在中断上下文调用。
 */
void DeviceParams_PrintEvent(DeviceParamPrintEvent event)
{
    print_device_params_event(event, NULL, NULL, NULL);
}

/*
 * 函数用途：打印两份设备参数之间的差异。
 * 调用场景：保存入口拿到旧参数快照和新参数镜像后统一输出。
 * 关键约束：该函数只打印、不修改参数、不写 FRAM。
 */
void DeviceParams_PrintDiff(const DeviceParameters *old_params, const DeviceParameters *new_params)
{
    uint32_t total_count;
    uint32_t item_count;

    if ((old_params == NULL) || (new_params == NULL)) {
        return;
    }

    total_count = count_device_params_diff(old_params, new_params);
    if (total_count == 0U) {
        return;
    }

    printf("[参数][写入][成功] 参数写入完成，变更 %lu 项\r\n", (unsigned long)total_count);
    item_count = (uint32_t)(sizeof(g_device_param_print_table) / sizeof(g_device_param_print_table[0]));
    for (uint32_t index = 0U; index < item_count; index++) {
        const ParamPrintItem *item = &g_device_param_print_table[index];
        if (item->type == PARAM_PRINT_TYPE_RELAY_BLOCK) {
            print_relay_alarm_diff(old_params, new_params);
        } else if (device_param_item_can_diff(item) != 0) {
            if ((item->type == PARAM_PRINT_TYPE_I32) ||
                (item->type == PARAM_PRINT_TYPE_I32_UNIT)) {
                if (device_param_item_read_i32(old_params, item) != device_param_item_read_i32(new_params, item)) {
                    print_device_param_diff_item(old_params, new_params, item);
                }
            } else if (device_param_item_read_u32(old_params, item) != device_param_item_read_u32(new_params, item)) {
                print_device_param_diff_item(old_params, new_params, item);
            }
        }
    }
}

/*
 * 函数用途：记录 Modbus 写参前的设备参数快照。
 * 调用场景：0x10 写入持久化参数区之前调用，供主循环延后保存时打印差异。
 * 关键约束：该函数只复制内存，不打印、不写 FRAM。
 */
void DeviceParams_CaptureWriteSnapshot(const DeviceParameters *params)
{
    if ((g_device_params_write_snapshot_valid == 0U) && (params != NULL)) {
        memcpy(&g_device_params_write_snapshot, params, sizeof(DeviceParameters));
        g_device_params_write_snapshot_valid = 1U;
    }
}

void print_device_params(void)
{
    DeviceParams_PrintEvent(PARAM_PRINT_MANUAL_FULL);
}
/* ========================= 测量结果打印（可选） ========================= */
/* 注: 该部分与参数结构无强耦合，仅保留你现有打印习惯；如果不需要可移除 */

/**
 * @brief 打印单个密度测点信息
 * @param title 输出标题
 * @param d     密度测点数据指针
 */
void PrintDensity(const char *title, const DensityMeasurement *d)
{
    if (!d) return;

    printf("  [%s]\r\n", title);
    printf("    温度: %lu\r\n", (unsigned long)d->temperature);
    printf("    密度: %lu\r\n", (unsigned long)d->density);
    printf("    标准密度: %lu\r\n", (unsigned long)d->standard_density);
    printf("    VCF20: %lu\r\n", (unsigned long)d->vcf20);
    printf("    计重密度: %lu\r\n", (unsigned long)d->weight_density);
    printf("    温度位置: %lu\r\n", (unsigned long)d->temperature_position);
}

/**
 * @brief 打印完整测量结果
 * @param m 测量结果指针
 */
void PrintMeasurementResult(const MeasurementResult *m)
{
    if (!m) return;

    printf("\r\n=====================【设备实时测量结果】=====================\r\n");

    /* 1. 设备状态 */
    printf("【设备状态】\r\n");
    printf("  工作模式: %lu\r\n", (unsigned long)m->device_status.work_mode);
    printf("  设备状态: %d\r\n",  (int)m->device_status.device_state);
    printf("  错误代码: %lu\r\n", (unsigned long)m->device_status.error_code);
    printf("  当前指令: %d\r\n",  (int)m->device_status.current_command);
    printf("  零点状态: %lu\r\n", (unsigned long)m->device_status.zero_point_status);

    printf("--------------------------------------------------------------\r\n");

    /* 2. 调试数据 */
    printf("【调试数据】\r\n");
    printf("  编码值: %ld\r\n",        (long)m->debug_data.current_encoder_value);
    printf("  传感器位置: %ld mm\r\n", (long)m->debug_data.sensor_position);
    printf("  尺带长度: %ld mm\r\n",   (long)m->debug_data.cable_length);
    printf("  电机步进: %ld\r\n",      (long)m->debug_data.motor_step);
    printf("  电机距离: %ld (0.1mm)\r\n",(long)m->debug_data.motor_distance);

    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->debug_data.frequency);
    printf("  温度: %.2f ℃\r\n",       m->debug_data.temperature / 100.0f);
    printf("  空气中频率: %lu Hz\r\n", (unsigned long)m->debug_data.air_frequency);
    printf("  幅值: %lu\r\n",          (unsigned long)m->debug_data.current_amplitude);
    printf("  水位电容快照(0.1pF): %lu\r\n",(unsigned long)m->debug_data.water_capacitance_x10);

    printf("  当前扭力值: %lu\r\n",    (unsigned long)m->debug_data.current_weight);
    printf("  扭力参数: %lu\r\n",      (unsigned long)m->debug_data.weight_param);

    printf("  X角度: %ld\r\n", (long)m->debug_data.angle_x);
    printf("  Y角度: %ld\r\n", (long)m->debug_data.angle_y);

    printf("  电机速度(0.01m/min): %lu\r\n", (unsigned long)m->debug_data.motor_speed);
    printf("  电机状态: %lu (%s)\r\n",
           (unsigned long)m->debug_data.motor_state,
           (m->debug_data.motor_state == 0) ? "停止" :
           (m->debug_data.motor_state == 1) ? "上行" :
           (m->debug_data.motor_state == 2) ? "下行" : "未知");

    printf("--------------------------------------------------------------\r\n");

    /* 3. 液位测量 */
    printf("【液位测量】\r\n");
    printf("  跟随液位: %lu mm\r\n",   (unsigned long)m->oil_measurement.oil_level);
    printf("  空气频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.air_frequency);
    printf("  油中频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.oil_frequency);
    printf("  跟随频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.follow_frequency);
    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.current_frequency);

    printf("--------------------------------------------------------------\r\n");

    /* 4. 水位测量 */
    printf("【水位测量】\r\n");
    printf("  水位值: %lu mm\r\n", (unsigned long)m->water_measurement.water_level);
    printf("  零点电容: %.3f\r\n", m->water_measurement.zero_capacitance);
    printf("  油区电容: %.3f\r\n", m->water_measurement.oil_capacitance);
    printf("  当前电容: %.3f\r\n", m->water_measurement.current_capacitance);

    printf("--------------------------------------------------------------\r\n");

    /* 5. 实高测量 */
    printf("【实高测量】\r\n");
    printf("  标定液位实高: %lu mm\r\n", (unsigned long)m->height_measurement.calibrated_liquid_level);
    printf("  当前实高: %lu mm\r\n",     (unsigned long)m->height_measurement.current_real_height);

    printf("--------------------------------------------------------------\r\n");

    /* 6. 单点密度测量 */
    printf("【单点密度测量】\r\n");
    PrintDensity("单点测量", &m->single_point_measurement);

    /* 7. 单点监测 */
    printf("【单点监测】\r\n");
    PrintDensity("单点监测", &m->single_point_monitoring);

    printf("--------------------------------------------------------------\r\n");

    /* 8. 密度分布（概要） */
    printf("【密度分布】\r\n");
    printf("  平均温度: %lu\r\n",     (unsigned long)m->density_distribution.average_temperature);
    printf("  平均密度: %lu\r\n",     (unsigned long)m->density_distribution.average_density);
    printf("  平均计重密度: %lu\r\n", (unsigned long)m->density_distribution.average_weight_density);
    printf("  测量点数: %lu\r\n",         (unsigned long)m->density_distribution.measurement_points);
    printf("  测量时液位(0.1mm): %lu\r\n",(unsigned long)m->density_distribution.Density_oil_level);
    printf("  测量时液位(实际): %.1f mm\r\n",
           (double)m->density_distribution.Density_oil_level / 10.0);

    printf("  --- 单点数据（仅打印前10个）---\r\n");
    for (uint32_t i = 0; i < 10 && i < m->density_distribution.measurement_points; i++)
    {
        const DensityMeasurement *d = &m->density_distribution.single_density_data[i];
        printf("    [%02lu] 温度=%lu 密度=%lu 标密=%lu VCF=%lu 重量密度=%lu 位置=%lu\r\n",
               (unsigned long)i,
               (unsigned long)d->temperature,
               (unsigned long)d->density,
               (unsigned long)d->standard_density,
               (unsigned long)d->vcf20,
               (unsigned long)d->weight_density,
               (unsigned long)d->temperature_position);
    }

    printf("--------------------------------------------------------------\r\n");
    printf("【继电器报警输出运行态】\r\n");
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        const RelayAlarmRuntimeState *state = &m->relay_alarm_runtime[channel];
        printf("  继电器%lu: value=%.1f HH=%lu H=%lu HH_H=%lu L=%lu LL=%lu LL_L=%lu any=%lu clear=%lu\r\n",
               (unsigned long)(channel + 1U),
               (double)state->alarm_value,
               (unsigned long)state->HH_alarm,
               (unsigned long)state->H_alarm,
               (unsigned long)state->HH_H_alarm,
               (unsigned long)state->L_alarm,
               (unsigned long)state->LL_alarm,
               (unsigned long)state->LL_L_alarm,
               (unsigned long)state->any_error,
               (unsigned long)state->clear_alarm);
    }

    printf("========================【打印结束】========================\r\n");
}
/**
 * @brief 默认指令 -> 测量命令 映射
 * @param def_cmd  DefaultCommandType
 * @return         CommandType（测量命令）；无匹配返回 CMD_UNKNOWN
 */
CommandType DefaultCmd_To_MeasureCmd(DefaultCommandType def_cmd)
{
    switch (def_cmd) {

    case CMD_NONE_DEF:
        return CMD_NONE;

    case CMD_BACK_ZERO_DEF:
        return CMD_BACK_ZERO;

    case CMD_FIND_OIL_DEF:
        return CMD_FIND_OIL;

    case CMD_MONITOR_SINGLE_DEF:
        return CMD_MONITOR_SINGLE;

    case CMD_FOLLOW_WATER_DEF:
        return CMD_FOLLOW_WATER;

    default:
        return CMD_NONE;
    }
}
