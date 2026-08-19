/*
 * 模块职责：把上层统一传感器操作映射到DSM、V3、DM4-V4及预留Safe实现。
 * 流程边界：本文件只做能力分派和协议适配，不负责上电识别、错误锁存或测量状态机。
 * DM4约束：类型14代表唯一物理身份，是否进入Safe驱动由运行态会话决定。
 */
#include "sensor_driver_registry.h"

#include "app_main.h"
#include "Protocols/Dsm/dsm_sensor_communication.h"
#include "Protocols/MultiparamV3/multiparam_v3_communication.h"
#include "Protocols/Dm4/V4/multiparam_v4_communication.h"
#include "Protocols/Dm4/V4/multiparam_v4_measurement.h"
#include "Protocols/Dm4/Safe/sensor_safe_legacy_adapter.h"
#include "system_parameter.h"

/*
 * 函数用途：把统一的密度模式操作适配到DSM协议实现。
 * 调用场景：SensorService通过DSM驱动操作表切换模式。
 * 关键约束：只转换返回类型，不增加重试、等待或错误翻译。
 */
static uint32_t SensorDriverDsm_EnableDensityMode(void)
{
    return (uint32_t)DSM_EnableDensityMode();
}

/*
 * 函数用途：把统一的液位模式操作适配到DSM协议实现。
 * 调用场景：SensorService通过DSM驱动操作表切换模式。
 * 关键约束：电机停稳和模式稳定等待由服务层负责，本函数不重复处理。
 */
static uint32_t SensorDriverDsm_EnableLevelMode(void)
{
    return (uint32_t)DSM_EnableLevelMode();
}

/*
 * 函数用途：把统一的密度组合量读取适配到DSM协议实现。
 * 调用场景：密度测量或部件诊断选择DSM驱动时。
 * 关键约束：输出顺序保持频率、密度、温度，不在驱动层应用整机修正。
 */
static uint32_t SensorDriverDsm_ReadDensity(float *frequency_hz,
                                            float *density_kg_m3,
                                            float *temperature_c)
{
    return (uint32_t)DSM_Read_Frequency_Density_Temp(frequency_hz,
                                                     density_kg_m3,
                                                     temperature_c);
}

/*
 * 函数用途：把统一的密度模式操作适配到多参数V3协议。
 * 调用场景：V3密度测量和部件诊断准备阶段。
 * 关键约束：只调用协议切换，不承担三秒稳定等待。
 */
static uint32_t SensorDriverV3_EnableDensityMode(void)
{
    return (uint32_t)MULTIPARAM_V3_SwitchToDensityMode();
}

/*
 * 函数用途：把统一的液位模式操作适配到多参数V3协议。
 * 调用场景：V3液位搜索、跟随和恢复阶段。
 * 关键约束：电机停稳和稳定等待由SensorService统一处理。
 */
static uint32_t SensorDriverV3_EnableLevelMode(void)
{
    return (uint32_t)MULTIPARAM_V3_SwitchToLevelMode();
}

/*
 * 函数用途：把统一的液位频率读取适配到多参数V3协议。
 * 调用场景：可靠液位频率读取选择V3驱动时。
 * 关键约束：不在适配层过滤0或6500Hz以上值，业务校验由服务层完成。
 */
static uint32_t SensorDriverV3_ReadLevelFrequency(uint32_t *frequency_hz)
{
    return (uint32_t)MULTIPARAM_V3_Read_LevelFrequency(frequency_hz);
}
/*
 * 函数用途：按旧V3顺序组合读取温度、密度和密度频率。
 * 调用场景：统一密度读取选择多参数V3驱动时。
 * 关键约束：温度、密度、频率依次执行，首个错误立即返回且不输出部分成功结果。
 */
static uint32_t SensorDriverV3_ReadDensity(float *frequency_hz,
                                           float *density_kg_m3,
                                           float *temperature_c)
{
    float frequency_45 = 0.0f;
    float frequency_22_5 = 0.0f;
    uint32_t result;

    /* 保留原V3部件参数读取顺序：温度 -> 密度 -> 三组频率，首个错误立即返回。 */
    result = (uint32_t)MULTIPARAM_V3_Read_Temperature(temperature_c);
    if (result == NO_ERROR) {
        result = (uint32_t)MULTIPARAM_V3_Read_Density(density_kg_m3);
    }
    if (result == NO_ERROR) {
        result = (uint32_t)MULTIPARAM_V3_Read_DensityFrequency(frequency_hz,
                                                               &frequency_45,
                                                               &frequency_22_5);
    }
    return result;
}

/*
 * 函数用途：在DM4主动上发期间取得一次交互窗口并执行模式命令。
 * 调用场景：驱动操作表切换密度或液位模式。
 * 关键约束：无论命令成功与否都尝试恢复原主动通信方式。
 */
static uint32_t SensorDriverV4_RunModeCommand(uint32_t (*command)(void))
{
    multiparam_v4_communication_mode_t original_mode;
    uint32_t command_result;
    uint32_t restore_result;

    if (command == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    original_mode = MULTIPARAM_V4_GetCommunicationMode();
    if (original_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return command();
    }
    if (original_mode != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    command_result = MULTIPARAM_V4_EnterInteractive();
    if (command_result != NO_ERROR) {
        return command_result;
    }
    command_result = command();
    /* 即使模式命令失败也恢复主动上发，避免诊断失败后UART6长期停在交互态。 */
    restore_result = MULTIPARAM_V4_EnterActive();
    return (command_result != NO_ERROR) ? command_result : restore_result;
}

/*
 * 函数用途：通过V4交互窗口切换DM4到密度模式。
 * 调用场景：统一服务层请求DM4密度模式时。
 * 关键约束：由RunModeCommand负责主动/交互切换及失败后的主动流恢复。
 */
static uint32_t SensorDriverV4_EnableDensityMode(void)
{
    return SensorDriverV4_RunModeCommand(MULTIPARAM_V4_SelectDensityMode);
}

/*
 * 函数用途：通过V4交互窗口切换DM4到液位模式。
 * 调用场景：统一服务层请求DM4液位模式时。
 * 关键约束：进入液位模式时传感器会关闭水位和零点霍尔功能，服务层随后等待稳定。
 */
static uint32_t SensorDriverV4_EnableLevelMode(void)
{
    return SensorDriverV4_RunModeCommand(MULTIPARAM_V4_SelectLevelMode);
}

/*
 * 函数用途：根据最新主动快照判断目标功能是否已经处于期望状态。
 * 调用场景：发送L/M功能命令前避免重复翻转非幂等开关。
 * 关键约束：快照必须新鲜且通信处于主动模式；开启状态还必须同时为密度模式。
 */
static uint8_t SensorDriverV4_ActiveFeatureMatches(
    multiparam_v4_feature_state_t target_feature,
    uint8_t enabled)
{
    multiparam_v4_snapshot_t snapshot;

    if (MULTIPARAM_V4_GetCommunicationMode() != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return 0U;
    }
    if ((MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) != NO_ERROR) ||
        ((HAL_GetTick() - snapshot.received_tick) > MULTIPARAM_V4_ACTIVE_TIMEOUT_MS)) {
        return 0U;
    }
    if (enabled != 0U) {
        /* 水位电容和零点霍尔只能在密度模式开启，主动帧状态字同时确认模式与功能。 */
        return (uint8_t)(((snapshot.measurement_mode == MULTIPARAM_V4_MEASUREMENT_DENSITY) &&
                          (snapshot.feature_state == target_feature)) ? 1U : 0U);
    }
    return (uint8_t)((snapshot.feature_state != target_feature) ? 1U : 0U);
}

/*
 * 函数用途：在V4主动上发期间取得交互窗口并执行一次功能状态调整。
 * 调用场景：水位电容或零点霍尔需要使能、关闭时。
 * 关键约束：功能命令结果优先返回；无论成功失败都尝试恢复原主动通信方式。
 */
static uint32_t SensorDriverV4_RunFeatureCommand(uint32_t (*command)(uint8_t),
                                                 uint8_t enabled)
{
    multiparam_v4_communication_mode_t original_mode;
    uint32_t command_result;
    uint32_t restore_result;

    if (command == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    original_mode = MULTIPARAM_V4_GetCommunicationMode();
    if (original_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return command(enabled);
    }
    if (original_mode != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    command_result = MULTIPARAM_V4_EnterInteractive();
    if (command_result != NO_ERROR) {
        return command_result;
    }
    command_result = command(enabled);
    /* 功能开关在传感器端保持；这里只恢复通信方式，不在读取完成后关闭功能。 */
    restore_result = MULTIPARAM_V4_EnterActive();
    return (command_result != NO_ERROR) ? command_result : restore_result;
}

/*
 * 函数用途：把DM4测水功能调整为明确状态并避免无效重复命令。
 * 调用场景：读取水位电容前或维护调试显式设置功能时。
 * 关键约束：目标状态先由主动快照确认；互斥功能和密度模式由协议层保证。
 */
static uint32_t SensorDriverV4_SetWaterEnabled(uint8_t enabled)
{
    uint8_t target = (enabled != 0U) ? 1U : 0U;

    if (SensorDriverV4_ActiveFeatureMatches(MULTIPARAM_V4_FEATURE_WATER, target) != 0U) {
        return NO_ERROR;
    }
    return SensorDriverV4_RunFeatureCommand(MULTIPARAM_V4_EnsureWaterEnabled, target);
}

/*
 * 函数用途：把DM4零点霍尔功能调整为明确状态并避免无效重复命令。
 * 调用场景：读取磁零点电压前或维护调试显式设置功能时。
 * 关键约束：目标状态先由主动快照确认；互斥功能和密度模式由协议层保证。
 */
static uint32_t SensorDriverV4_SetMagneticZeroEnabled(uint8_t enabled)
{
    uint8_t target = (enabled != 0U) ? 1U : 0U;

    if (SensorDriverV4_ActiveFeatureMatches(MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO, target) != 0U) {
        return NO_ERROR;
    }
    return SensorDriverV4_RunFeatureCommand(MULTIPARAM_V4_EnsureMagneticZeroEnabled, target);
}

/*
 * 函数用途：在功能刚使能后等待读取器返回首个非陈旧值。
 * 调用场景：DM4水位电容和零点霍尔读取。
 * 关键约束：只重试SENSOR_DATA_STALE；命令切换立即退出，等待总时长受主动帧超时限制。
 */
static uint32_t SensorDriverV4_WaitForValue(uint32_t (*reader)(float *), float *value)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t result;

    do {
        /* 使能后的旧快照可能还没有目标字段，等待第一帧新鲜有效值后再返回。 */
        result = reader(value);
        if (result != SENSOR_DATA_STALE) {
            break;
        }
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        if ((HAL_GetTick() - start_tick) >= MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) {
            break;
        }
        HAL_Delay(MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS);
    } while (1);
    return result;
}

/*
 * 函数用途：确保DM4测水开启并读取首个新鲜水位电容值。
 * 调用场景：SensorService读取DM4水位电容。
 * 关键约束：读取结束后保持测水开启；使能或等待失败时传播原错误。
 */
static uint32_t SensorDriverV4_ReadWaterCapacitance(float *capacitance_pf)
{
    /* 协议层会先关闭零点霍尔再开启水位，保证两个密度模式功能不同时开启。 */
    uint32_t result = SensorDriverV4_SetWaterEnabled(1U);

    if (result == NO_ERROR) {
        result = SensorDriverV4_WaitForValue(
            MULTIPARAM_V4_MeasurementReadWaterCapacitance,
            capacitance_pf);
    }
    return result;
}

/*
 * 函数用途：确保DM4零点霍尔开启并读取首个新鲜电压值。
 * 调用场景：SensorService读取DM4磁零点辅助量。
 * 关键约束：读取结束后保持零点霍尔开启；协议层会先关闭互斥的测水功能。
 */
static uint32_t SensorDriverV4_ReadMagneticZeroVoltage(float *voltage_v)
{
    /* 协议层会先关闭水位再开启零点霍尔，功能阶段结束后保持零点霍尔开启。 */
    uint32_t result = SensorDriverV4_SetMagneticZeroEnabled(1U);

    if (result == NO_ERROR) {
        result = SensorDriverV4_WaitForValue(
            MULTIPARAM_V4_MeasurementReadMagneticZeroVoltage,
            voltage_v);
    }
    return result;
}

/*
 * 函数用途：读取DM4主动快照中的频率、密度、温度并执行物理范围校验。
 * 调用场景：统一密度读取选择DM4-V4驱动时。
 * 关键约束：无效浮点值由测量适配层处理；有限但越界的值返回响应格式异常。
 */
static uint32_t SensorDriverV4_ReadDensity(float *frequency_hz,
                                           float *density_kg_m3,
                                           float *temperature_c)
{
    uint32_t result = MULTIPARAM_V4_MeasurementReadDensity(frequency_hz,
                                                           density_kg_m3,
                                                           temperature_c);
    if ((result == NO_ERROR) &&
        ((*frequency_hz < 0.0f) || (*frequency_hz > 6600.0f) ||
         (*density_kg_m3 < 0.0f) || (*density_kg_m3 > 3000.0f) ||
         (*temperature_c < -200.0f) || (*temperature_c > 300.0f))) {
        result = SENSOR_RESP_FORMAT_ERROR;
    }
    return result;
}

/*
 * 函数用途：根据已建立的Safe会话动态汇总可用能力。
 * 调用场景：统一服务层查询DM4-Safe预留驱动能力时。
 * 关键约束：水位和姿态能力必须由HELLO后的适配器确认，未确认时不得发布。
 */

static uint32_t SensorDriverSafe_GetCapabilities(void)
{
    uint32_t capabilities = SENSOR_CAP_DENSITY |
                            SENSOR_CAP_LEVEL |
                            SENSOR_CAP_SAFE_SESSION;

    if (SensorSafeAdapter_SupportsWaterCap() != 0U) {
        capabilities |= SENSOR_CAP_WATER;
    }
    if (SensorSafeAdapter_SupportsGyro() != 0U) {
        capabilities |= SENSOR_CAP_GYRO;
    }
    return capabilities;
}

/* 未识别状态的空驱动；所有能力关闭且没有协议回调。 */
static const SensorDriverOps s_none_driver = {
    .kind = SENSOR_DRIVER_NONE,
    .capabilities = SENSOR_CAP_NONE
};

/* DSM一代文本协议驱动表；能力与原DSM接口保持一致。 */
static const SensorDriverOps s_dsm_driver = {
    .kind = SENSOR_DRIVER_DSM,
    .capabilities = SENSOR_CAP_DENSITY | SENSOR_CAP_LEVEL | SENSOR_CAP_WATER |
                    SENSOR_CAP_GYRO | SENSOR_CAP_SUPPLY_VOLTAGE |
                    SENSOR_CAP_DENSITY_ANALYSIS,
    .enable_density_mode = SensorDriverDsm_EnableDensityMode,
    .enable_level_mode = SensorDriverDsm_EnableLevelMode,
    .read_density = SensorDriverDsm_ReadDensity,
    .read_level_frequency = Read_Level_Frequency,
    .read_water_capacitance = Read_Water_Capacitance,
    .read_gyro = Read_Gyro_Angle,
    .read_supply_voltage = Read_Sensor_Voltage,
    .read_density_analysis = DSM_ReadDensityAnalysis
};

/* 多参数V3固定帧驱动表。 */
static const SensorDriverOps s_v3_driver = {
    .kind = SENSOR_DRIVER_MULTIPARAM_V3,
    .capabilities = SENSOR_CAP_DENSITY | SENSOR_CAP_LEVEL,
    .enable_density_mode = SensorDriverV3_EnableDensityMode,
    .enable_level_mode = SensorDriverV3_EnableLevelMode,
    .read_density = SensorDriverV3_ReadDensity,
    .read_level_frequency = SensorDriverV3_ReadLevelFrequency
};

/* DM4普通V4协议驱动表，支持主动/交互以及密度模式互斥功能。 */
static const SensorDriverOps s_v4_driver = {
    .kind = SENSOR_DRIVER_DM4,
    .capabilities = SENSOR_CAP_DENSITY | SENSOR_CAP_LEVEL | SENSOR_CAP_WATER |
                    SENSOR_CAP_GYRO | SENSOR_CAP_MAGNETIC_ZERO |
                    SENSOR_CAP_ACTIVE_REPORT,
    .enable_density_mode = SensorDriverV4_EnableDensityMode,
    .enable_level_mode = SensorDriverV4_EnableLevelMode,
    .read_density = SensorDriverV4_ReadDensity,
    .read_level_frequency = MULTIPARAM_V4_MeasurementReadLevelFrequency,
    .read_water_capacitance = SensorDriverV4_ReadWaterCapacitance,
    .read_gyro = MULTIPARAM_V4_MeasurementReadGyro,
    .read_magnetic_zero_voltage = SensorDriverV4_ReadMagneticZeroVoltage
};

/* DM4预留Safe会话驱动表；仅在显式Safe会话激活后选择。 */
static const SensorDriverOps s_safe_driver = {
    .kind = SENSOR_DRIVER_DM4,
    .capabilities = SENSOR_CAP_DENSITY | SENSOR_CAP_LEVEL | SENSOR_CAP_SAFE_SESSION,
    .get_capabilities = SensorDriverSafe_GetCapabilities,
    .enable_density_mode = SensorSafeAdapter_EnableDensityMode,
    .enable_level_mode = SensorSafeAdapter_EnableLevelMode,
    .read_density = SensorSafeAdapter_ReadDensity,
    .read_level_frequency = SensorSafeAdapter_ReadLevelFrequency,
    .read_water_capacitance = SensorSafeAdapter_ReadWaterCapacitance,
    .read_gyro = SensorSafeAdapter_ReadGyroAngle
};

/*
 * 函数用途：读取驱动的固定能力或运行时动态能力。
 * 调用场景：SensorService和部件诊断调用可选操作之前。
 * 关键约束：空驱动返回无能力；存在get_capabilities时以动态结果为准而非做并集。
 */
uint32_t SensorDriver_GetCapabilities(const SensorDriverOps *driver)
{
    /* Safe能力可由会话适配器动态给出，其余驱动使用注册表中的固定能力位。 */
    if (driver == NULL) {
        return SENSOR_CAP_NONE;
    }
    return (driver->get_capabilities != NULL)
               ? driver->get_capabilities()
               : driver->capabilities;
}

/*
 * 函数用途：判断驱动是否完整具备指定能力位集合。
 * 调用场景：调用水位、姿态、供电或扩展诊断操作之前。
 * 关键约束：要求所有请求位都存在；空驱动通过GetCapabilities自然返回不支持。
 */
uint8_t SensorDriver_HasCapability(const SensorDriverOps *driver, uint32_t capability)
{
    return ((SensorDriver_GetCapabilities(driver) & capability) == capability) ? 1U : 0U;
}

/*
 * 函数用途：根据物理传感器类型和DM4会话状态返回唯一驱动操作表。
 * 调用场景：SensorRuntime每次分派业务操作或查询驱动类别时。
 * 关键约束：DM4物理类型固定为14；只有已激活Safe会话才选择预留Safe驱动。
 */
const SensorDriverOps *SensorDriverRegistry_Resolve(uint32_t sensor_type,
                                                    uint8_t dm4_safe_session_active)
{
    /* 未识别或保留类型统一返回空驱动，调用方通过能力位和空操作指针拒绝业务调用。 */
    switch ((SENSOR_TYPE)sensor_type) {
    case DSM_SENSOR:
        return &s_dsm_driver;
    case LTD_SENSOR:
        return &s_v3_driver;
    case DM4_SENSOR:
        return (dm4_safe_session_active != 0U) ? &s_safe_driver : &s_v4_driver;
    default:
        return &s_none_driver;
    }
}
