#include "relay_output.h"

#include "main.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RELAY_OUTPUT_ACTIVE_LEVEL   GPIO_PIN_SET /* 继电器输出参数：继电器 输出 有效 液位。 */
#define RELAY_OUTPUT_INACTIVE_LEVEL GPIO_PIN_RESET /* 继电器输出参数：继电器 输出 无效 液位。 */

typedef enum {
    /* 继电器报警阈值的比较方向。 */
    RELAY_COMPARE_GREATER = 0U, /* 过程量大于阈值时满足报警比较条件。 */
    RELAY_COMPARE_LESS = 1U /* 过程量小于阈值时满足报警比较条件。 */
} RelayCompareType;

typedef struct {
    /* 继电器逻辑通道到 GPIO 端口和引脚的只读映射。 */
    GPIO_TypeDef *port; /* 继电器通道对应的 GPIO 端口。 */
    uint16_t pin; /* 继电器通道对应的 GPIO 引脚位掩码。 */
} RelayOutputIo;

typedef struct {
    /* 一次继电器判定使用的测量与抑制快照；整轮四通道计算必须复用同一份输入。 */
    uint32_t manual_alarm_inhibit; /* 人工报警抑制在本轮继电器判定中的快照值。 */
    uint32_t maintenance_mode_active; /* 维护模式在本轮继电器判定中的快照值。 */
    uint32_t oil_level; /* 本轮继电器判定使用的油位过程量。 */
    uint32_t probe_at_liquid_level; /* 本轮继电器判定使用的探头已到液面标志。 */
    uint32_t liquid_stable; /* 本轮继电器判定使用的液面稳定标志。 */
    int32_t temperature; /* 本轮继电器判定使用的有符号温度过程量。 */
    uint32_t water_level; /* 本轮继电器判定使用的水位过程量。 */
    int32_t sensor_position; /* 本轮继电器判定使用的有符号传感器位置。 */
} RelayOutputMeasurementSnapshot;

/* 四路继电器逻辑通道到 GPIO 端口和引脚的只读映射表。 */
static const RelayOutputIo relay_output_ios[RELAY_OUTPUT_COUNT] = {
    { RELAY1_GPIO_Port, RELAY1_Pin },
    { RELAY2_GPIO_Port, RELAY2_Pin },
    { RELAY3_GPIO_Port, RELAY3_Pin },
    { RELAY4_GPIO_Port, RELAY4_Pin },
};

static volatile uint8_t relay_output_initialized = 0U; /* 继电器输出 GPIO 已完成安全初始态设置的标志。 */
static volatile uint8_t relay_output_updating = 0U; /* 四路继电器输出正在整批更新的互斥标志。 */
static volatile uint8_t relay_output_state_mask = 0U; /* 四路继电器当前逻辑动作状态位掩码；低四位分别对应通道 1～4，批量更新和单路设置后均按有效通道掩码保存。 */

/**
 * @brief 把继电器报警阈值原始位模式还原为单精度浮点值。
 *
 * @param raw 继电器报警阈值快照中的 IEEE 754 Float32 原始 32 位位模式。
 * @return 计算后的业务数值。
 */
static float RelayOutput_RawToFloat(uint32_t raw)
{
    float value;
    /* 把持久化的 uint32_t 原始位模式解释为 IEEE-754 Float32 阈值；memcpy 用于位级转换，不能改成数值强制转换。 */
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/**
 * @brief 把继电器比较值按 0.1 单位进行符号对称的四舍五入。
 *
 * @details 调用场景：RelayOutput_Compare 比较当前值、阈值和滞回前统一量化。
 * @note 关键约束：负数使用减 0.5 后截断，避免统一加 0.5 导致负值向零偏移。
 *
 * @param value 继电器输出使用的输入数值。
 * @return 返回工程值按 0.1 单位正负对称四舍五入后的有符号整数；溢出时饱和到 INT32 边界。
 */
static int32_t RelayOutput_ToTenths(float value)
{
    double scaled;

    if (!isfinite(value)) {
        return 0;
    }
    scaled = ((double)value) * 10.0;
    if (scaled >= (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled <= (double)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)(scaled + ((scaled >= 0.0) ? 0.5 : -0.5));
}

/**
 * @brief 将工程浮点值饱和并四舍五入为 0.1 单位，再按比较方向和滞回量判断阈值是否成立。
 *
 * @param current 当前继电器报警输入值。
 * @param target 本次比较、运动或写入的目标值。该浮点数是继电器报警阈值，函数按高限或低限方向与 current 比较。
 * @param hysteresis 滞回。
 * @param type 类型。
 * @return 1 表示当前 0.1 单位值按比较方向和滞回门限满足动作或释放条件；否则返回 0。
 */
static uint8_t RelayOutput_Compare(float current,
                                   float target,
                                   float hysteresis,
                                   RelayCompareType type)
{
    int32_t current_tenths;
    int32_t target_tenths;
    int32_t hysteresis_tenths;

    current_tenths = RelayOutput_ToTenths(current);
    target_tenths = RelayOutput_ToTenths(target);
    hysteresis_tenths = RelayOutput_ToTenths(hysteresis);

    if (type == RELAY_COMPARE_GREATER) {
        const int64_t release_threshold =
            (int64_t)target_tenths - (int64_t)hysteresis_tenths;
        return ((int64_t)current_tenths >= release_threshold) ? 1U : 0U;
    }

    {
        const int64_t release_threshold =
            (int64_t)target_tenths + (int64_t)hysteresis_tenths;
        return ((int64_t)current_tenths <= release_threshold) ? 1U : 0U;
    }
}

/**
 * @brief 进入继电器输出临界区并保存中断状态。
 * @return 返回进入继电器临界区前的 PRIMASK 中断屏蔽状态，供退出临界区时原样恢复。
 */
static uint32_t RelayOutput_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    /* 进入临界区，保护继电器输出共享状态，避免中断同时修改。 */
    __disable_irq();
    return primask;
}

/**
 * @brief 恢复进入继电器输出临界区前的中断状态。
 *
 * @param primask 进入临界区前保存的中断屏蔽状态。
 */
static void RelayOutput_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/**
 * @brief 在短临界区内复制指定通道的完整继电器配置快照；参数无效时不输出。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @param cfg 可写单路继电器报警配置对象；函数按职责从寄存器还原或更新模式、报警源、四级阈值、滞回、阻尼及锁存清除字段。
 */
static void RelayOutput_CopyConfigSnapshot(uint32_t channel, RelayAlarmConfig *cfg)
{
    uint32_t primask;
    const volatile RelayAlarmConfig *src;

    if ((cfg == NULL) || (channel >= RELAY_ALARM_CHANNEL_COUNT)) {
        return;
    }

    src = &g_deviceParams.relayAlarm[channel];
    primask = RelayOutput_EnterCritical();
    cfg->operating_mode = src->operating_mode;
    cfg->digital_source = src->digital_source;
    cfg->contact_type = src->contact_type;
    cfg->alarm_mode = src->alarm_mode;
    cfg->error_value = src->error_value;
    cfg->alarm_source = src->alarm_source;
    cfg->HH_alarm_value = src->HH_alarm_value;
    cfg->H_alarm_value = src->H_alarm_value;
    cfg->L_alarm_value = src->L_alarm_value;
    cfg->LL_alarm_value = src->LL_alarm_value;
    cfg->alarm_hysteresis = src->alarm_hysteresis;
    cfg->damping_factor = src->damping_factor;
    cfg->clear_alarm = src->clear_alarm;
    RelayOutput_ExitCritical(primask);
}

/**
 * @brief 在短临界区内复制报警抑制状态及油位、温度、水位和浮子位置测量快照。
 *
 * @param snapshot 继电器报警计算使用的测量快照输出对象；在临界区内复制液位、温度、水位、浮子位置及各字段有效性，避免一次判定读取到跨周期数据。
 */
static void RelayOutput_CopyMeasurementSnapshot(RelayOutputMeasurementSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return;
    }

    primask = RelayOutput_EnterCritical();
    snapshot->manual_alarm_inhibit = g_measurement.device_status.manual_alarm_inhibit;
    snapshot->maintenance_mode_active = g_measurement.device_status.maintenance_mode_active;
    snapshot->oil_level = g_measurement.oil_measurement.oil_level;
    snapshot->probe_at_liquid_level = g_measurement.oil_measurement.probe_at_liquid_level;
    snapshot->liquid_stable = g_measurement.oil_measurement.liquid_stable;
    snapshot->temperature = (int32_t)g_measurement.debug_data.temperature;
    snapshot->water_level = g_measurement.water_measurement.water_level;
    snapshot->sensor_position = g_measurement.debug_data.sensor_position;
    RelayOutput_ExitCritical(primask);
}

/**
 * @brief 在短临界区内复制指定通道的报警运行态快照；参数无效时不输出。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @param state 用于接收继电器报警运行态一致快照的输出对象。
 */
static void RelayOutput_CopyRuntimeSnapshot(uint32_t channel, RelayAlarmRuntimeState *state)
{
    uint32_t primask;
    const volatile RelayAlarmRuntimeState *src;

    if ((state == NULL) || (channel >= RELAY_ALARM_CHANNEL_COUNT)) {
        return;
    }

    src = &g_measurement.relay_alarm_runtime[channel];
    primask = RelayOutput_EnterCritical();
    state->alarm_value = src->alarm_value;
    state->HH_alarm = src->HH_alarm;
    state->H_alarm = src->H_alarm;
    state->HH_H_alarm = src->HH_H_alarm;
    state->L_alarm = src->L_alarm;
    state->LL_alarm = src->LL_alarm;
    state->LL_L_alarm = src->LL_L_alarm;
    state->any_error = src->any_error;
    state->clear_alarm = src->clear_alarm;
    RelayOutput_ExitCritical(primask);
}

/**
 * @brief 在短临界区内把指定通道的本轮报警运行态原子发布到共享测量结构。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @param state 已经完成本轮判定、准备提交到共享运行态的继电器状态。
 */
static void RelayOutput_CommitRuntimeState(uint32_t channel, const RelayAlarmRuntimeState *state)
{
    uint32_t primask;
    volatile RelayAlarmRuntimeState *dst;

    if ((state == NULL) || (channel >= RELAY_ALARM_CHANNEL_COUNT)) {
        return;
    }

    dst = &g_measurement.relay_alarm_runtime[channel];
    primask = RelayOutput_EnterCritical();
    dst->alarm_value = state->alarm_value;
    dst->HH_alarm = state->HH_alarm;
    dst->H_alarm = state->H_alarm;
    dst->HH_H_alarm = state->HH_H_alarm;
    dst->L_alarm = state->L_alarm;
    dst->LL_alarm = state->LL_alarm;
    dst->LL_L_alarm = state->LL_L_alarm;
    dst->any_error = state->any_error;
    dst->clear_alarm = state->clear_alarm;
    RelayOutput_ExitCritical(primask);
}

/**
 * @brief 发布继电器报警屏蔽状态和四路最终逻辑动作位图。
 * @param alarm_inhibited 非0表示当前报警动作被手动操作或维护模式屏蔽。
 * @param logical_action_mask bit0~bit3分别表示K1~K4最终逻辑报警动作。
 */
static void RelayOutput_CommitPublishedStatus(uint8_t alarm_inhibited,
                                              uint8_t logical_action_mask)
{
    uint32_t primask = RelayOutput_EnterCritical();

    g_measurement.device_status.relay_alarm_inhibit_effective =
            (alarm_inhibited != 0U) ? 1U : 0U;
    g_measurement.device_status.relay_alarm_action_mask =
            (uint32_t)(logical_action_mask & 0x0FU);
    RelayOutput_ExitCritical(primask);
}

/**
 * @brief 按报警源返回对应的无效工程值哨兵；未知来源返回 0。
 *
 * @param source 待判断或转换的数据来源枚举值。该 RelayAlarmSource 指定液位、温度、水位或浮子位置，用于选择测量无效时的报警策略。
 * @return 计算后的业务数值。
 */
static float RelayOutput_GetInvalidAlarmValue(uint32_t source)
{
    switch (source) {
    case RELAY_ALARM_SOURCE_TANK_LEVEL:
        return ((float)UNVALID_LEVEL) / 10.0f;
    case RELAY_ALARM_SOURCE_LIQUID_TEMP:
        return ((float)UNVALID_TEMPERATURE_WIRELESS) / 100.0f;
    case RELAY_ALARM_SOURCE_WATER_LEVEL:
        return ((float)LEVEL_DOWNLIMITWATER) / 10.0f;
    case RELAY_ALARM_SOURCE_DISPLACER_POS:
        return ((float)UNVALID_POSITION) / 10.0f;
    default:
        break;
    }
    return 0.0f;
}

/**
 * @brief 按报警源写入无效值，并把四级报警、组合报警、故障和清除请求复位为非活动态。
 *
 * @param state 待恢复为初始未激活状态的继电器运行态对象。
 * @param cfg 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 */
static void RelayOutput_ResetRuntimeState(RelayAlarmRuntimeState *state, const RelayAlarmConfig *cfg)
{
    uint32_t source = RELAY_ALARM_SOURCE_NONE;

    if (state == NULL) {
        return;
    }

    if (cfg != NULL) {
        source = cfg->alarm_source;
    }

    state->alarm_value = RelayOutput_GetInvalidAlarmValue(source);
    state->HH_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->H_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->HH_H_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->L_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->LL_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->LL_L_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->any_error = RELAY_ALARM_STATE_INACTIVE;
    state->clear_alarm = RELAY_ALARM_CLEAR_NO;
}

/**
 * @brief 把四路继电器逻辑动作位掩码写入 GPIO 输出。
 *
 * @param mask 四路继电器逻辑动作位掩码，低 4 位分别对应通道 1 至 4。
 */
static void RelayOutput_WriteMask(uint8_t mask)
{
    uint8_t active_mask = 0U;

    for (uint32_t channel = 0U; channel < (uint32_t)RELAY_OUTPUT_COUNT; channel++) {
        const uint8_t bit = (uint8_t)(1U << channel);
        const GPIO_PinState level = ((mask & bit) != 0U) ?
                                    RELAY_OUTPUT_ACTIVE_LEVEL :
                                    RELAY_OUTPUT_INACTIVE_LEVEL;
        HAL_GPIO_WritePin(relay_output_ios[channel].port, relay_output_ios[channel].pin, level);
        active_mask |= bit;
    }

    relay_output_state_mask = (uint8_t)(mask & active_mask);
}

/**
 * @brief 从一致测量快照取得配置所选报警源的工程值；样本无效或液位未稳定时保留哨兵并返回失败。
 *
 * @param cfg 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 * @param snapshot 只读继电器测量一致性快照；包含液位、温度、水位和浮子位置及有效标志，用于按当前报警源取得同一周期输入值。
 * @param value 用于返回配置所选报警源的工程值。
 * @return 1 表示已取得有效报警源工程值并写入输出参数；0 表示配置、测量样本或液位稳定性不满足要求。
 */
static uint8_t RelayOutput_GetAlarmSourceValue(const RelayAlarmConfig *cfg,
                                               const RelayOutputMeasurementSnapshot *snapshot,
                                               float *value)
{
    if ((cfg == NULL) || (snapshot == NULL) || (value == NULL)) {
        return 0U;
    }

    *value = RelayOutput_GetInvalidAlarmValue(cfg->alarm_source);

    switch (cfg->alarm_source) {
    case RELAY_ALARM_SOURCE_TANK_LEVEL:
        if (snapshot->oil_level == UNVALID_LEVEL) {
            return 0U;
        }
        if ((snapshot->probe_at_liquid_level == 0U) &&
            (snapshot->liquid_stable == 0U)) {
            return 0U;
        }
        *value = ((float)snapshot->oil_level) / 10.0f;
        return 1U;

    case RELAY_ALARM_SOURCE_LIQUID_TEMP:
        if ((snapshot->temperature == 0) ||
            (snapshot->temperature == (int32_t)UNVALID_TEMPERATURE_REALTIME) ||
            (snapshot->temperature == (int32_t)UNVALID_TEMPERATURE_WIRELESS)) {
            return 0U;
        }
        *value = ((float)snapshot->temperature) / 100.0f;
        return 1U;

    case RELAY_ALARM_SOURCE_WATER_LEVEL:
        if (snapshot->water_level == LEVEL_DOWNLIMITWATER) {
            return 0U;
        }
        *value = ((float)snapshot->water_level) / 10.0f;
        return 1U;

    case RELAY_ALARM_SOURCE_DISPLACER_POS:
        if (snapshot->sensor_position == (int32_t)UNVALID_POSITION) {
            return 0U;
        }
        *value = ((float)snapshot->sensor_position) / 10.0f;
        return 1U;

    default:
        break;
    }

    return 0U;
}

/**
 * @brief 测量源无效时清除四级模拟报警，再按 error_value 配置置位对应的故障替代状态。
 *
 * @param cfg 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 * @param state 样本无效时待更新阻尼和失效状态的继电器运行态对象。
 */
static void RelayOutput_ApplyInvalidState(const RelayAlarmConfig *cfg,
                                          RelayAlarmRuntimeState *state)
{
    if ((cfg == NULL) || (state == NULL)) {
        return;
    }

    state->alarm_value = RelayOutput_GetInvalidAlarmValue(cfg->alarm_source);
    state->HH_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->H_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->L_alarm = RELAY_ALARM_STATE_INACTIVE;
    state->LL_alarm = RELAY_ALARM_STATE_INACTIVE;

    switch (cfg->error_value) {
    case RELAY_ALARM_ERROR_HH_H:
        state->HH_alarm = RELAY_ALARM_STATE_ACTIVE;
        break;
    case RELAY_ALARM_ERROR_H:
        state->H_alarm = RELAY_ALARM_STATE_ACTIVE;
        break;
    case RELAY_ALARM_ERROR_L:
        state->L_alarm = RELAY_ALARM_STATE_ACTIVE;
        break;
    case RELAY_ALARM_ERROR_LL_L:
        state->LL_alarm = RELAY_ALARM_STATE_ACTIVE;
        break;
    case RELAY_ALARM_ERROR_ALL_ALARMS:
        state->HH_alarm = RELAY_ALARM_STATE_ACTIVE;
        state->H_alarm = RELAY_ALARM_STATE_ACTIVE;
        state->L_alarm = RELAY_ALARM_STATE_ACTIVE;
        state->LL_alarm = RELAY_ALARM_STATE_ACTIVE;
        break;
    default:
        break;
    }
}

/**
 * @brief 按报警模式处理阈值、滞回和锁存清除请求，返回本级报警的新状态。
 *
 * @param current 当前继电器报警输入值。
 * @param target 本次比较、运动或写入的目标值。该浮点数是当前报警级别阈值，函数结合 hysteresis 和上次状态执行回差判定。
 * @param hysteresis 滞回。
 * @param compare_type 类型。
 * @param current_state 本轮判断前的继电器报警运行态。
 * @param mode 工作模式。
 * @param state 用于保存本轮报警、阻尼和锁存结果的继电器运行态。
 * @return 返回本轮阈值、滞回和锁存处理后的 RELAY_ALARM_STATE_ACTIVE 或 RELAY_ALARM_STATE_INACTIVE。
 */
static uint32_t RelayOutput_JudgeAlarm(float current,
                                       float target,
                                       float hysteresis,
                                       RelayCompareType compare_type,
                                       uint32_t current_state,
                                       uint32_t mode,
                                       RelayAlarmRuntimeState *state)
{
    const uint8_t threshold_reached = RelayOutput_Compare(current, target, 0.0f, compare_type);

    if (mode == RELAY_ALARM_MODE_LATCHING) {
        if (threshold_reached != 0U) {
            state->clear_alarm = RELAY_ALARM_CLEAR_NO;
            return RELAY_ALARM_STATE_ACTIVE;
        }

        if (current_state == RELAY_ALARM_STATE_ACTIVE) {
            if (state->clear_alarm == RELAY_ALARM_CLEAR_YES) {
                return RELAY_ALARM_STATE_INACTIVE;
            }
            return RELAY_ALARM_STATE_ACTIVE;
        }

        return RELAY_ALARM_STATE_INACTIVE;
    }

    if (mode == RELAY_ALARM_MODE_ON) {
        const float lag = (current_state == RELAY_ALARM_STATE_ACTIVE) ? hysteresis : 0.0f;
        return (RelayOutput_Compare(current, target, lag, compare_type) != 0U) ?
               RELAY_ALARM_STATE_ACTIVE : RELAY_ALARM_STATE_INACTIVE;
    }

    return RELAY_ALARM_STATE_INACTIVE;
}

/**
 * @brief 对有效模拟量分别计算 HH、H、L、LL 四级报警，并更新本轮报警值和各级状态。
 *
 * @param cfg 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 * @param state 样本有效时待更新报警和阻尼结果的继电器运行态对象。
 * @param value 继电器输出应用有效状态使用的输入数值。
 */
static void RelayOutput_ApplyValidState(const RelayAlarmConfig *cfg,
                                        RelayAlarmRuntimeState *state,
                                        float value)
{
    const float hysteresis = RelayOutput_RawToFloat(cfg->alarm_hysteresis);
    const uint32_t mode = cfg->alarm_mode;

    state->alarm_value = value;
    state->HH_alarm = RelayOutput_JudgeAlarm(value,
                                             RelayOutput_RawToFloat(cfg->HH_alarm_value),
                                             hysteresis,
                                             RELAY_COMPARE_GREATER,
                                             state->HH_alarm,
                                             mode,
                                             state);
    state->H_alarm = RelayOutput_JudgeAlarm(value,
                                            RelayOutput_RawToFloat(cfg->H_alarm_value),
                                            hysteresis,
                                            RELAY_COMPARE_GREATER,
                                            state->H_alarm,
                                            mode,
                                            state);
    state->L_alarm = RelayOutput_JudgeAlarm(value,
                                            RelayOutput_RawToFloat(cfg->L_alarm_value),
                                            hysteresis,
                                            RELAY_COMPARE_LESS,
                                            state->L_alarm,
                                            mode,
                                            state);
    state->LL_alarm = RelayOutput_JudgeAlarm(value,
                                             RelayOutput_RawToFloat(cfg->LL_alarm_value),
                                             hysteresis,
                                             RELAY_COMPARE_LESS,
                                             state->LL_alarm,
                                             mode,
                                             state);
}

/**
 * @brief 按 digital_source 选择单级、组合或任意故障状态；非法选择返回非活动态。
 *
 * @param cfg 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 * @param state 用于保存数字量报警源判定结果的继电器运行态。
 * @return 返回所选数字报警源的活动状态；来源非法时返回 RELAY_ALARM_STATE_INACTIVE。
 */
static uint32_t RelayOutput_SelectDigitalState(const RelayAlarmConfig *cfg,
                                               const RelayAlarmRuntimeState *state)
{
    if ((cfg == NULL) || (state == NULL)) {
        return RELAY_ALARM_STATE_INACTIVE;
    }

    switch (cfg->digital_source) {
    case RELAY_ALARM_DIGITAL_H:
        return state->H_alarm;
    case RELAY_ALARM_DIGITAL_HH:
        return state->HH_alarm;
    case RELAY_ALARM_DIGITAL_H_OR_HH:
        return state->HH_H_alarm;
    case RELAY_ALARM_DIGITAL_L:
        return state->L_alarm;
    case RELAY_ALARM_DIGITAL_LL:
        return state->LL_alarm;
    case RELAY_ALARM_DIGITAL_L_OR_LL:
        return state->LL_L_alarm;
    case RELAY_ALARM_DIGITAL_ANY:
        return state->any_error;
    default:
        return RELAY_ALARM_STATE_INACTIVE;
    }
}

/**
 * @brief 消费指定通道的锁存报警清除命令。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @param cfg 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 * @param state 待清除锁存报警并记录消费结果的继电器运行态对象。
 */
static void RelayOutput_ConsumeClearCommand(uint32_t channel,
                                            const RelayAlarmConfig *cfg,
                                            RelayAlarmRuntimeState *state)
{
    uint32_t primask;

    if ((cfg == NULL) || (state == NULL)) {
        return;
    }

    if (cfg->clear_alarm == RELAY_ALARM_CLEAR_YES) {
        state->clear_alarm = RELAY_ALARM_CLEAR_YES;
        primask = RelayOutput_EnterCritical();
        g_deviceParams.relayAlarm[channel].clear_alarm = RELAY_ALARM_CLEAR_NO;
        RelayOutput_ExitCritical(primask);
    }
}

/**
 * @brief 按单通道配置、数据有效性、锁存和滞回规则计算并提交继电器输出。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @param measurement 本次处理使用的测量结果对象。该只读一致性快照包含液位、温度、水位、浮子位置及各字段有效性，用于单路继电器报警计算。
 * @param alarm_inhibited 报警。
 * @param logical_action_active 用于返回本轮逻辑报警动作是否处于有效状态。
 * @return 1 表示本轮通道计算完成且输出状态有效；配置或采样无效时按安全状态处理并返回 0。
 */
static uint8_t RelayOutput_UpdateChannel(uint32_t channel,
                                         const RelayOutputMeasurementSnapshot *measurement,
                                         uint8_t alarm_inhibited,
                                         uint8_t *logical_action_active)
{
    RelayAlarmConfig cfg;
    RelayAlarmRuntimeState state;
    float alarm_value = 0.0f;
    uint32_t digital_state;
    uint8_t coil_active;

    RelayOutput_CopyConfigSnapshot(channel, &cfg);
    RelayOutput_CopyRuntimeSnapshot(channel, &state);
    RelayOutput_ConsumeClearCommand(channel, &cfg, &state);

    if (logical_action_active != NULL) {
        *logical_action_active = 0U;
    }

    if ((cfg.operating_mode != RELAY_ALARM_OPERATING_OUTPUT_PASSIVE) ||
        (cfg.alarm_source >= RELAY_ALARM_SOURCE_NONE)) {
        RelayOutput_ResetRuntimeState(&state, &cfg);
        RelayOutput_CommitRuntimeState(channel, &state);
        return 0U;
    }

    if (RelayOutput_GetAlarmSourceValue(&cfg, measurement, &alarm_value) != 0U) {
        RelayOutput_ApplyValidState(&cfg, &state, alarm_value);
    } else {
        RelayOutput_ApplyInvalidState(&cfg, &state);
    }

    state.HH_H_alarm = ((state.HH_alarm == RELAY_ALARM_STATE_ACTIVE) ||
                        (state.H_alarm == RELAY_ALARM_STATE_ACTIVE)) ?
                       RELAY_ALARM_STATE_ACTIVE : RELAY_ALARM_STATE_INACTIVE;
    state.LL_L_alarm = ((state.LL_alarm == RELAY_ALARM_STATE_ACTIVE) ||
                        (state.L_alarm == RELAY_ALARM_STATE_ACTIVE)) ?
                       RELAY_ALARM_STATE_ACTIVE : RELAY_ALARM_STATE_INACTIVE;
    state.any_error = ((state.HH_H_alarm == RELAY_ALARM_STATE_ACTIVE) ||
                       (state.LL_L_alarm == RELAY_ALARM_STATE_ACTIVE)) ?
                      RELAY_ALARM_STATE_ACTIVE : RELAY_ALARM_STATE_INACTIVE;

    digital_state = RelayOutput_SelectDigitalState(&cfg, &state);
    /* 屏蔽期间继续计算阈值和锁存，只把最终逻辑动作压为0。 */
    coil_active = ((digital_state == RELAY_ALARM_STATE_ACTIVE) &&
                   (alarm_inhibited == 0U)) ? 1U : 0U;
    if (logical_action_active != NULL) {
        *logical_action_active = coil_active;
    }

    if (cfg.contact_type == RELAY_ALARM_CONTACT_NORMALLY_CLOSED) {
        coil_active = (coil_active == 0U) ? 1U : 0U;
    }

    RelayOutput_CommitRuntimeState(channel, &state);
    return coil_active;
}

/**
 * @brief 汇总四路输出、报警、锁存和无效状态位掩码。
 * @return 返回汇总四路输出、报警、锁存和无效状态位掩码对应的位掩码；各位含义由相邻枚举或宏定义。
 */
static uint8_t RelayOutput_BuildStateMask(void)
{
    RelayOutputMeasurementSnapshot measurement;
    uint8_t physical_mask = 0U;
    uint8_t logical_action_mask = 0U;
    uint8_t logical_action_active;
    uint8_t alarm_inhibited;

    RelayOutput_CopyMeasurementSnapshot(&measurement);
    alarm_inhibited = ((measurement.manual_alarm_inhibit != 0U) ||
                       (measurement.maintenance_mode_active != 0U)) ? 1U : 0U;

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        logical_action_active = 0U;
        if (RelayOutput_UpdateChannel(channel,
                                      &measurement,
                                      alarm_inhibited,
                                      &logical_action_active) != 0U) {
            physical_mask |= (uint8_t)(1U << channel);
        }
        if (logical_action_active != 0U) {
            logical_action_mask |= (uint8_t)(1U << channel);
        }
    }

    RelayOutput_CommitPublishedStatus(alarm_inhibited, logical_action_mask);
    return physical_mask;
}

/**
 * @brief 初始化继电器输出运行态并按安全默认值驱动四路 GPIO。
 */
void RelayOutput_Init(void)
{
    RelayAlarmConfig cfg;
    RelayAlarmRuntimeState state;

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        RelayOutput_CopyConfigSnapshot(channel, &cfg);
        RelayOutput_ResetRuntimeState(&state, &cfg);
        RelayOutput_CommitRuntimeState(channel, &state);
    }
    RelayOutput_CommitPublishedStatus(0U, 0U);
    RelayOutput_WriteMask(0U);
    relay_output_initialized = 1U;
}

/**
 * @brief 请求清除四路继电器锁存报警。
 * @note 可由主循环调用；只写一次性运行请求，不保存到FRAM。
 */
void RelayOutput_RequestClearAllLatchedAlarms(void)
{
    uint32_t primask = RelayOutput_EnterCritical();

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        g_deviceParams.relayAlarm[channel].clear_alarm = RELAY_ALARM_CLEAR_YES;
    }
    RelayOutput_ExitCritical(primask);
}


/**
 * @brief 根据最新测量值、故障和报警配置刷新四路继电器输出。
 */
void RelayOutput_Update(void)
{
    uint8_t mask;

    if ((relay_output_initialized == 0U) || (relay_output_updating != 0U)) {
        return;
    }

    relay_output_updating = 1U;
    mask = RelayOutput_BuildStateMask();
    RelayOutput_WriteMask(mask);
    relay_output_updating = 0U;
}

/**
 * @brief 按逻辑动作和触点类型设置指定继电器通道。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @param active 目标活动状态，true 表示活动。
 */
void RelayOutput_SetChannel(RelayOutputChannel channel, uint8_t active)
{
    uint8_t mask;

    if ((relay_output_initialized == 0U) || (channel >= RELAY_OUTPUT_COUNT)) {
        return;
    }

    mask = relay_output_state_mask;
    if (active != 0U) {
        mask |= (uint8_t)(1U << (uint32_t)channel);
    } else {
        mask &= (uint8_t)~(uint8_t)(1U << (uint32_t)channel);
    }

    RelayOutput_WriteMask(mask);
}

/**
 * @brief 返回四路继电器当前逻辑动作的位掩码。
 * @return 返回四路继电器当前逻辑动作的位掩码对应的位掩码；各位含义由相邻枚举或宏定义。
 */
uint8_t RelayOutput_GetStateMask(void)
{
    return relay_output_state_mask;
}

/**
 * @brief 复制指定通道的继电器报警运行态快照。
 *
 * @param channel 零基通道号。合法范围为 0～3，用于选择四路继电器配置、运行态、锁存命令和物理输出。
 * @return 成功时返回指向复制指定通道的继电器报警运行态快照的指针；输入非法或未找到匹配项时返回 NULL。
 */
const volatile RelayAlarmRuntimeState *RelayOutput_GetRuntimeState(uint32_t channel)
{
    if (channel >= RELAY_ALARM_CHANNEL_COUNT) {
        return NULL;
    }
    return &g_measurement.relay_alarm_runtime[channel];
}
