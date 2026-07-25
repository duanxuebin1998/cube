#include "relay_alarm_config.h"
#include "stateformodbus.h"

#include <math.h>
#include <string.h>

typedef struct {
    float minimum;
    float maximum;
    bool bounded;
} RelayAlarmPhysicalRange;

/*
 * 函数用途：把继电器配置中的 IEEE754 原始位还原为 Float32。
 * 调用场景：阈值和滞回校验读取持久化配置时调用。
 * 关键约束：只按位复制，不改变共享协议和 FRAM 字段布局。
 */
static float RelayAlarmConfig_RawToFloat(uint32_t raw)
{
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

/*
 * 函数用途：把归一化后的 Float32 按位保存为继电器配置原始值。
 * 调用场景：旧 FRAM 非法字段恢复为 0 时调用。
 * 关键约束：只按位复制，不执行数值类型转换。
 */
static uint32_t RelayAlarmConfig_FloatToRaw(float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

/*
 * 函数用途：取得报警源允许的物理范围。
 * 调用场景：校验四级阈值和报警滞回前调用。
 * 关键约束：液位使用液位罐高，水位使用水位罐高；浮子位置暂不增加物理边界。
 */
static RelayAlarmPhysicalRange RelayAlarmConfig_GetPhysicalRange(
    const DeviceParameters *params,
    uint32_t alarm_source)
{
    RelayAlarmPhysicalRange range = {0.0f, 0.0f, false};

    if (params == NULL) {
        return range;
    }

    switch (alarm_source) {
    case RELAY_ALARM_SOURCE_TANK_LEVEL:
        range.maximum = ((float)params->tankHeight) / 10.0f;
        range.bounded = true;
        break;
    case RELAY_ALARM_SOURCE_LIQUID_TEMP:
        range.minimum = -50.0f;
        range.maximum = 200.0f;
        range.bounded = true;
        break;
    case RELAY_ALARM_SOURCE_WATER_LEVEL:
        /* 水位报警上限只取水位罐高，不借用液位罐高。 */
        range.maximum = ((float)params->water_tank_height) / 10.0f;
        range.bounded = true;
        break;
    case RELAY_ALARM_SOURCE_DISPLACER_POS:
    case RELAY_ALARM_SOURCE_NONE:
    default:
        break;
    }

    return range;
}

/*
 * 函数用途：校验单个报警阈值的有限性和当前报警源物理范围。
 * 调用场景：禁用通道逐项配置、完整配置校验和旧 FRAM 逐字段归一化共用。
 * 关键约束：报警源枚举非法时不得把阈值按无界值放行。
 */
static bool RelayAlarmConfig_ThresholdValueIsValid(
    const DeviceParameters *params,
    const RelayAlarmConfig *config,
    uint32_t raw_value)
{
    RelayAlarmPhysicalRange range;
    float value;

    if ((params == NULL) || (config == NULL)) {
        return false;
    }
    if (config->alarm_source > RELAY_ALARM_SOURCE_NONE) {
        return false;
    }

    value = RelayAlarmConfig_RawToFloat(raw_value);
    if (!isfinite(value)) {
        return false;
    }

    range = RelayAlarmConfig_GetPhysicalRange(params, config->alarm_source);
    if (range.bounded &&
        ((value < range.minimum) || (value > range.maximum))) {
        return false;
    }
    return true;
}

/*
 * 函数用途：只校验 HH、H、L、LL 四级阈值的相对顺序。
 * 调用场景：完整配置启用校验和旧 FRAM 顺序冲突识别。
 * 关键约束：允许相邻阈值相等；顺序冲突时迁移流程保留原值并禁用通道。
 */
static bool RelayAlarmConfig_ThresholdOrderIsValid(
    const RelayAlarmConfig *config)
{
    float hh;
    float h;
    float l;
    float ll;

    if (config == NULL) {
        return false;
    }
    hh = RelayAlarmConfig_RawToFloat(config->HH_alarm_value);
    h = RelayAlarmConfig_RawToFloat(config->H_alarm_value);
    l = RelayAlarmConfig_RawToFloat(config->L_alarm_value);
    ll = RelayAlarmConfig_RawToFloat(config->LL_alarm_value);
    return (hh >= h) && (h >= l) && (l >= ll);
}

/*
 * 函数用途：校验 HH、H、L、LL 四级阈值的有限性、顺序和物理范围。
 * 调用场景：启用通道的完整配置校验。
 * 关键约束：禁用通道逐项写入不调用本函数，启用前必须满足 HH >= H >= L >= LL。
 */
static bool RelayAlarmConfig_ThresholdsAreValid(
    const DeviceParameters *params,
    const RelayAlarmConfig *config)
{
    if ((params == NULL) || (config == NULL)) {
        return false;
    }
    return RelayAlarmConfig_ThresholdValueIsValid(
               params, config, config->HH_alarm_value) &&
           RelayAlarmConfig_ThresholdValueIsValid(
               params, config, config->H_alarm_value) &&
           RelayAlarmConfig_ThresholdValueIsValid(
               params, config, config->L_alarm_value) &&
           RelayAlarmConfig_ThresholdValueIsValid(
               params, config, config->LL_alarm_value) &&
           RelayAlarmConfig_ThresholdOrderIsValid(config);
}

/*
 * 函数用途：校验报警滞回为有限非负值，并限制在有界报警源的量程跨度内。
 * 调用场景：FC10 候选提交和旧 FRAM 归一化共用。
 * 关键约束：浮子位置没有新增物理范围，但仍拒绝负数、NaN 和无穷值。
 */
static bool RelayAlarmConfig_HysteresisIsValid(
    const DeviceParameters *params,
    const RelayAlarmConfig *config)
{
    RelayAlarmPhysicalRange range;
    float hysteresis;

    if ((params == NULL) || (config == NULL)) {
        return false;
    }
    if (config->alarm_source > RELAY_ALARM_SOURCE_NONE) {
        return false;
    }

    hysteresis = RelayAlarmConfig_RawToFloat(config->alarm_hysteresis);
    if (!isfinite(hysteresis) || (hysteresis < 0.0f)) {
        return false;
    }

    range = RelayAlarmConfig_GetPhysicalRange(params, config->alarm_source);
    if (range.bounded && (hysteresis > (range.maximum - range.minimum))) {
        return false;
    }
    return true;
}

/*
 * 函数用途：校验一路继电器报警配置的枚举、阻尼、阈值和滞回。
 * 调用场景：CPU2 权威写入校验和启动归一化调用。
 * 关键约束：当前阻尼字段为预留项，只接受 0。
 */
bool RelayAlarmConfig_IsValid(const DeviceParameters *params,
                              const RelayAlarmConfig *config)
{
    if ((params == NULL) || (config == NULL)) {
        return false;
    }

    if ((config->operating_mode > RELAY_ALARM_OPERATING_OUTPUT_PASSIVE) ||
        (config->digital_source > RELAY_ALARM_DIGITAL_ANY) ||
        (config->contact_type > RELAY_ALARM_CONTACT_NORMALLY_CLOSED) ||
        (config->alarm_mode > RELAY_ALARM_MODE_LATCHING) ||
        (config->error_value > RELAY_ALARM_ERROR_ALL_ALARMS) ||
        (config->alarm_source > RELAY_ALARM_SOURCE_NONE) ||
        (config->damping_factor != 0U) ||
        (config->clear_alarm > RELAY_ALARM_CLEAR_YES)) {
        return false;
    }

    return RelayAlarmConfig_ThresholdsAreValid(params, config) &&
           RelayAlarmConfig_HysteresisIsValid(params, config);
}

/*
 * 函数用途：判断一段 FC10 写区间是否触及指定的完整 32 位字段。
 * 调用场景：继电器候选写校验按字段判断本次实际修改范围。
 * 关键约束：共享写入只接受完整 32 位字段，不能把相邻未写字段纳入校验。
 */
static bool RelayAlarmConfig_FieldIsTouched(uint16_t start,
                                            uint16_t count,
                                            uint16_t field_address)
{
    return LtdModbus_RangeContains(start, count, field_address, REG_STRIDE);
}

/*
 * 函数用途：校验禁用通道本次实际触及的字段。
 * 调用场景：先逐项配置阈值、报警源和其它字段，再单独启用通道。
 * 关键约束：阈值只校验本字段有限性和物理范围，不要求尚未配置完成的四级顺序。
 */
static bool RelayAlarmConfig_TouchedFieldsAreValid(
    const DeviceParameters *params,
    const RelayAlarmConfig *config,
    uint32_t channel,
    uint16_t start,
    uint16_t count)
{
    if ((params == NULL) || (config == NULL)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(channel)) &&
        (config->operating_mode > RELAY_ALARM_OPERATING_OUTPUT_PASSIVE)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(channel)) &&
        (config->digital_source > RELAY_ALARM_DIGITAL_ANY)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(channel)) &&
        (config->contact_type > RELAY_ALARM_CONTACT_NORMALLY_CLOSED)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(channel)) &&
        (config->alarm_mode > RELAY_ALARM_MODE_LATCHING)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(channel)) &&
        (config->error_value > RELAY_ALARM_ERROR_ALL_ALARMS)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(channel)) &&
        (config->alarm_source > RELAY_ALARM_SOURCE_NONE)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(channel)) &&
        !RelayAlarmConfig_ThresholdValueIsValid(
            params, config, config->HH_alarm_value)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(channel)) &&
        !RelayAlarmConfig_ThresholdValueIsValid(
            params, config, config->H_alarm_value)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(channel)) &&
        !RelayAlarmConfig_ThresholdValueIsValid(
            params, config, config->L_alarm_value)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(channel)) &&
        !RelayAlarmConfig_ThresholdValueIsValid(
            params, config, config->LL_alarm_value)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(channel)) &&
        !RelayAlarmConfig_HysteresisIsValid(params, config)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(channel)) &&
        (config->damping_factor != 0U)) {
        return false;
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count,
            HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(channel)) &&
        (config->clear_alarm > RELAY_ALARM_CLEAR_YES)) {
        return false;
    }
    return true;
}

/*
 * 函数用途：按 FC10 实际写区间校验继电器候选参数。
 * 调用场景：CPU2 权威写入口解析完整候选快照后、提交运行态和 FRAM 前调用。
 * 关键约束：禁用通道允许逐项形成暂时不完整组合；启用或已启用通道必须完整合法；
 *           罐高变化只复核启用且实际使用对应液位源的通道。
 */
bool RelayAlarmConfig_WriteCandidateIsValid(const DeviceParameters *params,
                                            uint16_t start,
                                            uint16_t count)
{
    uint32_t channel;

    if ((params == NULL) || (count == 0U)) {
        return false;
    }
    for (channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        const RelayAlarmConfig *config = &params->relayAlarm[channel];
        uint32_t channel_start =
            HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(channel);
        uint32_t channel_end = channel_start +
            HOLDREGISTER_RELAY_ALARM_CHANNEL_REG_COUNT;
        uint32_t write_end = (uint32_t)start + (uint32_t)count;
        bool channel_touched =
            ((uint32_t)start < channel_end) && (write_end > channel_start);

        if (!channel_touched) {
            continue;
        }
        if (config->operating_mode == RELAY_ALARM_OPERATING_OUTPUT_PASSIVE) {
            if (!RelayAlarmConfig_IsValid(params, config)) {
                return false;
            }
        } else if (!RelayAlarmConfig_TouchedFieldsAreValid(
                       params, config, channel, start, count)) {
            return false;
        }
    }

    if (RelayAlarmConfig_FieldIsTouched(
            start, count, HOLDREGISTER_DEVICEPARAM_TANKHEIGHT)) {
        for (channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
            const RelayAlarmConfig *config = &params->relayAlarm[channel];

            if ((config->operating_mode ==
                 RELAY_ALARM_OPERATING_OUTPUT_PASSIVE) &&
                (config->alarm_source == RELAY_ALARM_SOURCE_TANK_LEVEL) &&
                !RelayAlarmConfig_IsValid(params, config)) {
                return false;
            }
        }
    }
    if (RelayAlarmConfig_FieldIsTouched(
            start, count, HOLDREGISTER_DEVICEPARAM_WATER_TANK_HEIGHT)) {
        for (channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
            const RelayAlarmConfig *config = &params->relayAlarm[channel];

            if ((config->operating_mode ==
                 RELAY_ALARM_OPERATING_OUTPUT_PASSIVE) &&
                (config->alarm_source == RELAY_ALARM_SOURCE_WATER_LEVEL) &&
                !RelayAlarmConfig_IsValid(params, config)) {
                return false;
            }
        }
    }
    return true;
}

/*
 * 函数用途：按字段修复旧 FRAM 中的非法继电器配置并报告通道掩码。
 * 调用场景：CPU2 上电加载 DeviceParameters 后调用。
 * 关键约束：非法通道必须禁用；合法的一次性清报警命令只清零，不计为非法通道。
 */
uint32_t RelayAlarmConfig_Normalize(DeviceParameters *params,
                                    uint32_t *invalid_channel_mask)
{
    const uint32_t zero_float_raw = RelayAlarmConfig_FloatToRaw(0.0f);
    uint32_t changed_mask = 0U;
    uint32_t invalid_mask = 0U;
    uint32_t channel;

    if (invalid_channel_mask != NULL) {
        *invalid_channel_mask = 0U;
    }
    if (params == NULL) {
        return 0U;
    }

    for (channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        RelayAlarmConfig *config = &params->relayAlarm[channel];
        const uint32_t channel_bit = 1UL << channel;
        bool changed = false;
        bool invalid = false;

        if (config->operating_mode > RELAY_ALARM_OPERATING_OUTPUT_PASSIVE) {
            config->operating_mode = RELAY_ALARM_OPERATING_DISABLED;
            changed = true;
            invalid = true;
        }
        if (config->digital_source > RELAY_ALARM_DIGITAL_ANY) {
            config->digital_source = RELAY_ALARM_DIGITAL_NONE;
            changed = true;
            invalid = true;
        }
        if (config->contact_type > RELAY_ALARM_CONTACT_NORMALLY_CLOSED) {
            config->contact_type = RELAY_ALARM_CONTACT_NORMALLY_OPEN;
            changed = true;
            invalid = true;
        }
        if (config->alarm_mode > RELAY_ALARM_MODE_LATCHING) {
            config->alarm_mode = RELAY_ALARM_MODE_OFF;
            changed = true;
            invalid = true;
        }
        if (config->error_value > RELAY_ALARM_ERROR_ALL_ALARMS) {
            config->error_value = RELAY_ALARM_ERROR_NO_ALARM;
            changed = true;
            invalid = true;
        }
        if (config->alarm_source > RELAY_ALARM_SOURCE_NONE) {
            config->alarm_source = RELAY_ALARM_SOURCE_NONE;
            changed = true;
            invalid = true;
        }

        if (!RelayAlarmConfig_ThresholdValueIsValid(
                params, config, config->HH_alarm_value)) {
            config->HH_alarm_value = zero_float_raw;
            changed = true;
            invalid = true;
        }
        if (!RelayAlarmConfig_ThresholdValueIsValid(
                params, config, config->H_alarm_value)) {
            config->H_alarm_value = zero_float_raw;
            changed = true;
            invalid = true;
        }
        if (!RelayAlarmConfig_ThresholdValueIsValid(
                params, config, config->L_alarm_value)) {
            config->L_alarm_value = zero_float_raw;
            changed = true;
            invalid = true;
        }
        if (!RelayAlarmConfig_ThresholdValueIsValid(
                params, config, config->LL_alarm_value)) {
            config->LL_alarm_value = zero_float_raw;
            changed = true;
            invalid = true;
        }
        if (!RelayAlarmConfig_ThresholdOrderIsValid(config)) {
            /* 无法可靠判断哪一级阈值错误，保留四个值并禁用通道等待人工确认。 */
            invalid = true;
        }
        if (!RelayAlarmConfig_HysteresisIsValid(params, config)) {
            config->alarm_hysteresis = zero_float_raw;
            changed = true;
            invalid = true;
        }
        if (config->damping_factor != 0U) {
            config->damping_factor = 0U;
            changed = true;
            invalid = true;
        }
        if (config->clear_alarm > RELAY_ALARM_CLEAR_YES) {
            config->clear_alarm = RELAY_ALARM_CLEAR_NO;
            changed = true;
            invalid = true;
        } else if (config->clear_alarm == RELAY_ALARM_CLEAR_YES) {
            /* 清报警是合法运行命令，但不能跨重启保留。 */
            config->clear_alarm = RELAY_ALARM_CLEAR_NO;
            changed = true;
        }

        if (invalid) {
            if (config->operating_mode != RELAY_ALARM_OPERATING_DISABLED) {
                config->operating_mode = RELAY_ALARM_OPERATING_DISABLED;
                changed = true;
            }
            invalid_mask |= channel_bit;
        }
        if (changed) {
            changed_mask |= channel_bit;
        }
    }

    if (invalid_channel_mask != NULL) {
        *invalid_channel_mask = invalid_mask;
    }
    return changed_mask;
}
