#include "relay_output.h"

#include "main.h"
#include <stddef.h>
#include <string.h>

#define RELAY_OUTPUT_ACTIVE_LEVEL   GPIO_PIN_SET
#define RELAY_OUTPUT_INACTIVE_LEVEL GPIO_PIN_RESET

typedef enum {
    RELAY_COMPARE_GREATER = 0U,
    RELAY_COMPARE_LESS = 1U
} RelayCompareType;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} RelayOutputIo;

typedef struct {
    uint32_t manual_alarm_inhibit;
    uint32_t oil_level;
    uint32_t probe_at_liquid_level;
    uint32_t liquid_stable;
    int32_t temperature;
    uint32_t water_level;
    int32_t sensor_position;
} RelayOutputMeasurementSnapshot;

static const RelayOutputIo relay_output_ios[RELAY_OUTPUT_COUNT] = {
    { RELAY1_GPIO_Port, RELAY1_Pin },
    { RELAY2_GPIO_Port, RELAY2_Pin },
    { RELAY3_GPIO_Port, RELAY3_Pin },
    { RELAY4_GPIO_Port, RELAY4_Pin },
};

static volatile uint8_t relay_output_initialized = 0U;
static volatile uint8_t relay_output_updating = 0U;
static volatile uint8_t relay_output_update_pending = 0U;
static volatile uint8_t relay_output_state_mask = 0U;

static float RelayOutput_RawToFloat(uint32_t raw)
{
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint8_t RelayOutput_Compare(float current,
                                   float target,
                                   float hysteresis,
                                   RelayCompareType type)
{
    int32_t current_tenths;
    int32_t target_tenths;
    int32_t hysteresis_tenths;

    current_tenths = (int32_t)((current * 10.0f) + 0.5f);
    target_tenths = (int32_t)((target * 10.0f) + 0.5f);
    hysteresis_tenths = (int32_t)((hysteresis * 10.0f) + 0.5f);

    if (type == RELAY_COMPARE_GREATER) {
        return (current_tenths >= (target_tenths - hysteresis_tenths)) ? 1U : 0U;
    }

    return (current_tenths <= (target_tenths + hysteresis_tenths)) ? 1U : 0U;
}

static uint32_t RelayOutput_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void RelayOutput_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

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

static void RelayOutput_CopyMeasurementSnapshot(RelayOutputMeasurementSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return;
    }

    primask = RelayOutput_EnterCritical();
    snapshot->manual_alarm_inhibit = g_measurement.device_status.manual_alarm_inhibit;
    snapshot->oil_level = g_measurement.oil_measurement.oil_level;
    snapshot->probe_at_liquid_level = g_measurement.oil_measurement.probe_at_liquid_level;
    snapshot->liquid_stable = g_measurement.oil_measurement.liquid_stable;
    snapshot->temperature = (int32_t)g_measurement.debug_data.temperature;
    snapshot->water_level = g_measurement.water_measurement.water_level;
    snapshot->sensor_position = g_measurement.debug_data.sensor_position;
    RelayOutput_ExitCritical(primask);
}

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
        if ((snapshot->temperature == (int32_t)UNVALID_TEMPERATURE_REALTIME) ||
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

static uint8_t RelayOutput_UpdateChannel(uint32_t channel,
                                         const RelayOutputMeasurementSnapshot *measurement)
{
    RelayAlarmConfig cfg;
    RelayAlarmRuntimeState state;
    float alarm_value = 0.0f;
    uint32_t digital_state;
    uint8_t coil_active;

    RelayOutput_CopyConfigSnapshot(channel, &cfg);
    RelayOutput_CopyRuntimeSnapshot(channel, &state);
    RelayOutput_ConsumeClearCommand(channel, &cfg, &state);

    if ((measurement != NULL) && (measurement->manual_alarm_inhibit != 0U)) {
        RelayOutput_CommitRuntimeState(channel, &state);
        return 0U;
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
    coil_active = (digital_state == RELAY_ALARM_STATE_ACTIVE) ? 1U : 0U;

    if (cfg.contact_type == RELAY_ALARM_CONTACT_NORMALLY_CLOSED) {
        coil_active = (coil_active == 0U) ? 1U : 0U;
    }

    RelayOutput_CommitRuntimeState(channel, &state);
    return coil_active;
}

static uint8_t RelayOutput_BuildStateMask(void)
{
    RelayOutputMeasurementSnapshot measurement;
    uint8_t mask = 0U;

    RelayOutput_CopyMeasurementSnapshot(&measurement);

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        if (RelayOutput_UpdateChannel(channel, &measurement) != 0U) {
            mask |= (uint8_t)(1U << channel);
        }
    }

    return mask;
}

void RelayOutput_Init(void)
{
    RelayAlarmConfig cfg;
    RelayAlarmRuntimeState state;

    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        RelayOutput_CopyConfigSnapshot(channel, &cfg);
        RelayOutput_ResetRuntimeState(&state, &cfg);
        RelayOutput_CommitRuntimeState(channel, &state);
    }
    RelayOutput_WriteMask(0U);
    relay_output_initialized = 1U;
    relay_output_update_pending = 1U;
}

void RelayOutput_RequestUpdate(void)
{
    relay_output_update_pending = 1U;
}

void RelayOutput_ProcessPending(void)
{
    if (relay_output_update_pending == 0U) {
        return;
    }

    relay_output_update_pending = 0U;
    RelayOutput_Update();
}

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

uint8_t RelayOutput_GetStateMask(void)
{
    return relay_output_state_mask;
}

const volatile RelayAlarmRuntimeState *RelayOutput_GetRuntimeState(uint32_t channel)
{
    if (channel >= RELAY_ALARM_CHANNEL_COUNT) {
        return NULL;
    }
    return &g_measurement.relay_alarm_runtime[channel];
}
