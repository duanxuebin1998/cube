/*
 * multiparam_v4_measurement.c
 * 多参数V4主动与交互测量结果的运行态发布和业务读取接口。
 */

#include "multiparam_v4_measurement.h"

#include "main.h"
#include "multiparam_v4_communication.h"
#include "system_parameter.h"

#include <math.h>

#define MULTIPARAM_V4_FREQUENCY_MAX_HZ       6600
#define MULTIPARAM_V4_TEMPERATURE_MIN_C       (-200.0f)
#define MULTIPARAM_V4_TEMPERATURE_MAX_C       300.0f
#define MULTIPARAM_V4_DENSITY_MIN_KG_M3       0.0f
#define MULTIPARAM_V4_DENSITY_MAX_KG_M3       3000.0f
#define MULTIPARAM_V4_CAPACITANCE_MAX_PF      100000.0f
#define MULTIPARAM_V4_ANGLE_ABS_MAX_DEG       360.0f
#define MULTIPARAM_V4_STATUS_TEMPERATURE_NEW  (1UL << 8U)
#define MULTIPARAM_V4_STATUS_DENSITY_NEW      (1UL << 9U)
#define MULTIPARAM_V4_STATUS_GYRO_ABNORMAL    (1UL << 16U)
#define MULTIPARAM_V4_STATUS_WATER_ABNORMAL   (1UL << 17U)
#define MULTIPARAM_V4_INTERACTIVE_DEBUG_REFRESH_INTERVAL_MS 500U
#define MULTIPARAM_V4_PARAM_MAGNETIC_ZERO_VOLTAGE            3U
#define MULTIPARAM_V4_PARAM_DYNAMIC_VISCOSITY                 8U
#define MULTIPARAM_V4_PARAM_KINEMATIC_VISCOSITY               9U
#define MULTIPARAM_V4_PARAM_SUPPLY_VOLTAGE                    10U

static uint32_t s_last_published_generation = 0U;
static uint32_t s_last_interactive_debug_refresh_tick = 0U;
static uint8_t s_interactive_debug_refresh_valid = 0U;

void MULTIPARAM_V4_MeasurementInit(void)
{
    s_last_published_generation = 0U;
    g_measurement.debug_data.magnetic_zero_voltage = 0.0f;
    g_measurement.debug_data.dynamic_viscosity_cp = 0.0f;
    g_measurement.debug_data.kinematic_viscosity_cst = 0.0f;
    g_measurement.debug_data.supply_voltage_v = 0.0f;
    s_last_interactive_debug_refresh_tick = 0U;
    s_interactive_debug_refresh_valid = 0U;
}

/*
 * 函数用途：在交互通信方式下读取R08、R09和R10并刷新调试快照。
 * 调用场景：任一多参数V4交互测量入口成功取得核心业务数据后调用。
 * 关键约束：各字段独立更新并使用单次短事务；附加调试读取失败不影响已成功的核心业务读取。
 */
uint32_t MULTIPARAM_V4_MeasurementRefreshInteractiveDebugData(void)
{
    static const uint8_t parameters[] = {
        MULTIPARAM_V4_PARAM_DYNAMIC_VISCOSITY,
        MULTIPARAM_V4_PARAM_KINEMATIC_VISCOSITY,
        MULTIPARAM_V4_PARAM_SUPPLY_VOLTAGE
    };
    float value;
    uint32_t first_error = NO_ERROR;
    uint32_t now;
    uint32_t primask;
    uint32_t result;
    uint8_t updated = 0U;

    if (g_deviceParams.sensorType != MULTIPARAM_V4_SENSOR) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() != MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    now = HAL_GetTick();
    if ((s_interactive_debug_refresh_valid != 0U) &&
        ((now - s_last_interactive_debug_refresh_tick) <
         MULTIPARAM_V4_INTERACTIVE_DEBUG_REFRESH_INTERVAL_MS)) {
        return NO_ERROR;
    }

    for (uint32_t index = 0U; index < (sizeof(parameters) / sizeof(parameters[0])); index++) {
        result = MULTIPARAM_V4_ReadFloatParamOnce(parameters[index], &value);
        if (result == STATE_SWITCH) {
            s_last_interactive_debug_refresh_tick = HAL_GetTick();
            s_interactive_debug_refresh_valid = 1U;
            return STATE_SWITCH;
        }
        if (result != NO_ERROR) {
            if (first_error == NO_ERROR) {
                first_error = result;
            }
            continue;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        if (parameters[index] == MULTIPARAM_V4_PARAM_DYNAMIC_VISCOSITY) {
            g_measurement.debug_data.dynamic_viscosity_cp = value;
        } else if (parameters[index] == MULTIPARAM_V4_PARAM_KINEMATIC_VISCOSITY) {
            g_measurement.debug_data.kinematic_viscosity_cst = value;
        } else {
            g_measurement.debug_data.supply_voltage_v = value;
        }
        if (primask == 0U) {
            __enable_irq();
        }
        updated = 1U;
    }

    s_last_interactive_debug_refresh_tick = HAL_GetTick();
    s_interactive_debug_refresh_valid = 1U;
    return (updated != 0U) ? NO_ERROR : first_error;
}


/*
 * 函数用途：把浮点水位电容安全换算为0.1pF运行态原始值。
 * 调用场景：主动快照发布到g_measurement.debug_data。
 */
static uint32_t MULTIPARAM_V4_CapacitanceToRaw(float capacitance_pf)
{
    if ((!isfinite(capacitance_pf)) ||
        (capacitance_pf < 0.0f) ||
        (capacitance_pf > MULTIPARAM_V4_CAPACITANCE_MAX_PF)) {
        return UINT32_MAX;
    }
    return (uint32_t)(capacitance_pf * 10.0f + 0.5f);
}

/*
 * 函数用途：复制一份完整且未超时的主动快照。
 * 调用场景：PendSV发布运行态以及各测量读取接口。
 * 关键约束：只读已发布快照，不驱动流解析或UART恢复。
 */
static uint32_t MULTIPARAM_V4_CopyFreshSnapshot(multiparam_v4_snapshot_t *snapshot)
{
    uint32_t result;

    if (g_deviceParams.sensorType != MULTIPARAM_V4_SENSOR) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return SENSOR_STREAM_NOT_ACTIVE;
    }
    result = MULTIPARAM_V4_CopyLatestSnapshot(snapshot);
    if (result != NO_ERROR) {
        return result;
    }
    if ((HAL_GetTick() - snapshot->received_tick) > MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) {
        return SENSOR_DATA_STALE;
    }
    return NO_ERROR;
}

/*
 * 函数用途：把PendSV刚发布的新主动快照直接刷新到CPU2运行数据。
 * 调用场景：统一PendSV_Handler确认本轮有新快照后。
 * 关键约束：所有换算先在局部完成，临界区内只提交一组32位结果，不打印、不等待。
 */
uint32_t MULTIPARAM_V4_MeasurementProcessDeferred(void)
{
    multiparam_v4_snapshot_t snapshot;
    uint32_t capacitance_raw;
    uint32_t temperature_raw = 0U;
    uint32_t primask;
    uint8_t frequency_valid;
    uint8_t temperature_valid;
    uint8_t magnetic_zero_valid;
    uint8_t capacitance_valid;
    uint8_t dynamic_viscosity_valid;
    uint8_t kinematic_viscosity_valid;
    uint8_t supply_voltage_valid;
    uint8_t angle_valid;
    uint32_t result = MULTIPARAM_V4_CopyFreshSnapshot(&snapshot);

    if (result != NO_ERROR) {
        return result;
    }
    if (snapshot.generation == s_last_published_generation) {
        return NO_ERROR;
    }

    frequency_valid = (uint8_t)(((snapshot.measurement_frequency_hz >= 0) &&
                                 (snapshot.measurement_frequency_hz <= MULTIPARAM_V4_FREQUENCY_MAX_HZ))
                                    ? 1U : 0U);
    temperature_valid = (uint8_t)((isfinite(snapshot.temperature_c) &&
                                   (snapshot.temperature_c >= MULTIPARAM_V4_TEMPERATURE_MIN_C) &&
                                   (snapshot.temperature_c <= MULTIPARAM_V4_TEMPERATURE_MAX_C))
                                      ? 1U : 0U);
    if (temperature_valid != 0U) {
        temperature_raw = TEMP_TO_RAW(snapshot.temperature_c);
    }
    magnetic_zero_valid = snapshot.magnetic_zero_valid;
    capacitance_raw = MULTIPARAM_V4_CapacitanceToRaw(snapshot.water_capacitance_pf);
    capacitance_valid = (uint8_t)(((snapshot.water_capacitance_valid != 0U) &&
                                   (capacitance_raw != UINT32_MAX) &&
                                   ((snapshot.status_word & MULTIPARAM_V4_STATUS_WATER_ABNORMAL) == 0U))
                                      ? 1U : 0U);
    dynamic_viscosity_valid = (uint8_t)(isfinite(snapshot.dynamic_viscosity_cp) ? 1U : 0U);
    kinematic_viscosity_valid = (uint8_t)(isfinite(snapshot.kinematic_viscosity_cst) ? 1U : 0U);
    supply_voltage_valid = (uint8_t)(isfinite(snapshot.supply_voltage_v) ? 1U : 0U);
    angle_valid = (uint8_t)((isfinite(snapshot.angle_x_deg) &&
                             isfinite(snapshot.angle_y_deg) &&
                             (fabsf(snapshot.angle_x_deg) <= MULTIPARAM_V4_ANGLE_ABS_MAX_DEG) &&
                             (fabsf(snapshot.angle_y_deg) <= MULTIPARAM_V4_ANGLE_ABS_MAX_DEG) &&
                             ((snapshot.status_word & MULTIPARAM_V4_STATUS_GYRO_ABNORMAL) == 0U))
                                ? 1U : 0U);

    /* 主动帧内各参数可能来自不同测量时点，只更新本帧中各自有效的字段。 */
    primask = __get_PRIMASK();
    __disable_irq();
    s_last_published_generation = snapshot.generation;
    if (frequency_valid != 0U) {
        g_measurement.debug_data.frequency = (uint32_t)snapshot.measurement_frequency_hz;
        if (snapshot.measurement_mode == MULTIPARAM_V4_MEASUREMENT_LEVEL) {
            g_measurement.oil_measurement.current_frequency =
                (uint32_t)snapshot.measurement_frequency_hz;
        }
    }
    if (temperature_valid != 0U) {
        g_measurement.debug_data.temperature = temperature_raw;
    }
    if (magnetic_zero_valid != 0U) {
        g_measurement.debug_data.magnetic_zero_voltage = snapshot.magnetic_zero_voltage;
    }
    if (dynamic_viscosity_valid != 0U) {
        g_measurement.debug_data.dynamic_viscosity_cp = snapshot.dynamic_viscosity_cp;
    }
    if (kinematic_viscosity_valid != 0U) {
        g_measurement.debug_data.kinematic_viscosity_cst = snapshot.kinematic_viscosity_cst;
    }
    if (supply_voltage_valid != 0U) {
        g_measurement.debug_data.supply_voltage_v = snapshot.supply_voltage_v;
    }
    if (capacitance_valid != 0U) {
        g_measurement.debug_data.water_capacitance_x10 = capacitance_raw;
        g_measurement.water_measurement.current_capacitance = snapshot.water_capacitance_pf;
    }
    if (angle_valid != 0U) {
        g_measurement.debug_data.angle_x = (int32_t)(snapshot.angle_x_deg * 100.0f);
        g_measurement.debug_data.angle_y = (int32_t)(snapshot.angle_y_deg * 100.0f);
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return NO_ERROR;
}

/*
 * 函数用途：完成核心交互读取后尝试刷新附加调试量。
 * 调用场景：R04、R05、R11或R12业务结果已校验并发布后。
 * 关键约束：仅命令切换需要打断当前业务；附加量通信失败不推翻核心读取结果。
 */
static uint32_t MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead(void)
{
    uint32_t result = MULTIPARAM_V4_MeasurementRefreshInteractiveDebugData();

    return (result == STATE_SWITCH) ? STATE_SWITCH : NO_ERROR;
}

/*
 * 函数用途：在交互通信方式下读取并发布密度模式的R04、R07和R06。
 * 调用场景：密度测量流程需要一次同步结果时。
 * 关键约束：先读R02确认密度模式，全部核心量通过范围检查后才更新输出和调试快照。
 */
static uint32_t MULTIPARAM_V4_ReadInteractiveDensity(float *frequency_hz,
                                                      float *density_kg_m3,
                                                      float *temperature_c)
{
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    int32_t frequency_value;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
    if (result != NO_ERROR) {
        return result;
    }
    if (mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) {
        return SENSOR_MODE_MISMATCH;
    }
    result = MULTIPARAM_V4_ReadIntParam(4U, &frequency_value);
    if (result == NO_ERROR) {
        result = MULTIPARAM_V4_ReadFloatParam(7U, density_kg_m3);
    }
    if (result == NO_ERROR) {
        result = MULTIPARAM_V4_ReadFloatParam(6U, temperature_c);
    }
    if (result != NO_ERROR) {
        return result;
    }
    if ((frequency_value < 0) || (frequency_value > MULTIPARAM_V4_FREQUENCY_MAX_HZ) ||
        (!isfinite(*density_kg_m3)) || (*density_kg_m3 < MULTIPARAM_V4_DENSITY_MIN_KG_M3) ||
        (*density_kg_m3 > MULTIPARAM_V4_DENSITY_MAX_KG_M3) ||
        (!isfinite(*temperature_c)) || (*temperature_c < MULTIPARAM_V4_TEMPERATURE_MIN_C) ||
        (*temperature_c > MULTIPARAM_V4_TEMPERATURE_MAX_C)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *frequency_hz = (float)frequency_value;
    primask = __get_PRIMASK();
    __disable_irq();
    g_measurement.debug_data.frequency = (uint32_t)frequency_value;
    g_measurement.debug_data.temperature = TEMP_TO_RAW(*temperature_c);
    if (primask == 0U) {
        __enable_irq();
    }
    return MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead();
}

uint32_t MULTIPARAM_V4_MeasurementReadDensity(float *frequency_hz,
                                               float *density_kg_m3,
                                               float *temperature_c)
{
    multiparam_v4_snapshot_t snapshot;
    uint32_t result;

    if ((frequency_hz == NULL) || (density_kg_m3 == NULL) || (temperature_c == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return MULTIPARAM_V4_ReadInteractiveDensity(frequency_hz,
                                                    density_kg_m3,
                                                    temperature_c);
    }
    result = MULTIPARAM_V4_CopyFreshSnapshot(&snapshot);
    if (result != NO_ERROR) {
        return result;
    }
    if (snapshot.measurement_mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) {
        return SENSOR_MODE_MISMATCH;
    }
    if ((snapshot.status_word &
         (MULTIPARAM_V4_STATUS_TEMPERATURE_NEW | MULTIPARAM_V4_STATUS_DENSITY_NEW)) !=
        (MULTIPARAM_V4_STATUS_TEMPERATURE_NEW | MULTIPARAM_V4_STATUS_DENSITY_NEW)) {
        return SENSOR_DATA_STALE;
    }
    if (isnan(snapshot.density_kg_m3) || isnan(snapshot.temperature_c)) {
        return SENSOR_DATA_STALE;
    }
    if ((snapshot.measurement_frequency_hz < 0) ||
        (snapshot.measurement_frequency_hz > MULTIPARAM_V4_FREQUENCY_MAX_HZ) ||
        (!isfinite(snapshot.density_kg_m3)) ||
        (snapshot.density_kg_m3 < MULTIPARAM_V4_DENSITY_MIN_KG_M3) ||
        (snapshot.density_kg_m3 > MULTIPARAM_V4_DENSITY_MAX_KG_M3) ||
        (!isfinite(snapshot.temperature_c)) ||
        (snapshot.temperature_c < MULTIPARAM_V4_TEMPERATURE_MIN_C) ||
        (snapshot.temperature_c > MULTIPARAM_V4_TEMPERATURE_MAX_C)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *frequency_hz = (float)snapshot.measurement_frequency_hz;
    *density_kg_m3 = snapshot.density_kg_m3;
    *temperature_c = snapshot.temperature_c;
    return NO_ERROR;
}

uint32_t MULTIPARAM_V4_MeasurementReadLevelFrequency(uint32_t *frequency_hz)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    int32_t frequency_value;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if (frequency_hz == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
        if (result != NO_ERROR) {
            return result;
        }
        if (mode != MULTIPARAM_V4_MEASUREMENT_LEVEL) {
            return SENSOR_MODE_MISMATCH;
        }
        result = MULTIPARAM_V4_ReadIntParam(4U, &frequency_value);
        if (result != NO_ERROR) {
            return result;
        }
        if ((frequency_value < 0) || (frequency_value > MULTIPARAM_V4_FREQUENCY_MAX_HZ)) {
            return SONIC_FREQ_ABNORMAL;
        }
        *frequency_hz = (uint32_t)frequency_value;
        primask = __get_PRIMASK();
        __disable_irq();
        g_measurement.debug_data.frequency = (uint32_t)frequency_value;
        g_measurement.oil_measurement.current_frequency = (uint32_t)frequency_value;
        if (primask == 0U) {
            __enable_irq();
        }
        return MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead();
    }
    result = MULTIPARAM_V4_CopyFreshSnapshot(&snapshot);
    if (result != NO_ERROR) {
        return result;
    }
    if (snapshot.measurement_mode != MULTIPARAM_V4_MEASUREMENT_LEVEL) {
        return SENSOR_MODE_MISMATCH;
    }
    if ((snapshot.measurement_frequency_hz < 0) ||
        (snapshot.measurement_frequency_hz > MULTIPARAM_V4_FREQUENCY_MAX_HZ)) {
        return SONIC_FREQ_ABNORMAL;
    }
    *frequency_hz = (uint32_t)snapshot.measurement_frequency_hz;
    return NO_ERROR;
}

uint32_t MULTIPARAM_V4_MeasurementReadWaterCapacitance(float *capacitance_pf)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t capacitance_raw;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if (capacitance_pf == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
        if (result != NO_ERROR) {
            return result;
        }
        if ((mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
            (feature != MULTIPARAM_V4_FEATURE_WATER)) {
            result = MULTIPARAM_V4_EnsureWaterEnabled(1U);
            if (result != NO_ERROR) {
                return result;
            }
            result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
            if (result != NO_ERROR) {
                return result;
            }
        }
        if ((mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
            (feature != MULTIPARAM_V4_FEATURE_WATER)) {
            return SENSOR_STREAM_STATE_ERROR;
        }
        result = MULTIPARAM_V4_ReadFloatParam(5U, capacitance_pf);
        if (result != NO_ERROR) {
            return result;
        }
        if (isnan(*capacitance_pf)) {
            return SENSOR_DATA_STALE;
        }
        capacitance_raw = MULTIPARAM_V4_CapacitanceToRaw(*capacitance_pf);
        if ((!isfinite(*capacitance_pf)) || (*capacitance_pf < 0.0f) ||
            (*capacitance_pf > MULTIPARAM_V4_CAPACITANCE_MAX_PF) ||
            ((status_word & MULTIPARAM_V4_STATUS_WATER_ABNORMAL) != 0U)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        primask = __get_PRIMASK();
        __disable_irq();
        g_measurement.debug_data.water_capacitance_x10 = capacitance_raw;
        g_measurement.water_measurement.current_capacitance = *capacitance_pf;
        if (primask == 0U) {
            __enable_irq();
        }
        return MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead();
    }
    result = MULTIPARAM_V4_CopyFreshSnapshot(&snapshot);
    if (result != NO_ERROR) {
        return result;
    }
    if ((snapshot.measurement_mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
        (snapshot.feature_state != MULTIPARAM_V4_FEATURE_WATER)) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    if (isnan(snapshot.water_capacitance_pf)) {
        return SENSOR_DATA_STALE;
    }
    if ((snapshot.water_capacitance_valid == 0U) ||
        (!isfinite(snapshot.water_capacitance_pf)) ||
        (snapshot.water_capacitance_pf < 0.0f) ||
        (snapshot.water_capacitance_pf > MULTIPARAM_V4_CAPACITANCE_MAX_PF) ||
        ((snapshot.status_word & MULTIPARAM_V4_STATUS_WATER_ABNORMAL) != 0U)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *capacitance_pf = snapshot.water_capacitance_pf;
    capacitance_raw = MULTIPARAM_V4_CapacitanceToRaw(*capacitance_pf);
    primask = __get_PRIMASK();
    __disable_irq();
    g_measurement.debug_data.water_capacitance_x10 = capacitance_raw;
    g_measurement.water_measurement.current_capacitance = *capacitance_pf;
    if (primask == 0U) {
        __enable_irq();
    }
    return NO_ERROR;
}

/*
 * 函数用途：读取V4零点霍尔对应的R03磁零点电压。
 * 调用场景：业务层已经请求零点霍尔值，交互方式可在本函数内自动准备M功能。
 * 关键约束：M仅在密度模式有效且与L互斥；读取成功后保持M开启。
 */
uint32_t MULTIPARAM_V4_MeasurementReadMagneticZeroVoltage(float *voltage_v)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if (voltage_v == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
        if (result != NO_ERROR) {
            return result;
        }
        if ((mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
            (feature != MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO)) {
            result = MULTIPARAM_V4_EnsureMagneticZeroEnabled(1U);
            if (result != NO_ERROR) {
                return result;
            }
            result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
            if (result != NO_ERROR) {
                return result;
            }
        }
        if ((mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
            (feature != MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO)) {
            return SENSOR_STREAM_STATE_ERROR;
        }
        result = MULTIPARAM_V4_ReadFloatParam(MULTIPARAM_V4_PARAM_MAGNETIC_ZERO_VOLTAGE,
                                              voltage_v);
        if (result != NO_ERROR) {
            return result;
        }
        if (isnan(*voltage_v)) {
            return SENSOR_DATA_STALE;
        }
        if (!isfinite(*voltage_v)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        primask = __get_PRIMASK();
        __disable_irq();
        g_measurement.debug_data.magnetic_zero_voltage = *voltage_v;
        if (primask == 0U) {
            __enable_irq();
        }
        return MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead();
    }

    result = MULTIPARAM_V4_CopyFreshSnapshot(&snapshot);
    if (result != NO_ERROR) {
        return result;
    }
    if ((snapshot.measurement_mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
        (snapshot.feature_state != MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO)) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    if (isnan(snapshot.magnetic_zero_voltage)) {
        return SENSOR_DATA_STALE;
    }
    if ((snapshot.magnetic_zero_valid == 0U) ||
        (!isfinite(snapshot.magnetic_zero_voltage))) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *voltage_v = snapshot.magnetic_zero_voltage;
    primask = __get_PRIMASK();
    __disable_irq();
    g_measurement.debug_data.magnetic_zero_voltage = *voltage_v;
    if (primask == 0U) {
        __enable_irq();
    }
    return NO_ERROR;
}

uint32_t MULTIPARAM_V4_MeasurementReadGyro(float *angle_x_deg, float *angle_y_deg)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if ((angle_x_deg == NULL) || (angle_y_deg == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
        if (result != NO_ERROR) {
            return result;
        }
        result = MULTIPARAM_V4_ReadFloatParam(11U, angle_x_deg);
        if (result == NO_ERROR) {
            result = MULTIPARAM_V4_ReadFloatParam(12U, angle_y_deg);
        }
        if (result != NO_ERROR) {
            return result;
        }
        if ((!isfinite(*angle_x_deg)) || (!isfinite(*angle_y_deg)) ||
            (fabsf(*angle_x_deg) > MULTIPARAM_V4_ANGLE_ABS_MAX_DEG) ||
            (fabsf(*angle_y_deg) > MULTIPARAM_V4_ANGLE_ABS_MAX_DEG) ||
            ((status_word & MULTIPARAM_V4_STATUS_GYRO_ABNORMAL) != 0U)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        primask = __get_PRIMASK();
        __disable_irq();
        g_measurement.debug_data.angle_x = (int32_t)(*angle_x_deg * 100.0f);
        g_measurement.debug_data.angle_y = (int32_t)(*angle_y_deg * 100.0f);
        if (primask == 0U) {
            __enable_irq();
        }
        return MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead();
    }
    result = MULTIPARAM_V4_CopyFreshSnapshot(&snapshot);
    if (result != NO_ERROR) {
        return result;
    }
    if (isnan(snapshot.angle_x_deg) || isnan(snapshot.angle_y_deg)) {
        return SENSOR_DATA_STALE;
    }
    if ((!isfinite(snapshot.angle_x_deg)) || (!isfinite(snapshot.angle_y_deg)) ||
        (fabsf(snapshot.angle_x_deg) > MULTIPARAM_V4_ANGLE_ABS_MAX_DEG) ||
        (fabsf(snapshot.angle_y_deg) > MULTIPARAM_V4_ANGLE_ABS_MAX_DEG) ||
        ((snapshot.status_word & MULTIPARAM_V4_STATUS_GYRO_ABNORMAL) != 0U)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *angle_x_deg = snapshot.angle_x_deg;
    *angle_y_deg = snapshot.angle_y_deg;
    return NO_ERROR;
}
