/*
 * 模块职责：把DM4-V4主动快照或交互寄存器值转换为CPU2统一测量量。
 * 调用边界：协议收发、CRC和流重组由V4通信层负责；本层只做新鲜度、数值有效性和运行态发布。
 * 中断约束：PendSV入口只复制已解析快照并更新内存，不发串口、不打印、不阻塞等待。
 */

#include "multiparam_v4_measurement.h"

#include "main.h"
#include "multiparam_v4_communication.h"
#include "multiparam_v4_internal.h" /* 频率解码与原始值转换接口，支撑R04/R07/R06无效值判定。 */
#include "system_parameter.h"

#include <math.h>
/* V4测量频率允许的最大值，单位Hz。 */
#define MULTIPARAM_V4_FREQUENCY_MAX_HZ       6600
/* V4温度业务有效范围下限，单位摄氏度。 */
#define MULTIPARAM_V4_TEMPERATURE_MIN_C       (-200.0f)
/* V4温度业务有效范围上限，单位摄氏度。 */
#define MULTIPARAM_V4_TEMPERATURE_MAX_C       300.0f
/* V4密度业务有效范围下限，单位kg/m3。 */
#define MULTIPARAM_V4_DENSITY_MIN_KG_M3       0.0f
/* V4密度业务有效范围上限，单位kg/m3。 */
#define MULTIPARAM_V4_DENSITY_MAX_KG_M3       3000.0f
/* V4水位电容业务有效范围上限，单位pF。 */
#define MULTIPARAM_V4_CAPACITANCE_MAX_PF      100000.0f
/* V4姿态角绝对值业务上限，单位度。 */
#define MULTIPARAM_V4_ANGLE_ABS_MAX_DEG       360.0f
/* R02状态字中温度数据已更新标志位。 */
#define MULTIPARAM_V4_STATUS_TEMPERATURE_NEW  (1UL << 8U)
/* R02状态字中密度数据已更新标志位。 */
#define MULTIPARAM_V4_STATUS_DENSITY_NEW      (1UL << 9U)
/* R02状态字：黏度数据新更新标志位。 */
#define MULTIPARAM_V4_STATUS_VISCOSITY_NEW    (1UL << 10U)
/* R02状态字中陀螺仪异常标志位。 */
#define MULTIPARAM_V4_STATUS_GYRO_ABNORMAL    (1UL << 16U)
/* R02状态字中水位电容异常标志位。 */
#define MULTIPARAM_V4_STATUS_WATER_ABNORMAL   (1UL << 17U)
/* R02状态字中电压检测正常标志位，极性与Bit16、Bit17相反。 */
#define MULTIPARAM_V4_STATUS_VOLTAGE_NORMAL   (1UL << 18U)
/* 交互方式附加调试量的最短刷新间隔，单位ms。 */
#define MULTIPARAM_V4_INTERACTIVE_DEBUG_REFRESH_INTERVAL_MS 500U
/* V4零点霍尔电压参数号R03。 */
#define MULTIPARAM_V4_PARAM_MAGNETIC_ZERO_VOLTAGE            3U
/* V4动力黏度参数号R08。 */
#define MULTIPARAM_V4_PARAM_DYNAMIC_VISCOSITY                 8U
/* V4运动黏度参数号R09。 */
#define MULTIPARAM_V4_PARAM_KINEMATIC_VISCOSITY               9U
/* V4供电电压参数号R10。 */
#define MULTIPARAM_V4_PARAM_SUPPLY_VOLTAGE                    10U

/* 最近已发布的主动快照代次，防止同一主动帧重复写入g_measurement。 */
static uint32_t s_last_published_generation = 0U;
/* 最近一次交互附加调试量刷新节拍，单位ms。 */
static uint32_t s_last_interactive_debug_refresh_tick = 0U;
/* 交互附加刷新节拍已经建立基线的标志。 */
static uint8_t s_interactive_debug_refresh_valid = 0U;

/*
 * 函数用途：重置V4测量发布代次和交互附加量刷新节流状态。
 * 调用场景：DM4识别完成并重新初始化V4通信上下文之后。
 * 关键约束：只清理CPU2适配层缓存，不启动UART6接收，也不改变传感器工作模式。
 */
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

    if (g_deviceParams.sensorType != DM4_SENSOR) {
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

    if (g_deviceParams.sensorType != DM4_SENSOR) {
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

    frequency_valid = (uint8_t)(((snapshot.measurement_frequency_valid != 0U) &&
                                 (snapshot.measurement_frequency_hz <= MULTIPARAM_V4_FREQUENCY_MAX_HZ))
                                    ? 1U : 0U);
    temperature_valid = (uint8_t)((((snapshot.status_word &
                                    MULTIPARAM_V4_STATUS_TEMPERATURE_NEW) != 0U) &&
                                   isfinite(snapshot.temperature_c) &&
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
    dynamic_viscosity_valid =
        (uint8_t)((((snapshot.status_word & MULTIPARAM_V4_STATUS_VISCOSITY_NEW) != 0U) &&
                   isfinite(snapshot.dynamic_viscosity_cp)) ? 1U : 0U);
    kinematic_viscosity_valid =
        (uint8_t)((((snapshot.status_word & MULTIPARAM_V4_STATUS_VISCOSITY_NEW) != 0U) &&
                   isfinite(snapshot.kinematic_viscosity_cst)) ? 1U : 0U);
    supply_voltage_valid =
        (uint8_t)((isfinite(snapshot.supply_voltage_v) &&
                   ((snapshot.status_word & MULTIPARAM_V4_STATUS_VOLTAGE_NORMAL) != 0U)) ? 1U : 0U);
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
 * 函数用途：识别V4密度模式中R04是否为无效频率值。
 * 调用场景：交互读取和主动快照统一判断R04数据有效性。
 * 关键约束：0、INT32_MIN及超过业务上限的频率均按无效值处理。
 */
static uint8_t MULTIPARAM_V4_IsDensityNotScanned(int32_t raw_frequency)
{
    uint32_t frequency_hz = 0U;
    uint8_t fast_sweep_completed = 0U;

    /* R04为0表示快扫尚未扫到结果。 */
    if (raw_frequency == 0) {
        return 1U;
    }
    /* 解码失败的组合值同样代表未上报有效频率，按未扫到交给上层等待。 */
    if (MULTIPARAM_V4_DecodeFrequency(raw_frequency,
                                      &frequency_hz,
                                      &fast_sweep_completed) == 0U) {
        return 1U;
    }
    /* 超过业务上限的频率视为无效，避免误报格式错误打断密度等待。 */
    return (uint8_t)(frequency_hz > MULTIPARAM_V4_FREQUENCY_MAX_HZ);
}

/*
 * 函数用途：在交互通信方式下读取并发布密度模式的R04、R07和R06。
 * 调用场景：密度测量流程需要一次同步结果时。
 * 关键约束：先读R02确认密度模式；R04/R07无效值按0归一后交给上层继续等待，R06温度无效置NAN。
 */
static uint32_t MULTIPARAM_V4_ReadInteractiveDensity(float *frequency_hz,
                                                      float *density_kg_m3,
                                                      float *temperature_c)
{
    multiparam_v4_operating_state_t operating_state;
    int32_t frequency_value;
    uint32_t normalized_frequency;
    uint8_t fast_sweep_completed;
    uint8_t density_not_scanned = 0U;
    uint32_t temperature_raw;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
    if (result != NO_ERROR) {
        return result;
    }
    if (operating_state.measurement_mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) {
        return SENSOR_MODE_MISMATCH;
    }
    result = MULTIPARAM_V4_ReadIntParam(4U, &frequency_value);
    if (result == NO_ERROR) {
        /* 先判定R04有效性：无效时跳过频率解码，后续统一按未扫到语义归零等待。 */
        if (MULTIPARAM_V4_IsDensityNotScanned(frequency_value) != 0U) {
            density_not_scanned = 1U;
            normalized_frequency = 0U;
            fast_sweep_completed = 0U;
        } else if (MULTIPARAM_V4_DecodeFrequency(frequency_value,
                                                 &normalized_frequency,
                                                 &fast_sweep_completed) == 0U) {
            density_not_scanned = 1U;
            normalized_frequency = 0U;
            fast_sweep_completed = 0U;
        }
    }
    if (result == NO_ERROR) {
        uint32_t density_raw = 0U;

        /* R07改读原始值：范围外的密度归0交给上层继续等待，不再提前返回格式错误。 */
        result = MULTIPARAM_V4_ReadParamRaw(7U, &density_raw);
        if (result == NO_ERROR) {
            *density_kg_m3 = MULTIPARAM_V4_RawToFloat(density_raw);
            if ((!isfinite(*density_kg_m3)) ||
                (*density_kg_m3 < MULTIPARAM_V4_DENSITY_MIN_KG_M3) ||
                (*density_kg_m3 > MULTIPARAM_V4_DENSITY_MAX_KG_M3)) {
                *density_kg_m3 = 0.0f;
            }
        }
        if (density_not_scanned != 0U) {
            *density_kg_m3 = 0.0f;
        }
    }
    if (result == NO_ERROR) {
        /* R06温度无效时置NAN：温度异常不中断密度等待，由上层按无效温度处理。 */
        result = MULTIPARAM_V4_ReadParamRaw(6U, &temperature_raw);
        if (result == NO_ERROR) {
            *temperature_c = MULTIPARAM_V4_RawToFloat(temperature_raw);
            if ((!isfinite(*temperature_c)) ||
                (*temperature_c < MULTIPARAM_V4_TEMPERATURE_MIN_C) ||
                (*temperature_c > MULTIPARAM_V4_TEMPERATURE_MAX_C)) {
                *temperature_c = NAN;
            }
        }
    }
    if (result != NO_ERROR) {
        return result;
    }
    /* R04无效时按0处理，R07无效值也按0交给上层继续等待。 */
    if (density_not_scanned != 0U) {
        normalized_frequency = 0U;
        fast_sweep_completed = 0U;
        *density_kg_m3 = 0.0f;
    } else {
        density_not_scanned = 0U;
    }

    *frequency_hz = (float)normalized_frequency;
    primask = __get_PRIMASK();
    __disable_irq();
    g_measurement.debug_data.frequency = normalized_frequency;
    /* 温度无效时调试快照记0，避免NAN转原始值写入垃圾数据。 */
    g_measurement.debug_data.temperature = isfinite(*temperature_c)
                                               ? TEMP_TO_RAW(*temperature_c)
                                               : 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return MULTIPARAM_V4_RefreshInteractiveDebugAfterCoreRead();
}

/*
 * 函数用途：按当前V4通信方式读取频率、密度和温度三项核心量。
 * 调用场景：DM4驱动的统一密度读取回调。
 * 关键约束：主动方式消费同一份新鲜快照；未扫到及无效值按0/NAN归一后返回，交给上层等待。
 */
uint32_t MULTIPARAM_V4_MeasurementReadDensity(float *frequency_hz,
                                               float *density_kg_m3,
                                               float *temperature_c)
{
    multiparam_v4_snapshot_t snapshot;
    uint8_t density_not_scanned;
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
    density_not_scanned =
        MULTIPARAM_V4_IsDensityNotScanned(snapshot.measurement_frequency_raw);
    /* 快照温度越界或非有限值时置NAN，温度异常不阻断密度结果发布。 */
    if ((!isfinite(snapshot.temperature_c)) ||
        (snapshot.temperature_c < MULTIPARAM_V4_TEMPERATURE_MIN_C) ||
        (snapshot.temperature_c > MULTIPARAM_V4_TEMPERATURE_MAX_C)) {
        snapshot.temperature_c = NAN;
    }
    /* 密度未扫到或范围外时归0，交给上层等待循环继续轮询。 */
    if (density_not_scanned != 0U) {
        snapshot.density_kg_m3 = 0.0f;
    } else if ((!isfinite(snapshot.density_kg_m3)) ||
               (snapshot.density_kg_m3 < MULTIPARAM_V4_DENSITY_MIN_KG_M3) ||
               (snapshot.density_kg_m3 > MULTIPARAM_V4_DENSITY_MAX_KG_M3)) {
        snapshot.density_kg_m3 = 0.0f;
    }
    /* 未扫到时频率与密度输出0，向调用方表达继续等待语义。 */
    *frequency_hz = (density_not_scanned != 0U)
                        ? 0.0f
                        : (float)snapshot.measurement_frequency_hz;
    *density_kg_m3 = (density_not_scanned != 0U) ? 0.0f : snapshot.density_kg_m3;
    *temperature_c = snapshot.temperature_c;
    return NO_ERROR;
}

/*
 * 函数用途：按当前V4通信方式读取液位模式频率。
 * 调用场景：液位搜索、跟随和模式状态判断。
 * 关键约束：只返回采样结果，不在本层切换液位模式或驱动电机执行恢复动作。
 */
uint32_t MULTIPARAM_V4_MeasurementReadLevelFrequency(uint32_t *frequency_hz)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_operating_state_t operating_state;
    int32_t frequency_value;
    uint32_t normalized_frequency;
    uint8_t fast_sweep_completed;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if (frequency_hz == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
        if (result != NO_ERROR) {
            return result;
        }
        if (operating_state.measurement_mode != MULTIPARAM_V4_MEASUREMENT_LEVEL) {
            return SENSOR_MODE_MISMATCH;
        }
        result = MULTIPARAM_V4_ReadIntParam(4U, &frequency_value);
        if (result != NO_ERROR) {
            return result;
        }
        if ((MULTIPARAM_V4_DecodeFrequency(frequency_value,
                                           &normalized_frequency,
                                           &fast_sweep_completed) == 0U) ||
            (normalized_frequency > MULTIPARAM_V4_FREQUENCY_MAX_HZ)) {
            return SONIC_FREQ_ABNORMAL;
        }
        *frequency_hz = normalized_frequency;
        primask = __get_PRIMASK();
        __disable_irq();
        g_measurement.debug_data.frequency = normalized_frequency;
        g_measurement.oil_measurement.current_frequency = normalized_frequency;
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
    if ((snapshot.measurement_frequency_valid == 0U) ||
        (snapshot.measurement_frequency_hz > MULTIPARAM_V4_FREQUENCY_MAX_HZ)) {
        return SONIC_FREQ_ABNORMAL;
    }
    *frequency_hz = (uint32_t)snapshot.measurement_frequency_hz;
    return NO_ERROR;
}

/*
 * 函数用途：按当前V4通信方式读取水位电容并校验浮点有效性。
 * 调用场景：SensorService已经确认能力并完成测水使能之后。
 * 关键约束：测水开关适用于两种模式且可与磁零点同时开启，本函数只确保Bit1有效。
 */
uint32_t MULTIPARAM_V4_MeasurementReadWaterCapacitance(float *capacitance_pf)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_operating_state_t operating_state;
    uint32_t capacitance_raw;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if (capacitance_pf == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
        if (result != NO_ERROR) {
            return result;
        }
        if (operating_state.water_enabled == 0U) {
            result = MULTIPARAM_V4_EnsureWaterEnabled(1U);
            if (result != NO_ERROR) {
                return result;
            }
            result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
            if (result != NO_ERROR) {
                return result;
            }
        }
        if (operating_state.water_enabled == 0U) {
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
    if (snapshot.water_enabled == 0U) {
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
 * 关键约束：磁零点开关适用于两种模式且可与测水同时开启；读取成功后保持M开启。
 */
uint32_t MULTIPARAM_V4_MeasurementReadMagneticZeroVoltage(float *voltage_v)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_operating_state_t operating_state;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if (voltage_v == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
        if (result != NO_ERROR) {
            return result;
        }
        if (operating_state.magnetic_zero_enabled == 0U) {
            result = MULTIPARAM_V4_EnsureMagneticZeroEnabled(1U);
            if (result != NO_ERROR) {
                return result;
            }
            result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
            if (result != NO_ERROR) {
                return result;
            }
        }
        if (operating_state.magnetic_zero_enabled == 0U) {
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
    if (snapshot.magnetic_zero_enabled == 0U) {
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

/*
 * 函数用途：按当前V4通信方式读取X、Y双轴姿态角。
 * 调用场景：罐高、回零和部件参数诊断通过DM4驱动调用。
 * 关键约束：任一角度非有限值或超出协议范围时整组返回格式异常，不发布半组新数据。
 */
uint32_t MULTIPARAM_V4_MeasurementReadGyro(float *angle_x_deg, float *angle_y_deg)
{
    multiparam_v4_snapshot_t snapshot;
    multiparam_v4_operating_state_t operating_state;
    uint32_t status_word;
    uint32_t primask;
    uint32_t result;

    if ((angle_x_deg == NULL) || (angle_y_deg == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
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
