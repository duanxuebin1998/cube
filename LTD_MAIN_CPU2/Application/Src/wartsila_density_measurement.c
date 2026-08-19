/*
 * wartsila_density_measurement.c
 * 瓦西莱密度梯度测量
 *  Created on: 2025年12月1日
 *      Author: Duan Xuebin
 */


#include "wartsila_density_measurement.h"
#include "measure_oilLevel.h"
#include "Protocols/MultiparamV3/multiparam_v3_communication.h"
#include "sensor_service.h"
#include "measure_density.h"
#include "abortable_delay.h"

#define WARTSILA_POINT_POSITION_TOLERANCE_MM 1.0f /* Wartsila 单点位置允许误差，单位 mm。 */
#define WARTSILA_POINT_POSITION_RETRY_MAX    1U /* Wartsila 密度测量参数：测点 位置 重试 最大值。 */

/**
 * @brief 将 mm 位置转换为 0.1mm 无符号结果，负位置按 0 上报，超范围按上限上报。
 *
 * @param pos_mm 位置值，单位 mm。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint32_t PositionMm_ToU01mmClamped(float pos_mm)
{
    if (pos_mm <= 0.0f) {
        return 0U;
    }
    if (pos_mm >= 429496729.0f) {
        return UINT32_MAX;
    }

    return (uint32_t)(pos_mm * 10.0f + 0.5f);
}


#define WARTSILA_AIR_DENSITY_THRESHOLD      100.0f /* Wartsila 密度测量参数：空气 密度 阈值。 */
#define WARTSILA_LEVEL_DOWN_SPEED_X100      50U /* Wartsila 密度测量参数：液位 下降 SPEED 放大 100 倍。 */
#define WARTSILA_LEVEL_DOWN_POLL_MS         80U /* Wartsila 密度测量参数：液位 下降 轮询 毫秒。 */
#define WARTSILA_LEVEL_DOWN_TIMEOUT_MS      (60U * 60U * 1000U) /* Wartsila 密度测量参数：液位 下降 超时 毫秒。 */
#define WARTSILA_DENSITY_SAMPLE_MS          200U /* Wartsila 密度测量参数：密度 SAMPLE 毫秒。 */
#define WARTSILA_DENSITY_MAX_WAIT_MS        (5U * 60U * 1000U) /* Wartsila 密度测量参数：密度 最大值 等待 毫秒。 */
#define WARTSILA_DENSITY_FREQ_EPS_HZ        1.0f /* Wartsila 密度测量参数：密度 频率 允许误差 Hz。 */
#define WARTSILA_DENSITY_VALUE_EPS          0.1f /* Wartsila 密度测量参数：密度 值 允许误差。 */
#define WARTSILA_DENSITY_TEMP_EPS_C         0.2f /* Wartsila 密度测量参数：密度 温度 允许误差 C。 */

typedef struct {
    /* Wartsila 剖面单点的原始测量、换算结果、实际位置和气相判定依据。 */
    DensityMeasurement measurement; /* 该测点保留的完整原始测量结构，供诊断和后续换算复核。 */
    float frequency_hz; /* 该测点换算后的传感器频率，单位为 Hz。 */
    float density_value; /* 该测点换算后的密度值，单位遵循当前密度算法输出。 */
    float temperature_c; /* 该测点换算后的温度，单位为 ℃。 */
    float actual_position_mm; /* 采样时由位置模型换算的实际位置，单位为 mm。 */
    bool is_air; /* 该测点被判定为气相的标志；非零/true 时不作为有效液相密度点。 */
    const char *air_reason; /* 气相判定原因的只读文本；非气相点可以为空。 */
} WartsilaPointSample;

/**
 * @brief 按密度和频率判定样本是否为空气点，并返回判定原因。
 *
 * @param density_value 密度数值。
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param air_reason 用于返回当前样本被判为空气点的原因文字。
 * @return true 表示密度低于空气阈值，或频率高于油位频率界限，并按优先级写入对应原因；false 表示两个空气判据均不成立，样本按液体点处理并写入 liquid 原因。
 */
static bool Wartsila_IsAirPoint(float density_value, float frequency_hz, const char **air_reason)
{
    if (density_value < WARTSILA_AIR_DENSITY_THRESHOLD) {
        if (air_reason != NULL) {
            *air_reason = "density_low";
        }
        return true;
    }

    if (frequency_hz > (float)g_deviceParams.oilLevelFrequency) {
        if (air_reason != NULL) {
            *air_reason = "frequency_high";
        }
        return true;
    }

    if (air_reason != NULL) {
        *air_reason = "liquid";
    }
    return false;
}

/**
 * @brief 把工程量和实际位置编码到单个瓦锡兰点阵样本。
 *
 * @param sample 待处理的单次测量样本。
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param density_value 密度数值。
 * @param temperature_c 温度值，单位 ℃。
 * @param actual_position_mm 实际位置。
 * @param is_air true 表示当前点为空气点，false 表示液体测点。
 * @param air_reason 用于返回当前样本被判为空气点的原因文字。
 */
static void Wartsila_FillPointSample(WartsilaPointSample *sample,
                                     float frequency_hz,
                                     float density_value,
                                     float temperature_c,
                                     float actual_position_mm,
                                     bool is_air,
                                     const char *air_reason)
{
    memset(sample, 0, sizeof(*sample));
    sample->frequency_hz = frequency_hz;
    sample->density_value = density_value;
    sample->temperature_c = temperature_c;
    sample->actual_position_mm = actual_position_mm;
    sample->is_air = is_air;
    sample->air_reason = air_reason;

    sample->measurement.density = DENSITY_TO_RAW(density_value);
    sample->measurement.standard_density = DENSITY_TO_RAW(density_value);
    sample->measurement.weight_density = DENSITY_TO_RAW(density_value);
    sample->measurement.temperature = TEMP_TO_RAW(temperature_c);
    sample->measurement.temperature_position = PositionMm_ToU01mmClamped(actual_position_mm);
    sample->measurement.vcf20 = 1U;
}

/**
 * @brief 移动到目标测点并复核实际位置，超出容差时有限重试。
 *
 * @param target_mm 目标位置，单位 mm。
 * @param point_no 本轮瓦锡兰密度分布测点的现场编号，用于位置偏差超限后的重试日志，不参与目标位置计算。
 * @param actual_position_mm 用于返回电机停止后确认的实际位置，单位 mm。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t Wartsila_MoveToDensityPoint(float target_mm,
                                            uint32_t point_no,
                                            float *actual_position_mm)
{
    uint32_t ret = NO_ERROR;
    float cur_mm = 0.0f;

    if (actual_position_mm == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    for (uint32_t attempt = 0U; attempt <= WARTSILA_POINT_POSITION_RETRY_MAX; attempt++) {
        if (attempt > 0U) {
            printf("瓦锡兰分布测量 第%lu个点偏差超限，重试按位置定位\r\n",
                   (unsigned long)point_no);
        }

        ret = MotorCtrl_JogMoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        CHECK_ERROR(ret);

        MotorCtrl_SnapshotSensorPositionMm(&cur_mm);
        float deviation_mm = cur_mm - target_mm;
        float abs_deviation_mm = (deviation_mm >= 0.0f) ? deviation_mm : -deviation_mm;

        printf("瓦锡兰分布测量 第%lu个点定位：目标=%.3fmm 实际=%.3fmm 偏差=%.3fmm\r\n",
               (unsigned long)point_no,
               target_mm,
               cur_mm,
               deviation_mm);

        if (abs_deviation_mm <= WARTSILA_POINT_POSITION_TOLERANCE_MM) {
            *actual_position_mm = cur_mm;
            return NO_ERROR;
        }
    }

    *actual_position_mm = cur_mm;
    printf("瓦锡兰分布测量 第%lu个点定位失败：目标=%.3fmm 实际=%.3fmm 偏差超出%.3fmm\r\n",
           (unsigned long)point_no,
           target_mm,
           cur_mm,
           WARTSILA_POINT_POSITION_TOLERANCE_MM);
    return POSITION_ARRIVAL_DEVIATION;
}

/**
 * @brief 循环读取密度组合值直至稳定或超时，并标记空气点。
 *
 * 函数先切换传感器到密度模式，并把 spreadPointHoverTime 秒换算为连续稳定窗口；配置为 0 时使用 5 秒兼容窗口。
 * 循环读取频率、密度和温度，频率非正时清除当前稳定基准并继续等待；满足空气点判定条件时立即连同当前位置和判定原因写入 sample。
 * 液体点要求频率、密度和温度相对参考值持续位于各自允许偏差内，任一量越界都会更新参考值并重新开始稳定计时；窗口满足后发布参考组合及当前机械位置。
 * 等待达到 WARTSILA_DENSITY_MAX_WAIT_MS 时，若从未得到有效频率或最新频率仍无效则返回 SONIC_FREQ_ABNORMAL；已有有效频率但始终未形成稳定液体点时按密度 0
 * 的空气点完成。
 * 新命令、模式切换、传感器读取或可中断延时错误均立即原样返回，不把未确认的中间样本写成成功点。
 *
 * @param sample 用于接收本测点频率、密度、温度、机械位置、空气标志和分类原因的可写对象；只在测点确认成功时可用。
 * @return NO_ERROR 表示已写入空气点、稳定液体点或超时降级空气点；STATE_SWITCH 表示被新命令中断，SONIC_FREQ_ABNORMAL
 *         表示总等待结束仍无有效频率，其他值为模式切换、传感器读取或可中断延时错误。
 * @note sample 只在确认空气点、稳定液体点或超时降级空气点时整体填充；错误返回时调用方不得使用其中未完成的数据。
 */
static uint32_t Wartsila_ReadPointAndClassify(WartsilaPointSample *sample)
{
    uint32_t ret = NO_ERROR;
    uint32_t hover_time_s = g_deviceParams.spreadPointHoverTime;
    uint32_t stable_win_ms = hover_time_s * 1000U;
    uint32_t start_tick = 0U;
    uint32_t stable_start = 0U;
    uint8_t first_sample = 1U;
    uint8_t has_valid_frequency = 0U;
    uint8_t current_frequency_invalid = 0U;
    float ref_freq = 0.0f;
    float ref_density = 0.0f;
    float ref_temp = 0.0f;
    float cur_freq = 0.0f;
    float cur_density = 0.0f;
    float cur_temp = 0.0f;
    float cur_mm = 0.0f;

    if (sample == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = SensorService_EnableDensityMode();
    if (ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    CHECK_ERROR(ret);

    if (stable_win_ms == 0U) {
        stable_win_ms = 5000U;
    }

    start_tick = HAL_GetTick();
    while (1) {
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }

        uint32_t now = HAL_GetTick();
        if ((now - start_tick) >= WARTSILA_DENSITY_MAX_WAIT_MS) {
            if ((has_valid_frequency == 0U) || (current_frequency_invalid != 0U)) {
                return SONIC_FREQ_ABNORMAL;
            }
            /* 密度读取超过 5 分钟仍未形成有效液体点时，按密度 0 的空气点处理。 */
            MotorCtrl_SnapshotSensorPositionMm(&cur_mm);
            Wartsila_FillPointSample(sample,
                                     cur_freq,
                                     0.0f,
                                     cur_temp,
                                     cur_mm,
                                     true,
                                     "density_timeout_zero");
            return NO_ERROR;
        }

        ret = SensorService_ReadDensity(&cur_freq, &cur_density, &cur_temp);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            return ret;
        }

        if (cur_freq <= 0.0f) {
            current_frequency_invalid = 1U;
            first_sample = 1U;
            ret = AbortableDelay_CommandSwitch(WARTSILA_DENSITY_SAMPLE_MS, 50U);
            if (ret != NO_ERROR) {
                return ret;
            }
            continue;
        }
        has_valid_frequency = 1U;
        current_frequency_invalid = 0U;

        MotorCtrl_SnapshotSensorPositionMm(&cur_mm);

        const char *air_reason = NULL;
        if (Wartsila_IsAirPoint(cur_density, cur_freq, &air_reason)) {
            Wartsila_FillPointSample(sample,
                                     cur_freq,
                                     cur_density,
                                     cur_temp,
                                     cur_mm,
                                     true,
                                     air_reason);
            return NO_ERROR;
        }


        if (first_sample != 0U) {
            ref_freq = cur_freq;
            ref_density = cur_density;
            ref_temp = cur_temp;
            stable_start = now;
            first_sample = 0U;
        } else {
            float df = fabsf(cur_freq - ref_freq);
            float dd = fabsf(cur_density - ref_density);
            float dt = fabsf(cur_temp - ref_temp);
            if ((df > WARTSILA_DENSITY_FREQ_EPS_HZ) ||
                (dd > WARTSILA_DENSITY_VALUE_EPS) ||
                (dt > WARTSILA_DENSITY_TEMP_EPS_C)) {
                ref_freq = cur_freq;
                ref_density = cur_density;
                ref_temp = cur_temp;
                stable_start = now;
            }
        }

        if ((first_sample == 0U) && ((now - stable_start) >= stable_win_ms)) {
            Wartsila_FillPointSample(sample,
                                     ref_freq,
                                     ref_density,
                                     ref_temp,
                                     cur_mm,
                                     false,
                                     "liquid");
            return NO_ERROR;
        }

        ret = AbortableDelay_CommandSwitch(WARTSILA_DENSITY_SAMPLE_MS, 50U);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
}

/**
 * @brief 从空气点持续下行搜索液体，执行命令切换、扭力、丢步和超时保护。
 *
 * @param air_point_mm 测点。
 * @param level_mm 用于返回重新进入液体时确认的液位位置，单位 mm。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t Wartsila_MoveDownToLiquidAfterAirPoint(float air_point_mm, float *level_mm)
{
    uint32_t ret = NO_ERROR;
    bool is_moving = false;
    float cur_mm = air_point_mm;
    uint32_t start_tick = 0U;

    if (level_mm == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = SensorService_EnableLevelMode();
    if (ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    CHECK_ERROR(ret);

    MotorCtrl_LostStepInit();
    ret = MotorCtrl_MoveDown(WARTSILA_LEVEL_DOWN_SPEED_X100);
    CHECK_ERROR(ret);

    start_tick = HAL_GetTick();
    while (1) {
        Level_StateTypeDef level_state = AIR;

        ret = determine_level_status_motion(&level_state);
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        if (level_state == OIL) {
            ret = MotorCtrl_SlowStop();
            CHECK_ERROR(ret);
            MotorCtrl_SnapshotSensorPositionMm(&cur_mm);
            *level_mm = cur_mm;
            printf("瓦锡兰分布测量 空气点后慢速下行识别到液体，液位位置=%.3fmm\r\n", cur_mm);
            return NO_ERROR;
        }

        ret = MotorCtrl_IsDriverMoving(&stepper, &is_moving);
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }
        if (!is_moving) {
            ret = MotorCtrl_MoveDown(WARTSILA_LEVEL_DOWN_SPEED_X100);
            CHECK_ERROR(ret);
        }

        MotorCtrl_SnapshotSensorPositionMm(&cur_mm);
        if (g_measurement.debug_data.sensor_position < (int32_t)g_deviceParams.blindZone) {
            (void)MotorCtrl_SlowStop();
            return MEASUREMENT_OILLEVEL_LOW;
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        ret = CheckWeightCollision();
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        ret = MotorCtrl_CheckDriverGstat();
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        /* 从气相点下行寻找液面超过总时限时先慢停，再按液位过低返回，防止电机持续向下运行。 */
        if ((HAL_GetTick() - start_tick) > WARTSILA_LEVEL_DOWN_TIMEOUT_MS) {
            (void)MotorCtrl_SlowStop();
            return MEASUREMENT_OILLEVEL_LOW;
        }

        ret = AbortableDelay_CommandSwitch(WARTSILA_LEVEL_DOWN_POLL_MS, 20U);
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }
    }
}

/**
 * @brief 按最终液位裁剪空气点和液面以上点，并重算有效点累计值。
 *
 * @param dist 密度分布测量结果对象。
 * @param valid_points 用于返回油位以下仍可执行的有效测点数量。
 * @param sum_temp 用于累计保留测点温度的输出变量。
 * @param sum_density 用于累计保留测点密度的输出变量。
 * @param level_mm 液位。
 * @param min_gap_surface 瓦锡兰测点距液面的最小允许间隔，单位 0.1 mm。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t Wartsila_TrimPointsByOilLevel(DensityDistribution *dist,
                                              uint32_t *valid_points,
                                              uint32_t *sum_temp,
                                              uint32_t *sum_density,
                                              float level_mm,
                                              uint32_t min_gap_surface)
{
    if ((dist == NULL) || (valid_points == NULL) || (sum_temp == NULL) || (sum_density == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    float valid_limit_mm = level_mm - (float)min_gap_surface;
    while (*valid_points > 0U) {
        DensityMeasurement *last_pt = &dist->single_density_data[*valid_points - 1U];
        float point_pos_mm = (float)last_pt->temperature_position / 10.0f;
        if (point_pos_mm >= valid_limit_mm) {
            *sum_temp -= last_pt->temperature;
            *sum_density -= last_pt->density;
            memset(last_pt, 0, sizeof(*last_pt));
            (*valid_points)--;
            printf("瓦锡兰分布测量 高位点位置=%.3fmm 不小于有效边界=%.3fmm，删除后剩余=%lu\r\n",
                   point_pos_mm,
                   valid_limit_mm,
                   (unsigned long)*valid_points);
            continue;
        }

        break;
    }

    if (*valid_points == 0U) {
        return MEASUREMENT_DENSITY_NO_VALID_POINT;
    }

    return NO_ERROR;
}

/**
 * @brief  Wartsila 密度分布测量（点间只按位置移动，到点后用密度/频率判定空气）
 *
 * 点间移动和起始点定位不做运动中液位检测；到点后使用密度模式读取实际密度值和浮点频率。
 * 空气点不保存，切液位模式慢速下行，首次识别到 OIL 时把当前位置作为液位。
 * 最终只保留位置严格小于“液位 - 距离限制”的液体点。
 *
 * @param  dist  输出测量结果的结构体指针（一般为 &g_measurement.density_distribution）
 * @return 错误码，NO_ERROR 表示成功
 */
uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist)
{
    if (dist == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    memset(dist, 0, sizeof(DensityDistribution));

    uint32_t start_pos_mm    = g_deviceParams.wartsila_lower_density_limit;
    uint32_t end_pos_mm      = g_deviceParams.wartsila_upper_density_limit;
    uint32_t step_mm         = g_deviceParams.wartsila_density_interval;
    uint32_t min_gap_surface = g_deviceParams.wartsila_max_height_above_surface;

    if (step_mm == 0U) {
        printf("瓦锡兰分布测量无法形成测点：测量间距为0\r\n");
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }
    if (end_pos_mm <= start_pos_mm) {
        printf("瓦锡兰分布测量参数异常：结束位置<=起始位置 (%lu <= %lu)\r\n",
               (unsigned long)end_pos_mm,
               (unsigned long)start_pos_mm);
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    float range_mm = (float)(end_pos_mm - start_pos_mm);
    uint32_t max_points_by_range = (uint32_t)(range_mm / (float)step_mm) + 1U;
    if (max_points_by_range == 0U) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }
    if (max_points_by_range > MAX_MEASUREMENT_POINTS) {
        max_points_by_range = MAX_MEASUREMENT_POINTS;
    }

    printf("瓦锡兰分布测量开始：起始=%lumm, 最高=%lumm, 间距=%lumm, 理论点数=%lu\r\n",
           (unsigned long)start_pos_mm,
           (unsigned long)end_pos_mm,
           (unsigned long)step_mm,
           (unsigned long)max_points_by_range);

    uint32_t ret = NO_ERROR;
    uint32_t valid_points = 0U;
    uint32_t sum_temp = 0U;
    uint32_t sum_density = 0U;
    bool oil_level_found = false;
    float oil_level_mm = -1.0f;
    float target_mm = (float)start_pos_mm;

    for (uint32_t i = 0U; i < max_points_by_range; i++) {
        if (i > 0U) {
            target_mm += (float)step_mm;
            if (target_mm > (float)end_pos_mm + 0.01f) {
                break;
            }
        }

        float cur_mm = 0.0f;
        ret = Wartsila_MoveToDensityPoint(target_mm, i + 1U, &cur_mm);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        CHECK_ERROR(ret);

        WartsilaPointSample sample;
        ret = Wartsila_ReadPointAndClassify(&sample);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        CHECK_ERROR(ret);

        cur_mm = sample.actual_position_mm;
        if (sample.is_air) {
            printf("瓦锡兰分布测量 第%lu个点判定为空气：目标=%.3fmm 实际=%.3fmm 密度=%.3f 频率=%.3f 原因=%s\r\n",
                   (unsigned long)(i + 1U),
                   target_mm,
                   cur_mm,
                   sample.density_value,
                   sample.frequency_hz,
                   sample.air_reason);

            ret = Wartsila_MoveDownToLiquidAfterAirPoint(cur_mm, &oil_level_mm);
            if (ret == STATE_SWITCH) {
                return STATE_SWITCH;
            }
            CHECK_ERROR(ret);
            oil_level_found = true;
            break;
        }

        if (valid_points >= MAX_MEASUREMENT_POINTS) {
            break;
        }

        dist->single_density_data[valid_points] = sample.measurement;
        sum_temp += sample.measurement.temperature;
        sum_density += sample.measurement.density;
        valid_points++;

        printf("瓦锡兰分布测量 液体点%lu：位置=%.3fmm 密度=%lu(%.3f) 温度=%lu(%.2f) 频率=%.3f\r\n",
               (unsigned long)valid_points,
               cur_mm,
               (unsigned long)sample.measurement.density,
               sample.density_value,
               (unsigned long)sample.measurement.temperature,
               sample.temperature_c,
               sample.frequency_hz);
    }

    if (!oil_level_found) {
        printf("瓦锡兰分布测量执行到最高点仍未遇到空气，无法识别液位\r\n");
        return MEASUREMENT_DENSITY_SURFACE_NOTFOUND;
    }

    if (valid_points == 0U) {
        printf("瓦锡兰分布测量已识别液面，但没有得到有效液体测点\r\n");
        return MEASUREMENT_DENSITY_NO_VALID_POINT;
    }

    ret = Wartsila_TrimPointsByOilLevel(dist,
                                        &valid_points,
                                        &sum_temp,
                                        &sum_density,
                                        oil_level_mm,
                                        min_gap_surface);
    CHECK_ERROR(ret);

    dist->measurement_points       = valid_points;
    dist->Density_oil_level        = PositionMm_ToU01mmClamped(oil_level_mm);
    dist->average_temperature      = (sum_temp    + valid_points / 2U) / valid_points;
    dist->average_density          = (sum_density + valid_points / 2U) / valid_points;
    dist->average_standard_density = dist->average_density;
    dist->average_weight_density   = dist->average_density;
    dist->average_vcf20            = 0U;

    printf("瓦锡兰分布测量完成：有效点数=%lu, 液位=%.1fmm, 平均密度=%lu(%.3f), 平均温度=%lu(%.2f)\r\n",
           (unsigned long)valid_points,
           oil_level_mm,
           (unsigned long)dist->average_density,
           RAW_TO_DENSITY(dist->average_density),
           (unsigned long)dist->average_temperature,
           RAW_TO_TEMP(dist->average_temperature));

    return NO_ERROR;
}
