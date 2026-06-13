/*
 * wartsila_density_measurement.c
 * 瓦西莱密度梯度测量
 *  Created on: 2025年12月1日
 *      Author: Duan Xuebin
 */


#include "wartsila_density_measurement.h"
#include "measure_oilLevel.h"
#include "ltd_sensor_communication.h"
#include "sensor.h"
#include "measure_density.h"
#include "abortable_delay.h"

uint32_t motorMoveUpToPositionOrAir(float target_mm, Level_StateTypeDef *final_state);

#define WARTSILA_POINT_POSITION_TOLERANCE_MM 1.0f
#define WARTSILA_POINT_POSITION_RETRY_MAX    1U

/**
 * @brief 将 mm 位置转换为 0.1mm 无符号结果，负位置按 0 上报，超范围按上限上报。
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


#define WARTSILA_AIR_DENSITY_THRESHOLD      100.0f
#define WARTSILA_LEVEL_DOWN_SPEED_X100      50U
#define WARTSILA_LEVEL_DOWN_POLL_MS         80U
#define WARTSILA_LEVEL_DOWN_TIMEOUT_MS      (60U * 60U * 1000U)
#define WARTSILA_DENSITY_SAMPLE_MS          200U
#define WARTSILA_DENSITY_MAX_WAIT_MS        (5U * 60U * 1000U)
#define WARTSILA_DENSITY_FREQ_EPS_HZ        1.0f
#define WARTSILA_DENSITY_VALUE_EPS          0.1f
#define WARTSILA_DENSITY_TEMP_EPS_C         0.2f

typedef struct {
    DensityMeasurement measurement;
    float frequency_hz;
    float density_value;
    float temperature_c;
    float actual_position_mm;
    bool is_air;
    const char *air_reason;
} WartsilaPointSample;

/**
 * @brief 执行测量流程中的 Wartsila_IsAirPoint 逻辑。
 *
 * @param density_value 待处理数值。
 * @param frequency_hz 业务参数。
 * @param air_reason 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
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
 * @brief 执行测量流程中的 Wartsila_FillPointSample 逻辑。
 *
 * @param sample 业务参数。
 * @param frequency_hz 业务参数。
 * @param density_value 待处理数值。
 * @param temperature_c 业务参数。
 * @param actual_position_mm 业务参数。
 * @param is_air 业务参数。
 * @param air_reason 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
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
 * @brief 执行测量流程中的 Wartsila_MoveToDensityPoint 逻辑。
 *
 * @param target_mm 业务参数。
 * @param point_no 输入/输出指针。
 * @param actual_position_mm 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Wartsila_MoveToDensityPoint(float target_mm,
                                            uint32_t point_no,
                                            float *actual_position_mm)
{
    uint32_t ret = NO_ERROR;
    float cur_mm = 0.0f;

    if (actual_position_mm == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    for (uint32_t attempt = 0U; attempt <= WARTSILA_POINT_POSITION_RETRY_MAX; attempt++) {
        if (attempt > 0U) {
            printf("瓦锡兰分布测量 第%lu个点偏差超限，重试按位置定位\r\n",
                   (unsigned long)point_no);
        }

        ret = MotorCtrl_MoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());
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
    return MEASUREMENT_POSITION_ERROR;
}

/**
 * @brief 读取测量流程中的 Wartsila_ReadPointAndClassify 逻辑。
 *
 * @param sample 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
        return PARAM_ADDRESS_OVERFLOW;
    }

    ret = EnableDensityMode();
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

        ret = Read_Density(&cur_freq, &cur_density, &cur_temp);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
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
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
    }
}

/**
 * @brief 执行测量流程中的 Wartsila_MoveDownToLiquidAfterAirPoint 逻辑。
 *
 * @param air_point_mm 业务参数。
 * @param level_mm 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Wartsila_MoveDownToLiquidAfterAirPoint(float air_point_mm, float *level_mm)
{
    uint32_t ret = NO_ERROR;
    bool is_moving = false;
    float cur_mm = air_point_mm;
    uint32_t start_tick = 0U;

    if (level_mm == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    ret = EnableLevelMode();
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
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
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
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
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
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        ret = CheckWeightCollision();
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        ret = MotorCtrl_CheckDriverGstat();
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }

        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if ((HAL_GetTick() - start_tick) > WARTSILA_LEVEL_DOWN_TIMEOUT_MS) {
            (void)MotorCtrl_SlowStop();
            return MOTOR_RUN_TIMEOUT;
        }

        ret = AbortableDelay_CommandSwitch(WARTSILA_LEVEL_DOWN_POLL_MS, 20U);
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }
    }
}

/**
 * @brief 执行测量流程中的 Wartsila_TrimPointsByOilLevel 逻辑。
 *
 * @param dist 业务参数。
 * @param valid_points 待处理数值。
 * @param sum_temp 业务参数。
 * @param sum_density 业务参数。
 * @param level_mm 业务参数。
 * @param min_gap_surface 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Wartsila_TrimPointsByOilLevel(DensityDistribution *dist,
                                              uint32_t *valid_points,
                                              uint32_t *sum_temp,
                                              uint32_t *sum_density,
                                              float level_mm,
                                              uint32_t min_gap_surface)
{
    if ((dist == NULL) || (valid_points == NULL) || (sum_temp == NULL) || (sum_density == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
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
        return PARAM_ADDRESS_OVERFLOW;
    }

    memset(dist, 0, sizeof(DensityDistribution));

    uint32_t start_pos_mm    = g_deviceParams.wartsila_lower_density_limit;
    uint32_t end_pos_mm      = g_deviceParams.wartsila_upper_density_limit;
    uint32_t step_mm         = g_deviceParams.wartsila_density_interval;
    uint32_t min_gap_surface = g_deviceParams.wartsila_max_height_above_surface;

    if (step_mm == 0U) {
        printf("瓦锡兰分布测量参数异常：步长=0\r\n");
        return PARAM_RANGE_ERROR;
    }
    if (end_pos_mm <= start_pos_mm) {
        printf("瓦锡兰分布测量参数异常：结束位置<=起始位置 (%lu <= %lu)\r\n",
               (unsigned long)end_pos_mm,
               (unsigned long)start_pos_mm);
        return PARAM_RANGE_ERROR;
    }

    float range_mm = (float)(end_pos_mm - start_pos_mm);
    uint32_t max_points_by_range = (uint32_t)(range_mm / (float)step_mm) + 1U;
    if (max_points_by_range == 0U) {
        return PARAM_RANGE_ERROR;
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
        printf("瓦锡兰分布测量没有得到任何有效液体测点\r\n");
        return MEASUREMENT_DENSITY_SURFACE_NOTFOUND;
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

/**
 * @brief 电机向上运行到指定目标位置，途中若检测到传感器进入空气立即停止
 *
 * @param target_mm      目标绝对位置（单位：mm）
 * @param final_state    [可选] 最终状态输出（AIR / OIL），可为 NULL
 *
 * @return NO_ERROR 表示正常结束（到达目标或遇到空气）
 *         其他错误码表示电机或硬件异常
 */
uint32_t motorMoveUpToPositionOrAir(float target_mm, Level_StateTypeDef *final_state)
{
	uint32_t hz = 0;
	uint32_t ret = NO_ERROR;
    bool is_moving = true;
    if (final_state) {
        *final_state = OIL;
    }

    /* 读取当前高度（mm） */
    float cur_mm;
    MotorCtrl_SnapshotSensorPositionMm(&cur_mm);

    printf("上行到目标或空气：当前：%.3fmm, 目标：%.3fmm\r\n",
           cur_mm, target_mm);

    /* 如果当前就超过目标，不需要移动 */
    if (cur_mm >= target_mm) {
        printf("当前位置已高于目标点，无需上行。\r\n");
        return NO_ERROR;
    }
    /* 切换频率模式 */
    ret = EnableLevelMode();
    CHECK_ERROR(ret);
    printf("上行到目标或空气：液位测量模式已稳定，开始上行检测。\r\n");
    /* 下发上行运动指令（长度设为足够大） */
    float max_move = target_mm - cur_mm;   /* 理论需要跑的距离 */

    ret = MotorCtrl_MoveNoWait(3*max_move, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100()); /* 走三倍距离保证一定会跑到 */
    CHECK_ERROR(ret);

    /* 进入循环检测：空气 + 到位 + 安全检查 */
    uint32_t start_tick = HAL_GetTick();
    const uint32_t MAX_WAIT_MS = 60*60000;    /* 最长等待 60s*60 =1小时，防止死循环 */

    while (1) {
        ret = MotorCtrl_IsDriverMoving(&stepper, &is_moving);
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_SlowStop();
            return ret;
        }
        if (!is_moving) {
            break;
        }

        /* 1) 检测空气状态 */
        /* 如果传感器是 LTD 传感器 */
        /* 这里处于电机运动监测环节，只允许做轻量频率读取；
           不走 DSM_Get_LevelMode_Frequence() 的重恢复逻辑，
           否则异常时会停电机、重切模式并等待，破坏当前运动流程。 */
		if (g_deviceParams.sensorType == LTD_SENSOR) {
	    	ret = DSM_V2_Read_LevelFrequency(&hz);
			/* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
	    	if (ret != NO_ERROR) {
				(void)MotorCtrl_SlowStop();
				return ret;  /* 读取失败前先停止电机 */
	    	}
	    	if (hz == 0 || hz > g_deviceParams.oilLevelFrequency) {
			if (final_state) *final_state = AIR; /* 读到0或者异常频率认为是空气 */
	            printf("上行到目标或空气：频率检测到到达液面，立即停止电机！\r\n");
	            ret = MotorCtrl_SlowStop();
	            CHECK_ERROR(ret);
			break;  /* 读到0也返回 */
	    	}
	    	 HAL_Delay(80);
		}
        Level_StateTypeDef st = OIL;
        ret = determine_level_status_motion(&st);
        CHECK_ERROR(ret);
        if (final_state) *final_state = st;

        if (st == AIR) {
            printf("上行到目标或空气：检测到进入空气，立即停止电机！\r\n");
            ret = MotorCtrl_SlowStop();
            CHECK_ERROR(ret);
            break;
        }

        /* 2) 检测当前位置是否已经到达目标点 */
        MotorCtrl_SnapshotSensorPositionMm(&cur_mm);

        if (cur_mm >= target_mm - 0.05f) {   /* 加一点浮动允许 */
            printf("上行到目标或空气：已到达目标位置 %.3fmm\r\n", cur_mm);
            ret = MotorCtrl_SlowStop();
            CHECK_ERROR(ret);
            break;
        }

        /* 3) 其他安全检测 */
        ret = CheckWeightCollision();
        CHECK_ERROR(ret);

        ret = MotorCtrl_CheckDriverGstat(); /* 电机状态检测 */
        CHECK_ERROR(ret);

        /* 4) 超时保护 */
        if (HAL_GetTick() - start_tick > MAX_WAIT_MS) {
            printf("上行到目标或空气：运行超时！\r\n");
            RETURN_ERROR(MOTOR_RUN_TIMEOUT);
        }

        HAL_Delay(80);
    }

    /* 结束后，再读一次最终位置 */
    MotorCtrl_SnapshotSensorPositionMm(&cur_mm);
    printf("上行到目标或空气结束：最终位置 %.3fmm\r\n", cur_mm);

    return NO_ERROR;
}
