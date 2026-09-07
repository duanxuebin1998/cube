/*
 * measure_density_si.c
 *
 * 文件职责：实现 SI Profile 独立测点生成、探底周期、候选发布和回液位提交，不参与普通分布测量。
 */

#include "measure_density.h"
#include "measure_density_internal.h"
#include "../measure_commands_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "abortable_delay.h"
#include "fault_manager.h"
#include "measure_tank_height.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"

/* SI 剖面默认首个后续测点绝对位置 100.0 mm，单位为 0.1 mm。 */
#define SI_PROFILE_DEFAULT_FIRST_POINT_01MM 1000U
/* SI 剖面默认相邻测点间隔 1000.0 mm，单位为 0.1 mm。 */
#define SI_PROFILE_DEFAULT_INCREMENT_01MM 10000U
/* SI 剖面到达每个测点后的默认停留时间 10 s。 */
#define SI_PROFILE_DEFAULT_DWELL_TIME_S 10U
/* SI 剖面探底间隔参数允许上限 1000；用于拒绝异常配置，具体距离单位沿用设备参数定义。 */
#define SI_PROFILE_MAX_BOTTOM_DETECT_INTERVAL 1000U
/* SI 剖面空气/液体判别密度阈值 100.0；低于该值视为空气区域，单位与测量密度字段一致。 */
#define SI_PROFILE_AIR_DENSITY_THRESHOLD 100.0f
/* SI 剖面稳定判定的密度采样周期 200 ms。 */
#define SI_PROFILE_DENSITY_SAMPLE_MS 200U
/* SI 剖面判断密度稳定所需的连续观察窗口 5000 ms。 */
#define SI_PROFILE_DENSITY_STABLE_WINDOW_MS 5000U
/* SI 剖面单点等待密度稳定的最长时间 5 min；超时后必须结束该点等待并进入错误处理。 */
#define SI_PROFILE_DENSITY_MAX_WAIT_MS (5U * 60U * 1000U)
/* SI 剖面稳定窗口允许的频率波动上限 1.0 Hz。 */
#define SI_PROFILE_DENSITY_FREQ_EPS_HZ 1.0f
/* SI 剖面稳定窗口允许的密度数值波动上限 0.1，单位与密度字段一致。 */
#define SI_PROFILE_DENSITY_VALUE_EPS 0.1f
/* SI 剖面稳定窗口允许的温度波动上限 0.2 ℃。 */
#define SI_PROFILE_DENSITY_TEMP_EPS_C 0.2f
/* SI 剖面本周期罐底位置基准已经建立的标志。 */
static uint8_t s_si_profile_bottom_ref_valid = 0U;
/* SI 剖面由可信罐底确定的 Point0 实际测量位置，单位为 0.1 mm。 */
static int32_t s_si_profile_point0_position_01mm = 0;
/* 距上次 SI 探底完成后已执行的剖面次数。 */
static uint32_t s_si_profile_count_since_bottom = 0U;
/* SI 剖面上电后的首轮标志，用于强制建立罐底基准。 */
static uint8_t s_si_profile_first_run = 1U;
/* SI 剖面本轮测量候选点阵，完成全部校验前不对外发布。 */
static DensityDistribution s_si_profile_candidate;
/* SI 候选点阵已经完整生成并可提交的标志。 */
static uint8_t s_si_profile_candidate_valid = 0U;

typedef struct {
    /* SI 剖面单点采样的原始结果、换算结果和气相判定信息。 */
    DensityMeasurement measurement; /* 该测点保留的完整原始测量结构，供诊断和后续换算复核。 */
    float frequency_hz; /* 该测点换算后的传感器频率，单位为 Hz。 */
    float density_value; /* 该测点换算后的密度值，单位遵循当前密度算法输出。 */
    float temperature_c; /* 该测点换算后的温度，单位为 ℃。 */
    uint8_t is_air; /* 该测点被判定为气相的标志；非零/true 时不作为有效液相密度点。 */
    const char *air_reason; /* 气相判定原因的只读文本；非气相点可以为空。 */
} SiProfilePointSample;

/**
 * @brief 清除尚未完成的 SI Profile 候选结果。
 *
 * @details 调用场景：新一轮准备、取消、失败或最终结果提交后。
 * @note 关键约束：候选缓冲不直接发布给 CPU3，清除动作不改变已发布完成计数。
 */
static void SiProfile_ClearCandidate(void)
{
    memset(&s_si_profile_candidate, 0, sizeof(s_si_profile_candidate));
    s_si_profile_candidate_valid = 0U;
}

/**
 * @brief 清空对外发布的分布结果载荷，同时保留完成计数。
 *
 * @details 调用场景：Point0 建立新周期，或 Point0 后取消、失败。
 * @note 关键约束：完成计数是最终提交边沿，不得在中间态清零或提前递增。
 *
 * @param blocked_by_process 阻塞处理。
 */
static void SiProfile_ClearPublishedPayload(uint32_t blocked_by_process)
{
    DensityMeasurement empty_point = {0};

    g_measurement.density_distribution.average_temperature = 0U;
    g_measurement.density_distribution.average_density = 0U;
    g_measurement.density_distribution.average_standard_density = 0U;
    g_measurement.density_distribution.average_vcf20 = 0U;
    g_measurement.density_distribution.average_weight_density = 0U;
    g_measurement.density_distribution.measurement_points = 0U;
    g_measurement.density_distribution.Density_oil_level = 0U;
    g_measurement.density_distribution.profile_complete_latched = 0U;
    g_measurement.density_distribution.profile_source = PROFILE_SOURCE_NONE;
    g_measurement.density_distribution.profile_blocked_by_process = blocked_by_process;
    g_measurement.density_distribution.profile_temp_deviation_alarm = 0U;
    g_measurement.density_distribution.profile_density_deviation_alarm = 0U;

    for (uint32_t i = 0U; i < MAX_MEASUREMENT_POINTS; i++) {
        g_measurement.density_distribution.single_density_data[i] = empty_point;
    }
}

/**
 * @brief Point0 有效样本写入候选缓冲后建立新的 SI Profile 周期。
 *
 * @details 调用场景：SI Profile 第一个有效液体点完成写入后。
 * @note 关键约束：载荷和阶段先发布，内存屏障后最后递增周期计数。
 */
static void SiProfile_BeginCycleAtPoint0(void)
{
    SiProfile_ClearPublishedPayload(0U);
    g_measurement.si_profile_runtime.progress_points = 1U;
    g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_MEASURING;
    __DMB();
    g_measurement.si_profile_runtime.cycle_counter++;
}

/**
 * @brief 发布 SI Profile 已完成写入的有效液体点数。
 *
 * @details 调用场景：Point0 之后每个有效液体点写入候选缓冲后。
 * @note 关键约束：空气点、失败样本和重试不得调用本函数。
 *
 * @param valid_points 有效。
 */
static void SiProfile_UpdateProgress(uint32_t valid_points)
{
    __DMB();
    g_measurement.si_profile_runtime.progress_points = valid_points;
}

/**
 * @brief 按当前 SI Profile 阶段处理显式取消或命令切换。
 *
 * @details 调用场景：取消命令入口，以及 SI Profile 相关流程返回 STATE_SWITCH 时。
 * @note 关键约束：Point0 前保留旧结果；Point0 后清除不完整载荷且不增加完成计数。
 */
void SiProfile_HandleCancel(void)
{
    uint32_t phase = g_measurement.si_profile_runtime.phase;

    if (phase == SI_PROFILE_PHASE_PREPARING) {
        SiProfile_ClearCandidate();
        g_measurement.si_profile_runtime.progress_points = 0U;
        __DMB();
        g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_ABORTED;
    } else if ((phase == SI_PROFILE_PHASE_MEASURING) ||
               (phase == SI_PROFILE_PHASE_RETURNING_LEVEL)) {
        SiProfile_ClearCandidate();
        SiProfile_ClearPublishedPayload(0U);
        g_measurement.si_profile_runtime.progress_points = 0U;
        __DMB();
        g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_ABORTED;
    }
}

/**
 * @brief 按当前 SI Profile 阶段处理真实测量失败。
 *
 * @details 调用场景：SI 采点错误，以及回液位流程的真实错误出口。
 * @note 关键约束：Point0 前保留旧结果；Point0 后清除不完整载荷并保留原错误上报链路。
 */
void SiProfile_HandleFailure(void)
{
    uint32_t phase = g_measurement.si_profile_runtime.phase;
    uint8_t profile_start_failed =
            (g_measurement.device_status.current_command == CMD_SI_PROFILE) ? 1U : 0U;

    if ((phase == SI_PROFILE_PHASE_PREPARING) ||
        ((profile_start_failed != 0U) &&
         (phase != SI_PROFILE_PHASE_MEASURING) &&
         (phase != SI_PROFILE_PHASE_RETURNING_LEVEL))) {
        SiProfile_ClearCandidate();
        g_measurement.density_distribution.profile_blocked_by_process = 1U;
        g_measurement.si_profile_runtime.progress_points = 0U;
        __DMB();
        g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_FAILED;
    } else if ((phase == SI_PROFILE_PHASE_MEASURING) ||
               (phase == SI_PROFILE_PHASE_RETURNING_LEVEL)) {
        SiProfile_ClearCandidate();
        SiProfile_ClearPublishedPayload(1U);
        g_measurement.si_profile_runtime.progress_points = 0U;
        __DMB();
        g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_FAILED;
    }
}

/**
 * @brief 回到稳定液位后一次性提交 SI Profile 候选结果。
 *
 * @details 调用场景：找液位成功且完成可选位置源切换后的二次定位，在进入液位跟随前。
 * @note 关键约束：完整载荷、Complete 和阶段先写完，内存屏障后最后递增完成计数。
 *
 * @return NO_ERROR 表示候选点阵已在稳定液位且电机停稳后一次性发布；无有效候选返回 MEASUREMENT_DENSITY_NO_VALID_POINT，液位未恢复返回 MEASUREMENT_OILLEVEL_NOTFOUND，电机未停返回 POSITION_MOTOR_NOT_STOPPED。
 */
uint32_t SiProfile_CompleteAfterReturnToLevel(void)
{
    uint32_t previous_complete_counter;
    uint32_t final_points;

    if (g_measurement.si_profile_runtime.phase != SI_PROFILE_PHASE_RETURNING_LEVEL) {
        return NO_ERROR;
    }
    if (s_si_profile_candidate_valid == 0U) {
        return MEASUREMENT_DENSITY_NO_VALID_POINT;
    }
    if ((g_measurement.oil_measurement.probe_at_liquid_level == 0U) ||
        (g_measurement.oil_measurement.liquid_stable == 0U)) {
        return MEASUREMENT_OILLEVEL_NOTFOUND;
    }
    if (g_measurement.debug_data.motor_state != 0U) {
        return POSITION_MOTOR_NOT_STOPPED;
    }

    previous_complete_counter =
            g_measurement.density_distribution.profile_complete_counter;
    final_points = s_si_profile_candidate.measurement_points;
    s_si_profile_candidate.Density_oil_level =
            g_measurement.oil_measurement.oil_level;
    s_si_profile_candidate.profile_complete_counter = previous_complete_counter;
    s_si_profile_candidate.profile_complete_latched = 0U;
    s_si_profile_candidate.profile_source = PROFILE_SOURCE_NONE;
    s_si_profile_candidate.profile_blocked_by_process = 0U;

    g_measurement.density_distribution = s_si_profile_candidate;
    g_measurement.density_distribution.profile_complete_counter =
            previous_complete_counter;
    g_measurement.density_distribution.measurement_points = final_points;
    g_measurement.density_distribution.Density_oil_level =
            g_measurement.oil_measurement.oil_level;
    g_measurement.density_distribution.profile_source = PROFILE_SOURCE_SI;
    g_measurement.density_distribution.profile_complete_latched = 1U;
    g_measurement.density_distribution.profile_blocked_by_process = 0U;
    g_measurement.si_profile_runtime.progress_points = final_points;
    g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_COMPLETE;
    s_si_profile_candidate_valid = 0U;
    __DMB();
    g_measurement.density_distribution.profile_complete_counter =
            previous_complete_counter + 1U;

    return NO_ERROR;
}

/**
 * @brief 返回 SI profile 参数快照，并补齐旧 FRAM 或非法写入产生的默认值。
 *
 * @details 调用场景：SI profile 每轮开始前，由 CPU2 独立 profile 流程调用。
 * @note 关键约束：这里只做运行期兜底，不写回 FRAM；参数持久化归一化仍由参数存储层负责。
 *
 * @param first_point_01mm SI Profile 中 Point0 之后首个候选点的绝对位置，单位 0.1 mm。
 * @param increment_01mm SI Profile 相邻候选测点之间的间距，单位 0.1 mm。
 * @param dwell_time_s 用于返回归一化后的测点停留时间，单位 s。
 * @param bottom_detect_interval 用于返回 SI Profile 重新探底的周期间隔；1 表示每轮探底。
 */
static void SiProfile_GetParams(uint32_t *first_point_01mm,
                                    uint32_t *increment_01mm,
                                    uint32_t *dwell_time_s,
                                    uint32_t *bottom_detect_interval)
{
    uint32_t first_point = g_deviceParams.si_profile_first_point;
    uint32_t increment = g_deviceParams.si_profile_increment;
    uint32_t dwell = g_deviceParams.si_profile_dwell_time;
    uint32_t interval = g_deviceParams.si_profile_bottom_detect_interval;

    if (first_point == 0U) {
        first_point = SI_PROFILE_DEFAULT_FIRST_POINT_01MM;
    }
    if (increment == 0U) {
        increment = SI_PROFILE_DEFAULT_INCREMENT_01MM;
    }
    if ((dwell == 0U) || (dwell > 3600U)) {
        dwell = SI_PROFILE_DEFAULT_DWELL_TIME_S;
    }
    if ((interval == 0U) || (interval > SI_PROFILE_MAX_BOTTOM_DETECT_INTERVAL)) {
        interval = 1U;
    }

    *first_point_01mm = first_point;
    *increment_01mm = increment;
    *dwell_time_s = dwell;
    *bottom_detect_interval = interval;
}

/**
 * @brief 按 SI 探底频次判断本轮 profile 是否需要重新探底。
 *
 * @details 调用场景：SI profile 开始阶段。
 * @note 关键约束：首轮必须探底；interval=1 表示每次探底，N 表示每 N 次 profile 探底。
 *
 * @param bottom_detect_interval 罐底。
 * @return 1 表示首轮或当前间隔已到期，本轮必须重新探底；0 表示可沿用已有底部参考。
 */
static uint8_t SiProfile_ShouldDetectBottom(uint32_t bottom_detect_interval)
{
    if ((s_si_profile_first_run != 0U) ||
        (s_si_profile_bottom_ref_valid == 0U) ||
        (bottom_detect_interval <= 1U)) {
        return 1U;
    }

    return ((s_si_profile_count_since_bottom + 1U) >= bottom_detect_interval) ? 1U : 0U;
}

/**
 * @brief 按本轮是否执行探底更新 SI 点阵距上次探底计数，并在上限处饱和。
 *
 * @param bottom_detect_required 罐底需求标志。
 */
static void SiProfile_AdvanceBottomDetectTriggerCount(uint8_t bottom_detect_required)
{
    if (bottom_detect_required != 0U) {
        s_si_profile_count_since_bottom = 1U;
    } else if (s_si_profile_count_since_bottom < UINT32_MAX) {
        s_si_profile_count_since_bottom++;
    }
}

/**
 * @brief 把探底记录的尺带长度换算为 SI Profile 的底部坐标。
 *
 * @details 调用场景：复用普通探底参考或本轮 SI 新探底成功后。
 * @note 关键约束：使用 tankHeight-TankHeight_GetBottomCableLength01mm()，并钳位到有符号位置可表达范围。
 *
 * @param bottom_cable_01mm 探底流程记录的罐底尺带长度，单位 0.1 mm。
 * @return 返回 tankHeight 减尺带长度得到的 SI 底部坐标，单位 0.1 mm；结果钳位到 0 至 INT32_MAX。
 */
static int32_t SiProfile_ResolveBottomPositionFromCable(int32_t bottom_cable_01mm)
{
    int64_t resolved_bottom =
            (int64_t)g_deviceParams.tankHeight - (int64_t)bottom_cable_01mm;

    if (resolved_bottom < 0) {
        resolved_bottom = 0;
    } else if (resolved_bottom > INT32_MAX) {
        resolved_bottom = INT32_MAX;
    }

    return (int32_t)resolved_bottom;
}

/**
 * @brief 根据可信罐底参考恢复不会重新触底的 Point0 位置。
 *
 * @details 调用场景：上电后复用普通探底参考，或本轮不重新探底时沿用既有 Point0。
 * @note 关键约束：偏移量必须与 SearchBottom 成功收尾的上提距离保持一致。
 *
 * @param bottom_cable_01mm 探底流程记录的罐底尺带长度，单位 0.1 mm。
 * @return 按探底模式返回罐底上方的释放位置，单位 0.1 mm，并钳位到 INT32_MAX。
 */
static int32_t SiProfile_ResolveReleasedPoint0FromCable(int32_t bottom_cable_01mm)
{
    int64_t point0_position =
            (int64_t)SiProfile_ResolveBottomPositionFromCable(bottom_cable_01mm) +
            ((g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) ?
             (int64_t)BOTTOM_WEIGHT_RELEASE_MARGIN_01MM :
             (int64_t)BOTTOM_GYRO_RELEASE_MARGIN_01MM);

    if (point0_position > INT32_MAX) {
        point0_position = INT32_MAX;
    }

    return (int32_t)point0_position;
}

/**
 * @brief 在本轮新探底前快照普通探底流程已经建立的可信底部参考。
 *
 * @details 调用场景：SI Profile 每轮探底判断之前。
 * @note 关键约束：只在静态旧底无效且共享有效标志置位时采纳；失败后不得重读可能改写的 TankHeight_GetBottomCableLength01mm()。
 */
static void SiProfile_CaptureSharedBottomReference(void)
{
    if ((s_si_profile_bottom_ref_valid == 0U) &&
        (g_measurement.height_measurement.bottom_reference_valid != 0U)) {
        s_si_profile_point0_position_01mm =
                SiProfile_ResolveReleasedPoint0FromCable(TankHeight_GetBottomCableLength01mm());
        s_si_profile_bottom_ref_valid = 1U;
        printf("SI Profile 已快照普通探底参考: 尺带=%ld(0.1mm) Point0=%.1fmm\r\n",
               (long)TankHeight_GetBottomCableLength01mm(),
               (double)s_si_profile_point0_position_01mm / 10.0);
    }
}

/**
 * @brief 确定 SI profile 的 Point0 实际测量位置。
 *
 * @details 调用场景：探底成功、探底失败或本轮跳过探底后。
 * @note 关键约束：新探底成功后直接采用 SearchBottom 上提后的当前位置；后续周期沿用该释放位置，不再回到触底坐标。
 *
 * @param bottom_search_done 本轮已经调用 SearchBottom 的标志。
 * @param bottom_search_ret 本轮 SearchBottom 的返回值。
 * @return 返回 SI Profile 使用的 Point0 实际位置，单位 0.1 mm；没有有效参考时返回当前位置作为兼容回退。
 */
static int32_t SiProfile_SelectPoint0Position(uint8_t bottom_search_done,
                                                  uint32_t bottom_search_ret)
{
    int32_t current_position = g_measurement.debug_data.sensor_position;

    if (current_position < 0) {
        current_position = 0;
    }

    if ((bottom_search_done != 0U) && (bottom_search_ret == NO_ERROR)) {
        int32_t resolved_bottom_position =
                SiProfile_ResolveBottomPositionFromCable(TankHeight_GetBottomCableLength01mm());

        /* SearchBottom 已按当前模式完成释放，直接采用实际停留位置。 */
        s_si_profile_point0_position_01mm = current_position;
        s_si_profile_bottom_ref_valid = 1U;
        s_si_profile_first_run = 0U;
        g_measurement.height_measurement.bottom_reference_valid = 1U;
        printf("SI Profile 新探底基准: 罐高=%lu(0.1mm) 尺带=%ld(0.1mm) 罐底=%.1fmm Point0=%.1fmm\r\n",
               (unsigned long)g_deviceParams.tankHeight,
               (long)TankHeight_GetBottomCableLength01mm(),
               (double)resolved_bottom_position / 10.0,
               (double)s_si_profile_point0_position_01mm / 10.0);
        return s_si_profile_point0_position_01mm;
    }

    if (s_si_profile_bottom_ref_valid != 0U) {
        printf("SI Profile 探底未更新，沿用已释放 Point0 位置 %.1fmm\r\n",
               (double)s_si_profile_point0_position_01mm / 10.0);
        return s_si_profile_point0_position_01mm;
    }

    printf("SI Profile 无旧 Point0 位置，使用当前位置 %.1fmm 继续\r\n",
           (double)current_position / 10.0);
    return current_position;
}

/**
 * @brief 按 SI profile 语义生成 Point0 和后续候选停点。
 *
 * @details 调用场景：SI profile 确定 Point0 实际测量位置后。
 * @note 关键约束：不预先找液位；Point0 使用罐底释放后的实际位置，后续点保持 40001 绝对位置语义。
 *
 * @param point0_position_01mm SI Profile 使用的 Point0 实际测量位置，单位 0.1 mm。
 * @param first_point_01mm SI Profile 中 Point0 之后首个候选点的绝对位置，单位 0.1 mm。
 * @param increment_01mm SI Profile 相邻候选测点之间的间距，单位 0.1 mm。
 * @param points01 用于接收 Point0 及后续候选停点的数组，元素单位为 0.1 mm。
 * @param point_count 用于返回实际生成的候选停点数量，最大为 200。
 * @return NO_ERROR 表示已生成至少一个且不超过 200 个合法停点；空输出指针返回 SYSTEM_CALL_CONDITION_ERROR，Point0、首点、间距、容量或生成结果非法时返回 MEASUREMENT_DENSITY_PLAN_INVALID。
 */
static uint32_t SiProfile_BuildPoints(int32_t point0_position_01mm,
                                          uint32_t first_point_01mm,
                                          uint32_t increment_01mm,
                                          int32_t *points01,
                                          uint32_t *point_count)
{
    uint32_t n = 0U;
    int64_t target;
    int64_t tank_height = (int64_t)g_deviceParams.tankHeight;
    int64_t bottom = (int64_t)point0_position_01mm;

    if ((points01 == NULL) || (point_count == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    *point_count = 0U;
    if ((increment_01mm == 0U) || (first_point_01mm == 0U)) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }
    /* 40001 是绝对坐标。首点必须高于已释放 Point0，禁止反向下探或默默改写测点。 */
    if ((bottom < 0) || (tank_height <= 0) || (tank_height > INT32_MAX) ||
        ((int64_t)first_point_01mm <= bottom) ||
        ((int64_t)first_point_01mm > tank_height)) {
        printf("SI Profile 点位计划无效: Point0=%.1fmm 首点=%.1fmm 罐高=%.1fmm\r\n",
               (double)bottom / 10.0,
               (double)first_point_01mm / 10.0,
               (double)tank_height / 10.0);
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    points01[n++] = (int32_t)bottom;
    target = (int64_t)first_point_01mm;

    while ((n < MAX_MEASUREMENT_POINTS) && (target <= tank_height)) {
        points01[n] = (int32_t)target;
        n++;
        target += (int64_t)increment_01mm;
    }

    *point_count = n;
    return (n > 0U) ? NO_ERROR : MEASUREMENT_DENSITY_PLAN_INVALID;
}

/**
 * @brief 向 SI 剖面结果区记录当前测点位置，单位 0.1 mm。
 *
 * @param point_index 测点索引。
 * @param movement_position_01mm 当前测点实际到达的位置，单位 0.1 mm。
 * @return 返回写入 SI Profile 结果区的非负位置值，单位 0.1 mm；超出无符号范围时按钳位规则处理。
 */
static uint32_t SiProfile_ReportPosition01mm(uint32_t point_index, int32_t movement_position_01mm)
{
    (void)point_index;
    return DensityInternal_ValueToU01mmClamped(movement_position_01mm, "SI Profile position");
}

/**
 * @brief 判断 SI profile 到点后的密度读数是否已经进入空气区。
 *
 * @details 调用场景：SI profile 逐点运行时，不预先找液位，依靠到点读数决定是否停止。
 * @note 关键约束：空气点只作为停止条件，不写入 profile 有效点阵。
 *
 * @param density_value 密度数值。
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param air_reason 用于返回当前样本被判为空气点的原因文字。
 * @return 1 表示 SI profile 到点后的密度读数已经进入空气区；0 表示 SI profile 到点后的密度读数尚未进入空气区。
 */
static uint8_t SiProfile_IsAirPoint(float density_value, float frequency_hz, const char **air_reason)
{
    if (density_value < SI_PROFILE_AIR_DENSITY_THRESHOLD) {
        if (air_reason != NULL) {
            *air_reason = "density_low";
        }
        return 1U;
    }

    if ((g_deviceParams.oilLevelFrequency != 0U) &&
        (frequency_hz > (float)g_deviceParams.oilLevelFrequency)) {
        if (air_reason != NULL) {
            *air_reason = "frequency_high";
        }
        return 1U;
    }

    if (air_reason != NULL) {
        *air_reason = "liquid";
    }
    return 0U;
}

/**
 * @brief 填充 SI profile 单点样本。
 *
 * @details 调用场景：SI profile 到点后完成空气/液体分类并生成可写入点阵的数据。
 * @note 关键约束：调用方负责保证空气点不计入有效点数。
 *
 * @param sample 待处理的单次测量样本。
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param density_value 密度数值。
 * @param temperature_c 温度值，单位 ℃。
 * @param is_air true 表示当前点为空气点，false 表示液体测点。
 * @param air_reason 用于返回当前样本被判为空气点的原因文字。
 */
static void SiProfile_FillPointSample(SiProfilePointSample *sample,
                                          float frequency_hz,
                                          float density_value,
                                          float temperature_c,
                                          uint8_t is_air,
                                          const char *air_reason)
{
    if (sample == NULL) {
        return;
    }

    memset(sample, 0, sizeof(*sample));
    sample->frequency_hz = frequency_hz;
    sample->density_value = density_value;
    sample->temperature_c = temperature_c;
    sample->is_air = is_air;
    sample->air_reason = air_reason;

    sample->measurement.density = DENSITY_TO_RAW(density_value);
    sample->measurement.standard_density = DENSITY_TO_RAW(density_value);
    sample->measurement.weight_density = DENSITY_TO_RAW(density_value);
    sample->measurement.temperature = TEMP_TO_RAW(temperature_c);
    sample->measurement.temperature_position = DensityInternal_CurrentPositionToU01mmClamped();
    sample->measurement.vcf20 = 1U;
}

/**
 * @brief SI profile 到点后读取密度并判断该点是液体点还是液面以上点。
 *
 * @details 调用场景：SiProfile_RunPoints01mmWithDwell() 逐点调用。
 * @note 关键约束：零值和非有限值不在首次读取时判空气；等待满 5 分钟后沿用原有末次液体值、频率异常或零密度空气处理，不新增整轮未就绪出口。
 *
 * @param sample 待处理的单次测量样本。
 * @param stable_win_ms 传感器读数必须持续满足稳定条件的窗口时长，单位 ms。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t SiProfile_ReadPointAndClassify(SiProfilePointSample *sample,
                                                   uint32_t stable_win_ms)
{
    uint32_t ret;
    uint32_t start_tick;
    uint32_t stable_start = 0U;
    uint32_t invalid_sample_count = 0U;
    uint8_t first_sample = 1U;
    uint8_t has_valid_frequency = 0U;
    uint8_t current_frequency_invalid = 0U;
    uint8_t have_last_liquid = 0U;
    float ref_freq = 0.0f;
    float ref_density = 0.0f;
    float ref_temp = 0.0f;
    float cur_freq = 0.0f;
    float cur_density = 0.0f;
    float cur_temp = 0.0f;
    float last_liquid_freq = 0.0f;
    float last_liquid_density = 0.0f;
    float last_liquid_temp = 0.0f;

    if (sample == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }


    if (stable_win_ms == 0U) {
        stable_win_ms = 5000U;
    }

    start_tick = HAL_GetTick();
    while (1) {
        const char *air_reason = NULL;
        uint32_t now;

        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }

        now = HAL_GetTick();
        if ((now - start_tick) >= SI_PROFILE_DENSITY_MAX_WAIT_MS) {
            if (have_last_liquid != 0U) {
                SiProfile_FillPointSample(sample,
                                          last_liquid_freq,
                                          last_liquid_density,
                                          last_liquid_temp,
                                          0U,
                                          "timeout_last_liquid");
                return NO_ERROR;
            }
            if ((has_valid_frequency == 0U) || (current_frequency_invalid != 0U)) {
                return SONIC_FREQ_ABNORMAL;
            }
            SiProfile_FillPointSample(sample,
                                      cur_freq,
                                      0.0f,
                                      cur_temp,
                                      1U,
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

        if ((!isfinite(cur_freq)) || (!isfinite(cur_density)) || (!isfinite(cur_temp)) ||
            (cur_freq <= 0.0f) || (cur_density <= 0.0f)) {
            if (isfinite(cur_freq) && (cur_freq > 0.0f)) {
                has_valid_frequency = 1U;
                current_frequency_invalid = 0U;
            } else {
                current_frequency_invalid = 1U;
            }
            invalid_sample_count++;
            first_sample = 1U;
            stable_start = 0U;
            if ((invalid_sample_count == 1U) || ((invalid_sample_count % 25U) == 0U)) {
                printf("SI Profile 测点数据未稳定: 频率=%.3f 密度=%.3f 温度=%.3f 次数=%lu\r\n",
                       (double)cur_freq,
                       (double)cur_density,
                       (double)cur_temp,
                       (unsigned long)invalid_sample_count);
            }
            ret = AbortableDelay_CommandSwitch(SI_PROFILE_DENSITY_SAMPLE_MS, 50U);
            if (ret != NO_ERROR) {
                return ret;
            }
            continue;
        }
        invalid_sample_count = 0U;
        has_valid_frequency = 1U;
        current_frequency_invalid = 0U;

        if (SiProfile_IsAirPoint(cur_density, cur_freq, &air_reason) != 0U) {
            SiProfile_FillPointSample(sample,
                                          cur_freq,
                                          cur_density,
                                          cur_temp,
                                          1U,
                                          air_reason);
            return NO_ERROR;
        }

        last_liquid_freq = cur_freq;
        last_liquid_density = cur_density;
        last_liquid_temp = cur_temp;
        have_last_liquid = 1U;

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
            if ((df > SI_PROFILE_DENSITY_FREQ_EPS_HZ) ||
                (dd > SI_PROFILE_DENSITY_VALUE_EPS) ||
                (dt > SI_PROFILE_DENSITY_TEMP_EPS_C)) {
                ref_freq = cur_freq;
                ref_density = cur_density;
                ref_temp = cur_temp;
                stable_start = now;
            }
        }

        if ((first_sample == 0U) && ((now - stable_start) >= stable_win_ms)) {
            SiProfile_FillPointSample(sample,
                                          ref_freq,
                                          ref_density,
                                          ref_temp,
                                          0U,
                                          "liquid");
            return NO_ERROR;
        }

        ret = AbortableDelay_CommandSwitch(SI_PROFILE_DENSITY_SAMPLE_MS, 50U);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
}

/**
 * @brief 执行 SI profile 候选点阵，遇到液面以上点时停止且不写入该点。
 *
 * @details 调用场景：CMD_SiProfile() 已确定底部基准并生成候选停点后。
 * @note 关键约束：至少 Point0 有效才认为本轮 profile 成功；完成后 Density_oil_level 记录最后一个有效 profile 点位置。
 *
 * @param p01 按执行顺序排列的测点位置数组，元素单位为 0.1 mm。
 * @param n 测点数组中参与打印或测量的有效点数。
 * @param dist 密度分布测量结果对象。
 * @param dwell_time_s 测点到位后的稳定停留时间，单位 s。
 * @param point0_already_reached 非零表示新探底已上提到 Point0，本轮禁止再次下行定位 Point0。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t SiProfile_RunPoints01mmWithDwell(const int32_t *p01,
                                                     uint32_t n,
                                                     DensityDistribution *dist,
                                                     uint32_t dwell_time_s,
                                                     uint8_t point0_already_reached)
{
    uint32_t valid = 0U;
    uint64_t sum_temp_raw = 0U;
    uint64_t sum_dens_raw = 0U;
    uint32_t dwell_ms = dwell_time_s * 1000U;
    uint32_t last_valid_position_01mm = 0U;
    uint8_t stopped_by_air = 0U;

    if ((p01 == NULL) || (dist == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if ((n == 0U) || (n > MAX_MEASUREMENT_POINTS)) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    memset(dist, 0, sizeof(*dist));

    for (uint32_t i = 0U; i < n; i++) {
        float pos_mm = (float)p01[i] / 10.0f;
        uint32_t ret;
        SiProfilePointSample sample;
        uint32_t report_position_01mm;

        if ((i == 0U) && (point0_already_reached != 0U)) {
            printf("SI Profile 候选点0复用探底上提后位置 %.1f mm，不再下行\r\n", pos_mm);
        } else {
            printf("SI Profile 移动到候选点%lu 位置 %.1f mm\r\n",
                   (unsigned long)i,
                   pos_mm);

            ret = MotorCtrl_JogMoveToPosition(pos_mm, MotorCtrl_GetDefaultSpeedX100());
            if (ret == STATE_SWITCH) {
                return STATE_SWITCH;
            }
            if (ret != NO_ERROR) {
                printf("SI Profile 电机移动失败: 位置=%.1fmm 错误码=%lu\r\n", pos_mm, (unsigned long)ret);
                return ret;
            }
        }

        ret = SensorService_EnableDensityMode();
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            printf("SI Profile 切换密度模式失败: 位置=%.1fmm 错误码=%lu\r\n", pos_mm, (unsigned long)ret);
            return ret;
        }

        /*
         * 先切换密度模式，再执行40003配置的到点驻留时间，让传感器在驻留期间完成模式稳定；
         * 驻留结束后仍使用固定稳定窗口读取，保证1～3600秒配置都按原值执行。
         */
        if (dwell_ms > 0U) {
            ret = AbortableDelay_CommandSwitch(dwell_ms, 100U);
            if (ret != NO_ERROR) {
                return ret;
            }
        }

        ret = SiProfile_ReadPointAndClassify(&sample,
                                             SI_PROFILE_DENSITY_STABLE_WINDOW_MS);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            printf("SI Profile 单点读取失败: 位置=%.1fmm 错误码=%lu\r\n", pos_mm, (unsigned long)ret);
            return ret;
        }
        if (sample.is_air != 0U) {
            printf("SI Profile 候选点%lu 判定为液面以上: 位置=%.1fmm 密度=%.3f 频率=%.3f 原因=%s\r\n",
                   (unsigned long)i,
                   pos_mm,
                   (double)sample.density_value,
                   (double)sample.frequency_hz,
                   (sample.air_reason != NULL) ? sample.air_reason : "air");
            stopped_by_air = 1U;
            break;
        }

        report_position_01mm = SiProfile_ReportPosition01mm(i, p01[i]);
        sample.measurement.temperature_position = report_position_01mm;
        dist->single_density_data[valid] = sample.measurement;
        dist->single_density_data[valid].temperature_position = report_position_01mm;
        sum_temp_raw += sample.measurement.temperature;
        sum_dens_raw += sample.measurement.density;
        last_valid_position_01mm = sample.measurement.temperature_position;
        valid++;
        dist->measurement_points = valid;

        if (valid == 1U) {
            SiProfile_BeginCycleAtPoint0();
        } else {
            SiProfile_UpdateProgress(valid);
        }

        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前 SI Profile\r\n");
            return STATE_SWITCH;
        }
        if (valid >= MAX_MEASUREMENT_POINTS) {
            break;
        }
    }

    if (valid == 0U) {
        printf("SI Profile 未得到任何有效测点\r\n");
        return MEASUREMENT_DENSITY_NO_VALID_POINT;
    }

    dist->measurement_points = valid;
    dist->Density_oil_level = last_valid_position_01mm;
    dist->average_temperature = (uint32_t)(sum_temp_raw / valid);
    dist->average_density = (uint32_t)(sum_dens_raw / valid);
    dist->average_standard_density = dist->average_density;
    dist->average_vcf20 = 0U;
    dist->average_weight_density = dist->average_density;

    printf("SI Profile 有效点数=%lu 最后有效点=%.1fmm 停止原因=%s\r\n",
           (unsigned long)valid,
           (double)last_valid_position_01mm / 10.0,
           (stopped_by_air != 0U) ? "液面以上点" : "点数或上限");

    return NO_ERROR;
}

/**
 * @brief 执行 SI 独立 profile 测量。
 *
 * 调用场景：CPU3 SI Profile 线圈或自动调度下发 CMD_SI_PROFILE 后由命令分发调用。
 * 函数清空上一轮 SI 候选结果，复位进度并进入准备阶段，从独立 SI 参数区取得首点、点距、驻留时间和探底周期；这些参数不复用普通密度分布测量配置。
 * 到达探底周期时先执行 SearchBottom；命令切换立即取消本轮，其他探底错误只记录诊断，并允许点阵生成按已有罐底参考或当前位置继续。
 * 点阵生成成功后逐点移动、驻留并采集到局部候选结构；生成或测量失败会清除候选、发布错误并退出，未完成的候选不会成为对外最终结果。
 * 采点收尾与命令排队共用关中断临界区：若已有新命令或非回液位命令，本轮候选立即取消；否则锁存候选、更新进度阶段，并在命令槽为空时排队 CMD_FIND_OIL。
 * 函数结束时只进入“等待回液位”阶段；真正的 SI 最终完成快照必须等回到稳定液位且电机停止后由后续流程发布。
 *
 * @note 关键约束：不复用普通分布测 profile 参数；失败不锁存完成态，命令切换直接退出。
 * @note 候选锁存和回液位命令排队期间以新到命令为最高优先级，不得覆盖用户已经送达的其他命令。
 */
void CMD_SiProfile(void)
{
    uint32_t ret;
    uint32_t first_point_01mm;
    uint32_t increment_01mm;
    uint32_t dwell_time_s;
    uint32_t bottom_detect_interval;
    uint8_t should_detect_bottom;
    uint8_t bottom_search_done = 0U;
    uint32_t bottom_search_ret = NO_ERROR;
    int32_t point0_position_01mm;
    int32_t points01[MAX_MEASUREMENT_POINTS];
    uint32_t point_count = 0U;
    uint32_t primask;
    uint8_t queue_conflict = 0U;
    uint8_t queued_find_oil = 0U;

    memset(points01, 0, sizeof(points01));
    SiProfile_ClearCandidate();
    g_measurement.si_profile_runtime.progress_points = 0U;
    g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_PREPARING;
    g_measurement.device_status.device_state = STATE_SPREADPOINTING;

    SiProfile_GetParams(&first_point_01mm,
                            &increment_01mm,
                            &dwell_time_s,
                            &bottom_detect_interval);

    printf("SI Profile 开始: first=%lu(0.1mm) increment=%lu(0.1mm) dwell=%lus bottomInterval=%lu\r\n",
           (unsigned long)first_point_01mm,
           (unsigned long)increment_01mm,
           (unsigned long)dwell_time_s,
           (unsigned long)bottom_detect_interval);

    SiProfile_CaptureSharedBottomReference();
    should_detect_bottom = SiProfile_ShouldDetectBottom(bottom_detect_interval);
    SiProfile_AdvanceBottomDetectTriggerCount(should_detect_bottom);
    if (should_detect_bottom != 0U) {
        bottom_search_done = 1U;
        bottom_search_ret = SearchBottom();
        if (bottom_search_ret == STATE_SWITCH) {
            SiProfile_HandleCancel();
            return;
        }
        if (bottom_search_ret != NO_ERROR) {
            printf("SI Profile 探底失败，错误码=0x%08lX，将按旧底部或当前位置继续\r\n",
                   (unsigned long)bottom_search_ret);
        }
    }

    point0_position_01mm = SiProfile_SelectPoint0Position(bottom_search_done, bottom_search_ret);

    ret = SiProfile_BuildPoints(point0_position_01mm,
                                    first_point_01mm,
                                    increment_01mm,
                                    points01,
                                    &point_count);
    if (ret != NO_ERROR) {
        SiProfile_HandleFailure();
        printf("SI Profile 生成点位失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
        return;
    }

    DensityInternal_PrintPoints01mm("SI Profile", points01, point_count);
    ret = SiProfile_RunPoints01mmWithDwell(points01,
                                           point_count,
                                           &s_si_profile_candidate,
                                           dwell_time_s,
                                           ((bottom_search_done != 0U) &&
                                            (bottom_search_ret == NO_ERROR)) ? 1U : 0U);
    if (ret == STATE_SWITCH) {
        SiProfile_HandleCancel();
        return;
    }

    if (ret != NO_ERROR) {
        SiProfile_HandleFailure();
        printf("SI Profile 测量失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
        return;
    }

    /* 采点收尾和命令排队采用同一临界区，避免覆盖并发到达的其他命令。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((new_command_ready != 0U) ||
        ((g_deviceParams.command != CMD_NONE) &&
         (g_deviceParams.command != CMD_FIND_OIL))) {
        queue_conflict = 1U;
    } else {
        s_si_profile_candidate_valid = 1U;
        g_measurement.si_profile_runtime.progress_points =
                s_si_profile_candidate.measurement_points;
        g_measurement.si_profile_runtime.phase = SI_PROFILE_PHASE_RETURNING_LEVEL;
        __DMB();
        if (g_deviceParams.command == CMD_NONE) {
            DeviceCommand_Queue(CMD_FIND_OIL);
            queued_find_oil = 1U;
        }
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (queue_conflict != 0U) {
        printf("SI Profile 采点结束时已有其他命令，取消未完成候选结果\r\n");
        SiProfile_HandleCancel();
        return;
    }

    Print_DensitySpreadResult(&s_si_profile_candidate);
    if (queued_find_oil != 0U) {
        printf("SI Profile 采点完成，已排队回液位命令\r\n");
    } else {
        printf("SI Profile 采点完成，沿用已排队的回液位命令\r\n");
    }
    printf("SI Profile 等待回液位: 点数=%lu 周期=%lu\r\n",
           (unsigned long)s_si_profile_candidate.measurement_points,
           (unsigned long)g_measurement.si_profile_runtime.cycle_counter);
}
