/*
 * 文件职责：频率液位查找和跟随闭环实现。
 * 本文件负责频率目标、死区和调速逻辑，结果副作用统一交给运行时服务。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "oil_level_reference_refresh.h"
#include "abortable_delay.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"
#include "weight.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static uint32_t FrequencyLevel_RunClosedLoopConfigured(uint32_t follow_mode,
                                                       uint32_t target_frequency_hz,
                                                       float deadband_override_hz,
                                                       uint8_t fixed_target_mode,
                                                       OilLevelRefreshSearch *search);

/**
 * @brief 使液位 AO 样本失效、慢停电机并返回原频率闭环错误码。
 * @param error_code 需要保持的原错误码或 STATE_SWITCH。
 * @param reason 频率闭环退出原因，用于诊断日志。
 * @return 完成统一清理后的原 error_code。
 */
static uint32_t FrequencyLevel_StopAndReturn(uint32_t error_code, const char *reason)
{
    return OilLevelRuntime_StopAndInvalidate(
            error_code,
            reason,
            OIL_LEVEL_RUNTIME_DOMAIN_FREQUENCY);
}

/**
 * @brief 校验固定频率目标非零且不超过传感器有效上限。
 *
 * @param target_frequency_hz 待校验的固定目标频率，单位 Hz。
 * @return PARAM_RANGE_ERROR 表示参数超出允许范围；NO_ERROR 表示操作成功。
 */
static uint32_t FrequencyLevel_ValidateTarget(uint32_t target_frequency_hz)
{
    if (target_frequency_hz == 0U) {
        printf("固定频率找液位\t目标频率为0\r\n");
        return PARAM_CONFIG_MISSING;
    }
    if (target_frequency_hz > FREQUENCY_LEVEL_VALID_MAX_HZ) {
        printf("固定频率找液位\t目标频率超出有效范围：%lu Hz\r\n",
               (unsigned long)target_frequency_hz);
        return PARAM_RANGE_ERROR;
    }
    return NO_ERROR;
}

/**
 * @brief 校验固定频率目标、可选死区和运动安全边界参数。
 *
 * @param target_frequency_hz 固定频率目标，单位 Hz。
 * @param deadband_hz 本次覆盖死区，单位 Hz；0 表示由生产闭环读取设备死区。
 * @return NO_ERROR 表示配置可用于闭环；其他值表示缺失或越界。
 */
static uint32_t FrequencyLevel_ValidateFixedConfig(uint32_t target_frequency_hz,
                                                   uint32_t deadband_hz)
{
    uint32_t ret = FrequencyLevel_ValidateTarget(target_frequency_hz);
    int64_t zero_weight_limit;

    if (ret != NO_ERROR) {
        return ret;
    }
    if (((deadband_hz != 0U) &&
         (deadband_hz > FREQUENCY_LEVEL_VALID_MAX_HZ)) ||
        (g_deviceParams.tankHeight <= 1000U) ||
        ((int64_t)g_deviceParams.blindZone >=
         ((int64_t)g_deviceParams.tankHeight - 1000LL)) ||
        (g_deviceParams.full_weight == 0U) ||
        (g_deviceParams.full_weight > (uint32_t)INT32_MAX) ||
        (g_deviceParams.bottom_weight_threshold > (uint32_t)INT32_MAX) ||
        (g_deviceParams.zero_weight_threshold_ratio > 100U)) {
        return PARAM_RANGE_ERROR;
    }
    zero_weight_limit = ((int64_t)g_deviceParams.full_weight *
                         (100LL + (int64_t)g_deviceParams.zero_weight_threshold_ratio)) /
                        100LL;
    if ((zero_weight_limit > (int64_t)INT32_MAX) ||
        ((int64_t)g_deviceParams.bottom_weight_threshold >= zero_weight_limit)) {
        return PARAM_RANGE_ERROR;
    }
    return NO_ERROR;
}

/**
 * @brief 按固定频率目标和死区闭环跟随液位。
 *
 * @param follow_mode 跟随模式。
 * @return 返回整机错误码；NO_ERROR 表示固定频率闭环正常结束，其他值表示命令切换、传感器或电机控制失败。
 */
uint32_t FrequencyLevel_RunFixedClosedLoop(uint32_t follow_mode)
{
    uint32_t ret = FrequencyLevel_ValidateFixedConfig(
            g_deviceParams.oilLevelFrequency,
            0U);
    if (ret != NO_ERROR) {
        return ret;
    }
    return FrequencyLevel_RunClosedLoopConfigured(follow_mode,
                                                  g_deviceParams.oilLevelFrequency,
                                                  0.0f,
                                                  1U, NULL);
}

/**
 * @brief 使用串口本次覆盖目标和死区运行一次方法5固定频率搜索。
 *
 * @param target_frequency_hz 本次搜索目标频率，单位 Hz。
 * @param deadband_hz 本次搜索死区半宽，单位 Hz。
 * @return NO_ERROR 表示稳定命中液位；其他值为参数、命令切换、传感器、电机或安全保护错误。
 * @note 本接口只在运行期间临时覆盖并在返回前恢复公开目标，不修改设备参数或 FRAM；串口调试与生产方法5共用闭环和保护出口。
 */
uint32_t OilLevel_RunFixedFrequencySearch(uint32_t target_frequency_hz,
                                         uint32_t deadband_hz)
{
    uint32_t ret = FrequencyLevel_ValidateFixedConfig(target_frequency_hz,
                                                      deadband_hz);
    uint32_t saved_follow_frequency = g_measurement.oil_measurement.follow_frequency;

    if ((ret == NO_ERROR) && (deadband_hz == 0U)) {
        printf("固定频率找液位\t目标、死区或安全边界参数无效：%lu Hz\r\n",
               (unsigned long)deadband_hz);
        ret = PARAM_RANGE_ERROR;
    }
    if (ret != NO_ERROR) {
        return ret;
    }

    OilLevel_ResetSearchRuntimeState();
    ret = FrequencyLevel_RunClosedLoopConfigured(OIL_LEVEL_CLOSED_LOOP_SEARCH,
                                                 target_frequency_hz,
                                                 (float)deadband_hz,
                                                 1U, NULL);
    g_measurement.oil_measurement.follow_frequency = saved_follow_frequency;
    return ret;
}
/* 读取一次并以整数 Hz 返回。 */
/* 这里的循环是业务层“等待有效频率”，不是底层串口通信重试； */
/* 真正的通信重试统一收敛在各协议层，所有 UART6 传感器/无线协议统一使用 UART6_COMM_MAX_RETRY。 */
/* 如果频率连续 3 次为 0 或大于 6500Hz，且电机静止，则上行 1mm 后切密度/液位模式恢复； */
/* 若多轮恢复后仍无有效频率，则返回 SONIC_FREQ_ABNORMAL。 */

/*
 * 函数用途：综合显示状态和TMC5130运动状态判断电机是否已经停止。
 * 调用场景：液位频率连续异常后决定是否允许执行1mm恢复微动。
 * 关键约束：会访问电机驱动，只能在线程态调用；读取失败按“仍在运动”处理。
 */
uint32_t FrequencyLevel_GetCompatThresholdHz(uint32_t raw_threshold)
{
    if (raw_threshold == 0U) {
        return 0U;
    }
    return (raw_threshold + (DENSITY_PARAM_MIGRATE_FACTOR / 2U)) / DENSITY_PARAM_MIGRATE_FACTOR;
}

/**
 * @brief 取得频率液位闭环使用的 Hz 死区，并为未配置值选用默认死区。
 *
 * @param raw_threshold 兼容参数中的频率阈值 x10 定点值，单位 0.1 Hz；0 时使用默认频率死区。
 * @return 兼容阈值换算结果为 0 时返回默认死区 15.0 Hz；否则以 float 返回换算后的整数 Hz 阈值。
 */
static float FrequencyLevel_GetDeadband(uint32_t raw_threshold)
{
    uint32_t threshold_hz = FrequencyLevel_GetCompatThresholdHz(raw_threshold);

    if (threshold_hz == 0U) {
        return FREQUENCY_LEVEL_DEFAULT_DEADBAND_HZ;
    }
    return (float)threshold_hz;
}

/**
 * @brief 根据频率偏差按比例计算速度模式速度，速度受默认运行速度上限保护。
 *
 * @param frequency_error 当前频率减跟随目标频率得到的有符号偏差，单位 Hz；函数按绝对值计算速度。
 * @param deadband 无需运动的频率死区半宽，单位 Hz。
 * @param max_speed_x100 本次允许的速度上限，单位 0.01 m/min；0 表示改用电机默认速度。
 * @return 频率偏差绝对值未超过 deadband 时返回 0；否则返回比例计算并限制后的速度，单位 0.01 m/min，范围不低于最小速度且不高于有效上限。
 */
static uint32_t FrequencyLevel_ComputeSpeedX100(float frequency_error, float deadband, uint32_t max_speed_x100)
{
    float abs_error;
    float speed_f;
    uint32_t speed_x100;
    uint32_t effective_max_speed = max_speed_x100;

    if (effective_max_speed == 0U) {
        effective_max_speed = MotorCtrl_GetDefaultSpeedX100();
    }
    if (effective_max_speed < FREQUENCY_LEVEL_MIN_SPEED_X100) {
        effective_max_speed = FREQUENCY_LEVEL_MIN_SPEED_X100;
    }

    abs_error = fabsf(frequency_error);
    if (abs_error <= deadband) {
        return 0U;
    }

    speed_f = (float)FREQUENCY_LEVEL_MIN_SPEED_X100 +
              (FREQUENCY_LEVEL_KP_SPEED_X100_PER_HZ * (abs_error - deadband));
    if (speed_f < (float)FREQUENCY_LEVEL_MIN_SPEED_X100) {
        speed_f = (float)FREQUENCY_LEVEL_MIN_SPEED_X100;
    }
    if (speed_f > (float)effective_max_speed) {
        speed_f = (float)effective_max_speed;
    }

    speed_x100 = (uint32_t)(speed_f + 0.5f);
    if (speed_x100 == 0U) {
        speed_x100 = FREQUENCY_LEVEL_MIN_SPEED_X100;
    }
    return speed_x100;
}

/**
 * @brief 根据频差计算方法5首次换向后的精细连续速度。
 *
 * @param frequency_error 当前频率减目标频率的有符号偏差，单位 Hz。
 * @param deadband 停机死区半宽，单位 Hz。
 * @return 死区内返回0，死区外返回按频差比例计算并限制后的速度，单位 m/min。
 */
static float FrequencyLevel_ComputeFineSpeedMMin(float frequency_error, float deadband)
{
    float abs_error = fabsf(frequency_error);
    float error_ratio;
    float speed_m_min;

    if (abs_error <= deadband) {
        return 0.0f;
    }

    error_ratio = (abs_error - deadband) / FREQUENCY_LEVEL_FINE_FULL_SPEED_ERROR_HZ;
    if (error_ratio > 1.0f) {
        error_ratio = 1.0f;
    }
    speed_m_min = FREQUENCY_LEVEL_FINE_MIN_SPEED_M_MIN +
                  (FREQUENCY_LEVEL_FINE_MAX_SPEED_M_MIN -
                   FREQUENCY_LEVEL_FINE_MIN_SPEED_M_MIN) *
                  error_ratio * error_ratio;
    if (speed_m_min < FREQUENCY_LEVEL_FINE_MIN_SPEED_M_MIN) {
        speed_m_min = FREQUENCY_LEVEL_FINE_MIN_SPEED_M_MIN;
    }
    if (speed_m_min > FREQUENCY_LEVEL_FINE_MAX_SPEED_M_MIN) {
        speed_m_min = FREQUENCY_LEVEL_FINE_MAX_SPEED_M_MIN;
    }
    return speed_m_min;
}

/**
 * @brief 启动或更新方法5的精细连续速度，换向前先慢停。
 *
 * @param dir 本轮运动方向。
 * @param speed_m_min 本轮精细速度，单位 m/min；0表示停止。
 * @param active_dir 当前实际运动方向输出。
 * @param active_speed_m_min 当前实际精细速度输出，单位 m/min。
 * @return NO_ERROR表示命令执行成功，其他值为停机或电机控制错误。
 */
static uint32_t FrequencyLevel_StartOrUpdateFineMotion(int dir,
                                                       float speed_m_min,
                                                       int *active_dir,
                                                       float *active_speed_m_min)
{
    uint32_t ret;

    if ((active_dir == NULL) || (active_speed_m_min == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    if (speed_m_min <= 0.0f) {
        if (*active_dir != OIL_LEVEL_DIRECTION_NONE) {
            ret = MotorCtrl_SlowStop();
            if (ret != NO_ERROR) {
                return ret;
            }
        }
        *active_dir = OIL_LEVEL_DIRECTION_NONE;
        *active_speed_m_min = 0.0f;
        return NO_ERROR;
    }

    if ((*active_dir != OIL_LEVEL_DIRECTION_NONE) && (*active_dir != dir)) {
        ret = MotorCtrl_SlowStop();
        if (ret != NO_ERROR) {
            return ret;
        }
        *active_dir = OIL_LEVEL_DIRECTION_NONE;
        *active_speed_m_min = 0.0f;
    }

    if ((*active_dir == dir) &&
        (fabsf(speed_m_min - *active_speed_m_min) < FREQUENCY_LEVEL_FINE_SPEED_EPS_M_MIN)) {
        return NO_ERROR;
    }

    ret = MotorCtrl_StartFineVelocity(dir, speed_m_min);
    if (ret != NO_ERROR) {
        return ret;
    }

    *active_dir = dir;
    *active_speed_m_min = speed_m_min;
    return NO_ERROR;
}

/**
 * @brief 判断频率是否已进入稳定区，相对频率法额外避开空气/油中端点。
 *
 * @param frequency_error 频率故障。
 * @param deadband 目标判定使用的死区宽度，与函数处理的频率或密度值采用相同单位。
 * @return 1 表示频率已进入稳定区，相对频率法额外避开空气/油中端点；0 表示频率尚未进入稳定区，相对频率法额外避开空气/油中端点。
 */
static uint8_t FrequencyLevel_IsStableInsideBand(float frequency_error, float deadband,
                                               const OilLevelRefreshSearch *search)
{
    if (fabsf(frequency_error) > deadband) {
        return 0U;
    }

    if (g_deviceParams.liquidLevelMeasurementMethod == OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ) {
        float air_frequency = (float)((search != NULL) ? search->reference.air_frequency : g_measurement.oil_measurement.air_frequency);
        float oil_frequency = (float)((search != NULL) ? search->reference.oil_frequency : g_measurement.oil_measurement.oil_frequency);
        float current_frequency = (float)g_measurement.oil_measurement.current_frequency;

        if ((air_frequency > oil_frequency) &&
            ((air_frequency - oil_frequency) > (2.0f * FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ))) {
            if (((air_frequency - current_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ) ||
                ((current_frequency - oil_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ)) {
                return 0U;
            }
        }
    }

    return 1U;
}

/**
 * @brief 相对频率法在目标死区内但靠近端点时补充运动方向。
 *
 * @param dir 前级频率比较得到的运动方向；OIL_LEVEL_DIRECTION_NONE 表示已经落入目标死区。
 * @return 通常原样返回 dir；仅在连续相对频率法已进入死区且当前频率靠近空气端或油端时，分别修正为下行或上行方向。
 */
static int FrequencyLevel_CorrectRelativeEndpointDirection(int dir, const OilLevelRefreshSearch *search)
{
    if ((dir == OIL_LEVEL_DIRECTION_NONE) &&
        (g_deviceParams.liquidLevelMeasurementMethod == OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ)) {
        float air_frequency = (float)((search != NULL) ? search->reference.air_frequency : g_measurement.oil_measurement.air_frequency);
        float oil_frequency = (float)((search != NULL) ? search->reference.oil_frequency : g_measurement.oil_measurement.oil_frequency);
        float current_frequency = (float)g_measurement.oil_measurement.current_frequency;

        if ((air_frequency > oil_frequency) &&
            ((air_frequency - oil_frequency) > (2.0f * FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ))) {
            if ((air_frequency - current_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ) {
                dir = MOTOR_DIRECTION_DOWN;
            } else if ((current_frequency - oil_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ) {
                dir = MOTOR_DIRECTION_UP;
            }
        }
    }

    return dir;
}



/**
 * @brief 记录频率闭环确认到的液位，并同步外部协议稳定标志。
 *
 * @param tag 用于区分诊断来源的只读标签文字。
 */
static void FrequencyLevel_RecordCurrentPosition(const char *tag)
{
    OilLevelRuntime_RecordCurrentPosition(tag, OIL_LEVEL_RUNTIME_DOMAIN_FREQUENCY);
}

/**
 * @brief 使用已解析的生产目标执行相对频率连续闭环。
 *
 * @param follow_mode 0 表示首次搜索，非 0 表示持续跟随。
 * @return 频率闭环的真实结果码。
 */
uint32_t FrequencyLevel_RunClosedLoop(uint32_t follow_mode)
{
    return FrequencyLevel_RunClosedLoopConfigured(
            follow_mode,
            g_measurement.oil_measurement.follow_frequency,
            0.0f,
            0U, NULL);
}

/* 候选定位复用生产连续闭环；不改正式目标、不发布中途结果，预算由调用方统一提供。 */
uint32_t FrequencyLevel_SearchReference(OilLevelRefreshSearch *search, bool fixed)
{
    if (search == NULL) { return SYSTEM_CALL_CONDITION_ERROR; }
    return FrequencyLevel_RunClosedLoopConfigured(OIL_LEVEL_CLOSED_LOOP_SEARCH,
            search->reference.target_frequency, 0.0f, fixed ? 1U : 0U, search);
}

/**
 * @brief 在频率闭环运动期间检查位置、称重碰撞和丢步。
 *
 * @param guard_direction 本轮位置和称重保护使用的方向；无方向时只刷新位置。
 * @param check_lost_step 非0表示执行丢步检测；精细速度低于0.03m/min时传0。
 * @param command_direction 当前已经下发并覆盖上一时段的有效运动方向。
 * @param effective_command_speed_m_min 当前运动阶段已下发的有效指令速度，单位m/min。
 * @return NO_ERROR表示可继续；其他值为位置、称重通信/碰撞或丢步错误。
 * @note 保护方向可以是下一步期望方向；丢步检测必须使用当前已下发方向，避免换向前提前切换累计方向。
 */
static uint32_t FrequencyLevel_CheckMotionGuards(int guard_direction,
                                                 uint8_t check_lost_step,
                                                 int command_direction,
                                                 float effective_command_speed_m_min)
{
    uint32_t ret = MotorCtrl_PollRuntimePosition();
    int64_t zero_weight_limit;
    int32_t current_weight;

    if (ret != NO_ERROR) {
        return ret;
    }
    if (((guard_direction == MOTOR_DIRECTION_DOWN) ||
         (guard_direction == OIL_LEVEL_DIRECTION_NONE)) &&
        ((int64_t)g_measurement.debug_data.sensor_position <=
         (int64_t)g_deviceParams.blindZone)) {
        return MEASUREMENT_OILLEVEL_LOW;
    }
    if (((guard_direction == MOTOR_DIRECTION_UP) ||
         (guard_direction == OIL_LEVEL_DIRECTION_NONE)) &&
        (g_deviceParams.tankHeight > 1000U) &&
        ((int64_t)g_measurement.debug_data.sensor_position >=
         ((int64_t)g_deviceParams.tankHeight - 1000LL))) {
        return MEASUREMENT_OILLEVEL_HIGH;
    }
    ret = Weight_CheckCommunicationTimeout();
    if (ret != NO_ERROR) {
        return ret;
    }
    current_weight = (int32_t)weight_parament.current_weight;
    zero_weight_limit = ((int64_t)g_deviceParams.full_weight *
                         (100LL + (int64_t)g_deviceParams.zero_weight_threshold_ratio)) /
                        100LL;
    if (((guard_direction == MOTOR_DIRECTION_UP) ||
         (guard_direction == OIL_LEVEL_DIRECTION_NONE)) &&
        ((int64_t)current_weight >= zero_weight_limit)) {
        return WEIGHT_COLLISION_DETECTED;
    }
    if (((guard_direction == MOTOR_DIRECTION_DOWN) ||
         (guard_direction == OIL_LEVEL_DIRECTION_NONE)) &&
        (current_weight <= (int32_t)g_deviceParams.bottom_weight_threshold)) {
        return WEIGHT_COLLISION_DETECTED;
    }
    if (guard_direction == OIL_LEVEL_DIRECTION_NONE) {
        return NO_ERROR;
    }

    ret = CheckWeightCollision();
    if ((ret != NO_ERROR) || (check_lost_step == 0U)) {
        return ret;
    }
    return MotorCtrl_CheckLostStepAutoTimingWithSpeed(
            g_measurement.debug_data.sensor_position,
            command_direction,
            effective_command_speed_m_min);
}

/**
 * @brief 在运动保护通过后读取一次液位频率。
 *
 * @param active_dir 当前已经下发的运动方向。
 * @param check_lost_step 非0表示本轮执行丢步检测。
 * @param effective_command_speed_m_min 当前运动阶段已下发的有效指令速度，单位m/min。
 * @return NO_ERROR表示保护和频率读取均成功；其他值为安全保护或传感器错误。
 */
static uint32_t FrequencyLevel_ReadCurrentWithGuards(int active_dir,
                                                     uint8_t check_lost_step,
                                                     float effective_command_speed_m_min)
{
    uint32_t ret = NO_ERROR;

    if (active_dir != OIL_LEVEL_DIRECTION_NONE) {
        ret = FrequencyLevel_CheckMotionGuards(active_dir,
                                               check_lost_step,
                                               active_dir,
                                               effective_command_speed_m_min);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
    return OilLevel_ReadFrequencySample(
            &g_measurement.oil_measurement.current_frequency);
}

/**
 * @brief 到达盲区时停止下行；首次搜索退出，持续跟随等待频率要求上行撤回。
 *
 * @param follow_mode 0 表示首次搜索，非 0 表示持续跟随。
 * @param active_dir 当前运动方向输出。
 * @param active_speed_x100 当前运动速度输出。
 * @param stable_count 当前稳定计数输出。
 * @return 首次搜索返回 MEASUREMENT_OILLEVEL_LOW；持续跟随在已停机并可重新闭环时返回 NO_ERROR，其他值为停机、命令切换或传感器错误。
 * @note 等待期间不下行、不发布液位；仅当当前频率低于目标时允许上行离开盲区。
 */
static uint32_t FrequencyLevel_HandleLowLimit(uint32_t follow_mode,
                                              int *active_dir,
                                              uint32_t *active_speed_x100,
                                              uint32_t *stable_count,
                                              float deadband)
{
    uint32_t ret;

    if ((follow_mode == 0U) || (active_dir == NULL) ||
        (active_speed_x100 == NULL) || (stable_count == NULL)) {
        return (follow_mode == 0U) ?
               MEASUREMENT_OILLEVEL_LOW : SYSTEM_CALL_CONDITION_ERROR;
    }
    ret = LevelVelocity_StartOrUpdateMotion(OIL_LEVEL_DIRECTION_NONE,
                                            0U,
                                            active_dir,
                                            active_speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }
    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;
    *stable_count = 0U;
    while (1) {
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        ret = OilLevel_ReadValidatedFrequency(
                &g_measurement.oil_measurement.current_frequency);
        if (ret != NO_ERROR) {
            return ret;
        }
        if (((float)g_measurement.oil_measurement.current_frequency -
             (float)g_measurement.oil_measurement.follow_frequency) <
            -deadband) {
            return NO_ERROR;
        }
        ret = AbortableDelay_CommandSwitch(FREQUENCY_LEVEL_SAMPLE_DELAY_MS, 50U);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
}

/**
 * @brief 相对频率和固定频率共用的速度模式连续找液位/跟随闭环。
 *
 * 连续相对频率法、连续固定频率法和 LF 串口调试共用本闭环；目标频率由调用方作为本轮局部值传入，不修改设备参数。生产查找使用
 * oilLevelThreshold，生产跟随使用 oilLevelHysteresisThreshold，LF 调试可通过 deadband_override_hz 只覆盖本次查找死区。
 * 每轮从传感器读取当前液位频率，以当前频率减目标频率的偏差决定下行、上行或停止；连续相对频率法在空气端或油端附近会修正方向，避免把端点误判为最终死区。
 * 频率满足死区及端点一致性条件时停止速度运动并累计稳定样本，达到门限后记录当前液位、同步密度分布液位及 AO 过程样本；查找模式随后返回，跟随模式继续监测。
 * 离开稳定区后按频率偏差调速；固定目标首次换向前沿用常规速度，换向后在0.0007~0.10m/min之间按频差二次降速。首次查找到达盲区立即停止并返回低液位，持续跟随在盲区停机等待，只有频率要求上行时才撤回安全区，同时持续检查位置上限、扭力碰撞和适用速度范围内的丢步。
 * 查找模式受 FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS 限制，跟随模式持续运行直至命令切换或故障。所有异常出口都使 AO 过程样本失效并尝试慢停电机。
 *
 * @param follow_mode 0 表示执行一次频率液位查找并在稳定后返回；非 0 表示使用跟随滞回死区持续闭环，直至命令切换或故障。
 * @param target_frequency_hz 本轮闭环目标频率，单位 Hz；函数同时更新公开运行态频率用于诊断和盲区等待。
 * @param deadband_override_hz 大于 0 时覆盖本次闭环死区，单位 Hz；0 表示按查找/跟随模式读取设备参数。
 * @param fixed_target_mode 非 0 表示本轮使用固定目标语义，禁止应用相对频率端点修正。
 * @return 查找模式稳定完成返回 NO_ERROR；STATE_SWITCH
 *         表示被新命令正常打断，其他值区分目标未配置、频率读取、盲区恢复、位置、扭力、丢步、电机控制和查找超时错误；跟随模式正常运行时不主动返回。
 * @note follow_mode 非 0 时函数设计为长期运行，不应把缺少正常返回理解为死循环缺陷；退出由命令切换和安全故障驱动。
 */
static uint32_t FrequencyLevel_RunClosedLoopConfigured(uint32_t follow_mode,
                                                       uint32_t target_frequency_hz,
                                                       float deadband_override_hz,
                                                       uint8_t fixed_target_mode,
                                                       OilLevelRefreshSearch *search)
{
    uint32_t ret;
    uint32_t start_tick;
    uint32_t stable_count = 0U;
    int active_dir = OIL_LEVEL_DIRECTION_NONE;
    uint32_t active_speed_x100 = 0U;
    float active_fine_speed_m_min = 0.0f;
    uint8_t fine_motion_active = 0U;
    uint8_t lost_step_check_active = 0U;
    int initial_dir = OIL_LEVEL_DIRECTION_NONE;
    uint8_t slow_speed_latched = (follow_mode != 0U) ? 1U : 0U;
    float deadband;
    float follow_deadband;
    uint32_t method = (fixed_target_mode != 0U) ?
                      OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ :
                      g_deviceParams.liquidLevelMeasurementMethod;
    const char *method_text = "频率";

    if (target_frequency_hz == 0U) {
        printf("频率找液位\t目标频率未配置\r\n");
        return PARAM_CONFIG_MISSING;
    }
    if (search == NULL) {
        g_measurement.oil_measurement.follow_frequency = target_frequency_hz;
    }

    if (method == OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ) {
        method_text = "连续相对频率";
    } else if (method == OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ) {
        method_text = "连续固定频率";
    }

    deadband = FrequencyLevel_GetDeadband(g_deviceParams.oilLevelThreshold);
    follow_deadband = FrequencyLevel_GetDeadband(g_deviceParams.oilLevelHysteresisThreshold);
    if (follow_mode != 0U) {
        deadband = follow_deadband;
    }
    if (deadband_override_hz > 0.0f) {
        deadband = deadband_override_hz;
    }

    OilLevelRuntime_ClearStableState();

    /* 刷新始终处于液位模式，不重复切模式及附带无预算的模式等待。 */
    ret = (search != NULL) ? NO_ERROR : SensorService_EnableLevelMode();
    if (ret != NO_ERROR) {
        return FrequencyLevel_StopAndReturn(ret, "切换液位模式失败");
    }
    MotorCtrl_LostStepInit();

    printf("频率找液位\t开始闭环\t方法=%s\t模式=%s\t目标频率=%lu Hz\t死区=%.1f Hz\r\n",
           method_text,
           (follow_mode != 0U) ? "跟随" : "查找",
           (unsigned long)g_measurement.oil_measurement.follow_frequency,
           (double)deadband);

    start_tick = HAL_GetTick();
    while (1) {
        float frequency_error;
        uint32_t speed_x100;
        int dir;
        uint8_t previous_motion_active;
        ret = OilLevelReferenceRefresh_Check(search);
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "刷新预算或运行状态退出");
        }
        if (follow_mode != 0U) {
            ret = OilLevelReferenceRefresh_Poll(stable_count >= FREQUENCY_LEVEL_STABLE_COUNT);
            if (ret != NO_ERROR) { return FrequencyLevel_StopAndReturn(ret, "液位定时矫正退出"); }
            target_frequency_hz = g_measurement.oil_measurement.follow_frequency;
        }
        if (HasEffectiveCommandSwitchRequest()) {
            return FrequencyLevel_StopAndReturn(STATE_SWITCH, "命令切换");
        }

        if (fixed_target_mode != 0U) {
            ret = FrequencyLevel_ReadCurrentWithGuards(
                    active_dir,
                    lost_step_check_active,
                    (fine_motion_active != 0U) ?
                    active_fine_speed_m_min : ((float)active_speed_x100 / 100.0f));
            if (ret == MEASUREMENT_OILLEVEL_LOW) {
                ret = FrequencyLevel_HandleLowLimit(follow_mode,
                                                    &active_dir,
                                                    &active_speed_x100,
                                                    &stable_count,
                                                    deadband);
                if (ret == NO_ERROR) {
                    continue;
                }
            }
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "读取频率或运动保护失败");
            }
        } else {
            ret = OilLevel_ReadFrequencySample(
                    &g_measurement.oil_measurement.current_frequency);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "读取液位频率失败");
            }
        }

        if (g_measurement.oil_measurement.current_frequency == 0U) {
            ret = LevelVelocity_StartOrUpdateMotion(OIL_LEVEL_DIRECTION_NONE,
                                                    0U,
                                                    &active_dir,
                                                    &active_speed_x100);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "频率未稳定停机失败");
            }
            fine_motion_active = 0U;
            active_fine_speed_m_min = 0.0f;
            lost_step_check_active = 0U;
            stable_count = 0U;
            OilLevelRuntime_ClearStableState();
            printf("频率找液位\t频率未稳定，停机等待下一样本\r\n");

            if ((follow_mode == 0U) &&
                ((HAL_GetTick() - start_tick) > FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS)) {
                return FrequencyLevel_StopAndReturn(
                        MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT,
                        "频率闭环超时");
            }
            ret = OilLevelReferenceRefresh_Delay(search, FREQUENCY_LEVEL_SAMPLE_DELAY_MS);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "命令切换");
            }
            continue;
        }

        if (g_measurement.oil_measurement.current_frequency > FREQUENCY_LEVEL_VALID_MAX_HZ) {
            ret = LevelVelocity_StartOrUpdateMotion(OIL_LEVEL_DIRECTION_NONE,
                                                    0U,
                                                    &active_dir,
                                                    &active_speed_x100);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "频率越界停机失败");
            }
            fine_motion_active = 0U;
            active_fine_speed_m_min = 0.0f;
            lost_step_check_active = 0U;
            stable_count = 0U;
            OilLevelRuntime_ClearStableState();
            if (search != NULL) {
                return FrequencyLevel_StopAndReturn(SONIC_FREQ_ABNORMAL, "刷新频率越界");
            }
            ret = OilLevel_ReadValidatedFrequencyWithDeadline(
                    &g_measurement.oil_measurement.current_frequency,
                    start_tick,
                    (follow_mode == 0U) ? FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS : 0U);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "液位频率越界恢复失败");
            }
        }

        frequency_error = (float)g_measurement.oil_measurement.current_frequency -
                          (float)target_frequency_hz;
        ret = OilLevelReferenceRefresh_Check(search);
        if (ret != NO_ERROR) { return FrequencyLevel_StopAndReturn(ret, "刷新采样后截止"); }
        printf("频率找液位\t当前频率=%lu Hz\t目标=%lu Hz\t偏差=%.1f Hz\r\n",
               (unsigned long)g_measurement.oil_measurement.current_frequency,
               (unsigned long)target_frequency_hz,
               (double)frequency_error);

        if (frequency_error > deadband) {
            dir = MOTOR_DIRECTION_DOWN;
        } else if (frequency_error < -deadband) {
            dir = MOTOR_DIRECTION_UP;
        } else {
            dir = OIL_LEVEL_DIRECTION_NONE;
        }
        if (fixed_target_mode == 0U) {
            dir = FrequencyLevel_CorrectRelativeEndpointDirection(dir, search);
        }

        if (fixed_target_mode != 0U) {
            ret = FrequencyLevel_CheckMotionGuards(
                    dir,
                    ((active_dir != OIL_LEVEL_DIRECTION_NONE) &&
                     (lost_step_check_active != 0U)) ? 1U : 0U,
                    active_dir,
                    (fine_motion_active != 0U) ?
                    active_fine_speed_m_min : ((float)active_speed_x100 / 100.0f));
            if (ret == MEASUREMENT_OILLEVEL_LOW) {
                ret = FrequencyLevel_HandleLowLimit(follow_mode,
                                                    &active_dir,
                                                    &active_speed_x100,
                                                    &stable_count,
                                                    deadband);
                if (ret == NO_ERROR) {
                    continue;
                }
            }
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "运动安全保护");
            }
        }

        if (((fixed_target_mode != 0U) && (fabsf(frequency_error) <= deadband)) ||
            ((fixed_target_mode == 0U) &&
             (FrequencyLevel_IsStableInsideBand(frequency_error, deadband, search) != 0U))) {
            ret = LevelVelocity_StartOrUpdateMotion(OIL_LEVEL_DIRECTION_NONE, 0U, &active_dir, &active_speed_x100);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "停止确认失败");
            }
            fine_motion_active = 0U;
            active_fine_speed_m_min = 0.0f;
            lost_step_check_active = 0U;
            stable_count++;
            printf("频率找液位\t进入死区\t稳定计数=%lu/%lu\r\n",
                   (unsigned long)stable_count,
                   (unsigned long)FREQUENCY_LEVEL_STABLE_COUNT);
            if (stable_count >= FREQUENCY_LEVEL_STABLE_COUNT) {
                FrequencyLevel_RecordCurrentPosition((follow_mode != 0U) ? "频率跟随" : "频率查找");
                if (follow_mode == 0U) {
                    return NO_ERROR;
                }
                stable_count = FREQUENCY_LEVEL_STABLE_COUNT;
            }
            ret = OilLevelReferenceRefresh_Delay(search, DensityLevel_GetStableDelayMs());
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "命令切换");
            }
            continue;
        }

        stable_count = 0U;
        OilLevelRuntime_ClearStableState();
        /* 固定频率搜索首次跨越目标后进入精细比例调速；换向后速度仍随频差减小。 */
        if (fixed_target_mode != 0U) {
            if (initial_dir == OIL_LEVEL_DIRECTION_NONE) {
                initial_dir = dir;
            } else if (dir != initial_dir) {
                slow_speed_latched = 1U;
            }
        }

        if ((fixed_target_mode != 0U) &&
            (active_dir != OIL_LEVEL_DIRECTION_NONE) &&
            (dir != active_dir) &&
            (lost_step_check_active != 0U)) {
            /* 换向慢停前结束旧速度区间，停机等待时间不按旧指令速度累计。 */
            ret = MotorCtrl_CheckLostStepAutoTimingWithSpeed(
                    g_measurement.debug_data.sensor_position,
                    active_dir,
                    0.0f);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "换向前丢步检测失败");
            }
        }

        previous_motion_active =
            (active_dir != OIL_LEVEL_DIRECTION_NONE) ? 1U : 0U;

        if ((fixed_target_mode != 0U) && (slow_speed_latched != 0U)) {
            float fine_speed_m_min = FrequencyLevel_ComputeFineSpeedMMin(frequency_error, deadband);
            float feedback_move_mm = fine_speed_m_min * 1000.0f *
                                     FREQUENCY_LEVEL_FEEDBACK_WINDOW_S / 60.0f;

            ret = FrequencyLevel_StartOrUpdateFineMotion(dir,
                                                         fine_speed_m_min,
                                                         &active_dir,
                                                         &active_fine_speed_m_min);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "启动精细速度模式失败");
            }
            active_speed_x100 = 0U;
            fine_motion_active = 1U;
            printf("频率找液位\t精细比例闭环\t方向=%s\t速度=%.4f m/min\t4秒理论位移=%.3f mm\r\n",
                   MotorCtrl_DirectionText(dir),
                   (double)fine_speed_m_min,
                   (double)feedback_move_mm);
        } else {
            speed_x100 = FrequencyLevel_ComputeSpeedX100(
                    frequency_error,
                    deadband,
                    MotorCtrl_GetDefaultSpeedX100());
            if (speed_x100 == 0U) {
                speed_x100 = FREQUENCY_LEVEL_MIN_SPEED_X100;
            }
            printf("频率找液位\t速度闭环\t方向=%s\t速度=%.2f m/min\r\n",
                   MotorCtrl_DirectionText(dir),
                   (double)speed_x100 / 100.0);

            ret = LevelVelocity_StartOrUpdateMotion(dir,
                                                    speed_x100,
                                                    &active_dir,
                                                    &active_speed_x100);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "启动速度模式失败");
            }
        }

        if (fixed_target_mode != 0U) {
            uint8_t next_motion_active =
                (active_dir != OIL_LEVEL_DIRECTION_NONE) ? 1U : 0U;
            uint8_t next_lost_step_check_active =
                ((fine_motion_active == 0U) ||
                 (active_fine_speed_m_min >= FREQUENCY_LEVEL_FINE_LOST_STEP_MIN_SPEED_M_MIN)) ?
                1U : 0U;

            if ((next_motion_active != previous_motion_active) ||
                (next_lost_step_check_active != lost_step_check_active)) {
                /* 只在启停或检测启用状态变化时重建窗口；换向和速度变化由方式5检测器累计。 */
                MotorCtrl_LostStepInit();
                printf("频率找液位\t丢步检测新窗口\t方向=%s\t模式=%s\t检测=%s\r\n",
                       MotorCtrl_DirectionText(active_dir),
                       (fine_motion_active != 0U) ? "精细速度" : "常规速度",
                       (next_lost_step_check_active != 0U) ? "启用" : "暂停");
            }
            lost_step_check_active = next_lost_step_check_active;
        }

        if (fixed_target_mode != 0U) {
            ret = FrequencyLevel_CheckMotionGuards(
                    active_dir,
                    lost_step_check_active,
                    active_dir,
                    (fine_motion_active != 0U) ?
                    active_fine_speed_m_min : ((float)active_speed_x100 / 100.0f));
            if (ret == MEASUREMENT_OILLEVEL_LOW) {
                ret = LevelVelocity_StartOrUpdateMotion(OIL_LEVEL_DIRECTION_NONE,
                                                        0U,
                                                        &active_dir,
                                                        &active_speed_x100);
                if (ret != NO_ERROR) {
                    return FrequencyLevel_StopAndReturn(ret, "盲区停机失败");
                }
                if (follow_mode != 0U) {
                    ret = FrequencyLevel_HandleLowLimit(follow_mode,
                                                        &active_dir,
                                                        &active_speed_x100,
                                                        &stable_count,
                                                        deadband);
                    if (ret == NO_ERROR) {
                        continue;
                    }
                } else {
                    ret = MEASUREMENT_OILLEVEL_LOW;
                }
            }
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "运动安全保护");
            }
        } else {
            ret = OilLevel_UpdatePositionAndCheckBounds();
            if ((ret == MEASUREMENT_OILLEVEL_LOW) && (search == NULL)) {
                active_dir = OIL_LEVEL_DIRECTION_NONE;
                active_speed_x100 = 0U;
                ret = OilLevel_WaitForBlindZone();
                if (ret == NO_ERROR) {
                    continue;
                }
            }
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn((uint32_t)ret, "位置越界");
            }
            ret = CheckWeightCollision();
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "扭力碰撞");
            }
            ret = MotorCtrl_CheckLostStepAutoTiming(
                    g_measurement.debug_data.sensor_position);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "丢步检测失败");
            }
        }

        if ((follow_mode == 0U) &&
            ((HAL_GetTick() - start_tick) > FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS)) {
            return FrequencyLevel_StopAndReturn(MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT, "频率闭环超时");
        }

        ret = OilLevelReferenceRefresh_Delay(search, FREQUENCY_LEVEL_SAMPLE_DELAY_MS);
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "命令切换");
        }
    }
}
