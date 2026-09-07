#include "oil_level_reference_refresh.h"
#include "oil_level_internal.h"
#include "abortable_delay.h"
#include "error_log.h"
#include "sensor_service.h"
#include "weight.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static OilLevelRefreshSchedule s_schedule;
static DeviceParameters s_config;
static bool s_config_valid;
static bool s_following;
static bool s_holding;
static uint32_t s_density_start;
static uint32_t s_density_fresh_count;
/* 调试器置1请求单轮；生产协议没有对应命令，实际消费仍通过跟随安全门。 */
volatile bool g_oil_level_refresh_debug_request;

/* 主线程进入跟随时核对完整持久配置，避免停机改参后使用旧方法的请求或失败计数。 */
void OilLevelReferenceRefresh_EnterFollow(void)
{
    /* command是运行态，不是配置变化；故障后重新找到液面进入跟随时允许重新建立周期。 */
    s_config.command = g_deviceParams.command;
    if (!s_config_valid || s_schedule.exhausted ||
        memcmp(&s_config, (const void *)&g_deviceParams, sizeof(s_config)) != 0) {
        memcpy(&s_config, (const void *)&g_deviceParams, sizeof(s_config));
        (void)OilLevelRefresh_InitSchedule(&s_schedule,
                s_config.liquidLevelReferenceRefreshEnable,
                s_config.liquidLevelReferenceRefreshIntervalMin);
        s_config_valid = true;
        g_oil_level_refresh_debug_request = false;
    }
    s_following = true;
    s_holding = false;
    s_density_fresh_count = 0U;
}

/* 任何跟随出口都释放保持；命令取消不算刷新失败，不将候选数据带入下一条任务。 */
uint32_t OilLevelReferenceRefresh_LeaveFollow(uint32_t result)
{
    s_holding = false;
    s_following = false;
    g_oil_level_refresh_debug_request = false;
    OilLevelRefresh_CancelActive(&s_schedule);
    /* 真故障打断后，重新找液位才建立新周期；普通取消仍保留待执行请求。 */
    if ((result != NO_ERROR) && (result != STATE_SWITCH)) {
        s_config_valid = false;
    }
    return result;
}

/* 调试器入口只锁存事件，不读串口、不移动电机，实际执行仍需稳定停机窗口。 */
bool OilLevelReferenceRefresh_DebugRequest(void)
{
    if (!s_following || !s_schedule.enabled || s_schedule.active || s_schedule.exhausted) {
        return false;
    }
    g_oil_level_refresh_debug_request = true;
    return true;
}

/* 结果服务据此抑制刷新中间位置，实时位置及其它测量通道不受影响。 */
bool OilLevelReferenceRefresh_IsHolding(void)
{
    return s_holding;
}

/* 刷新各等待点先处理实际故障和取消，再判断本阶段截止；不生成最终故障日志。 */
uint32_t OilLevelReferenceRefresh_Check(const OilLevelRefreshSearch *search)
{
    if (g_measurement.device_status.error_code != NO_ERROR) {
        return g_measurement.device_status.error_code;
    }
    if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }
    if ((search != NULL) &&
        ((HAL_GetTick() - search->deadline.start_tick) >= search->deadline.timeout_ms)) {
        return OIL_LEVEL_REFRESH_STAGE_EXPIRED;
    }
    return NO_ERROR;
}

/* 刷新等待按50ms检查预算，普通搜索沿用原可打断延时。 */
uint32_t OilLevelReferenceRefresh_Delay(const OilLevelRefreshSearch *search, uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t ret;
    if (search == NULL) {
        return AbortableDelay_CommandSwitch(ms, 50U);
    }
    do {
        uint32_t elapsed;
        ret = OilLevelReferenceRefresh_Check(search);
        if (ret != NO_ERROR) {
            return ret;
        }
        elapsed = HAL_GetTick() - start;
        if (elapsed >= ms) {
            return NO_ERROR;
        }
        ret = AbortableDelay_CommandSwitch((ms - elapsed > 50U) ? 50U : ms - elapsed, 50U);
    } while (ret == NO_ERROR);
    return ret;
}

/* 0是合法未稳定状态，先停机等待；读错透传，预算耗尽不是采样故障，不做额外模式恢复。 */
uint32_t OilLevelReferenceRefresh_Read(const OilLevelRefreshSearch *search,
                                     volatile uint32_t *frequency)
{
    uint32_t ret;
    if (search == NULL) {
        return OilLevel_ReadValidatedFrequency(frequency);
    }
    while (1) {
        ret = OilLevelReferenceRefresh_Check(search);
        if (ret != NO_ERROR) { return ret; }
        ret = OilLevel_ReadFrequencySample(frequency);
        if (ret != NO_ERROR) { return ret; }
        ret = OilLevelReferenceRefresh_Check(search);
        if (ret != NO_ERROR) { return ret; }
        if (*frequency > FREQUENCY_LEVEL_VALID_MAX_HZ) { return SONIC_FREQ_ABNORMAL; }
        if (*frequency != 0U) { return NO_ERROR; }
        ret = MotorCtrl_SlowStop();
        if (ret != NO_ERROR) { return ret; }
        /* 合法未稳定值会暂停运动，恢复后不能把停机等待时间计入丢步窗口。 */
        MotorCtrl_LostStepInit();
        ret = OilLevelReferenceRefresh_Delay(search, 1000U);
        if (ret != NO_ERROR) { return ret; }
    }
}

/* 位移前后保留位置和碰撞保护；这些调用可能已停机，不在这里判断运动速度。 */
static uint32_t RefreshCheckSafety(void)
{
    uint32_t ret = OilLevel_UpdatePositionAndCheckBounds();
    if (ret != NO_ERROR) { return ret; }
    /* 刷新不允许负位置或边界等值；不能由提交函数把负位置截成合法零液位。 */
    if ((int64_t)g_measurement.debug_data.sensor_position <= (int64_t)g_deviceParams.blindZone) {
        return MEASUREMENT_OILLEVEL_LOW;
    }
    return CheckWeightCollision();
}

/* 端点离面100mm沿用原搜索幅度；空间不足立即走边界故障，不削减幅度伪称端点成立。 */
static uint32_t RefreshMoveEndpoint(const OilLevelRefreshSearch *search, int dir)
{
    int64_t target;
    uint32_t ret = RefreshCheckSafety();
    if (ret != NO_ERROR) { return ret; }
    target = (int64_t)g_measurement.debug_data.sensor_position +
            ((dir == MOTOR_DIRECTION_UP) ? 1000LL : -1000LL);
    if (target <= (int64_t)g_deviceParams.blindZone) { return MEASUREMENT_OILLEVEL_LOW; }
    if (target >= (int64_t)g_deviceParams.tankHeight - 1000LL) { return MEASUREMENT_OILLEVEL_HIGH; }
    ret = MotorCtrl_MoveAndWaitUntil(100.0f, dir, MotorCtrl_GetDefaultSpeedX100(), &search->deadline);
    if (ret != NO_ERROR) { return ret; }
    return RefreshCheckSafety();
}

/* 寻相保留配置的安全中间频率，不用实际跟随目标替代；丢步只检查已下发运动的连续段。 */
static uint32_t RefreshSeekPhase(const OilLevelRefreshSearch *search, bool air)
{
    uint32_t sample = 0U;
    uint32_t ret;
    bool motion_started = false;
    /* 每次寻相独立建窗，隔离上一相位的停止、采样和换向。 */
    MotorCtrl_LostStepInit();
    while (1) {
        ret = RefreshCheckSafety();
        if (ret != NO_ERROR) { return ret; }
        if (motion_started) {
            /* 按已下发的运动命令检测，不能因实际不转就跳过，否则会漏检真实堵转。 */
            ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
            if (ret != NO_ERROR) { return ret; }
        }
        ret = OilLevelReferenceRefresh_Read(search, &sample);
        if (ret != NO_ERROR) { return ret; }
        if (air ? (sample > g_deviceParams.oilLevelFrequency) :
                  (sample < g_deviceParams.oilLevelFrequency)) {
            ret = MotorCtrl_SlowStop();
            if (ret != NO_ERROR) { return ret; }
            return RefreshMoveEndpoint(search, air ? MOTOR_DIRECTION_UP : MOTOR_DIRECTION_DOWN);
        }
        ret = air ? MotorCtrl_MoveUp(MotorCtrl_GetDefaultSpeedX100()) :
                    MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());
        if (ret != NO_ERROR) { return ret; }
        motion_started = true;
        ret = OilLevelReferenceRefresh_Delay(search, FREQUENCY_LEVEL_SAMPLE_DELAY_MS);
        if (ret != NO_ERROR) { return ret; }
    }
}

/* 与原端点相同：10个有效样本、2秒间隔、剔除两高两低后平均；不叠加端点重试。 */
static uint32_t RefreshAverage(const OilLevelRefreshSearch *search, uint32_t *result)
{
    uint32_t values[10];
    uint32_t sum = 0U;
    uint32_t ret;
    for (unsigned i = 0U; i < 10U; ++i) {
        ret = OilLevelReferenceRefresh_Read(search, &values[i]);
        if (ret != NO_ERROR) { return ret; }
        for (unsigned j = i; (j > 0U) && (values[j] < values[j - 1U]); --j) {
            uint32_t value = values[j];
            values[j] = values[j - 1U];
            values[j - 1U] = value;
        }
        ret = OilLevelReferenceRefresh_Delay(search, 2000U);
        if (ret != NO_ERROR) { return ret; }
    }
    for (unsigned i = 2U; i < 8U; ++i) { sum += values[i]; }
    *result = (sum + 3U) / 6U;
    return NO_ERROR;
}

/* 调用原方法对应的定位算法，候选基准显式传递，搜索超时统一受本轮截止限制。 */
static uint32_t RefreshLocate(OilLevelRefreshSearch *search)
{
    uint32_t ret = (s_config.liquidLevelMeasurementMethod <= 1U) ?
            OilLevel_SearchPreciseReference(search) :
            FrequencyLevel_SearchReference(search, s_config.liquidLevelMeasurementMethod == 5U);
    if (ret != NO_ERROR) { return ret; }
    ret = RefreshCheckSafety();
    if (ret != NO_ERROR) { return ret; }
    return OilLevelReferenceRefresh_Check(search);
}

/* 成功或有效回退后才恢复发布当前液位；不返回刷新前的机械坐标。 */
static uint32_t RefreshPublish(const OilLevelRefreshCandidate *reference)
{
    s_holding = false;
    OilLevelRuntime_CommitReference(g_measurement.debug_data.sensor_position, reference);
    return g_measurement.device_status.error_code;
}

/* 单轮频率刷新事务：3次含首次共用180秒，失败只允许在剩余总预算内按可用旧目标回退。 */
static uint32_t RefreshFrequencyRound(void)
{
    OilLevelRefreshBudget budget;
    const OilLevelRefreshLimits limits = {
        OIL_LEVEL_REFRESH_TOTAL_MS, OIL_LEVEL_REFRESH_HOLD_MS, OIL_LEVEL_REFRESH_ROLLBACK_MS
    };
    OilLevelRefreshSearch search = {0};
    OilLevelRefreshCandidate old = {
        g_measurement.oil_measurement.air_frequency,
        g_measurement.oil_measurement.oil_frequency,
        g_measurement.oil_measurement.follow_frequency
    };
    OilLevelRefreshCandidate checked;
    uint32_t ret = NO_ERROR;
    bool old_disproved = false;
    const uint32_t method = s_config.liquidLevelMeasurementMethod;
    const uint32_t start = HAL_GetTick();

    if (!OilLevelRefresh_StartBudget(&budget, &limits, start) ||
        !OilLevelRefresh_StartHold(&budget, start)) { return SYSTEM_CALL_CONDITION_ERROR; }
    s_holding = true;
    search.deadline.start_tick = start;
    search.deadline.timeout_ms = OIL_LEVEL_REFRESH_TOTAL_MS - OIL_LEVEL_REFRESH_ROLLBACK_MS;
    printf("液位定时矫正\t开始\t方法=%lu\t轮=%lu\t总计300秒/采样定位180秒/回退最多120秒\r\n",
           (unsigned long)method, (unsigned long)s_schedule.failed_rounds + 1UL);

    for (uint32_t attempt = 1U; attempt <= OIL_LEVEL_REFRESH_ATTEMPTS_MAX; ++attempt) {
        uint32_t air = 0U, oil = 0U;
        ret = RefreshSeekPhase(&search, false);
        if (ret == NO_ERROR) { ret = RefreshAverage(&search, &oil); }
        if (ret == NO_ERROR) { ret = RefreshSeekPhase(&search, true); }
        if (ret == NO_ERROR) { ret = RefreshAverage(&search, &air); }
        if (ret != NO_ERROR) { break; }
        printf("液位定时矫正\t尝试=%lu/3\t空气=%lu\t油中=%lu\t空气变化=%ld\t油中变化=%ld\r\n",
               (unsigned long)attempt, (unsigned long)air, (unsigned long)oil,
               (long)((int32_t)air - (int32_t)old.air_frequency),
               (long)((int32_t)oil - (int32_t)old.oil_frequency));

        /* 可靠的新端点不支持旧目标时，不再凭旧目标停稳恢复有效输出。 */
        if ((air > oil) && ((air - oil) > 400U)) {
            old_disproved = old_disproved || !((old.target_frequency > oil + 200U) &&
                                               (old.target_frequency + 200U < air));
        }
        if (OilLevelRefresh_BuildFrequencyCandidate(method, air, oil,
                s_config.oilLevelFrequency, &search.reference) == OIL_LEVEL_REFRESH_CANDIDATE_OK) {
            ret = RefreshMoveEndpoint(&search, MOTOR_DIRECTION_DOWN);
            if (ret == NO_ERROR) { ret = RefreshLocate(&search); }
            if (ret != NO_ERROR) { break; }
            /* 定位和最后保护检查全部成功，才一次性提升候选基准。 */
            ret = RefreshPublish(&search.reference);
            if (ret != NO_ERROR) { return ret; }
            if ((attempt > 1U) || (s_schedule.failed_rounds != 0U)) {
                /* 打印有效基准重试成功，回退成功本身不打印刷新成功。 */
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE, "液位定时矫正", "有效基准重新定位成功",
                                 attempt, OIL_LEVEL_REFRESH_ATTEMPTS_MAX);
            }
            OilLevelRefresh_CompleteSuccess(&s_schedule, HAL_GetTick());
            printf("液位定时矫正\t完成\t用时=%lums\t目标=%luHz\r\n",
                   (unsigned long)(HAL_GetTick() - start), (unsigned long)search.reference.target_frequency);
            return NO_ERROR;
        }
        /* 打印端点或目标窗口不成立的有界重试，不当作最终故障。 */
        ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE, "液位定时矫正", "端点或目标窗口无效",
                       attempt, OIL_LEVEL_REFRESH_ATTEMPTS_MAX, MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED);
    }

    if ((ret != NO_ERROR) && (ret != OIL_LEVEL_REFRESH_STAGE_EXPIRED)) { return ret; }
    ret = MotorCtrl_SlowStop();
    if (ret != NO_ERROR) { return ret; }
    if (old_disproved ||
        (OilLevelRefresh_BuildFrequencyCandidate(1U, old.air_frequency, old.oil_frequency,
             old.target_frequency, &checked) != OIL_LEVEL_REFRESH_CANDIDATE_OK) ||
        !OilLevelRefresh_StartRollback(&budget, HAL_GetTick())) {
        return MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED;
    }
    search.reference = old;
    search.deadline.start_tick = budget.rollback_tick;
    search.deadline.timeout_ms = OIL_LEVEL_REFRESH_TOTAL_MS - (budget.rollback_tick - start);
    if (search.deadline.timeout_ms > OIL_LEVEL_REFRESH_ROLLBACK_MS) {
        search.deadline.timeout_ms = OIL_LEVEL_REFRESH_ROLLBACK_MS;
    }
    printf("液位定时矫正\t按旧目标回退\t剩余=%lums\r\n", (unsigned long)search.deadline.timeout_ms);
    ret = RefreshLocate(&search);
    if (ret == OIL_LEVEL_REFRESH_STAGE_EXPIRED) { return MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED; }
    if (ret != NO_ERROR) { return ret; }
    if (OilLevelRefresh_DeferAfterRecovery(&s_schedule, HAL_GetTick()) == OIL_LEVEL_REFRESH_FAILURE_EXHAUSTED) {
        return MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED;
    }
    ret = RefreshPublish(NULL);
    printf("液位定时矫正\t有效回退完成，恢复实时跟随\t5分钟后重试\t累计失败=%lu\r\n",
           (unsigned long)s_schedule.failed_rounds);
    return ret;
}

/* 每轮检查到期锁存；不稳定期间仍推进计时，安全门内用驱动真实停机状态确认。 */
static uint32_t RefreshPollWindow(bool stable, OilLevelRefreshWindow *window)
{
    bool moving = true;
    uint32_t ret;
    *window = (OilLevelRefreshWindow){
        s_following && (g_measurement.device_status.device_state == STATE_FLOWOIL),
        stable, false, g_measurement.device_status.error_code == NO_ERROR
    };
    if (!s_schedule.enabled || (s_config.liquidLevelMeasurementMethod == 3U)) { return NO_ERROR; }
    if (stable && window->following && window->fault_free) {
        ret = MotorCtrl_IsDriverMoving(&stepper, &moving);
        if (ret != NO_ERROR) { return ret; }
        window->motor_stopped = !moving;
    }
    (void)OilLevelRefresh_PollReady(&s_schedule, HAL_GetTick(), window);
    if (g_oil_level_refresh_debug_request) {
        g_oil_level_refresh_debug_request = false;
        s_schedule.pending = true;
    }
    return NO_ERROR;
}

/* 频率跟随安全窗口内执行完整事务，普通跟随仍由原算法负责。 */
uint32_t OilLevelReferenceRefresh_Poll(bool stable)
{
    OilLevelRefreshWindow window;
    uint32_t ret = RefreshPollWindow(stable, &window);
    if (ret != NO_ERROR) { return ret; }
    if (OilLevelRefresh_Begin(&s_schedule, HAL_GetTick(), &window)) {
        ret = RefreshFrequencyRound();
        if (ret != NO_ERROR) {
            s_holding = false;
            OilLevelRefresh_CancelActive(&s_schedule);
            return OilLevelRuntime_StopAndInvalidate(ret, "液位定时矫正退出", OIL_LEVEL_RUNTIME_DOMAIN_FREQUENCY);
        }
    }
    return NO_ERROR;
}

/* 密度法不离面、不保持输出；触发读取不计数，后续连续3次稳定读取即可，不额外判断数据新旧。 */
uint32_t OilLevelReferenceRefresh_DensitySample(bool stable, float density,
                                              float frequency, float temperature)
{
    OilLevelRefreshWindow window;
    uint32_t ret = RefreshPollWindow(stable, &window);
    if (ret != NO_ERROR) { return ret; }
    if (!s_schedule.active) {
        if (OilLevelRefresh_Begin(&s_schedule, HAL_GetTick(), &window)) {
            s_density_start = HAL_GetTick();
            s_density_fresh_count = 0U;
        }
        return NO_ERROR;
    }
    if ((HAL_GetTick() - s_density_start) >= OIL_LEVEL_REFRESH_TOTAL_MS) {
        if (!stable || !window.motor_stopped ||
            (OilLevelRefresh_DeferAfterRecovery(&s_schedule, HAL_GetTick()) == OIL_LEVEL_REFRESH_FAILURE_EXHAUSTED)) {
            return MEASUREMENT_LEVEL_REFERENCE_REFRESH_FAILED;
        }
        return NO_ERROR;
    }
    if (!stable || !window.motor_stopped || !isfinite(density) ||
        !isfinite(frequency) || !isfinite(temperature)) {
        s_density_fresh_count = 0U;
        return NO_ERROR;
    }
    if (++s_density_fresh_count >= DENSITY_LEVEL_STABLE_COUNT) {
        printf("液位定时矫正\t密度复核完成\t密度=%.3f\t频率=%.3f\t温度=%.3f\t用时=%lums\r\n",
               (double)density, (double)frequency, (double)temperature,
               (unsigned long)(HAL_GetTick() - s_density_start));
        OilLevelRefresh_CompleteSuccess(&s_schedule, HAL_GetTick());
    }
    return NO_ERROR;
}
