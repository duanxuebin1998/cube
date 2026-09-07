#include "oil_level_refresh_policy.h"
#include "oil_level_strategy.h"

#include <stddef.h>
#include <string.h>

/* 主流程提供当前窗口快照；缺失任一条件均不得离开正常跟随。 */
static bool RefreshWindow_IsReady(const OilLevelRefreshWindow *window)
{
    return (window != NULL) && window->following && window->stable &&
           window->motor_stopped && window->fault_free;
}

/* 初始化不读取HAL时钟；首次稳定由PollReady记录，避免上电相位影响。 */
bool OilLevelRefresh_InitSchedule(OilLevelRefreshSchedule *schedule,
                                  uint32_t enabled, uint32_t interval_min)
{
    if (schedule == NULL) {
        return false;
    }
    memset(schedule, 0, sizeof(*schedule));
    if ((enabled > 1U) || (interval_min < OIL_LEVEL_REFRESH_INTERVAL_MIN_MIN) ||
        (interval_min > OIL_LEVEL_REFRESH_INTERVAL_MIN_MAX)) {
        return false;
    }
    schedule->enabled = (enabled != 0U);
    schedule->interval_ms = interval_min * 60000U;
    return true;
}

/* 无符号时间差支持tick回绕；调用间隔不得跨越完整32位tick周期。 */
bool OilLevelRefresh_PollReady(OilLevelRefreshSchedule *schedule, uint32_t now,
                               const OilLevelRefreshWindow *window)
{
    if ((schedule == NULL) || !schedule->enabled || schedule->exhausted) {
        return false;
    }
    if (!schedule->armed) {
        if (RefreshWindow_IsReady(window)) {
            schedule->anchor_tick = now;
            schedule->armed = true;
        }
        return false;
    }
    if ((uint32_t)(now - schedule->anchor_tick) >= schedule->interval_ms) {
        schedule->pending = true;
    }
    if (schedule->retry_wait) {
        if ((uint32_t)(now - schedule->retry_tick) < OIL_LEVEL_REFRESH_RETRY_DELAY_MS) {
            return false;
        }
        schedule->retry_wait = false;
    }
    return schedule->pending && !schedule->active && RefreshWindow_IsReady(window);
}

/* 消费时再次校验窗口，避免仅依赖先前轮询结果启动运动。 */
bool OilLevelRefresh_Begin(OilLevelRefreshSchedule *schedule, uint32_t now,
                           const OilLevelRefreshWindow *window)
{
    if (!OilLevelRefresh_PollReady(schedule, now, window)) {
        return false;
    }
    schedule->active = true;
    return true;
}

/* 仅完整刷新成功才清连续失败，并以成功时刻安排下一周期。 */
void OilLevelRefresh_CompleteSuccess(OilLevelRefreshSchedule *schedule, uint32_t now)
{
    if ((schedule == NULL) || !schedule->active) {
        return;
    }
    schedule->anchor_tick = now;
    schedule->failed_rounds = 0U;
    schedule->armed = true;
    schedule->pending = false;
    schedule->active = false;
    schedule->retry_wait = false;
}

/* 不能在旧基准不适用或真实故障时调用；这些情况由原故障出口处理。 */
OilLevelRefreshFailureResult OilLevelRefresh_DeferAfterRecovery(
        OilLevelRefreshSchedule *schedule, uint32_t now)
{
    if ((schedule == NULL) || !schedule->active) {
        return OIL_LEVEL_REFRESH_FAILURE_NOT_ACTIVE;
    }
    schedule->active = false;
    schedule->pending = true;
    schedule->failed_rounds++;
    if (schedule->failed_rounds >= OIL_LEVEL_REFRESH_FAILED_ROUNDS_MAX) {
        schedule->exhausted = true;
        schedule->retry_wait = false;
        return OIL_LEVEL_REFRESH_FAILURE_EXHAUSTED;
    }
    schedule->retry_tick = now;
    schedule->retry_wait = true;
    return OIL_LEVEL_REFRESH_FAILURE_DEFERRED;
}

/* 取消不统计故障，也不清除已经到期但尚未成功的请求。 */
void OilLevelRefresh_CancelActive(OilLevelRefreshSchedule *schedule)
{
    if (schedule != NULL) {
        schedule->active = false;
    }
}

/* 先验证有序端点，再做差值和边界比较，禁止无符号下溢。 */
OilLevelRefreshCandidateResult OilLevelRefresh_BuildFrequencyCandidate(
        uint32_t method, uint32_t air, uint32_t oil, uint32_t fixed_target,
        OilLevelRefreshCandidate *candidate)
{
    OilLevelRefreshCandidate next;
    bool fixed;

    if (candidate == NULL) {
        return OIL_LEVEL_REFRESH_CANDIDATE_BAD_ARGUMENT;
    }
    switch (method) {
    case OIL_LEVEL_METHOD_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
        fixed = false;
        break;
    case OIL_LEVEL_METHOD_FIXED_FREQ:
    case OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ:
        fixed = true;
        break;
    default:
        return OIL_LEVEL_REFRESH_CANDIDATE_NOT_FREQUENCY_METHOD;
    }
    if ((air < 1U) || (air > 6500U) || (oil < 1U) || (oil > 6500U)) {
        return OIL_LEVEL_REFRESH_CANDIDATE_OUT_OF_RANGE;
    }
    if (air <= oil) {
        return OIL_LEVEL_REFRESH_CANDIDATE_BAD_ORDER;
    }
    if ((air - oil) <= 400U) {
        return OIL_LEVEL_REFRESH_CANDIDATE_SMALL_GAP;
    }
    next.air_frequency = air;
    next.oil_frequency = oil;
    next.target_frequency = fixed ? fixed_target : ((air + oil) / 2U);
    /* 整数Hz差401时中值贴住200Hz边界，不存在满足旧定位算法的稳定整数目标。 */
    if ((next.target_frequency <= (oil + 200U)) ||
        (next.target_frequency >= (air - 200U))) {
        return OIL_LEVEL_REFRESH_CANDIDATE_TARGET_OUTSIDE;
    }
    *candidate = next;
    return OIL_LEVEL_REFRESH_CANDIDATE_OK;
}

/* 不填入产品默认时限；缺失或矛盾配置必须拒绝启动预算。 */
bool OilLevelRefresh_StartBudget(OilLevelRefreshBudget *budget,
                                 const OilLevelRefreshLimits *limits, uint32_t now)
{
    if (budget == NULL) {
        return false;
    }
    memset(budget, 0, sizeof(*budget));
    if ((limits == NULL) || (limits->total_ms == 0U) ||
        (limits->total_ms >= 0x80000000U) || (limits->hold_ms == 0U) ||
        (limits->hold_ms > limits->total_ms) || (limits->rollback_ms == 0U) ||
        (limits->rollback_ms >= limits->hold_ms)) {
        return false;
    }
    budget->limits = *limits;
    budget->start_tick = now;
    budget->started = true;
    return true;
}

/* 返回期和保持期均受整轮上限约束；所有超时判断优先于继续采样。 */
OilLevelRefreshBudgetResult OilLevelRefresh_CheckBudget(
        const OilLevelRefreshBudget *budget, uint32_t now)
{
    uint32_t elapsed;
    uint32_t held = 0U;

    if ((budget == NULL) || !budget->started) {
        return OIL_LEVEL_REFRESH_BUDGET_INVALID;
    }
    elapsed = (uint32_t)(now - budget->start_tick);
    if (elapsed >= budget->limits.total_ms) {
        return OIL_LEVEL_REFRESH_BUDGET_TOTAL_EXPIRED;
    }
    if (budget->holding) {
        held = (uint32_t)(now - budget->hold_tick);
        if (held >= budget->limits.hold_ms) {
            return OIL_LEVEL_REFRESH_BUDGET_HOLD_EXPIRED;
        }
    }
    if (budget->rolling_back) {
        if ((uint32_t)(now - budget->rollback_tick) >= budget->limits.rollback_ms) {
            return OIL_LEVEL_REFRESH_BUDGET_ROLLBACK_EXPIRED;
        }
    } else if (budget->holding &&
               ((elapsed >= (budget->limits.total_ms - budget->limits.rollback_ms)) ||
                (held >= (budget->limits.hold_ms - budget->limits.rollback_ms)))) {
        return OIL_LEVEL_REFRESH_BUDGET_RETURN_REQUIRED;
    }
    return OIL_LEVEL_REFRESH_BUDGET_CONTINUE;
}

/* 仅首次保持记录起点；总预算不足以预留回退时拒绝离面。 */
bool OilLevelRefresh_StartHold(OilLevelRefreshBudget *budget, uint32_t now)
{
    if (OilLevelRefresh_CheckBudget(budget, now) != OIL_LEVEL_REFRESH_BUDGET_CONTINUE) {
        return false;
    }
    if (!budget->holding) {
        if ((uint32_t)(now - budget->start_tick) >=
            (budget->limits.total_ms - budget->limits.rollback_ms)) {
            return false;
        }
        budget->hold_tick = now;
        budget->holding = true;
    }
    return true;
}

/* 回退不会重置整轮或保持截止时间；迟到的回退仍须遵守先到的截止时间。 */
bool OilLevelRefresh_StartRollback(OilLevelRefreshBudget *budget, uint32_t now)
{
    OilLevelRefreshBudgetResult status = OilLevelRefresh_CheckBudget(budget, now);

    if (((status != OIL_LEVEL_REFRESH_BUDGET_CONTINUE) &&
         (status != OIL_LEVEL_REFRESH_BUDGET_RETURN_REQUIRED)) || !budget->holding) {
        return false;
    }
    if (!budget->rolling_back) {
        budget->rolling_back = true;
        budget->rollback_tick = now;
    }
    return true;
}
