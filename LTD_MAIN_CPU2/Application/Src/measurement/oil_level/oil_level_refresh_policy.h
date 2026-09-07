#ifndef INC_OIL_LEVEL_REFRESH_POLICY_H_
#define INC_OIL_LEVEL_REFRESH_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

/* 仅为内部策略，不读写参数、输出或硬件；调用方须在同一业务上下文串行调用。 */
#define OIL_LEVEL_REFRESH_INTERVAL_MIN_MIN (60U)
#define OIL_LEVEL_REFRESH_INTERVAL_MIN_MAX (10080U)
#define OIL_LEVEL_REFRESH_INTERVAL_MIN_DEFAULT (1440U)
#define OIL_LEVEL_REFRESH_RETRY_DELAY_MS (300000U)
#define OIL_LEVEL_REFRESH_FAILED_ROUNDS_MAX (3U)
/* 2026-09-07确认的台架初值，尚不代表现场最大允许失联时间。 */
#define OIL_LEVEL_REFRESH_TOTAL_MS (300000U)
#define OIL_LEVEL_REFRESH_HOLD_MS (300000U)
#define OIL_LEVEL_REFRESH_ROLLBACK_MS (120000U)
#define OIL_LEVEL_REFRESH_ATTEMPTS_MAX (3U)

typedef struct {
    bool following;
    bool stable;
    bool motor_stopped;
    bool fault_free;
} OilLevelRefreshWindow;

typedef struct {
    uint32_t interval_ms;
    uint32_t anchor_tick;
    uint32_t retry_tick;
    uint32_t failed_rounds;
    bool enabled;
    bool armed;
    bool pending;
    bool active;
    bool retry_wait;
    bool exhausted;
} OilLevelRefreshSchedule;

typedef enum {
    OIL_LEVEL_REFRESH_FAILURE_NOT_ACTIVE = 0,
    OIL_LEVEL_REFRESH_FAILURE_DEFERRED,
    OIL_LEVEL_REFRESH_FAILURE_EXHAUSTED
} OilLevelRefreshFailureResult;

typedef struct {
    uint32_t air_frequency;
    uint32_t oil_frequency;
    uint32_t target_frequency;
} OilLevelRefreshCandidate;

typedef enum {
    OIL_LEVEL_REFRESH_CANDIDATE_OK = 0,
    OIL_LEVEL_REFRESH_CANDIDATE_BAD_ARGUMENT,
    OIL_LEVEL_REFRESH_CANDIDATE_NOT_FREQUENCY_METHOD,
    OIL_LEVEL_REFRESH_CANDIDATE_OUT_OF_RANGE,
    OIL_LEVEL_REFRESH_CANDIDATE_BAD_ORDER,
    OIL_LEVEL_REFRESH_CANDIDATE_SMALL_GAP,
    OIL_LEVEL_REFRESH_CANDIDATE_TARGET_OUTSIDE
} OilLevelRefreshCandidateResult;

/* 时限必须由已确认的产品策略传入；本模块不提供未经确认的保持/回退默认值。 */
typedef struct {
    uint32_t total_ms;
    uint32_t hold_ms;
    uint32_t rollback_ms;
} OilLevelRefreshLimits;

typedef struct {
    OilLevelRefreshLimits limits;
    uint32_t start_tick;
    uint32_t hold_tick;
    uint32_t rollback_tick;
    bool started;
    bool holding;
    bool rolling_back;
} OilLevelRefreshBudget;

typedef enum {
    OIL_LEVEL_REFRESH_BUDGET_INVALID = 0,
    OIL_LEVEL_REFRESH_BUDGET_CONTINUE,
    OIL_LEVEL_REFRESH_BUDGET_RETURN_REQUIRED,
    OIL_LEVEL_REFRESH_BUDGET_TOTAL_EXPIRED,
    OIL_LEVEL_REFRESH_BUDGET_HOLD_EXPIRED,
    OIL_LEVEL_REFRESH_BUDGET_ROLLBACK_EXPIRED
} OilLevelRefreshBudgetResult;

/* 上电或明确重置周期时初始化；失败清空状态，不支持运行中隐式改参数。 */
bool OilLevelRefresh_InitSchedule(OilLevelRefreshSchedule *schedule,
                                  uint32_t enabled, uint32_t interval_min);
/* 首次稳定后计时，到期仅锁存请求；中断中不得据此执行采样或运动。 */
bool OilLevelRefresh_PollReady(OilLevelRefreshSchedule *schedule, uint32_t now,
                               const OilLevelRefreshWindow *window);
/* 主流程安全窗口内消费请求；运行期间仍保留pending，防止中断丢失。 */
bool OilLevelRefresh_Begin(OilLevelRefreshSchedule *schedule, uint32_t now,
                           const OilLevelRefreshWindow *window);
/* 全部候选和稳定确认成功后调用；回退成功不能调用本接口。 */
void OilLevelRefresh_CompleteSuccess(OilLevelRefreshSchedule *schedule, uint32_t now);
/* 仅有效稳定液位恢复后调用；负责轮间5分钟等待及连续3轮失败计数。 */
OilLevelRefreshFailureResult OilLevelRefresh_DeferAfterRecovery(
        OilLevelRefreshSchedule *schedule, uint32_t now);
/* 正常命令切换只取消执行，保留请求和失败计数；不决定改配置后的重入策略。 */
void OilLevelRefresh_CancelActive(OilLevelRefreshSchedule *schedule);

/* 只处理有效采样；未稳定值须由调用方先跳过。失败不改候选或任何全局基准。 */
OilLevelRefreshCandidateResult OilLevelRefresh_BuildFrequencyCandidate(
        uint32_t method, uint32_t air, uint32_t oil, uint32_t fixed_target,
        OilLevelRefreshCandidate *candidate);

/* 每轮仅初始化一次；全部时限非零且小于半个tick周期，回退须小于保持上限。 */
bool OilLevelRefresh_StartBudget(OilLevelRefreshBudget *budget,
                                 const OilLevelRefreshLimits *limits, uint32_t now);
/* 首次暂停正常液位发布时调用；重复调用不重置保持起点。 */
bool OilLevelRefresh_StartHold(OilLevelRefreshBudget *budget, uint32_t now);
/* 仅已离面并开始保持的流程需要回退；重复调用不重置回退起点。 */
bool OilLevelRefresh_StartRollback(OilLevelRefreshBudget *budget, uint32_t now);
/* 主流程每次等待/采样/运动检查时调用；只返回决策，不停止电机或修改输出。 */
OilLevelRefreshBudgetResult OilLevelRefresh_CheckBudget(
        const OilLevelRefreshBudget *budget, uint32_t now);

#endif /* INC_OIL_LEVEL_REFRESH_POLICY_H_ */
