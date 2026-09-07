/*
 * 文件职责：为所有液位算法提供共享的运行时副作用处理。
 * 本接口统一负责结果发布、AO 发布和停机处理；找液位策略及电机控制
 * 仍由各算法模块负责。
 */
#ifndef INC_OIL_LEVEL_RUNTIME_H_
#define INC_OIL_LEVEL_RUNTIME_H_

#include <stdint.h>
#include "oil_level_refresh_policy.h"

/* 标识液位结果来自哪一类算法，用于统一发布和诊断日志，不改变测量策略。 */
typedef enum {
    OIL_LEVEL_RUNTIME_DOMAIN_GENERIC = 0,   /* 通用传统流程。 */
    OIL_LEVEL_RUNTIME_DOMAIN_DENSITY = 1,   /* 密度闭环流程。 */
    OIL_LEVEL_RUNTIME_DOMAIN_FREQUENCY = 2, /* 频率闭环流程。 */
    OIL_LEVEL_RUNTIME_DOMAIN_SYNC = 3       /* 校准和结果同步流程。 */
} OilLevelRuntimeDomain;

/**
 * @brief 非零退出时撤销液位 AO 样本并执行慢停。
 * @param error_code 需要保持不变的错误码或 STATE_SWITCH。
 * @param reason 可选的诊断原因文本。
 * @param domain 用于诊断标识的算法族。
 * @return 完成统一清理后返回原始 error_code。
 */
uint32_t OilLevelRuntime_StopAndInvalidate(uint32_t error_code,
                                           const char *reason,
                                           OilLevelRuntimeDomain domain);
/** @brief 新一轮找液位开始时撤销上一次液位 AO 样本。 */
void OilLevelRuntime_InvalidateLevelSample(void);
/** @brief 新一轮测量开始前清除对外液位命中和稳定标志。 */
void OilLevelRuntime_ClearStableState(void);
/**
 * @brief 提交一个有符号位置，并发布所有对外可见的结果。
 * @param oil_level 以 0.1 mm 为单位的位置；负值按 0 上报。
 * @param reason 负位置日志使用的可选诊断原因文本。
 */
void OilLevelRuntime_CommitLevel(int32_t oil_level, const char *reason);
/* 主线程候选成功后一次性提交端点、目标和液位；临界区只写快照，AO通信在区外执行。 */
void OilLevelRuntime_CommitReference(int32_t oil_level, const OilLevelRefreshCandidate *reference);
/**
 * @brief 读取当前有效位置、提交结果并输出对应算法族的日志。
 * @param tag 诊断日志使用的可选操作标签。
 * @param domain 日志前缀使用的算法族。
 */
void OilLevelRuntime_RecordCurrentPosition(const char *tag,
                                           OilLevelRuntimeDomain domain);

#endif /* INC_OIL_LEVEL_RUNTIME_H_ */
