#ifndef INC_FAULT_RECOVERY_H_
/* INC_FAULT_RECOVERY_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_FAULT_RECOVERY_H_

#include <stdint.h>
#include "system_parameter.h"

typedef struct {
    uint8_t handled;              /* 本轮主循环是否已被恢复模块接管，用于阻止空闲错误兜底抢先处理。 */
    uint8_t should_retry_command; /* 设备恢复成功后置 1，主循环据此重跑原始命令。 */
    CommandType retry_command;    /* 需要重跑的原始命令；恢复模块只返回命令，不直接调用测量入口。 */
} FaultRecoveryResult;

/**
 * @brief 命令执行结束后更新自动恢复状态。
 *
 * 该函数只记录恢复上下文，不会立即重跑命令；恢复检查由 FaultRecovery_Poll() 周期触发。
 * @param command 刚执行完的测量命令。
 */
void FaultRecovery_UpdateAfterCommand(CommandType command);

/**
 * @brief 因新命令或命令切换取消当前自动恢复。
 *
 * 取消恢复是用户主动切换流程，不写错误日志，只清理恢复上下文并打印普通提示。
 * @param reason 调用侧语义说明，当前仅保留给调试阅读。
 */
void FaultRecovery_Cancel(const char *reason);

/**
 * @brief 周期执行自动恢复检查。
 *
 * 自动恢复按原始失败命令决定是否继续；本函数只确认设备是否恢复，
 * 恢复成功时通过返回值通知主循环重跑命令，不直接调用测量命令入口。
 * @return handled=1 表示本轮主循环已处理恢复逻辑；should_retry_command=1 表示需要重跑 retry_command。
 */
FaultRecoveryResult FaultRecovery_Poll(void);

/**
 * @brief 查询自动故障恢复是否正在占用设备恢复流程。
 *
 * @details 调用场景：CPU2持久参数最终写门禁判断错误态是否真正空闲。
 * @note 关键约束：只读返回恢复上下文，不清故障、不取消恢复、不触发重试。
 */
bool FaultRecovery_IsActive(void);

#endif /* INC_FAULT_RECOVERY_H_ */