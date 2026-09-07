#ifndef OIL_LEVEL_REFERENCE_REFRESH_H
#define OIL_LEVEL_REFERENCE_REFRESH_H

#include "oil_level_refresh_policy.h"
#include "motor_ctrl.h"

/* 模块内部的预算截止，必须由刷新编排消费，不向共享故障寄存器发布。 */
#define OIL_LEVEL_REFRESH_STAGE_EXPIRED MOTOR_MOVE_DEADLINE_REACHED

typedef struct {
    OilLevelRefreshCandidate reference;
    MotorMoveDeadline deadline;
} OilLevelRefreshSearch;

/* 跟随入口/出口调用；同配置取消保留请求，配置改变重新计时，均不保留候选采样。 */
void OilLevelReferenceRefresh_EnterFollow(void);
uint32_t OilLevelReferenceRefresh_LeaveFollow(uint32_t result);
/* 频率跟随每轮调用；只有稳定且停机才会同步执行刷新。 */
uint32_t OilLevelReferenceRefresh_Poll(bool stable);
/* 密度跟随复核入口；触发当次不计数，后续连续3次稳定读取完成复核，重复缓存数据也计数。 */
uint32_t OilLevelReferenceRefresh_DensitySample(bool stable, float density,
                                              float frequency, float temperature);
/* 台架调试器置1；变量被轮询代码引用，链接裁剪后仍保留，不新增生产协议命令。 */
extern volatile bool g_oil_level_refresh_debug_request;
/* 内部请求辅助接口；生产链接未引用时允许裁剪，台架使用上面的变量。 */
bool OilLevelReferenceRefresh_DebugRequest(void);
/* 仅控制液位结果发布；故障和命令切换不受此标志屏蔽。 */
bool OilLevelReferenceRefresh_IsHolding(void);
/* 有界搜索的共用检查、延时和读样本；空上下文不改变原普通流程。 */
uint32_t OilLevelReferenceRefresh_Check(const OilLevelRefreshSearch *search);
uint32_t OilLevelReferenceRefresh_Delay(const OilLevelRefreshSearch *search, uint32_t ms);
uint32_t OilLevelReferenceRefresh_Read(const OilLevelRefreshSearch *search,
                                    volatile uint32_t *frequency);
/* 原步进/连续定位的候选入口，结果不写正式基准，稳定后由编排提交。 */
uint32_t OilLevel_SearchPreciseReference(OilLevelRefreshSearch *search);
uint32_t FrequencyLevel_SearchReference(OilLevelRefreshSearch *search, bool fixed);

#endif
