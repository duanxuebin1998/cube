#ifndef PART_DIAGNOSTICS_H_
#define PART_DIAGNOSTICS_H_

#include <stdint.h>

/*
 * 函数用途：逐项读取并核对整机部件参数。
 * 调用场景：故障恢复复核和部件参数周期刷新。
 * 关键约束：不调用MeasureStart、不改变当前命令状态。
 */
uint32_t PartDiagnostics_CheckAll(void);

/*
 * 函数用途：执行读取部件参数命令并进入周期刷新。
 * 调用场景：测量命令分发收到CMD_READ_PART_PARAMS时。
 * 关键约束：循环可被新命令打断，错误通过统一SET_ERROR入口传播。
 */
void PartDiagnostics_HandleCommand(void);

#endif /* PART_DIAGNOSTICS_H_ */
