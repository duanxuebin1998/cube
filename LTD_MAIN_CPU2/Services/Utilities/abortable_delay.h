#ifndef ABORTABLE_DELAY_H_
/* ABORTABLE_DELAY_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define ABORTABLE_DELAY_H_

#include <stdint.h>

/**
 * @brief 可被命令切换打断的分片延时。
 *
 * 长时间等待必须调用该函数，避免业务流程在等待稳定、等待模式切换等阶段无法响应新命令。
 * 返回 STATE_SWITCH 时调用方应按正常状态切换退出，不作为故障打印。
 * 该函数会阻塞当前任务，不允许在中断上下文调用。
 *
 * @param total_ms 总等待时间，单位 ms。
 * @param step_ms 单次 HAL_Delay 分片时间，传 0 时默认 50ms。
 * @return NO_ERROR 表示完整等待结束；STATE_SWITCH 表示检测到命令切换。
 */
uint32_t AbortableDelay_CommandSwitch(uint32_t total_ms, uint32_t step_ms);

#endif /* ABORTABLE_DELAY_H_ */