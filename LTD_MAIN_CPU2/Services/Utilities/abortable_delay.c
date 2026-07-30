#include "abortable_delay.h"

#include "main.h"
#include "system_parameter.h"

/**
 * @brief 分片执行可打断延时，供测量和传感器稳定等待使用。
 *
 * 每个分片只调用一次 HAL_Delay，分片结束后检查有效命令切换请求；
 * 返回 STATE_SWITCH 时调用方应按正常切换退出，不进入错误日志。
 * 该函数会阻塞当前任务，不允许在中断上下文调用。
 *
 * @param total_ms 本次分片延时或串口接收排空允许占用的总时长，单位 ms。
 * @param step_ms 可打断延时每次检查命令切换的分片时长，单位 ms。
 * @return NO_ERROR 表示 total_ms 已按 step_ms 分片等待完成；任一分片检测到有效新命令时立即返回 STATE_SWITCH。
 */
uint32_t AbortableDelay_CommandSwitch(uint32_t total_ms, uint32_t step_ms)
{
    uint32_t elapsed = 0U;

    if (step_ms == 0U) {
        step_ms = 50U;
    }

    while (elapsed < total_ms) {
        uint32_t wait_ms = step_ms;
        if ((total_ms - elapsed) < wait_ms) {
            wait_ms = total_ms - elapsed;
        }

        /* 分片等待，避免长 HAL_Delay() 期间无法响应新命令。 */
        HAL_Delay(wait_ms);
        elapsed += wait_ms;

        /* 只把有效命令切换作为正常打断返回，不在延时函数里打印错误。 */
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
    }

    return NO_ERROR;
}