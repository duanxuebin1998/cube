#ifndef INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_
/* INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_

#include <stdint.h>

/* 定频找液位配置允许的最大目标频率 6500 Hz；超过该上限的外部参数必须拒绝。 */
#define FIXED_FREQUENCY_LEVEL_MAX_HZ 6500U

/**
 * @brief 打印固定频率找液位的生产目标、首次找液死区和持续跟随死区。
 *
 * 函数用途：打印当前参数解析出的固定频率找液位配置。
 * 关键约束：只读参数，不初始化电机、不启动运动。
 */
void FixedFrequencyLevelSearch_PrintCurrentConfig(void);

/**
 * @brief 解析本次 LF 覆盖值并调用生产方法5固定频率闭环。
 *
 * LF=<目标频率>[,<死区>] 只覆盖本次运行；LF 使用 oilLevelFrequency 和 oilLevelThreshold。
 * 实际方向、速度、稳定判定、盲区、罐高、称重碰撞、丢步、命令切换和超时保护均由方法5生产闭环执行。
 *
 * 函数用途：执行一条已经通过严格校验的 LF 找液位命令。
 * 关键约束：命令覆盖值只对本次运行有效，不写设备参数或 FRAM。
 *
 * @param command 以 NUL 结尾且已通过严格语法校验的 LF 命令；LF=<Hz>[,<死区Hz>] 可覆盖本次目标和死区，LF 或 LF? 不携带覆盖值时使用已配置或默认值。
 * @return 方法5生产闭环返回的真实结果码；NO_ERROR 表示稳定命中，STATE_SWITCH 表示被新命令中断。
 */
uint32_t FixedFrequencyLevelSearch_Run(const uint8_t *command);

#endif /* INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_ */
