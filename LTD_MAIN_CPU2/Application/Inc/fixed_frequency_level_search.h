#ifndef INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_
#define INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_

#include <stdint.h>

#define FIXED_FREQUENCY_LEVEL_MAX_HZ 6500U

/*
 * 函数用途：打印当前参数解析出的固定频率找液位配置。
 * 关键约束：只读参数，不初始化电机、不启动运动。
 */
void FixedFrequencyLevelSearch_PrintCurrentConfig(void);

/*
 * 函数用途：执行一条已经通过严格校验的 LF 找液位命令。
 * 关键约束：命令覆盖值只对本次运行有效，不写设备参数或 FRAM。
 */
uint32_t FixedFrequencyLevelSearch_Run(const uint8_t *command);

#endif /* INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_ */
