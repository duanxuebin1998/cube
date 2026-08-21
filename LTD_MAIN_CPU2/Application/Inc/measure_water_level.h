/**
 * @file measure_water_level.h
 * @brief 水位慢速搜索、状态翻转定位和持续跟随的公共接口。
 *
 * 位置换算、水区状态、搜索参考和标定命令属于 water_level 目录私有实现，不在此公开。
 */

#ifndef INC_MEASURE_WATER_LEVEL_H_
#define INC_MEASURE_WATER_LEVEL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取并保存空气区零点电容。
 * @return NO_ERROR 表示电容读取和参数保存完成；传感器或存储错误原样返回。
 */
uint32_t read_zero_capacitance(void);

/**
 * @brief 执行零点准备、空气区避让、粗找和精找组成的一次水位搜索。
 * @return NO_ERROR 表示最终水位已发布；命令切换、运动或传感器错误原样返回。
 */
uint32_t SearchWaterLevel(void);


/**
 * @brief 使用普通丢失恢复策略持续跟随水位。
 * @return STATE_SWITCH 表示新命令正常打断；其它返回值为搜索、运动或传感器结果。
 */
uint32_t FollowWaterLevel(void);

/**
 * @brief 通过连续水区状态翻转估算并对齐当前水位。
 * @param stable_win_ms 稳定判定窗口，单位 ms；传入 0 时在形成第二个翻转点后立即完成判定。
 * @return NO_ERROR 表示翻转位置已稳定并完成对齐；其它值为运动或传感器错误。
 */
uint32_t FindWaterLevel_FastByStateFlip_StableExit(uint32_t stable_win_ms);

/**
 * @brief 使用状态翻转丢失恢复策略持续跟随水位。
 * @return STATE_SWITCH 表示新命令正常打断；其它返回值为搜索、运动或传感器结果。
 */
uint32_t FollowWaterLevel_fast(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MEASURE_WATER_LEVEL_H_ */
