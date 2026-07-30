#ifndef INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_
/* INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_

#include <stdint.h>

/* 定频找液位配置允许的最大目标频率 6500 Hz；超过该上限的外部参数必须拒绝。 */
#define FIXED_FREQUENCY_LEVEL_MAX_HZ 6500U

/**
 * @brief 校验并打印固定频率找液位的目标、死区、位置、扭力、速度和稳定采样配置。
 *
 * 函数用途：打印当前参数解析出的固定频率找液位配置。
 * 关键约束：只读参数，不初始化电机、不启动运动。
 */
void FixedFrequencyLevelSearch_PrintCurrentConfig(void);

/**
 * @brief 按固定目标频率和死区闭环搜索液位。
 *
 * 函数接受已经通过串口语法分类的 LF 命令；LF=<目标频率>[,<死区>] 提供本次覆盖值，未提供的字段由 FixedLevel_BuildConfig
 * 使用设备参数或默认值，并同时建立位置、扭力和最大速度保护边界。
 * 进入液位频率模式后清除旧液位稳定标志并初始化丢步检测；循环内在读取频率前后均检查位置、扭力通信、碰撞和丢步保护，避免一次传感器事务期间越过机械边界。
 * 当前频率落入目标死区时先慢停电机，并要求连续 LF_STABLE_COUNT 次稳定采样；确认后把当前位置同时写入油位结果和密度分布油位，置位探头到液面及液体稳定标志。
 * 频率偏差超出死区时按偏差符号选择上行或下行，按偏差量计算并限制 0.01 m/min 速度；方向反转前先慢停，再启动新的速度方向。
 * 命令切换、总超时、传感器读取、位置、扭力、碰撞、丢步或电机控制失败均进入统一 stop 出口，清除稳定标志并尝试慢停，禁止异常返回后继续保持速度模式。
 *
 * 函数用途：执行一条已经通过严格校验的 LF 找液位命令。
 * 关键约束：命令覆盖值只对本次运行有效，不写设备参数或 FRAM。
 *
 * @param command 以 NUL 结尾且已通过严格语法校验的 LF 命令；LF=<Hz>[,<死区Hz>] 可覆盖本次目标和死区，LF 或 LF? 不携带覆盖值时使用已配置或默认值。
 * @return NO_ERROR 表示频率连续稳定达到门限、当前位置已发布且电机已停止；STATE_SWITCH
 *         表示被新命令中断，其他值区分配置或范围、总超时、传感器、位置、扭力、丢步和电机慢停错误。
 * @note 除稳定成功直接返回外，所有退出都通过 FixedLevel_Stop；若原错误为 NO_ERROR 而慢停失败，则返回慢停错误，否则保留最先发生的业务错误。
 */
uint32_t FixedFrequencyLevelSearch_Run(const uint8_t *command);

#endif /* INC_FIXED_FREQUENCY_LEVEL_SEARCH_H_ */
