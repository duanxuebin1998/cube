/**
 * @file oil_level_internal.h
 * @brief oil_level 目录内部共享的闭环常量和跨文件函数契约。
 *
 * 该头文件不再代替各实现文件包含硬件服务。每个 .c 必须显式包含自己使用的
 * MotorControl、Sensor、Weight、参数、编码器、故障或延时接口。
 */

#ifndef INC_OIL_LEVEL_INTERNAL_H_
#define INC_OIL_LEVEL_INTERNAL_H_

#include "measure_oil_level.h"
#include "oil_level_runtime.h"
#include "oil_level_strategy.h"

#include <stdint.h>

/* 停止频率样本无效时的恢复抬升距离，单位 mm。 */
#define OIL_LEVEL_FREQ_RECOVERY_LIFT_MM (1.0f)
/* 传感器切换到密度模式后的稳定等待时间，单位 ms。 */
#define OIL_LEVEL_DENSITY_MODE_SETTLE_MS (3000U)
/* 传统频率跟随单轮允许的最大加速次数。 */
#define MAX_TIMES_WHEN_FRE_FOLLOW (15U)

/** 密度和频率闭环共用的运行目的。 */
typedef enum {
    OIL_LEVEL_CLOSED_LOOP_SEARCH = 0U, /* 找液位：稳定后返回调用方。 */
    OIL_LEVEL_CLOSED_LOOP_FOLLOW = 1U  /* 跟随：持续运行直到命令切换或故障。 */
} OilLevelClosedLoopMode;

/* 未选择上行或下行运动时使用的方向哨兵值。 */
#define OIL_LEVEL_DIRECTION_NONE (-1)
/* 密度闭环允许的最小线速度，单位 0.01 m/min。 */
#define DENSITY_LEVEL_MIN_SPEED_X100 (10U)
/* 密度闭环调整速度时的最小变化量，单位 0.01 m/min。 */
#define DENSITY_LEVEL_SPEED_DELTA_X100 (5U)
/* 密度闭环判稳所需的连续有效样本数。 */
#define DENSITY_LEVEL_STABLE_COUNT (3U)
/* 密度连续无效达到该次数后终止闭环。 */
#define DENSITY_LEVEL_INVALID_DENSITY_LIMIT (5U)
/* 密度闭环采样周期，单位 ms。 */
#define DENSITY_LEVEL_SAMPLE_DELAY_MS (200U)
/* 找液位模式的密度闭环超时时间，单位 ms。 */
#define DENSITY_LEVEL_SEARCH_TIMEOUT_MS (600000U)
/* 未配置时使用的密度目标死区，单位 kg/m3。 */
#define DENSITY_LEVEL_DEFAULT_DEADBAND_KGM3 (0.5f)
/* 密度误差到速度的比例系数，单位 0.01 m/min 每 kg/m3。 */
#define DENSITY_LEVEL_KP_SPEED_X100_PER_KGM3 (20.0f)

/* 频率闭环允许的最小常规线速度，单位 0.01 m/min。 */
#define FREQUENCY_LEVEL_MIN_SPEED_X100 (10U)
/* 频率细调允许的最小线速度，单位 m/min。 */
#define FREQUENCY_LEVEL_FINE_MIN_SPEED_M_MIN (0.0007f)
/* 频率细调允许的最大线速度，单位 m/min。 */
#define FREQUENCY_LEVEL_FINE_MAX_SPEED_M_MIN (0.1000f)
/* 频率误差达到该值时使用细调最大速度，单位 Hz。 */
#define FREQUENCY_LEVEL_FINE_FULL_SPEED_ERROR_HZ (450.0f)
/* 细调速度不低于该值时才启用丢步检测，单位 m/min。 */
#define FREQUENCY_LEVEL_FINE_LOST_STEP_MIN_SPEED_M_MIN (0.0300f)
/* 判断细调速度未变化时使用的浮点容差，单位 m/min。 */
#define FREQUENCY_LEVEL_FINE_SPEED_EPS_M_MIN (0.0000005f)
/* 频率闭环位置反馈估算窗口，单位 s。 */
#define FREQUENCY_LEVEL_FEEDBACK_WINDOW_S (4.0f)
/* 频率闭环判稳所需的连续有效样本数。 */
#define FREQUENCY_LEVEL_STABLE_COUNT (3U)
/* 频率闭环采样周期，单位 ms。 */
#define FREQUENCY_LEVEL_SAMPLE_DELAY_MS (200U)
/* 找液位模式的频率闭环超时时间，单位 ms。 */
#define FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS (600000U)
/* 未配置时使用的频率目标死区，单位 Hz。 */
#define FREQUENCY_LEVEL_DEFAULT_DEADBAND_HZ (15.0f)
/* 频率误差到常规速度的比例系数，单位 0.01 m/min 每 Hz。 */
#define FREQUENCY_LEVEL_KP_SPEED_X100_PER_HZ (0.10f)
/* 相对频率闭环远离油/空气端点的保护边距，单位 Hz。 */
#define FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ (200.0f)
/* 液位频率参数允许的最大值，单位 Hz。 */
#define FREQUENCY_LEVEL_VALID_MAX_HZ (6500U)

/* 跟随偏差确认轮询周期，单位 ms。 */
#define OIL_LEVEL_FOLLOW_CONFIRM_POLL_MS (200U)
/* 液位滞后确认参数允许的最大时间，单位 s。 */
#define OIL_LEVEL_HYSTERESIS_TIME_MAX_S (60U)
/* 闭环判稳后额外保持的时间，单位 ms。 */
#define OIL_LEVEL_CLOSED_LOOP_STABLE_DELAY_MS (1000U)

/** 根据设备参数换算跟随偏差确认窗口，单位 ms。 */
uint32_t OilLevel_GetFollowChangeConfirmTimeMs(void);
/** 在确认窗口内持续检查跟随偏差；检测失败时通过 ret_code 返回错误。 */
uint8_t OilLevel_ConfirmFollowDeviation(uint32_t *ret_code);
/** 清除一次找液位使用的临时运行状态。 */
void OilLevel_ResetSearchRuntimeState(void);
/** 打印当前跟随位置和位置源诊断信息。 */
void OilLevel_PrintFollowPositionInfo(void);
/** 尝试停机后返回调用方给出的原错误码。 */
uint32_t OilLevel_StopBeforeReturn(uint32_t error_code, const char *reason);
/** 把当前位置提交为液位结果并输出原因日志。 */
void OilLevel_SyncCurrentPositionToResult(const char *reason);
/** 读取当前位置、发布液位并检查上下边界。 */
uint32_t OilLevel_UpdatePositionAndCheckBounds(void);
/** 在盲区内等待频率越过跟随目标。 */
uint32_t OilLevel_WaitForBlindZone(void);
/** 返回当前频率相对液位目标的有符号差值，单位 Hz。 */
float OilLevel_GetFrequencyDifference(void);
/** 判断当前频率是否位于空气侧。 */
uint8_t OilLevel_IsCurrentFrequencyInAir(void);
/** 判断当前频率是否位于油侧。 */
uint8_t OilLevel_IsCurrentFrequencyInOil(void);
/** 判断电机驱动是否已经停止。 */
uint8_t OilLevel_IsMotorStopped(void);
/** 停机状态下恢复无效的液位频率样本。 */
uint32_t OilLevel_RecoverLevelFrequencyWhenStopped(void);
/** 读取一次原始频率样本，0表示当前未稳定。 */
uint32_t OilLevel_ReadFrequencySample(volatile uint32_t *frequency_out);
/** 读取一个通过范围校验的频率样本。 */
uint32_t OilLevel_ReadValidatedFrequency(volatile uint32_t *frequency_out);
/** 在统一截止时间内读取一个通过范围校验的频率样本，超时值0表示不限制。 */
uint32_t OilLevel_ReadValidatedFrequencyWithDeadline(
    volatile uint32_t *frequency_out,
    uint32_t start_tick,
    uint32_t timeout_ms);
/** 读取一组频率样本并返回平均值。 */
uint32_t OilLevel_ReadAverageFrequency(volatile uint32_t *frequency_out);
/** 带阶段日志和重试的平均频率读取入口。 */
uint32_t OilLevel_ReadAverageFrequencyWithRetry(volatile uint32_t *frequency_out,
                                                const char *stage_text);
/** 按方向和速度启动运动，或更新已经运行的速度。 */
uint32_t LevelVelocity_StartOrUpdateMotion(int dir,
                                           uint32_t speed_x100,
                                           int *active_dir,
                                           uint32_t *active_speed_x100);
/** 执行密度闭环找液位或跟随。 */
uint32_t DensityLevel_RunClosedLoop(uint32_t follow_mode);
/** 返回密度闭环判稳后的保持时间，单位 ms。 */
uint32_t DensityLevel_GetStableDelayMs(void);
/** 执行固定目标频率闭环找液位或跟随。 */
uint32_t FrequencyLevel_RunFixedClosedLoop(uint32_t follow_mode);
/** 执行相对频率闭环找液位或跟随。 */
uint32_t FrequencyLevel_RunClosedLoop(uint32_t follow_mode);
/** 把兼容参数换算为有效频率阈值，单位 Hz。 */
uint32_t FrequencyLevel_GetCompatThresholdHz(uint32_t raw_threshold);
/** 传统步进流程向油侧搜索液位边界。 */
int SearchOil(void);
/** 传统步进流程向空气侧搜索液位边界。 */
int SearchAir(void);
/** 按单位距离频率变化执行传统精找。 */
int SearchOilPrecise(float per_mm_frequency);

#endif /* INC_OIL_LEVEL_INTERNAL_H_ */
