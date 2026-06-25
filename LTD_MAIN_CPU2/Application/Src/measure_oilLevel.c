/*
 * measure_findOil.c
 * 液位测量核心模块 - 通过密度传感器频率寻找/跟踪液位
 *
 * 核心功能：
 * 1. 通过频率特征识别液位位置
 * 2. 动态调整电机位置使传感器稳定在液位界面
 * 3. 处理液位过低/过高/振动管角度异常等边界情况
 * 4. 实现液位跟随算法
 *
 * 设计要点：
 * - 基于密度传感器在空气/油中频率差异原理（空气约6500Hz，油约4500Hz）
 * - 采用自适应步长控制策略：小偏差小步进，大偏差加速移动
 * - 多重保护机制：电机丢步检测、传感器倾斜保护、盲区处理、死循环预防
 */

#include "main.h"
#include "measure_oilLevel.h"
#include "motor_ctrl.h"
#include "sensor.h"
#include "weight.h"
#include "measure_zero.h"
#include "system_parameter.h"
#include "encoder.h"
#include "error_log.h"
#include "abortable_delay.h"
#include "AoOutput/ao_output.h"
#include "fault_manager.h"
#include <math.h>

/* 函数原型声明 */
static int SearchOil();   /* 粗略搜索液位 */
static int SearchAir(); /* 精确搜索液位 */
static int SearchOilPrecise(float per_mm_Frequency);
static int determineTheSensorPositionAndUpdateTheLevelValue(void);
static int waitForTheLiquidLevelToExceedTheBlindZone(void);
static uint32_t determine_level_status_internal(Level_StateTypeDef *state_out, uint8_t allow_mode_recovery);
static uint32_t OilLevel_StopBeforeReturn(uint32_t error_code, const char *reason);
static uint32_t OilLevel_ClampLevelForReport(int32_t oil_level, const char *reason);
static void OilLevel_UpdateAoOutput(void);
static void OilLevel_SyncCurrentPositionToResult(const char *reason);

#define OIL_LEVEL_METHOD_RELATIVE_FREQ 0U /* 液位测量方法枚举值：油品 液位 METHOD RELATIVE 频率。 */
#define OIL_LEVEL_METHOD_FIXED_FREQ    1U /* 液位测量方法枚举值：油品 液位 METHOD FIXED 频率。 */
#define OIL_LEVEL_METHOD_DENSITY       2U /* 液位测量方法枚举值：油品 液位 METHOD 密度。 */
#define OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ 4U /* 液位测量方法枚举值：油品 液位 METHOD CONTINUOUS RELATIVE 频率。 */
#define OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ    5U /* 液位测量方法枚举值：油品 液位 METHOD CONTINUOUS FIXED 频率。 */

#define DENSITY_LEVEL_RUN_SEARCH              0U /* 密度法液位控制参数：运行 搜索。 */
#define DENSITY_LEVEL_RUN_FOLLOW              1U /* 密度法液位控制参数：运行 跟随。 */
#define DENSITY_LEVEL_DIR_NONE                (-1) /* 密度法液位控制参数：方向 无。 */
#define DENSITY_LEVEL_MIN_SPEED_X100          10U /* 密度法液位控制参数：最小值 SPEED 放大 100 倍。 */
#define DENSITY_LEVEL_SPEED_DELTA_X100        5U /* 密度法液位控制参数：SPEED 变化量 放大 100 倍。 */
#define DENSITY_LEVEL_STABLE_COUNT            3U /* 密度法液位控制参数：稳定 数量。 */
#define DENSITY_LEVEL_INVALID_DENSITY_LIMIT   5U /* 密度法液位控制参数：无效 密度 限值。 */
#define DENSITY_LEVEL_SAMPLE_DELAY_MS         200U /* 密度法液位控制参数：SAMPLE 延时 毫秒。 */
#define DENSITY_LEVEL_SEARCH_TIMEOUT_MS       600000U /* 密度法液位控制参数：搜索 超时 毫秒。 */
#define DENSITY_LEVEL_DEFAULT_DEADBAND_KGM3   0.5f /* 密度法液位跟随默认死区，单位 kg/m3。 */
#define DENSITY_LEVEL_KP_SPEED_X100_PER_KGM3  20.0f /* 密度法液位跟随比例速度系数，单位 0.01 速度每 kg/m3。 */

#define FREQUENCY_LEVEL_RUN_SEARCH              0U /* 频率法液位控制参数：运行 搜索。 */
#define FREQUENCY_LEVEL_RUN_FOLLOW              1U /* 频率法液位控制参数：运行 跟随。 */
#define FREQUENCY_LEVEL_MIN_SPEED_X100          10U /* 频率法液位控制参数：最小值 SPEED 放大 100 倍。 */
#define FREQUENCY_LEVEL_STABLE_COUNT            3U /* 频率法液位控制参数：稳定 数量。 */
#define FREQUENCY_LEVEL_SAMPLE_DELAY_MS         200U /* 频率法液位控制参数：SAMPLE 延时 毫秒。 */
#define FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS       600000U /* 频率法液位控制参数：搜索 超时 毫秒。 */
#define FREQUENCY_LEVEL_DEFAULT_DEADBAND_HZ     15.0f /* 频率法液位跟随默认死区，单位 Hz。 */
#define FREQUENCY_LEVEL_KP_SPEED_X100_PER_HZ    0.10f /* 频率法液位跟随比例速度系数，单位 0.01 速度每 Hz。 */
#define FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ 200.0f /* 相对频率法边界判定余量，单位 Hz。 */

static uint32_t DensityLevel_StopAndReturn(uint32_t error_code, const char *reason);
static uint32_t DensityLevel_RunClosedLoop(uint32_t follow_mode);
static uint32_t DensityLevel_ReadCurrent(float *density, float *frequency, float *temperature);
static uint32_t DensityLevel_ComputeSpeedX100(float density_error, float deadband, uint32_t max_speed_x100);
static uint32_t DensityLevel_StartOrUpdateMotion(int dir, uint32_t speed_x100, int *active_dir, uint32_t *active_speed_x100);
static float DensityLevel_GetDeadband(uint32_t raw_threshold);
static uint32_t DensityLevel_GetStableDelayMs(void);
static void DensityLevel_RecordCurrentPosition(const char *tag);

static uint32_t FrequencyLevel_StopAndReturn(uint32_t error_code, const char *reason);
static uint32_t FrequencyLevel_RunClosedLoop(uint32_t follow_mode);
static uint32_t FrequencyLevel_ComputeSpeedX100(float frequency_error, float deadband, uint32_t max_speed_x100);
static float FrequencyLevel_GetDeadband(uint32_t raw_threshold);
static uint8_t FrequencyLevel_IsStableInsideBand(float frequency_error, float deadband);
static int FrequencyLevel_CorrectRelativeEndpointDirection(int dir);
static void FrequencyLevel_RecordCurrentPosition(const char *tag);

static void OilLevel_PrintFollowPositionInfo(void);
uint32_t FollowOilLevel(void);

/**
 * @brief 液位流程故障退出前统一停止电机，避免非阻塞搜索动作继续运行。
 */
static uint32_t OilLevel_StopBeforeReturn(uint32_t error_code, const char *reason)
{
    /* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
    if (error_code == NO_ERROR) {
        return NO_ERROR;
    }
    (void)MotorCtrl_SlowStop();

    return error_code;
}



static void OilLevel_UpdateAoOutput(void)
{
    uint32_t ao_ret = AoOutput_Update();

    if ((ao_ret != NO_ERROR) && (ao_ret != STATE_SWITCH) &&
        (g_measurement.device_status.error_code == NO_ERROR)) {
        FaultManager_SetErrorState(ao_ret, GetShortFilename(__FILE__), __LINE__, __func__);
    }
}

/**
 * @brief 液位结果字段是无符号，上报前负位置统一按0处理。
 */
static uint32_t OilLevel_ClampLevelForReport(int32_t oil_level, const char *reason)
{
    const char *tag = (reason != NULL) ? reason : "液位";

    if (oil_level < 0) {
        printf("液位流程\t%s位置为负：%ld(0.1mm)，按0上报\r\n", tag, (long)oil_level);
        return 0U;
    }

    return (uint32_t)oil_level;
}
/**
 * @brief 将当前传感器位置立即同步到液位测量结果和密度分布液位缓存。
 */
static void OilLevel_SyncCurrentPositionToResult(const char *reason)
{
    int32_t oil_level = g_measurement.debug_data.sensor_position;
    const char *tag = (reason != NULL) ? reason : "同步";

    if (oil_level < 0) {
        printf("液位流程\t%s后液位位置为负：%ld(0.1mm)，按0上报\r\n", tag, (long)oil_level);
        oil_level = 0;
    }

    g_measurement.oil_measurement.oil_level = (uint32_t)oil_level;
    g_measurement.density_distribution.Density_oil_level = g_measurement.oil_measurement.oil_level;
    /* 修正液位同步的是当前传感器位置，需要恢复液位有效标志。 */
    g_measurement.oil_measurement.probe_at_liquid_level = 1U;
    g_measurement.oil_measurement.liquid_stable = 1U;
    OilLevel_UpdateAoOutput();

    printf("液位流程\t%s后同步液位：%lu(0.1mm)\r\n",
           tag,
           (unsigned long)g_measurement.oil_measurement.oil_level);
}

/* 液位跟随打印液位值时，同时输出当前记步来源和两套尺带长度，便于现场比对。 */
static void OilLevel_PrintFollowPositionInfo(void)
{
    double motor_cable_mm;
    const double encoder_cable_mm = (double)encoder_get_cable_length_01mm() / 10.0;

    if (MotorCtrl_IsPositionSourceMotor()) {
        motor_cable_mm = (double)g_measurement.debug_data.cable_length / 10.0;
    } else {
        motor_cable_mm = (double)g_measurement.debug_data.motor_distance / 10.0;
    }

    printf("\t{编码模式}%s\t{电机记步尺带长度}%.1f\t{编码记步尺带长度}%.1f",
           MotorCtrl_IsPositionSourceMotor() ? "电机记步" : "编码轮记步",
           motor_cable_mm,
           encoder_cable_mm);
}


static uint32_t DensityLevel_StopAndReturn(uint32_t error_code, const char *reason)
{
    if (error_code == NO_ERROR) {
        return NO_ERROR;
    }
    if (error_code != STATE_SWITCH) {
        printf("密度找液位\t%s\t停止电机后返回\t错误码=0x%08lX\r\n",
               (reason != NULL) ? reason : "闭环退出",
               (unsigned long)error_code);
    }
    (void)MotorCtrl_SlowStop();
    return error_code;
}

static float DensityLevel_GetDeadband(uint32_t raw_threshold)
{
    if (raw_threshold == 0U) {
        return DENSITY_LEVEL_DEFAULT_DEADBAND_KGM3;
    }
    return RAW_TO_DENSITY(raw_threshold);
}

static uint32_t DensityLevel_GetStableDelayMs(void)
{
    if (g_deviceParams.oilLevelHysteresisTime == 0U) {
        return 1000U;
    }
    return g_deviceParams.oilLevelHysteresisTime * 1000U;
}

static uint32_t DensityLevel_ReadCurrent(float *density, float *frequency, float *temperature)
{
    uint32_t ret;

    if ((density == NULL) || (frequency == NULL) || (temperature == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    ret = Read_Density(frequency, density, temperature);
    if (ret != NO_ERROR) {
        return ret;
    }

    if ((*density <= 0.0f) || (*density > 2000.0f)) {
        printf("密度找液位\t密度值无效\t密度=%.3f\r\n", (double)*density);
        return DENSITY_INVALID;
    }

    return NO_ERROR;
}

static uint32_t DensityLevel_ComputeSpeedX100(float density_error, float deadband, uint32_t max_speed_x100)
{
    float abs_error;
    float speed_f;
    uint32_t speed_x100;
    uint32_t effective_max_speed = max_speed_x100;

    if (effective_max_speed == 0U) {
        effective_max_speed = MotorCtrl_GetDefaultSpeedX100();
    }
    if (effective_max_speed < DENSITY_LEVEL_MIN_SPEED_X100) {
        effective_max_speed = DENSITY_LEVEL_MIN_SPEED_X100;
    }

    abs_error = fabsf(density_error);
    if (abs_error <= deadband) {
        return 0U;
    }

    speed_f = (float)DENSITY_LEVEL_MIN_SPEED_X100 +
              (DENSITY_LEVEL_KP_SPEED_X100_PER_KGM3 * (abs_error - deadband));
    if (speed_f < (float)DENSITY_LEVEL_MIN_SPEED_X100) {
        speed_f = (float)DENSITY_LEVEL_MIN_SPEED_X100;
    }
    if (speed_f > (float)effective_max_speed) {
        speed_f = (float)effective_max_speed;
    }

    speed_x100 = (uint32_t)(speed_f + 0.5f);
    if (speed_x100 == 0U) {
        speed_x100 = DENSITY_LEVEL_MIN_SPEED_X100;
    }
    return speed_x100;
}

static uint32_t DensityLevel_StartOrUpdateMotion(int dir,
                                                 uint32_t speed_x100,
                                                 int *active_dir,
                                                 uint32_t *active_speed_x100)
{
    uint32_t ret;
    uint32_t old_speed;
    uint32_t delta_speed;

    if ((active_dir == NULL) || (active_speed_x100 == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    if (speed_x100 == 0U) {
        if (*active_dir != DENSITY_LEVEL_DIR_NONE) {
            ret = MotorCtrl_SlowStop();
            if (ret != NO_ERROR) {
                return ret;
            }
            *active_dir = DENSITY_LEVEL_DIR_NONE;
            *active_speed_x100 = 0U;
        }
        return NO_ERROR;
    }

    if ((*active_dir != DENSITY_LEVEL_DIR_NONE) && (*active_dir != dir)) {
        ret = MotorCtrl_SlowStop();
        if (ret != NO_ERROR) {
            return ret;
        }
        *active_dir = DENSITY_LEVEL_DIR_NONE;
        *active_speed_x100 = 0U;
    }

    old_speed = *active_speed_x100;
    delta_speed = (speed_x100 >= old_speed) ? (speed_x100 - old_speed) : (old_speed - speed_x100);
    if ((*active_dir == dir) && (delta_speed < DENSITY_LEVEL_SPEED_DELTA_X100)) {
        return NO_ERROR;
    }

    ret = MotorCtrl_StartVelocity(dir, speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }

    *active_dir = dir;
    *active_speed_x100 = speed_x100;
    return NO_ERROR;
}

static void DensityLevel_RecordCurrentPosition(const char *tag)
{
    g_measurement.oil_measurement.oil_level =
            OilLevel_ClampLevelForReport(g_measurement.debug_data.sensor_position, tag);
    g_measurement.density_distribution.Density_oil_level = g_measurement.oil_measurement.oil_level;
    g_measurement.oil_measurement.probe_at_liquid_level = 1U;
    g_measurement.oil_measurement.liquid_stable = 1U;
    OilLevel_UpdateAoOutput();
    printf("密度找液位\t%s\t液位=%lu(0.1mm)\r\n",
           (tag != NULL) ? tag : "记录",
           (unsigned long)g_measurement.oil_measurement.oil_level);
}

static uint32_t DensityLevel_RunClosedLoop(uint32_t follow_mode)
{
    uint32_t ret;
    uint32_t start_tick;
    uint32_t stable_count = 0U;
    uint32_t invalid_density_count = 0U;
    int active_dir = DENSITY_LEVEL_DIR_NONE;
    uint32_t active_speed_x100 = 0U;
    float target_density;
    float deadband;
    float follow_deadband;

    if (g_deviceParams.oilLevelDensity == 0U) {
        printf("密度找液位\t目标密度未配置\r\n");
        return PARAM_ERROR;
    }

    target_density = RAW_TO_DENSITY(g_deviceParams.oilLevelDensity);
    deadband = DensityLevel_GetDeadband(g_deviceParams.oilLevelThreshold);
    follow_deadband = DensityLevel_GetDeadband(g_deviceParams.oilLevelHysteresisThreshold);
    if (follow_mode != 0U) {
        deadband = follow_deadband;
    }

    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;

    ret = EnableDensityMode();
    if (ret != NO_ERROR) {
        return DensityLevel_StopAndReturn(ret, "切换密度模式失败");
    }

    printf("密度找液位\t开始闭环\t模式=%s\t目标密度=%.3f\t死区=%.3f\r\n",
           (follow_mode != 0U) ? "跟随" : "查找",
           (double)target_density,
           (double)deadband);

    start_tick = HAL_GetTick();
    while (1) {
        float density = 0.0f;
        float frequency = 0.0f;
        float temperature = 0.0f;
        float density_error;
        uint32_t speed_x100;
        int dir;

        if (HasEffectiveCommandSwitchRequest()) {
            return DensityLevel_StopAndReturn(STATE_SWITCH, "命令切换");
        }

        ret = DensityLevel_ReadCurrent(&density, &frequency, &temperature);
        if (ret == DENSITY_INVALID) {
            invalid_density_count++;
            if (invalid_density_count >= DENSITY_LEVEL_INVALID_DENSITY_LIMIT) {
                return DensityLevel_StopAndReturn(DENSITY_INVALID, "连续密度无效");
            }
            ret = AbortableDelay_CommandSwitch(DENSITY_LEVEL_SAMPLE_DELAY_MS, 50U);
            if (ret != NO_ERROR) {
                return DensityLevel_StopAndReturn(ret, "命令切换");
            }
            continue;
        }
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "读取密度失败");
        }
        invalid_density_count = 0U;

        density_error = target_density - density;
        g_measurement.oil_measurement.current_frequency = (uint32_t)(frequency + 0.5f);

        printf("密度找液位\t当前密度=%.3f\t目标=%.3f\t偏差=%.3f\t温度=%.3f\r\n",
               (double)density,
               (double)target_density,
               (double)density_error,
               (double)temperature);

        if (density_error > deadband) {
            dir = MOTOR_DIRECTION_DOWN;
        } else if (density_error < -deadband) {
            dir = MOTOR_DIRECTION_UP;
        } else {
            dir = DENSITY_LEVEL_DIR_NONE;
        }

        if (dir == DENSITY_LEVEL_DIR_NONE) {
            ret = DensityLevel_StartOrUpdateMotion(dir, 0U, &active_dir, &active_speed_x100);
            if (ret != NO_ERROR) {
                return DensityLevel_StopAndReturn(ret, "停止确认失败");
            }
            stable_count++;
            printf("密度找液位\t进入死区\t稳定计数=%lu/%lu\r\n",
                   (unsigned long)stable_count,
                   (unsigned long)DENSITY_LEVEL_STABLE_COUNT);
            if (stable_count >= DENSITY_LEVEL_STABLE_COUNT) {
                DensityLevel_RecordCurrentPosition((follow_mode != 0U) ? "密度跟随" : "密度查找");
                if (follow_mode == 0U) {
                    return NO_ERROR;
                }
                stable_count = DENSITY_LEVEL_STABLE_COUNT;
            }
            ret = AbortableDelay_CommandSwitch(DensityLevel_GetStableDelayMs(), 100U);
            if (ret != NO_ERROR) {
                return DensityLevel_StopAndReturn(ret, "命令切换");
            }
            continue;
        }

        stable_count = 0U;
        g_measurement.oil_measurement.probe_at_liquid_level = 0U;
        g_measurement.oil_measurement.liquid_stable = 0U;
        speed_x100 = DensityLevel_ComputeSpeedX100(density_error, deadband, MotorCtrl_GetDefaultSpeedX100());
        printf("密度找液位\t速度闭环\t方向=%s\t速度=%.2f m/min\r\n",
               MotorCtrl_DirectionText(dir),
               (double)speed_x100 / 100.0);

        ret = DensityLevel_StartOrUpdateMotion(dir, speed_x100, &active_dir, &active_speed_x100);
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "启动速度模式失败");
        }

        ret = determineTheSensorPositionAndUpdateTheLevelValue();
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "位置越界");
        }

        ret = CheckWeightCollision();
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "称重碰撞");
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "丢步检测失败");
        }

        if ((follow_mode == 0U) &&
            ((HAL_GetTick() - start_tick) > DENSITY_LEVEL_SEARCH_TIMEOUT_MS)) {
            return DensityLevel_StopAndReturn(MEASUREMENT_TIMEOUT, "密度闭环超时");
        }

        ret = AbortableDelay_CommandSwitch(DENSITY_LEVEL_SAMPLE_DELAY_MS, 50U);
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "命令切换");
        }
    }
}

/**
 * @brief 液位测量与跟随主流程
 *        包含3个阶段：启用液位模式、搜索液位、跟随液位
 *        每个阶段失败时自动重试3次（状态切换时不重试）
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */

/**
 * @brief 频率连续找液位异常退出统一停机，避免速度模式保持运行。
 */
static uint32_t FrequencyLevel_StopAndReturn(uint32_t error_code, const char *reason)
{
    if (error_code == NO_ERROR) {
        return NO_ERROR;
    }
    if (error_code != STATE_SWITCH) {
        printf("频率找液位\t%s\t停止电机后返回\t错误码=0x%08lX\r\n",
               (reason != NULL) ? reason : "闭环退出",
               (unsigned long)error_code);
    }
    (void)MotorCtrl_SlowStop();
    return error_code;
}

/**
 * @brief 获取频率闭环死区，参数未配置时使用保守默认值。
 */
static float FrequencyLevel_GetDeadband(uint32_t raw_threshold)
{
    if (raw_threshold == 0U) {
        return FREQUENCY_LEVEL_DEFAULT_DEADBAND_HZ;
    }
    return (float)raw_threshold;
}

/**
 * @brief 根据频率偏差按比例计算速度模式速度，速度受默认运行速度上限保护。
 */
static uint32_t FrequencyLevel_ComputeSpeedX100(float frequency_error, float deadband, uint32_t max_speed_x100)
{
    float abs_error;
    float speed_f;
    uint32_t speed_x100;
    uint32_t effective_max_speed = max_speed_x100;

    if (effective_max_speed == 0U) {
        effective_max_speed = MotorCtrl_GetDefaultSpeedX100();
    }
    if (effective_max_speed < FREQUENCY_LEVEL_MIN_SPEED_X100) {
        effective_max_speed = FREQUENCY_LEVEL_MIN_SPEED_X100;
    }

    abs_error = fabsf(frequency_error);
    if (abs_error <= deadband) {
        return 0U;
    }

    speed_f = (float)FREQUENCY_LEVEL_MIN_SPEED_X100 +
              (FREQUENCY_LEVEL_KP_SPEED_X100_PER_HZ * (abs_error - deadband));
    if (speed_f < (float)FREQUENCY_LEVEL_MIN_SPEED_X100) {
        speed_f = (float)FREQUENCY_LEVEL_MIN_SPEED_X100;
    }
    if (speed_f > (float)effective_max_speed) {
        speed_f = (float)effective_max_speed;
    }

    speed_x100 = (uint32_t)(speed_f + 0.5f);
    if (speed_x100 == 0U) {
        speed_x100 = FREQUENCY_LEVEL_MIN_SPEED_X100;
    }
    return speed_x100;
}

/**
 * @brief 判断频率是否已进入稳定区，相对频率法额外避开空气/油中端点。
 */
static uint8_t FrequencyLevel_IsStableInsideBand(float frequency_error, float deadband)
{
    if (fabsf(frequency_error) > deadband) {
        return 0U;
    }

    if (g_deviceParams.liquidLevelMeasurementMethod == OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ) {
        float air_frequency = (float)g_measurement.oil_measurement.air_frequency;
        float oil_frequency = (float)g_measurement.oil_measurement.oil_frequency;
        float current_frequency = (float)g_measurement.oil_measurement.current_frequency;

        if ((air_frequency > oil_frequency) &&
            ((air_frequency - oil_frequency) > (2.0f * FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ))) {
            if (((air_frequency - current_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ) ||
                ((current_frequency - oil_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ)) {
                return 0U;
            }
        }
    }

    return 1U;
}

/**
 * @brief 相对频率法在目标死区内但靠近端点时补充运动方向。
 */
static int FrequencyLevel_CorrectRelativeEndpointDirection(int dir)
{
    if ((dir == DENSITY_LEVEL_DIR_NONE) &&
        (g_deviceParams.liquidLevelMeasurementMethod == OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ)) {
        float air_frequency = (float)g_measurement.oil_measurement.air_frequency;
        float oil_frequency = (float)g_measurement.oil_measurement.oil_frequency;
        float current_frequency = (float)g_measurement.oil_measurement.current_frequency;

        if ((air_frequency > oil_frequency) &&
            ((air_frequency - oil_frequency) > (2.0f * FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ))) {
            if ((air_frequency - current_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ) {
                dir = MOTOR_DIRECTION_DOWN;
            } else if ((current_frequency - oil_frequency) <= FREQUENCY_LEVEL_RELATIVE_EDGE_MARGIN_HZ) {
                dir = MOTOR_DIRECTION_UP;
            }
        }
    }

    return dir;
}



/**
 * @brief 记录频率闭环确认到的液位，并同步外部协议稳定标志。
 */
static void FrequencyLevel_RecordCurrentPosition(const char *tag)
{
    g_measurement.oil_measurement.oil_level =
            OilLevel_ClampLevelForReport(g_measurement.debug_data.sensor_position, tag);
    g_measurement.density_distribution.Density_oil_level = g_measurement.oil_measurement.oil_level;
    g_measurement.oil_measurement.probe_at_liquid_level = 1U;
    g_measurement.oil_measurement.liquid_stable = 1U;
    OilLevel_UpdateAoOutput();
    printf("频率找液位\t%s\t液位=%lu(0.1mm)\r\n",
           (tag != NULL) ? tag : "记录",
           (unsigned long)g_measurement.oil_measurement.oil_level);
}

/**
 * @brief 相对频率和绝对频率共用的速度模式连续找液位/跟随闭环。
 */
static uint32_t FrequencyLevel_RunClosedLoop(uint32_t follow_mode)
{
    uint32_t ret;
    uint32_t start_tick;
    uint32_t stable_count = 0U;
    int active_dir = DENSITY_LEVEL_DIR_NONE;
    uint32_t active_speed_x100 = 0U;
    float deadband;
    float follow_deadband;
    uint32_t method = g_deviceParams.liquidLevelMeasurementMethod;
    const char *method_text = "频率";

    if ((method == OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ) &&
        (g_measurement.oil_measurement.follow_frequency == 0U)) {
        g_measurement.oil_measurement.follow_frequency = g_deviceParams.oilLevelFrequency;
    }
    if (g_measurement.oil_measurement.follow_frequency == 0U) {
        printf("频率找液位\t目标频率未配置\r\n");
        return PARAM_ERROR;
    }

    if (method == OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ) {
        method_text = "连续相对频率";
    } else if (method == OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ) {
        method_text = "连续绝对频率";
    }

    deadband = FrequencyLevel_GetDeadband(g_deviceParams.oilLevelThreshold);
    follow_deadband = FrequencyLevel_GetDeadband(g_deviceParams.oilLevelHysteresisThreshold);
    if (follow_mode != 0U) {
        deadband = follow_deadband;
    }

    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;

    ret = EnableLevelMode();
    if (ret != NO_ERROR) {
        return FrequencyLevel_StopAndReturn(ret, "切换液位模式失败");
    }

    printf("频率找液位\t开始闭环\t方法=%s\t模式=%s\t目标频率=%lu Hz\t死区=%.1f Hz\r\n",
           method_text,
           (follow_mode != 0U) ? "跟随" : "查找",
           (unsigned long)g_measurement.oil_measurement.follow_frequency,
           (double)deadband);

    start_tick = HAL_GetTick();
    while (1) {
        float frequency_error;
        uint32_t speed_x100;
        int dir;

        if (HasEffectiveCommandSwitchRequest()) {
            return FrequencyLevel_StopAndReturn(STATE_SWITCH, "命令切换");
        }

        ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "读取液位频率失败");
        }

        frequency_error = (float)g_measurement.oil_measurement.current_frequency -
                          (float)g_measurement.oil_measurement.follow_frequency;
        printf("频率找液位\t当前频率=%lu Hz\t目标=%lu Hz\t偏差=%.1f Hz\r\n",
               (unsigned long)g_measurement.oil_measurement.current_frequency,
               (unsigned long)g_measurement.oil_measurement.follow_frequency,
               (double)frequency_error);

        if (frequency_error > deadband) {
            dir = MOTOR_DIRECTION_DOWN;
        } else if (frequency_error < -deadband) {
            dir = MOTOR_DIRECTION_UP;
        } else {
            dir = DENSITY_LEVEL_DIR_NONE;
        }
        dir = FrequencyLevel_CorrectRelativeEndpointDirection(dir);

        if (FrequencyLevel_IsStableInsideBand(frequency_error, deadband) != 0U) {
            ret = DensityLevel_StartOrUpdateMotion(DENSITY_LEVEL_DIR_NONE, 0U, &active_dir, &active_speed_x100);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "停止确认失败");
            }
            stable_count++;
            printf("频率找液位\t进入死区\t稳定计数=%lu/%lu\r\n",
                   (unsigned long)stable_count,
                   (unsigned long)FREQUENCY_LEVEL_STABLE_COUNT);
            if (stable_count >= FREQUENCY_LEVEL_STABLE_COUNT) {
                FrequencyLevel_RecordCurrentPosition((follow_mode != 0U) ? "频率跟随" : "频率查找");
                if (follow_mode == 0U) {
                    return NO_ERROR;
                }
                stable_count = FREQUENCY_LEVEL_STABLE_COUNT;
            }
            ret = AbortableDelay_CommandSwitch(DensityLevel_GetStableDelayMs(), 100U);
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn(ret, "命令切换");
            }
            continue;
        }

        stable_count = 0U;
        g_measurement.oil_measurement.probe_at_liquid_level = 0U;
        g_measurement.oil_measurement.liquid_stable = 0U;
        speed_x100 = FrequencyLevel_ComputeSpeedX100(frequency_error, deadband, MotorCtrl_GetDefaultSpeedX100());
        if (speed_x100 == 0U) {
            speed_x100 = FREQUENCY_LEVEL_MIN_SPEED_X100;
        }
        printf("频率找液位\t速度闭环\t方向=%s\t速度=%.2f m/min\r\n",
               MotorCtrl_DirectionText(dir),
               (double)speed_x100 / 100.0);

        ret = DensityLevel_StartOrUpdateMotion(dir, speed_x100, &active_dir, &active_speed_x100);
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "启动速度模式失败");
        }

        ret = determineTheSensorPositionAndUpdateTheLevelValue();
        if (ret == MEASUREMENT_OILLEVEL_LOW) {
            active_dir = DENSITY_LEVEL_DIR_NONE;
            active_speed_x100 = 0U;
            ret = waitForTheLiquidLevelToExceedTheBlindZone();
            if (ret != NO_ERROR) {
                return FrequencyLevel_StopAndReturn((uint32_t)ret, "等待液位离开盲区失败");
            }
            continue;
        }
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn((uint32_t)ret, "位置越界");
        }

        ret = CheckWeightCollision();
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "称重碰撞");
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "丢步检测失败");
        }

        if ((follow_mode == 0U) &&
            ((HAL_GetTick() - start_tick) > FREQUENCY_LEVEL_SEARCH_TIMEOUT_MS)) {
            return FrequencyLevel_StopAndReturn(MEASUREMENT_TIMEOUT, "频率闭环超时");
        }

        ret = AbortableDelay_CommandSwitch(FREQUENCY_LEVEL_SAMPLE_DELAY_MS, 50U);
        if (ret != NO_ERROR) {
            return FrequencyLevel_StopAndReturn(ret, "命令切换");
        }
    }
}


/**
 * @brief 搜索并跟随液位
 *
 * 该函数执行液位测量的完整流程，包括回零点检查、液位搜索和液位跟随。
 * 首先检查设备是否需要回零点，如需要则先执行回零点操作。
 * 然后进行液位搜索，最多尝试3次。搜索成功后，根据设备状态执行液位位置修正。
 * 最后进入液位跟随状态，同样最多尝试3次，完成液位的持续跟踪。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 操作成功完成
 *                  - 其他错误码: 操作失败，具体错误码由 SearchZero、SearchOilLevel 或 FollowOilLevel 返回
 *
 * @note 函数执行流程：
 *       1. 检查并执行回零点操作（如果设备需要）
 *       2. 搜索液位（最多3次尝试）
 *       3. 修正液位位置（如果处于校准状态）
 *       4. 跟随液位（最多3次尝试）
 *
 * @note 每次失败后会延迟1秒后重试，如果检测到状态切换（STATE_SWITCH）则中止当前步骤
 */
uint32_t SearchAndFollowOilLevel(void) {
	uint32_t ret;
	uint8_t try_times;
	printf("液位测量\t开始\r\n");
	/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
	if ((g_measurement.device_status.zero_point_status == 1)&&(g_deviceParams.error_auto_back_zero==1)){
		printf("液位测量\t设备需要回零点\r\n");
		ret = SearchZero();  /* 如果设备需要回零点，先执行回零点测量 */
		CHECK_ERROR(ret);  /* 检查回零点是否成功 */
		printf("液位测量\t回零点完成\r\n");
	}
	printf("液位测量\t初始重量：%d\r\n", weight_parament.stable_weight);
	/* ************** Step 1: 搜索液位 ************** */
	try_times = 0;
	while (try_times < 3) {
		try_times++;

		ret = SearchOilLevel();

		/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
		if (ret == NO_ERROR) {
			if (try_times > 1U) {
				/* 错误 阶段：重试成功 模块：测量 操作：搜索液位 原因：恢复成功 尝试：try_times/3U */
				ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
				                 ERROR_LOG_OP_SEARCH_OIL_LEVEL,
				                 ERROR_LOG_REASON_RECOVER_OK,
				                 try_times,
				                 3U);
			}
			break;
		} else if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		} else {
			/* 错误 阶段：错误重试 模块：测量 操作：搜索液位 原因：搜索失败 尝试：try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
			ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
			               ERROR_LOG_OP_SEARCH_OIL_LEVEL,
			               ERROR_LOG_REASON_SEARCH_FAIL,
			               try_times,
			               3U,
			               ret);
			(void)MotorCtrl_SlowStop();
			HAL_Delay(1000);
		}
	}
	/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
	if (ret != NO_ERROR) {
		CHECK_ERROR(ret);
	}
	/* 修正液位位置 */
	if (g_measurement.device_status.device_state == STATE_CALIBRATIONOILING) {
		CorrectOilLevelProcess();
	}
	/* ************** Step 2: 跟随液位 ************** */
	g_measurement.device_status.device_state = STATE_FLOWOIL; /* 切换到液位跟随状态 */
	try_times = 0;
	while (try_times < 3) {
		try_times++;

		ret = FollowOilLevel();

		/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
		if (ret == NO_ERROR) {
			if (try_times > 1U) {
				/* 错误 阶段：重试成功 模块：测量 操作：跟随液位 原因：恢复成功 尝试：try_times/3U */
				ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
				                 ERROR_LOG_OP_FOLLOW_OIL_LEVEL,
				                 ERROR_LOG_REASON_RECOVER_OK,
				                 try_times,
				                 3U);
			}
			break;
		} else if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		} else {
			/* 错误 阶段：错误重试 模块：测量 操作：跟随液位 原因：跟随失败 尝试：try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
			ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
			               ERROR_LOG_OP_FOLLOW_OIL_LEVEL,
			               ERROR_LOG_REASON_FOLLOW_FAIL,
			               try_times,
			               3U,
			               ret);
			HAL_Delay(1000);
		}
	}
	/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
	if (ret != NO_ERROR) {
		CHECK_ERROR(ret);
	}

	printf("液位流程\t全部完成\r\n");
	return NO_ERROR;
}

/**
 * @brief 搜索液位高度
 *
 * 该函数通过粗找和精找两个阶段完成液位高度的测量。粗找阶段判断传感器当前状态（在油中或在空气中），
 * 并执行相应的搜索操作；精找阶段根据设定的目标频率进行精确液位定位。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 液位测量成功
 *                  - MEASUREMENT_WEIGHT_DOWN_FAIL: 粗找液位失败（超过最大尝试次数）
 *                  - 其他错误码: 具体错误状态
 *
 * @note 函数执行流程：
 *       1. 启用液位模式（最多尝试3次）
 *       2. 粗找阶段：判断传感器在油中或空气中，执行SearchAir或SearchOil
 *       3. 精找阶段：根据测量方法计算目标频率，调用SearchOilPrecise
 *       4. 记录最终液位位置
 *
 * @note 依赖函数：
 *       - EnableLevelMode(): 启用液位模式
 *       - determine_level_status(): 判断液位状态
 *       - fault_info_init(): 清除故障信息
 *       - DSM_Get_LevelMode_Frequence_Avg(): 获取平均频率
 *       - MotorCtrl_MoveAndWait(): 电机移动
 *       - SearchAir(): 寻找空气
 *       - SearchOil(): 寻找液位
 *       - SearchOilPrecise(): 精确搜索液位
 */
uint32_t SearchOilLevel(void) {
    switch (g_deviceParams.liquidLevelMeasurementMethod) {
    case OIL_LEVEL_METHOD_DENSITY:
        g_measurement.device_status.device_state = STATE_FINDOIL;
        return DensityLevel_RunClosedLoop(DENSITY_LEVEL_RUN_SEARCH);
    default:
        break;
    }

    uint32_t ret;
    uint32_t last_coarse_ret = MEASUREMENT_OILLEVEL_NOTFOUND;
    uint8_t coarse_found = 0U;
    int mode_try_times = 0;
    int coarse_try_times = 0;
    /* 找液位开始时先清除液位命中/稳定状态，防止 CPU3 读到上一轮结果。 */
    g_measurement.oil_measurement.probe_at_liquid_level = 0;
    g_measurement.oil_measurement.liquid_stable = 0;
    fault_info_init();  /* 清除故障信息 */
    /* ************** Step 1: 启用液位模式 ************** */
    while (mode_try_times < 3) {
        mode_try_times++;

        ret = EnableLevelMode();

        /* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            if (mode_try_times > 1) {
                /* 错误 阶段：重试成功 模块：传感器 操作：启用液位模式 原因：恢复成功 尝试：mode_try_times/3U */
                ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                 ERROR_LOG_OP_ENABLE_LEVEL_MODE,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)mode_try_times,
                                 3U);
            }
            break;
        } else if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		} else {
            /* 错误 阶段：错误重试 模块：传感器 操作：启用液位模式 原因：模式启用失败 尝试：mode_try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_ENABLE_LEVEL_MODE,
                           ERROR_LOG_REASON_MODE_FAIL,
                           (uint32_t)mode_try_times,
                           3U,
                           ret);
            HAL_Delay(500);
        }
    }
    /* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        CHECK_ERROR(ret);
    }

    /* ************** 粗找阶段 - 带重试机制 ************** */
    while (coarse_try_times < 3) {  /* 这里重试没有起作用 */
        coarse_try_times++;
        fault_info_init(); /* 清除故障信息 */
        ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.current_frequency);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        /* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            last_coarse_ret = ret;
            /* 错误 阶段：错误重试 模块：传感器 操作：读取液位频率 原因：ErrorLog_GetReasonByCode(ret) 尝试：coarse_try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_READ_LEVEL_FREQ,
                           ErrorLog_GetReasonByCode(ret),
                           (uint32_t)coarse_try_times,
                           3U,
                           ret);
            HAL_Delay(500);
            continue;
        }
        if (INOIL) {
            /* 如果当前传感器在盲区以上100mm */
            if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
                /* 向下运行保证传感器全部在油 */
                ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
                CHECK_ERROR(ret); /* 检查下行是否成功 */
            }
            /* 取油中频率 */
            ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.oil_frequency);
            printf("液位测量\t油中频率：%ld\r\n", g_measurement.oil_measurement.oil_frequency);
            CHECK_ERROR(ret);  /* 检查获取油中频率是否成功 */

            printf("液位测量\t传感器已在液位中，向上寻找空气\r\n");
            ret = SearchAir();
            CHECK_ERROR(ret); /* 检查寻找空气是否成功 */
            coarse_found = 1U;
            break;
        } else if (INAIR) {
            printf("液位测量\t传感器在空气中，向下寻找液位\r\n");
            ret = SearchOil();
            CHECK_ERROR(ret); /* 检查寻找液位是否成功 */
            coarse_found = 1U;
            break;
        }
    }
    if (coarse_found == 0U) {
        RETURN_ERROR(last_coarse_ret);
    }
    if (coarse_try_times > 1) {
        /* 错误 阶段：重试成功 模块：测量 操作：搜索液位 原因：恢复成功 尝试：coarse_try_times/3U */
        ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                         ERROR_LOG_OP_SEARCH_OIL_LEVEL,
                         ERROR_LOG_REASON_RECOVER_OK,
                         (uint32_t)coarse_try_times,
                         3U);
    }
    printf("液位测量\t粗找液位完成\r\n");
    /* ************** 精找阶段 ************** */
    /* 精确找液位 */
    switch (g_deviceParams.liquidLevelMeasurementMethod) {
    case OIL_LEVEL_METHOD_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
        g_measurement.oil_measurement.follow_frequency =
                (g_measurement.oil_measurement.air_frequency + g_measurement.oil_measurement.oil_frequency) / 2U;
        break;
    case OIL_LEVEL_METHOD_FIXED_FREQ:
    case OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ:
        g_measurement.oil_measurement.follow_frequency = g_deviceParams.oilLevelFrequency;
        break;
    default:
        g_measurement.oil_measurement.follow_frequency = g_deviceParams.oilLevelFrequency;
        break;
    }
    printf("液位查找\t目标频率：%lu Hz\r\n",
           (unsigned long)g_measurement.oil_measurement.follow_frequency);

    switch (g_deviceParams.liquidLevelMeasurementMethod) {
    case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ:
        ret = FrequencyLevel_RunClosedLoop(FREQUENCY_LEVEL_RUN_SEARCH);
        if (ret != NO_ERROR) {
            CHECK_ERROR(ret);
        }
        return NO_ERROR;
    case OIL_LEVEL_METHOD_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_FIXED_FREQ:
    default:
        ret = SearchOilPrecise(100);
        /* 旧频率方式和未识别方法保留原有步进式精找路径。 */
        if (ret != NO_ERROR) {
            CHECK_ERROR(ret);
        }
        break;
    }
    /* ************** 最终校验与记录 ************** */
    /* 记录最终液位位置 */
    g_measurement.oil_measurement.oil_level =
            OilLevel_ClampLevelForReport(g_measurement.debug_data.sensor_position, "液位测量");
    g_measurement.density_distribution.Density_oil_level = g_measurement.oil_measurement.oil_level;
    /* 打印测量结果 */
    printf("液位测量\t液位：%lu(0.1mm)\r\n", (unsigned long)g_measurement.oil_measurement.oil_level);

    /* 成功找到液位后才置位 SI7000 的 Probe At Liquid Level 和液体稳定状态。 */
    g_measurement.oil_measurement.probe_at_liquid_level = 1;
    g_measurement.oil_measurement.liquid_stable = 1;
    OilLevel_UpdateAoOutput();
    return NO_ERROR;  /* 返回成功状态 */
}
/**
 * @brief 液位跟随函数，用于持续监测并跟踪液位变化
 *
 * 该函数通过超声波频率信号实现液位的实时监测与跟随。当液位稳定时，电机保持静止；
 * 当检测到液位变动时，重新执行精确搜索并调整传感器位置，确保液位值准确更新。
 *
 * @return uint32_t 返回操作状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部包含无限循环，持续监测液位状态，仅在发生错误时退出
 * @note 液位稳定判断基于频率波动阈值（oilLevelHysteresisThreshold）
 * @note 当液位过低进入盲区时，会调用 waitForTheLiquidLevelToExceedTheBlindZone 等待液位恢复
 */
uint32_t FollowOilLevel(void) {
	uint32_t ret;
	/* 切换到跟随状态 */
	g_measurement.device_status.device_state = STATE_FLOWOIL;

	switch (g_deviceParams.liquidLevelMeasurementMethod) {
	case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
	case OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ:
		return FrequencyLevel_RunClosedLoop(FREQUENCY_LEVEL_RUN_FOLLOW);
	case OIL_LEVEL_METHOD_DENSITY:
		return DensityLevel_RunClosedLoop(DENSITY_LEVEL_RUN_FOLLOW);
	default:
		break;
	}

	/* 液位跟随主循环 */

	while (1) {
		printf("液位跟随\t");

		/* 获取当前频率 */
		ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
		CHECK_ERROR(ret);  /* 检查开启液位模式是否成功 */
		/* 稳定性判断（频率波动在阈值内） */
		if (fabs(frequency_difference) < g_deviceParams.oilLevelHysteresisThreshold) {
			/* 液位稳定时电机不动作，直接打印寄存器中保存的液位值 */
			printf("液位稳定,电机不动作\t");
			printf("液位跟随\t液位值为%ld (0.1mm)", g_measurement.oil_measurement.oil_level);
			ret = (int)AbortableDelay_CommandSwitch(3000U, 100U);
			CHECK_COMMAND_SWITCH(ret);
		} else {
			/* 检测到液位变动，重新跟踪 */
			printf("识别到液位变动\t");
			g_measurement.device_status.device_state = STATE_FINDOIL;
			ret = SearchOilPrecise(100);
			/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
			if (ret != NO_ERROR) {
				return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
			} else {
				g_measurement.device_status.device_state = STATE_FLOWOIL;
				ret = determineTheSensorPositionAndUpdateTheLevelValue();
				if (ret == MEASUREMENT_OILLEVEL_LOW) {
					/* 处理盲区状态 */
					ret = waitForTheLiquidLevelToExceedTheBlindZone();
					CHECK_COMMAND_SWITCH(ret);
					CHECK_ERROR(ret);
				} else {
					CHECK_ERROR(ret);
				}
			}
		}
	}
}

/**
 * @brief 搜索油液液位
 *
 * 该函数通过电机控制传感器移动，完成油液液位的测量。主要流程包括：
 * 1. 若尺带长度较长，先将电机上行到安全位置确保传感器在空气中。
 * 2. 长距离下行寻找油面，实时监控重量状态、检测丢步和碰撞。
 * 3. 检测到液位后慢速停止电机。
 * 4. 若传感器未完全浸入油中，继续下行确保传感器全部在油中。
 * 5. 获取油中频率值。
 * 6. 调用 SearchAir() 向上寻找空气以完成测量。
 *
 * @return int 返回状态码：
 *             - NO_ERROR: 测量成功
 *             - MEASUREMENT_OILLEVEL_LOW: 到达位置下限（盲区）
 *             - 其他错误码: 电机操作、丢步检测或碰撞检测失败
 *
 * @note 函数内部会调用以下辅助函数：
 *       - determine_level_status(): 判断当前重量状态
 *       - MotorCtrl_MoveDown(): 电机下行
 *       - MotorCtrl_CheckLostStepAutoTiming(): 丢步检测
 *       - CheckWeightCollision(): 碰撞检测
 *       - DSM_Get_LevelMode_Frequence_Avg(): 获取油中频率
 *       - SearchAir(): 向上寻找空气
 *
 * @note 输出信息包括：
 *       - 初始重量
 *       - 寻找液位过程中的传感器位置和称重值
 *       - 油中频率值
 */
static int SearchOil() {
	uint32_t ret;
    Level_StateTypeDef level_state;

	printf("液位测量\t初始重量：%d\r\n", weight_parament.stable_weight);

	/* 向上运行保证传感器全部在空气 */
	if (g_measurement.debug_data.cable_length > 1000) { /* 如果尺带长度大于200mm，先将电机上行到安全位置 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查上行是否成功 */
	}
	/* 长距离下行寻找油面 */
	MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
	/* 持续监控重量状态，直到检测到液位 */
    while (1) {
        ret = determine_level_status_motion(&level_state);
        CHECK_ERROR(ret);
        if (level_state == OIL) {
            break;
        }
		ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());  /* 启动电机向下运动 */
		CHECK_ERROR(ret); /* 检查上行是否成功 */
		/* 实时输出编码器位置和重量值（用于调试） */
		printf("液位测量\t长距离寻找液位\t{传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("\t{称重值}%d\r\n", weight_parament.current_weight);
		CHECK_ERROR(ret);
		/* 丢步检测 */
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); /* 检查丢步检测是否成功 */

		/* 称重检测 */
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); /* 检查碰撞检测是否成功 */

		/* 盲区检测 */
		if (g_measurement.debug_data.sensor_position < g_deviceParams.blindZone) {
			printf("超声波找液位\t到达位置下限\r\n");
			return OilLevel_StopBeforeReturn(MEASUREMENT_OILLEVEL_LOW, "到达液位下限");
		}
	}
	ret = MotorCtrl_SlowStop(); /* 到达零点后慢速停止电机 */
	CHECK_ERROR(ret); /* 检查慢速停止是否成功 */
	if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
		/* 向下运行保证传感器全部在油 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查下行是否成功 */
	}
	/* 取油中频率 */
	ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.oil_frequency);
	printf("液位测量\t油中频率：%ld\r\n", g_measurement.oil_measurement.oil_frequency);
	CHECK_ERROR(ret);  /* 检查获取油中频率是否成功 */

	printf("液位测量\t传感器已在液位中，向上寻找空气\r\n");
	ret = SearchAir();
	CHECK_ERROR(ret); /* 检查寻找空气是否成功 */

	return NO_ERROR;
}

/**
 * @brief 搜索空气中的零点位置并获取空气中频率值
 *
 * 该函数通过控制电机上行，持续监控重量状态，直到检测到液位（从油中进入空气）。
 * 到达零点后，慢速停止电机，并确保传感器全部位于空气中，然后获取空气中频率值。
 * 如果传感器位置超出盲区范围，还会将传感器下移至油中进行后续操作。
 *
 * @return int 返回状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部会调用以下辅助函数：
 *       - determine_level_status(): 判断当前液位状态（空气/油中）
 *       - MotorCtrl_MoveUp(): 以指定速度控制电机上行
 *       - MotorCtrl_CheckLostStepAutoTiming(): 自动定时检测电机丢步
 *       - CheckWeightCollision(): 检测称重碰撞
 *       - MotorCtrl_SlowStop(): 慢速停止电机
 *       - MotorCtrl_MoveAndWait(): 移动电机并等待停止
 *       - DSM_Get_LevelMode_Frequence_Avg(): 获取平均频率值
 *
 * @note 输出信息包括：
 *       - 初始重量值
 *       - 传感器位置和称重值（实时调试信息）
 *       - 空气中频率值
 */
static int SearchAir() {
	uint32_t ret;
    Level_StateTypeDef level_state;
/* uint32_t frequency; / /当前频率 */
	printf("液位测量\t初始重量：%d\r\n", weight_parament.stable_weight);

	/* 持续监控重量状态，直到检测到液位 */
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* TODO：电机可能存在上行停止重新加速 */
    while (1) {
        ret = determine_level_status_motion(&level_state);
        CHECK_ERROR(ret);
        if (level_state == AIR) {
            break;
        }
		ret = MotorCtrl_MoveUp(MotorCtrl_GetDefaultSpeedX100());  /* 启动电机向下运动 */
		CHECK_ERROR(ret); /* 检查上行是否成功 */
		/* 实时输出编码器位置和重量值（用于调试） */
		printf("液位测量\t长距离寻找空气\t{传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("\t{称重值}%d\r\n", weight_parament.current_weight);
		CHECK_ERROR(ret);
		/* 丢步检测 */
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); /* 检查丢步检测是否成功 */
/* / /称重检测 */
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); /* 检查碰撞检测是否成功 */
	}
	ret = MotorCtrl_SlowStop(); /* 到达零点后慢速停止电机 */
	CHECK_ERROR(ret); /* 检查慢速停止是否成功 */
	/* 向上运行保证传感器全部在空气 */
	if (g_measurement.debug_data.cable_length > 1000) { /* 如果尺带长度大于200mm，先将电机上行到安全位置 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查上行是否成功 */
	}
	/* 取空气中频率 */
	ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.air_frequency);
	printf("液位测量\t空气中频率：%ld\r\n", g_measurement.oil_measurement.air_frequency);
	CHECK_ERROR(ret);  /* 检查获取空气中频率是否成功 */
	if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
		/* 向下运行保证传感器全部在油 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查下行是否成功 */
	}

	return NO_ERROR;
}

/**
 * @brief 确定液位状态
 *
 * 该函数通过获取当前液位模式的频率值，判断传感器当前所处的状态（空气中或油中）。
 * 根据频率值与阈值的比较，返回相应的液位状态枚举值。
 *
 * @return Level_StateTypeDef 返回液位状态：
 *             - AIR: 传感器在空气中（频率超过上限阈值）
 *             - OIL: 传感器在油中（频率低于下限阈值）
 *
 * @note 函数内部会调用 DSM_Get_LevelMode_Frequence 获取当前频率值。
 * @note 输出信息包括当前频率值和传感器状态（空气中或油中）。
 */
static uint32_t determine_level_status_internal(Level_StateTypeDef *state_out, uint8_t allow_mode_recovery) {
    uint32_t ret;
    uint32_t current_frequency = 0U;
    const char *mode_text;

    if (state_out == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    mode_text = allow_mode_recovery ? "静态" : "运动";
    if (allow_mode_recovery) {
        ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
    } else if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = Read_Level_Frequency(&current_frequency);
    } else {
        ret = DSM_V2_Read_LevelFrequency(&current_frequency);
    }

    CHECK_COMMAND_SWITCH(ret);
    /* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
    }

    if (!allow_mode_recovery) {
        g_measurement.oil_measurement.current_frequency = current_frequency;
    }

    printf("液位状态检测[%s]\t当前频率\t%ld\t", mode_text, g_measurement.oil_measurement.current_frequency);
    if (INAIR) {
        printf("传感器在空气中\r\n");
        *state_out = AIR;
    } else {
        printf("传感器在油中\r\n");
        *state_out = OIL;
    }
    return NO_ERROR;
}

/**
 * @brief 执行液位测量中的 determine_level_status 逻辑。
 *
 * @param state_out 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t determine_level_status(Level_StateTypeDef *state_out) {
    return determine_level_status_internal(state_out, 1U);
}

/**
 * @brief 执行液位测量中的 determine_level_status_motion 逻辑。
 *
 * @param state_out 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t determine_level_status_motion(Level_StateTypeDef *state_out) {
    return determine_level_status_internal(state_out, 0U);
}
/**
 * @func: SearchOilPrecise
 * @brief 通过频率跟随策略定位液位界面
 * @param per_mm_Frequency 每毫米对应的频率变化量（频率-位置换算系数）
 * @return 执行状态码
 *
 * 算法原理：
 * 1. 持续比较当前频率与目标频率(空气/油频率均值)
 * 2. 根据频率偏差计算电机移动距离：
 *    - 小偏差：按线性关系移动 (frequency_difference / per_mm_Frequency)
 *    - 大偏差：采用补偿步长 (4 * overTime + ...)
 * 3. 达到稳定条件（连续10次频率波动<阈值）时退出
 *
 * 特殊处理：
 * - 死循环保护：超过100次循环强制退出
 * - 超限保护：连续加速移动仍无法跟踪时报错
 */
static int SearchOilPrecise(float per_mm_Frequency) {
	/* 初始化状态变量 */
	int ret;
	uint32_t dir;
	float runlenth;
	int followTime = 0, overTime = 0, lowerTime = 0, loopTime = 0;

	printf("进入频率跟随区间\r\n");

	/* 主跟随循环（需满足连续10次稳定） */
	while (followTime < 10) {
		/* 延时保证传感器稳定性（总延时3秒） */
		ret = (int)AbortableDelay_CommandSwitch(2000U, 100U);
		if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		}

		/* 获取当前传感器频率 */
		ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
		/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
		if (ret != NO_ERROR)
			return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");

		/* 死循环保护（>100次循环退出） */
		if (loopTime++ > 100) {
			printf("频率跟随可能陷入循环，跳出频率跟随\r\n");
			break;
		}

		/* 计算当前频率与目标频率差值 */
/* float frequency_difference = g_measurement.oil_measurement.follow_frequency - g_measurement.oil_measurement.current_frequency; */
		printf("当前频率%ld\t频率阈值%ld\t阈值差%f\r\n", g_measurement.oil_measurement.current_frequency, g_measurement.oil_measurement.follow_frequency,
		frequency_difference);

		/* 方向决策树（基于频率偏差） */
		if (frequency_difference > 500 ||  /* 超大正偏差 */
				(g_measurement.oil_measurement.air_frequency - g_measurement.oil_measurement.current_frequency < 200)) { /* 接近空气频率 */
			/* 向下加速移动（补偿步长） */
			overTime++;
			lowerTime = 0;
			runlenth = 4.0 * overTime + frequency_difference / per_mm_Frequency - 4.0;
			dir = MOTOR_DIRECTION_DOWN;
		} else if (frequency_difference > g_deviceParams.oilLevelThreshold) { /* 可接受正偏差 */
			/* 标准向下移动 */
			overTime = 0;
			lowerTime = 0;
			runlenth = frequency_difference / per_mm_Frequency;
			dir = MOTOR_DIRECTION_DOWN;
		} else if (frequency_difference < -500 ||  /* 超大负偏差 */
				(g_measurement.oil_measurement.current_frequency - g_measurement.oil_measurement.oil_frequency < 200)) { /* 接近油中频率 */
			/* 向上加速移动 */
			lowerTime++;
			overTime = 0;
			runlenth = -(frequency_difference / per_mm_Frequency) + 4.0 * lowerTime - 4.0;
			dir = MOTOR_DIRECTION_UP;
		} else if (frequency_difference < -g_deviceParams.oilLevelThreshold) { /* 可接受负偏差 */
			/* 标准向上移动 */
			overTime = 0;
			lowerTime = 0;
			runlenth = -(frequency_difference / per_mm_Frequency);
			dir = MOTOR_DIRECTION_UP;
		} else {  /* 频率在稳定范围内 */
			runlenth = 0;
			overTime = 0;
			lowerTime = 0;
		}

		/* 稳定性检测（连续稳定计数） */
		if ((abs(frequency_difference) <= g_deviceParams.oilLevelThreshold) && (g_measurement.oil_measurement.air_frequency - g_measurement.oil_measurement.current_frequency > 200)
				&& (g_measurement.oil_measurement.current_frequency - g_measurement.oil_measurement.oil_frequency > 200)) {
			followTime++;
			runlenth = 0;
			printf("频率跟随\t电机不动作\t等待频率稳定\t频率稳定次数%d\r\n", followTime);
		} else {
			followTime = 0;  /* 重置连续稳定计数 */
		}

		/* 电机移动控制 */
		if (runlenth != 0) {
			/* 步长微调策略 */
			if (runlenth < 5)
				runlenth = runlenth / 2;
			if (runlenth < 0.3)
				runlenth = 0.1;
			printf("频率跟随\t电机移动\t距离%f\t方向%s\r\n", runlenth, (dir == MOTOR_DIRECTION_UP) ? "上" : "下");
			/* 执行电机移动（带丢步检测） */
			ret = MotorCtrl_MoveAndWait(runlenth, dir, MotorCtrl_GetDefaultSpeedX100());
			/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
			if (NO_ERROR != ret)
				return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
		}

		/* 超限保护（连续多次加速仍无法跟踪） */
		if (overTime > MAX_TIMES_WHEN_FRE_FOLLOW || lowerTime > MAX_TIMES_WHEN_FRE_FOLLOW) {
			return OilLevel_StopBeforeReturn(MEASUREMENT_OVERSPEED, "探头频率异常");
		}

		/* 更新传感器位置及液位值 */
		ret = determineTheSensorPositionAndUpdateTheLevelValue();
		if (ret == MEASUREMENT_OILLEVEL_LOW) {
			/* 处理液位过低（进入盲区） */
			ret = waitForTheLiquidLevelToExceedTheBlindZone();
			CHECK_COMMAND_SWITCH(ret);
		}
		/* 先处理异常边界，避免液位测量状态机带故障继续运行。 */
		if (ret != NO_ERROR)
			return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
	}
	return NO_ERROR;  /* 成功定位液位界面 */
}

/**
 * @func: int determineTheSensorPositionAndUpdateTheLevelValue(void)
 * @description: 判断传感器位置并更新液位值
 * 主要功能：
 *  1. 根据磁角度计算当前油位值
 *  2. 根据稳定性条件更新液位测量值
 *  3. 检查油位是否超出上下限范围
 *  4. 当超限时停止电机并返回状态
 * @return:
 *  正常范围：NO_ERROR
 *  到达下限：MEASURE_DOWNLIMIT
 *  到达上限：MEASURE_UPLIMIT
 */
static int determineTheSensorPositionAndUpdateTheLevelValue(void) {
	int32_t oil_level;         /* 计算得到的当前油位高度（从参考点起算） */
/* int32_t ret; / / ret: 操作返回值 */

	oil_level = g_measurement.debug_data.sensor_position; /* 从传感器位置获取当前油位高度 */
	/* 判断是否需要更新液位值 */
	/* 条件1: 频率差在稳定阈值内（系统稳定） */
	/* 条件2: 新旧液位值差异大于100（需要强制更新） */
	/* 条件3：处在液位跟随状态 */
	/* if (((abs(frequency_difference) < g_deviceParams.oilLevelThreshold) || (abs((int) oil_level - (int) g_measurement.oil_measurement.oil_level) > 100)) && (g_measurement.device_status.device_state == STATE_FLOWOIL)) { */
	if  (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		/* 更新当前液位值 */
		g_measurement.oil_measurement.oil_level = OilLevel_ClampLevelForReport(oil_level, "液位跟随");
		g_measurement.density_distribution.Density_oil_level = g_measurement.oil_measurement.oil_level;
		/* 正常跟随更新结果时恢复有效标志，覆盖前序清零状态。 */
		g_measurement.oil_measurement.probe_at_liquid_level = 1U;
		g_measurement.oil_measurement.liquid_stable = 1U;
		OilLevel_UpdateAoOutput();
		/* 打印正常液位值信息 */
		printf("液位跟随\t液位值为%lu (0.1mm)", (unsigned long)g_measurement.oil_measurement.oil_level);
		OilLevel_PrintFollowPositionInfo();
		printf("\r\n");
	} else {
		/* 系统不稳定时仅打印动态液位值（不更新测量值） */
		printf("液位跟随\t动态液位值为%.1f", (float)g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\r\n");
	}

    int64_t oil_level_s64 = (int64_t)oil_level;
    int64_t upper_limit_01mm = (int64_t)g_deviceParams.tankHeight - 1000;

	if (((int64_t)g_deviceParams.tankHeight > 1000) &&
        (oil_level_s64 >= upper_limit_01mm)) {
		/* 到达上下限不是有效液位点，必须清除给外部协议看的命中状态。 */
		g_measurement.oil_measurement.probe_at_liquid_level = 0;
		g_measurement.oil_measurement.liquid_stable = 0;
		printf("超声波找液位\t到达位置上限\r\n");
		return OilLevel_StopBeforeReturn(MEASUREMENT_OILLEVEL_HIGH, "到达液位上限");
	}
	/* 步骤4: 负位置允许作为正常结果，上报时按0；非负且低于盲区才按下限处理。 */
	else if ((oil_level_s64 >= 0) && (oil_level_s64 < (int64_t)g_deviceParams.blindZone)) {
		/* 盲区内位置不作为 SI7000 的有效液位命中，避免 PLC 误判液位稳定。 */
		g_measurement.oil_measurement.probe_at_liquid_level = 0;
		g_measurement.oil_measurement.liquid_stable = 0;
		printf("超声波找液位\t到达位置下限\r\n");
		return OilLevel_StopBeforeReturn(MEASUREMENT_OILLEVEL_LOW, "到达液位下限");
	}
	/* 步骤5: 正常返回（无错误） */
	return NO_ERROR;
}

/**
 * @func: int waitForTheLiquidLevelToExceedTheBlindZone(void)
 * @description:在盲区等待，直到液位超过盲区退出
 * @return NO_ERROR
 */
static int waitForTheLiquidLevelToExceedTheBlindZone(void) {
    int32_t ret;
    /* 运行到盲区 */
    /* 等待脱离盲区期间液位尚未确认，外部协议侧保持未命中/不稳定。 */
    g_measurement.oil_measurement.probe_at_liquid_level = 0;
    g_measurement.oil_measurement.liquid_stable = 0;

    while (1) {
        if (HasEffectiveCommandSwitchRequest()) {
            printf("盲区等待\t检测到状态切换，退出等待\r\n");
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
        CHECK_COMMAND_SWITCH(ret);
        CHECK_ERROR(ret);    /* 检查开启液位模式是否成功 */
        printf("盲区等待\t频率阈值\t%ld\t当前频率\t%ld\t阈值差\t%f\r\n", g_measurement.oil_measurement.follow_frequency, g_measurement.oil_measurement.current_frequency,
        frequency_difference);
        if (g_measurement.oil_measurement.current_frequency > g_measurement.oil_measurement.follow_frequency) {
            break;
        }
        HAL_Delay(1000);
        if (HasEffectiveCommandSwitchRequest()) {
            printf("盲区等待\t检测到状态切换，退出等待\r\n");
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        /* 打断监测 */
    }
    return NO_ERROR;
}
/**
 * @brief 液位标定处理函数
 *
 * 该函数用于执行液位标定流程，包括：
 * - 根据缆线长度和标定液位计算并设置罐体高度
 * - 标定完成后清零标定液位值
 * - 更新传感器高度数据
 * - 保存设备参数
 *
 * @note 函数内部会调用以下辅助函数：
 *       - MotorCtrl_RefreshPositionFromActiveSource(): 按当前记步源刷新当前位置
 *       - save_device_params(): 保存设备参数
 *
 * @note 输出信息包括：
 *       - 标定开始提示
 *       - 标定完成后的罐体高度值
 */
void CorrectOilLevelProcess(void) {
    printf("液位流程\t开始标定液位\r\n");
    int64_t tank_height = (int64_t)g_measurement.debug_data.cable_length +
                          (int64_t)g_deviceParams.calibrateOilLevel + 1;

    if ((tank_height < 0) || (tank_height > (int64_t)UINT32_MAX)) {
        printf("液位流程\t修正后罐高非法：%ld(0.1mm)，取消修正，保留原罐高和修正参数\r\n",
               (long)tank_height);
        return;
    }
    g_deviceParams.tankHeight = (uint32_t)tank_height;
    g_deviceParams.calibrateOilLevel = 0; /* 标定完成后清零 */
    printf("液位流程\t标定完成，罐高设置为：%lu(0.1mm)\r\n", (unsigned long)g_deviceParams.tankHeight);
    MotorCtrl_RefreshPositionFromActiveSource();  /* 修正罐高后按当前记步源刷新当前位置 */
    OilLevel_SyncCurrentPositionToResult("液位修正");
    save_device_params(); /* 保存参数 */
}
