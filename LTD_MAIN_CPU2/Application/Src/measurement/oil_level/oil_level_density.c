/*
 * 文件职责：密度液位查找和跟随闭环实现。
 * 本文件负责密度目标、死区和调速逻辑，结果副作用统一交给运行时服务。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "oil_level_reference_refresh.h"
#include "abortable_delay.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 使液位 AO 样本失效、慢停电机并返回原密度闭环错误码。
 * @param error_code 需要保持的原错误码或 STATE_SWITCH。
 * @param reason 密度闭环退出原因，用于诊断日志。
 * @return 完成统一清理后的原 error_code。
 */
static uint32_t DensityLevel_StopAndReturn(uint32_t error_code, const char *reason)
{
    return OilLevelRuntime_StopAndInvalidate(
            error_code,
            reason,
            OIL_LEVEL_RUNTIME_DOMAIN_DENSITY);
}

/**
 * @brief 把密度液位阈值原始值转换为闭环死区，并为未配置值选用默认死区。
 *
 * @param raw_threshold 设备参数中的密度阈值原始值，单位 0.01 kg/m3；0 表示未配置。
 * @return raw_threshold 为 0 时返回默认死区 0.5 kg/m3；否则把 0.01 kg/m3 原始阈值换算为 kg/m3 浮点死区。
 */
static float DensityLevel_GetDeadband(uint32_t raw_threshold)
{
    if (raw_threshold == 0U) {
        return DENSITY_LEVEL_DEFAULT_DEADBAND_KGM3;
    }
    return RAW_TO_DENSITY(raw_threshold);
}

/**
 * @brief 返回密度液位闭环进入死区后的固定稳定等待时间。
 *
 * @return 返回 OIL_LEVEL_CLOSED_LOOP_STABLE_DELAY_MS 定义的固定稳定等待时间，单位 ms，当前配置为 1000 ms。
 */
uint32_t DensityLevel_GetStableDelayMs(void)
{
    return OIL_LEVEL_CLOSED_LOOP_STABLE_DELAY_MS;
}

/**
 * @brief 读取频率、密度和温度，并拒绝超出 0～3000 kg/m3 的密度样本。
 *
 * @param density 实时密度输出指针，成功时写入 kg/m3 浮点值，并由函数校验 0 至 3000 kg/m3 范围。
 * @param frequency 实时密度传感器频率输出指针，成功时写入 Hz 浮点值。
 * @param temperature 实时温度输出指针，成功时写入 ℃ 浮点值。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t DensityLevel_ReadCurrent(float *density, float *frequency, float *temperature)
{
    uint32_t ret;

    if ((density == NULL) || (frequency == NULL) || (temperature == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = SensorService_ReadDensity(frequency, density, temperature);
    if (ret != NO_ERROR) {
        return ret;
    }

    if ((*density <= 0.0f) || (*density > 3000.0f)) {
        printf("密度找液位\t密度值无效\t密度=%.3f\r\n", (double)*density);
        return DENSITY_INVALID;
    }

    return NO_ERROR;
}

/**
 * @brief 按超出死区的密度误差线性计算速度，并限制到有效速度范围。
 *
 * @param density_error 密度故障。
 * @param deadband 目标判定使用的死区宽度，与函数处理的频率或密度值采用相同单位。
 * @param max_speed_x100 液位闭环允许使用的最大电机速度，单位 0.01 m/min。
 * @return 返回按密度误差计算并钳位后的电机线速度，单位 0.01 m/min；误差位于死区内时返回 0。
 */
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

/**
 * @brief 反向前先慢停，同向小幅调速不重复下发，其余情况更新运动方向和速度。
 *
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @param active_dir 用于返回当前已经启动或继续执行的电机运动方向。
 * @param active_speed_x100 当前液位闭环已经下发的电机速度，单位 0.01 m/min；用于判断是否需要重发速度命令。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */

static void DensityLevel_RecordCurrentPosition(const char *tag)
{
    OilLevelRuntime_RecordCurrentPosition(tag, OIL_LEVEL_RUNTIME_DOMAIN_DENSITY);
}

/**
 * @brief 按密度目标执行液位搜索或跟随闭环，并统一处理无效样本、命令切换、调速和停机。
 *
 * 函数要求 oilLevelDensity 已配置，把内部原始密度换算为 kg/m3；查找模式使用 oilLevelThreshold 死区，跟随模式改用
 * oilLevelHysteresisThreshold，并在启动时清除旧的到液面和稳定标志。
 * 切换到密度模式后周期读取密度、频率和温度；短时 DENSITY_INVALID 允许按上限累计重试，连续无效达到 DENSITY_LEVEL_INVALID_DENSITY_LIMIT
 * 才作为故障退出。
 * 目标密度减当前密度为正时下行、为负时上行；落入死区后停止速度运动并累计稳定样本，达到门限时记录当前液位、同步密度分布液位并刷新 AO 过程样本。
 * 查找模式确认稳定后返回成功；跟随模式保持闭环运行，离开死区后重新清除稳定标志，并按密度偏差计算、钳位 0.01 m/min 电机速度。
 * 运动期间持续检查业务位置、扭力碰撞和丢步；查找模式另有总超时，跟随模式没有正常超时退出。所有异常和命令切换都经 DensityLevel_StopAndReturn 慢停电机并使本轮 AO
 * 过程样本失效。
 *
 * @param follow_mode 0 表示执行一次密度液位查找并在稳定后返回；非 0 表示使用跟随滞回死区持续闭环，直至命令切换或故障。
 * @return 查找模式稳定完成返回 NO_ERROR；STATE_SWITCH
 *         表示被新命令正常打断，其他值区分目标未配置、连续密度无效、传感器、位置、扭力、丢步、电机控制和查找超时错误；跟随模式正常运行时不主动返回。
 * @note follow_mode 非 0 时函数设计为长期运行，不应把缺少正常返回理解为死循环缺陷；退出由命令切换和安全故障驱动。
 */
uint32_t DensityLevel_RunClosedLoop(uint32_t follow_mode)
{
    uint32_t ret;
    uint32_t start_tick;
    uint32_t stable_count = 0U;
    uint32_t invalid_density_count = 0U;
    int active_dir = OIL_LEVEL_DIRECTION_NONE;
    uint32_t active_speed_x100 = 0U;
    float target_density;
    float deadband;
    float follow_deadband;

    if (g_deviceParams.oilLevelDensity == 0U) {
        printf("密度找液位\t目标密度未配置\r\n");
        return PARAM_CONFIG_MISSING;
    }

    target_density = RAW_TO_DENSITY(g_deviceParams.oilLevelDensity);
    deadband = DensityLevel_GetDeadband(g_deviceParams.oilLevelThreshold);
    follow_deadband = DensityLevel_GetDeadband(g_deviceParams.oilLevelHysteresisThreshold);
    if (follow_mode != 0U) {
        deadband = follow_deadband;
    }

    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;

    ret = SensorService_EnableDensityMode();
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
            if (follow_mode != 0U) {
                uint32_t refresh_ret = OilLevelReferenceRefresh_DensitySample(false, density, frequency, temperature);
                if (refresh_ret != NO_ERROR) { return DensityLevel_StopAndReturn(refresh_ret, "密度周期复核退出"); }
            }
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
            dir = OIL_LEVEL_DIRECTION_NONE;
        }

        if (dir == OIL_LEVEL_DIRECTION_NONE) {
            ret = LevelVelocity_StartOrUpdateMotion(dir, 0U, &active_dir, &active_speed_x100);
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
            if (follow_mode != 0U) {
                ret = OilLevelReferenceRefresh_DensitySample(stable_count >= DENSITY_LEVEL_STABLE_COUNT,
                                                             density, frequency, temperature);
                if (ret != NO_ERROR) { return DensityLevel_StopAndReturn(ret, "密度周期复核退出"); }
            }
            ret = AbortableDelay_CommandSwitch(DensityLevel_GetStableDelayMs(), 100U);
            if (ret != NO_ERROR) {
                return DensityLevel_StopAndReturn(ret, "命令切换");
            }
            continue;
        }

        stable_count = 0U;
        if (follow_mode != 0U) {
            ret = OilLevelReferenceRefresh_DensitySample(false, density, frequency, temperature);
            if (ret != NO_ERROR) { return DensityLevel_StopAndReturn(ret, "密度周期复核退出"); }
        }
        g_measurement.oil_measurement.probe_at_liquid_level = 0U;
        g_measurement.oil_measurement.liquid_stable = 0U;
        speed_x100 = DensityLevel_ComputeSpeedX100(density_error, deadband, MotorCtrl_GetDefaultSpeedX100());
        printf("密度找液位\t速度闭环\t方向=%s\t速度=%.2f m/min\r\n",
               MotorCtrl_DirectionText(dir),
               (double)speed_x100 / 100.0);

        ret = LevelVelocity_StartOrUpdateMotion(dir, speed_x100, &active_dir, &active_speed_x100);
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "启动速度模式失败");
        }

        ret = OilLevel_UpdatePositionAndCheckBounds();
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "位置越界");
        }

        ret = CheckWeightCollision();
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "扭力碰撞");
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "丢步检测失败");
        }

        if ((follow_mode == 0U) &&
            ((HAL_GetTick() - start_tick) > DENSITY_LEVEL_SEARCH_TIMEOUT_MS)) {
            return DensityLevel_StopAndReturn(MEASUREMENT_DENSITY_LEVEL_TIMEOUT, "密度闭环超时");
        }

        ret = AbortableDelay_CommandSwitch(DENSITY_LEVEL_SAMPLE_DELAY_MS, 50U);
        if (ret != NO_ERROR) {
            return DensityLevel_StopAndReturn(ret, "命令切换");
        }
    }
}



/**
 * @brief 频率连续找液位异常退出统一停机，避免速度模式保持运行。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是液位流程准备返回的失败原因，函数先停止电机并保持原错误归因。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @return 非零 error_code 在慢停后原样返回，以保留频率闭环退出根因；正常路径返回 NO_ERROR。
 */
