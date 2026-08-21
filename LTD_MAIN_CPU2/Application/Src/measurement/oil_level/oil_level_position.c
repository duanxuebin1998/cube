/*
 * 文件职责：提供所有液位算法共用的当前位置发布、边界保护和盲区等待流程。
 * 本文件不决定液位搜索策略，也不实现电机调速；结果提交统一调用运行时服务。
 */
#include "oil_level_internal.h"
#include "fault_manager.h"
#include "motor_ctrl.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 按当前设备状态发布实时液位位置。
 *
 * @details 只有设备处于液体流动状态时才提交液位结果；其它状态仅输出动态位置，
 *          避免粗找、盲区等待和异常处理过程覆盖最终液位结果。
 *
 * @param oil_level 当前传感器位置，单位为 0.1 mm。
 */
static void OilLevel_PositionPublishFollow(int32_t oil_level)
{
    if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
        /* 统一提交结果字段、稳定标志和 AO 样本，保持各算法输出口径一致。 */
        OilLevelRuntime_CommitLevel(oil_level, "液位跟随");
        printf("液位跟随\t液位值为%lu (0.1mm)",
               (unsigned long)g_measurement.oil_measurement.oil_level);
        OilLevel_PrintFollowPositionInfo();
        printf("\r\n");
    } else {
        printf("液位跟随\t动态液位值为%.1f",
               (float)g_measurement.debug_data.sensor_position / 10.0f);
        MotorCtrl_PrintPositionRefs();
        printf("\r\n");
    }
}

/**
 * @brief 检查当前位置是否越过罐高上限或盲区下限。
 *
 * @param oil_level 待检查的有符号液位，单位为 0.1 mm。
 * @return NO_ERROR 表示位置仍在允许区间；越界时返回对应液位错误码。
 * @note 边界位置不是有效液位点，退出前清除命中和稳定标志并统一停机。
 */
static uint32_t OilLevel_PositionCheckLimit(int32_t oil_level)
{
    int64_t oil_level_s64 = (int64_t)oil_level; /* 扩展后参与边界计算，避免无符号回绕。 */
    int64_t upper_limit_01mm = (int64_t)g_deviceParams.tankHeight - 1000; /* 罐高下方 100 mm 安全线。 */

    if (((int64_t)g_deviceParams.tankHeight > 1000) &&
        (oil_level_s64 >= upper_limit_01mm)) {
        g_measurement.oil_measurement.probe_at_liquid_level = 0U;
        g_measurement.oil_measurement.liquid_stable = 0U;
        printf("超声波找液位\t到达位置上限\r\n");
        /* 保留原错误码和停机路径，避免改变上层故障处理。 */
        return OilLevel_StopBeforeReturn(MEASUREMENT_OILLEVEL_HIGH, "到达液位上限");
    }
    if ((oil_level_s64 >= 0) &&
        (oil_level_s64 < (int64_t)g_deviceParams.blindZone)) {
        g_measurement.oil_measurement.probe_at_liquid_level = 0U;
        g_measurement.oil_measurement.liquid_stable = 0U;
        printf("超声波找液位\t到达位置下限\r\n");
        /* 保留原错误码和停机路径，避免改变上层故障处理。 */
        return OilLevel_StopBeforeReturn(MEASUREMENT_OILLEVEL_LOW, "到达液位下限");
    }

    return NO_ERROR;
}

/**
 * @brief 发布当前位置并执行统一边界检查。
 *
 * @details 密度、频率和旧步进流程共用此入口，消除算法模块对旧步进文件的依赖。
 *
 * @return NO_ERROR 表示位置有效；越界时返回液位过高或液位过低错误码。
 */
uint32_t OilLevel_UpdatePositionAndCheckBounds(void)
{
    int32_t oil_level = g_measurement.debug_data.sensor_position; /* 本轮传感器实时位置，单位为 0.1 mm。 */

    /* 先按当前设备状态发布位置，再复用统一边界保护。 */
    OilLevel_PositionPublishFollow(oil_level);
    return OilLevel_PositionCheckLimit(oil_level);
}

/**
 * @brief 在盲区内周期读取液位频率，直到频率越过跟随目标。
 *
 * @details 等待期间清除液位命中和稳定标志；每次读取前后检查命令切换，
 *          保留原有 1 秒等待节奏和传感器错误透传规则。
 *
 * @return NO_ERROR 表示频率已越过跟随目标；STATE_SWITCH 表示被命令切换打断；
 *         其它非零值表示传感器或流程错误。
 */
uint32_t OilLevel_WaitForBlindZone(void)
{
    uint32_t ret;

    /* 进入盲区等待后，液位尚未确认，外部协议保持未命中和不稳定。 */
    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;

    while (1) {
        if (HasEffectiveCommandSwitchRequest()) {
            printf("盲区等待\t检测到状态切换，退出等待\r\n");
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        /* 通过公共可靠读频接口读取，统一传感器错误和恢复语义。 */
        ret = OilLevel_ReadValidatedFrequency(&g_measurement.oil_measurement.current_frequency);
        CHECK_COMMAND_SWITCH(ret);
        CHECK_ERROR(ret);
        printf("盲区等待\t频率阈值\t%ld\t当前频率\t%ld\t阈值差\t%f\r\n",
               g_measurement.oil_measurement.follow_frequency,
               g_measurement.oil_measurement.current_frequency,
               (double)OilLevel_GetFrequencyDifference());
        if (g_measurement.oil_measurement.current_frequency >
            g_measurement.oil_measurement.follow_frequency) {
            break;
        }

        HAL_Delay(1000);
        if (HasEffectiveCommandSwitchRequest()) {
            printf("盲区等待\t检测到状态切换，退出等待\r\n");
            /* 延时后再次检查，避免命令切换被带入下一次测频。 */
            return STATE_SWITCH;
        }
    }

    return NO_ERROR;
}
