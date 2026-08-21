/**
 * @file water_level_fast_search.c
 * @brief 通过水区状态翻转估算初始水位并对齐到跟随起点。
 */

#include "water_level_internal.h"
#include "AoOutput/ao_output.h"
#include "fault_manager.h"
#include "measure_zero.h"
#include "motor_ctrl.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"

#include <stdint.h>
#include <stdio.h>

/**
 * @brief 停机后对齐到目标水位附近（通过“目标水位 -> 目标缆长 -> 计算delta -> 精确移动”）
 * @param[in] lvl_target_01mm   目标水位（0.1mm），例如 lvl_avg
 * @return NO_ERROR / 错误码
 */
static uint32_t AlignToWaterLevel_01mm(int32_t lvl_target_01mm)
{
    uint32_t ret;

    int32_t cable_now_01mm    = g_measurement.debug_data.cable_length; /* 0.1mm */
    int32_t cable_target_01mm = WaterLevel_CableTargetFromLevel(lvl_target_01mm); /* 0.1mm */
    int32_t delta_01mm        = cable_target_01mm - cable_now_01mm; /* 0.1mm */

    int32_t abs_delta_01mm = (delta_01mm >= 0) ? delta_01mm : -delta_01mm;

    printf("快速跟随\t对齐: 目标液位=%.1fmm 当前缆长=%.1fmm 目标缆长=%.1fmm 差值：%+.1fmm\r\n",
           lvl_target_01mm / 10.0f,
           cable_now_01mm / 10.0f,
           cable_target_01mm / 10.0f,
           delta_01mm / 10.0f);

    uint32_t dir = (delta_01mm >= 0) ? MOTOR_DIRECTION_DOWN : MOTOR_DIRECTION_UP;
    float move_mm = abs_delta_01mm / 10.0f;

    printf("快速跟随\t对齐: 方向：%s 移动=%.1fmm\r\n",
           (dir == MOTOR_DIRECTION_DOWN) ? "DOWN" : "UP",
           move_mm);

    ret = MotorCtrl_MoveAndWait(move_mm, dir, MotorCtrl_GetDefaultSpeedX100());
    if (ret != NO_ERROR) return ret;

    return NO_ERROR;
}

/**
 * @brief 快速找跟随点（基于“状态翻转 + 窗口判稳”）
 *
 * 流程梳理：
 * 1. 先读取当前是 WATER_LEVEL_STATE_DETECTED 还是 WATER_LEVEL_STATE_NORMAL
 * 2. 若当前在水里，则连续上行直到提出水面；若当前不在水里，则连续下行直到进入水里
 * 3. 每发生一次 WATER_LEVEL_STATE_DETECTED/WATER_LEVEL_STATE_NORMAL 翻转，就记录一次翻转水位
 * 4. 从第二次翻转开始，用最近两次翻转位置的平均值作为当前液面估计值
 * 5. 若连续 stable_win_ms 内该估计值波动不超过阈值，则认为“快速找点完成”，停机并对齐到目标水位附近
 *
 * 说明：
 * - 这个函数的职责是“给 FollowWaterLevelCore 找一个初始跟随点”
 * - 它本身不是长期跟随主循环；退出后由上层立即转入闭环跟随
 * - 当前上层快速模式传入 stable_win_ms=0，因此只要求形成可用跟随点，不在这里长期等待
 *
 * 依赖：
 *  - MotorCtrl_MoveUp()/MotorCtrl_MoveDown(): 每次调用推动继续运动（你现有粗找就是这样用的）
 *  - WaterLevel_CheckStatus(): 返回 WATER_LEVEL_STATE_DETECTED / WATER_LEVEL_STATE_NORMAL
 *  - MotorCtrl_CheckLostStepAutoTiming(): 丢步检测（可选但建议保留）
 *
 * @param stable_win_ms 稳定判定窗口时长（ms）
 * 退出条件：
 *   连续 stable_win_ms 内 water_level 波动（max-min）不超过阈值 -> 认为稳定找到水位，退出
 *
 * @return NO_ERROR 表示检测到水区状态翻转并通过稳定窗口确认；命令切换、超时、传感器通信或电机错误由过程检查返回。
 */
uint32_t FindWaterLevel_FastByStateFlip_StableExit(uint32_t stable_win_ms)
{
    uint32_t ret;
    WaterLevelState water_state = WATER_LEVEL_STATE_NORMAL;

    /* ===== 稳定判定参数 ===== */
    const uint32_t stable_window_ms = stable_win_ms;

    /* ===== 零点保护（沿用你之前逻辑，可按需删） ===== */
    const int32_t ZERO_NEAR_TH = 1000; /* 100mm -> 1000(0.1mm) */

    /* 稳定窗口统计量 */
    uint32_t win_start_tick = HAL_GetTick();
    int32_t  min_level =  2147483647;
    int32_t  max_level = -2147483647;
    /* 翻转缓存 */
    static uint8_t have_last_flip = 0;
    static int32_t last_flip_lvl  = 0;
    AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    have_last_flip = 0; /* 每次调用都重置，确保独立测量 */
    /* -------------------- 零点检查 -------------------- */
    if (g_measurement.water_measurement.zero_capacitance == 0)
    {
        if(g_deviceParams.zero_cap==0){
            printf("水位测量\t获取零点电容值\r\n");
            ret = SearchZero();
            CHECK_ERROR(ret);
            printf("水位测量\t回零点完成\r\n");
        }
        else {
			g_measurement.water_measurement.zero_capacitance = (float)g_deviceParams.zero_cap/10.0;
		}
    }
    /* 打印零点电容值，水位电容阈值，水位滞后阈值 */
    printf("水位测量\t零点电容值：%lu\r\n", g_deviceParams.zero_cap);
    printf("水位测量\t水位电容阈值：%lu\r\n", g_deviceParams.water_cap_threshold);
    printf("水位测量\t水位寻找电容阈值：%lu\r\n", g_deviceParams.water_find_cap_threshold);
    printf("快速跟随\t开始(稳定判定：%lu.%03lus内波动<=%.1fmm退出)\r\n",
           (unsigned long)(stable_window_ms / 1000u),
           (unsigned long)(stable_window_ms % 1000u),
           g_deviceParams.water_stable_threshold / 10.0f);

    /* 初始状态 */
    ret = WaterLevel_CheckStatus(&water_state);
    CHECK_ERROR(ret);

    while (1)
    {
        /* ===================== 段 A：在水里 -> 连续上行直到出水(WATER_LEVEL_STATE_NORMAL) ===================== */
        if (water_state == WATER_LEVEL_STATE_DETECTED)
        {
            printf("快速跟随\t当前：水区 -> 连续上行直到空气区\r\n");
            MotorCtrl_LostStepInit();

            while (1)
            {
                if (g_measurement.debug_data.cable_length <= ZERO_NEAR_TH)
                {
                    printf("快速跟随\t零点附近仍在水区，停止(位置=%.1fmm)\r\n",
                           g_measurement.debug_data.sensor_position / 10.0f);
                    RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
                }

                ret = MotorCtrl_MoveUp(MotorCtrl_GetDefaultSpeedX100());
                CHECK_ERROR(ret);

                ret = WaterLevel_CheckStatus(&water_state);
                CHECK_ERROR(ret);

                if (water_state == WATER_LEVEL_STATE_NORMAL) {
                    break;
                }

                ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
                CHECK_ERROR(ret);

                CHECK_COMMAND_SWITCH(NO_ERROR);
            }
        }
        /* ===================== 段 B：不在水里 -> 连续下行直到进水(WATER_LEVEL_STATE_DETECTED) ===================== */
        else
        {
            printf("快速跟随\t当前：空气区 -> 连续下行直到水区\r\n");
            MotorCtrl_LostStepInit();

            while (1)
            {
                ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());
                CHECK_ERROR(ret);

                ret = WaterLevel_CheckStatus(&water_state);
                CHECK_ERROR(ret);

                if (water_state == WATER_LEVEL_STATE_DETECTED) {
                    break;
                }

                ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
                CHECK_ERROR(ret);

                CHECK_COMMAND_SWITCH(NO_ERROR);
            }
        }

        /* ===== 翻转结束：先计算“本次翻转水位样本” ===== */
        int32_t lvl_flip = WaterLevel_CalcFromCable(); /* 0.1mm */


        /* 第一次翻转：只缓存，不更新水位/不判稳 */
        if (!have_last_flip)
        {
            have_last_flip = 1;
            last_flip_lvl  = lvl_flip;
            win_start_tick = HAL_GetTick(); /* 开始稳定判定计时 */
            printf("快速跟随\t第1次翻转 液位=%.1fmm -> 缓存(等待第二次翻转后才开始更新/判稳)\r\n",
                   lvl_flip / 10.0f);

            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue; /* 回到外层 while(1)，继续下一次翻转 */
        }

        /* 第二次及以后：用最近两次翻转的平均值作为“当前水位” */
        int32_t lvl_avg;
        {
            /* 无偏平均：避免奇数和截断偏差 */
            int32_t a = last_flip_lvl;
            int32_t b = lvl_flip;
            lvl_avg = (a / 2) + (b / 2) + ((a & 1) && (b & 1));  /* 两个都是奇数时补 1 */
        }

        /* 更新“对外水位值”（只在两次翻转后才更新） */
        WaterLevel_SetAndLog(lvl_avg);

        printf("快速跟随\t翻转液位1=%.1f  液位2=%.1f  平均=%.1fmm\r\n",
               last_flip_lvl / 10.0f,
               lvl_flip      / 10.0f,
               lvl_avg       / 10.0f);

        /* 更新缓存：为下一次平均做准备 */
        last_flip_lvl = lvl_flip;

        /* ===== 稳定判定窗口统计（从第二次翻转开始） ===== */
        {
            int32_t lvl = lvl_avg; /* 0.1mm */

            if (lvl < min_level) min_level = lvl;
            if (lvl > max_level) max_level = lvl;

            if ((max_level - min_level) > g_deviceParams.water_stable_threshold)
            {
                win_start_tick = HAL_GetTick();
                min_level = lvl;
                max_level = lvl;
                printf("快速跟随\t波动超限 -> 重置稳定窗口(最小=最大=%.1fmm)\r\n", lvl / 10.0f);
                /* 如果设备状态不是水位跟随状态，设置水位跟随状态 */
				if (g_measurement.device_status.device_state != STATE_FOLLOW_WATERING) {
					g_measurement.device_status.device_state = STATE_FOLLOW_WATERING;
				}
            }
            else
            {
                uint32_t elapsed = HAL_GetTick() - win_start_tick;

                printf("快速跟随\t稳定窗口：%lus  波动=%.1fmm(阈值%.1fmm)\r\n",
                       (unsigned long)(elapsed / 1000u),
                       (max_level - min_level) / 10.0f,
                       g_deviceParams.water_stable_threshold / 10.0f);

                if (elapsed >= stable_window_ms)
                {
                    printf("快速跟随\t稳定满足：连续%lu.%03lus内波动<=阈值 -> 退出\r\n",
                           (unsigned long)(stable_window_ms / 1000u),
                           (unsigned long)(stable_window_ms % 1000u));
                    ret = MotorCtrl_SlowStop();
                    CHECK_ERROR(ret);
                    printf("快速跟随\t运行到水位附近\r\n");
                    ret = AlignToWaterLevel_01mm(lvl_avg);
                    CHECK_ERROR(ret);
                    return NO_ERROR;
                }
            }
        }

        CHECK_COMMAND_SWITCH(NO_ERROR);
    }
}
