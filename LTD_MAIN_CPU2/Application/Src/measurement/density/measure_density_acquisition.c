/*
 * measure_density_acquisition.c
 *
 * 文件职责：实现密度传感器稳定窗口采集和单点读数封装，供分布测量与单点命令共用。
 */

#include "measure_density.h"
#include "measure_density_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "abortable_delay.h"
#include "error_log.h"
#include "fault_manager.h"
#include "sensor_service.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"

/**
 * @brief 单点密度测量（带稳定判定与超时兜底）。
 *
 * 【总体逻辑】。
 * 1) 周期性读取 频率 / 密度 / 温度。
 * 2) 仅当“密度为非零”时，才参与稳定判定。
 * 3) 若在稳定窗口内，三项数据变化均不超过阈值，则判定“数据稳定”。
 * 4) 若 5 分钟内始终未稳定：- 若曾读到非零密度：取最后一次非零密度作为结果。
 * 若 5 分钟内从未读到非零密度：输出 0 作为结果。
 * 【关键口径】。
 * 密度 == 0：- 不参与稳定判定。
 * 不能作为“稳定值”。
 * 但在“完全无有效密度”的异常场景下，可作为最终兜底输出。
 * 其他错误码 模式切换/通信等异常。
 *
 * @param result 单点测量结果结构体（RAW 编码）。
 * @param stable_win_ms 传感器读数必须持续满足稳定条件的窗口时长，单位 ms。
 * @param stable_out 用于返回稳定窗口是否已满足发布条件。
 * @return NO_ERROR 成功（稳定或兜底）。
 */
uint32_t DensityInternal_ReadSensorWithStableWindow(volatile DensityMeasurement *result,
                                                       uint32_t stable_win_ms,
                                                       uint8_t *stable_out)
{
    uint32_t ret = 0;

    if (stable_out != NULL) {
        *stable_out = 0U;
    }
    if (result == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    /* ---------- 切换到密度测量模式（防御性调用） ---------- */
    ret = SensorService_EnableDensityMode();
    if (ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    CHECK_ERROR(ret);


    /* 若参数未配置，使用默认 5 秒 */
    if (stable_win_ms == 0) {
        stable_win_ms = 5000U;
    }

    /* ---------- 最大等待时间：5 分钟 ----------
     * 防止传感器异常或工况不稳定导致死等
     */
    const uint32_t MAX_WAIT_MS = 5U * 60U * 1000U;

    /* ---------- 稳定判定阈值 ----------
     * 任一项超出阈值，均认为“不稳定”，需要重新计时
     */
    const float FREQ_EPS    = 1.0f;   /* 频率变化阈值（Hz） */
    const float DENSITY_EPS = 0.1f;   /* 密度变化阈值 */
    const float TEMP_EPS    = 0.2f;   /* 温度变化阈值（℃） */

    /* 采样周期 */
    const uint32_t SAMPLE_INTERVAL_MS = 200U;
    const uint32_t density_sample_retry_max = MAX_WAIT_MS / SAMPLE_INTERVAL_MS;
    uint32_t density_read_retry_count = 0U;
    uint32_t density_zero_retry_count = 0U;

    /* ---------- 时间与状态变量 ---------- */
    uint32_t t_start      = HAL_GetTick();  /* 整个流程起始时间 */
    uint32_t stable_start = 0;              /* 当前稳定窗口起始时间 */
    uint8_t  first_sample = 1;              /* 是否为首次有效样本 */

    /* ---------- 参考值（用于稳定判定） ----------
     * 仅在“非零密度样本”下才会更新
     */
    float ref_freq = 0.0f;
    float ref_density = 0.0f;
    float ref_temp = 0.0f;

    /* 当前读取值 */
    float cur_freq = 0.0f;
    float cur_density = 0.0f;
    float cur_temp = 0.0f;

    /* ---------- 超时兜底用变量 ----------
     * 用于记录“最后一次非零密度样本”
     */
    uint8_t have_last_nonzero = 0;
    float last_density_nz = 0.0f;
    float last_temp_nz = 0.0f;

    while (1) {

        /* 固定点监测会长期调用本函数；每轮采样前先检查命令切换，避免传感器异常时卡在内部循环。 */
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }

        uint32_t now = HAL_GetTick();

        /* ======================================================
         * 1) 超时兜底处理（5 分钟）
         * ====================================================== */
        if (now - t_start >= MAX_WAIT_MS) {

            /* --- 情况 A：曾经读到过非零密度 --- */
            if (have_last_nonzero) {

                printf("单点测量 超时未稳定，取最后一次非零密度作为结果。\r\n");

                result->density          = DENSITY_TO_RAW(last_density_nz);
                result->temperature      = TEMP_TO_RAW(last_temp_nz);
                result->standard_density = DENSITY_TO_RAW(last_density_nz);
                result->weight_density   = DENSITY_TO_RAW(last_density_nz);

            }
            /* --- 情况 B：5 分钟内从未读到非零密度 --- */
            else {

                printf("单点测量 5分钟内未读到非零密度，输出0作为结果。\r\n");

                /* 注意：
                 * 这里的 0 仅用于流程兜底，不代表有效密度
                 */
                result->density          = 0;
                result->standard_density = 0;
                result->weight_density   = 0;

                /* 温度可取最近一次值（即使为 0） */
                result->temperature = TEMP_TO_RAW(cur_temp);
            }

            result->vcf20 = 1;
            result->temperature_position = DensityInternal_CurrentPositionToU01mmClamped();

            return NO_ERROR;
        }

        /* ======================================================
         * 2) 读取传感器
         * ====================================================== */
        ret = SensorService_ReadDensity(&cur_freq, &cur_density, &cur_temp);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            density_read_retry_count++;
            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_READ_FLOAT_PARAM,
                           ErrorLog_GetReasonByCode(ret),
                           1U,
                           1U,
                           ret);
            /* SensorService_ReadDensity() 底层已经完成协议重试和链路诊断；这里必须向上返回错误码，
             * 让固定点监测外层 SET_ERROR(ret) 置错误状态，而不是在内部循环吞掉故障。 */
            return ret;
        }
        if (density_read_retry_count > 0U) {
            ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                             ERROR_LOG_OP_READ_FLOAT_PARAM,
                             ERROR_LOG_REASON_RECOVER_OK,
                             density_read_retry_count + 1U,
                             density_sample_retry_max);
        }
        density_read_retry_count = 0U;

        CHECK_COMMAND_SWITCH(ret);

        printf("单点读数: 位置=%.1fmm  频率=%.3f Hz  密度=%.4f  温度=%.3f ℃\r\n",
               (double)g_measurement.debug_data.sensor_position / 10.0,
               cur_freq, cur_density, cur_temp);

        /* ======================================================
         * 3) 密度为 0 的处理策略
         * ======================================================
         *  - 认为是“无效密度”
         *  - 不参与稳定判定
         *  - 不更新参考值
         */
        if (fabsf(cur_density) < 1e-6f) {
            density_zero_retry_count++;
            if ((density_zero_retry_count == 1U) ||
                ((density_zero_retry_count % 5U) == 0U)) {
                printf("单点读数: 密度为0，等待稳定/未测到，已等待约%lu ms\r\n",
                       (unsigned long)(density_zero_retry_count * SAMPLE_INTERVAL_MS));
            }
            /* 密度为 0 时会持续重试；这里同样使用可打断延时响应退出命令。 */
            ret = AbortableDelay_CommandSwitch(SAMPLE_INTERVAL_MS, 50U);
            if (ret != NO_ERROR) {
                return ret;
            }
            continue;
        }
        if (density_zero_retry_count > 0U) {
            printf("单点读数: 密度已恢复为非零，等待稳定次数=%lu\r\n",
                   (unsigned long)density_zero_retry_count);
        }
        density_zero_retry_count = 0U;

        /* 记录最后一次“非零密度”样本（用于超时兜底） */
        have_last_nonzero = 1;
        last_density_nz   = cur_density;
        last_temp_nz      = cur_temp;

        /* ======================================================
         * 4) 稳定判定逻辑（仅对非零密度生效）
         * ====================================================== */
        if (first_sample) {

            /* 首次有效样本：直接作为参考值 */
            ref_freq    = cur_freq;
            ref_density = cur_density;
            ref_temp    = cur_temp;
            stable_start = now;
            first_sample = 0;

        } else {

            float df = fabsf(cur_freq    - ref_freq);
            float dd = fabsf(cur_density - ref_density);
            float dt = fabsf(cur_temp    - ref_temp);

            /* 任一项超阈值，认为不稳定，重置参考值与计时 */
            if (df > FREQ_EPS || dd > DENSITY_EPS || dt > TEMP_EPS) {
                ref_freq    = cur_freq;
                ref_density = cur_density;
                ref_temp    = cur_temp;
                stable_start = now;
            }
        }

        /* ======================================================
         * 5) 稳定窗口满足：判定稳定
         * ====================================================== */
        if (!first_sample && (now - stable_start >= stable_win_ms)) {

            printf("单点测量 数据稳定，位置=%.1fmm，稳定窗口=%lu ms\r\n",
                   (double)g_measurement.debug_data.sensor_position / 10.0,
                   (unsigned long)stable_win_ms);

            result->temperature_position = DensityInternal_CurrentPositionToU01mmClamped();
            result->density          = DENSITY_TO_RAW(ref_density);
            result->temperature      = TEMP_TO_RAW(ref_temp);
            result->standard_density = DENSITY_TO_RAW(ref_density);
            result->weight_density   = DENSITY_TO_RAW(ref_density);
            result->vcf20            = 1;

            if (stable_out != NULL) {
                *stable_out = 1U;
            }
            return NO_ERROR;
        }

        /* 普通采样间隔也要可打断，固定点监测才能在稳定等待期间退出。 */
        ret = AbortableDelay_CommandSwitch(SAMPLE_INTERVAL_MS, 50U);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
}

/**
 * @brief 读取单点密度、温度和频率并写入测量结果。
 *
 * @param result 单点测量输出记录；读取成功时写入频率、密度、温度及当前位置等字段。
 * @return NO_ERROR 表示频率、密度和温度已通过稳定窗口并写入单点结果；其他值为传感器通信、数据有效性、命令切换或稳定采样失败码。
 */
uint32_t SinglePoint_ReadSensor(volatile DensityMeasurement *result)
{
    uint32_t stable_win_ms = g_deviceParams.spreadPointHoverTime * 1000U;
    uint32_t ret;
    uint8_t stable = 0U;

    do {
        ret = DensityInternal_ReadSensorWithStableWindow(result, stable_win_ms, &stable);
        if (ret != NO_ERROR) {
            return ret;
        }
        if (stable == 0U) {
            printf("固定点等待超过5分钟仍未稳定，本轮兜底不发布，重新开始稳定窗口\r\n");
        }
    } while (stable == 0U);

    return NO_ERROR;
}
