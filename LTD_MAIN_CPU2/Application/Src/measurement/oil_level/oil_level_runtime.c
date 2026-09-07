/*
 * 文件职责：所有液位算法共用的结果、副作用和退出服务。
 * 搜索、密度和频率模块在位置稳定后调用本文件。
 * 本文件不决定运动方向、
 * 目标值或闭环停止条件。
 */
#include "oil_level_runtime.h"
#include "oil_level_reference_refresh.h"
#include "AoOutput/ao_output.h"
#include "fault_manager.h"
#include "motor_ctrl.h"
#include "system_parameter.h"
#include <stdio.h>

/**
 * @brief 将运行域转换为诊断前缀，不改变任何测量状态。
 * @param domain 产生当前诊断的算法域。
 * @return 返回静态只读诊断文字。
 */
static const char *OilLevelRuntime_GetDomainText(OilLevelRuntimeDomain domain)
{
    switch (domain) {
    case OIL_LEVEL_RUNTIME_DOMAIN_DENSITY:
        return "密度找液位";
    case OIL_LEVEL_RUNTIME_DOMAIN_FREQUENCY:
        return "频率找液位";
    case OIL_LEVEL_RUNTIME_DOMAIN_SYNC:
        return "液位结果";
    default:
        return "液位";
    }
}

/*
 * 本模块统一处理各算法族共有的停机和结果副作用，不
 * 决定找液位策略或电机控制方式。
 */
uint32_t OilLevelRuntime_StopAndInvalidate(uint32_t error_code,
                                           const char *reason,
                                           OilLevelRuntimeDomain domain)
{
    if (error_code == NO_ERROR) {
        return NO_ERROR;
    }

    /* 预算截止仅通知刷新编排回退，真实停机失败仍必须使旧样本失效。 */
    if (error_code == OIL_LEVEL_REFRESH_STAGE_EXPIRED) {
        uint32_t stop_ret = MotorCtrl_SlowStop();
        if (stop_ret == NO_ERROR) { return error_code; }
        error_code = stop_ret;
    }

    AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_TANK_LEVEL);
    /* 取消不能掩盖真实停机失败；已有具体故障仍保留最初原因。 */
    {
        uint32_t stop_ret = MotorCtrl_SlowStop();
        if ((error_code == STATE_SWITCH) && (stop_ret != NO_ERROR) && (stop_ret != STATE_SWITCH)) {
            error_code = stop_ret;
        }
    }
    if ((domain != OIL_LEVEL_RUNTIME_DOMAIN_GENERIC) && (error_code != STATE_SWITCH)) {
        printf("%s\t%s\t停止并返回\t错误码=0x%08lX\r\n",
               OilLevelRuntime_GetDomainText(domain),
               (reason != NULL) ? reason : "闭环退出",
               (unsigned long)error_code);
    }
    return error_code;
}

/**
 * @brief 使本轮液位 AO 过程样本失效。
 * @note 在新一轮搜索开始或搜索失败退出时调用，避免继续发布上一轮液位。
 */
void OilLevelRuntime_InvalidateLevelSample(void)
{
    /* 新一轮找液位尚未成功前，撤销上一轮液位 AO 样本。 */
    AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_TANK_LEVEL);
}

/**
 * @brief 清除液位到位和稳定标志。
 * @note 在重新搜索或重新进入闭环前调用，不修改已经发布的液位数值。
 */
void OilLevelRuntime_ClearStableState(void)
{
    if (OilLevelReferenceRefresh_IsHolding()) { return; }
    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;
}

/*
 * 统一提交有符号传感器位置，确保
 * SI 标志、密度缓存、AO 样本和 AO 故障处理
 * 在找液位、跟随和校准流程中保持一致。
 */
static void OilLevelRuntime_CommitConfigured(int32_t oil_level, const char *reason,
                                             const OilLevelRefreshCandidate *reference)
{
    const char *commit_reason = (reason != NULL) ? reason : "记录"; /* 负位置诊断来源。 */
    uint32_t ao_ret; /* AO 刷新结果，仅在整机尚无故障时升级。 */
    uint32_t primask;

    /* 刷新离面位置仅用于安全检查，不能成为液位、密度液位缓存或AO的新过程样本。 */
    if (OilLevelReferenceRefresh_IsHolding()) { return; }

    if (oil_level < 0) {
        printf("液位结果\t%s位置为负%ld(0.1mm)，按0上报\r\n",
               commit_reason,
               (long)oil_level);
        oil_level = 0;
    }

    /* 板间中断不能读到新端点配旧液位的半提交快照；外设操作不进入临界区。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (reference != NULL) {
        g_measurement.oil_measurement.air_frequency = reference->air_frequency;
        g_measurement.oil_measurement.oil_frequency = reference->oil_frequency;
        g_measurement.oil_measurement.follow_frequency = reference->target_frequency;
    }
    g_measurement.oil_measurement.oil_level = (uint32_t)oil_level;
    g_measurement.density_distribution.Density_oil_level =
            g_measurement.oil_measurement.oil_level;
    g_measurement.oil_measurement.probe_at_liquid_level = 1U;
    g_measurement.oil_measurement.liquid_stable = 1U;

    if (g_measurement.oil_measurement.oil_level == UNVALID_LEVEL) {
        AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_TANK_LEVEL);
    } else {
        AoOutput_PublishProcessSample(AO_PROCESS_SOURCE_TANK_LEVEL,
                                      (int32_t)g_measurement.oil_measurement.oil_level,
                                      1U);
    }

    __set_PRIMASK(primask);
    ao_ret = AoOutput_Update();
    if ((ao_ret != NO_ERROR) && (ao_ret != STATE_SWITCH) &&
        (g_measurement.device_status.error_code == NO_ERROR)) {
        /* AO 刷新失败时转入通用设备故障状态。 */
        FaultManager_SetErrorState(ao_ret,
                                    GetShortFilename(__FILE__),
                                    __LINE__,
                                    __func__);
    }
}

/* 普通液位结果继续通过统一提交入口发布，不改变端点。 */
void OilLevelRuntime_CommitLevel(int32_t oil_level, const char *reason)
{
    OilLevelRuntime_CommitConfigured(oil_level, reason, NULL);
}

/* 刷新候选必须已通过稳定和运动保护，由刷新编排解除保持后调用。 */
void OilLevelRuntime_CommitReference(int32_t oil_level, const OilLevelRefreshCandidate *reference)
{
    OilLevelRuntime_CommitConfigured(oil_level, "液位定时矫正", reference);
}

/*
 * 读取当前有效位置并输出算法族诊断日志；
 * 结果提交与产生位置的具体算法解耦；调用方提供算法族，
 * 使结果更新不依赖具体的位置产生算法。
 */
void OilLevelRuntime_RecordCurrentPosition(const char *tag,
                                           OilLevelRuntimeDomain domain)
{
    const char *reason = (tag != NULL) ? tag : "记录";

    /* 所有算法族通过同一入口提交位置、稳定标志和 AO 样本。 */
    OilLevelRuntime_CommitLevel(g_measurement.debug_data.sensor_position, reason);

    if (domain == OIL_LEVEL_RUNTIME_DOMAIN_SYNC) {
        printf("液位结果\t%s同步液位=%lu(0.1mm)\r\n",
               reason,
               (unsigned long)g_measurement.oil_measurement.oil_level);
    } else {
        printf("%s\t%s\t液位=%lu(0.1mm)\r\n",
               OilLevelRuntime_GetDomainText(domain),
               reason,
               (unsigned long)g_measurement.oil_measurement.oil_level);
    }
}
