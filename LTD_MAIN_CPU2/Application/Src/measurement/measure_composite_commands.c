/*
 * measure_composite_commands.c
 *
 * 文件职责：封装综合测量命令，统一发布完整候选结果。
 */

#include "measure_commands_internal.h"

#include <stdio.h>

#include "measure_density.h"
#include "fault_manager.h"
#include "system_parameter.h"

/**
 * @brief 将综合测量使用的分布测量模式参数转换为密度内核枚举。
 *
 * @details 调用场景：CMD_SYNTHETIC 执行分布密度子流程前调用。
 * @note 关键约束：CPU3 菜单保存 0~3 索引，0 兼容默认普通分布测。
 *
 * @return 参数值 1、2、3 分别返回 DENS_MODE_GB、DENS_MODE_METER、DENS_MODE_INTERVAL；0 或未知值返回兼容默认
 *         DENS_MODE_SPREAD。
 */
static DensitySpreadModeId CMD_SyntheticDensityModeFromParam(void)
{
    switch (g_deviceParams.spreadMeasurementMode) {
    case 1U:
        return DENS_MODE_GB;
    case 2U:
        return DENS_MODE_METER;
    case 3U:
        return DENS_MODE_INTERVAL;
    case 0U:
    default:
        return DENS_MODE_SPREAD;
    }
}

/**
 * @brief 把综合测量选定的密度内核模式映射到现有点阵来源枚举。
 *
 * @details 调用场景：综合测量发布最终点阵前调用。
 * @note 关键约束：只复用现有来源值，不新增共享协议枚举。
 *
 * @param mode 综合测量实际采用的密度分布内核模式枚举。
 * @return GB、密度计和区间模式分别返回 PROFILE_SOURCE_GB、PROFILE_SOURCE_METER、PROFILE_SOURCE_INTERVAL；普通或未知模式返回
 *         PROFILE_SOURCE_STANDARD。
 */
static ProfileSource CMD_SyntheticProfileSourceFromMode(DensitySpreadModeId mode)
{
    switch (mode) {
    case DENS_MODE_GB:
        return PROFILE_SOURCE_GB;
    case DENS_MODE_METER:
        return PROFILE_SOURCE_METER;
    case DENS_MODE_INTERVAL:
        return PROFILE_SOURCE_INTERVAL;
    case DENS_MODE_SPREAD:
    default:
        return PROFILE_SOURCE_STANDARD;
    }
}

/**
 * @brief 按配置执行分布测量，液位搜索和密度模式切换统一由密度内核负责。
 */
void CMD_SyntheticMeasurement(void) {
	uint32_t ret = 0;
    DensitySpreadModeId density_mode = CMD_SyntheticDensityModeFromParam();
    ProfileSource profile_source = CMD_SyntheticProfileSourceFromMode(density_mode);
	DensityDistribution temp = {0};   /* 本次测量结果临时缓存 */

    /* 新一轮综合测量先关闭旧完成锁存，CPU3仍保留已确认快照。 */
    DensityProfile_Begin();
	/* 设置设备状态：分布测量中 */
	g_measurement.device_status.device_state = STATE_SYNTHETICING;

    /* 密度测量内核统一负责找液位和切换传感器模式，避免入口层重复驱动电机。 */
    printf("综合测量\t分布测模式参数=%lu, 内核模式=%u\r\n",
           (unsigned long)g_deviceParams.spreadMeasurementMode,
           (unsigned int)density_mode);
    ret = Density_MeasureByMode_Exact(density_mode, &temp);
    if (ret == STATE_SWITCH) {
        /* 命令切换属于正常退出，不发布未完成的密度分布结果。 */
        return;
    }
    if (ret != NO_ERROR) {
        printf("综合测量\t分布密度测量失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
        return;
    }


    /* 4. 测量成功，按实际内核模式统一发布点阵来源和完成代际。 */
    DensityProfile_PublishResult(&temp, profile_source);
    Print_DensitySpreadResult(&temp);
/* 测量结束，状态切换为分布测量完成 */
	g_measurement.device_status.device_state = STATE_SYNTHETICING_OVER;

	return;
}
