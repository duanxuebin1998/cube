/**
 * @file water_level_calibration.c
 * @brief 水位真值标定流程，负责反算水位罐高、持久化参数并恢复原跟随状态。
 */
#include "water_level_internal.h"
#include "system_parameter.h"
#include "../measure_commands_internal.h"
#include "error_log.h"
#include "fault_manager.h"
#include "AoOutput/ao_output.h"

#include <stdio.h>
/**
 * @brief 用当前尺带长度和命令修正量计算水罐高度，校验后同步水位并持久化。
 *
 * @return NO_ERROR 表示修正后的水罐高度、同步水位和持久化参数均完成；输入范围、当前位置、参数写回或保存失败时由检查宏返回对应错误码。
 */
static uint32_t CorrectWaterTankHeightProcess(void)
{
    int32_t new_height;

    printf("水位标定\t开始\r\n");
    printf("水位标定\t当前缆长=%.1fmm  标定真值=%.1fmm\r\n",
           g_measurement.debug_data.cable_length / 10.0f,
           DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL) / 10.0f);

    /* 核心公式：water_tank_height = cable_length_at_water + calibrateWaterLevel */
    new_height = (int32_t)g_measurement.debug_data.cable_length
              + (int32_t)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL);

    /* 合理性保护 */
    if (new_height <= 0 || new_height > 5000000) { /* 例：500m -> 5,000,000(0.1mm) */
        printf("水位标定\t计算得到水位罐高非法：%ld(0.1mm)\r\n", new_height);
        RETURN_ERROR(MEASUREMENT_WATER_CALC_OUT_OF_RANGE);
    }

    g_deviceParams.water_tank_height = new_height;
    /* 水位罐高变化后立即归一化 AO 量程，避免旧量程阻塞后续命令。 */
    (void)normalize_ao_params_after_write();
	WaterLevel_SyncFromCable();
    /* 标定完成后清零，防止重复触发 */
    DeviceCommandArguments_ClearIfUnchanged(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL);

    save_device_params();

    printf("水位标定\t完成 水位罐高=%.1fmm\r\n",
           g_deviceParams.water_tank_height / 10.0f);

    return NO_ERROR;
}
 /**
  * @brief 按用户给出的水位真值修正水罐高度；非跟随态先重找水位，跟随态修正后恢复原跟随模式。
  *
  * 标定真值为 0 时立即报告未配置错误；非跟随态调用 SearchWaterLevel 精确定位水面，命令切换时禁止继续使用旧位置修正参数。
  * 定位或沿用跟随位置后调用 CorrectWaterTankHeightProcess 计算并保存水罐高度；原先处于跟随态时按 water_level_mode 恢复普通或快速跟随。
  */
 void CMD_CalibrateWaterLevel(void)
{
    uint32_t ret = NO_ERROR;
    uint8_t  resume_follow = 0;


    resume_follow = (g_measurement.device_status.device_state == STATE_FOLLOW_WATERING);

    /*
     * 无论当前是否处于跟随态，水位标定都依赖用户给出的真值。
     * 若真值为 0，则无法反推 water_tank_height。
     */
    if (DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL) == 0) {
        printf("水位标定\t未设置标定水位真值(标定水位=0)，无法标定\r\n");
        SET_ERROR(MEASUREMENT_WATER_CALIBRATION_NOT_CONFIGURED);
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_WATERING;

    if (resume_follow)
    {
		printf("水位标定\t当前处于水位跟随状态\t修正后继续跟随\r\n");
    }
    else
	{
		printf("水位标定\t当前状态需要重新寻找水位\r\n");

		/* 先找并精确定位水位界面 */
		ret = SearchWaterLevel();
        if (ret == STATE_SWITCH)
        {
            /* 新命令属于正常切换，立即退出标定，禁止继续修正和保存旧标定结果。 */
            return;
        }
		SET_ERROR(ret);
	}
    /*
     * SearchWaterLevel() 成功后：
     * - 探头在水位界面附近
     * - g_measurement.debug_data.cable_length 已更新
     */
    ret = CorrectWaterTankHeightProcess();
    if (ret == STATE_SWITCH)
    {
        /* 新命令接管后不得继续跟随，也不得发布标定完成状态。 */
        return;
    }
    SET_ERROR(ret);

    if (resume_follow)
    {
        printf("水位标定\t修正完成，恢复水位跟随\r\n");
        g_measurement.device_status.device_state = STATE_FOLLOW_WATERING;

        if (g_deviceParams.water_level_mode == 0) {
            ret = FollowWaterLevel();
        } else {
            ret = FollowWaterLevel_fast();
        }
        if (ret == STATE_SWITCH) {
            return;
        }
        SET_ERROR(ret);
        return;
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_WATER_OVER;

    return;
}
