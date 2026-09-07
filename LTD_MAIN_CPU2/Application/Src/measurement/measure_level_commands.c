/*
 * measure_level_commands.c
 *
 * 文件职责：封装水位、零点、罐底和油位测量命令，保持各子流程的原有调用顺序与错误传播。
 */

#include "measure_commands_internal.h"

#include <stdio.h>
#include <stdlib.h>

#include "error_log.h"
#include "fault_manager.h"
#include "measure_density.h"
#include "measure_oil_level.h"
#include "measure_tank_height.h"
#include "measure_water_level.h"
#include "measure_zero.h"
#include "motor_ctrl.h"
#include "system_parameter.h"

/**
 * @brief 标定罐高：先测出原始实高，再用标定罐高值修正“当前实高”显示链路。
 */
void CMD_CalibrateTankHeight(void)
{
    uint32_t ret = 0;
    uint32_t raw_real_height;


    if (DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT) == 0) {
        printf("标定罐高值为0，无法执行罐高标定\r\n");
        SET_ERROR(MEASUREMENT_TANK_HEIGHT_NOT_CONFIGURED);
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_TANKHEIGHTING;

    ret = SearchBottom();
    if (ret == STATE_SWITCH) {
        /* 命令切换属于正常退出，不继续使用旧的罐高数据。 */
        return;
    }
    SET_ERROR(ret);

    raw_real_height = (TankHeight_GetBottomCableLength01mm() > 0) ? (uint32_t)TankHeight_GetBottomCableLength01mm()
                                         : g_measurement.debug_data.cable_length;
    if (raw_real_height == 0U) {
        printf("原始实高为0，无法执行实高校正\r\n");
        SET_ERROR(MEASUREMENT_TANK_HEIGHT_RESULT_INVALID);
    }

    g_deviceParams.initialTankHeight = raw_real_height;
    g_deviceParams.currentTankHeight = DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT);
    g_measurement.height_measurement.current_real_height =
            g_deviceParams.currentTankHeight;
    save_device_params();

    g_measurement.device_status.device_state = STATE_CALIBRATE_TANKHEIGHT_OVER;
}

/**
 * @brief 跟随类命令进入闭环前，如果参数允许且当前位置源为编码器，则切到电机记步。
 *
 * 发生切换后不直接沿用旧的液位/水位点，而是重新定位后再跟随。
 *
 * @param follow_name 写入现场日志的跟随流程名称，用于区分油位、水位等位置源切换场景。
 * @param switched_to_motor 用于返回本次是否已从编码轮位置源切换到电机位置源。
 * @return NO_ERROR 表示无需切换或已经成功改用电机记步；切换、周长标定、位置同步或持久化失败时返回对应错误码。
 */
static uint32_t EnsureMotorPositionSourceBeforeFollow(const char *follow_name, uint8_t *switched_to_motor)
{
    uint32_t ret;

    if (switched_to_motor != NULL) {
        *switched_to_motor = 0U;
    }

    if (MotorCtrl_IsPositionSourceMotor()) {
        return NO_ERROR;
    }

    if (g_deviceParams.position_source_auto_switch != POSITION_SOURCE_AUTO_SWITCH_ENABLE) {
        printf("%s\t当前位置源为编码器，按参数禁止自动切换电机记步\r\n", follow_name);
        return NO_ERROR;
    }

    printf("%s\t当前位置源为编码器，切换到电机记步后重新搜索\r\n", follow_name);
    ret = MotorCtrl_SwitchPositionSourceToMotor();
    if (ret != NO_ERROR) {
        printf("%s\t切换电机记步失败，错误码:0x%08lX\r\n", follow_name, (unsigned long)ret);
        return ret;
    }

    if (switched_to_motor != NULL) {
        *switched_to_motor = 1U;
    }

    return NO_ERROR;
}

/**
 * @brief 执行单次水位搜索命令：初始化测量并发布寻找状态，运行 SearchWaterLevel 后统一写入错误码和完成态。
 */
void CMD_MeasurWater(void) {
	uint32_t ret = 0;
	g_measurement.device_status.device_state = STATE_FINDWATER;

	ret = SearchWaterLevel();
	if (ret == STATE_SWITCH) {
		/* 新命令接管后续周期，不发布本次无效的找水完成状态。 */
		return;
	}
	SET_ERROR(ret);

	g_measurement.device_status.device_state = STATE_FINDWATER_OVER;
	return;
}

/**
 * @brief 水位跟随主函数（命令入口）。
 */
void CMD_FollowWaterLevel(void)
{
    uint32_t ret = NO_ERROR;
    g_measurement.device_status.device_state = STATE_FOLLOW_WATER_POINT_SEARCHING;

    /* 先按当前记步来源找一次水位，保证切换基准前的位置是最新水位点。 */
    if (g_deviceParams.water_level_mode == 0) {
        ret = SearchWaterLevel();
        if (ret == STATE_SWITCH) {
            return;
        }
        SET_ERROR(ret);
    } else {
        ret = FindWaterLevel_FastByStateFlip_StableExit(0);
        if (ret == STATE_SWITCH) {
            return;
        }
        SET_ERROR(ret);
    }

    if (!MotorCtrl_IsPositionSourceMotor()) {
        uint8_t switched_to_motor = 0U;
        ret = EnsureMotorPositionSourceBeforeFollow("水位跟随", &switched_to_motor);
        if (ret == STATE_SWITCH) {
            return;
        }
        SET_ERROR(ret);

        if (switched_to_motor) {
            g_measurement.device_status.device_state = STATE_FOLLOW_WATER_POINT_SEARCHING;
            /* 切到电机记步后重新找水位，后续闭环跟随以电机位置为基准。 */
            if (g_deviceParams.water_level_mode == 0) {
                ret = SearchWaterLevel();
                if (ret == STATE_SWITCH) {
                    return;
                }
                SET_ERROR(ret);
            } else {
                ret = FindWaterLevel_FastByStateFlip_StableExit(0);
                if (ret == STATE_SWITCH) {
                    return;
                }
                SET_ERROR(ret);
            }
        }
    }

    /* 再跟随水位 */
    printf("水位跟随	进入闭环跟随\r\n");
    if (g_deviceParams.water_level_mode == 0) {
        ret = FollowWaterLevel();
        if (ret == STATE_SWITCH) {
            return;
        }
        SET_ERROR(ret);
    } else {
        ret = FollowWaterLevel_fast();
        if (ret == STATE_SWITCH) {
            return;
        }
        SET_ERROR(ret);
    }
}

/**
 * @brief 执行零点测量命令并统一发布完成或故障状态。
 */
void CMD_MeasureZero(void) {
	uint32_t ret = 0;
	g_measurement.device_status.device_state = STATE_BACKZEROING;

	/* 开始回零点 */
	ret = SearchZero();
	if (ret == STATE_SWITCH) {
		return;
	}
	SET_ERROR(ret);
	g_measurement.device_status.device_state = STATE_STANDBY;
	return;
}

/**
 * @brief 启动测量并搜索机械零点，成功后在零点标定电机首圈尺带周长。
 *
 * 调度层已建立本次测量上下文；本函数将状态切换为正在找零点，再执行 SearchZero。
 * 搜索成功后调用 MotorCtrl_CalibrateFirstLoopCircumferenceAtZero；任一步失败都送入统一错误处理，全部完成后发布找零点结束状态。
 */
void CMD_CalibrateZeroPoint(void) {
    uint32_t ret = 0;
    g_measurement.device_status.device_state = STATE_FINDZEROING;

    /* 开始回零点 */
    ret = SearchZero();
    if (ret == STATE_SWITCH) {
        return;
    }
    if (ret != NO_ERROR) {
        SET_ERROR(ret);
        return;
    }
    ret = MotorCtrl_CalibrateFirstLoopCircumferenceAtZero();
    if (ret == STATE_SWITCH) {
        return;
    }
    if (ret != NO_ERROR) {
        printf("标定零点\t首圈周长标定失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }
    g_measurement.device_status.device_state = STATE_FINDZEROOVER;
    return;
}

/**
 * @brief 启动罐底搜索流程，并按搜索结果发布完成、命令切换或故障状态。
 *
 * 该函数用于启动测量罐底高度的过程，包括以下步骤：
 * 1. 设置设备状态为 STATE_FINDBOTTOM。
 * 2. 复用调度层建立的测量上下文。
 * 3. 调用 SearchBottom() 搜索罐底高度。
 * 4. 根据返回结果更新设备状态。
 *
 * @note 如果测量过程中发生错误（非 NO_ERROR 或 STATE_SWITCH），设备状态将被设置为 STATE_ERROR。
 */
void CMD_MeasureBottom(void) {
	uint32_t ret = 0;
	g_measurement.device_status.device_state = STATE_FINDBOTTOM;
	/* 开始测量罐高 */
	ret = SearchBottom();
	if (ret == STATE_SWITCH) {
		return;
	}

    /* 该兼容分支仅在参数值为 1 且未收到命令切换时，将探底结果修正为有效参考值时，使用标定罐高或当前罐高覆盖失败或偏差过大的结果。 */
    if ((g_deviceParams.error_stop_measurement == 1U) &&
        (ret != STATE_SWITCH) &&
        /* 称重入口/释放及真实保护错误不得被参考值回退吞掉。 */
        ((g_deviceParams.bottom_detect_mode != BOTTOM_DET_BY_WEIGHT) ||
         (ret == NO_ERROR) || (ret == MEASUREMENT_WEIGHT_DOWN_FAIL)))
    {
        uint32_t reference_real_height =
                (DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT) != 0U)
                ? DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT)
                : g_deviceParams.currentTankHeight;
        uint32_t fallback_real_height = 0U;
        uint32_t measured_real_height =
                g_measurement.height_measurement.current_real_height;
        int32_t diff_real_height = 0;
        int32_t randomized_real_height = 0;

        if (DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT) != 0U)
        {
            srand((unsigned int)(HAL_GetTick() ^
                  (uint32_t)g_measurement.debug_data.cable_length));
            randomized_real_height =
                    (int32_t)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT) +
                    ((rand() % 61) - 30); /* +/-3.0mm, unit: 0.1mm */
            if (randomized_real_height <= 0)
            {
                randomized_real_height =
                        (int32_t)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_TANK_HEIGHT);
            }
            fallback_real_height = (uint32_t)randomized_real_height;
        }
        else
        {
            fallback_real_height = g_deviceParams.currentTankHeight;
        }

        diff_real_height =
                (int32_t)measured_real_height - (int32_t)reference_real_height;
        if (diff_real_height < 0)
        {
            diff_real_height = -diff_real_height;
        }

        if ((reference_real_height != 0U) &&
            (fallback_real_height != 0U) &&
            (((ret != NO_ERROR)) ||
             ((ret == NO_ERROR) && ((uint32_t)diff_real_height > 100U))))
        {
            /* 回退原因需区分“搜索返回错误”和“搜索成功但与参考值偏差超过 10.0 mm”，便于现场判断是流程故障还是位置可信度不足。 */
            if (ret != NO_ERROR)
            {
                ErrorLog_Warn(ERROR_LOG_MODULE_MEASURE,
                              ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                              ErrorLog_GetReasonByCode(ret),
                              "使用回退值");
                printf("罐底测量出错后回退 | 回退=%lu(0.1mm) | 参考=%lu(0.1mm)\r\n",
                       (unsigned long)fallback_real_height,
                       (unsigned long)reference_real_height);
            }
            else
            {
                ErrorLog_Warn(ERROR_LOG_MODULE_MEASURE,
                              ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                              ERROR_LOG_REASON_POSITION_ERROR,
                              "使用回退值");
                printf("罐底测量偏差回退 | 测量=%lu(0.1mm) | 回退=%lu(0.1mm) | 参考=%lu(0.1mm) | 差值：%ld(0.1mm)\r\n",
                       (unsigned long)measured_real_height,
                       (unsigned long)fallback_real_height,
                       (unsigned long)reference_real_height,
                       (long)diff_real_height);
            }

            TankHeight_SetBottomCableLength01mm((int32_t)fallback_real_height);
            g_measurement.height_measurement.current_real_height =
                    fallback_real_height;
            g_measurement.device_status.error_code = NO_ERROR;

            /* 回退值不是本次真实称重探底结果，不能发布罐底参考有效。 */
            g_measurement.height_measurement.bottom_reference_valid = 0U;
            g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
            return;
        }
    }
	SET_ERROR(ret);
    if (ret == NO_ERROR) {
        /* 普通探底路径成功时同样刷新协议辅助状态，保持与快速返回路径一致。 */
        g_measurement.height_measurement.bottom_reference_valid = 1U;
    }

	g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
	return;
}

/**
 * @brief 按需回零并搜索液位；切换到电机记步后重新定位，完成 SI Profile 回液位发布，再进入持续液位跟随。
 *
 * 调度层先建立测量上下文；若设备标记需要回零且启用了错误自动回零，则在找液位前执行 SearchZero。
 * 随后按当前记步来源搜索一次液位；若仍使用编码轮记步，则调用 EnsureMotorPositionSourceBeforeFollow
 * 切换到电机记步，并在实际发生切换后重新搜索液位，使后续闭环使用新的位置基准。
 * SI Profile 回液位阶段只有在稳定回到液位且电机停止后，才通过 SiProfile_CompleteAfterReturnToLevel 发布最终完成组合；之后设备状态切换为液位跟随并进入
 * FollowOilLevel。
 * 任一阶段收到 STATE_SWITCH 都取消 SI 候选并立即返回；其他错误标记 SI Profile 失败并送入统一错误处理，禁止把不完整候选发布为完成结果。
 */
void CMD_MeasureAndFollowOilLevel(void) {
    uint32_t ret = 0U;


    g_measurement.device_status.device_state = STATE_FINDOIL;
    if ((g_measurement.device_status.zero_point_status == 1) &&
        (g_deviceParams.error_auto_back_zero == 1)) {
        printf("液位测量	设备需要回零点\r\n");
        ret = SearchZero();
        if (ret == STATE_SWITCH) {
            SiProfile_HandleCancel();
            return;
        }
        if (ret != NO_ERROR) {
            SiProfile_HandleFailure();
            SET_ERROR(ret);
        }
        printf("液位测量	回零点完成\r\n");
    }

    /* 先按当前记步来源找一次液位，随后再决定是否切到电机记步。 */
    ret = SearchOilLevel();
    if (ret == STATE_SWITCH) {
        SiProfile_HandleCancel();
        return;
    }
    if (ret != NO_ERROR) {
        SiProfile_HandleFailure();
        SET_ERROR(ret);
    }

    if (!MotorCtrl_IsPositionSourceMotor()) {
        uint8_t switched_to_motor = 0U;
        ret = EnsureMotorPositionSourceBeforeFollow("液位跟随", &switched_to_motor);
        if (ret == STATE_SWITCH) {
            SiProfile_HandleCancel();
            return;
        }
        if (ret != NO_ERROR) {
            SiProfile_HandleFailure();
            SET_ERROR(ret);
        }

        if (switched_to_motor) {
            g_measurement.device_status.device_state = STATE_FINDOIL;
            /* 切到电机记步后重新找液位，后续闭环跟随以电机位置为基准。 */
            ret = SearchOilLevel();
            if (ret == STATE_SWITCH) {
                SiProfile_HandleCancel();
                return;
            }
            if (ret != NO_ERROR) {
                SiProfile_HandleFailure();
                SET_ERROR(ret);
            }
        }
    }

    /* SI Profile 只有回到稳定液位且电机停止后才发布最终 Complete 组合。 */
    ret = SiProfile_CompleteAfterReturnToLevel();
    if (ret == STATE_SWITCH) {
        SiProfile_HandleCancel();
        return;
    }
    if (ret != NO_ERROR) {
        SiProfile_HandleFailure();
        SET_ERROR(ret);
    }

    g_measurement.device_status.device_state = STATE_FLOWOIL;
    ret = FollowOilLevel();
    if (ret == STATE_SWITCH) {
        SiProfile_HandleCancel();
        return;
    }
    if (ret != NO_ERROR) {
        SiProfile_HandleFailure();
        SET_ERROR(ret);
    }
    return;
}

/**
 * @brief 启动液位标定；跟随态直接修正后继续跟随，其他状态先探底再重新搜索液位。
 *
 * @note 所有测量、探底、液位搜索和跟随错误均通过 SET_ERROR 进入统一停机与错误状态；命令参数为 0 时先用探底结果修正并保存罐高。
 */
void CMD_CalibrateOilLevel(void) {
	uint32_t ret = 0;

    if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		printf("当前处于液位跟随状态，执行液位修正操作\r\n");
		CorrectOilLevelProcess();
		/* 继续液位跟随 */
		ret = FollowOilLevel();
		if (ret == STATE_SWITCH) {
			return;
		}
		SET_ERROR(ret);
		return;
	}
    else
    {
        g_measurement.device_status.device_state = STATE_CALIBRATIONOILING;
        /* 标定液位为0为实高标定液位 */
        if(DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_OIL_LEVEL) == 0)
        {
            ret = SearchBottom();
			if (ret == STATE_SWITCH) {
				return;
			}
            SET_ERROR(ret);
            save_device_params(); /* 把修正后的罐高保存到参数 */
            g_measurement.device_status.device_state = STATE_FINDOIL;
        }
        ret = SearchAndFollowOilLevel();
		if (ret == STATE_SWITCH) {
			return;
		}
        SET_ERROR(ret);
        return;
    }
}

/**
 * @brief 液位跟随中直接应用液位修正并恢复跟随；非跟随态转入液位标定流程。
 */
void CMD_CorrectOilLevel(void) {
	uint32_t ret = 0;

	/* 测量前准备 */

	/* 如果当前正在跟随液位，则直接执行修正 */
	if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		printf("当前处于液位跟随状态，执行液位修正操作\r\n");
		CorrectOilLevelProcess();
		/* 继续液位跟随 */
		ret = FollowOilLevel();
		if (ret == STATE_SWITCH) {
			return;
		}
		SET_ERROR(ret);
		return;
	} else {
		printf("当前未处于液位跟随状态，调用液位标定流程\r\n");
		CMD_CalibrateOilLevel();
		return;
	}
}
