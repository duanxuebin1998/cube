/**
 * @file water_level_search.c
 * @brief 水位慢速搜索流程，负责零点准备、空气区避让、粗找重试和精找重试。
 *
 * 本文件只编排一次水位搜索，不承担持续跟随、传感器剖面测试或水位标定。
 */
#include "water_level_internal.h"
#include "fault_manager.h"
#include "system_parameter.h"
#include "motor_ctrl.h"
#include "measure_zero.h"
#include "sensor_service.h"
#include "error_log.h"
#include "AoOutput/ao_output.h"

#include <stdio.h>
#include <stdlib.h>
/* 搜索开始时向零点方向避让的距离，单位 mm。 */
#define WATER_INIT_UP_MM             (100.0f)
/* 水位粗找允许的最大尝试次数。 */
#define WATER_ROUGH_RETRY_MAX        (3U)
/* 水位精找允许的最大尝试次数。 */
#define WATER_PRECISE_RETRY_MAX      (3U)
/* 粗找急停后等待水位电容稳定的时间，单位 ms。 */
#define WATER_ROUGH_CONFIRM_DELAY_MS (3000U)
/* 粗找复核失败后向上恢复的距离，单位 mm。 */
#define WATER_FAIL_RECOVER_UP_MM     (100.0f)
/* 精找接近粗定位点时切换到中低速的距离阈值，单位 0.1 mm。 */
#define WATER_V1_SLOWDOWN_TH         (1000)
/* 精找更接近粗定位点时切换到最低速的距离阈值，单位 0.1 mm。 */
#define WATER_V2_SLOWDOWN_TH         (100)

/* 粗找和精找阶段实现只在本慢速搜索文件内调用。 */
static int SearchWaterRough(void);
static int SearchWaterPrecise(void);
/**
 * @brief 水位测量函数 - 执行完整的慢速找水流程
 *
 * 流程梳理：
 * 1. 先确保零点状态和 zero_capacitance 可用；必要时先回零点并读取空气区电容
 * 2. 若当前探头已经在水区，则先上行避让，保证后续是从空气区往下找水
 * 3. 进入粗找：持续下探，首次检测到 WATER_LEVEL_STATE_DETECTED 后急停，等待 3s 再复判
 * 4. 粗找成功后记录一个粗略水位，再进入精找：先上提 100mm，再降速下探做精定位
 * 5. 精找成功后，统一用 water_tank_height - cable_length 回写最终水位
 *
 * 说明：
 * - 本函数负责“找到一个可靠水位点”，不负责长期闭环跟随
 * - 状态切换（STATE_SWITCH）时不做本步骤内重试，由上层状态机决定后续动作
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
uint32_t SearchWaterLevel(void)
{
    uint32_t ret;
    uint32_t last_rough_ret = MEASUREMENT_WATERLEVEL_LOW;
    uint8_t  try_times   = 0;
    WaterLevelState water_state = WATER_LEVEL_STATE_NORMAL;

    AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    fault_info_init();
    printf("水位测量\t开始\r\n");


    /* -------------------- 零点检查 -------------------- */
    if ((g_measurement.device_status.zero_point_status == 1)&&(g_deviceParams.error_auto_back_zero==1))
    {
        printf("水位测量\t设备需要回零点\r\n");
        ret = SearchZero();
        CHECK_ERROR(ret);
        printf("水位测量\t回零点完成\r\n");
    }
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
    /* -------------------- 初始位置调整（避让） -------------------- */
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = MotorCtrl_MoveAndWait(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    printf("水位测量\t初始位置：%.1f", g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\r\n");

    ret = WaterLevel_CheckStatus(&water_state);
    CHECK_ERROR(ret);

    if (water_state != WATER_LEVEL_STATE_NORMAL)
    {
        /* 约定：sensor_position 单位为 0.1mm（从你打印 /10 推断） */
        const int32_t ZERO_NEAR_TH = 1000;   /* 100mm -> 1000(0.1mm) */
        const float   UP_STEP_MM   = 100.0f; /* 每次上行 100mm */

        printf("水位测量\t当前在水区，执行上行避让\r\n");

        while (1)
        {
            /* 距离零点 <= 100mm：认为已到零点附近，仍在水里 -> 报错退出 */
            if (g_measurement.debug_data.sensor_position <= ZERO_NEAR_TH)
            {
                printf("水位测量\t零点附近仍在水区，无法避让(位置=%.1fmm)", g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\r\n");

                RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
            }

            /* 距离零点 > 100mm：上行 100mm */
            ret = MotorCtrl_MoveAndWait(UP_STEP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
            CHECK_ERROR(ret);
            printf("水位测量\t上行%.0fmm\r\n", (double)UP_STEP_MM);

            /* 检测是否已提出水面 */
            ret = WaterLevel_CheckStatus(&water_state);
            CHECK_ERROR(ret);

            if (water_state == WATER_LEVEL_STATE_NORMAL)
            {
                printf("水位测量\t已提出水面，继续测量\r\n");
                break;
            }

            /* 仍在水里：继续上行，直到提出水面或到零点附近触发报错 */
            CHECK_COMMAND_SWITCH(NO_ERROR);
        }
    }

    /* ************** 粗找阶段 - 带重试机制 ************** */
    try_times = 0;
    while (try_times < WATER_ROUGH_RETRY_MAX)
    {
        try_times++;

        fault_info_init();
        ret = SearchWaterRough();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        CHECK_COMMAND_SWITCH(ret);

        if (ret != NO_ERROR)
        {
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_WATER_ROUGH,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           (uint32_t)WATER_ROUGH_RETRY_MAX,
                           ret);
            last_rough_ret = ret;
            HAL_Delay(1000);
            continue;
        }
        else
        {
            if (try_times > 1U)
            {
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_WATER_ROUGH,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 (uint32_t)WATER_ROUGH_RETRY_MAX);
            }
            break;
        }
    }

    if (ret != NO_ERROR)
    {
        RETURN_ERROR(last_rough_ret);
    }

    printf("水位测量\t粗找完成：水位：%ld\r\n", WaterLevel_GetSearchReference01mm());

    /* ************** 精找阶段 - 带重试 ************** */
    try_times = 0;
    while (try_times < WATER_PRECISE_RETRY_MAX)
    {
        try_times++;

        ret = SearchWaterPrecise();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        if (ret == NO_ERROR)
        {
            if (try_times > 1U)
            {
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_WATER_PRECISE,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 (uint32_t)WATER_PRECISE_RETRY_MAX);
            }
            break;
        }
        else
        {
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_WATER_PRECISE,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           (uint32_t)WATER_PRECISE_RETRY_MAX,
                           ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        CHECK_ERROR(ret);
    }

    /* ************** 最终记录 ************** */
    WaterLevel_SyncFromCable();

    printf("水位测量\t水位：%ld mm\r\n", g_measurement.water_measurement.water_level);

    return NO_ERROR;
}
/**
 * @brief 持续下行直至传感器进入水区；停稳复核成功后记录水位，否则上提 100 mm 并返回低水位错误。
 * @return NO_ERROR 表示下行进入水区且停稳复核通过；未能稳定确认水区时上提 100 mm 并返回 MEASUREMENT_WATERLEVEL_LOW，命令切换、传感器或电机错误由检查入口原样返回。
 */
static int SearchWaterRough(void)
{
    uint32_t ret;
    WaterLevelState water_state = WATER_LEVEL_STATE_NORMAL;

    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 持续下探直到检测到 WATER_LEVEL_STATE_DETECTED */
    while (1)
    {
	ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());  /* 启动电机向下运动 */
	CHECK_ERROR(ret); /* 检查上行是否成功 */

        ret = WaterLevel_CheckStatus(&water_state);
        CHECK_ERROR(ret);

        if (water_state != WATER_LEVEL_STATE_NORMAL) {
            break;
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t长距离寻找水位\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t");

    }

    ret = MotorCtrl_SlowStop();
    CHECK_ERROR(ret);

    HAL_Delay(WATER_ROUGH_CONFIRM_DELAY_MS);

    printf("水位测量\t确认粗找水位位置\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t");


    /* 停稳后再读一次，确认状态 */
    ret = WaterLevel_CheckStatus(&water_state);
    CHECK_ERROR(ret);

    if (water_state == WATER_LEVEL_STATE_DETECTED)
    {
        WaterLevel_SetSearchReference01mm(WaterLevel_CalcFromCable());
        return NO_ERROR;
    }
    else
    {

        ret = MotorCtrl_MoveAndWait(WATER_FAIL_RECOVER_UP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);

        printf("水位测量\t上行%.0fmm\r\n", (double)WATER_FAIL_RECOVER_UP_MM);
        return MEASUREMENT_WATERLEVEL_LOW;
    }
}
/**
 * @brief 精确搜索水位 - 使用变速策略精确定位
 *
 * @return NO_ERROR 表示变速精找已收敛并记录水位；命令切换、传感器数据无效、位置边界或电机运动错误由检查宏返回。
 */
static int SearchWaterPrecise(void)
{
    uint32_t ret;
    WaterLevelState water_state = WATER_LEVEL_STATE_NORMAL;
    uint32_t speed_x100;

    if (g_measurement.debug_data.sensor_position > 2000)
    {
        ret = MotorCtrl_MoveAndWait(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    /* 粗找停稳后探头可能仍在水中，细找前先上行脱离水区，避免第一次采样直接结束。 */
    ret = WaterLevel_CheckStatus(&water_state);
    CHECK_ERROR(ret);
    if (water_state == WATER_LEVEL_STATE_DETECTED)
    {
        const int32_t zero_near_th = 1000;

        MotorCtrl_LostStepInit();
        printf("水位测量\t细找前仍在水中，先上行脱离水区\r\n");
        while (water_state == WATER_LEVEL_STATE_DETECTED)
        {
            if (g_measurement.debug_data.cable_length <= zero_near_th)
            {
                ret = MotorCtrl_SlowStop();
                CHECK_ERROR(ret);
                RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
            }

            ret = MotorCtrl_MoveUp(40U);
            CHECK_ERROR(ret);

            ret = WaterLevel_CheckStatus(&water_state);
            CHECK_ERROR(ret);

            ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
            CHECK_ERROR(ret);

            CHECK_COMMAND_SWITCH(NO_ERROR);
        }

        ret = MotorCtrl_SlowStop();
        CHECK_ERROR(ret);
        printf("水位测量\t已脱离水区，开始低速下行细找\r\n");
    }

    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    while (1)
    {
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if ((WaterLevel_CalcFromCable() - WaterLevel_GetSearchReference01mm()) < WATER_V2_SLOWDOWN_TH)
        {
            speed_x100 = 4;
        }
        else if ((WaterLevel_CalcFromCable() - WaterLevel_GetSearchReference01mm()) < WATER_V1_SLOWDOWN_TH)
        {
            speed_x100 = 40;
        }

	ret = MotorCtrl_MoveDown(speed_x100);  /* 启动电机向下运动 */
	CHECK_ERROR(ret); /* 检查上行是否成功 */

        ret = WaterLevel_CheckStatus(&water_state);
        CHECK_ERROR(ret);

        if (water_state == WATER_LEVEL_STATE_DETECTED) {
            break;
        }

        /* 走过头保护 */
/* if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - WaterLevel_GetSearchReference01mm() < WATER_OVERSHOOT_TH) */
/* { */
/* printf("水位测量\t精确寻找水位未找到水位\r\n"); */
/* RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW); */
/* } */

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t精确寻找水位\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t速度(0.01m/min)\t%lu\t", (unsigned long)g_measurement.debug_data.motor_speed);
    }

    ret = MotorCtrl_SlowStop();
    CHECK_ERROR(ret);

    WaterLevel_SetSearchReference01mm(WaterLevel_CalcFromCable());
    return NO_ERROR;
}
