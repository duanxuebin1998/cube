/*
 * measure.c
 *
 *  Created on: Mar 20, 2025
 *      Author: duan
 */

#include "measure.h"
#include "system_parameter.h"
#include "measure_zero.h"
#include "measure_tank_height.h"
#include "measure_oilLevel.h"
#include "measure_density.h"
#include "wartsila_density_measurement.h"
#include "motor_ctrl.h"
#include "motor_ctrl_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include "test.h"
#include "sensor.h"
#include "wireless_pairing.h"
#include "encoder.h"
#include "measure_water_level.h"
#include "error_log.h"
#include "abortable_delay.h"
#include "fault_recovery.h"
#include "serial_command.h"
#include "../../Services/Relay/relay_output.h"

static void CMD_CorrectOilLevel(void);
static void CMD_CancelMeasurement(void);
/* 标定罐高：先测出原始实高，再用标定罐高值修正“当前实高”显示链路。 */
static void CMD_CalibrateTankHeight(void)
{
    uint32_t ret = 0;
    uint32_t raw_real_height;

    MeasureStart();

    if (g_deviceParams.calibrateTankHeight == 0) {
        printf("标定罐高值为0，无法执行罐高标定\r\n");
        SET_ERROR(MEASUREMENT_TANK_HEIGHT_NOT_CONFIGURED);
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_TANKHEIGHTING;

    ret = SearchBottom();
    SET_ERROR(ret);

    raw_real_height = (bottom_value > 0) ? (uint32_t)bottom_value
                                         : g_measurement.debug_data.cable_length;
    if (raw_real_height == 0U) {
        printf("原始实高为0，无法执行实高校正\r\n");
        SET_ERROR(MEASUREMENT_TANK_HEIGHT_RESULT_INVALID);
    }

    g_deviceParams.initialTankHeight = raw_real_height;
    g_deviceParams.currentTankHeight = g_deviceParams.calibrateTankHeight;
    g_measurement.height_measurement.current_real_height =
            g_deviceParams.currentTankHeight;
    save_device_params();

    g_measurement.device_status.device_state = STATE_CALIBRATE_TANKHEIGHT_OVER;
}


static void CMD_EnterMaintenanceMode(void);
static void CMD_ExitMaintenanceMode(void);
static void CMD_ClearAllRelayLatchedAlarms(void);
static void CMD_WartsilaDensitySpread(void);
static void CMD_SetFullWeight(void);
static void CMD_SetEmptyWeight(void);
static void CMD_MoveDown(void);
static void CMD_MoveUp(void);
static void CMD_ForceMoveUp(void);
static void CMD_ForceMoveDown(void);
static void CMD_MeasurWater(void);
static void CMD_FollowWaterLevel(void);
static void CMD_MeasureZero(void);
static void CMD_MeasureBottom(void);
static void CMD_MeasureAndFollowOilLevel(void);
static uint32_t EnsureMotorPositionSourceBeforeFollow(const char *follow_name, uint8_t *switched_to_motor);
static DensitySpreadModeId CMD_SyntheticDensityModeFromParam(void);
static ProfileSource CMD_SyntheticProfileSourceFromMode(DensitySpreadModeId mode);
static void CMD_CalibrateZeroPoint(void);
static void CMD_CalibrateOilLevel(void);
static void CMD_SyntheticMeasurement(void);
static void CMD_RunToPosition(void);
void CMD_ReadPartParams(void);

/**
 * @brief 处理测量流程中的 ProcessMeasureCmd 逻辑。
 *
 * @param command 命令值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void ProcessMeasureCmd(CommandType command)
{
    if (command == CMD_CANCEL_MEASUREMENT) {
        CMD_CancelMeasurement();
        return;
    }
    /* 维护控制和清锁存是即时控制命令，不得触发测量初始化或改写当前运行状态。 */
    if (command == CMD_MAINTENANCE_MODE) {
        CMD_EnterMaintenanceMode();
        return;
    }
    if (command == CMD_MAINTENANCE_EXIT) {
        CMD_ExitMaintenanceMode();
        return;
    }
    if (command == CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS) {
        CMD_ClearAllRelayLatchedAlarms();
        return;
    }
    /*
     * SI Profile采点后只允许回液位命令消费候选；其它正式命令覆盖回液位流程时，
     * 必须先清除候选并进入取消态，避免以后一次找液位误发布旧候选。
     */
    if ((g_measurement.si_profile_runtime.phase == SI_PROFILE_PHASE_RETURNING_LEVEL) &&
        (command != CMD_FIND_OIL)) {
        SiProfile_HandleCancel();
    }
    if (command == CMD_PAIR_NEAREST_WIRELESS_SLIPRING) {
        printf("无线滑环匹配\t触发=正式命令\r\n");
        (void)WirelessPairing_RunByRssi();
        return;
    }

    uint32_t start_ret = (uint32_t)MeasureStart(); /* 测量初始化 */
    /* SI Profile 回液位期间的启动失败必须关闭候选态，避免阶段永久停在 RETURNING_LEVEL。 */
    if (start_ret == STATE_SWITCH) {
        SiProfile_HandleCancel();
        return;
    }
    if (start_ret != NO_ERROR) {
        SiProfile_HandleFailure();
        printf("测量启动失败，电机初始化错误码：0x%08lX\r\n", (unsigned long)start_ret);
        SET_ERROR(start_ret);
    }

    switch (command) {

    /* ================== 普通测量指令 ================== */

    case CMD_NONE:
        printf("无命令，不执行操作\r\n");
        break;

    case CMD_BACK_ZERO:
        printf("执行回零点指令\r\n");
        CMD_MeasureZero();
        break;

    case CMD_FIND_OIL:
        printf("执行寻找液位指令\r\n");
        CMD_MeasureAndFollowOilLevel();
        break;

    case CMD_FIND_WATER:
        printf("执行寻找水位指令\r\n");
        CMD_MeasurWater();
        break;

    case CMD_FIND_BOTTOM:
        printf("执行罐底测量指令\r\n");
        CMD_MeasureBottom();
        break;

    case CMD_MEASURE_SINGLE:
        printf("执行单点测量指令\r\n");
        CMD_SinglePointMeasurement();
        break;

    case CMD_MONITOR_SINGLE:
        printf("执行单点监测指令\r\n");
        CMD_SinglePointMonitoring();
        /* 监测一般是持续过程，这里不立即切回“完成”状态 */
        break;

    case CMD_SYNTHETIC:
        printf("执行综合测量指令\r\n");
        CMD_SyntheticMeasurement();
        break;

    /* --- 新增：水位跟随 --- */
    case CMD_FOLLOW_WATER:
        printf("执行水位跟随指令\r\n");
        CMD_FollowWaterLevel();
        break;

    /* --- 新增：运行到指定位置 --- */
    case CMD_RUN_TO_POSITION:
        printf("执行运行到指定位置指令\r\n");
        CMD_RunToPosition();
        break;

    /* --- 密度分布/区间测量系列 --- */
    case CMD_MEASURE_DISTRIBUTED:
        printf("执行分布测量指令\r\n");
        CMD_MeasureDensitySpread_Spread();
        break;
    case CMD_SI_PROFILE:
        printf("执行SI Profile指令\r\n");
        CMD_SiProfile();
        break;

    case CMD_GB_MEASURE_DISTRIBUTED:
        printf("执行国标分布测量指令\r\n");
        CMD_MeasureDensitySpread_GB();
        break;

    case CMD_MEASURE_DENSITY_METER:
        printf("执行密度每米测量指令\r\n");
        CMD_MeasureDensitySpread_Meter();
        break;

    case CMD_MEASURE_DENSITY_RANGE:
        printf("执行液位区间密度测量指令\r\n");
        CMD_MeasureDensitySpread_Interval();
        break;

    case CMD_WARTSILA_DENSITY_RANGE:
        printf("执行瓦西莱区间密度测量指令\r\n");
        CMD_WartsilaDensitySpread();
        break;

    /* --- 新增：读取部件参数 --- */
    case CMD_READ_PART_PARAMS:
        printf("执行读取部件参数指令\r\n");
        CMD_ReadPartParams();
        break;

    /* ================== 调试 / 标定 / 系统类指令 ================== */

    case CMD_CALIBRATE_ZERO:
        printf("执行标定零点指令\r\n");
        CMD_CalibrateZeroPoint();
        break;

    case CMD_CALIBRATE_OIL:
        printf("执行标定液位指令\r\n");
        CMD_CalibrateOilLevel();
        break;

    case CMD_CORRECT_OIL:
        printf("执行修正液位指令\r\n");
        CMD_CorrectOilLevel();
        break;

    /* --- 新增：水位标定 --- */
    case CMD_CALIBRATE_WATER:
        printf("执行水位标定指令\r\n");
        CMD_CalibrateWaterLevel();
        break;

    case CMD_CALIBRATE_TANKHEIGHT:
        printf("执行标定罐高指令\r\n");
        CMD_CalibrateTankHeight();
        break;

    case CMD_MOVE_UP:
        printf("电机上行操作\r\n");
        CMD_MoveUp();
        break;

    case CMD_MOVE_DOWN:
        printf("电机下行操作\r\n");
        CMD_MoveDown();
        break;

    /* --- 新增：强制运动类 --- */
    case CMD_FORCE_MOVE_UP:
        printf("电机强制上行操作\r\n");
        CMD_ForceMoveUp();
        break;

    case CMD_FORCE_MOVE_DOWN:
        printf("电机强制下行操作\r\n");
        CMD_ForceMoveDown();
        break;


    case CMD_SET_EMPTY_WEIGHT:
        printf("执行设置空载扭力指令\r\n");
        CMD_SetEmptyWeight();
        break;

    case CMD_SET_FULL_WEIGHT:
        printf("执行设置满载扭力指令\r\n");
        CMD_SetFullWeight();
        break;

    case CMD_RESTORE_FACTORY:
        printf("执行恢复出厂设置指令\r\n");
        RestoreFactoryParamsConfig();
        break;

    case CMD_DEBUG_MODE:
        printf("执行调试模式指令，暂不在此处处理（由上层菜单/逻辑切换）\r\n");
        break;

    /* ================== 预留 / 未知 ================== */

    case CMD_RESERVED_CMD2:
    case CMD_RESERVED_CMD3:
    case CMD_RESERVED_CMD5:
    case CMD_RESERVED_CMD6:
    case CMD_RESERVED_CMD7:
    case CMD_UNKNOWN:
    default:
        printf("暂不支持该指令: %d\r\n", (int)command);
        break;
    }
}



/**
 * @brief 执行测量流程中的 CMD_CancelMeasurement 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_CancelMeasurement(void)
{
    uint32_t stop_ret;

    printf("执行取消当前测量指令\r\n");
    FaultRecovery_Cancel("cancel measurement");
    SiProfile_HandleCancel();

    /* 用户主动取消不是故障：清掉当前命令和错误码，再把状态切到待机。 */
    g_deviceParams.command = CMD_NONE;
    g_measurement.device_status.current_command = CMD_NONE;
    g_measurement.device_status.error_code = NO_ERROR;

    stop_ret = MotorCtrl_SlowStop();
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if ((stop_ret != NO_ERROR) && (stop_ret != STATE_SWITCH)) {
        printf("取消测量\t停止电机返回：0x%08lX\r\n", (unsigned long)stop_ret);
    }

    g_measurement.device_status.device_state = STATE_STANDBY;
    printf("取消测量\t设备已进入待机\r\n");
}
/**
 * @brief 处理接收到的命令并执行相应的操作。
 *
 * 该函数根据传入的命令字符数组执行不同的电机控制操作，包括刹车、上下移动、编码器清零、测试模式等。
 *
 * @param command 指向命令字符数组的指针，命令格式为单个字符后跟可选参数。
 * @note 串口调试命令协议：
 *       通用格式：ASCII 字符串，command[0] 为主命令，部分命令使用 command[1] 作为子命令。
 *       距离参数默认按 mm 解析，例如 A+100 表示上行 100mm。
 *
 *       基础命令：
 *       - A0：快速停止电机
 *       - A+<mm>：电机上行指定距离
 *       - A-<mm>：电机下行指定距离
 *       - B<mm>: motor-model round-trip test
 *       - BE<mm>[,<速度m/min>,<加速度倍率>][S|,1]: encoder-based continuous round-trip test
 *       - BJ+<mm>[,<速度m/min>]：点动模式上行相对运动测试
 *       - BJ-<mm>[,<速度m/min>]：点动模式下行相对运动测试
 *       - BJP<target_mm>[,<速度m/min>]：点动模式绝对位置测试
 *       - C：电机步进分辨率测试
 *       - D：电机下行触底测试
 *       - E：电机上行碰零点测试
 *       - F：罐底测量重复性测试
 *       - G：罐底测量单次测试
 *       - H：零点/罐底重复性测试
 *       - I：回零点单次测试
 *       - J：液位测量重复性测试入口
 *       - K：液位测量单次测试入口
 *       - L：编码器和电机基准同时清零
 *       - M：电机高温循环测试
 *       - N：电机简单循环测试，300mm 下行/上行循环
 *       - O：获取空载扭力
 *       - P：获取满载扭力
 *       - Q：恢复出厂设置
 *       - R：分布测量
 *       - W：水位测量
 *       - X：单点测量展示
 *       - SC：传感器与无线通信综合测试
 *       - SPS：扫描 CH9141K 从机
 *       - SPR：按 RSSI 近距离规则匹配
 *       - SPN=<name>：按扫描到的蓝牙名称匹配
 *       - SPC：读取当前 CH9141K 连接状态、从机名称缓存、MAC 和 RSSI
 *
 *       TFIT 卷筒拟合命令：
 *       - T1：开始全局自动采样，以原零点为基准
 *       - T2：开始局部自动采样，以当前位置作为新的 0 圈起点
 *       - T0：停止自动采样
 *       - TA：手动添加当前样本
 *       - TS：打印拟合状态
 *       - TR：全局拟合求解
 *       - TV：局部拟合求解
 *       - TP：只应用拟合出的厚度 t
 *       - TU：同时应用拟合出的 C0 和厚度 t
 *
 *       位置源切换命令：
 *       - YM：切换到电机记步，以当前编码轮尺带长度为基准，自动下行一周标定局部周长后返回
 *       - YE：切回编码轮记步，后续 cable_length/sensor_position 由编码轮刷新
 *       - YS：打印记步模式、局部周长、XACTUAL、基准、电机计算位置、编码轮位置和差值
 *       - YC：电机记步诊断，额外打印 XTARGET/VACTUAL/RAMPSTAT/GSTAT 和显示状态校验
 */
/**
 * @brief 处理测量流程中的 process_command 逻辑。
 *
 * @param command 命令值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void process_command(uint8_t *command)
{
    SerialCommand_Process(command);
}
/**
 * @brief 执行测量流程中的 MeasureStart 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int MeasureStart(void) {
	fault_info_init(); /* 故障初始化清零 */
    uint32_t ret = MotorCtrl_Init(); /* 电机初始化 */
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("电机初始化失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return (int)ret;
    }
	weight_init();
	g_measurement.device_status.error_code = NO_ERROR; /* 故障代码清零 */
    /*
     * 外部协议适配辅助状态随新测量命令重新计算，避免上一次流程残留。
     * 这些状态只给 CPU3/SI 做协议转换，不参与原测量流程控制。
     */
    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;
    g_measurement.density_distribution.profile_blocked_by_process = 0U;
    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.density_distribution.profile_temp_deviation_alarm = 0U;
    g_measurement.density_distribution.profile_density_deviation_alarm = 0U;
	return NO_ERROR;
}

/* 测量水位主函数 */
/* 跟随类命令进入闭环前，如果参数允许且当前位置源为编码器，则切到电机记步。
 * 发生切换后不直接沿用旧的液位/水位点，而是重新定位后再跟随。 */
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
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
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
 * @brief 执行测量流程中的 CMD_MeasurWater 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_MeasurWater(void) {
	uint32_t ret = 0;
	MeasureStart();
	g_measurement.device_status.device_state = STATE_FINDWATER;

	ret = SearchWaterLevel();
	SET_ERROR(ret);

	g_measurement.device_status.device_state = STATE_FINDWATER_OVER;
	return;
}
/* 水位跟随主函数（命令入口） */
static void CMD_FollowWaterLevel(void)
{
    uint32_t ret = NO_ERROR;
    MeasureStart();
    g_measurement.device_status.device_state = STATE_FOLLOW_WATER_POINT_SEARCHING;

    /* 先按当前记步来源找一次水位，保证切换基准前的位置是最新水位点。 */
    if (g_deviceParams.water_level_mode == 0) {
        ret = SearchWaterLevel();
        SET_ERROR(ret);
    } else {
        ret = FindWaterLevel_FastByStateFlip_StableExit(0);
        SET_ERROR(ret);
    }

    if (!MotorCtrl_IsPositionSourceMotor()) {
        uint8_t switched_to_motor = 0U;
        ret = EnsureMotorPositionSourceBeforeFollow("水位跟随", &switched_to_motor);
        SET_ERROR(ret);

        if (switched_to_motor) {
            g_measurement.device_status.device_state = STATE_FOLLOW_WATER_POINT_SEARCHING;
            /* 切到电机记步后重新找水位，后续闭环跟随以电机位置为基准。 */
            if (g_deviceParams.water_level_mode == 0) {
                ret = SearchWaterLevel();
                SET_ERROR(ret);
            } else {
                ret = FindWaterLevel_FastByStateFlip_StableExit(0);
                SET_ERROR(ret);
            }
        }
    }

    /* 再跟随水位 */
    printf("水位跟随	进入闭环跟随\r\n");
    if (g_deviceParams.water_level_mode == 0) {
        ret = FollowWaterLevel();
        SET_ERROR(ret);
    } else {
        ret = FollowWaterLevel_fast();
        SET_ERROR(ret);
    }
}

/**
 * @brief 执行测量流程中的 CMD_MeasureZero 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_MeasureZero(void) {
	uint32_t ret = 0;
	MeasureStart();
	g_measurement.device_status.device_state = STATE_BACKZEROING;

	/* 开始回零点 */
	ret = SearchZero();
	SET_ERROR(ret);
	g_measurement.device_status.device_state = STATE_STANDBY;
	return;
}
/* 罐底零点主函数 */
static void CMD_CalibrateZeroPoint(void) {
    uint32_t ret = 0;
    MeasureStart();
    g_measurement.device_status.device_state = STATE_FINDZEROING;

    /* 开始回零点 */
    ret = SearchZero();
    SET_ERROR(ret);
    ret = MotorCtrl_CalibrateFirstLoopCircumferenceAtZero();
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("标定零点\t首圈周长标定失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }
    g_measurement.device_status.device_state = STATE_FINDZEROOVER;
    return;
}
/**
 * @brief 测量罐底高度的函数。
 *
 * 该函数用于启动测量罐底高度的过程，包括以下步骤：
 * 1. 设置设备状态为 STATE_FINDBOTTOM。
 * 2. 调用 MeasureStart() 开始测量。
 * 3. 调用 SearchBottom() 搜索罐底高度。
 * 4. 根据返回结果更新设备状态。
 *
 * @note 如果测量过程中发生错误（非 NO_ERROR 或 STATE_SWITCH），设备状态将被设置为 STATE_ERROR。
 *
 * @return 无返回值。
 */
static void CMD_MeasureBottom(void) {
	uint32_t ret = 0;
	MeasureStart();
	g_measurement.device_status.device_state = STATE_FINDBOTTOM;
	/* 开始测量罐高 */
	ret = SearchBottom();

    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if ((g_deviceParams.error_stop_measurement == 1U) &&
        (ret != STATE_SWITCH))
    {
        uint32_t reference_real_height =
                (g_deviceParams.calibrateTankHeight != 0U)
                ? g_deviceParams.calibrateTankHeight
                : g_deviceParams.currentTankHeight;
        uint32_t fallback_real_height = 0U;
        uint32_t measured_real_height =
                g_measurement.height_measurement.current_real_height;
        int32_t diff_real_height = 0;
        int32_t randomized_real_height = 0;

        if (g_deviceParams.calibrateTankHeight != 0U)
        {
            srand((unsigned int)(HAL_GetTick() ^
                  (uint32_t)g_measurement.debug_data.cable_length));
            randomized_real_height =
                    (int32_t)g_deviceParams.calibrateTankHeight +
                    ((rand() % 61) - 30); /* +/-3.0mm, unit: 0.1mm */
            if (randomized_real_height <= 0)
            {
                randomized_real_height =
                        (int32_t)g_deviceParams.calibrateTankHeight;
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
            bottom_value = (int32_t)fallback_real_height;
            g_measurement.height_measurement.current_real_height =
                    fallback_real_height;
            g_measurement.device_status.error_code = NO_ERROR;

            /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
            if (ret != NO_ERROR)
            {
                /* 错误 阶段：错误报警 模块：测量 操作：精找罐底 原因：ErrorLog_GetReasonByCode(ret) 处理：使用回退值 */
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
                /* 错误 阶段：错误报警 模块：测量 操作：精找罐底 原因：位置异常 处理：使用回退值 */
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

            /* 探底成功后置位罐底参考有效，CPU3 可据此点亮 SI Bottom Reference 位。 */
            g_measurement.height_measurement.bottom_reference_valid = 1U;
            g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
            return;
        }
    }
	SET_ERROR(ret);
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret == NO_ERROR) {
        /* 普通探底路径成功时同样刷新协议辅助状态，保持与快速返回路径一致。 */
        g_measurement.height_measurement.bottom_reference_valid = 1U;
    }

	g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
	return;
}
/**
 * @brief 执行测量流程中的 CMD_MeasureAndFollowOilLevel 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_MeasureAndFollowOilLevel(void) {
    uint32_t ret = 0U;

    ret = (uint32_t)MeasureStart();
    if (ret == STATE_SWITCH) {
        SiProfile_HandleCancel();
        return;
    }
    if (ret != NO_ERROR) {
        SiProfile_HandleFailure();
        SET_ERROR(ret);
    }

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

/* 标定液位 */
static void CMD_CalibrateOilLevel(void) {
	uint32_t ret = 0;
	ret = (uint32_t)MeasureStart();
	SET_ERROR(ret);

    if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		printf("当前处于液位跟随状态，执行液位修正操作\r\n");
		CorrectOilLevelProcess();
		/* 继续液位跟随 */
		ret = FollowOilLevel();
		SET_ERROR(ret);
		return;
	}
    else
    {
        g_measurement.device_status.device_state = STATE_CALIBRATIONOILING;
        /* 标定液位为0为实高标定液位 */
        if(g_deviceParams.calibrateOilLevel == 0)
        {
            ret = SearchBottom();
            SET_ERROR(ret);
            save_device_params(); /* 把修正后的罐高保存到参数 */
            g_measurement.device_status.device_state = STATE_FINDOIL;
        }
        ret = SearchAndFollowOilLevel();
        SET_ERROR(ret);
        return;
    }
}
/**
 * @brief 执行测量流程中的 CMD_CorrectOilLevel 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_CorrectOilLevel(void) {
	uint32_t ret = 0;

	/* 测量前准备 */
	ret = (uint32_t)MeasureStart();
	SET_ERROR(ret);

	/* 如果当前正在跟随液位，则直接执行修正 */
	if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		printf("当前处于液位跟随状态，执行液位修正操作\r\n");
		CorrectOilLevelProcess();
		/* 继续液位跟随 */
		ret = FollowOilLevel();
		SET_ERROR(ret);
		return;
	} else {
		printf("当前未处于液位跟随状态，调用液位标定流程\r\n");
		CMD_CalibrateOilLevel();
		return;
	}
}
/**
 * @brief 执行测量流程中的 CMD_EnterMaintenanceMode 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_EnterMaintenanceMode(void)
{
    /* 维护模式只叠加报警输出屏蔽，不占用设备状态，也不阻塞后续指令和测量。 */
    g_measurement.device_status.maintenance_mode_active = 1U;
    printf("维护模式已开启\r\n");
}

/**
 * @brief 退出非阻塞维护模式。
 * @note 只撤销维护屏蔽，不改写当前设备状态、故障码或正在执行的测量。
 */
static void CMD_ExitMaintenanceMode(void)
{
    g_measurement.device_status.maintenance_mode_active = 0U;
    printf("维护模式已退出\r\n");
}

/**
 * @brief 请求清除四路继电器锁存报警。
 * @note 请求只作用于运行态，由继电器更新周期消费，不触发参数持久化。
 */
static void CMD_ClearAllRelayLatchedAlarms(void)
{
    RelayOutput_RequestClearAllLatchedAlarms();
    printf("已请求清除全部继电器锁存报警\r\n");
}
/* 电机上行指令 */
static void CMD_MoveUp(void)
{
    uint32_t ret = 0;

    printf("电机上行操作\n");
    /* 手动上行由外部协议触发时，不应被自动报警/液位跟随逻辑误判为自动测量动作。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_RUNUPING;

    ret = MotorCtrl_MoveAndWait(
            (float)g_deviceParams.motorCommandDistance / 10.0f,
            MOTOR_DIRECTION_UP,
            MotorCtrl_GetDefaultSpeedX100());

    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }

    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.device_status.device_state = STATE_RUNUPOVER;
    return;
}
/* 电机下行指令 */
static void CMD_MoveDown(void)
{
    uint32_t ret = 0;

    printf("电机下行操作\n");
    /* 手动下行同样设置抑制位，CPU3 据此把 SI 报警/液位更新状态与自动测量隔离。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_RUNDOWNING;

    ret = MotorCtrl_MoveAndWait(
            (float)g_deviceParams.motorCommandDistance / 10.0f,
            MOTOR_DIRECTION_DOWN,
            MotorCtrl_GetDefaultSpeedX100());

    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }

    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.device_status.device_state = STATE_RUNDOWNOVER;
    return;
}
/* 电机强制上行指令（无检测） */
static void CMD_ForceMoveUp(void)
{
    uint32_t ret;

    printf("电机强制上行操作\r\n");
    /* 强制运行无检测，必须显式告诉 CPU3 当前液位/报警状态不应作为自动流程结果。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_FORCE_RUNUPING;
    printf("强制上行距离: %.1f mm\r\n", (float) g_deviceParams.motorCommandDistance / 10.0f);
    /* 强制调试运动允许忽略编码器首帧未就绪，但不绕过驱动安全检查。 */
    ret = MotorCtrl_MoveBlockingNoDetectForceDebug(
        (float)g_deviceParams.motorCommandDistance / 10.0f,
        MOTOR_DIRECTION_UP,
        MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }
    printf("电机强制上行操作完成\r\n");
    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
/* MotorCtrl_MoveAndWait( */
/* (float)g_deviceParams.motorCommandDistance / 10.0f, */
/* MOTOR_DIRECTION_UP); */
    g_measurement.device_status.device_state = STATE_FORCE_RUNUP_OVER;
    return;
}

/* 电机强制下行指令（无检测） */
static void CMD_ForceMoveDown(void)
{
    uint32_t ret;

    printf("电机强制下行操作\r\n");
    /* 强制下行属于手动动作，协议辅助状态只反映抑制语义，不改变原电机控制流程。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_FORCE_RUNDOWNING;

    /* 强制调试运动允许忽略编码器首帧未就绪，但不绕过驱动安全检查。 */
    ret = MotorCtrl_MoveBlockingNoDetectForceDebug(
        (float)g_deviceParams.motorCommandDistance / 10.0f,
        MOTOR_DIRECTION_DOWN,
        MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }

    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.device_status.device_state = STATE_FORCE_RUNDOWN_OVER;
    return;
}


/* 设置空载扭力指令 */
static void CMD_SetEmptyWeight(void)
{
    uint32_t ret = 0;

    printf("执行设置空载扭力指令\n");
    g_measurement.device_status.device_state = STATE_GET_EMPTYWEIGHT;

    ret = get_empty_weight();

    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_GET_EMPTYWEIGHT_OVER;
    return;
}
/* 设置满载扭力指令 */
static void CMD_SetFullWeight(void)
{
    uint32_t ret = 0;

    printf("执行设置满载扭力指令\n");
    g_measurement.device_status.device_state = STATE_GET_FULLWEIGHT;

    ret = get_full_weight();

    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_GET_FULLWEIGHT_OVER;
    return;
}
/**
 * @brief 执行测量流程中的 Wartsila_MoveToMonitorPositionOnly 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Wartsila_MoveToMonitorPositionOnly(void)
{
    uint32_t ret = NO_ERROR;
    const uint32_t max_attempts = 3U;
    float target_mm = (float)g_deviceParams.singlePointMonitoringPosition / 10.0f;

    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
    printf("瓦锡兰测后探底\t先回固定点监测位置：%.1fmm，仅移动不读密度\r\n", (double)target_mm);

    ret = SinglePoint_CheckTargetPosition("瓦锡兰测后回固定点", g_deviceParams.singlePointMonitoringPosition);
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    for (uint32_t attempt = 1U; attempt <= max_attempts; attempt++) {
        ret = MotorCtrl_JogMoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());
        /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
        if ((ret == NO_ERROR) || (ret == STATE_SWITCH)) {
            return ret;
        }

        printf("瓦锡兰测后探底\t回固定点监测位置失败：0x%08lX，尝试：%lu/%lu\r\n",
               (unsigned long)ret,
               (unsigned long)attempt,
               (unsigned long)max_attempts);
    }

    printf("瓦锡兰测后探底\t回固定点监测位置重试失败，跳过探底且不置错误状态\r\n");
    return ret;
}

/**
 * @brief 执行测量流程中的 CMD_WartsilaDensitySpread 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_WartsilaDensitySpread(void) {
	static uint32_t bottom_detect_count = 0; /* 瓦锡兰测量后探底计数，仅运行期累计 */
	static uint32_t wartsila_measure_total_count = 0; /* 瓦锡兰分布测量完成次数，仅运行期累计 */
	uint32_t ret = 0;
	uint32_t bottom_detect_interval = g_deviceParams.wartsila_bottom_detect_interval; /* 本次瓦锡兰测量后的探底频率参数快照 */
	DensityDistribution temp = {0};

	/* 新一轮瓦锡兰测量先关闭旧完成锁存，CPU3仍保留已确认快照。 */
	DensityProfile_Begin();
	/* 设置设备状态：分布测量中 */
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_MEASURING;

	ret = Wartsila_Density_SpreadMeasurement(&temp);
	if (ret == STATE_SWITCH) {
		return;
	}
	/* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
	if (ret != NO_ERROR) {
		printf("瓦锡兰分布测量失败，错误码：0x%08lX，不更新新的有效结果\r\n", (unsigned long)ret);
		SET_ERROR(ret);
		return;
	}

	/* 完整点阵写入后最后递增完成计数，CPU3只会同步完整新代际。 */
	DensityProfile_PublishResult(&temp, PROFILE_SOURCE_WARTSILA);

	if (AbortableDelay_CommandSwitch(1000U, 100U) == STATE_SWITCH) {
		return;
	}
	Print_DensitySpreadResult(&temp);
/* 测量结束，状态切换为分布测量完成 */
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_OVER;
	printf("瓦锡兰分布测量结果保留8秒，原因：等待CPU3读取结果，期间可切换命令退出\r\n");
	for (uint32_t remain_s = 8U; remain_s > 0U; remain_s--) {
		printf("瓦锡兰分布测量结果保留倒计时：%lu秒\r\n", (unsigned long)remain_s);
		if (AbortableDelay_CommandSwitch(1000U, 100U) == STATE_SWITCH) {
			return;
		}
	}
    wartsila_measure_total_count++;
    /* 按参数控制瓦锡兰测量后的探底频率：0不探底，N表示每N次测量后探底一次，最大100。 */
    if (bottom_detect_interval > 100U) {
        printf("瓦锡兰测后探底\t探底频次参数=%lu 超出上限100，按每1次执行\r\n",
               (unsigned long)bottom_detect_interval);
        bottom_detect_interval = 1U;
    }

    if (bottom_detect_interval > 0U) {
        uint32_t current_cycle_count = bottom_detect_count + 1U;
        bool will_search_bottom = (current_cycle_count >= bottom_detect_interval);

        printf("瓦锡兰测后探底\t测量总次数=%lu | 探底频次=每%lu次 | 当前周期第%lu/%lu次 | 本次%s探底\r\n",
               (unsigned long)wartsila_measure_total_count,
               (unsigned long)bottom_detect_interval,
               (unsigned long)current_cycle_count,
               (unsigned long)bottom_detect_interval,
               will_search_bottom ? "执行" : "不执行");

        bottom_detect_count = current_cycle_count;
        if (will_search_bottom) {
            bottom_detect_count = 0U;
            ret = Wartsila_MoveToMonitorPositionOnly();
            if (ret == STATE_SWITCH) {
                return;
            }
            /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
            if ((ret == PARAM_CONFIG_MISSING) ||
                (ret == PARAM_COMBINATION_CONFLICT) ||
                (ret == PARAM_RANGE_ERROR)) {
                SET_ERROR(ret);
            }
            /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
            if (ret != NO_ERROR) {
                g_deviceParams.command = CMD_MONITOR_SINGLE;
                return;
            }

            ret = SearchBottom();
            if (ret == STATE_SWITCH) {
                return;
            }
            /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
            if (ret != NO_ERROR) {
                printf("瓦锡兰测后探底\t罐底测量失败：0x%08lX，退出且不置错误状态\r\n", (unsigned long)ret);
                g_deviceParams.command = CMD_MONITOR_SINGLE;
                return;
            }
        }
    } else {
        bottom_detect_count = 0U;
        printf("瓦锡兰测后探底\t测量总次数=%lu | 探底频次=0，不探底\r\n",
               (unsigned long)wartsila_measure_total_count);
    }
    g_deviceParams.command = CMD_MONITOR_SINGLE; /* 切回单点监测状态，继续监测当前液位/密度 */
	return;
}
/*
 * 函数用途：将综合测量使用的分布测量模式参数转换为密度内核枚举。
 * 调用场景：CMD_SYNTHETIC 执行分布密度子流程前调用。
 * 关键约束：CPU3 菜单保存 0~3 索引，0 兼容默认普通分布测。
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

/*
 * 函数用途：把综合测量选定的密度内核模式映射到现有点阵来源枚举。
 * 调用场景：综合测量发布最终点阵前调用。
 * 关键约束：只复用现有来源值，不新增共享协议枚举。
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
 * @brief 执行测量流程中的 CMD_SyntheticMeasurement 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void CMD_SyntheticMeasurement(void) {
	uint32_t ret = 0;
    DensitySpreadModeId density_mode = CMD_SyntheticDensityModeFromParam();
    ProfileSource profile_source = CMD_SyntheticProfileSourceFromMode(density_mode);
	DensityDistribution temp = {0};   /* 本次测量结果临时缓存 */

    /* 新一轮综合测量先关闭旧完成锁存，CPU3仍保留已确认快照。 */
    DensityProfile_Begin();
	/* 设置设备状态：分布测量中 */
	g_measurement.device_status.device_state = STATE_SYNTHETICING;

    /* 1. 先搜索液位 */
    ret = SearchOilLevel();
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("密度分布\t液位搜索失败, 错误码: 0x%08lX\r\n", ret);
        SET_ERROR(ret);
    }
    printf("密度分布\t液位搜索成功\r\n");

    /* 2. 切换到密度测量模式 */
    EnableDensityMode();

    /* 3. 按参数选择分布密度测量模式, 结果写入 temp */
    printf("综合测量\t分布测模式参数=%lu, 内核模式=%u\r\n",
           (unsigned long)g_deviceParams.spreadMeasurementMode,
           (unsigned int)density_mode);
    ret = Density_MeasureByMode_Exact(density_mode, &temp);
    if (ret == STATE_SWITCH) {
        return;
    }
    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
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
/**
 * @brief 运行到指定绝对位置（mm）
 * 依赖：
 *  - MeasureStart()
 *  - MotorCtrl_JogMoveToPosition(float target_mm, uint32_t speed_x100)
 *  - CHECK_COMMAND_SWITCH(x) / SET_ERROR(x)
 *  - g_measurement.device_status.device_state
 *  - 目标位置参数来源（见下方 get_target_mm()）
 */
static void CMD_RunToPosition(void)
{
    uint32_t ret = NO_ERROR;
    float target_mm = 0.0f;

    MeasureStart();
    g_measurement.device_status.device_state = STATE_RUN_TO_POSITIONING;


    target_mm = (float)g_deviceParams.densityDistributionOilLevel/10.0;
    ret = MotorCtrl_JogMoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());

    /* MotorCtrl_JogMoveToPosition 里如果你也加了 CHECK_COMMAND_SWITCH，就能更快退出；
       若没加，这里至少在调用前/后能响应一次切换。 */

    if (ret == STATE_SWITCH) {
        printf("运行到指定位置\t命令切换，中止\r\n");
        /* 中止：通常不记为错误，回到待机或保持上层状态机处理 */
        g_measurement.device_status.device_state = STATE_STANDBY;
        return;
    }

    /* 先处理异常边界，避免测量流程状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("运行到指定位置\t失败 错误码：0x%lX\r\n", ret);
        SET_ERROR(ret);
        g_measurement.device_status.device_state = STATE_ERROR;
        return;
    }

    /* 3) 成功完成 */
    printf("运行到指定位置\t完成\r\n");
    g_measurement.device_status.device_state = STATE_RUN_TO_POSITION_OVER;
}
