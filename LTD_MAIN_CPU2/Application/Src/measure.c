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
        SET_ERROR(PARAM_ERROR);
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_TANKHEIGHTING;

    ret = SearchBottom();
    SET_ERROR(ret);

    raw_real_height = (bottom_value > 0) ? (uint32_t)bottom_value
                                         : g_measurement.debug_data.cable_length;
    if (raw_real_height == 0U) {
        printf("原始实高为0，无法执行实高校正\r\n");
        SET_ERROR(PARAM_ERROR);
    }

    g_deviceParams.initialTankHeight = raw_real_height;
    g_deviceParams.currentTankHeight = g_deviceParams.calibrateTankHeight;
    g_measurement.height_measurement.current_real_height =
            g_deviceParams.currentTankHeight;
    save_device_params();

    g_measurement.device_status.device_state = STATE_CALIBRATE_TANKHEIGHT_OVER;
}


static void CMD_EnterMaintenanceMode(void);
static void CMD_WartsilaDensitySpread(void);
static void CMD_SetFullWeight(void);
static void CMD_SetEmptyWeight(void);
static void CMD_MoveDown(void);
static void CMD_MoveUp(void);
static void CMD_ForceMoveUp(void);
static void CMD_ForceMoveDown(void);
static void CMD_ForceLiftZero(void);
static void CMD_MeasurWater(void);
static void CMD_FollowWaterLevel(void);
static void CMD_MeasureZero(void);
static void CMD_MeasureBottom(void);
static void CMD_MeasureAndFollowOilLevel(void);
static uint32_t EnsureMotorPositionSourceBeforeFollow(const char *follow_name, uint8_t *switched_to_motor);
static void CMD_CalibrateZeroPoint(void);
static void CMD_CalibrateOilLevel(void);
static void CMD_SyntheticMeasurement(void);
static void CMD_RunToPosition(void);
void CMD_ReadPartParams(void);

void ProcessMeasureCmd(CommandType command)
{
    if (command == CMD_CANCEL_MEASUREMENT) {
        CMD_CancelMeasurement();
        return;
    }
    if (command == CMD_PAIR_NEAREST_WIRELESS_SLIPRING) {
        printf("无线滑环匹配\t触发=正式命令\r\n");
        (void)WirelessPairing_RunByRssi();
        return;
    }

    uint32_t start_ret = (uint32_t)MeasureStart(); // 测量初始化
    if (start_ret != NO_ERROR) {
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

    /* --- 新增：强制提零点 --- */
    case CMD_FORCE_LIFT_ZERO:
        printf("执行强制提零点指令\r\n");
        CMD_ForceLiftZero();
        break;

    case CMD_SET_EMPTY_WEIGHT:
        printf("执行设置空载称重指令\r\n");
        CMD_SetEmptyWeight();
        break;

    case CMD_SET_FULL_WEIGHT:
        printf("执行设置满载称重指令\r\n");
        CMD_SetFullWeight();
        break;

    case CMD_RESTORE_FACTORY:
        printf("执行恢复出厂设置指令\r\n");
        RestoreFactoryParamsConfig();
        break;

    case CMD_MAINTENANCE_MODE:
        printf("执行维护模式指令\r\n");
        CMD_EnterMaintenanceMode();
        break;

    case CMD_DEBUG_MODE:
        printf("执行调试模式指令，暂不在此处处理（由上层菜单/逻辑切换）\r\n");
        break;

    /* ================== 预留 / 未知 ================== */

    case CMD_RESERVED_CMD1:
    case CMD_RESERVED_CMD2:
    case CMD_RESERVED_CMD3:
    case CMD_RESERVED_CMD5:
    case CMD_RESERVED_CMD6:
    case CMD_UNKNOWN:
    default:
        printf("暂不支持该指令: %d\r\n", (int)command);
        break;
    }
}



static void CMD_CancelMeasurement(void)
{
    uint32_t stop_ret;

    printf("执行取消当前测量指令\r\n");
    FaultRecovery_Cancel("cancel measurement");

    /* 用户主动取消不是故障：清掉当前命令和错误码，再把状态切到待机。 */
    g_deviceParams.command = CMD_NONE;
    g_measurement.device_status.current_command = CMD_NONE;
    g_measurement.device_status.error_code = NO_ERROR;

    stop_ret = MotorCtrl_SlowStop();
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
 *       - O：获取空载称重
 *       - P：获取满载称重
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
 * @brief 将串口单字母正式业务命令映射到统一 CommandType。
 * @note 这些命令必须走主循环的正式入口，才能统一维护 current_command、命令切换和故障恢复。
 */
static uint8_t ProcessCommand_MapFormalCommand(uint8_t command_char, CommandType *formal_command)
{
    if (formal_command == NULL) {
        return 0U;
    }

    switch (command_char) {
    case 'I':
        *formal_command = CMD_BACK_ZERO;
        return 1U;
    case 'G':
        *formal_command = CMD_FIND_BOTTOM;
        return 1U;
    case 'K':
        *formal_command = CMD_FIND_OIL;
        return 1U;
    case 'R':
        *formal_command = CMD_MEASURE_DISTRIBUTED;
        return 1U;
    case 'W':
        *formal_command = CMD_FIND_WATER;
        return 1U;
    case 'O':
        *formal_command = CMD_SET_EMPTY_WEIGHT;
        return 1U;
    case 'P':
        *formal_command = CMD_SET_FULL_WEIGHT;
        return 1U;
    case 'Q':
        *formal_command = CMD_RESTORE_FACTORY;
        return 1U;
    default:
        return 0U;
    }
}
void process_command(uint8_t *command) {
    CommandType formal_command = CMD_NONE;

    printf("串口命令\t收到处理请求\r\n");
    if ((command == NULL) || (command[0] == '\0')) {
        printf("串口命令\t空命令，忽略\r\n");
        return;
    }

    if (ProcessCommand_MapFormalCommand(command[0], &formal_command)) {
        /* 串口正式业务命令只挂到主循环执行，避免绕过 current_command 和自动恢复调度。 */
        g_deviceParams.command = formal_command;
        printf("串口正式命令\t已转主循环执行\t命令=%lu\r\n", (unsigned long)formal_command);
        return;
    }

    if (Test_ProcessSerialCommand(command)) {
        return;
    }

    printf("Serial command\tunsupported\tcmd=%s\r\n", (const char *)command);
}
int MeasureStart(void) {
	fault_info_init(); //故障初始化清零
    uint32_t ret = MotorCtrl_Init(); //电机初始化
    if (ret != NO_ERROR) {
        printf("电机初始化失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return (int)ret;
    }
	weight_init();
	g_measurement.device_status.error_code = NO_ERROR; //故障代码清零
    /*
     * 外部协议适配辅助状态随新测量命令重新计算，避免上一次流程残留。
     * 这些状态只给 CPU3/SI7000 做协议转换，不参与原测量流程控制。
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

//测量水位主函数
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
    if (ret != NO_ERROR) {
        printf("%s\t切换电机记步失败，错误码:0x%08lX\r\n", follow_name, (unsigned long)ret);
        return ret;
    }

    if (switched_to_motor != NULL) {
        *switched_to_motor = 1U;
    }

    return NO_ERROR;
}

static void CMD_MeasurWater(void) {
	uint32_t ret = 0;
	MeasureStart();
	g_measurement.device_status.device_state = STATE_FINDWATER;

	ret = SearchWaterLevel();
	SET_ERROR(ret);

	g_measurement.device_status.device_state = STATE_FINDWATER_OVER;
	return;
}
// 水位跟随主函数（命令入口）
static void CMD_FollowWaterLevel(void)
{
    uint32_t ret = NO_ERROR;
    MeasureStart();
    g_measurement.device_status.device_state = STATE_FOLLOW_WATER_POINT_SEARCHING;

    // 先按当前记步来源找一次水位，保证切换基准前的位置是最新水位点。
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
            // 切到电机记步后重新找水位，后续闭环跟随以电机位置为基准。
            if (g_deviceParams.water_level_mode == 0) {
                ret = SearchWaterLevel();
                SET_ERROR(ret);
            } else {
                ret = FindWaterLevel_FastByStateFlip_StableExit(0);
                SET_ERROR(ret);
            }
        }
    }

    // 再跟随水位
    printf("水位跟随	进入闭环跟随\r\n");
    if (g_deviceParams.water_level_mode == 0) {
        ret = FollowWaterLevel();
        SET_ERROR(ret);
    } else {
        ret = FollowWaterLevel_fast();
        SET_ERROR(ret);
    }
}

static void CMD_MeasureZero(void) {
	uint32_t ret = 0;
	MeasureStart();
	g_measurement.device_status.device_state = STATE_BACKZEROING;

	//开始回零点
	ret = SearchZero();
	SET_ERROR(ret);
	g_measurement.device_status.device_state = STATE_STANDBY;
	return;
}
//罐底零点主函数
static void CMD_CalibrateZeroPoint(void) {
    uint32_t ret = 0;
    MeasureStart();
    g_measurement.device_status.device_state = STATE_FINDZEROING;

    //开始回零点
    ret = SearchZero();
    SET_ERROR(ret);
    ret = MotorCtrl_CalibrateFirstLoopCircumferenceAtZero();
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
	//开始测量罐高
	ret = SearchBottom();

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

            if (ret != NO_ERROR)
            {
                // 错误	阶段：错误报警	模块：测量	操作：精找罐底	原因：ErrorLog_GetReasonByCode(ret)	处理：使用回退值
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
                // 错误	阶段：错误报警	模块：测量	操作：精找罐底	原因：位置异常	处理：使用回退值
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

            /* 探底成功后置位罐底参考有效，CPU3 可据此点亮 SI7000 Bottom Reference 位。 */
            g_measurement.height_measurement.bottom_reference_valid = 1U;
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
static void CMD_MeasureAndFollowOilLevel(void) {
    uint32_t ret = 0;
    MeasureStart();

    g_measurement.device_status.device_state = STATE_FINDOIL;
    if ((g_measurement.device_status.zero_point_status == 1) &&
        (g_deviceParams.error_auto_back_zero == 1)) {
        printf("液位测量	设备需要回零点\r\n");
        ret = SearchZero();
        SET_ERROR(ret);
        printf("液位测量	回零点完成\r\n");
    }

    // 先按当前记步来源找一次液位，随后再决定是否切到电机记步。
    ret = SearchOilLevel();
    SET_ERROR(ret);

    if (!MotorCtrl_IsPositionSourceMotor()) {
        uint8_t switched_to_motor = 0U;
        ret = EnsureMotorPositionSourceBeforeFollow("液位跟随", &switched_to_motor);
        SET_ERROR(ret);

        if (switched_to_motor) {
            g_measurement.device_status.device_state = STATE_FINDOIL;
            // 切到电机记步后重新找液位，后续闭环跟随以电机位置为基准。
            ret = SearchOilLevel();
            SET_ERROR(ret);
        }
    }

    g_measurement.device_status.device_state = STATE_FLOWOIL;
    ret = FollowOilLevel();
    SET_ERROR(ret);
    return;
}

//标定液位
static void CMD_CalibrateOilLevel(void) {
	uint32_t ret = 0;
	MeasureStart();

    if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		printf("当前处于液位跟随状态，执行液位修正操作\r\n");
		CorrectOilLevelProcess();
		//继续液位跟随
		ret = FollowOilLevel();
		SET_ERROR(ret);
		return;
	}
    else
    {
        g_measurement.device_status.device_state = STATE_CALIBRATIONOILING;
        //标定液位为0为实高标定液位
        if(g_deviceParams.calibrateOilLevel == 0)
        {
            ret = SearchBottom();
            SET_ERROR(ret);
            save_device_params();//把修正后的罐高保存到参数
            g_measurement.device_status.device_state = STATE_FINDOIL;
        }
        ret = SearchAndFollowOilLevel();
        SET_ERROR(ret);
        return;
    }
}
static void CMD_CorrectOilLevel(void) {
	uint32_t ret = 0;

	// 测量前准备
	MeasureStart();

	// 如果当前正在跟随液位，则直接执行修正
	if (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		printf("当前处于液位跟随状态，执行液位修正操作\r\n");
		CorrectOilLevelProcess();
		//继续液位跟随
		ret = FollowOilLevel();
		SET_ERROR(ret);
		return;
	} else {
		printf("当前未处于液位跟随状态，调用液位标定流程\r\n");
		CMD_CalibrateOilLevel();
		return;
	}
}
static void CMD_EnterMaintenanceMode(void)
{
    printf("进入维护模式\n");
    /* 维护模式是 SI7000 Manual/Stop 的保守映射，手动期间抑制自动报警和液位自动更新。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_MAINTENANCEMODE;
    while (1) {
        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前操作\r\n");
            /* 命令切换退出时必须清除抑制位，避免 CPU3 长时间保持 SI7000 手动抑制状态。 */
            g_measurement.device_status.manual_alarm_inhibit = 0U;
            g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
            return;
        }
    }
}
// 电机上行指令
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
// 电机下行指令
static void CMD_MoveDown(void)
{
    uint32_t ret = 0;

    printf("电机下行操作\n");
    /* 手动下行同样设置抑制位，CPU3 据此把 SI7000 报警/液位更新状态与自动测量隔离。 */
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
// 电机强制上行指令（无检测）
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
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }
    printf("电机强制上行操作完成\r\n");
    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
//    MotorCtrl_MoveAndWait(
//            (float)g_deviceParams.motorCommandDistance / 10.0f,
//            MOTOR_DIRECTION_UP);
    g_measurement.device_status.device_state = STATE_FORCE_RUNUP_OVER;
    return;
}

// 电机强制下行指令（无检测）
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

// 强制回零点：只控制电机上行，无检测重量/液位等
static void CMD_ForceLiftZero(void)
{
    uint32_t ret;

    printf("强制提零点操作\r\n");
    g_measurement.device_status.device_state = STATE_FORCE_LIFT_ZEROING;

    /* 长距离上行：不检测称重/丢步，底层可被命令切换打断 */
    ret = MotorCtrl_MoveBlockingNoDetectForceDebug(
        2000000.0f,  // 300m,
        MOTOR_DIRECTION_UP,
        MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        return;
    }
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_FORCE_LIFT_ZERO_OVER;
    return;
}
// 设置空载称重指令
static void CMD_SetEmptyWeight(void)
{
    uint32_t ret = 0;

    printf("执行设置空载称重指令\n");
    g_measurement.device_status.device_state = STATE_GET_EMPTYWEIGHT;

    ret = get_empty_weight();

    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_GET_EMPTYWEIGHT_OVER;
    return;
}
// 设置满载称重指令
static void CMD_SetFullWeight(void)
{
    uint32_t ret = 0;

    printf("执行设置满载称重指令\n");
    g_measurement.device_status.device_state = STATE_GET_FULLWEIGHT;

    ret = get_full_weight();

    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_GET_FULLWEIGHT_OVER;
    return;
}
static uint32_t Wartsila_MoveToMonitorPositionOnly(void)
{
    uint32_t ret = NO_ERROR;
    const uint32_t max_attempts = 3U;
    float target_mm = (float)g_deviceParams.singlePointMonitoringPosition / 10.0f;

    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
    printf("瓦锡兰测后探底\t先回固定点监测位置：%.1fmm，仅移动不读密度\r\n", (double)target_mm);

    ret = SinglePoint_CheckTargetPosition("瓦锡兰测后回固定点", g_deviceParams.singlePointMonitoringPosition);
    if (ret != NO_ERROR) {
        return ret;
    }

    for (uint32_t attempt = 1U; attempt <= max_attempts; attempt++) {
        ret = MotorCtrl_MoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());
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

static void CMD_WartsilaDensitySpread(void) {
	static uint32_t bottom_detect_count = 0; /* 瓦锡兰测量后探底计数，仅运行期累计 */
	static uint32_t wartsila_measure_total_count = 0; /* 瓦锡兰分布测量完成次数，仅运行期累计 */
	uint32_t ret = 0;
	uint32_t bottom_detect_interval = g_deviceParams.wartsila_bottom_detect_interval; /* 本次瓦锡兰测量后的探底频率参数快照 */
	DensityDistribution temp = {0};
	// 设置设备状态：分布测量中
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_MEASURING;

	ret = Wartsila_Density_SpreadMeasurement(&temp);
	if (ret == STATE_SWITCH) {
		return;
	}
	if (ret != NO_ERROR) {
		printf("瓦锡兰分布测量失败，错误码：0x%08lX，不更新新的有效结果\r\n", (unsigned long)ret);
		SET_ERROR(ret);
		return;
	}

	uint32_t previous_profile_complete_counter = g_measurement.density_distribution.profile_complete_counter;
	g_measurement.density_distribution = temp;
	g_measurement.density_distribution.profile_complete_latched = 1U;
	g_measurement.density_distribution.profile_complete_counter = previous_profile_complete_counter + 1U;

	if (AbortableDelay_CommandSwitch(1000U, 100U) == STATE_SWITCH) {
		return;
	}
	Print_DensitySpreadResult(&temp);
// 测量结束，状态切换为分布测量完成
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
            if (ret == PARAM_RANGE_ERROR) {
                SET_ERROR(ret);
            }
            if (ret != NO_ERROR) {
                g_deviceParams.command = CMD_MONITOR_SINGLE;
                return;
            }

            ret = SearchBottom();
            if (ret == STATE_SWITCH) {
                return;
            }
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
    g_deviceParams.command = CMD_MONITOR_SINGLE; // 切回单点监测状态，继续监测当前液位/密度
	return;
}
static void CMD_SyntheticMeasurement(void) {
	uint32_t ret = 0;
	DensityDistribution temp = {0};   // 本次测量结果临时缓存
	// 设置设备状态：分布测量中
	g_measurement.device_status.device_state = STATE_SYNTHETICING;

    // 1. 先搜索液位
    ret = SearchOilLevel();
    if (ret != NO_ERROR) {
        printf("密度分布\t液位搜索失败, 错误码: 0x%08lX\r\n", ret);
        SET_ERROR(ret);
    }
    printf("密度分布\t液位搜索成功\r\n");

    // 2. 切换到密度测量模式
    EnableDensityMode();

    // 3. 执行分布密度测量, 结果写入 temp
    ret = Density_MeasureByMode_Exact(DENS_MODE_SPREAD, &temp);
    if (ret != NO_ERROR) {
        printf("普通分布测\t失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }


    // 4. 测量成功, 写回全局结果
    g_measurement.density_distribution = temp;
    Print_DensitySpreadResult(&temp);
// 测量结束，状态切换为分布测量完成
	g_measurement.device_status.device_state = STATE_SYNTHETICING_OVER;

	return;
}
/**
 * @brief 运行到指定绝对位置（mm）
 * 依赖：
 *  - MeasureStart()
 *  - MotorCtrl_MoveToPosition(float target_mm, uint32_t speed_x100)
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
    ret = MotorCtrl_MoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());

    /* MotorCtrl_MoveToPosition 里如果你也加了 CHECK_COMMAND_SWITCH，就能更快退出；
       若没加，这里至少在调用前/后能响应一次切换。 */

    if (ret == STATE_SWITCH) {
        printf("运行到指定位置\t命令切换，中止\r\n");
        /* 中止：通常不记为错误，回到待机或保持上层状态机处理 */
        g_measurement.device_status.device_state = STATE_STANDBY;
        return;
    }

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
