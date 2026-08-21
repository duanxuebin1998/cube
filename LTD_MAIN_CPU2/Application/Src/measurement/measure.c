/*
 * measure.c
 *
 * 文件职责：保留测量命令的统一初始化、传感器门禁和命令分发，不实现具体测量流程。
 */

#include "measure.h"
#include "measure_commands_internal.h"

#include <stdio.h>

#include "encoder.h"
#include "fault_manager.h"
#include "measure_density.h"
#include "motor_ctrl.h"
#include "part_diagnostics.h"
#include "power_monitor.h"
#include "sensor_service.h"
#include "system_parameter.h"
#include "Wireless/wireless_pairing.h"

/*
 * 函数用途：判断正式命令是否必须依赖本次运行期确认的传感器身份。
 * 调用场景：ProcessMeasureCmd 在解除故障锁存、使能电机和进入具体业务流程前调用。
 * 关键约束：回零、零点标定、维护、配对、纯运动、扭力标定和恢复出厂可在传感器离线时执行；新增命令默认按需要传感器处理。
 */
uint8_t Measure_CommandRequiresDetectedSensor(CommandType command)
{
    switch (command) {
    case CMD_NONE:
    case CMD_BACK_ZERO:
    case CMD_CALIBRATE_ZERO:
    case CMD_CANCEL_MEASUREMENT:
    case CMD_RUN_TO_POSITION:
    case CMD_MOVE_UP:
    case CMD_MOVE_DOWN:
    case CMD_FORCE_MOVE_UP:
    case CMD_FORCE_MOVE_DOWN:
    case CMD_SET_EMPTY_WEIGHT:
    case CMD_SET_FULL_WEIGHT:
    case CMD_RESTORE_FACTORY:
    case CMD_DEBUG_MODE:
    case CMD_MAINTENANCE_MODE:
    case CMD_MAINTENANCE_EXIT:
    case CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS:
    case CMD_PAIR_NEAREST_WIRELESS_SLIPRING:
    case CMD_RESERVED_CMD2:
    case CMD_RESERVED_CMD3:
    case CMD_RESERVED_CMD5:
    case CMD_RESERVED_CMD6:
    case CMD_RESERVED_CMD7:
    case CMD_UNKNOWN:
        return 0U;
    default:
        return 1U;
    }
}

/**
 * @brief 按命令类别执行即时控制，或完成测量启动门禁后分派业务测量流程。
 *
 * 取消测量、维护模式进入或退出、清除继电器锁存以及无线滑环配对属于即时控制命令，直接执行并返回，不触发新的测量初始化。
 * 新的普通业务命令会先取消不应继续保留的 SI 回液位候选，解除上一流程的编码器故障锁存，再通过电源监控门禁和 MeasureStart 建立本轮测量上下文。
 * 电源门禁失败时发布电源错误并停止分派；MeasureStart 返回 STATE_SWITCH 时取消 SI 候选并退出，其他启动错误通过统一错误入口发布后由具体命令流程继续按现有状态约束处理。
 * 通过启动阶段后，函数按 CommandType 分派回零、液位、水位、罐底、单点、综合、四类密度分布、瓦锡兰、部件读取、标定、手动或强制运动、扭力采集以及恢复出厂流程。
 *
 * @param command 命令值。该 CommandType 是已经通过队列和参数绑定检查的测量命令，函数按类型分发到具体业务流程。
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
        (void)WirelessPairing_RunByRssiWithValidator(SensorService_Detect);
        return;
    }

    if ((Measure_CommandRequiresDetectedSensor(command) != 0U) &&
        (SensorService_IsDetectionValid() == 0U)) {
        uint32_t sensor_detect_ret = SensorService_Detect();

        if (SensorService_IsDetectionValid() == 0U) {
            if (sensor_detect_ret == STATE_SWITCH) {
                return;
            }
            if (sensor_detect_ret == NO_ERROR) {
                sensor_detect_ret = SENSOR_DEVICE_COMM_TIMEOUT;
            }
            /* 识别无效时不得让 FRAM 中的上次 sensorType 进入新的测量流程。 */
            printf("测量命令被拒绝：本次传感器识别无效，错误码=0x%08lX\r\n",
                   (unsigned long)sensor_detect_ret);
            SiProfile_HandleFailure();
            SET_ERROR(sensor_detect_ret);
            return;
        }
    }

    /*
     * 只有新的顶层正式命令拥有解除编码器故障锁存的权限。
     * 具体命令处理函数复用本轮上下文，不再重复调用 MeasureStart。
     */
    /*
     * 产品边界规定“收到新的正式命令”即允许清编码器锁存，因此先清编码器，
     * 再现场复核电源门禁；即使电源拒绝启动，也不恢复上一流程的编码器锁存。
     */
    Encoder_BeginNewProcess();
    uint32_t power_start_ret = PowerMonitor_BeginNewProcess();
    if (power_start_ret != NO_ERROR) {
        printf("新流程被拒绝：电源监控故障码=0x%08lX\r\n",
               (unsigned long)power_start_ret);
        SiProfile_HandleFailure();
        SET_ERROR(power_start_ret);
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
        PartDiagnostics_HandleCommand();
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
 * @brief 初始化故障、驱动和扭力模块，并清除新测量流程的协议辅助运行态。
 * @return NO_ERROR 表示故障现场、驱动、扭力模块和本轮协议辅助状态均已初始化；其他值为电机驱动初始化或测量前置检查返回的真实错误码。
 */
int MeasureStart(void) {
	fault_info_init(); /* 故障初始化清零 */
    uint32_t ret = MotorCtrl_Init(); /* 电机初始化 */
    if (ret != NO_ERROR) {
        printf("电机初始化失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return (int)ret;
    }
	weight_init();

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
