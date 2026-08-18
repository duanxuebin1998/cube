/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Application\Src\app_main.c
 * @Description  : 主函数
 * @Author       : Aubon
 * @Date         : 2026-02-03 14:06:14
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-07-29 14:17:38
 * Copyright 2026 Aubon, All Rights Reserved. 
 * 2026-02-03 14:06:14
 */

#include "app_main.h"
#include "main.h"
#include "system_parameter.h"
#include "motor_ctrl.h"
#include "measure.h"
#include "encoder.h"
#include "hart.h"
#include "hostcommu.h"
#include "test.h"
#include "ad5421.h"
#include "AoOutput/ao_output.h"
#include "sensor.h"
#include "ch9141_at.h"
#include "fault_recovery.h"
#include "power_monitor.h"
#include "serial_command.h"
#include "multiparam_v4_communication.h"
#include "../../Services/Relay/relay_output.h"

/**
 * @brief 判断错误码是否属于编码器故障范围。
 *
 * @param error_code 待记录、转换或判断的错误码。该整机错误码用于识别编码器类故障并选择启动或运行期处理路径。
 * @return true 表示错误码属于编码器通信、校验、就绪或持久化故障集合；否则返回 false。
 */
static uint8_t App_IsEncoderErrorCode(uint32_t error_code) {
	return Encoder_IsRuntimeFaultCode(error_code) ? 1U : 0U;
}

/**
 * @brief 执行正式测量命令并处理命令后的恢复、调试刷新和延迟保存。
 *
 * @param command 主循环已经取出的正式 CommandType 测量命令，用于选择具体测量流程。
 */
static void App_ExecuteMeasureCommand(CommandType command) {
    printf("当前命令：%d\r\n", command);

    /* 统一命令分发入口：后续会进入 measure.c，根据命令类型执行具体业务流程。 */
    ProcessMeasureCmd(command);
    /* 命令结束后只更新恢复上下文，不在这里嵌套重跑命令。 */
    FaultRecovery_UpdateAfterCommand(command);

    /* 命令执行完成后清掉 current_command，表示系统重新回到“无命令执行中”。 */
    g_measurement.device_status.current_command = CMD_NONE;

    /* 如果命令执行过程中触发了“延迟保存参数”，这里顺手处理一次。 */
    process_device_params_deferred_tasks();
}

/**
 * @brief 主循环空闲时处理全局错误兜底。
 * @note 只有在没有待执行命令、也没有正在执行命令时，才根据全局 error_code 挂错误态；新命令优先由主循环前面的命令分支处理。
 *
 * @return 1 表示命令槽和当前命令均为空时发现非 NO_ERROR、非 STATE_SWITCH 的残留故障，且已进入或保持错误态；0 表示未命中该故障兜底条件，本次可能仅屏蔽电机位置源模式下的编码器错误，或在故障清除后恢复待机。
 */
static uint8_t App_HandleIdleGlobalError(void) {
	uint32_t error_code = g_measurement.device_status.error_code;

	/* 电机记步源模式下，编码器只作为后台采集对象。
	 * 编码器悬空或 SSI 异常不能再把整机打进错误态。 */
	if (MotorCtrl_IsPositionSourceMotor() && App_IsEncoderErrorCode(error_code)) {
		g_measurement.device_status.error_code = NO_ERROR;
		error_code = NO_ERROR;
	}

	if ((g_deviceParams.command == CMD_NONE) &&
		(g_measurement.device_status.current_command == CMD_NONE) &&
		(error_code != NO_ERROR) &&
		(error_code != STATE_SWITCH)) {
		/* 空闲且没有待执行命令时发现残留故障码，若尚未进入正式错误态则通过统一入口完成停机和故障现场发布。 */
		if (g_measurement.device_status.device_state != STATE_ERROR) {
            FaultManager_SetGlobalErrorState(error_code,
                                             GetShortFilename(__FILE__),
                                             __LINE__,
                                             __func__);
		} else {
			/* 已经处于错误态时不再重复慢停，避免驱动失效后刷“电机被禁止”；恢复模块会负责重新初始化。 */
			g_measurement.device_status.device_state = STATE_ERROR;
			g_measurement.device_status.zero_point_status = 1;
		}
		/* 条件判断后可能并发收到新命令，不再清除待执行命令。 */
		g_measurement.device_status.current_command = CMD_NONE;
		return 1;
	}

	/* 只有故障码已经清除且命令槽、当前命令都为空，才允许错误态回到待机，避免恢复尚未结束便重新接收业务。 */
	if ((g_measurement.device_status.device_state == STATE_ERROR) &&
		(g_deviceParams.command == CMD_NONE) &&
		(g_measurement.device_status.current_command == CMD_NONE) &&
		(error_code == NO_ERROR)) {
		g_measurement.device_status.device_state = STATE_STANDBY;
	}

	return 0;
}
/**
 * @brief 按安全启动顺序初始化电源监控、参数、位置、通信、输出和电机子系统。
 *
 * 先启动 24 V 电源监控，再加载设备参数和恢复编码器位置；只有位置可信时才开放掉电紧急保存。
 * 随后初始化 HART、扭力、主机 Modbus、继电器和 AO，最后在电源门禁允许时初始化电机，并在故障信息模块就绪后重新发布启动阶段锁存的最高优先级错误。
 *
 * @note 电源监控或位置恢复失败会保持运动禁止；AO 初始化失败只记录辅助输出状态，不覆盖更高优先级的整机启动故障。
 */
void App_Init(void) {
    /*
     * 各子系统返回值保留到其初始化边界；startup_init_error汇总需要在
     * fault_info_init之后重新发布的启动故障，避免初始化函数清掉诊断证据。
     */
    uint32_t motor_init_ret;
    uint32_t encoder_init_ret;
    uint32_t ao_init_ret;
    uint32_t sensor_detect_ret;
    CommandType power_on_command;
    uint32_t startup_init_error = NO_ERROR;
    uint32_t power_monitor_start_ret;
	printf("LTD重启！\n");
    /*
     * 电源监控必须先于参数、编码器和电机初始化进入默认安全状态，
     * 防止启动期在24V采样尚不可信时使能驱动或发起普通FRAM写入。
     */
    power_monitor_start_ret = PowerMonitor_Start();
    if (power_monitor_start_ret != NO_ERROR) {
        startup_init_error = power_monitor_start_ret;
    }
	init_device_params(); /* 初始化设备参数 */
	/* 持久化命令不能跨重启执行；在主机通信开放前清除，后续到达的新命令不得再覆盖。 */
	g_deviceParams.command = CMD_NONE;
	/* 维护模式属于易失运行态；每次上电都必须关闭并清空对外发布状态。 */
	g_measurement.device_status.maintenance_mode_active = 0U;
	g_measurement.device_status.relay_alarm_inhibit_effective = 0U;
	g_measurement.device_status.relay_alarm_action_mask = 0U;
	encoder_init_ret = Initialize_Encoder(); /* 初始化编码器 */
	if (encoder_init_ret != NO_ERROR) {
		startup_init_error = encoder_init_ret;
	}
    /*
     * 编码器已在恢复A/B后消费上次掉电回执；确认失败时升级为23-6，
     * 其优先级高于普通监控启动故障，并保持运动禁止。
     */
    if (Encoder_DidBootDetectPowerLossSaveFailure()) {
        PowerMonitor_ReportEmergencyPersistenceFailure();
        startup_init_error = POWER_LOSS_POSITION_SAVE_FAILED;
    }
    /* 只有已恢复可信位置时才开放掉电保存，避免无效位置使 PendSV 持续重投。 */
    if (Encoder_HasTrustedPosition()) {
        PowerMonitor_ArmEmergencyPersistence();
    }
	/* 这 1 秒延时保留给外设稳定，但必须放在编码器启动之后，让编码器先采集首帧。 */
	HAL_Delay(1000);
	HartInit();
	weight_init();
	HostCommuInit(); /* 初始化Modbus通信 */
	RelayOutput_Init(); /* 初始化继电器报警输出，默认全部释放 */
    ao_init_ret = AoOutput_Init();
    if (ao_init_ret != NO_ERROR) {
        /* AO 输出是辅助输出服务，启动期 AD5421 暂时不可读只记录运行态，不阻塞整机测量。 */
        printf("AO初始化失败，仅记录AO运行态：0x%08lX\r\n", (unsigned long)ao_init_ret);
    }
    /*
     * 电源门禁已锁存时不触碰电机驱动初始化；仍返回明确故障码，
     * 使上层进入统一故障状态而不是把“未初始化电机”误当成成功。
     */
    if (PowerMonitor_IsMotorInhibited()) {
        motor_init_ret = PowerMonitor_GetLatchedFaultCode();
        if (motor_init_ret == NO_ERROR) {
            motor_init_ret = POWER_MONITOR_INIT_FAILED;
        }
    } else {
        motor_init_ret = MotorCtrl_Init();
    }
	/* 电机初始化失败时保留真实驱动错误，并只在尚无启动错误时登记为首个失败；后续初始化仍继续执行以收集完整启动状态。 */
	if (motor_init_ret != NO_ERROR) {
		g_measurement.device_status.error_code = motor_init_ret;
		if (startup_init_error == NO_ERROR) {
			startup_init_error = motor_init_ret;
		}
		printf("电机初始化失败：0x%08lX\r\n", (unsigned long)motor_init_ret);
	}
	fault_info_init(); /* 初始化故障信息 */
	/*
	 * fault_info_init 会清故障码，启动阶段初始化错误需要恢复，
	 * 避免只出现一次的硬件诊断丢失。
	 */
	if (startup_init_error != NO_ERROR) {
		FaultManager_LatchAsyncError(startup_init_error);
	}
	CH9141_AT_NotifySensorPowerOn();
	sensor_detect_ret = DetectSensorType(); /* 检测传感器类型 */
	g_measurement.device_status.zero_point_status=1; /* 设置零点状态为需要回零点 */
	power_on_command = DefaultCmd_To_MeasureCmd(g_deviceParams.powerOnDefaultCommand);
	if (power_on_command != CMD_NONE) {
		/* 启动期已有硬件初始化错误时，先保留错误码并阻止默认命令继续下发。 */
		if (startup_init_error != NO_ERROR) {
			printf("上电默认命令被拦截：启动初始化失败 0x%08lX\r\n",
			       (unsigned long)startup_init_error);
		} else if (sensor_detect_ret == STATE_SWITCH) {
			/* 探测被新命令打断时保留新命令优先级，不能再排入上电默认命令。 */
			printf("上电默认命令被拦截：传感器识别已被新命令打断\r\n");
		} else if ((Measure_CommandRequiresDetectedSensor(power_on_command) != 0U) &&
		           (Sensor_IsDetectionValid() == 0U)) {
			/* 只有依赖传感器数据的默认命令才要求本次识别有效。 */
			printf("上电默认命令被拦截：传感器识别失败 0x%08lX\r\n",
			       (unsigned long)sensor_detect_ret);
		} else if ((!MotorCtrl_IsPositionSourceMotor()) && (!Encoder_IsReady())) {
			/* 编码轮记步模式下，上电默认命令不能早于编码器首帧有效位置。 */
			g_measurement.device_status.error_code = ENCODER_FIRST_SAMPLE_TIMEOUT;
			printf("上电默认命令被拦截：编码器首帧尚未就绪\r\n");
		} else {
			DeviceCommand_Queue(power_on_command);
			printf("上电默认命令：%d\r\n", (int)power_on_command);
		}
	}
	/* 测试函数 */
/* Test_main(); / / 测试函数 */
/* motor_text(300.0f, 0U); / /电机测试 */
/* MotorCtrl_SwitchPositionSourceToMotor();/ /切换成电机记步测试 */
}
/* 主循环任务 */
/**
 * @brief 主循环本身不直接做测量，它更像一个“调度器”。
 *
 * 进入主业务分派前，先在线程态处理电源监控延后故障、紧急保存报告、电机运行位置、扭力通信超时、主机通信日志和模拟量输出诊断。
 * 每轮循环只做一件最高优先级的事，优先级从高到低如下。
 * 1. 接收侧刚送进来的原始命令（new_command_ready）。
 * 2. 已经挂到 g_deviceParams.command 的正式命令。
 * 3. 测量命令失败后的自动恢复检查。
 * 4. 系统完全空闲时的错误兜底。
 * 5. 本轮尾声的参数延迟保存和统一节拍延时。
 * 最新串口原始命令和正式设备命令都会取消等待中的自动恢复，防止旧恢复动作抢在用户新命令前执行；恢复模块只决定本轮是否已处理以及是否需要重跑命令，实际命令选择与执行仍由主循环统一完成。
 * 这样设计的目的，是避免“错误态”或“后台任务”抢在新命令前面执行，从而把恢复动作、重试动作、强制运动动作卡死。
 * 恢复分支、错误兜底和普通循环尾声都会推进参数延迟保存并保留 50 ms 主循环节拍；前两类分支完成后立即返回，不再执行本轮后续业务。
 */
void App_MainLoop(void) {
    CommandType pending_command = CMD_NONE;
    uint32_t power_fault_code;
    uint32_t v4_quality_error;

	/* 测试指令 */
	/* MULTIPARAM_V3_Test_AllParams(); / / 多参数传感器 V3.0 测试函数 */
	/* Test_FRAM_ReadWrite(); */
	/* printf("{encoder}%d\r\n{torque}%d\r\n", (int) g_encoder_count, g_weight); */
	/* printf("位置%d", g_measurement.debug_data.sensor_position); */
	/* HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET); */
	/* HAL_UART_Transmit_DMA(&huart2, "123456", 6); / / 通过UART发送响应 */
	// DSM_EnableDensityMode();

	/* 电源监控故障先在线程态发布；恢复服务每轮最多执行一次局部ADC/DMA重启。 */
    power_fault_code = PowerMonitor_ProcessDeferred();
    if (power_fault_code != NO_ERROR) {
        FaultManager_LatchAsyncError(power_fault_code);
    }
	/* 后台轻量检查：这里只做一次快速轮询，不在主循环里展开复杂处理。 */
	/* 主动帧由PendSV解包并更新运行数据；主循环只处理UART恢复和超时诊断。 */
	MULTIPARAM_V4_Service();
    v4_quality_error = MULTIPARAM_V4_TakeQualityError();
    if ((v4_quality_error != NO_ERROR) &&
        (g_measurement.device_status.error_code == NO_ERROR)) {
        FaultManager_LatchAsyncError(v4_quality_error);
    }
	/* 先输出PendSV已完成的紧急保存快照，确保所有printf仍在线程态。 */
	SerialCommand_ProcessDeferredReports();
	(void)MotorCtrl_PollRuntimePosition();
	(void)Weight_CheckCommunicationTimeout();
	HostCommu_ProcessDeferredLogs();
	AoOutput_ProcessDeferredDiagnostics();

	/* 第一优先级：处理刚收到的原始命令。
	 * 这一层通常来自调试口/串口缓存，process_command() 会把字符命令翻译成具体动作，
	 * 必要时再写入 g_deviceParams.command。 */
	if (new_command_ready) {
		/* 新串口帧优先级最高，先取消等待中的自动恢复，再复制并处理完整帧。 */
		FaultRecovery_Cancel("serial command");
		SerialCommand_ProcessReady();
	}
	/* 第二优先级：执行已经挂起的正式命令。
	 * 这类命令通常来自上位机、参数区或其他控制入口，是系统真正的业务入口。 */
    else if (DeviceCommand_TakePending(&pending_command)) {
        /* 参数命令同样打断自动恢复，主循环本轮只执行最新命令。 */
        FaultRecovery_Cancel("formal command");
        App_ExecuteMeasureCommand(pending_command);
    }
    /* 第三优先级：自动恢复期间保持原错误状态，每 1 秒检查一次部件参数。
     * 恢复模块只返回是否需要重跑命令，真正执行仍留在主循环。 */
    else {
        FaultRecoveryResult recovery = FaultRecovery_Poll();
        if (recovery.handled) {
            /* 恢复模块只返回是否需要重跑命令，真正命令分发仍由主循环统一执行。 */
            if (recovery.should_retry_command) {
                CommandType selected_command = CMD_NONE;
                bool pending_selected =
                    DeviceCommand_PrepareRecoveryExecution(
                        recovery.retry_command, &selected_command);

                if (pending_selected) {
                    FaultRecovery_Cancel("formal command");
                }
                App_ExecuteMeasureCommand(selected_command);
            }
            process_device_params_deferred_tasks();
            /* 新命令分派完成后仍维持 50 ms 主循环节拍，避免连续命令长期占用前台并挤压后台任务。 */
            HAL_Delay(50);
            return;
        }

        /* 第四优先级：没有新命令且没有自动恢复动作时，才做错误态兜底。
         * 也就是说：自动恢复会先保持原错误状态，空闲兜底只处理未纳入恢复流程的全局错误。 */
        if (App_HandleIdleGlobalError()) {
            process_device_params_deferred_tasks();
            HAL_Delay(50); /* 错误兜底提前返回前仍维持 50 ms 主循环节拍，避免故障检查和日志在空闲状态高频重复。 */
            return;
        }
    }
	/* 本轮尾声：无论本轮是否空闲，只要没提前 return，就统一走一次节拍延时。 */
	process_device_params_deferred_tasks();
	HAL_Delay(50); /* 固定 50 ms 前台循环周期，并为后台参数任务和串口事务留出执行时间。 */
}
