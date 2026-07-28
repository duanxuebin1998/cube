/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Application\Src\app_main.c
 * @Description  : 主函数
 * @Author       : Aubon
 * @Date         : 2026-02-03 14:06:14
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-05-13 15:17:02
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
#include "../../Services/Relay/relay_output.h"

/**
 * @brief 判断错误码是否属于编码器故障范围。
 */
static uint8_t App_IsEncoderErrorCode(uint32_t error_code) {
	return (error_code >= ENCODER_TIMEOUT) && (error_code <= ENCODER_FIRST_SAMPLE_TIMEOUT);
}

/**
 * @brief 执行正式测量命令并处理命令后的恢复、调试刷新和延迟保存。
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
		/* 先处理异常边界，避免应用主循环状态机带故障继续运行。 */
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

	/* 先处理异常边界，避免应用主循环状态机带故障继续运行。 */
	if ((g_measurement.device_status.device_state == STATE_ERROR) &&
		(g_deviceParams.command == CMD_NONE) &&
		(g_measurement.device_status.current_command == CMD_NONE) &&
		(error_code == NO_ERROR)) {
		g_measurement.device_status.device_state = STATE_STANDBY;
	}

	return 0;
}
/* 初始化函数 */
void App_Init(void) {
    /*
     * 各子系统返回值保留到其初始化边界；startup_init_error汇总需要在
     * fault_info_init之后重新发布的启动故障，避免初始化函数清掉诊断证据。
     */
    uint32_t motor_init_ret;
    uint32_t encoder_init_ret;
    uint32_t ao_init_ret;
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
	/* 先处理异常边界，避免应用主循环状态机带故障继续运行。 */
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
	DetectSensorType(); /* 检测传感器类型 */
	g_deviceParams.command = CMD_NONE; /* 清除命令 */
	g_measurement.device_status.zero_point_status=1; /* 设置零点状态为需要回零点 */
	if (g_deviceParams.powerOnDefaultCommand != CMD_NONE) {
		/* 启动期已有硬件初始化错误时，先保留错误码并阻止默认命令继续下发。 */
		if (startup_init_error != NO_ERROR) {
			printf("上电默认命令被拦截：启动初始化失败 0x%08lX\r\n",
			       (unsigned long)startup_init_error);
		} else if ((!MotorCtrl_IsPositionSourceMotor()) && (!Encoder_IsReady())) {
			/* 编码轮记步模式下，上电默认命令不能早于编码器首帧有效位置。 */
			g_measurement.device_status.error_code = ENCODER_FIRST_SAMPLE_TIMEOUT;
			printf("上电默认命令被拦截：编码器首帧尚未就绪\r\n");
		} else {
			DeviceCommand_Queue(DefaultCmd_To_MeasureCmd(g_deviceParams.powerOnDefaultCommand));
			printf("上电默认命令：%d\r\n", g_deviceParams.command);
		}
	}
	/* 测试函数 */
/* Test_main(); / / 测试函数 */
/* motor_text(300.0f, 0U); / /电机测试 */
/* MotorCtrl_SwitchPositionSourceToMotor();/ /切换成电机记步测试 */
}
/* 主循环任务 */
/*
 * 主循环本身不直接做测量，它更像一个“调度器”。
 * 每轮循环只做一件最高优先级的事，优先级从高到低如下：
 * 1. 接收侧刚送进来的原始命令（new_command_ready）
 * 2. 已经挂到 g_deviceParams.command 的正式命令
 * 3. 测量命令失败后的自动恢复检查
 * 4. 系统完全空闲时的错误兜底
 * 5. 本轮尾声的参数延迟保存和统一节拍延时
 *
 * 这样设计的目的，是避免“错误态”或“后台任务”抢在新命令前面执行，
 * 从而把恢复动作、重试动作、强制运动动作卡死。
 */
void App_MainLoop(void) {
    CommandType pending_command = CMD_NONE;
    uint32_t power_fault_code;

	/* 测试指令 */
	/* DSM_V2_Test_AllParams(); / / 二代传感器测试函数 */
	/* Sensor_Test(); / / 传感器测试 */
	/* Test_FRAM_ReadWrite(); */
/* printf("{encoder}%d\r\n{torque}%d\r\n", (int) g_encoder_count, g_weight); */
	/* printf("位置%d", g_measurement.debug_data.sensor_position); */
	/* HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET); */
	/* HAL_UART_Transmit_DMA(&huart2, "123456", 6); / / 通过UART发送响应 */


	/* 电源监控故障先在线程态发布；恢复服务每轮最多执行一次局部ADC/DMA重启。 */
    power_fault_code = PowerMonitor_ProcessDeferred();
    if (power_fault_code != NO_ERROR) {
        FaultManager_LatchAsyncError(power_fault_code);
    }
	/* 后台轻量检查：这里只做一次快速轮询，不在主循环里展开复杂处理。 */
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
            /* 应用主循环与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
            HAL_Delay(50);
            return;
        }

        /* 第四优先级：没有新命令且没有自动恢复动作时，才做错误态兜底。
         * 也就是说：自动恢复会先保持原错误状态，空闲兜底只处理未纳入恢复流程的全局错误。 */
        if (App_HandleIdleGlobalError()) {
            process_device_params_deferred_tasks();
            /* 应用主循环与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
            HAL_Delay(50); /* 出错分支也保持和主循环一致的节拍 */
            return;
        }
    }
	/* 本轮尾声：无论本轮是否空闲，只要没提前 return，就统一走一次节拍延时。 */
	process_device_params_deferred_tasks();
	/* 应用主循环与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
	HAL_Delay(50); /* 延时50ms */
}
