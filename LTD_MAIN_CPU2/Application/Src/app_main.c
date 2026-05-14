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
#include "sensor.h"
#include "fault_recovery.h"

/**
 * @brief 判断错误码是否属于编码器故障范围。
 */
static uint8_t App_IsEncoderErrorCode(uint32_t error_code) {
	return (error_code >= ENCODER_TIMEOUT) && (error_code <= ENCODER_OCF_INCOMPLETE);
}

/**
 * @brief 执行正式测量命令并处理命令后的恢复、调试刷新和延迟保存。
 */
static void App_ExecuteMeasureCommand(CommandType command) {
    printf("当前命令：%d\r\n", command);
    g_measurement.device_status.current_command = command;

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
		if (g_measurement.device_status.device_state != STATE_ERROR) {
            FaultManager_SetErrorState(error_code,
                                       GetShortFilename(__FILE__),
                                       __LINE__,
                                       __func__);
		} else {
			HandleError();
			g_measurement.device_status.device_state = STATE_ERROR;
			g_measurement.device_status.zero_point_status = 1;
		}
		g_deviceParams.command = CMD_NONE;
		g_measurement.device_status.current_command = CMD_NONE;
		return 1;
	}

	if ((g_measurement.device_status.device_state == STATE_ERROR) &&
		(g_deviceParams.command == CMD_NONE) &&
		(g_measurement.device_status.current_command == CMD_NONE) &&
		(error_code == NO_ERROR)) {
		g_measurement.device_status.device_state = STATE_STANDBY;
	}

	return 0;
}
// 初始化函数
void App_Init(void) {
	printf("LTD重启！\n");
	HAL_Delay(1000); // 延时1000ms
	init_device_params(); // 初始化设备参数
	Initialize_Encoder(); // 初始化编码器
	HartInit(); // 初始化AD5421
	weight_init();
	HostCommuInit(); // 初始化Modbus通信
	AD5421_SetCurrent(6.0); // 设置初始电流为4mA
	MotorCtrl_Init(); //电机初始化
	fault_info_init(); // 初始化故障信息
	DetectSensorType(); // 检测传感器类型
	g_deviceParams.command = CMD_NONE; // 清除命令
	g_measurement.device_status.zero_point_status=1; // 设置零点状态为需要回零点
	if (g_deviceParams.powerOnDefaultCommand != CMD_NONE) {
		g_deviceParams.command = DefaultCmd_To_MeasureCmd(g_deviceParams.powerOnDefaultCommand); // 上电默认命令
		printf("上电默认命令：%d\r\n", g_deviceParams.command);
	}
	//测试函数
//	Test_main(); // 测试函数
//	motor_text(300.0f, 0U); //电机测试
//	MotorCtrl_SwitchPositionSourceToMotor();//切换成电机记步测试
}
// 主循环任务
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

	/*测试指令*/
	//		DSM_V2_Test_AllParams(); // 二代传感器测试函数
	//		Sensor_Test(); // 传感器测试
	//		Test_FRAM_ReadWrite();
//			printf("{encoder}%d\r\n{weight}%d\r\n", (int) g_encoder_count, g_weight);
	//		printf("位置%d", g_measurement.debug_data.sensor_position);
	//		HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET);
	//		HAL_UART_Transmit_DMA(&huart2, "123456", 6);  // 通过UART发送响应


	/* 后台轻量检查：这里只做一次快速轮询，不在主循环里展开复杂处理。 */
	(void)MotorCtrl_PollRuntimePosition();
	(void)Weight_CheckCommunicationTimeout();
	HostCommu_ProcessDeferredLogs();

	/* 第一优先级：处理刚收到的原始命令。
	 * 这一层通常来自调试口/串口缓存，process_command() 会把字符命令翻译成具体动作，
	 * 必要时再写入 g_deviceParams.command。 */
	if (new_command_ready) {
		/* 新串口命令优先级最高，先取消等待中的自动恢复，避免恢复命令和新命令竞争。 */
		FaultRecovery_Cancel("serial command");
		new_command_ready = 0;  // 本轮已经接管这条新命令，先清标志避免重复处理
		process_command(received_buffer);
	}
	/* 第二优先级：执行已经挂起的正式命令。
	 * 这类命令通常来自上位机、参数区或其他控制入口，是系统真正的业务入口。 */
    else if (g_deviceParams.command != CMD_NONE) {
        CommandType command = g_deviceParams.command;

        /* 参数命令同样打断自动恢复，主循环本轮只执行最新命令。 */
        FaultRecovery_Cancel("formal command");
        g_deviceParams.command = CMD_NONE; // 取走后立即清空，避免下轮重复执行
        App_ExecuteMeasureCommand(command);
    }
    /* 第三优先级：自动恢复期间保持原错误状态，每 1 秒检查一次部件参数。
     * 恢复模块只返回是否需要重跑命令，真正执行仍留在主循环。 */
    else {
        FaultRecoveryResult recovery = FaultRecovery_Poll();
        if (recovery.handled) {
            /* 恢复模块只返回是否需要重跑命令，真正命令分发仍由主循环统一执行。 */
            if (recovery.should_retry_command) {
                App_ExecuteMeasureCommand(recovery.retry_command);
            }
            process_device_params_deferred_tasks();
            HAL_Delay(50);
            return;
        }

        /* 第四优先级：没有新命令且没有自动恢复动作时，才做错误态兜底。
         * 也就是说：自动恢复会先保持原错误状态，空闲兜底只处理未纳入恢复流程的全局错误。 */
        if (App_HandleIdleGlobalError()) {
            process_device_params_deferred_tasks();
            HAL_Delay(50); // 出错分支也保持和主循环一致的节拍
            return;
        }
    }
	/* 本轮尾声：无论本轮是否空闲，只要没提前 return，就统一走一次节拍延时。 */
	process_device_params_deferred_tasks();
	HAL_Delay(50); // 延时50ms
}
