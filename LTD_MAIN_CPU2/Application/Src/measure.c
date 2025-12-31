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
#include <stdio.h>
#include <stdlib.h>
#include "test.h"
#include "sensor.h"
#include "measure_water_level.h"

static void CMD_CorrectOilLevel(void);
static void CMD_EnterMaintenanceMode(void);
static void CMD_WartsilaDensitySpread(void);
static void CMD_SetFullWeight(void);
static void CMD_SetEmptyWeight(void);
static void CMD_MoveDown(void);
static void CMD_MoveUp(void);
static void CMD_MeasurWater(void);
static void CMD_MeasureZero(void);
static void CMD_MeasureBottom(void);
static void CMD_MeasureAndFollowOilLevel(void);
static void CMD_CalibrateZeroPoint(void);
static void CMD_CalibrateOilLevel(void);
static void CMD_SyntheticMeasurement(void);


void ProcessMeasureCmd(CommandType command)
{
	MeasureStart();//测量初始化
	switch (command) {

	/* ================== 普通测量指令 ================== */

	case CMD_NONE:
		printf("无命令，不执行操作\n");
		break;

	case CMD_BACK_ZERO:
		printf("执行回零点指令\n");
		CMD_MeasureZero();
		break;

	case CMD_FIND_OIL:
		printf("执行寻找液位指令\n");
		CMD_MeasureAndFollowOilLevel();
		break;

	case CMD_FIND_WATER:
		printf("执行寻找水位指令\n");
		CMD_MeasurWater();
		break;

	case CMD_FIND_BOTTOM:
		printf("执行罐底测量指令\n");
		CMD_MeasureBottom();
		break;

	case CMD_MEASURE_SINGLE:
		printf("执行单点测量指令\n");
		CMD_SinglePointMeasurement();
		break;

	case CMD_MONITOR_SINGLE:
		printf("执行单点监测指令\n");
		CMD_SinglePointMonitoring();
		// 监测一般是持续过程，这里不立即切回“完成”状态
		break;

	case CMD_SYNTHETIC:
		printf("执行综合测量指令，暂不支持该指令\n");
		CMD_SyntheticMeasurement();
		break;

	/* 密度分布/区间测量系列，目前只有一个实现 */
	case CMD_MEASURE_DISTRIBUTED:
		printf("执行分布测量指令\n");
		CMD_MeasureDensitySpread_Spread();
		break;

	case CMD_GB_MEASURE_DISTRIBUTED:
		printf("执行国标分布测量指令，暂不支持该指令\n");
		CMD_MeasureDensitySpread_GB();
		break;

	case CMD_MEASURE_DENSITY_METER:
		printf("执行密度每米测量指令，暂不支持该指令\n");
		CMD_MeasureDensitySpread_Meter();
		break;

	case CMD_MEASURE_DENSITY_RANGE:
		printf("执行液位区间密度测量指令，暂不支持该指令\n");
		CMD_MeasureDensitySpread_Interval();
		break;

	case CMD_WARTSILA_DENSITY_RANGE:
		printf("执行瓦西莱区间密度测量指令\n");
		CMD_WartsilaDensitySpread();
		break;
	/* ================== 调试 / 标定 / 系统类指令 ================== */

	case CMD_CALIBRATE_ZERO:
		printf("执行标定零点指令\n");
		CMD_CalibrateZeroPoint();
		break;

	case CMD_CALIBRATE_OIL:
		printf("执行标定液位指令\n");
		CMD_CalibrateOilLevel();
		break;

	case CMD_CORRECT_OIL:
		printf("执行修正液位指令\n");
		CMD_CorrectOilLevel();
		break;

	case CMD_MOVE_UP:
		printf("电机上行操作\n");
        CMD_MoveUp();
		break;

	case CMD_MOVE_DOWN:
		printf("电机下行操作\n");
		CMD_MoveDown();
		break;

	case CMD_SET_EMPTY_WEIGHT:
		printf("执行设置空载称重指令\n");
		CMD_SetEmptyWeight();
		break;

	case CMD_SET_FULL_WEIGHT:
		printf("执行设置满载称重指令\n");
		CMD_SetFullWeight();
		break;

	case CMD_RESTORE_FACTORY:
		printf("执行恢复出厂设置指令\n");
		RestoreFactoryParamsConfig();
		break;

	case CMD_MAINTENANCE_MODE:
		printf("执行维护模式指令，暂不支持该指令\n");
		CMD_EnterMaintenanceMode();
		break;

	case CMD_DEBUG_MODE:
		printf("执行调试模式指令，暂不在此处处理（由上层菜单/逻辑切换）\n");
		break;


	/* ================== 预留 / 未知 ================== */

	case CMD_RESERVED_CMD1:
	case CMD_RESERVED_CMD2:
	case CMD_RESERVED_CMD3:
	case CMD_RESERVED_CMD4:
	case CMD_RESERVED_CMD5:
	case CMD_RESERVED_CMD6:
	case CMD_UNKNOWN:
	default:
		printf("暂不支持该指令: %d\n", command);
		break;
	}
}


/**
 * @brief 处理接收到的命令并执行相应的操作。
 *
 * 该函数根据传入的命令字符数组执行不同的电机控制操作，包括刹车、上下移动、编码器清零、测试模式等。
 *
 * @param command 指向命令字符数组的指针，命令格式为单个字符后跟可选参数。
 * @note 命令格式说明：
 *       - 'A0': 刹车操作
 *       - 'A+<数字>': 电机上行指定距离
 *       - 'A-<数字>': 电机下行指定距离
 *       - 'B': 进入编码器测试模式（循环上下移动并打印编码器值）
 *       - 'C': 电机步进分辨率测试
 *       - 'D': 电机下行触底测试
 *       - 'E': 电机上行碰零点测试
 *       - 'F': 罐底测量重复性测试
 *       - 'G': 罐底测量单次测试
 *       - 'H': 零点/罐底测量重复性测试
 *       - 'I': 零点单次测试
 *       - 'J': 液位测量重复性测试
 *       - 'K': 液位测量单次测试
 *       - 'L': 编码器值清零
 *       - 'M': 电机高温测试
 *       - 'N': 电机简单测试模式（循环上下移动）
 *       - 'O': 获取空载称重
 */
void process_command(uint8_t *command) {
	printf("Command received\n");
//	stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
//	stpr_enableDriver(&stepper);
	if (command[0] == 'A') {
		// 处理命令
		if (command[1] == '0') {
			// 刹车操作
			motorQuickStop();
		} else if (command[1] == '+') {
			int mm = atoi((char*) &command[2]);  // 获取数字
			printf("start up%d\n", mm);
			motorMoveNoWait((float) mm, MOTOR_DIRECTION_UP);  // 电机上行
		} else if (command[1] == '-') {
			int mm = atoi((char*) &command[2]);  // 获取数字
			printf("start down%d\n", mm);
			motorMoveNoWait((float) mm, MOTOR_DIRECTION_DOWN);  // 电机下行
		}
	}
	if (command[0] == 'B') {  //
		printf("motor text start\n");
		printf("***编码值清零***\r\n");
		g_encoder_count = 0; // 重置编码器计数
		int mm = atoi((char*) &command[1]);  // 获取数字
		printf("start up%d\n", mm);
		while (1) {
			stpr_enableDriver(&stepper);  //使能电机
//						stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
					//		stpr_moveTo(&stepper, -30 * 1600 * 32, 1600 * 2 * 32);
			motorMoveNoWait(200000, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			printf("start down\n");
			while (abs(g_encoder_count) < 43 * mm) {
				printf("{encoder}%d\t{weight}%d\r\n", (int) g_encoder_count, weight_parament.current_weight);
				HAL_Delay(50);
			}
			motorSlowStop();
			while (stpr_isMoving(&stepper));
			printf("down over!\n");

			HAL_Delay(1000);
			motorMoveNoWait(200000, MOTOR_DIRECTION_UP);
			printf("start up to zero\n");
			HAL_Delay(1000);
			while (abs(g_encoder_count) > 1000) {
				printf("{encoder}%d\t{weight}%d\r\n", (int) g_encoder_count, weight_parament.current_weight);
//				DSMSendcommand3times(DSM_POWER, strlen(DSM_POWER));
				HAL_Delay(50);
			}
			motorSlowStop();
			while (stpr_isMoving(&stepper));
			printf("上行结束\n");
			stpr_disableDriver(&stepper); //使能电机
		}
	}
	if (command[0] == 'C') {
		printf("***电机4步进分辨率测试***\r\n");
		motor_step_text();
	}
	if (command[0] == 'D') {
		printf("***电机4步进下行触底测试***\r\n");
		motor_step_down_text();
	}
	if (command[0] == 'E') {
		printf("***电机4步进上行碰零点测试***\r\n");
		motor_step_up_text();
	}
	if (command[0] == 'F') {
		printf("***罐底测量重复性测试***\r\n");
		while (1) {
			SearchBottom();
		}
	}
	if (command[0] == 'G') {
		printf("***罐底测量单次测试***\r\n");
		SearchBottom();
	}
	if (command[0] == 'H') {
		printf("***零点/罐底测量重复性测试***\r\n");
		while (1) {
			CMD_MeasureZero();
			SearchBottom();
		}
	}
	if (command[0] == 'I') {
		printf("执行回零点指令\n");
		CMD_MeasureZero(); //

	}
	if (command[0] == 'J') {
		printf("***液位测量重复性测试***\r\n");

	}
	if (command[0] == 'K') {
		printf("***液位测量单次测试***\r\n");

	}
	if (command[0] == 'L') {
		printf("***编码值清零***\r\n");
		g_encoder_count = 0; // 重置编码器计数
	}
	if (command[0] == 'M') {
		printf("***电机高温测试***\r\n");
		while (1) {
			motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			stpr_waitMove(&stepper);
		}

	}
	if (command[0] == 'N') {
		printf("motor text start\n");
		while (1) {
			stpr_enableDriver(&stepper);  //使能电机
//						stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
					//		stpr_moveTo(&stepper, -30 * 1600 * 32, 1600 * 2 * 32);
			motorMoveNoWait(300, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			printf("start down\n");
			HAL_Delay(1000);
			printf("down over!\n");
			stpr_waitMove(&stepper);
			motorMoveNoWait(300, MOTOR_DIRECTION_UP);
			printf("start up to zero\n");
			HAL_Delay(1000);
			stpr_waitMove(&stepper);
			printf("上行结束\n");
			HAL_Delay(1000);
			stpr_disableDriver(&stepper); //使能电机
		}
	}
	if (command[0] == 'O') //获取空载称重
			{
		printf("Get empty weight\n");
		get_empty_weight();

	}
	if (command[0] == 'P') //获取满载称重
			{
		printf("Get full weight\n");
		get_full_weight();

	}
	if (command[0] == 'Q') //获取满载称重
			{
		printf("恢复出场设置\n");
		RestoreFactoryParamsConfig(); //恢复出厂设置

	}
	if (command[0] == 'R') //获取满载称重
	{
		printf("执行分布测量指令\n");
		CMD_MeasureDensitySpread_Spread();

	}
	if (command[0] == 'W') //获取满载称重
	{
		printf("执行水位测量指令\n");
        CMD_MeasurWater();

	}
}

int MeasureStart(void) {
	fault_info_init(); //故障初始化清零
	g_measurement.device_status.error_code = NO_ERROR; //故障代码清零
	return NO_ERROR;
}

//测量水位主函数
static void CMD_MeasurWater(void) {
	uint32_t ret = 0;
	MeasureStart();
	g_measurement.device_status.device_state = STATE_FINDWATER;

	//开始回零点
	ret = SearchWaterLevel();
	SET_ERROR(ret);
	g_measurement.device_status.device_state = STATE_FINDWATER_OVER;
	//开始水位跟随
	ret = FollowWaterLevel();
	return;
}
//罐底零点主函数
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
	SET_ERROR(ret);

	g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
	return;
}
static void CMD_MeasureAndFollowOilLevel(void) {
	uint32_t ret = 0;
	MeasureStart();

	g_measurement.device_status.device_state = STATE_FINDOIL;
	//开始测量罐高
	ret = SearchAndFollowOilLevel();
	SET_ERROR(ret);
	return;
}

//标定液位
static void CMD_CalibrateOilLevel(void) {
	uint32_t ret = 0;
	MeasureStart();

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
	printf("Entering maintenance mode \n");
	g_measurement.device_status.device_state = STATE_MAINTENANCEMODE;
	while (1) {
		CHECK_COMMAND_SWITCH_NO_RETURN();
	}
}
// 电机上行指令
static void CMD_MoveUp(void)
{
    uint32_t ret = 0;

    printf("电机上行操作\n");
    g_measurement.device_status.device_state = STATE_RUNUPING;

    ret = motorMoveAndWaitUntilStop(
            (float)g_deviceParams.motorCommandDistance / 10.0f,
            MOTOR_DIRECTION_UP);

    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_RUNUPOVER;
    return;
}
// 电机下行指令
static void CMD_MoveDown(void)
{
    uint32_t ret = 0;

    printf("电机下行操作\n");
    g_measurement.device_status.device_state = STATE_RUNDOWNING;

    ret = motorMoveAndWaitUntilStop(
            (float)g_deviceParams.motorCommandDistance / 10.0f,
            MOTOR_DIRECTION_DOWN);

    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_RUNDOWNOVER;
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
static void CMD_WartsilaDensitySpread(void) {
	uint32_t ret = 0;
	// 设置设备状态：分布测量中
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_START;

	ret = Wartsila_Density_SpreadMeasurement(&g_measurement.density_distribution);
// 记录/上报错误码（和零点测量一样用 SET_ERROR）
	SET_ERROR(ret);

	HAL_Delay(1000);
	Print_DensitySpreadResult(&g_measurement.density_distribution);
// 测量结束，状态切换为分布测量完成
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_OVER;

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
        printf("普通分布测\t失败, err=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }


    // 4. 测量成功, 写回全局结果
    g_measurement.density_distribution = temp;
    Print_DensitySpreadResult(&g_measurement.density_distribution);
// 测量结束，状态切换为分布测量完成
	g_measurement.device_status.device_state = STATE_SYNTHETICING_OVER;

	return;
}
