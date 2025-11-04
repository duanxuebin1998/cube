/*
 * measureTankHeight.c - 罐体高度测量模块
 *
 * 功能说明：
 *   该模块控制电机运动，通过重量传感器检测罐底位置，实现罐体高度的精确测量。
 *   测量过程分为两个阶段：粗略搜索和精确搜索。
 *
 * 创建日期: Feb 27, 2025
 * 作者: 1
 */

#include "measure_tank_height.h"
#include "weight.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_zero.h"
#include "motor_ctrl.h"
// 全局变量：存储最终确定的罐底位置（编码器计数值）
int32_t bottom_value = -100000000; // 初始值设为较大数值作为无效状态标识

// 函数原型声明
static int SearchBottomRough();   // 粗略搜索罐底
static int SearchBottomPrecise(); // 精确搜索罐底

/**
 * @brief 主测量函数 - 执行罐底位置的完整搜索流程
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
//uint32_t SearchBottom(void) {
//	uint32_t ret;
//	int try_times = 0;
//	fault_info_init();  // 清除故障信息
//	printf("罐底测量\t开始\r\n");
//	if (g_measurement.device_status.zero_point_status == 1) {
//		printf("罐底测量\t设备需要回零点\r\n");
//		ret = SearchZero();  // 如果设备需要回零点，先执行回零点测量
//		CHECK_ERROR(ret);  // 检查回零点是否成功
//		printf("罐底测量\t回零点完成\r\n");
//	}
//	if (g_measurement.debug_data.cable_length > 2000) {  // 如果尺带长度大于200mm，先将电机上行到安全位置
//		ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_UP);
//		CHECK_ERROR(ret);  // 检查上行是否成功
//		printf("罐底测量\t上行完成\r\n");
//	}
//	printf("罐底测量\t初始重量：%d\r\n", weight_parament.stable_weight);
//	/*************** 粗找阶段 - 带重试机制 ***************/
//	while (try_times < 3) {
//		try_times++;
//		printf("罐底测量\t粗找罐底第%d次尝试\r\n", try_times);
//
//		ret = SearchBottomRough();
//		CHECK_COMMAND_SWITCH(ret);// 检查是否收到命令切换请求
//		if (ret != NO_ERROR) {
//			printf("罐底测量\t粗找失败[%d]:0x%lX\r\n", try_times, ret);
//			continue; // 继续尝试
//		}
//		else {
//			break;  // 成功找到罐底，退出循环
//		}
//	}
//	CHECK_ERROR(ret); // 检查粗找罐底是否成功,是否有错误代码
//	if (try_times > 3) {
//		printf("罐底测量\t粗找罐底失败(尝试%d次)\r\n", try_times);
//		RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL);
//	}
//	printf("罐底测量\t粗找罐底完成：实高：%ld mm\r\n", bottom_value);
//	/*************** 精找阶段 - 增强错误处理 ***************/
//	// 电机继续提升，为精确搜索做准备
//	ret = motorMoveAndWaitUntilStop(200, MOTOR_DIRECTION_UP);  // 电机小幅度提升，为精确搜索做准备
//	CHECK_ERROR(ret); // 检查提升是否成功
//	// 第一次精确找罐底
//	ret = SearchBottomPrecise();  // 执行精确搜索
//	if (ret != NO_ERROR) {
//		printf("罐底测量\t第一次精找失败:0x%lX\r\n", ret);
//		CHECK_ERROR(ret);
//	}
//	printf("罐底测量\t第一次精找完成\r\n");
//
//	// 再次下行，进一步精确
//	ret = motorMoveAndWaitUntilStop(100, MOTOR_DIRECTION_UP);
//	CHECK_ERROR(ret); // 检查下行是否成功
//	printf("罐底测量\t电机上行完成\r\n");
//
//	// 第二次精确找零点
//	ret = SearchBottomPrecise();
//	if (ret != NO_ERROR) {
//		printf("罐底测量\t第二次精找失败:0x%lX\r\n", ret);
//		CHECK_ERROR(ret);
//	}
//	printf("罐底测量\t第二次精找完成\r\n");
//	/*************** 最终校验与记录 ***************/
//	g_measurement.height_measurement.current_real_height = g_measurement.debug_data.cable_length;  // 更新测量数据中的传感器位置
//	// 打印测量结果
//	printf("罐底测量\t实高：%ld mm\r\n", g_measurement.debug_data.cable_length);
//
////	if (g_measurement.height_measurement.current_real_height < 0) {
////		printf("罐底测量\t传感器位置异常，可能需要重新校准\r\n");
////		CHECK_ERROR(MEASUREMENT_POSITION_ABNORMAL);  // 如果位置小于0，打印错误信息并返回
////	} else if (g_measurement.height_measurement.current_real_height > g_deviceParams.tankHeight) {
////		printf("罐底测量\t传感器位置超出罐体高度\r\n");
////		CHECK_ERROR(MEASUREMENT_POSITION_OUT_OF_RANGE);  // 如果位置超过罐体高度，打印错误信息并返回
////	}
//	// 电机继续下行，完成流程
//	ret = motorMoveAndWaitUntilStop(100, MOTOR_DIRECTION_UP);
//	CHECK_ERROR(ret); // 检查下行是否成功
//	printf("罐底测量\t电机上行完成，流程结束\r\n");
//
//	return NO_ERROR;  // 返回成功状态
//}
/**
 * @brief 罐底测量函数 - 执行完整的罐底搜索流程
 *        包含：粗找罐底（3次重试） + 两次精找（各3次重试）
 *        状态切换时不重试
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
uint32_t SearchBottom(void)
{
    uint32_t ret;
    uint8_t try_times = 0;

    fault_info_init();  // 清除故障信息
    printf("罐底测量\t开始\r\n");

    /* -------------------- 零点检查 -------------------- */
    if (g_measurement.device_status.zero_point_status == 1)
    {
        printf("罐底测量\t设备需要回零点\r\n");
        ret = SearchZero();       // 执行回零点测量
        CHECK_ERROR(ret);         // 检查是否成功
        printf("罐底测量\t回零点完成\r\n");
    }

    /* -------------------- 初始位置调整 -------------------- */
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_UP);
        CHECK_ERROR(ret);
        printf("罐底测量\t上行完成\r\n");
    }

    printf("罐底测量\t初始重量：%d\r\n", weight_parament.stable_weight);

    /*************** 粗找阶段 - 带重试机制 ***************/
    try_times = 0;
    while (try_times < 3)
    {
        try_times++;
        printf("罐底测量\t粗找罐底第%d次尝试\r\n", try_times);

        fault_info_init();  // 清除故障信息
        ret = SearchBottomRough();

        if (ret == STATE_SWITCH)
        {
            printf("罐底测量\t检测到命令切换，中止粗找\r\n");
            break;
        }

        CHECK_COMMAND_SWITCH(ret);

        if (ret != NO_ERROR)
        {
            printf("罐底测量\t粗找失败[%d]:0x%lX\r\n", try_times, ret);
            HAL_Delay(1000);
            continue;  // 继续尝试
        }
        else
        {
            printf("罐底测量\t粗找罐底成功\r\n");
            break;
        }
    }

    if (ret != NO_ERROR)
    {
        printf("罐底测量\t粗找罐底失败(尝试%d次)\r\n", try_times);
        RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL);
    }

    printf("罐底测量\t粗找罐底完成：实高：%ld mm\r\n", bottom_value);

    /*************** 精找阶段1 - 带重试 ***************/

    try_times = 0;
    while (try_times < 3)
    {
        try_times++;
        printf("罐底测量\t第一次精找第%d次尝试\r\n", try_times);

        ret = SearchBottomPrecise();

        if (ret == STATE_SWITCH)
        {
            printf("罐底测量\t检测到命令切换，中止第一次精找\r\n");
            break;
        }

        if (ret == NO_ERROR)
        {
            printf("罐底测量\t第一次精找完成\r\n");
            break;
        }
        else
        {
            printf("罐底测量\t第一次精找失败:0x%lX\r\n", ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        printf("罐底测量\t第一次精找失败(尝试%d次)\r\n", try_times);
        CHECK_ERROR(ret);
    }

    /*************** 精找阶段2 - 带重试 ***************/
    printf("罐底测量\t电机上行完成，准备第二次精找\r\n");

    try_times = 0;
    while (try_times < 3)
    {
        try_times++;
        printf("罐底测量\t第二次精找第%d次尝试\r\n", try_times);

        ret = SearchBottomPrecise();

        if (ret == STATE_SWITCH)
        {
            printf("罐底测量\t检测到命令切换，中止第二次精找\r\n");
            break;
        }

        if (ret == NO_ERROR)
        {
            printf("罐底测量\t第二次精找完成\r\n");
            break;
        }
        else
        {
            printf("罐底测量\t第二次精找失败:0x%lX\r\n", ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        printf("罐底测量\t第二次精找失败(尝试%d次)\r\n", try_times);
        CHECK_ERROR(ret);
    }

    /*************** 最终校验与记录 ***************/
    g_measurement.height_measurement.current_real_height = g_measurement.debug_data.cable_length;
    printf("罐底测量\t实高：%ld mm\r\n", g_measurement.debug_data.cable_length);

    // 电机上行，完成流程
    ret = motorMoveAndWaitUntilStop(100, MOTOR_DIRECTION_UP);
    CHECK_ERROR(ret);
    printf("罐底测量\t电机上行完成，流程结束\r\n");

    return NO_ERROR;
}

/**
 * @brief 粗略搜索罐底 - 快速下探直到检测到罐底
 *
 * @return int 错误代码（NO_ERROR表示成功）
 */
static int SearchBottomRough() {
	uint32_t ret;
	ret = motorMove_down();  // 启动电机向下运动
	CHECK_ERROR(ret); // 检查上行是否成功
	// 持续监控重量状态，直到检测到罐底
	while (check_bottom_status() == NORMAL) {
		// 实时输出编码器位置和重量值（用于调试）
		printf("罐底测量\t长距离寻找罐底\t{传感器位置}%.1f\t{称重值}%d\r\n", (float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); // 检查碰撞检测是否成功
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
	}
	ret = motorQuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	HAL_Delay(3000); // 短暂等待
	// 优化：检查是否真正到达零点
	if (check_bottom_status() == BOTTOM)
	{
		// 记录首次检测到的罐底位置
		bottom_value = g_measurement.debug_data.cable_length;
		return NO_ERROR;
	}
	else {
		printf("罐底测量\t粗找罐底未成功，尝试再次粗找\r\n");
		ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_UP);
		CHECK_ERROR(ret);  // 检查上行是否成功
		printf("罐底测量\t上行100mm\r\n");
		return MEASUREMENT_WEIGHT_DOWN_FAIL;
	}
}

/**
 * @brief 精确搜索罐底 - 使用变速策略精确定位罐底
 *
 * @return int 错误代码（NO_ERROR表示成功）
 */
static int SearchBottomPrecise() {
	uint32_t ret;
	int v1flag = 0;  // 第一速度切换标志
	int v2flag = 0;  // 第二速度切换标志
	printf("罐底测量\t稳定重量：%d\r\n", weight_parament.stable_weight);
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_UP);
        CHECK_ERROR(ret);
        printf("罐底测量\t上行完成\r\n");
    }
	ret = motorMove_down();     // 启动电机向下运动
	CHECK_ERROR(ret); // 检查上行是否成功
	// 持续监控重量状态，直到检测到罐底
	while (check_bottom_status() != BOTTOM) {
		// 实时输出编码器位置和重量值（用于调试）
		printf("罐底测量\t精确寻找罐底\t{传感器位置}%.1f\t{称重值}%d\r\n", (float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
		// 距离上次粗定位位置4096步时减速（约10cm）
		if ((bottom_value-g_measurement.debug_data.cable_length  < 1000) && (v1flag == 0)) {
			stpr_setVelocity(&stepper, 16 * 32 * 40); // 设置中等速度
			printf("罐底测量\t速度变化：%d\r\n", 40); // 速度变化通知
			v1flag = 1;  // 设置速度切换标志（只执行一次）
		}

		// 距离粗定位位置100步时再次减速（约2.3mm）
		if ((bottom_value-g_measurement.debug_data.cable_length < 100) && (v2flag == 0)) {
			stpr_setVelocity(&stepper, 16 * 32 * 2);  // 设置慢速
			printf("罐底测量\t速度变化：%d\r\n", 2); // 速度变化通知
			v2flag = 1;  // 设置速度切换标志（只执行一次）
		}
		// 距离粗定位位置100步时再次减速（约2.3mm）
		if (bottom_value-g_measurement.debug_data.cable_length < -100)  {
			printf("罐底测量\tt精确寻找罐底未找到罐底\r\n");
			RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL); // 如果编码器位置异常，返回错误
		}
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); // 检查碰撞检测是否成功
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
	}
	ret = motorQuickStop();
	CHECK_ERROR(ret); // 检查快速停止是否成功
	// 更新罐底位置并停止电机
	bottom_value = g_measurement.debug_data.cable_length;
	return NO_ERROR;
}
