/*
 * measure_zero.c
 *
 *  用于测量和校准设备的零点位置
 *  包含粗略和精确两步寻找零点的过程
 *  依赖外部的重量传感器和电机控制接口
 */

#include "weight.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_zero.h"
#include "system_parameter.h"
#include "motor_ctrl.h"

// 全局变量，记录零点的编码器数值
int32_t zero_position ;

// 内部函数声明：粗略和精确寻找零点
static int SearchZeroRough();
static int SearchZeroPrecise();

/**
 * @brief 主零点搜索流程
 *        先进行多次粗略找零点，成功后再进行两次精确找零点
 *        最终记录零点编码器值
 * @return 0表示成功，1表示三次粗找零点失败
 */
int SearchZero(void) {
	uint32_t ret;
	uint8_t try_times = 0;
	fault_info_init(); // 清除故障信息
	printf("零点测量\t开始\r\n");
	if (g_measurement.debug_data.cable_length <2000) { // 如果尺带长度大于200mm，先将电机下行到安全位置
		ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_DOWN);
		CHECK_ERROR(ret); // 检查下行是否成功
		printf("零点测量\t脱离零点完成\r\n");
	}
	printf("零点测量\t初始重量：%d\r\n", weight_parament.stable_weight);
	/*************** 粗找阶段 - 带重试机制 ***************/
	while (try_times < 3) {
		try_times++;
		//清除错误
		fault_info_init(); // 清除故障信息
		printf("零点测量\t粗找零点第%d次尝试\r\n", try_times);

		ret = SearchZeroRough();
		CHECK_COMMAND_SWITCH(ret);// 检查是否收到命令切换请求
		//位置判断，与上一个零点距离差
		if (abs(g_measurement.debug_data.cable_length) > 100) { //
			printf("零点测量\t编码值异常，可能需要重新校准\r\n");
			ret =MEASUREMENT_ZERO_OUT_OF_RANGE;
			ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_DOWN);
			CHECK_ERROR(ret); // 检查下行是否成功
			continue; // 继续尝试
		}
		HAL_Delay(3000); // 短暂等待
		// 优化：检查是否真正到达零点
		if (check_zero_point_status() == ZERO)
			break;
		if (ret != NO_ERROR) {
			printf("零点测量\t粗找失败[%d]:0x%X\r\n", try_times,  (unsigned int)ret);
			continue; // 继续尝试
		}
	}
	CHECK_ERROR(ret); // 检查粗找零点是否成功,是否有错误代码
	if (try_times >= 3) {
		printf("零点测量\t粗找零点失败(尝试%d次)\r\n", try_times);
		CHECK_ERROR(MEASUREMENT_WEIGHT_UP_FAIL);
	}
	printf("零点测量\t粗找零点完成\t位置%d\r\n",zero_position);

	/*************** 精找阶段 - 增强错误处理 ***************/
	// 电机继续下行，为后续精确找零点做准备
	ret = motorMoveAndWaitUntilStop(200.0, MOTOR_DIRECTION_DOWN);
	CHECK_ERROR(ret); // 检查下行是否成功
	// 第一次精确找零点
	ret = SearchZeroPrecise();
	if (ret != NO_ERROR) {
		printf("零点测量\t第一次精找失败:0x%X\r\n",  (unsigned int)ret);
		CHECK_ERROR (ret);
	}
	printf("零点测量\t第一次精找完成\t位置%d\r\n",zero_position);

	// 再次下行，进一步精确
	ret = motorMoveAndWaitUntilStop(100.0, MOTOR_DIRECTION_DOWN);
	CHECK_ERROR(ret); // 检查下行是否成功
	printf("零点测量\t电机下行完成\r\n");

	// 第二次精确找零点
	ret = SearchZeroPrecise();
	if (ret != NO_ERROR) {
		printf("零点测量\t第二次精找失败:0x%X\r\n",  (unsigned int)ret);
		CHECK_ERROR (ret);
	}
	printf("零点测量\t第二次精找完成\r\n");

	/*************** 最终校验与记录 ***************/
	// 记录最终零点编码器值
	printf("零点测量完成 \t当前编码值\t %ld\r\n", g_encoder_count);
	printf("{zero_value}%ld mm\r\n", g_measurement.debug_data.cable_length);
	if (abs(g_measurement.debug_data.cable_length) > 100) { //
		printf("零点测量\t编码值异常，可能需要重新校准\r\n");
		CHECK_ERROR(MEASUREMENT_ZERO_OUT_OF_RANGE); // 检查零点编码值是否在合理范围内
	} else {
		set_encoder_zero(); // 设置编码器零点
		printf("零点测量\t编码器零点设置成功\r\n");
		// 电机继续下行，完成流程
		ret = motorMoveAndWaitUntilStop(100, MOTOR_DIRECTION_DOWN);
		CHECK_ERROR(ret); // 检查下行是否成功
		printf("零点测量\t电机下行完成，流程结束\r\n");
	}
	g_measurement.device_status.zero_point_status = 0; // 设置零点状态为正常
	return NO_ERROR; // 返回无错误状态
}

/**
 * @brief 粗略寻找零点
 *        电机上行，直到重量状态为ZERO
 *        记录此时编码器值为零点
 * @return 总是返回0
 */
static int SearchZeroRough() {
	uint32_t ret;
	ret = motorMove_up();// 电机上行
	CHECK_ERROR(ret); // 检查上行是否成功
	// 循环直到重量状态为ZERO
	while (check_zero_point_status() != ZERO) {
		// 实时打印编码器和重量信息
		printf("零点测量\t长距离寻找零点\t{传感器位置}%.1f\t{称重值}%d\r\n", (float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
		//碰撞检测
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); // 检查碰撞检测是否成功
		//丢步检测
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
	}
	zero_position = g_measurement.debug_data.cable_length;
	ret = motorQuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	return NO_ERROR; // 返回无错误状态
}

/**
 * @brief 精确寻找零点
 *        电机上行，靠近零点时逐步降低速度，提高精度
 *        记录此时编码器值为零点
 * @return 总是返回0
 */
static int SearchZeroPrecise() {
	uint32_t ret;
	int v1flag = 0;// 速度调整标志1
	int v2flag = 0;// 速度调整标志2
	// motorMove_up()：电机上行
	ret = motorMove_up();
	CHECK_ERROR(ret); // 检查上行是否成功
	while (check_zero_point_status() != ZERO) {
		printf("零点测量\t精确寻找零点\t{传感器位置}%.1f\t{称重值}%d\r\n", (float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
		// 距离零点4096以内，速度降为40
		if (((g_measurement.debug_data.cable_length-zero_position)< 1000) && (v1flag == 0)) {
			printf("零点位置%d\t尺带长度%d\t", zero_position, g_measurement.debug_data.cable_length);
			stpr_setVelocity(&stepper, 16 * 32 * 40);
			printf("速度调整\t{Velocity}%d\r\n", 40);
			v1flag = 1;
		}
		// 距离零点100以内，速度降为2
		if (((g_measurement.debug_data.cable_length-zero_position) < 100) && (v2flag == 0)) {

			printf("零点位置%d\t尺带长度%d\t", zero_position, g_measurement.debug_data.cable_length);
			stpr_setVelocity(&stepper, 16 * 32 * 2);
			printf("速度调整\t{Velocity}%d\r\n", 2);
			v2flag = 1;
		}
		if(g_measurement.debug_data.cable_length<(zero_position-100)) {
			printf("零点测量\t精找零点超出范围\t尺带长度\t%d\r\n", g_measurement.debug_data.cable_length);
			ret = motorQuickStop(); // 到达零点后快速停止电机
			CHECK_ERROR(ret); // 检查快速停止是否成功
			CHECK_ERROR(MEASUREMENT_WEIGHT_UP_FAIL); // 检查快速停止是否成功
		}
		//碰撞检测
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); // 检查碰撞检测是否成功

		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
	}
	zero_position = g_measurement.debug_data.cable_length;
	ret = motorQuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	return NO_ERROR; // 返回无错误状态
}
