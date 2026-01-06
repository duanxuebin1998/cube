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
/* 全局/参数区：由寄存器或本机参数配置 */
BottomDetectMode g_bottom_det_mode = BOTTOM_DET_BY_WEIGHT;

/* 角度阈值：单位“度”，建议可配置 */
float g_bottom_gyro_th_deg = 10.0f;

static GyroZeroRef g_gyro_zero_ref = {0};
// 函数原型声明
static int SearchBottomRough();   // 粗略搜索罐底
static int SearchBottomPrecise(); // 精确搜索罐底


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
//    printf("罐底测量\t电机上行完成，准备第二次精找\r\n");
//
//    try_times = 0;
//    while (try_times < 3)
//    {
//        try_times++;
//        printf("罐底测量\t第二次精找第%d次尝试\r\n", try_times);
//
//        ret = SearchBottomPrecise();
//
//        if (ret == STATE_SWITCH)
//        {
//            printf("罐底测量\t检测到命令切换，中止第二次精找\r\n");
//            break;
//        }
//
//        if (ret == NO_ERROR)
//        {
//            printf("罐底测量\t第二次精找完成\r\n");
//            break;
//        }
//        else
//        {
//            printf("罐底测量\t第二次精找失败:0x%lX\r\n", ret);
//            HAL_Delay(1000);
//        }
//    }
//
//    if (ret != NO_ERROR)
//    {
//        printf("罐底测量\t第二次精找失败(尝试%d次)\r\n", try_times);
//        CHECK_ERROR(ret);
//    }

    /*************** 最终校验与记录 ***************/
    g_measurement.height_measurement.current_real_height = g_measurement.debug_data.cable_length;
    printf("罐底测量\t实高：%ld mm\r\n", g_measurement.debug_data.cable_length);
    if(g_measurement.device_status.device_state == STATE_CALIBRATIONOILING)
    {
		g_deviceParams.tankHeight = g_measurement.debug_data.cable_length +  1;//TODO:这里的1应该改成液位传感器到底部距离差
		printf("罐底测量\t标定完成，罐高设置为：%ld mm\r\n", g_deviceParams.tankHeight);
		update_sensor_height_from_encoder();	//更新罐高数据
    }
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
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
		printf("罐底测量\t长距离寻找罐底\t{传感器位置}%.1f\t称重\t= %ld\t", (float)(g_measurement.debug_data.sensor_position)/10.0,weight_parament.current_weight);
	}
	ret = motorQuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	HAL_Delay(3000); // 短暂等待
	// 优化：检查是否真正到达零点
	printf("罐底测量\t确认粗找罐底位置\t{传感器位置}%.1f\t", (float)(g_measurement.debug_data.sensor_position)/10.0);
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
//		ret = CheckWeightCollision();
//		CHECK_ERROR(ret); // 检查碰撞检测是否成功
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
		printf("罐底测量\t精确寻找罐底\t{传感器位置}%.1f\t", (float)(g_measurement.debug_data.sensor_position)/10.0);
	}
	ret = motorQuickStop();
	CHECK_ERROR(ret); // 检查快速停止是否成功
	// 更新罐底位置并停止电机
	bottom_value = g_measurement.debug_data.cable_length;
	return NO_ERROR;
}
uint32_t Bottom_SaveGyroZeroRef(void)
{
    float ax = 0.0f, ay = 0.0f;
    uint32_t ret = Read_Gyro_Angle(&ax, &ay);   // 你前面已加的 Ch 指令函数
    if (ret != NO_ERROR) {
        g_gyro_zero_ref.valid = 0;
        return ret;
    }

    g_gyro_zero_ref.x0_deg = ax;
    g_gyro_zero_ref.y0_deg = ay;
    g_gyro_zero_ref.valid  = 1;

    printf("陀螺仪零点基准 | X0=%.2f | Y0=%.2f\r\n", ax, ay);
    return NO_ERROR;
}


/**
 * @brief 罐底状态检测（不锁存）
 * @return BOTTOM（到达罐底）或 NORMAL（未到罐底）
 */
Weight_StateTypeDef check_bottom_status(void)
{
    /* -------- 方式1：称重阈值 -------- */
    if (g_bottom_det_mode == BOTTOM_DET_BY_WEIGHT) {

        int lower_limit = BOTTOM_WEIGHT_THRESHOLD;
        int current     = weight_parament.current_weight;

        Weight_StateTypeDef state =
            (current < lower_limit) ? BOTTOM : NORMAL;

        printf("罐底检测(称重) | 当前:%d | 阈值:%d | 状态:%s\r\n",
               current,
               lower_limit,
               (state == BOTTOM) ? "到达罐底" : "正常");

        return state;
    }

    /* -------- 方式2：陀螺仪角度变化（不锁存） -------- */
    if (!g_gyro_zero_ref.valid) {
        printf("罐底检测(陀螺仪) | 零点基准无效\r\n");
        return NORMAL;
    }

    float ax = 0.0f, ay = 0.0f;
    uint32_t ret = Read_Gyro_Angle(&ax, &ay);
    if (ret != NO_ERROR) {
        printf("罐底检测(陀螺仪) | 读取失败 ret=%lu\r\n",
               (unsigned long)ret);
        return NORMAL;
    }

    float dx = fabsf(ax - g_gyro_zero_ref.x0_deg);
    float dy = fabsf(fabsf(ay) - fabsf(g_gyro_zero_ref.y0_deg));
    float dsum = dx + dy;

    Weight_StateTypeDef state =
        (dsum > g_bottom_gyro_th_deg) ? BOTTOM : NORMAL;

    printf("罐底检测 (称重) %d (陀螺仪) | X=%.2f Y=%.2f | X0=%.2f Y0=%.2f | "
           "dX=%.2f dY=%.2f sum=%.2f | th=%.2f | 状态:%s\r\n",
		   weight_parament.current_weight,
           ax, ay,
           g_gyro_zero_ref.x0_deg, g_gyro_zero_ref.y0_deg,
           dx, dy, dsum,
           g_bottom_gyro_th_deg,
           (state == BOTTOM) ? "到达罐底" : "正常");

    return state;
}

