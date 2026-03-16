/*
 * test.c
 * 测试函数，用来临时测试某些功能
 *  Created on: Jul 22, 2025
 *      Author: Duan Xuebin
 */

#include <ltd_sensor_communication.h>
#include "test.h"
#include "measure.h"
#include "motor_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include "system_parameter.h"
#include "measure_tank_height.h"
#include "weight.h"
#include "system_parameter.h"
#include "sensor.h"
#include <mb85rs2m.h>
#include "my_crc.h"
#include "ad5421.h"
#include "encoder.h"
#include <stddef.h>
//电机小步进上行测试
void motor_step_up_text(void) {
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("start up\n");
	for (i = 0; i < 24000; i++) {
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{传感器位置}%.1f\t{称重值}%d\r\n", i, (float) (g_measurement.debug_data.sensor_position) / 10.0, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
	}
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}
//电机小步进下行测试
void motor_step_down_text(void) {
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("start down\n");
	for (i = 0; i < 24000; i++) {
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{传感器位置}%.1f\t{称重值}%d\r\n", i, (float) (g_measurement.debug_data.sensor_position) / 10.0, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
	}
	printf("down over!\n");
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}
//电机步进测试
void motor_step_text(void) {
	int i = 0;
//	float density, viscosity, temp;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("4步进测试\n");
	printf("start down\n");
	for (i = 0; i < 24000; i++) {
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
	}
	printf("down over!\n");
	printf("start up\n");
	for (i = 0; i < 24000; i++) {
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);

	}
	printf("8步进测试\n");
	printf("start down\n");
	for (i = 0; i < 12000; i++) {
		ticks = 8 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
	}
	printf("down over!\n");
	printf("start up\n");
	for (i = 0; i < 12000; i++) {
		ticks = -8 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);

	}
	printf("40步进测试\n");
	printf("start down\n");
	for (i = 0; i < 2400; i++) {
		ticks = 40 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
	}
	printf("down over!\n");
	printf("start up\n");
	for (i = 0; i < 2400; i++) {
		ticks = -40 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);

	}
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}

///*********************** 测试函数 ***********************/
void Test_Params_Storage(void) {
	// 备份原始参数
	DeviceParameters original = g_deviceParams;

	// 测试写读校验
	g_deviceParams.tankHeight = 1234; // 测试数据
	save_device_params(); //存储

	if (load_device_params()) {
		if (g_deviceParams.tankHeight != 1234) {
			printf("数据加载失败");
			// 数据验证失败处理
		} else {
			printf("数据加载成功: tankHeight = %lu", g_deviceParams.tankHeight);
		}
	} else {
		printf("CRC校验失败");
	}

	// 恢复原始参数
	g_deviceParams = original;
	save_device_params(); //存储
}

#define TEST_ENCODER_SLOT_SIZE  (0x40u)
#define TEST_ENCODER_A_ADDRESS   FRAM_ANGLE_ADDRESS
#define TEST_ENCODER_B_ADDRESS   (TEST_ENCODER_A_ADDRESS + TEST_ENCODER_SLOT_SIZE)

void Test_ParamEncoder_AB_Backup(void)
{
    DeviceParameters param_backup = g_deviceParams;
    uint8_t param_a_raw[sizeof(DeviceParameters)] = {0};
    uint8_t param_b_raw[sizeof(DeviceParameters)] = {0};

    uint8_t encoder_a_raw[TEST_ENCODER_SLOT_SIZE] = {0};
    uint8_t encoder_b_raw[TEST_ENCODER_SLOT_SIZE] = {0};

    const uint32_t param_magic_offset = (uint32_t)offsetof(DeviceParameters, magic);

    ReadMultiData(param_a_raw, FRAM_PARAM_A_ADDRESS, sizeof(param_a_raw));
    ReadMultiData(param_b_raw, FRAM_PARAM_B_ADDRESS, sizeof(param_b_raw));
    ReadMultiData(encoder_a_raw, TEST_ENCODER_A_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    ReadMultiData(encoder_b_raw, TEST_ENCODER_B_ADDRESS, TEST_ENCODER_SLOT_SIZE);

    printf("\r\n===== AB双备份回退测试开始 =====\r\n");

    /* Case-1: 参数A损坏，读取应回退到B */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, FRAM_PARAM_A_ADDRESS + param_magic_offset);

    if (load_device_params()) {
        printf("[PASS] 参数区: A损坏后已回退到B\r\n");
    } else {
        printf("[FAIL] 参数区: A损坏后未能回退到B\r\n");
    }

    /* Case-2: 参数A/B都损坏，读取应报错 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, FRAM_PARAM_A_ADDRESS + param_magic_offset);
    WriteSingleData(0u, FRAM_PARAM_B_ADDRESS + param_magic_offset);

    if ((!load_device_params()) && (g_measurement.device_status.error_code == PARAM_EEPROM_FAIL)) {
        printf("[PASS] 参数区: A/B都损坏时已报错 PARAM_EEPROM_FAIL\r\n");
    } else {
        printf("[FAIL] 参数区: A/B都损坏时报错不符合预期, err=0x%08lX\r\n",
               (unsigned long)g_measurement.device_status.error_code);
    }

    /* 恢复参数区并回写双分区 */
    WriteMultiData(param_a_raw, FRAM_PARAM_A_ADDRESS, sizeof(param_a_raw));
    WriteMultiData(param_b_raw, FRAM_PARAM_B_ADDRESS, sizeof(param_b_raw));
    g_deviceParams = param_backup;
    save_device_params();

    /* Case-3: 编码A损坏，初始化应回退到B */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, TEST_ENCODER_A_ADDRESS);
    Initialize_Encoder();

    if (g_measurement.device_status.error_code != ENCODER_POWERON_FAIL) {
        printf("[PASS] 编码区: A损坏后已回退到B\r\n");
    } else {
        printf("[FAIL] 编码区: A损坏后未能回退到B\r\n");
    }

    /* Case-4: 编码A/B都损坏，应报错 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, TEST_ENCODER_A_ADDRESS);
    WriteSingleData(0u, TEST_ENCODER_B_ADDRESS);
    Initialize_Encoder();

    if (g_measurement.device_status.error_code == ENCODER_POWERON_FAIL) {
        printf("[PASS] 编码区: A/B都损坏时已报错 ENCODER_POWERON_FAIL\r\n");
    } else {
        printf("[FAIL] 编码区: A/B都损坏时报错不符合预期, err=0x%08lX\r\n",
               (unsigned long)g_measurement.device_status.error_code);
    }

    /* 恢复编码区 */
    WriteMultiData(encoder_a_raw, TEST_ENCODER_A_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    WriteMultiData(encoder_b_raw, TEST_ENCODER_B_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    Initialize_Encoder();

    printf("===== AB双备份回退测试结束 =====\r\n\r\n");
}
static void Sensor_CommCheckAndLog(const char *tag)
{
    float temp = 0.0f;
    float frequency = 0.0f;
    float density = 0.0f;
    float hz_45 = 0.0f, hz_225 = 0.0f;

    static uint32_t comm_fail_cnt = 0;
    uint32_t ret;

    /* 读温度 */
    ret = DSM_V2_Read_Temperature(&temp);
    if (ret == NO_ERROR) {
        printf("[SENSOR][OK ] %s Temp=%.3f C\r\n", tag, temp);
    } else {
        comm_fail_cnt++;
        printf("[SENSOR][ERR] %s Temp fail, ret=%lu, fail_cnt=%lu\r\n",
               tag, (unsigned long)ret, (unsigned long)comm_fail_cnt);
    }

    /* 读密度 */
    ret = DSM_V2_Read_Density(&density);
    if (ret == NO_ERROR) {
        printf("[SENSOR][OK ] %s Density=%.3f\r\n", tag, density);
    } else {
        comm_fail_cnt++;
        printf("[SENSOR][ERR] %s Density fail, ret=%lu, fail_cnt=%lu\r\n",
               tag, (unsigned long)ret, (unsigned long)comm_fail_cnt);
    }

    /* 读频率 + 扫频均值 */
    ret = DSM_V2_Read_DensityFrequency(&frequency, &hz_45, &hz_225);
    if (ret == NO_ERROR) {
        printf("[SENSOR][OK ] %s F=%.1f Hz, Hz45=%.2f, Hz22.5=%.2f\r\n",
               tag, frequency, hz_45, hz_225);
    } else {
        comm_fail_cnt++;
        printf("[SENSOR][ERR] %s Freq fail, ret=%lu, fail_cnt=%lu\r\n",
               tag, (unsigned long)ret, (unsigned long)comm_fail_cnt);
    }
}


/* ========================= 主测试函数 ========================= */
void motor_text(void)
{
    uint32_t loop_cnt = 0;

    printf("motor text start\r\n");
    motor_Init(); // 电机初始化

    while (1) {
        /* ---------- 下行 ---------- */
        motorMoveNoWaitWithSpeed(300, MOTOR_DIRECTION_DOWN, motorGetDefaultSpeedX100());
        HAL_Delay(1000);
        printf("[LOOP %lu] start down\r\n", (unsigned long)(loop_cnt + 1));

        stpr_waitMove(&stepper);
        printf("[LOOP %lu] down over!\r\n", (unsigned long)(loop_cnt + 1));

        /* 下行结束后：传感器通讯一次 */
//        Sensor_CommCheckAndLog("after DOWN");

        /* ---------- 回零（上行到0） ---------- */
        stpr_moveTo(&stepper, 0, velocity); // 最大速度为 1600 * 2 * 32
        printf("[LOOP %lu] start up to zero\r\n", (unsigned long)(loop_cnt + 1));
        HAL_Delay(1000);

        stpr_waitMove(&stepper);
        printf("[LOOP %lu] up over (to zero)\r\n", (unsigned long)(loop_cnt + 1));

        /* 上行结束后：传感器通讯一次 */
//        Sensor_CommCheckAndLog("after UP");

        /* 本轮完成计数（下+上算一轮） */
        loop_cnt++;
        printf("[LOOP %lu] cycle done\r\n", (unsigned long)loop_cnt);
    }
}
#include <ltd_sensor_communication.h>
#include <stdio.h>

/**
 * @brief  测试V2协议通讯与关键参数读取
 * @note   可在初始化完成后调用，例如 main() 或 sensor init 后
 */
void DSM_V2_Test_AllParams(void) {
	printf("\r\n===== DSM V2 通讯测试开始 =====\r\n");

//    // 1. 切换到液位模式
//    int ret = DSM_V2_SwitchToLevelMode();
//    if (ret == NO_ERROR)
//        printf("切换液位模式成功\r\n");
//    else {
//        printf("切换液位模式失败，错误码 %d\r\n", ret);
//        return; // 通讯异常，后面读也没意义
//    }

	// 2. 定义变量
	float temp = 0, rho = 0, mu = 0, nu = 0;
	uint32_t freq = 0, sensor_id = 0;

	// 3. 依次读取各参数
//    if (DSM_V2_Read_SoftwareVersion(&ver) == NO_ERROR)
//        printf("软件版本: %.3f\r\n", ver);
//    else printf("读取软件版本失败\r\n");

	if (DSM_V2_Read_Temperature(&temp) == NO_ERROR) {
		printf("温度值: %.3f ℃\r\n", temp);
		g_measurement.single_point_monitoring.temperature = (int) (temp * 100) + 20000;
	} else
		printf("读取温度失败\r\n");

	if (DSM_V2_Read_Density(&rho) == NO_ERROR) {
		g_measurement.single_point_monitoring.density = (int) (rho * 10);
		printf("密度值: %.3f\r\n", rho);
	} else
		printf("读取密度失败\r\n");

	if (DSM_V2_Read_DynamicViscosity(&mu) == NO_ERROR)
		printf("动力粘度: %.3f\r\n", mu);
	else
		printf("读取动力粘度失败\r\n");

	if (DSM_V2_Read_KinematicViscosity(&nu) == NO_ERROR)
		printf("运动粘度: %.3f\r\n", nu);
	else
		printf("读取运动粘度失败\r\n");

    if (DSM_V2_Read_LevelFrequency(&freq) == NO_ERROR)
        printf("液位频率: %lu Hz\r\n", (unsigned long)freq);
    else printf("读取液位频率失败\r\n");

    if (DSM_V2_Read_SensorID(&sensor_id) == NO_ERROR)
        printf("传感器号: %lu\r\n", (unsigned long)sensor_id);
    else printf("读取传感器号失败\r\n");

	printf("===== DSM V2 通讯测试结束 =====\r\n\r\n");
}
//测试主函数
void Test_main(void) {
	Test_FRAM_ReadWrite(); //测试FRAM读写
//	motor_text();
	Test_Params_Storage(); //测试参数存储
	CRC32_HAL_Test(); //CRC校验测试
}

