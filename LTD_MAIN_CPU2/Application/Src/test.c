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
#define MOTOR_TEXT_ENCODER_MAX_ANGLE             4096.0f
#define MOTOR_TEXT_ENCODER_GUARD_MIN_MM          10.0f
#define MOTOR_TEXT_ENCODER_GUARD_MARGIN_MM       5.0f
#define MOTOR_TEXT_ENCODER_GUARD_SCALE           1.20f
#define MOTOR_TEXT_ENCODER_POLL_MS               20U
#define MOTOR_TEXT_ENCODER_START_GRACE_MS        500U
static uint8_t Test_ShouldAbortForCommandSwitch(void)
{
    if (!HasEffectiveCommandSwitchRequest()) {
        return 0;
    }

    printf("检测到命令切换请求，停止当前串口测试\r\n");
    MotorCtrl_SlowStop();
    return 1;
}

static int32_t Test_GetEncoderValue(void)
{
    update_sensor_height_from_encoder();
    return -g_encoder_count;
}

static uint8_t Test_EncoderTargetReached(int32_t current, int32_t target, int dir)
{
    if (dir == MOTOR_DIRECTION_DOWN) {
        return (current >= target) ? 1U : 0U;
    }

    return (current <= target) ? 1U : 0U;
}

static int32_t Test_EncoderDistanceMmToCount(float distance_mm)
{
    float encoder_count;

    if ((distance_mm <= 0.0f) || (g_deviceParams.encoder_wheel_circumference_mm == 0U)) {
        return 0;
    }

    encoder_count = (distance_mm * MOTOR_TEXT_ENCODER_MAX_ANGLE * 1000.0f) /
                    (float)g_deviceParams.encoder_wheel_circumference_mm;
    if (encoder_count < 1.0f) {
        return 1;
    }

    return (int32_t)(encoder_count + 0.5f);
}

static float Test_EncoderDistanceToGuardMm(float distance_mm)
{
    float guard_mm;

    if (distance_mm <= 0.0f) {
        return 0.0f;
    }

    guard_mm = distance_mm * MOTOR_TEXT_ENCODER_GUARD_SCALE +
               MOTOR_TEXT_ENCODER_GUARD_MARGIN_MM;

    if (guard_mm < MOTOR_TEXT_ENCODER_GUARD_MIN_MM) {
        guard_mm = MOTOR_TEXT_ENCODER_GUARD_MIN_MM;
    }

    return guard_mm;
}

static float Test_EncoderCountToDistanceMm(int32_t encoder_count)
{
    if (g_deviceParams.encoder_wheel_circumference_mm == 0U) {
        return 0.0f;
    }

    return ((float)encoder_count * (float)g_deviceParams.encoder_wheel_circumference_mm) /
           (MOTOR_TEXT_ENCODER_MAX_ANGLE * 1000.0f);
}

static uint32_t Test_MoveUntilEncoderTarget(int32_t target_encoder,
                                            int32_t origin_encoder,
                                            int dir,
                                            float guard_mm,
                                            uint32_t speed,
                                            const char *phase_name)
{
    uint32_t ret;
    uint32_t start_tick;
    int32_t current_encoder;

    ret = MotorCtrl_MoveNoWait(guard_mm, dir, speed);
    if (ret != NO_ERROR) {
        printf("%s encoder move command failed, error=0x%08lX\r\n",
               phase_name, (unsigned long)ret);
        return ret;
    }

    start_tick = HAL_GetTick();
    while (1) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return STATE_SWITCH;
        }

        ret = MotorCtrl_PollRuntimePosition();
        if (ret != NO_ERROR) {
            return ret;
        }
        current_encoder = Test_GetEncoderValue();

        if (Test_EncoderTargetReached(current_encoder, target_encoder, dir)) {
            ret = MotorCtrl_SlowStop();
            if (ret != NO_ERROR) {
                printf("%s encoder stop failed, error=0x%08lX\r\n",
                       phase_name, (unsigned long)ret);
                return ret;
            }
            printf("%s encoder target reached | current=%ld(%.2fmm) | target=%ld(%.2fmm)\r\n",
                   phase_name,
                   (long)current_encoder,
                   Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
                   (long)target_encoder,
                   Test_EncoderCountToDistanceMm(target_encoder - origin_encoder));
            return NO_ERROR;
        }

        if (((HAL_GetTick() - start_tick) > MOTOR_TEXT_ENCODER_START_GRACE_MS) &&
            (MotorCtrl_GetDisplayState() == 0U)) {
            printf("%s motor stopped before encoder target | current=%ld(%.2fmm) | target=%ld(%.2fmm)\r\n",
                   phase_name,
                   (long)current_encoder,
                   Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
                   (long)target_encoder,
                   Test_EncoderCountToDistanceMm(target_encoder - origin_encoder));
            return MOTOR_STEP_ERROR;
        }

        HAL_Delay(MOTOR_TEXT_ENCODER_POLL_MS);
    }
}

//电机小步进上行测试
void motor_step_up_text(void) {
    int i = 0;
    int32_t ticks = 4 * 32;
    printf("motor STEP text start\n");
    stpr_enableDriver(&stepper);
    printf("start up\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{传感器位置}%.1f", i, (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t{称重值}%d\r\n", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }
    stpr_disableDriver(&stepper);
    printf("motor text over\n");
}

//电机小步进下行测试
void motor_step_down_text(void) {
    int i = 0;
    int32_t ticks = 4 * 32;
    printf("motor STEP text start\n");
    stpr_enableDriver(&stepper);
    printf("start down\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{传感器位置}%.1f", i, (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t{称重值}%d\r\n", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }
    printf("down over!\n");
    stpr_disableDriver(&stepper);
    printf("motor text over\n");
}

//电机步进测试
void motor_step_text(void) {
    int i = 0;
    int32_t ticks = 4 * 32;
    printf("motor STEP text start\n");
    stpr_enableDriver(&stepper);

    printf("4步进测试\n");
    printf("start down\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("down over!\n");
    printf("start up\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("8步进测试\n");
    printf("start down\n");
    for (i = 0; i < 12000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 8 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("down over!\n");
    printf("start up\n");
    for (i = 0; i < 12000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -8 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("40步进测试\n");
    printf("start down\n");
    for (i = 0; i < 2400; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 40 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("down over!\n");
    printf("start up\n");
    for (i = 0; i < 2400; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -40 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    stpr_disableDriver(&stepper);
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
        printf("[通过] 参数区: A损坏后已回退到B\r\n");
    } else {
        printf("[失败] 参数区: A损坏后未能回退到B\r\n");
    }

    /* Case-2: 参数A/B都损坏，读取应报错 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, FRAM_PARAM_A_ADDRESS + param_magic_offset);
    WriteSingleData(0u, FRAM_PARAM_B_ADDRESS + param_magic_offset);

    if ((!load_device_params()) && (g_measurement.device_status.error_code == PARAM_EEPROM_FAIL)) {
        printf("[通过] 参数区: A/B都损坏时已报错 PARAM_EEPROM_FAIL\r\n");
    } else {
        printf("[失败] 参数区: A/B都损坏时报错不符合预期, 错误码：0x%08lX\r\n",
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
        printf("[通过] 编码区: A损坏后已回退到B\r\n");
    } else {
        printf("[失败] 编码区: A损坏后未能回退到B\r\n");
    }

    /* Case-4: 编码A/B都损坏，应报错 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, TEST_ENCODER_A_ADDRESS);
    WriteSingleData(0u, TEST_ENCODER_B_ADDRESS);
    Initialize_Encoder();

    if (g_measurement.device_status.error_code == ENCODER_POWERON_FAIL) {
        printf("[通过] 编码区: A/B都损坏时已报错 ENCODER_POWERON_FAIL\r\n");
    } else {
        printf("[失败] 编码区: A/B都损坏时报错不符合预期, 错误码：0x%08lX\r\n",
               (unsigned long)g_measurement.device_status.error_code);
    }

    /* 恢复编码区 */
    WriteMultiData(encoder_a_raw, TEST_ENCODER_A_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    WriteMultiData(encoder_b_raw, TEST_ENCODER_B_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    Initialize_Encoder();

    printf("===== AB双备份回退测试结束 =====\r\n\r\n");
}
static void __attribute__((unused)) Sensor_CommCheckAndLog(const char *tag)
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
        printf("[传感器][正常] %s 温度=%.3f C\r\n", tag, temp);
    } else {
        comm_fail_cnt++;
        printf("[传感器][异常] %s 温度读取失败，错误码：%lu，失败次数：%lu\r\n",
               tag, (unsigned long)ret, (unsigned long)comm_fail_cnt);
    }

    /* 读密度 */
    ret = DSM_V2_Read_Density(&density);
    if (ret == NO_ERROR) {
        printf("[传感器][正常] %s 密度=%.3f\r\n", tag, density);
    } else {
        comm_fail_cnt++;
        printf("[传感器][异常] %s 密度读取失败，错误码：%lu，失败次数：%lu\r\n",
               tag, (unsigned long)ret, (unsigned long)comm_fail_cnt);
    }

    /* 读频率 + 扫频均值 */
    ret = DSM_V2_Read_DensityFrequency(&frequency, &hz_45, &hz_225);
    if (ret == NO_ERROR) {
        printf("[传感器][正常] %s 频率=%.1f Hz, 45度频率=%.2f, 22.5度频率=%.2f\r\n",
               tag, frequency, hz_45, hz_225);
    } else {
        comm_fail_cnt++;
        printf("[传感器][异常] %s 频率读取失败，错误码：%lu，失败次数：%lu\r\n",
               tag, (unsigned long)ret, (unsigned long)comm_fail_cnt);
    }
}


/* ========================= 主测试函数 ========================= */
void motor_text(float run_distance_mm, uint8_t enable_sensor_comm)
{
    uint32_t loop_cnt = 0;
    uint32_t ret;
    uint32_t speed = MotorCtrl_GetDefaultSpeedX100();

    if (run_distance_mm <= 0.0f) {
        printf("motor_text参数异常，运行距离=%.2fmm\r\n", run_distance_mm);
        return;
    }

    ret = (uint32_t)MeasureStart();
    if (ret != NO_ERROR) {
        printf("motor_text初始化失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    printf("motor text start | distance=%.2fmm | sensor_comm=%u\r\n",
           run_distance_mm,
           (unsigned int)enable_sensor_comm);

    ret = MotorCtrl_Init(); // 电机初始化，保持与原测试入口一致
    if (ret != NO_ERROR) {
        printf("motor_text电机初始化失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    ret = stpr_setPos(&stepper, 0); // 将当前位置定义为本轮测试零点，上行回 0 会回到这里
    if (ret != NO_ERROR) {
        printf("motor_text设置测试零点失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    while (1) {
        if (Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }

        /* ---------- 下行 ---------- */
        stpr_enableDriver(&stepper);
        ret = MotorCtrl_MoveNoWait(run_distance_mm, MOTOR_DIRECTION_DOWN, speed);
        if ((ret == STATE_SWITCH) || Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }
        if (ret != NO_ERROR) {
            printf("[LOOP %lu] down command failed, error=0x%08lX\r\n",
                   (unsigned long)(loop_cnt + 1), (unsigned long)ret);
            MotorCtrl_SlowStop();
            stpr_disableDriver(&stepper);
            return;
        }

        printf("[LOOP %lu] start down\r\n", (unsigned long)(loop_cnt + 1));
        ret = stpr_waitMove(&stepper);
        if ((ret == STATE_SWITCH) || Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }
        if (ret != NO_ERROR) {
            printf("[LOOP %lu] down failed, error=0x%08lX\r\n",
                   (unsigned long)(loop_cnt + 1), (unsigned long)ret);
            stpr_disableDriver(&stepper);
            return;
        }
        printf("[LOOP %lu] down over!\r\n", (unsigned long)(loop_cnt + 1));

        if (enable_sensor_comm != 0U) {
            Sensor_CommCheckAndLog("after DOWN");
        }

        if (Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }

        /* ---------- 回零（上行到本轮测试零点） ---------- */
        ret = stpr_moveTo(&stepper, 0, velocity);
        if ((ret == STATE_SWITCH) || Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }
        if (ret != NO_ERROR) {
            printf("[LOOP %lu] up command failed, error=0x%08lX\r\n",
                   (unsigned long)(loop_cnt + 1), (unsigned long)ret);
            MotorCtrl_SlowStop();
            stpr_disableDriver(&stepper);
            return;
        }

        printf("[LOOP %lu] start up to zero\r\n", (unsigned long)(loop_cnt + 1));
        ret = stpr_waitMove(&stepper);
        if ((ret == STATE_SWITCH) || Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }
        if (ret != NO_ERROR) {
            printf("[LOOP %lu] up failed, error=0x%08lX\r\n",
                   (unsigned long)(loop_cnt + 1), (unsigned long)ret);
            stpr_disableDriver(&stepper);
            return;
        }
        printf("[LOOP %lu] up over (to zero)\r\n", (unsigned long)(loop_cnt + 1));

        if (enable_sensor_comm != 0U) {
            Sensor_CommCheckAndLog("after UP");
        }

        loop_cnt++;
        printf("[LOOP %lu] cycle done\r\n", (unsigned long)loop_cnt);
    }
}

void motor_text_encoder(float run_distance_mm, uint8_t enable_sensor_comm)
{
    uint32_t loop_cnt = 0;
    uint32_t ret;
    uint32_t speed = MotorCtrl_GetDefaultSpeedX100();
    float guard_mm;
    int32_t run_encoder_count;
    int32_t origin_encoder;
    int32_t current_encoder;
    int32_t down_target_encoder;

    if (run_distance_mm <= 0.0f) {
        printf("motor_text_encoder invalid distance=%.2fmm\r\n", run_distance_mm);
        return;
    }

    run_encoder_count = Test_EncoderDistanceMmToCount(run_distance_mm);
    guard_mm = Test_EncoderDistanceToGuardMm(run_distance_mm);
    if ((run_encoder_count <= 0) || (guard_mm <= 0.0f)) {
        printf("motor_text_encoder invalid encoder wheel circumference=%lu, distance=%.2fmm\r\n",
               (unsigned long)g_deviceParams.encoder_wheel_circumference_mm,
               run_distance_mm);
        return;
    }

    ret = (uint32_t)MeasureStart();
    if (ret != NO_ERROR) {
        printf("motor_text_encoder init failed, error=0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    ret = MotorCtrl_Init();
    if (ret != NO_ERROR) {
        printf("motor_text_encoder motor init failed, error=0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    origin_encoder = Test_GetEncoderValue();
    down_target_encoder = origin_encoder + run_encoder_count;

    printf("motor encoder text start | distance=%.2fmm | origin=%ld(0.00mm) | down_target=%ld(%.2fmm) | encoder_delta=%ld(%.2fmm) | guard=%.2fmm | sensor_comm=%u\r\n",
           run_distance_mm,
           (long)origin_encoder,
           (long)down_target_encoder,
           Test_EncoderCountToDistanceMm(down_target_encoder - origin_encoder),
           (long)run_encoder_count,
           Test_EncoderCountToDistanceMm(run_encoder_count),
           guard_mm,
           (unsigned int)enable_sensor_comm);

    while (1) {
        if (Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }

        current_encoder = Test_GetEncoderValue();

        stpr_enableDriver(&stepper);
        printf("[LOOP %lu] encoder down start | current=%ld(%.2fmm) | origin=%ld(0.00mm) | target=%ld(%.2fmm)\r\n",
               (unsigned long)(loop_cnt + 1U),
               (long)current_encoder,
               Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
               (long)origin_encoder,
               (long)down_target_encoder,
               Test_EncoderCountToDistanceMm(down_target_encoder - origin_encoder));

        ret = Test_MoveUntilEncoderTarget(down_target_encoder,
                                          origin_encoder,
                                          MOTOR_DIRECTION_DOWN,
                                          guard_mm,
                                          speed,
                                          "DOWN");
        if (ret != NO_ERROR) {
            stpr_disableDriver(&stepper);
            return;
        }

        if (enable_sensor_comm != 0U) {
            Sensor_CommCheckAndLog("after ENCODER DOWN");
        }

        if (Test_ShouldAbortForCommandSwitch()) {
            stpr_disableDriver(&stepper);
            return;
        }

        current_encoder = Test_GetEncoderValue();
        printf("[LOOP %lu] encoder up start | current=%ld(%.2fmm) | target=%ld(0.00mm)\r\n",
               (unsigned long)(loop_cnt + 1U),
               (long)current_encoder,
               Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
               (long)origin_encoder);

        ret = Test_MoveUntilEncoderTarget(origin_encoder,
                                          origin_encoder,
                                          MOTOR_DIRECTION_UP,
                                          guard_mm,
                                          speed,
                                          "UP");
        if (ret != NO_ERROR) {
            stpr_disableDriver(&stepper);
            return;
        }

        if (enable_sensor_comm != 0U) {
            Sensor_CommCheckAndLog("after ENCODER UP");
        }

        loop_cnt++;
        current_encoder = Test_GetEncoderValue();
        printf("[LOOP %lu] encoder cycle done | current=%ld(%.2fmm)\r\n",
               (unsigned long)loop_cnt,
               (long)current_encoder,
               Test_EncoderCountToDistanceMm(current_encoder - origin_encoder));
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
static uint8_t Demo_SinglePointDisplay_ShouldAbort(void)
{
    if (!HasEffectiveCommandSwitchRequest()) {
        return 0;
    }

    printf("单点展示检测到新命令，退出当前演示\r\n");
    g_measurement.device_status.device_state = STATE_STANDBY;
    g_measurement.debug_data.motor_state = 0U;
    return 1;
}

static void Demo_SinglePointDisplay_UpdateResult(volatile DensityMeasurement *result,
                                                 uint32_t temperature_raw,
                                                 uint32_t density_raw,
                                                 uint32_t pos_01mm,
                                                 uint32_t standard_density_raw,
                                                 uint32_t vcf20_raw,
                                                 uint32_t weight_density_raw)
{
    if (result == NULL) {
        return;
    }

    result->temperature = temperature_raw;
    result->density = density_raw;
    result->temperature_position = pos_01mm;
    result->standard_density = standard_density_raw;
    result->vcf20 = vcf20_raw;
    result->weight_density = weight_density_raw;
}

void Demo_SinglePointDisplayMock(void)
{
    static const int16_t temp_wave_x100[]    = { 0, 6, 12, 18, 24, 18, 12, 6, 0, -4, -8, -4 };
    static const int16_t density_wave_x10[]  = { 0, 1, 2, 3, 2, 1, 0, -1, -2, -1, 0, 1 };
    static const int16_t pos_wave_01mm[]     = { 0, 2, 4, 6, 8, 6, 4, 2, 0, -2, -4, -2 };
    const uint32_t wave_count = (uint32_t)(sizeof(temp_wave_x100) / sizeof(temp_wave_x100[0]));
    uint32_t target_pos_01mm;
    uint32_t tank_height_01mm;
    uint32_t start_pos_01mm;
    uint32_t current_pos_01mm;
    uint32_t cable_01mm;
    uint32_t i;

    target_pos_01mm = g_deviceParams.singlePointMeasurementPosition;
    if (target_pos_01mm == 0U) {
        if (g_measurement.debug_data.sensor_position > 0) {
            target_pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position;
        } else if (g_deviceParams.tankHeight > 0U) {
            target_pos_01mm = g_deviceParams.tankHeight / 2U;
        } else {
            target_pos_01mm = 15000U;
        }
    }

    tank_height_01mm = g_deviceParams.tankHeight;
    if ((tank_height_01mm == 0U) || (tank_height_01mm <= target_pos_01mm)) {
        tank_height_01mm = target_pos_01mm + 12000U;
    }

    start_pos_01mm = (target_pos_01mm > 3000U) ? (target_pos_01mm - 3000U) : 0U;
    current_pos_01mm = start_pos_01mm;

    printf("\r\n===== 单点测量展示模式开始 =====\r\n");
    printf("展示说明: 不读真实传感器、不驱动电机，只刷新单点测量显示字段\r\n");
    printf("停止方式: 下发任意新命令即可退出展示\r\n");

    g_measurement.device_status.zero_point_status = 0U;
    g_measurement.device_status.error_code = NO_ERROR;
    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
    g_measurement.debug_data.motor_state = 2U;
    g_measurement.debug_data.motor_speed = 120U;

    for (i = 0U; i < 6U; i++) {
        if (Demo_SinglePointDisplay_ShouldAbort()) {
            return;
        }

        current_pos_01mm = start_pos_01mm +
                (uint32_t)(((uint64_t)(target_pos_01mm - start_pos_01mm) * (uint64_t)(i + 1U)) / 6U);
        cable_01mm = (tank_height_01mm > current_pos_01mm) ? (tank_height_01mm - current_pos_01mm) : 0U;

        g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
        g_measurement.debug_data.sensor_position = (int32_t)current_pos_01mm;
        g_measurement.debug_data.cable_length = (int32_t)cable_01mm;
        g_measurement.debug_data.motor_distance = (int32_t)cable_01mm;
        g_measurement.debug_data.temperature = 22580U;
        g_measurement.debug_data.frequency = 121300U + i * 20U;
        g_measurement.debug_data.air_frequency = 121900U;
        g_measurement.debug_data.current_amplitude = 88U + i;
        g_measurement.debug_data.current_weight = 3200U + i * 10U;

        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_measurement,
                                             22580U,
                                             8348U,
                                             current_pos_01mm,
                                             8338U,
                                             9997U,
                                             8342U);
        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_monitoring,
                                             22580U,
                                             8348U,
                                             current_pos_01mm,
                                             8338U,
                                             9997U,
                                             8342U);

        printf("单点展示\t运行到测量点 [%lu/6] 位置=%.1fmm\r\n",
               (unsigned long)(i + 1U),
               current_pos_01mm / 10.0f);
        HAL_Delay(500);
    }

    printf("单点展示\t已到达展示点，开始刷新虚拟温度/密度\r\n");

    for (i = 0U;; i++) {
        uint32_t idx = i % wave_count;
        uint32_t temperature_raw;
        uint32_t density_raw;
        uint32_t standard_density_raw;
        uint32_t weight_density_raw;
        uint32_t vcf20_raw;

        if (Demo_SinglePointDisplay_ShouldAbort()) {
            return;
        }

        current_pos_01mm = (uint32_t)((int32_t)target_pos_01mm + pos_wave_01mm[idx]);
        cable_01mm = (tank_height_01mm > current_pos_01mm) ? (tank_height_01mm - current_pos_01mm) : 0U;

        temperature_raw = (uint32_t)(20000 + 2650 + temp_wave_x100[idx]);
        density_raw = (uint32_t)(8350 + density_wave_x10[idx]);
        standard_density_raw = density_raw - 8U;
        weight_density_raw = density_raw - 4U;
        vcf20_raw = 9995U + (idx % 6U);

        g_measurement.device_status.device_state = STATE_SINGLEPOINTING;
        g_measurement.debug_data.motor_state = 0U;
        g_measurement.debug_data.motor_speed = 0U;
        g_measurement.debug_data.sensor_position = (int32_t)current_pos_01mm;
        g_measurement.debug_data.cable_length = (int32_t)cable_01mm;
        g_measurement.debug_data.motor_distance = (int32_t)cable_01mm;
        g_measurement.debug_data.temperature = temperature_raw;
        g_measurement.debug_data.frequency = 121500U + idx * 15U;
        g_measurement.debug_data.air_frequency = 121980U;
        g_measurement.debug_data.current_amplitude = 96U + (idx % 5U);
        g_measurement.debug_data.current_weight = 3280U + (idx % 4U) * 8U;

        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_measurement,
                                             temperature_raw,
                                             density_raw,
                                             current_pos_01mm,
                                             standard_density_raw,
                                             vcf20_raw,
                                             weight_density_raw);
        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_monitoring,
                                             temperature_raw,
                                             density_raw,
                                             current_pos_01mm,
                                             standard_density_raw,
                                             vcf20_raw,
                                             weight_density_raw);

        printf("单点展示\t状态=固定点测量中 位置=%.1fmm 温度=%.2fC 密度=%.1f 标密=%.1f VCF20=%lu 重量密度=%.1f\r\n",
               current_pos_01mm / 10.0f,
               RAW_TO_TEMP(temperature_raw),
               RAW_TO_DENSITY(density_raw),
               RAW_TO_DENSITY(standard_density_raw),
               (unsigned long)vcf20_raw,
               RAW_TO_DENSITY(weight_density_raw));

        HAL_Delay(500);
    }
}
static uint8_t Test_TMC5130_IsValidGstat(uint32_t gstat)
{
    return ((gstat & ~0x07UL) == 0UL);
}

/**
 * @brief TMC5130 静态 SPI 通信测试。
 *
 * 不启动电机，只重复读取 GSTAT/DRV_STATUS/IOIN/IFCNT，用于判断静止状态下
 * SPI 是否仍有 0xFFFFFFFF、0x00FFFFFF、0x00000100 等非法读数。
 */
void Test_TMC5130_SPI_Static(void)
{
    const uint32_t loops = 2000U;
    const uint32_t delay_ms = 10U;
    uint32_t gstat_ok = 0U;
    uint32_t gstat_invalid = 0U;
    uint32_t gstat_read_fail = 0U;
    uint32_t drv_read_fail = 0U;
    uint32_t ioin_read_fail = 0U;
    uint32_t ifcnt_read_fail = 0U;
    uint32_t chopconf_zero = 0U;      /* CHOPCONF 为 0 的次数，用于判断配置丢失/24V 掉电。 */
    uint32_t chopconf_read_fail = 0U; /* CHOPCONF 读取失败次数，用于区分总线失败和配置清零。 */
    uint32_t xactual_read_fail = 0U;  /* XACTUAL 读取失败次数，用于定位运行期位置读数异常。 */
    int32_t gstat = 0;
    int32_t drvstatus = 0;
    int32_t ioin = 0;
    int32_t ifcnt = 0;
    int32_t chopconf = 0;
    int32_t xactual = 0;

    printf("TMC5130 SPI静态测试开始 | loops=%lu | delay=%lums\r\n",
           (unsigned long)loops,
           (unsigned long)delay_ms);
    printf("测试期间不下发运动命令，只读取寄存器；如静止也大量非法，优先检查CS/MISO/供电/复位。\r\n");

    for (uint32_t i = 1U; i <= loops; ++i) {
        if (Test_ShouldAbortForCommandSwitch()) {
            break;
        }

        if (!stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat)) {
            gstat_read_fail++;
            printf("[TMC SPI %lu] GSTAT读取失败\r\n", (unsigned long)i);
        } else if (!Test_TMC5130_IsValidGstat((uint32_t)gstat)) {
            gstat_invalid++;
            printf("[TMC SPI %lu] GSTAT非法=0x%08lX invalid=0x%08lX\r\n",
                   (unsigned long)i,
                   (unsigned long)((uint32_t)gstat),
                   (unsigned long)(((uint32_t)gstat) & ~0x07UL));
        } else {
            gstat_ok++;
        }

        if (!stpr_tryReadInt(&stepper, TMC5130_DRVSTATUS, &drvstatus)) {
            drv_read_fail++;
        }
        if (!stpr_tryReadInt(&stepper, TMC5130_IOIN, &ioin)) {
            ioin_read_fail++;
        }
        if (!stpr_tryReadInt(&stepper, TMC5130_IFCNT, &ifcnt)) {
            ifcnt_read_fail++;
        }
        /* 静态测试同时读取 CHOPCONF，验证 SPI 正常时配置是否被 24V 掉电清零。 */
        if (!stpr_tryReadInt(&stepper, TMC5130_CHOPCONF, &chopconf)) {
            chopconf_read_fail++;
        } else if (chopconf == 0) {
            chopconf_zero++;
        }
        /* XACTUAL 单独统计失败次数，便于和运行期“疑似 SPI 全 0”日志对照。 */
        if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual)) {
            xactual_read_fail++;
        }

        if ((i == 1U) || ((i % 100U) == 0U)) {
            printf("[TMC SPI %lu] GSTAT=0x%08lX DRV=0x%08lX IOIN=0x%08lX IFCNT=%ld CHOP=0x%08lX XACTUAL=%ld | ok=%lu invalid=%lu fail=%lu cfg0=%lu\r\n",
                   (unsigned long)i,
                   (unsigned long)((uint32_t)gstat),
                   (unsigned long)((uint32_t)drvstatus),
                   (unsigned long)((uint32_t)ioin),
                   (long)ifcnt,
                   (unsigned long)((uint32_t)chopconf),
                   (long)xactual,
                   (unsigned long)gstat_ok,
                   (unsigned long)gstat_invalid,
                   (unsigned long)gstat_read_fail,
                   (unsigned long)chopconf_zero);
        }

        HAL_Delay(delay_ms);
    }

    printf("TMC5130 SPI静态测试结束 | GSTAT ok=%lu invalid=%lu read_fail=%lu | DRV_fail=%lu IOIN_fail=%lu IFCNT_fail=%lu CHOP_zero=%lu CHOP_fail=%lu XACTUAL_fail=%lu\r\n",
           (unsigned long)gstat_ok,
           (unsigned long)gstat_invalid,
           (unsigned long)gstat_read_fail,
           (unsigned long)drv_read_fail,
           (unsigned long)ioin_read_fail,
           (unsigned long)ifcnt_read_fail,
           (unsigned long)chopconf_zero,
           (unsigned long)chopconf_read_fail,
           (unsigned long)xactual_read_fail);
}
//测试主函数
void Test_main(void) {
	Test_FRAM_ReadWrite(); //测试FRAM读写
//	motor_text(300.0f, 0U);
	Test_Params_Storage(); //测试参数存储
	CRC32_HAL_Test(); //CRC校验测试
}

