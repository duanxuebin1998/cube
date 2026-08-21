/**
 * @file service_debug_storage.c
 * @brief CPU2 参数与编码器 FRAM 双备份维护测试。
 */

#include "service_debug_internal.h"
#include "encoder.h"
#include "error_log.h"
#include "system_parameter.h"
#include <mb85rs2m.h>

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_ENCODER_SLOT_SIZE  (0x40u) /* 测试编码器参数 A/B 分区占用的 FRAM 字节数。 */
#define TEST_ENCODER_A_ADDRESS   FRAM_ANGLE_ADDRESS /* 测试编码器参数 A 分区 FRAM 起始地址。 */
#define TEST_ENCODER_B_ADDRESS   (TEST_ENCODER_A_ADDRESS + TEST_ENCODER_SLOT_SIZE) /* 测试编码器参数 B 分区 FRAM 起始地址。 */

/**
 * @brief 写入测试罐高并回读校验设备参数存储，最后恢复原始参数。
 *
 * @note 该维护测试会真实写入 FRAM 参数区两次；仅允许在受控测试环境执行，中途掉电可能使测试值保留到下次启动。
 */
void Test_Params_Storage(void) {
	/* 备份原始参数 */
	DeviceParameters original = g_deviceParams;

	/* 测试写读校验 */
	g_deviceParams.tankHeight = 1234; /* 测试数据 */
	save_device_params(); /* 存储 */

	if (load_device_params()) {
		if (g_deviceParams.tankHeight != 1234) {
			printf("数据加载失败");
			/* 数据验证失败处理 */
		} else {
			printf("数据加载成功: tankHeight = %lu", g_deviceParams.tankHeight);
		}
	} else {
		printf("CRC校验失败");
	}

	/* 恢复原始参数 */
	g_deviceParams = original;
	save_device_params(); /* 存储 */
}

/**
 * @brief 验证参数区和编码器位置区的 FRAM A/B 双备份回退与全损坏报错逻辑。
 *
 * 测试开始时完整备份参数 A/B 分区、编码器 A/B 测试分区以及当前 g_deviceParams，随后通过破坏 magic 字段模拟单分区和双分区损坏。
 * 参数区分别验证 A 损坏时能回退到 B，以及 A/B 均损坏时必须返回 PARAM_UNINITIALIZED；编码器区分别验证 A 损坏时能从 B 恢复，以及 A/B 均损坏时必须发布
 * ENCODER_POWERON_FAIL。
 * 参数测试后恢复两个原始分区并重新保存当前参数，编码器测试后恢复两个原始分区并重新初始化编码器；每个用例只通过打印报告结果，不返回汇总状态。
 *
 * @note 该测试会真实改写 FRAM；若测试在恢复步骤前掉电或被复位，参数区或编码器位置区可能保持故意制造的损坏状态，只能在可恢复的维护环境执行。
 */
void Test_ParamEncoder_AB_Backup(void)
{
    DeviceParameters param_backup = g_deviceParams;
    /* FRAM 参数区与编码器测试区 A/B 双分区的原始备份缓冲；故障注入结束后按原字节恢复，避免测试破坏现场持久数据。 */
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

    /* Case-2: 参数A/B魔术字都损坏，读取应判定为参数未初始化 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, FRAM_PARAM_A_ADDRESS + param_magic_offset);
    WriteSingleData(0u, FRAM_PARAM_B_ADDRESS + param_magic_offset);

    if ((!load_device_params()) && (g_measurement.device_status.error_code == PARAM_UNINITIALIZED)) {
        printf("[通过] 参数区: A/B魔术字都损坏时已报错 PARAM_UNINITIALIZED\r\n");
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
