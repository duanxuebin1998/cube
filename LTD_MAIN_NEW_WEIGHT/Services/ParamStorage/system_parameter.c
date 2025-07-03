/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */
#include "system_parameter.h"
#include "mb85rs2m.h"
#include <stdint.h>
#include <string.h>

volatile MeasurementResult g_measurement;     // 测量结果
volatile DeviceParameters g_deviceParams;     // 设备参数

// CRC计算函数
uint32_t calculate_crc(const uint8_t *data, uint32_t length)
{
	uint32_t crc = CRC_SEED;
	// 实现伪代码，需替换为实际CRC算法
	for (uint32_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		// 这里插入CRC多项式计算逻辑
	}
	return crc;
}

/*********************** 参数存储逻辑 ***********************/
void save_device_params(void)
{
	DeviceParameters params = g_deviceParams;

	// 计算CRC（不包含crc字段自身）
	uint32_t crc_size = sizeof(params) - sizeof(params.crc);
	params.crc = calculate_crc((uint8_t*) &params, crc_size);

	// 通过现有接口写入
	WriteMultiData((uint8_t*) &params,
	FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));
}

/*********************** 参数加载逻辑 ***********************/
int load_device_params(void)
{
	DeviceParameters temp;

	// 通过现有接口读取
	ReadMultiData((uint8_t*) &temp,
	FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));

	// 验证CRC
	uint32_t saved_crc = temp.crc;
	temp.crc = 0; // 清零后计算
	uint32_t calc_crc = calculate_crc((uint8_t*) &temp, sizeof(temp));

	if (calc_crc == saved_crc)
	{
		memcpy((void* volatile ) &g_deviceParams, &temp,
				sizeof(DeviceParameters));
		return 1; // 校验成功
	}
	return 0; // 校验失败
}

/*********************** 参数初始化 ***********************/
void init_device_params(void)
{
	// 尝试加载参数
	if (!load_device_params())
	{
		// 加载失败时初始化默认值
		memset((void* volatile ) &g_deviceParams, 0, sizeof(DeviceParameters));
//        g_deviceParams.tankHeight = 1000; // 示例默认值
//        // ...其他参数默认值...
		save_device_params(); // 保存默认参数
	}
}

///*********************** 测试函数 ***********************/
//void Test_FRAM_ReadWrite(void) {
//    // 备份原始参数
//    DeviceParameters original = g_deviceParams;
//
//    // 测试写读校验
//    g_deviceParams.tankHeight = 1234;
//    save_device_params();
//
//    if(load_device_params()) {
//        if(g_deviceParams.tankHeight != 1234) {
//            // 数据验证失败处理
//        }
//    } else {
//        // CRC校验失败处理
//    }
//
//    // 恢复原始参数
//    g_deviceParams = original;
//    save_device_params();
//}
