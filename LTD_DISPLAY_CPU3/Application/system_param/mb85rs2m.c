#include <mb85rs2m.h>
#include <stdio.h>
#include "usart.h"

/**
 * @brief 向 FRAM 发送 WREN 指令以置位写使能锁存。
 */
static void WriteEnableLatch(void);

/**
 * @brief 向 FRAM 发送 WREN 指令以置位写使能锁存。
 */
static void WriteEnableLatch(void)
{
	/* 选择 FRAM（拉低 CS 引脚） */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	/* 发送写使能命令 */
	uint8_t cmd = MB_WRITEENABLE;
	HAL_SPI_Transmit(&FRAM_SPI, &cmd, 1, HAL_MAX_DELAY);

	/* 取消选择 FRAM（拉高 CS 引脚） */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 写数据到 FRAM。
 *
 * @param data 准备写入 CPU3 参数 FRAM 的 32 位原始值；函数按固定字节顺序从 address 开始写入四个连续字节。
 * @param address CPU3 参数 FRAM 的绝对字节地址；多字节读写从该地址开始并按连续地址递增。
 */
void WriteSingleData(uint32_t data, uint32_t address)
{
	WriteEnableLatch();  /* 使能写操作 */

	/* 发送写命令 */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
	uint8_t write_cmd = MB_WRITEDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &write_cmd, 1, HAL_MAX_DELAY);

	/* 发送地址（3个字节） */
	uint8_t address_bytes[3] =
	{ (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF };
	HAL_SPI_Transmit(&FRAM_SPI, address_bytes, 3, HAL_MAX_DELAY);

	/* 发送数据（4字节） */
	uint8_t data_bytes[4] =
			{ (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data
					& 0xFF };
	HAL_SPI_Transmit(&FRAM_SPI, data_bytes, 4, HAL_MAX_DELAY);

	/* 取消选择 FRAM */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 从 FRAM 读取数据。
 *
 * @param address CPU3 参数 FRAM 的绝对字节地址；多字节读写从该地址开始并按连续地址递增。
 * @return 返回从指定 FRAM 绝对地址按大端顺序组合的 32 位原始值。
 */
uint32_t ReadSingleData(uint32_t address)
{
	uint32_t data = 0;

	/* 选择 FRAM */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	/* 发送读取命令 */
	uint8_t read_cmd = MB_READDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &read_cmd, 1, HAL_MAX_DELAY);

	/* 发送地址（3个字节） */
	uint8_t address_bytes[3] =
	{ (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF };
	HAL_SPI_Transmit(&FRAM_SPI, address_bytes, 3, HAL_MAX_DELAY);

	/* 读取数据（4字节） */
	uint8_t received_data[4];
	HAL_SPI_Receive(&FRAM_SPI, received_data, 4, HAL_MAX_DELAY);  /* 仅接收数据 */

	/* 组合数据 */
	data = (received_data[0] << 24) | (received_data[1] << 16)
			| (received_data[2] << 8) | received_data[3];

	/* 取消选择 FRAM */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);

	return data;
}

/**
 * @brief 写入两个数据。
 *
 * @param steps 与卷绕圈数一同保存的编码器累计步数。
 * @param circle 与编码器位置一同保存的卷绕圈数。
 * @param address CPU3 参数 FRAM 的绝对字节地址；多字节读写从该地址开始并按连续地址递增。
 */
void WriteTwoData(int steps, int circle, int address)
{
	WriteEnableLatch();

	address *= 4;  /* 调整地址 */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	/* 发送写命令 */
	uint8_t cmd = MB_WRITEDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &cmd, 1, HAL_MAX_DELAY);  /* 假设 FRAM_SPI 是你的 SPI1 句柄 */

	/* 发送地址 */
	uint8_t addr[3] =
	{ (address >> 16) & 0xff, (address >> 8) & 0xff, address & 0xff };
	HAL_SPI_Transmit(&FRAM_SPI, addr, 3, HAL_MAX_DELAY);

	/* 发送第一个数据（steps） */
	uint8_t steps_bytes[4] =
	{ (steps >> 24) & 0xff, (steps >> 16) & 0xff, (steps >> 8) & 0xff, steps
			& 0xff };
	HAL_SPI_Transmit(&FRAM_SPI, steps_bytes, 4, HAL_MAX_DELAY);

	/* 发送第二个数据（circle） */
	uint8_t circle_bytes[4] =
	{ (circle >> 24) & 0xff, (circle >> 16) & 0xff, (circle >> 8) & 0xff, circle
			& 0xff };
	HAL_SPI_Transmit(&FRAM_SPI, circle_bytes, 4, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 从 24 位 FRAM 地址连续阻塞写入数据；当前接口不校验范围和 HAL 返回值。
 *
 * @param p_array 待连续写入 FRAM 的源字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length)
{
	uint8_t data[4];  /* 用来存储地址和命令数据 */

	/* 使能写操作 */
	WriteEnableLatch();

	/* 选择芯片 */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	/* 发送写命令 */
	data[0] = MB_WRITEDATA;
	HAL_SPI_Transmit(&FRAM_SPI, data, 1, HAL_MAX_DELAY);  /* 发送命令 */

	/* 发送地址 */
	data[0] = (startcnt >> 16) & 0xff;
	data[1] = (startcnt >> 8) & 0xff;
	data[2] = startcnt & 0xff;
	HAL_SPI_Transmit(&FRAM_SPI, data, 3, HAL_MAX_DELAY);  /* 发送地址 */

	/* 发送数据 */
	HAL_SPI_Transmit(&FRAM_SPI, p_array, length, HAL_MAX_DELAY);  /* 发送数据 */

	/* 取消芯片选择 */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}


/**
 * @brief 从 24 位 FRAM 地址连续阻塞读取数据；当前接口不校验范围和 HAL 返回值。
 *
 * @param p_array 用于接收 FRAM 连续读取结果的目标字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length)
{
	uint8_t address[3];  /* 存储地址的字节 */

	/* 选择 FRAM（拉低 CS 引脚） */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	/* 发送读取命令 */
	uint8_t cmd = MB_READDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &cmd, 1, HAL_MAX_DELAY);

	/* 发送地址，地址分为 3 个字节（16 位地址） */
	address[0] = (startcnt >> 16) & 0xFF;
	address[1] = (startcnt >> 8) & 0xFF;
	address[2] = startcnt & 0xFF;

	HAL_SPI_Transmit(&FRAM_SPI, address, 3, HAL_MAX_DELAY);  /* 发送地址 */

	/* 读取数据 */
	HAL_SPI_Receive(&FRAM_SPI, p_array, length, HAL_MAX_DELAY); /* 直接接收数据，无需同时发送虚拟字节 */

	/* 取消选择 FRAM（拉高 CS 引脚） */
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 在固定地址执行破坏性 FRAM 单字和多字节读写校验，并逐项打印比对结果。
 *
 * 单字测试依次在逻辑地址 50、511、100 和 200 写入普通模式、全零及全一数据，延时后回读 32 位值并打印通过或失败。
 * 多字节测试从字节地址 0x0100 写入固定八字节序列，再逐字节回读比较并输出每个地址的结果。
 *
 * @note 该测试会覆盖上述固定 FRAM 地址且不会恢复原内容，只能在确认这些地址允许被破坏的维护环境执行，禁止在生产流程或有效参数镜像上调用。
 */
void Test_FRAM_ReadWrite(void)
{
	uint32_t test_cases[][2] =
	{
	{ 50, 0x12345678 },          /* 起始地址 */
			{ 511, 0xA5A5A5A5 },        /* 最大逻辑地址（511 *4 = 2044，在2KB范围内） */
			{ 100, 0x00000000 },        /* 全零测试 */
			{ 200, 0xFFFFFFFF }         /* 全一测试 */
	};

	/* 测试单字节数据读写 */
	for (int i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++)
	{
		uint32_t logic_addr = test_cases[i][0];
		uint32_t write_data = test_cases[i][1];
		uint32_t read_data;

		/* 写入数据到 FRAM */
		WriteSingleData(write_data, logic_addr);
		HAL_Delay(10);  /* 测试用 10 ms 观察间隔；FRAM 写接口返回时 SPI 事务已经结束，这不是 Flash 式写周期等待。 */

		/* 从 FRAM 读取数据 */
		read_data = ReadSingleData(logic_addr);

		/* 检查读写数据是否一致 */
		if (read_data == write_data)
		{
			printf("Test %d (Single Byte) passed: Addr=0x%04lX, Data=0x%08lX\n",
					i, logic_addr, read_data);
		}
		else
		{
			printf(
					"Test %d (Single Byte) FAILED: Addr=0x%04lX, Write=0x%08lX, Read=0x%08lX\n",
					i, logic_addr, write_data, read_data);
		}
		HAL_Delay(100);  /* 拉开相邻单字节测试项，便于串口逐条观察地址和读回数据。 */
	}

	/* 测试多字节数据读写 */
	uint32_t start_address = 0x0100;  /* 写入起始地址 */
	uint8_t write_data[8] =
	{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };  /* 要写入的数据 */
	uint8_t read_data[8];  /* 用于存储读取的数据 */

	/* 写数据到 FRAM */
	WriteMultiData(write_data, start_address, sizeof(write_data));
	HAL_Delay(10);  /* 测试用 10 ms 观察间隔；MB85RS2M 写入不需要额外内部编程时间。 */

	/* 从 FRAM 读取数据 */
	ReadMultiData(read_data, start_address, sizeof(read_data));
	HAL_Delay(10);  /* 测试用 10 ms 观察间隔；读接口返回时数据已经写入 read_data。 */

	/* 检查读写数据是否一致 */
	for (int i = 0; i < sizeof(write_data); i++)
	{
		if (read_data[i] != write_data[i])
		{
			printf(
					"Test (Multi Byte) FAILED: Addr=0x%04lX, Write=0x%02X, Read=0x%02X\n",
					start_address + i, write_data[i], read_data[i]);
		}
		else
		{
			printf("Test (Multi Byte) passed: Addr=0x%04lX, Data=0x%02X\n",
					start_address + i, read_data[i]);
		}
	}
}
