#include <mb85rs2m.h>
#include <stdio.h>
#include "usart.h"

static void WriteEnableLatch(void);
/*
 **Function: write enable
 **Parameter: None
 **Return value: None
 */
static void WriteEnableLatch(void)
{
	// 选择 FRAM（拉低 CS 引脚）
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	// 发送写使能命令
	uint8_t cmd = MB_WRITEENABLE;
	HAL_SPI_Transmit(&FRAM_SPI, &cmd, 1, HAL_MAX_DELAY);

	// 取消选择 FRAM（拉高 CS 引脚）
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

// 写数据到 FRAM
void WriteSingleData(uint32_t data, uint32_t address)
{
	WriteEnableLatch();  // 使能写操作

	// 发送写命令
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
	uint8_t write_cmd = MB_WRITEDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &write_cmd, 1, HAL_MAX_DELAY);

	// 发送地址（3个字节）
	uint8_t address_bytes[3] =
	{ (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF };
	HAL_SPI_Transmit(&FRAM_SPI, address_bytes, 3, HAL_MAX_DELAY);

	// 发送数据（4字节）
	uint8_t data_bytes[4] =
			{ (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data
					& 0xFF };
	HAL_SPI_Transmit(&FRAM_SPI, data_bytes, 4, HAL_MAX_DELAY);

	// 取消选择 FRAM
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

// 从 FRAM 读取数据
uint32_t ReadSingleData(uint32_t address)
{
	uint32_t data = 0;

	// 选择 FRAM
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	// 发送读取命令
	uint8_t read_cmd = MB_READDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &read_cmd, 1, HAL_MAX_DELAY);

	// 发送地址（3个字节）
	uint8_t address_bytes[3] =
	{ (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF };
	HAL_SPI_Transmit(&FRAM_SPI, address_bytes, 3, HAL_MAX_DELAY);

	// 读取数据（4字节）
	uint8_t received_data[4];
	HAL_SPI_Receive(&FRAM_SPI, received_data, 4, HAL_MAX_DELAY);  // 仅接收数据

	// 组合数据
	data = (received_data[0] << 24) | (received_data[1] << 16)
			| (received_data[2] << 8) | received_data[3];

	// 取消选择 FRAM
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);

	return data;
}

// 写入两个数据
void WriteTwoData(int steps, int circle, int address)
{
	WriteEnableLatch();

	address *= 4;  // 调整地址
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	// 发送写命令
	uint8_t cmd = MB_WRITEDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &cmd, 1, HAL_MAX_DELAY);  // 假设 FRAM_SPI 是你的 SPI1 句柄

	// 发送地址
	uint8_t addr[3] =
	{ (address >> 16) & 0xff, (address >> 8) & 0xff, address & 0xff };
	HAL_SPI_Transmit(&FRAM_SPI, addr, 3, HAL_MAX_DELAY);

	// 发送第一个数据（steps）
	uint8_t steps_bytes[4] =
	{ (steps >> 24) & 0xff, (steps >> 16) & 0xff, (steps >> 8) & 0xff, steps
			& 0xff };
	HAL_SPI_Transmit(&FRAM_SPI, steps_bytes, 4, HAL_MAX_DELAY);

	// 发送第二个数据（circle）
	uint8_t circle_bytes[4] =
	{ (circle >> 24) & 0xff, (circle >> 16) & 0xff, (circle >> 8) & 0xff, circle
			& 0xff };
	HAL_SPI_Transmit(&FRAM_SPI, circle_bytes, 4, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}
// Write data to FRAM
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length)
{
	uint8_t data[4];  // 用来存储地址和命令数据

	// 使能写操作
	WriteEnableLatch();

	// 选择芯片
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	// 发送写命令
	data[0] = MB_WRITEDATA;
	HAL_SPI_Transmit(&FRAM_SPI, data, 1, HAL_MAX_DELAY);  // 发送命令

	// 发送地址
	data[0] = (startcnt >> 16) & 0xff;
	data[1] = (startcnt >> 8) & 0xff;
	data[2] = startcnt & 0xff;
	HAL_SPI_Transmit(&FRAM_SPI, data, 3, HAL_MAX_DELAY);  // 发送地址

	// 发送数据
	HAL_SPI_Transmit(&FRAM_SPI, p_array, length, HAL_MAX_DELAY);  // 发送数据

	// 取消芯片选择
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

// Read data from FRAM
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length)
{
	uint8_t address[3];  // 存储地址的字节

	// 选择 FRAM（拉低 CS 引脚）
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

	// 发送读取命令
	uint8_t cmd = MB_READDATA;
	HAL_SPI_Transmit(&FRAM_SPI, &cmd, 1, HAL_MAX_DELAY);

	// 发送地址，地址分为 3 个字节（16 位地址）
	address[0] = (startcnt >> 16) & 0xFF;
	address[1] = (startcnt >> 8) & 0xFF;
	address[2] = startcnt & 0xFF;

	HAL_SPI_Transmit(&FRAM_SPI, address, 3, HAL_MAX_DELAY);  // 发送地址

	// 读取数据
	HAL_SPI_Receive(&FRAM_SPI, p_array, length, HAL_MAX_DELAY); // 直接接收数据，无需同时发送虚拟字节

	// 取消选择 FRAM（拉高 CS 引脚）
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

// 测试 FRAM 数据读写功能，包括单字节和多字节的读写
void Test_FRAM_ReadWrite(void)
{
	uint32_t test_cases[][2] =
	{
	{ 50, 0x12345678 },          // 起始地址
			{ 511, 0xA5A5A5A5 },        // 最大逻辑地址（511 *4 = 2044，在2KB范围内）
			{ 100, 0x00000000 },        // 全零测试
			{ 200, 0xFFFFFFFF }         // 全一测试
	};

	// 测试单字节数据读写
	for (int i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++)
	{
		uint32_t logic_addr = test_cases[i][0];
		uint32_t write_data = test_cases[i][1];
		uint32_t read_data;

		// 写入数据到 FRAM
		WriteSingleData(write_data, logic_addr);
		HAL_Delay(10);  // 等待10ms，确保写入完成

		// 从 FRAM 读取数据
		read_data = ReadSingleData(logic_addr);

		// 检查读写数据是否一致
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
		HAL_Delay(100);  // 适当延时
	}

	// 测试多字节数据读写
	uint32_t start_address = 0x0100;  // 写入起始地址
	uint8_t write_data[8] =
	{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };  // 要写入的数据
	uint8_t read_data[8];  // 用于存储读取的数据

	// 写数据到 FRAM
	WriteMultiData(write_data, start_address, sizeof(write_data));
	HAL_Delay(10);  // 等待写入操作完成

	// 从 FRAM 读取数据
	ReadMultiData(read_data, start_address, sizeof(read_data));
	HAL_Delay(10);  // 等待读取操作完成

	// 检查读写数据是否一致
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
