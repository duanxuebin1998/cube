#ifndef sil__MB85RS16_H
#define sil__MB85RS16_H
#include "main.h"

#define FRAM_CS_Pin GPIO_PIN_11
#define FRAM_CS_GPIO_Port GPIOE
#define FRAM_SPI hspi4

#define MB_WRITEENABLE 0x06  /* Write enable */
#define MB_WRITEDATA  0x02 /* Write data */
#define MB_READDATA  0x03  /* Read data */

extern SPI_HandleTypeDef FRAM_SPI;

#define FRAM_ANGLE_ADDRESS 0x1000  /* 定义 FRAM 存储角度值的地址 */
#define FRAM_ENCODER_ADDRESS 0x1004  /* 定义 FRAM 存储角度值的地址 */

/**
 * @brief 写入或设置FRAM 存储中的 WriteMultiData 逻辑。
 *
 * @param p_array 输入/输出指针。
 * @param startcnt 业务参数。
 * @param length 数据长度。
 */
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length); /* 写入多字节数据到 FRAM */
/**
 * @brief 读取FRAM 存储中的 ReadMultiData 逻辑。
 *
 * @param p_array 输入/输出指针。
 * @param startcnt 业务参数。
 * @param length 数据长度。
 */
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length); /* 从 FRAM 读取多字节数据 */

/**
 * @brief 读取FRAM 存储中的 ReadSingleData 逻辑。
 *
 * @param address 地址参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t ReadSingleData(uint32_t address); /* 从 FRAM 读取单个数据 */
/**
 * @brief 写入或设置FRAM 存储中的 WriteSingleData 逻辑。
 *
 * @param data 数据缓冲区。
 * @param address 地址参数。
 */
void WriteSingleData(uint32_t data, uint32_t address); /* 写入单个数据到 FRAM */
/**
 * @brief 写入或设置FRAM 存储中的 WriteTwoData 逻辑。
 *
 * @param steps 业务参数。
 * @param circle 业务参数。
 * @param address 地址参数。
 */
void WriteTwoData(int steps, int circle, int address); /* 写入两个数据到 FRAM */
/**
 * @brief 读取FRAM 存储中的 Test_FRAM_ReadWrite 逻辑。
 */
void Test_FRAM_ReadWrite(void); /* 测试 FRAM 数据读写功能，包括单字节和多字节的读写 */

#endif

