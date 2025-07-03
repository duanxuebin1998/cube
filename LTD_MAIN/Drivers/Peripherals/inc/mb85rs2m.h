#ifndef sil__MB85RS16_H
#define sil__MB85RS16_H
#include "main.h"

#define FRAM_CS_Pin GPIO_PIN_11
#define FRAM_CS_GPIO_Port GPIOE
#define FRAM_SPI hspi4

#define MB_WRITEENABLE 0x06  /*Write enable*/
#define MB_WRITEDATA  0x02 /*Write data*/
#define MB_READDATA  0x03  /*Read data*/

extern SPI_HandleTypeDef FRAM_SPI;

#define FRAM_ANGLE_ADDRESS 0x0000  // 定义 FRAM 存储角度值的地址
#define FRAM_ENCODER_ADDRESS 0x0004  // 定义 FRAM 存储角度值的地址

void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length);
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length);

uint32_t ReadSingleData(uint32_t address);
void WriteSingleData(uint32_t data, uint32_t address);
void WriteTwoData(int steps, int circle, int address);
void Test_FRAM_ReadWrite(void);

#endif

