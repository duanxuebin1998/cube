#ifndef sil__AD5421_H
#define sil__AD5421_H
#include "main.h"

#define AD5421_CS_GPIO_PORT   GPIOB
#define AD5421_CS_PIN         GPIO_PIN_6

/* ============================== */
/* 寄存器地址宏定义 */
/* ============================== */
#define DELAY30US  delay_us(100)
#define SPI2WRITE00 SPI2_ReadWriteByte(0x00)

/* 满量程电流范围 */
#define STARTFULLSCALE 3.2
#define STOPFULLSCALE 24.0
/* AD5421 寄存器指令 */
#define WRITEDAC 0x01u
#define WRITECONTROL 0x02u
#define RESETAD5421REG 0x07u
#define READCONTROL 0x82u
#define READFAULT 0x85u
/* SPI 看门狗开关 */
#define CUR_SPION_COMMAND 0xC000u /* SPI 看门狗开启，4s */
#define CUR_SPIOFF_COMMAND 0x1000u /* SPI 看门狗关闭 */

#define AD5421_FAULT_FLAG_SPI_WRITE    0x00000001u
#define AD5421_FAULT_FLAG_SPI_READ     0x00000002u
#define AD5421_FAULT_FLAG_READBACK     0x00000004u
#define AD5421_FAULT_FLAG_PIN          0x00000008u
#define AD5421_FAULT_FLAG_STATUS       0x00000010u



/* 电流设置 */
#define CURRENT_INIT 				3.5
#define CURRENT_UNDERAOL 			3.6
#define CURRENT_GREATERAOH 			21.0
#define CURRENT_EQUIPMENT_ERROR 	22.0
#define CURRENT_CHECKMODE_MIN 		3.5
#define CURRENT_CHECKMODE_MAX 		24.0
#define CURRENT_CONFIGMODE 			23.0
#define CURRENT_MEAMODE 			21.5



/* 电流设置 */
#define CURRENT_CHECKMODE_MIN 3.5
#define CURRENT_CHECKMODE_MAX 24.0



/* void CurrentStateJudgeAndSend(void); */
uint32_t Ad5421Init(void);
uint32_t AD5421_SetCurrent(float mA);
uint32_t AD5421_SetCurrentX100(uint32_t mA_x100);
uint32_t AD5421_PollDiagnostics(void);
uint32_t AD5421_GetFaultFlags(void);
uint32_t AD5421_GetFaultRegister(void);

#endif

